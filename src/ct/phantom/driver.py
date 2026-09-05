"""Driving the breathing phantom — a separate process, on its own bus.

Deliberately independent of the controller. The phantom rig and the sensing rig share
nothing but a wall clock, which is what makes "how well is the sensing doing" an honest
question: if the controller could read the phantom's commanded position, comparing the two
would prove nothing.

The waveform comes from any registered :class:`~ct.interfaces.SignalSource`, so the same
``sinusoid``, ``lujan`` and ``rc_piecewise`` models the estimator was validated against in
session 001 drive the physical phantom too — and ``csv`` replays the collected breathing
recordings.

Both processes log against ``time.monotonic()`` on the same host, so ``ct-compare`` can
align them directly. Different hosts would need clock synchronisation and the alignment
would be worth exactly as much as that synchronisation; the tooling assumes one host and
says so.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ct.geometry import AxisCalibration
from ct.hw.config import MotorConfig
from ct.rt.telemetry import TelemetryWriter


@dataclass
class PhantomLimits:
    """Motion limits. A phantom that slams is a phantom that breaks or hurts someone."""

    travel_mm: tuple[float, float] = (0.0, 60.0)
    v_max_mm_s: float = 200.0
    ramp_s: float = 3.0
    """Fade the amplitude in and out over this long, so the phantom never starts or stops
    at a non-zero velocity."""


class PhantomDriver:
    """Commands one motor to follow a breathing waveform, and logs what it did."""

    def __init__(
        self,
        bus: Any,
        codec: Any,
        config: MotorConfig,
        calibration: AxisCalibration,
        source: Any,
        *,
        center_mm: float = 30.0,
        limits: PhantomLimits | None = None,
        telemetry: TelemetryWriter | None = None,
        dry_run: bool = False,
    ) -> None:
        self.bus = bus
        self.codec = codec
        self.config = config
        self.calibration = calibration
        self.source = source
        self.center_mm = float(center_mm)
        self.limits = limits or PhantomLimits()
        self.telemetry = telemetry
        self.dry_run = dry_run

        self._t0: float | None = None
        self._measured_counts: float | None = None
        self.commands = 0
        self.clipped = 0

    # -- waveform --------------------------------------------------------------

    def target_mm(self, elapsed: float, duration: float | None = None) -> float:
        """Commanded phantom position at ``elapsed`` seconds into the run."""
        value = float(np.asarray(self.source.clean(np.array([elapsed]))).ravel()[0])
        target = self.center_mm + value * self._ramp(elapsed, duration)
        lo, hi = self.limits.travel_mm
        if not lo <= target <= hi:
            self.clipped += 1
            target = min(max(target, lo), hi)
        return target

    def _ramp(self, elapsed: float, duration: float | None) -> float:
        """Raised-cosine fade at both ends, in ``[0, 1]``.

        A raised cosine rather than a linear ramp because it starts and ends with zero
        *slope* as well as zero amplitude — the phantom neither jerks into motion nor
        stops abruptly, which matters for a mechanism carrying a chest-wall surrogate.
        """
        ramp = self.limits.ramp_s
        if ramp <= 0:
            return 1.0
        factor = 1.0
        if elapsed < ramp:
            factor = 0.5 * (1.0 - np.cos(np.pi * elapsed / ramp))
        if duration is not None:
            remaining = duration - elapsed
            if remaining < ramp:
                factor = min(factor, 0.5 * (1.0 - np.cos(np.pi * max(remaining, 0.0) / ramp)))
        return float(np.clip(factor, 0.0, 1.0))

    # -- running ---------------------------------------------------------------

    def enable(self) -> None:
        can_id, data, extended = self.codec.enable(self.config.can_id)
        self._send(can_id, data, extended)

    def disable(self) -> None:
        can_id, data, extended = self.codec.disable(self.config.can_id)
        self._send(can_id, data, extended)

    def step(self, t: float, frames: list[tuple[float, int, bytes]],
             duration: float | None = None) -> dict[str, Any]:
        """One tick: read feedback, command the next position, return the log record."""
        if self._t0 is None:
            self._t0 = t
        elapsed = t - self._t0

        for stamp, can_id, data in frames:
            parsed = self.codec.parse(can_id, data)
            if parsed is not None and int(parsed.get("node_id", -1)) == self.config.can_id:
                self._measured_counts = parsed["position"]

        commanded = self.target_mm(elapsed, duration)
        gains = self.config.gains
        can_id, data, extended = self.codec.command(
            self.config.can_id,
            position=self.calibration.to_counts(commanded),
            velocity=abs(self.calibration.rate_to_counts_s(self.limits.v_max_mm_s)),
            kp=gains.get("kp", 0.0),
            kd=gains.get("kd", 0.0),
        )
        self._send(can_id, data, extended)

        measured = (
            None if self._measured_counts is None
            else self.calibration.to_mm(self._measured_counts)
        )
        record = {
            "t": t,
            "elapsed": elapsed,
            "commanded_mm": commanded,
            "measured_mm": measured,
            "error_mm": None if measured is None else measured - commanded,
        }
        if self.telemetry is not None:
            self.telemetry.write(record)
        return record

    def _send(self, can_id: int, data: bytes, extended: bool) -> None:
        if self.dry_run:
            return
        self.bus.send(can_id, data, extended=extended)
        self.commands += 1

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "commands": self.commands,
            "clipped": self.clipped,
            "center_mm": self.center_mm,
            "travel_mm": list(self.limits.travel_mm),
        }


SEAM_JUMP_FACTOR = 5.0
"""A step this many times the 95th-percentile step is a profile loop restart, not motion.

``run_breathing_profile.py --loop`` replays a finite CSV end to end, so unless the profile
happens to start and end at the same position there is a discontinuity at every seam. They
are real and worth counting — a seam is a moment no sensor could have tracked — but they
are not breathing.

Against the 95th percentile rather than the median because ``measured_mm`` is a zero-order
hold between status broadcasts: on a 25 Hz broadcast under an 85 Hz command tick, two thirds
of the steps are exactly zero, the median with them, and every ordinary riser then reads as
an enormous multiple of it. The p95 step is a real riser in both the smooth and the
staircase case, and a seam is several times larger than one either way.
"""


def _phantom_series(
    records: list[dict[str, Any]], field: str
) -> tuple[np.ndarray, np.ndarray, str]:
    """Phantom ``(t, y, field_used)`` for ``field``, which may be ``"auto"``.

    ``auto`` prefers ``measured_mm`` — what the motor's own status broadcast says it
    actually did — and falls back to ``commanded_mm``. Logs written before the phantom
    script recorded feedback have no ``measured_mm`` at all, and so do runs where the
    broadcast never arrived; both must degrade to commanded rather than fail.
    """
    candidates = ["measured_mm", "commanded_mm"] if field == "auto" else [field]
    for name in candidates:
        rows = [r for r in records if r.get(name) is not None]
        if len(rows) >= 10:
            t = np.array([r["t"] for r in rows], dtype=float)
            y = np.array([r[name] for r in rows], dtype=float)
            return t, y, name
    available = sorted({k for r in records for k, v in r.items() if v is not None})
    raise ValueError(
        f"phantom log has no usable '{field}' column (fields present: {available}). "
        "Was it written by a run that recorded motor feedback?"
    )


def _apply_lag(phantom: np.ndarray, sensor: np.ndarray, shift: int) -> tuple[np.ndarray, np.ndarray]:
    """Slide the two series into alignment by ``shift`` samples and trim to the overlap.

    A positive ``shift`` means the sensor lags: sample ``i`` of the phantom lines up with
    sample ``i + shift`` of the sensor.
    """
    if shift > 0:
        return phantom[:-shift], sensor[shift:]
    if shift < 0:
        return phantom[-shift:], sensor[:shift]
    return phantom, sensor


def count_seams(y: np.ndarray) -> int:
    """Profile loop-restart discontinuities in a *raw, un-interpolated* phantom series.

    Must be called before any resampling. On the common grid a seam spans two samples
    rather than one, so a per-sample rule counts each of them roughly twice — on the real
    bench log, 11 restarts (166 s of a 14.83 s profile, exactly right) become 18.
    """
    if y.size < 3:
        return 0
    steps = np.abs(np.diff(y))
    reference = float(np.quantile(steps, 0.95))
    if reference <= 0:
        return 0
    return int(np.count_nonzero(steps > SEAM_JUMP_FACTOR * reference))


SENSOR_SIGN = {"tactile_mm": 1.0, "tactile_raw_mm": 1.0, "tof_mm": -1.0}
"""Which way each controller column moves when the phantom surface advances.

Tactile deflection *grows* as the surface pushes the arm back; ToF *distance* shrinks.
This is fixed physics per sensor, not something to discover per run — and discovering it
is not even possible from one run, because for a near-sinusoidal signal an inverted sensor
is indistinguishable from a correctly-signed one half a breath away. Orienting both to the
phantom up front also makes their amplitude ratios directly comparable: on the bench the
ToF reads 0.79-1.10 of real excursion against the tactile arm's 0.16-0.76.
"""


def _refine_peak(corr: np.ndarray, idx: int) -> float:
    """Sub-sample offset of a correlation peak, in samples.

    The raw ``argmax`` quantises the lag to one grid sample — 7.9 ms at the bench's
    127 Hz. That was tolerable while the lag was a curiosity; it is not now that the
    number sets the forecast horizon.
    """
    from ct.identification.spectral import _parabolic_offset_linear  # noqa: PLC0415

    return _parabolic_offset_linear(corr, idx)


def _breath_period_s(grid: np.ndarray, gp: np.ndarray) -> float | None:
    """Dominant breathing period of the phantom series, or ``None`` if unmeasurable."""
    from ct.identification.spectral import fft_peak_omega  # noqa: PLC0415

    try:
        omega, _ = fft_peak_omega(grid, gp)
    except (ValueError, np.linalg.LinAlgError):
        return None
    return float(2.0 * np.pi / omega) if omega > 0 else None


def compare_logs(
    phantom_path: str | Path,
    controller_path: str | Path,
    *,
    max_lag_s: float = 1.0,
    phase: str | None = None,
    phantom_field: str = "commanded_mm",
    sensor_field: str = "tactile_mm",
    sensor_sign: float | None = None,
) -> dict[str, Any]:
    """Align a phantom log against a controller log and score the sensing.

    Both are timestamped with ``time.monotonic()`` on the same host, so alignment is a
    straight interpolation onto a common grid. Three numbers come out:

    - **RMSE and bias** of what the controller sensed against what the phantom did.
    - **Amplitude ratio**, how much of the phantom's real excursion survives the tactile
      chain. Measured at ~0.33 on the bench: the lever deflects and the skin deforms, so
      two thirds of the motion never reaches the sensor.
    - **Lag**, from the cross-correlation peak. That number *is* ``tau_s`` — the sensor
      latency term in the forecast horizon — measured rather than assumed. It is one of
      the entries in :mod:`ct.unknowns`, which makes this tool the way to close it out.

    ``phase`` restricts the controller side to records in one procedure state, and on a
    real run it is the difference between a meaningful answer and a meaningless one. Scored
    over a whole ``approach_and_seat`` run, where the base spends most of its time moving
    and the sensor is not yet seated, run ``20260901-165415`` reports correlation 0.039;
    scored over its ``standoff_hold`` alone, the same data gives 0.599, and 0.961 once the
    lag is taken out. Averaging across phases does not dilute the result, it destroys it.

    ``phantom_field`` chooses what counts as truth: ``"commanded_mm"`` (what the profile
    asked for), ``"measured_mm"`` (what the motor's status broadcast says it did), or
    ``"auto"`` for measured-with-fallback. Comparing the two isolates the phantom's own
    tracking error from the sensing chain's.

    ``sensor_field`` chooses which controller column is the sensor. The default
    ``tactile_mm`` is the contact chain the estimator actually consumes; passing
    ``tof_mm`` measures the *non-contact* path over the same bus, the same tick loop and
    the same motion, which is what separates sensing latency from contact settling. On the
    bench that split is stark: the ToF lags under ~0.1 s where the tactile arm lags
    0.28-0.56 s, so all but a few tens of milliseconds of the historical 0.677 s "tau_s"
    is viscoelastic settling in the contact, not latency in the sensor.

    ``sensor_sign`` orients that column to the phantom; it defaults from
    :data:`SENSOR_SIGN` and rarely needs passing. All metrics are reported in the oriented
    frame, so ``amplitude_ratio`` is positive for a working sensor of either polarity.

    Sign convention: the controller senses tactile *deflection*, which increases as the
    phantom surface advances toward the rig, so the two series are compared after removing
    each one's mean. Only the shape and timing are meaningful; the offsets are two
    different datums.
    """
    from ct.rt.telemetry import load_jsonl  # noqa: PLC0415

    sign = float(SENSOR_SIGN.get(sensor_field, 1.0) if sensor_sign is None else sensor_sign)

    phantom = load_jsonl(phantom_path)
    all_controller = load_jsonl(controller_path)
    controller = [r for r in all_controller if r.get(sensor_field) is not None]
    if not controller:
        available = sorted({k for r in all_controller for k, v in r.items() if v is not None})
        raise ValueError(
            f"controller log has no usable '{sensor_field}' column "
            f"(fields present: {available})."
        )
    if phase is not None:
        controller = [r for r in controller if r.get("phase") == phase]
    if len(phantom) < 10 or len(controller) < 10:
        raise ValueError(
            f"not enough overlapping records: {len(phantom)} phantom, {len(controller)} "
            + (f"controller in phase '{phase}'. " if phase else "controller. ")
            + "Were both logs written by runs of the same session?"
        )

    tp, yp, field_used = _phantom_series(phantom, phantom_field)
    # Seams are a property of the commanded profile -- the playback loop restarting -- and
    # are counted on the raw series, before any interpolation. See count_seams.
    t_commanded, commanded, _ = _phantom_series(phantom, "commanded_mm")
    tc = np.array([r["t"] for r in controller], dtype=float)
    yc = np.array([r[sensor_field] for r in controller], dtype=float)

    t_start, t_end = max(tp[0], tc[0]), min(tp[-1], tc[-1])
    if t_end - t_start < 1.0:
        raise ValueError(
            f"logs overlap for only {t_end - t_start:.3f} s. They must be from runs that "
            "were live at the same time, on the same host."
        )

    fs = 1.0 / float(np.median(np.diff(tc)))
    grid = np.arange(t_start, t_end, 1.0 / fs)
    gp = np.interp(grid, tp, yp)
    gc = np.interp(grid, tc, yc) * sign
    gp -= gp.mean()
    gc -= gc.mean()

    # Breathing is periodic, so the correlation surface is periodic in the lag: there is a
    # sidelobe at every lag +/- T_breath, and nothing in an argmax prefers the true one.
    # Search wider than half a period and the answer can alias to a neighbouring cycle --
    # not a hypothetical, a +/-3s scan of the ToF against this bench's ~5.8s breathing
    # returned -2.77s. Clamp the search to just inside T/2 and say so when the caller's
    # max_lag_s was the thing that had to give.
    breath_period_s = _breath_period_s(grid, gp)
    requested_shift = int(max_lag_s * fs)
    ambiguity_shift = (
        int(0.45 * breath_period_s * fs) if breath_period_s is not None else requested_shift
    )
    max_shift = max(1, min(requested_shift, ambiguity_shift))

    # Normalise each shift by the energy of the two segments that actually overlap at it.
    # Raw np.correlate is a bare dot product over n-|k| terms, so the shrinking overlap
    # imposes a triangular taper pulling the peak toward zero lag; dividing by the overlap
    # count alone over-corrects and pushes it the other way (on a synthetic 0.235s lag that
    # lands 0.9 samples high, worse than not interpolating). Dividing by sqrt(Ec*Ep) is the
    # per-shift correlation coefficient and is unbiased in the lag.
    n = len(gp)
    lags = np.arange(-max_shift, max_shift + 1)
    cp = np.concatenate([[0.0], np.cumsum(gp**2)])
    cc = np.concatenate([[0.0], np.cumsum(gc**2)])
    kept = np.empty(lags.size)
    for i, k in enumerate(lags):
        # k > 0: sensor lags, so gc[k:] lines up with gp[:n-k].
        pa, pb = (0, n - k) if k >= 0 else (-k, n)
        ca, cb = (k, n) if k >= 0 else (0, n + k)
        energy = (cp[pb] - cp[pa]) * (cc[cb] - cc[ca])
        kept[i] = np.dot(gc[ca:cb], gp[pa:pb]) / np.sqrt(energy) if energy > 0 else 0.0

    peak = int(np.argmax(kept))
    best = int(lags[peak])
    lag_s = float((best + _refine_peak(kept, peak)) / fs)

    # Every amplitude/agreement number must be computed AFTER taking the lag out. On the
    # unshifted series a 0.677s lag against a ~4.9s breath projects the phantom onto the
    # sensor at cos(2*pi*0.677/4.93) ~ 0.63, so the scale reads 0.203 where the true ratio
    # is 0.326 -- confirmed independently by Stage 1, which fits A_1 = 0.346mm to the sensor
    # column of aligned.csv and 1.063mm to the truth column. Reporting the unshifted number
    # as "the fraction of real excursion the sensor sees" conflates delay with attenuation
    # and understates the sensor by a third.
    ap, ac = _apply_lag(gp, gc, int(best))
    scale = float(np.dot(ac, ap) / np.dot(ap, ap)) if np.dot(ap, ap) > 0 else float("nan")
    residual = ac - scale * ap
    return {
        "overlap_s": float(t_end - t_start),
        "fs": float(fs),
        "samples": int(grid.size),
        "phase": phase,
        "phantom_field_used": field_used,
        "sensor_field_used": sensor_field,
        "sensor_sign": sign,
        "breath_period_s": breath_period_s,
        # Residual of the sensor against the lag-aligned, amplitude-matched phantom: what an
        # estimator would still have to contend with once the two characterised effects
        # (delay and attenuation) are taken out. This is the number the EKF has to beat.
        "rmse_mm": float(np.sqrt(np.mean(residual**2))),
        "bias_mm": float(np.mean(residual)),
        "amplitude_ratio": scale,
        "lag_s": lag_s,
        "lag_note": (
            "normalized cross-correlation peak, refined to sub-sample by parabolic fit. "
            "For sensor_field='tactile_mm' this is the WHOLE sensing lag, most of which is "
            "viscoelastic settling in the contact rather than latency in the sensor -- "
            "compare against sensor_field='tof_mm' to split them."
        ),
        # A peak up against the edge of the search range is not a measurement, it is the
        # range running out.
        "lag_at_search_edge": bool(max_shift > 0 and abs(best) >= 0.8 * max_shift),
        # ...and a search range wider than half a breath cannot distinguish a lag from the
        # same lag one cycle over, however confident the peak looks.
        "lag_ambiguous": bool(
            breath_period_s is not None and requested_shift > ambiguity_shift
        ),
        "max_lag_searched_s": float(max_shift / fs),
        "correlation": float(np.corrcoef(ac, ap)[0, 1]),
        "correlation_unshifted": float(np.corrcoef(gc, gp)[0, 1]),
        "metric_note": (
            "amplitude_ratio, rmse_mm, bias_mm and correlation are all computed after "
            "removing lag_s. correlation_unshifted is the same comparison without that "
            "shift, and is a measure of delay as much as of tracking: on the bench it reads "
            "0.599 where the aligned correlation is 0.961."
        ),
        "seams": count_seams(commanded[(t_commanded >= t_start) & (t_commanded <= t_end)]),
    }
