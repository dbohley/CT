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


def compare_logs(
    phantom_path: str | Path,
    controller_path: str | Path,
    *,
    max_lag_s: float = 1.0,
    phase: str | None = None,
    phantom_field: str = "commanded_mm",
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

    Sign convention: the controller senses tactile *deflection*, which increases as the
    phantom surface advances toward the rig, so the two series are compared after removing
    each one's mean. Only the shape and timing are meaningful; the offsets are two
    different datums.
    """
    from ct.rt.telemetry import load_jsonl  # noqa: PLC0415

    phantom = load_jsonl(phantom_path)
    controller = [r for r in load_jsonl(controller_path) if r.get("tactile_mm") is not None]
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
    yc = np.array([r["tactile_mm"] for r in controller], dtype=float)

    t_start, t_end = max(tp[0], tc[0]), min(tp[-1], tc[-1])
    if t_end - t_start < 1.0:
        raise ValueError(
            f"logs overlap for only {t_end - t_start:.3f} s. They must be from runs that "
            "were live at the same time, on the same host."
        )

    fs = 1.0 / float(np.median(np.diff(tc)))
    grid = np.arange(t_start, t_end, 1.0 / fs)
    gp = np.interp(grid, tp, yp)
    gc = np.interp(grid, tc, yc)
    gp -= gp.mean()
    gc -= gc.mean()

    max_shift = int(max_lag_s * fs)
    correlation = np.correlate(gc, gp, mode="full")
    lags = np.arange(-len(gp) + 1, len(gp))
    keep = np.abs(lags) <= max_shift
    best = lags[keep][int(np.argmax(correlation[keep]))]
    lag_s = float(best / fs)

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
        # Residual of the sensor against the lag-aligned, amplitude-matched phantom: what an
        # estimator would still have to contend with once the two characterised effects
        # (delay and attenuation) are taken out. This is the number the EKF has to beat.
        "rmse_mm": float(np.sqrt(np.mean(residual**2))),
        "bias_mm": float(np.mean(residual)),
        "amplitude_ratio": scale,
        "lag_s": lag_s,
        "lag_note": "cross-correlation peak; this is a direct measurement of latency.tau_s",
        # A peak up against the edge of the search range is not a measurement, it is the
        # range running out. The real bench lag is 0.677s against a 1.0s default.
        "lag_at_search_edge": bool(max_shift > 0 and abs(best) >= 0.8 * max_shift),
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
