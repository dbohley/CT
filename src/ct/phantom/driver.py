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


def compare_logs(
    phantom_path: str | Path,
    controller_path: str | Path,
    *,
    max_lag_s: float = 1.0,
) -> dict[str, Any]:
    """Align a phantom log against a controller log and score the sensing.

    Both are timestamped with ``time.monotonic()`` on the same host, so alignment is a
    straight interpolation onto a common grid. Three numbers come out:

    - **RMSE and bias** of what the controller sensed against what the phantom was
      commanded to do.
    - **Lag**, from the cross-correlation peak. That number *is* ``tau_s`` — the sensor
      latency term in the forecast horizon — measured rather than assumed. It is one of
      the entries in :mod:`ct.unknowns`, which makes this tool the way to close it out.

    Sign convention: the controller senses tactile *deflection*, which increases as the
    phantom surface advances toward the rig, so the two series are compared after removing
    each one's mean. Only the shape and timing are meaningful; the offsets are two
    different datums.
    """
    from ct.rt.telemetry import load_jsonl  # noqa: PLC0415

    phantom = [r for r in load_jsonl(phantom_path) if r.get("commanded_mm") is not None]
    controller = [r for r in load_jsonl(controller_path) if r.get("tactile_mm") is not None]
    if len(phantom) < 10 or len(controller) < 10:
        raise ValueError(
            f"not enough overlapping records: {len(phantom)} phantom, {len(controller)} "
            "controller. Were both logs written by runs of the same session?"
        )

    tp = np.array([r["t"] for r in phantom], dtype=float)
    yp = np.array([r["commanded_mm"] for r in phantom], dtype=float)
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

    scale = float(np.dot(gc, gp) / np.dot(gp, gp)) if np.dot(gp, gp) > 0 else float("nan")
    residual = gc - gp
    return {
        "overlap_s": float(t_end - t_start),
        "fs": float(fs),
        "samples": int(grid.size),
        "rmse_mm": float(np.sqrt(np.mean(residual**2))),
        "bias_mm": float(np.mean(residual)),
        "amplitude_ratio": scale,
        "lag_s": lag_s,
        "lag_note": "cross-correlation peak; this is a direct measurement of latency.tau_s",
        "correlation": float(np.corrcoef(gc, gp)[0, 1]),
    }
