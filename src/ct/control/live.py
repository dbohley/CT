"""Turning a live sensor stream into something Stage 1 can identify.

The estimator's identifier is a batch, FFT-based method: it wants a uniformly sampled
record. A CAN sensor gives neither — samples arrive when frames arrive, at whatever jitter
the bus and the loop produce, and the control loop runs faster than the sensor publishes.

:class:`SignalAccumulator` bridges the two. It collects ``(t, y)`` pairs as they genuinely
arrive, then resamples onto a uniform grid when a batch is asked for.

**Why resample rather than pass the raw stamps.** ``CSVSource`` already refuses records
whose timestamps deviate more than 5% from their median step — a check that exists because
the FFT quietly produces wrong answers on non-uniform data rather than complaining. Real
bus arrivals will trip that check. Interpolating onto a uniform grid is the honest fix, and
doing it here means the identifier keeps the guarantee it was written against. The
interpolation error is bounded by the jitter, which is orders of magnitude below the
breathing signal.

Only the *identifier* needs this. The EKF handles irregular ``dt`` natively — ``step(t, y)``
takes the real timestamp and scales ``Q`` accordingly — so tracking is fed raw arrivals,
not the resampled grid.
"""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

from ct.types import SignalBatch


class SignalAccumulator:
    """A bounded, time-ordered buffer of live sensor samples."""

    def __init__(self, maxlen: int = 200_000) -> None:
        self._t: deque[float] = deque(maxlen=maxlen)
        self._y: deque[float] = deque(maxlen=maxlen)
        self._last_stamp: float | None = None
        self.duplicates_skipped = 0

    def offer(self, stamp: float, value: float) -> bool:
        """Add a sample if it is genuinely new. Returns whether it was taken.

        The control loop ticks faster than the sensor publishes, so the same reading is
        offered several times. Deduplicating on the arrival stamp — rather than on the
        loop tick — is what keeps the record's sample rate equal to the *sensor's* rate
        instead of the loop's, which is what the identifier needs to see.
        """
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.duplicates_skipped += 1
            return False
        self._t.append(float(stamp))
        self._y.append(float(value))
        self._last_stamp = stamp
        return True

    def clear(self) -> None:
        self._t.clear()
        self._y.clear()
        self._last_stamp = None

    def __len__(self) -> int:
        return len(self._t)

    @property
    def duration(self) -> float:
        if len(self._t) < 2:
            return 0.0
        return self._t[-1] - self._t[0]

    @property
    def t0(self) -> float | None:
        return self._t[0] if self._t else None

    @property
    def t_last(self) -> float | None:
        """Arrival time of the newest sample, on the loop's clock.

        Needed because :meth:`to_batch` rebases time to zero for the identifier, while the
        tracker is fed raw loop timestamps. Without handing this back, the tracker's first
        step would span the whole gap between the two clocks — a jump of tens of seconds,
        which is exactly the kind of mismatch the residual monitor is built to catch.
        """
        return self._t[-1] if self._t else None

    @property
    def median_fs(self) -> float | None:
        """Sample rate implied by the median arrival interval."""
        if len(self._t) < 2:
            return None
        dt = np.diff(np.asarray(self._t, dtype=float))
        median = float(np.median(dt))
        return 1.0 / median if median > 0 else None

    def jitter_fraction(self) -> float:
        """Worst deviation from the median step, as a fraction of it.

        The same statistic ``CSVSource`` rejects on above 5%. Reported in the run summary
        so the real bus's timing quality is a measured number rather than an assumption.
        """
        if len(self._t) < 3:
            return 0.0
        dt = np.diff(np.asarray(self._t, dtype=float))
        median = float(np.median(dt))
        if median <= 0:
            return 0.0
        return float(np.abs(dt - median).max() / median)

    def window(self, t_start: float, t_end: float) -> tuple[np.ndarray, np.ndarray]:
        """Raw samples in ``[t_start, t_end)``, without resampling."""
        t = np.asarray(self._t, dtype=float)
        y = np.asarray(self._y, dtype=float)
        mask = (t >= t_start) & (t < t_end)
        return t[mask], y[mask]

    def to_batch(self, fs: float | None = None, t_start: float | None = None) -> SignalBatch:
        """Uniformly resampled batch, ready for Stage 1.

        Time is rebased to zero so the identification result's ``t0`` means "start of the
        calibration window", matching what the offline pipeline produces. The caller keeps
        the real start time; mixing loop-monotonic values into the estimator would make
        two runs incomparable for no benefit.
        """
        if len(self._t) < 4:
            raise ValueError(
                f"need at least 4 samples to identify, have {len(self._t)}. The tactile "
                "sensor may not be publishing."
            )
        t = np.asarray(self._t, dtype=float)
        y = np.asarray(self._y, dtype=float)
        if t_start is not None:
            mask = t >= t_start
            t, y = t[mask], y[mask]

        fs = fs or self.median_fs
        if not fs or fs <= 0:
            raise ValueError("could not infer a sample rate from the accumulated stamps")

        span = t[-1] - t[0]
        n = int(np.floor(span * fs)) + 1
        if n < 4:
            raise ValueError(f"accumulated window is only {span:.3f} s at {fs:g} Hz")
        grid = t[0] + np.arange(n) / fs
        resampled = np.interp(grid, t, y)
        return SignalBatch(t=grid - grid[0], y=resampled, fs=float(fs))

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "samples": len(self._t),
            "duration_s": self.duration,
            "median_fs": self.median_fs,
            "jitter_fraction": self.jitter_fraction(),
            "duplicates_skipped": self.duplicates_skipped,
        }


class AmplitudeWatcher:
    """Peak-to-trough of a signal over a rolling window, and whether it has settled.

    Used by APPROACH's seating sub-step. The question it answers is "is the sensor now
    following the whole breath, or still clipping at one end", and the observable answer is
    that the measured amplitude stops growing as the sensor is seated deeper.

    Only fed while the base is stationary. A moving base adds its own displacement to the
    tactile reading, which would look exactly like a larger breathing amplitude and would
    make the seating criterion satisfy itself.
    """

    def __init__(self, window_s: float) -> None:
        self.window_s = float(window_s)
        self._t: deque[float] = deque()
        self._y: deque[float] = deque()

    def reset(self) -> None:
        self._t.clear()
        self._y.clear()

    def add(self, t: float, y: float) -> None:
        self._t.append(t)
        self._y.append(y)
        while self._t and (t - self._t[0]) > self.window_s:
            self._t.popleft()
            self._y.popleft()

    @property
    def span(self) -> float:
        return (self._t[-1] - self._t[0]) if len(self._t) >= 2 else 0.0

    @property
    def full(self) -> bool:
        """Whether the window holds enough time to judge an amplitude."""
        return self.span >= self.window_s * 0.95 and len(self._t) >= 8

    @property
    def amplitude(self) -> float:
        if not self._y:
            return 0.0
        return max(self._y) - min(self._y)

    @property
    def peak(self) -> float:
        return max(self._y) if self._y else 0.0

    @property
    def trough(self) -> float:
        return min(self._y) if self._y else 0.0
