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


class BreathPeakWatcher:
    """The typical breathing peak, measured over a whole number of real breaths.

    Standoff positions the base so the *breathing peak* of the tactile deflection lands on a
    target. Getting that number right needs an estimator that is both unbiased and phrased in
    breaths rather than seconds, for two reasons that each cost a real bench run.

    **``max`` over a time window is biased, and the bias grows with the window.** Real
    breathing varies breath to breath: over 34 breaths of the ``emma_normal_breathing``
    profile the per-breath peak has mean 2.125 mm and std 0.540 mm. The maximum of ``N``
    draws from that sits above the typical peak by an amount that grows with ``N`` --
    +0.30 mm at 2 breaths, +0.59 at 4, +0.84 at 8. Because standoff *retreats* whenever the
    measured peak exceeds its target, that bias alone drives spurious retreats, and
    "measuring more carefully" by waiting longer makes it strictly worse. The mean of the
    per-breath peaks has no such bias, and its standard error falls the way an average
    should: 0.382 mm at 2 breaths, 0.270 at 4, 0.190 at 8.

    **A fixed time window is not a fixed number of breaths.** The caller's
    ``min_breaths * nominal_breath_s`` gave 2.0 x 4.0 = 8.0 s, but the same profile really
    breathes at 5.51 s, so "two breaths" was 1.45 of them -- sometimes containing two peaks
    and sometimes one. Counting real breaths removes the dependence on a nominal period that
    nothing keeps honest, and :attr:`period_s` reports what the breathing actually was.

    Breaths are segmented at upward crossings of the running mean, with hysteresis at a
    fraction of the observed amplitude so that noise near the mean cannot split one breath
    into several. Only *completed* breaths count, so a partial one at either end is never
    mistaken for a shallow one.

    Validated against real data: 33 breaths found in 180 s of ``emma_normal_breathing``
    against a zero-crossing reference of 37, and 10 against 11 on the tactile trace of run
    ``20260903-152502``. The shortfall is the partial breaths at each end, which is the
    intended behaviour. The trailing reference window means the crossing level tracks slow
    baseline wander (emma drifts -0.00513 mm/s), but a baseline moving as fast as the
    breathing amplitude itself would still defeat it -- roughly ten times the rate the bench
    has ever shown.
    """

    def __init__(self, n_breaths: float = 2.0, hysteresis_fraction: float = 0.15,
                 reference_window_s: float = 30.0) -> None:
        self.n_breaths = max(1, int(n_breaths))
        self.hysteresis_fraction = float(hysteresis_fraction)
        # The crossing level and the hysteresis band are computed over a trailing window, not
        # over everything seen. Against the whole history any base motion in the record --
        # even one step -- inflates max-min far past the breathing amplitude, the band grows
        # with it, and the signal then never crosses: replaying run 20260903-152502's whole
        # standoff phase this way detects *zero* breaths in 66s of clearly breathing data.
        self.reference_window_s = float(reference_window_s)
        self._t: deque[float] = deque()
        self._y: deque[float] = deque()
        self._peaks: list[float] = []
        self._starts: list[float] = []
        self._current_peak: float | None = None
        self._current_start: float | None = None
        self._above = False

    def reset(self) -> None:
        """Forget everything. Called after each base step, so no sample from before the
        move can contribute to the measurement that judges where the move landed."""
        self._t.clear()
        self._y.clear()
        self._peaks.clear()
        self._starts.clear()
        self._current_peak = None
        self._current_start = None
        self._above = False

    def add(self, t: float, y: float) -> None:
        self._t.append(t)
        self._y.append(y)
        while self._t and (t - self._t[0]) > self.reference_window_s:
            self._t.popleft()
            self._y.popleft()
        if len(self._y) < 8:
            return

        # Reference the running mean rather than a fixed level: a real subject profile
        # carries ~1.5mm of slow baseline wander (session 009), which a fixed threshold
        # would eventually sit outside entirely.
        values = np.fromiter(self._y, dtype=float)
        mean = float(values.mean())
        amplitude = float(values.max() - values.min())
        band = self.hysteresis_fraction * amplitude

        if self._above:
            if y < mean - band:
                # Breath complete: bank its peak and wait for the next upward crossing.
                if self._current_peak is not None and self._current_start is not None:
                    self._peaks.append(self._current_peak)
                    self._starts.append(self._current_start)
                self._current_peak = None
                self._current_start = None
                self._above = False
            elif self._current_peak is None or y > self._current_peak:
                self._current_peak = y
        elif y > mean + band:
            self._above = True
            self._current_peak = y
            self._current_start = t

    @property
    def breaths(self) -> int:
        """Completed breaths observed since the last reset."""
        return len(self._peaks)

    @property
    def ready(self) -> bool:
        return self.breaths >= self.n_breaths

    @property
    def peak(self) -> float:
        """Mean of the last ``n_breaths`` per-breath peaks. Unbiased; see the class docstring."""
        if not self._peaks:
            return 0.0
        recent = self._peaks[-self.n_breaths:]
        return float(np.mean(recent))

    @property
    def peak_spread(self) -> float:
        """Peak-to-peak spread of the breaths being averaged.

        How much the subject's own breathing varied over the measurement, which is the floor
        on how tightly standoff can position. Worth reporting next to the tolerance.
        """
        recent = self._peaks[-self.n_breaths:]
        return float(max(recent) - min(recent)) if len(recent) >= 2 else 0.0

    @property
    def period_s(self) -> float | None:
        """Mean detected breath period, or None before two breaths have started."""
        if len(self._starts) < 2:
            return None
        starts = self._starts[-(self.n_breaths + 1):]
        if len(starts) < 2:
            return None
        return float((starts[-1] - starts[0]) / (len(starts) - 1))
