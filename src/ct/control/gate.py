"""The firing gate: may the needle move, right now?

This is the module ``CLAUDE.md`` has been pointing at since session 001 — the thing that
consumes ``(s_hat, P)``. It is also the concrete form of the paper's methodological
argument: choosing an EKF over WFLC's LMS buys an explicit covariance, and
:meth:`FiringGate.evaluate` spends it. An LMS predictor could answer "where will the skin
be"; it could not answer "and how sure are you", which is the question that decides
whether a needle moves.

Three conditions, all of which must hold:

1. **The forecast lands at end-exhale.** Not "now is exhale" — *``h`` seconds from now* is
   exhale, because that is when the needle arrives.
2. **The forecast is confident enough.** ``sqrt(forecast_variance(h))`` under threshold.
3. **One firing per breath.** A refractory interlock, so a wide window cannot produce a
   burst of increments inside one cycle.

Why "end-exhale" is a fraction of excursion, not an angle
---------------------------------------------------------

The obvious implementation is a window on ``theta``. It is also wrong in a way that only
shows up on real data: the phase at which the waveform bottoms out depends on ``phi_1``
and on every harmonic's contribution, so a fixed ``theta`` window means something
different for every patient and drifts as the model adapts. Instead the gate evaluates the
tracked model over one full cycle, finds its actual minimum and maximum, and asks whether
the forecast value sits within the bottom ``exhale_band_frac`` of that excursion. That is
waveform-relative, so it means the same thing for any breathing shape.

**Sign.** The tracked signal is tactile *deflection*: the sensor presses harder as the
chest expands, so inhale is the maximum and end-exhale is the **minimum**. The band is
therefore at the bottom. Getting this backwards would fire every insertion at peak inhale,
against the one clinical premise the whole design rests on.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ct.layout import StateLayout
from ct.tracking.measurement import advance_phase, measurement

#: Samples per breath used to find the waveform's extrema. 64 puts the turning point
#: within about 6 degrees of phase — far finer than a band test needs, and this runs on
#: every tick, so the resolution is not free.
CYCLE_SAMPLES = 64


def cycle_extrema(s: np.ndarray, layout: StateLayout, n: int = CYCLE_SAMPLES) -> tuple[float, float]:
    """Minimum and maximum of the tracked waveform over one full breath.

    Evaluated from the model rather than from recent history: the model is what the
    forecast is drawn from, so the comparison stays self-consistent even when the last
    breath was atypical.
    """
    omega_r = float(s[layout.omega])
    if omega_r <= 0:
        value = measurement(s, layout)
        return value, value
    period = 2.0 * np.pi / omega_r
    offsets = np.linspace(0.0, period, n, endpoint=False)
    values = np.array([measurement(advance_phase(s, layout, float(dt)), layout) for dt in offsets])
    return float(values.min()), float(values.max())


@dataclass(frozen=True)
class GateDecision:
    """Why the gate said what it said.

    Every field lands in telemetry. A run that fired at the wrong moment, or never fired,
    is diagnosed by reading these back — which is why the decision is a record rather than
    a bare bool.
    """

    fire: bool
    t: float
    reason: str
    forecast: float = 0.0
    forecast_std: float = 0.0
    y_min: float = 0.0
    y_max: float = 0.0
    band_top: float = 0.0
    in_window: bool = False
    confident: bool = False
    refractory_ok: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "fire": self.fire,
            "reason": self.reason,
            "forecast": self.forecast,
            "forecast_std": self.forecast_std,
            "band_top": self.band_top,
            "in_window": self.in_window,
            "confident": self.confident,
            "refractory_ok": self.refractory_ok,
        }


class FiringGate:
    """Decides whether the needle may move on this tick."""

    def __init__(
        self,
        exhale_band_frac: float,
        max_forecast_std: float,
        *,
        refractory_breaths: float = 0.9,
        min_excursion: float = 1e-6,
    ) -> None:
        if not 0 < exhale_band_frac <= 1:
            raise ValueError(f"exhale_band_frac must be in (0, 1], got {exhale_band_frac}")
        if max_forecast_std <= 0:
            raise ValueError("max_forecast_std must be positive")
        self.exhale_band_frac = float(exhale_band_frac)
        self.max_forecast_std = float(max_forecast_std)
        self.refractory_breaths = float(refractory_breaths)
        """Fraction of a breath that must pass between firings.

        Just under a whole breath, because the intent is *one bite per cycle*. Half a
        breath looks like enough — the exhale band is only open once per cycle — but the
        band has width, and at 0.5 the run took 6 increments over 4.9 breaths. Sitting
        just short of 1.0 rather than at it leaves room for the tracked period to be
        slightly off without skipping a breath entirely.
        """

        self.min_excursion = float(min_excursion)
        self._last_fire_t: float | None = None
        self.evaluations = 0
        self.firings = 0

    def reset(self) -> None:
        self._last_fire_t = None

    def evaluate(
        self,
        t: float,
        tracker: Any,
        h: float,
        extrema: tuple[float, float] | None = None,
    ) -> GateDecision:
        """Apply all three conditions and say which one, if any, refused.

        ``extrema`` lets a caller pass in a cycle sweep it has already paid for this tick;
        omitted, the sweep happens here.
        """
        self.evaluations += 1
        s, _P = tracker.state
        layout = tracker.layout
        omega_r = float(s[layout.omega])

        forecast = float(tracker.forecast(h))
        forecast_std = float(np.sqrt(max(0.0, tracker.forecast_variance(h))))
        y_min, y_max = extrema if extrema is not None else cycle_extrema(s, layout)
        excursion = y_max - y_min

        if excursion < self.min_excursion:
            # A flat model means the estimator has not found a breath yet. Refusing is the
            # only safe answer: with no excursion, "end-exhale" is not defined.
            return GateDecision(
                fire=False, t=t, reason="waveform has no excursion; model not converged",
                forecast=forecast, forecast_std=forecast_std, y_min=y_min, y_max=y_max,
            )

        band_top = y_min + self.exhale_band_frac * excursion
        in_window = forecast <= band_top
        confident = forecast_std <= self.max_forecast_std
        refractory_ok = self._refractory_ok(t, omega_r)

        fire = in_window and confident and refractory_ok
        if fire:
            self._last_fire_t = t
            self.firings += 1

        return GateDecision(
            fire=fire,
            t=t,
            reason=self._reason(in_window, confident, refractory_ok, forecast_std),
            forecast=forecast,
            forecast_std=forecast_std,
            y_min=y_min,
            y_max=y_max,
            band_top=band_top,
            in_window=in_window,
            confident=confident,
            refractory_ok=refractory_ok,
        )

    def _refractory_ok(self, t: float, omega_r: float) -> bool:
        if self._last_fire_t is None or omega_r <= 0:
            return True
        period = 2.0 * np.pi / omega_r
        return (t - self._last_fire_t) >= self.refractory_breaths * period

    def _reason(self, in_window: bool, confident: bool, refractory_ok: bool, std: float) -> str:
        if in_window and confident and refractory_ok:
            return "fire: forecast at end-exhale and within uncertainty budget"
        if not refractory_ok:
            return "holding: already fired this breath"
        if not in_window:
            return "holding: forecast not at end-exhale"
        return f"holding: forecast std {std:.4f} mm exceeds budget {self.max_forecast_std:.4f} mm"

    @property
    def stats(self) -> dict[str, Any]:
        return {
            "exhale_band_frac": self.exhale_band_frac,
            "max_forecast_std": self.max_forecast_std,
            "evaluations": self.evaluations,
            "firings": self.firings,
        }
