"""State 2 — ESTIMATE: identify the breathing model and wait for the filter to converge.

The state with the least new code in it, deliberately. Everything it does was built and
validated in session 001: it collects a window of tactile signal, hands it to the
registered ``Identifier``, seeds the registered ``Tracker``, and waits. No estimator logic
lives here — this is a *consumer* of the pipeline, and if it needed anything more than the
existing interfaces, that would be evidence the estimator's boundaries were drawn wrong.

Once started, the tracker never stops. :meth:`ProcedureContext.update` steps it every tick
for the rest of the run, through INSERT and ADVANCE. ESTIMATE is only the state in which
we *wait* for it, and its exit criteria are the three questions worth asking before
trusting a forecast to move a needle:

1. Has it run long enough to have seen several breaths?
2. Is it statistically consistent — windowed NIS inside its band? NIS far below 1 means
   the filter is over-trusting its model, which ``CLAUDE.md`` records happening exactly
   when ``R`` came from the residual-variance fallback rather than a breath hold.
3. Is the forecast at the working horizon actually precise enough to act on?
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ct.control.context import ProcedureContext
from ct.control.state import ProcedureState, Transition
from ct.control.states.base import BaseState
from ct.registry import register_state


@register_state("estimate")
class EstimateState(BaseState):
    state = ProcedureState.ESTIMATE

    def __init__(self) -> None:
        super().__init__()
        self.identified_at: float | None = None
        self.nis_window: deque[float] = deque()
        self._last_nis_sample: int = 0

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        cfg = ctx.config.estimate
        self.timeout_s = cfg.timeout_s
        self.identified_at = None
        self.nis_window = deque(maxlen=cfg.nis_window)
        self._last_nis_sample = ctx.samples_tracked

        # Both axes park for the whole state. The base must not move — it is holding the
        # tactile sensor against the skin, and any base motion would be indistinguishable
        # from breathing in the signal being identified.
        ctx.base.hold()
        ctx.needle.hold()
        ctx.accumulator.clear()
        ctx.log("estimate_enter", {"calib_seconds": cfg.calib_seconds})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired

        ctx.base.hold()
        ctx.needle.hold()

        if not ctx.has_model:
            return self._maybe_identify(ctx)
        return self._check_convergence(ctx)

    # -- stage 1 ---------------------------------------------------------------

    def _maybe_identify(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.estimate
        if ctx.accumulator.duration < cfg.calib_seconds:
            return None

        try:
            ident = ctx.identify_and_start_tracker()
        except Exception as exc:  # noqa: BLE001 - reported as a fault, not a crash
            return Transition(
                ProcedureState.FAULT, f"identification failed: {type(exc).__name__}: {exc}"
            )

        self.identified_at = ctx.state.t
        diagnostics = dict(getattr(ident, "diagnostics", {}) or {})
        ctx.log(
            "estimate_identified",
            {
                "K": ident.K,
                "R": ident.R,
                "omega_hat": diagnostics.get("omega_hat"),
                "bpm_hat": diagnostics.get("bpm_hat"),
                "samples": len(ctx.accumulator),
                "jitter_fraction": ctx.accumulator.jitter_fraction(),
                "breath_hold_used": ctx.breath_hold_window is not None,
            },
        )
        return None

    # -- stage 2 convergence ---------------------------------------------------

    def _check_convergence(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.estimate

        # Only take one NIS reading per *tracked sample*, not per tick: the loop runs
        # faster than the sensor, and resampling the same innovation would make the window
        # look longer and steadier than it is.
        if ctx.samples_tracked != self._last_nis_sample and ctx.last_nis is not None:
            self.nis_window.append(ctx.last_nis)
            self._last_nis_sample = ctx.samples_tracked

        assert self.identified_at is not None
        settled_for = ctx.state.t - self.identified_at
        need = cfg.settle_breaths * ctx.breath_period
        if settled_for < need:
            return None

        if len(self.nis_window) < min(cfg.nis_window, 20):
            return None

        nis_mean = float(np.mean(self.nis_window))
        lo, hi = cfg.nis_band
        consistent = lo <= nis_mean <= hi

        h = ctx.horizon()
        std = ctx.forecast_std(h)
        precise = std <= cfg.max_forecast_std_mm

        if consistent and precise:
            ctx.log(
                "estimate_converged",
                {"nis_mean": nis_mean, "forecast_std_mm": std, "h": h,
                 "omega_r": ctx.omega_r, "settled_for_s": settled_for},
            )
            return Transition(
                ProcedureState.INSERT,
                f"model converged (NIS {nis_mean:.2f}, forecast std {std:.3f} mm at h={h:.3f} s)",
            )

        self.notes = {"nis_mean": nis_mean, "forecast_std_mm": std, "consistent": consistent,
                      "precise": precise}
        return None

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        if not ctx.has_model:
            return (
                f"only {ctx.accumulator.duration:.1f} s of signal accumulated of the "
                f"{ctx.config.estimate.calib_seconds:g} s needed; is the tactile sensor publishing?"
            )
        notes = self.notes or {}
        return (
            f"model identified but not converged: NIS {notes.get('nis_mean', float('nan')):.2f} "
            f"(band {ctx.config.estimate.nis_band}), forecast std "
            f"{notes.get('forecast_std_mm', float('nan')):.3f} mm "
            f"(budget {ctx.config.estimate.max_forecast_std_mm:g} mm)"
        )
