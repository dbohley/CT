"""State 3 — INSERT: hold standoff against the receding skin, then drive in at end-exhale.

Two things happen at once, and they are the two halves of the whole design meeting for the
first time.

**Tracking (feedforward + feedback).** The skin is moving; the needle holds a constant
standoff from it. The reference comes from the *forecast* — where the skin will be in ``h``
seconds, not where it is now — and the lead compensator closes the remaining error. This is
the separation ``CLAUDE.md`` insists on: the breathing forecast is open-loop disturbance
feedforward, the servo is a closed loop, and they meet only here, at the reference.

**Firing.** When the forecast says the skin will be at end-exhale in ``h`` seconds, and the
forecast is confident enough, drive the needle to ``initial_depth_mm`` past the predicted
skin position. The horizon is what makes this work: ``h`` already contains ``T_ins``, so
the forecast is evaluated at the moment the needle *arrives*, not the moment it departs.

The target is computed from the forecast rather than the measurement for the same reason.
Aiming at where the skin is now would put the tip wherever the skin had moved to by the
time it got there — which, at end-exhale, is the flattest and most forgiving part of the
cycle, but not zero.
"""

from __future__ import annotations

from ct.control.context import ProcedureContext
from ct.control.gate import FiringGate
from ct.control.state import ProcedureState, Transition
from ct.control.states.base import BaseState
from ct.registry import register_state


@register_state("insert")
class InsertState(BaseState):
    state = ProcedureState.INSERT

    def __init__(self) -> None:
        super().__init__()
        self.gate: FiringGate | None = None
        self.fired = False
        self.fired_at: float | None = None
        self.target_needle_mm: float | None = None
        self.target_skin_x: float | None = None

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        cfg = ctx.config.insert
        self.timeout_s = cfg.timeout_s
        self.fired = False
        self.fired_at = None
        self.target_needle_mm = None
        self.gate = FiringGate(
            exhale_band_frac=cfg.exhale_band_frac,
            max_forecast_std=cfg.max_forecast_std_mm,
        )
        ctx.servo.reset(u0=0.0, y0=0.0)
        ctx.base.hold()
        ctx.log("insert_enter", {"initial_depth_mm": cfg.initial_depth_mm, "h": ctx.horizon()})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired
        if not ctx.has_model:
            return Transition(ProcedureState.FAULT, "INSERT reached without a tracked model")

        ctx.base.hold()

        trip = ctx.safety.check_insert_preconditions(ctx.state, ctx.state.t)
        if trip is not None:
            return Transition(ProcedureState.FAULT, trip.detail)

        if self.fired:
            return self._drive(ctx)
        return self._wait_and_track(ctx)

    # -- before firing ---------------------------------------------------------

    def _wait_and_track(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.insert
        assert self.gate is not None

        h = ctx.horizon()
        decision = self.gate.evaluate(ctx.state.t, ctx.tracker, h, ctx.cycle_extrema())

        if decision.fire:
            skin_x = ctx.forecast_skin_x(h)
            target = ctx.needle_mm_for_depth(cfg.initial_depth_mm, skin_x)
            lo, hi = ctx.geometry.needle.travel_mm
            if not lo <= target <= hi:
                return Transition(
                    ProcedureState.FAULT,
                    f"insertion to {cfg.initial_depth_mm:g} mm needs needle at {target:.2f} mm, "
                    f"outside travel [{lo:.1f}, {hi:.1f}] mm",
                )
            self.fired = True
            self.fired_at = ctx.state.t
            self.target_needle_mm = target
            self.target_skin_x = skin_x
            self.notes["arrival_target_mm"] = target
            ctx.insertion_reference_skin_x = skin_x
            ctx.log("insert_fired", {**decision.to_dict(), "h": h,
                                     "target_needle_mm": target, "forecast_skin_x": skin_x})
            ctx.needle.move_to(target, v_max_mm_s=cfg.drive_speed_mm_s)
            return None

        if cfg.track_standoff:
            self._hold_standoff(ctx, h)
        else:
            ctx.needle.hold()
        return None

    def _hold_standoff(self, ctx: ProcedureContext, h: float) -> None:
        """Keep a constant gap to the skin while waiting for the window.

        Feedforward from the forecast sets the reference; the lead compensator supplies
        the correction for whatever the axis has not managed to follow. Commanding the
        forecast alone would leave exactly the residual lag that ``tau_cl`` describes.
        """
        standoff = ctx.config.approach.standoff_mm
        skin_x = ctx.forecast_skin_x(h)
        reference_mm = ctx.geometry.needle_mm_for_tip_at(skin_x - standoff, ctx.state.base_mm)

        error = reference_mm - ctx.state.needle_mm
        correction = ctx.servo.update(error, limit=ctx.geometry.needle.v_max_mm_s)
        command = reference_mm + correction

        lo, hi = ctx.geometry.needle.travel_mm
        command = min(max(command, lo), hi)
        ctx.needle.move_to(command, v_max_mm_s=ctx.config.insert.drive_speed_mm_s)

    # -- after firing ----------------------------------------------------------

    def _drive(self, ctx: ProcedureContext) -> Transition | None:
        cfg = ctx.config.insert
        assert self.target_needle_mm is not None
        ctx.needle.move_to(self.target_needle_mm, v_max_mm_s=cfg.drive_speed_mm_s)

        if not self._arrived(ctx, cfg.arrival_tol_mm, cfg.stall_velocity_mm_s,
                             cfg.stall_time_s):
            return None

        # Reported against end-exhale, the phase the insertion was aimed at. The
        # instantaneous depth is logged beside it because the difference between the two
        # *is* the breathing excursion, and seeing them diverge is the quickest check that
        # the needle is holding position rather than riding.
        depth = ctx.depth_at_exhale()
        instantaneous = ctx.insertion_depth_mm() if ctx.state.skin_x is not None else None
        drive_time = ctx.state.t - (self.fired_at or ctx.state.t)
        ctx.log(
            "insert_complete",
            {"needle_mm": ctx.state.needle_mm, "depth_mm": depth,
             "depth_instantaneous_mm": instantaneous,
             "drive_time_s": drive_time, "T_ins_configured": ctx.config.insert.timeout_s,
             "target_skin_x": self.target_skin_x},
        )
        shown = f"{depth:.2f}" if depth is not None else "unknown"
        return Transition(
            ProcedureState.ADVANCE,
            f"needle in tissue at {shown} mm depth (drive took {drive_time * 1e3:.0f} ms)",
        )

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        if self.fired:
            return (
                f"needle commanded to {self.target_needle_mm:.2f} mm but reached "
                f"{ctx.state.needle_mm:.2f} mm"
            )
        gate = self.gate
        evaluations = gate.evaluations if gate else 0
        return (
            f"firing gate never opened in {evaluations} evaluations; check "
            "procedure.insert.exhale_band_frac and max_forecast_std_mm"
        )
