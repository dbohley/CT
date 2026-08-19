"""State 4 — ADVANCE: float in tissue, and take one bite per breath at end-exhale.

The needle is in tissue and the motor is backdrivable, so between increments it is left at
zero stiffness and simply **rides with the tissue** as the phantom breathes. That is the
whole idea: a needle that holds a fixed position while tissue moves around it is shearing
through that tissue every cycle, whereas a floating needle moves with it and only advances
when told to.

Advancing only near end-exhale is the clinical premise the procedure is built on — exhale
is the more reproducible end of the breathing cycle, so a target's position is most
repeatable there. Each increment is therefore gated on the same three conditions INSERT
used, with one important difference in how the move is computed.

**Increments are relative to the measured position, not to a stored reference.** The
needle has been floating, so wherever it is now is where the tissue has carried it. Adding
the increment to a remembered command would silently undo all the riding it just did and
put the tip somewhere nobody asked for. Reading the encoder and adding from there is the
only version that means "two more millimetres than you currently are".
"""

from __future__ import annotations

from ct.control.context import ProcedureContext
from ct.control.gate import FiringGate
from ct.control.state import ProcedureState, Transition
from ct.control.states.base import BaseState
from ct.registry import register_state

ARRIVAL_TOL_MM = 0.15


@register_state("advance")
class AdvanceState(BaseState):
    state = ProcedureState.ADVANCE

    def __init__(self) -> None:
        super().__init__()
        self.gate: FiringGate | None = None
        self.increments = 0
        self.moving = False
        self.target_needle_mm: float | None = None
        self.settle_until = 0.0
        self.depth_history: list[float] = []
        self.increment_times: list[float] = []
        """When each increment fired. The spacing is the refractory interlock, observable."""

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        cfg = ctx.config.advance
        self.timeout_s = cfg.timeout_s
        self.increments = 0
        self.moving = False
        self.depth_history = []
        self.increment_times = []
        self.gate = FiringGate(
            exhale_band_frac=cfg.exhale_band_frac,
            max_forecast_std=cfg.max_forecast_std_mm,
        )
        ctx.base.hold()
        self._float(ctx)
        ctx.log(
            "advance_enter",
            {"total_depth_mm": cfg.total_depth_mm, "increment_mm": cfg.increment_mm},
        )

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired
        if not ctx.has_model:
            return Transition(ProcedureState.FAULT, "ADVANCE reached without a tracked model")

        ctx.base.hold()

        depth = self._depth(ctx)
        if depth is not None and depth >= ctx.config.advance.total_depth_mm:
            ctx.log("advance_target_reached",
                    {"depth_mm": depth, "increments": self.increments})
            return Transition(
                ProcedureState.DONE,
                f"reached {depth:.2f} mm in {self.increments} increments",
            )

        if self.moving:
            return self._finish_increment(ctx)
        return self._maybe_increment(ctx, depth)

    # -- floating and gating ---------------------------------------------------

    def _maybe_increment(self, ctx: ProcedureContext, depth: float | None) -> Transition | None:
        cfg = ctx.config.advance
        assert self.gate is not None

        if self.increments >= cfg.max_increments:
            return Transition(
                ProcedureState.FAULT,
                f"{self.increments} increments without reaching {cfg.total_depth_mm:g} mm; "
                "the needle may be slipping or the depth measurement may be wrong",
            )

        decision = self.gate.evaluate(
            ctx.state.t, ctx.tracker, ctx.horizon(), ctx.cycle_extrema()
        )
        if not decision.fire:
            self._float(ctx)
            return None

        # Relative to *measured* position. See the module docstring.
        remaining = cfg.total_depth_mm - depth if depth is not None else cfg.increment_mm
        step = min(cfg.increment_mm, max(remaining, 0.0))
        if step <= 0:
            return None

        target = ctx.state.needle_mm + step
        lo, hi = ctx.geometry.needle.travel_mm
        if target > hi:
            return Transition(
                ProcedureState.FAULT,
                f"next increment needs needle at {target:.2f} mm, past its travel limit "
                f"{hi:.1f} mm",
            )

        self.target_needle_mm = target
        self.notes["arrival_target_mm"] = target
        self.notes.pop("stalled_since", None)
        self.moving = True
        self.increments += 1
        self.increment_times.append(ctx.state.t)
        ctx.needle.move_to(target, v_max_mm_s=cfg.increment_speed_mm_s)
        ctx.log(
            "advance_increment",
            {**decision.to_dict(), "n": self.increments, "step_mm": step,
             "from_mm": ctx.state.needle_mm, "target_mm": target, "depth_mm": depth},
        )
        return None

    def _finish_increment(self, ctx: ProcedureContext) -> Transition | None:
        """Hold the commanded position briefly, then release back to float."""
        assert self.target_needle_mm is not None
        ctx.needle.move_to(self.target_needle_mm,
                           v_max_mm_s=ctx.config.advance.increment_speed_mm_s)

        cfg = ctx.config.insert
        arrived = self._arrived(ctx, ARRIVAL_TOL_MM, cfg.stall_velocity_mm_s,
                                cfg.stall_time_s)
        if arrived and not self.settle_until:
            self.settle_until = ctx.state.t + ctx.config.advance.settle_s
        if not self.settle_until or ctx.state.t < self.settle_until:
            return None

        self.moving = False
        self.settle_until = 0.0
        depth = self._depth(ctx)
        if depth is not None:
            self.depth_history.append(depth)
        ctx.log("advance_settled", {"n": self.increments, "depth_mm": depth,
                                    "needle_mm": ctx.state.needle_mm})
        self._float(ctx)
        return None

    def _float(self, ctx: ProcedureContext) -> None:
        """Release the needle to ride with the tissue."""
        try:
            ctx.needle.float_free()
        except Exception as exc:  # noqa: BLE001
            # A codec that cannot float is a configuration problem, surfaced once and
            # loudly rather than every tick. Holding position is the safe fallback, but it
            # is not what this state is supposed to do, so it is logged as a degradation.
            if "float_unsupported" not in self.notes:
                self.notes["float_unsupported"] = str(exc)
                ctx.log("advance_float_unsupported", {"detail": str(exc)})
            ctx.needle.hold()

    def _depth(self, ctx: ProcedureContext) -> float | None:
        """Insertion depth against the end-exhale skin position.

        Not the instantaneous one. Between increments the needle floats and rides with the
        tissue, so depth measured against a moving skin swings by the whole breathing
        excursion and the termination test would fire on whichever tick happened to catch
        a peak. The end-exhale datum is stable and is the same reference every increment
        was commanded against.
        """
        return ctx.depth_at_exhale()

    def exit(self, ctx: ProcedureContext) -> None:
        ctx.base.hold()
        ctx.needle.hold()

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        depth = self._depth(ctx)
        shown = f"{depth:.2f}" if depth is not None else "unknown"
        gate = self.gate
        return (
            f"{self.increments} increments reached depth {shown} mm of "
            f"{ctx.config.advance.total_depth_mm:g} mm; gate opened "
            f"{gate.firings if gate else 0} times in {gate.evaluations if gate else 0} evaluations"
        )
