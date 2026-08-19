"""Operator-commanded exits, and the two resting states.

WITHDRAW and RETRACT are deliberately separate and deliberately ordered: the needle comes
out of the tissue before the base comes off the phantom. Backing the base away with the
needle still inserted would drag the needle sideways through tissue, so RETRACT refuses to
run while the needle is extended past its standoff.

Neither is part of the automatic sequence. They are entered on command, which is what the
user asked for: finish the insertion, then ask for the needle back, then ask for the base
back.
"""

from __future__ import annotations

from ct.control.context import ProcedureContext
from ct.control.state import ProcedureState, Transition
from ct.control.states.base import BaseState
from ct.registry import register_state

ARRIVAL_TOL_MM = 0.2


@register_state("withdraw")
class WithdrawState(BaseState):
    """Retract the needle to zero, leaving the base seated."""

    state = ProcedureState.WITHDRAW
    timeout_s = 120.0

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        ctx.base.hold()
        ctx.log("withdraw_enter", {"from_needle_mm": ctx.state.needle_mm})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired

        ctx.base.hold()
        target = ctx.geometry.needle.travel_mm[0]
        ctx.needle.move_to(target, v_max_mm_s=ctx.config.withdraw_speed_mm_s)

        if abs(ctx.state.needle_mm - target) > ARRIVAL_TOL_MM:
            return None
        ctx.log("withdraw_complete", {"needle_mm": ctx.state.needle_mm})
        return Transition(ProcedureState.DONE, "needle withdrawn")

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        return f"needle still at {ctx.state.needle_mm:.2f} mm"


@register_state("retract")
class RetractState(BaseState):
    """Back the base away from the phantom to its home position."""

    state = ProcedureState.RETRACT
    timeout_s = 180.0

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        ctx.needle.hold()
        ctx.log("retract_enter", {"from_base_mm": ctx.state.base_mm})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        expired = self.timed_out(ctx)
        if expired is not None:
            return expired

        # Refuse to move the base while the needle is still out. Dragging an inserted
        # needle sideways is the one motion in this procedure that is unambiguously bad.
        needle_home = ctx.geometry.needle.travel_mm[0]
        if ctx.state.needle_mm > needle_home + ARRIVAL_TOL_MM:
            return Transition(
                ProcedureState.FAULT,
                f"refusing to retract the base with the needle extended to "
                f"{ctx.state.needle_mm:.2f} mm; run WITHDRAW first",
            )

        ctx.needle.hold()
        target = ctx.geometry.base.travel_mm[0]
        ctx.base.move_to(target, v_max_mm_s=ctx.config.retract_speed_mm_s)

        if abs(ctx.state.base_mm - target) > ARRIVAL_TOL_MM:
            return None
        ctx.log("retract_complete", {"base_mm": ctx.state.base_mm})
        return Transition(ProcedureState.DONE, "base retracted to home")

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        return f"base still at {ctx.state.base_mm:.2f} mm"


@register_state("idle")
class IdleState(BaseState):
    """Nothing commanded. The only state it is safe to start or stop in."""

    state = ProcedureState.IDLE

    def update(self, ctx: ProcedureContext) -> Transition | None:
        ctx.base.hold()
        ctx.needle.hold()
        return None


@register_state("done")
class DoneState(BaseState):
    """Finished. Axes hold; the loop stops on the next tick."""

    state = ProcedureState.DONE

    def update(self, ctx: ProcedureContext) -> Transition | None:
        ctx.base.hold()
        ctx.needle.hold()
        return None


@register_state("fault")
class FaultState(BaseState):
    """Something tripped. Motion stopped, latched until an operator resets.

    ``stop`` rather than ``disable``: a disabled motor goes limp, and a limp base drops
    whatever it was holding against the phantom. Commanding zero motion while staying
    energised leaves the rig where it is, which is the safer of the two on a rig with a
    needle in tissue.
    """

    state = ProcedureState.FAULT

    def enter(self, ctx: ProcedureContext) -> None:
        super().enter(ctx)
        ctx.base.stop()
        ctx.needle.stop()
        ctx.log("fault_enter", {"trips": [str(t) for t in ctx.safety.trips]})

    def update(self, ctx: ProcedureContext) -> Transition | None:
        ctx.base.stop()
        ctx.needle.stop()
        return None

    def exit(self, ctx: ProcedureContext) -> None:
        """No-op: leave the axes stopped, not holding."""
