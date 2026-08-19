"""The procedure-state interface, and the bookkeeping every state shares."""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from ct.control.context import ProcedureContext
from ct.control.state import ProcedureState, Transition


@runtime_checkable
class ProcedureStateImpl(Protocol):
    """One state of the insertion procedure.

    Structural, like every other boundary in this package: a replacement state needs
    matching methods and nothing else, so an alternative approach strategy plugs in
    through ``@register_state`` and a config string.
    """

    state: ProcedureState

    def enter(self, ctx: ProcedureContext) -> None:
        """Called once on entry. Set up, and command a safe initial output."""
        ...

    def update(self, ctx: ProcedureContext) -> Transition | None:
        """Called every tick. Return a transition to move on, or ``None`` to stay."""
        ...

    def exit(self, ctx: ProcedureContext) -> None:
        """Called once on leaving, including on fault. Leave the rig safe."""
        ...


class BaseState:
    """Shared entry-time bookkeeping and timeout handling.

    Inheriting is optional — the boundary is the Protocol above — but every built-in state
    does, because all of them want the same two things: to know how long they have been
    running, and to fail rather than hang if their exit condition never arrives.
    """

    state: ProcedureState = ProcedureState.IDLE
    timeout_s: float = 0.0

    def __init__(self) -> None:
        self.entered_at: float = 0.0
        self.notes: dict[str, Any] = {}

    def enter(self, ctx: ProcedureContext) -> None:
        self.entered_at = ctx.state.t
        self.notes = {}

    def update(self, ctx: ProcedureContext) -> Transition | None:  # pragma: no cover - abstract
        raise NotImplementedError

    def exit(self, ctx: ProcedureContext) -> None:
        """Default: leave both axes holding position. Safe, and usually right."""
        ctx.base.hold()
        ctx.needle.hold()

    def elapsed(self, ctx: ProcedureContext) -> float:
        return ctx.state.t - self.entered_at

    def _arrived(
        self,
        ctx: ProcedureContext,
        tol_mm: float,
        stall_velocity_mm_s: float = 0.0,
        stall_time_s: float = 0.0,
        axis: str = "needle",
    ) -> bool:
        """Whether the axis has reached its target, or has stopped trying.

        Two ways to arrive, and the second is not a fudge. Under MIT-mode impedance
        control a loaded axis settles with a steady-state error of about
        ``load / kp`` — the needle pushing into tissue simply stops short, permanently.
        Waiting for a tight tolerance that the physics forbids turns a successful
        insertion into a timeout. So: within tolerance, *or* not moving any more.

        The distinction is recorded in :attr:`notes` so a run can be read back to see
        which one happened, and by how much the needle fell short.
        """
        target = self.notes.get("arrival_target_mm")
        if target is None:
            return False
        position = getattr(ctx.state, f"{axis}_mm")
        error = abs(position - target)
        if error <= tol_mm:
            self.notes["arrival"] = "in_tolerance"
            return True

        if stall_velocity_mm_s <= 0 or stall_time_s <= 0:
            return False
        moving = abs(getattr(ctx, axis).velocity_mm_s) > stall_velocity_mm_s
        if moving:
            self.notes.pop("stalled_since", None)
            return False
        since = self.notes.setdefault("stalled_since", ctx.state.t)
        if ctx.state.t - since < stall_time_s:
            return False
        self.notes["arrival"] = "stalled"
        self.notes["shortfall_mm"] = error
        ctx.log(
            "axis_stalled",
            {"axis": axis, "target_mm": target, "reached_mm": position,
             "shortfall_mm": error,
             "hint": "steady-state error under impedance control is roughly load/kp; "
                     "raise kp or widen arrival_tol_mm"},
        )
        return True

    def timed_out(self, ctx: ProcedureContext) -> Transition | None:
        """A fault transition once the state's budget is spent, or ``None``.

        Every state gets one. A procedure that waits forever for a condition that will
        never arrive is worse than one that stops and says which condition it was.
        """
        if self.timeout_s <= 0:
            return None
        if self.elapsed(ctx) < self.timeout_s:
            return None
        return Transition(
            ProcedureState.FAULT,
            f"{self.state.value} timed out after {self.timeout_s:g} s "
            f"({self._timeout_detail(ctx)})",
        )

    def _timeout_detail(self, ctx: ProcedureContext) -> str:
        """What the state was still waiting for. Overridden where it helps."""
        return "exit condition never met"
