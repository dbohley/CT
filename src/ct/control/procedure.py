"""The state machine: one tick's worth of decisions, in order.

Every tick, in this sequence and no other:

1. **Read the world.** Fold received frames into the axes and sensors, and step the
   estimator. Happens before anything else so every subsequent decision sees the same
   snapshot.
2. **Check safety.** Before commanding anything. A tripped monitor goes straight to FAULT,
   which stops motion — checking after commanding would mean acting on a rig already known
   to be unsafe.
3. **Run the active state**, which issues exactly one command per axis.
4. **Record it.**

The order is the design. Read, judge, act, log — never act then judge.

Transitions carry the reason they fired, and the reason lands in telemetry. A run that
ended in the wrong place is then readable without re-deriving what the criteria were
doing, which is the difference between a state machine you can debug and one you have to
re-instrument.
"""

from __future__ import annotations

from typing import Any

from ct.control.context import ProcedureContext
from ct.control.state import ProcedureState, Transition
from ct.control.states import DEFAULT_STATE_NAMES
from ct.registry import build_state


class Procedure:
    """Drives the states, and owns the transition history."""

    def __init__(
        self,
        ctx: ProcedureContext,
        *,
        start: ProcedureState = ProcedureState.APPROACH,
        state_names: dict[str, str] | None = None,
        stop_at: ProcedureState | None = None,
    ) -> None:
        self.ctx = ctx
        self.stop_at = stop_at
        """Finish early at this state, for bench testing one piece at a time."""

        names = {**DEFAULT_STATE_NAMES, **(state_names or {})}
        self.states: dict[ProcedureState, Any] = {
            ProcedureState(key): build_state(name) for key, name in names.items()
        }

        self.current = start
        self.history: list[dict[str, Any]] = []
        self._entered = False
        self.finished = False

    # -- running ---------------------------------------------------------------

    def tick(self, t: float, frames: dict[str, list[tuple[float, int, bytes]]],
             deadline_misses: int = 0) -> dict[str, Any]:
        """One pass. Returns the record for this tick."""
        state = self.ctx.update(t, frames)
        self.ctx.procedure_state = self.current

        if not self._entered:
            self._enter(self.current)

        trip = self.ctx.safety.check(state, self.current, deadline_misses=deadline_misses)
        if trip is not None and self.current is not ProcedureState.FAULT:
            self._transition(Transition(ProcedureState.FAULT, str(trip)))
            return self._record(state)

        impl = self.states[self.current]
        try:
            transition = impl.update(self.ctx)
        except Exception as exc:  # noqa: BLE001
            # A state raising is a bug, but it must still leave the rig safe. Faulting
            # stops the axes; propagating would leave them holding their last command.
            self.ctx.safety.trip("state_error", f"{type(exc).__name__}: {exc}", t)
            self._transition(Transition(ProcedureState.FAULT, f"{self.current.value} raised: {exc}"))
            return self._record(state)

        if transition is not None:
            self._transition(transition)

        return self._record(state)

    def _enter(self, state: ProcedureState) -> None:
        self.states[state].enter(self.ctx)
        self._entered = True
        self.ctx.procedure_state = state

    def _transition(self, transition: Transition) -> None:
        previous = self.current
        self.states[previous].exit(self.ctx)
        self.ctx.log(
            "transition",
            {"from": previous.value, "to": transition.to.value, "why": transition.why},
        )
        self.history.append(
            {
                "t": self.ctx.state.t,
                "from": previous.value,
                "to": transition.to.value,
                "why": transition.why,
            }
        )
        self.current = transition.to
        self._enter(self.current)

        if self.current.is_terminal or self.current is self.stop_at:
            self.finished = True

    def _record(self, state: Any) -> dict[str, Any]:
        record: dict[str, Any] = {
            "t": state.t,
            "state": self.current.value,
            "base_mm": state.base_mm,
            "needle_mm": state.needle_mm,
            "tactile_mm": state.tactile_mm,
            "tof_mm": state.tof_mm,
            "skin_x": state.skin_x,
            "in_contact": state.in_contact,
        }
        if state.stale:
            record["stale"] = list(state.stale)
        if self.ctx.has_model:
            h = self.ctx.horizon()
            record.update(
                {
                    "h": h,
                    "omega_r": self.ctx.omega_r,
                    "forecast_mm": self.ctx.forecast_deflection(h),
                    "forecast_std_mm": self.ctx.forecast_std(h),
                    "nis": self.ctx.last_nis,
                }
            )
            if state.skin_x is not None:
                record["depth_mm"] = self.ctx.insertion_depth_mm()
        return record

    # -- commands --------------------------------------------------------------

    def command(self, state: ProcedureState, why: str = "operator command") -> None:
        """Force a transition. How WITHDRAW and RETRACT are entered."""
        if self.current is ProcedureState.FAULT and state is not ProcedureState.IDLE:
            raise RuntimeError(
                "cannot leave FAULT by command; the trip is latching and needs an operator "
                "reset. Start a new run once the cause is understood."
            )
        self._transition(Transition(state, why))
        self.finished = self.current.is_terminal

    @property
    def summary(self) -> dict[str, Any]:
        return {
            "final_state": self.current.value,
            "finished": self.finished,
            "transitions": self.history,
            "safety": self.ctx.safety.stats,
            "context": self.ctx.stats,
        }
