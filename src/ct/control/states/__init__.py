"""The procedure states.

Importing this package registers all of them, the same way importing ``ct.sources``
registers the built-in signal sources. A replacement — a different approach strategy, a
gated withdrawal — plugs in with ``@register_state`` and a config string.
"""

from __future__ import annotations

from ct.control.states.advance import AdvanceState
from ct.control.states.approach import ApproachState
from ct.control.states.base import BaseState, ProcedureStateImpl
from ct.control.states.estimate import EstimateState
from ct.control.states.exit_states import (
    DoneState,
    FaultState,
    IdleState,
    RetractState,
    WithdrawState,
)
from ct.control.states.insert import InsertState

__all__ = [
    "ProcedureStateImpl",
    "BaseState",
    "ApproachState",
    "EstimateState",
    "InsertState",
    "AdvanceState",
    "WithdrawState",
    "RetractState",
    "IdleState",
    "DoneState",
    "FaultState",
]

#: Default name for each :class:`~ct.control.state.ProcedureState`. Overridable per run
#: through ``procedure.states``, which is how an alternative implementation gets swapped in.
DEFAULT_STATE_NAMES: dict[str, str] = {
    "idle": "idle",
    "approach": "approach",
    "estimate": "estimate",
    "insert": "insert",
    "advance": "advance",
    "withdraw": "withdraw",
    "retract": "retract",
    "done": "done",
    "fault": "fault",
}
