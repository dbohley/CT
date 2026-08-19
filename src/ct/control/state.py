"""The insertion procedure's states, and the transitions between them.

Deliberately dependency-free — it imports nothing from the rest of the package — so that
:mod:`ct.unknowns` can name states without dragging in the hardware layer.

Naming: these are ``ProcedureState``, never "phase". In this repo "phase" already means
``phi_k`` or ``theta``, and "Stage" already means identification/tracking. Overloading
either would make half the comments in ``control/`` ambiguous.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ProcedureState(Enum):
    """Where the procedure is.

    The four numbered states run in order and are the substance of the procedure. The
    rest are entry, operator-commanded exits, and failure.
    """

    IDLE = "idle"
    """Nothing commanded. Axes holding position. The only state safe to start or stop in."""

    APPROACH = "approach"
    """1. Drive the base in on ToF, make tactile contact, seat until the full breathing
    excursion is visible, then back off to standoff."""

    ESTIMATE = "estimate"
    """2. Base parked. Identify the harmonic model and run the EKF until it converges."""

    INSERT = "insert"
    """3. Hold standoff against the receding skin, then drive the needle in at end-exhale."""

    ADVANCE = "advance"
    """4. Float in tissue; advance one increment per breath at end-exhale until depth."""

    WITHDRAW = "withdraw"
    """Operator-commanded: retract the needle to zero, leaving the base seated."""

    RETRACT = "retract"
    """Operator-commanded: back the base away from the phantom to home."""

    DONE = "done"
    """Target depth reached, or a commanded exit completed. Axes holding."""

    FAULT = "fault"
    """A safety limit tripped or a state raised. Motion stopped; requires operator reset."""

    @property
    def is_terminal(self) -> bool:
        return self in (ProcedureState.DONE, ProcedureState.FAULT)

    @property
    def moves_needle(self) -> bool:
        """States that can command needle motion. Used by the dry-run guard."""
        return self in (
            ProcedureState.INSERT,
            ProcedureState.ADVANCE,
            ProcedureState.WITHDRAW,
        )

    @property
    def moves_base(self) -> bool:
        return self in (ProcedureState.APPROACH, ProcedureState.RETRACT)


#: The automatic run order. Anything not here is entered only on command or on fault.
MAIN_SEQUENCE: tuple[ProcedureState, ...] = (
    ProcedureState.APPROACH,
    ProcedureState.ESTIMATE,
    ProcedureState.INSERT,
    ProcedureState.ADVANCE,
)


@dataclass(frozen=True)
class Transition:
    """A state's request to move on, with the reason it gives.

    The reason is not decoration: it lands in the telemetry log, so a run that ended in
    the wrong place can be read back without re-deriving what the criteria were doing.
    """

    to: ProcedureState
    why: str

    def __str__(self) -> str:
        return f"-> {self.to.value} ({self.why})"


def next_in_sequence(state: ProcedureState) -> ProcedureState:
    """The state that follows ``state`` in the main run, or ``DONE`` at the end."""
    if state not in MAIN_SEQUENCE:
        return ProcedureState.DONE
    i = MAIN_SEQUENCE.index(state)
    return MAIN_SEQUENCE[i + 1] if i + 1 < len(MAIN_SEQUENCE) else ProcedureState.DONE
