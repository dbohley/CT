"""The insertion controller: the four-state procedure, the servo, and the safety monitor.

Deliberately *not* imported by :mod:`ct`. The estimator half of this package must keep
working — and keep being testable — without the hardware layer present, so the dependency
runs one way only: ``control`` may import the estimator, never the reverse.

Importing this module registers the built-in procedure states, the same way importing
``ct.sources`` registers the built-in signal sources.
"""

from __future__ import annotations

from ct.control.state import (
    MAIN_SEQUENCE,
    ProcedureState,
    Transition,
    next_in_sequence,
)

__all__ = [
    "ProcedureState",
    "Transition",
    "MAIN_SEQUENCE",
    "next_in_sequence",
]

# Populates the procedure-state registry. Keep last: the state modules import the names
# above. Mirrors the pattern in `ct/__init__.py`.
import ct.control.states  # noqa: E402,F401
