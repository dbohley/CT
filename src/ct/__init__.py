"""CT — respiratory-motion estimation for needle-insertion gating.

Two stages behind swappable interfaces:

    identify  (batch, offline)   : signal -> (K, s0, P0, Q, R)
    track     (recursive, online): (K, s0, P0, Q, R) + samples -> s_hat, P

plus a forecast operation that evaluates the tracked model at ``t + h``.

Scope note: this package is the *estimator only*. The needle servo, the gating
logic, and the safety/residual monitor consume ``(s_hat, P)`` but live elsewhere.
"""

from ct.layout import StateLayout
from ct.types import IdentificationResult, SignalBatch, TrackerStep

# Importing the stage packages populates the registry, so a config string like
# `source: {name: lujan}` resolves no matter which entry point got there first.
# Keep this last: the stage modules import the names above.
import ct.identification  # noqa: E402,F401
import ct.sources  # noqa: E402,F401
import ct.tracking  # noqa: E402,F401
from ct.registry import (  # noqa: E402
    available,
    build_identifier,
    build_source,
    build_tracker,
)

__all__ = [
    "StateLayout",
    "SignalBatch",
    "IdentificationResult",
    "TrackerStep",
    "build_source",
    "build_identifier",
    "build_tracker",
    "available",
]
