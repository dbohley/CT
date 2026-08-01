"""The three swap boundaries.

These are structural (:class:`typing.Protocol`) rather than nominal: a new
implementation just needs matching methods, no inheritance and no import of this
module. Combined with :mod:`ct.registry`, that means a replacement tracker or
identifier is reachable from a YAML config with no change to the CLI.

    1. SignalSource -- produces (t, y) samples: synthetic model or real sensor
    2. Identifier   -- calibration batch -> (K, s0, P0, Q, R)
    3. Tracker      -- that, plus live samples -> (s_hat, P) each step
"""

from __future__ import annotations

from typing import Any, Iterator, Protocol, runtime_checkable

import numpy as np

from ct.types import IdentificationResult, SignalBatch, TrackerStep


@runtime_checkable
class SignalSource(Protocol):
    """Anything that can produce a 1-D breathing trace."""

    fs: float

    def batch(self, duration_s: float, t0: float = 0.0) -> SignalBatch:
        """Return ``duration_s`` seconds of samples starting at ``t0``."""
        ...

    def stream(self, duration_s: float, t0: float = 0.0) -> Iterator[tuple[float, float]]:
        """Yield ``(t, y)`` one sample at a time — the online-tracking entry point."""
        ...


@runtime_checkable
class Identifier(Protocol):
    """Stage 1: batch identification of the harmonic model."""

    def identify(self, batch: SignalBatch) -> IdentificationResult:
        """Fit the model and produce the tracker's initial conditions and noise."""
        ...


@runtime_checkable
class Tracker(Protocol):
    """Stage 2: recursive state estimation."""

    def init(self, result: IdentificationResult, t0: float | None = None) -> None:
        """Seed the filter from an identification result."""
        ...

    def step(self, t: float, y: float) -> TrackerStep:
        """Advance to time ``t`` and fold in measurement ``y``."""
        ...

    def forecast(self, h: float) -> float:
        """Predicted signal value at ``t_now + h``.

        ``h`` is supplied from outside this package -- it is the full horizon
        ``tau_s + tau_c + tau_cl(omega_r) + T_ins``. The estimator never computes
        or decomposes ``h``; it only evaluates the model there.
        """
        ...

    @property
    def state(self) -> tuple[np.ndarray, np.ndarray]:
        """Current ``(s_hat, P)``."""
        ...

    @property
    def config(self) -> dict[str, Any]:
        """Serialisable description, for run provenance."""
        ...
