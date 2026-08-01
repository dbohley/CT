"""Data carried across the three interface boundaries.

These types are deliberately dumb: they hold arrays and metadata, never
behaviour. Anything that can be swapped (source / identifier / tracker) speaks
only in terms of what is defined here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SignalBatch:
    """A block of ``(t, y)`` samples from any signal source.

    ``y_clean`` and ``truth`` are populated by synthetic sources and are ``None``
    for real sensor data. Nothing in the estimator may depend on them; they exist
    purely for validation and plotting.
    """

    t: np.ndarray
    y: np.ndarray
    fs: float
    y_clean: np.ndarray | None = None
    truth: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if self.t.shape != self.y.shape:
            raise ValueError(f"t {self.t.shape} and y {self.y.shape} must match")
        if self.t.ndim != 1:
            raise ValueError("t and y must be 1-D")
        if self.y_clean is not None and self.y_clean.shape != self.y.shape:
            raise ValueError("y_clean must match y")
        if self.fs <= 0:
            raise ValueError(f"fs must be positive, got {self.fs}")

    @property
    def N(self) -> int:
        return int(self.t.size)

    @property
    def Ts(self) -> float:
        return 1.0 / self.fs

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0]) if self.N > 1 else 0.0

    def slice_time(self, t_start: float, t_end: float) -> SignalBatch:
        """Sub-batch over ``[t_start, t_end)``, preserving truth arrays."""
        m = (self.t >= t_start) & (self.t < t_end)
        return SignalBatch(
            t=self.t[m],
            y=self.y[m],
            fs=self.fs,
            y_clean=None if self.y_clean is None else self.y_clean[m],
            truth=self.truth,
        )


@dataclass(frozen=True)
class IdentificationResult:
    """Everything Stage 1 hands to Stage 2.

    ``R`` is a plain float: the sensor is 1-D, so the measurement covariance and
    the innovation covariance are scalars throughout.
    """

    K: int
    s0: np.ndarray
    P0: np.ndarray
    Q: np.ndarray
    R: float
    t0: float = 0.0
    Ts: float = 0.0
    diagnostics: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        n = 2 * self.K + 3
        if self.s0.shape != (n,):
            raise ValueError(f"s0 must have shape ({n},), got {self.s0.shape}")
        for name, M in (("P0", self.P0), ("Q", self.Q)):
            if M.shape != (n, n):
                raise ValueError(f"{name} must have shape ({n}, {n}), got {M.shape}")
        if self.R <= 0:
            raise ValueError(f"R must be positive, got {self.R}")

    def save(self, path: str | Path) -> None:
        """Persist to ``.npz`` so ``ct-track`` can consume a prior ``ct-identify`` run."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            K=self.K,
            s0=self.s0,
            P0=self.P0,
            Q=self.Q,
            R=self.R,
            t0=self.t0,
            Ts=self.Ts,
            diagnostics=np.array(self.diagnostics, dtype=object),
        )

    @classmethod
    def load(cls, path: str | Path) -> IdentificationResult:
        d = np.load(path, allow_pickle=True)
        return cls(
            K=int(d["K"]),
            s0=d["s0"],
            P0=d["P0"],
            Q=d["Q"],
            R=float(d["R"]),
            t0=float(d["t0"]),
            Ts=float(d["Ts"]),
            diagnostics=d["diagnostics"].item(),
        )


@dataclass(frozen=True)
class TrackerStep:
    """One predict+update cycle's worth of output."""

    t: float
    s: np.ndarray
    P: np.ndarray
    y_pred: float
    innovation: float
    S: float
    nis: float
