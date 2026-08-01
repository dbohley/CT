"""State-vector index layout.

This module is the *single source of truth* for the ordering of the EKF state

    s = [a0, A_1, phi_1, A_2, phi_2, ..., A_K, phi_K, theta, omega_r]

with dimension ``n = 2K + 3``. No other module may hard-code an index into ``s``;
everything goes through :class:`StateLayout`. That is what lets the tracker be
swapped (UKF, particle filter, IMM) without the identifier or the diagnostics
silently reading the wrong slot.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

TWO_PI = 2.0 * np.pi


@dataclass(frozen=True)
class StateLayout:
    """Index accessors for a K-harmonic respiratory state vector."""

    K: int

    def __post_init__(self) -> None:
        if self.K < 1:
            raise ValueError(f"K must be >= 1, got {self.K}")

    @property
    def n(self) -> int:
        """State dimension, ``2K + 3``."""
        return 2 * self.K + 3

    @property
    def a0(self) -> int:
        """Index of the DC offset."""
        return 0

    def A(self, k: int) -> int:
        """Index of the amplitude of harmonic ``k`` (1-based)."""
        self._check_k(k)
        return 1 + 2 * (k - 1)

    def phi(self, k: int) -> int:
        """Index of the phase of harmonic ``k`` (1-based)."""
        self._check_k(k)
        return 2 + 2 * (k - 1)

    @property
    def theta(self) -> int:
        """Index of the instantaneous fundamental phase."""
        return 2 * self.K + 1

    @property
    def omega(self) -> int:
        """Index of the fundamental angular frequency (rad/s)."""
        return 2 * self.K + 2

    @property
    def amplitude_idx(self) -> np.ndarray:
        """Indices of all ``A_k``, ascending in ``k``."""
        return np.array([self.A(k) for k in range(1, self.K + 1)], dtype=int)

    @property
    def phase_idx(self) -> np.ndarray:
        """Indices of all ``phi_k``, ascending in ``k``."""
        return np.array([self.phi(k) for k in range(1, self.K + 1)], dtype=int)

    @property
    def names(self) -> list[str]:
        """Human-readable state names, in state order. Used by plots and tables."""
        out = ["a0"]
        for k in range(1, self.K + 1):
            out += [f"A_{k}", f"phi_{k}"]
        return out + ["theta", "omega_r"]

    # -- convenience unpacking -------------------------------------------------

    def unpack(self, s: np.ndarray) -> tuple[float, np.ndarray, np.ndarray, float, float]:
        """Split a state vector into ``(a0, A[1..K], phi[1..K], theta, omega_r)``."""
        s = np.asarray(s, dtype=float)
        if s.shape[-1] != self.n:
            raise ValueError(f"state has {s.shape[-1]} entries, expected {self.n}")
        return (
            float(s[self.a0]),
            s[self.amplitude_idx],
            s[self.phase_idx],
            float(s[self.theta]),
            float(s[self.omega]),
        )

    def pack(
        self,
        a0: float,
        A: np.ndarray,
        phi: np.ndarray,
        theta: float,
        omega_r: float,
    ) -> np.ndarray:
        """Build a state vector from its parts."""
        A = np.asarray(A, dtype=float)
        phi = np.asarray(phi, dtype=float)
        if A.size != self.K or phi.size != self.K:
            raise ValueError(f"expected {self.K} amplitudes and phases")
        s = np.empty(self.n, dtype=float)
        s[self.a0] = a0
        s[self.amplitude_idx] = A
        s[self.phase_idx] = phi
        s[self.theta] = theta
        s[self.omega] = omega_r
        return s

    def _check_k(self, k: int) -> None:
        if not 1 <= k <= self.K:
            raise IndexError(f"harmonic index k={k} outside 1..{self.K}")


def wrap_angle(x: np.ndarray | float) -> np.ndarray | float:
    """Wrap angles to ``(-pi, pi]``.

    Applied to ``theta`` and every ``phi_k`` after each EKF update so the phase
    states cannot drift to large magnitudes (which would degrade the covariance
    conditioning over a long run).
    """
    wrapped = -np.mod(-np.asarray(x, dtype=float) + np.pi, TWO_PI) + np.pi
    return float(wrapped) if np.isscalar(x) or np.ndim(x) == 0 else wrapped
