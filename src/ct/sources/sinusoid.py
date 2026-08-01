"""Multi-harmonic sinusoid — the source the estimator's model matches exactly.

This is the sanity-check generator: if identification and tracking cannot recover
truth here, nothing downstream is meaningful. It optionally ramps ``omega_r``
linearly so the EKF's frequency state has something to track.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.registry import register_source
from ct.sources.base import SyntheticSource


@register_source("sinusoid")
class SinusoidSource(SyntheticSource):
    """``y(t) = a0 + sum_k A_k sin(k*theta(t) + phi_k)``.

    With ``omega_ramp`` non-zero the fundamental frequency is
    ``omega(t) = omega0 + omega_ramp * t``, so the exact phase is
    ``theta(t) = omega0*t + omega_ramp*t^2/2`` — note this is a genuine
    time-advance of the phase, which is why every harmonic ``k`` picks up
    ``k*theta`` rather than a shared angular offset.
    """

    def __init__(
        self,
        fs: float = 50.0,
        noise_std: float = 0.0,
        seed: int = 0,
        breaths_per_min: float = 15.0,
        amplitudes: list[float] | None = None,
        phases: list[float] | None = None,
        a0: float = 0.0,
        omega_ramp: float = 0.0,
    ) -> None:
        super().__init__(fs=fs, noise_std=noise_std, seed=seed)
        self.breaths_per_min = float(breaths_per_min)
        self.omega0 = 2.0 * np.pi * self.breaths_per_min / 60.0
        self.amplitudes = np.asarray(amplitudes if amplitudes is not None else [10.0], float)
        self.phases = np.asarray(
            phases if phases is not None else [0.0] * self.amplitudes.size, float
        )
        if self.phases.size != self.amplitudes.size:
            raise ValueError("amplitudes and phases must have the same length")
        self.a0 = float(a0)
        self.omega_ramp = float(omega_ramp)

    def theta(self, t: np.ndarray) -> np.ndarray:
        return self.omega0 * t + 0.5 * self.omega_ramp * t**2

    def omega(self, t: np.ndarray) -> np.ndarray:
        return self.omega0 + self.omega_ramp * np.asarray(t, dtype=float)

    def clean(self, t: np.ndarray) -> np.ndarray:
        th = self.theta(np.asarray(t, dtype=float))
        y = np.full_like(th, self.a0)
        for i, (A, phi) in enumerate(zip(self.amplitudes, self.phases), start=1):
            y = y + A * np.sin(i * th + phi)
        return y

    def truth_dict(self) -> dict[str, Any]:
        return {
            "model": "sinusoid",
            "a0": self.a0,
            "amplitudes": self.amplitudes.tolist(),
            "phases": self.phases.tolist(),
            "omega_r": self.omega0,
            "omega_ramp": self.omega_ramp,
            "breaths_per_min": self.breaths_per_min,
            "n_harmonics": int(self.amplitudes.size),
        }
