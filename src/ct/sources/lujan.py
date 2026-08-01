"""Lujan respiratory-motion model.

    x(t) = b - a * cos^{2n}(pi*t/T - phi)

A single smooth closed form. ``n=1`` reduces to a pure sinusoid; ``n=2,3`` add the
flat-topped exhale / sharp inhale asymmetry seen in real breathing. Used to
validate the 95%-energy ``K`` rule, whose expected answers are ``n=1 -> K=1``,
``n=2 -> K=2``, ``n=3 -> K=2`` (the third harmonic of ``n=3`` carries under 0.4%
of the energy).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ct.registry import register_source
from ct.sources.base import SyntheticSource


@register_source("lujan")
class LujanSource(SyntheticSource):
    """Lujan ``cos^{2n}`` breathing model."""

    def __init__(
        self,
        fs: float = 50.0,
        noise_std: float = 0.0,
        seed: int = 0,
        breaths_per_min: float = 15.0,
        n: int = 2,
        a: float = 10.0,
        b: float = 0.0,
        phi: float = 0.0,
    ) -> None:
        super().__init__(fs=fs, noise_std=noise_std, seed=seed)
        if n < 1:
            raise ValueError("n must be >= 1")
        self.breaths_per_min = float(breaths_per_min)
        self.T = 60.0 / self.breaths_per_min
        self.n = int(n)
        self.a = float(a)
        self.b = float(b)
        self.phi = float(phi)

    @property
    def omega_r(self) -> float:
        """Fundamental angular frequency. The argument advances by ``pi`` per
        period, and ``cos^{2n}`` has period ``pi``, so the signal period is ``T``."""
        return 2.0 * np.pi / self.T

    def clean(self, t: np.ndarray) -> np.ndarray:
        arg = np.pi * np.asarray(t, dtype=float) / self.T - self.phi
        return self.b - self.a * np.cos(arg) ** (2 * self.n)

    def fourier_coefficients(self) -> np.ndarray:
        """Exact harmonic amplitudes ``A_k`` for ``k = 1..n``.

        ``cos^{2n}(u)`` expands to a finite cosine series with only ``n``
        harmonics of the fundamental, from the binomial identity

            cos^{2n}(u) = 2^{-2n} [ C(2n,n) + 2*sum_{k=1..n} C(2n, n-k) cos(2k u) ]

        so the model has an exact, finite ``K``. This is the ground truth the
        energy-based ``K`` selection is checked against.
        """
        from scipy.special import comb

        norm = 2.0 ** (-2 * self.n)
        return np.array(
            [self.a * norm * 2.0 * comb(2 * self.n, self.n - k) for k in range(1, self.n + 1)]
        )

    def truth_dict(self) -> dict[str, Any]:
        return {
            "model": "lujan",
            "n": self.n,
            "a": self.a,
            "b": self.b,
            "phi": self.phi,
            "T": self.T,
            "omega_r": self.omega_r,
            "breaths_per_min": self.breaths_per_min,
            "exact_amplitudes": self.fourier_coefficients().tolist(),
            "n_harmonics": self.n,
        }
