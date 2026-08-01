"""Shared machinery for synthetic signal sources.

Subclasses supply :meth:`clean` (the noise-free waveform) and :meth:`truth_dict`
(the generating parameters). Sampling, noise, batching and streaming are handled
here so every source presents an identical face to the estimator.
"""

from __future__ import annotations

from typing import Any, Iterator

import numpy as np

from ct.types import SignalBatch


class SyntheticSource:
    """Base for closed-form generators. Satisfies :class:`ct.interfaces.SignalSource`."""

    def __init__(self, fs: float = 50.0, noise_std: float = 0.0, seed: int = 0) -> None:
        if fs <= 0:
            raise ValueError("fs must be positive")
        if noise_std < 0:
            raise ValueError("noise_std must be non-negative")
        self.fs = float(fs)
        self.noise_std = float(noise_std)
        self.seed = int(seed)

    # -- to be provided by subclasses -----------------------------------------

    def clean(self, t: np.ndarray) -> np.ndarray:
        """Noise-free signal at times ``t`` (seconds)."""
        raise NotImplementedError

    def truth_dict(self) -> dict[str, Any]:
        """Generating parameters, for validation and plot overlays."""
        raise NotImplementedError

    # -- common behaviour ------------------------------------------------------

    def time_grid(self, duration_s: float, t0: float = 0.0) -> np.ndarray:
        n = int(round(duration_s * self.fs))
        if n < 1:
            raise ValueError(f"duration {duration_s}s at fs={self.fs} yields no samples")
        return t0 + np.arange(n, dtype=float) / self.fs

    def batch(self, duration_s: float, t0: float = 0.0) -> SignalBatch:
        t = self.time_grid(duration_s, t0)
        y_clean = np.asarray(self.clean(t), dtype=float)
        # A fresh RNG each call keeps batch() a pure function of its arguments,
        # so repeated calls (and stream(), which delegates here) agree exactly.
        rng = np.random.default_rng(self.seed)
        noise = rng.normal(0.0, self.noise_std, size=t.size) if self.noise_std > 0 else 0.0
        return SignalBatch(
            t=t,
            y=y_clean + noise,
            fs=self.fs,
            y_clean=y_clean,
            truth={**self.truth_dict(), "noise_std": self.noise_std, "source": self.name},
        )

    def stream(self, duration_s: float, t0: float = 0.0) -> Iterator[tuple[float, float]]:
        b = self.batch(duration_s, t0)
        yield from zip(b.t.tolist(), b.y.tolist())

    @property
    def name(self) -> str:
        return getattr(type(self), "registry_name", type(self).__name__)

    def __repr__(self) -> str:
        return f"{type(self).__name__}(fs={self.fs}, noise_std={self.noise_std}, seed={self.seed})"
