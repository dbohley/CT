"""Shared fixtures.

Tests use short records (60-120 s) so the suite stays fast; the CLI configs use
longer ones. Where a test needs statistical power rather than speed it says so.
"""

from __future__ import annotations

import numpy as np
import pytest

from ct.identification.fft_identifier import FFTHarmonicIdentifier
from ct.registry import build_source


@pytest.fixture
def clean_sinusoid():
    """Noiseless two-harmonic signal — the estimator's model, exactly."""
    return build_source(
        "sinusoid",
        {
            "fs": 50.0,
            "noise_std": 0.0,
            "breaths_per_min": 15.0,
            "amplitudes": [10.0, 3.0],
            "phases": [0.4, 1.1],
            "a0": 2.0,
        },
    )


@pytest.fixture
def noisy_sinusoid():
    return build_source(
        "sinusoid",
        {
            "fs": 50.0,
            "noise_std": 0.15,
            "seed": 3,
            "breaths_per_min": 15.0,
            "amplitudes": [10.0, 3.0],
            "phases": [0.4, 1.1],
        },
    )


@pytest.fixture
def identifier():
    return FFTHarmonicIdentifier(Kmax=8, energy_threshold=0.95)


@pytest.fixture
def rng():
    return np.random.default_rng(20260801)
