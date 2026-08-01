"""Stage 1: does it recover what it should, and does K land where documented?"""

from __future__ import annotations

import numpy as np
import pytest

from ct.identification.harmonic_ls import design_matrix, fit_harmonics, select_K
from ct.identification.spectral import autocorr_peak_omega, coarse_omega, fft_peak_omega
from ct.layout import StateLayout, wrap_angle
from ct.registry import build_source

# -- frequency estimation ------------------------------------------------------


@pytest.mark.parametrize("bpm", [8.0, 12.0, 15.0, 22.0, 35.0])
def test_both_frequency_estimators_recover_a_known_rate(bpm):
    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.1, "seed": 1, "breaths_per_min": bpm})
    b = src.batch(180.0)
    expected = 2 * np.pi * bpm / 60.0
    assert fft_peak_omega(b.t, b.y)[0] == pytest.approx(expected, rel=0.02)
    assert autocorr_peak_omega(b.t, b.y)[0] == pytest.approx(expected, rel=0.02)


def test_coarse_omega_reports_the_search_resolution():
    """P0's omega entry is built from this, so it must actually be the bin width."""
    src = build_source("sinusoid", {"fs": 50.0, "noise_std": 0.1})
    b = src.batch(120.0)
    _, info = coarse_omega(b.t, b.y)
    assert info["resolution_omega"] == pytest.approx(2 * np.pi / (b.N * b.Ts), rel=1e-6)


def test_frequency_estimators_disagreeing_raises_a_warning():
    """Two independent estimators exist precisely to catch bad recordings."""
    rng = np.random.default_rng(0)
    t = np.arange(0, 120, 0.02)
    # Two comparable tones: the FFT and autocorrelation peaks land differently.
    y = np.sin(2 * np.pi * 0.25 * t) + 0.95 * np.sin(2 * np.pi * 0.15 * t) + 0.05 * rng.normal(size=t.size)
    with pytest.warns(UserWarning, match="disagree"):
        coarse_omega(t, y)


def test_record_too_short_for_the_band_is_an_error():
    """With a 1 s record the bins are 1 Hz apart, so none lands inside the
    5-45 bpm band (0.083-0.75 Hz) and there is nothing to peak-pick."""
    t = np.arange(0, 1.0, 0.02)
    with pytest.raises(ValueError, match="too short"):
        fft_peak_omega(t, np.sin(2 * np.pi * 0.25 * t))


# -- harmonic regression -------------------------------------------------------


def test_regression_recovers_known_amplitudes_and_phases():
    A_true, phi_true, a0_true = np.array([10.0, 3.0, 1.0]), np.array([0.4, 1.1, -2.0]), 2.5
    omega = 2 * np.pi / 4.0
    t = np.arange(0, 200, 0.02)
    k = np.arange(1, 4)
    y = a0_true + np.sum(A_true[:, None] * np.sin(k[:, None] * omega * t + phi_true[:, None]), axis=0)

    fit = fit_harmonics(t, y, omega, 3)
    assert fit.a0 == pytest.approx(a0_true, abs=1e-8)
    np.testing.assert_allclose(fit.amplitudes, A_true, atol=1e-8)
    np.testing.assert_allclose(wrap_angle(fit.phases - phi_true), 0.0, atol=1e-8)
    assert fit.residual_var < 1e-18


def test_regression_is_linear_in_its_coefficients():
    """X @ coeffs must reproduce the fitted values exactly — that is the whole
    reason Stage 1 is ordinary least squares and not an optimisation."""
    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.1, "seed": 2})
    b = src.batch(120.0)
    fit = fit_harmonics(b.t, b.y, 2 * np.pi / 4.0, 4)
    X = design_matrix(b.t, fit.omega, 4, fit.t_ref)
    np.testing.assert_allclose(X @ fit.coeffs, fit.fitted, atol=1e-10)


def test_residual_variance_recovers_the_injected_noise():
    src = build_source(
        "sinusoid", {"fs": 50.0, "noise_std": 0.25, "seed": 8, "amplitudes": [10.0, 3.0]}
    )
    b = src.batch(200.0)
    fit = fit_harmonics(b.t, b.y, 2 * np.pi / 4.0, 4)
    assert np.sqrt(fit.residual_var) == pytest.approx(0.25, rel=0.05)


def test_amplitude_phase_convention_matches_the_model():
    """alpha sin + beta cos == A sin(. + phi) with phi = atan2(beta, alpha)."""
    alpha, beta, omega = 3.0, -4.0, 1.5
    t = np.arange(0, 60, 0.01)
    y = alpha * np.sin(omega * t) + beta * np.cos(omega * t)
    fit = fit_harmonics(t, y, omega, 1)
    assert fit.amplitudes[0] == pytest.approx(5.0, abs=1e-8)
    np.testing.assert_allclose(
        fit.amplitudes[0] * np.sin(omega * t + fit.phases[0]), y, atol=1e-8
    )


def test_fit_rejects_too_few_samples():
    t = np.arange(0, 0.1, 0.02)
    with pytest.raises(ValueError, match="more samples"):
        fit_harmonics(t, np.sin(t), 1.5, 8)


# -- K selection ---------------------------------------------------------------


def test_select_K_uses_squared_amplitudes():
    """Parseval: energy goes as A^2, not A. Equal energies must need K=2."""
    K, info = select_K(np.array([1.0, 1.0, 0.0, 0.0]), 0.95)
    assert K == 2
    np.testing.assert_allclose(info["cumulative_energy"][:2], [0.5, 1.0])


def test_select_K_is_one_when_the_fundamental_dominates():
    assert select_K(np.array([10.0, 0.1, 0.05]), 0.95)[0] == 1


def test_select_K_rejects_a_bad_threshold():
    with pytest.raises(ValueError):
        select_K(np.array([1.0, 0.5]), 1.5)


@pytest.mark.parametrize("n, expected_K", [(1, 1), (2, 2), (3, 2)])
def test_lujan_K_selection_matches_the_documented_values(n, expected_K, identifier):
    """The settled result: n=1 -> K=1, n=2 -> K=2, n=3 -> K=2, the last because
    the third harmonic carries under 0.4% of the energy."""
    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.0, "n": n, "a": 10.0})
    result = identifier.identify(src.batch(180.0))
    assert result.K == expected_K


def test_lujan_n3_third_harmonic_is_below_the_documented_energy_share(identifier):
    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.0, "n": 3, "a": 10.0})
    frac = identifier.identify(src.batch(180.0)).diagnostics["energy_fraction"]
    assert frac[2] < 0.004


def test_rc_piecewise_needs_a_tighter_threshold_than_lujan():
    """Measured behaviour of the kink: unlike Lujan, whose series terminates at
    k=n, the RC model's coefficients never reach zero, so the K the rule picks
    keeps climbing as the threshold tightens. Recorded so a change in the
    generator or the rule shows up here."""
    from ct.identification.fft_identifier import FFTHarmonicIdentifier

    src = build_source("rc_piecewise", {"fs": 50.0, "noise_std": 0.0})
    batch = src.batch(180.0)
    Ks = [
        FFTHarmonicIdentifier(Kmax=10, energy_threshold=th).identify(batch).K
        for th in (0.95, 0.99, 0.999)
    ]
    assert Ks == sorted(Ks) and Ks[-1] > Ks[0]
    assert Ks[0] <= 2 and Ks[-1] >= 3


# -- assembled result ----------------------------------------------------------


def test_identification_result_is_well_formed(noisy_sinusoid, identifier):
    result = identifier.identify(noisy_sinusoid.batch(180.0))
    layout = StateLayout(result.K)

    assert result.s0.shape == (layout.n,)
    assert result.P0.shape == result.Q.shape == (layout.n, layout.n)
    assert result.R > 0
    assert np.all(np.linalg.eigvalsh(result.P0) > 0), "P0 must be positive definite"
    assert np.all(np.diag(result.Q) > 0), "Q must be positive on every state"
    assert result.Ts == pytest.approx(0.02)


def test_s0_recovers_the_generating_parameters(identifier):
    src = build_source(
        "sinusoid",
        {"fs": 50.0, "noise_std": 0.05, "seed": 1, "amplitudes": [10.0, 3.0], "phases": [0.4, 1.1]},
    )
    result = identifier.identify(src.batch(180.0))
    layout = StateLayout(result.K)
    _, A, _, _, omega = layout.unpack(result.s0)

    assert result.K == 2
    np.testing.assert_allclose(A, [10.0, 3.0], rtol=0.02)
    assert omega == pytest.approx(2 * np.pi / 4.0, rel=0.01)


def test_theta0_is_the_phase_at_the_end_of_the_window(identifier):
    """Tracking must resume exactly where identification stopped."""
    src = build_source("sinusoid", {"fs": 50.0, "noise_std": 0.0})
    batch = src.batch(120.0)
    result = identifier.identify(batch)
    layout = StateLayout(result.K)

    assert result.t0 == pytest.approx(batch.t[-1])
    expected = wrap_angle(result.s0[layout.omega] * (batch.t[-1] - batch.t[0]))
    assert wrap_angle(result.s0[layout.theta] - expected) == pytest.approx(0.0, abs=1e-6)


# The spliced-in still segment is, by construction, not breathing, so the
# frequency cross-check fires on the sliding window that covers it. That is the
# cross-check doing its job, not a defect in the test.
@pytest.mark.filterwarnings("ignore:FFT and autocorrelation:UserWarning")
def test_R_prefers_a_breath_hold_segment_over_the_residual():
    """The residual fallback is an upper bound; a still segment measures the
    sensor alone. On a mismatched model the difference is large."""
    from ct.identification.fft_identifier import FFTHarmonicIdentifier

    src = build_source("rc_piecewise", {"fs": 50.0, "noise_std": 0.2, "seed": 6})
    batch = src.batch(180.0)
    # Splice in a still segment with only sensor noise present.
    rng = np.random.default_rng(11)
    hold = (batch.t >= 100.0) & (batch.t < 110.0)
    y = batch.y.copy()
    y[hold] = batch.y_clean[hold].mean() + rng.normal(0, 0.2, int(hold.sum()))
    from ct.types import SignalBatch

    spliced = SignalBatch(t=batch.t, y=y, fs=batch.fs, y_clean=batch.y_clean, truth=batch.truth)

    with_hold = FFTHarmonicIdentifier(breath_hold_window=(100.0, 110.0)).identify(spliced)
    without = FFTHarmonicIdentifier().identify(spliced)

    assert with_hold.diagnostics["R_source"] == "breath_hold"
    assert np.sqrt(with_hold.R) == pytest.approx(0.2, rel=0.15)
    assert without.R > with_hold.R


def test_K_override_bypasses_the_energy_rule():
    from ct.identification.fft_identifier import FFTHarmonicIdentifier

    src = build_source("lujan", {"fs": 50.0, "noise_std": 0.05, "n": 1})
    result = FFTHarmonicIdentifier(Kmax=8, K_override=4).identify(src.batch(120.0))
    assert result.K == 4
    assert result.diagnostics["K_energy_rule"] == 1


def test_identification_result_survives_a_save_load_roundtrip(tmp_path, noisy_sinusoid, identifier):
    """ct-identify writes this file and ct-track reads it."""
    from ct.types import IdentificationResult

    original = identifier.identify(noisy_sinusoid.batch(120.0))
    path = tmp_path / "ident.npz"
    original.save(path)
    loaded = IdentificationResult.load(path)

    assert loaded.K == original.K
    assert loaded.R == pytest.approx(original.R)
    assert loaded.t0 == pytest.approx(original.t0)
    assert loaded.Ts == pytest.approx(original.Ts)
    np.testing.assert_allclose(loaded.s0, original.s0)
    np.testing.assert_allclose(loaded.P0, original.P0)
    np.testing.assert_allclose(loaded.Q, original.Q)
