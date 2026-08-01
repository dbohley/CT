"""Stage 2: does the filter converge, stay consistent, and stay numerically sane?"""

from __future__ import annotations

import numpy as np
import pytest

from ct.diagnostics import metrics
from ct.identification.fft_identifier import FFTHarmonicIdentifier
from ct.layout import StateLayout, wrap_angle
from ct.registry import build_source
from ct.run import track
from ct.tracking.harmonic_ekf import HarmonicEKF


def identify_and_track(source, duration=180.0, calib=60.0, tracker=None, identifier=None):
    batch = source.batch(duration)
    identifier = identifier or FFTHarmonicIdentifier(Kmax=8, energy_threshold=0.95)
    ident = identifier.identify(batch.slice_time(0.0, calib))
    tracker = tracker or HarmonicEKF()
    history = track(tracker, batch.slice_time(calib, duration + 1), ident)
    return ident, history, tracker


def test_converges_to_truth_on_a_noiseless_sinusoid():
    src = build_source(
        "sinusoid",
        {"fs": 50.0, "noise_std": 0.0, "amplitudes": [10.0, 3.0], "phases": [0.4, 1.1], "a0": 2.0},
    )
    ident, history, _ = identify_and_track(src)
    layout = StateLayout(ident.K)
    final = history.s[-1]

    assert ident.K == 2
    assert final[layout.a0] == pytest.approx(2.0, abs=1e-3)
    np.testing.assert_allclose(final[layout.amplitude_idx], [10.0, 3.0], atol=1e-3)
    assert final[layout.omega] == pytest.approx(2 * np.pi / 4.0, rel=1e-4)


def test_one_step_prediction_beats_the_raw_measurement(noisy_sinusoid):
    """The filter has to be worth having: its prediction must be closer to the
    clean signal than the noisy measurement it was given."""
    ident, history, _ = identify_and_track(noisy_sinusoid)
    warmup = 500
    filtered = metrics.rmse(history.y_pred[warmup:], history.y_clean[warmup:])
    raw = metrics.rmse(history.y[warmup:], history.y_clean[warmup:])
    assert filtered < 0.4 * raw


def test_nis_is_consistent_on_a_correctly_specified_run(noisy_sinusoid):
    """NIS ~ chi2(1): mean near 1, and about 95% of samples inside the band.
    Systematically high means Q or R is too small; low means too large."""
    _, history, _ = identify_and_track(noisy_sinusoid, duration=300.0, calib=90.0)
    warmup = 500
    nis = history.nis[warmup:]
    lo, hi = metrics.nis_bounds(0.05)

    assert np.mean(nis) == pytest.approx(1.0, abs=0.25)
    assert 0.90 < np.mean((nis >= lo) & (nis <= hi)) < 0.99


def test_normalised_innovations_are_unit_variance(noisy_sinusoid):
    _, history, _ = identify_and_track(noisy_sinusoid, duration=300.0, calib=90.0)
    nu = metrics.normalised_innovations(history.innovation[500:], history.S[500:])
    assert np.mean(nu) == pytest.approx(0.0, abs=0.05)
    assert np.std(nu) == pytest.approx(1.0, abs=0.15)


def test_tracks_a_drifting_fundamental_frequency():
    """omega_r is a state precisely so breathing rate can wander. Confirm the
    filter follows it rather than sitting on its initial estimate."""
    ramp = 0.0008
    src = build_source(
        "sinusoid",
        {"fs": 50.0, "noise_std": 0.05, "seed": 4, "amplitudes": [10.0, 3.0], "omega_ramp": ramp},
    )
    ident, history, _ = identify_and_track(src, duration=300.0, calib=60.0)
    layout = StateLayout(ident.K)

    omega_true_end = src.omega0 + ramp * history.t[-1]
    tracked_end = history.s[-1, layout.omega]
    initial = ident.s0[layout.omega]

    assert abs(tracked_end - omega_true_end) < abs(initial - omega_true_end)
    assert tracked_end == pytest.approx(omega_true_end, rel=0.02)


def test_covariance_stays_symmetric_and_positive_definite(noisy_sinusoid):
    """Joseph form exists for this; a long run is where the short form decays."""
    ident, history, tracker = identify_and_track(noisy_sinusoid, duration=300.0, calib=60.0)
    assert history.n_steps > 10000
    assert np.all(history.P_diag > 0)

    _, P = tracker.state
    np.testing.assert_allclose(P, P.T, atol=1e-14)
    assert np.all(np.linalg.eigvalsh(P) > 0)


def test_joseph_and_short_form_agree_early_but_joseph_stays_definite(noisy_sinusoid):
    a = identify_and_track(noisy_sinusoid, duration=120.0, tracker=HarmonicEKF(joseph=True))
    b = identify_and_track(noisy_sinusoid, duration=120.0, tracker=HarmonicEKF(joseph=False))
    # Algebraically identical, so the state paths should be very close.
    np.testing.assert_allclose(a[1].s[:200], b[1].s[:200], rtol=1e-6, atol=1e-8)
    assert np.all(np.linalg.eigvalsh(a[2].state[1]) > 0)


def test_phases_stay_wrapped(noisy_sinusoid):
    ident, history, _ = identify_and_track(noisy_sinusoid, duration=300.0, calib=60.0)
    layout = StateLayout(ident.K)
    for idx in [*layout.phase_idx, layout.theta]:
        assert np.all(np.abs(history.s[:, idx]) <= np.pi + 1e-9)


def test_theta_advances_at_omega_between_updates():
    """The one kinematic row of the process model, checked directly."""
    layout = StateLayout(2)
    ekf = HarmonicEKF()
    from ct.types import IdentificationResult

    s0 = layout.pack(a0=0.0, A=[10.0, 3.0], phi=[0.0, 0.0], theta=0.0, omega_r=1.5)
    ekf.init(
        IdentificationResult(
            K=2, s0=s0, P0=np.eye(layout.n) * 1e-12, Q=np.eye(layout.n) * 1e-18, R=1e6, Ts=0.02
        )
    )
    # A huge R makes the update a no-op, isolating the prediction step.
    s_pred, _ = ekf.predict(0.02)
    assert s_pred[layout.theta] == pytest.approx(0.03)
    assert s_pred[layout.omega] == pytest.approx(1.5)


def test_amplitude_clamping_folds_sign_into_phase():
    """A_k < 0 is the same waveform as (A_k > 0, phi_k + pi); folding preserves
    the signal, truncating at zero would bias the amplitude."""
    layout = StateLayout(1)
    ekf = HarmonicEKF(clamp_amplitudes=True)
    s = layout.pack(a0=0.0, A=[-4.0], phi=[0.3], theta=0.7, omega_r=1.5)

    from ct.tracking.measurement import measurement

    before = measurement(s.copy(), layout)
    folded = ekf._constrain(s.copy(), layout)

    assert folded[layout.A(1)] == pytest.approx(4.0)
    assert measurement(folded, layout) == pytest.approx(before, abs=1e-12)


def test_omega_bounds_are_enforced_when_requested():
    layout = StateLayout(1)
    ekf = HarmonicEKF(omega_bounds=(1.0, 2.0))
    s = layout.pack(a0=0.0, A=[4.0], phi=[0.0], theta=0.0, omega_r=5.0)
    assert ekf._constrain(s, layout)[layout.omega] == pytest.approx(2.0)


def test_using_the_tracker_before_init_is_an_error():
    with pytest.raises(RuntimeError, match="before init"):
        HarmonicEKF().step(0.0, 1.0)


def test_measurements_going_backwards_in_time_are_rejected(noisy_sinusoid, identifier):
    batch = noisy_sinusoid.batch(60.0)
    ident = identifier.identify(batch)
    ekf = HarmonicEKF()
    ekf.init(ident)
    ekf.step(ident.t0 + 0.02, 1.0)
    with pytest.raises(ValueError, match="precedes"):
        ekf.step(ident.t0, 1.0)


def test_Q_scales_with_the_gap_between_samples(noisy_sinusoid, identifier):
    """A dropped sample must inflate the covariance by the time actually elapsed,
    not by one nominal step."""
    batch = noisy_sinusoid.batch(60.0)
    ident = identifier.identify(batch)

    ekf = HarmonicEKF(scale_q_with_dt=True)
    ekf.init(ident)
    _, P_one = ekf.predict(ident.Ts)
    _, P_five = ekf.predict(5 * ident.Ts)

    growth_one = np.diag(P_one) - np.diag(ident.P0)
    growth_five = np.diag(P_five) - np.diag(ident.P0)
    assert np.all(growth_five >= growth_one - 1e-18)
    idx = StateLayout(ident.K).a0
    assert growth_five[idx] == pytest.approx(5 * growth_one[idx], rel=1e-6)


def test_forecast_variance_grows_with_the_horizon(noisy_sinusoid):
    """The quantity a downstream gate would threshold on."""
    _, _, tracker = identify_and_track(noisy_sinusoid, duration=180.0)
    variances = [tracker.forecast_variance(h) for h in (0.0, 0.1, 0.5, 1.0, 2.0)]
    assert all(v > 0 for v in variances)
    assert variances[-1] > variances[0]


def test_tracker_satisfies_the_protocol():
    from ct.interfaces import Tracker

    assert isinstance(HarmonicEKF(), Tracker)


def test_wrapping_does_not_change_the_predicted_measurement(noisy_sinusoid):
    """Wrapping is a numerical convenience; it must be observationally silent."""
    wrapped = identify_and_track(noisy_sinusoid, 120.0, tracker=HarmonicEKF(wrap_phases=True))
    plain = identify_and_track(noisy_sinusoid, 120.0, tracker=HarmonicEKF(wrap_phases=False))
    np.testing.assert_allclose(wrapped[1].y_pred, plain[1].y_pred, atol=1e-8)

    layout = StateLayout(wrapped[0].K)
    delta = wrapped[1].s[:, layout.theta] - plain[1].s[:, layout.theta]
    np.testing.assert_allclose(wrap_angle(delta), 0.0, atol=1e-8)
