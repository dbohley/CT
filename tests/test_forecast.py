"""Forecasting is a time-advance, not a phase shift.

This file is the anti-regression guard for the single easiest mistake in the
whole pipeline: rotating every harmonic's phase by one common delay-derived
angle. That advances the fundamental correctly and under-rotates harmonic k by a
factor of k, distorting the waveform's shape instead of shifting it in time.
"""

from __future__ import annotations

import numpy as np
import pytest

from ct.forecast import forecast_value, horizon_from_components, naive_common_phase_forecast
from ct.layout import StateLayout
from ct.tracking.measurement import measurement


def two_harmonic_state():
    layout = StateLayout(2)
    s = layout.pack(a0=1.5, A=[10.0, 3.0], phi=[0.4, 1.1], theta=0.0, omega_r=2 * np.pi / 4.0)
    return layout, s


def truth(t: float, layout: StateLayout, s: np.ndarray) -> float:
    """Exact signal value at time t for the state defined at t=0."""
    a0, A, phi, theta0, omega = layout.unpack(s)
    k = np.arange(1, layout.K + 1)
    return float(a0 + np.sum(A * np.sin(k * (theta0 + omega * t) + phi)))


@pytest.mark.parametrize("h", [0.0, 0.05, 0.25, 0.5, 1.0, 2.0, 3.7])
def test_forecast_matches_the_true_future_value(h):
    layout, s = two_harmonic_state()
    assert forecast_value(s, layout, h) == pytest.approx(truth(h, layout, s), abs=1e-9)


def test_each_harmonic_rotates_by_k_times_omega_h():
    """Harmonic k must pick up k*omega*h, not omega*h."""
    layout, s = two_harmonic_state()
    h = 0.3
    omega = s[layout.omega]
    from ct.tracking.measurement import advance_phase

    advanced = advance_phase(s, layout, h)
    theta_new = advanced[layout.theta]
    # theta advanced by exactly omega*h ...
    assert theta_new - s[layout.theta] == pytest.approx(omega * h)
    # ... and harmonic k's argument therefore advanced by k*omega*h.
    for k in (1, 2):
        arg_before = k * s[layout.theta] + s[layout.phi(k)]
        arg_after = k * theta_new + advanced[layout.phi(k)]
        assert arg_after - arg_before == pytest.approx(k * omega * h)


@pytest.mark.parametrize("h", [0.25, 0.5, 0.9])
def test_common_phase_rotation_is_wrong_whenever_a_second_harmonic_exists(h):
    """The regression this file exists for.

    The naive method must be measurably wrong against the true future value,
    while the correct method is exact. If this ever stops failing for the naive
    method, someone has reintroduced the bug and this is the only thing that will
    say so.

    The error is swept over a full cycle of starting phases rather than checked
    at one: at isolated values of theta the two methods coincide by accident,
    which is exactly how this bug survives a spot check.
    """
    layout, s = two_harmonic_state()
    errors = []
    for theta0 in np.linspace(-np.pi, np.pi, 60, endpoint=False):
        s_theta = s.copy()
        s_theta[layout.theta] = theta0
        exact = truth(h, layout, s_theta)
        # The correct method is exact at every phase ...
        assert forecast_value(s_theta, layout, h) == pytest.approx(exact, abs=1e-9)
        errors.append(abs(naive_common_phase_forecast(s_theta, layout, h) - exact))

    errors = np.array(errors)
    # ... while the naive one is wrong by order A_2 somewhere in the cycle.
    assert errors.max() > 1.0, f"naive forecast peak error {errors.max()} is suspiciously small"
    assert np.sqrt(np.mean(errors**2)) > 0.5


def test_the_two_methods_agree_only_for_a_single_harmonic():
    """With K=1 there is no higher harmonic to under-rotate, which is exactly why
    the bug survives casual testing on a pure sinusoid."""
    layout = StateLayout(1)
    s = layout.pack(a0=0.0, A=[10.0], phi=[0.3], theta=0.2, omega_r=1.6)
    h = 0.4
    assert naive_common_phase_forecast(s, layout, h) == pytest.approx(
        forecast_value(s, layout, h), abs=1e-9
    )


def test_zero_horizon_returns_the_current_value():
    layout, s = two_harmonic_state()
    assert forecast_value(s, layout, 0.0) == pytest.approx(measurement(s, layout))


def test_forecast_is_periodic_in_the_breathing_period():
    layout, s = two_harmonic_state()
    T = 2 * np.pi / s[layout.omega]
    assert forecast_value(s, layout, 0.7) == pytest.approx(forecast_value(s, layout, 0.7 + T))


def test_negative_horizon_looks_backwards():
    """Nothing forbids h < 0; it is the same time-advance run in reverse."""
    layout, s = two_harmonic_state()
    assert forecast_value(s, layout, -0.3) == pytest.approx(truth(-0.3, layout, s), abs=1e-9)


def test_horizon_assembles_from_its_four_components():
    h = horizon_from_components(
        tau_sensor=0.02, tau_compute=0.005, tau_closed_loop=0.08, T_insertion=0.15
    )
    assert h == pytest.approx(0.255)


def test_horizon_rejects_negative_components():
    with pytest.raises(ValueError, match="tau_closed_loop"):
        horizon_from_components(tau_closed_loop=-0.1)


def test_tracker_forecast_agrees_with_the_free_function(noisy_sinusoid, identifier):
    """The tracker's forecast() and ct.forecast must be the same operation."""
    from ct.tracking.harmonic_ekf import HarmonicEKF

    batch = noisy_sinusoid.batch(120.0)
    ident = identifier.identify(batch.slice_time(0, 60))
    ekf = HarmonicEKF()
    ekf.init(ident)
    for t, y in zip(batch.t[3000:3500], batch.y[3000:3500]):
        ekf.step(float(t), float(y))

    s, _ = ekf.state
    layout = StateLayout(ident.K)
    for h in (0.1, 0.25, 0.5):
        assert ekf.forecast(h) == pytest.approx(forecast_value(s, layout, h))
