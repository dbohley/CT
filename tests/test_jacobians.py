"""The analytic Jacobians are hard-coded for speed; these tests are what keeps
them honest.

Finite differencing appears here and nowhere else in the project — at sensor rate
it would cost K+3 extra model evaluations per step for a strictly worse
derivative.
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.optimize import approx_fprime

from ct.layout import StateLayout
from ct.tracking.measurement import (
    measurement,
    measurement_jacobian,
    transition,
    transition_matrix,
)


def random_state(layout: StateLayout, rng) -> np.ndarray:
    return layout.pack(
        a0=float(rng.normal(0, 5)),
        A=rng.uniform(0.5, 12.0, layout.K),
        phi=rng.uniform(-np.pi, np.pi, layout.K),
        theta=float(rng.uniform(-np.pi, np.pi)),
        omega_r=float(rng.uniform(0.6, 3.0)),
    )


@pytest.mark.parametrize("K", [1, 2, 3, 5])
def test_measurement_jacobian_matches_finite_differences(K, rng):
    layout = StateLayout(K)
    for _ in range(25):
        s = random_state(layout, rng)
        analytic = measurement_jacobian(s, layout)
        numeric = approx_fprime(s, lambda z: measurement(z, layout), 1e-7)
        np.testing.assert_allclose(analytic, numeric, atol=1e-5, rtol=1e-4)


@pytest.mark.parametrize("K", [1, 2, 4])
def test_transition_jacobian_matches_finite_differences(K, rng):
    layout = StateLayout(K)
    Ts = 0.02
    for _ in range(10):
        s = random_state(layout, rng)
        F = transition_matrix(layout, Ts)
        numeric = np.vstack(
            [
                approx_fprime(s, lambda z, i=i: transition(z, layout, Ts)[i], 1e-7)
                for i in range(layout.n)
            ]
        )
        np.testing.assert_allclose(F, numeric, atol=1e-6)


@pytest.mark.parametrize("K", [1, 3])
def test_transition_is_linear_so_F_reproduces_f_exactly(K, rng):
    """The process model is linear, so F @ s must equal f(s) with no remainder."""
    layout = StateLayout(K)
    Ts = 0.02
    s = random_state(layout, rng)
    np.testing.assert_allclose(transition_matrix(layout, Ts) @ s, transition(s, layout, Ts))


def test_transition_matrix_shape_and_sparsity():
    layout = StateLayout(3)
    F = transition_matrix(layout, 0.02)
    expected = np.eye(layout.n)
    expected[layout.theta, layout.omega] = 0.02
    np.testing.assert_allclose(F, expected)


def test_omega_has_no_direct_measurement_sensitivity(rng):
    """omega_r is observable only through theta's dynamics, never directly."""
    layout = StateLayout(3)
    for _ in range(10):
        H = measurement_jacobian(random_state(layout, rng), layout)
        assert H[layout.omega] == 0.0


def test_dh_dtheta_weights_each_harmonic_by_k(rng):
    """dh/dtheta = sum_k k A_k cos(k theta + phi_k) — the k factor is what makes
    higher harmonics rotate faster, and is the same fact the forecast relies on."""
    layout = StateLayout(3)
    s = random_state(layout, rng)
    _, A, phi, theta, _ = layout.unpack(s)
    k = np.arange(1, 4)
    expected = float(np.sum(k * A * np.cos(k * theta + phi)))
    assert measurement_jacobian(s, layout)[layout.theta] == pytest.approx(expected)


def test_measurement_evaluates_the_documented_model(rng):
    layout = StateLayout(2)
    s = random_state(layout, rng)
    a0, A, phi, theta, _ = layout.unpack(s)
    expected = a0 + A[0] * np.sin(theta + phi[0]) + A[1] * np.sin(2 * theta + phi[1])
    assert measurement(s, layout) == pytest.approx(expected)
