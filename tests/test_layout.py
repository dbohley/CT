"""StateLayout is the only place state ordering is encoded, so it gets tested hard."""

from __future__ import annotations

import numpy as np
import pytest

from ct.layout import StateLayout, wrap_angle


@pytest.mark.parametrize("K", [1, 2, 3, 5, 8])
def test_dimension_is_2K_plus_3(K):
    assert StateLayout(K).n == 2 * K + 3


@pytest.mark.parametrize("K", [1, 2, 3, 5, 8])
def test_indices_are_a_permutation(K):
    """Every slot is claimed exactly once — no overlap, no gap."""
    L = StateLayout(K)
    idx = [L.a0, *L.amplitude_idx, *L.phase_idx, L.theta, L.omega]
    assert sorted(idx) == list(range(L.n))


@pytest.mark.parametrize("K", [1, 2, 4])
def test_pack_unpack_roundtrip(K, rng):
    L = StateLayout(K)
    a0 = float(rng.normal())
    A = rng.uniform(1, 10, K)
    phi = rng.uniform(-np.pi, np.pi, K)
    theta, omega = float(rng.normal()), 1.6

    s = L.pack(a0, A, phi, theta, omega)
    a0_b, A_b, phi_b, theta_b, omega_b = L.unpack(s)

    assert a0_b == pytest.approx(a0)
    np.testing.assert_allclose(A_b, A)
    np.testing.assert_allclose(phi_b, phi)
    assert theta_b == pytest.approx(theta)
    assert omega_b == pytest.approx(omega)


def test_names_match_dimension():
    L = StateLayout(3)
    assert L.names == ["a0", "A_1", "phi_1", "A_2", "phi_2", "A_3", "phi_3", "theta", "omega_r"]
    assert len(L.names) == L.n


def test_harmonic_index_bounds():
    L = StateLayout(2)
    with pytest.raises(IndexError):
        L.A(3)
    with pytest.raises(IndexError):
        L.phi(0)


def test_K_must_be_positive():
    with pytest.raises(ValueError):
        StateLayout(0)


def test_unpack_rejects_wrong_size():
    with pytest.raises(ValueError):
        StateLayout(2).unpack(np.zeros(5))


@pytest.mark.parametrize(
    "raw, expected",
    [(0.0, 0.0), (np.pi, np.pi), (-np.pi, np.pi), (3 * np.pi, np.pi), (2.5 * np.pi, 0.5 * np.pi)],
)
def test_wrap_angle_scalars(raw, expected):
    assert wrap_angle(raw) == pytest.approx(expected)


def test_wrap_angle_is_idempotent_and_preserves_the_angle(rng):
    x = rng.uniform(-40, 40, 500)
    w = wrap_angle(x)
    assert np.all(w > -np.pi) and np.all(w <= np.pi)
    np.testing.assert_allclose(wrap_angle(w), w, atol=1e-12)
    # Wrapping must not change what the angle means.
    np.testing.assert_allclose(np.sin(w), np.sin(x), atol=1e-9)
    np.testing.assert_allclose(np.cos(w), np.cos(x), atol=1e-9)
