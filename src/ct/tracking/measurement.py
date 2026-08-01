"""Measurement model and its analytic Jacobians.

Shared by the tracker and the forecaster so there is exactly one definition of
what the model *says*, and the two can never drift apart.

    h(s)  = a0 + sum_{k=1..K} A_k sin(k*theta + phi_k)

    dh/da0    = 1
    dh/dA_k   = sin(k*theta + phi_k)
    dh/dphi_k = A_k cos(k*theta + phi_k)
    dh/dtheta = sum_k k A_k cos(k*theta + phi_k)
    dh/domega = 0                       (omega enters only through theta's dynamics)

The Jacobians are hard-coded, never finite-differenced at runtime: this runs at
sensor rate inside a real-time loop, and numeric differencing would cost K+3
extra evaluations per step for a strictly worse derivative. Finite differences
appear only in the test suite, as a check on these expressions.
"""

from __future__ import annotations

import numpy as np

from ct.layout import StateLayout


def measurement(s: np.ndarray, layout: StateLayout) -> float:
    """``h(s)`` — the predicted sensor reading."""
    a0, A, phi, theta, _ = layout.unpack(s)
    k = np.arange(1, layout.K + 1)
    return float(a0 + np.sum(A * np.sin(k * theta + phi)))


def measurement_jacobian(s: np.ndarray, layout: StateLayout) -> np.ndarray:
    """``H = dh/ds``, shape ``(1, n)`` flattened to ``(n,)``."""
    _, A, phi, theta, _ = layout.unpack(s)
    k = np.arange(1, layout.K + 1)
    arg = k * theta + phi
    sin_arg, cos_arg = np.sin(arg), np.cos(arg)

    H = np.zeros(layout.n, dtype=float)
    H[layout.a0] = 1.0
    H[layout.amplitude_idx] = sin_arg
    H[layout.phase_idx] = A * cos_arg
    H[layout.theta] = float(np.sum(k * A * cos_arg))
    H[layout.omega] = 0.0
    return H


def transition_matrix(layout: StateLayout, Ts: float) -> np.ndarray:
    """``F = df/ds``.

    The process model is one kinematic row plus persistence:

        theta_k = theta_{k-1} + omega_r * Ts     (exact, by definition of theta)
        s_k(i)  = s_{k-1}(i)                     for every other state

    so ``F`` is the identity with a single off-diagonal ``Ts`` coupling ``omega``
    into ``theta``. The persistence assumption is not a claim that amplitudes are
    constant — ``Q`` is where their drift is accounted for.
    """
    F = np.eye(layout.n)
    F[layout.theta, layout.omega] = Ts
    return F


def transition(s: np.ndarray, layout: StateLayout, Ts: float) -> np.ndarray:
    """``f(s)`` — advance the state by ``Ts``. Linear, so it matches ``F`` exactly."""
    out = np.array(s, dtype=float, copy=True)
    out[layout.theta] = s[layout.theta] + s[layout.omega] * Ts
    return out


def advance_phase(s: np.ndarray, layout: StateLayout, dt: float) -> np.ndarray:
    """Advance the model forward in time by ``dt``.

    This is the entire content of "prediction": ``theta -> theta + omega_r*dt``.
    Harmonic ``k`` then rotates by ``k*omega_r*dt`` automatically, because its
    argument is ``k*theta + phi_k``.

    Do NOT instead compute a single delay-derived angle and add it to every
    harmonic's phase. That correctly advances only the fundamental; every higher
    harmonic ends up under-rotated by a factor of ``k``, which distorts the
    waveform shape rather than shifting it in time.
    """
    return transition(s, layout, dt)
