"""Estimating ``R`` and ``Q`` from the calibration recording.

``R`` is easy: the sensor is 1-D, so it is a single number.

``Q`` is the interesting one. The process model says every state except ``theta``
persists, which is a lie of convenience — amplitudes and phases really do drift
breath to breath. ``Q`` is where that lie is paid for, so it is measured directly:
refit each breath on its own, look at how much each fitted state moves *between*
breaths, and convert that breath-timescale variance to a per-sample one.

Amplitude-like, phase-like and frequency-like states get separate values. A
single global scalar ``Q`` would either over-trust the phases or under-trust the
amplitudes; there is no value that is right for both.
"""

from __future__ import annotations

import numpy as np

from ct.identification.harmonic_ls import HarmonicFit, fit_harmonics
from ct.identification.spectral import DEFAULT_BPM_RANGE, coarse_omega
from ct.layout import TWO_PI, StateLayout, wrap_angle


def estimate_R_from_breath_hold(y: np.ndarray) -> float:
    """Preferred ``R``: sample variance over a still / breath-hold segment.

    With no breathing present, everything left is sensor noise — no model
    mismatch is folded in.
    """
    y = np.asarray(y, dtype=float)
    if y.size < 2:
        raise ValueError("need at least 2 samples for a breath-hold variance")
    return float(np.var(y, ddof=1))


def estimate_R_from_residuals(fit: HarmonicFit) -> float:
    """Fallback ``R``: Stage-1 residual variance.

    Treat this as an **upper bound only** — it contains sensor noise *plus* every
    part of the waveform the truncated harmonic model failed to represent.
    """
    return float(fit.residual_var)


def circular_variance(angles: np.ndarray) -> float:
    """Variance of angles about their circular mean, in rad^2.

    Plain ``np.var`` on phases straddling +-pi reports a spurious ~pi^2; this
    measures deviation from the circular mean instead.
    """
    angles = np.asarray(angles, dtype=float)
    if angles.size < 2:
        return 0.0
    mean = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
    return float(np.var(wrap_angle(angles - mean), ddof=1))


def breath_boundaries(t: np.ndarray, fit: HarmonicFit) -> np.ndarray:
    """Times where the fitted fundamental completes a cycle.

    Boundaries sit where ``omega*(t - t_ref) + phi_1`` crosses a multiple of
    ``2*pi``, so every segment spans exactly one breath and all segments share a
    common phase origin — which is what makes the per-breath fitted phases
    comparable to each other.
    """
    phi1 = float(np.arctan2(fit.coeffs[2], fit.coeffs[1]))
    T = TWO_PI / fit.omega
    m0 = np.ceil((fit.omega * (t[0] - fit.t_ref) + phi1) / TWO_PI)
    m1 = np.floor((fit.omega * (t[-1] - fit.t_ref) + phi1) / TWO_PI)
    if m1 <= m0:
        return np.array([t[0], t[-1]])
    ms = np.arange(m0, m1 + 1)
    edges = fit.t_ref + (TWO_PI * ms - phi1) / fit.omega
    return edges[(edges >= t[0]) & (edges <= t[-1] + 1e-9)] if edges.size >= 2 else np.array(
        [t[0], t[0] + T]
    )


def per_breath_refits(
    t: np.ndarray, y: np.ndarray, fit: HarmonicFit, K: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Refit each breath independently at the same ``omega`` and ``t_ref``.

    Returns ``(a0_per_breath, A_per_breath, phi_per_breath, n_breaths)`` with the
    amplitude/phase arrays shaped ``(n_breaths, K)``.
    """
    edges = breath_boundaries(t, fit)
    min_samples = 2 * (1 + 2 * K)
    a0s, As, phis = [], [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (t >= lo) & (t < hi)
        if int(m.sum()) < min_samples:
            continue
        # Same omega and same t_ref as the global fit: the per-breath phases are
        # then directly comparable, and their spread is real drift, not framing.
        f = fit_harmonics(t[m], y[m], fit.omega, K, t_ref=fit.t_ref)
        a0s.append(f.a0)
        As.append(f.amplitudes)
        phis.append(f.phases)
    if len(a0s) < 2:
        return np.array([]), np.zeros((0, K)), np.zeros((0, K)), len(a0s)
    return np.array(a0s), np.vstack(As), np.vstack(phis), len(a0s)


def windowed_omega_variance(
    t: np.ndarray,
    y: np.ndarray,
    omega: float,
    breaths_per_window: float = 4.0,
    bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
) -> tuple[float, int]:
    """Variance of ``omega`` across overlapping multi-breath windows.

    Deliberately *not* per-breath: one breath does not resolve frequency well
    enough for its scatter to mean anything, so frequency drift is measured over
    a few breaths at a time and attributed to the same breath timescale.
    """
    T = TWO_PI / omega
    win = breaths_per_window * T
    span = float(t[-1] - t[0])
    if span < 2.0 * win:
        return float("nan"), 0
    step = win / 2.0
    starts = np.arange(t[0], t[-1] - win, step)
    estimates = []
    for s in starts:
        m = (t >= s) & (t < s + win)
        if int(m.sum()) < 8:
            continue
        try:
            w, _ = coarse_omega(t[m], y[m], bpm_range=bpm_range)
            estimates.append(w)
        except (ValueError, np.linalg.LinAlgError):
            continue
    if len(estimates) < 2:
        return float("nan"), len(estimates)
    return float(np.var(estimates, ddof=1)), len(estimates)


def estimate_Q(
    t: np.ndarray,
    y: np.ndarray,
    fit: HarmonicFit,
    K: int,
    Ts: float,
    omega_resolution: float,
    q_scale: float = 1.0,
    q_floor: float = 1e-12,
    bpm_range: tuple[float, float] = DEFAULT_BPM_RANGE,
) -> tuple[np.ndarray, dict]:
    """Per-state process noise from breath-to-breath variability.

    ``Q_i ~= var_breath_to_breath(i) * (Ts / T_breath)`` — the measured spread is
    what accumulates over one breath, and the filter takes one ``Ts`` step at a
    time.
    """
    layout = StateLayout(K)
    T_breath = TWO_PI / fit.omega
    ratio = Ts / T_breath

    a0s, As, phis, n_breaths = per_breath_refits(t, y, fit, K)

    q = np.full(layout.n, q_floor, dtype=float)
    notes: dict = {"n_breaths": n_breaths, "T_breath": T_breath, "Ts": Ts, "Ts_over_T": ratio}

    if n_breaths >= 2:
        q[layout.a0] = np.var(a0s, ddof=1) * ratio
        for k in range(1, K + 1):
            q[layout.A(k)] = np.var(As[:, k - 1], ddof=1) * ratio
            q[layout.phi(k)] = circular_variance(phis[:, k - 1]) * ratio
        # theta is a phase-like state: give it the fundamental's phase scale.
        q[layout.theta] = q[layout.phi(1)]
    else:
        notes["warning"] = (
            "fewer than 2 complete breaths in the calibration window; "
            "Q for amplitude/phase states fell back to the floor"
        )

    var_omega, n_windows = windowed_omega_variance(t, y, fit.omega, bpm_range=bpm_range)
    if not np.isfinite(var_omega):
        # Not enough record to measure drift: fall back to the frequency-search
        # resolution, the same scale used for the omega entry of P0.
        var_omega = omega_resolution**2
        notes["omega_source"] = "resolution_fallback"
    else:
        notes["omega_source"] = f"{n_windows} sliding windows"
    q[layout.omega] = var_omega * ratio

    q = np.maximum(q * q_scale, q_floor)
    notes["q_diag"] = q.copy()
    notes["state_names"] = layout.names
    return np.diag(q), notes
