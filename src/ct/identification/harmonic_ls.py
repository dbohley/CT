"""Least-squares harmonic regression at a fixed fundamental frequency.

With ``omega_r_hat`` held fixed the model

    x(t) ~= a0 + sum_{k=1..Kmax} [ alpha_k sin(k w t) + beta_k cos(k w t) ]

is *linear* in its coefficients — ``sin(k w t)`` and ``cos(k w t)`` are just
numbers once ``w`` and ``t`` are known. So this is ordinary least squares, and it
comes with a closed-form coefficient covariance that seeds ``P0``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import linalg


@dataclass(frozen=True)
class HarmonicFit:
    """Result of one OLS harmonic regression."""

    omega: float
    Kmax: int
    t_ref: float
    coeffs: np.ndarray  # [a0, alpha_1, beta_1, ..., alpha_Kmax, beta_Kmax]
    cov: np.ndarray  # Cov(coeffs) = sigma^2 (X^T X)^-1
    residual_var: float  # sigma_hat^2
    residuals: np.ndarray
    fitted: np.ndarray

    @property
    def a0(self) -> float:
        return float(self.coeffs[0])

    @property
    def alpha(self) -> np.ndarray:
        return self.coeffs[1::2]

    @property
    def beta(self) -> np.ndarray:
        return self.coeffs[2::2]

    @property
    def amplitudes(self) -> np.ndarray:
        """``A_k = sqrt(alpha_k^2 + beta_k^2)``."""
        return np.hypot(self.alpha, self.beta)

    @property
    def phases(self) -> np.ndarray:
        """``phi_k = atan2(beta_k, alpha_k)``, so ``alpha sin + beta cos = A sin(. + phi)``."""
        return np.arctan2(self.beta, self.alpha)

    def phase_at(self, t: float) -> float:
        """Fundamental phase ``theta = omega * (t - t_ref)`` at absolute time ``t``."""
        return self.omega * (t - self.t_ref)


def design_matrix(t: np.ndarray, omega: float, Kmax: int, t_ref: float = 0.0) -> np.ndarray:
    """Columns ``[1, sin(w tr), cos(w tr), sin(2w tr), cos(2w tr), ...]``.

    Times are referenced to ``t_ref`` so the fitted phases stay small and
    interpretable regardless of the absolute clock the recording carries.
    """
    if Kmax < 1:
        raise ValueError("Kmax must be >= 1")
    tr = np.asarray(t, dtype=float) - t_ref
    X = np.empty((tr.size, 1 + 2 * Kmax), dtype=float)
    X[:, 0] = 1.0
    for k in range(1, Kmax + 1):
        X[:, 2 * k - 1] = np.sin(k * omega * tr)
        X[:, 2 * k] = np.cos(k * omega * tr)
    return X


def fit_harmonics(
    t: np.ndarray,
    y: np.ndarray,
    omega: float,
    Kmax: int,
    t_ref: float | None = None,
) -> HarmonicFit:
    """Ordinary least squares fit of ``Kmax`` harmonics at fixed ``omega``."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    t_ref = float(t[0]) if t_ref is None else float(t_ref)

    X = design_matrix(t, omega, Kmax, t_ref)
    n, p = X.shape
    if n <= p:
        raise ValueError(f"need more samples ({n}) than coefficients ({p}); reduce Kmax")

    coeffs, *_ = linalg.lstsq(X, y)
    fitted = X @ coeffs
    residuals = y - fitted
    dof = n - p
    residual_var = float(residuals @ residuals) / dof

    # Cov(beta_hat) = sigma^2 (X^T X)^-1, via the pseudo-inverse for safety when
    # a short record makes the normal matrix poorly conditioned.
    XtX_inv = linalg.pinv(X.T @ X)
    cov = residual_var * XtX_inv

    return HarmonicFit(
        omega=float(omega),
        Kmax=int(Kmax),
        t_ref=t_ref,
        coeffs=coeffs,
        cov=cov,
        residual_var=residual_var,
        residuals=residuals,
        fitted=fitted,
    )


def select_K(amplitudes: np.ndarray, energy_threshold: float = 0.95) -> tuple[int, dict]:
    """Smallest ``K`` capturing ``energy_threshold`` of the harmonic energy.

    Parseval: harmonic energy is proportional to ``A_k^2``, so the criterion is

        sum_{k=1..K} A_k^2 / sum_{k=1..Kmax} A_k^2 >= threshold

    The DC term is excluded — it carries no shape information and would otherwise
    dominate an offset signal.
    """
    if not 0 < energy_threshold <= 1:
        raise ValueError("energy_threshold must be in (0, 1]")
    A = np.asarray(amplitudes, dtype=float)
    energy = A**2
    total = float(energy.sum())
    if total <= 0:
        raise ValueError("all harmonic amplitudes are zero")
    cumulative = np.cumsum(energy) / total
    K = int(np.searchsorted(cumulative, energy_threshold) + 1)
    K = min(K, A.size)
    return K, {
        "amplitudes": A,
        "energy_fraction": energy / total,
        "cumulative_energy": cumulative,
        "energy_threshold": energy_threshold,
        "K": K,
    }


def rect_to_polar_jacobian(alpha: float, beta: float) -> np.ndarray:
    """Delta-method Jacobian of ``(A, phi)`` w.r.t. ``(alpha, beta)``.

    ``A = hypot(alpha, beta)``, ``phi = atan2(beta, alpha)``, so

        dA/dalpha   = alpha/A      dA/dbeta   = beta/A
        dphi/dalpha = -beta/A^2    dphi/dbeta = alpha/A^2

    Used to push the OLS coefficient covariance into the amplitude/phase
    parameterisation the EKF state actually uses.
    """
    A2 = alpha**2 + beta**2
    A = np.sqrt(A2)
    if A < 1e-12:
        # Phase is undefined at zero amplitude; a large finite value keeps P0
        # positive-definite while honestly reporting "we know nothing".
        return np.array([[1.0, 0.0], [0.0, 0.0]])
    return np.array([[alpha / A, beta / A], [-beta / A2, alpha / A2]])
