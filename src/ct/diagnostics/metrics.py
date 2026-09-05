"""Filter-health metrics.

These are how you tell "the filter is running" from "the filter is right". The
central one is the NIS: for a correctly-specified filter the normalised
innovation squared is chi-square with 1 degree of freedom (one degree because the
measurement is scalar). Consistently high NIS means ``Q`` or ``R`` is too small
or the model is wrong; consistently low means they are too large and the filter
is ignoring data it should be using.
"""

from __future__ import annotations

import numpy as np
from scipy import stats


def nis_bounds(alpha: float = 0.05, dof: int = 1, n_average: int = 1) -> tuple[float, float]:
    """Two-sided acceptance interval for the (optionally averaged) NIS.

    Averaging ``n_average`` samples tightens the interval by the usual
    chi-square(N*dof)/N scaling — useful when eyeballing a long run, where a few
    isolated excursions past the single-sample bound mean nothing.
    """
    lo = stats.chi2.ppf(alpha / 2.0, dof * n_average) / n_average
    hi = stats.chi2.ppf(1.0 - alpha / 2.0, dof * n_average) / n_average
    return float(lo), float(hi)


def rmse(a: np.ndarray, b: np.ndarray) -> float:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.sqrt(np.mean((a - b) ** 2)))


def normalised_innovations(innovation: np.ndarray, S: np.ndarray) -> np.ndarray:
    """``nu / sqrt(S)`` — should be zero-mean, unit-variance and white."""
    return np.asarray(innovation, float) / np.sqrt(np.asarray(S, float))


def innovation_autocorrelation(nu: np.ndarray, max_lag: int = 30) -> tuple[np.ndarray, float]:
    """Sample autocorrelation of the normalised innovations, plus its 95% bound.

    A white sequence stays inside ``+-1.96/sqrt(N)``. Structure here means the
    model is leaving something predictable in the residual — usually too few
    harmonics, or an ``omega_r`` that is drifting faster than ``Q`` allows.
    """
    nu = np.asarray(nu, float)
    nu = nu - nu.mean()
    n = nu.size
    max_lag = int(min(max_lag, n - 2))
    denom = float(nu @ nu)
    if denom <= 0:
        return np.zeros(max_lag + 1), 0.0
    ac = np.array([float(nu[: n - k] @ nu[k:]) / denom for k in range(max_lag + 1)])
    return ac, 1.96 / np.sqrt(n)


def ljung_box(nu: np.ndarray, max_lag: int = 20) -> tuple[float, float]:
    """Ljung-Box test for whiteness. Returns ``(statistic, p_value)``.

    Small p means the innovations are correlated, i.e. the filter is
    mis-specified. Reported alongside the autocorrelation plot so the eyeball
    check has a number behind it.
    """
    nu = np.asarray(nu, float)
    n = nu.size
    ac, _ = innovation_autocorrelation(nu, max_lag)
    lags = np.arange(1, min(max_lag, ac.size - 1) + 1)
    stat = n * (n + 2) * float(np.sum(ac[1 : lags.size + 1] ** 2 / (n - lags)))
    return stat, float(stats.chi2.sf(stat, lags.size))


def summarize(
    innovation: np.ndarray,
    S: np.ndarray,
    nis: np.ndarray,
    alpha: float = 0.05,
) -> dict[str, float]:
    """One-line health summary of a tracking run."""
    nu = normalised_innovations(innovation, S)
    lo, hi = nis_bounds(alpha)
    inside = float(np.mean((nis >= lo) & (nis <= hi)))
    stat, p = ljung_box(nu)
    return {
        "n_steps": int(np.size(nis)),
        "innovation_mean": float(np.mean(innovation)),
        "innovation_std": float(np.std(innovation)),
        "normalised_innovation_mean": float(np.mean(nu)),
        "normalised_innovation_std": float(np.std(nu)),
        "nis_mean": float(np.mean(nis)),
        "nis_expected": 1.0,
        "nis_fraction_inside": inside,
        "nis_fraction_expected": 1.0 - alpha,
        "ljung_box_stat": stat,
        "ljung_box_p": p,
    }


def is_frequency_locked(
    tracked_mean: float, tracked_std: float, ident_bpm: float, tolerance: float = 0.15
) -> bool:
    """Did the tracker hold the frequency Stage 1 handed it, within ``tolerance``?

    Both the tracked mean's distance from Stage 1's rate and the tracked std must be under
    ``tolerance * ident_bpm`` — a tight mean with a wide std is still not locked, since
    ``phi_1`` can wander far enough between samples to fake a low mean. Shared by the bench
    report (``scripts/plot_approach_and_seat.py``) and any offline sweep that needs the exact
    same definition of "locked", so the threshold can't drift between the two.
    """
    return (
        abs(tracked_mean - ident_bpm) < tolerance * ident_bpm
        and tracked_std < tolerance * ident_bpm
    )


def forecast_errors(
    predicted: np.ndarray, actual: np.ndarray, warmup: int = 0
) -> dict[str, float]:
    """Forecast accuracy, discarding the filter warm-up transient.

    Samples where ``actual`` is NaN are dropped rather than counted. That is how
    a forecast reaching past the end of a finite recording is handled: there is
    no ground truth at ``t+h``, so it is not scored. ``n_unscored`` reports how
    many were skipped, so a silently shrinking sample is visible.
    """
    p = np.asarray(predicted, float)[warmup:]
    a = np.asarray(actual, float)[warmup:]
    valid = np.isfinite(a) & np.isfinite(p)
    err = p[valid] - a[valid]
    if err.size == 0:
        return {"rmse": float("nan"), "mae": float("nan"), "max_abs": float("nan"),
                "bias": float("nan"), "n": 0, "n_unscored": int(valid.size)}
    return {
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mae": float(np.mean(np.abs(err))),
        "max_abs": float(np.max(np.abs(err))),
        "bias": float(np.mean(err)),
        "n": int(err.size),
        "n_unscored": int(valid.size - err.size),
    }
