"""Diagnostic figures.

Every plot answers a specific question about the filter:

    signal        does the generated / loaded trace look like breathing?
    identification did the harmonic fit capture the waveform, and where did K land?
    tracking      is the one-step prediction following the measurement?
    states        are the states converging, and are the +-3 sigma bounds honest?
    innovations   are the residuals white and correctly scaled (NIS ~ 1)?
    forecast      does forecast error grow smoothly with the horizon?

Rendering uses the Agg backend so the scripts work headless.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

import numpy as np  # noqa: E402

from ct.diagnostics import metrics  # noqa: E402
from ct.layout import StateLayout  # noqa: E402
from ct.types import IdentificationResult, SignalBatch  # noqa: E402

_SECONDS_SHOWN = 30.0


def _save(fig: plt.Figure, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def _window(t: np.ndarray, seconds: float = _SECONDS_SHOWN) -> np.ndarray:
    """Mask for the last ``seconds`` of a trace — full runs are unreadable."""
    return t >= (t[-1] - seconds)


def plot_signal(batch: SignalBatch, path: str | Path, title: str = "Signal") -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(11, 6))
    axes[0].plot(batch.t, batch.y, lw=0.8, label="measured")
    if batch.y_clean is not None:
        axes[0].plot(batch.t, batch.y_clean, lw=1.0, alpha=0.7, label="clean")
    axes[0].set(title=f"{title} — full record", ylabel="displacement")
    axes[0].legend(loc="upper right")

    m = _window(batch.t)
    axes[1].plot(batch.t[m], batch.y[m], lw=1.0, marker=".", ms=2, label="measured")
    if batch.y_clean is not None:
        axes[1].plot(batch.t[m], batch.y_clean[m], lw=1.2, alpha=0.8, label="clean")
    axes[1].set(title=f"last {_SECONDS_SHOWN:.0f}s", xlabel="t [s]", ylabel="displacement")
    axes[1].legend(loc="upper right")
    for ax in axes:
        ax.grid(alpha=0.3)
    return _save(fig, path)


def plot_identification(
    result: IdentificationResult, batch: SignalBatch, path: str | Path
) -> Path:
    d = result.diagnostics
    fig, axes = plt.subplots(2, 2, figsize=(13, 7))

    # Spectrum with the identified fundamental marked.
    ax = axes[0, 0]
    f, mag = d["spectrum_freqs"], d["spectrum_mag"]
    band = f <= min(f[-1], 8 * d["bpm_hat"] / 60.0)
    ax.semilogy(f[band], np.maximum(mag[band], 1e-12), lw=0.9)
    for k in range(1, d["Kmax"] + 1):
        fk = k * d["bpm_hat"] / 60.0
        if fk <= f[band][-1]:
            ax.axvline(fk, color="C1" if k <= result.K else "C3", ls="--", alpha=0.6, lw=0.9)
    ax.set(
        title=f"spectrum — f0={d['bpm_hat'] / 60:.4f} Hz ({d['bpm_hat']:.2f} bpm)",
        xlabel="frequency [Hz]",
        ylabel="|Y|",
    )
    ax.grid(alpha=0.3)

    # Harmonic energy and the K cut.
    ax = axes[0, 1]
    ks = np.arange(1, d["amplitudes_wide"].size + 1)
    ax.bar(ks, d["energy_fraction"], alpha=0.7, label="per-harmonic energy")
    ax.plot(ks, d["cumulative_energy"], "o-", color="C1", label="cumulative")
    ax.axhline(d["energy_threshold"], color="C3", ls="--", label="threshold")
    ax.axvline(result.K + 0.5, color="k", ls=":", label=f"K = {result.K}")
    ax.set(title="harmonic energy (Parseval)", xlabel="harmonic k", ylabel="fraction", ylim=(0, 1.05))
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Fit overlay over the last few breaths.
    from ct.identification.harmonic_ls import design_matrix

    ax = axes[1, 0]
    layout = StateLayout(result.K)
    a0, A, phi, _, omega = layout.unpack(result.s0)
    t_ref = float(batch.t[0])
    X = design_matrix(batch.t, omega, result.K, t_ref)
    coeffs = np.concatenate([[a0], np.ravel([[Ak * np.cos(p), Ak * np.sin(p)] for Ak, p in zip(A, phi)])])
    fitted = X @ coeffs
    m = _window(batch.t)
    ax.plot(batch.t[m], batch.y[m], lw=0.9, label="measured")
    ax.plot(batch.t[m], fitted[m], lw=1.3, label=f"K={result.K} fit")
    ax.set(title="harmonic fit (last 30s of calibration)", xlabel="t [s]", ylabel="displacement")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Residual: what the truncated model failed to represent.
    ax = axes[1, 1]
    resid = batch.y - fitted
    ax.plot(batch.t, resid, lw=0.7)
    ax.axhline(0, color="k", lw=0.6)
    ax.set(
        title=f"fit residual — var={result.diagnostics['residual_var']:.4g}, R={result.R:.4g}",
        xlabel="t [s]",
        ylabel="residual",
    )
    ax.grid(alpha=0.3)
    return _save(fig, path)


def plot_tracking(history: Any, path: str | Path) -> Path:
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    m = _window(history.t)

    ax = axes[0]
    ax.plot(history.t[m], history.y[m], lw=0.8, alpha=0.6, label="measured")
    ax.plot(history.t[m], history.y_pred[m], lw=1.3, label="EKF one-step prediction")
    if history.y_clean is not None:
        ax.plot(history.t[m], history.y_clean[m], lw=1.0, ls="--", alpha=0.8, label="clean truth")
    if history.forecast is not None:
        ax.plot(
            history.t[m] + history.horizon,
            history.forecast[m],
            lw=1.1,
            alpha=0.85,
            label=f"forecast at t+{history.horizon:g}s",
        )
    ax.set(title=f"tracking — last {_SECONDS_SHOWN:.0f}s", ylabel="displacement")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(history.t, history.innovation, lw=0.7, label="innovation")
    ax.plot(history.t, 3 * np.sqrt(history.S), color="C3", lw=0.8, alpha=0.7, label="+-3 sqrt(S)")
    ax.plot(history.t, -3 * np.sqrt(history.S), color="C3", lw=0.8, alpha=0.7)
    ax.set(title="innovation over the whole run", xlabel="t [s]", ylabel="y - y_pred")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    return _save(fig, path)


def plot_states(history: Any, layout: StateLayout, path: str | Path, truth: dict | None = None) -> Path:
    names = layout.names
    n = layout.n
    ncols = 3
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 2.6 * nrows), squeeze=False)

    for i in range(nrows * ncols):
        ax = axes[i // ncols][i % ncols]
        if i >= n:
            ax.axis("off")
            continue
        s = history.s[:, i]
        sd = np.sqrt(np.maximum(history.P_diag[:, i], 0.0))
        ax.fill_between(history.t, s - 3 * sd, s + 3 * sd, alpha=0.25, label="+-3 sigma")
        ax.plot(history.t, s, lw=1.0)
        tv = _truth_value(names[i], truth)
        if tv is not None:
            ax.axhline(tv, color="C3", ls="--", lw=1.0, label="truth")
        ax.set_title(names[i], fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="best")
    fig.suptitle("state trajectories with +-3 sigma bounds", y=1.0)
    return _save(fig, path)


def _truth_value(name: str, truth: dict | None) -> float | None:
    """Ground-truth overlay for a state, when the generator provides one.

    Only states with an unambiguous counterpart are overlaid: ``omega_r`` always,
    amplitudes when the source publishes exact Fourier coefficients. ``theta``
    and the phases depend on the time origin and are deliberately left alone.
    """
    if not truth:
        return None
    if name == "omega_r":
        return truth.get("omega_r")
    if name.startswith("A_"):
        k = int(name.split("_")[1])
        exact = truth.get("exact_amplitudes") or truth.get("amplitudes")
        if exact is not None and k <= len(exact):
            return float(exact[k - 1])
    return None


def plot_innovations(history: Any, path: str | Path, warmup: int = 0) -> Path:
    nu = metrics.normalised_innovations(history.innovation, history.S)[warmup:]
    nis = history.nis[warmup:]
    t = history.t[warmup:]
    lo, hi = metrics.nis_bounds(0.05)

    fig, axes = plt.subplots(2, 2, figsize=(13, 7))

    ax = axes[0, 0]
    ax.plot(t, nu, lw=0.6)
    for level, style in ((0, "-"), (2, "--"), (-2, "--")):
        ax.axhline(level, color="k" if level == 0 else "C3", lw=0.8, ls=style)
    ax.set(title="normalised innovation", xlabel="t [s]", ylabel="nu / sqrt(S)")
    ax.grid(alpha=0.3)

    ax = axes[0, 1]
    ax.semilogy(t, np.maximum(nis, 1e-12), lw=0.6)
    ax.axhline(1.0, color="k", lw=0.9, label="expected NIS = 1")
    ax.axhspan(lo, hi, color="C2", alpha=0.15, label="95% chi2(1) band")
    ax.set(title=f"NIS — mean {np.mean(nis):.3f}", xlabel="t [s]", ylabel="NIS")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1, 0]
    ac, bound = metrics.innovation_autocorrelation(nu)
    ax.stem(np.arange(ac.size), ac, basefmt=" ")
    ax.axhspan(-bound, bound, color="C2", alpha=0.2, label="95% white-noise band")
    _, p = metrics.ljung_box(nu)
    ax.set(title=f"innovation autocorrelation (Ljung-Box p={p:.3g})", xlabel="lag", ylabel="r")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1, 1]
    ax.hist(nu, bins=50, density=True, alpha=0.7)
    grid = np.linspace(-4, 4, 200)
    ax.plot(grid, np.exp(-0.5 * grid**2) / np.sqrt(2 * np.pi), color="C3", label="N(0,1)")
    ax.set(title="normalised innovation distribution", xlabel="nu / sqrt(S)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    return _save(fig, path)


def plot_forecast(history: Any, path: str | Path, warmup: int = 0) -> Path:
    if history.forecast is None or history.forecast_target is None:
        raise ValueError("history has no forecast to plot")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    m = _window(history.t)

    ax = axes[0]
    ax.plot(history.t[m] + history.horizon, history.forecast_target[m], lw=1.2, label="truth at t+h")
    ax.plot(
        history.t[m] + history.horizon,
        history.forecast[m],
        lw=1.2,
        ls="--",
        label=f"forecast (h={history.horizon:g}s)",
    )
    ax.set(title=f"forecast vs truth, last {_SECONDS_SHOWN:.0f}s", ylabel="displacement")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    # NaNs mark forecasts reaching past the end of the record, which have no
    # truth to be scored against; they show as gaps and are excluded from RMSE.
    err = history.forecast - history.forecast_target
    ax.plot(history.t[warmup:], err[warmup:], lw=0.7)
    ax.axhline(0, color="k", lw=0.6)
    ax.set(
        title=f"forecast error — RMSE {np.sqrt(np.nanmean(err[warmup:] ** 2)):.4g}",
        xlabel="t [s]",
        ylabel="predicted - actual",
    )
    ax.grid(alpha=0.3)
    return _save(fig, path)


def plot_horizon_sweep(rows: Sequence[dict], path: str | Path, amplitude: float | None = None) -> Path:
    h = np.array([r["horizon"] for r in rows])
    rmse = np.array([r["rmse"] for r in rows])
    mx = np.array([r["max_abs"] for r in rows])

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(h, rmse, "o-", label="RMSE")
    ax.plot(h, mx, "s--", alpha=0.7, label="max |error|")
    if amplitude:
        ax.axhline(0.05 * amplitude, color="C3", ls=":", label="5% of amplitude")
    ax.set(
        title="forecast error vs horizon",
        xlabel="horizon h [s]",
        ylabel="error [displacement units]",
    )
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    return _save(fig, path)
