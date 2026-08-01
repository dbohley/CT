"""Orchestration: config -> source -> identify -> track -> forecast -> metrics.

The CLI scripts are thin wrappers over this module, so every entry point runs the
same code path and a result obtained from ``ct-pipeline`` is reproducible from a
notebook or a test with three lines.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

import ct.identification  # noqa: F401  (registers identifiers)
import ct.sources  # noqa: F401  (registers sources)
import ct.tracking  # noqa: F401  (registers trackers)
from ct.config import RunConfig
from ct.diagnostics import metrics
from ct.layout import StateLayout
from ct.registry import build_identifier, build_source, build_tracker
from ct.types import IdentificationResult, SignalBatch


@dataclass
class TrackingHistory:
    """Per-step record of a tracking run, as arrays ready for plotting."""

    t: np.ndarray
    s: np.ndarray  # (N, n)
    P_diag: np.ndarray  # (N, n)
    y: np.ndarray
    y_pred: np.ndarray
    innovation: np.ndarray
    S: np.ndarray
    nis: np.ndarray
    y_clean: np.ndarray | None = None
    forecast: np.ndarray | None = None
    forecast_target: np.ndarray | None = None
    horizon: float = 0.0

    @property
    def n_steps(self) -> int:
        return int(self.t.size)


@dataclass
class PipelineResult:
    """Everything one end-to-end run produced."""

    config: RunConfig
    batch: SignalBatch
    calibration: SignalBatch
    ident: IdentificationResult
    history: TrackingHistory
    summary: dict[str, Any] = field(default_factory=dict)

    @property
    def layout(self) -> StateLayout:
        return StateLayout(self.ident.K)


def build_source_from_config(cfg: RunConfig):
    params = dict(cfg.source.get("params") or {})
    # fs/seed live at the top level of the config so every stage agrees on them;
    # push them down unless the source spec overrides explicitly.
    params.setdefault("fs", cfg.fs)
    if cfg.source["name"] != "csv":
        params.setdefault("seed", cfg.seed)
    return build_source(cfg.source["name"], params)


def load_batch(cfg: RunConfig, source: Any = None) -> SignalBatch:
    """Full recording for a run: ``duration`` seconds from the configured source."""
    source = source or build_source_from_config(cfg)
    return source.batch(cfg.duration)


def truth_function(source: Any, batch: SignalBatch) -> Callable[[np.ndarray], np.ndarray]:
    """Best available ground truth at arbitrary times, for forecast scoring.

    Synthetic sources can be evaluated exactly at ``t+h``. Real data cannot, so it
    falls back to interpolating the measured trace — which folds sensor noise into
    the score, and is worth remembering when comparing the two.
    """
    if hasattr(source, "clean"):
        return lambda tq: np.asarray(source.clean(np.asarray(tq, float)), float)
    ref = batch.y_clean if batch.y_clean is not None else batch.y
    # NaN outside the record rather than np.interp's default clamp-to-endpoint:
    # a forecast at t+h past the last sample has no truth to be scored against,
    # and clamping would silently charge the filter for the record simply ending.
    return lambda tq: np.interp(np.asarray(tq, float), batch.t, ref, left=np.nan, right=np.nan)


def track(
    tracker: Any,
    batch: SignalBatch,
    ident: IdentificationResult,
    horizon: float = 0.0,
    truth_at: Callable[[np.ndarray], np.ndarray] | None = None,
) -> TrackingHistory:
    """Run a tracker over a batch, recording everything each step produced."""
    tracker.init(ident)
    n = batch.N
    layout = StateLayout(ident.K)

    s_hist = np.empty((n, layout.n))
    p_hist = np.empty((n, layout.n))
    y_pred = np.empty(n)
    innov = np.empty(n)
    S_arr = np.empty(n)
    nis = np.empty(n)
    fcast = np.empty(n) if horizon > 0 else None

    for i, (t, y) in enumerate(zip(batch.t, batch.y)):
        out = tracker.step(float(t), float(y))
        s_hist[i] = out.s
        p_hist[i] = np.diag(out.P)
        y_pred[i] = out.y_pred
        innov[i] = out.innovation
        S_arr[i] = out.S
        nis[i] = out.nis
        if fcast is not None:
            fcast[i] = tracker.forecast(horizon)

    target = None
    if fcast is not None and truth_at is not None:
        target = truth_at(batch.t + horizon)

    return TrackingHistory(
        t=batch.t,
        s=s_hist,
        P_diag=p_hist,
        y=batch.y,
        y_pred=y_pred,
        innovation=innov,
        S=S_arr,
        nis=nis,
        y_clean=batch.y_clean,
        forecast=fcast,
        forecast_target=target,
        horizon=horizon,
    )


def run_pipeline(cfg: RunConfig, warmup_breaths: float = 2.0) -> PipelineResult:
    """Generate/load -> identify on the calibration window -> track the rest."""
    source = build_source_from_config(cfg)
    batch = load_batch(cfg, source)

    t0 = float(batch.t[0])
    calibration = batch.slice_time(t0, t0 + cfg.calib_seconds)
    tracking = batch.slice_time(t0 + cfg.calib_seconds, float(batch.t[-1]) + 1.0)
    if tracking.N < 10:
        raise ValueError(
            f"only {tracking.N} samples left to track after a {cfg.calib_seconds}s "
            f"calibration window; increase duration or reduce calib_seconds"
        )

    identifier = build_identifier(cfg.identifier["name"], cfg.identifier.get("params"))
    ident = identifier.identify(calibration)

    tracker = build_tracker(cfg.tracker["name"], cfg.tracker.get("params"))
    history = track(
        tracker,
        tracking,
        ident,
        horizon=cfg.horizon,
        truth_at=truth_function(source, batch),
    )

    T_breath = 2.0 * np.pi / ident.diagnostics["omega_hat"]
    warmup = int(min(history.n_steps // 2, round(warmup_breaths * T_breath * cfg.fs)))

    summary: dict[str, Any] = {
        "K": ident.K,
        "bpm_hat": ident.diagnostics["bpm_hat"],
        "R": ident.R,
        "R_source": ident.diagnostics["R_source"],
        "warmup_steps": warmup,
        "warmup_breaths": warmup_breaths,
        **metrics.summarize(
            history.innovation[warmup:], history.S[warmup:], history.nis[warmup:]
        ),
    }
    if history.y_clean is not None:
        summary["tracking_rmse_vs_clean"] = metrics.rmse(
            history.y_pred[warmup:], history.y_clean[warmup:]
        )
        summary["noise_std"] = float(np.std(history.y[warmup:] - history.y_clean[warmup:]))
    if history.forecast is not None and history.forecast_target is not None:
        summary["forecast"] = metrics.forecast_errors(
            history.forecast, history.forecast_target, warmup=warmup
        )
        summary["horizon"] = cfg.horizon

    return PipelineResult(
        config=cfg,
        batch=batch,
        calibration=calibration,
        ident=ident,
        history=history,
        summary=summary,
    )


def sweep_horizons(cfg: RunConfig, horizons: list[float]) -> list[dict[str, Any]]:
    """Forecast accuracy as a function of ``h``.

    Identification and tracking are done once; only the forecast is re-evaluated
    per horizon, since ``h`` does not affect the filter state at all.
    """
    source = build_source_from_config(cfg)
    batch = load_batch(cfg, source)
    t0 = float(batch.t[0])
    calibration = batch.slice_time(t0, t0 + cfg.calib_seconds)
    tracking = batch.slice_time(t0 + cfg.calib_seconds, float(batch.t[-1]) + 1.0)

    identifier = build_identifier(cfg.identifier["name"], cfg.identifier.get("params"))
    ident = identifier.identify(calibration)
    tracker = build_tracker(cfg.tracker["name"], cfg.tracker.get("params"))

    layout = StateLayout(ident.K)
    from ct.forecast import forecast_value

    history = track(tracker, tracking, ident, horizon=0.0)
    truth_at = truth_function(source, batch)
    T_breath = 2.0 * np.pi / ident.diagnostics["omega_hat"]
    warmup = int(min(history.n_steps // 2, round(2.0 * T_breath * cfg.fs)))

    rows = []
    for h in horizons:
        pred = np.array([forecast_value(s, layout, h) for s in history.s])
        actual = truth_at(history.t + h)
        rows.append({"horizon": float(h), **metrics.forecast_errors(pred, actual, warmup)})
    return rows
