#!/usr/bin/env python3
"""Plot and score a run recorded by scripts/run_needle_sine_tracking.py.

Reads samples.jsonl (t, commanded_target_mm, motor_position_rad, motor_torque_nm, ...) via
ct.rt.telemetry.load_jsonl/to_arrays and the sibling summary.json, and renders a figure:
commanded vs. measured needle position on top, tracking error below, and a third panel for
torque if any reply data exists. Matches this project's existing plot conventions
(ct.diagnostics.rig_plots / scripts/plot_approach_and_stop.py -- Agg backend, dpi=130 PNG).

Tracking-quality numbers are reused rather than reimplemented: RMSE/MAE/max-abs/bias come
from ct.diagnostics.metrics.forecast_errors (same function ct-sweep-horizon uses to score the
estimator's own forecasts), and amplitude ratio / phase lag come from a short cross-
correlation calculation adapted from ct.phantom.driver.compare_logs's technique -- that
function itself isn't called directly because it's shaped for two separate JSONL logs with
different schemas (commanded_mm/tactile_mm across two processes), not one combined log with
a shared time axis.

    python scripts/plot_needle_sine_tracking.py                          # most recent run
    python scripts/plot_needle_sine_tracking.py --run outputs/needle_sine_tracking/20260827-120000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ct.diagnostics.metrics import forecast_errors  # noqa: E402
from ct.rt.telemetry import load_jsonl, to_arrays  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "needle_sine_tracking"
DRUM_RADIUS_M = 0.018  # keep in sync with run_needle_sine_tracking.py
DIRECTION_SIGN = -1  # keep in sync with run_needle_sine_tracking.py


def _latest_run_dir() -> Path:
    runs = sorted(p for p in DEFAULT_RUNS_DIR.iterdir() if p.is_dir()) if DEFAULT_RUNS_DIR.exists() else []
    if not runs:
        raise FileNotFoundError(f"no runs found under {DEFAULT_RUNS_DIR}")
    return runs[-1]


def resolve_run(run_arg: str | None) -> Path:
    """--run may be a run directory or a direct path to samples.jsonl; None picks the latest."""
    if run_arg is None:
        return _latest_run_dir()
    path = Path(run_arg)
    if path.is_file():
        return path.parent
    return path


def lag_and_amplitude_ratio(t: np.ndarray, commanded: np.ndarray, measured: np.ndarray,
                             max_lag_s: float = 2.0) -> dict[str, float]:
    """Cross-correlation lag + least-squares amplitude ratio between two series on one shared
    time axis, both already sampled at reply timestamps. Same technique as
    ct.phantom.driver.compare_logs, adapted here for one combined log instead of two."""
    fs = 1.0 / float(np.median(np.diff(t)))
    gc = measured - measured.mean()
    gp = commanded - commanded.mean()

    max_shift = int(max_lag_s * fs)
    correlation = np.correlate(gc, gp, mode="full")
    lags = np.arange(-len(gp) + 1, len(gp))
    keep = np.abs(lags) <= max_shift
    best = lags[keep][int(np.argmax(correlation[keep]))]
    lag_s = float(best / fs)

    denom = float(np.dot(gp, gp))
    amplitude_ratio = float(np.dot(gc, gp) / denom) if denom > 0 else float("nan")
    return {"lag_s": lag_s, "amplitude_ratio": amplitude_ratio, "fs": fs}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default=None,
                         help="run directory or samples.jsonl path; default: most recent under "
                              "outputs/needle_sine_tracking/")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = resolve_run(args.run)
    jsonl_path = run_dir / "samples.jsonl"
    summary_path = run_dir / "summary.json"

    if not jsonl_path.exists():
        print(f"error: no samples.jsonl found at {jsonl_path}")
        return 1

    records = load_jsonl(jsonl_path)
    if not records:
        print(f"error: {jsonl_path} has no records")
        return 1

    cols = to_arrays(records, ["t", "commanded_target_mm", "motor_position_rad", "motor_torque_nm"])
    t = cols["t"]
    commanded_mm = cols["commanded_target_mm"]
    motor_position_rad = cols["motor_position_rad"]
    motor_torque_nm = cols["motor_torque_nm"]
    has_motor_telemetry = bool(np.any(~np.isnan(motor_position_rad)))
    has_torque = bool(np.any(~np.isnan(motor_torque_nm)))

    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    start_rad = summary.get("start_rad", 0.0)
    amplitude_mm = summary.get("amplitude_mm")
    frequency_hz = summary.get("frequency_hz")

    title = "needle sine tracking"
    if amplitude_mm is not None and frequency_hz is not None:
        title += f" — {amplitude_mm:.1f}mm @ {frequency_hz:.3f}Hz"

    metrics: dict[str, float] = {}
    if has_motor_telemetry:
        measured_mm = (motor_position_rad - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0
        valid = np.isfinite(measured_mm) & np.isfinite(commanded_mm)
        if int(np.sum(valid)) >= 10:
            errs = forecast_errors(commanded_mm, measured_mm)
            metrics.update(errs)
            lag_info = lag_and_amplitude_ratio(t[valid], commanded_mm[valid], measured_mm[valid])
            metrics.update(lag_info)
        else:
            measured_mm = np.full_like(commanded_mm, np.nan)
    else:
        measured_mm = np.full_like(commanded_mm, np.nan)

    n_panels = 2 + (1 if has_torque else 0)
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 4 + 3 * n_panels), sharex=True)
    axes = np.atleast_1d(axes)

    axes[0].plot(t, commanded_mm, lw=1.0, label="commanded", color="C0")
    if has_motor_telemetry:
        axes[0].plot(t, measured_mm, lw=0.8, label="measured (motor reply)", color="C1", ls="--", marker=".", ms=3)
    axes[0].set_ylabel("needle extension [mm]")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[0].set_title(title)

    if has_motor_telemetry:
        axes[1].plot(t, commanded_mm - measured_mm, lw=0.8, color="C3", label="commanded - measured")
        if metrics.get("rmse") is not None and np.isfinite(metrics["rmse"]):
            axes[1].axhline(metrics["rmse"], color="k", lw=0.6, ls=":", alpha=0.6, label=f"RMSE={metrics['rmse']:.3f}mm")
            axes[1].axhline(-metrics["rmse"], color="k", lw=0.6, ls=":", alpha=0.6)
    axes[1].set_ylabel("tracking error [mm]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(alpha=0.3)

    if has_torque:
        axes[2].plot(t, motor_torque_nm, lw=0.8, color="C2", label="motor_torque_nm")
        axes[2].set_ylabel("torque [N*m]")
        axes[2].legend(loc="upper right", fontsize=8)
        axes[2].grid(alpha=0.3)

    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()
    out_path = run_dir / "needle_sine_tracking.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    print(f"saved: {out_path}")
    if metrics:
        print(f"\ntracking summary ({int(metrics.get('n', 0))} scored samples):")
        print(f"  RMSE:            {metrics['rmse']:.4f} mm")
        print(f"  MAE:             {metrics['mae']:.4f} mm")
        print(f"  max abs error:   {metrics['max_abs']:.4f} mm")
        print(f"  bias:            {metrics['bias']:.4f} mm")
        print(f"  amplitude ratio: {metrics['amplitude_ratio']:.3f} (measured/commanded, 1.0 = perfect)")
        print(f"  lag:             {metrics['lag_s']*1000:.1f} ms (cross-correlation peak)")
    elif has_motor_telemetry:
        print("\nnot enough real motor replies in this run to compute tracking metrics "
              "(need >= 10) -- check motor_replies_seen in summary.json.")
    else:
        print("\nno motor telemetry in this run -- nothing to score against the command.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
