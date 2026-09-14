#!/usr/bin/env python3
"""See the frequency-lock fix and the forecast, zoomed in on one collected run.

Two panels, matching plot_lag_detail.py's zoomed-to-a-few-breaths style rather than the
compressed full-run figure:

- **Frequency lock over time**: tracked omega_r (bpm) under the BASELINE config (q_scale=0.5,
  no omega_bounds -- what configs/bench_aligned.yaml had before the session-018 sweep) against
  the TUNED config (q_scale=0.5, omega_bounds_fraction=0.1 -- what it has now), with Stage 1's
  own rate as a flat reference line. This is where the fix is dramatic: the real 10-run sweep
  found the baseline keeps lock on only 2/10 runs.
- **Forecast vs raw sensor vs truth**, zoomed to a few breaths, for the tuned config, RMSE
  annotated (the session-008 "payoff panel", but zoomed and dedicated rather than one of six
  panels in a compressed full-run figure).

Deliberately not oversold as "RMSE always improves" -- fixing lock does not uniformly improve
forecast RMSE per run (checked directly: some runs get slightly worse). What it reliably fixes
is the catastrophic-lock failure mode; the plot shows both panels so that is visible honestly
rather than implied.

    python scripts/plot_ekf_detail.py outputs/param_sweep_runs/emma_normal_breathing/trial_2
"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ct.config import RunConfig
from ct.layout import StateLayout
from ct.run import run_pipeline

TO_BPM = 60.0 / (2.0 * np.pi)


def run_variant(run_dir: Path, config_name: str, q_scale: float, omega_bounds_fraction: float | None):
    cfg = RunConfig.from_yaml(config_name)
    aligned = run_dir / "aligned.csv"
    cfg.source = {**cfg.source, "params": {**cfg.source["params"], "path": str(aligned)}}
    cfg.identifier = {**cfg.identifier, "params": {**cfg.identifier["params"], "q_scale": q_scale}}
    tracker_params = {k: v for k, v in cfg.tracker["params"].items() if k != "omega_bounds_fraction"}
    if omega_bounds_fraction is not None:
        tracker_params["omega_bounds_fraction"] = omega_bounds_fraction
    cfg.tracker = {**cfg.tracker, "params": tracker_params}

    span = float(np.loadtxt(aligned, delimiter=",", skiprows=1, usecols=0)[-1])
    if cfg.calib_seconds > 0.5 * span:
        cfg.calib_seconds = round(0.5 * span, 1)
    return run_pipeline(cfg)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dir", type=Path)
    p.add_argument("--config", default="bench_aligned")
    p.add_argument("--baseline-q-scale", type=float, default=0.5, dest="baseline_q_scale")
    p.add_argument("--tuned-q-scale", type=float, default=0.5, dest="tuned_q_scale")
    p.add_argument("--tuned-omega-bounds-fraction", type=float, default=0.1,
                   dest="tuned_omega_bounds_fraction")
    p.add_argument("--breaths", type=float, default=4.0, help="breath periods shown in panel B")
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--out_title", type=str, default=None)
    args = p.parse_args()
    subtitle = args.out_title if args.out_title is not None else ""
    out = args.out or args.run_dir / f"{args.run_dir.parent.name}_ekf_paper.pdf"

    print("running baseline (no omega_bounds)...")
    baseline = run_variant(args.run_dir, args.config, args.baseline_q_scale, None)
    print("running tuned (omega_bounds_fraction set)...")
    tuned = run_variant(args.run_dir, args.config, args.tuned_q_scale,
                        args.tuned_omega_bounds_fraction)
    hb, ht = baseline.history, tuned.history

    fig, ax2 = plt.subplots(figsize=(10, 3))
    if ht.forecast is not None and ht.forecast_target is not None:
        span_s = args.breaths * (2.0 * np.pi / tuned.ident.diagnostics["omega_hat"])
        t0 = ht.t[0] + 0.5 * (ht.t[-1] - ht.t[0] - span_s)
        m = (ht.t >= t0) & (ht.t <= t0 + span_s)
        te = ht.t[m] - ht.t[0]
        err = ht.forecast - ht.forecast_target
        naive_err = ht.y - ht.forecast_target
        rmse = float(np.sqrt(np.nanmean(err**2)))
        naive_rmse = float(np.sqrt(np.nanmean(naive_err**2)))

        ax2.plot(te, ht.forecast_target[m], color="#5FA73D", lw=2.0, alpha=0.8, label="truth", zorder=2)
        ax2.scatter(te, ht.y[m], color="#5B5A59", s=1.0, alpha=0.8, label="raw sensor", zorder=3)
        ax2.plot(te, ht.forecast[m], color="#FF0000", lw=2.0,
                 label=f"forecast (h={ht.horizon:.3f}s)", zorder=1)
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("[mm]")
        ax2.set_title(f"forecast RMSE {rmse:.4f}mm vs raw sensor RMSE {naive_rmse:.4f}mm "
                      f"(tuned config)")
        ax2.legend(fontsize=8, loc="upper right")
        ax2.grid(True) 

    else:
        ax2.text(0.5, 0.5, "no forecast (horizon=0)", transform=ax2.transAxes, ha="center")

    fig.suptitle(f"{subtitle}")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
