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
    args = p.parse_args()

    out = args.out or args.run_dir / "ekf_detail.png"

    print("running baseline (no omega_bounds)...")
    baseline = run_variant(args.run_dir, args.config, args.baseline_q_scale, None)
    print("running tuned (omega_bounds_fraction set)...")
    tuned = run_variant(args.run_dir, args.config, args.tuned_q_scale,
                        args.tuned_omega_bounds_fraction)

    ident_bpm = float(tuned.ident.diagnostics["bpm_hat"])
    layout_b = StateLayout(baseline.ident.K)
    layout_t = StateLayout(tuned.ident.K)
    hb, ht = baseline.history, tuned.history

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    ax1.plot(hb.t - hb.t[0], hb.s[:, layout_b.omega] * TO_BPM, color="tab:red", lw=1.2,
             label=f"baseline (q_scale={args.baseline_q_scale}, no omega_bounds)")
    ax1.plot(ht.t - ht.t[0], ht.s[:, layout_t.omega] * TO_BPM, color="tab:blue", lw=1.2,
             label=f"tuned (q_scale={args.tuned_q_scale}, "
                    f"omega_bounds=+-{args.tuned_omega_bounds_fraction:.0%})")
    ax1.axhline(ident_bpm, color="black", ls="--", lw=1, label=f"Stage 1 rate ({ident_bpm:.2f} bpm)")
    b_mean, b_std = float(hb.s[:, layout_b.omega].mean() * TO_BPM), float(hb.s[:, layout_b.omega].std() * TO_BPM)
    t_mean, t_std = float(ht.s[:, layout_t.omega].mean() * TO_BPM), float(ht.s[:, layout_t.omega].std() * TO_BPM)
    ax1.set_ylabel("tracked omega_r (bpm)")
    ax1.set_xlabel("time (s)")
    ax1.set_title(f"frequency lock -- baseline {b_mean:.2f}+-{b_std:.2f} bpm vs "
                  f"tuned {t_mean:.2f}+-{t_std:.2f} bpm (truth {ident_bpm:.2f})")
    ax1.legend(fontsize=8, loc="upper right")

    if ht.forecast is not None and ht.forecast_target is not None:
        span_s = args.breaths * (2.0 * np.pi / tuned.ident.diagnostics["omega_hat"])
        t0 = ht.t[0] + 0.5 * (ht.t[-1] - ht.t[0] - span_s)
        m = (ht.t >= t0) & (ht.t <= t0 + span_s)
        te = ht.t[m] - ht.t[0]
        err = ht.forecast - ht.forecast_target
        naive_err = ht.y - ht.forecast_target
        rmse = float(np.sqrt(np.nanmean(err**2)))
        naive_rmse = float(np.sqrt(np.nanmean(naive_err**2)))

        ax2.plot(te, ht.forecast_target[m], color="tab:red", lw=2.0, alpha=0.9, label="truth", zorder=3)
        ax2.plot(te, ht.y[m], color="tab:gray", lw=1.2, alpha=0.8, label="raw sensor", zorder=2)
        ax2.plot(te, ht.forecast[m], color="#00204d", lw=1.4, ls=(0, (5, 2)),
                 label=f"forecast (h={ht.horizon:.3f}s)", zorder=4)
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("[mm]")
        ax2.set_title(f"forecast RMSE {rmse:.4f}mm vs raw sensor RMSE {naive_rmse:.4f}mm "
                      f"(tuned config)")
        ax2.legend(fontsize=8, loc="upper right")
    else:
        ax2.text(0.5, 0.5, "no forecast (horizon=0)", transform=ax2.transAxes, ha="center")

    fig.suptitle(f"{args.run_dir.parent.name}/{args.run_dir.name}")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
