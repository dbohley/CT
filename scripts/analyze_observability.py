#!/usr/bin/env python3
"""Check whether the EKF really does worse at the troughs, and why.

Two candidate mechanisms, both real and both already named somewhere in this repo, and this
script is what decides which one (or both) actually shows up in a given run's numbers:

1. **Observability.** ``measurement_jacobian`` (``ct.tracking.measurement``) gives
   ``dh/dtheta = sum_k k*A_k*cos(k*theta+phi_k)``, which vanishes near *any* extremum of the
   dominant harmonic -- peak or trough alike. Near an extremum the innovation carries almost
   no information to correct ``theta``/``omega``, only ``A_k``/``phi_k``. This predicts
   **symmetric** degradation: peaks should be just as bad as troughs.
2. **Contact loss.** ``ct.control.states.approach`` documents that a shallow seat lets the
   tactile arm lose the skin at end-exhale, clipping the trough of the waveform. This predicts
   **asymmetric** degradation: troughs specifically worse than peaks, because it is a physical
   effect at one end of the excursion, not a property of the harmonic model.

Buckets the same run's tracked history two ways -- by ``|H_theta|`` (tests mechanism 1) and by
whether each sample sits near the top or bottom of the waveform (tests mechanism 2) -- and
reports forecast error and innovation magnitude for each. Reuses the exact estimator path
``scripts/plot_approach_and_seat.py`` already runs (``ct.run.run_pipeline``); no reimplementation
of Stage 1/2.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from ct.config import RunConfig
from ct.run import run_pipeline
from ct.tracking.measurement import measurement_jacobian


def _rmse(x: np.ndarray) -> float:
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x**2))) if x.size else float("nan")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("aligned_csv", type=Path)
    p.add_argument("--config", default="bench_aligned")
    p.add_argument("--quantiles", type=int, default=4)
    p.add_argument("--edge-fraction", type=float, default=0.15,
                   help="fraction of the value range counted as 'near peak' / 'near trough'")
    args = p.parse_args()

    cfg = RunConfig.from_yaml(args.config)
    cfg.source = {**cfg.source, "params": {**cfg.source.get("params", {}), "path": str(args.aligned_csv)}}
    result = run_pipeline(cfg)
    h, layout = result.history, result.layout

    H_theta = np.array([abs(measurement_jacobian(s, layout)[layout.theta]) for s in h.s])
    if h.forecast is None or h.forecast_target is None:
        raise SystemExit("this run has no forecast_target (needs y_clean in the CSV) -- nothing to score")
    err = np.abs(h.forecast - h.forecast_target)
    innov = np.abs(h.innovation)
    valid = np.isfinite(err)

    print(f"n={int(valid.sum())} scored samples, horizon h={h.horizon:.3f}s\n")

    print("-- by |H_theta| (tests the observability mechanism; expect low quartile worse, "
          "peaks AND troughs pooled) --")
    edges = np.quantile(H_theta[valid], np.linspace(0, 1, args.quantiles + 1))
    for b in range(args.quantiles):
        lo, hi = edges[b], edges[b + 1]
        m = valid & (H_theta >= lo) & (H_theta <= hi if b == args.quantiles - 1 else H_theta < hi)
        print(f"  |H_theta| in [{lo:.3f}, {hi:.3f})  n={int(m.sum()):5d}  "
              f"forecast RMSE {_rmse(err[m]):.4f} mm  mean|innovation| {np.nanmean(innov[m]):.4f} mm")

    print("\n-- by position in the waveform (tests the contact-loss mechanism; peak vs trough, "
          "not pooled) --")
    y = h.y_clean if h.y_clean is not None else h.y
    lo_edge = np.quantile(y[valid], args.edge_fraction)
    hi_edge = np.quantile(y[valid], 1.0 - args.edge_fraction)
    trough = valid & (y <= lo_edge)
    peak = valid & (y >= hi_edge)
    mid = valid & ~trough & ~peak
    for name, m in (("trough (bottom %d%%)" % (100 * args.edge_fraction), trough),
                    ("mid-slope", mid),
                    ("peak (top %d%%)" % (100 * args.edge_fraction), peak)):
        print(f"  {name:22s}  n={int(m.sum()):5d}  forecast RMSE {_rmse(err[m]):.4f} mm  "
              f"mean|innovation| {np.nanmean(innov[m]):.4f} mm")

    trough_rmse, peak_rmse = _rmse(err[trough]), _rmse(err[peak])
    print(f"\ntrough/peak forecast RMSE ratio: {trough_rmse / peak_rmse:.2f}"
          f"  (1.0 = symmetric, i.e. mechanism 1 alone; >>1 favors mechanism 2)")


if __name__ == "__main__":
    main()
