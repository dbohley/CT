#!/usr/bin/env python3
"""How much do the FFT and autocorrelation frequency estimators disagree, per run?

``ct.identification.spectral.coarse_omega`` runs two independent frequency estimators (FFT
peak, autocorrelation peak) and warns via ``warnings.warn`` when they disagree by more than
10% -- a cheap early signal the recording is not a clean periodic breathing trace. Running the
full parameter sweep (``sweep_ekf_params.py``) calls ``identify()`` once per ``q_scale`` (11
values by default) and each of those calls ``coarse_omega`` again internally, several more
times, while estimating Q's ``omega`` variance over sliding sub-windows -- so a real sweep run
can fire this warning 100+ times, each with a different embedded percentage so Python's default
warning dedup does not collapse them. That volume is what makes it look "massive"; this script
answers the actual question -- how much do the two estimators disagree, per run -- with one
clean row each, instead of wading through the raw warning spam.

    python scripts/analyze_frequency_disagreement.py outputs/param_sweep_runs
    python scripts/analyze_frequency_disagreement.py outputs/param_sweep_runs/derek_normal_breathing/trial_2
"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

from sweep_ekf_params import ensure_aligned_csv, resolve_run_dirs


def check_one_run(aligned_csv: Path, config_name: str) -> dict:
    from ct.config import RunConfig
    from ct.registry import build_identifier
    from ct.run import build_source_from_config, load_batch

    cfg = RunConfig.from_yaml(config_name)
    cfg.source = {**cfg.source, "params": {**cfg.source["params"], "path": str(aligned_csv)}}
    source = build_source_from_config(cfg)
    batch = load_batch(cfg, source)
    t0 = float(batch.t[0])
    calib = batch.slice_time(t0, t0 + min(cfg.calib_seconds, 0.5 * batch.duration))

    identifier = build_identifier(cfg.identifier["name"], cfg.identifier.get("params"))
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        ident = identifier.identify(calib)
        n_warnings = sum(1 for w in caught if "disagree" in str(w.message))

    return {
        "stage1_disagreement": ident.diagnostics.get("relative_disagreement", float("nan")),
        "bpm_hat": ident.diagnostics["bpm_hat"],
        "sub_window_warnings": n_warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--config", default="bench_aligned")
    parser.add_argument("--min-contact-s", type=float, default=1.0, dest="min_contact_s")
    args = parser.parse_args()

    run_dirs = resolve_run_dirs(args.inputs, args.min_contact_s)
    rows = []
    for run_dir in run_dirs:
        aligned = ensure_aligned_csv(run_dir)
        if aligned is None:
            print(f"skipping {run_dir}: could not build aligned.csv")
            continue
        result = check_one_run(aligned, args.config)
        rows.append((run_dir, result))

    rows.sort(key=lambda r: r[1]["stage1_disagreement"], reverse=True)
    print(f"\n{'run':50s} {'disagreement':>13s} {'bpm_hat':>8s} {'sub-window warnings':>20s}")
    for run_dir, r in rows:
        label = f"{run_dir.parent.name}/{run_dir.name}"
        print(f"{label:50s} {r['stage1_disagreement']:>12.1%} {r['bpm_hat']:>8.2f} "
              f"{r['sub_window_warnings']:>20d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
