#!/usr/bin/env python3
"""Sweep EKF tuning parameters across several collected bench runs and pick the safest winner.

Session 015 found ``identifier.params.q_scale=0.5`` measurably improved NIS, frequency lock and
forecast RMSE on one run (``20260903-171153``) -- then found, on a second trial of the exact
same profile (``20260903-185614``), that frequency lock was broken at *every* ``q_scale`` from
0.3 to 1.0, including the pre-session-015 default. One run cannot answer whether a setting
generalizes. This script takes several collected runs (from
``scripts/collect_sweep_trial.py``, or any directory of run directories) and sweeps
``identifier.params.q_scale`` x ``tracker.params.omega_bounds`` (as a fraction of Stage 1's
rate) over all of them, applying a safety-first rule: a setting that breaks frequency lock on
even one run is disqualified, no matter how good it is elsewhere. The winner is the minimum
mean forecast RMSE among what survives that -- and if nothing survives it, this says so plainly
rather than picking the least-bad option and calling it safe.

Hardware-free. Each run directory found under the given input path(s) is checked with the same
``validate_trial()`` the collector uses (fault_reason, phase_reached, contact_t), and anything
invalid is skipped with a printed reason -- there is no manifest to consult, since
``collect_sweep_trial.py`` runs one trial per invocation and never writes one. For each valid
run it needs ``<run>/aligned.csv``; if that is missing it is built by invoking
``scripts/plot_approach_and_seat.py --run <dir> --no-estimator`` (the existing, proven
CSV-builder -- not reimplemented here).

For each run, ``identify()`` is called once per ``q_scale`` (``q_scale`` affects ``Q``, computed
inside ``identify()``), and ``track()`` is called once per ``omega_bounds`` fraction against that
same identification (``omega_bounds`` is a tracker-only param) -- so the work is
``runs x |q_scale|`` identifications plus ``runs x |q_scale| x |omega_bounds|`` track calls,
not that many full pipelines.

    python scripts/sweep_ekf_params.py outputs/param_sweep_runs
    python scripts/sweep_ekf_params.py outputs/param_sweep_runs --quick
    python scripts/sweep_ekf_params.py outputs/approach_and_seat/20260903-171153 outputs/approach_and_seat/20260903-185614
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from _sweep_common import DEFAULT_MIN_CONTACT_S, validate_trial
from ct.config import RunConfig
from ct.diagnostics import metrics as metrics_mod
from ct.diagnostics.metrics import is_frequency_locked
from ct.layout import StateLayout
from ct.registry import build_identifier, build_tracker
from ct.run import build_source_from_config, load_batch, resolve_tracker_params, track, truth_function

REPO_ROOT = Path(__file__).resolve().parent.parent
PLOT_SCRIPT = REPO_ROOT / "scripts" / "plot_approach_and_seat.py"

DEFAULT_Q_SCALE_GRID = [0.3, 0.35, 0.4, 0.42, 0.45, 0.48, 0.5, 0.55, 0.6, 0.7, 1.0]
DEFAULT_OMEGA_BOUNDS_GRID: list[float | None] = [None, 0.1, 0.2, 0.3]
QUICK_Q_SCALE_GRID = [0.4, 0.5, 0.7, 1.0]
QUICK_OMEGA_BOUNDS_GRID: list[float | None] = [None, 0.2]


def resolve_run_dirs(inputs: list[Path], min_contact_s: float = DEFAULT_MIN_CONTACT_S) -> list[Path]:
    """Each input may be a bare run directory or a directory containing several run
    directories (searched recursively) -- either way, every candidate is checked with
    ``validate_trial()`` against its own summary.json, and anything invalid is skipped and
    named, since nothing here writes a manifest of what was already accepted."""
    candidates: list[Path] = []
    for inp in inputs:
        if (inp / "samples.jsonl").exists():
            candidates.append(inp)
            continue
        candidates += sorted({p.parent for p in inp.rglob("samples.jsonl")})
    seen: set[Path] = set()
    dirs = []
    for d in candidates:
        if d not in seen:
            seen.add(d)
            dirs.append(d)

    out = []
    for d in dirs:
        summary_path = d / "summary.json"
        if not summary_path.exists():
            print(f"  skipping {d}: no summary.json")
            continue
        summary = json.loads(summary_path.read_text())
        ok, reason = validate_trial(summary, min_contact_s)
        if not ok:
            print(f"  skipping {d}: {reason}")
            continue
        out.append(d)
    return out


def ensure_aligned_csv(run_dir: Path) -> Path | None:
    aligned = run_dir / "aligned.csv"
    if aligned.exists():
        return aligned
    print(f"  building aligned.csv for {run_dir}...")
    result = subprocess.run(
        [sys.executable, str(PLOT_SCRIPT), "--run", str(run_dir), "--no-estimator"],
        capture_output=True, text=True,
    )
    if result.returncode != 0 or not aligned.exists():
        print(f"  skipping {run_dir}: could not build aligned.csv ({result.stderr.strip()[-300:]})")
        return None
    return aligned


def sweep_one_run(
    aligned_csv: Path, run_label: str, config_name: str,
    q_scale_grid: list[float], omega_bounds_grid: list[float | None],
) -> list[dict[str, Any]]:
    """Identify once per q_scale, track once per omega_bounds fraction against it."""
    base = RunConfig.from_yaml(config_name)
    base.source = {**base.source, "params": {**base.source["params"], "path": str(aligned_csv)}}

    span = float(np.loadtxt(aligned_csv, delimiter=",", skiprows=1, usecols=0)[-1])
    if base.calib_seconds > 0.5 * span:
        base.calib_seconds = round(0.5 * span, 1)

    source = build_source_from_config(base)
    batch = load_batch(base, source)
    t0 = float(batch.t[0])
    calibration = batch.slice_time(t0, t0 + base.calib_seconds)
    tracking = batch.slice_time(t0 + base.calib_seconds, float(batch.t[-1]) + 1.0)
    if tracking.N < 10:
        print(f"  skipping {run_label}: only {tracking.N} samples left to track")
        return []
    truth_at = truth_function(source, batch)

    rows: list[dict[str, Any]] = []
    for q_scale in q_scale_grid:
        ident_params = {**base.identifier.get("params", {}), "q_scale": q_scale}
        identifier = build_identifier(base.identifier["name"], ident_params)
        ident = identifier.identify(calibration)
        ident_bpm = float(ident.diagnostics["bpm_hat"])
        omega_hat = float(ident.diagnostics["omega_hat"])
        layout = StateLayout(ident.K)
        T_breath = 2.0 * np.pi / omega_hat
        warmup = int(min((tracking.N) // 2, round(2.0 * T_breath * base.fs)))

        for frac in omega_bounds_grid:
            raw_params = {k: v for k, v in base.tracker.get("params", {}).items()
                          if k not in ("omega_bounds", "omega_bounds_fraction")}
            if frac is not None:
                raw_params["omega_bounds_fraction"] = frac
            tracker_params = resolve_tracker_params(raw_params, ident_bpm)
            tracker = build_tracker(base.tracker["name"], tracker_params)
            history = track(tracker, tracking, ident, horizon=base.horizon, truth_at=truth_at)

            to_bpm = 60.0 / (2.0 * np.pi)
            tracked = history.s[:, layout.omega] * to_bpm
            mean, std = float(tracked.mean()), float(tracked.std())
            locked = is_frequency_locked(mean, std, ident_bpm)
            nis_mean = float(np.mean(history.nis[warmup:]))
            if history.forecast is not None and history.forecast_target is not None:
                fe = metrics_mod.forecast_errors(history.forecast, history.forecast_target, warmup=warmup)
                rmse = fe["rmse"]
            else:
                rmse = float("nan")

            rows.append({
                "run": run_label, "q_scale": q_scale,
                "omega_bounds_frac": "unset" if frac is None else frac,
                "ident_bpm": ident_bpm, "tracked_mean_bpm": mean, "tracked_std_bpm": std,
                "locked": locked, "nis_mean": nis_mean, "forecast_rmse_mm": rmse,
            })
    return rows


def aggregate(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    runs = sorted({r["run"] for r in rows})
    n_runs = len(runs)
    keys = sorted(
        {(r["q_scale"], r["omega_bounds_frac"]) for r in rows},
        key=lambda kv: (kv[0], -1 if kv[1] == "unset" else kv[1]),
    )
    summary = []
    for q_scale, frac in keys:
        subset = [r for r in rows if r["q_scale"] == q_scale and r["omega_bounds_frac"] == frac]
        locked_count = sum(1 for r in subset if r["locked"])
        rmses = [r["forecast_rmse_mm"] for r in subset if np.isfinite(r["forecast_rmse_mm"])]
        summary.append({
            "q_scale": q_scale, "omega_bounds_frac": frac, "n_runs": n_runs,
            "locked_count": locked_count, "all_locked": locked_count == n_runs,
            "mean_rmse_mm": float(np.mean(rmses)) if rmses else float("nan"),
            "max_rmse_mm": float(np.max(rmses)) if rmses else float("nan"),
        })
    return summary


def pick_best(summary: list[dict[str, Any]]) -> dict[str, Any] | None:
    qualified = [s for s in summary if s["all_locked"]]
    if not qualified:
        return None
    return min(qualified, key=lambda s: s["mean_rmse_mm"])


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def plot_heatmap(summary: list[dict[str, Any]], best: dict[str, Any] | None, out_path: Path) -> None:
    q_values = sorted({s["q_scale"] for s in summary})
    frac_values = sorted({s["omega_bounds_frac"] for s in summary},
                          key=lambda f: (-1 if f == "unset" else f))
    grid = np.full((len(frac_values), len(q_values)), np.nan)
    disqualified = np.zeros_like(grid, dtype=bool)
    for s in summary:
        i, j = frac_values.index(s["omega_bounds_frac"]), q_values.index(s["q_scale"])
        grid[i, j] = s["mean_rmse_mm"]
        disqualified[i, j] = not s["all_locked"]

    fig, ax = plt.subplots(figsize=(max(9.0, 1.1 * len(q_values) + 2), 1.1 * len(frac_values) + 2))
    im = ax.imshow(grid, aspect="auto", cmap="viridis_r")
    fig.colorbar(im, ax=ax, label="mean forecast RMSE (mm), across runs")
    ax.set_xticks(range(len(q_values)))
    ax.set_xticklabels([f"{q:g}" for q in q_values])
    ax.set_yticks(range(len(frac_values)))
    ax.set_yticklabels([f"unset" if f == "unset" else f"+-{f:.0%}" for f in frac_values])
    ax.set_xlabel("q_scale")
    ax.set_ylabel("omega_bounds width")
    for i in range(len(frac_values)):
        for j in range(len(q_values)):
            if disqualified[i, j]:
                ax.text(j, i, "X", ha="center", va="center", color="red", fontsize=12, fontweight="bold")
    if best is not None:
        j = q_values.index(best["q_scale"])
        i = frac_values.index(best["omega_bounds_frac"])
        ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, edgecolor="lime", linewidth=3))
    ax.set_title("EKF parameter sweep -- red X = broke frequency lock on at least one run")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_per_run_lines(rows: list[dict[str, Any]], frac: Any, out_path: Path) -> None:
    subset = [r for r in rows if r["omega_bounds_frac"] == frac]
    runs = sorted({r["run"] for r in subset})
    fig, ax = plt.subplots(figsize=(8, 5))
    for run in runs:
        run_rows = sorted([r for r in subset if r["run"] == run], key=lambda r: r["q_scale"])
        xs = [r["q_scale"] for r in run_rows]
        ys = [r["forecast_rmse_mm"] for r in run_rows]
        locked = [r["locked"] for r in run_rows]
        ax.plot(xs, ys, marker="o", label=run)
        for x, y, ok in zip(xs, ys, locked):
            if not ok:
                ax.plot(x, y, marker="x", color="red", markersize=10, markeredgewidth=2)
    ax.set_xlabel("q_scale")
    ax.set_ylabel("forecast RMSE (mm)")
    frac_label = "unset" if frac == "unset" else f"+-{frac:.0%}"
    ax.set_title(f"forecast RMSE vs q_scale per run (omega_bounds={frac_label}) -- red x = lock broken")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("inputs", nargs="+", type=Path,
                         help="run directories, or a directory containing several nested run dirs")
    parser.add_argument("--config", default="bench_aligned")
    parser.add_argument("--quick", action="store_true", help="thinner grids, for fast iteration")
    parser.add_argument("--q-scale-grid", default=None, dest="q_scale_grid",
                         help="comma-separated, overrides --quick")
    parser.add_argument("--omega-bounds-grid", default=None, dest="omega_bounds_grid",
                         help="comma-separated fractions; include 'unset' for no bound")
    parser.add_argument("--min-contact-s", type=float, default=DEFAULT_MIN_CONTACT_S,
                         dest="min_contact_s", help="validation threshold, matches collect_sweep_trial.py")
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    q_grid = (
        [float(x) for x in args.q_scale_grid.split(",")] if args.q_scale_grid
        else QUICK_Q_SCALE_GRID if args.quick else DEFAULT_Q_SCALE_GRID
    )
    omega_grid: list[float | None] = (
        [None if x.strip() == "unset" else float(x) for x in args.omega_bounds_grid.split(",")]
        if args.omega_bounds_grid
        else QUICK_OMEGA_BOUNDS_GRID if args.quick else DEFAULT_OMEGA_BOUNDS_GRID
    )

    run_dirs = resolve_run_dirs(args.inputs, args.min_contact_s)
    if not run_dirs:
        print("no run directories found")
        return 1
    print(f"{len(run_dirs)} run(s): {[f'{d.parent.name}/{d.name}' for d in run_dirs]}")
    print(f"grid: q_scale={q_grid} x omega_bounds={omega_grid} "
          f"({len(run_dirs) * len(q_grid)} identify() calls, "
          f"{len(run_dirs) * len(q_grid) * len(omega_grid)} track() calls)")

    out_root = args.out or (REPO_ROOT / "outputs" / "param_sweep_runs" / "results")
    out_root.mkdir(parents=True, exist_ok=True)

    all_rows: list[dict[str, Any]] = []
    for run_dir in run_dirs:
        aligned = ensure_aligned_csv(run_dir)
        if aligned is None:
            continue
        label = f"{run_dir.parent.name}/{run_dir.name}"
        print(f"sweeping {label}...")
        all_rows += sweep_one_run(aligned, label, args.config, q_grid, omega_grid)

    if not all_rows:
        print("no runs produced any results")
        return 1

    write_csv(all_rows, out_root / "sweep_results.csv")
    summary = aggregate(all_rows)
    write_csv(summary, out_root / "sweep_summary.csv")
    best = pick_best(summary)

    if best is not None:
        print(f"\nBEST (safety-first): q_scale={best['q_scale']}, "
              f"omega_bounds={best['omega_bounds_frac']} -- "
              f"locked on {best['locked_count']}/{best['n_runs']} runs, "
              f"mean forecast RMSE {best['mean_rmse_mm']:.4f}mm")
        (out_root / "best_params.json").write_text(json.dumps(best, indent=2))
    else:
        max_locked = max(s["locked_count"] for s in summary)
        candidates = sorted(
            [s for s in summary if s["locked_count"] == max_locked],
            key=lambda s: s["mean_rmse_mm"],
        )[:5]
        print(f"\nNO setting keeps frequency lock on every run ({len(run_dirs)} runs). "
              f"Best achievable is locking {max_locked}/{len(run_dirs)}. Top candidates by "
              f"mean RMSE among those:")
        for c in candidates:
            print(f"  q_scale={c['q_scale']}, omega_bounds={c['omega_bounds_frac']}: "
                  f"locked {c['locked_count']}/{c['n_runs']}, RMSE {c['mean_rmse_mm']:.4f}mm")
        (out_root / "best_params.json").write_text(json.dumps({"best": None, "candidates": candidates}, indent=2))

    plot_heatmap(summary, best, out_root / "heatmap.png")
    winning_frac = best["omega_bounds_frac"] if best is not None else (
        max(summary, key=lambda s: (s["locked_count"], -s["mean_rmse_mm"]))["omega_bounds_frac"]
    )
    plot_per_run_lines(all_rows, winning_frac, out_root / "rmse_vs_q_scale.png")
    print(f"\nwrote {out_root}/sweep_results.csv, sweep_summary.csv, best_params.json, "
          f"heatmap.png, rmse_vs_q_scale.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
