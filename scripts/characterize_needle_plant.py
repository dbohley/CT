#!/usr/bin/env python3
"""One entry point for the full needle plant characterization protocol this session's manual
back-and-forth converged on, so it can be run once and produce one organized write-up-ready
report instead of several ad hoc invocations and mental bookkeeping.

**What this actually does, and why it's shaped this way.** Everything here calls the existing,
already-hardware-verified scripts (`run_needle_step_response.py`, `fit_needle_plant.py`,
`plot_needle_step_response.py`) as subprocesses rather than reimplementing any CAN logic --
consistent with this repo's convention of duplicating small protocol constants across bench
scripts rather than importing between them, and it means every safety check, travel cap, and
interactive "Proceed? [y/N]" confirmation those scripts already have is inherited unchanged.
**Real motion still requires you to confirm at each stage** -- this script organizes the
sequence and the analysis, it does not remove the operator from the loop for any commanded
motion, matching this project's established preference (session 017) for a human watching every
physical move rather than an unattended batch.

Two real findings from this session's manual runs shaped the protocol:

1. A single step's fit is unreliable on its own -- `wn`/`zeta` swung wildly (10-34 rad/s,
   0.17-1.02) between otherwise-identical single-rep runs, while `K` (the DC gain) was
   consistently repeatable. Only comparing multiple reps revealed which numbers were real.
2. The **first** rep of a batch run right after the needle has been sitting still consistently
   looked different from the reps after it (one clear outlier: wn=25.9/zeta=0.17 vs. the other
   four's tight 10.1-12.1 rad/s / 0.71-0.79 cluster) -- the natural explanation is static
   friction/stiction on breakaway from a dead stop, though this is not yet confirmed against
   more than one occurrence.

So this script:

1. Pre-flight: `read_needle_position.py` -- refuses to proceed on a decoded fault or no reply.
2. **Cold-start stage** (`--cold-start-reps`, default 3): single-rep steps run as *separate*
   invocations, `--cold-start-rest-s` (default 5s) apart, to see whether the first-rep effect
   in (2) above reproduces on repeat "first moves after rest" rather than being a one-off.
   ``--cold-start-rest-s`` is a best-effort proxy for "the needle has been sitting still" --
   whether 5s is actually long enough to reset any stiction/backlash state is itself unverified
   and worth adjusting (up or down) if the cold-start reps don't look consistent with each other.
3. **Warm stage** (`--velocities`, default "0.15,0.30"; `--reps` each, default 5): a multi-rep
   batch per velocity, back-to-back reps (the way session data already showed rep 0 of a batch
   can itself look different from reps 1+, since only ~0.5s separates them -- not the same as
   the cold-start stage's actual rest, but reported the same way for consistency).
4. Fits every stage's data (`fit_needle_plant.py`) and plots it (`plot_needle_step_response.py`,
   best-effort -- a plotting failure does not abort the run).
5. Aggregates everything into one `report.json` and one `report.md` (a table-based summary
   meant to paste directly into a session doc), splitting every stage's first rep from its
   later reps and flagging >25% relative spread the same way `fit_needle_plant.py` already does
   for per-step drift, plus a velocity-to-velocity linearity check across the warm stage.

    python scripts/characterize_needle_plant.py --dry-run
    python scripts/characterize_needle_plant.py
    python scripts/characterize_needle_plant.py --cold-start-reps 5 --velocities 0.15,0.30,0.45
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_plant_characterization"

DEFAULT_VELOCITIES = "0.15,0.30"
DEFAULT_REPS = 5
DEFAULT_COLD_START_REPS = 3
DEFAULT_COLD_START_REST_S = 5.0
DEFAULT_STEP_DURATION_S = 1.0
DEFAULT_COMMAND_HZ = 50.0
DEFAULT_STEP_TRAVEL_MM = 15.0
DEFAULT_MAX_STEP_TRAVEL_MM = 20.0
DEFAULT_RETRACT_VELOCITY_RAD_S = 0.2

MATERIAL_SPREAD_THRESHOLD = 0.25  # matches fit_needle_plant.py's own std/mean flag threshold


def parse_velocities(raw: str) -> list[float]:
    values = [float(v) for v in raw.split(",") if v.strip()]
    if not values:
        raise ValueError(f"--velocities produced no values from {raw!r}")
    return values


def run(cmd: list[str], dry_run: bool) -> int:
    print("  $ " + " ".join(cmd))
    if dry_run:
        return 0
    result = subprocess.run(cmd)
    return result.returncode


def run_or_abort(cmd: list[str], dry_run: bool, what: str) -> None:
    code = run(cmd, dry_run)
    if code != 0:
        print(f"\n{what} failed (exit {code}) -- stopping the characterization run here.")
        sys.exit(code)


def preflight(dry_run: bool) -> None:
    print("\n=== pre-flight: read_needle_position.py ===")
    run_or_abort([sys.executable, str(SCRIPTS_DIR / "read_needle_position.py")], dry_run,
                 "pre-flight check")


def run_step_response(
    out_dir: Path, velocities: str, reps: int, step_duration_s: float, command_hz: float,
    step_travel_mm: float, max_step_travel_mm: float, retract_velocity_rad_s: float,
    zero: bool, dry_run: bool, what: str,
) -> None:
    print(f"\n=== {what} ===")
    print(f"  -> {out_dir}")
    cmd = [
        sys.executable, str(SCRIPTS_DIR / "run_needle_step_response.py"),
        "--velocities", velocities,
        "--reps", str(reps),
        "--step-duration-s", str(step_duration_s),
        "--command-hz", str(command_hz),
        "--step-travel-mm", str(step_travel_mm),
        "--max-step-travel-mm", str(max_step_travel_mm),
        "--retract-velocity-rad-s", str(retract_velocity_rad_s),
        "--out", str(out_dir),
    ]
    if zero:
        cmd.append("--zero")
    run_or_abort(cmd, dry_run, what)


def fit_and_plot(out_dir: Path, dry_run: bool) -> dict[str, Any] | None:
    fit_cmd = [sys.executable, str(SCRIPTS_DIR / "fit_needle_plant.py"), "--run", str(out_dir)]
    print(f"  $ {' '.join(fit_cmd)}")
    if dry_run:
        return None
    result = subprocess.run(fit_cmd, capture_output=True, text=True)
    print(result.stdout)
    if result.returncode != 0:
        print(f"  fit failed (exit {result.returncode}):\n{result.stderr}")
        return None

    plot_cmd = [sys.executable, str(SCRIPTS_DIR / "plot_needle_step_response.py"), "--run", str(out_dir)]
    plot_result = subprocess.run(plot_cmd, capture_output=True, text=True)
    if plot_result.returncode != 0:
        print(f"  (plot failed, non-fatal: {plot_result.stderr.strip()[:200]})")

    fit_path = out_dir / "plant_fit.json"
    if not fit_path.exists():
        return None
    return json.loads(fit_path.read_text())


def split_first_vs_rest(per_step: list[dict]) -> tuple[dict, list[dict]]:
    """The first rep of any batch (cold-start or warm) is treated separately throughout this
    script -- see module docstring for why."""
    ordered = sorted(per_step, key=lambda r: r.get("step_index", 0))
    return ordered[0], ordered[1:]


def stats(values: list[float]) -> dict[str, float]:
    import numpy as np
    arr = np.array(values, dtype=float)
    mean = float(arr.mean())
    std = float(arr.std())
    return {"mean": mean, "std": std, "spread": std / abs(mean) if mean else float("nan"),
            "min": float(arr.min()), "max": float(arr.max()), "n": len(values)}


def summarize_rest(rest: list[dict]) -> dict[str, dict[str, float]] | None:
    if len(rest) < 2:
        return None
    return {key: stats([r[key] for r in rest]) for key in ("K", "wn", "zeta")}


def markdown_table(rows: list[dict], columns: list[str]) -> str:
    header = "| " + " | ".join(columns) + " |"
    sep = "|" + "|".join("---" for _ in columns) + "|"
    lines = [header, sep]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(c, "")) for c in columns) + " |")
    return "\n".join(lines)


def build_report(cold_start: list[dict[str, Any]], warm: dict[str, dict[str, Any]]) -> dict[str, Any]:
    report: dict[str, Any] = {"cold_start": [], "warm": {}, "linearity_check": {}, "flags": []}

    for i, fit in enumerate(cold_start):
        if fit is None:
            continue
        first, rest = split_first_vs_rest(fit["per_step"])
        report["cold_start"].append({"trial": i, "fit": first})

    warm_rest_summaries: dict[str, dict] = {}
    for v_str, fit in warm.items():
        if fit is None:
            continue
        first, rest = split_first_vs_rest(fit["per_step"])
        rest_summary = summarize_rest(rest)
        report["warm"][v_str] = {"first_rep": first, "rest_reps": rest, "rest_summary": rest_summary}
        if rest_summary is not None:
            warm_rest_summaries[v_str] = rest_summary
            for key in ("K", "wn", "zeta"):
                if rest_summary[key]["spread"] > MATERIAL_SPREAD_THRESHOLD:
                    report["flags"].append(
                        f"v={v_str}: {key} varies >{MATERIAL_SPREAD_THRESHOLD*100:.0f}% across "
                        f"reps 1+ (spread={rest_summary[key]['spread']*100:.1f}%) -- the linear "
                        "model may only be a local approximation at this velocity."
                    )

    velocities = list(warm_rest_summaries.keys())
    for i in range(len(velocities)):
        for j in range(i + 1, len(velocities)):
            va, vb = velocities[i], velocities[j]
            comparison = {}
            for key in ("K", "wn", "zeta"):
                ma, mb = warm_rest_summaries[va][key]["mean"], warm_rest_summaries[vb][key]["mean"]
                rel_diff = abs(ma - mb) / max(abs(ma), abs(mb), 1e-9)
                comparison[key] = {"a": ma, "b": mb, "rel_diff": rel_diff}
                if rel_diff > MATERIAL_SPREAD_THRESHOLD:
                    report["flags"].append(
                        f"{key} differs {rel_diff*100:.1f}% between v={va} and v={vb} (reps 1+ "
                        "means) -- worth checking whether the linear model holds across "
                        "amplitudes this different."
                    )
            report["linearity_check"][f"{va}_vs_{vb}"] = comparison

    if len(cold_start) >= 2:
        cs_fits = [c["fit"] for c in report["cold_start"]]
        cs_stats = {key: stats([f[key] for f in cs_fits]) for key in ("K", "wn", "zeta")}
        report["cold_start_summary"] = cs_stats
        for key in ("K", "wn", "zeta"):
            if cs_stats[key]["spread"] > MATERIAL_SPREAD_THRESHOLD:
                report["flags"].append(
                    f"cold-start {key} varies >{MATERIAL_SPREAD_THRESHOLD*100:.0f}% across "
                    f"{len(cold_start)} independent cold-start trials (spread="
                    f"{cs_stats[key]['spread']*100:.1f}%) -- the stiction/breakaway effect "
                    "itself may not be consistent, not just different from the warm reps."
                )

    return report


def render_markdown(report: dict[str, Any], run_dir: Path, args: argparse.Namespace) -> str:
    lines = [f"# Needle plant characterization — {run_dir.name}", ""]
    lines.append(f"Settings: `--command-hz {args.command_hz} --step-duration-s "
                 f"{args.step_duration_s} --step-travel-mm {args.step_travel_mm}`")
    lines.append("")

    lines.append("## Cold-start reps (single step from rest, separate invocations)")
    lines.append("")
    if report["cold_start"]:
        rows = [{"trial": c["trial"], "K": f"{c['fit']['K']:.3f}", "wn": f"{c['fit']['wn']:.2f}",
                 "zeta": f"{c['fit']['zeta']:.3f}", "residual_rms": f"{c['fit']['residual_rms']:.4f}"}
                for c in report["cold_start"]]
        lines.append(markdown_table(rows, ["trial", "K", "wn", "zeta", "residual_rms"]))
        if "cold_start_summary" in report:
            s = report["cold_start_summary"]
            lines.append("")
            lines.append(f"Across {len(report['cold_start'])} trials: "
                         f"K={s['K']['mean']:.3f}±{s['K']['std']:.3f}, "
                         f"wn={s['wn']['mean']:.2f}±{s['wn']['std']:.2f}, "
                         f"zeta={s['zeta']['mean']:.3f}±{s['zeta']['std']:.3f}")
    else:
        lines.append("(skipped)")
    lines.append("")

    lines.append("## Warm multi-rep runs")
    lines.append("")
    for v_str, data in report["warm"].items():
        lines.append(f"### v={v_str} rad/s")
        lines.append("")
        first = data["first_rep"]
        rest = data["rest_reps"]
        rows = [{"rep": "0 (first)", "K": f"{first['K']:.3f}", "wn": f"{first['wn']:.2f}",
                 "zeta": f"{first['zeta']:.3f}", "residual_rms": f"{first['residual_rms']:.4f}"}]
        for r in rest:
            rows.append({"rep": r.get("step_index", "?"), "K": f"{r['K']:.3f}", "wn": f"{r['wn']:.2f}",
                        "zeta": f"{r['zeta']:.3f}", "residual_rms": f"{r['residual_rms']:.4f}"})
        lines.append(markdown_table(rows, ["rep", "K", "wn", "zeta", "residual_rms"]))
        if data["rest_summary"] is not None:
            s = data["rest_summary"]
            lines.append("")
            lines.append(f"Reps 1+ ({len(rest)}): K={s['K']['mean']:.3f}±{s['K']['std']:.3f}, "
                         f"wn={s['wn']['mean']:.2f}±{s['wn']['std']:.2f}, "
                         f"zeta={s['zeta']['mean']:.3f}±{s['zeta']['std']:.3f}")
        lines.append("")

    if report["linearity_check"]:
        lines.append("## Linearity check (reps 1+ means, across velocities)")
        lines.append("")
        for pair, comp in report["linearity_check"].items():
            rows = [{"param": k, "a": f"{v['a']:.3f}", "b": f"{v['b']:.3f}",
                    "rel_diff": f"{v['rel_diff']*100:.1f}%"} for k, v in comp.items()]
            lines.append(f"**{pair}**")
            lines.append("")
            lines.append(markdown_table(rows, ["param", "a", "b", "rel_diff"]))
            lines.append("")

    lines.append("## Flags")
    lines.append("")
    if report["flags"]:
        for flag in report["flags"]:
            lines.append(f"- {flag}")
    else:
        lines.append("(none)")

    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--velocities", default=DEFAULT_VELOCITIES,
                         help="comma-separated velocities [rad/s] for the warm multi-rep stage")
    parser.add_argument("--reps", type=int, default=DEFAULT_REPS,
                         help="reps per velocity in the warm stage")
    parser.add_argument("--cold-start-reps", type=int, default=DEFAULT_COLD_START_REPS, dest="cold_start_reps",
                         help="number of separate single-step cold-start trials; 0 to skip")
    parser.add_argument("--cold-start-rest-s", type=float, default=DEFAULT_COLD_START_REST_S,
                         dest="cold_start_rest_s", help="seconds between cold-start trials")
    parser.add_argument("--step-duration-s", type=float, default=DEFAULT_STEP_DURATION_S, dest="step_duration_s")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ, dest="command_hz")
    parser.add_argument("--step-travel-mm", type=float, default=DEFAULT_STEP_TRAVEL_MM, dest="step_travel_mm")
    parser.add_argument("--max-step-travel-mm", type=float, default=DEFAULT_MAX_STEP_TRAVEL_MM,
                         dest="max_step_travel_mm")
    parser.add_argument("--retract-velocity-rad-s", type=float, default=DEFAULT_RETRACT_VELOCITY_RAD_S,
                         dest="retract_velocity_rad_s")
    parser.add_argument("--zero", action="store_true",
                         help="re-zero before the very first stage only (pre-flight/cold-start "
                              "trial 0) -- only if you've confirmed the needle is at the "
                              "position you want to call 'retracted'")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/needle_plant_characterization/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print every command that would run, run nothing")
    args = parser.parse_args(argv)

    try:
        velocities = parse_velocities(args.velocities)
    except ValueError as exc:
        print(f"error: {exc}")
        return 1

    run_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    print(f"characterization run directory: {run_dir}")
    print(f"stages: pre-flight, {args.cold_start_reps} cold-start trial(s) "
          f"({args.cold_start_rest_s:.0f}s apart), warm runs at {velocities} rad/s "
          f"({args.reps} reps each)")
    if not args.dry_run:
        print("\nYou will be asked to confirm before each stage's real motion.")

    preflight(args.dry_run)

    cold_start_fits: list[dict[str, Any] | None] = []
    for i in range(args.cold_start_reps):
        stage_dir = run_dir / f"cold_start_{i:02d}"
        run_step_response(
            stage_dir, str(velocities[0]), 1, args.step_duration_s, args.command_hz,
            args.step_travel_mm, args.max_step_travel_mm, args.retract_velocity_rad_s,
            zero=(args.zero and i == 0), dry_run=args.dry_run,
            what=f"cold-start trial {i + 1}/{args.cold_start_reps} (v={velocities[0]} rad/s)",
        )
        fit = fit_and_plot(stage_dir, args.dry_run)
        cold_start_fits.append(fit)
        if i < args.cold_start_reps - 1:
            print(f"  resting {args.cold_start_rest_s:.0f}s before the next cold-start trial...")
            if not args.dry_run:
                time.sleep(args.cold_start_rest_s)

    warm_fits: dict[str, dict[str, Any] | None] = {}
    for v in velocities:
        v_str = str(v)
        stage_dir = run_dir / f"warm_v{v_str}"
        run_step_response(
            stage_dir, v_str, args.reps, args.step_duration_s, args.command_hz,
            args.step_travel_mm, args.max_step_travel_mm, args.retract_velocity_rad_s,
            zero=False, dry_run=args.dry_run, what=f"warm stage: v={v_str} rad/s, {args.reps} reps",
        )
        warm_fits[v_str] = fit_and_plot(stage_dir, args.dry_run)

    if args.dry_run:
        print("\n--dry-run: no report generated (no data was collected).")
        return 0

    cold_start_valid = [f for f in cold_start_fits if f is not None]
    if len(cold_start_valid) < len(cold_start_fits):
        print(f"\nwarning: {len(cold_start_fits) - len(cold_start_valid)} cold-start trial(s) "
              "produced no fit and were dropped from the report.")

    report = build_report(cold_start_valid, warm_fits)
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "report.json").write_text(json.dumps(report, indent=2))
    markdown = render_markdown(report, run_dir, args)
    (run_dir / "report.md").write_text(markdown)

    print(f"\nsaved: {run_dir / 'report.json'}")
    print(f"saved: {run_dir / 'report.md'}")
    print("\n" + markdown)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
