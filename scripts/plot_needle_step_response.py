#!/usr/bin/env python3
"""Plot a run recorded by scripts/run_needle_step_response.py.

Reads samples.jsonl (t, elapsed, phase, step_index, commanded_velocity_limit_rad_s,
motor_velocity_rad_s, motor_position_mm, ...) via ct.rt.telemetry.load_jsonl/to_arrays and the
sibling summary.json, and renders one figure per step: commanded velocity limit vs. measured
velocity on top, commanded vs. measured position below.

This is a look-and-sanity-check tool, not the fitter (scripts/fit_needle_plant.py does that) --
its job is answering the question run_needle_step_response.py's docstring raises: does the
velocity channel actually show a recognizable second-order transient, or does the firmware's
trajectory controller instead produce a simple ramp with no discernible transient shape (a
jerk-limited trapezoid, common in commercial trajectory firmware)? If it's the latter, the
second-order model doesn't apply and that is itself a finding, not something to force-fit.

**The velocity panel plots differentiated position, not the reply's own motor_velocity_rad_s,
against the commanded limit** -- the first real run (2026-09-05) showed that reply field
reading 6-7x too high and essentially uncorrelated (r=-0.06) with actual motion, matching a
provisional-units caveat scripts/listen_needle_motor.py had already raised about this exact
field. See scripts/fit_needle_plant.py's docstring for the full story. The raw reply is still
drawn, on its own right-hand axis, purely so a future run can show whether that offset is
consistent (worth pinning down properly) or was specific to that one run.

    python scripts/plot_needle_step_response.py                          # most recent run
    python scripts/plot_needle_step_response.py --run outputs/needle_step_response/20260905-120000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ct.rt.telemetry import load_jsonl, to_arrays  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "needle_step_response"
DRUM_RADIUS_M = 0.018  # keep in sync with run_needle_step_response.py / fit_needle_plant.py


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default=None,
                         help="run directory or samples.jsonl path; default: most recent under "
                              "outputs/needle_step_response/")
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

    step_indices = sorted({int(r["step_index"]) for r in records if r.get("step_index") is not None})
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}

    n_steps = len(step_indices)
    if n_steps == 0:
        print("error: no step_index found in records")
        return 1

    fig, axes = plt.subplots(2, n_steps, figsize=(4.5 * n_steps, 7), sharey="row", squeeze=False)

    for col, step_index in enumerate(step_indices):
        step_records = [r for r in records if r.get("step_index") == step_index]
        step_cols = to_arrays(step_records, [
            "t", "elapsed", "commanded_velocity_limit_rad_s", "motor_velocity_rad_s",
            "commanded_target_mm", "motor_position_mm",
        ])
        is_step_phase = np.array([r.get("phase") == "step" for r in step_records])

        ax_v, ax_p = axes[0][col], axes[1][col]

        # Velocity panel: the "step" phase only, x-axis reset to elapsed-since-step-start --
        # this is the transient scripts/fit_needle_plant.py fits against. The fit target is
        # |d(position)/dt|, not the reply's motor_velocity_rad_s -- see module docstring.
        t_step = step_cols["elapsed"][is_step_phase]
        diff_velocity_rad_s = np.abs(
            (np.gradient(step_cols["motor_position_mm"], step_cols["elapsed"]) / 1000.0) / DRUM_RADIUS_M
        )[is_step_phase]
        ax_v.plot(t_step, step_cols["commanded_velocity_limit_rad_s"][is_step_phase],
                  lw=1.0, color="C0", label="commanded limit")
        ax_v.plot(t_step, diff_velocity_rad_s,
                  lw=0.8, color="C1", ls="--", marker=".", ms=3, label="|d(position)/dt| (fit target)")
        ax_v.set_title(f"step {step_index}")
        ax_v.set_ylabel("velocity [rad/s]" if col == 0 else "")
        ax_v.set_xlabel("t since step start [s]")
        ax_v.grid(alpha=0.3)

        ax_v_raw = ax_v.twinx()
        ax_v_raw.plot(t_step, np.abs(step_cols["motor_velocity_rad_s"][is_step_phase]),
                      lw=0.6, color="C3", ls=":", alpha=0.7, label="|raw reply| (untrusted scale)")
        ax_v_raw.set_ylabel("raw reply [rad/s]" if col == n_steps - 1 else "", color="C3")
        ax_v_raw.tick_params(axis="y", labelcolor="C3")

        if col == 0:
            lines_l, labels_l = ax_v.get_legend_handles_labels()
            lines_r, labels_r = ax_v_raw.get_legend_handles_labels()
            ax_v.legend(lines_l + lines_r, labels_l + labels_r, loc="lower right", fontsize=7)

        # Position panel: the full step+retract timeline, on the run's own clock, so the two
        # phases don't overlap on the x-axis the way per-phase "elapsed" would.
        t_full = step_cols["t"] - step_cols["t"][0]
        ax_p.plot(t_full, step_cols["commanded_target_mm"], lw=1.0, color="C0", label="commanded")
        ax_p.plot(t_full, step_cols["motor_position_mm"], lw=0.8, color="C1", ls="--", marker=".", ms=3,
                  label="measured")
        ax_p.set_xlabel("t since step+retract start [s]")
        ax_p.set_ylabel("position [mm]" if col == 0 else "")
        ax_p.grid(alpha=0.3)
        if col == 0:
            ax_p.legend(loc="lower right", fontsize=8)

    fig.suptitle(f"needle step response — {summary.get('velocities_rad_s', '?')} rad/s, "
                 f"{summary.get('reps', '?')} rep(s) each")
    fig.tight_layout()
    out_path = run_dir / "needle_step_response.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    print(f"saved: {out_path}")
    print("\neyeball check: does each 'step' panel's measured velocity show a recognizable "
          "second-order rise (possible overshoot, then settle) toward the commanded limit? If "
          "it instead looks like a straight ramp with a sharp corner (no overshoot, no visible "
          "transient shape), the firmware's trajectory controller is likely jerk-limited rather "
          "than second-order -- treat that as a finding, not something to force-fit with "
          "scripts/fit_needle_plant.py.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
