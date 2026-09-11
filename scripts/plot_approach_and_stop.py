#!/usr/bin/env python3
"""Plot a run recorded by scripts/run_approach_and_stop.py.

Reads samples.jsonl (t, tof_mm, dist_cm, in_contact, and -- if the motor
actually replied, still unconfirmed -- motor_position_rad) via
ct.rt.telemetry.load_jsonl/to_arrays and the sibling summary.json, and renders a figure:
the tactile/encoder signal (with the contact threshold marked) on top, standoff distance
below, and a third panel for the motor's own reported position if any reply data exists in
the run -- the direct way to see whether an overshoot is the motor's control loop still
moving (motor_position_rad keeps climbing) or something downstream, like drivetrain
compliance (motor_position_rad stops early while the sensors keep drifting). All panels
share a contact-event marker and held-phase shading. Matches this project's existing plot
conventions (ct.diagnostics.plots / rig_plots -- Agg backend, dashed axvline event markers,
axvspan shading, dpi=130 PNG) rather than inventing a new look; this is the first script to
turn a bench script's JSONL into a figure.

    python scripts/plot_approach_and_stop.py                              # most recent run
    python scripts/plot_approach_and_stop.py --run outputs/approach_and_stop/20260827-172537
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
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "approach_and_stop"


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
                              "outputs/approach_and_stop/")
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
    cols = to_arrays(records, ["t", "tof_mm", "dist_cm", "motor_position_rad"])
    t, tof_mm, dist_cm = cols["t"], cols["tof_mm"], cols["dist_cm"]
    motor_position_rad = cols["motor_position_rad"]
    has_motor_telemetry = bool(np.any(~np.isnan(motor_position_rad)))

    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    contact_detected = bool(summary.get("contact_detected", False))
    time_to_contact_s = summary.get("time_to_contact_s")
    held_duration_s = summary.get("held_duration_s")
    threshold_cm = summary.get("contact_threshold_cm")

    if contact_detected and time_to_contact_s is not None:
        title = (f"approach & stop — contact at t={time_to_contact_s:.2f}s, "
                 f"held {held_duration_s:.2f}s" if held_duration_s is not None
                 else f"approach & stop — contact at t={time_to_contact_s:.2f}s")
    else:
        title = "approach & stop — no contact detected"

    n_panels = 3 if has_motor_telemetry else 2
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 8 if has_motor_telemetry else 6), sharex=True)

    axes[0].plot(t, dist_cm, lw=0.8, label="dist_cm (tactile/encoder)")
    if threshold_cm is not None:
        axes[0].axhline(threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6, label="contact threshold")
        axes[0].axhline(-threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6)
    axes[0].set_ylabel("dist_cm [cm]")
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[0].set_title(title)

    axes[1].plot(t, tof_mm, lw=0.8, label="tof_mm", color="C1")
    axes[1].set_ylabel("tof_mm [mm]")
    axes[1].legend(loc="upper left", fontsize=8)
    axes[1].grid(alpha=0.3)

    if has_motor_telemetry:
        axes[2].plot(t, motor_position_rad, lw=0.8, label="motor_position_rad (measured)", color="C2")
        axes[2].set_ylabel("motor position [rad]")
        axes[2].legend(loc="upper left", fontsize=8)
        axes[2].grid(alpha=0.3)

    axes[-1].set_xlabel("t [s]")

    if contact_detected and time_to_contact_s is not None:
        for ax in axes:
            ax.axvline(time_to_contact_s, color="k", lw=0.7, ls="--", alpha=0.6, label="_nolegend_")
            ax.axvspan(time_to_contact_s, t[-1], color="#ffe4c4", zorder=0)
        axes[0].annotate("contact", xy=(time_to_contact_s, axes[0].get_ylim()[1]),
                          xytext=(3, -10), textcoords="offset points", fontsize=8)

    fig.tight_layout()
    out_path = run_dir / "approach_and_stop.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    print(f"saved: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
