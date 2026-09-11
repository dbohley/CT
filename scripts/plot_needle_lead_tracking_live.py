#!/usr/bin/env python3
"""Plot and score a paired run recorded by scripts/run_needle_lead_tracking_live.py.

Reads both `<run>/uncompensated/samples.jsonl` and `<run>/compensated/samples.jsonl` plus the
shared `summary.json`, scores each phase the same way scripts/plot_needle_sine_tracking.py
already does (`ct.diagnostics.metrics.forecast_errors` for RMSE/MAE/bias,
a cross-correlation lag + least-squares amplitude ratio adapted from the same source), and
renders one figure with the reference and both measured traces overlaid so the compensator's
effect is visible directly, not just in a table.

**Real vs. predicted.** Next to the measured numbers this also prints what
`ct.control.servo.plant_response` / `closed_loop_response` / `residual_lag` predict at the run's
actual frequency, from the plant/lead model recorded in `summary.json` -- the same self-check
`scripts/simulate_needle_lead_tracking.py` already runs against its own simulated output, now
comparing against real hardware instead. Agreement or disagreement here is itself a finding
(session 019 found the real control architecture is feedforward+feedback, not the unity feedback
`closed_loop_response` assumes -- watch for the same mismatch here, not just for the compensator
"working").

    python scripts/plot_needle_lead_tracking_live.py                        # most recent run
    python scripts/plot_needle_lead_tracking_live.py --run outputs/needle_lead_live_tracking/20260907-120000
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ct.control.servo import closed_loop_response, plant_response, residual_lag  # noqa: E402
from ct.diagnostics.metrics import forecast_errors  # noqa: E402
from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel  # noqa: E402
from ct.rt.telemetry import load_jsonl, to_arrays  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "needle_lead_live_tracking"
DRUM_RADIUS_M = 0.018  # keep in sync with run_needle_lead_tracking_live.py
DIRECTION_SIGN = -1  # keep in sync with run_needle_lead_tracking_live.py


def _latest_run_dir() -> Path:
    runs = sorted(p for p in DEFAULT_RUNS_DIR.iterdir() if p.is_dir()) if DEFAULT_RUNS_DIR.exists() else []
    if not runs:
        raise FileNotFoundError(f"no runs found under {DEFAULT_RUNS_DIR}")
    return runs[-1]


def resolve_run(run_arg: str | None) -> Path:
    if run_arg is None:
        return _latest_run_dir()
    return Path(run_arg)


def lag_and_amplitude_ratio(t: np.ndarray, commanded: np.ndarray, measured: np.ndarray,
                             max_lag_s: float = 2.0) -> dict[str, float]:
    """Same technique as scripts/plot_needle_sine_tracking.py -- cross-correlation lag plus
    least-squares amplitude ratio between two series on one shared time axis."""
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


def score_phase(run_dir: Path, phase: str, start_rad: float) -> dict:
    jsonl_path = run_dir / phase / "samples.jsonl"
    records = load_jsonl(jsonl_path)
    if not records:
        return {"error": f"no records in {jsonl_path}"}

    cols = to_arrays(records, ["t", "reference_mm", "motor_position_rad"])
    t = cols["t"]
    reference_mm = cols["reference_mm"]
    motor_position_rad = cols["motor_position_rad"]
    has_telemetry = bool(np.any(~np.isnan(motor_position_rad)))
    if not has_telemetry:
        return {"error": f"no motor telemetry in {jsonl_path}", "t": t, "reference_mm": reference_mm,
                "measured_mm": np.full_like(reference_mm, np.nan)}

    measured_mm = (motor_position_rad - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0
    valid = np.isfinite(measured_mm) & np.isfinite(reference_mm)
    result: dict = {"t": t, "reference_mm": reference_mm, "measured_mm": measured_mm}
    if int(np.sum(valid)) >= 10:
        result.update(forecast_errors(reference_mm, measured_mm))
        result.update(lag_and_amplitude_ratio(t[valid], reference_mm[valid], measured_mm[valid]))
    else:
        result["error"] = f"fewer than 10 valid samples in {jsonl_path}"
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default=None,
                         help="run directory (containing uncompensated/ and compensated/); "
                              "default: most recent under outputs/needle_lead_live_tracking/")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = resolve_run(args.run)
    summary_path = run_dir / "summary.json"
    if not summary_path.exists():
        print(f"error: no summary.json found at {summary_path}")
        return 1
    summary = json.loads(summary_path.read_text())

    start_rad = summary.get("start_rad", 0.0)
    amplitude_mm = summary.get("amplitude_mm")
    frequency_hz = summary.get("frequency_hz")
    plant_raw = summary.get("plant", {})
    lead_raw = summary.get("lead", {})

    results = {phase: score_phase(run_dir, phase, start_rad) for phase in ("uncompensated", "compensated")}

    title = "needle lead compensation — live hardware"
    if amplitude_mm is not None and frequency_hz is not None:
        title += f" — {amplitude_mm:.1f}mm @ {frequency_hz:.3f}Hz"

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    colors = {"uncompensated": "C3", "compensated": "C2"}
    reference_plotted = False
    for phase in ("uncompensated", "compensated"):
        r = results[phase]
        if "t" not in r:
            continue
        if not reference_plotted:
            axes[0].plot(r["t"], r["reference_mm"], lw=1.2, color="C0", label="reference (input)")
            reference_plotted = True
        axes[0].plot(r["t"], r["measured_mm"], lw=1.0, color=colors[phase],
                     ls="--" if phase == "uncompensated" else "-", label=f"measured ({phase})")
        if np.any(np.isfinite(r["measured_mm"])):
            axes[1].plot(r["t"], r["reference_mm"] - r["measured_mm"], lw=0.8, color=colors[phase],
                         label=f"error ({phase})" + (f", RMSE={r['rmse']:.3f}mm" if "rmse" in r else ""))

    axes[0].set_ylabel("needle extension [mm]")
    axes[0].set_title(title)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].set_ylabel("tracking error [mm]")
    axes[1].set_xlabel("t [s]")
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    out_path = run_dir / "needle_lead_tracking_live.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"saved: {out_path}")

    print(f"\n{'':16s}{'RMSE':>10s}{'amp ratio':>12s}{'lag':>10s}")
    for phase in ("uncompensated", "compensated"):
        r = results[phase]
        if "rmse" in r:
            print(f"{phase:16s}{r['rmse']:>8.4f}mm{r['amplitude_ratio']:>12.3f}{r['lag_s'] * 1000:>8.1f}ms")
        else:
            print(f"{phase:16s}{r.get('error', 'no data')}")

    if "rmse" in results["uncompensated"] and "rmse" in results["compensated"]:
        rmse_u, rmse_c = results["uncompensated"]["rmse"], results["compensated"]["rmse"]
        if rmse_u > 0:
            improvement = (1.0 - rmse_c / rmse_u) * 100.0
            print(f"\nRMSE change with compensator: {improvement:+.1f}%")

    if frequency_hz is not None and plant_raw and lead_raw:
        omega = 2.0 * math.pi * float(frequency_hz)
        plant = PlantModel(**plant_raw)
        lead = LeadCompensator(**lead_raw)
        config = AxisServoConfig(plant=plant, lead=lead)
        predicted_open = abs(plant_response(plant, omega))
        predicted_open_lag_ms = -np.angle(plant_response(plant, omega)) / omega * 1000.0
        predicted_closed = abs(closed_loop_response(config, omega))
        predicted_closed_lag_ms = residual_lag(config, omega) * 1000.0

        print(f"\npredicted from the recorded plant/lead model (plant K={plant.K}, wn={plant.wn}, "
              f"zeta={plant.zeta}; lead zero={lead.zero}, pole={lead.pole}, gain={lead.gain}), "
              f"unity-feedback assumption -- see ct.control.servo:")
        print(f"  open loop:   |G|={predicted_open:.3f}, lag={predicted_open_lag_ms:.1f}ms")
        print(f"  closed loop: |T|={predicted_closed:.3f}, residual_lag={predicted_closed_lag_ms:.1f}ms")
        print("  (real vs. predicted disagreement here is itself informative -- session 019 found "
              "the real control code commands reference+correction, a feedforward+feedback "
              "architecture the unity-feedback formulas above don't model; see docs/sessions/019.)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
