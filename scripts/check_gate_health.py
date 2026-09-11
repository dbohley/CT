#!/usr/bin/env python3
"""Replay a recorded bench run's EKF and firing gate offline, and say whether they were healthy.

Answers the question that cost session 023 three hardware runs: *would the gate have fired, and
if not, why not?* -- in a few seconds, from a log, without the rig.

It imports the identifier/tracker/gate constants from ``scripts/run_approach_and_seat.py`` itself
rather than restating them, so it cannot drift from what actually ships. Point it at any run
directory containing ``samples.jsonl`` with a ``standoff_hold`` phase:

    python scripts/check_gate_health.py outputs/needle_gating_live/moira/2
    python scripts/check_gate_health.py outputs/needle_gating_live/*/*

Two things are checked, and the first is the one that matters:

**theta/omega** -- how fast the tracked phase really advanced, as a fraction of the omega the
filter reports. The process model *is* ``theta += omega*Ts``, so a healthy filter sits at ~1.0.
The EKF can instead pin theta on the steep part of the sine and fit the breath by wiggling it
(``dh/dtheta = A_k*cos(...)``, so a larger ``A_k`` needs a smaller wiggle -- self-reinforcing).
Run ``moira/2`` at ``q_scale=0.2`` measured **0.004** with ``A_1`` diverging 1.04 -> 8.47mm, and
none of it was visible downstream: the model still tracked the sensor to 0.125mm RMSE. What broke
was the gate, because ``cycle_extrema`` sweeps theta a full cycle and so reported a ~16mm band for
a ~2mm waveform, putting end-exhale somewhere the model never goes -- 50355 evaluations, 1 firing,
no fault, a silent deadlock.

**fires / cadence** -- the gate should fire about once per breath (its refractory is 0.9 breaths).
Zero firings over a long run, or a cadence far from the period, means the band is unreachable.

A run that predates a fix will fail here; that is the point. Re-run it after changing a constant
to see whether the change would have helped, before spending a phantom run finding out.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import pathlib

import numpy as np

from ct.control.gate import FiringGate, cycle_extrema
from ct.control.live import SignalAccumulator
from ct.registry import build_identifier, build_tracker
from ct.run import resolve_tracker_params

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_H = 0.33   # a representative horizon; the health checks below are not sensitive to it


def _bench_module():
    """The bench script's own constants, so this check cannot drift from what ships."""
    path = REPO_ROOT / "scripts" / "run_approach_and_seat.py"
    spec = importlib.util.spec_from_file_location("_ras", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_samples(run_dir: pathlib.Path) -> list[dict]:
    """Tolerates a torn final line, so a run still in progress can be checked."""
    out = []
    for line in (run_dir / "samples.jsonl").read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def check(run_dir: pathlib.Path, ras, horizon: float) -> dict | None:
    recs = load_samples(run_dir)
    accumulator = SignalAccumulator()
    for r in recs:
        if r["phase"] == "standoff_hold":
            accumulator.offer(r["t"], r["tactile_mm"])
    if len(accumulator) < 4:
        return None

    ident = build_identifier(ras.IDENTIFIER_NAME, ras.IDENTIFIER_PARAMS).identify(
        accumulator.to_batch())
    tracker = build_tracker(
        ras.TRACKER_NAME,
        resolve_tracker_params(ras.TRACKER_PARAMS, float(ident.diagnostics["bpm_hat"])),
    )
    tracker.init(ident, t0=accumulator.t_last)
    layout = tracker.layout
    gate = FiringGate(exhale_band_frac=ras.DEFAULT_EXHALE_BAND_FRAC,
                      max_forecast_std=ras.DEFAULT_MAX_FORECAST_STD_MM)

    fires: list[float] = []
    previous_theta = None
    advance = 0.0
    t_first = t_last = None
    for r in recs:
        if r["t"] <= accumulator.t_last:
            continue
        tracker.step(float(r["t"]), float(r["tactile_mm"]))
        s, _P = tracker.state
        theta = float(s[layout.theta])
        if previous_theta is not None:
            advance += (theta - previous_theta + np.pi) % (2.0 * np.pi) - np.pi
        previous_theta = theta
        if t_first is None:
            t_first = r["elapsed"]
        t_last = r["elapsed"]
        if gate.evaluate(r["elapsed"], tracker, horizon, cycle_extrema(s, layout)).fire:
            fires.append(r["elapsed"])

    if t_first is None or t_last <= t_first:
        return None
    s, _P = tracker.state
    omega = float(s[layout.omega])
    gaps = np.diff(fires) if len(fires) > 1 else np.array([])
    return {
        "K": ident.K,
        "R": ident.R,
        "A_1": float(s[layout.A(1)]),
        "period_s": 2.0 * np.pi / omega if omega > 0 else float("nan"),
        "theta_over_omega": (advance / (t_last - t_first)) / omega if omega > 0 else float("nan"),
        "fires": len(fires),
        "cadence_s": float(gaps.mean()) if len(gaps) else float("nan"),
        "tracked_s": t_last - t_first,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=pathlib.Path,
                        help="run directories containing samples.jsonl")
    parser.add_argument("--horizon", type=float, default=DEFAULT_H)
    args = parser.parse_args(argv)

    ras = _bench_module()
    print(f"q_scale={ras.IDENTIFIER_PARAMS['q_scale']}  "
          f"exhale_band_frac={ras.DEFAULT_EXHALE_BAND_FRAC}  "
          f"max_forecast_std={ras.DEFAULT_MAX_FORECAST_STD_MM}  "
          f"omega_bounds_fraction={ras.TRACKER_PARAMS.get('omega_bounds_fraction')}\n")
    print(f"{'run':26s} {'K':>2} {'A_1':>7} {'theta/omega':>12} {'fires':>6} "
          f"{'cadence':>9} {'period':>8} {'verdict':>9}")

    worst = 0
    for run_dir in args.runs:
        name = "/".join(run_dir.parts[-2:])
        if not (run_dir / "samples.jsonl").exists():
            print(f"{name:26s} no samples.jsonl")
            continue
        result = check(run_dir, ras, args.horizon)
        if result is None:
            print(f"{name:26s} never reached standoff_hold -- nothing to replay")
            continue
        phase_ok = result["theta_over_omega"] >= ras.PHASE_ADVANCE_MIN_FRAC
        cadence_ok = (result["fires"] > 1
                      and 0.8 * result["period_s"] <= result["cadence_s"] <= 1.5 * result["period_s"])
        verdict = "OK" if (phase_ok and cadence_ok) else ("DEGENERATE" if not phase_ok else "NO FIRE")
        worst = max(worst, 0 if verdict == "OK" else 1)
        print(f"{name:26s} {result['K']:>2} {result['A_1']:7.2f} "
              f"{result['theta_over_omega']:12.3f} {result['fires']:6d} "
              f"{result['cadence_s']:8.2f}s {result['period_s']:7.2f}s {verdict:>9}")

    if worst:
        print("\nDEGENERATE: theta stopped advancing -- the filter is wiggling it instead of "
              "sweeping it, so A_k diverges and the gate's band becomes meaningless. Lower "
              "--q-scale.\nNO FIRE: the phase is healthy but the gate still would not fire at "
              "about one bite per breath -- check --exhale-band-frac and --max-forecast-std-mm.")
    return worst


if __name__ == "__main__":
    raise SystemExit(main())
