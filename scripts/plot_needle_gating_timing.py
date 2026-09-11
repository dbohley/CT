#!/usr/bin/env python3
"""Diagnostic: gate-fire decision vs. actual needle activation, against breathing motion.

**Why this exists.** The firing gate (``ct.control.gate.FiringGate``) decides *when* to fire
based on a forecast at horizon ``h = tau_s + tau_c + tau_cl(omega_r) + T_ins``. None of those
four terms account for the dead time between "gate fires" and "needle motor actually starts
driving" on the real bench script (``scripts/run_approach_and_seat.py``'s
``reenter_needle_mode()``: ``CLEAR_ERRORS`` + 0.1s sleep, ``ENTER_MODE`` + 0.5s sleep, before the
first drive frame can go out). ``T_ins`` accounts for drive *duration* once moving, not this
pre-drive re-arm overhead -- so the gate may be firing too late relative to what actually
matters, which is the breath position when motion *starts*, not when it finishes.

This script is diagnostic only -- it does not change ``LatencyBudget``, ``FiringGate``, or the
horizon formula. It runs an offline identify -> track -> gate simulation over three signals (a
synthetic sinusoid, and the trailing ``--window-s`` seconds of the Moira and Emma real breathing
recordings) and plots each breathing trace with two families of vertical markers: gate-fire
instants, and gate-fire + ``--rearm-delay-s`` ("actual activation" instants).

**The re-arm delay is a placeholder, not a measured minimum.** No comment anywhere in this
repo's needle bring-up code cites a datasheet spec or bench measurement behind the 0.1s/0.5s
sleeps in ``reenter_needle_mode()`` -- they are defensively-chosen round numbers copied unchanged
into every script that does this handshake. The isolated float -> re-arm -> confirm-fresh-reply
smoke test that would actually measure this has never been run. So ``--rearm-delay-s`` defaults
to 0.6 (the sum of those two sleeps) but is exposed as a flag and labeled everywhere in output as
**unverified** -- re-run with a real number once that smoke test happens.

**The servo/plant ramp-up itself is not modeled or plotted.** The real measured needle plant
(``wn=27 rad/s``) settles in tens of milliseconds -- invisible against a multi-second breath
cycle. The only delay worth marking is the re-arm sequence above.

    python scripts/plot_needle_gating_timing.py --source all
    python scripts/plot_needle_gating_timing.py --source emma --rearm-delay-s 0.15
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ct.control.gate import FiringGate, cycle_extrema
from ct.control.live import SignalAccumulator
from ct.control.servo import LeadServo
from ct.hw.config import AxisServoConfig, LatencyConfig, LeadCompensator, PlantModel
from ct.registry import build_identifier, build_tracker
from ct.rt.latency import LatencyBudget
from ct.run import resolve_tracker_params
from ct.sources.csv_source import CSVSource
from ct.sources.sinusoid import SinusoidSource

REPO_ROOT = Path(__file__).resolve().parent.parent
BREATHE_PROFILES_DIR = REPO_ROOT / "breathe_profiles"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_gating_timing"

# Plant/lead/latency: session 019/021's measured, hardware-validated values -- mirrors
# scripts/run_approach_and_seat.py's NEEDLE_* constants exactly (no shared config module for
# these across analysis scripts, matching this repo's established per-script-constants
# convention -- see that script's own comment on the same point).
NEEDLE_PLANT = PlantModel(K=0.92, wn=27.0, zeta=0.35)
NEEDLE_LEAD = LeadCompensator(zero=1.0, pole=5.0, gain=12.74)
NEEDLE_SERVO_CONFIG = AxisServoConfig(
    plant=NEEDLE_PLANT, lead=NEEDLE_LEAD,
    correction_limit_mm=2.0, correction_rate_limit_mm_s=5.0,
)
NEEDLE_COMMAND_HZ = 200.0  # matches configs/rig_bench.yaml's procedure.loop_rate_hz
NEEDLE_LATENCY_CONFIG = LatencyConfig(tau_s=0.02, tau_c=0.005, T_ins=0.15, tau_cl_fallback=0.05,
                                      measure_tau_c=False)

IDENTIFIER_NAME = "fft_harmonic"
IDENTIFIER_PARAMS = {"Kmax": 6, "energy_threshold": 0.95, "p0_inflation": 2.0}
TRACKER_NAME = "harmonic_ekf"
# omega_bounds_fraction: session 018's real 10-run sweep finding -- a fixed rad/s range can't
# cover subjects at very different breathing rates, but +-10% of THIS run's own Stage-1 rate
# kept frequency lock on all 10 real bench runs where q_scale alone locked only 2/10. Without
# it, real (noisier) subject data can drift omega_r slightly negative and crash
# ct.control.servo.residual_lag's non-negativity check -- confirmed while building this script.
TRACKER_PARAMS = {"joseph": True, "wrap_phases": True, "omega_bounds_fraction": 0.1}

DEFAULT_EXHALE_BAND_FRAC = 0.15
DEFAULT_MAX_FORECAST_STD_MM = 0.5
DEFAULT_CALIB_S = 60.0
DEFAULT_WINDOW_S = 300.0
DEFAULT_REARM_DELAY_S = 0.6  # UNVERIFIED PLACEHOLDER -- see module docstring
SINUSOID_BREATHS_PER_MIN = 15.0
SINUSOID_AMPLITUDE_MM = 3.0  # -> 6mm peak-to-peak, matches generate_sinusoid_profile.py's live default

PROFILES = {
    "moira": BREATHE_PROFILES_DIR / "moira_normal_breathing.csv",
    "emma": BREATHE_PROFILES_DIR / "emma_normal_breathing.csv",
}


def load_signal(source_name: str, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    """(t, y) for the trailing ``window_s`` seconds, rebased so t starts at 0.

    For the real profiles this is the last ``window_s`` seconds of the recording, per this
    script's whole reason for existing -- the start of a real subject take is not
    representative of steady-state breathing, and a longer recording gives more of it to skip.
    """
    if source_name == "sinusoid":
        src = SinusoidSource(fs=120.0, breaths_per_min=SINUSOID_BREATHS_PER_MIN,
                              amplitudes=[SINUSOID_AMPLITUDE_MM])
        batch = src.batch(duration_s=window_s, t0=0.0)
        return batch.t, batch.y

    path = PROFILES[source_name]
    csv_src = CSVSource(path=path, t_column="time_s", y_column="y_mm")
    if csv_src.duration < window_s:
        raise ValueError(
            f"{source_name}: recording is only {csv_src.duration:.1f}s, shorter than the "
            f"requested trailing window of {window_s:.1f}s"
        )
    t0 = csv_src.duration - window_s
    batch = csv_src.batch(duration_s=window_s, t0=t0)
    return batch.t - batch.t[0], batch.y


def run_gate_simulation(t: np.ndarray, y: np.ndarray, calib_s: float, exhale_band_frac: float,
                         max_forecast_std_mm: float, rearm_delay_s: float) -> dict:
    """Identify+seed on the first ``calib_s`` seconds, then track+gate the remainder.

    Mirrors the exact identify -> seed -> step pattern in
    scripts/run_approach_and_seat.py (itself matching
    ct.control.context.ProcedureContext.identify_and_start_tracker/_feed_estimator).
    """
    calib_mask = t < calib_s
    if calib_mask.sum() < 4:
        raise ValueError(f"only {calib_mask.sum()} samples in the {calib_s:.1f}s calibration "
                          f"window -- widen --calib-s or shorten --window-s")

    accumulator = SignalAccumulator()
    for ti, yi in zip(t[calib_mask], y[calib_mask]):
        accumulator.offer(float(ti), float(yi))

    batch = accumulator.to_batch()
    t_end = accumulator.t_last
    ident_result = build_identifier(IDENTIFIER_NAME, IDENTIFIER_PARAMS).identify(batch)
    tracker_params = resolve_tracker_params(TRACKER_PARAMS, ident_result.diagnostics["bpm_hat"])
    tracker = build_tracker(TRACKER_NAME, tracker_params)
    tracker.init(ident_result, t0=t_end)

    servo = LeadServo(NEEDLE_SERVO_CONFIG, Ts=1.0 / NEEDLE_COMMAND_HZ)
    latency = LatencyBudget(NEEDLE_LATENCY_CONFIG, tau_cl=servo.residual_lag)
    gate = FiringGate(exhale_band_frac=exhale_band_frac, max_forecast_std=max_forecast_std_mm)

    fire_events: list[dict] = []
    track_mask = ~calib_mask
    omega_r_history: list[float] = []
    nis_history: list[float] = []

    for ti, yi in zip(t[track_mask], y[track_mask]):
        step = tracker.step(float(ti), float(yi))
        nis_history.append(float(step.nis))
        s, _P = tracker.state
        omega_r = float(s[tracker.layout.omega])
        omega_r_history.append(omega_r)

        h = latency.horizon(omega_r)
        extrema = cycle_extrema(s, tracker.layout)
        decision = gate.evaluate(float(ti), tracker, h, extrema)
        if decision.fire:
            activation_t = float(ti) + rearm_delay_s
            # Did the delay push activation outside the window the gate fired into? Approximate
            # the breath value at activation time from the nearest actual sample (the real
            # observable), not a further model extrapolation on top of the forecast already used
            # to fire.
            idx = int(np.argmin(np.abs(t - activation_t)))
            y_at_activation = float(y[idx]) if idx < t.size else None
            escapes_band = (
                y_at_activation is not None and y_at_activation > decision.band_top
            )
            fire_events.append({
                "fire_t": float(ti),
                "activation_t": activation_t,
                "band_top": decision.band_top,
                "y_min": decision.y_min,
                "y_max": decision.y_max,
                "forecast": decision.forecast,
                "y_at_activation": y_at_activation,
                "escapes_band": bool(escapes_band),
                "escape_mm": (
                    float(y_at_activation - decision.band_top) if escapes_band else 0.0
                ),
            })

    return {
        "calib_end_t": float(calib_s),
        "fire_events": fire_events,
        "gate_stats": gate.stats,
        "omega_r_mean": float(np.mean(omega_r_history)) if omega_r_history else None,
        "omega_r_std": float(np.std(omega_r_history)) if omega_r_history else None,
        "nis_mean": float(np.mean(nis_history)) if nis_history else None,
    }


def plot_source(source_name: str, t: np.ndarray, y: np.ndarray, sim: dict, out_dir: Path,
                 rearm_delay_s: float) -> Path:
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(t, y, color="C0", lw=0.8, label="breathing signal")

    ax.axvline(sim["calib_end_t"], color="0.25", lw=1.2, ls=(0, (6, 3)), alpha=0.8,
               label="calib -> track")

    fire_label_used = False
    activation_label_used = False
    for event in sim["fire_events"]:
        ax.axvline(event["fire_t"], color="C2", lw=1.1, ls="--", alpha=0.7,
                   label="_nolegend_" if fire_label_used else "gate fires (decision)")
        fire_label_used = True
        ax.axvline(event["activation_t"], color="C3", lw=1.1, ls=":", alpha=0.8,
                   label="_nolegend_" if activation_label_used else
                   f"needle activates (+{rearm_delay_s:.2f}s, UNVERIFIED placeholder)")
        activation_label_used = True

    n_escapes = sum(1 for e in sim["fire_events"] if e["escapes_band"])
    ax.set_title(f"{source_name}: {len(sim['fire_events'])} firings, {n_escapes} escape the "
                 f"exhale band by activation time")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("y (mm)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    out_path = out_dir / f"{source_name}.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return out_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", choices=["sinusoid", "moira", "emma", "all"], default="all")
    parser.add_argument("--calib-s", type=float, default=DEFAULT_CALIB_S, dest="calib_s")
    parser.add_argument("--window-s", type=float, default=DEFAULT_WINDOW_S, dest="window_s")
    parser.add_argument("--rearm-delay-s", type=float, default=DEFAULT_REARM_DELAY_S,
                         dest="rearm_delay_s",
                         help="UNVERIFIED PLACEHOLDER -- sum of reenter_needle_mode()'s two "
                              "sleep() calls in run_approach_and_seat.py, not a bench-measured "
                              "minimum. See module docstring.")
    parser.add_argument("--exhale-band-frac", type=float, default=DEFAULT_EXHALE_BAND_FRAC,
                         dest="exhale_band_frac")
    parser.add_argument("--max-forecast-std-mm", type=float, default=DEFAULT_MAX_FORECAST_STD_MM,
                         dest="max_forecast_std_mm")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    sources = ["sinusoid", "moira", "emma"] if args.source == "all" else [args.source]
    args.out.mkdir(parents=True, exist_ok=True)

    print(f"rearm_delay_s={args.rearm_delay_s:.3f}s -- UNVERIFIED PLACEHOLDER, see module "
          f"docstring; not a bench-measured minimum")

    summary: dict = {"rearm_delay_s": args.rearm_delay_s, "rearm_delay_verified": False,
                      "calib_s": args.calib_s, "window_s": args.window_s, "sources": {}}

    for source_name in sources:
        print(f"\n=== {source_name} ===")
        t, y = load_signal(source_name, args.window_s)
        sim = run_gate_simulation(t, y, args.calib_s, args.exhale_band_frac,
                                   args.max_forecast_std_mm, args.rearm_delay_s)

        print(f"  tracked omega_r: mean={sim['omega_r_mean']:.4f} rad/s, "
              f"std={sim['omega_r_std']:.4f} rad/s (large std relative to mean suggests lost "
              f"frequency lock -- check before trusting the firings below)")
        print(f"  mean NIS: {sim['nis_mean']:.4f} (expected ~1.0 if well-calibrated)")

        n_fire = len(sim["fire_events"])
        n_escape = sum(1 for e in sim["fire_events"] if e["escapes_band"])
        print(f"  gate fired {n_fire} times; {n_escape} activation(s) land outside the exhale "
              f"band the gate fired into")
        if n_escape:
            worst = max(e["escape_mm"] for e in sim["fire_events"])
            print(f"  worst escape: {worst:.4f}mm past band_top")

        out_path = plot_source(source_name, t, y, sim, args.out, args.rearm_delay_s)
        print(f"  saved {out_path}")

        summary["sources"][source_name] = {
            "n_firings": n_fire,
            "n_escapes": n_escape,
            "omega_r_mean": sim["omega_r_mean"],
            "omega_r_std": sim["omega_r_std"],
            "nis_mean": sim["nis_mean"],
            "gate_stats": sim["gate_stats"],
            "fire_events": sim["fire_events"],
        }

    summary_path = args.out / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"\nsaved {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
