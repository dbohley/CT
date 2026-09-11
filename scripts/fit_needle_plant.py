#!/usr/bin/env python3
"""Fit `PlantModel(K, wn, zeta)` (src/ct/hw/config.py) from one or more
scripts/run_needle_step_response.py runs.

**What is fit against what.** `plant_response()` in src/ct/control/servo.py is explicit that
`wn`/`zeta` describe the axis's mechanical resonance on the *velocity* channel -- the model is
type-1 (`K*wn^2 / (s*(s^2+2*zeta*wn*s+wn^2))`), so position is velocity's free integral and does
not carry the classic underdamped step-response landmarks the way velocity does. The fit target
is therefore a velocity signal -- but which one required a real run to settle, not just theory:

**The reply's own `motor_velocity_rad_s` is not used as the fit target.** The first real run
(2026-09-05, `outputs/needle_step_response/20260905-151246`) showed it reading a near-constant
~1.0-1.1 rad/s throughout a commanded 0.15 rad/s step -- 6.7-7x too high, and essentially
uncorrelated (r=-0.06) with the differentiated position. `scripts/listen_needle_motor.py` had
already flagged why: "the manual is internally inconsistent about units ('rad/s' in one
sentence, 'r/s' in another for the same field) -- treat decoded velocity as provisional." This
is that provisional-ness becoming concrete: rescaling the reply's decoded value by 30/200
(the ratio between this codec's GL-II-manual range, `MOTOR_REPLY_V_MAX=200`, and the different,
MIT-mode-GUI-confirmed range `read_needle_position.py` uses, `V_MAX=30`) reproduces the
commanded velocity almost exactly (103%) -- suggestive of a units mix-up, but not yet nailed
down precisely (30/200=6.67 vs the 2*pi=6.28 a rad/s-vs-revolutions/s mixup would predict; one
run is not enough to tell which, if either, is the whole story). **Numerically differentiated
position is used instead** -- it needed no rescaling to land within 93% of the commanded
velocity limit, is independently corroborated by every past session that has trusted position
(dead-reckoning fixes, lag measurements), and does not depend on resolving the reply-field
question first. The raw reply velocity is still logged and reported as a diagnostic (mean
magnitude and correlation against differentiated position), because a future run pinning down
its real scale would be a second, independent estimate worth having.

Since `commanded_velocity_limit_rad_s` is a speed *magnitude* (the GL-II protocol takes a speed
limit and infers direction from position error, not a signed velocity command), the fit works
in magnitudes throughout: `abs()` of the differentiated velocity against `V`.

**Numerical fit, not closed-form.** Percent-overshoot / log-decrement formulas need a clearly
resolved overshoot peak and only work in the underdamped regime -- the real axis's damping is
unknown (the placeholder zeta=0.7 is barely underdamped; a real cable/capstan-driven axis could
easily be over-damped, where those formulas don't apply at all). Instead this simulates the
candidate model's own step response via scipy.signal and fits (K, wn, zeta) to the measured
velocity trace with scipy.optimize.curve_fit -- one code path that covers any damping regime,
plus a directly reportable residual RMS instead of hand-identified landmark points. The
simulated response is generated on a uniform internal grid and interpolated onto the actual
(wall-clock-jittered, non-uniform) sample times, since scipy.signal.step's lsim backend
requires uniform spacing and real timestamps never are.

Each (velocity, rep) is fit independently first, for a repeatability/linearity check: a
material drift in K, wn or zeta across step sizes is itself a finding (the linear second-order
model may only be a local approximation), not just noise to average away. A pooled fit
(all steps and reps concatenated, each rescaled by its own commanded velocity limit) is then
reported as the recommended single result.

    python scripts/fit_needle_plant.py --run outputs/needle_step_response/20260905-120000
    python scripts/fit_needle_plant.py --run outputs/needle_step_response/20260905-120000 \\
        outputs/needle_step_response/20260905-130000
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import signal
from scipy.optimize import curve_fit

from ct.cli._common import save_json
from ct.rt.telemetry import load_jsonl, to_arrays

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "needle_step_response"
DRUM_RADIUS_M = 0.018  # keep in sync with run_needle_step_response.py

# PlantModel.__post_init__ requires wn > 0, zeta > 0 (src/ct/hw/config.py) -- fit bounds mirror that.
K_BOUNDS = (0.01, 10.0)
WN_BOUNDS = (0.1, 500.0)
ZETA_BOUNDS = (0.01, 5.0)


def _latest_run_dir() -> Path:
    runs = sorted(p for p in DEFAULT_RUNS_DIR.iterdir() if p.is_dir()) if DEFAULT_RUNS_DIR.exists() else []
    if not runs:
        raise FileNotFoundError(f"no runs found under {DEFAULT_RUNS_DIR}")
    return runs[-1]


def resolve_run(run_arg: str) -> Path:
    path = Path(run_arg)
    if path.is_file():
        return path.parent
    return path


def step_velocity_response(t: np.ndarray, K: float, wn: float, zeta: float, V: float) -> np.ndarray:
    """Simulated velocity response of the type-1 plant's second-order block to a velocity-limit
    step of size V, i.e. K*wn^2/(s^2+2*zeta*wn*s+wn^2) driven by a step of height V.

    `t` is real wall-clock-derived elapsed time, not guaranteed uniformly spaced --
    `scipy.signal.step`'s `lsim` backend requires a uniform grid, so the response is
    simulated on one internally and interpolated back onto the requested (possibly ragged)
    `t`, dense enough (4x the requested sample count, floored at 500) to resolve a transient
    much faster than the sample spacing without the interpolation itself smearing it.
    """
    system = signal.TransferFunction([K * wn**2], [1.0, 2.0 * zeta * wn, wn**2])
    n_sim = max(500, 4 * len(t))
    t_uniform = np.linspace(0.0, float(t.max()), n_sim)
    _, y_uniform = signal.step(system, T=t_uniform)
    return V * np.interp(t, t_uniform, y_uniform)


def fit_one_step(t: np.ndarray, measured_velocity: np.ndarray, V: float) -> dict:
    """Fit (K, wn, zeta) to one step's velocity transient. Returns the fit plus residual RMS.

    ``measured_velocity`` is taken as a magnitude (``abs()``'d here) because ``V`` is the
    commanded speed *limit*, not a signed velocity -- the GL-II position/velocity protocol
    infers direction from position error, so the sign of the real measured velocity depends on
    which way this particular step happened to move, not on anything the model should fit.
    """
    measured_velocity = np.abs(measured_velocity)
    valid = np.isfinite(measured_velocity)
    t, measured_velocity = t[valid], measured_velocity[valid]
    if t.size < 5:
        return {"K": float("nan"), "wn": float("nan"), "zeta": float("nan"),
                "residual_rms": float("nan"), "n": int(t.size)}

    def model(t_: np.ndarray, K: float, wn: float, zeta: float) -> np.ndarray:
        return step_velocity_response(t_, K, wn, zeta, V)

    # Initial guess: K from the settled (last 10%) measured velocity over V; wn/zeta from the
    # code defaults -- reasonable starting points, not claims about the real axis.
    tail = measured_velocity[int(0.9 * len(measured_velocity)):]
    K0 = float(np.mean(tail) / V) if V != 0 else 1.0
    K0 = min(max(K0, K_BOUNDS[0]), K_BOUNDS[1])
    p0 = [K0, 60.0, 0.7]
    bounds = ([K_BOUNDS[0], WN_BOUNDS[0], ZETA_BOUNDS[0]], [K_BOUNDS[1], WN_BOUNDS[1], ZETA_BOUNDS[1]])

    try:
        popt, _ = curve_fit(model, t, measured_velocity, p0=p0, bounds=bounds, maxfev=10000)
    except RuntimeError as exc:
        return {"K": float("nan"), "wn": float("nan"), "zeta": float("nan"),
                "residual_rms": float("nan"), "n": int(t.size), "error": str(exc)}

    K, wn, zeta = (float(v) for v in popt)
    residual = model(t, K, wn, zeta) - measured_velocity
    return {"K": K, "wn": wn, "zeta": zeta, "residual_rms": float(np.sqrt(np.mean(residual**2))), "n": int(t.size)}


def load_run_steps(run_dir: Path) -> list[dict]:
    """One entry per (velocity, rep) step: t (reset to step-phase start), the fit target
    (position numerically differentiated into rad/s -- see module docstring for why this is
    used instead of the reply's own motor_velocity_rad_s), the commanded velocity limit V, and
    the raw reply velocity kept alongside purely as a diagnostic.

    Also carries `started_from_rest`: whether the retract phase immediately before this step
    actually measured back to start_rad in time (`summary.json`'s `step_records[i]
    ["retract_arrived"]`, from run_needle_step_response.py -- absent in runs recorded before
    2026-09-05's fix, in which case this is `None`, meaning unknown rather than clean). A step
    whose predecessor's retract didn't arrive started with real leftover velocity in the wrong
    direction, not from rest -- 2026-09-05's v=0.30 run fit `wn=300, zeta=0.026` from exactly
    this, and main() excludes any such step from the aggregate statistics by default.
    """
    jsonl_path = run_dir / "samples.jsonl"
    if not jsonl_path.exists():
        raise FileNotFoundError(f"no samples.jsonl found at {jsonl_path}")
    records = load_jsonl(jsonl_path)

    summary_path = run_dir / "summary.json"
    retract_arrived_by_index: dict[int, bool] = {}
    if summary_path.exists():
        summary = json.loads(summary_path.read_text())
        for rec in summary.get("step_records", []):
            if "retract_arrived" in rec:
                retract_arrived_by_index[int(rec["step_index"])] = bool(rec["retract_arrived"])

    steps = []
    step_indices = sorted({int(r["step_index"]) for r in records if r.get("step_index") is not None})
    first_index = min(step_indices) if step_indices else 0
    for step_index in step_indices:
        if step_index == first_index:
            started_from_rest = True  # the very first step of a run has no preceding retract
        else:
            started_from_rest = retract_arrived_by_index.get(step_index - 1)  # None = unknown (legacy run)
        step_records = [r for r in records if r.get("step_index") == step_index and r.get("phase") == "step"]
        if not step_records:
            continue
        cols = to_arrays(step_records, ["elapsed", "commanded_velocity_limit_rad_s",
                                         "motor_velocity_rad_s", "motor_position_mm"])
        V = float(cols["commanded_velocity_limit_rad_s"][0])
        if cols["elapsed"].size > 1:
            diff_velocity_mm_s = np.gradient(cols["motor_position_mm"], cols["elapsed"])
        else:
            diff_velocity_mm_s = np.full_like(cols["elapsed"], np.nan)
        diff_velocity_rad_s = (diff_velocity_mm_s / 1000.0) / DRUM_RADIUS_M
        steps.append({
            "step_index": step_index,
            "run_dir": str(run_dir),
            "t": cols["elapsed"],
            "V": V,
            "measured_velocity": diff_velocity_rad_s,
            "raw_reply_velocity": cols["motor_velocity_rad_s"],
            "started_from_rest": started_from_rest,
        })
    return steps


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", nargs="+", default=None,
                         help="one or more run directories; default: most recent under "
                              "outputs/needle_step_response/")
    parser.add_argument("--include-tainted", action="store_true", dest="include_tainted",
                         help="include steps whose preceding retract did not finish in time "
                              "(started_from_rest=False) in the aggregate stats and pooled fit "
                              "anyway -- off by default, since such a step's transient reflects "
                              "a mid-retract reversal, not a from-rest step response")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_args = args.run or [str(_latest_run_dir())]
    run_dirs = [resolve_run(r) for r in run_args]

    all_steps = []
    for run_dir in run_dirs:
        all_steps.extend(load_run_steps(run_dir))

    if not all_steps:
        print("error: no steps found in the given run(s)")
        return 1

    print(f"loaded {len(all_steps)} step(s) from {len(run_dirs)} run(s)\n")

    per_step_fits = []
    for step in all_steps:
        fit = fit_one_step(step["t"], step["measured_velocity"], step["V"])
        fit["step_index"], fit["run_dir"], fit["V"] = step["step_index"], step["run_dir"], step["V"]
        fit["started_from_rest"] = step["started_from_rest"]
        per_step_fits.append(fit)

        note = f" ({fit['error']})" if "error" in fit else ""
        rest_flag = {True: "", False: "  [DID NOT START FROM REST -- excluded from stats, see --include-tainted]",
                     None: "  [unknown whether it started from rest -- pre-fix run]"}[step["started_from_rest"]]
        print(f"  step {fit['step_index']:>2} (run={Path(fit['run_dir']).name}, V={fit['V']:.3f}rad/s): "
              f"K={fit['K']:.3f} wn={fit['wn']:.2f} zeta={fit['zeta']:.3f} "
              f"residual_rms={fit['residual_rms']:.4f}{note}{rest_flag}")

        diff_vel = step["measured_velocity"]  # the fit target: |d(position)/dt|, rad/s
        raw_vel = np.abs(step["raw_reply_velocity"])
        valid = np.isfinite(diff_vel) & np.isfinite(raw_vel)
        if int(np.sum(valid)) >= 5:
            corr = float(np.corrcoef(diff_vel[valid], raw_vel[valid])[0, 1])
            ratio = float(np.mean(raw_vel[valid]) / max(np.mean(diff_vel[valid]), 1e-9))
            print(f"           diagnostic: raw reply's motor_velocity_rad_s vs |d(position)/dt| "
                  f"(the fit target): corr={corr:.3f}, mean ratio={ratio:.2f}x "
                  f"({'consistent, both usable' if 0.8 < ratio < 1.25 else 'diverges -- see this script docstring'})")

    tainted = [f for f in per_step_fits if f["started_from_rest"] is False]
    if tainted and not args.include_tainted:
        print(f"\nexcluding {len(tainted)} step(s) that did not start from rest "
              f"(steps {[f['step_index'] for f in tainted]}) -- pass --include-tainted to "
              "override.")

    def usable(f: dict) -> bool:
        return np.isfinite(f["K"]) and (args.include_tainted or f["started_from_rest"] is not False)

    valid_fits = [f for f in per_step_fits if usable(f)]
    if not valid_fits:
        print("\nerror: no step produced a usable fit")
        return 1

    for key in ("K", "wn", "zeta"):
        values = np.array([f[key] for f in valid_fits])
        print(f"\n{key}: mean={values.mean():.4f} std={values.std():.4f} "
              f"(n={len(values)}, range {values.min():.4f}-{values.max():.4f})")
        if values.std() / max(abs(values.mean()), 1e-9) > 0.25:
            print(f"  -- {key} varies materially across steps (std/mean > 25%); the linear "
                  "second-order model may only be a local approximation. Worth a session-doc callout.")

    # Pooled fit: concatenate every usable step's (t, measured_velocity/V) as if it were one
    # step at unit commanded velocity, so a single (K, wn, zeta) is fit across all data at once.
    # Same started_from_rest filter as the per-step aggregate, for the same reason.
    pooled_steps = [s for s in all_steps if args.include_tainted or s["started_from_rest"] is not False]
    t_pool = np.concatenate([s["t"] for s in pooled_steps])
    v_pool = np.concatenate([s["measured_velocity"] / s["V"] for s in pooled_steps])
    pooled = fit_one_step(t_pool, v_pool, V=1.0)

    print(f"\npooled fit (all steps concatenated, rescaled by each step's own V): "
          f"K={pooled['K']:.4f} wn={pooled['wn']:.2f} zeta={pooled['zeta']:.4f} "
          f"residual_rms={pooled['residual_rms']:.4f}")

    result = {
        "per_step": per_step_fits,
        "pooled": pooled,
        "run_dirs": [str(d) for d in run_dirs],
        "n_steps": len(all_steps),
    }
    out_path = save_json(result, run_dirs[0] / "plant_fit.json")
    print(f"\nsaved: {out_path}")
    print("\nnext: put the pooled (or per-step, if pooling looks wrong) K/wn/zeta into "
          "configs/rig_bench.yaml's servo.needle.plant, then re-derive the lead compensator "
          "against it rather than hand-tuning (src/ct/control/servo.py has the tested "
          "plant_response/lead_response/closed_loop_response/bandwidth/residual_lag functions "
          "to design and verify against).")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
