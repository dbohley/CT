#!/usr/bin/env python3
"""Write synthetic breathing profiles that each carry one disturbance, for measuring how long the
estimator takes to notice it.

Four profiles, in the ``time_s,y_mm`` format ``scripts/run_breathing_profile.py`` reads (the same
format as the real recordings in ``breathe_profiles/``; ``ct-generate`` writes ``t,y`` instead and
the driver cannot read it):

    amplitude_step     2.0mm -> 3.0mm            at the onset, rate held at 15 bpm
    frequency_step     15 bpm -> 20 bpm          at the onset, amplitude held at 2.0mm
    frequency_wander   rate random-walks about 15 bpm after the onset, amplitude held
    both_step          2.0mm -> 3.0mm AND 15 -> 20 bpm at the same instant

Each is written alongside a ``<name>.onset.json`` sidecar recording the exact instant the change
took effect, the parameters either side of it, and the profile's peak-to-peak travel.

**Everything is built on a phase accumulator** -- ``theta += omega(t)*dt`` and ``y = A(t)*sin(theta)``
-- rather than by concatenating separately-generated segments. A rate change then leaves *position*
continuous and only its derivative steps, which is what a breathing-rate change physically is. A
concatenation would put a position discontinuity into the file, and the phantom would answer it
with a motor jerk that is not a breathing disturbance at all and would swamp what we are trying to
measure.

**The amplitude change is applied at a zero crossing** of ``sin(theta)`` at or after the nominal
onset, for the same reason: ``A*sin(theta)`` only changes continuously with ``A`` where
``sin(theta)`` is zero. The sidecar records the crossing that was actually used, not the nominal
time that was asked for.

**Why the baseline is as long as it is.** The disturbance has to land after the rig has finished
approach, seat, standoff and its calibration recording, because that calibration window has to be
clean breathing -- it is the distribution everything downstream is judged "out of". Approach
through standoff has taken 100-137s on every real run and cannot be hurried; add the 60s
recording and the estimator is not seeded until ~160-200s. 240s clears that with margin to spare.

The run then stops ``--monitor-after-s`` (30s) after the disturbance rather than watching for a
fixed several minutes, so a whole run is ~4.5 minutes end to end, of which the part that actually
measures anything is the last ~90s. Total profile duration is 300s: longer than any run, so the
driver's ``--loop`` never wraps and there is exactly one onset.

**Pass ``--max-travel-mm 6.5`` when playing these.** ``run_breathing_profile.py`` REFUSES to play a
file whose whole-file peak-to-peak exceeds the clamp, and the disturbed segment of the amplitude
profiles is exactly 6.00mm against a 6.0 default -- close enough that a float hair decides it, and
it would refuse only after the rig had already approached and seated.

    python scripts/generate_disturbance_profiles.py
    python scripts/run_approach_and_seat.py --monitor-s 240 --max-travel-mm 6.5 \
        --profile outputs/disturbance_detection/profiles/amplitude_step.csv \
        --out outputs/disturbance_detection/amplitude_step/1
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "disturbance_detection" / "profiles"

DEFAULT_FS = 120.0            # matches breathe_profiles/*.csv's real sample rate
# The disturbance has to land after the rig is seated and calibrated, and the rig cannot be
# hurried: approach+seat+standoff has taken 100-137s on every real run, and the calibration
# recording is 60s on top. 240s clears the worst of those with ~40s to spare. Earlier than this
# and a slow approach makes the run miss its own disturbance; later is just dead time.
DEFAULT_BASELINE_S = 240.0
# Only 30s of disturbed data is actually used (--monitor-after-s), so 60s is already generous.
# This was 600s, which made every run ~7 minutes of which 4 were spent watching nothing happen.
DEFAULT_DURATION_S = 300.0
DEFAULT_AMPLITUDE_MM = 2.0    # 4mm peak-to-peak baseline
DEFAULT_AMPLITUDE_STEP_MM = 3.0   # 6mm peak-to-peak -- see the --max-travel-mm note above
DEFAULT_BPM = 15.0
DEFAULT_BPM_STEP = 20.0
DEFAULT_WANDER_BPM = 4.0      # bound on the random walk's excursion from baseline
DEFAULT_WANDER_TAU_S = 8.0    # how fast the walk decorrelates; ~2 breaths
DEFAULT_SEED = 20260910


def _wander_bpm(n: int, dt: float, base_bpm: float, bound_bpm: float, tau_s: float,
                rng: np.random.Generator) -> np.ndarray:
    """A bounded Ornstein-Uhlenbeck walk about ``base_bpm``.

    Bounded rather than a free random walk so the phantom's excursion and velocity stay
    predictable -- an unbounded walk can drift to a rate the motor cannot track, which would make
    the experiment measure the motor rather than the estimator.
    """
    theta = dt / max(tau_s, 1e-6)
    sigma = bound_bpm / 2.0                      # ~2 sigma inside the bound
    out = np.empty(n)
    x = 0.0
    for i in range(n):
        x += -theta * x + np.sqrt(2.0 * theta) * sigma * rng.standard_normal()
        out[i] = base_bpm + float(np.clip(x, -bound_bpm, bound_bpm))
    return out


def build_profile(kind: str, args, rng: np.random.Generator) -> tuple[pd.DataFrame, dict]:
    """One profile plus its onset sidecar, integrated as a phase accumulator."""
    dt = 1.0 / args.fs
    n = int(round(args.duration_s * args.fs))
    t = np.arange(n) * dt

    # Pass 1: the undisturbed phase, purely to locate the instant everything will change at.
    # Every profile changes at the same kind of instant -- the first upward zero crossing at or
    # after the nominal onset -- so that each has exactly ONE onset, and the four are directly
    # comparable. That matters: an earlier version let the rate change at the nominal time while
    # the amplitude waited for the crossing, which gave `both_step` two onsets 3s apart, and the
    # offline control duly "detected" the disturbance 2.89s BEFORE the sidecar's onset.
    omega_base = args.bpm * 2.0 * np.pi / 60.0
    theta_base = np.arange(n) * omega_base * dt
    sin_base = np.sin(theta_base)
    idx = np.arange(n)
    crossings = idx[:-1][(sin_base[:-1] <= 0) & (sin_base[1:] > 0)]
    candidates = crossings[crossings >= int(round(args.baseline_s * args.fs))]
    if len(candidates) == 0:
        raise ValueError(f"{kind}: no zero crossing after {args.baseline_s}s to step at")
    step_i = int(candidates[0]) + 1
    onset_s = float(t[step_i])

    # Pass 2: apply every change at that one index.
    bpm = np.full(n, args.bpm)
    if kind in ("frequency_step", "both_step"):
        bpm[step_i:] = args.bpm_step
    elif kind == "frequency_wander":
        bpm[step_i:] = _wander_bpm(n - step_i, dt, args.bpm, args.wander_bpm,
                                    args.wander_tau_s, rng)

    # Integrate the rate into a phase. This is what keeps position continuous across a rate
    # change -- only its derivative steps, which is what a breathing-rate change physically is.
    omega = bpm * 2.0 * np.pi / 60.0
    theta = np.cumsum(omega) * dt
    sin_theta = np.sin(theta)

    amplitude = np.full(n, args.amplitude_mm)
    if kind in ("amplitude_step", "both_step"):
        # sin(theta) is ~0 here by construction, and A*sin(theta) is only continuous in A where
        # sin(theta) == 0 -- so this is the one place the amplitude can change without stepping
        # the commanded position and making the phantom jerk.
        amplitude[step_i:] = args.amplitude_step_mm

    y = amplitude * sin_theta
    df = pd.DataFrame({"time_s": t, "y_mm": y})

    jump = float(abs(y[max(1, int(round(onset_s * args.fs)))]
                     - y[max(0, int(round(onset_s * args.fs)) - 1)]))
    sidecar = {
        "profile": kind,
        "onset_s": onset_s,
        "nominal_onset_s": args.baseline_s,
        "duration_s": float(args.duration_s),
        "fs_hz": float(args.fs),
        "baseline": {"amplitude_mm": float(args.amplitude_mm), "bpm": float(args.bpm)},
        "disturbed": {
            "amplitude_mm": float(args.amplitude_step_mm
                                   if kind in ("amplitude_step", "both_step") else args.amplitude_mm),
            "bpm": (f"wander {args.bpm} +-{args.wander_bpm}" if kind == "frequency_wander"
                    else float(args.bpm_step if kind in ("frequency_step", "both_step") else args.bpm)),
        },
        "peak_to_peak_mm": float(y.max() - y.min()),
        "sample_step_at_onset_mm": jump,
        "notes": (
            "onset_s is when EVERY change in this profile took effect -- there is exactly one "
            "onset per profile, at the first upward zero crossing at or after nominal_onset_s. "
            "The crossing is used because A*sin(theta) is only continuous in A where "
            "sin(theta)==0, and the rate change is deferred to the same instant so a combined "
            "profile does not end up with two onsets seconds apart. sample_step_at_onset_mm is "
            "the position change across that single sample -- no larger than an ordinary "
            "sample-to-sample step, i.e. no discontinuity for the phantom to jerk at."
        ),
    }
    return df, sidecar


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    p.add_argument("--fs", type=float, default=DEFAULT_FS)
    p.add_argument("--baseline-s", type=float, default=DEFAULT_BASELINE_S, dest="baseline_s",
                    help="clean breathing before the disturbance; must outlast approach+seat+"
                         "standoff+--record-s, which has taken 160-185s on real runs")
    p.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S, dest="duration_s")
    p.add_argument("--amplitude-mm", type=float, default=DEFAULT_AMPLITUDE_MM, dest="amplitude_mm")
    p.add_argument("--amplitude-step-mm", type=float, default=DEFAULT_AMPLITUDE_STEP_MM,
                    dest="amplitude_step_mm")
    p.add_argument("--bpm", type=float, default=DEFAULT_BPM)
    p.add_argument("--bpm-step", type=float, default=DEFAULT_BPM_STEP, dest="bpm_step")
    p.add_argument("--wander-bpm", type=float, default=DEFAULT_WANDER_BPM, dest="wander_bpm")
    p.add_argument("--wander-tau-s", type=float, default=DEFAULT_WANDER_TAU_S, dest="wander_tau_s")
    p.add_argument("--seed", type=int, default=DEFAULT_SEED,
                    help="fixes the wander profile, so a rerun reproduces the same disturbance")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(args.seed)

    for kind in ("amplitude_step", "frequency_step", "frequency_wander", "both_step"):
        df, sidecar = build_profile(kind, args, rng)
        csv_path = args.out_dir / f"{kind}.csv"
        json_path = args.out_dir / f"{kind}.onset.json"
        df.to_csv(csv_path, index=False)
        json_path.write_text(json.dumps(sidecar, indent=2) + "\n")
        print(f"wrote {csv_path}")
        print(f"  onset at {sidecar['onset_s']:.3f}s (nominal {sidecar['nominal_onset_s']:.0f}s), "
              f"p2p {sidecar['peak_to_peak_mm']:.2f}mm, "
              f"step across the onset sample {sidecar['sample_step_at_onset_mm']:.4f}mm")
        if sidecar["peak_to_peak_mm"] > 6.0:
            print(f"  NOTE: exceeds run_breathing_profile.py's default --max-travel-mm 6.0 -- "
                  f"play this with --max-travel-mm 6.5")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
