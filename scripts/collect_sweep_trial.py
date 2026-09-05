#!/usr/bin/env python3
"""Run ONE bench trial for the EKF parameter sweep.

Replaces the earlier all-in-one batch collector (which looped every profile x trial
automatically with an internal retry-on-failure): the user wants one trial per invocation, so a
bad trial is just "run this same command again" rather than something an automatic loop has to
detect and recover from -- and a human is watching every physical move, since each trial is its
own command.

    python scripts/collect_sweep_trial.py --list
        Prints the exact commands still needed to reach --trials-per-profile good trials for
        every profile in --profiles, counting only VALID trials already on disk (run this again
        any time to see what's left -- it never goes stale).

    python scripts/collect_sweep_trial.py --profile emma
        Runs exactly one trial: auto-picks the next trial number for "emma" under
        outputs/param_sweep_runs/emma_normal_breathing/trial_<n>/, runs
        run_approach_and_seat.py normally (its usual "Proceed?" confirmation still applies --
        this is one human-supervised trial, not an unattended batch), then validates the result
        and prints a plain GOOD/DISCARD verdict.

        The base is left where the trial ended -- `run_approach_and_seat.py` already
        de-energizes the motor (EXIT_MODE) on every exit, fault or not, so it is compliant and
        can be pushed back by hand before the next trial. Automatic retraction
        (`--retract-only-mm`) is not called here: it was not reliably moving the base on real
        hardware, and moving it back by hand is the simple fix while that gets sorted out.

Runs are collected under outputs/param_sweep_runs/, never outputs/approach_and_seat/ (that
directory is for the original one-off bench-development runs, not the sweep's systematic set),
and accumulate across separate invocations rather than starting a fresh timestamped folder each
time -- that accumulation IS "the folder of data" scripts/sweep_ekf_params.py sweeps over.

    python scripts/collect_sweep_trial.py --profile emma --dry-run
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

from _sweep_common import (
    DEFAULT_MIN_CONTACT_S,
    DEFAULT_OUT_ROOT,
    DEFAULT_PROFILES,
    DEFAULT_TRIALS_PER_PROFILE,
    REPO_ROOT,
    resolve_profile_name,
    validate_trial,
)

RUN_SCRIPT = REPO_ROOT / "scripts" / "run_approach_and_seat.py"
DEFAULT_RECORD_S = 180.0

TRIAL_DIR_RE = re.compile(r"^trial_(\d+)$")


def next_trial_dir(profile_root: Path) -> Path:
    """Next trial_<n> directory for this profile -- scans what's already there, +1."""
    existing = [
        int(m.group(1)) for p in profile_root.glob("trial_*") if p.is_dir()
        for m in [TRIAL_DIR_RE.match(p.name)] if m
    ]
    n = max(existing, default=0) + 1
    return profile_root / f"trial_{n}"


def count_valid_trials(profile_root: Path, min_contact_s: float) -> int:
    if not profile_root.exists():
        return 0
    count = 0
    for trial_dir in profile_root.glob("trial_*"):
        summary_path = trial_dir / "summary.json"
        if not summary_path.exists():
            continue
        try:
            summary = json.loads(summary_path.read_text())
        except json.JSONDecodeError:
            continue
        ok, _ = validate_trial(summary, min_contact_s)
        if ok:
            count += 1
    return count


def list_remaining(args: argparse.Namespace) -> int:
    profiles = [resolve_profile_name(p.strip()) for p in args.profiles.split(",") if p.strip()]
    print(f"target: {args.trials_per_profile} valid trial(s) per profile "
          f"({args.out_root})\n")
    any_remaining = False
    for profile in profiles:
        have = count_valid_trials(args.out_root / profile, args.min_contact_s)
        need = max(0, args.trials_per_profile - have)
        print(f"# {profile}: {have}/{args.trials_per_profile} valid trials so far")
        for _ in range(need):
            any_remaining = True
            print(f"python scripts/collect_sweep_trial.py --profile {profile}")
    if not any_remaining:
        print("\nall profiles have enough valid trials. Next:")
        print(f"  python scripts/sweep_ekf_params.py {args.out_root}")
    return 0


def run_one_trial(args: argparse.Namespace) -> int:
    profile = resolve_profile_name(args.profile)
    profile_root = args.out_root / profile
    profile_root.mkdir(parents=True, exist_ok=True)
    trial_dir = next_trial_dir(profile_root)

    cmd = [sys.executable, str(RUN_SCRIPT), "--profile", profile, "--out", str(trial_dir),
           "--record-s", str(args.record_s)]
    if args.velocity is not None:
        cmd += ["--velocity", str(args.velocity)]
    if args.yes:
        cmd += ["--yes"]
    if args.dry_run:
        cmd += ["--dry-run"]
    print(f"running: {' '.join(cmd)}")
    proc = subprocess.run(cmd)

    summary_path = trial_dir / "summary.json"
    if not summary_path.exists():
        print(f"\nDISCARD -- no summary.json written (exit code {proc.returncode}); "
              f"re-run this same command: python scripts/collect_sweep_trial.py "
              f"--profile {args.profile}")
        verdict_ok = False
    else:
        summary = json.loads(summary_path.read_text())
        ok, reason = validate_trial(summary, args.min_contact_s)
        if ok:
            print(f"\nGOOD -- usable ({trial_dir})")
        else:
            print(f"\nDISCARD -- {reason}\nre-run this same command: "
                  f"python scripts/collect_sweep_trial.py --profile {args.profile}")
        verdict_ok = ok

    print("\nbase is de-energized (compliant) where the trial ended -- move it back by hand "
          "before the next trial.")
    return 0 if verdict_ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", default=None,
                         help="short form ok, e.g. 'emma' (resolved against breathe_profiles/)")
    parser.add_argument("--list", action="store_true",
                         help="print the remaining commands needed to reach "
                              "--trials-per-profile valid trials for each of --profiles, "
                              "instead of running anything")
    parser.add_argument("--profiles", default=",".join(DEFAULT_PROFILES),
                         help="comma-separated, used by --list")
    parser.add_argument("--trials-per-profile", type=int, default=DEFAULT_TRIALS_PER_PROFILE,
                         dest="trials_per_profile", help="used by --list")
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT, dest="out_root")
    parser.add_argument("--record-s", type=float, default=DEFAULT_RECORD_S, dest="record_s")
    parser.add_argument("--velocity", type=float, default=None)
    parser.add_argument("--min-contact-s", type=float, default=DEFAULT_MIN_CONTACT_S,
                         dest="min_contact_s")
    parser.add_argument("--yes", action="store_true",
                         help="skip run_approach_and_seat.py's own confirmation (opt-in -- "
                              "default keeps it, since one trial is meant to be watched)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    if args.list:
        return list_remaining(args)
    if not args.profile:
        print("error: --profile is required (or use --list)")
        return 1
    return run_one_trial(args)


if __name__ == "__main__":
    raise SystemExit(main())
