#!/usr/bin/env python3
"""List (and, on request, delete) invalid trial folders under outputs/param_sweep_runs/.

Defaults to listing only -- nothing is removed unless --delete is passed, and even then the
final count is printed before anything happens. Exists so cleanup is a reviewed, auditable
command rather than a hand-run ``rm -rf``: the same tool that decides which trials feed
``sweep_ekf_params.py`` (``validate_trial()`` in ``_sweep_common.py``) is what decides what is
safe to remove here, so a folder never gets deleted for a reason the sweep itself would not
also have excluded it for.

    python scripts/clean_bad_trials.py
    python scripts/clean_bad_trials.py --delete
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

from _sweep_common import DEFAULT_MIN_CONTACT_S, DEFAULT_OUT_ROOT, validate_trial


def find_bad_trials(out_root: Path, min_contact_s: float) -> list[tuple[Path, str]]:
    bad = []
    for profile_dir in sorted(out_root.glob("*")):
        if not profile_dir.is_dir() or profile_dir.name == "results":
            continue
        for trial_dir in sorted(profile_dir.glob("trial_*")):
            summary_path = trial_dir / "summary.json"
            if not summary_path.exists():
                bad.append((trial_dir, "no summary.json"))
                continue
            summary = json.loads(summary_path.read_text())
            ok, reason = validate_trial(summary, min_contact_s)
            if not ok:
                bad.append((trial_dir, reason))
    return bad


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT, dest="out_root")
    parser.add_argument("--min-contact-s", type=float, default=DEFAULT_MIN_CONTACT_S,
                         dest="min_contact_s")
    parser.add_argument("--delete", action="store_true",
                         help="actually remove the listed folders (default: list only)")
    args = parser.parse_args()

    bad = find_bad_trials(args.out_root, args.min_contact_s)
    if not bad:
        print("no invalid trial folders found.")
        return 0

    for trial_dir, reason in bad:
        print(f"{trial_dir}: {reason}")
    print(f"\n{len(bad)} invalid trial folder(s) found under {args.out_root}")

    if not args.delete:
        print("listing only -- pass --delete to remove these.")
        return 0

    print(f"deleting {len(bad)} folder(s)...")
    for trial_dir, _ in bad:
        shutil.rmtree(trial_dir)
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
