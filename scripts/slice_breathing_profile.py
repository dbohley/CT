#!/usr/bin/env python3
"""Slice the trailing N seconds of a real breathing profile for phantom playback.

``scripts/run_breathing_profile.py`` (and therefore ``scripts/run_approach_and_seat.py
--profile``) loads a profile CSV and loops the **whole file** from the start. That's the wrong
behavior when the point is to play the *last* ``--window-s`` seconds of a recording -- the same
trailing window ``scripts/plot_needle_gating_timing.py`` uses offline (steady-state breathing,
not the less-representative start of a take) -- so passing ``--profile moira_normal_breathing``
directly to the phantom driver does not reproduce that offline analysis; it plays the entire
~482s/~618s recording from t=0. This script slices the trailing window into its own CSV first.

Time is rebased to start at 0 in the sliced file, matching ``ct.sources.csv_source.CSVSource
.batch(t0=duration-window_s)``'s own slicing logic exactly, so the two are directly comparable.

    python scripts/slice_breathing_profile.py --profile moira_normal_breathing
    python scripts/slice_breathing_profile.py --profile emma_normal_breathing --window-s 300
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
BREATHE_PROFILES_DIR = REPO_ROOT / "breathe_profiles"
LIVE_DIR = REPO_ROOT / "outputs" / "needle_gating_live"
DEFAULT_WINDOW_S = 300.0


def resolve_profile(name: str) -> Path:
    """Same resolution idiom as run_breathing_profile.py's resolve_profile."""
    for candidate in (Path(name), BREATHE_PROFILES_DIR / name, BREATHE_PROFILES_DIR / f"{name}.csv"):
        if candidate.exists():
            return candidate
    available = ", ".join(p.stem for p in BREATHE_PROFILES_DIR.glob("*.csv"))
    raise FileNotFoundError(f"no profile matching {name!r}. Available in {BREATHE_PROFILES_DIR}: {available}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", required=True, help="bare name (resolved against "
                         "breathe_profiles/) or a path to a time_s,y_mm CSV")
    parser.add_argument("--window-s", type=float, default=DEFAULT_WINDOW_S, dest="window_s")
    parser.add_argument("--out", type=Path, default=None,
                         help="default outputs/needle_gating_live/profiles/<name>_last<window>s.csv")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    path = resolve_profile(args.profile)
    df = pd.read_csv(path)
    for col in ("time_s", "y_mm"):
        if col not in df.columns:
            raise KeyError(f"column '{col}' not in {path} (has {list(df.columns)})")

    total_duration = float(df["time_s"].iloc[-1] - df["time_s"].iloc[0])
    if total_duration < args.window_s:
        raise ValueError(f"{path.name}: recording is only {total_duration:.1f}s, shorter than "
                          f"the requested trailing window of {args.window_s:.1f}s")

    t0 = df["time_s"].iloc[-1] - args.window_s
    sliced = df[df["time_s"] >= t0].copy()
    sliced["time_s"] = sliced["time_s"] - sliced["time_s"].iloc[0]

    out = args.out or (LIVE_DIR / "profiles" / f"{path.stem}_last{int(args.window_s)}s.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    sliced[["time_s", "y_mm"]].to_csv(out, index=False)

    travel_mm = float(sliced["y_mm"].max() - sliced["y_mm"].min())
    print(f"wrote {out}")
    print(f"  {path.name}: took the last {args.window_s:.1f}s of {total_duration:.1f}s total "
          f"({len(sliced)} samples)")
    print(f"  peak-to-peak travel: {travel_mm:.2f}mm")
    if travel_mm > 6.0:
        print(f"  NOTE: exceeds run_breathing_profile.py's default --max-travel-mm 6.0mm -- "
              f"pass --max-travel-mm {travel_mm + 0.5:.1f} or higher when playing this file")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
