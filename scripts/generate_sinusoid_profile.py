#!/usr/bin/env python3
"""Write a synthetic sinusoid breathing trace in the format the phantom driver expects.

``scripts/run_breathing_profile.py`` (and therefore ``scripts/run_approach_and_seat.py
--profile``) reads a ``time_s,y_mm`` CSV -- the same format as the real recordings in
``breathe_profiles/``. ``ct-generate`` writes ``t,y`` instead, so its output isn't directly
usable here; this script uses ``ct.sources.sinusoid.SinusoidSource`` (the same generator
``ct-generate`` uses) directly and writes the columns the phantom driver actually needs.

Duration defaults to an exact whole number of breath cycles so ``--loop``ing the file doesn't
introduce a discontinuity at the wrap-around. Defaults (15 breaths/min, 3mm amplitude -> 6mm
peak-to-peak) match ``scripts/plot_needle_gating_timing.py``'s sinusoid case, so a live run
against this profile is a fair comparison against that offline diagnostic. 6mm peak-to-peak also
sits exactly at ``run_breathing_profile.py``'s default ``--max-travel-mm`` safety clamp, so no
override is needed to play it.

    python scripts/generate_sinusoid_profile.py
    python scripts/run_breathing_profile.py --profile outputs/needle_gating_live/profiles/sinusoid_profile.csv --dry-run
    python scripts/run_approach_and_seat.py --insert-needle --profile outputs/needle_gating_live/profiles/sinusoid_profile.csv
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd

from ct.sources.sinusoid import SinusoidSource

REPO_ROOT = Path(__file__).resolve().parent.parent
LIVE_DIR = REPO_ROOT / "outputs" / "needle_gating_live"
DEFAULT_OUT = LIVE_DIR / "profiles" / "sinusoid_profile.csv"

DEFAULT_BREATHS_PER_MIN = 15.0
DEFAULT_AMPLITUDE_MM = 3.0  # -> 6mm peak-to-peak, exactly the phantom driver's default safety clamp
DEFAULT_CYCLES = 15  # ~60s at 15 breaths/min
DEFAULT_FS = 120.0  # matches breathe_profiles/*.csv's real sample rate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--breaths-per-min", type=float, default=DEFAULT_BREATHS_PER_MIN,
                         dest="breaths_per_min")
    parser.add_argument("--amplitude-mm", type=float, default=DEFAULT_AMPLITUDE_MM,
                         dest="amplitude_mm")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                         help="whole breath cycles to write -- keeps --loop wrap-around clean")
    parser.add_argument("--fs", type=float, default=DEFAULT_FS)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    period_s = 60.0 / args.breaths_per_min
    duration_s = args.cycles * period_s

    source = SinusoidSource(fs=args.fs, breaths_per_min=args.breaths_per_min,
                             amplitudes=[args.amplitude_mm])
    batch = source.batch(duration_s=duration_s, t0=0.0)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame({"time_s": batch.t, "y_mm": batch.y}).to_csv(args.out, index=False)

    print(f"wrote {args.out}")
    print(f"  {args.breaths_per_min:.1f} breaths/min, {args.amplitude_mm:.1f}mm amplitude, "
          f"{args.cycles} cycles ({duration_s:.2f}s), fs={args.fs:.0f}Hz, {batch.N} samples")
    print(f"  peak-to-peak: {batch.y.max() - batch.y.min():.2f}mm")
    print(f"\nrun_breathing_profile.py rebases about this file's own mean and loops it "
          f"continuously -- the {args.cycles}-cycle duration means the loop wraps cleanly with "
          f"no phase jump.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
