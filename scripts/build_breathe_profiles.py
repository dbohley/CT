#!/usr/bin/env python3
"""Reduce raw OptiTrack takes in unfiltered_data/ to one breathing trace per subject.

Each export carries 70 columns: two rigid bodies (``tube``, ``crab``) whose position and
rotation columns are all-zero placeholder garbage, their rigid-body markers, and the nine
``Unlabeled NNNN`` chest-wall markers that actually carry the breathing signal. The useful
signal is the arithmetic mean of those nine markers' Y position -- confirmed exactly against
``unfiltered_data/Moira_normal average marker motion.csv``, which --verify re-checks on every
run so a future take proves the reduction is still the same operation.

Output is ``time_s,y_mm`` at the take's original absolute timestamps, matching the format of
breathe_profiles/breathing_profile_1.csv.

    python scripts/build_breathe_profiles.py
    python scripts/build_breathe_profiles.py --in-dir unfiltered_data --out-dir breathe_profiles
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IN_DIR = REPO_ROOT / "unfiltered_data"
DEFAULT_OUT_DIR = REPO_ROOT / "breathe_profiles"

# The OptiTrack CSV preamble: metadata, blank, then Type / Name / ID / Parent /
# Position-or-Rotation / axis, then the samples.
HEADER_ROWS = 8
TYPE_ROW, AXIS_ROW = 2, 7
TIME_COL = 1

EXPECTED_MARKERS = 9
REFERENCE_TAKE = "Moira normal breathing.csv"
REFERENCE_TRACE = "Moira_normal average marker motion.csv"


def marker_y_columns(path: Path) -> list[int]:
    """Column indices of the unlabeled markers' Y position, read from the header itself.

    Selecting by ``Type == "Marker"`` is what drops the rigid bodies and their
    ``Rigid Body Marker`` children, so this survives a take with a different rigid-body
    count -- no index into the row is ever hard-coded.
    """
    with path.open(newline="") as fh:
        reader = csv.reader(fh)
        rows = [next(reader) for _ in range(HEADER_ROWS)]
    cols = [
        i
        for i in range(len(rows[TYPE_ROW]))
        if rows[TYPE_ROW][i] == "Marker" and rows[AXIS_ROW][i] == "Y"
    ]
    if len(cols) != EXPECTED_MARKERS:
        names = sorted({rows[3][i] for i in cols})
        raise ValueError(
            f"{path.name}: expected {EXPECTED_MARKERS} unlabeled markers, found "
            f"{len(cols)} ({names})"
        )
    return cols


def nan_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous ``True`` runs of *mask* as ``(start, length)`` pairs."""
    if not mask.any():
        return []
    idx = np.flatnonzero(mask)
    breaks = np.flatnonzero(np.diff(idx) > 1)
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    return [(int(s), int(e - s + 1)) for s, e in zip(starts, ends)]


def reduce_take(path: Path, max_gap_frames: int) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int]]]:
    """Mean the nine markers' Y into one trace, filling short dropped-frame gaps.

    Returns ``(t, y_mm, gaps)``. Dropped frames are all-marker-simultaneous in these takes,
    so a gap is a genuine hole in the signal rather than a partial average -- short ones are
    interpolated to keep the uniform 120 Hz grid, and a long one is an error rather than
    something to silently smooth over.
    """
    ycols = marker_y_columns(path)
    df = pd.read_csv(path, skiprows=HEADER_ROWS, header=None, usecols=[TIME_COL] + ycols, low_memory=False)
    t = df[TIME_COL].to_numpy(dtype=float)
    y = df[ycols].to_numpy(dtype=float).mean(axis=1)

    missing = np.isnan(y)
    gaps = nan_runs(missing)
    too_long = [(s, n) for s, n in gaps if n > max_gap_frames]
    if too_long:
        s, n = too_long[0]
        raise ValueError(
            f"{path.name}: {len(too_long)} gap(s) longer than {max_gap_frames} frames; "
            f"first is {n} frames at t={t[s]:.6f}s (frame {s})"
        )
    if missing.any():
        y[missing] = np.interp(t[missing], t[~missing], y[~missing])
    return t, y, gaps


def write_trace(path: Path, t: np.ndarray, y: np.ndarray) -> None:
    pd.DataFrame({"time_s": t, "y_mm": y}).to_csv(path, index=False, float_format="%.6f")


def subject_of(path: Path) -> str:
    return re.sub(r"\s+", "_", path.stem.strip()).lower()


def verify_against_reference(in_dir: Path, max_gap_frames: int) -> None:
    """Re-derive the Moira trace and check it against the hand-produced reference."""
    take, ref_path = in_dir / REFERENCE_TAKE, in_dir / REFERENCE_TRACE
    if not (take.exists() and ref_path.exists()):
        print(f"  skipped: {REFERENCE_TAKE} or {REFERENCE_TRACE} not in {in_dir}")
        return

    ref = pd.read_csv(ref_path)
    t, y, _ = reduce_take(take, max_gap_frames)
    i0 = int(np.argmin(np.abs(t - ref["time_s"].iloc[0])))
    window = slice(i0, i0 + len(ref))

    dt = np.abs(t[window] - ref["time_s"].to_numpy(dtype=float)).max()
    dy = np.abs(y[window] - ref["y_mm"].to_numpy(dtype=float)).max()
    print(f"  {len(ref)} rows over t=[{ref['time_s'].iloc[0]:.6f}, {ref['time_s'].iloc[-1]:.6f}]s")
    print(f"  max |dt| = {dt:.2e} s    max |dy| = {dy:.2e} mm")
    # Both references are written to 6 decimal places, so agreement is bounded by rounding.
    if dt >= 2e-6 or dy >= 1e-5:
        raise AssertionError(
            f"reduction does not reproduce {REFERENCE_TRACE}: max |dt|={dt:.3e}s, max |dy|={dy:.3e}mm"
        )
    print("  OK -- reduction reproduces the reference")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--in-dir", type=Path, default=DEFAULT_IN_DIR, help="directory of raw OptiTrack exports")
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="where to write the traces")
    p.add_argument("--max-gap-frames", type=int, default=12, help="longest dropped-frame run to interpolate")
    p.add_argument("--no-verify", dest="verify", action="store_false", help="skip the reference check")
    args = p.parse_args()

    takes = sorted(args.in_dir.glob("* normal breathing.csv"))
    if not takes:
        raise SystemExit(f"no '* normal breathing.csv' takes found in {args.in_dir}")

    if args.verify:
        print(f"Verifying against {REFERENCE_TRACE}")
        verify_against_reference(args.in_dir, args.max_gap_frames)
        print()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for take in takes:
        t, y, gaps = reduce_take(take, args.max_gap_frames)
        out = args.out_dir / f"{subject_of(take)}.csv"
        write_trace(out, t, y)
        filled = sum(n for _, n in gaps)
        note = f", {filled} frame(s) in {len(gaps)} gap(s) interpolated" if filled else ""
        print(
            f"{take.name:35s} -> {out.name:32s} {len(t):6d} rows, "
            f"t=[{t[0]:.3f}, {t[-1]:.3f}]s, y={y.min():.2f}..{y.max():.2f} mm{note}"
        )


if __name__ == "__main__":
    main()
