#!/usr/bin/env python3
"""How long after a breathing disturbance does the estimator notice it?

Consumes a ``--monitor-s`` run of ``scripts/run_approach_and_seat.py`` played against one of
``scripts/generate_disturbance_profiles.py``'s profiles, and reports the detection latency.

**The statistic is NIS**, ``innovation^2 / S`` -- how surprising each measurement was in units of
the variance the filter itself predicted for it. That is what "out of distribution" means for a
Kalman filter, and ``HarmonicEKF.step()`` has always returned it; the run logs it per tick.

**The threshold is learned from the run's own clean baseline**, not configured: take the sliding
mean of NIS over the window between the EKF being seeded and the disturbance arriving, and put the
threshold at its ``--quantile``. Detection is the first crossing sustained for ``--hold-s``, which
stops a single noisy sample from counting as a detection.

**The true onset is recovered exactly, not estimated.** The phantom driver records
``commanded_mm`` -- literally the profile values it sent -- but no profile-time field, and it
loops, so where the run sits inside the file is not known a priori. ``align_profile`` searches
playback offsets for the one that reproduces ``commanded_mm``, which pins the run to the file to
the sample; the sidecar's ``onset_s`` then maps straight into run time. Locating the onset from
the waveform's *shape* instead was tried and abandoned: shape descriptors need a multi-breath
window and land about a second out, which is ten times the detector lag being measured.

**The latency is reported decomposed**, because not all of it belongs to the detector::

    t_onset(phantom) ----> t_onset(sensor) ----> t_detect
                     lag                  detector

The tactile chain lags the phantom by 0.28-0.70s depending on seating depth (session 013), so a
single end-to-end number would silently charge the detector for contact viscoelasticity. That lag
is measured per run by cross-correlation over the clean baseline (``measure_sensing_lag``), not
differenced from two noisy onset estimates.

Both processes log raw ``time.monotonic()`` as ``t``, which is what puts the two logs on one axis
(see ``ct.phantom.driver``).

    python scripts/analyze_disturbance_detection.py --self-test
    python scripts/analyze_disturbance_detection.py outputs/disturbance_detection/*/[0-9]*
    python scripts/analyze_disturbance_detection.py <run> --no-plot
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
PROFILE_DIR = REPO_ROOT / "outputs" / "disturbance_detection" / "profiles"

DEFAULT_WINDOW_S = 1.0     # sliding mean over NIS; ~a quarter breath, long enough to average
DEFAULT_HOLD_S = 0.2       # a crossing must persist this long to count
DEFAULT_QUANTILE = 0.999
DEFAULT_SETTLE_S = 10.0    # skip this much after seeding before trusting the baseline
DEFAULT_GUARD_S = 5.0      # and stop the baseline this far before the onset


def load_jsonl(path: Path) -> list[dict]:
    """Tolerates a torn final line, so a run still in progress can be analysed."""
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return out


def align_profile(t: np.ndarray, commanded: np.ndarray, profile_t: np.ndarray,
                   profile_y: np.ndarray, duration_s: float) -> float | None:
    """The run time at which the phantom was playing profile time zero.

    ``commanded_mm`` is not merely *like* the profile, it IS the profile -- the driver commands
    those offsets verbatim -- so where the run sits inside the file can be recovered exactly by a
    search over playback offsets rather than inferred from the waveform's shape.

    This replaces locating the disturbance by changepoint, which is not accurate enough for the
    job: shape descriptors need a multi-breath window and so localise the onset to about a second,
    while the detector lag being measured is under a tenth of one. Two variants were measured
    against a known onset before this was abandoned -- a trailing window landed 2.0s late, and a
    split-point search on a centred window 3.0s early.
    """
    coarse = np.arange(0.0, max(duration_s - (t[-1] - t[0]), 0.0) + 1.0, 0.5)
    best_offset, best_err = None, np.inf
    for grid in (coarse, None):
        if grid is None:
            grid = np.arange(best_offset - 0.5, best_offset + 0.5, 0.002)
        for offset in grid:
            model = np.interp((t - t[0]) + offset, profile_t, profile_y)
            err = float(np.mean((commanded - model) ** 2))
            if err < best_err:
                best_err, best_offset = err, float(offset)
    if best_offset is None:
        return None
    # Playback of profile time 0 happened this far before the log's first sample.
    return float(t[0] - best_offset)


def measure_sensing_lag(pt: np.ndarray, pc: np.ndarray, st: np.ndarray, sy: np.ndarray,
                         lo: float, hi: float, max_lag_s: float = 1.2) -> float | None:
    """How far the tactile chain lags the phantom, from a cross-correlation over clean baseline.

    Measured per run rather than assumed: session 013 found this moves from 0.28s to 0.70s with
    seating depth, and it is an order of magnitude larger than the detector lag it would otherwise
    be charged to. Restricted to the pre-disturbance baseline so the disturbance itself cannot
    influence the alignment.

    Normalised by ``sqrt(Ec*Ep)`` rather than by the overlap count, and clamped to just inside half
    a breath -- both are session 013's findings about this exact correlation: the overlap-count
    form over-corrects the triangular taper and biases the peak outward, and a periodic signal has
    a sidelobe every period that ``argmax`` cannot tell from the true one.
    """
    m = (st >= lo) & (st <= hi)
    if m.sum() < 200:
        return None
    grid = np.arange(lo, hi, 0.01)
    sensor = np.interp(grid, st[m], sy[m])
    truth = np.interp(grid, pt, pc)
    sensor = sensor - sensor.mean()
    truth = truth - truth.mean()
    if sensor.std() < 1e-9 or truth.std() < 1e-9:
        return None

    # Clamp to just inside half a period, so a sidelobe cannot be mistaken for the peak.
    crossings = np.flatnonzero(np.diff(np.sign(truth)) != 0)
    if len(crossings) >= 3:
        period = 2.0 * (grid[crossings[-1]] - grid[crossings[0]]) / (len(crossings) - 1)
        max_lag_s = min(max_lag_s, 0.45 * period)

    lags = np.arange(-int(max_lag_s / 0.01), int(max_lag_s / 0.01) + 1)
    best, best_lag = -np.inf, 0
    for lag in lags:
        if lag >= 0:
            a, b = sensor[lag:], truth[:len(truth) - lag] if lag else truth
        else:
            a, b = sensor[:lag], truth[-lag:]
        n = min(len(a), len(b))
        if n < 100:
            continue
        a, b = a[:n], b[:n]
        denom = np.sqrt(float(a @ a) * float(b @ b))
        score = float(a @ b) / denom if denom > 0 else 0.0
        if score > best:
            best, best_lag = score, lag
    return float(best_lag * 0.01)


def detect(t: np.ndarray, nis: np.ndarray, baseline: tuple[float, float],
           window_s: float, hold_s: float, quantile: float) -> tuple[float | None, float, np.ndarray]:
    """First sustained crossing of the baseline's own high quantile. Returns (t, threshold, series).

    Crossings are only looked for from the END of the baseline window onwards. Everything before
    that is either the filter still settling after being seeded -- which produces a transient large
    enough to trip any threshold, and which is excluded from learning the threshold precisely
    because it is not normal operation -- or the baseline itself, which is normal by construction.
    Scanning the whole series instead let the settling transient masquerade as a detection several
    seconds BEFORE the disturbance, which the self-test caught as a profile that "could not be
    detected" when in fact it had been detected far too early.
    """
    grid = np.arange(t[0] + window_s, t[-1], 0.05)
    smoothed = np.full(len(grid), np.nan)
    for i, centre in enumerate(grid):
        m = (t > centre - window_s) & (t <= centre)
        if m.any():
            smoothed[i] = float(np.nanmean(nis[m]))

    lo, hi = baseline
    base = smoothed[(grid >= lo) & (grid <= hi) & ~np.isnan(smoothed)]
    if len(base) < 20:
        return None, float("nan"), np.vstack([grid, smoothed])
    threshold = float(np.quantile(base, quantile))

    over = smoothed > threshold
    need = max(1, int(round(hold_s / 0.05)))
    run = 0
    for i, flag in enumerate(over):
        if grid[i] < hi or np.isnan(smoothed[i]):
            run = 0
            continue
        run = run + 1 if flag else 0
        if run >= need:
            return float(grid[i - need + 1]), threshold, np.vstack([grid, smoothed])
    return None, threshold, np.vstack([grid, smoothed])


def monotonic_origin(records: list[dict]) -> float:
    """The raw time.monotonic() value this run's `elapsed` axis calls zero.

    Same idiom as plot_approach_and_seat.py's _monotonic_origin -- it is what puts two
    independently-started processes on one axis.
    """
    return float(records[0]["t"]) - float(records[0].get("elapsed", 0.0))


def analyse(run_dir: Path, args) -> dict | None:
    records = load_jsonl(run_dir / "samples.jsonl")
    summary = json.loads((run_dir / "summary.json").read_text())
    monitor = summary.get("monitor") or {}
    if not monitor.get("enabled"):
        print(f"{run_dir}: not a --monitor-s run")
        return None

    tracked = [r for r in records if r.get("nis") is not None]
    if len(tracked) < 100:
        print(f"{run_dir}: only {len(tracked)} tracked samples -- nothing to analyse")
        return None
    t = np.array([r["elapsed"] for r in tracked])
    nis = np.array([r["nis"] for r in tracked])
    tactile = np.array([r["tactile_mm"] for r in tracked])
    ident_t = float(monitor.get("ident_t") or t[0])

    # Ground truth, exactly: align the phantom's commanded_mm against the profile it played, then
    # map the sidecar's onset into run time.
    profile_name = args.profile or run_dir.parent.name
    profile_csv = PROFILE_DIR / f"{profile_name}.csv"
    sidecar = PROFILE_DIR / f"{profile_name}.onset.json"
    if not (profile_csv.exists() and sidecar.exists()):
        print(f"{run_dir}: no profile '{profile_name}' in {PROFILE_DIR} -- pass --profile")
        return None
    meta = json.loads(sidecar.read_text())
    prof = np.loadtxt(profile_csv, delimiter=",", skiprows=1)

    phantom_path = run_dir / "phantom" / "samples.jsonl"
    if not phantom_path.exists():
        print(f"{run_dir}: no phantom log -- cannot locate the disturbance in run time")
        return None
    ph = [r for r in load_jsonl(phantom_path) if r.get("commanded_mm") is not None]
    origin = monotonic_origin(records)
    pt = np.array([float(r["t"]) - origin for r in ph])
    pc = np.array([float(r["commanded_mm"]) for r in ph])
    t_zero = align_profile(pt, pc, prof[:, 0], prof[:, 1], meta["duration_s"])
    if t_zero is None:
        print(f"{run_dir}: could not align the phantom log against {profile_name}")
        return None
    onset_phantom = t_zero + meta["onset_s"]

    # The sensing lag, measured over the clean baseline rather than differenced from two noisy
    # onset estimates. The tactile chain lags 0.28-0.70s with seating depth (session 013), so it
    # has to be measured per run, and it is far larger than the detector lag it would otherwise
    # contaminate.
    sensing_lag = measure_sensing_lag(pt, pc, t, tactile,
                                       ident_t + args.settle_s, onset_phantom - args.guard_s)
    onset_sensor = onset_phantom + sensing_lag if sensing_lag is not None else None

    onset = onset_phantom
    if not (t[0] < onset < t[-1]):
        print(f"{run_dir}: the disturbance at {onset:.1f}s falls outside the tracked window "
              f"{t[0]:.1f}-{t[-1]:.1f}s -- the baseline was too short or --monitor-s too brief")
        return None
    baseline = (ident_t + args.settle_s, onset - args.guard_s)
    if baseline[1] - baseline[0] < 20.0:
        print(f"{run_dir}: only {baseline[1] - baseline[0]:.0f}s of clean baseline before the "
              f"onset -- the calibration window probably overlapped the disturbance")

    t_detect, threshold, series = detect(t, nis, baseline, args.window_s, args.hold_s,
                                          args.quantile)

    # How WELL it was detected, not just whether. A latency alone says nothing about whether the
    # detector cleared its threshold by a hair or by two orders of magnitude, and those are very
    # different claims about the same number.
    grid, smoothed = series
    base_m = (grid >= baseline[0]) & (grid <= baseline[1]) & ~np.isnan(smoothed)
    after_m = (grid >= onset) & ~np.isnan(smoothed)
    baseline_median = float(np.median(smoothed[base_m])) if base_m.any() else float("nan")
    peak_after = float(np.nanmax(smoothed[after_m])) if after_m.any() else float("nan")
    frac_above = float(np.mean(smoothed[after_m] > threshold)) if after_m.any() else float("nan")
    margin = peak_after / threshold if threshold > 0 else float("nan")
    separation = peak_after / baseline_median if baseline_median > 0 else float("nan")

    # The same question in millimetres: how far the model's reconstruction fell behind the sensor
    # once the disturbance hit. This is what "the estimator noticed" means physically.
    resid = np.array([abs(r["innovation_mm"]) if r.get("innovation_mm") is not None else np.nan
                      for r in tracked])
    rb = (t >= baseline[0]) & (t <= baseline[1])
    ra = t >= onset
    resid_base = float(np.sqrt(np.nanmean(resid[rb] ** 2))) if rb.any() else float("nan")
    resid_after = float(np.sqrt(np.nanmean(resid[ra] ** 2))) if ra.any() else float("nan")

    if t_detect is None or t_detect < onset:
        grade = "MISSED"
    elif margin >= 10.0 and frac_above >= 0.5:
        grade = "strong"
    elif margin >= 3.0:
        grade = "clear"
    else:
        grade = "marginal"

    result = {
        "run": str(run_dir),
        "profile": run_dir.parent.name,
        "onset_phantom_s": onset_phantom,
        "onset_sensor_s": onset_sensor,
        "detected_s": t_detect,
        "latency_total_s": (t_detect - onset_phantom
                             if t_detect is not None and onset_phantom is not None else None),
        "latency_after_sensor_s": (t_detect - onset_sensor
                                    if t_detect is not None and onset_sensor is not None else None),
        "sensing_lag_s": (onset_sensor - onset_phantom
                           if onset_sensor is not None and onset_phantom is not None else None),
        "grade": grade,
        "nis_threshold": threshold,
        "nis_baseline_median": baseline_median,
        "nis_peak_after": peak_after,
        "detection_margin_x": margin,          # peak NIS as a multiple of the threshold
        "separation_x": separation,            # ...and as a multiple of the baseline level
        "fraction_above_threshold": frac_above,
        "residual_rmse_baseline_mm": resid_base,
        "residual_rmse_after_mm": resid_after,
        "residual_growth_x": (resid_after / resid_base
                               if resid_base and resid_base > 0 else float("nan")),
        "quality_note": (
            "detection_margin_x is how far the NIS peak cleared the threshold; >=10 with more "
            "than half the post-onset window above it is graded strong, >=3 clear, otherwise "
            "marginal. residual_growth_x says the same thing in millimetres -- how much worse "
            "the model's reconstruction of the sensor got once the disturbance hit."
        ),
        "baseline_window_s": [baseline[0], baseline[1]],
        "phase_advance_ok": (
            monitor.get("phase_advance_rad_s") is not None
            and monitor.get("omega_stage1")
            and monitor["phase_advance_rad_s"] >= 0.5 * monitor["omega_stage1"]
        ),
        "params": {"window_s": args.window_s, "hold_s": args.hold_s, "quantile": args.quantile},
    }

    if not args.no_plot:
        plot(run_dir, records, t, tactile, tracked, series, threshold, onset_phantom,
             onset_sensor, t_detect, baseline, result, args.zoom_s)
    return result


PHASE_COLORS = {"approach": "#d9d9d9", "seat": "#ffe4c4", "standoff": "#c6e2ff",
                "standoff_hold": "#c6ffd8", "monitor": "#ffffff"}


def plot(run_dir, records, t, tactile, tracked, series, threshold, onset_phantom, onset_sensor,
         t_detect, baseline, result, zoom_s: float) -> None:
    """Three panels: the whole run, a zoom on the disturbance, and the statistic.

    The whole-run panel exists because the tracked stretch is a small tail of the run -- plotting
    only that hides approach, seat, standoff and the calibration window that the threshold came
    from, which is most of what a reader needs to judge the run. The zoom exists because the
    interesting thing, the model's reconstruction coming apart, happens over a couple of breaths
    and is invisible at full-run scale.
    """
    y_pred = np.array([r.get("y_pred_mm") if r.get("y_pred_mm") is not None else np.nan
                       for r in tracked])
    all_t = np.array([r["elapsed"] for r in records])
    all_y = np.array([r["tactile_mm"] for r in records])
    phases = [r["phase"] for r in records]

    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.0, 1.2, 1.0], hspace=0.28)
    ax_run, ax_zoom, ax_nis = (fig.add_subplot(gs[0]), fig.add_subplot(gs[1]),
                                fig.add_subplot(gs[2]))

    # -- whole run, with the phases that produced it --------------------------------------
    start = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start]:
            ax_run.axvspan(all_t[start], all_t[min(i, len(all_t) - 1)],
                           color=PHASE_COLORS.get(phases[start], "#eeeeee"), alpha=0.5, zorder=0)
            start = i
    ax_run.plot(all_t, all_y, lw=0.6, color="C0", label="sensor tactile_mm")
    ax_run.plot(t, y_pred, lw=0.7, color="C4", alpha=0.9, label="model reconstruction")
    ax_run.set_ylabel("mm")
    ax_run.set_title(f"{result['profile']} -- {run_dir.name}   |   whole run "
                     f"(shaded by phase: approach / seat / standoff / hold / monitor)")

    # -- zoom on the disturbance ------------------------------------------------------------
    lo, hi = onset_phantom - zoom_s, onset_phantom + zoom_s
    m = (t >= lo) & (t <= hi)
    ax_zoom.plot(t[m], tactile[m], lw=1.3, color="C0", label="sensor")
    ax_zoom.plot(t[m], y_pred[m], lw=1.3, color="C4", label="model reconstruction")
    ax_zoom.fill_between(t[m], tactile[m], y_pred[m], color="C3", alpha=0.25,
                         label="reconstruction error")
    ax_zoom.set_ylabel("mm")
    ax_zoom.set_title(f"zoom: +/-{zoom_s:.0f}s around the disturbance -- the model tracks the "
                      f"breathing until it changes, then falls behind")

    # -- the statistic ----------------------------------------------------------------------
    ax_nis.plot(series[0], series[1], lw=1.0, color="C3", label="NIS (sliding mean)")
    ax_nis.axhline(threshold, color="k", ls="--", lw=1.1,
                   label=f"threshold = baseline q{result['params']['quantile']:g} "
                         f"= {threshold:.3g}")
    if np.isfinite(result["nis_baseline_median"]):
        ax_nis.axhline(result["nis_baseline_median"], color="k", ls=":", lw=0.9, alpha=0.6,
                       label=f"baseline median = {result['nis_baseline_median']:.3g}")
    ax_nis.set_yscale("log")
    ax_nis.set_ylabel("NIS")
    ax_nis.set_xlabel("elapsed (s)")

    for ax, span in ((ax_run, True), (ax_zoom, False), (ax_nis, True)):
        if span:
            ax.axvspan(*baseline, color="#c6e2ff", alpha=0.45, zorder=0,
                       label="baseline (threshold learned here)")
        if onset_phantom is not None:
            ax.axvline(onset_phantom, color="C2", lw=1.6, label="disturbance (phantom)")
        if onset_sensor is not None:
            ax.axvline(onset_sensor, color="C1", ls="--", lw=1.3, label="reaches the sensor")
        if t_detect is not None:
            ax.axvline(t_detect, color="C3", ls=":", lw=2.0, label="DETECTED")
        ax.grid(alpha=0.3)
    ax_zoom.set_xlim(lo, hi)
    for ax in (ax_run, ax_zoom, ax_nis):
        ax.legend(loc="upper left", fontsize=8, ncol=3)

    if result["latency_total_s"] is not None:
        verdict = (f"{result['grade'].upper()}   latency {result['latency_total_s']:.2f}s "
                   f"= {result['sensing_lag_s']:.2f}s sensing + "
                   f"{result['latency_after_sensor_s']:.2f}s detector\n"
                   f"NIS peaked {result['detection_margin_x']:.0f}x over threshold, "
                   f"{result['separation_x']:.0f}x over baseline; "
                   f"{result['fraction_above_threshold']:.0%} of the window after the "
                   f"disturbance stayed above it\n"
                   f"reconstruction error {result['residual_rmse_baseline_mm']:.3f}mm -> "
                   f"{result['residual_rmse_after_mm']:.3f}mm "
                   f"({result['residual_growth_x']:.1f}x worse)")
    else:
        verdict = (f"NOT DETECTED within the run (threshold {threshold:.3g}, NIS peaked "
                   f"{result['nis_peak_after']:.3g})")
    fig.text(0.5, 0.005, verdict, ha="center", fontsize=10,
             bbox=dict(boxstyle="round", fc="#fffbe6", ec="#999", alpha=0.95))

    fig.subplots_adjust(bottom=0.13, top=0.95, left=0.06, right=0.98)
    out = run_dir / "disturbance_detection.png"
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  saved {out}")


def summary_plot(results: list[dict], out_path: Path) -> None:
    usable = [r for r in results if r.get("latency_total_s") is not None]
    if not usable:
        return
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    labels = [f"{r['profile']}\n{Path(r['run']).name}" for r in usable]
    sensing = [r["sensing_lag_s"] for r in usable]
    detector = [r["latency_after_sensor_s"] for r in usable]
    x = np.arange(len(usable))
    ax.bar(x, sensing, label="sensing lag (phantom -> sensor)", color="C1")
    ax.bar(x, detector, bottom=sensing, label="detector lag (sensor -> detected)", color="C3")
    for i, r in enumerate(usable):
        ax.text(i, r["latency_total_s"], f" {r['latency_total_s']:.2f}s", ha="center",
                va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("seconds after the disturbance")
    ax.set_title("How FAST: detection latency, split by where the time goes")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y")

    # How well, beside how fast -- a latency alone does not say whether the detector cleared its
    # threshold by a hair or by three orders of magnitude.
    margins = [r["detection_margin_x"] for r in usable]
    colours = {"strong": "#2e7d32", "clear": "#f9a825", "marginal": "#ef6c00",
               "MISSED": "#b71c1c"}
    ax2.bar(x, margins, color=[colours.get(r["grade"], "#777") for r in usable])
    ax2.axhline(1.0, color="k", ls="--", lw=1.0, label="threshold")
    ax2.axhline(10.0, color="k", ls=":", lw=0.9, alpha=0.6, label="'strong' bar (10x)")
    ax2.set_yscale("log")
    for i, r in enumerate(usable):
        ax2.text(i, r["detection_margin_x"],
                 f" {r['grade']}\n {r['residual_growth_x']:.0f}x error",
                 ha="center", va="bottom", fontsize=8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, fontsize=8)
    ax2.set_ylabel("peak NIS / threshold")
    ax2.set_title("How WELL: margin over the threshold, and reconstruction-error growth")
    ax2.legend(fontsize=8, loc="lower right")
    ax2.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"\nsaved {out_path}")


def self_test(args) -> int:
    """Prove the detector fires on these disturbances before spending any rig time on them.

    Plays each generated profile through a simulated sensing chain -- session 013's measured gain
    and lag, session 012's noise floor -- and through the same identify/track/detect path a real
    run uses, with the onset known exactly. It cannot tell us the real gain or lag; that is what
    the hardware run is for. It does answer "does this fire at all, and roughly how fast", and it
    is what caught a generator bug that gave the combined profile two onsets 3s apart.

    Also checks align_profile(), which on a real run has to recover where the phantom was in the
    file without being told, and measure_sensing_lag() against a lag it was given.
    """
    import importlib.util

    import pandas as pd

    from ct.control.live import SignalAccumulator
    from ct.registry import build_identifier, build_tracker
    from ct.run import resolve_tracker_params

    spec = importlib.util.spec_from_file_location(
        "_ras", REPO_ROOT / "scripts" / "run_approach_and_seat.py")
    ras = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ras)

    fs, gain, lag_s, offset_mm, noise_mm = 124.0, 0.50, 0.35, 9.0, 0.010
    seed_len_s = 60.0        # matches the bench script's --record-s default for this experiment
    rng = np.random.default_rng(7)

    print(f"self-test: simulated chain gain={gain} lag={lag_s}s noise={noise_mm}mm "
          f"(session 012/013 numbers), seeding on the {seed_len_s:.0f}s ending 60s before each "
          f"profile's own onset\n")
    print(f"{'profile':18s} {'onset':>9} {'align err':>10} {'lag err':>8} {'detected':>9} "
          f"{'detector':>9}")

    failures = 0
    for kind in ("amplitude_step", "frequency_step", "frequency_wander", "both_step"):
        csv_path = PROFILE_DIR / f"{kind}.csv"
        if not csv_path.exists():
            print(f"{kind:18s} missing -- run scripts/generate_disturbance_profiles.py first")
            failures += 1
            continue
        df = pd.read_csv(csv_path)
        meta = json.loads((PROFILE_DIR / f"{kind}.onset.json").read_text())
        onset = meta["onset_s"]
        # Derived from each profile's own onset rather than fixed, so shortening the profiles
        # cannot silently leave the self-test seeding AFTER the disturbance it is meant to catch.
        seed_from_s = max(0.0, onset - seed_len_s - 60.0)

        t = np.arange(seed_from_s, meta["duration_s"] - 1.0, 1.0 / fs)
        delayed = np.interp(t - lag_s, df["time_s"], df["y_mm"])
        y = offset_mm + gain * delayed + rng.normal(0.0, noise_mm, len(t))

        # align_profile against the undelayed profile, standing in for the phantom's commanded_mm.
        truth = np.interp(t, df["time_s"], df["y_mm"])
        prof_t = df["time_s"].to_numpy()
        prof_y = df["y_mm"].to_numpy()
        t_zero = align_profile(t, truth, prof_t, prof_y, meta["duration_s"])
        found = None if t_zero is None else t_zero + onset
        align_err = float("nan") if found is None else found - onset
        measured_lag = measure_sensing_lag(t, truth, t, y, seed_from_s + 10.0, onset - 5.0)
        lag_err = float("nan") if measured_lag is None else measured_lag - lag_s

        acc = SignalAccumulator()
        seed_end = seed_from_s + seed_len_s
        for ti, yi in zip(t, y):
            if ti <= seed_end:
                acc.offer(ti, yi)
        ident = build_identifier(ras.IDENTIFIER_NAME, ras.IDENTIFIER_PARAMS).identify(
            acc.to_batch())
        tr = build_tracker(ras.TRACKER_NAME, resolve_tracker_params(
            ras.TRACKER_PARAMS, float(ident.diagnostics["bpm_hat"])))
        tr.init(ident, t0=acc.t_last)
        tt, nn = [], []
        for ti, yi in zip(t, y):
            if ti <= acc.t_last:
                continue
            tt.append(ti)
            nn.append(tr.step(float(ti), float(yi)).nis)
        tt, nn = np.array(tt), np.array(nn)

        t_det, _thr, _ = detect(tt, nn, (seed_end + args.settle_s, onset - args.guard_s),
                                 args.window_s, args.hold_s, args.quantile)
        at_sensor = onset + lag_s
        detected = "NOT DETECTED" if (t_det is None or t_det < onset) else f"{t_det:.2f}s"
        detector = "-" if (t_det is None or t_det < onset) else f"{t_det - at_sensor:.2f}s"
        print(f"{kind:18s} {onset:8.2f}s {align_err:9.3f}s {lag_err:7.3f}s {detected:>9} "
              f"{detector:>9}")
        # Alignment must be good to well under the detector lag it feeds, and the measured
        # sensing lag to a similar tolerance -- otherwise the reported latency is mostly error.
        if t_det is None or t_det < onset:
            failures += 1
        if abs(align_err) >= 0.05 or abs(lag_err) >= 0.05:
            failures += 1

    print("\nself-test " + ("PASSED" if failures == 0 else f"FAILED ({failures} problem(s))"))
    return 1 if failures else 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", nargs="*", type=Path)
    p.add_argument("--self-test", action="store_true", dest="self_test",
                    help="validate the detector against the generated profiles through a "
                         "simulated sensing chain, with no rig and no recorded run")
    p.add_argument("--window-s", type=float, default=DEFAULT_WINDOW_S, dest="window_s")
    p.add_argument("--hold-s", type=float, default=DEFAULT_HOLD_S, dest="hold_s")
    p.add_argument("--quantile", type=float, default=DEFAULT_QUANTILE)
    p.add_argument("--settle-s", type=float, default=DEFAULT_SETTLE_S, dest="settle_s")
    p.add_argument("--guard-s", type=float, default=DEFAULT_GUARD_S, dest="guard_s")
    p.add_argument("--profile", default=None,
                    help="profile name in outputs/disturbance_detection/profiles/ (default: the "
                         "run directory's parent name)")
    p.add_argument("--zoom-s", type=float, default=20.0, dest="zoom_s",
                    help="half-width of the zoom panel around the disturbance")
    p.add_argument("--no-plot", action="store_true", dest="no_plot")
    p.add_argument("--summary-out", type=Path, default=None, dest="summary_out")
    args = p.parse_args(argv)

    if args.self_test:
        return self_test(args)
    if not args.runs:
        p.error("give one or more run directories, or --self-test")

    results = []
    for run_dir in args.runs:
        if not (run_dir / "summary.json").exists():
            continue
        print(f"\n{run_dir}")
        r = analyse(run_dir, args)
        if r is None:
            continue
        results.append(r)
        if not r["phase_advance_ok"]:
            print("  WARNING: this run's filter was degenerate (theta stopped advancing) -- its "
                  "NIS series is meaningless and the latency below should not be reported. "
                  "See scripts/check_gate_health.py.")
        if r["detected_s"] is None:
            print(f"  NOT DETECTED within the run -- NIS peaked at {r['nis_peak_after']:.3g} "
                  f"against a threshold of {r['nis_threshold']:.3g}")
        else:
            print(f"  onset (phantom) {r['onset_phantom_s']:.2f}s | reaches the sensor "
                  f"{r['onset_sensor_s']:.2f}s | detected {r['detected_s']:.2f}s")
            print(f"  LATENCY   {r['latency_total_s']:.2f}s "
                  f"= {r['sensing_lag_s']:.2f}s sensing + "
                  f"{r['latency_after_sensor_s']:.2f}s detector")
            print(f"  HOW WELL  {r['grade'].upper()} -- NIS peaked "
                  f"{r['detection_margin_x']:.0f}x over threshold "
                  f"({r['separation_x']:.0f}x over baseline), and stayed above it for "
                  f"{r['fraction_above_threshold']:.0%} of the window after the disturbance")
            print(f"            reconstruction error "
                  f"{r['residual_rmse_baseline_mm']:.3f}mm -> "
                  f"{r['residual_rmse_after_mm']:.3f}mm ({r['residual_growth_x']:.1f}x worse)")

    if results:
        out = args.summary_out or (REPO_ROOT / "outputs" / "disturbance_detection"
                                   / "detection_latency.png")
        out.parent.mkdir(parents=True, exist_ok=True)
        summary_plot(results, out)
        (out.parent / "detection_latency.json").write_text(json.dumps(results, indent=2) + "\n")
        print(f"saved {out.parent / 'detection_latency.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
