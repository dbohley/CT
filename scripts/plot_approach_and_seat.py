#!/usr/bin/env python3
"""Plot a bench run against phantom ground truth, and run the FFT/EKF estimator on it.

Reads ``<run>/samples.jsonl`` (t, dist_cm, tof_mm, tactile_mm, phase) and the sibling
summary.json via ct.rt.telemetry, plus the phantom's own log at
``<run>/phantom/samples.jsonl``, and renders one figure whose panels are shaded by the run's
approach/seat/standoff/standoff_hold phases. Matches this project's existing plot conventions
(ct.diagnostics.rig_plots / scripts/plot_approach_and_stop.py -- Agg backend, dpi=130 PNG).

**The figure is about latency.** In a real procedure there is no phantom: there is a patient
breathing, and a sensor that reads them late. So ground truth here is ``measured_mm``, what
the phantom's motor *actually did*, never ``commanded_mm``, what it was told to do --
commanded folds the phantom's own tracking lag into a number that is supposed to be sensor
latency alone, and so overstates it. Both logs record the raw, un-rebased
``time.monotonic()`` as ``t``, which is the only reason two independent processes on two CAN
buses can share an x-axis; the phantom's samples are mapped onto the controller's ``elapsed``
axis through that shared clock.

The payoff panel is the forecast. The sensor reads the phantom ~677ms late, so forecasting
by exactly that should put the estimate back on top of where the phantom is *now*:

    sensor(t)             ~ ratio * truth(t - tau_s) + c
    forecast(t, h=tau_s)  ~ ratio * truth(t)         + c

If the pipeline works, the forecast tracks real-time truth while the raw sensor visibly
trails it. That is the one claim this whole repo exists to support, and it is directly
checkable on bench data.

**Metrics are computed over one phase, not the whole run**, via
``ct.phantom.driver.compare_logs(phase=...)`` -- the same function ``ct-compare`` uses, so
there is one implementation of the alignment and correlation. This matters more than it
sounds: on run 20260901-165415, scoring the whole run gives correlation 0.039 and scoring
``standoff_hold`` gives 0.599 (0.961 with the 0.677s lag removed). Approach, seat and standoff
are phases where the base is moving and the sensor is not yet seated; including them does not
weaken the answer, it replaces it with a meaningless one.

**``<run>/aligned.csv``** carries the analysis window as ``time_s,sensor_mm,truth_mm,y_clean``.
``y_clean`` is what a noiseless sensor would have read -- the phantom truth mapped into the
sensor's frame by the two measured constants, ``ratio * truth(t - lag) + offset``. That name
is not incidental: ``ct.sources.csv_source.CSVSource`` already reads a ``y_clean`` column into
``SignalBatch.y_clean``, and ``ct.run.truth_function`` already prefers it over the measured
trace, so the estimator scores its forecast against the **phantom** rather than against the
sensor with no new plumbing at all. CLAUDE.md's rule holds: ``y_clean`` is for validation and
plots only and no estimator code reads it. ``truth_mm`` stays raw -- unshifted, unscaled --
so the mapping can be redone differently.

The estimator itself runs through ``ct.run.run_pipeline`` on ``configs/bench_aligned.yaml``,
the same code path as ``ct-pipeline``, so a figure here and a CLI run cannot disagree.

    python scripts/plot_approach_and_seat.py                              # most recent run
    python scripts/plot_approach_and_seat.py --run outputs/approach_and_seat/20260828-120000
    python scripts/plot_approach_and_seat.py --no-estimator --phase all --max-lag 2.0
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ct.config import RunConfig  # noqa: E402
from ct.phantom.driver import SEAM_JUMP_FACTOR, compare_logs  # noqa: E402
from ct.rt.telemetry import load_jsonl, to_arrays  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_RUNS_DIR = REPO_ROOT / "outputs" / "approach_and_seat"

PHASE_COLORS = {
    "approach": "#d9d9d9",
    "seat": "#ffe4c4",
    "standoff": "#c6e2ff",
    "standoff_hold": "#c6ffd8",
}

DEFAULT_PHASE = "standoff_hold"  # the only phase where the sensor is seated and the base is still
DEFAULT_MAX_LAG_S = 2.0  # the bench measured 0.677s; ct-compare's 1.0s default leaves little room
DEFAULT_ESTIMATOR_CONFIG = "bench_aligned"
SWEEP_HORIZONS = np.linspace(0.0, 1.5, 16)  # brackets the measured 0.677s tau_s on both sides


def _latest_run_dir() -> Path:
    runs = sorted(p for p in DEFAULT_RUNS_DIR.iterdir() if p.is_dir()) if DEFAULT_RUNS_DIR.exists() else []
    if not runs:
        raise FileNotFoundError(f"no runs found under {DEFAULT_RUNS_DIR}")
    return runs[-1]


def resolve_run(run_arg: str | None) -> Path:
    """--run may be a run directory or a direct path to samples.jsonl; None picks the latest."""
    if run_arg is None:
        return _latest_run_dir()
    path = Path(run_arg)
    if path.is_file():
        return path.parent
    return path


def _shade_phases(ax, t: np.ndarray, phases: list[str]) -> None:
    """axvspan each contiguous run of the same phase, matching this project's existing
    ct.diagnostics.rig_plots state-shading convention."""
    if not phases:
        return
    start_i = 0
    for i in range(1, len(phases) + 1):
        if i == len(phases) or phases[i] != phases[start_i]:
            color = PHASE_COLORS.get(phases[start_i], "#eeeeee")
            ax.axvspan(t[start_i], t[i - 1] if i < len(phases) else t[-1], color=color, alpha=0.5, zorder=0)
            start_i = i


def _monotonic_origin(records: list[dict]) -> float:
    """The raw ``time.monotonic()`` value that the controller's ``elapsed`` axis calls zero.

    Both processes log un-rebased ``t``; subtracting this from the phantom's ``t`` is what
    puts two independently-started processes on one x-axis.
    """
    return float(records[0]["t"]) - float(records[0].get("elapsed", 0.0))


def load_phantom(run_dir: Path, origin: float) -> dict | None:
    """Phantom ground truth on the controller's ``elapsed`` axis, or None if not driven."""
    path = run_dir / "phantom" / "samples.jsonl"
    if not path.exists():
        return None
    records = load_jsonl(path)
    if not records:
        return None
    cols = to_arrays(records, ["t", "commanded_mm", "measured_mm"])
    measured = cols["measured_mm"]
    summary_path = run_dir / "phantom" / "summary.json"
    return {
        "path": path,
        "elapsed": cols["t"] - origin,
        "commanded_mm": cols["commanded_mm"],
        "measured_mm": measured,
        # Logs written before the phantom script recorded feedback have no measured_mm at
        # all; so do runs where the broadcast never arrived. Both must plot commanded only.
        "has_measured": bool(np.any(~np.isnan(measured))),
        "summary": json.loads(summary_path.read_text()) if summary_path.exists() else {},
    }


def seam_times(phantom: dict) -> np.ndarray:
    """When the looped profile restarted. No sensor can track those instants.

    Same rule as ct.phantom.driver.count_seams, on the same raw commanded series, so the
    marks on the figure and the count in the printed metrics cannot disagree.
    """
    y = phantom["commanded_mm"]
    finite = ~np.isnan(y)
    if finite.sum() < 3:
        return np.array([])
    t, y = phantom["elapsed"][finite], y[finite]
    steps = np.abs(np.diff(y))
    reference = float(np.quantile(steps, 0.95))
    if reference <= 0:
        return np.array([])
    return t[1:][steps > SEAM_JUMP_FACTOR * reference]


def analysis_span(t: np.ndarray, phases: list[str], phase: str | None) -> tuple[float, float]:
    """Time range of the analysis phase, for framing it on every panel."""
    if phase is None:
        return float(t[0]), float(t[-1])
    idx = [i for i, p in enumerate(phases) if p == phase]
    if not idx:
        return float(t[0]), float(t[-1])
    return float(t[idx[0]]), float(t[idx[-1]])


def _frame_span(ax, span: tuple[float, float]) -> None:
    """Outline the analysis window, so it is obvious which part the metrics describe."""
    for x in span:
        ax.axvline(x, color="k", lw=0.9, ls="-", alpha=0.35, label="_nolegend_")


def _centre_and_limits(t: np.ndarray, y: np.ndarray, span: tuple[float, float]) -> tuple[float, tuple[float, float]]:
    """Mean and y-limits taken over the analysis window only.

    Centring on the whole run would let the seating transient -- several mm of one-way base
    travel, an order of magnitude larger than the breathing -- set the scale, squashing the
    comparison this panel exists to show into a couple of pixels.
    """
    window = (t >= span[0]) & (t <= span[1]) & ~np.isnan(y)
    values = y[window] if window.any() else y[~np.isnan(y)]
    if values.size == 0:
        return 0.0, (-1.0, 1.0)
    centre = float(values.mean())
    half = max(float(np.abs(values - centre).max()) * 1.15, 1e-3)
    return centre, (-half, half)


def _plot_ground_truth(ax, t, phases, tactile_mm, phantom, span) -> None:
    """What the phantom actually did, next to what the sensor made of it.

    Means are removed from both: the phantom's position and the tactile deflection are two
    different datums, and only shape and timing are comparable. Twin axes because the two
    differ in scale by roughly the amplitude ratio -- which is the point of the next panel.
    """
    _shade_phases(ax, t, phases)
    truth_label = "measured_mm (motor)" if phantom["has_measured"] else "commanded_mm (profile)"
    truth = phantom["measured_mm"] if phantom["has_measured"] else phantom["commanded_mm"]
    centre, limits = _centre_and_limits(phantom["elapsed"], truth, span)
    if phantom["has_measured"]:
        ax.plot(phantom["elapsed"], phantom["commanded_mm"] - centre,
                lw=0.7, color="C7", alpha=0.6, label="commanded_mm (profile)")
    ax.plot(phantom["elapsed"], truth - centre, lw=1.0, color="C3",
            label=f"phantom {truth_label}")
    for x in seam_times(phantom):
        ax.axvline(x, color="C3", lw=0.5, ls=":", alpha=0.4, label="_nolegend_")
    ax.set_ylabel("phantom [mm]\n(centred on window)")
    ax.set_ylim(limits)
    ax.grid(alpha=0.3)
    _frame_span(ax, span)

    twin = ax.twinx()
    sensor_centre, sensor_limits = _centre_and_limits(t, tactile_mm, span)
    twin.plot(t, tactile_mm - sensor_centre, lw=0.8, color="C0", label="sensor tactile_mm")
    twin.set_ylabel("sensor [mm]")
    twin.set_ylim(sensor_limits)
    lines = ax.get_lines()[: 2 if phantom["has_measured"] else 1] + twin.get_lines()
    ax.legend(lines, [line.get_label() for line in lines], loc="upper left", fontsize=8)


def _plot_reconstruction(ax, t, phases, tactile_mm, phantom, metrics, span, run_dir) -> Path | None:
    """The sensor against ground truth after removing the measured lag and amplitude ratio.

    This is the honest picture of the sensing chain: what is left over once the two effects
    that *are* characterised have been taken out is what an estimator would have to contend
    with. Also writes aligned.csv, the same two series on one grid, for the FFT/EKF stage.
    """
    _shade_phases(ax, t, phases)
    ax.set_ylabel("reconstruction\n[mm]")
    ax.grid(alpha=0.3)
    _frame_span(ax, span)
    if metrics is None:
        ax.text(0.5, 0.5, "no analysable window", transform=ax.transAxes,
                ha="center", va="center", fontsize=9, color="gray")
        return None

    truth = phantom["measured_mm"] if metrics["phantom_field_used"] == "measured_mm" else phantom["commanded_mm"]
    finite = ~np.isnan(truth)
    tp, yp = phantom["elapsed"][finite], truth[finite]

    lo, hi = span
    grid = np.arange(max(lo, tp[0]), min(hi, tp[-1]), 1.0 / metrics["fs"])
    if grid.size < 2:
        return None
    # Shift the *truth* forward by the measured lag so it lands where the sensor saw it,
    # and scale it by the measured ratio. What remains is the residual the metrics report.
    truth_grid = np.interp(grid, tp + metrics["lag_s"], yp) * metrics["amplitude_ratio"]
    sensor_grid = np.interp(grid, t, tactile_mm)
    truth_grid -= truth_grid.mean()
    sensor_grid -= sensor_grid.mean()

    ax.plot(grid, truth_grid, lw=1.0, color="C3",
            label=f"truth, lag {metrics['lag_s']*1000:.0f}ms & x{metrics['amplitude_ratio']:.2f} applied")
    ax.plot(grid, sensor_grid, lw=0.8, color="C0", label="sensor tactile_mm")
    ax.plot(grid, sensor_grid - truth_grid, lw=0.6, color="0.35", alpha=0.9, label="residual")
    ax.legend(loc="upper left", fontsize=8, ncol=3)

    return write_aligned_csv(run_dir, grid, t, tactile_mm, tp, yp, metrics)


def write_aligned_csv(run_dir, grid, t, tactile_mm, tp, yp, metrics) -> Path:
    """``time_s,sensor_mm,truth_mm,y_clean`` over the analysis window.

    ``truth_mm`` is raw -- unshifted, unscaled. Dealing with the lag is the estimator's job
    and pre-applying it there would hand it the answer.

    ``y_clean`` is the noiseless sensor: the same truth mapped into the sensor's frame by the
    measured lag and ratio. The name is load-bearing -- ``CSVSource`` reads that exact column
    into ``SignalBatch.y_clean`` and ``ct.run.truth_function`` prefers it, which is what makes
    the estimator score its forecast against the phantom instead of against the sensor. The
    mapping is a two-parameter fit over the whole window; it cannot fake agreement with a
    waveform across ~18 breaths, which is what makes the forecast panel a real test.
    """
    sensor = np.interp(grid, t, tactile_mm)
    truth_raw = np.interp(grid, tp, yp)
    # Offset, not just scale: the two are different datums (tactile deflection vs phantom
    # position), so only shape and timing are comparable. Match means over the window.
    shifted = np.interp(grid, tp + metrics["lag_s"], yp)
    y_clean = metrics["amplitude_ratio"] * (shifted - shifted.mean()) + sensor.mean()

    aligned_path = run_dir / "aligned.csv"
    np.savetxt(
        aligned_path,
        np.column_stack([grid - grid[0], sensor, truth_raw, y_clean]),
        delimiter=",", header="time_s,sensor_mm,truth_mm,y_clean", comments="", fmt="%.6f",
    )
    return aligned_path


def _plot_ekf_tracking(ax, t, phases, result, window_t0, span) -> None:
    """What the EKF made of the sensor trace: y_pred with its +-sigma ribbon.

    Plotted on the run's own elapsed axis (the estimator works in aligned.csv's rebased
    time, which starts at the analysis window), so it lines up with every other panel.
    """
    _shade_phases(ax, t, phases)
    h = result.history
    te = h.t + window_t0
    sigma = np.sqrt(np.maximum(h.S, 0.0))
    ax.plot(te, h.y, lw=0.7, color="C0", alpha=0.7, label="sensor tactile_mm")
    ax.fill_between(te, h.y_pred - sigma, h.y_pred + sigma, color="C2", alpha=0.25,
                    label="+-1 sigma (sqrt S)")
    ax.plot(te, h.y_pred, lw=1.0, color="C2", label=f"EKF y_pred (K={result.ident.K})")
    # Everything before this is Stage 1's calibration window; the filter has seen none of it.
    calib_end = window_t0 + float(result.calibration.t[-1])
    ax.axvline(calib_end, color="C2", lw=1.0, ls="--", alpha=0.7, label="calib -> track")
    ax.set_ylabel("EKF [mm]")
    ax.grid(alpha=0.3)
    _frame_span(ax, span)
    ax.legend(loc="upper left", fontsize=8, ncol=2)


def _plot_forecast(ax, t, phases, result, window_t0, span, lag_s) -> None:
    """The payoff: forecasting by the measured lag should land on real-time truth.

    ``forecast_target`` is ``truth_function`` evaluated at ``t+h``, and because aligned.csv
    supplies ``y_clean`` that target is the *phantom* in the sensor's frame -- not the sensor.
    At ``h = tau_s`` it is therefore where the phantom actually is at time ``t``. The raw
    sensor is drawn alongside so the delay the forecast is undoing stays visible.
    """
    _shade_phases(ax, t, phases)
    h = result.history
    te = h.t + window_t0
    if h.forecast is None or h.forecast_target is None:
        ax.text(0.5, 0.5, "no forecast (horizon = 0)", transform=ax.transAxes,
                ha="center", va="center", fontsize=9, color="gray")
        ax.set_ylabel("forecast [mm]")
        return

    ax.plot(te, h.y, lw=0.7, color="C0", alpha=0.6,
            label=f"sensor (trails truth by {lag_s * 1000:.0f}ms)")
    ax.plot(te, h.forecast_target, lw=1.2, color="C3", label="phantom position NOW")
    ax.plot(te, h.forecast, lw=1.0, color="C4", ls="--",
            label=f"forecast at h={h.horizon:.3f}s")
    err = h.forecast - h.forecast_target
    ax.set_ylabel("forecast [mm]")
    ax.grid(alpha=0.3)
    _frame_span(ax, span)
    rmse = float(np.sqrt(np.nanmean(err**2)))
    naive = float(np.sqrt(np.nanmean((h.y - h.forecast_target) ** 2)))
    ax.set_title(f"forecast RMSE {rmse:.4f} mm vs {naive:.4f} mm for the raw sensor "
                 f"— {100 * (1 - rmse / naive):.0f}% of the lag error removed",
                 fontsize=9, loc="left")
    ax.legend(loc="upper left", fontsize=8, ncol=3)


def _write_sweep_figure(rows, result, run_dir: Path, cfg) -> Path:
    """Forecast error vs horizon, with the measured sensor lag marked.

    Reuses ct.diagnostics.plots.plot_horizon_sweep so this figure and ct-sweep-horizon's
    cannot drift apart, then annotates the one horizon that is a measurement rather than a
    choice. The amplitude line is 5% of the tracked signal's own peak-to-peak, which is what
    that helper's ``amplitude`` argument means.
    """
    from ct.diagnostics import plots  # noqa: PLC0415

    y = result.history.y
    amplitude = float(np.nanmax(y) - np.nanmin(y))
    path = plots.plot_horizon_sweep(rows, run_dir / "bench_horizon_sweep.png", amplitude=amplitude)

    # plot_horizon_sweep closes its figure, so the tau_s marker is drawn by reopening the
    # same data rather than by reaching into it -- cheap, and keeps that helper untouched.
    fig, ax = plt.subplots(figsize=(8, 5))
    h = np.array([r["horizon"] for r in rows])
    ax.plot(h, [r["rmse"] for r in rows], "o-", label="RMSE")
    ax.plot(h, [r["max_abs"] for r in rows], "s--", alpha=0.7, label="max |error|")
    ax.axhline(0.05 * amplitude, color="C3", ls=":", label="5% of amplitude")
    ax.axvline(cfg.horizon, color="C4", lw=1.2,
               label=f"measured sensor lag ({cfg.horizon:.3f}s)")
    ax.set(title="forecast error vs horizon, real bench data",
           xlabel="horizon h [s]", ylabel="error [mm]")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def ground_truth_metrics(phantom: dict, jsonl_path: Path, phase: str | None, args):
    """``compare_logs`` over the analysis window, degrading rather than failing.

    Two things can go wrong and neither should cost the whole figure: a run that faulted
    before ``standoff_hold`` has no analysable window, and a log written before the phantom
    script recorded motor feedback has no ``measured_mm`` to be truth.
    """
    try:
        return compare_logs(phantom["path"], jsonl_path, max_lag_s=args.max_lag,
                            phase=phase, phantom_field=args.truth)
    except ValueError as exc:
        if args.truth != "measured_mm" or "measured_mm" not in str(exc):
            print(f"note: no ground-truth metrics ({exc})")
            return None

    print("WARNING: this phantom log has no measured_mm, so ground truth is commanded_mm --\n"
          "         what the phantom was TOLD to do, not what it did. The lag reported below\n"
          "         therefore includes the phantom motor's own tracking lag on top of the\n"
          "         sensor's, and is an OVER-estimate of sensor latency. Re-run with the\n"
          "         current run_breathing_profile.py to separate them.")
    try:
        return compare_logs(phantom["path"], jsonl_path, max_lag_s=args.max_lag,
                            phase=phase, phantom_field="commanded_mm")
    except ValueError as exc:
        print(f"note: no ground-truth metrics ({exc})")
        return None


def run_estimator(aligned_path: Path, config_name: str, horizon: float | None):
    """Stage 1 + Stage 2 over the analysis window, via the same path as ``ct-pipeline``.

    Returns ``(result, sweep_rows, cfg)``, or ``(None, None, None)`` if the window is too
    short to both calibrate and track on.
    """
    from ct.run import run_pipeline, sweep_horizons  # noqa: PLC0415  (matplotlib import order)

    cfg = RunConfig.from_yaml(config_name)
    cfg.source = {**cfg.source, "params": {**cfg.source.get("params", {}), "path": str(aligned_path)}}
    if horizon is not None:
        cfg.horizon = float(horizon)

    span = float(np.loadtxt(aligned_path, delimiter=",", skiprows=1, usecols=0)[-1])
    # The config's calib_seconds assumes a --record-s 180 run. On a shorter hold (or one of
    # the 60s runs recorded before that default changed) it would swallow the whole window
    # and leave nothing to track, so clamp rather than fail -- and say so, because a
    # shortened calibration means fewer per-breath refits and a noisier Q.
    if cfg.calib_seconds > 0.5 * span:
        clamped = round(0.5 * span, 1)
        print(f"note: calib_seconds {cfg.calib_seconds:.0f}s exceeds half the {span:.0f}s "
              f"analysis window -- using {clamped:.0f}s so there is something left to track. "
              f"Record longer (--record-s 180) for a well-estimated Q.")
        cfg.calib_seconds = clamped

    try:
        result = run_pipeline(cfg)
        rows = sweep_horizons(cfg, [float(h) for h in SWEEP_HORIZONS])
    except ValueError as exc:
        print(f"note: estimator did not run ({exc})")
        return None, None, None
    return result, rows, cfg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default=None,
                         help="run directory or samples.jsonl path; default: most recent under "
                              "outputs/approach_and_seat/")
    parser.add_argument("--phase", default=DEFAULT_PHASE,
                         help="phase to compute the ground-truth metrics over; 'all' for the "
                              "whole run (which on a bench run is meaningless -- see the "
                              "module docstring)")
    parser.add_argument("--truth", default="measured_mm",
                         choices=["auto", "commanded_mm", "measured_mm"],
                         help="what counts as phantom ground truth. Defaults to what the "
                              "motor actually did; 'commanded_mm' folds the phantom's own "
                              "tracking lag into the measured sensor latency")
    parser.add_argument("--max-lag", type=float, default=DEFAULT_MAX_LAG_S,
                         help="widest sensor lag to search for [s]")
    parser.add_argument("--no-estimator", dest="estimator", action="store_false",
                         help="skip Stage 1/2 and just plot the run")
    parser.add_argument("--estimator-config", default=DEFAULT_ESTIMATOR_CONFIG,
                         help="config for the FFT/EKF run over aligned.csv")
    parser.add_argument("--horizon", type=float, default=None,
                         help="forecast horizon [s]; default is the config's, which is the "
                              "measured sensor lag")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = resolve_run(args.run)
    jsonl_path = run_dir / "samples.jsonl"
    summary_path = run_dir / "summary.json"

    if not jsonl_path.exists():
        print(f"error: no samples.jsonl found at {jsonl_path}")
        return 1

    records = load_jsonl(jsonl_path)
    if not records:
        print(f"error: {jsonl_path} has no records")
        return 1

    # Plot against "elapsed" (rebased to this run's own start), not "t" (the raw
    # time.monotonic() reading kept for cross-process alignment via ct-compare -- see
    # run_approach_and_seat.py's module docstring). "elapsed" is what's human-readable here.
    cols = to_arrays(records, ["elapsed", "dist_cm", "tof_mm", "tactile_mm"])
    t, dist_cm, tof_mm = cols["elapsed"], cols["dist_cm"], cols["tof_mm"]
    tactile_mm = cols["tactile_mm"]
    phases = [r.get("phase", "unknown") for r in records]

    phase = None if args.phase == "all" else args.phase
    phantom = load_phantom(run_dir, _monotonic_origin(records))
    metrics = None
    if phantom is not None:
        metrics = ground_truth_metrics(phantom, jsonl_path, phase, args)
    span = analysis_span(t, phases, phase)

    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    contact_threshold_cm = summary.get("params", {}).get("contact_threshold_cm")
    standoff_dist_cm = summary.get("standoff", {}).get("standoff_dist_cm")
    contact_t = summary.get("contact_t")
    standoff_crossed_t = summary.get("standoff", {}).get("crossed_t")
    phase_reached = summary.get("phase_reached", "unknown")
    fault_reason = summary.get("fault_reason")
    seat = summary.get("seat", {})
    standoff = summary.get("standoff", {})

    title = f"approach & seat — reached '{phase_reached}'"
    if fault_reason:
        title += " (FAULT)"

    # aligned.csv has to exist before the estimator can read it, and it is written by the
    # reconstruction panel -- so the figure is built in two passes: draw everything that
    # depends only on the logs, run Stage 1/2, then add its panels.
    estimator_panels = 2 if (args.estimator and metrics is not None) else 0
    n_panels = 2 + (2 if phantom is not None else 0) + estimator_panels
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 2.6 * n_panels), sharex=True)

    _shade_phases(axes[0], t, phases)
    axes[0].plot(t, dist_cm, lw=0.8, label="dist_cm")
    if contact_threshold_cm is not None:
        axes[0].axhline(contact_threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6, label="contact threshold")
        axes[0].axhline(-contact_threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6)
    if standoff_dist_cm is not None:
        axes[0].axhline(standoff_dist_cm, color="tab:blue", lw=0.6, ls="-.", alpha=0.7, label="standoff threshold")
        axes[0].axhline(-standoff_dist_cm, color="tab:blue", lw=0.6, ls="-.", alpha=0.7)
    axes[0].set_ylabel("dist_cm [cm]")
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(alpha=0.3)
    axes[0].set_title(title)

    _shade_phases(axes[1], t, phases)
    axes[1].plot(t, tof_mm, lw=0.8, label="tof_mm", color="C1")
    axes[1].set_ylabel("tof_mm [mm]")
    axes[1].legend(loc="upper left", fontsize=8)
    axes[1].grid(alpha=0.3)

    aligned_path: Path | None = None
    if phantom is not None:
        _plot_ground_truth(axes[2], t, phases, tactile_mm, phantom, span)
        aligned_path = _plot_reconstruction(
            axes[3], t, phases, tactile_mm, phantom, metrics, span, run_dir,
        )

    result = rows = None
    if estimator_panels and aligned_path is not None:
        result, rows, est_cfg = run_estimator(aligned_path, args.estimator_config, args.horizon)
        if result is not None:
            _plot_ekf_tracking(axes[4], t, phases, result, span[0], span)
            _plot_forecast(axes[5], t, phases, result, span[0], span, metrics["lag_s"])
            _write_sweep_figure(rows, result, run_dir, est_cfg)
        else:
            for ax in axes[4:6]:
                ax.text(0.5, 0.5, "estimator did not run", transform=ax.transAxes,
                        ha="center", va="center", fontsize=9, color="gray")

    axes[-1].set_xlabel("t [s]")

    for ax in axes:
        if contact_t is not None:
            ax.axvline(contact_t, color="k", lw=0.7, ls="--", alpha=0.6, label="_nolegend_")
        if standoff_crossed_t is not None:
            ax.axvline(standoff_crossed_t, color="k", lw=0.7, ls="--", alpha=0.6, label="_nolegend_")

    fig.tight_layout()
    out_path = run_dir / "approach_and_seat.png"
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    print(f"saved: {out_path}")
    print(f"\nphase reached: {phase_reached}" + (f"  FAULT: {fault_reason}" if fault_reason else ""))
    if seat.get("increments") is not None:
        print(f"seat: {seat['increments']} increment(s), accepted peak "
              f"{seat.get('accepted_peak_tactile_mm')}mm")
    if standoff_crossed_t is not None:
        increments_note = (f" after {standoff['increments']} creep-back step(s)"
                            if standoff.get("increments") is not None else "")
        print(f"standoff crossed at t={standoff_crossed_t:.2f}s{increments_note}")

    if phantom is None:
        print("\nno phantom log at <run>/phantom/samples.jsonl -- ground-truth panels omitted "
              "(was this run started with --no-phantom?)")
    else:
        _report_ground_truth(phantom, metrics, args, aligned_path)
    if result is not None:
        _report_estimator(result, rows, metrics)
    return 0


def _report_estimator(result, rows, metrics) -> None:
    ident, summary = result.ident, result.summary
    h = result.history
    print(f"\nestimator (Stage 1 + Stage 2, config '{result.config.name}'):")
    print(f"  K used / by the 95% rule   {ident.K} / {ident.diagnostics['K_energy_rule']}")
    print(f"  rate                       {summary['bpm_hat']:.3f} bpm")
    print(f"  R                          {summary['R']:.3g}  ({summary['R_source']})")
    print(f"  NIS mean                   {summary['nis_mean']:.3f}   (expected 1.0)")
    if summary["nis_mean"] > 3.0:
        print("    NIS well above 1: R is smaller than the innovations actually are. Expected "
              "with a free-air R, which cannot see the off-axis wobble of a loaded arm -- "
              "measure it in contact with a breath-hold segment to settle it.")
    elif summary["nis_mean"] < 0.3:
        print("    NIS well below 1: R is larger than the innovations actually are, so the "
              "filter is over-trusting its model and P understates the real uncertainty.")

    if h.forecast is None or h.forecast_target is None:
        return
    err = h.forecast - h.forecast_target
    rmse = float(np.sqrt(np.nanmean(err**2)))
    naive = float(np.sqrt(np.nanmean((h.y - h.forecast_target) ** 2)))
    print(f"\n  forecast at h = {h.horizon:.3f}s (the measured sensor lag), scored against "
          f"where the phantom actually is:")
    print(f"    forecast RMSE            {rmse:.4f} mm")
    print(f"    raw sensor RMSE          {naive:.4f} mm   (doing nothing, i.e. reading late)")
    print(f"    lag error removed        {100 * (1 - rmse / naive):.1f}%")
    if rows:
        # NOT a search for a "best" horizon -- h is set by the physics, not chosen. What the
        # sweep says is what the horizon COSTS: the error at h=0 is the filter's own
        # smoothing error, and the rise from there is the price of predicting ahead.
        at_zero = min(rows, key=lambda r: r["horizon"])
        at_h = min(rows, key=lambda r: abs(r["horizon"] - h.horizon))
        print(f"    cost of the horizon      {at_zero['rmse']:.4f} mm at h=0 -> "
              f"{at_h['rmse']:.4f} mm at h={at_h['horizon']:.2f}s "
              f"(+{at_h['rmse'] - at_zero['rmse']:.4f} mm, to buy back "
              f"{naive - rmse:.4f} mm of lag error)")


def _report_ground_truth(phantom: dict, metrics: dict | None, args, aligned_path: Path | None) -> None:
    feedback = phantom["summary"].get("replies_seen")
    if phantom["has_measured"]:
        hz = phantom["summary"].get("feedback_hz")
        rate = f" at {hz:.1f}Hz" if isinstance(hz, (int, float)) else ""
        print(f"\nphantom ground truth: measured_mm present ({feedback} status frame(s){rate})")
    else:
        print("\nphantom ground truth: commanded_mm only -- no motor feedback in this log. "
              "This measures the phantom AND the sensing chain together; re-run with the "
              "current run_breathing_profile.py to separate them.")

    if metrics is None:
        return
    scope = f"phase '{metrics['phase']}'" if metrics["phase"] else "the whole run"
    print(f"sensing vs {metrics['phantom_field_used']} over {scope} "
          f"({metrics['overlap_s']:.1f}s, {metrics['samples']} samples):")
    print(f"  lag              {metrics['lag_s'] * 1000:8.1f} ms   <- this is latency.tau_s")
    print(f"  amplitude ratio  {metrics['amplitude_ratio']:8.3f}      "
          f"<- fraction of real excursion the sensor sees")
    print(f"  correlation      {metrics['correlation']:8.3f}      (lag removed; "
          f"{metrics['correlation_unshifted']:.3f} unshifted)")
    print(f"  residual RMSE    {metrics['rmse_mm']:8.3f} mm   <- what is left for the EKF, "
          f"once lag and scale are taken out")
    if metrics["lag_at_search_edge"]:
        print(f"  WARNING: the lag peak is against the edge of the +/-{args.max_lag:.2f}s search "
              "range -- that is the range running out, not a measurement. Raise --max-lag.")
    if metrics["phase"] is None:
        print("  WARNING: --phase all, so this scored approach and seat too, where the base is "
              "moving and the sensor is not seated. That number is not meaningful.")
    if metrics["seams"]:
        print(f"  note: {metrics['seams']} profile loop-restart discontinuit(ies) in the window.")
    if aligned_path is not None:
        print(f"\nsaved: {aligned_path}")
        print("  run the estimator on it with:")
        print(f"    ct-identify --config bench_aligned --set source.params.path={aligned_path}")
        print("  or on truth_mm, for what a perfect sensor would have given the estimator:")
        print(f"    ct-identify --config bench_aligned --set source.params.path={aligned_path} \\\n"
              f"      --set source.params.y_column=truth_mm")


if __name__ == "__main__":
    raise SystemExit(main())
