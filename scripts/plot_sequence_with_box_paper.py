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

The payoff panel is the forecast. The sensor reads the phantom late, so forecasting by
exactly that lag should put the estimate back on top of where the phantom is *now*:

    sensor(t)             ~ ratio * truth(t - tau_s) + c
    forecast(t, h=tau_s)  ~ ratio * truth(t)         + c

If the pipeline works, the forecast tracks real-time truth while the raw sensor visibly
trails it. That is the one claim this whole repo exists to support, and it is directly
checkable on bench data.

**The lag is measured per run, not configured.** It used to be a frozen 0.677 s in
``configs/bench_aligned.yaml``, and it does not transfer between runs: across seven bench
runs the tactile chain lags 0.282-0.696 s, tracking how hard the arm is seated. Worse, most
of that is not sensor latency at all -- the ToF is non-contact but shares the bus, the tick
loop and the motion, and lags only 0.011-0.098 s. The rest is viscoelastic settling in the
contact. So the report prints the split, and the horizon is assembled from this run's own
numbers.

Two horizons are reported because two questions are being asked. ``h = tau_s`` is where the
forecast target is the phantom's position *now*, which is the claim above. ``h_control``
adds tau_c, tau_cl and T_ins and is what the rig will really use -- strictly harder, since
it aims past where lag-cancelling can reach.

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
    "insert_wait": "#fff2b3",
    "insert_drive": "#ffd699",
    "advance_wait": "#e0ccff",
    "advance_drive": "#c9a0ff",
    "insertion_complete": "#b3ffb3",
    "insertion_hold": "#b3ffb3",
    "needle_retract": "#ffb3d9",
    "retract_complete": "#b3ffb3",
}

DEFAULT_PHASE = "standoff_hold"  # the only phase where the sensor is seated and the base is still
# The tactile chain measures 0.28-0.70s across bench runs, so ct-compare's 1.0s default leaves
# little room. compare_logs clamps this to just inside half a breath anyway, and says when it does.
DEFAULT_MAX_LAG_S = 2.0
DEFAULT_ESTIMATOR_CONFIG = "bench_aligned"
DEFAULT_TAU_CL_S = 0.05   # LatencyConfig.tau_cl_fallback -- placeholder until the servo exists
DEFAULT_T_INS_S = 0.15    # LatencyConfig.T_ins -- placeholder until an insertion is timed


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


def _mark_needle_events(ax, summary: dict) -> None:
    """Overlay gate-fire (decision) vs. actual drive-start (activation) instants.

    Reads ``summary["insertion"]`` -- ``breakthrough.{fired_at,drive_started_at}`` and
    ``gated_advance.events[].{fired_at,drive_started_at}`` (run_approach_and_seat.py). Unlike
    scripts/plot_needle_gating_timing.py's offline diagnostic, which adds a placeholder
    re-arm delay to an assumed decision instant, ``drive_started_at`` here is the REAL elapsed
    time after ``reenter_needle_mode()``'s blocking re-arm sleep actually completes -- so the
    gap plotted is measured, not assumed.
    """
    insertion = summary.get("insertion") or {}
    if not insertion.get("enabled"):
        return

    fire_label_used = False
    activation_label_used = False

    def _mark(fired_at: float | None, drive_started_at: float | None) -> None:
        nonlocal fire_label_used, activation_label_used
        if fired_at is not None:
            ax.axvline(fired_at, color="C2", lw=1.2, ls="--", alpha=0.8,
                       label="_nolegend_" if fire_label_used else "gate fires (decision)")
            fire_label_used = True
        if drive_started_at is not None:
            ax.axvline(drive_started_at, color="C3", lw=1.2, ls=":", alpha=0.9,
                       label="_nolegend_" if activation_label_used
                       else "needle activates (measured)")
            activation_label_used = True

    # The breakthrough became abortable (and so multi-attempt) in session 023. Runs recorded
    # before that have only the scalar keys, so fall back to them rather than silently plotting
    # nothing for an older run's breakthrough.
    breakthrough = insertion.get("breakthrough", {})
    breakthrough_events = breakthrough.get("events")
    if breakthrough_events:
        for event in breakthrough_events:
            _mark(event.get("fired_at"), event.get("drive_started_at"))
    else:
        _mark(breakthrough.get("fired_at"), breakthrough.get("drive_started_at"))

    for event in insertion.get("gated_advance", {}).get("events", []):
        _mark(event.get("fired_at"), event.get("drive_started_at"))


def _mark_needle_floating(ax, records: list[dict], t: np.ndarray) -> None:
    """Shade every interval where the needle is NOT floating (re-arming or driving).

    Reads the per-tick ``needle_floating`` field directly (``None`` when ``--insert-needle``
    wasn't used -- a no-op then). This is a more precise signal than the phase shading: floating
    flips to ``False`` the instant ``reenter_needle_mode()`` is called, at the gate-fire tick --
    not a tick later when ``phase`` becomes ``insert_drive``/``advance_drive`` -- so it captures
    the re-arm window itself as "not floating", which the phase colors alone do not distinguish.
    Floating is the needle's default state for nearly this whole run, so shading the rare
    NOT-floating intervals is far less cluttered than shading floating itself.
    """
    floating = [r.get("needle_floating") for r in records]
    if all(v is None for v in floating):
        return

    label_used = False
    start_i = None
    for i, v in enumerate(floating):
        not_floating = v is False
        if not_floating and start_i is None:
            start_i = i
        elif not not_floating and start_i is not None:
            ax.axvspan(t[start_i], t[i - 1], color="#b30000", alpha=0.35, zorder=1,
                       label="_nolegend_" if label_used else "needle NOT floating (re-arm + drive)")
            label_used = True
            start_i = None
    if start_i is not None:
        ax.axvspan(t[start_i], t[-1], color="#b30000", alpha=0.35, zorder=1,
                   label="_nolegend_" if label_used else "needle NOT floating (re-arm + drive)")


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
    ax.legend(lines, [line.get_label() for line in lines], loc="upper left", fontsize=8, framealpha=0.75)


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
    ax.legend(loc="upper left", fontsize=8, ncol=3, framealpha=0.75)

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
    # The ribbon, the EKF line and the calibration divider used to be three different C2
    # things, so the uncertainty band read as a halo around its own line and the sensor
    # underneath was invisible. Neutral fill, dark line on top, neutral divider.
    ax.fill_between(te, h.y_pred - sigma, h.y_pred + sigma, color="0.55", alpha=0.30,
                    lw=0, label="+-1 sigma (sqrt S)", zorder=2)
    # The EKF tracks the sensor almost exactly, which is the result -- but at equal weights
    # the top line simply hides the one underneath and the panel looks like a single trace.
    # Sensor as a wide pale band, EKF as a thin dark line riding inside it.
    ax.plot(te, h.y, lw=2.6, color="C0", alpha=0.35, label="sensor tactile_mm", zorder=3)
    ax.plot(te, h.y_pred, lw=1.0, color="#0b6b3a", label=f"EKF y_pred (K={result.ident.K})",
            zorder=4)
    # Everything before this is Stage 1's calibration window; the filter has seen none of it.
    calib_end = window_t0 + float(result.calibration.t[-1])
    ax.axvline(calib_end, color="0.25", lw=1.2, ls=(0, (6, 3)), alpha=0.8, label="calib -> track")
    centre, limits = _centre_and_limits(te, h.y_pred, span)
    ax.set_ylim(limits[0] + centre, limits[1] + centre)
    ax.set_ylabel("EKF [mm]")
    ax.grid(alpha=0.3)
    _frame_span(ax, span)
    ax.legend(loc="upper left", fontsize=8, ncol=2, framealpha=0.75)


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

    # Three near-identical waveforms braided together at similar weight was the worst of the
    # readability problems. Give them a clear hierarchy: truth heavy and solid underneath,
    # the sensor thin and faded, the forecast dashed in a hue that reads against the truth's
    # red (C4 purple did not).
    ax.plot(te, h.y, lw=0.8, color="C0", alpha=0.45,
            label=f"sensor (trails truth by {lag_s * 1000:.0f}ms)", zorder=2)
    ax.plot(te, h.forecast_target, lw=2.0, color="C3", alpha=0.9,
            label="phantom position NOW", zorder=3)
    ax.plot(te, h.forecast, lw=1.4, color="#00204d", ls=(0, (5, 2)),
            label=f"forecast at h={h.horizon:.3f}s", zorder=4)
    err = h.forecast - h.forecast_target
    centre, limits = _centre_and_limits(te, h.forecast_target, span)
    ax.set_ylim(limits[0] + centre, limits[1] + centre)
    ax.set_ylabel("forecast [mm]")
    ax.grid(alpha=0.3)
    _frame_span(ax, span)
    rmse = float(np.sqrt(np.nanmean(err**2)))
    naive = float(np.sqrt(np.nanmean((h.y - h.forecast_target) ** 2)))
    ax.set_title(f"forecast RMSE {rmse:.4f} mm vs {naive:.4f} mm for the raw sensor "
                 f"— {100 * (1 - rmse / naive):.0f}% of the lag error removed",
                 fontsize=9, loc="left")
    ax.legend(loc="upper left", fontsize=8, ncol=3, framealpha=0.75)


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
               label=f"assembled horizon ({cfg.horizon:.3f}s)")
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


def measured_tau_c(records: list[dict], span: tuple[float, float]) -> float:
    """The loop's own p95 tick interval over the analysis window.

    The same statistic ``ct.rt.latency.LatencyEstimator`` uses at runtime, computed here
    from the log because this script has no live loop. p95 rather than the mean because
    the horizon has to cover a slow tick, not a typical one.
    """
    t = np.array([r["elapsed"] for r in records
                  if span[0] <= r.get("elapsed", -1) <= span[1]], dtype=float)
    if t.size < 10:
        return 0.0
    return float(np.quantile(np.diff(t), 0.95))


def build_horizon(metrics: dict | None, records: list[dict], span: tuple[float, float], args):
    """The two horizons that matter, from THIS run's own measured lag.

    The horizon used to be a frozen ``0.677`` in ``configs/bench_aligned.yaml`` -- one run's
    figure, reused for every later run. It does not transfer. The sensing lag tracks how hard
    the tactile arm is seated (0.279-0.566 s over six bench runs, correlation +0.63 with seat
    depth), so a constant is right for at most one of them: on run 20260902-160517, whose real
    lag is 0.279 s, forecasting at 0.677 s scored *worse* than not forecasting at all.

    Two horizons come out because two different questions are being asked, and conflating
    them is what made the forecast panel read as a failure:

    - ``h_now = tau_sensor``. ``forecast_target`` is ``y_clean(t+h)``, and ``y_clean`` is
      truth delayed by the measured lag -- so at ``h = lag`` the target is the phantom's
      position *right now*. This is the pipeline's headline claim, and the only horizon at
      which "put the late sensor back into the present" is what is being tested.
    - ``h_control``, all four terms. What the rig will really forecast at, because by the
      time the needle arrives the phantom has moved on again. Strictly harder: the target is
      ``h_control - lag`` into the genuine future, which no amount of lag-cancelling reaches.

    Only ``tau_sensor`` and ``tau_compute`` are measured. ``tau_cl`` needs a servo design and
    ``T_ins`` needs a timed insertion; both are still placeholders -- see ``ct-unknowns``.
    They are in the sum anyway, because a term left out of ``h`` is a silent error where a
    named placeholder is a visible one.
    """
    from ct.forecast import horizon_from_components  # noqa: PLC0415

    tau_s = max(float(metrics["lag_s"]) if metrics else 0.0, 0.0)
    terms = {
        "tau_sensor": tau_s,
        "tau_compute": measured_tau_c(records, span),
        "tau_closed_loop": args.tau_cl,
        "T_insertion": args.T_ins,
    }
    return tau_s, horizon_from_components(**terms), terms


def tof_reference(phantom: dict, jsonl_path: Path, phase: str | None, args, metrics):
    """The non-contact lag over the same window, for splitting sensing from contact.

    Optional in every sense: older logs have no ``tof_mm``, and a run that produced no
    tactile metrics has nothing to split. Never costs the figure.
    """
    if metrics is None:
        return None
    try:
        return compare_logs(phantom["path"], jsonl_path, max_lag_s=args.max_lag,
                            phase=phase, phantom_field=metrics["phantom_field_used"],
                            sensor_field="tof_mm")
    except (ValueError, KeyError):
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
    # Bracket the horizon actually in use rather than a fixed 0..1.5s: the point of the sweep
    # is what h costs around where h really is.
    sweep = np.linspace(0.0, max(2.5 * cfg.horizon, 0.3), 16)

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
        rows = sweep_horizons(cfg, [float(h) for h in sweep])
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
                         help="forecast horizon [s]. Default is assembled from this run's own "
                              "measured sensor lag plus tau_c, tau_cl and T_ins -- NOT a "
                              "constant, because the sensing lag moves with seating depth")
    parser.add_argument("--tau-cl", dest="tau_cl", type=float, default=DEFAULT_TAU_CL_S,
                         help="residual closed-loop servo lag [s]; placeholder until the "
                              "needle servo is designed")
    parser.add_argument("--T-ins", dest="T_ins", type=float, default=DEFAULT_T_INS_S,
                         help="insertion duration [s]; placeholder until one is timed")
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
    metrics = tof_metrics = None
    if phantom is not None:
        metrics = ground_truth_metrics(phantom, jsonl_path, phase, args)
        tof_metrics = tof_reference(phantom, jsonl_path, phase, args, metrics)
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
    # NOT sharex. The estimator panels only carry data over the analysis window -- ~90s of a
    # ~300s run -- so sharing the axis stretched them across three times their own span and
    # squeezed every breath into a few pixels. They get cropped to their own window below.
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 2.6 * n_panels), sharex=False)

    _shade_phases(axes[0], t, phases)
    axes[0].plot(t, dist_cm, lw=0.8, label="dist_cm")
    if contact_threshold_cm is not None:
        axes[0].axhline(contact_threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6, label="contact threshold")
        axes[0].axhline(-contact_threshold_cm, color="k", lw=0.6, ls=":", alpha=0.6)
    if standoff_dist_cm is not None:
        axes[0].axhline(standoff_dist_cm, color="tab:blue", lw=0.6, ls="-.", alpha=0.7, label="standoff threshold")
        axes[0].axhline(-standoff_dist_cm, color="tab:blue", lw=0.6, ls="-.", alpha=0.7)
    _mark_needle_floating(axes[0], records, t)
    _mark_needle_events(axes[0], summary)
    axes[0].set_ylabel("dist_cm [cm]")
    axes[0].legend(loc="upper left", fontsize=8, framealpha=0.75)
    axes[0].grid(alpha=0.3)
    axes[0].set_title(title)

    _shade_phases(axes[1], t, phases)
    axes[1].plot(t, tof_mm, lw=0.8, label="tof_mm", color="C1")
    axes[1].set_ylabel("tof_mm [mm]")
    axes[1].legend(loc="upper left", fontsize=8, framealpha=0.75)
    axes[1].grid(alpha=0.3)

    aligned_path: Path | None = None
    if phantom is not None:
        _plot_ground_truth(axes[2], t, phases, tactile_mm, phantom, span)
        aligned_path = _plot_reconstruction(
            axes[3], t, phases, tactile_mm, phantom, metrics, span, run_dir,
        )

    result = rows = None
    horizon, h_control, terms = build_horizon(metrics, records, span, args)
    if args.horizon is not None:
        horizon = float(args.horizon)
    if estimator_panels and aligned_path is not None:
        result, rows, est_cfg = run_estimator(aligned_path, args.estimator_config, horizon)
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

    # Diagnostic panels span the whole run; estimator panels span only what they describe --
    # which is not the analysis window but the *tracked* part of it, since Stage 1 eats the
    # first calib_seconds. On a 180s hold with a 90s calibration that is half the panel.
    run_span = (float(t[0]), float(t[-1]))
    est_span = span
    if result is not None:
        te = result.history.t + span[0]
        est_span = (float(te[0]), float(te[-1]))
    estimator_first = n_panels - estimator_panels
    for i, ax in enumerate(axes):
        ax.set_xlim(*(est_span if estimator_panels and i >= estimator_first else run_span))
    if estimator_panels:
        # The x-axis changes partway down the stack, which is worth saying out loud.
        axes[estimator_first - 1].set_xlabel("t [s]  (panels below are cropped to the "
                                             f"'{phase}' window)", fontsize=8)

    fig.tight_layout()
    out_path = run_dir / "sequence_patient1.pdf"
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
        _report_ground_truth(phantom, metrics, args, tof_metrics)
        if metrics is not None:
            _report_metric_warnings(metrics, args, aligned_path)
        _report_seating_quality(metrics, seat)
    if result is not None:
        _report_horizon(horizon, h_control, terms, args)
        _report_estimator(result, rows, metrics)
    return 0


def _report_seating_quality(metrics: dict | None, seat: dict) -> None:
    """Seating depth is the dominant lever on both the lag and the amplitude ratio.

    Across six bench runs, correlation with the accepted seat peak is +0.63 for the lag and
    -0.84 for the amplitude ratio: pressing harder does not couple more stiffly, it pushes
    into a softer, more dissipative regime and loads the pivot harder. Worth saying at the
    bench, where it can still be acted on, rather than only in a later comparison.
    """
    peak = seat.get("accepted_peak_tactile_mm")
    if metrics is None or peak is None:
        return
    print(f"\nseating quality: seat peak {peak:.3f} mm -> lag {metrics['lag_s'] * 1000:.0f} ms, "
          f"amplitude {metrics['amplitude_ratio']:.3f}, r {metrics['correlation']:.3f}")
    if peak > 1.0 or metrics["amplitude_ratio"] < 0.5:
        print("  Seated hard. Seat LIGHTER: the best bench run seated to 0.36mm and got "
              "0.28s / 0.76 / r=0.99,\n  the worst seated to 1.93mm and got 0.57s / 0.16 / "
              "r=0.81. This is worth more than any\n  amount of forecasting.")


def _report_horizon(horizon: float, h_control: float, terms: dict, args) -> None:
    """The four terms of h, which are measured, and the two horizons they make."""
    if args.horizon is not None:
        print(f"\nhorizon h = {horizon:.3f}s (forced by --horizon)")
        return
    print(f"\nhorizon h = {horizon:.3f}s -- this run's own measured sensing lag, not a "
          "configured constant.")
    print("  At this h the forecast target is where the phantom is NOW, so the panel above "
          "tests\n  exactly one thing: whether a signal read late can be put back into the "
          "present.")
    print(f"\n  the rig's real horizon is larger, h_control = {h_control:.3f}s:")
    print(f"    tau_sensor       {terms['tau_sensor']:.4f} s   MEASURED (this run's sensing lag)")
    print(f"    tau_compute      {terms['tau_compute']:.4f} s   MEASURED (p95 tick interval)")
    print(f"    tau_closed_loop  {terms['tau_closed_loop']:.4f} s   placeholder (no servo design yet)")
    print(f"    T_insertion      {terms['T_insertion']:.4f} s   placeholder (no insertion timed yet)")
    print(f"  Forecasting at h_control aims {h_control - terms['tau_sensor']:.3f}s into the "
          "genuine future, past\n  where lag-cancelling can reach -- see the sweep for what "
          "that costs. Run with\n  --horizon to score it directly.")


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
    _report_q_diagnostics(ident.diagnostics.get("q", {}))
    _report_frequency_lock(result)

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


def _report_q_diagnostics(q_info: dict) -> None:
    """Was Q measured off a healthy number of breaths, or did it fall back?

    ``estimate_Q`` (``ct.identification.noise``) already computes ``n_breaths`` and
    ``omega_source`` and hands them back in ``ident.diagnostics['q']`` -- this just makes them
    visible. A Q estimated from too few breaths is silently noisier without this line, and the
    NIS/frequency-lock numbers above it are only as trustworthy as this is.
    """
    if not q_info:
        return
    n_breaths = q_info.get("n_breaths")
    omega_source = q_info.get("omega_source", "?")
    print(f"  Q calibration              {n_breaths} breaths, "
          f"omega variance from {omega_source}")
    if q_info.get("warning"):
        print(f"    {q_info['warning']}")


def _report_frequency_lock(result) -> None:
    """Did the tracker hold the frequency Stage 1 handed it?

    This is the one diagnostic that ``y_pred`` cannot show. ``A_k``, ``phi_k`` and
    ``(theta, omega_r)`` are partially redundant: a steadily drifting ``phi_1`` mimics a
    frequency offset, so the filter can fit the measurement *perfectly* while holding an
    ``omega_r`` that is badly wrong. The tracking panel then looks flawless and only the
    forecast suffers -- because forecasting is the one operation that uses ``omega_r`` on
    its own, advancing ``theta`` by ``omega_r * h``.

    Observed on real bench data: run 20260902-160517 tracks at 4.55 +- 3.60 bpm against a
    true 10.34, with ``phi_1`` spinning 9.70 rad to compensate. Its forecast advances only
    ~0.03s of a requested 0.279s. Run 20260901-165415 holds 11.96 +- 1.18 against 12.19,
    ``phi_1`` drifts 0.15 rad, and its forecast works.
    """
    from ct.diagnostics.metrics import is_frequency_locked  # noqa: PLC0415
    from ct.layout import StateLayout  # noqa: PLC0415

    s = np.asarray(result.history.s)
    layout = StateLayout(result.ident.K)
    to_bpm = 60.0 / (2.0 * np.pi)
    ident_bpm = float(result.ident.s0[layout.omega]) * to_bpm
    tracked = s[:, layout.omega] * to_bpm
    phi_drift = float(np.ptp(np.unwrap(s[:, layout.phi(1)])))
    mean, std = float(tracked.mean()), float(tracked.std())
    print(f"  frequency lock             {mean:.2f} +- {std:.2f} bpm tracked vs "
          f"{ident_bpm:.2f} from Stage 1")
    if is_frequency_locked(mean, std, ident_bpm):
        return
    print(f"    LOST FREQUENCY LOCK. phi_1 drifted {phi_drift:.2f} rad absorbing the error, "
          "which is why\n    y_pred still fits and only the forecast is wrong: it advances "
          f"theta by omega*h, so at\n    {mean:.2f} bpm instead of {ident_bpm:.2f} it moves "
          f"~{mean / ident_bpm:.0%} of the horizon it was asked for.\n    Q and the "
          "phi/omega observability trade are the suspects, not R.")


def _report_ground_truth(phantom: dict, metrics: dict | None, args, tof_metrics: dict | None) -> None:
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
    print(f"  lag              {metrics['lag_s'] * 1000:8.1f} ms   <- WHOLE sensing lag, "
          f"mostly contact settling")
    print(f"  amplitude ratio  {metrics['amplitude_ratio']:8.3f}      "
          f"<- fraction of real excursion the sensor sees")
    print(f"  correlation      {metrics['correlation']:8.3f}      (lag removed; "
          f"{metrics['correlation_unshifted']:.3f} unshifted)")
    print(f"  residual RMSE    {metrics['rmse_mm']:8.3f} mm   <- what is left for the EKF, "
          f"once lag and scale are taken out")
    _report_latency_split(metrics, tof_metrics)


def _report_latency_split(metrics: dict, reference: dict | None) -> None:
    """How much of the sensing lag is the sensor, and how much is the contact.

    The ToF is non-contact but shares the bus, the tick loop and the motion, so it measures
    the sensing chain on its own. Everything above it is mechanical.
    """
    if reference is None:
        return
    floor, total = reference["lag_s"], metrics["lag_s"]
    print("\n  where that lag comes from:")
    print(f"    sensing floor    {floor * 1000:8.1f} ms   (tof_mm: non-contact, same bus and "
          f"tick, sees {reference['amplitude_ratio']:.2f} of excursion)")
    print(f"    contact excess   {(total - floor) * 1000:8.1f} ms   (viscoelastic settling in "
          f"the contact -- NOT sensor latency)")
    print("    The floor is an upper bound: the ToF quantises to 1mm on a ~4.8mm excursion. "
          "It is\n    consistent with the ~8ms frame period plus the ~11ms tick. Only the "
          "floor belongs in\n    latency.tau_s; the excess is a property of how the arm is "
          "seated, and moves with it.")


def _report_metric_warnings(metrics: dict, args, aligned_path: Path | None) -> None:
    if metrics["lag_at_search_edge"]:
        print(f"  WARNING: the lag peak is against the edge of the +/-{args.max_lag:.2f}s search "
              "range -- that is the range running out, not a measurement. Raise --max-lag.")
    if metrics["lag_ambiguous"]:
        print(f"  WARNING: --max-lag {args.max_lag:.2f}s exceeds half the "
              f"{metrics['breath_period_s']:.2f}s breath, so the search was clamped to "
              f"+/-{metrics['max_lag_searched_s']:.2f}s. Past that a lag cannot be told apart "
              "from the same lag one breath over.")
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
