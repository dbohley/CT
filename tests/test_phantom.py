"""Aligning a phantom log against a controller log.

``compare_logs`` is how ``latency.tau_s`` gets measured rather than assumed, and how the
sensing chain's amplitude loss gets quantified — so the thing to test is that it recovers a
lag and a scale that were deliberately injected, and that its two selectors (which phase to
score, and what counts as ground truth) actually change the answer.

The phase selector matters more than it looks. On the real bench run 20260901-165415,
scoring the whole run gives correlation 0.039 and scoring ``standoff_hold`` alone gives
0.599 — because approach and seat are phases where the base is moving and the sensor is not
yet seated. ``test_phase_filter_rescues_a_run_polluted_by_other_phases`` reproduces that
structure synthetically.
"""

from __future__ import annotations

import json
import math

import pytest

from ct.phantom.driver import compare_logs, count_seams

FS = 100.0
BREATH_S = 4.0


def _write(path, records):
    path.write_text("\n".join(json.dumps(r) for r in records) + "\n")
    return path


def _breath(t: float) -> float:
    return math.sin(2 * math.pi * t / BREATH_S)


def _logs(tmp_path, *, duration=40.0, lag_s=0.0, scale=1.0, phase="standoff_hold",
          noise_phases=(), measured_offset=None, t0=1000.0, tof_lag_s=None):
    """A phantom log and a controller log whose sensor lags and scales the phantom.

    ``noise_phases`` prepends records in other phases carrying a signal uncorrelated with
    the phantom, which is what a real approach/seat segment looks like to this function.
    """
    n = int(duration * FS)
    phantom = []
    for i in range(n):
        t = t0 + i / FS
        commanded = _breath(t - t0)
        record = {"t": t, "elapsed": i / FS, "commanded_mm": commanded}
        if measured_offset is not None:
            record["measured_mm"] = commanded * measured_offset
        phantom.append(record)

    controller = []
    for name in noise_phases:
        for i in range(int(8.0 * FS)):
            t = t0 + i / FS
            controller.append({"t": t, "elapsed": i / FS, "phase": name,
                               "tactile_mm": 5.0 * math.sin(2 * math.pi * t / 0.7)})
    for i in range(n):
        t = t0 + i / FS
        record = {"t": t, "elapsed": i / FS, "phase": phase,
                  "tactile_mm": 10.0 + scale * _breath(t - t0 - lag_s)}
        if tof_lag_s is not None:
            # The ToF measures DISTANCE, so it moves opposite to the surface it watches.
            record["tof_mm"] = 130.0 - _breath(t - t0 - tof_lag_s)
        controller.append(record)

    return (_write(tmp_path / "phantom.jsonl", phantom),
            _write(tmp_path / "controller.jsonl", controller))


# -- the two headline numbers -------------------------------------------------


def test_recovers_an_injected_lag_and_amplitude_ratio(tmp_path):
    phantom, controller = _logs(tmp_path, lag_s=0.2, scale=0.3)
    result = compare_logs(phantom, controller, phase="standoff_hold")

    assert result["lag_s"] == pytest.approx(0.2, abs=1.5 / FS)
    assert result["amplitude_ratio"] == pytest.approx(0.3, abs=0.01)
    assert result["phantom_field_used"] == "commanded_mm"


def test_zero_lag_and_unit_scale_come_back_clean(tmp_path):
    phantom, controller = _logs(tmp_path)
    result = compare_logs(phantom, controller, phase="standoff_hold")

    assert result["lag_s"] == pytest.approx(0.0, abs=1.5 / FS)
    assert result["correlation"] > 0.99
    assert result["rmse_mm"] < 0.05


@pytest.mark.parametrize("lag_s", [0.0, 0.3, 0.7, 1.1])
def test_amplitude_ratio_does_not_depend_on_the_lag(tmp_path, lag_s):
    """Attenuation and delay are separate effects and must be reported separately.

    Projecting the sensor onto an *unshifted* phantom folds the delay into the scale: a
    0.677s lag against a ~4.9s breath costs a factor of cos(2*pi*0.677/4.93) ~ 0.63, which
    is exactly why the real bench run once reported an amplitude ratio of 0.203 where the
    true figure — confirmed independently by Stage 1's A_1 fit, 0.346mm sensed against
    1.063mm of truth — is 0.326.
    """
    phantom, controller = _logs(tmp_path, lag_s=lag_s, scale=0.3)
    result = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=2.0)

    assert result["lag_s"] == pytest.approx(lag_s, abs=1.5 / FS)
    assert result["amplitude_ratio"] == pytest.approx(0.3, abs=0.01)
    assert result["correlation"] > 0.99
    assert result["rmse_mm"] < 0.05


def test_unshifted_correlation_is_reported_separately_and_is_worse(tmp_path):
    phantom, controller = _logs(tmp_path, lag_s=0.7, scale=0.3)
    result = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=2.0)

    assert result["correlation"] > 0.99
    assert result["correlation_unshifted"] < 0.7


# -- the phase selector -------------------------------------------------------


def test_phase_filter_rescues_a_run_polluted_by_other_phases(tmp_path):
    """Scoring every phase at once does not weaken the answer, it replaces it."""
    phantom, controller = _logs(tmp_path, scale=0.3, noise_phases=("approach", "seat"))

    whole_run = compare_logs(phantom, controller)
    hold_only = compare_logs(phantom, controller, phase="standoff_hold")

    assert hold_only["correlation"] > 0.99
    assert whole_run["correlation"] < hold_only["correlation"]
    assert whole_run["phase"] is None
    assert hold_only["phase"] == "standoff_hold"


def test_unknown_phase_is_an_error_not_a_silent_empty_answer(tmp_path):
    phantom, controller = _logs(tmp_path)
    with pytest.raises(ValueError, match="advance"):
        compare_logs(phantom, controller, phase="advance")


# -- what counts as ground truth ----------------------------------------------


def test_measured_is_preferred_when_present_and_commanded_when_not(tmp_path):
    phantom, controller = _logs(tmp_path, scale=0.3, measured_offset=0.5)
    measured = compare_logs(phantom, controller, phase="standoff_hold", phantom_field="auto")
    assert measured["phantom_field_used"] == "measured_mm"
    # The motor moved half as far as commanded, so the same sensor reading is twice the
    # fraction of it -- which is exactly why the two must not be conflated.
    assert measured["amplitude_ratio"] == pytest.approx(0.6, abs=0.05)

    commanded = compare_logs(phantom, controller, phase="standoff_hold", phantom_field="commanded_mm")
    assert commanded["phantom_field_used"] == "commanded_mm"
    assert commanded["amplitude_ratio"] == pytest.approx(0.3, abs=0.05)


def test_auto_falls_back_to_commanded_on_a_log_with_no_feedback(tmp_path):
    """Logs written before the phantom script recorded feedback must still compare."""
    phantom, controller = _logs(tmp_path, scale=0.3, measured_offset=None)
    result = compare_logs(phantom, controller, phase="standoff_hold", phantom_field="auto")
    assert result["phantom_field_used"] == "commanded_mm"


def test_asking_for_measured_when_there_is_none_says_so(tmp_path):
    phantom, controller = _logs(tmp_path, measured_offset=None)
    with pytest.raises(ValueError, match="measured_mm"):
        compare_logs(phantom, controller, phase="standoff_hold", phantom_field="measured_mm")


# -- splitting sensing latency from contact settling --------------------------


def test_the_tof_column_can_be_scored_and_its_inverted_polarity_is_handled(tmp_path):
    """The ToF is what separates sensor latency from contact settling.

    It is non-contact but shares the bus, the tick loop and the motion, so on the bench it
    lags 0.014-0.100s where the tactile arm lags 0.279-0.566s -- and the difference is
    mechanical, not electronic. Its distance reading moves *opposite* to the surface, which
    has to be handled by orientation rather than discovered: for a near-sinusoid an inverted
    sensor is indistinguishable from a correctly-signed one half a breath away, and a
    magnitude-based search picks whichever lobe is nearer.
    """
    phantom, controller = _logs(tmp_path, lag_s=0.5, scale=0.3, tof_lag_s=0.05)

    tactile = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=1.0)
    tof = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=1.0,
                       sensor_field="tof_mm")

    assert tactile["lag_s"] == pytest.approx(0.5, abs=1.5 / FS)
    assert tof["lag_s"] == pytest.approx(0.05, abs=1.5 / FS)
    assert tof["sensor_field_used"] == "tof_mm"
    # Oriented to the phantom, so both read as positive fractions of real excursion and are
    # directly comparable -- the whole point of measuring the floor this way.
    assert tof["sensor_sign"] == -1.0
    assert tof["amplitude_ratio"] == pytest.approx(1.0, abs=0.05)
    assert tof["correlation"] > 0.99
    # And the split itself: everything above the floor is contact, not sensing.
    assert tactile["lag_s"] - tof["lag_s"] == pytest.approx(0.45, abs=0.03)


def test_asking_for_a_sensor_column_that_is_not_there_says_so(tmp_path):
    phantom, controller = _logs(tmp_path)
    with pytest.raises(ValueError, match="tof_mm"):
        compare_logs(phantom, controller, phase="standoff_hold", sensor_field="tof_mm")


# -- guards -------------------------------------------------------------------


def test_the_lag_is_resolved_finer_than_the_sample_grid(tmp_path):
    """A raw argmax quantises the lag to one grid sample; the horizon deserves better.

    At the bench's ~127 Hz that quantum is 7.9 ms. It was tolerable while the lag was a
    curiosity and is not now that it sets the forecast horizon per run.
    """
    phantom, controller = _logs(tmp_path, lag_s=0.235, scale=0.3)
    result = compare_logs(phantom, controller, phase="standoff_hold")

    assert result["lag_s"] == pytest.approx(0.235, abs=0.5 / FS)
    # Strictly better than the grid could express on its own.
    assert result["lag_s"] not in (0.23, 0.24)


def test_a_search_wider_than_half_a_breath_is_clamped_and_flagged(tmp_path):
    """Breathing is periodic, so the correlation surface is too.

    There is a sidelobe every T_breath and an argmax has no way to prefer the true one. A
    +-3s scan of the ToF against this bench's ~5.8s breathing came back with -2.77s doing
    exactly that. The range is clamped to just inside T/2 and the caller is told.
    """
    phantom, controller = _logs(tmp_path, lag_s=0.3, scale=0.3)

    wide = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=3.0)

    assert wide["breath_period_s"] == pytest.approx(BREATH_S, rel=0.05)
    assert wide["lag_ambiguous"] is True
    assert wide["max_lag_searched_s"] < 0.5 * BREATH_S
    # Clamped, but still correct -- the guard narrows the search, it does not break it.
    assert wide["lag_s"] == pytest.approx(0.3, abs=1.5 / FS)

    narrow = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=1.0)
    assert narrow["lag_ambiguous"] is False





def test_lag_peak_against_the_search_edge_is_flagged(tmp_path):
    """A peak at the edge of the range is the range running out, not a measurement."""
    phantom, controller = _logs(tmp_path, lag_s=0.5)

    tight = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=0.5)
    assert tight["lag_at_search_edge"] is True

    roomy = compare_logs(phantom, controller, phase="standoff_hold", max_lag_s=2.0)
    assert roomy["lag_at_search_edge"] is False
    assert roomy["lag_s"] == pytest.approx(0.5, abs=1.5 / FS)


def test_non_overlapping_logs_are_an_error(tmp_path):
    """Two runs that were not live at the same time cannot be aligned by a shared clock."""
    phantom, _ = _logs(tmp_path, t0=1000.0)
    other = tmp_path / "other"
    other.mkdir()
    _, controller = _logs(other, t0=9000.0)
    with pytest.raises(ValueError, match="overlap"):
        compare_logs(phantom, controller)


def test_count_seams_finds_profile_loop_restarts():
    import numpy as np

    t = np.arange(0, 12.0, 1 / FS)
    smooth = np.sin(2 * np.pi * t / BREATH_S)
    y = smooth.copy()
    # Three restarts, each a jump far larger than any step the waveform itself takes.
    for i in (200, 500, 900):
        y[i:] += 5.0
    assert count_seams(y) == 3
    assert count_seams(smooth) == 0


def test_forecasting_by_the_measured_lag_recovers_real_time_truth(tmp_path):
    """The claim the whole pipeline exists to support, on a signal with a known lag.

    A sensor that reads the patient ``tau_s`` late is useless for gating unless something
    puts the reading back in the present. Forecasting the tracked model forward by exactly
    ``tau_s`` is that something: ``sensor(t) ~ ratio*truth(t-tau_s)+c``, so a forecast of the
    sensor at ``t+tau_s`` is ``ratio*truth(t)+c`` -- where the patient is *now*. This asserts
    the forecast beats simply reading the sensor, which is the do-nothing baseline. Measured
    at 62% of the lag error removed on real bench data (run 20260901-165415).
    """
    import numpy as np

    from ct.config import RunConfig
    from ct.run import run_pipeline

    tau_s, ratio, offset, fs = 0.7, 0.33, 5.0, 100.0
    t = np.arange(0.0, 240.0, 1 / fs)
    # Two harmonics, so the forecast has to rotate harmonic k by k*omega*h rather than
    # applying one common phase shift -- the error CLAUDE.md's settled decision 4 forbids.
    truth = np.sin(2 * np.pi * t / 5.0) + 0.3 * np.sin(4 * np.pi * t / 5.0 + 0.7)
    clean = ratio * np.interp(t, t + tau_s, truth) + offset
    rng = np.random.default_rng(0)
    sensor = clean + rng.normal(0.0, 0.005, t.size)

    csv = tmp_path / "aligned.csv"
    np.savetxt(csv, np.column_stack([t, sensor, clean]), delimiter=",",
               header="time_s,sensor_mm,y_clean", comments="", fmt="%.6f")

    cfg = RunConfig.from_yaml("bench_aligned")
    cfg.source = {**cfg.source, "params": {**cfg.source["params"], "path": str(csv)}}
    cfg.fs, cfg.horizon, cfg.calib_seconds, cfg.duration = fs, tau_s, 90.0, 240.0
    result = run_pipeline(cfg)

    h = result.history
    warmup = int(result.summary["warmup_steps"])
    err = h.forecast[warmup:] - h.forecast_target[warmup:]
    forecast_rmse = float(np.sqrt(np.nanmean(err**2)))
    # Doing nothing: read the sensor and treat it as the current position.
    naive_rmse = float(np.sqrt(np.nanmean((h.y[warmup:] - h.forecast_target[warmup:]) ** 2)))

    assert forecast_rmse < 0.5 * naive_rmse, (
        f"forecast RMSE {forecast_rmse:.4f} did not beat the raw sensor's {naive_rmse:.4f}"
    )


def test_count_seams_survives_a_zero_order_hold_staircase():
    """``measured_mm`` is held between status broadcasts, so most steps are exactly zero.

    A median-based threshold reads every ordinary riser as an enormous multiple of it and
    reports hundreds of phantom seams; the p95 reference does not.
    """
    import numpy as np

    t = np.arange(0, 12.0, 1 / FS)
    smooth = np.sin(2 * np.pi * t / BREATH_S)
    staircase = smooth[(np.arange(t.size) // 4) * 4]  # one real sample in four
    assert count_seams(staircase) == 0

    with_seams = staircase.copy()
    for i in (200, 500, 900):
        with_seams[i:] += 5.0
    assert count_seams(with_seams) == 3
