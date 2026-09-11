"""Servo, firing gate, safety monitor, and the live signal accumulator.

The gate tests are the ones worth reading. They pin the claim the paper makes: an EKF
gives an explicit covariance, and the gate spends it. An LMS predictor could answer "where
will the skin be"; it could not answer "and how sure are you", which is the question that
decides whether a needle moves.
"""

from __future__ import annotations

import numpy as np
import pytest

from ct.control.gate import FiringGate, cycle_extrema
from ct.control.live import AmplitudeWatcher, BreathPeakWatcher, SignalAccumulator
from ct.control.safety import SafetyMonitor
from ct.control.servo import LeadServo, bandwidth, residual_lag
from ct.control.state import ProcedureState, next_in_sequence
from ct.geometry import RigState
from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel, SafetyConfig
from ct.layout import StateLayout


# -- servo --------------------------------------------------------------------


@pytest.fixture
def servo_config():
    return AxisServoConfig(
        plant=PlantModel(K=1.0, wn=60.0, zeta=0.7),
        lead=LeadCompensator(zero=8.0, pole=80.0, gain=150.0),
    )


def test_a_lead_network_must_actually_lead():
    """``pole <= zero`` is a lag network — it would add delay where the design wants lead."""
    with pytest.raises(ValueError, match="pole > zero"):
        LeadCompensator(zero=80.0, pole=8.0)


def test_residual_lag_is_a_real_positive_lag_that_varies_with_rate(servo_config):
    """The settled decision, made concrete.

    ``tau_cl`` is the residual closed-loop tracking lag, so it must be a genuine lag and
    it must depend on the frequency being tracked. A constant would be the fixed ``tau_a``
    the design explicitly rejects.

    Note it varies only weakly across the breathing band, which is what a type-1 loop does
    — phase lag grows roughly linearly with ``omega``, so ``phase/omega`` is nearly flat.
    Weakly varying is still varying; it is not a constant, and it is not this module's job
    to pretend the dependence is stronger than the servo makes it.
    """
    omegas = (0.5, 1.0, 1.5, 2.0, 3.0)
    lags = [residual_lag(servo_config, omega) for omega in omegas]
    assert all(lag > 0 for lag in lags), "a lead-compensated type-1 loop lags; it does not lead"
    assert len(set(lags)) == len(lags), "tau_cl must depend on omega, not be a constant"


def test_the_loop_actually_tracks_at_breathing_frequencies(servo_config):
    """``|T| ~ 1`` across the band, or 'residual tracking lag' means nothing.

    An early version modelled the axis as type-0 and got ``|T| = 0.55`` — the needle would
    have followed barely half the skin's motion, and no forecast horizon could have fixed
    that.
    """
    from ct.control.servo import closed_loop_response

    for omega in (0.5, 1.5, 3.0):
        assert abs(closed_loop_response(servo_config, omega)) > 0.9


def test_residual_lag_is_finite_at_zero_frequency(servo_config):
    """``-angle/omega`` is numerically hopeless near zero; it must not produce a NaN.

    A NaN would propagate straight into the horizon and from there into every forecast.
    """
    lag = residual_lag(servo_config, 0.0)
    assert np.isfinite(lag) and lag >= 0


def test_residual_lag_rejects_negative_frequency(servo_config):
    with pytest.raises(ValueError, match="non-negative"):
        residual_lag(servo_config, -1.0)


def test_a_slower_loop_lags_more(servo_config):
    """The design trade, verified: bandwidth is what buys down ``tau_cl``.

    This is the sense in which ``tau_cl`` is a *servo-design output* rather than a
    physical constant — change the compensator and the horizon changes with it.
    """
    slower = AxisServoConfig(
        plant=servo_config.plant,
        lead=LeadCompensator(zero=8.0, pole=80.0, gain=40.0),
    )
    assert residual_lag(slower, 1.5) > residual_lag(servo_config, 1.5)
    assert bandwidth(slower) < bandwidth(servo_config)


def test_servo_bandwidth_clears_the_breathing_band(servo_config):
    """If it did not, the needle could not follow the reference whatever the forecast said."""
    assert bandwidth(servo_config) > 20 * 2.0  # comfortably above a fast breath


def test_lead_servo_settles_to_its_dc_gain(servo_config):
    """``gain*(s+z)/(s+p)`` has DC gain ``gain*z/p``, not ``gain``."""
    servo = LeadServo(servo_config, Ts=0.005)
    servo.reset()
    first = servo.update(1.0)
    assert first > 0, "a step in error should produce a correction in the same direction"
    outputs = [servo.update(1.0) for _ in range(400)]
    lead = servo_config.lead
    assert outputs[-1] == pytest.approx(lead.gain * lead.zero / lead.pole, rel=0.01)
    assert abs(outputs[-1] - outputs[-2]) < 1e-6, "should settle to a constant"
    assert first > outputs[-1], "a lead network kicks then relaxes; that is the lead"


def test_servo_output_limit_does_not_wind_up(servo_config):
    servo = LeadServo(servo_config, Ts=0.005)
    servo.reset()
    for _ in range(50):
        servo.update(1000.0, limit=5.0)
    assert servo.saturated > 0
    # Once the error reverses, the output must respond immediately rather than unwinding
    # a history it never actually applied.
    assert servo.update(-1000.0, limit=5.0) < 0


def test_rate_limit_ramps_a_large_step_instead_of_jumping(servo_config):
    """A sudden large error (a step command's initial error, not just a tracking disturbance)
    must not produce a full-size correction in one tick -- that is exactly the failure mode a
    lead compensator's zero is prone to, and what the units bug in insert.py's
    _hold_standoff() let happen unbounded before session 021."""
    servo = LeadServo(servo_config, Ts=0.005)
    servo.reset()
    max_step = 2.0 * 0.005  # rate_limit_mm_s * Ts
    first = servo.update(1000.0, limit=1000.0, rate_limit_mm_s=2.0)
    assert first == pytest.approx(max_step)
    assert servo.rate_limited == 1

    second = servo.update(1000.0, limit=1000.0, rate_limit_mm_s=2.0)
    assert second - first == pytest.approx(max_step)


def test_rate_limit_does_not_engage_when_unset(servo_config):
    """Backward compatible: omitting rate_limit_mm_s must reproduce the pre-existing,
    magnitude-only-clamped behavior exactly."""
    servo = LeadServo(servo_config, Ts=0.005)
    servo.reset()
    servo.update(1000.0, limit=5.0)
    assert servo.rate_limited == 0


def test_correction_limits_default_from_config():
    """A call site that just does update(error) -- the real control-code pattern after
    session 021's fix -- must get config-driven limits automatically, rather than needing to
    remember (or mis-remember, as insert.py did) the right number to pass in."""
    config = AxisServoConfig(
        plant=PlantModel(K=1.0, wn=60.0, zeta=0.7),
        lead=LeadCompensator(zero=8.0, pole=80.0, gain=150.0),
        correction_limit_mm=1.0,
        correction_rate_limit_mm_s=2.0,
    )
    servo = LeadServo(config, Ts=0.005)
    servo.reset()
    y = servo.update(1000.0)
    # The rate limit (0.01mm/tick here) is tighter than the magnitude limit (1.0mm) for this
    # first tick, so it's the rate limit that ultimately bounds the output -- but both are
    # applied in sequence (magnitude first, then rate), so a huge raw error trips both.
    assert y == pytest.approx(2.0 * 0.005)
    assert servo.saturated == 1
    assert servo.rate_limited == 1


def test_axis_servo_config_rejects_non_positive_correction_limits():
    with pytest.raises(ValueError, match="correction_limit_mm"):
        AxisServoConfig(correction_limit_mm=0.0)
    with pytest.raises(ValueError, match="correction_rate_limit_mm_s"):
        AxisServoConfig(correction_rate_limit_mm_s=-1.0)


# -- gate ---------------------------------------------------------------------


class _FakeTracker:
    """A tracker stub with a known waveform, so gate decisions are exactly predictable."""

    def __init__(self, K=1, A=5.0, omega=1.5, theta=0.0, variance=0.01):
        self.layout = StateLayout(K)
        self._s = self.layout.pack(
            a0=5.0, A=np.array([A] * K), phi=np.zeros(K), theta=theta, omega_r=omega
        )
        self.variance = variance

    @property
    def state(self):
        return self._s.copy(), np.eye(self.layout.n)

    def set_theta(self, theta):
        self._s[self.layout.theta] = theta

    def forecast(self, h):
        from ct.forecast import forecast_value

        return forecast_value(self._s, self.layout, h)

    def forecast_variance(self, h):
        return self.variance


def test_cycle_extrema_finds_the_real_waveform_range():
    tracker = _FakeTracker(A=5.0)
    y_min, y_max = cycle_extrema(*tracker.state[:1], tracker.layout) if False else cycle_extrema(
        tracker.state[0], tracker.layout
    )
    assert y_min == pytest.approx(0.0, abs=0.01)
    assert y_max == pytest.approx(10.0, abs=0.01)


def test_gate_opens_at_the_bottom_of_the_breath_not_the_top():
    """End-exhale is the *minimum* of tactile deflection.

    The sensor presses harder as the chest expands, so inhale is the maximum. Getting this
    backwards would fire every insertion at peak inhale — against the one clinical premise
    the whole procedure rests on.
    """
    gate = FiringGate(exhale_band_frac=0.15, max_forecast_std=1.0)
    tracker = _FakeTracker(A=5.0, omega=1.5)

    tracker.set_theta(-np.pi / 2)  # sin = -1 -> minimum deflection -> end-exhale
    assert gate.evaluate(0.0, tracker, h=0.0).fire

    gate.reset()
    tracker.set_theta(np.pi / 2)  # peak inhale
    decision = gate.evaluate(100.0, tracker, h=0.0)
    assert not decision.fire and "not at end-exhale" in decision.reason


def test_gate_refuses_when_the_forecast_is_not_confident_enough():
    """The covariance being spent. This is the whole argument for an EKF over LMS."""
    gate = FiringGate(exhale_band_frac=0.15, max_forecast_std=0.1)
    tracker = _FakeTracker(A=5.0, theta=-np.pi / 2, variance=4.0)  # std 2.0
    decision = gate.evaluate(0.0, tracker, h=0.0)
    assert not decision.fire
    assert decision.in_window and not decision.confident
    assert "exceeds budget" in decision.reason


def test_gate_fires_at_most_once_per_breath():
    """A wide band must not produce a burst of increments inside one cycle."""
    gate = FiringGate(exhale_band_frac=1.0, max_forecast_std=10.0, refractory_breaths=0.5)
    tracker = _FakeTracker(A=5.0, theta=-np.pi / 2, omega=1.5)
    period = 2 * np.pi / 1.5

    assert gate.evaluate(0.0, tracker, h=0.0).fire
    blocked = gate.evaluate(0.1, tracker, h=0.0)
    assert not blocked.fire and "already fired this breath" in blocked.reason
    assert gate.evaluate(period, tracker, h=0.0).fire
    assert gate.firings == 2


def test_gate_refuses_a_model_with_no_breath_in_it():
    """With no excursion, 'end-exhale' is not defined, so refusing is the only safe answer."""
    gate = FiringGate(exhale_band_frac=0.15, max_forecast_std=1.0)
    tracker = _FakeTracker(A=0.0)
    decision = gate.evaluate(0.0, tracker, h=0.0)
    assert not decision.fire and "no excursion" in decision.reason


def test_the_gate_looks_ahead_by_h_not_at_now():
    """The needle arrives in ``h`` seconds, so that is when the window has to be open."""
    gate = FiringGate(exhale_band_frac=0.15, max_forecast_std=1.0)
    omega = 1.5
    quarter = (np.pi / 2) / omega  # a quarter cycle in seconds
    tracker = _FakeTracker(A=5.0, omega=omega, theta=-np.pi)  # a quarter breath early

    assert not gate.evaluate(0.0, tracker, h=0.0).fire
    gate.reset()
    assert gate.evaluate(0.0, tracker, h=quarter).fire


# -- safety -------------------------------------------------------------------


def test_residual_monitor_survives_an_outlier_but_not_a_run():
    """One bad innovation is a glitch; a run of them means the model no longer fits."""
    monitor = SafetyMonitor(SafetyConfig(max_nis=25.0, nis_trip_count=5))
    for i in range(4):
        assert monitor.check_residual(float(i), 100.0) is None
    assert monitor.check_residual(4.0, 1.0) is None  # resets the run
    for i in range(4):
        assert monitor.check_residual(float(i), 100.0) is None
    trip = monitor.check_residual(10.0, 100.0)
    assert trip is not None and trip.kind == "residual"


def test_trips_latch():
    """Recovering automatically would mean deciding in software that a fault stopped
    mattering — with a needle in tissue, that is an operator's call."""
    monitor = SafetyMonitor(SafetyConfig())
    monitor.trip("test", "something", 1.0)
    assert monitor.tripped
    state = RigState(t=2.0, tactile_mm=1.0, in_contact=True)
    assert monitor.check(state, ProcedureState.ESTIMATE) is monitor.first_trip


def test_crushing_the_tactile_sensor_trips():
    monitor = SafetyMonitor(SafetyConfig(max_tactile_mm=10.0))
    state = RigState(t=1.0, tactile_mm=12.0, in_contact=True)
    trip = monitor.check(state, ProcedureState.ESTIMATE)
    assert trip is not None and trip.kind == "tactile_limit"


def test_stale_tactile_only_matters_once_something_depends_on_it():
    """During APPROACH's coarse drive the sensor legitimately has nothing to say."""
    monitor = SafetyMonitor(SafetyConfig())
    stale = RigState(t=1.0, stale=("tactile",))
    assert monitor.check(stale, ProcedureState.APPROACH) is None
    assert monitor.check(stale, ProcedureState.INSERT) is not None


def test_needle_motion_requires_contact():
    monitor = SafetyMonitor(SafetyConfig(require_contact_to_insert=True))
    trip = monitor.check_insert_preconditions(RigState(t=1.0, in_contact=False), 1.0)
    assert trip is not None and trip.kind == "no_contact"


# -- live signal --------------------------------------------------------------


def test_accumulator_deduplicates_on_arrival_not_on_tick():
    """The loop ticks faster than the sensor publishes.

    Counting repeats would make the record's sample rate the *loop's*, not the sensor's,
    and the identifier would be told the wrong thing about its own data.
    """
    acc = SignalAccumulator()
    assert acc.offer(1.0, 5.0)
    assert not acc.offer(1.0, 5.0)
    assert not acc.offer(0.9, 4.0)
    assert acc.offer(1.01, 6.0)
    assert len(acc) == 2 and acc.duplicates_skipped == 2


def test_accumulator_resamples_jittery_arrivals_onto_a_uniform_grid():
    """The FFT identifier needs uniform sampling; a real CAN bus will not provide it."""
    rng = np.random.default_rng(0)
    fs = 100.0
    t = np.arange(0, 10, 1 / fs) + rng.normal(0, 0.0005, int(10 * fs))
    t = np.sort(t)
    acc = SignalAccumulator()
    for stamp in t:
        acc.offer(float(stamp), float(np.sin(2 * np.pi * 0.25 * stamp)))

    assert acc.jitter_fraction() > 0
    batch = acc.to_batch()
    assert batch.fs == pytest.approx(fs, rel=0.05)
    assert np.allclose(np.diff(batch.t), 1 / batch.fs)
    assert batch.t[0] == pytest.approx(0.0)


def test_accumulator_refuses_to_identify_on_nothing():
    with pytest.raises(ValueError, match="at least 4 samples"):
        SignalAccumulator().to_batch()


def test_amplitude_watcher_reports_peak_to_trough_over_its_window():
    watcher = AmplitudeWatcher(window_s=1.0)
    for i in range(100):
        watcher.add(i * 0.02, np.sin(2 * np.pi * i * 0.02))
    assert watcher.full
    assert watcher.amplitude == pytest.approx(2.0, abs=0.1)
    assert watcher.span <= 1.0 + 1e-9


# -- breathing peaks, for standoff --------------------------------------------


def _breathing(peaks, period=5.5, fs=100.0, baseline=0.0, drift=0.0):
    """A breathing trace whose successive breaths reach exactly the given peaks."""
    t, y = [], []
    clock = 0.0
    for i, peak in enumerate(peaks):
        n = int(period * fs)
        for k in range(n):
            phase = 2 * np.pi * k / n
            t.append(clock)
            y.append(baseline + drift * clock + peak * 0.5 * (1 - np.cos(phase)))
            clock += 1 / fs
        del i
    return np.array(t), np.array(y)


def test_breath_peak_watcher_counts_whole_breaths_not_seconds():
    """The window used to be min_breaths * nominal_breath_s, and nothing kept the nominal
    honest: 2 x 4.0s = 8.0s against a real 5.51s period is 1.45 breaths, not 2."""
    t, y = _breathing([1.0] * 6, period=5.5)
    watcher = BreathPeakWatcher(n_breaths=2)

    ready_at = None
    for ti, yi in zip(t, y):
        watcher.add(ti, yi)
        if ready_at is None and watcher.ready:
            ready_at = ti

    assert watcher.breaths >= 2
    # Two whole breaths of a 5.5s cycle cannot be judged in the 8.0s the old window allowed.
    assert ready_at > 8.0
    assert watcher.period_s == pytest.approx(5.5, rel=0.05)


def test_breath_peak_is_unbiased_where_max_over_a_window_is_not():
    """The defect that drove the spurious retreats of run 20260903-152958.

    Real breathing varies breath to breath, so the MAXIMUM over a window sits above the
    typical peak by an amount that grows with the window -- and standoff retreats whenever
    its reading exceeds target, so that bias alone moves the base the wrong way. Replayed
    over the stationary measurement segments of the two runs on 2026-09-03, max-over-8s read
    +0.48 and +0.54mm higher than the mean of two real breaths.
    """
    peaks = [8.0, 9.0, 8.0, 9.0, 8.0, 9.0]      # typical peak is 8.5
    t, y = _breathing(peaks, period=5.0)

    breath = BreathPeakWatcher(n_breaths=2)
    window = AmplitudeWatcher(window_s=2 * 5.0)
    for ti, yi in zip(t, y):
        breath.add(ti, yi)
        window.add(ti, yi)

    assert breath.peak == pytest.approx(8.5, abs=0.05)   # unbiased
    assert window.peak == pytest.approx(9.0, abs=0.05)   # biased to the deepest breath
    assert window.peak - breath.peak > 0.4
    assert breath.peak_spread == pytest.approx(1.0, abs=0.05)


def test_breath_segmentation_survives_a_wandering_baseline():
    """Real subject profiles carry slow baseline wander (session 009), which a fixed crossing
    level would eventually sit outside entirely.

    ``emma_normal_breathing`` drifts -0.00513 mm/s over a 180s run, 0.92mm against a ~4.6mm
    excursion. 0.01 mm/s here is twice that rate at half the amplitude, so the drift-to-signal
    ratio is about four times what the bench actually sees.
    """
    t, y = _breathing([2.0] * 8, period=5.0, baseline=5.0, drift=0.01)
    watcher = BreathPeakWatcher(n_breaths=2)
    for ti, yi in zip(t, y):
        watcher.add(ti, yi)

    # Only completed breaths count, so the partial one at each end is not expected.
    assert watcher.breaths >= 6
    assert watcher.period_s == pytest.approx(5.0, rel=0.05)
    assert watcher.peak == pytest.approx(2.0 + 5.0, abs=0.5)


def test_breath_peak_watcher_ignores_a_partial_breath():
    """A half-finished breath must never be banked as a shallow one -- that would read as a
    peak below target and buy a step the base did not need."""
    t, y = _breathing([4.0, 4.0], period=5.0)
    cut = int(len(t) * 0.75)             # stop mid-way through the second breath
    watcher = BreathPeakWatcher(n_breaths=2)
    for ti, yi in zip(t[:cut], y[:cut]):
        watcher.add(ti, yi)

    assert watcher.breaths == 1
    assert not watcher.ready
    assert watcher.peak == pytest.approx(4.0, abs=0.05)


def test_a_plateau_of_sensor_jitter_is_not_a_sequence_of_breaths():
    """Real subjects pause at end-exhale; a sinusoid never does.

    The hysteresis band is a fraction of the window's own amplitude, so a window containing
    only a plateau shrinks it to nothing and jitter alone segments "breaths". Taken from
    outputs/needle_gating_live/moira/1, whose standoff loop reported two completed breaths in
    0.6s -- peaks agreeing to 0.0034mm -- and stepped the base 2.57mm on the strength of it.
    Values below are that run's real samples at 124Hz.
    """
    jitter = [6.489, 6.493, 6.495, 6.491, 6.490, 6.491, 6.491, 6.494,
              6.490, 6.490, 6.488, 6.489, 6.488, 6.487]
    watcher = BreathPeakWatcher(n_breaths=2)
    for i, value in enumerate(jitter * 6):     # ~0.7s of plateau
        watcher.add(i / 124.0, value)

    assert watcher.breaths == 0
    assert not watcher.ready


def test_a_breath_faster_than_any_real_subject_is_discarded():
    """60 breaths/min is the floor; anything quicker is a segmentation artifact, not a breath."""
    t, y = _breathing([3.0] * 4, period=0.4)   # 150 breaths/min
    watcher = BreathPeakWatcher(n_breaths=2)
    for ti, yi in zip(t, y):
        watcher.add(ti, yi)

    assert watcher.breaths == 0

    slow_t, slow_y = _breathing([3.0] * 4, period=5.0)
    slow = BreathPeakWatcher(n_breaths=2)
    for ti, yi in zip(slow_t, slow_y):
        slow.add(ti, yi)
    assert slow.ready                          # the guard must not reject real breathing


def test_resetting_forgets_samples_from_before_a_base_move():
    t, y = _breathing([3.0] * 4, period=5.0)
    watcher = BreathPeakWatcher(n_breaths=2)
    for ti, yi in zip(t, y):
        watcher.add(ti, yi)
    assert watcher.ready

    watcher.reset()
    assert watcher.breaths == 0
    assert not watcher.ready
    assert watcher.period_s is None


# -- state machine ------------------------------------------------------------


def test_the_main_sequence_runs_in_order():
    assert next_in_sequence(ProcedureState.APPROACH) is ProcedureState.ESTIMATE
    assert next_in_sequence(ProcedureState.ESTIMATE) is ProcedureState.INSERT
    assert next_in_sequence(ProcedureState.INSERT) is ProcedureState.ADVANCE
    assert next_in_sequence(ProcedureState.ADVANCE) is ProcedureState.DONE
    assert next_in_sequence(ProcedureState.WITHDRAW) is ProcedureState.DONE


def test_states_know_which_axes_they_may_move():
    assert ProcedureState.ADVANCE.moves_needle
    assert not ProcedureState.ADVANCE.moves_base
    assert ProcedureState.APPROACH.moves_base
    assert ProcedureState.FAULT.is_terminal
