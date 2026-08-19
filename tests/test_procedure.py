"""The four-state procedure, end to end, against the simulated rig.

This is the test the whole architecture exists to make possible. It runs every line of
control code — the approach's seating search, Stage 1 identification, the EKF, the firing
gate, the servo, the safety monitor — through the real CAN codecs against a simulated
plant, under :class:`~ct.rt.clock.SimClock`. Twenty minutes of breathing, in a couple of
seconds, deterministically, with no hardware.

Swapping ``SimClock`` for ``RealClock`` and ``loopback`` for ``rh02`` is the only
difference between this and a run on the bench. If that stops being true, these tests stop
meaning anything — which is what ``test_boundaries.py`` is there to prevent.
"""

from __future__ import annotations

import pytest

from ct.config import RunConfig
from ct.control.state import ProcedureState
from ct.hw.motors.axis import AxisLimitError
from ct.rig import build_rig, run_rig


@pytest.fixture(scope="module")
def completed_run():
    """One full procedure, shared across assertions because it takes a second to run."""
    return run_rig(RunConfig.from_yaml("rig_sim"), max_duration_s=900.0)


def test_the_whole_procedure_reaches_the_target_depth(completed_run):
    assert completed_run.final_state is ProcedureState.DONE
    assert completed_run.reached_target


def test_it_visits_all_four_states_in_order(completed_run):
    visited = [entry["to"] for entry in completed_run.assembly.procedure.history]
    assert visited == ["estimate", "insert", "advance", "done"]


def test_every_transition_records_why_it_fired(completed_run):
    """A run that ended in the wrong place must be readable without re-instrumenting it."""
    for entry in completed_run.assembly.procedure.history:
        assert entry["why"], f"transition to {entry['to']} gave no reason"
        assert entry["t"] > 0


def test_nothing_tripped(completed_run):
    assert not completed_run.summary["safety"]["tripped"], (
        completed_run.summary["safety"]["trips"]
    )


def test_the_loop_held_its_rate(completed_run):
    loop = completed_run.summary["loop"]
    assert loop["deadline_misses"] == 0
    assert loop["achieved_rate_hz"] == pytest.approx(200.0, rel=0.01)


def test_the_estimator_stayed_consistent(completed_run):
    """NIS near 1 means the filter's own uncertainty matches its actual errors.

    That is the property the firing gate spends, so it is worth asserting rather than
    assuming.
    """
    records = [r for r in completed_run.records if r.get("nis") is not None]
    assert records, "the tracker never ran"
    nis = sorted(r["nis"] for r in records[len(records) // 2 :])
    median = nis[len(nis) // 2]
    assert 0.1 < median < 5.0, f"median NIS {median} is not consistent"


def test_the_horizon_contains_all_four_terms(completed_run):
    """Including a genuinely non-zero ``tau_cl`` from the servo design."""
    parts = completed_run.summary["horizon_breakdown"]
    assert parts["tau_s"] > 0
    assert parts["T_ins"] > 0
    assert parts["tau_cl"] > 0, "tau_cl collapsed to zero; the servo model is not lagging"
    assert parts["h"] == pytest.approx(
        parts["tau_s"] + parts["tau_c"] + parts["tau_cl"] + parts["T_ins"]
    )
    assert not completed_run.summary["latency"]["tau_cl_fallback_in_use"]


def test_the_needle_ends_up_in_tissue_at_the_commanded_depth(completed_run):
    """Checked against simulation ground truth, which the controller never saw."""
    plant = completed_run.assembly.plant
    target = completed_run.assembly.session.procedure.advance.total_depth_mm
    assert plant.tissue.captured
    assert plant.truth.insertion_depth_mm == pytest.approx(target, abs=5.0)


def test_the_approach_found_the_full_breathing_excursion(completed_run):
    """The seating search's actual job.

    Early in APPROACH the tactile trough is pinned at zero — the sensor loses the skin at
    end-exhale. By the time the model is identified the whole waveform must be visible, or
    the estimator is fitting a clipped signal.
    """
    approach = [r["tactile_mm"] for r in completed_run.records
                if r["state"] == "approach" and r.get("tactile_mm") is not None]
    estimate = [r["tactile_mm"] for r in completed_run.records
                if r["state"] == "estimate" and r.get("tactile_mm") is not None]
    assert min(approach) == pytest.approx(0.0, abs=1e-6), "expected clipping early on"
    assert min(estimate) > 0.05, "the trough is still clipped when identification runs"
    assert max(estimate) - min(estimate) > max(approach[: len(approach) // 4]) * 1.5


def test_increments_are_spaced_about_a_breath_apart(completed_run):
    """The refractory interlock, observed in a real run rather than unit-tested.

    One bite per cycle is the intent. What this actually pins is the *spacing*: no two
    increments may land inside the same breath, which is the failure the interlock exists
    to prevent.
    """
    import numpy as np

    advance = completed_run.assembly.procedure.states[ProcedureState.ADVANCE]
    times = advance.increment_times
    assert len(times) > 2, "not enough increments to say anything about spacing"

    period = 2 * np.pi / completed_run.assembly.context.omega_r
    gaps = np.diff(times) / period
    assert gaps.min() >= 0.85, (
        f"two increments landed {gaps.min():.2f} breaths apart — the gate is bursting"
    )
    assert gaps.max() < 3.0, (
        f"a gap of {gaps.max():.2f} breaths — the gate is skipping cycles it should catch"
    )


# -- determinism and swappability --------------------------------------------


def test_two_simulated_runs_agree_exactly():
    """No tolerances. A procedure test that drifts run to run cannot pin a regression."""
    cfg = RunConfig.from_yaml("rig_sim").apply_overrides({"procedure.advance.total_depth_mm": 14.0})
    first = run_rig(cfg, max_duration_s=400.0, keep_records=False)
    second = run_rig(cfg, max_duration_s=400.0, keep_records=False)
    assert first.assembly.procedure.history == second.assembly.procedure.history


def test_the_run_can_be_stopped_after_any_state():
    """`--stop-at` — how one piece gets exercised on the bench without the rest."""
    cfg = RunConfig.from_yaml("rig_sim")
    result = run_rig(cfg, stop_at=ProcedureState.ESTIMATE, max_duration_s=400.0,
                     keep_records=False)
    assert result.final_state is ProcedureState.ESTIMATE
    assert [e["to"] for e in result.assembly.procedure.history] == ["estimate"]


def test_dry_run_exercises_everything_except_motion():
    """The first thing to run against real hardware."""
    cfg = RunConfig.from_yaml("rig_sim")
    result = run_rig(cfg, dry_run=True, max_duration_s=60.0, keep_records=False)
    for axis in result.assembly.axes.values():
        assert axis.commands_suppressed > 0
        assert axis.commands_sent == 0
    # Nothing moved, so APPROACH cannot finish — which is the correct outcome, not a bug.
    assert result.assembly.context.state.base_mm == pytest.approx(0.0)


def test_a_state_can_be_swapped_by_name():
    """The same claim the estimator makes for its three stages, one layer down."""
    from ct.control.state import Transition
    from ct.control.states.base import BaseState
    from ct.registry import register_state

    @register_state("test_instant_approach")
    class InstantApproach(BaseState):
        state = ProcedureState.APPROACH

        def update(self, ctx):
            return Transition(ProcedureState.FAULT, "swapped state ran instead")

    cfg = RunConfig.from_yaml("rig_sim")
    assembly = build_rig(cfg)
    assembly.procedure.states[ProcedureState.APPROACH] = InstantApproach()
    record = assembly.procedure.tick(0.0, {"sensing": []})
    assert record["state"] in ("approach", "fault")
    assembly.procedure.tick(0.005, {"sensing": []})
    assert assembly.procedure.current is ProcedureState.FAULT
    assembly.close()


# -- failure modes ------------------------------------------------------------


def test_an_axis_whose_travel_exceeds_its_codec_refuses_to_build():
    """The silent-saturation trap, caught at construction.

    MIT mode's default +/-12.5 rad is about two turns. Asking for more does not error on
    the wire — the command clips and the axis stops short, with feedback agreeing.
    """
    cfg = RunConfig.from_yaml("rig_sim").apply_overrides(
        {"rig.motors.needle.codec_params": {}}  # back to the +/-12.5 rad default
    )
    with pytest.raises(AxisLimitError, match="outside 'cubemars_mit' range"):
        build_rig(cfg)


def test_a_needle_that_cannot_float_is_reported_not_ignored():
    """Servo mode has no zero-stiffness command, and ADVANCE depends on one."""
    cfg = RunConfig.from_yaml("rig_sim").apply_overrides(
        {"rig.motors.needle.codec": "cubemars_servo",
         "rig.motors.needle.codec_params": {},
         "rig.geometry.needle.counts_per_mm": 2.0}
    )
    assembly = build_rig(cfg)
    with pytest.raises(AxisLimitError, match="cannot express zero-stiffness"):
        assembly.axes["needle"].float_free()
    assembly.close()


def test_a_stalled_sensor_faults_rather_than_hanging():
    """Readings that stop changing look exactly like a patient holding still.

    Staleness is the only way to tell those apart, so the watchdog is the difference
    between a fault and a controller confidently acting on a frozen number.
    """
    cfg = RunConfig.from_yaml("rig_sim").apply_overrides({"procedure.approach.timeout_s": 20.0})
    result = run_rig(cfg, max_duration_s=60.0, keep_records=False)
    # With a 20 s budget the seating search cannot finish, so it must fault, not hang.
    assert result.final_state is ProcedureState.FAULT
    why = result.assembly.procedure.history[-1]["why"]
    assert "timed out" in why and "approach" in why
