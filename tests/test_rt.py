"""Clock, loop, latency budget, mailbox, telemetry.

The point of most of this is :class:`SimClock`. Swapping it for ``RealClock`` is what
turns a twenty-minute insertion procedure into a test that runs in milliseconds and gives
the same answer every time — so if the loop's timing behaviour depended on which clock it
had, every procedure test below would be measuring the wrong thing.
"""

from __future__ import annotations

import pytest

from ct.forecast import horizon_from_components
from ct.hw.bus.mailbox import FrameQueue, Mailbox
from ct.hw.config import LatencyConfig
from ct.rt.clock import RealClock, SimClock, build_clock
from ct.rt.latency import LatencyBudget, LatencyEstimator
from ct.rt.loop import ControlLoop, LoopOverrun
from ct.rt.telemetry import TelemetryWriter, load_jsonl, to_arrays


# -- clock --------------------------------------------------------------------


def test_sim_clock_only_moves_when_asked():
    clock = SimClock()
    assert clock.now() == 0.0
    clock.sleep_until(5.0)
    assert clock.now() == 5.0
    clock.sleep_until(1.0)  # already past; must not go backwards
    assert clock.now() == 5.0
    assert clock.is_simulated


def test_sim_clock_rejects_negative_advances():
    with pytest.raises(ValueError, match="backwards"):
        SimClock().advance(-1.0)


def test_real_clock_is_monotonic_and_says_it_is_real():
    clock = RealClock()
    assert not clock.is_simulated
    assert clock.now() <= clock.now()


def test_clocks_resolve_by_name():
    assert isinstance(build_clock("sim"), SimClock)
    assert isinstance(build_clock("real"), RealClock)
    with pytest.raises(KeyError, match="unknown clock"):
        build_clock("sundial")


# -- loop ---------------------------------------------------------------------


class _NullBus:
    def __init__(self):
        self.polls = 0
        self.times = []

    def set_time(self, t):
        self.times.append(t)

    def poll(self):
        self.polls += 1
        return []

    def close(self):
        pass


def test_loop_runs_at_the_requested_rate_under_a_simulated_clock():
    bus = _NullBus()
    ticks = []
    loop = ControlLoop(SimClock(), rate_hz=100.0, buses={"b": bus},
                       on_tick=lambda info: ticks.append(info.t))
    stats = loop.run(duration_s=1.0)
    assert stats.ticks == 100
    assert stats.deadline_misses == 0
    assert ticks[1] - ticks[0] == pytest.approx(0.01)


def test_loop_is_deterministic():
    """Two identical simulated runs must agree exactly, not approximately.

    This is what lets a procedure test assert on transition times instead of tolerances.
    """
    def run():
        ticks = []
        ControlLoop(SimClock(), 200.0, {"b": _NullBus()},
                    on_tick=lambda i: ticks.append(i.t)).run(duration_s=0.5)
        return ticks

    assert run() == run()


def test_deadlines_are_absolute_so_a_slow_tick_does_not_shift_the_schedule():
    """Accumulating from ``now`` would let the loop quietly slow down, corrupting tau_c."""
    clock = SimClock()
    times = []

    def slow(info):
        times.append(info.t)
        if info.n == 2:
            clock.advance(0.05)  # one very late tick

    loop = ControlLoop(clock, rate_hz=100.0, buses={"b": _NullBus()}, on_tick=slow)
    loop.run(max_ticks=8)
    # The schedule recovers rather than drifting by the overrun.
    assert times[-1] - times[0] == pytest.approx(0.11, abs=1e-9)


def test_the_loop_stops_when_the_callback_says_so():
    loop = ControlLoop(SimClock(), 100.0, {"b": _NullBus()},
                       on_tick=lambda info: info.n < 5)
    assert loop.run(duration_s=10.0).ticks == 6


def test_persistent_overruns_raise_rather_than_degrading_silently():
    clock = SimClock()

    def always_late(info):
        clock.advance(0.05)

    loop = ControlLoop(clock, rate_hz=100.0, buses={"b": _NullBus()},
                       on_tick=always_late, max_deadline_misses=3)
    with pytest.raises(LoopOverrun, match="missed deadlines"):
        loop.run(duration_s=10.0)


def test_every_bus_is_polled_and_told_the_time():
    """`set_time` is what advances the simulated devices; the real bus ignores it."""
    bus = _NullBus()
    ControlLoop(SimClock(), 100.0, {"b": bus}, on_tick=lambda i: None).run(max_ticks=4)
    assert bus.polls == 4
    assert bus.times == [0.0, 0.01, 0.02, 0.03]


# -- latency ------------------------------------------------------------------


def test_latency_estimator_uses_the_fallback_until_it_has_data():
    estimator = LatencyEstimator(window=8, fallback=0.004)
    assert estimator.p95 == pytest.approx(0.004)
    for _ in range(8):
        estimator.record(0.001)
    assert estimator.p95 == pytest.approx(0.001)


def test_latency_estimator_reports_the_tail_not_the_average():
    """The horizon has to cover the slow ticks, not the typical one."""
    estimator = LatencyEstimator(window=100)
    for i in range(100):
        estimator.record(0.001 if i < 95 else 0.05)
    assert estimator.p50 == pytest.approx(0.001)
    assert estimator.p95 > estimator.p50
    assert estimator.worst == pytest.approx(0.05)


def test_horizon_matches_the_estimators_own_definition():
    """The budget must not grow a second, subtly different definition of ``h``."""
    config = LatencyConfig(tau_s=0.02, tau_c=0.005, T_ins=0.15, measure_tau_c=False)
    budget = LatencyBudget(config, tau_cl=lambda omega: 0.03)
    assert budget.horizon(1.5) == pytest.approx(
        horizon_from_components(0.02, 0.005, 0.03, 0.15)
    )


def test_tau_cl_is_a_function_of_breathing_rate_not_a_constant():
    """The settled decision, made testable.

    ``tau_cl`` is a servo-design output that varies with how fast the patient is
    breathing. A budget that returned the same number for every rate would have quietly
    reintroduced the fixed ``tau_a`` the design explicitly rejects.
    """
    budget = LatencyBudget(LatencyConfig(), tau_cl=lambda omega: 0.01 * omega)
    assert budget.horizon(2.0) > budget.horizon(1.0)
    assert not budget.uses_fallback_tau_cl


def test_a_budget_without_a_servo_says_it_is_guessing():
    budget = LatencyBudget(LatencyConfig(tau_cl_fallback=0.07))
    assert budget.uses_fallback_tau_cl
    assert budget.tau_closed_loop(1.5) == pytest.approx(0.07)


def test_breakdown_sums_to_the_horizon():
    budget = LatencyBudget(LatencyConfig(measure_tau_c=False), tau_cl=lambda w: 0.02)
    parts = budget.breakdown(1.5)
    assert parts["h"] == pytest.approx(budget.horizon(1.5))
    assert parts["tau_s"] + parts["tau_c"] + parts["tau_cl"] + parts["T_ins"] == pytest.approx(
        parts["h"]
    )


# -- mailbox ------------------------------------------------------------------


def test_mailbox_keeps_only_the_newest_frame_per_id():
    box = Mailbox()
    box.put(1.0, 0x100, b"\x01")
    box.put(2.0, 0x100, b"\x02")
    slot = box.get(0x100)
    assert slot.data == b"\x02" and slot.count == 2


def test_an_unseen_id_counts_as_stale():
    """Correct at startup, and it saves every caller a None check."""
    box = Mailbox()
    assert box.is_stale(0x999, now=1.0, budget_s=0.1)
    box.put(1.0, 0x999, b"")
    assert not box.is_stale(0x999, now=1.05, budget_s=0.1)
    assert box.is_stale(0x999, now=1.5, budget_s=0.1)


def test_frame_queue_drops_oldest_and_counts_it():
    """If the consumer fell behind, recent frames describe the world and old ones do not."""
    queue = FrameQueue(maxlen=3)
    for i in range(5):
        queue.put(float(i), 1, bytes([i]))
    drained = queue.drain()
    assert [d[2][0] for d in drained] == [2, 3, 4]
    assert queue.stats["dropped"] == 2
    assert queue.drain() == []


# -- telemetry ----------------------------------------------------------------


def test_telemetry_round_trips(tmp_path):
    path = tmp_path / "t.jsonl"
    with TelemetryWriter(path, flush_every=2) as writer:
        for i in range(5):
            writer.write({"t": float(i), "state": "approach", "x": i * 2})
    records = load_jsonl(path)
    assert len(records) == 5
    assert records[3]["x"] == 6


def test_truncated_logs_are_still_readable(tmp_path):
    """A run killed mid-flush is usually the most interesting one to read back."""
    path = tmp_path / "t.jsonl"
    path.write_text('{"t": 1.0}\n{"t": 2.0}\n{"t": 3.0, "brok')
    assert [r["t"] for r in load_jsonl(path)] == [1.0, 2.0]


def test_ragged_records_become_a_common_time_axis():
    """Ticks before the tracker exists have no forecast; padding keeps plots aligned."""
    arrays = to_arrays([{"t": 0.0, "h": 0.2}, {"t": 0.1}], ["t", "h"])
    assert arrays["t"].tolist() == [0.0, 0.1]
    assert arrays["h"][0] == pytest.approx(0.2)
    assert arrays["h"][1] != arrays["h"][1]  # nan
