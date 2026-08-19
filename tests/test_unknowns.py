"""The unknowns registry.

The list is only useful if it stays true. These tests are what stop it drifting into a
stale document: every key must resolve against the shipped rig config, so renaming a
config field without updating the entry fails here rather than silently leaving a number
nobody will ever be asked to measure.
"""

from __future__ import annotations

import pytest

from ct.config import RunConfig
from ct.control.state import ProcedureState
from ct.unknowns import (
    OWNERS,
    UNKNOWNS,
    Unknown,
    blocking,
    by_owner,
    gate_or_raise,
    outstanding,
    to_markdown,
)


@pytest.fixture(scope="module")
def rig_config():
    return RunConfig.from_yaml("rig_sim").to_dict()


def test_every_key_resolves_against_the_shipped_config(rig_config):
    """The guard that keeps the list honest.

    A key that resolves to nothing describes a config field that does not exist, which
    means the entry is documenting a plan rather than the code.
    """
    missing = [u.key for u in UNKNOWNS if not _resolves(u, rig_config)]
    assert not missing, f"keys that do not exist in configs/rig_sim.yaml: {missing}"


def _resolves(unknown: Unknown, cfg: dict) -> bool:
    from ct.unknowns import MISSING

    return unknown.resolve(cfg) is not MISSING


def test_keys_are_unique():
    keys = [u.key for u in UNKNOWNS]
    duplicates = {k for k in keys if keys.count(k) > 1}
    assert not duplicates, f"duplicate entries: {duplicates}"


def test_every_entry_says_how_to_measure_it():
    """An entry without a procedure is a complaint, not a work item."""
    for unknown in UNKNOWNS:
        assert len(unknown.how_to_measure) > 30, f"{unknown.key} has no real procedure"
        assert len(unknown.why) > 20, f"{unknown.key} does not say why it matters"
        assert unknown.units, f"{unknown.key} has no units"


def test_owners_are_real_places_to_look():
    for unknown in UNKNOWNS:
        assert unknown.owner in OWNERS


def test_entries_are_rejected_if_malformed():
    with pytest.raises(ValueError, match="unknown owner"):
        Unknown(key="a.b", units="mm", what="x", why="y", how_to_measure="z", owner="nobody",
                placeholder=1.0)
    with pytest.raises(ValueError, match="dotted config path"):
        Unknown(key="not a path", units="mm", what="x", why="y", how_to_measure="z",
                owner="mech", placeholder=1.0)


def test_the_shipped_sim_config_is_mostly_placeholders(rig_config):
    """Expected, and the point. Simulation runs on placeholders; hardware does not."""
    still = outstanding(rig_config)
    assert len(still) > len(UNKNOWNS) // 3
    assert blocking(rig_config, [ProcedureState.INSERT])


def test_a_measured_value_stops_being_outstanding(rig_config):
    """Setting the real number is all it takes — no second place to update."""
    entry = next(u for u in UNKNOWNS if u.key == "latency.tau_s")
    assert entry.is_placeholder(rig_config)

    measured = RunConfig.from_yaml("rig_sim").apply_overrides({"latency.tau_s": 0.0374})
    assert not entry.is_placeholder(measured.to_dict())


def test_yaml_round_trips_do_not_look_like_measurements(rig_config):
    """A list read back as a list, or an int as a float, must not read as 'measured'."""
    entry = next(u for u in UNKNOWNS if u.key == "rig.geometry.base.travel_mm")
    cfg = RunConfig.from_yaml("rig_sim").apply_overrides(
        {"rig.geometry.base.travel_mm": list(entry.placeholder)}
    )
    assert entry.is_placeholder(cfg.to_dict())


# -- the gate -----------------------------------------------------------------


def test_simulation_runs_freely_on_placeholders(rig_config):
    offenders = gate_or_raise(rig_config, list(ProcedureState), simulated=True)
    assert offenders, "should still report them, just not refuse"


def test_hardware_refuses_to_start_on_placeholders(rig_config):
    """The gate that keeps an unmeasured rig from moving a needle."""
    with pytest.raises(RuntimeError, match="block this run on hardware"):
        gate_or_raise(rig_config, [ProcedureState.INSERT], simulated=False)


def test_the_operator_can_override_deliberately(rig_config):
    offenders = gate_or_raise(
        rig_config, [ProcedureState.INSERT], simulated=False, allow_placeholders=True
    )
    assert offenders


def test_only_the_states_being_run_are_gated(rig_config):
    """Starting mid-procedure should not demand numbers that state never touches."""
    withdraw_only = blocking(rig_config, [ProcedureState.WITHDRAW])
    everything = blocking(rig_config, list(ProcedureState))
    assert len(withdraw_only) < len(everything)


# -- the document -------------------------------------------------------------


def test_markdown_names_every_entry(rig_config):
    text = to_markdown(rig_config)
    for unknown in UNKNOWNS:
        assert unknown.key in text
    assert "still a placeholder" in text
    for owner in by_owner():
        assert f"## {owner}" in text


def test_markdown_works_without_a_config():
    text = to_markdown(None)
    assert "still a placeholder" not in text
    assert UNKNOWNS[0].key in text
