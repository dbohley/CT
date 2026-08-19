"""Name -> class lookup for the three swappable stages.

A config file says ``source: {name: lujan, ...}`` and this module turns that into
a constructed object. Registering a new implementation is a decorator and an
import; nothing in the CLI, the configs, or the other two stages changes.
"""

from __future__ import annotations

from typing import Any, Callable, TypeVar

T = TypeVar("T")

SOURCES: dict[str, type] = {}
IDENTIFIERS: dict[str, type] = {}
TRACKERS: dict[str, type] = {}

# Rig-controller boundaries (session 002). Same mechanism, one layer down: a bus, a motor
# wire protocol or a procedure state is chosen by config string exactly as a tracker is.
BUSES: dict[str, type] = {}
CODECS: dict[str, type] = {}
SENSORS: dict[str, type] = {}
STATES: dict[str, type] = {}


def _make_register(table: dict[str, type], kind: str) -> Callable[[str], Callable[[type], type]]:
    def register(name: str) -> Callable[[type], type]:
        def deco(cls: type) -> type:
            if name in table and table[name] is not cls:
                raise ValueError(f"{kind} '{name}' is already registered to {table[name]}")
            table[name] = cls
            cls.registry_name = name  # type: ignore[attr-defined]
            return cls

        return deco

    return register


register_source = _make_register(SOURCES, "source")
register_identifier = _make_register(IDENTIFIERS, "identifier")
register_tracker = _make_register(TRACKERS, "tracker")

register_bus = _make_register(BUSES, "bus")
register_codec = _make_register(CODECS, "codec")
register_sensor = _make_register(SENSORS, "sensor")
register_state = _make_register(STATES, "state")


def _build(table: dict[str, type], kind: str, name: str, params: dict[str, Any] | None) -> Any:
    if name not in table:
        known = ", ".join(sorted(table)) or "(none)"
        raise KeyError(f"unknown {kind} '{name}'. Registered: {known}")
    return table[name](**(params or {}))


def build_source(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(SOURCES, "source", name, params)


def build_identifier(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(IDENTIFIERS, "identifier", name, params)


def build_tracker(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(TRACKERS, "tracker", name, params)


def build_bus(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(BUSES, "bus", name, params)


def build_codec(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(CODECS, "codec", name, params)


def build_sensor(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(SENSORS, "sensor", name, params)


def build_state(name: str, params: dict[str, Any] | None = None) -> Any:
    return _build(STATES, "state", name, params)


def available() -> dict[str, list[str]]:
    """Registered names by kind — printed by the CLIs on an unknown name.

    The rig kinds stay empty until ``ct.control`` is imported; the estimator does not
    pull in the hardware layer, so ``import ct`` alone does not populate them.
    """
    return {
        "sources": sorted(SOURCES),
        "identifiers": sorted(IDENTIFIERS),
        "trackers": sorted(TRACKERS),
        "buses": sorted(BUSES),
        "codecs": sorted(CODECS),
        "sensors": sorted(SENSORS),
        "states": sorted(STATES),
    }
