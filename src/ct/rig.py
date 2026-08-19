"""Assembling and running the rig — config in, procedure out.

The hardware layer's counterpart to :mod:`ct.run`, and the one module allowed to see both
the controller and the simulated plant. Rule 3 forbids ``control/``, ``hw/`` and ``rt/``
from importing ``plant/``; this is the orchestrator that wires one to the other, exactly as
``run.py`` wires sources to identifiers without either knowing about the other.

The whole sim/real difference lives in :func:`build_rig`, and amounts to two things: which
bus backend gets constructed, and whether a :class:`~ct.plant.rig.SimulatedRig` is attached
to it. Every line below that point is identical either way.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ct.config import RunConfig
from ct.control.context import ProcedureContext
from ct.control.procedure import Procedure
from ct.control.safety import SafetyMonitor
from ct.control.servo import LeadServo
from ct.control.state import ProcedureState
from ct.geometry import RigGeometry
from ct.hw.bus import build_bus_from_config
from ct.hw.config import RigSession
from ct.hw.motors.axis import CANAxis
from ct.hw.sensors.can_sensor import CANScalarSensor
from ct.registry import build_codec, build_identifier, build_source, build_tracker
from ct.rt.clock import RealClock, SimClock
from ct.rt.latency import LatencyBudget
from ct.rt.loop import ControlLoop, TickInfo
from ct.rt.telemetry import TelemetryWriter
from ct.unknowns import gate_or_raise


@dataclass
class RigAssembly:
    """Everything a run needs, constructed and wired."""

    session: RigSession
    buses: dict[str, Any]
    axes: dict[str, CANAxis]
    sensors: dict[str, CANScalarSensor]
    context: ProcedureContext
    procedure: Procedure
    clock: Any
    plant: Any | None = None
    """The simulated rig, when there is one. Ground truth — never routed to the controller."""

    def close(self) -> None:
        for bus in self.buses.values():
            bus.close()


@dataclass
class RigResult:
    config: RunConfig
    assembly: RigAssembly
    loop_stats: Any
    records: list[dict[str, Any]] = field(default_factory=list)
    summary: dict[str, Any] = field(default_factory=dict)

    @property
    def final_state(self) -> ProcedureState:
        return self.assembly.procedure.current

    @property
    def reached_target(self) -> bool:
        return self.final_state is ProcedureState.DONE


def build_rig(
    cfg: RunConfig,
    *,
    simulated: bool | None = None,
    dry_run: bool = False,
    start: ProcedureState = ProcedureState.APPROACH,
    stop_at: ProcedureState | None = None,
    telemetry: TelemetryWriter | None = None,
    allow_placeholders: bool = False,
) -> RigAssembly:
    """Construct buses, codecs, axes, sensors, the controller, and (in sim) the plant."""
    session = RigSession.from_run_config(cfg)
    geometry: RigGeometry = session.rig.geometry
    if simulated is None:
        simulated = session.rig.is_simulated

    # The placeholder gate. Simulation runs on placeholders by design; hardware does not.
    states_to_run = _states_from(start, stop_at)
    gate_or_raise(
        cfg.to_dict(), states_to_run, simulated=simulated, allow_placeholders=allow_placeholders
    )

    buses = {name: build_bus_from_config(name, bus_cfg)
             for name, bus_cfg in session.rig.buses.items()}

    codecs = {
        name: build_codec(motor.codec, motor.codec_params)
        for name, motor in session.rig.motors.items()
    }
    axes = {
        name: CANAxis(
            name=name,
            bus=buses[motor.bus],
            codec=codecs[name],
            config=motor,
            calibration=getattr(geometry, name),
            dry_run=dry_run,
        )
        for name, motor in session.rig.motors.items()
        if name in ("base", "needle")
    }
    sensors = {
        name: CANScalarSensor(name, sensor_cfg)
        for name, sensor_cfg in session.rig.sensors.items()
    }

    plant = None
    if simulated:
        from ct.plant.rig import SimulatedRig  # noqa: PLC0415 - rule 3: only the orchestrator

        source = build_source(cfg.source["name"], {**cfg.source.get("params", {}), "fs": cfg.fs})
        plant = SimulatedRig(
            rig=session.rig,
            source=source,
            codecs=codecs,
            tactile_noise_counts=_noise_counts(cfg, geometry),
            seed=cfg.seed,
        )
        # Attached to the sensing bus, which is where the motors and sensors live.
        sensing = session.rig.motors["base"].bus
        buses[sensing].attach(plant)

    servo = LeadServo(session.servo.needle, Ts=session.procedure.dt)
    latency = LatencyBudget(session.latency, tau_cl=servo.residual_lag)
    safety = SafetyMonitor(config=session.procedure.safety)

    def identify(batch: Any) -> Any:
        params = dict(cfg.identifier.get("params", {}))
        if context.breath_hold_window is not None and "breath_hold_window" not in params:
            # Rebase onto the accumulator's own clock: the identifier sees a batch whose
            # time starts at zero, not loop-monotonic time.
            t0 = context.accumulator.t0 or 0.0
            lo, hi = context.breath_hold_window
            params["breath_hold_window"] = [max(0.0, lo - t0), max(0.0, hi - t0)]
        return build_identifier(cfg.identifier["name"], params).identify(batch)

    def make_tracker() -> Any:
        return build_tracker(cfg.tracker["name"], cfg.tracker.get("params", {}))

    def log(event: str, detail: dict[str, Any]) -> None:
        if telemetry is not None:
            telemetry.write({"t": context.state.t, "event": event, **detail})

    context = ProcedureContext(
        geometry=geometry,
        axes=axes,
        sensors=sensors,
        config=session.procedure,
        latency=latency,
        safety=safety,
        servo=servo,
        build_tracker=make_tracker,
        identify=identify,
        log=log,
        dry_run=dry_run,
    )
    procedure = Procedure(context, start=start, stop_at=stop_at)
    clock = SimClock() if simulated else RealClock()

    return RigAssembly(
        session=session,
        buses=buses,
        axes=axes,
        sensors=sensors,
        context=context,
        procedure=procedure,
        clock=clock,
        plant=plant,
    )


def run_rig(
    cfg: RunConfig,
    *,
    simulated: bool | None = None,
    dry_run: bool = False,
    start: ProcedureState = ProcedureState.APPROACH,
    stop_at: ProcedureState | None = None,
    max_duration_s: float | None = None,
    telemetry_path: str | Path | None = None,
    allow_placeholders: bool = False,
    keep_records: bool = True,
) -> RigResult:
    """Run the procedure to completion, a fault, or the time limit."""
    telemetry = TelemetryWriter(telemetry_path) if telemetry_path else None
    assembly = build_rig(
        cfg,
        simulated=simulated,
        dry_run=dry_run,
        start=start,
        stop_at=stop_at,
        telemetry=telemetry,
        allow_placeholders=allow_placeholders,
    )
    procedure = assembly.procedure
    records: list[dict[str, Any]] = []

    for axis in assembly.axes.values():
        axis.enable()

    def on_tick(info: TickInfo) -> bool | None:
        record = procedure.tick(info.t, info.frames, deadline_misses=loop.stats.deadline_misses)
        record["tick"] = info.n
        if info.late_by > 0:
            record["late_by"] = info.late_by
        if telemetry is not None:
            telemetry.write(record)
        if keep_records:
            records.append(record)
        return not procedure.finished

    loop = ControlLoop(
        clock=assembly.clock,
        rate_hz=assembly.session.procedure.loop_rate_hz,
        buses=assembly.buses,
        on_tick=on_tick,
        max_deadline_misses=None,
    )

    try:
        stats = loop.run(duration_s=max_duration_s)
    finally:
        for axis in assembly.axes.values():
            axis.stop()
        if telemetry is not None:
            telemetry.close()
        assembly.close()

    summary = {
        **procedure.summary,
        "loop": stats.to_dict(),
        "latency": assembly.context.latency.stats,
        "servo": assembly.context.servo.stats,
        "horizon_breakdown": assembly.context.latency.breakdown(assembly.context.omega_r),
        "buses": {n: b.stats for n, b in assembly.buses.items()},
        "simulated": assembly.clock.is_simulated,
        "dry_run": dry_run,
    }
    if assembly.plant is not None:
        summary["plant"] = assembly.plant.stats

    return RigResult(
        config=cfg, assembly=assembly, loop_stats=stats, records=records, summary=summary
    )


def _states_from(start: ProcedureState, stop_at: ProcedureState | None) -> list[ProcedureState]:
    """States a run starting at ``start`` could reach. Drives the placeholder gate."""
    from ct.control.state import MAIN_SEQUENCE  # noqa: PLC0415

    if start not in MAIN_SEQUENCE:
        return [start]
    tail = list(MAIN_SEQUENCE[MAIN_SEQUENCE.index(start) :])
    if stop_at in tail:
        tail = tail[: tail.index(stop_at) + 1]
    return tail


def _noise_counts(cfg: RunConfig, geometry: RigGeometry) -> float:
    """Sensor noise for the simulated tactile channel, in counts.

    Taken from the signal source's ``noise_std`` so that a rig config and an estimator
    config describe the same noise level rather than two unrelated ones — the source's
    own noise is bypassed here, since the plant reads ``clean()`` and the sensor adds the
    noise at the point it is actually measured.
    """
    noise_mm = float(cfg.source.get("params", {}).get("noise_std", 0.0) or 0.0)
    return noise_mm / geometry.tactile_counts_to_mm
