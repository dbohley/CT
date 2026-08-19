"""``ct-phantom`` — drive the breathing phantom from a waveform.

A separate process from ``ct-rig``, on its own bus, by design: the controller must not be
able to see what the phantom was told to do, or comparing the two would prove nothing.

    ct-phantom --config rig_sim --duration 120
    ct-phantom --config rig_bench --source lujan --set source.params.n=2
    ct-phantom --config rig_bench --replay recordings/subject01.csv

Logs ``(t_monotonic, commanded_mm, measured_mm)`` to ``outputs/<name>/phantom.jsonl``.
Pair it with ``ct-compare`` afterwards to score the sensing and measure ``tau_s``.
"""

from __future__ import annotations

import argparse

from ct.cli._common import header, print_kv, resolve_config, save_json
from ct.geometry import AxisCalibration
from ct.hw.bus import build_bus_from_config
from ct.hw.config import RigSession
from ct.phantom.driver import PhantomDriver, PhantomLimits
from ct.registry import build_codec, build_source
from ct.rt.clock import RealClock, SimClock
from ct.rt.loop import ControlLoop, TickInfo
from ct.rt.telemetry import TelemetryWriter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Drive the breathing phantom.")
    parser.add_argument("--config", "-c", default="rig_sim")
    parser.add_argument("--name", default=None)
    parser.add_argument("--source", default=None, help="waveform source name")
    parser.add_argument("--replay", default=None, metavar="CSV",
                        help="replay a recorded breathing trace (shorthand for --source csv)")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--rate", type=float, default=200.0, help="command rate [Hz]")
    parser.add_argument("--center", type=float, default=30.0,
                        help="phantom mid-position [mm]; the waveform swings about this")
    parser.add_argument("--ramp", type=float, default=3.0,
                        help="fade-in/out [s], so the phantom never jerks into motion")
    parser.add_argument("--motor", default="phantom",
                        help="which entry in rig.motors drives the phantom")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="KEY=VALUE")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.replay:
        args.overrides = list(args.overrides) + [f"source.params.path={args.replay}"]
        args.source = "csv"
    cfg = resolve_config(args)
    run_dir = cfg.run_dir()

    session = RigSession.from_run_config(cfg)
    if args.motor not in session.rig.motors:
        print(
            f"config has no rig.motors.{args.motor}. Known: {sorted(session.rig.motors)}.\n"
            "The phantom motor needs its own entry, on its own bus."
        )
        return 2

    motor_cfg = session.rig.motors[args.motor]
    bus_cfg = session.rig.buses[motor_cfg.bus]
    simulated = bus_cfg.backend == "loopback" and not args.real

    bus = build_bus_from_config(motor_cfg.bus, bus_cfg)
    codec = build_codec(motor_cfg.codec, motor_cfg.codec_params)
    source = build_source(cfg.source["name"], {**cfg.source.get("params", {}), "fs": cfg.fs})

    calibration = AxisCalibration(
        counts_per_mm=session.rig.geometry.base.counts_per_mm,
        direction=1,
        travel_mm=(0.0, 2.0 * args.center),
        v_max_mm_s=200.0,
    )
    limits = PhantomLimits(travel_mm=calibration.travel_mm, ramp_s=args.ramp)

    if simulated:
        from ct.plant.rig import MotorPhysics, SimulatedMotor  # noqa: PLC0415

        bus.attach(
            _PhantomDevice(
                SimulatedMotor(
                    name="phantom",
                    config=motor_cfg,
                    codec=codec,
                    physics=MotorPhysics(inertia=0.05, damping=1.0),
                    initial_position=calibration.to_counts(args.center),
                )
            )
        )

    telemetry = TelemetryWriter(run_dir / "phantom.jsonl")
    driver = PhantomDriver(
        bus=bus, codec=codec, config=motor_cfg, calibration=calibration, source=source,
        center_mm=args.center, limits=limits, telemetry=telemetry, dry_run=args.dry_run,
    )
    driver.enable()

    clock = SimClock() if simulated else RealClock()

    def on_tick(info: TickInfo) -> None:
        driver.step(info.t, info.frames.get(motor_cfg.bus, []), duration=args.duration)

    loop = ControlLoop(clock=clock, rate_hz=args.rate, buses={motor_cfg.bus: bus},
                       on_tick=on_tick)
    try:
        stats = loop.run(duration_s=args.duration)
    finally:
        driver.disable()
        telemetry.close()
        bus.close()

    summary = {"driver": driver.stats, "loop": stats.to_dict(), "simulated": simulated,
               "source": cfg.source["name"]}
    save_json(summary, run_dir / "phantom_summary.json")

    if not args.quiet:
        header("phantom")
        print_kv(summary, indent=2)
        print(f"\n  log: {run_dir / 'phantom.jsonl'}")
    return 0


class _PhantomDevice:
    """Adapts a :class:`SimulatedMotor` to the loopback bus's device protocol."""

    def __init__(self, motor) -> None:
        self.motor = motor
        self._last = 0.0

    def on_frame(self, t: float, can_id: int, data: bytes, extended: bool) -> None:
        self.motor.on_frame(can_id, data)

    def emit(self, t: float) -> list[tuple[float, int, bytes]]:
        dt = t - self._last
        if dt > 0:
            steps = max(1, int(dt / 0.001))
            for _ in range(steps):
                self.motor.step(dt / steps)
            self._last = t
        return self.motor.emit(t)


if __name__ == "__main__":
    raise SystemExit(main())
