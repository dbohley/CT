"""``ct-rig`` — run the four-state insertion procedure, simulated or on hardware.

    ct-rig --config rig_sim                       full procedure, simulated
    ct-rig --config rig_sim --stop-at estimate    stop once the model has converged
    ct-rig --config rig_bench --dry-run           real bus, commands suppressed

``--dry-run`` is the first thing to run against real hardware: it exercises the bus, the
codecs, the geometry and the whole state machine with every motor command suppressed, so a
sign error in ``rig.geometry`` shows up as a log line rather than as motion.
"""

from __future__ import annotations

import argparse

from ct.cli._common import header, print_kv, resolve_config, save_json
from ct.control.state import ProcedureState
from ct.hw.motors.axis import AxisLimitError
from ct.rig import run_rig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the needle-insertion procedure.")
    parser.add_argument("--config", "-c", default="rig_sim", help="YAML config (name or path)")
    parser.add_argument("--name", default=None, help="run name; names the outputs/ subdirectory")
    parser.add_argument("--set", dest="overrides", action="append", default=[],
                        metavar="KEY=VALUE", help="override any config field (repeatable)")
    parser.add_argument("--from-state", default="approach",
                        help="start here instead of APPROACH, for bench testing one piece")
    parser.add_argument("--stop-at", default=None, help="finish once this state is reached")
    parser.add_argument("--max-duration", type=float, default=1200.0,
                        help="wall/sim seconds before giving up")
    parser.add_argument("--dry-run", action="store_true",
                        help="suppress every motor command; exercise everything else")
    parser.add_argument("--real", action="store_true",
                        help="force hardware mode even if all buses are loopback")
    parser.add_argument("--allow-placeholders", action="store_true",
                        help="run on hardware with unmeasured values (see ct-unknowns)")
    parser.add_argument("--plot", action="store_true", help="write diagnostic figures")
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    run_dir = cfg.run_dir()
    cfg.dump()

    try:
        start = ProcedureState(args.from_state)
        stop_at = ProcedureState(args.stop_at) if args.stop_at else None
    except ValueError as exc:
        print(f"{exc}. Known states: {', '.join(s.value for s in ProcedureState)}")
        return 2

    try:
        result = run_rig(
            cfg,
            simulated=False if args.real else None,
            dry_run=args.dry_run,
            start=start,
            stop_at=stop_at,
            max_duration_s=args.max_duration,
            telemetry_path=run_dir / "controller.jsonl",
            allow_placeholders=args.allow_placeholders,
        )
    except (AxisLimitError, ValueError, KeyError) as exc:
        # AxisLimitError subclasses RuntimeError, so it has to be caught first.
        print(f"\nconfiguration error:\n\n{exc}")
        return 2
    except RuntimeError as exc:
        # The placeholder gate, and any other refusal to start. These are operator-facing
        # messages that already say what to do; a traceback would bury them.
        print(f"\nrefusing to run:\n\n{exc}")
        return 2

    save_json(result.summary, run_dir / "rig_summary.json")

    if not args.quiet:
        header("transitions")
        for entry in result.assembly.procedure.history:
            print(f"  {entry['t']:9.2f}s  {entry['from']:>9} -> {entry['to']:<9} {entry['why']}")
        if not result.assembly.procedure.history:
            print("  (none — the procedure never left its starting state)")

        header("timing")
        print_kv(result.summary["loop"], indent=2)

        header("horizon")
        print_kv(result.summary["horizon_breakdown"], indent=2)

        if result.summary["safety"]["tripped"]:
            header("safety")
            for trip in result.summary["safety"]["trips"]:
                print(f"  [{trip['kind']}] at {trip['t']:.2f}s: {trip['detail']}")

        header("result")
        print(f"  final state : {result.final_state.value}")
        print(f"  telemetry   : {run_dir / 'controller.jsonl'}")
        print(f"  summary     : {run_dir / 'rig_summary.json'}")

    if args.plot:
        from ct.diagnostics import rig_plots  # noqa: PLC0415 - matplotlib is slow to import

        written = rig_plots.plot_run(result, run_dir)
        if not args.quiet:
            header("figures")
            for path in written:
                print(f"  {path}")

    return 0 if result.reached_target else 1


if __name__ == "__main__":
    raise SystemExit(main())
