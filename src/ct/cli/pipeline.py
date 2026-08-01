"""``ct-pipeline`` — the headline script: generate/load, identify, track, forecast.

    ct-pipeline --config lujan_n2 --calib-seconds 60 --horizon 0.25 --plot
    ct-pipeline --config rc_piecewise --set source.params.cardiac=true --plot

Prints an identification report, a filter-health summary and forecast accuracy,
and writes every diagnostic figure to ``outputs/<name>/``.
"""

from __future__ import annotations

import argparse

from ct.cli._common import add_common_args, header, print_kv, resolve_config, save_json
from ct.cli.identify import report as report_identification
from ct.run import run_pipeline


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-pipeline", description=__doc__.splitlines()[0])
    add_common_args(p)
    p.add_argument(
        "--warmup-breaths",
        type=float,
        default=2.0,
        help="breaths excluded from the metrics while the filter converges",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    run_dir = cfg.run_dir()
    cfg.dump()

    result = run_pipeline(cfg, warmup_breaths=args.warmup_breaths)
    report_identification(result.ident, args.quiet)

    if not args.quiet:
        header("stage 2 — tracking health")
        print_kv({k: v for k, v in result.summary.items() if k != "forecast"})
        if "forecast" in result.summary:
            header(f"forecast at h = {cfg.horizon} s")
            print_kv(result.summary["forecast"])

    result.ident.save(run_dir / "ident.npz")
    save_json(result.summary, run_dir / "summary.json")
    print(f"\nrun directory: {run_dir}")

    if args.plot:
        from ct.diagnostics import plots

        h = result.history
        warmup = int(result.summary["warmup_steps"])
        print(f"figure: {plots.plot_signal(result.batch, run_dir / 'signal.png', cfg.name)}")
        print(
            "figure: "
            f"{plots.plot_identification(result.ident, result.calibration, run_dir / 'identification.png')}"
        )
        print(f"figure: {plots.plot_tracking(h, run_dir / 'tracking.png')}")
        print(
            f"figure: {plots.plot_states(h, result.layout, run_dir / 'states.png', result.batch.truth)}"
        )
        print(f"figure: {plots.plot_innovations(h, run_dir / 'innovations.png', warmup)}")
        if h.forecast is not None:
            print(f"figure: {plots.plot_forecast(h, run_dir / 'forecast.png', warmup)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
