"""``ct-track`` — run the EKF over a trace using a saved identification.

    ct-track --input outputs/lujan_n2/signal.csv --ident outputs/lujan_n2/ident.npz --plot

With no ``--ident`` it identifies on the first ``--calib-seconds`` first, which
makes it a shorthand for ``ct-pipeline`` restricted to one trace.
"""

from __future__ import annotations

import argparse

from ct.cli._common import add_common_args, header, print_kv, resolve_config, save_json
from ct.diagnostics import metrics
from ct.layout import StateLayout
from ct.registry import build_identifier, build_tracker
from ct.run import build_source_from_config, track, truth_function
from ct.types import IdentificationResult


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-track", description=__doc__.splitlines()[0])
    add_common_args(p)
    p.add_argument("--input", default=None, help="CSV trace to track (else generate one)")
    p.add_argument("--ident", default=None, help="identification .npz from ct-identify")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    if args.input:
        cfg = cfg.apply_overrides({"source.name": "csv", "source.params.path": args.input})
    run_dir = cfg.run_dir()
    cfg.dump()

    source = build_source_from_config(cfg)
    batch = source.batch(cfg.duration)
    t0 = float(batch.t[0])

    if args.ident:
        ident = IdentificationResult.load(args.ident)
        # Track from wherever identification finished, so no sample is used twice.
        start = ident.t0
    else:
        calib = batch.slice_time(t0, t0 + cfg.calib_seconds)
        identifier = build_identifier(cfg.identifier["name"], cfg.identifier.get("params"))
        ident = identifier.identify(calib)
        start = t0 + cfg.calib_seconds

    tracking = batch.slice_time(start + 0.5 / cfg.fs, float(batch.t[-1]) + 1.0)
    if tracking.N < 10:
        raise SystemExit(f"only {tracking.N} samples to track; extend --duration")

    tracker = build_tracker(cfg.tracker["name"], cfg.tracker.get("params"))
    history = track(
        tracker, tracking, ident, horizon=cfg.horizon, truth_at=truth_function(source, batch)
    )

    T_breath = 2.0 * 3.141592653589793 / ident.diagnostics["omega_hat"]
    warmup = int(min(history.n_steps // 2, round(2.0 * T_breath * cfg.fs)))
    summary = {
        "K": ident.K,
        "samples_tracked": history.n_steps,
        "warmup_steps": warmup,
        **metrics.summarize(history.innovation[warmup:], history.S[warmup:], history.nis[warmup:]),
    }
    if history.forecast is not None and history.forecast_target is not None:
        summary["forecast"] = metrics.forecast_errors(
            history.forecast, history.forecast_target, warmup
        )
        summary["horizon"] = cfg.horizon

    if not args.quiet:
        header("stage 2 — tracking")
        print_kv(summary)

    save_json(summary, run_dir / "tracking_summary.json")

    if args.plot:
        from ct.diagnostics import plots

        layout = StateLayout(ident.K)
        print()
        print(f"figure: {plots.plot_tracking(history, run_dir / 'tracking.png')}")
        print(f"figure: {plots.plot_states(history, layout, run_dir / 'states.png', batch.truth)}")
        print(f"figure: {plots.plot_innovations(history, run_dir / 'innovations.png', warmup)}")
        if history.forecast is not None:
            print(f"figure: {plots.plot_forecast(history, run_dir / 'forecast.png', warmup)}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
