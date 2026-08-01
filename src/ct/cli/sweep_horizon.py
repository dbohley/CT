"""``ct-sweep-horizon`` — forecast error as a function of the horizon ``h``.

    ct-sweep-horizon --config lujan_n2 --horizons 0.05,0.1,0.2,0.4,0.8 --plot

The curve should grow *smoothly* with ``h``. A jump or a sawtooth is the
signature of the phase-rotation bug: advancing the model by rotating all
harmonics through one common angle instead of advancing ``theta``, which
distorts waveform shape rather than shifting it in time.
"""

from __future__ import annotations

import argparse

import numpy as np

from ct.cli._common import add_common_args, header, print_table, resolve_config, save_json
from ct.run import sweep_horizons


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-sweep-horizon", description=__doc__.splitlines()[0])
    add_common_args(p)
    p.add_argument(
        "--horizons",
        default="0.05,0.1,0.2,0.4,0.8",
        help="comma-separated horizons in seconds",
    )
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    run_dir = cfg.run_dir()
    cfg.dump()

    horizons = [float(x) for x in args.horizons.split(",") if x.strip()]
    rows = sweep_horizons(cfg, horizons)

    if not args.quiet:
        header(f"forecast error vs horizon — {cfg.name}")
        print_table(rows, ["horizon", "rmse", "mae", "max_abs", "bias", "n"])

    save_json(rows, run_dir / "horizon_sweep.json")

    if args.plot:
        from ct.diagnostics.plots import plot_horizon_sweep

        amp = None
        truth = (cfg.source.get("params") or {})
        for key in ("a", "amplitude"):
            if key in truth:
                amp = float(truth[key])
                break
        path = plot_horizon_sweep(rows, run_dir / "horizon_sweep.png", amplitude=amp)
        print(f"\nfigure: {path}")

    # A useful one-glance number: how far ahead can we see before the error
    # exceeds 5% of the signal's own peak-to-peak swing?
    rmses = np.array([r["rmse"] for r in rows])
    if not args.quiet and rmses.size > 1:
        print(f"\nRMSE growth from h={horizons[0]:g}s to h={horizons[-1]:g}s: "
              f"{rmses[0]:.4g} -> {rmses[-1]:.4g} ({rmses[-1] / max(rmses[0], 1e-12):.1f}x)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
