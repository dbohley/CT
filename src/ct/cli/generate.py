"""``ct-generate`` — write a synthetic breathing trace to CSV.

    ct-generate --config lujan_n2 --duration 120 --noise-std 0.05 --plot
    ct-generate --source rc_piecewise --set source.params.cardiac=true --plot
"""

from __future__ import annotations

import argparse
from pathlib import Path

from ct.cli._common import add_common_args, header, print_kv, resolve_config
from ct.run import build_source_from_config


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-generate", description=__doc__.splitlines()[0])
    add_common_args(p)
    p.add_argument("--out", default=None, help="output CSV path (default: outputs/<name>/signal.csv)")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    run_dir = cfg.run_dir()
    cfg.dump()

    source = build_source_from_config(cfg)
    batch = source.batch(cfg.duration)

    from ct.sources.csv_source import write_csv

    out = Path(args.out) if args.out else run_dir / "signal.csv"
    write_csv(batch, out)

    if not args.quiet:
        header(f"generated: {cfg.source['name']}")
        print_kv(
            {
                "samples": batch.N,
                "duration_s": batch.duration,
                "fs_hz": batch.fs,
                "mean": float(batch.y.mean()),
                "peak_to_peak": float(batch.y.max() - batch.y.min()),
                "csv": str(out),
            }
        )
        if batch.truth:
            header("generator truth")
            print_kv(batch.truth)

    if args.plot:
        from ct.diagnostics.plots import plot_signal

        path = plot_signal(batch, run_dir / "signal.png", title=cfg.source["name"])
        print(f"\nfigure: {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
