"""``ct-identify`` — run Stage 1 and report K, the harmonic table, R and Q.

    ct-identify --config lujan_n2 --plot
    ct-identify --input outputs/lujan_n2/signal.csv --kmax 8 --energy-threshold 0.95 --plot
"""

from __future__ import annotations

import argparse

import numpy as np

from ct.cli._common import add_common_args, header, print_kv, print_table, resolve_config, save_json
from ct.layout import StateLayout
from ct.registry import build_identifier
from ct.run import build_source_from_config


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-identify", description=__doc__.splitlines()[0])
    add_common_args(p)
    p.add_argument("--input", default=None, help="CSV trace to identify from (else generate one)")
    p.add_argument("--out", default=None, help="where to save the identification .npz")
    return p


def load_calibration(cfg, input_csv: str | None):
    """Calibration batch from a CSV if given, else from the configured source."""
    if input_csv:
        cfg = cfg.apply_overrides({"source.name": "csv", "source.params.path": input_csv})
    source = build_source_from_config(cfg)
    batch = source.batch(cfg.duration)
    t0 = float(batch.t[0])
    return cfg, batch.slice_time(t0, t0 + cfg.calib_seconds)


def report(result, quiet: bool = False) -> None:
    if quiet:
        return
    d = result.diagnostics
    header("stage 1 — identification")
    print_kv(
        {
            "K (energy rule)": d["K_energy_rule"],
            "K (used)": d["K_used"],
            "Kmax": d["Kmax"],
            "energy threshold": d["energy_threshold"],
            "omega_hat [rad/s]": d["omega_hat"],
            "rate [breaths/min]": d["bpm_hat"],
            "fft/autocorr disagreement": d["relative_disagreement"],
            "R": result.R,
            "R source": d["R_source"],
            "residual variance": d["residual_var"],
            "calibration window [s]": d["calibration_window"],
        }
    )

    header("harmonics")
    A = d["amplitudes_wide"]
    print_table(
        [
            {
                "k": k,
                "A_k": A[k - 1],
                "energy_frac": d["energy_fraction"][k - 1],
                "cumulative": d["cumulative_energy"][k - 1],
                "kept": "yes" if k <= result.K else "no",
            }
            for k in range(1, A.size + 1)
        ]
    )

    layout = StateLayout(result.K)
    header("initial state and uncertainty")
    print_table(
        [
            {
                "state": name,
                "s0": result.s0[i],
                "sigma0": float(np.sqrt(result.P0[i, i])),
                "sqrt(Q)": float(np.sqrt(result.Q[i, i])),
            }
            for i, name in enumerate(layout.names)
        ]
    )
    q = d["q"]
    print(f"\nQ from {q['n_breaths']} per-breath refits; omega from {q.get('omega_source')}")
    if "warning" in q:
        print(f"WARNING: {q['warning']}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    cfg, calib = load_calibration(cfg, args.input)
    run_dir = cfg.run_dir()
    cfg.dump()

    identifier = build_identifier(cfg.identifier["name"], cfg.identifier.get("params"))
    result = identifier.identify(calib)
    report(result, args.quiet)

    out = args.out or run_dir / "ident.npz"
    result.save(out)
    save_json(
        {k: v for k, v in result.diagnostics.items() if not isinstance(v, np.ndarray)},
        run_dir / "ident_summary.json",
    )
    print(f"\nsaved: {out}")

    if args.plot:
        from ct.diagnostics.plots import plot_identification

        print(f"figure: {plot_identification(result, calib, run_dir / 'identification.png')}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
