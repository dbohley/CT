"""``ct-validate-k`` — check the 95%-energy K rule against the reference models.

    ct-validate-k --kmax 10
    ct-validate-k --kmax 10 --noise-std 0.05 --duration 240

Expected from prior analysis of the Lujan model: ``n=1 -> K=1``, ``n=2 -> K=2``,
``n=3 -> K=2`` (the third harmonic of ``n=3`` carries under 0.4% of the energy).

The open question this script exists to answer is the RC-piecewise case: that
model has a possible kink at the inhale/exhale transition, and a kink makes
Fourier coefficients decay more slowly, which could push the required ``K`` above
what Lujan alone suggests. There is no pre-agreed expected value for it — the
script reports what it finds.
"""

from __future__ import annotations

import argparse
import warnings

import numpy as np

from ct.cli._common import add_common_args, header, print_table, resolve_config, save_json
from ct.identification.fft_identifier import FFTHarmonicIdentifier
from ct.registry import build_source

CASES = [
    ("sinusoid", {}, 1),
    ("lujan", {"n": 1}, 1),
    ("lujan", {"n": 2}, 2),
    ("lujan", {"n": 3}, 2),
    ("rc_piecewise", {}, None),
    ("rc_piecewise", {"cardiac": True}, None),
]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="ct-validate-k", description=__doc__.splitlines()[0])
    add_common_args(p)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = resolve_config(args)
    Kmax = int((cfg.identifier.get("params") or {}).get("Kmax", 10))
    threshold = float((cfg.identifier.get("params") or {}).get("energy_threshold", 0.95))
    noise_std = float((cfg.source.get("params") or {}).get("noise_std", 0.0))

    identifier = FFTHarmonicIdentifier(Kmax=Kmax, energy_threshold=threshold)
    rows, failures = [], []

    for name, params, expected in CASES:
        source = build_source(name, {"fs": cfg.fs, "noise_std": noise_std, "seed": cfg.seed, **params})
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # frequency cross-check chatter is not the point here
            result = identifier.identify(source.batch(cfg.duration))
        d = result.diagnostics
        label = name + ("".join(f" {k}={v}" for k, v in params.items()) if params else "")
        ok = "-" if expected is None else ("PASS" if result.K == expected else "FAIL")
        if ok == "FAIL":
            failures.append((label, expected, result.K))
        rows.append(
            {
                "model": label,
                "K": result.K,
                "expected": "-" if expected is None else expected,
                "check": ok,
                "bpm_hat": d["bpm_hat"],
                "energy@K": d["cumulative_energy"][result.K - 1],
                "A1": d["amplitudes_wide"][0],
                "A2/A1": d["amplitudes_wide"][1] / max(d["amplitudes_wide"][0], 1e-12),
                "A3/A1": d["amplitudes_wide"][2] / max(d["amplitudes_wide"][0], 1e-12),
            }
        )

    header(f"K selection at {threshold:.0%} energy, Kmax={Kmax}, noise_std={noise_std:g}")
    print_table(rows)

    header("cumulative energy by harmonic")
    for name, params, _ in CASES:
        source = build_source(name, {"fs": cfg.fs, "noise_std": noise_std, "seed": cfg.seed, **params})
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            r = identifier.identify(source.batch(cfg.duration))
        label = name + ("".join(f" {k}={v}" for k, v in params.items()) if params else "")
        cum = np.array2string(r.diagnostics["cumulative_energy"][:6], precision=5, suppress_small=True)
        print(f"{label:28s} {cum}")

    save_json(rows, cfg.run_dir() / "validate_k.json")

    if failures:
        print("\nMISMATCHES:")
        for label, exp, got in failures:
            print(f"  {label}: expected K={exp}, got K={got}")
        return 1
    print("\nAll models with a documented expectation matched.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
