"""``ct-compare`` — score the sensing against what the phantom was actually commanded.

    ct-compare outputs/rig_sim/phantom.jsonl outputs/rig_sim/controller.jsonl

The lag it reports is the cross-correlation peak between commanded phantom motion and
sensed tactile deflection. **That number is ``latency.tau_s``** — the sensor-latency term
of the forecast horizon, measured rather than assumed. It is one of the entries in
``ct-unknowns``, so this tool is how that entry gets closed out.

Both logs must come from runs that were live at the same time on the same host: alignment
is by ``time.monotonic()``, which is only comparable within one machine's uptime.

**In simulation this tool only checks its own plumbing.** ``ct-phantom`` and ``ct-rig``
each build their own simulated world on their own loopback bus, so there is no shared
phantom between them — the amplitude ratio and correlation will be poor and mean nothing.
The lag is still meaningful, because it recovers the sensor latency the simulated sensor
was configured with. On hardware there is one physical phantom and all three numbers
count.
"""

from __future__ import annotations

import argparse

from ct.cli._common import header, print_kv, save_json
from ct.phantom.driver import compare_logs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Align a phantom log against a controller log and score the sensing.",
    )
    parser.add_argument("phantom", help="phantom.jsonl from ct-phantom")
    parser.add_argument("controller", help="controller.jsonl from ct-rig")
    parser.add_argument("--max-lag", type=float, default=1.0,
                        help="widest lag to consider [s]")
    parser.add_argument("--out", default=None, help="write the result as JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = compare_logs(args.phantom, args.controller, max_lag_s=args.max_lag)
    except ValueError as exc:
        print(f"cannot compare: {exc}")
        return 2

    header("sensing vs commanded phantom motion")
    print_kv(result, indent=2)

    header("what to do with this")
    print(f"  Set  latency.tau_s: {result['lag_s']:.4f}   in your rig config.")
    if abs(result["amplitude_ratio"] - 1.0) > 0.1:
        print(
            f"  Amplitude ratio is {result['amplitude_ratio']:.3f}, not ~1. The tactile "
            "scale is off by about that factor: check rig.geometry.tactile_counts_to_mm."
        )
    if result["correlation"] < 0.9:
        print(
            f"  Correlation is only {result['correlation']:.3f}. The sensor is not tracking "
            "the phantom well — check seating before trusting anything downstream."
        )

    if args.out:
        save_json(result, args.out)
        print(f"\n  wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
