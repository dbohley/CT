"""``ct-compare`` — score the sensing against what the phantom was actually commanded.

    ct-compare outputs/rig_sim/phantom.jsonl outputs/rig_sim/controller.jsonl

The lag it reports is the cross-correlation peak between commanded phantom motion and
sensed tactile deflection. **That number is ``latency.tau_s``** — the sensor-latency term
of the forecast horizon, measured rather than assumed. It is one of the entries in
``ct-unknowns``, so this tool is how that entry gets closed out.

Both logs must come from runs that were live at the same time on the same host: alignment
is by ``time.monotonic()``, which is only comparable within one machine's uptime.

**Pass ``--phase`` on a real bench run.** Scored across a whole ``approach_and_seat`` run —
approach, seat and standoff included, where the base is moving and the sensor is not yet
seated — run ``20260901-165415`` reports correlation 0.039. Scored over its ``standoff_hold``
alone the same data gives 0.599, and 0.961 once the measured lag is removed. The unrestricted
number is not a weaker version of the answer; it is a different, meaningless one.

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
    parser.add_argument("--phase", default=None,
                        help="score only controller records in this procedure state "
                             "(e.g. standoff_hold). Strongly recommended on a bench run -- "
                             "see this module's docstring for why.")
    parser.add_argument("--truth", default="auto",
                        choices=["auto", "commanded_mm", "measured_mm"],
                        help="what counts as phantom ground truth: what the profile asked "
                             "for, what the motor's status broadcast says it did, or auto "
                             "(measured with fallback to commanded)")
    parser.add_argument("--out", default=None, help="write the result as JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = compare_logs(
            args.phantom, args.controller,
            max_lag_s=args.max_lag, phase=args.phase, phantom_field=args.truth,
        )
    except ValueError as exc:
        print(f"cannot compare: {exc}")
        return 2

    scope = f" in phase '{args.phase}'" if args.phase else " over the whole run"
    header(f"sensing vs {result['phantom_field_used']} phantom motion{scope}")
    print_kv(result, indent=2)

    header("what to do with this")
    print(f"  Set  latency.tau_s: {result['lag_s']:.4f}   in your rig config.")
    if result["lag_at_search_edge"]:
        print(
            f"  The lag peak ({result['lag_s']:.4f}s) is against the edge of the "
            f"+/-{args.max_lag:.2f}s search range -- that is the range running out, not a "
            "measurement. Re-run with a larger --max-lag."
        )
    if args.phase is None:
        print(
            "  No --phase given, so this scored the whole run. On a bench run that mixes "
            "approach/seat/standoff with the sensor unseated, the result is meaningless -- "
            "pass --phase standoff_hold."
        )
    if result["seams"]:
        print(
            f"  {result['seams']} profile loop-restart discontinuit(ies) in the phantom "
            "trace. No sensor can track those; they inflate the RMSE slightly."
        )
    if abs(result["amplitude_ratio"] - 1.0) > 0.1:
        print(
            f"  Amplitude ratio is {result['amplitude_ratio']:.3f}, not ~1 -- only that "
            "fraction of the phantom's real excursion reaches the sensor. Measured at ~0.33 "
            "on the bench, where it is mechanical (the lever deflects, the skin deforms), "
            "not a scale error. A ratio far from both 1 and the bench value is worth "
            "checking against rig.geometry.tactile_counts_to_mm."
        )
    if result["correlation"] < 0.9:
        print(
            f"  Correlation is only {result['correlation']:.3f} even with the "
            f"{result['lag_s']:.3f}s lag removed. The sensor is not tracking the phantom "
            "well — check seating before trusting anything downstream."
        )

    if args.out:
        save_json(result, args.out)
        print(f"\n  wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
