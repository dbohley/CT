"""``ct-compare`` — score the sensing against what the phantom was actually commanded.

    ct-compare outputs/rig_sim/phantom.jsonl outputs/rig_sim/controller.jsonl

The lag it reports is the cross-correlation peak between phantom motion and sensed
deflection. **It is the whole sensing lag, and on the tactile chain most of it is not
latency at all.**

Run it against ``--sensor tof_mm`` as well and the split is plain. The ToF is non-contact
but sits on the same CAN bus, in the same tick loop, watching the same motion, and it lags
0.014-0.100 s across six bench runs where the tactile arm lags 0.279-0.566 s. The
difference — 0.18-0.54 s — is viscoelastic settling in the *contact*, not latency in the
sensor, and it is what the historical 0.677 s figure was mostly made of. Only the ToF-class
floor belongs in ``latency.tau_s``; the contact excess is a mechanical property of how the
arm is seated, and it moves with seating depth rather than staying constant.

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
    parser.add_argument("--sensor", default="tactile_mm",
                        help="which controller column to score (default tactile_mm). "
                             "tof_mm measures the non-contact path over the same bus and "
                             "tick loop, which is what separates sensing latency from "
                             "contact settling.")
    parser.add_argument("--no-split", action="store_true",
                        help="skip the tactile-vs-ToF latency split")
    parser.add_argument("--out", default=None, help="write the result as JSON")
    return parser


def _reference_lag(args) -> dict | None:
    """The non-contact lag, for splitting sensing latency from contact settling."""
    if args.no_split or args.sensor != "tactile_mm":
        return None
    try:
        return compare_logs(
            args.phantom, args.controller,
            max_lag_s=args.max_lag, phase=args.phase, phantom_field=args.truth,
            sensor_field="tof_mm",
        )
    except ValueError:
        return None  # older logs have no tof_mm; the split is a bonus, not a requirement


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result = compare_logs(
            args.phantom, args.controller,
            max_lag_s=args.max_lag, phase=args.phase, phantom_field=args.truth,
            sensor_field=args.sensor,
        )
    except ValueError as exc:
        print(f"cannot compare: {exc}")
        return 2
    reference = _reference_lag(args)

    scope = f" in phase '{args.phase}'" if args.phase else " over the whole run"
    header(f"sensing vs {result['phantom_field_used']} phantom motion{scope}")
    print_kv(result, indent=2)

    header("what to do with this")
    if reference is not None:
        floor = reference["lag_s"]
        excess = result["lag_s"] - floor
        print(f"  total sensing lag   {result['lag_s']:.4f} s   ({args.sensor} vs phantom)")
        print(f"  sensing floor       {floor:.4f} s   (tof_mm, non-contact, same bus/tick)")
        print(f"  contact excess      {excess:.4f} s   (viscoelastic settling in the contact)")
        print(
            f"\n  Set  latency.tau_s: {floor:.4f}  -- the floor is the part that is really "
            "sensor latency,\n  and the only part a non-contact clinical sensor would still "
            "have. Treat the ToF figure as\n  an upper bound: it quantises to 1mm on a ~4.8mm "
            "excursion, so its correlation is only\n  ~0.3-0.5. It is consistent with the "
            "7.9ms frame period plus the ~10.7ms tick.\n"
            f"\n  The forecast still has to cover the whole {result['lag_s']:.4f} s while the "
            "tactile arm is the\n  sensor -- but that total is not a constant. It tracks how "
            "hard the arm is seated\n  (lag/seat-depth correlation +0.63 over six bench runs), "
            "so measure it per run rather\n  than freezing it in a config."
        )
    else:
        print(
            f"  Total sensing lag {result['lag_s']:.4f} s. This is NOT all latency: on the "
            "tactile chain\n  most of it is viscoelastic settling in the contact. Re-run "
            "with --sensor tof_mm to\n  measure the non-contact floor and split the two; "
            "only the floor belongs in latency.tau_s."
        )
    if result["lag_ambiguous"]:
        print(
            f"\n  --max-lag {args.max_lag:.2f}s exceeds half the {result['breath_period_s']:.2f}s "
            f"breath, so the search was clamped to +/-{result['max_lag_searched_s']:.2f}s. "
            "Beyond that a lag\n  is indistinguishable from the same lag one breath over."
        )
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
            "fraction of the phantom's real excursion reaches the sensor. It is mechanical "
            "(the lever deflects, the skin deforms), not a scale error, and it is almost "
            "entirely a STATIC loss: the measured lag alone would only account for a factor "
            "of 0.83-0.95. A ratio far from both 1 and the 0.16-0.76 bench range is worth "
            "checking against rig.geometry.tactile_counts_to_mm."
        )
    if result["correlation"] < 0.9:
        print(
            f"  Correlation is only {result['correlation']:.3f} even with the "
            f"{result['lag_s']:.3f}s lag removed. The sensor is not tracking the phantom "
            "well — check seating before trusting anything downstream. Seat LIGHTER, not "
            "harder: across six bench runs a 0.36mm seat gave 0.28s lag / 0.76 amplitude / "
            "r=0.99, while a 1.93mm seat gave 0.57s / 0.16 / r=0.81."
        )

    if args.out:
        save_json(result, args.out)
        print(f"\n  wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
