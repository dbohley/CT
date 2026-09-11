#!/usr/bin/env python3
"""Pick a lead compensator (zero, pole, gain) against a measured needle plant, replacing
hand-tuning with a numeric phase-margin sweep -- per src/ct/unknowns.py's own text on
`servo.needle.lead`: "a design *output* ... should be re-derived rather than hand-tuned once
the plant is known." Uses the already-tested frequency-response functions in
src/ct/control/servo.py (`plant_response`, `lead_response`, `closed_loop_response`,
`bandwidth`, `residual_lag`) rather than re-deriving any control theory here.

**Why this needed doing now.** The placeholder lead (`zero=8, pole=80, gain=150`) was sized
against the placeholder plant's `wn=60` (hw/config.py's own docstring: "crosses over near
63 rad/s"). The measured plant from docs/sessions/019 has `wn=27` -- less than half -- so that
same lead would cross over *above* the real axis's resonance, which is exactly the "closed loop
tracks a fraction of its reference and leads instead of lags" failure mode servo.py's own
docstring warns about. This does not need to be optimal (see the module's own scope note):
just no longer built against a fictitious plant.

**Method.** For a grid of `(zero, alpha=pole/zero, gain)`, compute the open-loop phase margin
and crossover frequency. Keep only combinations with margin at least `--min-margin-deg`
(default 40, hw/config.py's own "40 degrees is fine" floor), crossover no higher than
`--max-crossover-frac * wn` (default 0.5 -- comfortably below the plant's own resonance, so the
design doesn't depend on exactly where that peak sits), AND a third constraint added in session
021 (see below). Among survivors, the one with the lowest `residual_lag()` at
`--nominal-breathing-hz` wins -- that is the number that actually feeds the forecast horizon
`h`, so "fastest within a safe margin" is the right objective, not "hit an exact target margin":
for a plant this lightly damped (this one's `zeta` after measurement is ~0.35), phase margin is
not smooth or monotonic in gain -- it can hold near a large, very safe value across a wide
low-gain range and then fall sharply within a narrow gain window as the crossover jumps past the
resonance peak, so bisecting for one exact number can land you right next to that cliff instead
of a genuinely safe distance from it.

**The noise/lag-amplification constraint (session 021).** The original version of this script
picked gain purely from phase margin, with no accounting for how much it amplifies real
measurement error -- and a real hardware run of `scripts/run_needle_lead_tracking_live.py`
found that gap the hard way: the resulting gain=12.74 design oscillated visibly, tracing back to
real (not sensor-noise) tracking lag of ~0.14mm std that the compensator was amplifying by up to
its own high-frequency gain into a multi-millimetre correction demand. `--max-lag-std-mm` is
that measured lag (from the uncompensated phase of a live run, or
`scripts/measure_needle_position_noise.py`'s at-rest measurement, whichever is larger and more
representative of real operating conditions); `--max-projected-correction-mm` caps
`max_lag_std_mm * gain` -- the same conservative, worst-case-gain projection
`measure_needle_position_noise.py` already reports -- and any gain that would exceed it is
rejected before the phase-margin/crossover check even runs.

    python scripts/design_needle_lead.py
    python scripts/design_needle_lead.py --plant-k 0.92 --plant-wn 27.0 --plant-zeta 0.35
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from ct.control.servo import bandwidth, lead_response, plant_response, residual_lag
from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel

OMEGA_GRID = np.logspace(-3, 3, 20000)  # rad/s, spans far below breathing to far above any wn tested
GAIN_GRID = np.logspace(-1, 4, 400)  # 0.1 to 10000, dense enough to resolve a narrow safe window


def open_loop_phase_margin(plant: PlantModel, lead: LeadCompensator) -> tuple[float, float]:
    """Phase margin [deg] and crossover frequency [rad/s] of L = C*G, at the *first* (lowest
    frequency) point the magnitude falls through 0dB -- the classic gain-crossover definition.
    Phase is unwrapped across the whole grid before reading it at the crossover, because a
    type-1 plant's phase passes through -180 degrees well before any interesting crossover, and
    a single-point `angle()` call there would silently wrap into the wrong branch."""
    L = lead_response(lead, OMEGA_GRID) * plant_response(plant, OMEGA_GRID)
    mag_db = 20 * np.log10(np.abs(L))
    phase_deg = np.degrees(np.unwrap(np.angle(L)))
    falling = np.flatnonzero(np.diff(np.sign(mag_db)) < 0)
    if falling.size == 0:
        return float("nan"), float("nan")
    i = falling[0]
    log_lo, log_hi = np.log10(OMEGA_GRID[i]), np.log10(OMEGA_GRID[i + 1])
    log_w = np.interp(0.0, [mag_db[i], mag_db[i + 1]], [log_lo, log_hi])
    phase_at_c = np.interp(log_w, [log_lo, log_hi], [phase_deg[i], phase_deg[i + 1]])
    return 180.0 + float(phase_at_c), float(10**log_w)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plant-k", type=float, default=0.92, dest="plant_k")
    parser.add_argument("--plant-wn", type=float, default=27.0, dest="plant_wn")
    parser.add_argument("--plant-zeta", type=float, default=0.35, dest="plant_zeta")
    parser.add_argument("--min-margin-deg", type=float, default=40.0, dest="min_margin_deg",
                         help="reject candidates below this phase margin -- hw/config.py's own "
                              "documented floor")
    parser.add_argument("--max-crossover-frac", type=float, default=0.5, dest="max_crossover_frac",
                         help="reject candidates whose crossover exceeds this fraction of plant wn")
    parser.add_argument("--nominal-breathing-hz", type=float, default=0.25, dest="nominal_breathing_hz",
                         help="breathing rate to score residual_lag() at -- 0.25Hz = 15 breaths/min")
    parser.add_argument("--zero-candidates", type=str, default="1,2,3,4,6", dest="zero_candidates")
    parser.add_argument("--alpha-candidates", type=str, default="5,8,10,12,15", dest="alpha_candidates")
    parser.add_argument("--max-lag-std-mm", type=float, default=0.139, dest="max_lag_std_mm",
                         help="measured real tracking-lag std (NOT sensor noise -- see "
                              "scripts/measure_needle_position_noise.py's at-rest number, which "
                              "is much smaller) to design against. Default 0.139mm is the "
                              "uncompensated-phase measurement from a live run at 20Hz command "
                              "rate (session 021) -- re-measure and override if the command "
                              "rate or reference changes.")
    parser.add_argument("--max-projected-correction-mm", type=float, default=0.5,
                         dest="max_projected_correction_mm",
                         help="reject any gain for which max_lag_std_mm * gain (the same "
                              "worst-case-gain projection measure_needle_position_noise.py "
                              "reports) would exceed this -- keeps the compensator's normal-case "
                              "correction well under configs/rig_bench.yaml's "
                              "correction_limit_mm, rather than routinely pinning it")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    plant = PlantModel(K=args.plant_k, wn=args.plant_wn, zeta=args.plant_zeta)
    zeros = [float(z) for z in args.zero_candidates.split(",")]
    alphas = [float(a) for a in args.alpha_candidates.split(",")]
    omega_r_nominal = 2 * math.pi * args.nominal_breathing_hz
    max_crossover = args.max_crossover_frac * plant.wn

    print(f"plant: K={plant.K}, wn={plant.wn}, zeta={plant.zeta}")
    print(f"constraints: margin >= {args.min_margin_deg}deg, crossover <= "
          f"{args.max_crossover_frac}*wn = {max_crossover:.2f} rad/s, "
          f"{args.max_lag_std_mm:.3f}mm lag * gain <= {args.max_projected_correction_mm:.2f}mm "
          f"(gain <= {args.max_projected_correction_mm / max(args.max_lag_std_mm, 1e-9):.2f})")
    print(f"objective: minimize residual_lag() at {args.nominal_breathing_hz}Hz "
          f"({omega_r_nominal:.3f} rad/s) among candidates meeting all three\n")

    max_gain = args.max_projected_correction_mm / max(args.max_lag_std_mm, 1e-9)

    best = None
    for zero in zeros:
        for alpha in alphas:
            pole = zero * alpha
            best_for_pair = None
            for gain in GAIN_GRID:
                if gain > max_gain:
                    break  # GAIN_GRID is sorted ascending -- nothing past this point can pass
                lead = LeadCompensator(zero=zero, pole=pole, gain=float(gain))
                margin, omega_c = open_loop_phase_margin(plant, lead)
                if not np.isfinite(margin) or margin < args.min_margin_deg or omega_c > max_crossover:
                    continue
                config = AxisServoConfig(plant=plant, lead=lead)
                lag = residual_lag(config, omega_r_nominal)
                candidate = {"zero": zero, "pole": pole, "gain": float(gain), "margin_deg": margin,
                             "omega_c": omega_c, "bandwidth": bandwidth(config), "residual_lag_s": lag,
                             "projected_correction_mm": args.max_lag_std_mm * gain}
                if best_for_pair is None or lag < best_for_pair["residual_lag_s"]:
                    best_for_pair = candidate

            if best_for_pair is None:
                continue
            print(f"  zero={zero:5.2f} pole={pole:6.2f} (alpha={alpha:4.1f}): best gain="
                  f"{best_for_pair['gain']:8.2f} -> margin={best_for_pair['margin_deg']:5.1f}deg  "
                  f"crossover={best_for_pair['omega_c']:6.2f}rad/s  "
                  f"residual_lag={best_for_pair['residual_lag_s'] * 1000:6.1f}ms  "
                  f"projected_correction={best_for_pair['projected_correction_mm']:5.2f}mm")
            if best is None or best_for_pair["residual_lag_s"] < best["residual_lag_s"]:
                best = best_for_pair

    if best is None:
        print("\nno (zero, pole, gain) combination met all three constraints -- widen "
              "--zero-candidates/--alpha-candidates, raise --max-crossover-frac or "
              "--max-projected-correction-mm, or lower --min-margin-deg. If nothing works "
              "except relaxing --max-projected-correction-mm a lot, that itself is a finding: "
              "this plant/lag combination may not support a lead compensator with the phase "
              "margin this needs without also amplifying real tracking error past what's usable.")
        return 1

    print(f"\nchosen (lowest residual_lag among all candidates meeting all three constraints): "
          f"zero={best['zero']:.2f}, pole={best['pole']:.2f}, gain={best['gain']:.2f}")
    print(f"  phase margin {best['margin_deg']:.1f}deg at crossover {best['omega_c']:.2f}rad/s "
          f"({best['omega_c'] / plant.wn * 100:.0f}% of plant wn)")
    print(f"  closed-loop bandwidth {best['bandwidth']:.2f}rad/s")
    print(f"  residual_lag at {args.nominal_breathing_hz}Hz: {best['residual_lag_s'] * 1000:.1f}ms")
    print(f"  projected correction from {args.max_lag_std_mm:.3f}mm of real lag: "
          f"{best['projected_correction_mm']:.2f}mm (cap was {args.max_projected_correction_mm:.2f}mm)")
    print(f"\nfor configs/rig_bench.yaml: lead: {{zero: {best['zero']:.2f}, pole: {best['pole']:.2f}, "
          f"gain: {best['gain']:.2f}}}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
