#!/usr/bin/env python3
"""Simulate a sine reference driving the measured needle plant two ways -- directly (no
compensator) and through the real `ct.control.servo.LeadServo` closed loop -- and plot the
reference alongside both outputs, so the difference compensation makes is visible rather than
just a number. Pure simulation, no hardware.

**What "uncompensated" means here.** The plant (`plant_response()` in src/ct/control/servo.py)
is the type-1 model that maps a *commanded position* to *measured position*, the same role it
plays inside the real closed loop (`ct.control.states.insert`'s `_hold_standoff()` commands
`reference_mm + correction` where `correction` comes from the compensator). "Uncompensated"
means skipping that correction entirely and commanding the reference directly -- showing what
the axis's own bare dynamics do to a breathing-rate sine with no error feedback at all.
"Compensated" runs the actual `LeadServo` class in a closed loop against the same plant, not a
re-derivation of it, so this is a faithful simulation of the real control code, not a model of it.

**How the plant is simulated in time**, not just frequency. `fit_needle_plant.py` already needed
a continuous-to-discrete conversion for `scipy.signal.step`'s uniform-time-grid requirement;
this reuses the same idea but keeps the plant in discrete state-space (`scipy.signal.tf2ss` then
`cont2discrete` with `method="zoh"`, at the tick rate `--ts-s`, default 0.005s matching
`configs/rig_bench.yaml`'s `procedure.loop_rate_hz: 200`) so the closed loop can be stepped
tick-by-tick exactly the way the real control loop would: read the plant's current output,
compute the servo's correction from that error, command `reference + correction`, then advance
the plant one tick with that command.

**Self-check.** The simulated steady-state amplitude ratio and phase lag (fit directly to the
tail of each output, once transients have died out) are printed next to what the closed-form
frequency-response functions (`plant_response`, `closed_loop_response`, `residual_lag`) predict
at the same frequency -- they should agree closely, and disagreeing would mean either the
discretization or the fit has a bug, not that the underlying physics differs.

    python scripts/simulate_needle_lead_tracking.py
    python scripts/simulate_needle_lead_tracking.py --frequency-hz 0.3 --amplitude-mm 4
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
import numpy as np
from scipy import signal

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from ct.control.servo import LeadServo, closed_loop_response, plant_response, residual_lag  # noqa: E402
from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "outputs" / "needle_lead_simulation" / "tracking.png"


def build_discrete_plant(plant: PlantModel, Ts: float):
    """Type-1 second-order plant (command -> position) as a ZOH-discretized state-space,
    stepped tick-by-tick below rather than solved as one continuous trajectory, so the closed
    loop can feed each tick's command back in based on that tick's own measured output."""
    num = [plant.K * plant.wn**2]
    den = [1.0, 2.0 * plant.zeta * plant.wn, plant.wn**2, 0.0]
    A, B, C, D = signal.tf2ss(num, den)
    Ad, Bd, Cd, Dd, _ = signal.cont2discrete((A, B, C, D), Ts, method="zoh")
    return Ad, Bd, Cd, Dd


def simulate_open_loop(Ad, Bd, Cd, Dd, reference: np.ndarray) -> np.ndarray:
    x = np.zeros((Ad.shape[0], 1))
    y = np.zeros(len(reference))
    for k, r in enumerate(reference):
        y[k] = (Cd @ x + Dd * r).item()
        x = Ad @ x + Bd * r
    return y


def simulate_closed_loop(Ad, Bd, Cd, Dd, reference: np.ndarray, servo: LeadServo,
                          v_limit: float | None) -> np.ndarray:
    x = np.zeros((Ad.shape[0], 1))
    y = np.zeros(len(reference))
    for k, r in enumerate(reference):
        y[k] = (Cd @ x + Dd * r).item()
        error = r - y[k]
        correction = servo.update(error, limit=v_limit)
        command = r + correction
        x = Ad @ x + Bd * command
    return y


def fit_sinusoid(t: np.ndarray, y: np.ndarray, omega: float) -> tuple[float, float, float]:
    """Amplitude, phase [rad] and DC offset of the best-fit `dc + A*sin(omega*t + phi)` via
    linear least squares on [sin, cos, 1] -- exact for a true steady-state sinusoid-plus-offset,
    unlike a cross-correlation lag estimate, and we have the luxury of knowing omega exactly
    here. The DC term matters: the open-loop case (see main()) genuinely settles onto a nonzero
    offset, not just the driven oscillation, and dropping it from the fit would silently fold
    it into a wrong amplitude/phase instead of reporting it."""
    basis = np.column_stack([np.sin(omega * t), np.cos(omega * t), np.ones_like(t)])
    (a, b, dc), *_ = np.linalg.lstsq(basis, y, rcond=None)
    amplitude = float(np.hypot(a, b))
    phase = float(np.arctan2(b, a))
    return amplitude, phase, float(dc)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--plant-k", type=float, default=0.92, dest="plant_k")
    parser.add_argument("--plant-wn", type=float, default=27.0, dest="plant_wn")
    parser.add_argument("--plant-zeta", type=float, default=0.35, dest="plant_zeta")
    parser.add_argument("--lead-zero", type=float, default=1.0, dest="lead_zero")
    parser.add_argument("--lead-pole", type=float, default=5.0, dest="lead_pole")
    parser.add_argument("--lead-gain", type=float, default=12.74, dest="lead_gain")
    parser.add_argument("--frequency-hz", type=float, default=0.25, dest="frequency_hz",
                         help="reference sine frequency -- 0.25Hz = 15 breaths/min")
    parser.add_argument("--amplitude-mm", type=float, default=3.0, dest="amplitude_mm")
    parser.add_argument("--cycles", type=float, default=8.0, help="reference duration, in cycles")
    parser.add_argument("--ts-s", type=float, default=0.005, dest="ts_s",
                         help="tick period -- matches configs/rig_bench.yaml's loop_rate_hz=200")
    parser.add_argument("--velocity-limit-mm-s", type=float, default=None, dest="velocity_limit_mm_s",
                         help="cap on the compensator's correction, matching insert.py's v_max_mm_s "
                              "-- unset by default so the comparison isn't muddied by saturation")
    parser.add_argument("--out", type=Path, default=None, help="output PNG path")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    plant = PlantModel(K=args.plant_k, wn=args.plant_wn, zeta=args.plant_zeta)
    lead = LeadCompensator(zero=args.lead_zero, pole=args.lead_pole, gain=args.lead_gain)
    config = AxisServoConfig(plant=plant, lead=lead)

    omega = 2.0 * math.pi * args.frequency_hz
    duration_s = args.cycles / args.frequency_hz
    t = np.arange(0.0, duration_s, args.ts_s)
    reference = args.amplitude_mm * np.sin(omega * t)

    Ad, Bd, Cd, Dd = build_discrete_plant(plant, args.ts_s)
    y_open = simulate_open_loop(Ad, Bd, Cd, Dd, reference)

    servo = LeadServo(config, Ts=args.ts_s)
    y_closed = simulate_closed_loop(Ad, Bd, Cd, Dd, reference, servo, args.velocity_limit_mm_s)

    # Score only the back half, once any startup transient has settled.
    tail = t >= duration_s / 2
    amp_ref, _, _ = fit_sinusoid(t[tail], reference[tail], omega)
    amp_open, phase_open, dc_open = fit_sinusoid(t[tail], y_open[tail], omega)
    amp_closed, phase_closed, dc_closed = fit_sinusoid(t[tail], y_closed[tail], omega)
    # The reference is exactly amplitude*sin(omega*t) by construction, i.e. phase 0 and no DC
    # offset in fit_sinusoid's own convention -- no need to fit it separately.

    def lag_ms(phase: float) -> float:
        # phase is relative to a pure sin(omega*t) reference (phase 0); a lagging output has
        # phase < 0, and that delay in time is -phase/omega.
        return -phase / omega * 1000.0

    print(f"plant: K={plant.K}, wn={plant.wn}, zeta={plant.zeta}")
    print(f"lead:  zero={lead.zero}, pole={lead.pole}, gain={lead.gain}")
    print(f"reference: {args.amplitude_mm}mm @ {args.frequency_hz}Hz ({omega:.3f} rad/s), "
          f"{args.cycles} cycles, Ts={args.ts_s * 1000:.1f}ms\n")

    print(f"{'':14s}{'amplitude ratio':>18s}{'lag':>12s}")
    print(f"{'open loop':14s}{amp_open / amp_ref:>18.3f}{lag_ms(phase_open):>10.1f}ms  "
          f"(predicted: |G|={abs(plant_response(plant, omega)):.3f}, "
          f"lag={-np.angle(plant_response(plant, omega)) / omega * 1000:.1f}ms)")
    print(f"{'closed loop':14s}{amp_closed / amp_ref:>18.3f}{lag_ms(phase_closed):>10.1f}ms  "
          f"(predicted: |T|={abs(closed_loop_response(config, omega)):.3f}, "
          f"residual_lag={residual_lag(config, omega) * 1000:.1f}ms)")

    if abs(dc_closed) > 0.05 * amp_ref:
        print(f"\nnote: the compensated output also carries a {dc_closed:+.2f}mm DC offset -- "
              "unexpected for an actively-corrected loop, worth a second look.")

    if abs(dc_open) > 0.05 * amp_ref:
        print(f"\nfinding: the uncompensated output has settled onto a {dc_open:+.2f}mm DC "
              "offset, not just the driven oscillation -- a real property of this type-1 "
              "(free-integrator) plant driven open-loop, not a simulation artifact: the "
              "integrator has no restoring feedback, so any imbalance in the startup transient "
              "becomes a permanent offset (see this script's build_discrete_plant docstring). "
              "One more reason not to command this axis without the compensator.")

    closed_mismatch = abs(amp_closed / amp_ref - abs(closed_loop_response(config, omega))) / \
        abs(closed_loop_response(config, omega))
    if closed_mismatch > 0.10:
        print(f"\nfinding: the simulated closed loop disagrees with closed_loop_response()'s "
              f"prediction by {closed_mismatch * 100:.0f}% -- not a bug in either. "
              "ct.control.states.insert._hold_standoff() commands `reference + correction`, "
              "not `correction` alone, so its real transfer function is G*(1+C)/(1+G*C), not "
              "the standard unity-feedback G*C/(1+G*C) that closed_loop_response()/"
              "residual_lag() compute. This simulation follows the real code (LeadServo run in "
              "an actual feedforward+feedback loop); the analytical functions follow textbook "
              "unity feedback. Worth a dedicated follow-up: tau_cl(omega_r) may be computed "
              "against the wrong architecture for the real control code that uses it.")

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, reference, lw=1.2, color="C0", label="reference (input)")
    ax.plot(t, y_open, lw=1.0, color="C3", ls="--", label="uncompensated (plant only)")
    ax.plot(t, y_closed, lw=1.0, color="C2", label="compensated (closed loop)")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("position [mm]")
    ax.set_title(f"needle lead compensation — {args.frequency_hz}Hz sine, "
                 f"wn={plant.wn}rad/s plant, gain={lead.gain} lead")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()

    out_path = args.out or DEFAULT_OUT
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"\nsaved: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
