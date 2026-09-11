#!/usr/bin/env python3
"""Command the needle motor with a fixed velocity limit toward a far target and log the
resulting velocity/position transient, for step-response identification of `servo.needle.plant`
in src/ct/unknowns.py (currently the unmeasured placeholder feeding
src/ct/control/servo.py's tau_cl(omega_r)).

**Why this is not a plain position step.** `PlantModel` in src/ct/hw/config.py is a type-1
model: the motor produces velocity, and position is its free integral. A position command
stepped straight to a nearby target and fit against the *position* response does not show the
classic underdamped landmarks (percent overshoot, settling time) that identify `wn`/`zeta` --
those live on the velocity channel, and position free-integrates whatever that channel does.

The real motor only accepts (target_position_rad, velocity_limit_rad_s) -- there is no direct
velocity or torque command, and its own firmware runs the trajectory/velocity loop internally
(see scripts/test_needle_motor.py's docstring). So a velocity step is commanded indirectly:
the position target is placed far outside the measurement window (--step-travel-mm, well under
the safety cap --max-step-travel-mm) so the firmware never arrives and never decelerates within
--step-duration-s. What's left for its trajectory controller to do is ramp its actual velocity
up toward the commanded limit and hold it -- that ramp is the step-response transient this
script logs, and scripts/fit_needle_plant.py fits against the *velocity* channel, not position.

This assumes the firmware's velocity loop behaves roughly second-order. That is NOT confirmed
on this exact firmware -- eyeball the first run with scripts/plot_needle_step_response.py before
trusting a fit. A plain jerk-limited trapezoidal ramp (common in commercial trajectory
firmware) would look nothing like a second-order transient and is itself a finding worth a
session-doc callout, not something to force-fit.

Needle control mirrors scripts/test_needle_motor.py exactly (Gimbal Motor II position/velocity
protocol, node id 2, CLEAR_ERRORS/ENTER_MODE/EXIT_MODE frames) and reuses
scripts/run_needle_sine_tracking.py's CAN setup and periodic-resend/JSONL-logging pattern --
this motor ACKs each command with a real position/velocity/torque reply but does not broadcast
on its own, so the command is resent at --command-hz both to keep its trajectory controller fed
and to get fresh real telemetry throughout the run.

Each velocity level in --velocities is run --reps times: extend toward the far target and hold
for --step-duration-s, then retract back to the needle's own start position before the next
rep. This never asks the needle to go backward past where it started, same constraint
run_needle_sine_tracking.py's raised-cosine and test_needle_motor.py's extend/dwell/retract
bracket both respect. An in-loop cutoff also retracts early if measured travel approaches
--max-step-travel-mm before step-duration-s elapses, independent of the target-offset cap.

DRUM_RADIUS_M for the needle (0.018m) is a documented estimate from the known capstan drive,
not an independently calipers-measured value -- so the mm/rad conversion here carries more
uncertainty than the base's or phantom's. --max-step-travel-mm is a soft safety cap for that
reason, same as run_needle_sine_tracking.py's --max-amplitude-mm.

    python scripts/run_needle_step_response.py --dry-run
    python scripts/run_needle_step_response.py --velocities 0.15 --reps 1 --step-duration-s 3.0
    python scripts/run_needle_step_response.py --velocities 0.15,0.30 --reps 3
"""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import can

from ct.cli._common import save_json
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.rt.telemetry import TelemetryWriter

# ---------------- Needle motor config (mirrors scripts/test_needle_motor.py -- keep in sync) ----------------
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # confirm with `ls /dev/cu.*` -- may differ
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

MOTOR_NODE_ID = 2  # needle motor's labeled CAN node ID
POSITION_VELOCITY_MODE = 1
DRUM_RADIUS_M = 0.018  # 3.6cm capstan diameter / 2, from the needle's known capstan drive -- not calipers-measured
DIRECTION_SIGN = -1  # found empirically: +rad retracted (moved away from phantom), so flipped

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
SET_ZERO = bytes([0xFF] * 7 + [0xFE])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

# GL-II's documented feedback frame (arbitration ID = Master ID, default 0), shared across
# all GL-II modes -- see scripts/run_needle_sine_tracking.py, confirmed real 2026-08-27.
MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II
# ---------------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_step_response"

DEFAULT_VELOCITIES_RAD_S = "0.15,0.30"
DEFAULT_REPS = 3
DEFAULT_STEP_DURATION_S = 2.0
DEFAULT_STEP_TRAVEL_MM = 15.0
DEFAULT_MAX_STEP_TRAVEL_MM = 20.0
DEFAULT_RETRACT_VELOCITY_RAD_S = 0.2  # slow, fixed -- matches the spirit of test_needle_motor.py's default
DEFAULT_COMMAND_HZ = 20.0  # matches run_needle_sine_tracking.py / run_approach_and_stop.py
DEFAULT_SAMPLE_HZ = 100.0
INITIAL_REPLY_WAIT_S = 0.5
DWELL_BETWEEN_REPS_S = 0.5
SETTLE_MARGIN_S = 3.0  # added to the nominal run duration before the safety timeout fires
RETRACT_ARRIVAL_TOLERANCE_RAD = 0.01  # ~0.18mm at this drum radius -- "close enough to start_rad"
RETRACT_SAFETY_MARGIN_S = 1.0  # extra time on top of the computed retract duration, in case of slip


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def parse_velocities(raw: str) -> list[float]:
    values = [float(v) for v in raw.split(",") if v.strip()]
    if not values:
        raise ValueError(f"--velocities produced no values from {raw!r}")
    return values


def far_target_rad(start_rad: float, step_travel_mm: float) -> float:
    """A position command far enough outside the measurement window that the firmware never
    arrives (and so never decelerates) within one step's duration -- see module docstring."""
    travel_m = step_travel_mm / 1000.0
    return start_rad + DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--velocities", type=str, default=DEFAULT_VELOCITIES_RAD_S,
                         help="comma-separated velocity limits [rad/s] to step to, one run per value")
    parser.add_argument("--reps", type=int, default=DEFAULT_REPS,
                         help="repetitions of each velocity level, for repeatability stats")
    parser.add_argument("--step-duration-s", type=float, default=DEFAULT_STEP_DURATION_S, dest="step_duration_s",
                         help="how long to hold each step before retracting")
    parser.add_argument("--step-travel-mm", type=float, default=DEFAULT_STEP_TRAVEL_MM, dest="step_travel_mm",
                         help="notional far-target offset that keeps the firmware ramping "
                              "rather than arriving during the step")
    parser.add_argument("--max-step-travel-mm", type=float, default=DEFAULT_MAX_STEP_TRAVEL_MM,
                         dest="max_step_travel_mm",
                         help="hard safety cap on both the far-target offset and actual measured "
                              "travel during a step -- exceeding it retracts early")
    parser.add_argument("--retract-velocity-rad-s", type=float, default=DEFAULT_RETRACT_VELOCITY_RAD_S,
                         dest="retract_velocity_rad_s", help="fixed slow speed used to return to start_rad")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ,
                         help="rate to resend the current target -- also sets how often a fresh "
                              "real motor reply becomes available")
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ,
                         help="main loop tick / JSONL record rate")
    parser.add_argument("--zero", action="store_true",
                         help="also set the current position as zero before starting -- only do "
                              "this once you've confirmed the motor is at the capstan's true limit")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/needle_step_response/<timestamp>")
    parser.add_argument("--reenter-mode-per-rep", action="store_true", dest="reenter_mode_per_rep",
                         help="resend CLEAR_ERRORS+ENTER_MODE before every rep's step phase, not "
                              "just once at the top of the run -- a diagnostic test for whether "
                              "only-the-first-rep-fits-cleanly (docs/sessions/019 Finding 3) is "
                              "tied to a fresh mode-entry rather than elapsed time or the medium")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    try:
        velocities = parse_velocities(args.velocities)
    except ValueError as exc:
        print(f"error: {exc}")
        return 1

    if args.step_travel_mm > args.max_step_travel_mm:
        print(f"error: --step-travel-mm {args.step_travel_mm} exceeds --max-step-travel-mm "
              f"{args.max_step_travel_mm}. Raise --max-step-travel-mm explicitly if you really "
              f"want this -- it exists because DRUM_RADIUS_M for the needle is an estimate, not "
              f"a calipers measurement.")
        return 1

    n_steps = len(velocities) * args.reps
    nominal_total_s = n_steps * (args.step_duration_s + DWELL_BETWEEN_REPS_S) * 2  # step + retract, roughly

    print(f"needle motor (GL40II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"velocities: {velocities} rad/s, {args.reps} rep(s) each -> {n_steps} step(s), "
          f"~{nominal_total_s:.0f}s total (rough estimate)")
    print(f"each step: far target at {args.step_travel_mm:.1f}mm offset (cap {args.max_step_travel_mm:.1f}mm), "
          f"held {args.step_duration_s:.1f}s, then retracted at {args.retract_velocity_rad_s:.2f}rad/s")
    print(f"resending the current target at {args.command_hz:.0f}Hz; sampling/logging at "
          f"{args.sample_hz:.0f}Hz")
    print("every step extends toward the far target and every retract returns to the needle's "
          "own start position -- it never commands the needle backward past where it started.")
    if args.reenter_mode_per_rep:
        print("--reenter-mode-per-rep is ON: CLEAR_ERRORS+ENTER_MODE will be resent before every "
              "rep's step phase, not just once at the top (diagnostic test for Finding 3).")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        if args.zero:
            print(" ", universal_command(MOTOR_NODE_ID, SET_ZERO))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, velocities[0]), " <- initial, to read start_rad")
        if args.reenter_mode_per_rep:
            print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS), " <- resent before each rep")
            print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE), " <- resent before each rep")
        target = far_target_rad(0.0, args.step_travel_mm)
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target, velocities[0]),
              f"  (step at v={velocities[0]:.2f}rad/s, start_rad=0.0 assumed)")
        print(f"  ... (resent at {args.command_hz:.0f}Hz for {args.step_duration_s:.1f}s) ...")
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.retract_velocity_rad_s), "  (retract to start_rad)")
        print(f"  ... (repeated for each velocity x rep) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE))
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    print(f"\nThis will step the needle motor {n_steps} time(s), extending toward a far target "
          f"and retracting back each time. Watch it closely. Ctrl+C stops and de-energizes "
          f"immediately.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    writer = TelemetryWriter(jsonl_path)
    motor_reply_codec = CubeMarsMIT(
        p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
        v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
        t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
    )

    motion_commanded = False
    motor_replies_seen = 0
    last_motor_reply: dict | None = None
    early_cutoffs = 0
    retract_incomplete = 0
    step_records: list[dict] = []

    def read_reply(timeout: float) -> dict | None:
        nonlocal last_motor_reply, motor_replies_seen
        msg = bus.recv(timeout=timeout)
        if msg is None:
            return None
        parsed = motor_reply_codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
            last_motor_reply = parsed
            motor_replies_seen += 1
            return parsed
        return None

    def stop_motor(reason: str) -> None:
        print(f"stopping motor ({reason})...")
        bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))

    start_rad = 0.0
    start_rad_note = "assumed 0.0 -- no real reply arrived before starting (e.g. no hardware attached)"

    try:
        print("clearing errors...")
        bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)

        print("entering motor control mode...")
        bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.5)

        if args.zero:
            print("setting current position as zero...")
            bus.send(universal_command(MOTOR_NODE_ID, SET_ZERO))
            time.sleep(0.5)
            start_rad = 0.0
            start_rad_note = "zeroed via --zero immediately before this run"
        else:
            print("reading current position before starting...")
            bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.retract_velocity_rad_s))
            motion_commanded = True
            reply = read_reply(INITIAL_REPLY_WAIT_S)
            if reply is not None:
                start_rad = reply["position"]
                start_rad_note = "measured from the motor's first real reply"
            print(f"start_rad = {start_rad:.4f}rad ({start_rad_note})")

        t0 = time.monotonic()
        sample_period_s = 1.0 / max(args.sample_hz, 1e-6)
        command_period_s = 1.0 / max(args.command_hz, 1e-6)

        max_travel_rad = args.max_step_travel_mm / 1000.0 / DRUM_RADIUS_M

        def run_phase(phase: str, target: float, vel: float, duration_s: float,
                      arrival_tolerance_rad: float | None = None) -> tuple[bool, bool]:
            """Command `target` at velocity limit `vel` for up to `duration_s`, sampling/logging
            throughout. If `arrival_tolerance_rad` is given, also exits early once the measured
            position is within it of `target` -- used by the retract phase so it actually
            finishes rather than running for a fixed time that may not be enough (see module
            docstring: this is what silently produced a mid-retract reversal on 2026-09-05's
            characterization run whenever the step velocity exceeded the retract velocity).
            Returns (cutoff, arrived).
            """
            nonlocal motion_commanded
            phase_t0 = time.monotonic() - t0
            next_sample_at = 0.0
            next_command_at = 0.0
            cutoff = False
            arrived = False
            while True:
                elapsed = time.monotonic() - t0 - phase_t0
                if elapsed >= duration_s:
                    break

                read_reply(0.0)  # non-blocking

                if elapsed >= next_command_at:
                    bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target, vel))
                    motion_commanded = True
                    next_command_at = elapsed + command_period_s

                if elapsed >= next_sample_at:
                    current_rad = last_motor_reply["position"] if last_motor_reply else start_rad
                    travelled_rad = abs(current_rad - start_rad)
                    writer.write({
                        "t": time.monotonic() - t0,
                        "elapsed": elapsed,
                        "phase": phase,
                        "step_index": step_index,
                        "commanded_velocity_limit_rad_s": vel,
                        "commanded_target_rad": target,
                        "commanded_target_mm": (target - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0,
                        "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                        "motor_position_mm": (
                            (last_motor_reply["position"] - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0
                            if last_motor_reply else None
                        ),
                        "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                        "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                        "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                    })
                    next_sample_at = elapsed + sample_period_s

                    if phase == "step" and travelled_rad >= max_travel_rad:
                        print(f"  travel cutoff hit ({travelled_rad:.4f}rad >= "
                              f"{max_travel_rad:.4f}rad) -- retracting early.")
                        cutoff = True
                        break

                    if arrival_tolerance_rad is not None and last_motor_reply is not None \
                            and abs(current_rad - target) <= arrival_tolerance_rad:
                        arrived = True
                        break

                time.sleep(0.001)

            return cutoff, arrived

        step_index = 0
        for velocity in velocities:
            target_rad = far_target_rad(start_rad, args.step_travel_mm)
            for rep in range(args.reps):
                print(f"\nstep {step_index}: v={velocity:.3f}rad/s, rep {rep + 1}/{args.reps}")

                if args.reenter_mode_per_rep:
                    print("  resending CLEAR_ERRORS+ENTER_MODE before this rep (--reenter-mode-per-rep)...")
                    bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
                    time.sleep(0.1)
                    bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
                    time.sleep(0.5)

                cutoff, _ = run_phase("step", target_rad, velocity, args.step_duration_s)
                if cutoff:
                    early_cutoffs += 1

                travelled_rad = abs((last_motor_reply["position"] if last_motor_reply else start_rad) - start_rad)
                retract_duration_s = travelled_rad / max(args.retract_velocity_rad_s, 1e-6) + RETRACT_SAFETY_MARGIN_S
                _, arrived = run_phase("retract", start_rad, args.retract_velocity_rad_s, retract_duration_s,
                                       arrival_tolerance_rad=RETRACT_ARRIVAL_TOLERANCE_RAD)
                if not arrived:
                    retract_incomplete += 1
                    print(f"  warning: retract did not reach start_rad within {retract_duration_s:.2f}s -- "
                          "the next rep will not start from a clean rest position.")

                step_records.append({
                    "step_index": step_index, "velocity_rad_s": velocity, "rep": rep,
                    "cutoff": cutoff, "retract_arrived": arrived,
                })
                time.sleep(DWELL_BETWEEN_REPS_S)
                step_index += 1

        print("\nall steps complete.")
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        if motion_commanded:
            stop_motor("cleanup")
        writer.close()
        bus.shutdown()

        summary = {
            "velocities_rad_s": velocities,
            "reps": args.reps,
            "reenter_mode_per_rep": args.reenter_mode_per_rep,
            "step_duration_s": args.step_duration_s,
            "step_travel_mm": args.step_travel_mm,
            "max_step_travel_mm": args.max_step_travel_mm,
            "retract_velocity_rad_s": args.retract_velocity_rad_s,
            "start_rad": start_rad,
            "start_rad_note": start_rad_note,
            "command_hz": args.command_hz,
            "sample_hz": args.sample_hz,
            "motor_replies_seen": motor_replies_seen,
            "motor_reply_note": (
                "count of decoded GL-II feedback frames (ID 0) seen during this run. Confirmed "
                "real 2026-08-27 on the base motor, per-command ACK not a continuous broadcast -- "
                "should scale roughly with total run duration x command_hz."
            ),
            "early_cutoffs": early_cutoffs,
            "retract_incomplete": retract_incomplete,
            "retract_incomplete_note": (
                "count of reps where the retract phase did not measure back within "
                f"{RETRACT_ARRIVAL_TOLERANCE_RAD}rad of start_rad before its computed time "
                "budget ran out -- any such rep's *next* step did not start from rest, and its "
                "fit should not be trusted as a from-rest step response."
            ),
            "step_records": step_records,
            "drum_radius_m": DRUM_RADIUS_M,
            "drum_radius_note": "estimate from the known capstan drive, not calipers-measured",
        }
        save_json(summary, summary_path)
        print(f"\nsaved: {jsonl_path}")
        print(f"saved: {summary_path}")
        print(f"plot with: python scripts/plot_needle_step_response.py --run {out_dir}")
        print(f"fit with:  python scripts/fit_needle_plant.py --run {out_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
