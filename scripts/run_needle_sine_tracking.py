#!/usr/bin/env python3
"""Drive the needle motor through a sinusoidal position reference and log how closely it
tracks the command, for both a first look and as the raw data behind
`servo.needle.plant`/`servo.needle.lead` in src/ct/unknowns.py (currently unmeasured
placeholders feeding src/ct/control/servo.py's tau_cl(omega_r)).

Needle control mirrors scripts/test_needle_motor.py exactly (Gimbal Motor II
position/velocity protocol, node id 2, CLEAR_ERRORS/ENTER_MODE/EXIT_MODE frames) and reuses
scripts/run_approach_and_stop.py's periodic-resend pattern -- this motor ACKs each command
with a real position/velocity/torque reply (confirmed 2026-08-27) but does not broadcast on
its own, so the command is resent at --command-hz both to keep its trajectory controller fed
and to get fresh real telemetry throughout the run rather than a handful of snapshots.

**Command waveform is one-directional, not a plain sine.** At its starting position the
needle is already at the end of its capstan wrap and must never be commanded backward past
it. A plain sin(omega*t) swings equally either side of its start point -- exactly the motion
that isn't safe here. Instead this uses a raised cosine that starts and ends every cycle at
the needle's own current position and only moves in the extend direction:

    extend_frac(t) = 0.5 * (1 - cos(omega*t))     in [0, 1], zero at t=0, T/2, T, ...
    travel_mm(t)    = amplitude_mm * extend_frac(t)
    target_rad(t)   = start_rad + DIRECTION_SIGN * (travel_mm(t) / 1000.0) / DRUM_RADIUS_M

Running an integer number of full periods (--cycles) guarantees the command starts and ends
at start_rad with zero commanded velocity -- no separate return-to-start move, and no way for
phase to drift the command past the start point.

start_rad is read from the motor's own first real reply, not assumed to be 0.0 -- the
needle's zero calibration may have drifted since it was last zeroed (--zero re-zeros first,
same flag as test_needle_motor.py). This is what makes the "never go backward past start"
guarantee hold regardless of absolute-zero drift.

DRUM_RADIUS_M for the needle (0.018m) is a documented estimate from the known capstan drive,
not an independently calipers-measured value the way the base's and phantom's are -- so the
mm/rad conversion here carries more uncertainty than in those scripts. --max-amplitude-mm is
a soft safety cap for that reason.

    python scripts/run_needle_sine_tracking.py --dry-run
    python scripts/run_needle_sine_tracking.py
    python scripts/run_needle_sine_tracking.py --amplitude-mm 8 --frequency-hz 0.5 --cycles 8
"""

from __future__ import annotations

import argparse
import math
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
# all GL-II modes -- see scripts/run_approach_and_stop.py, confirmed real 2026-08-27.
MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II
# ---------------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_sine_tracking"

DEFAULT_AMPLITUDE_MM = 5.0  # matches test_needle_motor.py's DEFAULT_TRAVEL_MM
DEFAULT_FREQUENCY_HZ = 0.25  # ~15 breaths/min, breathing-rate scale
DEFAULT_CYCLES = 5
DEFAULT_VELOCITY_RAD_S = 0.3  # matches test_needle_motor.py's DEFAULT_VELOCITY_RAD_S
DEFAULT_COMMAND_HZ = 20.0  # matches run_approach_and_stop.py
DEFAULT_SAMPLE_HZ = 100.0
DEFAULT_MAX_AMPLITUDE_MM = 20.0
INITIAL_REPLY_WAIT_S = 0.5  # how long to wait for the first real reply before falling back to 0.0
SETTLE_MARGIN_S = 3.0  # added to the nominal run duration before the safety timeout fires


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def target_rad_at(t: float, start_rad: float, amplitude_mm: float, omega: float) -> float:
    extend_frac = 0.5 * (1.0 - math.cos(omega * t))
    travel_m = (amplitude_mm * extend_frac) / 1000.0
    return start_rad + DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--amplitude-mm", type=float, default=DEFAULT_AMPLITUDE_MM)
    parser.add_argument("--frequency-hz", type=float, default=DEFAULT_FREQUENCY_HZ)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--velocity-rad-s", type=float, default=DEFAULT_VELOCITY_RAD_S, dest="velocity_rad_s")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ,
                         help="rate to resend the current target -- also sets how often a fresh "
                              "real motor reply becomes available")
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ,
                         help="main loop tick / JSONL record rate")
    parser.add_argument("--zero", action="store_true",
                         help="also set the current position as zero before starting -- only do "
                              "this once you've confirmed the motor is at the capstan's true limit")
    parser.add_argument("--max-amplitude-mm", type=float, default=DEFAULT_MAX_AMPLITUDE_MM, dest="max_amplitude_mm")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/needle_sine_tracking/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    if args.amplitude_mm > args.max_amplitude_mm:
        print(f"error: --amplitude-mm {args.amplitude_mm} exceeds --max-amplitude-mm "
              f"{args.max_amplitude_mm}. Raise --max-amplitude-mm explicitly if you really want "
              f"this -- it exists because DRUM_RADIUS_M for the needle is an estimate, not a "
              f"calipers measurement.")
        return 1

    omega = 2.0 * math.pi * args.frequency_hz
    period_s = 1.0 / args.frequency_hz
    duration_s = args.cycles * period_s
    max_travel_rad = abs(DIRECTION_SIGN) * (args.amplitude_mm / 1000.0) / DRUM_RADIUS_M

    print(f"needle motor (GL40II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"waveform: raised-cosine, amplitude {args.amplitude_mm:.1f}mm ({max_travel_rad:.4f}rad "
          f"peak extension), frequency {args.frequency_hz:.3f}Hz, {args.cycles} cycle(s) "
          f"-> {duration_s:.1f}s total")
    print(f"velocity field sent with every command: {args.velocity_rad_s:.3f}rad/s (constant; "
          f"the position field alone carries the waveform)")
    print(f"resending the current target at {args.command_hz:.0f}Hz; sampling/logging at "
          f"{args.sample_hz:.0f}Hz")
    print("the waveform starts and ends each cycle at the needle's own current position and "
          "only moves in the extend direction -- it never asks the needle to retract past "
          "where it started.")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        if args.zero:
            print(" ", universal_command(MOTOR_NODE_ID, SET_ZERO))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity_rad_s), " <- initial, to read start_rad")
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            t = frac * period_s
            print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad_at(t, 0.0, args.amplitude_mm, omega),
                                            args.velocity_rad_s), f"  (t={t:.2f}s of first cycle, start_rad=0.0 assumed)")
        print(f"  ... (resent at {args.command_hz:.0f}Hz for {duration_s:.1f}s total) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE))
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    print(f"\nThis will move the needle motor continuously for {duration_s:.1f}s, extending up "
          f"to {args.amplitude_mm:.1f}mm and back, {args.cycles} time(s). Watch it closely. "
          f"Ctrl+C stops and de-energizes immediately.")
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
            print("reading current position before starting the waveform...")
            bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity_rad_s))
            motion_commanded = True
            reply = read_reply(INITIAL_REPLY_WAIT_S)
            if reply is not None:
                start_rad = reply["position"]
                start_rad_note = "measured from the motor's first real reply"
            print(f"start_rad = {start_rad:.4f}rad ({start_rad_note})")

        print(f"running the sine sweep for {duration_s:.1f}s "
              "(Ctrl+C to stop early and de-energize)...")
        t0 = time.monotonic()
        sample_period_s = 1.0 / max(args.sample_hz, 1e-6)
        command_period_s = 1.0 / max(args.command_hz, 1e-6)
        next_sample_at = 0.0
        next_command_at = 0.0
        safety_timeout_s = duration_s + SETTLE_MARGIN_S

        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= safety_timeout_s:
                print(f"\nreached safety timeout ({safety_timeout_s:.1f}s) -- stopping.")
                stop_motor("safety timeout")
                break

            target_rad = target_rad_at(min(elapsed, duration_s), start_rad, args.amplitude_mm, omega)

            read_reply(0.0)  # non-blocking

            if elapsed >= next_command_at:
                bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity_rad_s))
                motion_commanded = True
                next_command_at = elapsed + command_period_s

            if elapsed >= next_sample_at:
                writer.write({
                    "t": elapsed,
                    "commanded_target_rad": target_rad,
                    "commanded_target_mm": (target_rad - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0,
                    "commanded_velocity_rad_s": args.velocity_rad_s,
                    "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                    "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                    "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                    "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                })
                next_sample_at = elapsed + sample_period_s

            if elapsed >= duration_s:
                print(f"\nsine sweep complete at t={elapsed:.2f}s.")
                break

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        if motion_commanded:
            stop_motor("cleanup")
        writer.close()
        bus.shutdown()

    summary = {
        "amplitude_mm": args.amplitude_mm,
        "frequency_hz": args.frequency_hz,
        "cycles": args.cycles,
        "duration_s": duration_s,
        "start_rad": start_rad,
        "start_rad_note": start_rad_note,
        "velocity_rad_s": args.velocity_rad_s,
        "command_hz": args.command_hz,
        "sample_hz": args.sample_hz,
        "motor_replies_seen": motor_replies_seen,
        "motor_reply_note": (
            "count of decoded GL-II feedback frames (ID 0) seen during this run. Confirmed "
            "real 2026-08-27 on the base motor, per-command ACK not a continuous broadcast -- "
            "should scale roughly with duration_s x command_hz."
        ),
        "drum_radius_m": DRUM_RADIUS_M,
        "drum_radius_note": "estimate from the known capstan drive, not calipers-measured",
    }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")
    print(f"plot with: python scripts/plot_needle_sine_tracking.py --run {out_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
