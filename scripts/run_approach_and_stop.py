#!/usr/bin/env python3
"""Drive the base motor slowly toward the phantom and stop the instant tactile contact is
detected, logging everything possible.

Base motor control mirrors scripts/test_base_motor.py exactly (Gimbal Motor II
position/velocity protocol, CLEAR_ERRORS/ENTER_MODE/EXIT_MODE frames) -- no ct.hw codec
exists for this protocol, so this duplicates those constants rather than importing them
(same convention scripts/listen_needle_motor.py etc. already use). Sensor reading mirrors
scripts/measure_sensor_noise.py / ct.cli.sensor_bench (build_bus_from_config -> RH02Bus,
decoding the Teensy sketch's CAN ID 5 frame). The two buses are opened independently and
polled/commanded in the same loop.

**Motor position feedback: confirmed real, but only arrives as a per-command reply.** A
2026-08-27 run found the GL-II feedback frame (ID = Master ID, default 0 -- documented as
shared across all GL-II modes, decoded via ct.hw.motors.cubemars_mit.CubeMarsMIT.parse())
genuinely works, but only 4 replies arrived in a 19.6s run, clustered exactly at the 4
commands sent (CLEAR_ERRORS/ENTER_MODE/approach-move/hold-move) -- this motor ACKs each
command, it does not broadcast continuously. So this script now resends the current target
periodically (--command-hz, default 20Hz) rather than once, both to keep the motor's
trajectory controller fed the way CubeMars's own GUI does ("Timing Send") and, as a direct
side effect, to get a fresh real position reading every ~50ms instead of once at t=0.

**The overshoot bug's real cause, found from that one well-timed reply**: the hold target
was computed via dead reckoning from t=0 (`commanded_velocity * elapsed_time`), which
assumes the motor moved at a constant velocity from the very start. The one reply that
arrived within 8ms of the actual contact instant measured -1.2907 rad; dead reckoning at
that same instant predicted -1.4613 rad -- a 0.17 rad (4.44mm) gap, almost certainly from
the real acceleration ramp-up at the start of the move (which the manual documents exists,
but doesn't give numbers for) that constant-velocity dead reckoning ignores entirely. The
"hold" command was therefore accidentally asking the motor to travel another ~4.4mm
forward, not to stop -- it wasn't failing to stop, it was correctly executing a command
that was wrong. This is now fixed: the hold target is computed from the most recent real
reply (now frequent, thanks to periodic resending) plus a small dead-reckoned correction
only for the brief gap since that reply, rather than dead-reckoning the entire elapsed
time from t=0. Falls back to the old full-elapsed-time estimate only if no reply has
arrived yet at all.

**Stop mechanism**: the instant contact is detected, this commands the motor to actively
hold its position (stall), rather than de-energizing -- a de-energized motor is compliant
and can bounce/spring back off the phantom, which is exactly the behavior this avoids (e.g.
for video). Recording continues while held; Ctrl+C is the deliberate action that ends the
recording and de-energizes the motor (sends EXIT_MODE), releasing it back to compliant.

A max-duration safety fallback still fires EXIT_MODE if contact is never detected at all
(bad threshold, wiring issue, mispositioned phantom) -- this only applies before contact;
once holding, there is no time limit and the motor stays stalled until Ctrl+C.

    python scripts/run_approach_and_stop.py --dry-run
    python scripts/run_approach_and_stop.py
    python scripts/run_approach_and_stop.py --velocity 0.1 --contact-threshold-cm 0.03
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from pathlib import Path

import can

from ct.cli._common import save_json
from ct.hw.bus import build_bus_from_config
from ct.hw.config import BusConfig
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.rt.telemetry import TelemetryWriter

# ---------------- Base motor config (mirrors scripts/test_base_motor.py -- keep in sync) ----------------
MOTOR_CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # confirm with `ls /dev/cu.*` -- may differ
MOTOR_CAN_INTERFACE = "slcan"
MOTOR_BITRATE = 1_000_000

MOTOR_NODE_ID = 3  # base motor's labeled CAN node ID
POSITION_VELOCITY_MODE = 1
DRUM_RADIUS_M = 0.026  # measured capstan/drum radius
DIRECTION_SIGN = -1  # found empirically: +rad moved the wrong way, so flipped

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

# GL-II manual documents one feedback frame (arbitration ID = Master ID, default 0) shared
# identically across all GL-II modes -- structurally the same format CubeMarsMIT.parse()
# already decodes. Confirmed real 2026-08-27, but only as a per-command ACK (see module
# docstring) -- this is why the main loop resends periodically rather than once, and why
# the hold-position calculation now prefers this real data over pure dead reckoning.
# Ranges are GL-II's own documented values, not MIT mode's (see listen_base_motor.py).
MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II
# -----------------------------------------------------------------------------------------------------

# ---------------- Sensor config (mirrors ct.cli.sensor_bench / measure_sensor_noise.py) ----------------
SENSOR_CAN_CHANNEL = "/dev/cu.usbmodem20553962534B1"
SENSOR_CAN_INTERFACE = "slcan"
SENSOR_BITRATE = 1_000_000
SENSOR_CAN_ID = 5
_PAYLOAD = struct.Struct("<ff")  # float32 tof_mm, float32 dist_cm (angle field removed by firmware)
# -----------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "approach_and_stop"

DEFAULT_TRAVEL_MM = 40.0  # generous -- safely more than any real approach should need
DEFAULT_VELOCITY_RAD_S = 0.1  # slower than test_base_motor.py's 0.3 -- precision matters more here
DEFAULT_CONTACT_THRESHOLD_CM = 0.01  # changed to 0.01 since this is detectable and wont falsely trip
REACTION_LATENCY_S = 0.05  # conservative estimate: poll loop + CAN round trip
SETTLE_MARGIN_S = 3.0  # added on top of the move's own estimated duration, before the safety timeout fires
DEFAULT_COMMAND_HZ = 20.0  # resend rate for the current target -- see module docstring for why


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--travel-mm", type=float, default=DEFAULT_TRAVEL_MM)
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY_RAD_S, help="rad/s")
    parser.add_argument("--contact-threshold-cm", type=float, default=DEFAULT_CONTACT_THRESHOLD_CM,
                         dest="contact_threshold_cm")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ,
                         help="rate to resend the current target -- also sets how often a fresh "
                              "real motor reply becomes available (this motor ACKs each command "
                              "sent, it does not broadcast on its own)")
    parser.add_argument("--out", type=Path, default=None, help="output directory; default outputs/approach_and_stop/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    travel_m = args.travel_mm / 1000.0
    target_rad = DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)
    est_s = abs(target_rad) / max(args.velocity, 1e-6)
    overshoot_mm = args.velocity * DRUM_RADIUS_M * REACTION_LATENCY_S * 1000.0

    print(f"base motor (GL60II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"max travel: {args.travel_mm:.1f}mm -> {target_rad:.4f}rad at {args.velocity:.3f}rad/s "
          f"(~{est_s:.1f}s if contact is never detected)")
    print(f"contact threshold: abs(dist_cm) > {args.contact_threshold_cm}cm")
    print(f"estimated worst-case hold-position error past true contact (reaction latency "
          f"~{REACTION_LATENCY_S*1000:.0f}ms): ~{overshoot_mm:.3f}mm")
    print(f"resending the current target at {args.command_hz:.0f}Hz -- keeps the motor's own "
          f"controller fed and gives a fresh real position reading roughly every "
          f"{1000.0/args.command_hz:.0f}ms")
    print("on contact: stalls (holds position) and keeps recording -- Ctrl+C stops and releases (de-energizes)")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity))
        print(f"  ... (resent at {args.command_hz:.0f}Hz; sensor bus polled; on contact, a "
              f"hold-position frame stalls the motor there, also resent at {args.command_hz:.0f}Hz) ...")
        print("  ... (recording continues while held; Ctrl+C ends it) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE), " <- sent on Ctrl+C (release/de-energize)")
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    print(f"\nThis will move the base motor continuously toward the phantom until contact is "
          f"detected (or {est_s + SETTLE_MARGIN_S:.0f}s elapses with no contact, as a safety "
          f"fallback). On contact, the motor stalls and holds there -- it does NOT stop moving "
          f"or de-energize on its own. Recording continues until you press Ctrl+C, which stops "
          f"and de-energizes the motor. Watch it closely.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    motor_bus = can.interface.Bus(channel=MOTOR_CAN_CHANNEL, interface=MOTOR_CAN_INTERFACE, bitrate=MOTOR_BITRATE)
    sensor_bus = build_bus_from_config(
        "approach-sensors",
        BusConfig(backend="rh02", interface=SENSOR_CAN_INTERFACE, channel=SENSOR_CAN_CHANNEL, bitrate=SENSOR_BITRATE),
    )
    writer = TelemetryWriter(jsonl_path)
    motor_reply_codec = CubeMarsMIT(
        p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
        v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
        t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
    )

    contact_detected = False
    contact_record: dict | None = None
    motion_commanded = False
    last_elapsed = 0.0
    last_motor_reply: dict | None = None
    last_motor_reply_t: float | None = None
    motor_replies_seen = 0
    current_target_rad = target_rad
    next_command_at = 0.0
    command_period_s = 1.0 / max(args.command_hz, 1e-6)
    hold_rad: float | None = None
    hold_basis: str | None = None
    t0 = time.monotonic()
    timeout_s = est_s + SETTLE_MARGIN_S

    def stop_motor(reason: str) -> None:
        print(f"stopping motor ({reason})...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))

    try:
        print("clearing errors...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)

        print("entering motor control mode...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.5)

        print(f"commanding approach: {args.travel_mm:.1f}mm ({target_rad:.4f}rad) at {args.velocity:.3f}rad/s...")
        t0 = time.monotonic()
        motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity))
        motion_commanded = True
        next_command_at = command_period_s  # already sent once above; next resend one period from now

        print("polling for contact (Ctrl+C to stop and release once you're done recording)...")
        while True:
            elapsed = time.monotonic() - t0
            last_elapsed = elapsed
            if not contact_detected and elapsed >= timeout_s:
                print(f"\nno contact detected within {timeout_s:.0f}s -- stopping as a safety fallback. "
                      f"Check wiring, --contact-threshold-cm, or phantom positioning.")
                stop_motor("timeout, no contact detected")
                break

            # Non-blocking check for the motor's own GL-II feedback reply. Confirmed real
            # (2026-08-27) but arrives only as a per-command ACK, not a continuous broadcast --
            # see module docstring. The periodic resend below is what makes this arrive often
            # enough to be useful, not just additive background listening anymore.
            motor_msg = motor_bus.recv(timeout=0.0)
            if motor_msg is not None:
                parsed = motor_reply_codec.parse(motor_msg.arbitration_id, bytes(motor_msg.data))
                if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
                    last_motor_reply = parsed
                    last_motor_reply_t = elapsed
                    motor_replies_seen += 1

            # Resend the current target periodically -- keeps the motor's trajectory controller
            # fed (matches the GUI's own "Timing Send" pattern) and is what makes the reply above
            # arrive often enough to be useful for the hold-position correction below.
            if elapsed >= next_command_at:
                motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, current_target_rad, args.velocity))
                next_command_at = elapsed + command_period_s

            for _stamp, can_id, data in sensor_bus.poll():
                if can_id != SENSOR_CAN_ID or len(data) < _PAYLOAD.size:
                    continue
                tof_mm, dist_cm = _PAYLOAD.unpack(data[: _PAYLOAD.size])
                in_contact = abs(dist_cm) > args.contact_threshold_cm
                record = {
                    "t": elapsed,
                    "tof_mm": tof_mm,
                    "dist_cm": dist_cm,
                    "in_contact": in_contact,
                    "commanded_target_rad": target_rad,
                    "commanded_velocity_rad_s": args.velocity,
                    "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                    "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                    # Named "current" by CubeMarsMIT.parse() (reused from MIT mode's reply
                    # format), but decoded via the codec's torque range (t_min/t_max, N*m) --
                    # the GL-II manual's own feedback table calls this same bit field torque,
                    # not current/amperage. Renamed here to match what it actually is.
                    "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                    "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                }
                writer.write(record)
                if in_contact and not contact_detected:
                    contact_detected = True
                    contact_record = record
                    # Prefer the most recent REAL position, correcting only for the small gap
                    # since that reply (now ~1/command_hz at most, thanks to periodic resending)
                    # -- not dead reckoning across the whole elapsed approach, which is what
                    # produced the ~4.4mm overshoot bug this replaces. See module docstring.
                    if last_motor_reply is not None and last_motor_reply_t is not None:
                        gap_s = elapsed - last_motor_reply_t
                        raw_hold_rad = last_motor_reply["position"] + DIRECTION_SIGN * args.velocity * gap_s
                        hold_basis = f"measured position + {gap_s*1000:.0f}ms correction"
                    else:
                        raw_hold_rad = DIRECTION_SIGN * args.velocity * elapsed
                        hold_basis = "dead-reckoned (no motor reply received yet)"
                    lo, hi = sorted((0.0, target_rad))
                    hold_rad = min(max(raw_hold_rad, lo), hi)
                    current_target_rad = hold_rad
                    print(f"\nCONTACT DETECTED at t={elapsed:.3f}s  dist_cm={dist_cm:.4f}  tof_mm={tof_mm}")
                    print(f"holding at {hold_rad:.4f}rad ({hold_basis}) -- recording continues, "
                          f"Ctrl+C when you're done to stop and release.")
                    motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, hold_rad, args.velocity))
                    next_command_at = elapsed + command_period_s
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C -- releasing.")
    finally:
        # Sent unconditionally whenever motion was commanded, regardless of which path got us
        # here (contact, timeout, Ctrl+C, or any other exception) -- this protocol needs no
        # continuous refresh to keep moving, so an uncaught exception must not leave the motor
        # still executing its last commanded move unsupervised. Sending EXIT_MODE twice (it may
        # already have been sent above) is harmless.
        if motion_commanded:
            stop_motor("cleanup")
        writer.close()
        sensor_bus.close()
        motor_bus.shutdown()

    commanded_travel_mm = None
    held_duration_s = None
    if contact_record is not None:
        commanded_travel_mm = abs(args.velocity * contact_record["t"] * DRUM_RADIUS_M * 1000.0)
        held_duration_s = max(0.0, last_elapsed - contact_record["t"])

    summary = {
        "contact_detected": contact_detected,
        "time_to_contact_s": contact_record["t"] if contact_record else None,
        "held_duration_s": held_duration_s,
        "sensor_at_contact": (
            {"tof_mm": contact_record["tof_mm"], "dist_cm": contact_record["dist_cm"]}
            if contact_record else None
        ),
        "commanded_travel_mm_at_contact": commanded_travel_mm,
        "commanded_travel_note": (
            "estimated from commanded velocity x elapsed time -- NOT measured motor feedback. "
            "See motor_replies_seen: if nonzero, motor_position_rad in samples.jsonl is real "
            "measured feedback and should be preferred over this estimate."
        ),
        "hold_rad": hold_rad,
        "hold_basis": hold_basis,
        "motor_replies_seen": motor_replies_seen,
        "motor_reply_note": (
            "count of decoded GL-II feedback frames (ID 0) seen during this run. Confirmed "
            "real 2026-08-27, but arrives only as a per-command ACK, not a continuous "
            "broadcast -- roughly one per --command-hz resend, so this count should scale "
            "with run duration x command_hz. A much lower count than expected suggests a "
            "wiring/timing issue, not that the reply doesn't exist."
        ),
        "travel_mm_limit": args.travel_mm,
        "velocity_rad_s": args.velocity,
        "contact_threshold_cm": args.contact_threshold_cm,
    }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
