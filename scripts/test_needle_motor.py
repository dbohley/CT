#!/usr/bin/env python3
"""Slow, small-travel sanity test for the needle motor (CubeMars GL40II, position mode).

Reverted to this protocol after an extended MIT-mode investigation (structural v_des fix,
stale-controller-state fix, kp=0 testing) never produced clean motion -- some combination
of gains and velocity targets either faulted the drive or was too weak to move it at all.
Committing to position mode instead: mode-shifted addressing, float32 position+velocity
payload. This isn't a new guess -- it's the one configuration with actual proof of working
correctly on this exact motor (extended and retracted cleanly, direction confirmed) before
any of the MIT-mode detour began, and it matches what scripts/test_base_motor.py already
uses successfully. scripts/test_phantom_motor.py is on the equivalent real servo/position
mode for its motor family. All three motors are now on their proven protocols.

Moves only a few millimetres at low speed and back, so a wiring or direction mistake shows
up as a small, easily-stopped motion instead of a full-speed stroke. Run with --dry-run
first to see the exact frames this would send before anything moves.

Protocol: CubeMars Gimbal Motor II Drive User Manual, section 5.2 (position/velocity CAN
mode). CAN arbitration ID = (mode << 8) | node_id; a move command is `pos` (float32 rad) +
`vel` (float32 rad/s), little-endian, 8 bytes packed as "<ff". Unlike MIT mode, `vel` here
is a real firmware-enforced speed limit -- the motor's own trajectory controller handles
smoothness and safety internally; this script doesn't need to (and shouldn't) simulate its
own ramp on top of it.

This commands a real motor. Nothing in this file runs on import -- you run it yourself:

    python scripts/test_needle_motor.py --dry-run
    python scripts/test_needle_motor.py
"""

from __future__ import annotations

import argparse
import struct
import time

import can

# ---------------- Motor-specific config ----------------
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # needle's current adapter -- confirm with `ls /dev/cu.*`
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # GL-series driver board CAN bus speed

MOTOR_NODE_ID = 2  # needle motor's labeled CAN node ID
POSITION_VELOCITY_MODE = 1

DRUM_RADIUS_M = 0.018  # 3.6 cm capstan diameter / 2, from the needle's known capstan drive
DIRECTION_SIGN = -1  # found empirically: +rad retracted (moved away from phantom), so flipped
DEFAULT_TRAVEL_MM = 5.0  # small by design -- override with --travel-mm
DEFAULT_VELOCITY_RAD_S = 0.3  # slow by design -- override with --velocity

DWELL_AT_EXTEND_S = 1.0
SETTLE_MARGIN_S = 2.0  # added on top of the move's own estimated duration
# ---------------------------------------------------------

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
SET_ZERO = bytes([0xFF] * 7 + [0xFE])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])


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
    parser.add_argument(
        "--zero", action="store_true",
        help="also set the current position as zero before moving -- only do this once "
             "you've confirmed the motor is at the position you want to call 'retracted'",
    )
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    travel_m = args.travel_mm / 1000.0
    target_rad = DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)
    est_s = abs(target_rad) / max(args.velocity, 1e-6)

    print(f"needle motor (GL40II), node id {MOTOR_NODE_ID}, "
          f"arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"target: {args.travel_mm:.1f} mm -> {target_rad:.4f} rad "
          f"({target_rad * 180 / 3.14159265:.2f} deg) at {args.velocity:.2f} rad/s")
    print(f"expected time for the move: ~{est_s:.1f}s")

    if args.dry_run:
        print("\n--dry-run: not opening the bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        if args.zero:
            print(" ", universal_command(MOTOR_NODE_ID, SET_ZERO))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity))
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE))
        return 0

    print(f"\nThis will move the needle motor. Watch the axis -- it should extend outward by "
          f"{args.travel_mm:.1f} mm, dwell {DWELL_AT_EXTEND_S}s, then retract back to where it "
          "started. If it moves the other way, Ctrl+C immediately.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
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

        print(f"extending {args.travel_mm:.1f} mm ({target_rad:.4f} rad)...")
        bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity))
        time.sleep(est_s + SETTLE_MARGIN_S)

        print(f"dwelling {DWELL_AT_EXTEND_S}s at full extension...")
        time.sleep(DWELL_AT_EXTEND_S)

        print("retracting back to 0 rad...")
        bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity))
        time.sleep(est_s + SETTLE_MARGIN_S)

        print("exiting motor control mode (motor goes limp)...")
        bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
