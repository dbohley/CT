#!/usr/bin/env python3
"""Slow, small-travel sanity test for the phantom drive motor (CubeMars AK60-6 V3.0, servo mode).

Rewritten after diagnosis: an MIT-mode version of this script sent commands with no
visible effect on the motor. scripts/listen_phantom_motor.py then showed the bus
continuously broadcasting autonomous VESC-style status frames on an *extended* CAN ID
matching `node_id | (command << 8)` -- exactly the addressing scheme
ct.hw.motors.cubemars_servo.CubeMarsServo already implements in this repo, and something
an MIT-mode motor would not do unprompted. That's strong evidence this AK60-6 is flashed
for servo mode, not MIT mode. This version uses CubeMarsServo instead of CubeMarsMIT.

**This motor is on a genuinely separate CAN adapter**, not just a separate CAN ID on the
same wire: two CANable2 adapters are connected, serial 20563976534B (needle/base) and
serial 207635764E45 (this script). Safe to run alongside the needle/base scripts -- no
serial-port contention.

Servo mode's position command is in **degrees**, not radians (unlike MIT mode). It also
has no trustworthy velocity-limited move available here: CubeMarsServo.command()'s
`velocity` argument is electrical RPM for the motor's own speed-limited move
(CMD_SET_POS_SPD), and converting a desired mm/s into the right ERPM needs the motor's
pole-pair count, which isn't known. So, like the earlier MIT-mode attempt, this ramps the
plain position setpoint (CMD_SET_POS) in small steps at RAMP_HZ updates/sec instead of
using that field.

DRUM_RADIUS_M is measured (13 mm). Direction was confirmed inverted on the first slow
test, same as the needle and base motors, and is now corrected via DIRECTION_SIGN below.

This commands a real motor. Nothing in this file runs on import -- you run it yourself:

    python scripts/test_phantom_motor.py --dry-run
    python scripts/test_phantom_motor.py
"""

from __future__ import annotations

import argparse
import time

import can

from ct.hw.motors.cubemars_servo import CubeMarsServo

# ---------------- Motor-specific config ----------------
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # second CANable2 adapter -- confirm with `ls /dev/cu.*`
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # assumed, carried over from the needle/base scripts -- not independently confirmed

MOTOR_NODE_ID = 1  # phantom motor's labeled CAN node ID

DRUM_RADIUS_M = 0.013  # measured capstan/drum radius
DIRECTION_SIGN = -1  # found empirically: +rad moved the wrong way, so flipped
DEFAULT_TRAVEL_MM = 5.0  # small by design -- override with --travel-mm
DEFAULT_VELOCITY_RAD_S = 0.3  # ramp rate, not a motor-level speed limit -- see docstring

RAMP_HZ = 20.0  # position setpoint update rate while ramping toward target
DWELL_AT_EXTEND_S = 1.0
# ---------------------------------------------------------


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def to_message(frame: tuple[int, bytes, bool]) -> can.Message:
    can_id, data, extended = frame
    return can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)


def ramp_to(
    bus: can.BusABC, codec: CubeMarsServo, node_id: int, start_rad: float, target_rad: float,
    velocity_rad_s: float,
) -> None:
    """Step the position setpoint from start_rad to target_rad at RAMP_HZ, easing into it
    rather than issuing a single step command -- see module docstring for why."""
    distance = target_rad - start_rad
    duration_s = abs(distance) / max(velocity_rad_s, 1e-6)
    steps = max(1, int(duration_s * RAMP_HZ))
    for i in range(1, steps + 1):
        pos_rad = start_rad + distance * (i / steps)
        pos_deg = pos_rad * 180.0 / 3.14159265
        bus.send(to_message(codec.command(node_id, position=pos_deg, velocity=0.0)))
        time.sleep(1.0 / RAMP_HZ)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--travel-mm", type=float, default=DEFAULT_TRAVEL_MM)
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY_RAD_S, help="ramp rate, rad/s")
    parser.add_argument(
        "--zero", action="store_true",
        help="also set the current position as a temporary origin before moving -- only do this "
             "once you've confirmed the motor is at the position you want to call 'retracted'",
    )
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    travel_m = args.travel_mm / 1000.0
    target_rad = DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)
    duration_s = abs(target_rad) / max(args.velocity, 1e-6)

    codec = CubeMarsServo()

    print(f"phantom motor (AK60-6 V3.0, servo mode), node id {MOTOR_NODE_ID}")
    print(f"target: {args.travel_mm:.1f} mm -> {target_rad:.4f} rad "
          f"({target_rad * 180 / 3.14159265:.2f} deg), ramped over ~{duration_s:.1f}s")

    if args.dry_run:
        print("\n--dry-run: not opening the bus. Mode frames and the first/last ramp steps:")
        print("  enable: ", to_message(codec.enable(MOTOR_NODE_ID)))
        if args.zero:
            print("  zero:   ", to_message(codec.zero(MOTOR_NODE_ID)))
        steps = max(int(duration_s * RAMP_HZ), 1)
        first_deg = (target_rad / steps) * 180.0 / 3.14159265
        last_deg = target_rad * 180.0 / 3.14159265
        print("  first ramp step:", to_message(codec.command(MOTOR_NODE_ID, position=first_deg, velocity=0.0)))
        print("  last ramp step: ", to_message(codec.command(MOTOR_NODE_ID, position=last_deg, velocity=0.0)))
        print("  disable:", to_message(codec.disable(MOTOR_NODE_ID)))
        return 0

    print(f"\nThis will move the phantom motor. Watch the axis -- it should move {args.travel_mm:.1f} mm, "
          f"dwell {DWELL_AT_EXTEND_S}s, then return to where it started.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    try:
        print("enabling (zero-current wake-up frame)...")
        bus.send(to_message(codec.enable(MOTOR_NODE_ID)))
        time.sleep(0.5)

        if args.zero:
            print("setting current position as a temporary origin...")
            bus.send(to_message(codec.zero(MOTOR_NODE_ID)))
            time.sleep(0.5)

        print(f"moving {args.travel_mm:.1f} mm ({target_rad:.4f} rad)...")
        ramp_to(bus, codec, MOTOR_NODE_ID, 0.0, target_rad, args.velocity)

        print(f"dwelling {DWELL_AT_EXTEND_S}s...")
        time.sleep(DWELL_AT_EXTEND_S)

        print("returning to 0 rad...")
        ramp_to(bus, codec, MOTOR_NODE_ID, target_rad, 0.0, args.velocity)

        print("disabling (zero current, motor coasts)...")
        bus.send(to_message(codec.disable(MOTOR_NODE_ID)))
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
