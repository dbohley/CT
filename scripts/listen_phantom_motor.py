#!/usr/bin/env python3
"""Passive listener for the phantom motor's (AK60-6, servo mode) CAN bus.

Diagnostic only -- never transmits anything. Run this in one terminal while running
scripts/test_phantom_motor.py in another, to see whether the motor is replying at all,
and if so, what it's actually reporting (position/velocity/current) while commands are
sent. This is the concrete way to distinguish the leading hypotheses for why the phantom
motor didn't move on the last test:

- No frames at all arrive here -> wrong channel/bitrate, or nothing is reaching the motor.
- Frames arrive on other IDs but never on node_id | (0x29 << 8) (the servo-mode status-1 ID
  on this firmware, e.g. 0x2901 for node 1 -- see ct.hw.motors.cubemars_servo's module
  docstring for how this was confirmed; it is NOT stock VESC's CAN_PACKET_STATUS=9) -> the
  motor isn't replying to us, even if something else is on the bus.
- Reply frames arrive with near-zero current -> commanded kp/kd are probably too weak to
  produce real torque (though servo mode's own gains are internal -- see
  ct.hw.motors.cubemars_servo.CubeMarsServo.command()'s docstring).
- Reply frames arrive with current near the AK60-6's max and no position change -> a
  stall or fault, not a weak-gains problem.

This previously used ct.hw.motors.cubemars_mit.CubeMarsMIT (MIT mode's codec) and checked
arbitration_id == 0, MIT mode's reply convention -- both wrong for this motor. Per
scripts/test_phantom_motor.py's docstring, this project already found (once, via a raw hex
dump on this exact tool before it was updated) that the AK60-6 is flashed for **servo
mode**: an *extended* 29-bit ID, `node_id | (command << 8)`. test_phantom_motor.py and
run_breathing_profile.py were updated to ct.hw.motors.cubemars_servo.CubeMarsServo
accordingly; this listener was not, so it could never decode a real reply regardless of
channel. Fixed here to match -- and while fixing it, a second bug surfaced: real frames
arrived on `node_id | (0x29 << 8)`, not the `(9 << 8)` the codec assumed (stock VESC's
value). ct.hw.motors.cubemars_servo.CMD_STATUS_1 has been corrected to 0x29 -- this listener
would still have decoded nothing without that fix too.

Servo mode reports position in **degrees**, not radians (unlike MIT mode).

    python scripts/listen_phantom_motor.py
    python scripts/listen_phantom_motor.py --duration 60
"""

from __future__ import annotations

import argparse
import time

import can

from ct.hw.motors.cubemars_servo import CubeMarsServo

CAN_CHANNEL = "/dev/cu.usbmodem20563976534B1"  # phantom's adapter -- matches test_phantom_motor.py's
# true (post-override) channel, not its dead first CAN_CHANNEL line. This constant previously
# pointed at .../207635764E451, which scan_can_bus.py identifies as the needle/base adapter --
# a stale copy of test_phantom_motor.py's overridden first assignment, not its effective one.
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # matches test_phantom_motor.py; unconfirmed independently, see that script's docstring


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default=CAN_CHANNEL, help="CAN adapter device path")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to listen, 0 = until Ctrl+C")
    args = parser.parse_args()

    codec = CubeMarsServo()

    print(f"listening: interface={CAN_INTERFACE} channel={args.channel} bitrate={BITRATE}")
    print("This never transmits. Run scripts/test_phantom_motor.py in another terminal to generate traffic.\n")
    bus = can.interface.Bus(channel=args.channel, interface=CAN_INTERFACE, bitrate=BITRATE)

    t0 = time.monotonic()
    frames_seen = 0
    replies_seen = 0
    try:
        while args.duration <= 0 or (time.monotonic() - t0) < args.duration:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            frames_seen += 1
            t = time.monotonic() - t0
            reply = codec.parse(msg.arbitration_id, bytes(msg.data))
            if reply is not None:
                replies_seen += 1
                extra = ""
                if "temperature" in reply:
                    extra = f"  temp={reply['temperature']:.0f}C  error={int(reply['error'])}"
                print(f"t={t:6.2f}s  REPLY  node={int(reply['node_id'])}  "
                      f"pos={reply['position']:8.2f}deg  vel={reply['velocity']:8.1f}ERPM  "
                      f"current={reply['current']:7.3f}A{extra}")
                continue
            print(f"t={t:6.2f}s  frame  id=0x{msg.arbitration_id:06X}  data={msg.data.hex()}")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print(f"\n{frames_seen} frame(s) seen, {replies_seen} decoded as servo-mode status-1 replies from the motor.")
    if frames_seen == 0:
        print("Nothing arrived at all -- check channel/bitrate, or whether the CAN wiring to the "
              "motor driver is intact.")
    elif replies_seen == 0:
        print("Frames arrived but none decoded as a status-1 reply (node_id | (0x29 << 8)) -- the "
              "motor may not be replying, or the node id / reply format assumptions may be off.")
    else:
        print("Replies decoded -- check the reported current above: near-zero suggests weak/no torque "
              "commanded; near the AK60-6's max with no position change suggests a stall/fault.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
