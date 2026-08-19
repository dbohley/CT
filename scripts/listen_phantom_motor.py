#!/usr/bin/env python3
"""Passive listener for the phantom motor's (AK60-6, MIT mode) CAN bus.

Diagnostic only -- never transmits anything. Run this in one terminal while running
scripts/test_phantom_motor.py in another, to see whether the motor is replying at all,
and if so, what it's actually reporting (position/velocity/current) while commands are
sent. This is the concrete way to distinguish the leading hypotheses for why the phantom
motor didn't move on the last test:

- No frames at all arrive here -> wrong channel/bitrate, or nothing is reaching the motor.
- Frames arrive on other IDs but never on ID 0 (the MIT-mode reply ID) -> the motor isn't
  replying to us, even if something else is on the bus.
- Reply frames arrive with near-zero current -> commanded kp/kd are probably too weak to
  produce real torque.
- Reply frames arrive with current near the AK60-6's max and no position change -> a
  stall or fault, not a weak-gains problem.

Uses ct.hw.motors.cubemars_mit.CubeMarsMIT.parse() to decode replies -- the same codec
scripts/test_phantom_motor.py commands through, so the ranges below match that script's
(also unconfirmed for the AK60-6, see that script's docstring).

    python scripts/listen_phantom_motor.py
    python scripts/listen_phantom_motor.py --duration 60
"""

from __future__ import annotations

import argparse
import time

import can

from ct.hw.motors.cubemars_mit import CubeMarsMIT

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # phantom's adapter -- matches test_phantom_motor.py
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # matches test_phantom_motor.py; unconfirmed independently, see that script's docstring

# Same ranges test_phantom_motor.py uses -- AK80-9 defaults, unconfirmed for the AK60-6.
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -50.0, 50.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -18.0, 18.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to listen, 0 = until Ctrl+C")
    args = parser.parse_args()

    codec = CubeMarsMIT(
        p_min=P_MIN, p_max=P_MAX, v_min=V_MIN, v_max=V_MAX,
        kp_min=KP_MIN, kp_max=KP_MAX, kd_min=KD_MIN, kd_max=KD_MAX, t_min=T_MIN, t_max=T_MAX,
    )

    print(f"listening: interface={CAN_INTERFACE} channel={CAN_CHANNEL} bitrate={BITRATE}")
    print("This never transmits. Run scripts/test_phantom_motor.py in another terminal to generate traffic.\n")
    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)

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
            if msg.arbitration_id == 0:
                reply = codec.parse(msg.arbitration_id, bytes(msg.data))
                if reply is not None:
                    replies_seen += 1
                    print(f"t={t:6.2f}s  REPLY  node={int(reply['node_id'])}  "
                          f"pos={reply['position']:8.4f}rad  vel={reply['velocity']:8.3f}rad/s  "
                          f"current={reply['current']:7.3f}A")
                    continue
            print(f"t={t:6.2f}s  frame  id=0x{msg.arbitration_id:03X}  data={msg.data.hex()}")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print(f"\n{frames_seen} frame(s) seen, {replies_seen} decoded as MIT-mode replies from the motor.")
    if frames_seen == 0:
        print("Nothing arrived at all -- check channel/bitrate, or whether the CAN wiring to the "
              "motor driver is intact.")
    elif replies_seen == 0:
        print("Frames arrived but none decoded as MIT-mode replies on ID 0 -- the motor may not be "
              "replying, or the node id / reply format assumptions may be off.")
    else:
        print("Replies decoded -- check the reported current above: near-zero suggests kp/kd are too "
              "weak to move it; near the AK60-6's max with no position change suggests a stall/fault.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
