#!/usr/bin/env python3
"""Passive listener for the needle motor's (GL40II) CAN bus.

Diagnostic only -- never transmits anything. Run this in one terminal while running
scripts/test_needle_motor.py in another, to see whether the motor is replying at all,
and if so, what it's actually reporting (position/velocity/torque/fault) while commands
are sent.

The needle currently runs the Gimbal/Position-Velocity protocol (mode=1), not MIT mode
(mode=0) -- MIT mode was an abandoned detour, see docs/sessions/004. This listener still
applies: the CubeMars GL-II manual documents one feedback frame shared identically across
all three GL-II modes (arbitration ID = Master ID, default 0), which is exactly the format
CubeMarsMIT.parse() already decodes. Confirmed real 2026-08-27 for the base motor (see
scripts/run_approach_and_stop.py) as a per-command ACK, not a continuous broadcast -- the
same is expected but not yet independently confirmed for the needle specifically.

- No frames at all arrive here -> wrong channel/bitrate, or nothing is reaching the motor.
- Frames arrive on other IDs but never on ID 0 (the MIT-mode reply ID) -> the motor isn't
  replying to us, even if something else is on the bus.
- Reply shows a non-zero FAULT code -> that's the actual blocker, not a framing issue --
  see MIT_FAULT_CODES (this is what "node=146" turned out to be in an earlier session:
  node 2 reporting undervoltage, misdecoded before ct.hw.motors.cubemars_mit.CubeMarsMIT's
  parse() was fixed to split the id/fault nibbles).
- Reply frames arrive with near-zero torque and no fault -> commanded kp/kd are probably
  too weak to produce real torque.
- Reply frames arrive with torque near the GL40II's max and no position change -> a
  stall or mechanical block, not a weak-gains problem.

Uses ct.hw.motors.cubemars_mit.CubeMarsMIT.parse() to decode replies -- structurally the
right decoder even though the needle isn't running MIT mode, since the reply frame format
is shared across GL-II modes. The range constants below are GL-II's own documented values
(from the manual), NOT the MIT-mode ranges test_needle_motor.py's abandoned detour
confirmed via the GUI's MIT tab -- those were for a mode this motor isn't running.

Channel defaults to the bus the needle is currently wired to (it moved once already, from
the original needle/base adapter to the phantom's, to test whether the original bus itself
was the problem) -- override with --channel if it moves again.

    python scripts/listen_needle_motor.py
    python scripts/listen_needle_motor.py --channel /dev/cu.usbmodem20563976534B1
"""

from __future__ import annotations

import argparse
import time

import can

from ct.hw.motors.cubemars_mit import MIT_FAULT_CODES, CubeMarsMIT

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # needle's current adapter -- matches test_needle_motor.py
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # confirmed: GUI shows "CAN baud rate: 1000 kbps"

# GL-II manual's documented Position/Velocity-mode ranges (position, velocity) -- NOT the
# MIT-mode GUI-confirmed ranges test_needle_motor.py's docstring describes, which are for a
# mode this motor doesn't run. The manual is internally inconsistent about units ("rad/s" in
# one sentence, "r/s" in another for the same field) -- treat decoded velocity as provisional.
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -200.0, 200.0
# kp/kd are meaningless outside MIT mode (Position/Velocity mode has no such fields) --
# kept only because CubeMarsMIT's constructor requires them; never populated by a real reply.
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
# Torque range is NOT given in the GL-II manual -- this is the old MIT-mode value, kept as
# a best-effort placeholder. The bit field is read correctly; the N*m scale is not confirmed.
T_MIN, T_MAX = -10.0, 10.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default=CAN_CHANNEL, help="CAN adapter device path")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to listen, 0 = until Ctrl+C")
    args = parser.parse_args()

    codec = CubeMarsMIT(
        p_min=P_MIN, p_max=P_MAX, v_min=V_MIN, v_max=V_MAX,
        kp_min=KP_MIN, kp_max=KP_MAX, kd_min=KD_MIN, kd_max=KD_MAX, t_min=T_MIN, t_max=T_MAX,
    )

    print(f"listening: interface={CAN_INTERFACE} channel={args.channel} bitrate={BITRATE}")
    print("This never transmits. Run scripts/test_needle_motor.py in another terminal to generate traffic.\n")
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
            if msg.arbitration_id == 0:
                reply = codec.parse(msg.arbitration_id, bytes(msg.data))
                if reply is not None:
                    replies_seen += 1
                    fault = MIT_FAULT_CODES.get(int(reply["error"]))
                    fault_str = f"  FAULT={fault}" if fault else ""
                    print(f"t={t:6.2f}s  REPLY  node={int(reply['node_id'])}  "
                          f"pos={reply['position']:8.4f}rad  vel={reply['velocity']:8.3f}rad/s  "
                          # Labeled "current" by CubeMarsMIT.parse() (MIT-mode's own naming),
                          # but decoded via the torque range -- the GL-II manual's feedback
                          # table calls this same field torque, not current/amperage.
                          f"torque={reply['current']:7.3f}Nm{fault_str}")
                    continue
            print(f"t={t:6.2f}s  frame  id=0x{msg.arbitration_id:03X}  data={msg.data.hex()}")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    print(f"\n{frames_seen} frame(s) seen, {replies_seen} decoded as GL-II feedback replies from the motor.")
    if frames_seen == 0:
        print("Nothing arrived at all -- check channel/bitrate, or whether the CAN wiring to the "
              "motor driver is intact.")
    elif replies_seen == 0:
        print("Frames arrived but none decoded as a reply on ID 0 -- either this motor genuinely "
              "doesn't reply while running Position/Velocity mode, or the reply arrives on a "
              "different ID than the manual's documented default Master ID of 0. Check the raw "
              "frame lines above for anything unrecognized before concluding there's no reply at all.")
    else:
        print("Replies decoded -- check the reported torque above: near-zero suggests kp/kd are too "
              "weak to move it; near the GL40II's max with no position change suggests a stall/block.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
