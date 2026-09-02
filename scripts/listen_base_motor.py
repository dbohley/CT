#!/usr/bin/env python3
"""Passive listener for the base motor's (GL60II) CAN bus.

Diagnostic only -- never transmits anything. Run this in one terminal while running
scripts/test_base_motor.py in another, to see whether the motor is replying at all, and if
so, what it's actually reporting (position/velocity/torque/fault) while commands are sent.

The base runs the Gimbal/Position-Velocity protocol (mode=1). The CubeMars GL-II manual
documents one feedback frame shared identically across all three GL-II modes (arbitration
ID = Master ID, default 0) -- exactly the format ct.hw.motors.cubemars_mit.CubeMarsMIT's
parse() already decodes. Confirmed real 2026-08-27 (see scripts/run_approach_and_stop.py),
but only as a per-command ACK, not a continuous broadcast -- expect one reply per command
sent (test_base_motor.py sends only a handful per run), not a steady stream.

- No frames at all arrive here -> wrong channel/bitrate, or nothing is reaching the motor.
- Frames arrive on other IDs but never on ID 0 -> the motor isn't replying to us on the
  manual's documented default Master ID, even if something else is on the bus.
- Reply shows a non-zero FAULT code -> see MIT_FAULT_CODES.

Range constants are GL-II's own documented values (position, velocity), not MIT-mode's --
this motor has never run MIT mode, so there's no GUI-confirmed alternative to fall back on.
Torque range is not given in the GL-II manual (only position and speed are); the decoded
torque number uses a placeholder range and is not trustworthy yet, even though the bit
field itself is being read correctly.

    python scripts/listen_base_motor.py
    python scripts/listen_base_motor.py --channel /dev/cu.usbmodem20563976534B1
"""

from __future__ import annotations

import argparse
import time

import can

from ct.hw.motors.cubemars_mit import MIT_FAULT_CODES, CubeMarsMIT

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # base's current adapter -- matches test_base_motor.py
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

# GL-II manual's documented Position/Velocity-mode ranges. The manual is internally
# inconsistent about units ("rad/s" in one sentence, "r/s" in another for the same field)
# -- treat decoded velocity as provisional until checked against known behavior.
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -200.0, 200.0
# kp/kd are meaningless outside MIT mode -- kept only because CubeMarsMIT's constructor
# requires them; never populated by a real reply.
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
# Current/torque range is NOT given in the GL-II manual -- best-effort placeholder.
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
    print("This never transmits. Run scripts/test_base_motor.py in another terminal to generate traffic.\n")
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
        print("Replies decoded -- check the reported torque above: near-zero suggests weak load; "
              "a nonzero fault code is the actual blocker, not a framing issue.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
