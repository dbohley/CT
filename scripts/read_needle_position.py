#!/usr/bin/env python3
"""Read-only position/velocity/current/fault query for the needle motor (MIT mode).

Sends exactly one command: kp=0, kd=0, torque=0 -- a genuine zero-effort frame, the same
"float" command ct.hw.motors.cubemars_mit.CubeMarsMIT's own docs describe as true
zero-stiffness backdrive. That commands nothing regardless of the motor's current
position or velocity, so this never drives it anywhere -- it only elicits a reply so the
current state can be read. Enters motor control mode to get that reply, then exits
immediately after.

Useful for checking where the needle actually is (e.g. before deciding whether to pass
--zero on the next test_needle_motor.py run) without risking any motion at all.

    python scripts/read_needle_position.py
"""

from __future__ import annotations

import time

import can

from ct.hw.motors.cubemars_mit import MIT_FAULT_CODES, CubeMarsMIT

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # needle's current adapter -- matches test_needle_motor.py
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

MOTOR_NODE_ID = 2

# Confirmed via the GUI's Control Parameter sliders -- see test_needle_motor.py's docstring.
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -10.0, 10.0


def main() -> int:
    codec = CubeMarsMIT(
        p_min=P_MIN, p_max=P_MAX, v_min=V_MIN, v_max=V_MAX,
        kp_min=KP_MIN, kp_max=KP_MAX, kd_min=KD_MIN, kd_max=KD_MAX, t_min=T_MIN, t_max=T_MAX,
    )

    print(f"connecting: channel={CAN_CHANNEL} bitrate={BITRATE}, node id {MOTOR_NODE_ID}")
    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    reply = None
    try:
        can_id, data, ext = codec.enable(MOTOR_NODE_ID)
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=ext))
        time.sleep(0.2)

        can_id, data, ext = codec.command(MOTOR_NODE_ID, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=ext))

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=deadline - time.monotonic())
            if msg is None:
                break
            if msg.arbitration_id != 0:
                continue
            parsed = codec.parse(msg.arbitration_id, bytes(msg.data))
            if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
                reply = parsed
                break

        can_id, data, ext = codec.disable(MOTOR_NODE_ID)
        bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=ext))
    finally:
        bus.shutdown()

    if reply is None:
        print("no reply received -- motor may not be on this bus/id, or not replying.")
        return 1

    fault = MIT_FAULT_CODES.get(int(reply["error"]))
    print(f"position: {reply['position']:8.4f} rad ({reply['position'] * 180 / 3.14159265:7.2f} deg)")
    print(f"velocity: {reply['velocity']:8.4f} rad/s")
    print(f"current:  {reply['current']:8.4f} A")
    print(f"fault:    {fault or 'none'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
