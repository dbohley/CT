#!/usr/bin/env python3
"""Read-only(-ish) position/velocity/current/fault query for the needle motor.

**This used to speak MIT-mode addressing and was never updated after session 004 abandoned MIT
mode for this motor** (see docs/sessions/004) -- it was sending its probe to CAN ID 2 (the raw
node id) while the needle has run the Gimbal position/velocity protocol ever since, which
addresses frames at `(1<<8)|node_id = 0x102`. Nothing was listening on ID 2, so every call
printed "no reply received" regardless of whether the motor was fine. Found via
scripts/characterize_needle_plant.py's pre-flight stage refusing to proceed. Now mirrors
scripts/test_needle_motor.py's protocol like every other needle script.

**Not perfectly passive.** Unlike the old MIT-mode zero-effort probe (kp=0, kd=0, torque=0,
which genuinely commands nothing regardless of position), the Gimbal position/velocity
protocol has no such null command -- eliciting a reply means sending a real
(position, velocity-limit) frame, same bootstrap trick scripts/run_needle_step_response.py and
scripts/run_needle_sine_tracking.py already use to learn their own start_rad. This targets
0.0 rad at a slow, conservative velocity limit and immediately de-energizes (EXIT_MODE) the
moment a reply arrives or the wait times out, so any real displacement is bounded by
`velocity_limit * wait_time` (0.05rad/s * 0.5s = 0.025rad, about 0.45mm) -- small, brief, and
the same order of magnitude as the bootstrap step other already-run scripts perform.

Useful for checking where the needle actually is and whether it's replying/fault-free before
running anything else -- e.g. as characterize_needle_plant.py's pre-flight stage.

    python scripts/read_needle_position.py
"""

from __future__ import annotations

import struct
import time

import can

from ct.hw.motors.cubemars_mit import MIT_FAULT_CODES, CubeMarsMIT

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # needle's current adapter -- matches test_needle_motor.py
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

MOTOR_NODE_ID = 2
POSITION_VELOCITY_MODE = 1
PROBE_VELOCITY_RAD_S = 0.05  # slow and brief -- see module docstring
REPLY_WAIT_S = 0.5

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

# GL-II's documented feedback frame (arbitration ID = Master ID, default 0), shared across all
# GL-II modes -- see scripts/run_needle_sine_tracking.py, confirmed real 2026-08-27.
MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def main() -> int:
    codec = CubeMarsMIT(
        p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
        v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
        t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
    )

    print(f"connecting: channel={CAN_CHANNEL} bitrate={BITRATE}, node id {MOTOR_NODE_ID}, "
          f"arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    reply = None
    try:
        bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)
        bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.1)

        bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, PROBE_VELOCITY_RAD_S))

        deadline = time.monotonic() + REPLY_WAIT_S
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=deadline - time.monotonic())
            if msg is None:
                break
            parsed = codec.parse(msg.arbitration_id, bytes(msg.data))
            if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
                reply = parsed
                break
    finally:
        bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))
        bus.shutdown()

    if reply is None:
        print("no reply received -- motor may not be on this bus/id, or not replying.")
        return 1

    fault = MIT_FAULT_CODES.get(int(reply["error"]))
    print(f"position: {reply['position']:8.4f} rad ({reply['position'] * 180 / 3.14159265:7.2f} deg)")
    print(f"velocity: {reply['velocity']:8.4f} rad/s (this field's scale is unconfirmed -- see "
          f"scripts/fit_needle_plant.py's docstring)")
    print(f"current:  {reply['current']:8.4f} A")
    print(f"fault:    {fault or 'none (raw error nibble: ' + str(int(reply['error'])) + ')'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
