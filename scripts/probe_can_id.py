#!/usr/bin/env python3
"""Quick send-and-listen probe for one CAN node on the needle/base bus.

Sends a single, motion-incapable frame (CLEAR_ERRORS -- the Gimbal Motor II protocol's
fault-clear command, never a position/motion command) addressed to the given node id, and
listens for any CAN traffic on the bus before and after, printing everything raw. This is
a targeted version of the diagnosis already done for the phantom motor
(scripts/listen_phantom_motor.py): the point isn't to move anything, it's to see whether
*any* traffic appears on the bus around the send, given the LED evidence suggesting the
needle/base adapter stops transmitting successfully after its first frame.

Defaults to node id 2 (the needle motor), same addressing scheme as
scripts/test_needle_motor.py: arbitration id = (mode << 8) | node_id, mode=1
(position/velocity). Use --node-id 3 for the base motor instead -- same bus, same adapter.

This sends one real frame to the motor. Nothing in this file runs on import -- you run it
yourself:

    python scripts/probe_can_id.py
    python scripts/probe_can_id.py --node-id 3
"""

from __future__ import annotations

import argparse
import time

import can

#CAN_CHANNEL = "/dev/cu.usbmodem20563976534B1"  # needle/base adapter -- confirm with `ls /dev/cu.*`
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

POSITION_VELOCITY_MODE = 1
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def listen_for(bus: can.BusABC, seconds: float, label: str) -> int:
    count = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        msg = bus.recv(timeout=0.2)
        if msg is None:
            continue
        count += 1
        print(f"  [{label}] t={time.monotonic() - t0:5.2f}s  id=0x{msg.arbitration_id:03X}  "
              f"ext={msg.is_extended_id}  data={msg.data.hex()}")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--node-id", type=int, default=2, help="Gimbal Motor II node id (2=needle, 3=base)")
    parser.add_argument("--listen-before", type=float, default=1.0, help="seconds to listen before sending")
    parser.add_argument("--listen-after", type=float, default=3.0, help="seconds to listen after sending")
    args = parser.parse_args()

    can_id = pos_vel_can_id(args.node_id)
    print(f"probing node id {args.node_id}, arbitration id 0x{can_id:03X}, on {CAN_CHANNEL} @ {BITRATE}")
    print("Sends only CLEAR_ERRORS (fault-clear, not a motion command).")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    try:
        print(f"\nlistening for {args.listen_before:.1f}s before sending (ambient traffic check)...")
        before_count = listen_for(bus, args.listen_before, "before")

        print(f"\nsending CLEAR_ERRORS to node {args.node_id} (id 0x{can_id:03X})...")
        bus.send(can.Message(arbitration_id=can_id, data=CLEAR_ERRORS, is_extended_id=False))

        print(f"listening for {args.listen_after:.1f}s after sending...")
        after_count = listen_for(bus, args.listen_after, "after")
    finally:
        bus.shutdown()

    print(f"\n{before_count} frame(s) seen before sending, {after_count} frame(s) seen after.")
    if before_count == 0 and after_count == 0:
        print("Nothing arrived at all, in either window -- consistent with the bus being silent/broken, "
              "not just the motor being unresponsive.")
    elif after_count > before_count:
        print("More traffic after the send than before -- something reacted. Worth a closer look at what.")
    else:
        print("No visible change in traffic around the send.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
