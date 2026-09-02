#!/usr/bin/env python3
"""Scan for live CAN node IDs on the needle/base bus (or any bus via --channel).

Two phases, safest first:

1. **Passive listen** -- sends nothing, just watches the bus for a while. If any node is
   alive and broadcasting on its own (the way the phantom motor turned out to be, once
   scripts/listen_phantom_motor.py was pointed at it), this phase alone reveals it with
   zero transmission risk.
2. **Active scan** -- only reached if you confirm it. Steps through a range of candidate
   node IDs, sending the same motion-incapable CLEAR_ERRORS frame
   (scripts/probe_can_id.py uses the same one) to each in turn, and watches for any
   reaction. Point of this: confirm whether the needle/base motors are still actually
   configured at node IDs 2/3, or whether something reassigned them -- CAN node IDs live
   in the driver board's own stored config (set via CubeMars's own tool), completely
   independent of anything in this repo, so a reassignment wouldn't show up as an error
   anywhere in our scripts, just silence addressed to the wrong ID forever.

Uses the same Gimbal Motor II addressing as scripts/test_needle_motor.py:
arbitration id = (mode << 8) | node_id, mode=1 (position/velocity).

This can send real frames to the bus (phase 2 only, and only after confirming). Nothing
in this file runs on import -- you run it yourself:

    python scripts/scan_can_bus.py
    python scripts/scan_can_bus.py --channel /dev/cu.usbmodem20563976534B1
    python scripts/scan_can_bus.py --min-id 1 --max-id 32
"""

from __future__ import annotations

import argparse
import time

import can

CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # needle/base adapter -- confirm with `ls /dev/cu.*`.
# Was 20563976534B1; fixed per session 004's topology swap -- that address is now the
# phantom's adapter, not needle/base's. Anyone running this without --channel before this
# fix was listening on the wrong bus.
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

POSITION_VELOCITY_MODE = 1
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def drain(bus: can.BusABC, seconds: float, on_frame) -> int:
    count = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        count += 1
        on_frame(msg, time.monotonic() - t0)
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default=CAN_CHANNEL, help="CAN adapter device path")
    parser.add_argument("--listen-seconds", type=float, default=8.0, help="phase 1 passive listen duration")
    parser.add_argument("--min-id", type=int, default=1)
    parser.add_argument("--max-id", type=int, default=16)
    parser.add_argument("--settle-seconds", type=float, default=0.3, help="phase 2: listen time after each probe")
    args = parser.parse_args()

    print(f"channel={args.channel} interface={CAN_INTERFACE} bitrate={BITRATE}")
    bus = can.interface.Bus(channel=args.channel, interface=CAN_INTERFACE, bitrate=BITRATE)
    try:
        print(f"\n--- phase 1: passive listen for {args.listen_seconds:.1f}s, sending nothing ---")
        seen_ids: set[int] = set()

        def note(msg: can.Message, t: float) -> None:
            seen_ids.add(msg.arbitration_id)
            print(f"  t={t:5.2f}s  id=0x{msg.arbitration_id:03X}  ext={msg.is_extended_id}  "
                  f"data={msg.data.hex()}")

        passive_count = drain(bus, args.listen_seconds, note)
        print(f"\n{passive_count} frame(s) seen passively, from {len(seen_ids)} distinct id(s): "
              f"{sorted(f'0x{i:03X}' for i in seen_ids)}")
        if seen_ids:
            print("Something is alive and broadcasting on its own -- no need to guess further; "
                  "check those id(s) against what node id(s) you expect.")

        print(f"\n--- phase 2: active scan, node ids {args.min_id}..{args.max_id} ---")
        print("Sends CLEAR_ERRORS (not a motion command) to each id in turn.")
        if not confirm("Proceed with phase 2?"):
            print("skipped phase 2.")
            return 0

        reactive_ids: set[int] = set()
        for node_id in range(args.min_id, args.max_id + 1):
            can_id = pos_vel_can_id(node_id)
            before = set(seen_ids)

            def note2(msg: can.Message, t: float, _node_id=node_id) -> None:
                seen_ids.add(msg.arbitration_id)
                print(f"  probing node {_node_id} (0x{can_id:03X})  t={t:5.2f}s  "
                      f"reply-ish id=0x{msg.arbitration_id:03X}  data={msg.data.hex()}")

            bus.send(can.Message(arbitration_id=can_id, data=CLEAR_ERRORS, is_extended_id=False))
            drain(bus, args.settle_seconds, note2)
            if seen_ids - before:
                reactive_ids.add(node_id)

        print(f"\nnode ids that produced new traffic when probed: {sorted(reactive_ids) or 'none'}")
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
