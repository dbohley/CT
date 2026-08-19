"""``ct-sensor-bench`` — live-decode the Teensy bench sketch's CAN frame.

Talks to whatever is actually on the wire from ``CT_sensors/src/main.cpp`` right now: one
frame on CAN ID 5 carrying ToF distance, an encoder-derived distance, and encoder angle.
This predates the rig's config (``configs/rig_bench.yaml`` targets a different bitrate and
per-sensor CAN IDs for the eventual wiring) so it talks directly to :class:`RH02Bus`
rather than going through :class:`~ct.hw.config.RigSession`.

    ct-sensor-bench --interface slcan --channel /dev/cu.usbmodem20563976534B1
"""

from __future__ import annotations

import argparse
import struct
import time

from ct.hw.bus import build_bus_from_config
from ct.hw.config import BusConfig

_PAYLOAD = struct.Struct("<Hfh")  # uint16 tof_mm, float32 dist_cm, int16 angle_centideg


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print the Teensy bench sketch's CAN frame live.")
    parser.add_argument("--interface", default="slcan", help="python-can interface backend")
    parser.add_argument("--channel", required=True, help="e.g. /dev/cu.usbmodem... for slcan")
    parser.add_argument("--bitrate", type=int, default=500_000, help="matches the sketch's setBaudRate")
    parser.add_argument("--can-id", type=int, default=5, dest="can_id")
    parser.add_argument("--fd", action="store_true", help="the sketch is classic CAN; leave off")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    print(f"connecting: interface={args.interface} channel={args.channel} "
          f"bitrate={args.bitrate} can_id={args.can_id}")
    bus = build_bus_from_config(
        "bench",
        BusConfig(backend="rh02", interface=args.interface, channel=args.channel,
                  bitrate=args.bitrate, fd=args.fd),
    )
    print("connected. waiting for frames (Ctrl+C to stop)...")

    t0 = time.monotonic()
    try:
        while True:
            for _stamp, can_id, data in bus.poll():
                if can_id != args.can_id:
                    continue
                if len(data) < _PAYLOAD.size:
                    print(f"t={time.monotonic() - t0:6.2f}s  short frame ({len(data)} bytes), skipping")
                    continue
                tof_mm, dist_cm, angle_centideg = _PAYLOAD.unpack(data[: _PAYLOAD.size])
                print(f"t={time.monotonic() - t0:6.2f}s  ToF={tof_mm:4d}mm  "
                      f"dist={dist_cm:7.3f}cm  angle={angle_centideg / 100:7.2f}deg")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
