"""``ct-sensor-bench`` — live-decode the Teensy bench sketch's CAN frame.

Talks to whatever is actually on the wire from ``CT_sensors/src/can_sensors.cpp`` right
now: one frame on CAN ID 5 carrying ToF distance, an encoder-derived distance, and encoder
angle, sent at 100 Hz via a ``millis()``-scheduled loop (previously ``main.cpp``, retired
2026-08-25 after a reflash fixed a CAN no-ACK/retransmit-storm problem — see below). This
predates the rig's config (``configs/rig_bench.yaml`` targets per-sensor CAN IDs for the
eventual wiring) so it talks directly to :class:`RH02Bus` rather than going through
:class:`~ct.hw.config.RigSession`.

The bench is wired at **1 Mbps**, confirmed both by probing all 3 CAN adapters at both
500k/1M and by the current firmware's own explicit ``setBaudRate(1000000)``. Earlier
(``main.cpp``) the sketch's own comment claimed 500 kbps while actually running 1 Mbps —
that mismatch is gone now that the source says what it does.

**2026-08-24 → 2026-08-25 corruption incident, resolved by the firmware rewrite:** at the
old firmware's 5 Hz send rate, raw serial throughput measured ~12x the expected byte rate
(1280 B/s vs. ~110 B/s expected) with a matching parse-failure rate of roughly 60/sec at
the slcan-ASCII layer — a CAN no-ACK/retransmit storm, not a bug in this file or in
:mod:`ct.hw.bus`. ``can_sensors.cpp``'s new error counters (``can1.error()``,
``"No ACK from receiver"`` on write failure) and non-blocking 100 Hz output resolved it:
post-reflash, raw throughput matches the new expected rate (~2 kB/s @ 100 Hz) and 0/4227
captured frames were malformed in a 45s probe. If corruption reappears, check the
Teensy's own serial monitor for TX/RX error counts before suspecting this script.

The encoder-derived ``dist_cm`` field *is* the tactile signal here — the bench has a
mechanical deflecting arm/linkage behind the AS5048B encoder, not a separate force sensor.
"Touch" is reported whenever ``dist_cm`` has moved far enough from its zeroed-at-boot
baseline, per ``--contact-threshold-cm``.

    ct-sensor-bench --interface slcan --channel /dev/cu.usbmodem20553962534B1 \\
        --contact-threshold-cm 0.05
"""

from __future__ import annotations

import argparse
import struct
import time

from ct.hw.bus import build_bus_from_config
from ct.hw.config import BusConfig

_PAYLOAD = struct.Struct("<Hfh")  # uint16 tof_mm, float32 dist_cm, int16 angle_centideg

# Inferred: the one usbmodem port not already tied to a motor adapter (20563976534B1 /
# 207635764E451, see docs/sessions/004-...md) — not independently confirmed. Override
# with --channel if this is wrong.
_DEFAULT_CHANNEL = "/dev/cu.usbmodem20553962534B1"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print the Teensy bench sketch's CAN frame live.")
    parser.add_argument("--interface", default="slcan", help="python-can interface backend")
    parser.add_argument("--channel", default=_DEFAULT_CHANNEL, help="e.g. /dev/cu.usbmodem... for slcan")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="confirmed empirically 2026-08-24: "
                         "main.cpp's setBaudRate(500000) is stale, the wire is actually 1 Mbps")
    parser.add_argument("--can-id", type=int, default=5, dest="can_id")
    parser.add_argument("--fd", action="store_true", help="the sketch is classic CAN; leave off")
    parser.add_argument(
        "--contact-threshold-cm", type=float, default=0.05, dest="contact_threshold_cm",
        help="abs(dist_cm) above this counts as touching the phantom -- unmeasured starting "
             "point, tune down at the bench until stationary readings stop tripping it",
    )
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
                touch = "TOUCH" if abs(dist_cm) > args.contact_threshold_cm else "-----"
                print(f"t={time.monotonic() - t0:6.2f}s  ToF={tof_mm:4d}mm  "
                      f"dist={dist_cm:7.3f}cm  angle={angle_centideg / 100:7.2f}deg  {touch}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
