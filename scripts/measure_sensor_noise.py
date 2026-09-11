#!/usr/bin/env python3
"""Record both bench sensors while stationary and report their noise, framed around the
EKF's measurement noise ``R``.

Passive -- never transmits. Same CAN connection as ct-sensor-bench (build_bus_from_config
-> RH02Bus, decoding the Teensy sketch's CAN ID 5 frame), so it talks to whatever is
actually on the wire today rather than the rig's not-yet-real per-sensor CAN ids.

Why this matters for R: src/ct/control/context.py's `breath_hold_window` mechanism exists
specifically to replace the "residual-variance upper bound" R (CLAUDE.md: collapses NIS to
~0.027) with a genuine noise measurement -- `np.var(y, ddof=1)` over a stationary segment.
That is exactly what this script computes. `_read_state` in the same file shows *both*
sensors feed the real breathing measurement depending on state: ToF standoff distance when
not in contact, tactile deflection when in contact ("tactile wins whenever we are
touching"). So both numbers below matter, just for different regimes.

The wire already reports physical units (ToF mm, tactile/encoder-derived cm) rather than
raw counts, so there's no unmeasured counts-to-mm scale factor in the way here -- these
numbers convert to R in mm^2 with just a unit conversion.

**Caveat that does not have an established fix yet:** this is a stationary, free-air
measurement. The tactile arm is a mechanical deflecting linkage behind an encoder, not a
load cell -- once actually pressed against something, off-axis wobble (motion not purely
along the forward/back travel axis) will register on the encoder as spurious signal that
is not present in a free-air reading. The real in-contact noise is therefore probably
higher than what this script measures. No correction factor is established; this is
flagged as an open question, not silently corrected for.

    python scripts/measure_sensor_noise.py --duration 30
    python scripts/measure_sensor_noise.py --duration 60 --out outputs/sensor_noise/bench1
"""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import numpy as np

from ct.cli._common import save_json
from ct.hw.bus import build_bus_from_config
from ct.hw.config import BusConfig
from ct.rt.telemetry import TelemetryWriter, load_jsonl, to_arrays

_PAYLOAD = struct.Struct("<ff")  # float32 tof_mm, float32 dist_cm (angle field removed by firmware)

_DEFAULT_CHANNEL = "/dev/cu.usbmodem20553962534B1"  # sensor bus -- see ct.cli.sensor_bench
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "sensor_noise"

OFF_AXIS_CAVEAT = (
    "Stationary, free-air measurement. The tactile arm is a mechanical deflecting linkage "
    "behind an encoder, not a load cell -- once actually pressed against something, "
    "off-axis wobble (motion not purely along the forward/back travel axis) will register "
    "as spurious signal not present here. Real in-contact noise is probably higher than "
    "this number. No established correction factor exists yet -- treat as an open "
    "question, not something already accounted for."
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--interface", default="slcan", help="python-can interface backend")
    parser.add_argument("--channel", default=_DEFAULT_CHANNEL, help="e.g. /dev/cu.usbmodem... for slcan")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--can-id", type=int, default=5, dest="can_id")
    parser.add_argument("--fd", action="store_true", help="the sketch is classic CAN; leave off")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to record, 0 = until Ctrl+C")
    parser.add_argument("--out", type=Path, default=None, help="output directory; default outputs/sensor_noise/<timestamp>")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    print(f"connecting: interface={args.interface} channel={args.channel} "
          f"bitrate={args.bitrate} can_id={args.can_id}")
    bus = build_bus_from_config(
        "noise-bench",
        BusConfig(backend="rh02", interface=args.interface, channel=args.channel,
                  bitrate=args.bitrate, fd=args.fd),
    )
    print(f"connected. keep the rig stationary -- do not touch the tactile sensor.")
    print(f"recording for {'until Ctrl+C' if args.duration <= 0 else f'{args.duration:.0f}s'} "
          f"to {jsonl_path} ...")

    t0 = time.monotonic()
    writer = TelemetryWriter(jsonl_path)
    n = 0
    try:
        while args.duration <= 0 or (time.monotonic() - t0) < args.duration:
            for _stamp, can_id, data in bus.poll():
                if can_id != args.can_id or len(data) < _PAYLOAD.size:
                    continue
                tof_mm, dist_cm = _PAYLOAD.unpack(data[: _PAYLOAD.size])
                writer.write({
                    "t": time.monotonic() - t0,
                    "tof_mm": tof_mm,
                    "dist_cm": dist_cm,
                })
                n += 1
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        writer.close()
        bus.close()

    print(f"recorded {n} samples over {time.monotonic() - t0:.1f}s.")
    if n < 2:
        print("too few samples to compute noise statistics.")
        return 1

    records = load_jsonl(jsonl_path)
    cols = to_arrays(records, ["tof_mm", "dist_cm"])
    tof_mm = cols["tof_mm"][~np.isnan(cols["tof_mm"])]
    dist_cm = cols["dist_cm"][~np.isnan(cols["dist_cm"])]

    tof_var_mm2 = float(np.var(tof_mm, ddof=1))
    tof_std_mm = float(np.sqrt(tof_var_mm2))

    dist_var_cm2 = float(np.var(dist_cm, ddof=1))
    dist_var_mm2 = dist_var_cm2 * 100.0  # (cm->mm)^2 = 10^2
    dist_std_mm = float(np.sqrt(dist_var_mm2))

    print("\n=== ToF (standoff distance) -- R candidate for the NOT-in-contact regime ===")
    print(f"  n={len(tof_mm)}  mean={tof_mm.mean():.3f}mm  var={tof_var_mm2:.6f}mm^2  std={tof_std_mm:.4f}mm")

    print("\n=== tactile/encoder deflection -- R candidate for the IN-contact regime ===")
    print(f"  n={len(dist_cm)}  mean={dist_cm.mean():.5f}cm  var={dist_var_cm2:.8f}cm^2 "
          f"({dist_var_mm2:.6f}mm^2)  std={dist_std_mm:.4f}mm")
    print(f"\n  caveat: {OFF_AXIS_CAVEAT}")

    summary = {
        "duration_s": time.monotonic() - t0,
        "n_samples": n,
        "channel": args.channel,
        "bitrate": args.bitrate,
        "tof": {
            "regime": "not_in_contact",
            "n": int(len(tof_mm)),
            "mean_mm": float(tof_mm.mean()),
            "var_mm2": tof_var_mm2,
            "std_mm": tof_std_mm,
        },
        "tactile": {
            "regime": "in_contact",
            "n": int(len(dist_cm)),
            "mean_cm": float(dist_cm.mean()),
            "var_cm2": dist_var_cm2,
            "var_mm2": dist_var_mm2,
            "std_mm": dist_std_mm,
            "caveat": OFF_AXIS_CAVEAT,
        },
    }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
