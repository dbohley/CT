#!/usr/bin/env python3
"""Hold the needle motor at a fixed position with no compensator and measure how noisy its own
reported position is -- the number needed to answer whether the lead compensator's gain
(``configs/rig_bench.yaml``'s ``servo.needle.lead.gain``, 12.74) is too aggressive for what this
axis's real position feedback actually looks like.

**Why this matters now.** `scripts/run_needle_lead_tracking_live.py` showed the compensated
phase oscillating even with a rate limit on the correction (`rate_limited_steps` engaged on
~90% of ticks in one real run) -- the correction was chasing something that changes almost every
tick. A lead compensator's zero amplifies exactly that kind of tick-to-tick change (see
`ct.control.servo.LeadServo`'s docstring), by a factor that grows toward `lead.gain` at
frequencies above the pole. Until now "the gain is too high" was an inference from watching the
needle oscillate; this script turns it into a measured number: how much does the real position
reading actually move, tick to tick, when nothing is asking it to move at all.

**Command, don't read.** The GL-II position/velocity protocol has no "just tell me where you
are" query -- the only way to get a fresh reply is to send a position/velocity command and wait
for the ACK (confirmed real 2026-08-27, not a continuous broadcast). So "holding still" here
means repeatedly commanding the needle's own starting position back to itself, the same
periodic-resend pattern every other needle script already uses.

**Two different numbers are reported, and the second is the one that matters for compensator
design.** The raw log (sampled at --sample-hz, much faster than fresh replies actually arrive)
mostly just repeats the same stale reading between command ticks -- averaging across that would
understate the real noise. The per-tick series (one value per distinct fresh reply, at
--command-hz) is what a compensator running at that same rate actually sees, and its tick-to-tick
delta is exactly the quantity `LeadServo`'s zero amplifies. This script reports both, and
projects what the *current* compensator gain would do to that delta, so the "is the gain too
high" question has a number attached rather than a visual impression.

    python scripts/measure_needle_position_noise.py --dry-run
    python scripts/measure_needle_position_noise.py --duration-s 15
"""

from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import can

from ct.cli._common import save_json
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.rt.telemetry import TelemetryWriter, load_jsonl, to_arrays

# ---------------- Needle motor config (mirrors scripts/run_needle_sine_tracking.py -- keep in sync) --------
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # confirm with `ls /dev/cu.*` -- may differ
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

MOTOR_NODE_ID = 2
POSITION_VELOCITY_MODE = 1
DRUM_RADIUS_M = 0.018
DIRECTION_SIGN = -1

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
SET_ZERO = bytes([0xFF] * 7 + [0xFE])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0
# --------------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_position_noise"

DEFAULT_DURATION_S = 10.0
DEFAULT_HOLD_VELOCITY_RAD_S = 0.2  # slow, fixed -- matches other scripts' retract velocity
DEFAULT_COMMAND_HZ = 20.0
DEFAULT_SAMPLE_HZ = 100.0
DEFAULT_LEAD_GAIN = 12.74  # for the projection only -- matches configs/rig_bench.yaml
INITIAL_REPLY_WAIT_S = 0.5
SETTLE_MARGIN_S = 3.0


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S, dest="duration_s")
    parser.add_argument("--hold-velocity-rad-s", type=float, default=DEFAULT_HOLD_VELOCITY_RAD_S,
                         dest="hold_velocity_rad_s",
                         help="velocity field sent with every hold command -- the position "
                              "field is always the needle's own starting position")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ, dest="command_hz",
                         help="rate to resend the hold command -- also the rate at which fresh "
                              "position feedback becomes available, i.e. the rate a real "
                              "compensator would actually see this noise at")
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ, dest="sample_hz")
    parser.add_argument("--lead-gain", type=float, default=DEFAULT_LEAD_GAIN, dest="lead_gain",
                         help="compensator gain to project the measured noise through, for "
                              "an estimate of the resulting correction noise -- see "
                              "configs/rig_bench.yaml's servo.needle.lead.gain")
    parser.add_argument("--zero", action="store_true",
                         help="also set the current position as zero before starting")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/needle_position_noise/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    command_period_s = 1.0 / max(args.command_hz, 1e-6)

    print(f"needle motor (GL40II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"holding the needle's own starting position for {args.duration_s:.1f}s, resending at "
          f"{args.command_hz:.0f}Hz, sampling/logging at {args.sample_hz:.0f}Hz")
    print("no compensator, no reference motion -- this measures pure position-feedback noise "
          "at rest, the quantity LeadServo's zero amplifies by up to ~lead.gain at high frequency.")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        if args.zero:
            print(" ", universal_command(MOTOR_NODE_ID, SET_ZERO))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.hold_velocity_rad_s),
              " <- initial, to read start_rad")
        print(f"  ... (resent at {args.command_hz:.0f}Hz for {args.duration_s:.1f}s total) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE))
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    print(f"\nThis will hold the needle motor still for {args.duration_s:.1f}s. Ctrl+C stops and "
          "de-energizes immediately.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    writer = TelemetryWriter(jsonl_path)
    motor_reply_codec = CubeMarsMIT(
        p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
        v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
        t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
    )

    motion_commanded = False
    motor_replies_seen = 0
    last_motor_reply: dict | None = None

    def read_reply(timeout: float) -> dict | None:
        nonlocal last_motor_reply, motor_replies_seen
        msg = bus.recv(timeout=timeout)
        if msg is None:
            return None
        parsed = motor_reply_codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
            last_motor_reply = parsed
            motor_replies_seen += 1
            return parsed
        return None

    def stop_motor(reason: str) -> None:
        print(f"stopping motor ({reason})...")
        bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))

    start_rad = 0.0
    start_rad_note = "assumed 0.0 -- no real reply arrived before starting"

    try:
        print("clearing errors...")
        bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)

        print("entering motor control mode...")
        bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.5)

        if args.zero:
            print("setting current position as zero...")
            bus.send(universal_command(MOTOR_NODE_ID, SET_ZERO))
            time.sleep(0.5)
            start_rad = 0.0
            start_rad_note = "zeroed via --zero immediately before this run"
        else:
            print("reading current position before starting...")
            bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.hold_velocity_rad_s))
            motion_commanded = True
            reply = read_reply(INITIAL_REPLY_WAIT_S)
            if reply is not None:
                start_rad = reply["position"]
                start_rad_note = "measured from the motor's first real reply"
            print(f"start_rad = {start_rad:.4f}rad ({start_rad_note})")

        print(f"\nholding for {args.duration_s:.1f}s (Ctrl+C to stop early and de-energize)...")
        t0 = time.monotonic()
        sample_period_s = 1.0 / max(args.sample_hz, 1e-6)
        next_sample_at = 0.0
        next_command_at = 0.0
        safety_timeout_s = args.duration_s + SETTLE_MARGIN_S

        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= safety_timeout_s:
                print(f"\nreached safety timeout ({safety_timeout_s:.1f}s) -- stopping.")
                break

            read_reply(0.0)  # non-blocking

            if elapsed >= next_command_at:
                bus.send(build_pos_vel_frame(MOTOR_NODE_ID, start_rad, args.hold_velocity_rad_s))
                motion_commanded = True
                next_command_at = elapsed + command_period_s

            if elapsed >= next_sample_at:
                writer.write({
                    "t": elapsed,
                    "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                    "motor_position_mm": (
                        (last_motor_reply["position"] - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0
                        if last_motor_reply else None
                    ),
                    "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                    "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                })
                next_sample_at = elapsed + sample_period_s

            if elapsed >= args.duration_s:
                print(f"\nhold complete at t={elapsed:.2f}s.")
                break

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        if motion_commanded:
            stop_motor("cleanup")
        writer.close()
        bus.shutdown()

    stats = _compute_noise_stats(jsonl_path, args.lead_gain)
    summary = {
        "duration_s": args.duration_s,
        "hold_velocity_rad_s": args.hold_velocity_rad_s,
        "command_hz": args.command_hz,
        "sample_hz": args.sample_hz,
        "lead_gain_used_for_projection": args.lead_gain,
        "start_rad": start_rad,
        "start_rad_note": start_rad_note,
        "motor_replies_seen": motor_replies_seen,
        "noise_stats": stats,
        "drum_radius_m": DRUM_RADIUS_M,
    }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")
    _print_noise_stats(stats)

    return 0


def _compute_noise_stats(jsonl_path: Path, lead_gain: float) -> dict:
    """Two views of the same log: the raw sampled series (mostly repeats between fresh
    replies), and the per-tick series (one value per distinct fresh reply -- what a real
    compensator running at command_hz actually sees). The per-tick delta is what LeadServo's
    zero amplifies; the projection multiplies its std by lead_gain as a rough upper-bound
    estimate of the resulting correction noise (the true gain varies with frequency, reaching
    lead_gain only well above the pole -- see ct.control.servo.lead_response)."""
    import numpy as np

    records = load_jsonl(jsonl_path)
    if not records:
        return {"error": "no records"}
    cols = to_arrays(records, ["motor_position_mm"])
    raw_mm = cols["motor_position_mm"]
    raw_mm = raw_mm[np.isfinite(raw_mm)]
    if raw_mm.size < 2:
        return {"error": "fewer than 2 valid position samples"}

    per_tick_mm = [raw_mm[0]]
    for v in raw_mm[1:]:
        if v != per_tick_mm[-1]:
            per_tick_mm.append(v)
    per_tick_mm = np.array(per_tick_mm)
    deltas_mm = np.diff(per_tick_mm)

    return {
        "raw_samples": int(raw_mm.size),
        "raw_std_mm": float(np.std(raw_mm)),
        "raw_ptp_mm": float(np.ptp(raw_mm)),
        "per_tick_samples": int(per_tick_mm.size),
        "per_tick_std_mm": float(np.std(per_tick_mm)),
        "per_tick_ptp_mm": float(np.ptp(per_tick_mm)),
        "tick_to_tick_delta_std_mm": float(np.std(deltas_mm)) if deltas_mm.size else None,
        "tick_to_tick_delta_max_abs_mm": float(np.max(np.abs(deltas_mm))) if deltas_mm.size else None,
        "projected_correction_noise_std_mm": (
            float(np.std(deltas_mm) * lead_gain) if deltas_mm.size else None
        ),
        "projection_note": (
            "tick_to_tick_delta_std_mm * lead_gain -- a rough upper bound, since the "
            "compensator's actual gain at any given frequency is between the DC gain "
            "(gain*zero/pole) and lead_gain, reaching lead_gain only well above the pole. "
            "See ct.control.servo.lead_response for the real frequency-dependent value."
        ),
    }


def _print_noise_stats(stats: dict) -> None:
    if "error" in stats:
        print(f"\ncould not compute noise stats: {stats['error']}")
        return
    print(f"\nposition noise floor (at rest, no compensator):")
    print(f"  raw log:      std={stats['raw_std_mm']:.4f}mm  ptp={stats['raw_ptp_mm']:.4f}mm  "
          f"({stats['raw_samples']} samples)")
    print(f"  per-tick:     std={stats['per_tick_std_mm']:.4f}mm  ptp={stats['per_tick_ptp_mm']:.4f}mm  "
          f"({stats['per_tick_samples']} fresh replies)")
    if stats["tick_to_tick_delta_std_mm"] is not None:
        print(f"  tick-to-tick delta: std={stats['tick_to_tick_delta_std_mm']:.4f}mm  "
              f"max={stats['tick_to_tick_delta_max_abs_mm']:.4f}mm")
        print(f"\n  projected correction noise through the current compensator gain: "
              f"~{stats['projected_correction_noise_std_mm']:.4f}mm std "
              f"({stats['projection_note']})")


if __name__ == "__main__":
    raise SystemExit(main())
