#!/usr/bin/env python3
"""Drive the real needle motor through the same breathing-rate reference twice -- once commanded
directly (uncompensated) and once through the real `ct.control.servo.LeadServo` closed loop
(compensated) -- so the two real outputs can be compared against each other and against the
input, the hardware counterpart of `scripts/simulate_needle_lead_tracking.py`'s pure simulation.

**Reference waveform is unchanged from `scripts/run_needle_sine_tracking.py`.** At its starting
position the needle is already at the end of its capstan wrap and must never be commanded
backward past it, so both phases use the same one-directional raised cosine
(`target_rad_at()`), run for an integer number of cycles so each phase starts and ends the
motion at the needle's own start position.

**Why the compensator needs its own tick rate, not the simulation's.** The GL-II motor only
ACKs a position reply once per command sent (confirmed 2026-08-27, not a continuous broadcast),
so real position feedback can only update as often as commands go out. `LeadServo`'s Tustin
discretization is derived from `Ts` at construction time, so it is built here with
`Ts = 1/command_hz` -- the real achievable tick rate on this hardware -- not the 200Hz the pure
simulation used with no such constraint.

**A new safety clamp, because this is genuinely new territory.** The raised cosine's own
"never command past start_rad" guarantee holds only for the bare reference. Once a correction
from the compensator is added on top (`command = reference + correction`, matching
`ct.control.states.insert._hold_standoff()`'s real architecture), the total could in principle
push past either end. The compensated phase clamps every commanded position into
`[start_rad, far_end_rad]` (from `--max-amplitude-mm`, same cap `run_needle_sine_tracking.py`
already applies to the bare reference) before sending, and counts how often that clamp
triggers -- `clamped_commands` in the summary. `--correction-limit-mm` additionally bounds the
compensator's own output magnitude via `LeadServo.update`'s `limit` argument, the same mechanism
`simulate_needle_lead_tracking.py` exposes as `--velocity-limit-mm-s`.

**`--correction-rate-limit-mm-s` bounds how fast that output may change, separately from how
large it gets.** Added 2026-09-07 after a real run showed the correction repeatedly slamming
`--correction-limit-mm` from real tracking errors of only a few tenths of a millimetre -- not a
bug, but a direct consequence of this lead network's ~12.7x gain at frequencies above its pole
(`lead.gain`), which amplifies any real, ordinary tracking imperfection into a comparatively
huge correction demand in one tick. The rate limit forces that correction to ramp in over
several ticks instead of arriving all at once, which is what actually stops the resulting
overshoot-flip-overshoot oscillation; the magnitude limit alone does not, since a full-magnitude
correction is still large enough to overshoot on its own.

    python scripts/run_needle_lead_tracking_live.py --dry-run
    python scripts/run_needle_lead_tracking_live.py --amplitude-mm 3 --cycles 2
    python scripts/run_needle_lead_tracking_live.py --frequency-hz 0.25 --amplitude-mm 5 --cycles 8
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from pathlib import Path

import can

from ct.cli._common import save_json
from ct.control.servo import LeadServo
from ct.hw.config import AxisServoConfig, LeadCompensator, PlantModel
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.rt.telemetry import TelemetryWriter

# ---------------- Needle motor config (mirrors scripts/run_needle_sine_tracking.py -- keep in sync) --------
CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"  # confirm with `ls /dev/cu.*` -- may differ
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000

MOTOR_NODE_ID = 2  # needle motor's labeled CAN node ID
POSITION_VELOCITY_MODE = 1
DRUM_RADIUS_M = 0.018  # 3.6cm capstan diameter / 2, from the needle's known capstan drive -- not calipers-measured
DIRECTION_SIGN = -1  # found empirically: +rad retracted (moved away from phantom), so flipped

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
SET_ZERO = bytes([0xFF] * 7 + [0xFE])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II
# --------------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "needle_lead_live_tracking"

# Plant/lead defaults match configs/rig_bench.yaml's servo.needle section (session 019) and
# scripts/simulate_needle_lead_tracking.py's own CLI defaults, so a live run and a simulated run
# use the same model unless deliberately overridden.
DEFAULT_PLANT_K = 0.92
DEFAULT_PLANT_WN = 27.0
DEFAULT_PLANT_ZETA = 0.35
DEFAULT_LEAD_ZERO = 1.0
DEFAULT_LEAD_POLE = 5.0
DEFAULT_LEAD_GAIN = 12.74

DEFAULT_AMPLITUDE_MM = 5.0
DEFAULT_FREQUENCY_HZ = 0.25  # ~15 breaths/min, breathing-rate scale
DEFAULT_CYCLES = 5
DEFAULT_VELOCITY_RAD_S = 0.3  # matches run_needle_sine_tracking.py
DEFAULT_COMMAND_HZ = 20.0
DEFAULT_SAMPLE_HZ = 100.0
DEFAULT_MAX_AMPLITUDE_MM = 20.0
DEFAULT_CORRECTION_LIMIT_MM = 2.0  # conservative cap on the compensator's own output
DEFAULT_CORRECTION_RATE_LIMIT_MM_S = 5.0  # see ct.control.servo.LeadServo.update's rate_limit_mm_s;
# at the default 20Hz command rate this reaches the 2.0mm limit over ~8 ticks (0.4s) rather than
# in one tick -- added 2026-09-07 after a real run showed the correction repeatedly slamming its
# magnitude limit from real tracking errors of only 0.1-0.4mm (this lead network's gain is ~12.7x
# at frequencies above its pole, so that amplification alone explains it -- see docs/sessions).
INITIAL_REPLY_WAIT_S = 0.5
SETTLE_MARGIN_S = 3.0
DWELL_BETWEEN_PHASES_S = 1.0


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def target_rad_at(t: float, start_rad: float, amplitude_mm: float, omega: float) -> float:
    """One-directional raised cosine -- identical to run_needle_sine_tracking.py's version.
    Zero extension at t=0, T/2, T, ... so an integer number of cycles starts and ends at
    start_rad with zero commanded velocity."""
    extend_frac = 0.5 * (1.0 - math.cos(omega * t))
    travel_m = (amplitude_mm * extend_frac) / 1000.0
    return start_rad + DIRECTION_SIGN * (travel_m / DRUM_RADIUS_M)


def mm_to_rad(mm: float) -> float:
    return abs(DIRECTION_SIGN) * (mm / 1000.0) / DRUM_RADIUS_M


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--amplitude-mm", type=float, default=DEFAULT_AMPLITUDE_MM, dest="amplitude_mm")
    parser.add_argument("--frequency-hz", type=float, default=DEFAULT_FREQUENCY_HZ, dest="frequency_hz")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--velocity-rad-s", type=float, default=DEFAULT_VELOCITY_RAD_S, dest="velocity_rad_s",
                         help="velocity limit field sent with every command -- the position "
                              "field alone carries the waveform, same as run_needle_sine_tracking.py")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ, dest="command_hz",
                         help="rate to resend the current target -- also the compensator's "
                              "real tick rate (Ts = 1/command_hz), since fresh position "
                              "feedback only arrives once per command sent")
    parser.add_argument("--sample-hz", type=float, default=DEFAULT_SAMPLE_HZ, dest="sample_hz")
    parser.add_argument("--max-amplitude-mm", type=float, default=DEFAULT_MAX_AMPLITUDE_MM,
                         dest="max_amplitude_mm",
                         help="hard safety cap on the reference AND (for the compensated phase) "
                              "the total commanded position after the correction is added")
    parser.add_argument("--correction-limit-mm", type=float, default=DEFAULT_CORRECTION_LIMIT_MM,
                         dest="correction_limit_mm",
                         help="cap on the compensator's own output magnitude, via "
                              "LeadServo.update(limit=...)")
    parser.add_argument("--correction-rate-limit-mm-s", type=float,
                         default=DEFAULT_CORRECTION_RATE_LIMIT_MM_S, dest="correction_rate_limit_mm_s",
                         help="cap on how fast the correction may change per tick, via "
                              "LeadServo.update(rate_limit_mm_s=...) -- set very large (e.g. 1e6) "
                              "to effectively disable and see the un-rate-limited behavior")
    parser.add_argument("--plant-k", type=float, default=DEFAULT_PLANT_K, dest="plant_k")
    parser.add_argument("--plant-wn", type=float, default=DEFAULT_PLANT_WN, dest="plant_wn")
    parser.add_argument("--plant-zeta", type=float, default=DEFAULT_PLANT_ZETA, dest="plant_zeta")
    parser.add_argument("--lead-zero", type=float, default=DEFAULT_LEAD_ZERO, dest="lead_zero")
    parser.add_argument("--lead-pole", type=float, default=DEFAULT_LEAD_POLE, dest="lead_pole")
    parser.add_argument("--lead-gain", type=float, default=DEFAULT_LEAD_GAIN, dest="lead_gain")
    parser.add_argument("--zero", action="store_true",
                         help="also set the current position as zero before starting -- only do "
                              "this once you've confirmed the motor is at the capstan's true limit")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/needle_lead_live_tracking/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.amplitude_mm > args.max_amplitude_mm:
        print(f"error: --amplitude-mm {args.amplitude_mm} exceeds --max-amplitude-mm "
              f"{args.max_amplitude_mm}. Raise --max-amplitude-mm explicitly if you really want "
              f"this -- it exists because DRUM_RADIUS_M for the needle is an estimate, not a "
              f"calipers measurement.")
        return 1

    omega = 2.0 * math.pi * args.frequency_hz
    period_s = 1.0 / args.frequency_hz
    duration_s = args.cycles * period_s
    max_travel_rad = mm_to_rad(args.amplitude_mm)
    far_end_cap_rad = mm_to_rad(args.max_amplitude_mm)  # magnitude; sign applied relative to start_rad
    correction_limit_rad = mm_to_rad(args.correction_limit_mm)
    # mm_to_rad is a pure scale factor (rad = mm/1000/DRUM_RADIUS_M), so it converts a rate
    # (mm/s -> rad/s) exactly the same way it converts a position magnitude.
    correction_rate_limit_rad_s = mm_to_rad(args.correction_rate_limit_mm_s)

    plant = PlantModel(K=args.plant_k, wn=args.plant_wn, zeta=args.plant_zeta)
    lead = LeadCompensator(zero=args.lead_zero, pole=args.lead_pole, gain=args.lead_gain)
    servo_config = AxisServoConfig(plant=plant, lead=lead)
    command_period_s = 1.0 / max(args.command_hz, 1e-6)

    print(f"needle motor (GL40II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"waveform: raised-cosine, amplitude {args.amplitude_mm:.1f}mm ({max_travel_rad:.4f}rad "
          f"peak extension), frequency {args.frequency_hz:.3f}Hz, {args.cycles} cycle(s) "
          f"-> {duration_s:.1f}s per phase, 2 phases (uncompensated, compensated)")
    print(f"resending the current target at {args.command_hz:.0f}Hz; sampling/logging at "
          f"{args.sample_hz:.0f}Hz")
    print(f"plant: K={plant.K}, wn={plant.wn}, zeta={plant.zeta}  "
          f"lead: zero={lead.zero}, pole={lead.pole}, gain={lead.gain}  "
          f"(defaults from configs/rig_bench.yaml, session 019)")
    print(f"compensator tick rate Ts = 1/command_hz = {command_period_s * 1000:.1f}ms -- real "
          f"position feedback only arrives once per command sent, not continuously")
    print(f"correction limit {args.correction_limit_mm:.2f}mm, rate limit "
          f"{args.correction_rate_limit_mm_s:.2f}mm/s (reaches the full limit over "
          f"{args.correction_limit_mm / max(args.correction_rate_limit_mm_s, 1e-9):.2f}s) -- "
          f"this lead network's gain is ~{lead.gain:.1f}x at frequencies above its pole, so a "
          f"real tracking error of a few tenths of a mm can otherwise hit the magnitude limit "
          f"in a single tick and oscillate")
    print("both phases start and end each cycle at the needle's own current position and only "
          "move in the extend direction -- neither ever commands the needle backward past "
          "where it started, and the compensated phase additionally clamps the corrected "
          f"command into [start_rad, +{args.max_amplitude_mm:.1f}mm].")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        if args.zero:
            print(" ", universal_command(MOTOR_NODE_ID, SET_ZERO))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity_rad_s), " <- initial, to read start_rad")
        print("\n  -- uncompensated phase --")
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            t = frac * period_s
            print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad_at(t, 0.0, args.amplitude_mm, omega),
                                            args.velocity_rad_s), f"  (t={t:.2f}s of first cycle, start_rad=0.0 assumed)")
        print(f"  ... (resent at {args.command_hz:.0f}Hz for {duration_s:.1f}s total) ...")
        print("\n  -- dwell --\n  -- compensated phase (reference + LeadServo correction, clamped) --")
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            t = frac * period_s
            print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad_at(t, 0.0, args.amplitude_mm, omega),
                                            args.velocity_rad_s),
                  f"  (t={t:.2f}s, target shown -- real correction depends on live feedback)")
        print(f"  ... (resent at {args.command_hz:.0f}Hz for {duration_s:.1f}s total) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE))
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))

    print(f"\nThis will move the needle motor continuously for ~{2 * duration_s:.1f}s total "
          f"(two {duration_s:.1f}s phases), extending up to {args.amplitude_mm:.1f}mm and back, "
          f"{args.cycles} time(s) each. Watch it closely. Ctrl+C stops and de-energizes immediately.")
    if not confirm("Proceed?"):
        print("aborted.")
        return 1

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
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
    start_rad_note = "assumed 0.0 -- no real reply arrived before starting (e.g. no hardware attached)"
    phase_results: dict[str, dict] = {}

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
            bus.send(build_pos_vel_frame(MOTOR_NODE_ID, 0.0, args.velocity_rad_s))
            motion_commanded = True
            reply = read_reply(INITIAL_REPLY_WAIT_S)
            if reply is not None:
                start_rad = reply["position"]
                start_rad_note = "measured from the motor's first real reply"
            print(f"start_rad = {start_rad:.4f}rad ({start_rad_note})")

        far_end_rad = start_rad + DIRECTION_SIGN * far_end_cap_rad

        def run_phase(phase: str, jsonl_path: Path, use_compensator: bool) -> dict:
            nonlocal motion_commanded
            writer = TelemetryWriter(jsonl_path)
            servo = LeadServo(servo_config, Ts=command_period_s) if use_compensator else None
            if servo is not None:
                servo.reset(u0=0.0, y0=0.0)

            print(f"\nrunning {phase} phase for {duration_s:.1f}s "
                  "(Ctrl+C to stop early and de-energize)...")
            t0 = time.monotonic()
            sample_period_s = 1.0 / max(args.sample_hz, 1e-6)
            next_sample_at = 0.0
            next_command_at = 0.0
            safety_timeout_s = duration_s + SETTLE_MARGIN_S
            clamped_commands = 0
            phase_replies_start = motor_replies_seen

            # Held between command ticks so logging at sample_hz (faster than command_hz) does
            # not resample state that has not actually changed -- see the fix note below.
            command_rad = target_rad_at(0.0, start_rad, args.amplitude_mm, omega)
            correction_rad = 0.0
            error_rad = None
            clamped = False

            try:
                while True:
                    elapsed = time.monotonic() - t0
                    if elapsed >= safety_timeout_s:
                        print(f"\nreached safety timeout ({safety_timeout_s:.1f}s) -- stopping.")
                        break

                    reference_rad = target_rad_at(min(elapsed, duration_s), start_rad, args.amplitude_mm, omega)

                    read_reply(0.0)  # non-blocking

                    if elapsed >= next_command_at:
                        # LeadServo.update() must be called at the rate its Ts was built for
                        # (command_period_s), not on every ~1ms pass of this loop -- calling it
                        # ~50x faster than Ts runs the Tustin recursion far too fast and drives
                        # the correction straight to its saturation limit even for a tiny real
                        # error (confirmed on hardware 2026-09-07: 0.36mm error, 2mm correction).
                        clamped = False
                        if use_compensator and servo is not None:
                            measured_rad = last_motor_reply["position"] if last_motor_reply else start_rad
                            error_rad = reference_rad - measured_rad
                            correction_rad = servo.update(
                                error_rad, limit=correction_limit_rad,
                                rate_limit_mm_s=correction_rate_limit_rad_s,
                            )
                            command_rad = reference_rad + correction_rad
                            # Clamp the *total* command -- the raised cosine's own start_rad
                            # guarantee only covers the bare reference (see module docstring).
                            lo = min(start_rad, far_end_rad)
                            hi = max(start_rad, far_end_rad)
                            clamped_command_rad = min(max(command_rad, lo), hi)
                            if clamped_command_rad != command_rad:
                                clamped = True
                                clamped_commands += 1
                                command_rad = clamped_command_rad
                        else:
                            command_rad = reference_rad

                        bus.send(build_pos_vel_frame(MOTOR_NODE_ID, command_rad, args.velocity_rad_s))
                        motion_commanded = True
                        next_command_at = elapsed + command_period_s

                    if elapsed >= next_sample_at:
                        writer.write({
                            "t": elapsed,
                            "phase": phase,
                            "reference_rad": reference_rad,
                            "reference_mm": (reference_rad - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0,
                            "commanded_target_rad": command_rad,
                            "commanded_target_mm": (command_rad - start_rad) / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0,
                            "commanded_velocity_rad_s": args.velocity_rad_s,
                            "correction_rad": correction_rad,
                            "error_rad": error_rad,
                            "clamped": clamped,
                            "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                            "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                            "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                            "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                        })
                        next_sample_at = elapsed + sample_period_s

                    if elapsed >= duration_s:
                        print(f"{phase} phase complete at t={elapsed:.2f}s.")
                        break

                    time.sleep(0.001)
            finally:
                writer.close()

            return {
                "clamped_commands": clamped_commands,
                "replies_seen": motor_replies_seen - phase_replies_start,
                "servo_stats": servo.stats if servo is not None else None,
            }

        phase_results["uncompensated"] = run_phase(
            "uncompensated", out_dir / "uncompensated" / "samples.jsonl", use_compensator=False)
        time.sleep(DWELL_BETWEEN_PHASES_S)
        phase_results["compensated"] = run_phase(
            "compensated", out_dir / "compensated" / "samples.jsonl", use_compensator=True)

        print("\nboth phases complete.")
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        if motion_commanded:
            stop_motor("cleanup")
        bus.shutdown()

        summary = {
            "amplitude_mm": args.amplitude_mm,
            "frequency_hz": args.frequency_hz,
            "cycles": args.cycles,
            "duration_s_per_phase": duration_s,
            "start_rad": start_rad,
            "start_rad_note": start_rad_note,
            "velocity_rad_s": args.velocity_rad_s,
            "command_hz": args.command_hz,
            "sample_hz": args.sample_hz,
            "max_amplitude_mm": args.max_amplitude_mm,
            "correction_limit_mm": args.correction_limit_mm,
            "correction_rate_limit_mm_s": args.correction_rate_limit_mm_s,
            "servo_ts_s": command_period_s,
            "servo_ts_note": (
                "the compensator's Tustin discretization uses Ts = 1/command_hz, the real "
                "achievable tick rate, not the faster rate used in "
                "scripts/simulate_needle_lead_tracking.py's pure simulation -- real position "
                "feedback only arrives once per command sent (session 005)."
            ),
            "plant": {"K": plant.K, "wn": plant.wn, "zeta": plant.zeta},
            "lead": {"zero": lead.zero, "pole": lead.pole, "gain": lead.gain},
            "plant_lead_provenance": "configs/rig_bench.yaml servo.needle, session 019",
            "motor_replies_seen": motor_replies_seen,
            "phases": phase_results,
            "drum_radius_m": DRUM_RADIUS_M,
            "drum_radius_note": "estimate from the known capstan drive, not calipers-measured",
        }
        save_json(summary, out_dir / "summary.json")
        print(f"\nsaved: {out_dir}/summary.json")
        print(f"saved: {out_dir}/uncompensated/samples.jsonl")
        print(f"saved: {out_dir}/compensated/samples.jsonl")
        print(f"plot with: python scripts/plot_needle_lead_tracking_live.py --run {out_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
