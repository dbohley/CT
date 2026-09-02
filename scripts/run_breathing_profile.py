#!/usr/bin/env python3
"""Play a recorded breathing-position CSV on the phantom drive motor (CubeMars AK60-6 V3.0,
servo mode), selecting which recording to play from breathe_profiles/.

Adapted from scripts_reference/breathe_motor_can.py (Windows, COM port, velocity/ERPM
control), but that script's motor-mechanism constants (MOTOR_MM_PER_REV, GEAR_RATIO,
MOTOR_POLE_PAIRS -- a leadscrew behind a 6:1 gearbox) are unconfirmed for this rig and
untested. scripts/test_phantom_motor.py already proved a different, working mechanism on
real hardware -- a direct capstan drum (DRUM_RADIUS_M below) driven by position-in-degrees
commands, not velocity/ERPM. This script reuses that proven approach: interpolate the
CSV's own position at each control tick and command it directly, rather than
differentiating to a velocity and open-loop integrating (which drifts).

A CSV's ``y_mm`` column is an absolute reading from whatever sensor recorded it, not a
motor-relative distance -- e.g. breathing_profile_1.csv sits around 142mm, which would be a
wildly unsafe travel commanded literally. Position is therefore rebased to start at 0mm
relative to its own first sample: playback starts wherever the phantom motor already is.

**Anti-jolt note:** ``CubeMarsServo`` position is absolute in the motor's own persistent
frame, not "wherever it currently is" -- there is no homing procedure for this axis (see
test_phantom_motor.py's ``--zero`` flag, which exists for the same reason). Commanding the
profile's rebased 0mm start naively (i.e. assuming the motor is already at its internal
zero) caused a fast snap on startup whenever the motor's actual position differed from
that assumption. Fixed by listening for the motor's own autonomous status broadcast
(``codec.parse()`` on its status-1 frame -- these motors broadcast unprompted, per session
004) to learn the real current position before ramping, and ramping back to that same real
position at the end rather than to an arbitrary 0.

**Telemetry**: logs ``t``/``elapsed``/``commanded_mm``/``measured_mm``/``error_mm`` (during
actual profile playback only, not the ramp in/out) to
``outputs/breathing_profile/<timestamp>/samples.jsonl`` via the same
``ct.rt.telemetry.TelemetryWriter`` convention every other bench script uses -- the same five
field names ``ct.phantom.driver.PhantomDriver.step()`` already writes, so this is the existing
record shape rather than a new one. ``t`` is the
**raw, un-rebased** ``time.monotonic()`` reading -- matching
``ct.phantom.driver.PhantomDriver.step()``'s own record shape -- which is what makes it
directly comparable, with no shared clock needed beyond both processes running on the same
host, against another same-host script's own ``t`` column via
``ct.phantom.driver.compare_logs``/``ct-compare``. Rebasing ``t`` to this run's own start
(an earlier version of this script did) breaks that: two processes starting at different
real moments would then have incomparable "zero" points, and `compare_logs` would silently
report a wrong lag rather than erroring. ``elapsed`` (``t`` minus this run's own start) is
kept as a second field purely for human-readable plotting. The field names (``t``,
``commanded_mm``) are exactly what ``compare_logs`` expects from the "phantom" side.

**Ground truth**: ``measured_mm`` is what the motor's own status-1 broadcast says it actually
did, converted through the same capstan constants as the command -- the honest answer to "what
was the phantom doing", as opposed to ``commanded_mm``'s "what was it asked to do". Comparing
the two separates the phantom's own tracking error from the sensing chain's. It is recorded by
draining RX each tick, which this loop previously never did: every status frame was discarded
and the adapter's receive buffer grew unbounded for the length of the run. How densely the
AK60-6 broadcasts *while being commanded at 100Hz* is unmeasured (session 004 only established
that it broadcasts when idle), so ``samples.jsonl`` degrades to ``measured_mm: null`` and the
run's ``summary.json`` reports ``replies_seen`` and the observed feedback rate. Check that
number before treating ``measured_mm`` as ground truth rather than a sparse hint.

    python scripts/run_breathing_profile.py --list
    python scripts/run_breathing_profile.py --profile breathing_profile_1 --dry-run
    python scripts/run_breathing_profile.py --profile breathing_profile_1
    python scripts/run_breathing_profile.py --profile breathing_profile_1 --loop
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import can
import numpy as np

from ct.cli._common import save_json
from ct.hw.motors.cubemars_servo import CubeMarsServo
from ct.rt.telemetry import TelemetryWriter

# ---------------- Motor-specific config (mirrors scripts/test_phantom_motor.py -- keep in sync) ----------------
CAN_CHANNEL = "/dev/cu.usbmodem20563976534B1"  # phantom motor adapter -- confirm with `ls /dev/cu.*`
CAN_INTERFACE = "slcan"
BITRATE = 1_000_000  # assumed, carried over from test_phantom_motor.py -- not independently confirmed

MOTOR_NODE_ID = 1  # phantom motor's labeled CAN node ID
DRUM_RADIUS_M = 0.013  # measured capstan/drum radius, same mechanism as test_phantom_motor.py
DIRECTION_SIGN = -1  # found empirically: +rad moved the wrong way, so flipped
# ------------------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
PROFILES_DIR = REPO_ROOT / "breathe_profiles"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "breathing_profile"

DEFAULT_CONTROL_HZ = 100.0  # matches this project's standard sensor/control rate (rig_bench.yaml, fs)
DEFAULT_RAMP_HZ = 20.0  # update rate while ramping into/out of the profile, not the CSV's own speed
DEFAULT_RAMP_VELOCITY_RAD_S = 0.3  # ramp-in/out speed only -- playback speed comes from the CSV itself
DEFAULT_MAX_TRAVEL_MM = 6.0  # safety clamp -- raised from 5.0 to admit real subject profiles
# (emma 5.01mm, derek 4.21mm, jake 3.84mm, junrong 4.89mm, patient1 5.84mm all fit; moira
# 7.88mm and sara 9.03mm still need an explicit --max-travel-mm)
POSITION_READ_TIMEOUT_S = 2.0  # how long to wait for the motor's own status broadcast before giving up
MAX_RX_DRAIN_PER_TICK = 32  # bounded so a chatty bus can never stall the playback tick


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def rad_to_mm(rad: float) -> float:
    """Capstan geometry, in exactly one direction. Inverse of the target_rad computation."""
    return rad / DIRECTION_SIGN * DRUM_RADIUS_M * 1000.0


def drain_status(bus: can.BusABC, codec: CubeMarsServo, node_id: int) -> tuple[float | None, int]:
    """Latest status-1 position [rad] from this motor, and how many replies were drained.

    Non-blocking (``timeout=0.0``) and bounded: the playback tick has a 100Hz deadline to
    meet and must never wait on the bus. Returns ``(None, 0)`` when nothing was waiting.
    """
    latest_rad: float | None = None
    replies = 0
    for _ in range(MAX_RX_DRAIN_PER_TICK):
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        parsed = codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == node_id:
            latest_rad = parsed["position"] * math.pi / 180.0
            replies += 1
    return latest_rad, replies


def to_message(frame: tuple[int, bytes, bool]) -> can.Message:
    can_id, data, extended = frame
    return can.Message(arbitration_id=can_id, data=data, is_extended_id=extended)


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def list_profiles() -> list[Path]:
    return sorted(PROFILES_DIR.glob("*.csv"))


def resolve_profile(name: str) -> Path:
    """Bare name resolves against breathe_profiles/ (with or without .csv); a literal path
    is used as-is -- same idiom this project's other scripts use for --config."""
    for candidate in (Path(name), PROFILES_DIR / name, PROFILES_DIR / f"{name}.csv"):
        if candidate.exists():
            return candidate
    available = ", ".join(p.stem for p in list_profiles()) or "(none found)"
    raise FileNotFoundError(f"no profile matching {name!r}. Available in {PROFILES_DIR}: {available}")


def load_profile(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load a `time_s,y_mm` CSV. Time is shifted to start at 0s; position is rebased to
    start at 0mm relative to its own first sample -- see module docstring for why."""
    times: list[float] = []
    positions: list[float] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "time_s" not in reader.fieldnames or "y_mm" not in reader.fieldnames:
            raise ValueError(f"{path}: CSV must contain columns named exactly 'time_s' and 'y_mm'.")
        for row in reader:
            times.append(float(row["time_s"]))
            positions.append(float(row["y_mm"]))

    if len(times) < 2:
        raise ValueError(f"{path}: profile must contain at least 2 points.")

    times_arr = np.asarray(times, dtype=float)
    positions_arr = np.asarray(positions, dtype=float)
    order = np.argsort(times_arr)
    times_arr, positions_arr = times_arr[order], positions_arr[order]
    if np.any(np.diff(times_arr) <= 0):
        raise ValueError(f"{path}: time_s values must be strictly increasing after sorting.")

    times_arr = times_arr - times_arr[0]
    positions_arr = positions_arr - positions_arr[0]
    return times_arr, positions_arr


def read_current_position_rad(
    bus: can.BusABC, codec: CubeMarsServo, node_id: int, timeout_s: float = POSITION_READ_TIMEOUT_S,
) -> float | None:
    """Listen for the motor's own autonomous status-1 broadcast to learn where it
    actually is, rather than assuming it's sitting at the codec's internal zero --
    session 004 found these motors broadcast status frames unprompted, on their own,
    with zero commands sent. Returns None if nothing arrives within timeout_s."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            continue
        parsed = codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == node_id:
            return parsed["position"] * math.pi / 180.0


def ramp_to(
    bus: can.BusABC, codec: CubeMarsServo, node_id: int, start_rad: float, target_rad: float,
    velocity_rad_s: float = DEFAULT_RAMP_VELOCITY_RAD_S, hz: float = DEFAULT_RAMP_HZ,
) -> None:
    """Step the position setpoint from start_rad to target_rad at `hz`, easing into it
    rather than a single jump -- same approach as test_phantom_motor.py's ramp_to."""
    distance = target_rad - start_rad
    duration_s = abs(distance) / max(velocity_rad_s, 1e-6)
    steps = max(1, int(duration_s * hz))
    for i in range(1, steps + 1):
        pos_rad = start_rad + distance * (i / steps)
        bus.send(to_message(codec.command(node_id, position=rad_to_deg(pos_rad), velocity=0.0)))
        time.sleep(1.0 / hz)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", help="bare name (resolved against breathe_profiles/) or a path to a CSV")
    parser.add_argument("--list", action="store_true", help="list available profiles in breathe_profiles/ and exit")
    parser.add_argument("--loop", action="store_true", help="repeat the profile until Ctrl+C instead of once")
    parser.add_argument("--control-hz", type=float, default=DEFAULT_CONTROL_HZ, help="position-command update rate")
    parser.add_argument("--ramp-velocity", type=float, default=DEFAULT_RAMP_VELOCITY_RAD_S,
                         help="rad/s used only to ease into/out of the profile, not during playback")
    parser.add_argument("--max-travel-mm", type=float, default=DEFAULT_MAX_TRAVEL_MM,
                         help="refuse to run if the profile's rebased peak-to-peak travel exceeds this")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory for samples.jsonl; default outputs/breathing_profile/<timestamp>")
    parser.add_argument("--yes", action="store_true",
                         help="skip the Proceed? confirmation -- for launching this non-interactively "
                              "(e.g. scripts/run_approach_and_seat.py starting this as a subprocess), "
                              "not for interactive use")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    args = parser.parse_args()

    if args.list or not args.profile:
        profiles = list_profiles()
        print(f"available profiles in {PROFILES_DIR}:")
        for p in profiles:
            print(f"  {p.stem}")
        if not profiles:
            print("  (none found)")
        return 0 if args.list else 1

    try:
        path = resolve_profile(args.profile)
        times, positions_mm = load_profile(path)
    except (FileNotFoundError, ValueError) as e:
        print(f"error: {e}")
        return 1

    travel_mm = float(positions_mm.max() - positions_mm.min())
    if travel_mm > args.max_travel_mm:
        print(f"REFUSING: {path.name}'s rebased peak-to-peak travel is {travel_mm:.2f}mm, exceeding "
              f"--max-travel-mm {args.max_travel_mm:.2f}mm. Pass a higher --max-travel-mm if this is "
              f"genuinely intended.")
        return 1

    target_rad = DIRECTION_SIGN * (positions_mm / 1000.0 / DRUM_RADIUS_M)

    print(f"loaded {path.name}: {len(times)} points, {times[-1]:.2f}s, "
          f"travel {positions_mm.min():.2f} to {positions_mm.max():.2f}mm (relative to start)")
    print(f"phantom motor (AK60-6 V3.0, servo mode), node id {MOTOR_NODE_ID}, "
          f"{'looping until Ctrl+C' if args.loop else 'single pass'}, {args.control_hz:.0f}Hz commands")

    codec = CubeMarsServo()

    if args.dry_run:
        print("\n--dry-run: not opening the bus.")
        print("  enable: ", to_message(codec.enable(MOTOR_NODE_ID)))
        print("  first sample:", to_message(codec.command(MOTOR_NODE_ID, position=rad_to_deg(target_rad[0]), velocity=0.0)))
        print("  last sample: ", to_message(codec.command(MOTOR_NODE_ID, position=rad_to_deg(target_rad[-1]), velocity=0.0)))
        print("  disable:", to_message(codec.disable(MOTOR_NODE_ID)))
        return 0

    print(f"\nThis will move the phantom motor through {path.name} "
          f"({'repeating until Ctrl+C' if args.loop else 'once'}). Watch the axis.")
    if not args.yes and not confirm("Proceed?"):
        print("aborted.")
        return 1

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    writer = TelemetryWriter(jsonl_path)
    dt = 1.0 / args.control_hz
    last_rad = target_rad[0]
    measured_rad: float | None = None
    replies_seen = 0
    ticks = 0
    loops = 0
    playback_s = 0.0
    commanded_seen: list[float] = []
    measured_seen: list[float] = []
    t0 = time.monotonic()
    try:
        print("enabling (zero-current wake-up frame)...")
        bus.send(to_message(codec.enable(MOTOR_NODE_ID)))
        time.sleep(0.5)

        print("reading current position from the motor's status broadcast...")
        initial_rad = read_current_position_rad(bus, codec, MOTOR_NODE_ID)
        if initial_rad is None:
            print("warning: no status frame seen from the motor -- assuming it's at 0 rad. "
                  "If it isn't, this first move may jolt. Check the motor is powered and on this bus.")
            initial_rad = 0.0
        else:
            print(f"motor is currently at {initial_rad:.4f} rad ({rad_to_deg(initial_rad):.2f} deg)")

        print("ramping to the profile's start...")
        ramp_to(bus, codec, MOTOR_NODE_ID, initial_rad, target_rad[0], velocity_rad_s=args.ramp_velocity)

        print("playing profile (Ctrl+C to stop)...")
        playback_t0 = time.monotonic()
        try:
            while True:
                start = time.monotonic()
                while True:
                    elapsed = time.monotonic() - start
                    if elapsed >= times[-1]:
                        last_rad = target_rad[-1]
                        break
                    # Drain before commanding, so measured_mm is the freshest reading that
                    # could possibly precede this tick's command rather than trailing it.
                    latest_rad, replies = drain_status(bus, codec, MOTOR_NODE_ID)
                    if latest_rad is not None:
                        measured_rad = latest_rad
                    replies_seen += replies

                    rad = float(np.interp(elapsed, times, target_rad))
                    bus.send(to_message(codec.command(MOTOR_NODE_ID, position=rad_to_deg(rad), velocity=0.0)))
                    last_rad = rad
                    commanded_mm = rad_to_mm(rad)
                    measured_mm = None if measured_rad is None else rad_to_mm(measured_rad)
                    now = time.monotonic()
                    # "t" is the raw, un-rebased time.monotonic() reading -- what
                    # ct.phantom.driver.compare_logs actually needs to align this log against
                    # another process's on the same host (see that module's docstring). "elapsed"
                    # is rebased to this run's own start, kept only for human-readable plotting.
                    writer.write({
                        "t": now,
                        "elapsed": now - t0,
                        "commanded_mm": commanded_mm,
                        "measured_mm": measured_mm,
                        "error_mm": None if measured_mm is None else measured_mm - commanded_mm,
                    })
                    ticks += 1
                    if measured_mm is not None:
                        commanded_seen.append(commanded_mm)
                        measured_seen.append(measured_mm)
                    next_tick = start + elapsed + dt
                    sleep_s = next_tick - time.monotonic()
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                loops += 1
                if not args.loop:
                    break
        except KeyboardInterrupt:
            print("\nstopped by Ctrl+C.")
        playback_s = time.monotonic() - playback_t0

        print("ramping back to where the motor started...")
        ramp_to(bus, codec, MOTOR_NODE_ID, last_rad, initial_rad, velocity_rad_s=args.ramp_velocity)

        print("disabling (zero current, motor coasts)...")
        bus.send(to_message(codec.disable(MOTOR_NODE_ID)))
    finally:
        writer.close()
        bus.shutdown()

    feedback_hz = replies_seen / playback_s if playback_s > 0 else 0.0
    summary = {
        "profile": path.name,
        "profile_points": len(times),
        "profile_duration_s": float(times[-1]),
        "profile_travel_mm": travel_mm,
        "looped": args.loop,
        "loops_completed": loops,
        "playback_s": playback_s,
        "control_hz": args.control_hz,
        "ticks": ticks,
        "replies_seen": replies_seen,
        "feedback_hz": feedback_hz,
        "ticks_with_measurement": len(measured_seen),
        "commanded_travel_mm": (max(commanded_seen) - min(commanded_seen)) if commanded_seen else None,
        "measured_travel_mm": (max(measured_seen) - min(measured_seen)) if measured_seen else None,
        "feedback_note": (
            "replies_seen counts decoded servo-mode status-1 frames (node_id | (0x29 << 8)) "
            "drained during playback; feedback_hz is that over playback_s. This is the number "
            "that decides whether measured_mm is real ground truth or a sparse hint -- how "
            "densely this motor broadcasts while being commanded at control_hz was unmeasured "
            "before this script recorded it. A feedback_hz well below control_hz means "
            "measured_mm is a zero-order hold between real samples."
        ),
        "measured_travel_note": (
            "measured_travel_mm below commanded_travel_mm is the phantom's own tracking "
            "shortfall, entirely separate from anything the tactile sensor does. Keep the two "
            "apart: ct-compare --truth measured_mm scores the sensing chain alone, "
            "--truth commanded_mm scores phantom and sensing together."
        ),
    }
    save_json(summary, out_dir / "summary.json")

    print(f"saved: {jsonl_path}")
    print(f"saved: {out_dir / 'summary.json'}")
    if replies_seen:
        print(f"motor feedback: {replies_seen} status frame(s) over {playback_s:.1f}s "
              f"({feedback_hz:.1f}Hz) -- measured_mm is real on {len(measured_seen)}/{ticks} ticks")
    else:
        print("motor feedback: NONE seen during playback -- measured_mm is null throughout, so "
              "only commanded_mm is available as ground truth. Check that the motor is powered "
              "and on this bus (scripts/listen_phantom_motor.py sees its broadcast when idle).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
