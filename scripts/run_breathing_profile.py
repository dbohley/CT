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
wildly unsafe travel commanded literally. Position is therefore rebased about the profile's
own **mean**, and that mean is mapped onto wherever the phantom motor already is.

**Park the phantom anywhere and breathing happens there** -- established by *commanding* the
origin, not by reading one. ``CubeMarsServo`` position is absolute in the motor's own
persistent frame and there is no homing procedure for this axis, so at startup this script
sends ``codec.zero()`` (``CMD_SET_ORIGIN``, temporary) to make the present position 0, and
then commands the profile's offsets literally. ``scripts/test_phantom_motor.py --zero`` has
used exactly this on this motor on real hardware since session 004.

An earlier design instead *read* the position from the status broadcast and commanded
``initial_rad + profile_offset``. It still traversed on hardware, and worse, it could not be
diagnosed afterwards: subtracting the same read-back reference from both the commanded and
measured series shifts them together, so a wrong reference is invisible in the log. Commanding
the origin removes the dependency entirely -- there is no reference number in the motion
arithmetic to be wrong.

Three consequences worth knowing:

- **The readback is the safety check.** After zeroing, the motor must report ~0
  (``ZERO_READBACK_TOL_MM``). That is exactly the property playback depends on: commanding
  position 0 must mean "stay put". If ``SET_ORIGIN`` silently fails, the motor still reports
  its old position, and the run is refused rather than ramping to 0 and dragging the phantom
  across the bench.
- **The mean, not the first sample, is the datum.** Every recorded profile starts near its
  top, so referencing the first sample would leave the phantom sitting 0.5-1.1mm behind the
  parked position on average. Session 009 found a shift of exactly that size consumed the
  base's entire contact margin, so this is a real choice and not a cosmetic one.
- **The logged and motor frames now coincide**, since the motor's own zero is where the run
  started. ``measured_mm`` is the motor's reported position with no conversion, which is what
  makes a traverse visible in the log instead of hidden by it.

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
# After SET_ORIGIN the motor must report ~0, because that is precisely the property playback
# depends on: commanding position 0 has to mean "stay where you are". If the origin did not
# take, the readback shows the old position and the run is refused instead of lunging to it.
ZERO_READBACK_TOL_MM = 1.0
MAX_RX_DRAIN_PER_TICK = 32  # bounded so a chatty bus can never stall the playback tick
# Generous enough to clear the ~25 frames a 0.5s settle queues at this motor's ~51Hz, and
# still bounded so a runaway bus cannot spin here forever.
MAX_RX_FLUSH = 256


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
    """Load a `time_s,y_mm` CSV. Time is shifted to start at 0s; position is rebased about
    its own **mean**, so 0mm is the profile's average position -- see module docstring.

    Rebasing about the mean rather than the first sample is what makes "park the phantom
    where you want it" mean what an operator expects. Every recorded profile happens to
    start near its top, so first-sample rebasing would leave the phantom sitting 0.5-1.1mm
    *behind* the parked position on average (breathing_profile_1 -1.10, emma -1.02, moira
    -0.96, sara -0.52, derek -0.62). Session 009 found a shift of exactly that magnitude
    consumed the base's entire contact margin, so the datum is not a cosmetic choice.

    Caveat: for a profile longer than the run, the whole-file mean is not the mean of the
    slice that actually plays. Emma is 618s and a 180s run completes zero loops, and her
    30s rolling mean wanders 1.54mm across the recording, so the phantom can still drift
    off centre by that much. The whole-file mean is the only well-defined choice available
    here -- the script cannot know how long it will be left running.
    """
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
    positions_arr = positions_arr - positions_arr.mean()
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


def read_fresh_position_rad(
    bus: can.BusABC, codec: CubeMarsServo, node_id: int, timeout_s: float = POSITION_READ_TIMEOUT_S,
) -> float | None:
    """Position from a frame broadcast *after* this call, not one already queued.

    ``bus.recv()`` returns the OLDEST queued frame, and this motor broadcasts at ~51Hz, so
    after the 0.5s settle following SET_ORIGIN there are ~25 pre-command frames ahead of the
    one that matters. Reading naively therefore reports the position from *before* the origin
    was applied -- which is what made run 20260902-154504 refuse (pre-zero -0.2321 rad,
    "readback" -0.2286 rad) on a stale frame rather than on a real failure.

    Flushing what is queued first makes the reading unambiguously post-command.
    """
    for _ in range(MAX_RX_FLUSH):
        if bus.recv(timeout=0.0) is None:
            break
    return read_current_position_rad(bus, codec, node_id, timeout_s)


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

    # The profile's own offsets about its mean, and these are what get commanded verbatim:
    # SET_ORIGIN at startup makes the motor's parked position mean 0, so offset 0 is wherever
    # it is, and no reference number enters the arithmetic to be wrong.
    profile_rad = DIRECTION_SIGN * (positions_mm / 1000.0 / DRUM_RADIUS_M)

    print(f"loaded {path.name}: {len(times)} points, {times[-1]:.2f}s, "
          f"travel {positions_mm.min():+.2f} to {positions_mm.max():+.2f}mm (relative to its mean)")
    print(f"phantom motor (AK60-6 V3.0, servo mode), node id {MOTOR_NODE_ID}, "
          f"{'looping until Ctrl+C' if args.loop else 'single pass'}, {args.control_hz:.0f}Hz commands")
    print("playback is centred on wherever the motor already is -- it does not travel to a "
          "fixed origin first")

    codec = CubeMarsServo()

    if args.dry_run:
        print("\n--dry-run: not opening the bus. Positions below are the profile's own offsets "
              "about its mean, and they are sent literally -- SET_ORIGIN makes the motor's "
              "position at startup mean 0, so offset 0 is wherever it is parked.")
        print("  enable: ", to_message(codec.enable(MOTOR_NODE_ID)))
        print("  zero:   ", to_message(codec.zero(MOTOR_NODE_ID)), " <- 'here' becomes 0")
        print("  first sample (offset %+.4f rad):" % profile_rad[0],
              to_message(codec.command(MOTOR_NODE_ID, position=rad_to_deg(profile_rad[0]), velocity=0.0)))
        print("  last sample  (offset %+.4f rad):" % profile_rad[-1],
              to_message(codec.command(MOTOR_NODE_ID, position=rad_to_deg(profile_rad[-1]), velocity=0.0)))
        print("  disable:", to_message(codec.disable(MOTOR_NODE_ID)))
        return 0

    print(f"\nThis will move the phantom motor through {path.name} "
          f"({'repeating until Ctrl+C' if args.loop else 'once'}), centred on its CURRENT "
          f"position ({positions_mm.min():+.2f} to {positions_mm.max():+.2f}mm about it). "
          f"Watch the axis.")
    if not args.yes and not confirm("Proceed?"):
        print("aborted.")
        return 1

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"

    bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE, bitrate=BITRATE)
    writer = TelemetryWriter(jsonl_path)
    dt = 1.0 / args.control_hz
    # Commands are the profile's own offsets, unshifted, because SET_ORIGIN below makes the
    # motor's present position mean 0. No read-back number enters the motion arithmetic.
    target_rad = profile_rad
    last_rad = 0.0
    measured_rad: float | None = None
    replies_seen = 0
    ticks = 0
    loops = 0
    playback_s = 0.0
    commanded_seen: list[float] = []
    measured_seen: list[float] = []
    pre_zero_rad: float | None = None
    zero_readback_rad: float | None = None
    zero_applied = False
    refusal: str | None = None

    def write_summary() -> None:
        """Write summary.json from the ``finally``, so it survives an abnormal exit.

        It used to be built after the try/finally, which meant a Ctrl+C during the ramp-back
        -- or any refusal -- skipped it entirely. Three of the runs on 2026-09-02 have no
        summary for that reason, and the one number that would have diagnosed the traverse
        (where the motor thought it was) was in it.
        """
        feedback_hz = replies_seen / playback_s if playback_s > 0 else 0.0
        save_json({
            "profile": path.name,
            "profile_points": len(times),
            "profile_duration_s": float(times[-1]),
            "profile_travel_mm": travel_mm,
            "refusal": refusal,
            "zero_applied": zero_applied,
            "pre_zero_position_rad": pre_zero_rad,
            "zero_readback_rad": zero_readback_rad,
            "reference_note": (
                "playback is referenced by SET_ORIGIN, not by a position reading: the motor's "
                "position at startup is COMMANDED to become 0, and every profile position is "
                "an offset from that. So 0mm in samples.jsonl is both the profile's mean and "
                "the motor's own zero, and commanded_mm/measured_mm need no frame conversion. "
                "pre_zero_position_rad is what the motor reported BEFORE zeroing -- diagnostic "
                "only, nothing depends on it. zero_readback_rad is the check that the origin "
                "actually took: it must be ~0, because that is what makes commanding 0 mean "
                "'stay put' rather than 'traverse to the absolute origin'."
            ),
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
        }, out_dir / "summary.json")
        print(f"saved: {jsonl_path}")
        print(f"saved: {out_dir / 'summary.json'}")

    t0 = time.monotonic()
    try:
        print("enabling (zero-current wake-up frame)...")
        bus.send(to_message(codec.enable(MOTOR_NODE_ID)))
        time.sleep(0.5)

        # Recorded for diagnosis only -- nothing downstream depends on it being right. The
        # previous design made playback depend on exactly this number and it could not be
        # verified after the fact, which is what made a real traverse undiagnosable.
        pre_zero_rad = read_fresh_position_rad(bus, codec, MOTOR_NODE_ID)
        if pre_zero_rad is not None:
            print(f"motor reports {pre_zero_rad:.4f} rad ({rad_to_deg(pre_zero_rad):.2f} deg) "
                  f"before zeroing")

        # Make "here" the origin, rather than trusting a reading of where "here" is. This is
        # the pattern scripts/test_phantom_motor.py --zero already uses on this motor.
        print("setting the current position as a temporary origin...")
        bus.send(to_message(codec.zero(MOTOR_NODE_ID)))
        time.sleep(0.5)

        # The safety property, stated as a check: commanding 0 must mean "stay put". If
        # SET_ORIGIN did not take, the motor still reports its old position here and the run
        # is refused -- instead of ramping to 0 and dragging the phantom across the bench.
        # Must be a frame broadcast AFTER the zero command -- see read_fresh_position_rad.
        zero_readback_rad = read_fresh_position_rad(bus, codec, MOTOR_NODE_ID)
        if zero_readback_rad is None:
            refusal = (
                f"no status frame from the phantom motor within {POSITION_READ_TIMEOUT_S:.1f}s, "
                "so the origin could not be confirmed. Check the motor is powered and on this "
                "bus -- scripts/listen_phantom_motor.py sees its broadcast when idle."
            )
        elif abs(rad_to_mm(zero_readback_rad)) > ZERO_READBACK_TOL_MM:
            refusal = (
                f"SET_ORIGIN did not take: the motor still reports {zero_readback_rad:.4f} rad "
                f"({rad_to_mm(zero_readback_rad):+.2f}mm) where it should report ~0. Commanding "
                f"position 0 would therefore MOVE it by that much rather than hold it, which is "
                f"exactly the traverse this refuses to perform."
            )
        if refusal is not None:
            print(f"error: {refusal}")
            return 1  # the finally de-energises and writes the summary

        zero_applied = True
        print(f"origin set -- motor reads {zero_readback_rad:+.4f} rad "
              f"({rad_to_mm(zero_readback_rad):+.3f}mm), within {ZERO_READBACK_TOL_MM:.1f}mm of 0")

        last_rad = target_rad[0]
        print(f"ramping to the profile's start ({rad_to_mm(profile_rad[0]):+.2f}mm from here)...")
        ramp_to(bus, codec, MOTOR_NODE_ID, 0.0, target_rad[0], velocity_rad_s=args.ramp_velocity)

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
                    # No frame conversion: after SET_ORIGIN the motor's own zero IS where the
                    # run started, so the logged and absolute frames coincide. That is not a
                    # convenience -- the previous design subtracted a read-back reference from
                    # both series, which made a wrong reference shift them together and hid a
                    # real traverse. Here measured_mm is the motor's own reported position.
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
        try:
            ramp_to(bus, codec, MOTOR_NODE_ID, last_rad, 0.0, velocity_rad_s=args.ramp_velocity)
        except KeyboardInterrupt:
            # A second Ctrl+C, or one that lands during the ramp rather than during playback.
            # The inner handler above only wraps the playback loop, so this used to escape,
            # skip the disable below, and leave the motor holding position with a traceback
            # on screen -- visible in both runs of 2026-09-02 15:45.
            print("\ninterrupted during ramp-back -- stopping where it is.")
    except KeyboardInterrupt:
        # Anywhere else -- the ramp IN, the position read, the origin settle. Cleanup is in
        # the finally regardless; this only replaces a traceback with a sentence.
        print("\nstopped by Ctrl+C before playback finished.")
    finally:
        # De-energise on EVERY exit path, including an interrupted ramp-back and the
        # SET_ORIGIN refusal, rather than only on the one that runs to completion.
        try:
            print("disabling (zero current, motor coasts)...")
            bus.send(to_message(codec.disable(MOTOR_NODE_ID)))
        except Exception as exc:  # noqa: BLE001 - a dead bus must not mask the real error
            print(f"warning: could not send the disable frame ({exc})")
        writer.close()
        bus.shutdown()
        write_summary()

    if replies_seen:
        feedback_hz = replies_seen / playback_s if playback_s > 0 else 0.0
        print(f"motor feedback: {replies_seen} status frame(s) over {playback_s:.1f}s "
              f"({feedback_hz:.1f}Hz) -- measured_mm is real on {len(measured_seen)}/{ticks} ticks")
    else:
        print("motor feedback: NONE seen during playback -- measured_mm is null throughout, so "
              "only commanded_mm is available as ground truth. Check that the motor is powered "
              "and on this bus (scripts/listen_phantom_motor.py sees its broadcast when idle).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
