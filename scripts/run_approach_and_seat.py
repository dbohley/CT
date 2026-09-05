#!/usr/bin/env python3
"""Approach the phantom with the base motor, seat to the TRUE maximum breathing amplitude
(not just first contact), retract to a standoff distance, then hold and record.

Four phases, logged to one JSONL with a ``phase`` field:

**approach** -- creep forward until the tactile sensor reports any contact
(``abs(dist_cm) > --contact-threshold-cm``), as in scripts/run_approach_and_stop.py.

``--travel-mm`` is the *initial* target, not a bound: if the base reaches it without touching
anything, approach keeps walking the target forward until it does, stopping at the
``--max-approach-mm`` cap. That cap is the base's real safety limit. Without this the script
parked on its target and waited out a timeout that then blamed the clock for a distance
problem -- run 20260901-231950 arrived at 40.01mm at t=13s, sat there for 32 more seconds and
faulted with "no contact within 45s". Both bench runs before this change consumed their entire
40mm budget; the one that worked contacted at 40.01mm with a 6% margin over the threshold, so
the rig had been running with no headroom at all and only looked fine. Arrival is judged from
the motor's own replies (settled position error 0.006mm against ~0.87mm while moving), never
dead-reckoned -- dead reckoning is what caused session 005's overshoot bug.

**seat** -- the interesting one, and the reason this script exists. First contact is not
necessarily the deepest useful depth: if contact happens to land during exhale, the sensor
is only seeing part of the breath, and its measured peak-to-trough amplitude reads low. This
phase creeps deeper in small increments, watching amplitude after each, until amplitude stops
growing -- the signal that the sensor is finally seeing the whole breath, not a clipped
fraction of it. This mirrors ct.control.states.approach.ApproachState's SEAT sub-state
exactly (same algorithm, same two safeguards: increments are only commanded at a detected
*trough*, so a step never presses in during peak inhale; amplitude is only accumulated while
the base is stationary, so its own motion can't be mistaken for breathing amplitude) --
reusing ct.control.live.AmplitudeWatcher directly rather than reimplementing peak/trough
tracking. That real state machine has never been run against real hardware (rig_bench.yaml
still has unmeasured placeholder geometry, and ct.unknowns gates a real run on them); this
script proves the SEAT algorithm on the bench first, the same way sessions 004/005 proved the
base/needle motor protocols on the bench before any of this was wired into ct.rig.

**standoff** -- a closed loop on the tactile *reading*, driving the *base* (not the needle --
this script never commands the needle motor at all) forward past wherever seat stopped until
the sensor's **settled breathing peak** equals ``--standoff-dist-cm`` (default 0.6cm). The peak
over a breath is what the standoff distance means: the deepest the sensor reads at any point in
the cycle should sit *on* the target, never above it.

Two stages, because acceptance may only ever be decided from a peak measured over a full
breathing window with the base stationary and settled:

- *coarse* -- advance continuously at ``--creep-speed-mm-s`` until the instantaneous reading
  reaches ``--standoff-coarse-fraction`` of the target (default 0.5). Deliberately short, so
  coarse cannot overshoot even if it stops at a breathing trough.
- *fine* -- stop, wait out the settling, then measure the breathing peak as the **mean of
  ``--min-breaths`` counted breaths**. Accept anywhere in ``target +/- --standoff-tol-mm``;
  otherwise step toward the target (sized from the compliance ratio measured during this run,
  under-stepped so it converges from below), or retreat if the peak came out above the band,
  and re-measure. Once accepted the base does not move again for the rest of the run.

Both halves of that measurement were wrong until 2026-09-03, and together they cost a run
(13 fine steps, 6 retreats, 325s, cancelled, against 3 steps for the run before it):

- The peak was the ``max`` over a fixed time window, which is **biased high by the subject's
  own breath-to-breath variation**, and biased further the longer you wait. Replayed over the
  stationary measurement segments of that day's two runs it read +0.48 and +0.54mm high, worst
  case +1.77mm, against a 0.30mm tolerance -- so the loop retreated from positions that were
  actually short of target. The mean of a counted number of whole breaths has no such bias.
- The window was ``--min-breaths * --nominal-breath-s`` = 2 x 4.0s, but the profile really
  breathes at 5.5s, so "two breaths" was 1.45 of them. Counting real breaths removes the
  dependence on a nominal period that nothing keeps honest.

The acceptance band was also one-sided (``[target-tol, target]``), so any over-read cost a
base move rather than being tolerated. It is two-sided now.

An instantaneous reading taken while moving cannot decide acceptance, and run 20260901-161845
is the proof: the base stopped with the reading at exactly 6.007mm and never moved again, yet
the reading settled at 8.58mm -- 43% past target. Two effects, neither visible instant to
instant. The phantom's own log shows that stop landed at the bottom of a full exhale, and one
inhale later the same base position read 8.27mm. Separately, at matched phantom positions
before and after the stop the reading rose 1.39mm over five seconds with the motor stationary
-- the lever sinking further into the skin under sustained load. During the approach the
reading even sat flat at ~5.97mm for two seconds while the base advanced 1.3mm, because the
phantom was exhaling away at nearly the rate the base advanced.

Closing the loop on the reading rather than computing a distance to travel is likewise forced
by the physics. Measured across runs, the reading rises only **~0.22-0.68mm per 1mm of base
travel**: the lever is pressing into a compliant phantom, so travel splits between deflecting
the lever and deforming the skin. That ratio moves with skin consistency and with depth, so the
required travel cannot be known ahead of time -- only converged onto. It also means this phase
normally travels well past the depth seat accepted; seat and standoff measure different things
(amplitude plateaued vs. absolute peak reached) and need not coincide.

There is deliberately no distance-based safety bound (the base's absolute rad/mm position is
not a trustworthy reference across setups -- the bed can be re-clamped, motors re-zeroed
between sessions); the backstop is the blanket ``--max-runtime-s`` watchdog. Note the motor's
reply ``error`` field is *not* usable as a fault signal here -- see ``motor_error_note`` in the
saved summary.json. This is a different mechanism from the real STANDOFF sub-state, which
extends the needle from a computed skin position with the base held fixed -- appropriate once
the needle's own protocol is trusted enough to be part of a test; that is not this test.

**standoff_hold** -- hold there and keep recording for ``--record-s``, so the sensor's
reading of the phantom's real breathing motion can be checked afterward.

**This script drives the phantom itself**, launching
``scripts/run_breathing_profile.py --loop`` (its own CAN bus) as a subprocess right after the
confirmation prompt, so one invocation is genuinely enough -- that script's proven, already-
working homing/ramp/playback logic is reused unchanged rather than merged into this one's
control loop (two independent buses at two different natural rates; launching the working
script as-is carries far less risk than multiplexing both by hand in one process). Pass
``--no-phantom`` to skip this and drive the base motor alone (e.g. no phantom hardware
available). The phantom's own log lands at ``<out-dir>/phantom/samples.jsonl``; compare the
two afterward:

    python scripts/run_approach_and_seat.py
    ct-compare outputs/approach_and_seat/<ts>/phantom/samples.jsonl outputs/approach_and_seat/<ts>/samples.jsonl

That works with no new comparison code because this script's JSONL includes a ``tactile_mm``
field (``= dist_cm * 10.0``) and run_breathing_profile.py's includes ``commanded_mm`` --
exactly the two field names ct.phantom.driver.compare_logs already expects. Both scripts log
the raw, un-rebased ``time.monotonic()`` reading as ``t`` (a separate ``elapsed`` field is
kept, rebased to each run's own start, purely for human-readable plotting) -- required for
``compare_logs`` to align them correctly despite starting at different real moments; see
``ct.phantom.driver``'s module docstring.

**A caveat inherited from the sensor firmware, not introduced here**: ``dist_cm``'s sign is
arbitrary (a signed float zeroed once at boot, unclamped) -- every contact check in this
project uses ``abs(dist_cm)``, and so does this script. Treat ``--standoff-dist-cm`` as a
magnitude, not a signed target, until the sign is confirmed at the bench.

This script does not use ct.geometry/ProcedureContext -- the real per-mm calibration
constants (tactile_counts_to_mm, tactile_contact_counts) are still unmeasured placeholders,
whereas dist_cm/tof_mm are already unit-correct floats straight from firmware. Working
directly on those, like every other bench script so far, avoids depending on calibration that
doesn't exist yet.

    python scripts/run_approach_and_seat.py --dry-run
    python scripts/run_approach_and_seat.py
"""

from __future__ import annotations

import argparse
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import can
import numpy as np

from ct.cli._common import save_json
from ct.control.live import AmplitudeWatcher, BreathPeakWatcher
from ct.hw.bus import build_bus_from_config
from ct.hw.config import BusConfig
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.rt.telemetry import TelemetryWriter

# ---------------- Base motor config (mirrors scripts/run_approach_and_stop.py -- keep in sync) ----------------
MOTOR_CAN_CHANNEL = "/dev/cu.usbmodem207635764E451"
MOTOR_CAN_INTERFACE = "slcan"
MOTOR_BITRATE = 1_000_000

MOTOR_NODE_ID = 3  # base motor's labeled CAN node ID
POSITION_VELOCITY_MODE = 1
DRUM_RADIUS_M = 0.026  # measured capstan/drum radius
DIRECTION_SIGN = -1  # found empirically: +rad moved the wrong way, so flipped

ENTER_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MODE = bytes([0xFF] * 7 + [0xFD])
CLEAR_ERRORS = bytes([0xFF] * 7 + [0xFB])

MOTOR_REPLY_P_MIN, MOTOR_REPLY_P_MAX = -12.5, 12.5
MOTOR_REPLY_V_MIN, MOTOR_REPLY_V_MAX = -200.0, 200.0
MOTOR_REPLY_T_MIN, MOTOR_REPLY_T_MAX = -10.0, 10.0  # torque (N*m) range unconfirmed for GL-II
# -----------------------------------------------------------------------------------------------------

# ---------------- Retract-only mode (session 015: automated between-trial reset) -----------------
# A bounded, standalone base move -- no sensors, no phantom, no approach/seat/standoff state
# machine -- so a multi-trial sweep orchestrator can reset the base between physical trials
# without an operator backing it off by hand each time. Mirrors run_breathing_profile.py's
# read_fresh_position_rad exactly: bus.recv() returns the OLDEST queued frame, so a position
# read taken without flushing first can be stale by the ~25-frame margin session 012 measured.
RETRACT_FLUSH_MAX = 256
RETRACT_POSITION_TIMEOUT_S = 2.0
# -----------------------------------------------------------------------------------------------------

# ---------------- Sensor config (mirrors ct.cli.sensor_bench / measure_sensor_noise.py) ----------------
SENSOR_CAN_CHANNEL = "/dev/cu.usbmodem20553962534B1"
SENSOR_CAN_INTERFACE = "slcan"
SENSOR_BITRATE = 1_000_000
SENSOR_CAN_ID = 5
_PAYLOAD = struct.Struct("<Hfh")  # uint16 tof_mm, float32 dist_cm, int16 angle_centideg
# -----------------------------------------------------------------------------------------------------

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "approach_and_seat"

# The INITIAL approach target, not a bound -- approach auto-extends past it (see below).
# Raised from 40mm because 40 turned out to be exactly marginal, not generous: BOTH bench runs
# consumed every millimetre of it. Run 20260901-165415 made contact at 40.01mm with a peak
# reading of 0.00941cm against a 0.01 threshold -- a 6% margin -- and run 20260901-231950, on a
# profile whose baseline sat ~1mm further out, reached 0.00565 and never contacted at all.
DEFAULT_TRAVEL_MM = 60.0
# The real bound. Deliberately NOT called --max-travel-mm: on this script that name already
# means the phantom profile's peak-to-peak passthrough to run_breathing_profile.py.
DEFAULT_MAX_APPROACH_MM = 100.0
APPROACH_ARRIVAL_TOL_MM = 0.5  # settled position error is 0.006mm mean / 0.016mm max; while
# still moving it is ~0.87mm. 0.5mm separates the two by more than an order of magnitude.
APPROACH_BASELINE_S = 3.0  # pre-contact window used for the diagnostic baseline only
# dist_cm is a signed float zeroed once at FIRMWARE boot, and that zero drifts: measured at
# 0.0048mm resting on 2026-09-01 16:54 and 0.1967mm on 2026-09-02 15:46, a 40x creep over two
# days, while its std stayed at 0.002-0.014mm. The reading is clean; the datum moves. Once the
# drift passed the 0.1mm contact threshold every run declared contact on its first sample and
# skipped APPROACH entirely. So the zero is measured per run instead of trusted from boot.
DEFAULT_TARE_S = 2.0
# A free arm sits still (p2p 0.014-0.10mm over such a window); an arm already riding the
# breathing phantom swings with it (p2p 0.836mm measured during standoff_hold). Between those
# is where "the tare is about to hide real contact" lives.
TARE_CONTACT_P2P_MM = 0.3
DEFAULT_VELOCITY_RAD_S = 0.1  # approach + retract velocity
DEFAULT_CONTACT_THRESHOLD_CM = 0.01
DEFAULT_STANDOFF_DIST_CM = .9  # empirical: needle (unactuated, fully retracted) sits close to but not touching skin here
# Coarse continuous advance stops here, well short of the target, because an instantaneous
# reading taken while moving understates the settled breathing peak by the swing plus the
# viscoelastic rise (measured together at ~2.5mm in run 20260901-161845). Half the target
# leaves more headroom than that, so coarse cannot overshoot even if it stops at a trough.
DEFAULT_STANDOFF_COARSE_FRACTION = 0.5
DEFAULT_STANDOFF_TOL_MM = 0.3
"""Half-width of the accept band on the measured breathing peak, applied on BOTH sides.

It used to be one-sided -- accept in ``[target-tol, target]``, retreat above ``target`` -- and
that made overshoot cost a base move. Combined with a peak estimator biased ~0.5mm high (see
BreathPeakWatcher) the controller oscillated: run 20260903-152958 took 13 fine steps with 6
retreats over 325s and never converged, against 3 steps for the run before it.

0.3mm is viable only because the bias is gone. With an unbiased mean over two real breaths the
standard error is ~0.38mm, so a two-sided 0.3 accepts ~56% of attempts (91% within three
steps); raise it to 0.5 for ~81% per attempt if the extra steps are more annoying than the
extra millimetre is harmful.
"""

DEFAULT_STANDOFF_MAX_STEPS = 8
"""Give up after this many fine steps, the way approach gives up on travel (session 009).

Not a tuning knob so much as an admission that the loop can fail: a controller with no way to
stop is one the operator has to cancel, which is what happened on 2026-09-03, and a cancelled
run reports nothing about why.
"""
STANDOFF_ADVANCE_SAFETY = 0.6  # under-step when advancing, so the peak converges from below
STANDOFF_RETREAT_SAFETY = 0.8  # correcting an overshoot: get out of the unsafe zone promptly
STANDOFF_MIN_STEP_MM = 0.1
STANDOFF_MAX_STEP_MM = 3.0
DEFAULT_COMMAND_HZ = 20.0

# SEAT defaults match configs/rig_bench.yaml's procedure.approach.* block exactly, so a bench
# run and the real (never-yet-run-on-hardware) ApproachState use the same numbers by default.
DEFAULT_CREEP_INCREMENT_MM = 0.5
DEFAULT_CREEP_SPEED_MM_S = 1.0
DEFAULT_MIN_BREATHS = 2.0
DEFAULT_NOMINAL_BREATH_S = 4.0
DEFAULT_AMPLITUDE_TOL_MM = 0.2
DEFAULT_STABLE_INCREMENTS = 2
DEFAULT_MAX_SEAT_INCREMENTS = 40

# The hold window is what the estimator gets, and Stage 1 measures Q from breath-to-breath
# refits -- so this is really "how many breaths", not "how many seconds". 180s is ~36 breaths
# at 12bpm: 90s to calibrate (~18 refits, matching what session 006 had on the OptiTrack
# recordings) and 90s to track. The previous 60s afforded only ~6 refits and a noisy Q.
DEFAULT_RECORD_S = 180.0
DEFAULT_MAX_RUNTIME_S = 900.0  # blanket safety watchdog; raised with --record-s above

DEFAULT_PROFILE = "breathing_profile_1"
# Long enough to cover run_breathing_profile.py's slowest fail-fast path: interpreter start,
# loading a 74k-row profile, opening the bus, enable + 0.5s, then its 2.0s position read before
# it can refuse for want of a reference position. The old 2.0s expired while that read was
# still in progress, so the parent saw a live process and drove into a test with no phantom.
# The in-loop poll below is the real backstop; this only makes the common case fail cleanly.
PHANTOM_STARTUP_CHECK_S = 6.0
PHANTOM_SHUTDOWN_TIMEOUT_S = 20.0  # generous -- covers its own ramp-back-to-start before disabling
BREATHING_PROFILE_SCRIPT = REPO_ROOT / "scripts" / "run_breathing_profile.py"


def pos_vel_can_id(node_id: int) -> int:
    return (POSITION_VELOCITY_MODE << 8) | node_id


def build_pos_vel_frame(node_id: int, pos_rad: float, vel_rad_s: float) -> can.Message:
    data = struct.pack("<ff", pos_rad, vel_rad_s)
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=data, is_extended_id=False)


def universal_command(node_id: int, cmd_bytes: bytes) -> can.Message:
    return can.Message(arbitration_id=pos_vel_can_id(node_id), data=cmd_bytes, is_extended_id=False)


class SetupFailed(RuntimeError):
    """A pre-motion check failed. Carried by ``fault_reason`` and reported like any fault.

    Raised rather than returned so it unwinds to the same ``finally`` that de-energises the
    motor and stops the phantom, and still reaches the summary -- a bare ``raise`` would skip
    the summary, which is the mistake session 011 had to fix on the phantom side.
    """


def measure_tactile_zero(sensor_bus, duration_s: float) -> dict:
    """Sample the tactile reading with the base stationary, to establish this run's zero.

    Returns the raw signed ``dist_cm`` statistics -- signed, because the sign of ``dist_cm``
    is documented as arbitrary and the deflection that matters is ``|dist_cm - zero|`` in
    either direction. Taking the magnitude first would fold a negative rest position onto a
    positive one and make the tare wrong.

    Must be called before any motion is commanded: the whole point is to capture the arm at
    rest, and a moving base contaminates it immediately.
    """
    samples: list[float] = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        for _stamp, can_id, data in sensor_bus.poll():
            if can_id != SENSOR_CAN_ID or len(data) < _PAYLOAD.size:
                continue
            _tof_mm, dist_cm, _angle = _PAYLOAD.unpack(data[: _PAYLOAD.size])
            samples.append(float(dist_cm))
        time.sleep(0.005)

    if not samples:
        return {"n": 0, "zero_cm": 0.0, "zero_mm": 0.0, "std_mm": None, "p2p_mm": None}
    values = np.array(samples, dtype=float)
    return {
        "n": int(values.size),
        "zero_cm": float(values.mean()),
        "zero_mm": float(values.mean()) * 10.0,
        "std_mm": float(values.std()) * 10.0,
        "p2p_mm": float(values.max() - values.min()) * 10.0,
    }


def confirm(prompt: str) -> bool:
    return input(f"{prompt} [y/N] ").strip().lower() == "y"


def mm_to_rad(travel_mm: float) -> float:
    return DIRECTION_SIGN * (travel_mm / 1000.0) / DRUM_RADIUS_M


def settle_time_s(increment_mm: float, speed_mm_s: float) -> float:
    """How long to wait after a creep increment before trusting the sensor again.

    Mirrors ct.control.states.approach.ApproachState._settle_time exactly.
    """
    return max(increment_mm / max(speed_mm_s, 1e-6), 0.05) + 0.2


def at_trough(tactile_mm: float, watcher: AmplitudeWatcher) -> bool:
    """Mirrors ApproachState._at_trough: judged against the window's own range, not a model."""
    span = watcher.amplitude
    if span <= 0:
        return True
    return tactile_mm <= watcher.trough + 0.2 * span


def _read_fresh_base_position_rad(motor_bus, motor_reply_codec) -> float | None:
    """Position from a reply broadcast *after* this call, not one already queued.

    Same reasoning as run_breathing_profile.py's read_fresh_position_rad, against this file's
    own base-motor codec (CubeMarsMIT) rather than the phantom's CubeMarsServo.
    """
    for _ in range(RETRACT_FLUSH_MAX):
        if motor_bus.recv(timeout=0.0) is None:
            break
    deadline = time.monotonic() + RETRACT_POSITION_TIMEOUT_S
    while time.monotonic() < deadline:
        msg = motor_bus.recv(timeout=0.05)
        if msg is None:
            continue
        parsed = motor_reply_codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
            return float(parsed["position"])
    return None


def retract_only(retract_mm: float, velocity_rad_s: float, dry_run: bool) -> int:
    """Back the base off by ``retract_mm`` and exit -- no sensors, no phantom, no state machine.

    Exists so a multi-trial sweep orchestrator (``scripts/collect_param_sweep_runs.py``) can
    reset the base between physical trials without an operator doing it by hand each time --
    explicitly requested for that purpose, at this specific bounded distance, which is the
    authorization session 014's finding said was the actual bar (that session refused an
    *unrequested* auto-retract drafted as a side fix to a different problem). No interactive
    confirm: the orchestrator already asks once before the whole batch, and this is a single
    bounded move on one axis, not the full four-phase procedure the normal confirm() describes.

    The existing loaded-tare refusal in the normal run path is untouched and stays the real
    backstop -- if the arm is genuinely still loaded beyond what this clears, the *next*
    trial's tare check refuses exactly as it does today, rather than this mode trying to be a
    second safety system.
    """
    print(f"retract-only: backing off {retract_mm:.1f}mm at {velocity_rad_s:.3f}rad/s")
    if dry_run:
        print("--dry-run: not opening any bus.")
        print("  ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print("  ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        print(f"   <read current position, then command it minus {mm_to_rad(retract_mm):.4f}rad "
              f"at {velocity_rad_s:.3f}rad/s>")
        print("  ", universal_command(MOTOR_NODE_ID, EXIT_MODE), " <- sent on exit")
        return 0

    motor_bus = can.interface.Bus(channel=MOTOR_CAN_CHANNEL, interface=MOTOR_CAN_INTERFACE,
                                   bitrate=MOTOR_BITRATE)
    motor_reply_codec = CubeMarsMIT(
        p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
        v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
        t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
    )
    motion_commanded = False
    ok = False
    try:
        motor_bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)
        motor_bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.5)

        start_rad = _read_fresh_base_position_rad(motor_bus, motor_reply_codec)
        if start_rad is None:
            print(f"error: no reply from the base motor within {RETRACT_POSITION_TIMEOUT_S:.1f}s "
                  f"-- not commanding a move with no known starting position.")
            return 1

        # Retracting is moving AWAY from the phantom, i.e. the opposite of mm_to_rad's
        # "toward the phantom" convention (used everywhere else in this file for --travel-mm).
        target_rad = start_rad - mm_to_rad(retract_mm)
        print(f"  current position {start_rad:.4f}rad -> target {target_rad:.4f}rad "
              f"({retract_mm:.1f}mm back)")
        motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target_rad, velocity_rad_s))
        motion_commanded = True

        timeout_s = abs(mm_to_rad(retract_mm)) / max(velocity_rad_s, 1e-6) + 15.0
        deadline = time.monotonic() + timeout_s
        last_position = start_rad
        while time.monotonic() < deadline:
            motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, target_rad, velocity_rad_s))
            msg = motor_bus.recv(timeout=0.05)
            if msg is not None:
                parsed = motor_reply_codec.parse(msg.arbitration_id, bytes(msg.data))
                if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
                    last_position = float(parsed["position"])
                    error_mm = abs(last_position - target_rad) * DRUM_RADIUS_M * 1000.0
                    if error_mm <= APPROACH_ARRIVAL_TOL_MM:
                        ok = True
                        break
            time.sleep(0.02)
        if not ok:
            print(f"error: did not settle within {timeout_s:.1f}s "
                  f"(last known position {last_position:.4f}rad, target {target_rad:.4f}rad)")
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    finally:
        if motion_commanded:
            motor_bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))
        motor_bus.shutdown()
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--travel-mm", type=float, default=DEFAULT_TRAVEL_MM,
                         help="INITIAL approach target. Not a bound -- if the base arrives here "
                              "without contact it keeps creeping forward up to --max-approach-mm")
    parser.add_argument("--max-approach-mm", type=float, default=DEFAULT_MAX_APPROACH_MM,
                         dest="max_approach_mm",
                         help="hard bound on total forward approach travel; reaching it is a "
                              "fault. This is the base's real safety limit, not --travel-mm")
    parser.add_argument("--velocity", type=float, default=DEFAULT_VELOCITY_RAD_S, help="rad/s, approach + retract")
    parser.add_argument("--contact-threshold-cm", type=float, default=DEFAULT_CONTACT_THRESHOLD_CM,
                         dest="contact_threshold_cm")
    parser.add_argument("--standoff-dist-cm", type=float, default=DEFAULT_STANDOFF_DIST_CM,
                         dest="standoff_dist_cm")
    parser.add_argument("--standoff-coarse-fraction", type=float, default=DEFAULT_STANDOFF_COARSE_FRACTION,
                         dest="standoff_coarse_fraction",
                         help="fraction of the standoff target at which continuous coarse advance "
                              "stops and the step/settle/measure fine loop takes over")
    parser.add_argument("--standoff-tol-mm", type=float, default=DEFAULT_STANDOFF_TOL_MM,
                         dest="standoff_tol_mm",
                         help="half-width of the accept band around the standoff target. "
                              "Two-sided: the peak is accepted anywhere in [target-tol, "
                              "target+tol] and only retreats above it")
    parser.add_argument("--standoff-max-steps", type=int, default=DEFAULT_STANDOFF_MAX_STEPS,
                         dest="standoff_max_steps",
                         help="give up after this many fine steps, as a named fault naming the "
                              "peaks and the band, rather than stepping forever")
    parser.add_argument("--allow-loaded-tare", dest="allow_loaded_tare", action="store_true",
                         help="tare even if the arm is already in contact. Off by default: such "
                              "a zero hides real contact and every depth downstream is measured "
                              "from a datum that was never established")
    parser.add_argument("--creep-increment-mm", type=float, default=DEFAULT_CREEP_INCREMENT_MM)
    parser.add_argument("--creep-speed-mm-s", type=float, default=DEFAULT_CREEP_SPEED_MM_S)
    parser.add_argument("--min-breaths", type=float, default=DEFAULT_MIN_BREATHS)
    parser.add_argument("--nominal-breath-s", type=float, default=DEFAULT_NOMINAL_BREATH_S)
    parser.add_argument("--amplitude-tol-mm", type=float, default=DEFAULT_AMPLITUDE_TOL_MM)
    parser.add_argument("--stable-increments", type=int, default=DEFAULT_STABLE_INCREMENTS)
    parser.add_argument("--max-seat-increments", type=int, default=DEFAULT_MAX_SEAT_INCREMENTS)
    parser.add_argument("--record-s", type=float, default=DEFAULT_RECORD_S,
                         help="how long to hold at standoff and keep recording")
    parser.add_argument("--command-hz", type=float, default=DEFAULT_COMMAND_HZ)
    parser.add_argument("--max-runtime-s", type=float, default=DEFAULT_MAX_RUNTIME_S,
                         dest="max_runtime_s", help="blanket safety watchdog across the whole run")
    parser.add_argument("--profile", default=DEFAULT_PROFILE,
                         help="breathing profile to loop on the phantom (passed to run_breathing_profile.py)")
    parser.add_argument("--max-travel-mm", type=float, default=None, dest="max_travel_mm",
                         help="passed through to run_breathing_profile.py; overrides its own "
                              "default (6.0mm) if the profile's peak-to-peak travel needs more "
                              "(e.g. moira_normal_breathing at 7.88mm, sara at 9.03mm)")
    parser.add_argument("--tare-s", type=float, default=DEFAULT_TARE_S, dest="tare_s",
                         help="seconds of stationary tactile samples taken before any motion, "
                              "used as this run's zero. The firmware zero drifts between runs")
    parser.add_argument("--no-tare", dest="tare", action="store_false",
                         help="use the raw firmware zero instead of taring -- the pre-2026-09-02 "
                              "behaviour, kept for comparison")
    parser.add_argument("--no-phantom", action="store_true", dest="no_phantom",
                         help="don't drive the phantom -- base motor + sensors only")
    parser.add_argument("--out", type=Path, default=None,
                         help="output directory; default outputs/approach_and_seat/<timestamp>")
    parser.add_argument("--dry-run", action="store_true", help="print what would be sent, send nothing")
    parser.add_argument("--yes", action="store_true",
                         help="skip the interactive confirmation before commanding motion -- "
                              "for automated use (e.g. scripts/collect_param_sweep_runs.py), "
                              "mirroring run_breathing_profile.py's --yes")
    parser.add_argument("--retract-only-mm", type=float, default=None, dest="retract_only_mm",
                         help="back the base off by this many mm and exit -- no sensors, no "
                              "phantom, no approach/seat/standoff. For resetting between trials "
                              "in an automated multi-trial sweep (scripts/collect_param_sweep_runs.py); "
                              "ignores every other approach/seat/standoff flag.")
    args = parser.parse_args()

    if args.retract_only_mm is not None:
        return retract_only(args.retract_only_mm, args.velocity, args.dry_run)

    if args.travel_mm > args.max_approach_mm:
        print(f"error: --travel-mm {args.travel_mm:.1f} exceeds --max-approach-mm "
              f"{args.max_approach_mm:.1f}. The first is the initial target and the second is "
              f"the hard bound, so the bound has to be the larger of the two.")
        return 1

    target_rad = mm_to_rad(args.travel_mm)
    creep_rad = mm_to_rad(args.creep_increment_mm)
    creep_velocity_rad_s = (args.creep_speed_mm_s / 1000.0) / DRUM_RADIUS_M
    window_s = args.min_breaths * args.nominal_breath_s
    target_mm = args.standoff_dist_cm * 10.0

    print(f"base motor (GL60II), node id {MOTOR_NODE_ID}, arbitration id 0x{pos_vel_can_id(MOTOR_NODE_ID):03X}")
    print(f"phase 1 approach: toward {args.travel_mm:.1f}mm ({target_rad:.4f}rad) at {args.velocity:.3f}rad/s, "
          f"contact threshold abs(dist_cm) > {args.contact_threshold_cm}cm.")
    print(f"                  If it gets there without contact it KEEPS ADVANCING, up to a hard "
          f"cap of {args.max_approach_mm:.1f}mm total.")
    print(f"phase 2 seat: creep {args.creep_increment_mm:.2f}mm at a time, only at a detected trough, "
          f"watching amplitude over a {window_s:.1f}s window ({args.min_breaths:g} breaths @ "
          f"{args.nominal_breath_s:g}s); accept once amplitude grows by <= {args.amplitude_tol_mm:.2f}mm "
          f"for {args.stable_increments} checks in a row (cap {args.max_seat_increments} increments)")
    print(f"phase 3 standoff: coarse advance at {args.creep_speed_mm_s:.2f}mm/s to "
          f"{args.standoff_coarse_fraction:.0%} of target, then fine step/settle/measure until the "
          f"breathing peak, averaged over {args.min_breaths:g} counted breaths, lands in "
          f"[{target_mm - args.standoff_tol_mm:.2f}, {target_mm + args.standoff_tol_mm:.2f}]mm "
          f"(retreating only above it), giving up after {args.standoff_max_steps} steps -- "
          f"closed loop on the sensor, no distance bound")
    print(f"phase 4 standoff_hold: record for {args.record_s:.0f}s")
    print(f"resending the current target at {args.command_hz:.0f}Hz. This script never commands the needle motor.")

    if args.dry_run:
        print("\n--dry-run: not opening any bus. Frames that would be sent, in order:")
        print(" ", universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        print(" ", universal_command(MOTOR_NODE_ID, ENTER_MODE))
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, target_rad, args.velocity), "  (approach)")
        print(f"  ... (resent at {args.command_hz:.0f}Hz; if {args.travel_mm:.1f}mm is reached "
              f"without contact, the target keeps walking forward to at most "
              f"{args.max_approach_mm:.1f}mm) ...")
        print(" ", build_pos_vel_frame(MOTOR_NODE_ID, mm_to_rad(args.max_approach_mm), args.velocity),
              "  (approach, at the cap)")
        print(f"  ... (on contact, creep in {args.creep_increment_mm:.2f}mm steps at troughs "
              f"while watching amplitude) ...")
        print(f"  ... (seated; standoff coarse-advances at {args.creep_speed_mm_s:.2f}mm/s, then "
              f"fine step/settle/measure until the settled peak reaches {target_mm:.1f}mm) ...")
        print(f"  ... (hold at standoff, record for {args.record_s:.0f}s) ...")
        print(" ", universal_command(MOTOR_NODE_ID, EXIT_MODE), " <- sent on exit")
        return 0

    out_dir = args.out or (DEFAULT_OUT_DIR / time.strftime("%Y%m%d-%H%M%S"))
    jsonl_path = out_dir / "samples.jsonl"
    summary_path = out_dir / "summary.json"

    phantom_note = (
        f" This will also launch run_breathing_profile.py to loop '{args.profile}' on the "
        f"phantom motor for the duration of the run." if not args.no_phantom else
        " --no-phantom: the phantom will NOT be driven; SEAT will have no real signal to find."
    )
    print(f"\nThis will move the base motor toward the phantom by UP TO {args.max_approach_mm:.1f}mm "
          f"(advancing past {args.travel_mm:.1f}mm on its own if it has not touched by then), seat "
          f"past first contact to find true max breathing amplitude, creep further to standoff, and "
          f"hold+record for {args.record_s:.0f}s.{phantom_note} Watch it closely. Ctrl+C stops and "
          f"de-energizes at any point.")
    if not args.yes and not confirm("Proceed?"):
        print("aborted.")
        return 1

    phantom_proc: subprocess.Popen | None = None
    phantom_log = None
    if not args.no_phantom:
        phantom_out_dir = out_dir / "phantom"
        phantom_cmd = [sys.executable, str(BREATHING_PROFILE_SCRIPT),
                        "--profile", args.profile, "--loop", "--yes", "--out", str(phantom_out_dir)]
        if args.max_travel_mm is not None:
            phantom_cmd += ["--max-travel-mm", str(args.max_travel_mm)]
        print(f"launching phantom: {args.profile} (looped), logging to {phantom_out_dir}...")
        # Capture the subprocess's output to a file rather than letting it scroll past in the
        # shared terminal. The phantom prints the one thing needed to diagnose a bad startup
        # move -- where it thought the motor was, and whether SET_ORIGIN took -- and on
        # 2026-09-02 that line was lost to scrollback while a traverse went unexplained.
        phantom_out_dir.mkdir(parents=True, exist_ok=True)
        phantom_log = open(phantom_out_dir / "stdout.log", "w")
        phantom_proc = subprocess.Popen(phantom_cmd, stdout=phantom_log,
                                         stderr=subprocess.STDOUT)
        time.sleep(PHANTOM_STARTUP_CHECK_S)
        if phantom_proc.poll() is not None:
            phantom_log.close()
            print(f"error: run_breathing_profile.py exited (code {phantom_proc.returncode}) before "
                  f"the run started -- not proceeding into a test with no real phantom signal. "
                  f"Its output:")
            print("  " + (phantom_out_dir / "stdout.log").read_text().strip().replace("\n", "\n  "))
            return 1

    def stop_phantom() -> None:
        if phantom_proc is not None and phantom_proc.poll() is None:
            print("stopping phantom (SIGINT, so it ramps back and disables cleanly)...")
            phantom_proc.send_signal(signal.SIGINT)
            try:
                phantom_proc.wait(timeout=PHANTOM_SHUTDOWN_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                print("phantom didn't exit in time -- terminating.")
                phantom_proc.terminate()
                try:
                    phantom_proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    phantom_proc.kill()
        if phantom_log is not None and not phantom_log.closed:
            phantom_log.close()

    # Bus/writer setup deliberately guarded on its own -- a failure here (bad channel, port
    # busy) must not leave the just-launched phantom subprocess running unsupervised, since
    # nothing later in this function would ever reach the main try/finally to stop it.
    try:
        motor_bus = can.interface.Bus(channel=MOTOR_CAN_CHANNEL, interface=MOTOR_CAN_INTERFACE, bitrate=MOTOR_BITRATE)
        sensor_bus = build_bus_from_config(
            "approach-seat-sensors",
            BusConfig(backend="rh02", interface=SENSOR_CAN_INTERFACE, channel=SENSOR_CAN_CHANNEL, bitrate=SENSOR_BITRATE),
        )
        writer = TelemetryWriter(jsonl_path)
        motor_reply_codec = CubeMarsMIT(
            p_min=MOTOR_REPLY_P_MIN, p_max=MOTOR_REPLY_P_MAX,
            v_min=MOTOR_REPLY_V_MIN, v_max=MOTOR_REPLY_V_MAX,
            t_min=MOTOR_REPLY_T_MIN, t_max=MOTOR_REPLY_T_MAX,
        )
    except Exception:
        stop_phantom()
        raise

    motion_commanded = False
    last_motor_reply: dict | None = None
    last_motor_reply_t: float | None = None
    motor_replies_seen = 0
    current_target_rad = target_rad
    current_velocity_rad_s = args.velocity
    next_command_at = 0.0
    command_period_s = 1.0 / max(args.command_hz, 1e-6)

    phase = "approach"
    # Approach walks its target forward past --travel-mm rather than parking on it. Run
    # 20260901-231950 is why: the base arrived at 40.01mm at t=13s, sat there for 32 more
    # seconds, and then faulted with "no contact within 45s" -- blaming the clock for what was
    # a distance problem. It never had a way to say "I have run out of travel".
    approach_travelled_mm = args.travel_mm  # commanded so far, including any extension
    approach_extending = False
    approach_peak_deflection_mm = 0.0
    approach_baseline: list[float] = []
    approach_baseline_warned = False
    tare: dict | None = None
    tactile_zero_cm = 0.0   # 0.0 means "untared", i.e. the raw firmware zero
    tare_contact_warned = False
    watcher = AmplitudeWatcher(window_s=window_s)
    increments = 0
    stable_count = 0
    previous_amplitude: float | None = None
    settling_until = 0.0
    next_decision_at = 0.0
    pending_creep = False
    seat_rad: float | None = None
    accepted_seat_rad: float | None = None
    accepted_peak_mm: float | None = None
    contact_t: float | None = None
    standoff_segments = 0
    standoff_stage = "coarse"          # "coarse" (continuous) -> "fine" (step/settle/measure)
    standoff_advancing = False
    standoff_advanced_mm = 0.0         # signed: net travel during standoff, +ve = deeper
    standoff_step_remaining_mm = 0.0   # of the current fine step still to travel
    standoff_step_sign = 1.0           # +1 advancing deeper, -1 retreating
    standoff_retreat_steps = 0
    standoff_settled_peak_mm: float | None = None
    standoff_recent_peaks: list[float] = []     # for the give-up message
    # Standoff measures the breathing peak over whole real breaths, not a fixed time window.
    # See BreathPeakWatcher: max-over-window is biased high by the spread of the subject's own
    # breathing, and standoff RETREATS whenever the reading exceeds target, so that bias drives
    # spurious retreats. Replayed over the stationary measurement segments of the two runs on
    # 2026-09-03, max-over-8s read +0.48 and +0.54mm higher than the mean of two real breaths
    # (worst single segment +1.77mm) -- against a 0.30mm tolerance.
    breath_watcher = BreathPeakWatcher(n_breaths=args.min_breaths)
    standoff_crossed_t: float | None = None
    hold_started_at: float | None = None
    fault_reason: str | None = None

    def standoff_step_mm(peak_mm: float, target_mm: float, safety: float) -> float:
        """How far to move the base to shift the settled peak by ``target_mm - peak_mm``.

        Sized from the compliance ratio *measured during this run* -- how much reading each mm
        of travel has actually bought so far -- rather than a fixed increment, because that
        ratio (0.22-0.68 across runs) varies with skin consistency and depth and cannot be
        known in advance. ``safety`` under-steps so the peak converges from below instead of
        jumping past. Falls back to one creep increment before there is anything to measure.
        """
        if accepted_peak_mm is None or standoff_advanced_mm <= 1e-6:
            return args.creep_increment_mm
        ratio = (peak_mm - accepted_peak_mm) / standoff_advanced_mm
        if ratio <= 0.05:  # implausible/degenerate -- don't divide by it
            return args.creep_increment_mm
        step = abs(target_mm - peak_mm) / ratio * safety
        return min(max(step, STANDOFF_MIN_STEP_MM), STANDOFF_MAX_STEP_MM)

    def hold_basis_rad(elapsed: float) -> float:
        """Prefer the latest real reply plus a small residual correction over dead
        reckoning -- the same fix run_approach_and_stop.py's overshoot bug needed."""
        if last_motor_reply is not None and last_motor_reply_t is not None:
            gap_s = elapsed - last_motor_reply_t
            return last_motor_reply["position"] + DIRECTION_SIGN * current_velocity_rad_s * gap_s
        return current_target_rad

    def arrived_at_target() -> bool:
        """Has the base reached the position it was last commanded to?

        Judged from the motor's own replies, which arrive at ~19Hz alongside the 20Hz
        command resend. Measured in run 20260901-231950: once parked the position error is
        0.006mm mean / 0.016mm max, against ~0.87mm while still moving -- so the
        APPROACH_ARRIVAL_TOL_MM band separates the two by more than an order of magnitude.

        Returns False with no telemetry rather than guessing. Extending the target on a
        dead-reckoned guess is how run_approach_and_stop.py's overshoot bug happened
        (session 005); with no replies the approach timeout is the honest backstop.
        """
        if last_motor_reply is None:
            return False
        error_mm = abs(last_motor_reply["position"] - current_target_rad) * DRUM_RADIUS_M * 1000.0
        return error_mm <= APPROACH_ARRIVAL_TOL_MM

    def stop_motor(reason: str) -> None:
        print(f"stopping motor ({reason})...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, EXIT_MODE))

    t0 = time.monotonic()
    # Derived from the CAP, not from --travel-mm: the initial target is no longer as far as
    # the base will ever go, and a timeout sized to it would fire before the cap fault could,
    # which is exactly the misleading failure this change exists to remove.
    approach_timeout_s = abs(mm_to_rad(args.max_approach_mm)) / max(args.velocity, 1e-6) + 30.0

    try:
        print("clearing errors...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, CLEAR_ERRORS))
        time.sleep(0.1)
        print("entering motor control mode...")
        motor_bus.send(universal_command(MOTOR_NODE_ID, ENTER_MODE))
        time.sleep(0.5)

        # Establish this run's tactile zero BEFORE anything moves. The firmware zero drifts
        # (see DEFAULT_TARE_S), and once it passed the contact threshold every run declared
        # contact on its first sample and never had an approach phase at all.
        if args.tare:
            print(f"taring the tactile sensor over {args.tare_s:.1f}s (base stationary)...")
            tare = measure_tactile_zero(sensor_bus, args.tare_s)
            if tare["n"] == 0:
                fault_reason = (
                    f"no tactile samples in {args.tare_s:.1f}s of taring -- the sensor is not "
                    f"reporting on CAN id {SENSOR_CAN_ID}, so nothing this run measures would "
                    f"mean anything. Check the sensor bus."
                )
                raise SetupFailed
            tactile_zero_cm = tare["zero_cm"]
            print(f"  zero = {tare['zero_mm']:+.4f}mm  (std {tare['std_mm']:.4f}, "
                  f"p2p {tare['p2p_mm']:.4f}mm, {tare['n']} samples)")
            if tare["p2p_mm"] > TARE_CONTACT_P2P_MM:
                tare_contact_warned = True
                # A free arm reads 0.014mm p2p here; one riding the phantom read 8.330mm on
                # 2026-09-03. The tare's whole premise is that the sensor starts out of
                # contact, as it does in a real procedure, and a zero taken mid-swing hides
                # real contact: run 20260903-152958 then declared contact at t=0.006s, skipped
                # APPROACH entirely, seated at 0.219mm and never converged. If the base was
                # left in contact from a previous run, back it off by hand before this one.
                message = (
                    f"the arm is swinging {tare['p2p_mm']:.3f}mm over the tare window, above "
                    f"the {TARE_CONTACT_P2P_MM:.2f}mm expected of a free arm, so it is already "
                    f"riding the phantom rather than resting clear of it. Taring now would zero "
                    f"out real contact and every depth this run reports would be measured from "
                    f"a datum that was never established. Retract the base until the arm is "
                    f"free, then re-run (or pass --allow-loaded-tare to proceed anyway)."
                )
                if args.allow_loaded_tare:
                    print(f"\n  WARNING: {message}\n")
                else:
                    fault_reason = message
                    raise SetupFailed

        t0 = time.monotonic()
        print("commanding approach...")
        motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, current_target_rad, current_velocity_rad_s))
        motion_commanded = True
        next_command_at = command_period_s

        print("running (Ctrl+C to stop and release at any point)...")
        while True:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed >= args.max_runtime_s:
                fault_reason = f"max-runtime watchdog ({args.max_runtime_s:.0f}s) exceeded in phase '{phase}'"
                break

            # A phantom that dies mid-run used to go entirely unnoticed: the base kept
            # seating and standing off against a stationary surface, and the run looked
            # successful while measuring nothing. Cheap to check, so check every tick.
            if phantom_proc is not None and phantom_proc.poll() is not None:
                fault_reason = (
                    f"the phantom subprocess exited (code {phantom_proc.returncode}) during "
                    f"phase '{phase}' at t={elapsed:.1f}s -- there is no breathing signal to "
                    f"measure, so continuing would produce a run that looks fine and means "
                    f"nothing. Check its output above."
                )
                break

            motor_msg = motor_bus.recv(timeout=0.0)
            if motor_msg is not None:
                parsed = motor_reply_codec.parse(motor_msg.arbitration_id, bytes(motor_msg.data))
                if parsed is not None and int(parsed["node_id"]) == MOTOR_NODE_ID:
                    last_motor_reply = parsed
                    last_motor_reply_t = elapsed
                    motor_replies_seen += 1

            if elapsed >= next_command_at:
                if phase == "approach":
                    # Once the base reaches its target without contact, keep creeping instead
                    # of parking there. Same mechanism the standoff coarse stage uses below --
                    # walk the commanded target one tick's worth at a time -- rather than a
                    # second, differently-behaved way of advancing the same axis. The advance
                    # speed is the approach velocity, not the slower creep speed: nothing is
                    # touching yet, so there is nothing to creep up on.
                    if not approach_extending and arrived_at_target():
                        approach_extending = True
                        print(f"  approach: reached {approach_travelled_mm:.1f}mm without contact "
                              f"-- extending toward the {args.max_approach_mm:.0f}mm cap")
                    if approach_extending:
                        step_mm = abs(args.velocity) * DRUM_RADIUS_M * 1000.0 * command_period_s
                        step_mm = min(step_mm, args.max_approach_mm - approach_travelled_mm)
                        if step_mm > 0:
                            current_target_rad = current_target_rad + mm_to_rad(step_mm)
                            approach_travelled_mm += step_mm
                elif phase == "standoff" and standoff_advancing:
                    # Walk the commanded target by one tick's worth of travel, so the drive
                    # tracks a smooth constant-velocity ramp with the target always just ahead
                    # of actual position -- no large position error, no lurch, and stopping is
                    # instantaneous (just stop advancing). Coarse runs open-ended until the
                    # reading gate trips; fine walks an exact, pre-computed distance.
                    step_mm = args.creep_speed_mm_s * command_period_s
                    if standoff_stage == "fine":
                        step_mm = min(step_mm, standoff_step_remaining_mm)
                        standoff_step_remaining_mm -= step_mm
                        if standoff_step_remaining_mm <= 1e-9:
                            standoff_advancing = False
                            settling_until = elapsed + settle_time_s(
                                args.creep_increment_mm, args.creep_speed_mm_s
                            )
                            breath_watcher.reset()
                    signed_mm = standoff_step_sign * step_mm
                    current_target_rad = current_target_rad + mm_to_rad(signed_mm)
                    standoff_advanced_mm += signed_mm
                motor_bus.send(build_pos_vel_frame(MOTOR_NODE_ID, current_target_rad, current_velocity_rad_s))
                next_command_at = elapsed + command_period_s

            done = False
            for _stamp, can_id, data in sensor_bus.poll():
                if can_id != SENSOR_CAN_ID or len(data) < _PAYLOAD.size:
                    continue
                tof_mm, dist_cm, angle_centideg = _PAYLOAD.unpack(data[: _PAYLOAD.size])
                # Deflection from THIS run's measured rest, not from the firmware's boot-time
                # zero. Subtracting before taking the magnitude is what makes it work in
                # either direction, which matters because dist_cm's sign is arbitrary.
                # tactile_zero_cm is 0.0 under --no-tare, so this reduces to the old form.
                tactile_raw_mm = abs(dist_cm) * 10.0
                tactile_mm = abs(dist_cm - tactile_zero_cm) * 10.0
                in_contact = abs(dist_cm - tactile_zero_cm) > args.contact_threshold_cm

                if phase == "approach":
                    # Diagnostics, recorded whatever the outcome. "how close did the arm get"
                    # is what turned run 20260901-231950's useless "no contact within 45s" into
                    # a diagnosis, so the numbers are collected rather than reconstructed later.
                    approach_peak_deflection_mm = max(approach_peak_deflection_mm, tactile_mm)
                    if elapsed < APPROACH_BASELINE_S:
                        approach_baseline.append(tactile_mm)
                    if not approach_baseline_warned:
                        approach_baseline_warned = True
                        if in_contact:
                            # dist_cm is zeroed once at firmware boot and drifts between runs
                            # -- 0.002mm to 7.46mm across the runs of 2026-09-02. When it has
                            # drifted past the threshold, contact fires on this very first
                            # sample, there is no approach phase at all, and everything
                            # downstream is measuring from a datum that was never established.
                            # Run 20260902-151454 did exactly that at t=0.0003s and the base
                            # then drove 42.7mm during standoff. Warning only, by choice.
                            print(f"\nWARNING: tactile already reads {tactile_mm:.4f}mm, above the "
                                  f"{args.contact_threshold_cm * 10.0:.3f}mm contact threshold, "
                                  f"before any motion. The arm is pressed against something or the "
                                  f"firmware zero has drifted. Contact will fire immediately and "
                                  f"this run will have no real approach phase.\n")

                    if in_contact:
                        contact_t = elapsed
                        seat_rad = hold_basis_rad(elapsed)
                        current_target_rad = seat_rad
                        approach_extending = False
                        phase = "seat"
                        watcher.reset()
                        increments = 0
                        stable_count = 0
                        previous_amplitude = None
                        settling_until = elapsed + settle_time_s(args.creep_increment_mm, args.creep_speed_mm_s)
                        current_velocity_rad_s = creep_velocity_rad_s
                        print(f"\nCONTACT at t={elapsed:.3f}s dist_cm={dist_cm:.4f} "
                              f"after {approach_travelled_mm:.1f}mm -- entering seat phase")
                    elif approach_travelled_mm >= args.max_approach_mm - 1e-6 and arrived_at_target():
                        # The real failure, named. Faulting on distance the moment the cap is
                        # actually reached, instead of parking there and blaming a timeout 30s
                        # later, is the whole point of this branch.
                        baseline = (sum(approach_baseline) / len(approach_baseline)
                                    if approach_baseline else float("nan"))
                        threshold_mm = args.contact_threshold_cm * 10.0
                        fault_reason = (
                            f"extended to the {args.max_approach_mm:.1f}mm approach cap without "
                            f"contact (started at {args.travel_mm:.1f}mm, auto-extended "
                            f"{args.max_approach_mm - args.travel_mm:.1f}mm). Peak deflection "
                            f"{approach_peak_deflection_mm:.4f}mm = "
                            f"{100 * approach_peak_deflection_mm / threshold_mm:.0f}% of the "
                            f"{threshold_mm:.3f}mm contact threshold, against a "
                            f"{baseline:.4f}mm pre-contact baseline -- the arm is grazing at "
                            f"best. The phantom is further away than this cap reaches, or the "
                            f"tactile arm is not aligned with it."
                        )
                        done = True
                        break
                    elif elapsed >= approach_timeout_s:
                        fault_reason = (
                            f"no contact within {approach_timeout_s:.0f}s of approach, having "
                            f"travelled {approach_travelled_mm:.1f}mm of the "
                            f"{args.max_approach_mm:.1f}mm cap. The base is not reaching its "
                            f"commanded target -- check for a stall or a missing motor reply."
                        )
                        done = True
                        break

                elif phase == "seat":
                    if elapsed < settling_until:
                        pass  # parked, settling -- don't measure yet (matches ApproachState._seat)
                    else:
                        watcher.add(elapsed, tactile_mm)
                        if watcher.full:
                            # Two deliberately decoupled checks, each throttled differently.
                            # This is NOT a literal mirror of ApproachState._seat, and the
                            # deviation is evidence-based, found via offline simulation against
                            # a synthetic phantom (see docs/sessions -- to be written up) before
                            # trusting this on real hardware:
                            #
                            # 1. The grow/stable decision below fires at most once per full
                            #    window_s. The real ApproachState._seat recomputes it every
                            #    control tick once the window is full, with no cadence gate.
                            #    Simulating that literally showed it accepts a seat within 2-3
                            #    ticks (tens of milliseconds) of the window refilling post-creep,
                            #    because the windowed amplitude barely moves tick-to-tick -- "not
                            #    grown" trivially becomes true almost immediately regardless of
                            #    whether the true plateau was reached. That defeats the entire
                            #    point of stable_increments, which is meant to require several
                            #    independently-refreshed readings, not several milliseconds.
                            # 2. Trough checking (below) is deliberately NOT throttled the same
                            #    way -- it runs every tick once a creep is pending. Throttling
                            #    both together (an earlier version of this fix) was itself a bug:
                            #    a correctly-detected "still growing, should creep" signal could
                            #    sit unexecuted for a full window_s before the next trough check
                            #    even happened, at which point the amplitude (unchanged, because
                            #    no creep had occurred) looked falsely "stable."
                            if elapsed >= next_decision_at:
                                amplitude = watcher.amplitude
                                # ApproachState._seat also treats a saturated (clipped) reading
                                # as "not actually stable" even if it isn't growing, via
                                # ctx.geometry.is_tactile_clipped/tactile_saturation_mm -- both
                                # are calibration constants this script deliberately doesn't
                                # depend on (see module docstring). If the sensor's mechanical
                                # stroke is maxed out, this simpler check could accept a seat too
                                # early; watch the peak/trough printed at acceptance for a
                                # suspiciously round number.
                                grew = (previous_amplitude is None
                                        or (amplitude - previous_amplitude) > args.amplitude_tol_mm)
                                if grew:
                                    pending_creep = True
                                    stable_count = 0
                                else:
                                    pending_creep = False
                                    stable_count += 1
                                    if stable_count >= args.stable_increments:
                                        accepted_seat_rad = current_target_rad
                                        accepted_peak_mm = watcher.peak
                                        phase = "standoff"
                                        standoff_segments = 1
                                        standoff_stage = "coarse"
                                        standoff_advancing = True
                                        standoff_step_sign = 1.0
                                        watcher.reset()
                                        current_velocity_rad_s = creep_velocity_rad_s
                                        # current_target_rad stays at accepted_seat_rad -- the
                                        # command-send block above walks it forward continuously
                                        # from here while standoff_advancing is set.
                                        print(f"\nSEATED at t={elapsed:.3f}s after {increments} increment(s), "
                                              f"amplitude={amplitude:.3f}mm, peak={accepted_peak_mm:.3f}mm -- "
                                              f"seeking standoff")
                                previous_amplitude = amplitude
                                next_decision_at = elapsed + window_s

                                if phase == "seat" and increments >= args.max_seat_increments:
                                    fault_reason = (f"tactile amplitude never settled after {increments} increments "
                                                     f"(last {amplitude:.3f}mm, trough {watcher.trough:.3f}mm, "
                                                     f"peak {watcher.peak:.3f}mm)")
                                    done = True
                                    break

                            if phase == "seat" and pending_creep and at_trough(tactile_mm, watcher):
                                current_target_rad = current_target_rad + creep_rad
                                seat_rad = current_target_rad
                                increments += 1
                                settling_until = elapsed + settle_time_s(args.creep_increment_mm, args.creep_speed_mm_s)
                                watcher.reset()
                                pending_creep = False
                                next_decision_at = elapsed + window_s
                                print(f"  increment {increments}: creeping to {current_target_rad:.4f}rad")

                elif phase == "standoff":
                    # A closed loop on the tactile *reading*, never on base position, and the
                    # goal is that the SETTLED BREATHING PEAK lands on the target -- the sensor's
                    # maximum over a breath is what the standoff distance means.
                    #
                    # An instantaneous reading taken while moving cannot be used for that
                    # decision. Measured in run 20260901-161845: the base stopped with the
                    # reading at exactly 6.007mm and never moved again, yet the reading climbed
                    # to 8.27mm within three seconds. Two effects, both invisible instant to
                    # instant -- (a) the phantom's log shows the stop landed at the bottom of a
                    # full exhale, and one inhale later the same base position read 8.27mm;
                    # (b) at matched phantom positions before and after the stop the reading rose
                    # 1.39mm over five seconds with the motor stationary, the lever sinking
                    # further into the skin under sustained load. During the approach itself the
                    # reading even sat flat at ~5.97mm for two seconds while the base advanced
                    # 1.3mm, because the phantom was exhaling away at nearly the rate the base
                    # advanced. So: coarse motion may be gated on an instantaneous reading,
                    # acceptance may not.
                    if standoff_advancing:
                        # Coarse only: stop well short of the target and hand over to the fine
                        # loop. The headroom (half the target by default) exceeds the ~2.5mm of
                        # combined breathing swing + settling measured above, so coarse cannot
                        # overshoot even if it happens to stop at a trough. The fine stage's own
                        # motion is distance-bounded in the command block, not reading-gated.
                        if standoff_stage == "coarse" and tactile_mm >= target_mm * args.standoff_coarse_fraction:
                            standoff_stage = "fine"
                            standoff_advancing = False
                            current_target_rad = hold_basis_rad(elapsed)
                            settling_until = elapsed + settle_time_s(args.creep_increment_mm, args.creep_speed_mm_s)
                            breath_watcher.reset()
                            print(f"  standoff: coarse advance done at t={elapsed:.3f}s "
                                  f"(reading {tactile_mm:.3f}mm, {standoff_advanced_mm:.2f}mm travelled) "
                                  f"-- switching to fine step/settle/measure")
                    elif elapsed >= settling_until:
                        # settling_until already waited out the viscoelastic settling. The
                        # old extra gate -- two consecutive windows agreeing within tol --
                        # was meant to do the same job, but real breath-to-breath variation
                        # defeats it: only 44-56% of consecutive window pairs on this bench
                        # agree within 0.30mm, so it was a coin flip on every attempt rather
                        # than a settling test. Removed; the fixed timer does the real work.
                        breath_watcher.add(elapsed, tactile_mm)
                        if breath_watcher.ready:
                            peak_mm = breath_watcher.peak
                            standoff_recent_peaks.append(peak_mm)
                            lo = target_mm - args.standoff_tol_mm
                            hi = target_mm + args.standoff_tol_mm
                            if standoff_segments >= args.standoff_max_steps:
                                recent = ", ".join(f"{p:.3f}" for p in standoff_recent_peaks[-5:])
                                fault_reason = (
                                    f"standoff did not settle in {standoff_segments} fine "
                                    f"step(s) ({standoff_advanced_mm:.1f}mm advanced, "
                                    f"{standoff_retreat_steps} retreat(s)). Last peaks: "
                                    f"{recent} against band [{lo:.2f}, {hi:.2f}]mm. Either "
                                    f"--standoff-tol-mm {args.standoff_tol_mm:.2f} is tighter "
                                    f"than this subject's breath-to-breath spread "
                                    f"({breath_watcher.peak_spread:.3f}mm over the last "
                                    f"{args.min_breaths:g} breaths), or the target is out of "
                                    f"reach at this seating."
                                )
                                stop_motor("standoff did not settle")
                                done = True
                            elif peak_mm > hi:
                                # Overshot -- already pressed deeper than intended. Backing off
                                # is the correction (never the approach strategy: coarse/fine is
                                # what keeps this rare).
                                step = standoff_step_mm(peak_mm, target_mm, STANDOFF_RETREAT_SAFETY)
                                standoff_step_remaining_mm = step
                                standoff_step_sign = -1.0
                                standoff_advancing = True
                                standoff_segments += 1
                                standoff_retreat_steps += 1
                                breath_watcher.reset()
                                print(f"  standoff fine step {standoff_segments}: peak "
                                      f"{peak_mm:.3f}mm over {args.min_breaths:g} breaths, need "
                                      f"[{lo:.2f}, {hi:.2f}] ({peak_mm - hi:.2f}mm high) -- "
                                      f"RETREATING {step:.2f}mm  [retreats: {standoff_retreat_steps}]")
                            elif peak_mm >= lo:
                                # Accept and COMMIT. Nothing may move the base after this: the
                                # whole point of the hold is a stationary base under a breathing
                                # phantom, and a late correction would put a step transient into
                                # the middle of the record the estimator is scored on.
                                standoff_settled_peak_mm = peak_mm
                                standoff_crossed_t = elapsed
                                phase = "standoff_hold"
                                hold_started_at = elapsed
                                standoff_advancing = False
                                standoff_step_remaining_mm = 0.0
                                period = breath_watcher.period_s
                                period_note = f", breathing at {period:.2f}s" if period else ""
                                print(f"\nSTANDOFF reached at t={elapsed:.3f}s after {standoff_segments} "
                                      f"segment(s), peak={peak_mm:.3f}mm over {args.min_breaths:g} "
                                      f"breaths (target {target_mm:.1f} +/- {args.standoff_tol_mm:.2f}, "
                                      f"band [{lo:.2f}, {hi:.2f}]{period_note}) -- base now holding "
                                      f"and recording for {args.record_s:.0f}s")
                            else:
                                step = standoff_step_mm(peak_mm, target_mm, STANDOFF_ADVANCE_SAFETY)
                                standoff_step_remaining_mm = step
                                standoff_step_sign = 1.0
                                standoff_advancing = True
                                standoff_segments += 1
                                breath_watcher.reset()
                                print(f"  standoff fine step {standoff_segments}: peak "
                                      f"{peak_mm:.3f}mm over {args.min_breaths:g} breaths, need "
                                      f"[{lo:.2f}, {hi:.2f}] ({lo - peak_mm:.2f}mm low) -- "
                                      f"advancing {step:.2f}mm  [retreats: {standoff_retreat_steps}]")

                elif phase == "standoff_hold":
                    if hold_started_at is not None and (elapsed - hold_started_at) >= args.record_s:
                        done = True

                writer.write({
                    # "t" is the raw, un-rebased time.monotonic() reading -- what
                    # ct.phantom.driver.compare_logs needs to align this log against the
                    # phantom's own log on the same host (see that module's docstring and
                    # run_breathing_profile.py's matching fix). "elapsed" is rebased to this
                    # run's own start, kept only for human-readable plotting.
                    "t": now,
                    "elapsed": elapsed,
                    "tof_mm": tof_mm,
                    "dist_cm": dist_cm,
                    # tactile_mm is the TARED deflection -- the decision variable, and what
                    # the plots and ct-compare read. tactile_raw_mm is the old absolute form,
                    # kept so a run can still be compared against ones recorded before the
                    # firmware zero drifted, and so the drift itself stays visible.
                    "tactile_mm": tactile_mm,
                    "tactile_raw_mm": tactile_raw_mm,
                    "angle_deg": angle_centideg / 100.0,
                    "in_contact": in_contact,
                    "phase": phase,
                    "commanded_target_rad": current_target_rad,
                    "commanded_velocity_rad_s": current_velocity_rad_s,
                    "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                    "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                    "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                    "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                })
                if done:
                    break
            if done:
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C.")
    except SetupFailed:
        pass  # fault_reason is already set; reported and summarised below like any other fault
    finally:
        if motion_commanded:
            stop_motor("cleanup")
        writer.close()
        sensor_bus.close()
        motor_bus.shutdown()
        stop_phantom()

    if fault_reason:
        print(f"\nFAULT: {fault_reason}")

    summary = {
        "phase_reached": phase,
        "fault_reason": fault_reason,
        "contact_t": contact_t,
        "tare": {
            "applied": args.tare and tare is not None,
            "window_s": args.tare_s if args.tare else None,
            "zero_mm": (tare or {}).get("zero_mm"),
            "std_mm": (tare or {}).get("std_mm"),
            "p2p_mm": (tare or {}).get("p2p_mm"),
            "samples": (tare or {}).get("n"),
            "already_in_contact_warning": tare_contact_warned,
            "note": (
                "dist_cm is zeroed once at FIRMWARE boot and that zero drifts: 0.0048mm at "
                "rest on 2026-09-01 16:54, 0.1967mm on 2026-09-02 15:46, while its std stayed "
                "at 0.002-0.014mm. The reading is clean; the datum moves. Once the drift "
                "passed the contact threshold, every run declared contact on its first sample "
                "and had no APPROACH phase at all. zero_mm is what was subtracted this run. "
                "A p2p above ~0.3mm over the tare window means the arm was already riding the "
                "phantom's breathing -- i.e. in real contact, which the tare then hides; that "
                "is warned about, not prevented."
            ),
        },
        "approach": {
            "travelled_mm": approach_travelled_mm,
            "initial_target_mm": args.travel_mm,
            "extended_mm": max(0.0, approach_travelled_mm - args.travel_mm),
            "max_approach_mm": args.max_approach_mm,
            "peak_deflection_mm": approach_peak_deflection_mm,
            "baseline_deflection_mm": (sum(approach_baseline) / len(approach_baseline)
                                        if approach_baseline else None),
            "contact_threshold_mm": args.contact_threshold_cm * 10.0,
            "peak_fraction_of_threshold": (
                approach_peak_deflection_mm / (args.contact_threshold_cm * 10.0)
                if args.contact_threshold_cm > 0 else None
            ),
            "travel_note": (
                "travelled_mm is total commanded forward travel, including any auto-extension "
                "past initial_target_mm. Both runs before this field existed consumed exactly "
                "40mm, which was the whole travel budget at the time -- 20260901-165415 "
                "contacted at 40.01mm with a 6% margin over the threshold, and 20260901-231950 "
                "reached 57% of it and faulted. If travelled_mm is at max_approach_mm the base "
                "ran out of room, which is a distance problem and not a timing one."
            ),
            "deflection_note": (
                "peak_deflection_mm against baseline_deflection_mm is how to tell a real "
                "contact from the arm grazing. Measured in 20260901-231950: baseline 0.0031mm "
                "(std 0.0067), peak 0.0565mm, settled mean 0.0246mm -- only 3.2 sigma, and the "
                "pre-contact MAXIMUM of 0.0255mm already equalled the post-contact mean. That "
                "is why the threshold was left at 0.1mm rather than lowered: a threshold low "
                "enough to catch that grazing contact would have fired before it."
            ),
        },
        "seat": {
            "increments": increments,
            "accepted_seat_rad": accepted_seat_rad,
            "accepted_peak_tactile_mm": accepted_peak_mm,
        },
        "standoff": {
            "crossed_t": standoff_crossed_t,
            "standoff_dist_cm": args.standoff_dist_cm,
            "increments": standoff_segments,
            "advanced_mm": standoff_advanced_mm,
            "settled_peak_mm": standoff_settled_peak_mm,
            "final_stage": standoff_stage,
            "retreat_steps": standoff_retreat_steps,
            "max_steps": args.standoff_max_steps,
            "accept_band_mm": [
                args.standoff_dist_cm * 10.0 - args.standoff_tol_mm,
                args.standoff_dist_cm * 10.0 + args.standoff_tol_mm,
            ],
            "peaks_measured_mm": standoff_recent_peaks,
            "breath_period_s": breath_watcher.period_s,
            "breath_spread_mm": breath_watcher.peak_spread,
            "breath_note": (
                "breath_period_s is the period actually detected, against the configured "
                "nominal_breath_s. They differed by 38% on 2026-09-03 (5.51s real vs 4.0s "
                "nominal), which is why the peak is now measured over counted breaths rather "
                "than over min_breaths*nominal_breath_s seconds. breath_spread_mm is the "
                "subject's own breath-to-breath variation over the accepted measurement, and "
                "is the floor on how tightly standoff can position: a tolerance below it "
                "cannot be met reliably at any seating."
            ),
            "settled_peak_note": (
                "settled_peak_mm is the breathing PEAK measured over a full window with the base "
                "stationary and settled (two consecutive windows agreeing within --standoff-tol-mm) "
                "-- this is what the standoff target means, and it should read close to "
                "standoff_dist_cm*10. An instantaneous reading taken while moving is not a valid "
                "substitute: run 20260901-161845 stopped at exactly 6.007mm and settled at 8.58mm."
            ),
            "compliance_ratio_note": (
                "advanced_mm is how far the base actually travelled during standoff to raise the "
                "reading from seat's accepted peak to the target. Dividing the reading change by "
                "advanced_mm gives this run's compliance ratio (measured ~0.22-0.50mm of reading "
                "per mm of travel) -- it varies with skin consistency and depth, which is why "
                "standoff closes the loop on the reading instead of computing a travel distance."
            ),
        },
        "record_s": args.record_s,
        "phantom_driven": not args.no_phantom,
        "phantom_profile": args.profile if not args.no_phantom else None,
        "motor_replies_seen": motor_replies_seen,
        "motor_reply_note": (
            "count of decoded GL-II feedback frames (ID 0) seen during this run -- per-command "
            "ACK, not a continuous broadcast, roughly one per --command-hz resend."
        ),
        "motor_error_note": (
            "the logged motor_error field is CubeMarsMIT.parse()'s high-nibble split of data[0] "
            "applied to a GL-II frame, which does NOT use MIT's node|fault nibble convention. It "
            "reads 1 from ~0.8s after ENTER_MODE for the rest of every run, including every "
            "successful one, so it is a normal 'enabled/active' status bit, not a fault -- and 1 "
            "is not even a defined MIT fault code (MIT_FAULT_CODES covers 0x8-0xE). Logged as raw "
            "data; do NOT treat a nonzero value here as an error without decoding GL-II properly."
        ),
        "dist_cm_sign_note": (
            "dist_cm's sign is arbitrary (zeroed at firmware boot, unclamped) -- every threshold "
            "check in this script uses abs(dist_cm). Confirm the actual sign at the bench before "
            "trusting a directional reading."
        ),
        "params": {
            "travel_mm": args.travel_mm,
            "velocity_rad_s": args.velocity,
            "contact_threshold_cm": args.contact_threshold_cm,
            "standoff_dist_cm": args.standoff_dist_cm,
            "creep_increment_mm": args.creep_increment_mm,
            "creep_speed_mm_s": args.creep_speed_mm_s,
            "min_breaths": args.min_breaths,
            "nominal_breath_s": args.nominal_breath_s,
            "amplitude_tol_mm": args.amplitude_tol_mm,
            "standoff_coarse_fraction": args.standoff_coarse_fraction,
            "standoff_tol_mm": args.standoff_tol_mm,
            "standoff_max_steps": args.standoff_max_steps,
            "allow_loaded_tare": args.allow_loaded_tare,
            "stable_increments": args.stable_increments,
            "max_seat_increments": args.max_seat_increments,
        },
    }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")
    if phase == "standoff_hold" and not fault_reason:
        print(f"plot with: python scripts/plot_approach_and_seat.py --run {out_dir}")
        if not args.no_phantom:
            print(f"compare against ground truth with: ct-compare {out_dir / 'phantom' / 'samples.jsonl'} {jsonl_path}")

    return 0 if not fault_reason else 1


if __name__ == "__main__":
    raise SystemExit(main())
