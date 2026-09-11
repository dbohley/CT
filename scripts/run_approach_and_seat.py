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

**With --insert-needle (session 022), four more phases follow.** Once standoff_hold's
recording is long enough, the tactile stream identifies and seeds a real EKF
(``ct.identification``/``ct.tracking``, same identifier/tracker as ``configs/rig_bench.yaml``)
-- literally "the EKF has had time to generate itself while the base is seated". The needle
then floats (zero-torque disable -- see the needle-config constants below for why this needs
no MIT mode) until ``ct.control.gate.FiringGate`` fires at end-exhale, drives a
``--needle-breakthrough-mm`` (2cm default) breakthrough capped at ``--needle-velocity-mm-s``
using the real, hardware-validated ``ct.control.servo.LeadServo`` (session 019/021's measured
plant and lead), then floats again and attempts one more ``--needle-advance-mm`` gated push the
same way. Unlike the real ``InsertState``/``AdvanceState``, targets are relative to the
needle's own measured position, not an absolute skin depth -- this script deliberately has no
skin-position geometry calibration (see below), so "2cm in" means 2cm past wherever the needle
was when the gate fired.

**Both** drives are gated the same way, by the same rule: each aborts into float and retries at
the next end-exhale window (``--max-breakthrough-attempts`` / ``--max-advance-attempts``) when
the raw tactile reading returns to whatever it was at drive start, having first descended
``--abort-margin-mm`` below it. See ``drive_abort_decision``. The breakthrough used to run
uninterrupted, which mattered: measured on runs 7 and 8 it took 3.63s and 3.67s against a ~4.0s
breath, so half of every breakthrough happened during inhale -- exactly what the gate exists to
prevent. It is not free, but it is nearly so: replaying this rule over those same two drives,
the window closes at 1.691s/1.746s with the needle already 17.97mm/18.09mm of 20mm in, so the
expected cost is one extra window, not many.

**insertion_hold** then **needle_retract** end the run: the needle floats in place for
``--post-insert-hold-s``, then ramps smoothly back to the position it held when the *first*
breakthrough gate fired -- fully out -- at ``--retract-speed-mm-s``. The base never moves
(standoff's accept is a hard commit). ``--no-retract`` skips both and leaves the needle where
it is, as this script did before. An attempts-exhausted fault on either drive still routes
through hold+retract before exiting nonzero, so a failed run does not leave the needle
embedded; every other fault stops immediately.

Off by default -- with no flag this script behaves exactly as it always has, and existing
callers (``scripts/collect_param_sweep_runs.py``) see no change.

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
import json
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import can
import numpy as np

from ct.cli._common import save_json
from ct.control.gate import FiringGate, cycle_extrema
from ct.control.live import AmplitudeWatcher, BreathPeakWatcher, SignalAccumulator
from ct.control.servo import LeadServo
from ct.hw.bus import build_bus_from_config
from ct.hw.config import AxisServoConfig, BusConfig, LatencyConfig, LeadCompensator, PlantModel
from ct.hw.motors.cubemars_mit import CubeMarsMIT
from ct.identification.spectral import bpm_to_omega, omega_to_bpm
from ct.registry import build_identifier, build_tracker
from ct.rt.latency import LatencyBudget
from ct.run import resolve_tracker_params
from ct.rt.telemetry import TelemetryWriter
from ct.tracking.measurement import measurement as tracked_measurement

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

# ---------------- Needle motor config (session 022 -- shares the base's bus/channel; only the ----------
# ---------------- node id and drum radius differ, matching scripts/run_needle_lead_tracking_live.py) ---
# Floating (session 004's settled finding, restored to CLAUDE.md's Open Questions in session 022):
# the compliance requirement -- the needle must not stall between insertion attempts -- needs only
# zero-torque disable, which this protocol already provides via the SAME universal EXIT_MODE
# command sent at cleanup by every needle script this project has -- no MIT mode, no separate
# "float" primitive. Floating here means: send EXIT_MODE once; resuming active control means
# CLEAR_ERRORS+ENTER_MODE plus a fresh position read, the same startup sequence every needle
# script already uses. The one thing genuinely new is doing this more than once in a single run
# (every prior script does it exactly once at start, once at cleanup) -- verify with the isolated
# toggle smoke test before trusting anything built on top of it (see module docstring/plan).
NEEDLE_NODE_ID = 2  # needle motor's labeled CAN node ID (base is 3, above)
NEEDLE_DRUM_RADIUS_M = 0.018  # capstan drive -- see scripts/run_needle_step_response.py
NEEDLE_DIRECTION_SIGN = -1  # +rad retracts (away from phantom), same convention as needle scripts
NEEDLE_COMMAND_HZ = 200.0  # matches configs/rig_bench.yaml's procedure.loop_rate_hz -- session
# 021 found the lead compensator oscillates badly at 20Hz and works as designed at 200Hz; this
# is the real, validated operating rate, not a bench-script convenience default.
NEEDLE_COMMAND_PERIOD_S = 1.0 / NEEDLE_COMMAND_HZ
NEEDLE_INITIAL_REPLY_WAIT_S = 0.5

# Plant/lead: session 019/021's measured, hardware-validated values (configs/rig_bench.yaml's
# servo.needle block) -- not re-derived here, just reused.
NEEDLE_PLANT = PlantModel(K=0.92, wn=27.0, zeta=0.35)
NEEDLE_LEAD = LeadCompensator(zero=1.0, pole=5.0, gain=12.74)
NEEDLE_SERVO_CONFIG = AxisServoConfig(
    plant=NEEDLE_PLANT, lead=NEEDLE_LEAD,
    correction_limit_mm=2.0, correction_rate_limit_mm_s=5.0,
)
# Session 022 live test: this same config, reused for the insert_drive/advance_drive step
# insertions, saturated 77 and rate-limited 294 of ~600 ticks across one breakthrough+advance
# pair, and the raw position trace showed a fast ~19mm ramp followed by a near-stalled crawl
# that never reached the full 20mm target -- arriving on the 0.5mm tolerance, not true arrival.
# 2.0mm/5.0mm-s were sized for tracking a small, continuously-updating reference, not closing a
# single ~20mm step target against real phantom resistance. UNVALIDATED FIRST GUESS below, same
# status exhale_band_frac/inhale_abort_frac had before real data -- tune at the bench via
# --insertion-correction-limit-mm/--insertion-correction-rate-limit-mm-s.
NEEDLE_INSERTION_CORRECTION_LIMIT_MM = 10.0
NEEDLE_INSERTION_CORRECTION_RATE_LIMIT_MM_S = 40.0
NEEDLE_INSERTION_SERVO_CONFIG = AxisServoConfig(
    plant=NEEDLE_PLANT, lead=NEEDLE_LEAD,
    correction_limit_mm=NEEDLE_INSERTION_CORRECTION_LIMIT_MM,
    correction_rate_limit_mm_s=NEEDLE_INSERTION_CORRECTION_RATE_LIMIT_MM_S,
)

# Identifier/tracker: matches configs/rig_bench.yaml's identifier/tracker blocks exactly, so this
# bench script's EKF is the same one everything else in the project uses, not a reimplementation.
IDENTIFIER_NAME = "fft_harmonic"
IDENTIFIER_PARAMS = {"Kmax": 6, "energy_threshold": 0.95, "p0_inflation": 2.0}
# q_scale: real subject breathing is why this is here at all. Unscaled Q (the effective default,
# identical to q_scale=1.0) puts this EKF into a degenerate mode on moira -- theta stops
# advancing and A_1 diverges, see watch_phase_advance. The gate then cannot fire at all, while
# y_pred still fits the sensor to 0.125mm RMSE and nothing else looks wrong.
#
# The transition is SHARP, and it is a two-state switch rather than a dial. Measured over the
# whole 693s of outputs/needle_gating_live/moira/2, theta's real advance against a true
# 1.4357 rad/s:
#
#     q_scale   1.0     0.5     0.2  |  0.1     0.05    0.02    0.01
#     theta   0.012   0.009   0.006  | 1.442   1.465   1.464   1.476
#     A_1 end  8.33    8.47    8.47  |  1.04    1.06    1.08    1.07
#
# 0.2 was tried first and looked fine on a 74s window -- it is on the wrong side of the cliff and
# fails on a long run. 0.05 is two notches clear of the boundary and still adapts fast enough for
# a subject whose rate drifts (session 006 measured derek at 20.67 -> 14.00 bpm within one take).
# With it, the gate fires 125 times at a 4.29s cadence on moira/2 against a 4.13s period -- one
# bite per breath -- where q_scale=0.2 fired once in 693s. All three sinusoid runs are bit-for-bit
# unchanged at every value tested (8/4/5 fires, 4.01-4.04s cadence), so this is not a
# moira-specific workaround. Note bench_aligned.yaml's 0.5 is on the failing side.
IDENTIFIER_PARAMS["q_scale"] = 0.05
TRACKER_NAME = "harmonic_ekf"
# omega_bounds_fraction: CLAUDE.md session 018's 10-run sweep found omega_bounds at +-10% of
# Stage 1's own rate is what holds frequency lock (q_scale alone held 2/10 runs; the pair held
# 10/10). This script never adopted it, and run 7 paid for that: resuming the tracker after the
# drive+settle pause collapsed omega from 1.750 to 0.932 rad/s in ONE step against a true 1.565,
# leaving the model near half frequency and antiphase, so the gate fired at peak inhale while
# believing it was firing at the trough. Resolved per-run against bpm_hat by
# ct.run.resolve_tracker_params, since a fixed rad/s range cannot cover subjects at 10-22bpm.
TRACKER_PARAMS = {"joseph": True, "wrap_phases": True, "omega_bounds_fraction": 0.1}
OMEGA_DEVIATION_WARN_FRAC = 0.25  # warn once when tracked omega strays this far from Stage 1's
# rate -- run 7's collapse sat in the telemetry the whole time with nothing flagging it.
# Degeneracy watch. The EKF can fit the breath by wiggling theta on the steep part of the sine
# instead of sweeping it, which makes A_k diverge and cycle_extrema's band meaningless -- see
# watch_phase_advance for the measurements. Warn when theta's real advance falls below this
# fraction of omega, judged over a window long enough to average out the per-breath wobble.
# The healthy case is ~1.0 (measured 1.442 against omega 1.352 on moira/2 once fixed); the
# degenerate case measured 0.0044. Anything below half is unambiguous.
PHASE_ADVANCE_MIN_FRAC = 0.5
PHASE_ADVANCE_CHECK_S = 30.0
# One cap for a gated drive, expressed in breaths rather than seconds so it travels between
# subjects. The symmetric-return rule in drive_abort_decision is what actually ends a drive; this
# exists only for the case where the sensor never returns at all, and must never be able to
# compete with it -- a "backstop" that can fire first is how the retired model ceiling ended up
# deciding every abort in session 022. The old fixed 3.0s was sized on the sinusoid's 4.0s breath
# and fired as drive_overrun on moira's 4.65s one, capping the breakthrough at 19.2 of 20mm.
DEFAULT_MAX_DRIVE_BREATHS = 0.75
GATE_STALL_BREATHS = 4.0  # warn once when the gate has refused for this many breaths straight --
# the refractory is 0.9 breaths, so anything past ~2 is already abnormal and 4 cannot false-fire
# on a merely unlucky cycle. See watch_gate_stall.

# Latency budget: matches configs/rig_bench.yaml's latency block. tau_cl(omega_r) is computed
# from NEEDLE_SERVO_CONFIG via LeadServo.residual_lag, not this fallback -- see LatencyBudget.
# measure_tau_c=False: this bench script never calls LatencyBudget.compute.record(), so a live
# measurement would just silently always return the same configured fallback anyway -- False
# says so honestly instead of implying a measurement that never happens.
NEEDLE_LATENCY_CONFIG = LatencyConfig(tau_s=0.02, tau_c=0.005, T_ins=0.15, tau_cl_fallback=0.05,
                                      measure_tau_c=False)

DEFAULT_NEEDLE_BREAKTHROUGH_MM = 20.0
DEFAULT_NEEDLE_ADVANCE_MM = 20.0
DEFAULT_NEEDLE_VELOCITY_MM_S = 60.0
DEFAULT_EXHALE_BAND_FRAC = 0.15  # matches configs/rig_bench.yaml's procedure.insert/advance --
# reusing the number already carried through simulation, not inventing a new one for this script.
DEFAULT_MAX_FORECAST_STD_MM = 0.5
# --inhale-abort-frac is RETIRED -- accepted so an existing bench command line keeps running, but
# ignored and warned about. See drive_abort_decision()'s docstring for why it could never be a
# "backstop": it decided every abort in run 6 (8ms drives) precisely because it can fire before
# the real rule arms.
# The old DEFAULT_MAX_ADVANCE_DRIVE_S = 3.0 is gone. It took over the one legitimate job the
# model ceiling had, but it was sized on the sinusoid's 4.0s breath and does not travel: on
# moira's 4.65s breath it fired as drive_overrun and capped the breakthrough at 19.2 of 20mm.
# The cap is now DEFAULT_MAX_DRIVE_BREATHS x the tracked period, resolved by max_drive_s().
DEFAULT_ABORT_MARGIN_MM = 0.2  # arms/guards the raw-sensor symmetric-return rule. Comfortably
# above this sensor's measured noise (std 0.002-0.014mm at rest, session 012) and tiny against a
# ~5mm breathing excursion, so it confirms a real descent without being tripped by jitter.
DEFAULT_MAX_ADVANCE_ATTEMPTS = 5
# Higher than the advance's 5 on measured grounds, not caution: replaying drive_abort_decision
# over runs 7 and 8's recorded breakthroughs, the symmetric window closes at 1.691s/1.746s with
# the needle 17.97mm/18.09mm of 20mm in -- so ~2 windows is the expected cost, and the tail (the
# last ~1.5mm took another 1.9s of servo crawl against a phantom that is currently too thick) is
# precisely the part that varies with how tough the surface is.
DEFAULT_MAX_BREAKTHROUGH_ATTEMPTS = 8
DEFAULT_MONITOR_AFTER_S = 30.0  # how long a --monitor-s run keeps watching after the disturbance
# it was told to expect. Everything past this is watching a settled filter, and it is what made
# the first version of this experiment ~7 minutes a run.
DEFAULT_POST_INSERT_HOLD_S = 5.0  # needle floats, fully inserted, before the retract begins. Also
# the one genuinely stationary in-contact segment this script produces -- a future in-contact R
# measurement (CLAUDE.md's standing open question) could come from here, though nothing uses it yet.
DEFAULT_RETRACT_SPEED_MM_S = 5.0  # deliberately far below --needle-velocity-mm-s (60): withdrawal
# has no timing requirement at all, so it is ramped slowly rather than stepped.
DEFAULT_REARM_SLEEP_S = 0.6  # UNVERIFIED GUESS (0.1 CLEAR_ERRORS + 0.5 ENTER_MODE, historically)
# -- nothing has ever confirmed the needle motor needs any pause here at all. Drives BOTH the
# real sleep in reenter_needle_mode()/the initial needle bring-up AND the gate's horizon
# compensation below, from one value, so the two can never disagree. Pass 0.0 to test the
# hypothesis that no pause is needed; see the module docstring's re-arm section.
NEEDLE_ARRIVAL_TOL_MM = 0.5  # matches InsertConfig/AdvanceConfig's own defaults
NEEDLE_STALL_VELOCITY_MM_S = 0.3
NEEDLE_STALL_TIME_S = 0.4
NEEDLE_DRIVE_TIMEOUT_S = 60.0
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
_PAYLOAD = struct.Struct("<ff")  # float32 tof_mm, float32 dist_cm (angle field removed by firmware)
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
            _tof_mm, dist_cm = _PAYLOAD.unpack(data[: _PAYLOAD.size])
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


def drive_abort_decision(
    sensor_mm: float | None,
    reference_mm: float | None,
    descended: bool,
    margin_mm: float,
    drive_elapsed_s: float,
    max_drive_s: float,
) -> tuple[str | None, bool]:
    """Should this gated needle drive stop and float? Returns ``(rule, descended)``.

    Governs **both** gated drives -- ``insert_drive`` (breakthrough) and ``advance_drive``. It was
    written for the advance alone in session 022 and generalised unchanged: nothing in it is
    specific to which drive is running, because it reads only the raw sensor.

    ``rule`` is ``None`` to keep driving, else the name of the rule that tripped.

    **Raw sensor only -- deliberately model-free.** The drive references the sensor reading it
    started from: it must first descend ``margin_mm`` below that (proving it really is heading
    into the trough, which *arms* the return test), and then floats again the moment the reading
    climbs back to that same starting value. The active window therefore straddles the trough
    roughly symmetrically whatever the waveform's shape -- which matters because real subject
    profiles are not sinusoids.

    An earlier version also carried a model-based ceiling (``value(t)`` against a fraction of the
    EKF's swept band) as a parallel "backstop". It was neither: the gate fires on
    ``forecast(t + h)`` while that ceiling tested ``value(t)``, and during a descent ``value(t)``
    is *always* above ``forecast(t + h)`` -- that gap IS the horizon. So it tripped the instant a
    drive started, before this rule could arm, and made every abort decision in run 6 (measured:
    8ms drives). Its one legitimate job -- bounding a drive whose sensor never returns to
    reference, e.g. if DC drifts down as the needle embeds -- is now ``max_drive_s``, which is
    deterministic and cannot preempt the primary rule.

    Pure and side-effect free so the offline replay in the verification step exercises exactly
    this code, not a re-implementation of it.
    """
    if sensor_mm is not None and reference_mm is not None:
        if not descended and sensor_mm <= reference_mm - margin_mm:
            descended = True
        if descended:
            if sensor_mm >= reference_mm:
                return "symmetric_return", descended
        elif sensor_mm >= reference_mm + margin_mm:
            return "mistimed_rise", descended
    if drive_elapsed_s >= max_drive_s:
        return "drive_overrun", descended
    return None, descended


# Onset time in PROFILE seconds, from the sidecar beside --profile, or None if there isn't one.
# Set once at startup by load_disturbance_onset so the hot loop never touches the filesystem.
_DISTURBANCE_ONSET_S: float | None = None


def load_disturbance_onset(profile: str) -> float | None:
    """Read ``<profile>.onset.json`` if it exists. Absent for every ordinary profile."""
    global _DISTURBANCE_ONSET_S
    sidecar = Path(profile).with_suffix("").with_suffix(".onset.json")
    if not sidecar.exists():
        sidecar = Path(str(Path(profile).with_suffix("")) + ".onset.json")
    if sidecar.exists():
        try:
            _DISTURBANCE_ONSET_S = float(json.loads(sidecar.read_text())["onset_s"])
        except (OSError, ValueError, KeyError):
            _DISTURBANCE_ONSET_S = None
    return _DISTURBANCE_ONSET_S


def expected_disturbance_elapsed(t0: float, out_dir: Path) -> float | None:
    """When this run's profile will step, on the run's own elapsed axis, or None.

    Needs two things, both exact rather than assumed:

    - ``<profile>.onset.json``, written beside the CSV by
      scripts/generate_disturbance_profiles.py, giving the onset in PROFILE time.
    - when the phantom actually began playing profile time zero. That is not the subprocess
      launch: run_breathing_profile.py homes, zeroes and ramps first, and how long that takes is
      not knowable from here. Its log's FIRST sample is the first commanded profile point, and it
      is stamped with raw time.monotonic() on this same host -- so it is the honest answer, and it
      is already on disk by the time this is called.
    """
    global _DISTURBANCE_ONSET_S
    if _DISTURBANCE_ONSET_S is None:
        return None
    phantom_log = out_dir / "phantom" / "samples.jsonl"
    if not phantom_log.exists():
        return None
    try:
        with phantom_log.open() as handle:
            first = json.loads(handle.readline())
    except (OSError, ValueError):
        return None
    playback_start = float(first["t"]) - t0
    return playback_start + _DISTURBANCE_ONSET_S


def describe_abort(rule: str, sensor_mm: float | None, reference_mm: float | None,
                    trough_mm: float | None, elapsed: float, drive_elapsed_s: float,
                    args) -> str:
    """Human-readable reason for one of drive_abort_decision's three rules.

    Shared by both gated drives so the two cannot drift into describing the same rule
    differently -- the breakthrough and the advance trip identical rules for identical reasons.
    """
    if rule == "symmetric_return":
        return (f"sensor returned to its drive-start reading ({sensor_mm:.3f} >= "
                f"{reference_mm:.3f}mm) after reaching a trough of {trough_mm:.3f}mm -- "
                f"symmetric window complete at t={elapsed:.3f}s")
    if rule == "mistimed_rise":
        return (f"sensor rose {sensor_mm - reference_mm:+.3f}mm above its drive-start reading "
                f"({reference_mm:.3f}mm) without ever descending {args.abort_margin_mm:.2f}mm "
                f"first -- drive was mistimed onto a rise, bailing at t={elapsed:.3f}s")
    return (f"drive ran {drive_elapsed_s:.2f}s without the sensor returning to its start "
            f"reading ({reference_mm}mm, trough {trough_mm}mm) -- hit the drive cap at "
            f"t={elapsed:.3f}s")


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


NEEDLE_PROBE_VELOCITY_RAD_S = 0.05  # matches scripts/read_needle_position.py's convention


def _read_fresh_needle_position_rad(motor_bus, motor_reply_codec, probe_rad: float) -> float | None:
    """Position from a reply to a freshly-sent probe command, not one already queued.

    Unlike the base motor (already being commanded regularly whenever a read like this is
    needed elsewhere in this file), the needle may have been floating (EXIT_MODE'd, no
    commands in flight) since the last time it was read -- this motor only ACKs a reply per
    command sent (session 005), so a probe has to be sent first. ``probe_rad`` must be the
    best known estimate of where the needle actually is (the caller's own last real reply, or
    0.0 only on the very first-ever read of a run) -- probing at a stale or wrong position
    would command a real, unwanted move at NEEDLE_PROBE_VELOCITY_RAD_S before the fresh reply
    corrects it.
    """
    for _ in range(RETRACT_FLUSH_MAX):
        if motor_bus.recv(timeout=0.0) is None:
            break
    motor_bus.send(build_pos_vel_frame(NEEDLE_NODE_ID, probe_rad, NEEDLE_PROBE_VELOCITY_RAD_S))
    deadline = time.monotonic() + RETRACT_POSITION_TIMEOUT_S
    while time.monotonic() < deadline:
        msg = motor_bus.recv(timeout=0.05)
        if msg is None:
            continue
        parsed = motor_reply_codec.parse(msg.arbitration_id, bytes(msg.data))
        if parsed is not None and int(parsed["node_id"]) == NEEDLE_NODE_ID:
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
    parser.add_argument("--insert-needle", dest="insert_needle", action="store_true",
                         help="session 022: after standoff_hold's recording window, identify+seed "
                              "an EKF from it, then drive the needle -- a 2cm breakthrough once "
                              "the tracker fires at end-exhale, then one more 2cm gated push. Off "
                              "by default: this script otherwise never commands the needle motor "
                              "at all, and existing callers (scripts/collect_param_sweep_runs.py) "
                              "must see no behavior change.")
    parser.add_argument("--monitor-s", type=float, default=0.0, dest="monitor_s",
                         help="session 023: after standoff_hold's recording, identify+seed the EKF "
                              "exactly as --insert-needle does, then just WATCH for this many "
                              "seconds -- the needle is never commanded and nothing moves. For "
                              "measuring how long the estimator takes to notice a disturbance in "
                              "the breathing it is tracking; see "
                              "scripts/generate_disturbance_profiles.py and "
                              "scripts/analyze_disturbance_detection.py. 0 disables it.")
    parser.add_argument("--monitor-after-s", type=float, default=DEFAULT_MONITOR_AFTER_S,
                         dest="monitor_after_s",
                         help="with --monitor-s, if the profile has a <name>.onset.json sidecar "
                              "(scripts/generate_disturbance_profiles.py writes one), stop this "
                              "many seconds after the disturbance instead of running the full "
                              "--monitor-s. That is what keeps a run ~4.5min rather than ~7, "
                              "since everything after the disturbance is just watching")
    parser.add_argument("--needle-breakthrough-mm", type=float, default=DEFAULT_NEEDLE_BREAKTHROUGH_MM,
                         dest="needle_breakthrough_mm",
                         help="the initial insertion, relative to the needle's own position when "
                              "the gate fires -- this script has no skin-position calibration "
                              "(deliberately, see module docstring), so unlike the real InsertState "
                              "this is relative to measured position, not an absolute skin depth")
    parser.add_argument("--needle-advance-mm", type=float, default=DEFAULT_NEEDLE_ADVANCE_MM,
                         dest="needle_advance_mm", help="the one gated push after breakthrough")
    parser.add_argument("--needle-velocity-mm-s", type=float, default=DEFAULT_NEEDLE_VELOCITY_MM_S,
                         dest="needle_velocity_mm_s", help="velocity cap for both needle drives")
    parser.add_argument("--rearm-sleep-s", type=float, default=DEFAULT_REARM_SLEEP_S,
                         dest="rearm_sleep_s",
                         help="pause after CLEAR_ERRORS+ENTER_MODE before reading position and "
                              "driving -- UNVERIFIED GUESS, not a measured requirement. Also "
                              "feeds the gate's horizon compensation for this same delay, so the "
                              "two can never disagree. Pass 0.0 to test whether any pause is "
                              "needed at all.")
    parser.add_argument("--exhale-band-frac", type=float, default=DEFAULT_EXHALE_BAND_FRAC,
                         dest="exhale_band_frac",
                         help="fraction of the tracked waveform's excursion, from its minimum, "
                              "that counts as end-exhale -- see ct.control.gate")
    parser.add_argument("--max-forecast-std-mm", type=float, default=DEFAULT_MAX_FORECAST_STD_MM,
                         dest="max_forecast_std_mm",
                         help="firing gate's confidence budget -- see ct.control.gate")
    parser.add_argument("--q-scale", type=float, default=IDENTIFIER_PARAMS["q_scale"],
                         dest="q_scale",
                         help="process-noise scale for the EKF. Below ~0.2 the tracked "
                              "amplitude stays bounded on real subject breathing; at the "
                              "unscaled default it runs away and silently deadlocks the firing "
                              "gate (see IDENTIFIER_PARAMS' comment). The transition is sharp, "
                              "so treat this as a two-state switch, not a dial")
    parser.add_argument("--inhale-abort-frac", type=float, default=None,
                         dest="inhale_abort_frac",
                         help="RETIRED and ignored -- the raw-sensor rule (--abort-margin-mm) "
                              "replaced it. Still accepted so an existing command line runs; "
                              "passing it prints a warning and changes nothing")
    parser.add_argument("--max-drive-breaths", type=float, default=DEFAULT_MAX_DRIVE_BREATHS,
                         dest="max_drive_breaths",
                         help="cap on one gated drive, in breaths of whatever the tracker is "
                              "currently seeing. Only for the case where the sensor never returns "
                              "to its start reading at all; the symmetric-return rule is what "
                              "normally ends a drive, and this must never preempt it")
    parser.add_argument("--max-advance-drive-s", type=float, default=None,
                         dest="max_advance_drive_s",
                         help="OVERRIDE the cap above with a fixed number of seconds. Replaces it "
                              "outright -- there is no second ceiling and no min() of the two, so "
                              "exactly one number is the cap at any instant. Leave unset unless "
                              "the tracked rate is untrustworthy")
    parser.add_argument("--abort-margin-mm", type=float, default=DEFAULT_ABORT_MARGIN_MM,
                         dest="abort_margin_mm",
                         help="raw-sensor abort rule: the drive must first descend this far below "
                              "its start reading (confirming it is heading into the trough), after "
                              "which it floats again as soon as the reading returns to that start "
                              "value -- symmetric about the trough. Also bails immediately if the "
                              "reading instead rises this far, meaning the drive was mistimed")
    parser.add_argument("--max-advance-attempts", type=int, default=DEFAULT_MAX_ADVANCE_ATTEMPTS,
                         dest="max_advance_attempts",
                         help="give up the gated advance (fault) after this many inhale-aborted "
                              "attempts, mirroring AdvanceState.max_increments' role. The run "
                              "still holds and retracts before exiting nonzero")
    parser.add_argument("--max-breakthrough-attempts", type=int,
                         default=DEFAULT_MAX_BREAKTHROUGH_ATTEMPTS,
                         dest="max_breakthrough_attempts",
                         help="give up the gated breakthrough (fault) after this many aborted "
                              "attempts. Higher than the advance's default because the "
                              "breakthrough's measured tail needs ~2 windows even when it goes "
                              "well; the run still holds and retracts before exiting nonzero")
    parser.add_argument("--post-insert-hold-s", type=float, default=DEFAULT_POST_INSERT_HOLD_S,
                         dest="post_insert_hold_s",
                         help="how long the needle floats fully inserted before retracting")
    parser.add_argument("--retract-speed-mm-s", type=float, default=DEFAULT_RETRACT_SPEED_MM_S,
                         dest="retract_speed_mm_s",
                         help="withdrawal speed -- the commanded position is ramped back at this "
                              "rate (and the frame's velocity limit set to match), rather than "
                              "commanding the endpoint and letting the cap shape the move")
    parser.add_argument("--no-retract", dest="retract", action="store_false",
                         help="skip insertion_hold and needle_retract, leaving the needle floating "
                              "wherever it ended up -- this script's behavior before session 023")
    parser.add_argument("--insertion-correction-limit-mm", type=float,
                         default=NEEDLE_INSERTION_CORRECTION_LIMIT_MM,
                         dest="insertion_correction_limit_mm",
                         help="correction magnitude clamp for insert_drive/advance_drive only "
                              "(separate from the standoff-tracking servo) -- UNVALIDATED FIRST "
                              "GUESS, tune at the bench; see NEEDLE_INSERTION_SERVO_CONFIG")
    parser.add_argument("--insertion-correction-rate-limit-mm-s", type=float,
                         default=NEEDLE_INSERTION_CORRECTION_RATE_LIMIT_MM_S,
                         dest="insertion_correction_rate_limit_mm_s",
                         help="correction rate clamp for insert_drive/advance_drive only -- "
                              "UNVALIDATED FIRST GUESS, tune at the bench")
    args = parser.parse_args()

    # Whether the EKF runs at all. --insert-needle needs it to gate the needle; --monitor-s needs
    # it and nothing else. Everything needle-specific stays gated on args.insert_needle, so a
    # monitor run cannot command the needle even by accident.
    estimator_enabled = args.insert_needle or args.monitor_s > 0

    if args.insert_needle and args.monitor_s > 0:
        print("error: --monitor-s is a watch-only mode and --insert-needle drives the needle; "
              "pick one.")
        return 1

    if args.inhale_abort_frac is not None:
        print("warning: --inhale-abort-frac is retired and IGNORED -- the raw-sensor "
              "symmetric-return rule (--abort-margin-mm) decides when an advance drive floats. "
              "It was removed because it fired before that rule could arm, ending every advance "
              "drive in ~8ms; see drive_abort_decision() in this file.")

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
    # Needle drum, not the base's -- and defined here rather than with the rest of the needle
    # state so --dry-run can print the retract frame it would send.
    retract_velocity_rad_s = args.retract_speed_mm_s / 1000.0 / NEEDLE_DRUM_RADIUS_M
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
    if args.insert_needle:
        print(f"phase 5 insert_wait/insert_drive: once standoff_hold's recording identifies+seeds "
              f"an EKF, the needle floats until the firing gate fires at end-exhale "
              f"(exhale_band_frac={args.exhale_band_frac:.2f}, max_forecast_std_mm="
              f"{args.max_forecast_std_mm:.2f}), then drives toward "
              f"{args.needle_breakthrough_mm:.1f}mm in, capped at "
              f"{args.needle_velocity_mm_s:.1f}mm/s -- floating again once the raw sensor "
              f"descends {args.abort_margin_mm:.2f}mm and climbs back to its drive-start "
              f"reading, and resuming the REMAINDER at the next end-exhale window (up to "
              f"{args.max_breakthrough_attempts} attempts).")
        cap_note = (f"{args.max_advance_drive_s:.1f}s (fixed override)"
                    if args.max_advance_drive_s is not None
                    else f"{args.max_drive_breaths:g} breaths of whatever rate is tracked")
        print(f"phase 6 advance_wait/advance_drive: floats, then one more "
              f"{args.needle_advance_mm:.1f}mm push gated and aborted by the identical rule, "
              f"capped at {cap_note}, retrying up to {args.max_advance_attempts} times.")
        if args.retract:
            retract_s = args.needle_breakthrough_mm + args.needle_advance_mm
            print(f"phase 7 insertion_hold/needle_retract: floats in place for "
                  f"{args.post_insert_hold_s:.1f}s, then ramps back to the pre-insertion "
                  f"position at {args.retract_speed_mm_s:.1f}mm/s "
                  f"(~{retract_s:.0f}mm, ~{retract_s / max(args.retract_speed_mm_s, 1e-6):.0f}s). "
                  f"The base never moves. An attempts-exhausted fault on either drive still "
                  f"comes through here before exiting nonzero.")
        else:
            print("phase 7 insertion_hold: --no-retract, so the needle floats where it ended up "
                  f"after a {args.post_insert_hold_s:.1f}s hold and is left there.")
        print(f"needle commanded at {NEEDLE_COMMAND_HZ:.0f}Hz during active drives (matches "
              f"configs/rig_bench.yaml's procedure.loop_rate_hz -- session 021 found the "
              f"compensator needs this rate, not this script's own {args.command_hz:.0f}Hz base "
              f"cadence, to behave as designed).")
    elif args.monitor_s > 0:
        onset_s = load_disturbance_onset(args.profile)
        window = (f"until {args.monitor_after_s:.0f}s after the disturbance this profile carries "
                  f"at profile t={onset_s:.0f}s (capped at {args.monitor_s:.0f}s)"
                  if onset_s is not None else f"for {args.monitor_s:.0f}s")
        print(f"phase 5 monitor: once standoff_hold's recording identifies+seeds an EKF, watch "
              f"{window} and log the innovation and its NIS every tick. Nothing moves -- the base "
              f"is committed by standoff's accept and the needle motor is never commanded at all.")
        print(f"resending the current base target at {args.command_hz:.0f}Hz to hold position.")
    else:
        print(f"resending the current target at {args.command_hz:.0f}Hz. This script never commands the needle motor.")
        print("(neither --insert-needle nor --monitor-s set: ends after standoff_hold's "
              "recording, same as before.)")

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
        if args.insert_needle:
            print(" ", universal_command(NEEDLE_NODE_ID, CLEAR_ERRORS), "  <- needle: read start position")
            print(" ", universal_command(NEEDLE_NODE_ID, ENTER_MODE))
            print(" ", build_pos_vel_frame(NEEDLE_NODE_ID, 0.0, 0.05), "  (needle: probe for a fresh reply)")
            print(" ", universal_command(NEEDLE_NODE_ID, EXIT_MODE), "  <- needle floats (insert_wait)")
            print(f"  ... (identify+seed EKF from the {args.record_s:.0f}s recording) ...")
            print(f"  ... (on end-exhale fire: re-enter mode, drive toward "
                  f"{args.needle_breakthrough_mm:.1f}mm, cap {args.needle_velocity_mm_s:.1f}mm/s, "
                  f"abort-to-float on symmetric return, retry the REMAINDER up to "
                  f"{args.max_breakthrough_attempts}x) ...")
            print(" ", universal_command(NEEDLE_NODE_ID, EXIT_MODE), "  <- needle floats (advance_wait)")
            print(f"  ... (on end-exhale fire: re-enter mode, drive {args.needle_advance_mm:.1f}mm, "
                  f"same abort rule, retry up to {args.max_advance_attempts}x) ...")
            print(" ", universal_command(NEEDLE_NODE_ID, EXIT_MODE), "  <- needle floats (insertion_hold)")
            if args.retract:
                print(f"  ... (hold {args.post_insert_hold_s:.1f}s) ...")
                print(" ", universal_command(NEEDLE_NODE_ID, CLEAR_ERRORS), "  <- re-arm to retract")
                print(" ", universal_command(NEEDLE_NODE_ID, ENTER_MODE))
                print(" ", build_pos_vel_frame(NEEDLE_NODE_ID, 0.0, retract_velocity_rad_s),
                      f"  (needle_retract: ramped back at {args.retract_speed_mm_s:.1f}mm/s, "
                      f"resent at {NEEDLE_COMMAND_HZ:.0f}Hz, ending at the pre-insertion position)")
            print(" ", universal_command(NEEDLE_NODE_ID, EXIT_MODE), "  <- sent on exit")
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
    insert_note = (
        f" After that, the NEEDLE will float, then breakthrough-insert "
        f"{args.needle_breakthrough_mm:.1f}mm at end-exhale, then attempt one more "
        f"{args.needle_advance_mm:.1f}mm gated push (capped {args.needle_velocity_mm_s:.1f}mm/s)."
        if args.insert_needle else " This run does not command the needle motor."
    )
    print(f"\nThis will move the base motor toward the phantom by UP TO {args.max_approach_mm:.1f}mm "
          f"(advancing past {args.travel_mm:.1f}mm on its own if it has not touched by then), seat "
          f"past first contact to find true max breathing amplitude, creep further to standoff, and "
          f"hold+record for {args.record_s:.0f}s.{phantom_note}{insert_note} Watch it closely. "
          f"Ctrl+C stops and de-energizes at any point.")
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

    # -- needle + EKF state (session 022, only touched when --insert-needle) --------------------
    needle_motion_commanded = False
    last_needle_reply: dict | None = None
    needle_replies_seen = 0
    needle_floating = False
    needle_target_rad: float | None = None
    next_needle_command_at = 0.0
    needle_velocity_rad_s = (
        args.needle_velocity_mm_s / 1000.0 / NEEDLE_DRUM_RADIUS_M if args.insert_needle else 0.0
    )

    accumulator = SignalAccumulator()
    tracker = None  # built once standoff_hold's recording is long enough to identify from
    last_step = None            # newest TrackerStep, for the NIS/innovation telemetry
    monitor_started_at: float | None = None
    monitor_deadline: float | None = None
    disturbance_at: float | None = None   # when this profile's step is due, in run elapsed time
    # `servo` supplies only tau_cl(omega_r) to the latency budget (a function of plant/lead,
    # not of the correction clamps below) -- it never drives the needle. `insertion_servo` is
    # the one that actually runs insert_drive/advance_drive, with its own correction clamps
    # (looser than NEEDLE_SERVO_CONFIG's, see that constant's comment) since a single ~20mm
    # step against real phantom resistance is a different regime from continuous tracking.
    servo = LeadServo(NEEDLE_SERVO_CONFIG, Ts=NEEDLE_COMMAND_PERIOD_S) if args.insert_needle else None
    insertion_servo_config = AxisServoConfig(
        plant=NEEDLE_PLANT, lead=NEEDLE_LEAD,
        correction_limit_mm=args.insertion_correction_limit_mm,
        correction_rate_limit_mm_s=args.insertion_correction_rate_limit_mm_s,
    )
    insertion_servo = (
        LeadServo(insertion_servo_config, Ts=NEEDLE_COMMAND_PERIOD_S) if args.insert_needle else None
    )
    latency = LatencyBudget(NEEDLE_LATENCY_CONFIG, tau_cl=servo.residual_lag) if servo else None
    insert_gate = FiringGate(exhale_band_frac=args.exhale_band_frac,
                              max_forecast_std=args.max_forecast_std_mm) if args.insert_needle else None
    advance_gate = FiringGate(exhale_band_frac=args.exhale_band_frac,
                               max_forecast_std=args.max_forecast_std_mm) if args.insert_needle else None
    advance_attempts = 0
    breakthrough_attempts = 0
    insert_fired_at: float | None = None
    insert_drive_started_at: float | None = None
    insert_complete_t: float | None = None
    # One dict per fired attempt, including aborted ones. The two lists have identical shape so
    # the plot and any later analysis read one structure, not two.
    breakthrough_events: list[dict] = []
    current_breakthrough_event: dict | None = None
    breakthrough_abort_reason: str | None = None
    advance_events: list[dict] = []
    current_advance_event: dict | None = None
    advance_abort_reason: str | None = None
    # Fixed once, on the FIRST fire of their respective drive, and reused unchanged on every
    # retry -- NOT reset to a fresh position each attempt. Session 022's first live run drove a
    # full fresh 20mm from wherever the needle currently was on every retry (~60mm total travel
    # from a 20mm target, over a handful of inhale-aborted attempts) because the reference was
    # being retaken each time. Keeping the reference fixed makes error_mm naturally reflect the
    # REMAINING distance to the one true target, so a retry that already covered 15mm only drives
    # the last 5mm. The breakthrough got the same treatment in session 023 when it became
    # abortable -- until then it fired exactly once per run, so the bug was latent rather than
    # absent. insert_target_reference_rad doubles as the RETRACT HOME: it is by definition the
    # needle's position before any insertion happened.
    insert_target_reference_rad: float | None = None
    advance_target_reference_rad: float | None = None
    # Raw-sensor abort state. last_tactile_mm persists the newest reading out of the sensor-poll
    # loop so the needle-dispatch block (which runs BEFORE that loop each tick) can see it; the
    # resulting ~5-10ms lag is negligible against a multi-second breath. The others are reset per
    # attempt and drive the symmetric-return rule -- see drive_abort_decision.
    last_tactile_mm: float | None = None
    advance_descended = False
    advance_trough_mm: float | None = None
    insert_descended = False
    insert_trough_mm: float | None = None
    # insertion_hold / needle_retract state (session 023).
    hold_entered_at: float | None = None
    retract_started_at: float | None = None
    retract_complete_t: float | None = None
    retract_from_mm: float | None = None      # depth the ramp started from, for the summary
    retract_ramp_mm: float | None = None      # commanded depth, walked down to 0 at retract speed
    retract_deadline: float | None = None
    retract_final_error_mm: float | None = None
    # Frequency-lock watch. Run 7's omega collapse (1.750 -> 0.932 rad/s in one step) sat in the
    # telemetry unflagged and cost a whole debugging round; these make it loud and recorded.
    omega_stage1: float | None = None
    omega_seen_min: float | None = None
    omega_seen_max: float | None = None
    omega_warned = False
    gate_stall_since: float | None = None
    gate_stall_warned = False
    # Degeneracy watch: how far theta has actually advanced, against how far omega says it should.
    # See watch_phase_advance for the failure this exists to make audible.
    phase_prev: float | None = None
    phase_advance = 0.0
    phase_advance_t0: float | None = None
    phase_warned = False
    ident_result = None
    ident_t: float | None = None
    needle_drive_reference_rad: float | None = None
    needle_target_mm: float | None = None
    needle_stalled_since: float | None = None
    # Gate re-evaluation and tracker feeding both pause until this elapsed time -- a needle
    # drive physically disturbs the rig (confirmed on the same first live run: sensor readings
    # went "crazy" during/right after actuation), and neither firing again nor feeding the EKF
    # on that disturbance is safe. Starts at 0.0 so insert_wait's very first evaluation, before
    # anything has moved, is never held up by this.
    needle_settle_until = 0.0
    next_needle_command_at = 0.0

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

    def float_needle(reason: str) -> None:
        """Zero-torque disable -- see the needle-config comment block for why this is the
        whole compliance mechanism, not a stand-in for one."""
        nonlocal needle_floating
        print(f"needle floating ({reason})...")
        motor_bus.send(universal_command(NEEDLE_NODE_ID, EXIT_MODE))
        needle_floating = True

    def reenter_needle_mode() -> float | None:
        """Resume active needle control: CLEAR_ERRORS+ENTER_MODE, an optional pause (--rearm-
        sleep-s, an unverified guess -- see that flag's help), then a fresh position read probed
        from the best known estimate (never from an assumed 0.0 once the needle has moved -- see
        _read_fresh_needle_position_rad). Returns the fresh position, or None."""
        nonlocal needle_floating, last_needle_reply, needle_replies_seen
        motor_bus.send(universal_command(NEEDLE_NODE_ID, CLEAR_ERRORS))
        motor_bus.send(universal_command(NEEDLE_NODE_ID, ENTER_MODE))
        if args.rearm_sleep_s > 0:
            time.sleep(args.rearm_sleep_s)
        needle_floating = False
        probe_rad = last_needle_reply["position"] if last_needle_reply else 0.0
        fresh_rad = _read_fresh_needle_position_rad(motor_bus, motor_reply_codec, probe_rad)
        if fresh_rad is not None:
            last_needle_reply = {"position": fresh_rad, "velocity": 0.0, "current": 0.0, "error": 0}
            needle_replies_seen += 1
        return fresh_rad

    def enter_hold(elapsed: float, fault: str | None) -> None:
        """Finish the insertion sequence: float, hold, then retract.

        ``fault`` is None on a clean completion. When it is set -- an attempts-exhausted drive --
        the run still holds and retracts before exiting nonzero, because that failure leaves the
        needle embedded and stopping dead would leave it there for the operator to pull out by
        hand. Faults that are *not* about attempts (no motor reply, a drive timeout, a failed EKF
        seed) still stop immediately; see the terminal check at the bottom of the tick.
        """
        nonlocal phase, hold_entered_at, fault_reason
        if fault is not None:
            fault_reason = fault
            print(f"\nFAULT: {fault}")
        float_needle("insertion sequence over -- holding")
        hold_entered_at = elapsed
        phase = "insertion_hold"
        if args.retract:
            print(f"holding {args.post_insert_hold_s:.1f}s, then retracting at "
                  f"{args.retract_speed_mm_s:.1f}mm/s")
        else:
            print(f"holding {args.post_insert_hold_s:.1f}s (--no-retract: the needle stays "
                  f"where it is)")

    def needle_mm_to_rad(mm: float) -> float:
        return NEEDLE_DIRECTION_SIGN * (mm / 1000.0) / NEEDLE_DRUM_RADIUS_M

    def needle_rad_delta_to_mm(rad_delta: float) -> float:
        return rad_delta / NEEDLE_DIRECTION_SIGN * NEEDLE_DRUM_RADIUS_M * 1000.0

    def current_tracker_state() -> tuple:
        """The model's best current view, correct whether or not tracker.step() ran this tick.

        tracker.state alone is stale whenever step() has been paused (insert_drive/advance_drive
        deliberately pause it so a disturbed reading never corrupts the EKF -- see that guard in
        the sensor-poll loop). predict(dt) extrapolates through that gap via pure kinematics (no
        measurement, nothing committed) -- and when step() IS current, predict(0) is identical to
        .state, so this is a strict improvement to use everywhere, not just where staleness has
        been observed. dt MUST be computed against tracker.t's own frame: tracker.step() is fed
        raw time.monotonic() timestamps (mailbox.py's "host monotonic arrival time"), NOT this
        script's `elapsed` (which is rebased by t0) -- mixing the two, as an earlier version of
        this fix did, produces a huge bogus dt that still LOOKS plausible (theta only enters the
        model through sin/cos, so any dt lands somewhere on the sinusoid) while actually being
        phase-aliased nonsense unrelated to the real current breath position.
        """
        return tracker.predict(time.monotonic() - tracker.t)

    def watch_frequency_lock(s) -> None:
        """Record tracked omega's range and warn once if it strays from Stage 1's rate.

        The gate trusts the model's phase completely, so a collapsed omega makes it fire at the
        wrong point in the breath while every other number still looks reasonable -- run 7 fired
        at peak inhale this way, with the evidence sitting unread in the telemetry.
        """
        nonlocal omega_seen_min, omega_seen_max, omega_warned
        if omega_stage1 is None:
            return
        omega = float(s[tracker.layout.omega])
        omega_seen_min = omega if omega_seen_min is None else min(omega_seen_min, omega)
        omega_seen_max = omega if omega_seen_max is None else max(omega_seen_max, omega)
        if not omega_warned and abs(omega - omega_stage1) > OMEGA_DEVIATION_WARN_FRAC * omega_stage1:
            omega_warned = True
            print(f"\nWARNING: tracked frequency has drifted {abs(omega - omega_stage1) / omega_stage1:.0%} "
                  f"from Stage 1's rate ({omega_to_bpm(omega):.2f} vs {omega_to_bpm(omega_stage1):.2f} bpm). "
                  f"The firing gate uses the model's phase, so it may fire at the wrong point in "
                  f"the breath. Check abort_reference_mm in summary.json -- a high value means it "
                  f"fired near inhale.")

    def watch_phase_advance(elapsed: float, s) -> None:
        """Warn when `theta` stops advancing -- the degenerate mode that broke moira/2.

        The process model says theta advances at omega and everything else persists, so a healthy
        filter sweeps theta a full 2*pi every breath. The EKF can instead pin theta on the steep
        part of the sine and generate the whole waveform out of tiny theta wiggles: since
        dh/dtheta = A_k*cos(...), a larger A_k needs a smaller wiggle, so it is self-reinforcing.
        Measured on outputs/needle_gating_live/moira/2 at q_scale=0.2 -- theta advanced 0.0063
        rad/s against a true 1.4357, wiggling between -0.37 and -0.73 for the entire 693s run,
        while A_1 diverged 1.04 -> 8.47mm.

        Nothing downstream showed it: the model still tracked the sensor to 0.125mm RMSE. What
        broke was cycle_extrema, which sweeps theta a full cycle and so reported a ~16mm band for
        a ~2mm waveform, putting "end-exhale" somewhere the model never goes -- 50355 gate
        evaluations, one firing, no fault. Same (A_k, phi_k, theta, omega_r) redundancy as session
        013's frequency-lock collapse, in its most extreme form and just as silent.

        Diagnostic only. It decides nothing; --q-scale is what prevents the mode.
        """
        nonlocal phase_prev, phase_advance, phase_advance_t0, phase_warned
        theta = float(s[tracker.layout.theta])
        if phase_prev is not None:
            phase_advance += (theta - phase_prev + np.pi) % (2.0 * np.pi) - np.pi
        phase_prev = theta
        if phase_advance_t0 is None:
            phase_advance_t0 = elapsed
            return
        span = elapsed - phase_advance_t0
        if phase_warned or span < PHASE_ADVANCE_CHECK_S:
            return
        omega = float(s[tracker.layout.omega])
        rate = phase_advance / span
        if omega > 0 and rate < PHASE_ADVANCE_MIN_FRAC * omega:
            phase_warned = True
            print(f"\nWARNING: the tracked phase is barely advancing -- {rate:.4f} rad/s over "
                  f"{span:.0f}s against omega={omega:.4f}. The filter is fitting the breath by "
                  f"wiggling theta instead of sweeping it, so A_k diverges (A_1 is now "
                  f"{float(s[tracker.layout.A(1)]):.2f}mm) and the gate's band becomes "
                  f"meaningless. It will stop firing while every other number still looks fine. "
                  f"Lower --q-scale.")

    def max_drive_s(s) -> float:
        """The one cap on a gated drive, in seconds.

        Sized from the breath the tracker is actually seeing, so it travels between subjects: the
        old fixed 3.0s was sized on the sinusoid's 4.0s breath and fired as drive_overrun on
        moira's 4.65s one, capping the breakthrough at 19.2 of 20mm. --max-advance-drive-s
        REPLACES this outright when given; it is not a second ceiling, and nothing takes a min()
        of the two -- exactly one number is the cap at any instant, because a backstop that can
        fire before the primary rule arms stops being a backstop (session 022).
        """
        if args.max_advance_drive_s is not None:
            return args.max_advance_drive_s
        omega = float(s[tracker.layout.omega])
        period = 2.0 * np.pi / omega if omega > 0 else 4.0
        return args.max_drive_breaths * period

    def watch_gate_stall(elapsed: float, decision, s) -> None:
        """Say something when the gate stops being able to fire, instead of idling in silence.

        The first moira run sat in insert_wait for 70s across 7202 evaluations with one firing,
        no fault, and no output -- the tracked amplitude had run away (model excursion 1.37 ->
        6.48mm against a real ~1.5mm), so the bottom of cycle_extrema's band had drifted below
        anything the signal ever reaches and "end-exhale" had become unreachable. Every other
        number looked healthy, including the model-vs-sensor fit. Only the absence of firings
        showed it, and nothing was watching for that.
        """
        nonlocal gate_stall_since, gate_stall_warned
        if decision.fire:
            gate_stall_since = None
            gate_stall_warned = False
            return
        if gate_stall_since is None:
            gate_stall_since = elapsed
            return
        omega = float(s[tracker.layout.omega])
        period = 2.0 * np.pi / omega if omega > 0 else 4.0
        if gate_stall_warned or (elapsed - gate_stall_since) < GATE_STALL_BREATHS * period:
            return
        gate_stall_warned = True
        swept_min, swept_max = cycle_extrema(s, tracker.layout)
        print(f"\nWARNING: the firing gate has not fired for "
              f"{(elapsed - gate_stall_since) / period:.1f} breaths in {phase}. Last refusal: "
              f"{decision.reason}\n"
              f"  model sweeps [{swept_min:.3f}, {swept_max:.3f}]mm, needs the forecast at or "
              f"below {decision.band_top:.3f}mm; forecast now {decision.forecast:.3f}mm, std "
              f"{decision.forecast_std:.3f} against a budget of {args.max_forecast_std_mm:.2f}\n"
              f"  sensor is really at {last_tactile_mm:.3f}mm.\n"
              f"  If that band is far wider than the sensor's own swing, the filter has gone "
              f"degenerate -- check the phase-advance warning above and lower --q-scale. If they "
              f"are similar, the forecast simply is not reaching end-exhale: widen "
              f"--exhale-band-frac.")

    def needle_arrived(error_mm: float, velocity_mm_s: float, elapsed: float,
                        stalled_since: float | None) -> tuple[bool, float | None]:
        """Within tolerance, or stalled -- same two ways to arrive as BaseState._arrived(),
        since a loaded axis can settle short of an exact target and waiting for a tolerance
        the physics forbids turns a real arrival into a timeout."""
        if abs(error_mm) <= NEEDLE_ARRIVAL_TOL_MM:
            return True, stalled_since
        if velocity_mm_s <= NEEDLE_STALL_VELOCITY_MM_S:
            stalled_since = elapsed if stalled_since is None else stalled_since
            if elapsed - stalled_since >= NEEDLE_STALL_TIME_S:
                return True, stalled_since
        else:
            stalled_since = None
        return False, stalled_since

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

        if args.insert_needle:
            print("needle: clearing errors, entering control mode, reading start position...")
            motor_bus.send(universal_command(NEEDLE_NODE_ID, CLEAR_ERRORS))
            motor_bus.send(universal_command(NEEDLE_NODE_ID, ENTER_MODE))
            if args.rearm_sleep_s > 0:
                time.sleep(args.rearm_sleep_s)
            needle_motion_commanded = True
            start_needle_rad = _read_fresh_needle_position_rad(motor_bus, motor_reply_codec, 0.0)
            if start_needle_rad is None:
                fault_reason = (
                    f"no reply from the needle motor (node {NEEDLE_NODE_ID}) within "
                    f"{RETRACT_POSITION_TIMEOUT_S:.1f}s -- not proceeding into --insert-needle "
                    f"with no known needle position."
                )
                raise SetupFailed
            last_needle_reply = {"position": start_needle_rad, "velocity": 0.0,
                                  "current": 0.0, "error": 0}
            print(f"  needle start position = {start_needle_rad:.4f}rad -- floating now, until "
                  f"the firing gate fires.")
            float_needle("initial -- waiting for calibration + firing gate")

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

            # Drain every currently-queued reply, not just one -- a single recv() per
            # iteration falls behind once the needle is also being commanded (up to 200Hz,
            # session 021), since base and needle replies would then compete for one read per
            # ~5ms loop tick and the loser queues up, eventually delivering stale frames (the
            # same "bus.recv() returns the OLDEST queued frame" gotcha session 012 documented).
            for _ in range(32):
                motor_msg = motor_bus.recv(timeout=0.0)
                if motor_msg is None:
                    break
                parsed = motor_reply_codec.parse(motor_msg.arbitration_id, bytes(motor_msg.data))
                if parsed is None:
                    continue
                node_id = int(parsed["node_id"])
                if node_id == MOTOR_NODE_ID:
                    last_motor_reply = parsed
                    last_motor_reply_t = elapsed
                    motor_replies_seen += 1
                elif args.insert_needle and node_id == NEEDLE_NODE_ID:
                    last_needle_reply = parsed
                    needle_replies_seen += 1

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

            if args.insert_needle and elapsed >= next_needle_command_at:
                next_needle_command_at = elapsed + NEEDLE_COMMAND_PERIOD_S

                if phase == "insert_wait":
                    if (tracker is not None and last_needle_reply is not None
                            and elapsed >= needle_settle_until):
                        s, _P = current_tracker_state()
                        watch_frequency_lock(s)
                        watch_phase_advance(elapsed, s)
                        omega_r = float(s[tracker.layout.omega])
                        # + rearm_sleep_s: the gate forecasts h seconds ahead to decide "is this
                        # end-exhale", but the drive command isn't even sent until rearm_sleep_s
                        # after the decision (reenter_needle_mode()'s pause completes first) --
                        # so without this the gate is systematically firing later in the exhale
                        # window than the real activation instant actually lands. Same value that
                        # drives the real sleep, so this can never disagree with it (see
                        # --rearm-sleep-s's help).
                        h = latency.horizon(omega_r) + args.rearm_sleep_s
                        extrema = cycle_extrema(s, tracker.layout)
                        gate_decision = insert_gate.evaluate(elapsed, tracker, h, extrema)
                        watch_gate_stall(elapsed, gate_decision, s)
                        if gate_decision.fire:
                            if breakthrough_attempts >= args.max_breakthrough_attempts:
                                # Not a hard stop: the needle is partially embedded, so this
                                # routes through insertion_hold/needle_retract like any other
                                # completed run and still exits nonzero. See enter_hold().
                                enter_hold(
                                    elapsed,
                                    f"{breakthrough_attempts} aborted breakthrough attempts "
                                    f"without completing the {args.needle_breakthrough_mm:.1f}mm "
                                    f"push -- the surface is tougher than one exhale window "
                                    f"allows"
                                )
                            else:
                                fresh_rad = reenter_needle_mode()
                                if fresh_rad is None:
                                    fault_reason = (
                                        f"needle: no reply re-entering mode for breakthrough at "
                                        f"t={elapsed:.1f}s"
                                    )
                                else:
                                    breakthrough_attempts += 1
                                    # Fixed reference, established once on the FIRST fire and
                                    # reused on every retry -- NOT fresh_rad each time. Same fix
                                    # the advance needed; see the variable's own comment. It is
                                    # also the retract home.
                                    if insert_target_reference_rad is None:
                                        insert_target_reference_rad = fresh_rad
                                    needle_drive_reference_rad = insert_target_reference_rad
                                    needle_target_mm = args.needle_breakthrough_mm
                                    already_mm = needle_rad_delta_to_mm(
                                        fresh_rad - insert_target_reference_rad
                                    )
                                    remaining_mm = needle_target_mm - already_mm
                                    insertion_servo.reset(u0=0.0, y0=0.0)
                                    if insert_fired_at is None:
                                        insert_fired_at = elapsed
                                    # Real elapsed AFTER reenter_needle_mode()'s blocking re-arm
                                    # sleep, not the pre-sleep `elapsed` above -- the gap between
                                    # these two IS the real decision-to-activation delay this
                                    # telemetry exists to measure (see
                                    # scripts/plot_needle_gating_timing.py's offline placeholder
                                    # for the same quantity, estimated rather than measured there).
                                    drive_started_at = time.monotonic() - t0
                                    if insert_drive_started_at is None:
                                        insert_drive_started_at = drive_started_at
                                    current_breakthrough_event = {
                                        "attempt": breakthrough_attempts,
                                        "fired_at": elapsed,
                                        "drive_started_at": drive_started_at,
                                        "outcome": None,
                                        "already_advanced_mm": already_mm,
                                        "remaining_mm": remaining_mm,
                                        # Raw sensor reading at drive start -- the reference the
                                        # symmetric-return abort floats back at. Raw, not the
                                        # tracked model: see drive_abort_decision's docstring.
                                        "abort_reference_mm": last_tactile_mm,
                                    }
                                    breakthrough_events.append(current_breakthrough_event)
                                    needle_stalled_since = None
                                    insert_descended = False  # re-arm the symmetric-return rule
                                    insert_trough_mm = last_tactile_mm
                                    phase = "insert_drive"
                                    print(f"\nNEEDLE FIRED (breakthrough attempt "
                                          f"{breakthrough_attempts}) at t={elapsed:.3f}s: "
                                          f"{gate_decision.reason}, driving remaining "
                                          f"{remaining_mm:.1f}mm of "
                                          f"{args.needle_breakthrough_mm:.1f}mm total "
                                          f"({already_mm:.1f}mm already achieved) from "
                                          f"{fresh_rad:.4f}rad")

                elif phase == "insert_drive":
                    assert needle_drive_reference_rad is not None and needle_target_mm is not None
                    # Same raw-sensor symmetric-return rule the advance uses, via the same
                    # function. Session 022 deliberately left the breakthrough non-abortable, on
                    # the reasoning that a needle stopped mid-puncture is a worse physical state
                    # than one already through -- but the measured drives (3.63s/3.67s against a
                    # ~4.0s breath) spent half their time in inhale, which is what the gate
                    # exists to prevent. Aborting mid-puncture and resuming at the next window is
                    # the lesser of the two, and the needle is left floating in place either way.
                    abort_reference_mm = (
                        current_breakthrough_event.get("abort_reference_mm")
                        if current_breakthrough_event is not None else None
                    )
                    if last_tactile_mm is not None:
                        insert_trough_mm = (
                            last_tactile_mm if insert_trough_mm is None
                            else min(insert_trough_mm, last_tactile_mm)
                        )
                    drive_started = (
                        current_breakthrough_event["drive_started_at"]
                        if current_breakthrough_event is not None else elapsed
                    )
                    abort_rule, insert_descended = drive_abort_decision(
                        sensor_mm=last_tactile_mm,
                        reference_mm=abort_reference_mm,
                        descended=insert_descended,
                        margin_mm=args.abort_margin_mm,
                        drive_elapsed_s=elapsed - drive_started,
                        max_drive_s=max_drive_s(current_tracker_state()[0]),
                    )
                    if abort_rule is not None:
                        breakthrough_abort_reason = describe_abort(
                            abort_rule, last_tactile_mm, abort_reference_mm, insert_trough_mm,
                            elapsed, elapsed - drive_started, args,
                        )
                        already_before = (
                            current_breakthrough_event.get("already_advanced_mm", 0.0)
                            if current_breakthrough_event is not None else 0.0
                        )
                        cumulative_mm = needle_rad_delta_to_mm(
                            last_needle_reply["position"] - needle_drive_reference_rad
                        )
                        settle_s = settle_time_s(cumulative_mm - already_before,
                                                  args.needle_velocity_mm_s)
                        needle_settle_until = elapsed + settle_s
                        if current_breakthrough_event is not None:
                            current_breakthrough_event.update({
                                "outcome": "aborted", "abort_t": elapsed,
                                "abort_reason": breakthrough_abort_reason,
                                "abort_rule": abort_rule, "descended": insert_descended,
                                "trough_mm": insert_trough_mm,
                                "sensor_at_abort_mm": last_tactile_mm,
                                "reached_mm": cumulative_mm,
                            })
                        print(f"\nBREAKTHROUGH ABORTED: {breakthrough_abort_reason} -- "
                              f"{cumulative_mm:.1f}mm of {needle_target_mm:.1f}mm in, floating, "
                              f"waiting {settle_s:.2f}s for the rig to settle, then retrying next "
                              f"end-exhale window (attempt {breakthrough_attempts} of "
                              f"{args.max_breakthrough_attempts})")
                        float_needle("aborted breakthrough -- retry")
                        phase = "insert_wait"
                    else:
                        current_mm = needle_rad_delta_to_mm(
                            last_needle_reply["position"] - needle_drive_reference_rad
                        )
                        error_mm = needle_target_mm - current_mm
                        correction_mm = insertion_servo.update(error_mm)
                        command_mm = needle_target_mm + correction_mm
                        command_rad = needle_drive_reference_rad + needle_mm_to_rad(command_mm)
                        motor_bus.send(build_pos_vel_frame(NEEDLE_NODE_ID, command_rad, needle_velocity_rad_s))
                        needle_motion_commanded = True

                        velocity_mm_s = abs((last_needle_reply.get("velocity") or 0.0)) * NEEDLE_DRUM_RADIUS_M * 1000.0
                        arrived, needle_stalled_since = needle_arrived(
                            error_mm, velocity_mm_s, elapsed, needle_stalled_since
                        )
                        if arrived:
                            insert_complete_t = elapsed
                            drive_time = elapsed - drive_started
                            settle_s = settle_time_s(current_mm, args.needle_velocity_mm_s)
                            needle_settle_until = elapsed + settle_s
                            if current_breakthrough_event is not None:
                                current_breakthrough_event["outcome"] = "completed"
                                current_breakthrough_event["complete_t"] = elapsed
                                current_breakthrough_event["reached_mm"] = current_mm
                            print(f"\nbreakthrough complete at t={elapsed:.3f}s (drive took "
                                  f"{drive_time * 1e3:.0f}ms, final error {error_mm:+.3f}mm, "
                                  f"{breakthrough_attempts} attempt(s)) -- floating, waiting "
                                  f"{settle_s:.2f}s for the rig to settle before the gated "
                                  f"advance window")
                            float_needle("breakthrough complete -- advance_wait")
                            advance_gate.reset()
                            phase = "advance_wait"
                        elif elapsed - drive_started > NEEDLE_DRIVE_TIMEOUT_S:
                            fault_reason = (
                                f"needle breakthrough drive timed out after "
                                f"{NEEDLE_DRIVE_TIMEOUT_S:.0f}s (final error {error_mm:+.3f}mm, "
                                f"attempt {breakthrough_attempts})"
                            )

                elif phase == "advance_wait":
                    if (tracker is not None and last_needle_reply is not None
                            and elapsed >= needle_settle_until):
                        s, _P = current_tracker_state()
                        watch_frequency_lock(s)
                        watch_phase_advance(elapsed, s)
                        omega_r = float(s[tracker.layout.omega])
                        # + rearm_sleep_s: the gate forecasts h seconds ahead to decide "is this
                        # end-exhale", but the drive command isn't even sent until rearm_sleep_s
                        # after the decision (reenter_needle_mode()'s pause completes first) --
                        # so without this the gate is systematically firing later in the exhale
                        # window than the real activation instant actually lands. Same value that
                        # drives the real sleep, so this can never disagree with it (see
                        # --rearm-sleep-s's help).
                        h = latency.horizon(omega_r) + args.rearm_sleep_s
                        extrema = cycle_extrema(s, tracker.layout)
                        gate_decision = advance_gate.evaluate(elapsed, tracker, h, extrema)
                        watch_gate_stall(elapsed, gate_decision, s)
                        if gate_decision.fire:
                            if advance_attempts >= args.max_advance_attempts:
                                # Like the breakthrough's own exhaustion fault, this leaves the
                                # needle embedded -- more deeply, in fact -- so it routes through
                                # insertion_hold/needle_retract rather than stopping dead.
                                enter_hold(
                                    elapsed,
                                    f"{advance_attempts} inhale-aborted advance attempts "
                                    f"without completing the gated push"
                                )
                            else:
                                fresh_rad = reenter_needle_mode()
                                if fresh_rad is None:
                                    fault_reason = (
                                        f"needle: no reply re-entering mode for the gated "
                                        f"advance at t={elapsed:.1f}s"
                                    )
                                else:
                                    advance_attempts += 1
                                    # Fixed reference, established once on the FIRST fire and
                                    # reused on every retry -- NOT fresh_rad each time. See the
                                    # variable's own comment above for why: this is what makes
                                    # error_mm reflect remaining distance to the one true
                                    # target instead of re-driving the full distance from
                                    # wherever a prior aborted attempt left off.
                                    if advance_target_reference_rad is None:
                                        advance_target_reference_rad = fresh_rad
                                    needle_drive_reference_rad = advance_target_reference_rad
                                    needle_target_mm = args.needle_advance_mm
                                    already_mm = needle_rad_delta_to_mm(
                                        fresh_rad - advance_target_reference_rad
                                    )
                                    remaining_mm = needle_target_mm - already_mm
                                    insertion_servo.reset(u0=0.0, y0=0.0)
                                    current_advance_event = {
                                        "attempt": advance_attempts,
                                        "fired_at": elapsed,
                                        # Real elapsed AFTER reenter_needle_mode()'s blocking
                                        # re-arm sleep -- see the matching comment on
                                        # insert_drive_started_at above.
                                        "drive_started_at": time.monotonic() - t0,
                                        "outcome": None,
                                        "already_advanced_mm": already_mm,
                                        "remaining_mm": remaining_mm,
                                        # Raw sensor reading at drive start -- the reference the
                                        # symmetric-return abort floats back at. Raw, not the
                                        # tracked model: this decision must not depend on the EKF
                                        # (which disagreed with the sensor by up to 1.5mm on the
                                        # run that exposed the bug) or on any waveform assumption,
                                        # since real subject profiles are not sinusoids.
                                        "abort_reference_mm": last_tactile_mm,
                                    }
                                    advance_events.append(current_advance_event)
                                    needle_stalled_since = None
                                    advance_descended = False  # re-arm the symmetric-return rule
                                    advance_trough_mm = last_tactile_mm
                                    phase = "advance_drive"
                                    print(f"\nADVANCE FIRED (attempt {advance_attempts}) at "
                                          f"t={elapsed:.3f}s: {gate_decision.reason}, driving "
                                          f"remaining {remaining_mm:.1f}mm of "
                                          f"{args.needle_advance_mm:.1f}mm total "
                                          f"({already_mm:.1f}mm already achieved) from "
                                          f"{fresh_rad:.4f}rad")

                elif phase == "advance_drive":
                    assert needle_drive_reference_rad is not None and needle_target_mm is not None
                    # Abort decision: raw-sensor symmetric return about the trough, plus a
                    # deterministic drive cap. All of it lives in drive_abort_decision() so the
                    # offline replay exercises this exact code rather than a copy of it -- the
                    # last failure was "a different rule fired", which a re-implemented replay
                    # cannot catch. Nothing here reads the tracked model: the retired ceiling's
                    # cycle_extrema/tracked_measurement locals were still being computed every
                    # tick after it was removed, and are now gone with it.
                    abort_reference_mm = (
                        current_advance_event.get("abort_reference_mm")
                        if current_advance_event is not None else None
                    )
                    if last_tactile_mm is not None:
                        advance_trough_mm = (
                            last_tactile_mm if advance_trough_mm is None
                            else min(advance_trough_mm, last_tactile_mm)
                        )
                    drive_started = (
                        current_advance_event["drive_started_at"]
                        if current_advance_event is not None else elapsed
                    )
                    abort_rule, advance_descended = drive_abort_decision(
                        sensor_mm=last_tactile_mm,
                        reference_mm=abort_reference_mm,
                        descended=advance_descended,
                        margin_mm=args.abort_margin_mm,
                        drive_elapsed_s=elapsed - drive_started,
                        max_drive_s=max_drive_s(current_tracker_state()[0]),
                    )
                    if abort_rule is not None:
                        advance_abort_reason = describe_abort(
                            abort_rule, last_tactile_mm, abort_reference_mm, advance_trough_mm,
                            elapsed, elapsed - drive_started, args,
                        )
                        # Distance travelled in THIS attempt only, not the cumulative total
                        # against the fixed reference -- that's the actual physical disturbance
                        # this particular drive caused, which is what the settle time should
                        # scale with.
                        already_before_this_attempt = (
                            current_advance_event.get("already_advanced_mm", 0.0)
                            if current_advance_event is not None else 0.0
                        )
                        cumulative_mm = needle_rad_delta_to_mm(
                            last_needle_reply["position"] - needle_drive_reference_rad
                        )
                        travelled_this_attempt_mm = cumulative_mm - already_before_this_attempt
                        settle_s = settle_time_s(travelled_this_attempt_mm, args.needle_velocity_mm_s)
                        needle_settle_until = elapsed + settle_s
                        if current_advance_event is not None:
                            current_advance_event["outcome"] = "aborted"
                            current_advance_event["abort_t"] = elapsed
                            current_advance_event["abort_reason"] = advance_abort_reason
                            current_advance_event["abort_rule"] = abort_rule
                            current_advance_event["descended"] = advance_descended
                            current_advance_event["trough_mm"] = advance_trough_mm
                            current_advance_event["sensor_at_abort_mm"] = last_tactile_mm
                        print(f"\nADVANCE ABORTED: {advance_abort_reason} -- floating, waiting "
                              f"{settle_s:.2f}s for the rig to settle, then retrying next "
                              f"end-exhale window (attempt {advance_attempts} of "
                              f"{args.max_advance_attempts})")
                        float_needle("inhale-aborted advance -- retry")
                        phase = "advance_wait"
                    else:
                        current_mm = needle_rad_delta_to_mm(
                            last_needle_reply["position"] - needle_drive_reference_rad
                        )
                        error_mm = needle_target_mm - current_mm
                        correction_mm = insertion_servo.update(error_mm)
                        command_mm = needle_target_mm + correction_mm
                        command_rad = needle_drive_reference_rad + needle_mm_to_rad(command_mm)
                        motor_bus.send(build_pos_vel_frame(NEEDLE_NODE_ID, command_rad, needle_velocity_rad_s))
                        needle_motion_commanded = True

                        velocity_mm_s = abs((last_needle_reply.get("velocity") or 0.0)) * NEEDLE_DRUM_RADIUS_M * 1000.0
                        arrived, needle_stalled_since = needle_arrived(
                            error_mm, velocity_mm_s, elapsed, needle_stalled_since
                        )
                        drive_started_at = (
                            current_advance_event["drive_started_at"]
                            if current_advance_event is not None else elapsed
                        )
                        if arrived:
                            drive_time = elapsed - drive_started_at
                            if current_advance_event is not None:
                                current_advance_event["outcome"] = "completed"
                                current_advance_event["complete_t"] = elapsed
                            print(f"\ngated advance complete at t={elapsed:.3f}s (drive took "
                                  f"{drive_time * 1e3:.0f}ms, final error {error_mm:+.3f}mm, "
                                  f"{advance_attempts} attempt(s))")
                            enter_hold(elapsed, None)
                        elif elapsed - drive_started_at > NEEDLE_DRIVE_TIMEOUT_S:
                            if current_advance_event is not None:
                                current_advance_event["outcome"] = "timeout"
                                current_advance_event["timeout_t"] = elapsed
                            fault_reason = (
                                f"gated advance drive timed out after "
                                f"{NEEDLE_DRIVE_TIMEOUT_S:.0f}s (final error {error_mm:+.3f}mm, "
                                f"attempt {advance_attempts})"
                            )

                elif phase == "insertion_hold":
                    # Needle floating, fully inserted, base stationary. Nothing is commanded --
                    # the only reason this is a phase rather than a sleep is that the tick keeps
                    # running, so the sensor and the tracker keep recording through it.
                    if elapsed - (hold_entered_at or elapsed) >= args.post_insert_hold_s:
                        if not args.retract or insert_target_reference_rad is None:
                            # insert_target_reference_rad is set on the first breakthrough fire,
                            # and both routes into insertion_hold require that to have happened
                            # -- but skipping the retract beats crashing on it if that ever
                            # stops being true, since the needle is inserted at this point.
                            if args.retract:
                                print("\nskipping retract: no pre-insertion needle position was "
                                      "ever recorded, so there is no home to return to.")
                            phase = "insertion_complete"
                        else:
                            fresh_rad = reenter_needle_mode()
                            if fresh_rad is None:
                                fault_reason = (
                                    f"needle: no reply re-entering mode to retract at "
                                    f"t={elapsed:.1f}s -- the needle is still inserted"
                                )
                            else:
                                retract_from_mm = needle_rad_delta_to_mm(
                                    fresh_rad - insert_target_reference_rad
                                )
                                retract_ramp_mm = retract_from_mm
                                retract_started_at = elapsed
                                # travel/speed is the ramp's own duration; +5s covers the motor
                                # settling onto the last commanded point. A stuck needle should
                                # be named, not waited out forever or silently accepted.
                                retract_deadline = (
                                    elapsed
                                    + abs(retract_from_mm) / max(args.retract_speed_mm_s, 1e-6)
                                    + 5.0
                                )
                                phase = "needle_retract"
                                print(f"\nRETRACTING at t={elapsed:.3f}s: ramping "
                                      f"{retract_from_mm:.1f}mm back to the pre-insertion "
                                      f"position ({insert_target_reference_rad:.4f}rad) at "
                                      f"{args.retract_speed_mm_s:.1f}mm/s "
                                      f"(~{abs(retract_from_mm) / max(args.retract_speed_mm_s, 1e-6):.1f}s)")

                elif phase == "needle_retract":
                    # A ramped reference, not one endpoint command with a velocity cap: the
                    # commanded position is walked back one tick's worth of travel at a time so
                    # the target is always just ahead of actual position, exactly as standoff's
                    # coarse advance does for the base. The frame's velocity limit is set to the
                    # same speed as belt-and-braces.
                    #
                    # No lead compensator here. insertion_servo's clamps are sized for closing a
                    # 20mm step against real resistance; withdrawal is slow, has no timing
                    # requirement and no accuracy requirement beyond "it comes out", so the extra
                    # machinery would only add a second place for a sign convention to be wrong.
                    retract_ramp_mm = max(
                        0.0, retract_ramp_mm - args.retract_speed_mm_s * NEEDLE_COMMAND_PERIOD_S
                    )
                    command_rad = (
                        insert_target_reference_rad + needle_mm_to_rad(retract_ramp_mm)
                    )
                    motor_bus.send(
                        build_pos_vel_frame(NEEDLE_NODE_ID, command_rad, retract_velocity_rad_s)
                    )
                    needle_motion_commanded = True

                    # Judged from the motor's own replies, never dead-reckoned (sessions 005/009)
                    # -- but deliberately NOT via needle_arrived(): its stall detector fires when
                    # reply velocity sits under 0.3mm/s for 0.4s, which is exactly the state a
                    # ramp starting from rest is in, so it would declare arrival before the
                    # needle had moved at all.
                    error_mm = needle_rad_delta_to_mm(
                        last_needle_reply["position"] - insert_target_reference_rad
                    )
                    if retract_ramp_mm <= 1e-9 and abs(error_mm) <= NEEDLE_ARRIVAL_TOL_MM:
                        retract_complete_t = elapsed
                        retract_final_error_mm = error_mm
                        print(f"\nretract complete at t={elapsed:.3f}s "
                              f"({elapsed - (retract_started_at or elapsed):.1f}s, residual "
                              f"{error_mm:+.3f}mm from the pre-insertion position) -- floating.")
                        float_needle("retract complete")
                        phase = "retract_complete"
                    elif elapsed >= (retract_deadline or float("inf")):
                        retract_final_error_mm = error_mm
                        fault_reason = (
                            f"needle retract did not finish within "
                            f"{elapsed - (retract_started_at or elapsed):.1f}s -- still "
                            f"{error_mm:+.3f}mm from the pre-insertion position with "
                            f"{retract_ramp_mm:.2f}mm of ramp left. The needle may be stuck; it "
                            f"is left floating."
                        )

            if phase in ("retract_complete", "insertion_complete"):
                break
            # Attempt-exhaustion faults deliberately keep running so hold+retract can finish --
            # they set fault_reason AND enter insertion_hold (see enter_hold). Every other fault
            # stops here, as before.
            if fault_reason and phase not in ("insertion_hold", "needle_retract"):
                break

            done = False
            for _stamp, can_id, data in sensor_bus.poll():
                if can_id != SENSOR_CAN_ID or len(data) < _PAYLOAD.size:
                    continue
                tof_mm, dist_cm = _PAYLOAD.unpack(data[: _PAYLOAD.size])
                # Deflection from THIS run's measured rest, not from the firmware's boot-time
                # zero. Subtracting before taking the magnitude is what makes it work in
                # either direction, which matters because dist_cm's sign is arbitrary.
                # tactile_zero_cm is 0.0 under --no-tare, so this reduces to the old form.
                tactile_raw_mm = abs(dist_cm) * 10.0
                tactile_mm = abs(dist_cm - tactile_zero_cm) * 10.0
                in_contact = abs(dist_cm - tactile_zero_cm) > args.contact_threshold_cm
                # Hand the newest raw reading to the needle-dispatch block, which runs before
                # this loop on the next tick and uses it for the symmetric-return abort rule.
                last_tactile_mm = tactile_mm

                if estimator_enabled:
                    if tracker is None:
                        # Calibration window only -- base seated and stationary, matching what
                        # "the EKF has had time to generate itself while the base is seated"
                        # means. Accumulating from earlier phases would feed the identifier
                        # motion-contaminated samples.
                        if phase == "standoff_hold":
                            accumulator.offer(_stamp, tactile_mm)
                    else:
                        # Never paused. This used to skip insert_drive/advance_drive and the
                        # post-drive settle, on the assumption that actuation corrupts the
                        # sensor -- an assumption the data does not support. Run 7's raw signal
                        # is clean straight through BOTH drives (breakthrough: 5.76 -> 3.99
                        # trough -> 8.40 peak, a textbook waveform; advance attempt 1: 5.02 ->
                        # 3.68 -> back). What the pause did produce was multi-second gaps -- one
                        # was 6.1s, over 1.5 breaths -- and after them the tracker was seen
                        # collapsing to roughly half the true rate, going antiphase, and firing
                        # the next drive at peak inhale while every other number still looked
                        # sane. Feeding it continuously removes the gaps at the source; the gate
                        # is still held off after a drive by needle_settle_until, which is a
                        # separate guard and stays.
                        #
                        # The returned TrackerStep used to be discarded. It carries the
                        # innovation, its predicted variance S, and nis = innovation^2/S -- the
                        # standard "is this measurement out of distribution" statistic, computed
                        # for free on every tick. --monitor-s exists to measure how quickly it
                        # notices a disturbance, so it is logged now.
                        last_step = tracker.step(float(_stamp), float(tactile_mm))

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
                        if not estimator_enabled:
                            done = True
                        elif tracker is None:
                            if len(accumulator) < 4:
                                fault_reason = (
                                    f"standoff_hold recorded for {args.record_s:.0f}s but only "
                                    f"{len(accumulator)} tactile samples arrived -- cannot "
                                    f"identify from that. Check the sensor bus."
                                )
                                done = True
                            else:
                                print(f"\nstandoff_hold recording complete ({args.record_s:.0f}s, "
                                      f"{len(accumulator)} samples) -- identifying + seeding "
                                      f"the EKF...")
                                try:
                                    batch = accumulator.to_batch()
                                    t_end = accumulator.t_last
                                    ident_result = build_identifier(
                                        IDENTIFIER_NAME,
                                        {**IDENTIFIER_PARAMS, "q_scale": args.q_scale},
                                    ).identify(batch)
                                    # Resolve omega_bounds_fraction against THIS run's own
                                    # Stage-1 rate -- a fixed rad/s range cannot cover subjects
                                    # at 10-22bpm. See TRACKER_PARAMS' comment.
                                    stage1_bpm = float(ident_result.diagnostics["bpm_hat"])
                                    tracker_params = resolve_tracker_params(
                                        TRACKER_PARAMS, stage1_bpm
                                    )
                                    tracker = build_tracker(TRACKER_NAME, tracker_params)
                                    tracker.init(ident_result, t0=t_end)
                                    ident_t = elapsed
                                    omega_stage1 = bpm_to_omega(stage1_bpm)
                                    omega_seen_min = omega_seen_max = omega_stage1
                                    bounds = tracker_params.get("omega_bounds")
                                    print(f"  identified K={ident_result.K}, R={ident_result.R:.6g} "
                                          f"-- tracker seeded at {stage1_bpm:.2f}bpm "
                                          f"({omega_stage1:.4f}rad/s)"
                                          + (f", omega clamped to [{bounds[0]:.4f}, {bounds[1]:.4f}]"
                                             if bounds else "")
                                          + (", needle waiting for end-exhale to fire breakthrough"
                                             if args.insert_needle else
                                             f", monitoring for {args.monitor_s:.0f}s -- the "
                                             f"needle is never commanded"))
                                    monitor_started_at = elapsed
                                    if not args.insert_needle:
                                        disturbance_at = expected_disturbance_elapsed(t0, out_dir)
                                        if disturbance_at is None:
                                            print(f"  no disturbance sidecar for this profile -- "
                                                  f"monitoring the full {args.monitor_s:.0f}s")
                                        elif disturbance_at <= elapsed:
                                            print(f"\nWARNING: the disturbance was due at "
                                                  f"t={disturbance_at:.1f}s but the EKF was only "
                                                  f"seeded at t={elapsed:.1f}s -- it happened "
                                                  f"during calibration, so this run's baseline is "
                                                  f"contaminated and its latency is not usable. "
                                                  f"Approach+seat+standoff ran long; regenerate "
                                                  f"the profiles with a larger --baseline-s.")
                                        else:
                                            print(f"  disturbance due at t={disturbance_at:.1f}s "
                                                  f"({disturbance_at - elapsed:.0f}s of clean "
                                                  f"baseline first), then {args.monitor_after_s:.0f}s "
                                                  f"more before the run stops")
                                    phase = "insert_wait" if args.insert_needle else "monitor"
                                except Exception as exc:  # noqa: BLE001
                                    fault_reason = f"EKF identify/seed failed: {exc}"
                                    done = True

                elif phase == "monitor":
                    # --monitor-s: the estimator runs, the needle never moves, and nothing here
                    # commands anything. The base is already committed by standoff's accept, so
                    # this is a stationary observation window over a breathing phantom -- which is
                    # what makes the logged NIS a clean baseline distribution to judge a later
                    # disturbance against.
                    s_now, _P_now = current_tracker_state()
                    watch_frequency_lock(s_now)
                    watch_phase_advance(elapsed, s_now)
                    if monitor_deadline is None:
                        monitor_deadline = (monitor_started_at or elapsed) + args.monitor_s
                        if disturbance_at is not None:
                            monitor_deadline = min(monitor_deadline,
                                                    disturbance_at + args.monitor_after_s)
                    if elapsed >= monitor_deadline:
                        print(f"\nmonitoring complete at t={elapsed:.3f}s "
                              f"({elapsed - (monitor_started_at or elapsed):.0f}s of monitoring"
                              + (f", {elapsed - disturbance_at:.0f}s after the disturbance"
                                 if disturbance_at is not None else "") + ")")
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
                    "in_contact": in_contact,
                    "phase": phase,
                    "commanded_target_rad": current_target_rad,
                    "commanded_velocity_rad_s": current_velocity_rad_s,
                    "motor_position_rad": last_motor_reply["position"] if last_motor_reply else None,
                    "motor_velocity_rad_s": last_motor_reply["velocity"] if last_motor_reply else None,
                    "motor_torque_nm": last_motor_reply["current"] if last_motor_reply else None,
                    "motor_error": last_motor_reply["error"] if last_motor_reply else None,
                    "needle_floating": needle_floating if args.insert_needle else None,
                    "needle_position_rad": (
                        last_needle_reply["position"]
                        if args.insert_needle and last_needle_reply else None
                    ),
                    "needle_target_mm": needle_target_mm if args.insert_needle else None,
                    # The retract's commanded depth as it walks to 0. None outside
                    # needle_retract, so the ramp is directly checkable against
                    # needle_position_rad without having to infer it from the phase.
                    "retract_ramp_mm": (
                        retract_ramp_mm if phase == "needle_retract" else None
                    ),
                    "tracked_omega_r": (
                        float(current_tracker_state()[0][tracker.layout.omega])
                        if tracker is not None else None
                    ),
                    "tracked_value_mm": (
                        float(tracked_measurement(current_tracker_state()[0], tracker.layout))
                        if tracker is not None else None
                    ),
                    # From the TrackerStep this loop used to throw away. nis = innovation^2/S is
                    # the measurement's own out-of-distribution statistic: how surprising this
                    # sample was, in units of the variance the filter itself predicted for it.
                    # scripts/analyze_disturbance_detection.py thresholds a sliding mean of it.
                    "nis": float(last_step.nis) if last_step is not None else None,
                    "innovation_mm": float(last_step.innovation) if last_step is not None else None,
                    "y_pred_mm": float(last_step.y_pred) if last_step is not None else None,
                    "tracked_A_1_mm": (
                        float(current_tracker_state()[0][tracker.layout.A(1)])
                        if tracker is not None else None
                    ),
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
        if needle_motion_commanded:
            print("stopping needle (cleanup)...")
            motor_bus.send(universal_command(NEEDLE_NODE_ID, EXIT_MODE))
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
    if args.insert_needle:
        summary["insertion"] = {
            "enabled": True,
            "rearm_sleep_s": args.rearm_sleep_s,
            "rearm_sleep_note": (
                "UNVERIFIED GUESS, not a measured requirement -- feeds both the real pause in "
                "reenter_needle_mode()/the initial needle bring-up AND the gate's horizon "
                "compensation (h += rearm_sleep_s), so the two can never disagree. See "
                "--rearm-sleep-s's help."
            ),
            "calibration": {
                "record_s": args.record_s,
                "n_samples": len(accumulator),
                "identified": tracker is not None,
                "ident_t": ident_t,
                "ident_K": ident_result.K if ident_result is not None else None,
                "ident_R": ident_result.R if ident_result is not None else None,
                "omega_stage1": omega_stage1,
                "omega_tracked_min": omega_seen_min,
                "omega_tracked_max": omega_seen_max,
                "omega_deviation_warned": omega_warned,
                "q_scale": args.q_scale,
                "gate_stall_warned": gate_stall_warned,
                "gate_stall_note": (
                    "gate_stall_warned means the firing gate refused for "
                    "GATE_STALL_BREATHS straight breaths. The usual cause is the tracked "
                    "amplitude running away, which moves cycle_extrema's end-exhale band below "
                    "anything the sensor reaches and makes firing structurally impossible while "
                    "every other number still looks healthy -- lower --q-scale."
                ),
                "omega_note": (
                    "the firing gate uses the model's PHASE, so a tracked omega far from "
                    "omega_stage1 means it fired at the wrong point in the breath even though "
                    "every other number still looks sane. Run 7 collapsed 1.750 -> 0.932 rad/s "
                    "in one catch-up step after the tracker was paused across a drive, went "
                    "antiphase, and fired at peak inhale. omega_bounds now clamps this; a true "
                    "warning here means the clamp is being hit, not that it failed."
                ),
                "phase_advance_rad_s": (
                    phase_advance / (elapsed - phase_advance_t0)
                    if phase_advance_t0 is not None and elapsed > phase_advance_t0 else None
                ),
                "phase_advance_warned": phase_warned,
                "phase_advance_note": (
                    "how fast theta really advanced, against omega. They should match: the "
                    "process model IS theta += omega*Ts. The EKF can instead pin theta on the "
                    "steep part of the sine and fit the breath by wiggling it, which makes A_k "
                    "diverge and cycle_extrema's band meaningless, so the gate stops firing "
                    "while y_pred still fits perfectly. moira/2 at q_scale=0.2 measured 0.0063 "
                    "rad/s against a true 1.4357 with A_1 diverging 1.04 -> 8.47mm: 50355 gate "
                    "evaluations, 1 firing, no fault. --q-scale is what prevents it."
                ),
            },
            "breakthrough": {
                "target_mm": args.needle_breakthrough_mm,
                "velocity_cap_mm_s": args.needle_velocity_mm_s,
                # Scalars describe the FIRST fire and the FINAL completion, so tooling written
                # against the pre-session-023 single-attempt shape keeps working. `events` is
                # the full picture now that the breakthrough is abortable.
                "fired_at": insert_fired_at,
                "drive_started_at": insert_drive_started_at,
                "complete_t": insert_complete_t,
                "drive_time_s": (
                    insert_complete_t - insert_drive_started_at
                    if insert_complete_t is not None and insert_drive_started_at is not None
                    else None
                ),
                "attempts": breakthrough_attempts,
                "max_attempts": args.max_breakthrough_attempts,
                "last_abort_reason": breakthrough_abort_reason,
                "events": breakthrough_events,
                "timing_note": (
                    "fired_at is the gate's decision instant; drive_started_at is the elapsed "
                    "time AFTER reenter_needle_mode()'s blocking re-arm sleep completes and the "
                    "first drive CAN frame actually goes out -- the real decision-to-activation "
                    "gap is (drive_started_at - fired_at), not a placeholder. With more than one "
                    "attempt these scalars span the whole sequence (first fire to final "
                    "completion), so per-attempt timing must come from events."
                ),
                "gating_note": (
                    "the breakthrough became abortable in session 023, by the same rule as the "
                    "advance (drive_abort_decision). Before that it ran uninterrupted: measured "
                    "at 3.63s and 3.67s on runs 7/8 against a ~4.0s breath, so half of every "
                    "breakthrough happened during inhale. Replaying the rule over those drives "
                    "put the window close at 1.691s/1.746s with the needle already 17.97mm/"
                    "18.09mm of 20mm in, so ~2 attempts is the expected cost, not many."
                ),
            },
            "gated_advance": {
                "target_mm": args.needle_advance_mm,
                "velocity_cap_mm_s": args.needle_velocity_mm_s,
                "abort_margin_mm": args.abort_margin_mm,
                "max_drive_breaths": args.max_drive_breaths,
                "max_advance_drive_s_override": args.max_advance_drive_s,
                "attempts": advance_attempts,
                "max_attempts": args.max_advance_attempts,
                "last_abort_reason": advance_abort_reason,
                "events": advance_events,
                "events_note": (
                    "one entry per fired attempt, including inhale-aborted ones -- each has "
                    "fired_at (gate decision instant) and drive_started_at (elapsed AFTER "
                    "reenter_needle_mode()'s blocking re-arm sleep -- the real decision-to-"
                    "activation gap, not a placeholder), plus outcome ('completed'/'aborted'/"
                    "'timeout') and that outcome's own timestamp (complete_t/abort_t/timeout_t)."
                ),
                "attempts_note": (
                    "attempts counts inhale-aborted drives that had to retry, not total gate "
                    "evaluations -- see ct.control.gate.FiringGate.stats for that. A run that "
                    "completed on its first try shows attempts=1, not 0."
                ),
            },
            "retract": {
                "enabled": args.retract,
                "hold_s": args.post_insert_hold_s,
                "speed_mm_s": args.retract_speed_mm_s,
                "hold_entered_at": hold_entered_at,
                "started_at": retract_started_at,
                "complete_t": retract_complete_t,
                "travel_mm": retract_from_mm,
                "final_error_mm": retract_final_error_mm,
                "home_rad": insert_target_reference_rad,
                "note": (
                    "home_rad is the needle's own position when the FIRST breakthrough gate "
                    "fired -- by definition before any insertion -- so a retract that lands "
                    "within NEEDLE_ARRIVAL_TOL_MM of it has the needle fully out. "
                    "final_error_mm is measured from the motor's replies, not dead-reckoned. "
                    "The base does not move during any of this: standoff's accept is a hard "
                    "commit for the rest of the run."
                ),
            },
            "servo_stats": insertion_servo.stats if insertion_servo is not None else None,
            "servo_stats_note": (
                "stats for insertion_servo (drives insert_drive/advance_drive), not the "
                "latency-only `servo` used solely for tau_cl(omega_r) -- saturated_steps/"
                "rate_limited_steps here diagnose whether the insertion correction clamps "
                "(--insertion-correction-limit-mm/--insertion-correction-rate-limit-mm-s) are "
                "throttling the drive, not the tracking servo's separate, tighter clamps."
            ),
            "needle_replies_seen": needle_replies_seen,
            "gate_stats": {
                "insert": insert_gate.stats if insert_gate is not None else None,
                "advance": advance_gate.stats if advance_gate is not None else None,
            },
            "float_note": (
                "Floating is zero-torque disable (EXIT_MODE), the same universal command every "
                "needle bench script sends at cleanup -- session 004 settled that this is the "
                "whole compliance mechanism needed, not MIT mode. Repeating it several times in "
                "one run (float/re-enter per attempt) had not been exercised before this script; "
                "if this run behaved oddly around a phase transition, that toggle is the first "
                "thing to suspect."
            ),
        }
        summary["params"].update({
            "insert_needle": True,
            "needle_breakthrough_mm": args.needle_breakthrough_mm,
            "needle_advance_mm": args.needle_advance_mm,
            "needle_velocity_mm_s": args.needle_velocity_mm_s,
            "exhale_band_frac": args.exhale_band_frac,
            "max_forecast_std_mm": args.max_forecast_std_mm,
            "q_scale": args.q_scale,
            "abort_margin_mm": args.abort_margin_mm,
            "max_drive_breaths": args.max_drive_breaths,
            "max_advance_drive_s_override": args.max_advance_drive_s,
            "max_advance_attempts": args.max_advance_attempts,
            "max_breakthrough_attempts": args.max_breakthrough_attempts,
            "post_insert_hold_s": args.post_insert_hold_s,
            "retract_speed_mm_s": args.retract_speed_mm_s,
            "retract": args.retract,
            "needle_command_hz": NEEDLE_COMMAND_HZ,
            "insertion_correction_limit_mm": args.insertion_correction_limit_mm,
            "insertion_correction_rate_limit_mm_s": args.insertion_correction_rate_limit_mm_s,
        })
    else:
        summary["insertion"] = {"enabled": False}

    if args.monitor_s > 0:
        # A watch-only run. The estimator numbers live here rather than under "insertion",
        # which is about the needle.
        summary["monitor"] = {
            "enabled": True,
            "monitor_s": args.monitor_s,
            "started_at": monitor_started_at,
            "ended_at": elapsed if monitor_started_at is not None else None,
            "ident_t": ident_t,
            "ident_K": ident_result.K if ident_result is not None else None,
            "ident_R": ident_result.R if ident_result is not None else None,
            "q_scale": args.q_scale,
            "omega_stage1": omega_stage1,
            "omega_tracked_min": omega_seen_min,
            "omega_tracked_max": omega_seen_max,
            "omega_deviation_warned": omega_warned,
            "phase_advance_rad_s": (
                phase_advance / (elapsed - phase_advance_t0)
                if phase_advance_t0 is not None and elapsed > phase_advance_t0 else None
            ),
            "phase_advance_warned": phase_warned,
            "note": (
                "the needle motor was never commanded in this run. Per-tick nis/innovation_mm/"
                "y_pred_mm in samples.jsonl are what "
                "scripts/analyze_disturbance_detection.py thresholds. Check "
                "phase_advance_rad_s against omega_stage1 before believing any detection "
                "latency from this run: a filter whose theta has stopped advancing produces a "
                "NIS series that means nothing, and says so nowhere else."
            ),
        }
    save_json(summary, summary_path)
    print(f"\nsaved: {jsonl_path}")
    print(f"saved: {summary_path}")
    if (phase in ("standoff_hold", "insertion_complete", "retract_complete", "monitor")
            and not fault_reason):
        # insertion_complete now means "inserted, held, and left there" (--no-retract);
        # retract_complete means the needle came back out. Both are successful endings.
        print(f"plot with: python scripts/plot_approach_and_seat.py --run {out_dir}")
        if not args.no_phantom:
            print(f"compare against ground truth with: ct-compare {out_dir / 'phantom' / 'samples.jsonl'} {jsonl_path}")

    return 0 if not fault_reason else 1


if __name__ == "__main__":
    raise SystemExit(main())
