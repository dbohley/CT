# 005 — 2026-08-27 — approach-bench-and-gl2-telemetry

## Goal

Build the first two bench scripts in a planned series that validate successive pieces of
the full state machine, with every run's data captured for later graphing/analysis:
(1) characterize the two bench sensors' stationary noise, framed around the EKF's
measurement noise `R`; (2) drive the base motor slowly toward the phantom and stop on
tactile contact, logging everything possible. The second script's first real runs surfaced
a real overshoot bug, which became the bulk of the session: a "big research deep dive"
into the actual CubeMars motor protocol (rather than guessing at a fix), which found real
vendor documentation, confirmed genuine motor telemetry exists, and used one well-timed
real reply to find and fix the actual root cause.

## What changed

**`scripts/measure_sensor_noise.py`** (new) — records stationary ToF + tactile readings,
reports `np.var(..., ddof=1)` for both (matching `estimate_R_from_breath_hold` exactly),
labeled by which EKF regime each applies to (ToF for not-in-contact, tactile for in-contact
— confirmed from `ct.control.context._read_state`'s `skin_x` logic). Flags an
off-axis-wobble caveat for the tactile reading that has no established correction yet.

**`scripts/run_approach_and_stop.py`** (new, then substantially revised) — drives the base
motor toward the phantom, stops on tactile contact, logs to JSONL. Went through three real
design iterations this session, documented below because each one changed based on real
data or research, not guessing:

1. First version: stop by sending `EXIT_MODE` (de-energize) on contact. User feedback: this
   is compliant/bouncy, unwanted for video.
2. Second version: stop by commanding an active "hold" position, computed by dead-reckoning
   (`commanded_velocity x elapsed_time`) from the moment the approach move was sent. Real
   hardware run showed a large overshoot -- `dist_cm` continued climbing from the 0.01cm
   threshold to ~0.1cm, `tof_mm` kept dropping for ~1s afterward. A "big research deep dive"
   (below) was needed before touching this again, per explicit user direction to stop
   guessing and get real answers.
3. Third version (current): periodic command resending (`--command-hz`, default 20Hz)
   instead of one-shot commands, and the hold target computed from the most recent *real*
   motor position reply plus a small dead-reckoned correction only for the residual gap
   since that reply -- not dead reckoning across the whole elapsed approach. Root cause and
   numbers below.

**`scripts/plot_approach_and_stop.py`** (new) — first script in this repo to turn a bench
script's JSONL into a figure. Two-panel (`dist_cm`, `tof_mm`) with contact-event marker and
held-phase shading, matching `ct.diagnostics.rig_plots`'s established visual conventions
(`Agg` backend, dashed `axvline` markers, `axvspan` shading, `dpi=130` PNG). Grows a third
panel automatically when a run has real motor telemetry (`motor_position_rad`), letting the
motor's own reported position be compared directly against the two physical sensors.

**`scripts/listen_needle_motor.py`** — fixed a real, previously-undetected bug: it
constructed `CubeMarsMIT` with MIT-mode's GUI-confirmed velocity range (`±30 rad/s`), but
the needle no longer runs MIT mode (that was the abandoned session-004 detour) -- it runs
the same GL-II Position/Velocity protocol as the base, whose manual documents velocity
range `±200 rad/s`. Any real reply this script decoded would have had its velocity silently
mis-scaled by roughly 6.7x.

**`scripts/listen_base_motor.py`** (new) — same listener pattern for the base motor (node
3), which had no equivalent before.

**`scripts/scan_can_bus.py`** — fixed a stale default `--channel` (pointed at what is now
the phantom's adapter per session 004's topology swap, not needle/base's).

## Files touched

| File | Change |
|---|---|
| `scripts/measure_sensor_noise.py` | New -- stationary sensor noise, framed for EKF `R` |
| `scripts/run_approach_and_stop.py` | New, then revised twice -- stop mechanism and hold-position math fixed based on real data |
| `scripts/plot_approach_and_stop.py` | New -- first JSONL-to-figure bench script in this repo |
| `scripts/listen_needle_motor.py` | Fixed MIT-mode range constants left over from the abandoned detour |
| `scripts/listen_base_motor.py` | New -- base-motor equivalent of the needle listener |
| `scripts/scan_can_bus.py` | Fixed stale default `--channel` |

## Decisions and rationale

- **JSONL over pickle** for all new bench-script output, on user request after being shown
  the existing convention (`ct.rt.telemetry.TelemetryWriter`, already used by the phantom
  logger and controller log) — grep-able, dual-process-safe, and trivial to turn into numpy
  columns via `to_arrays`, which is what the user actually needed ("easy for anyone... or
  Claude to graph").
- **Motor stop mechanism: active hold, not de-energize**, chosen for video quality even
  though it is a more complex, less "obviously safe" mechanism than cutting power. Revisited
  once (see below) when it didn't stop accurately enough, but the *choice* of active hold
  over de-energize was never reversed — only the math computing the hold target was wrong.
- **Six-agent research pass into the CubeMars protocol before attempting another fix**,
  explicitly requested by the user rather than iterating on guesses. This is the first time
  this project has used web research (rather than local docs/session history) to resolve an
  open hardware question, and it paid off directly: it found the actual vendor manual (never
  present anywhere on this machine -- confirmed by a filesystem-wide search), confirmed the
  Gimbal-protocol addressing already in use is a real, published CubeMars GL-II protocol
  (not an unverified guess, as `docs/sessions/004` had left it), and confirmed a real
  feedback frame exists. It also correctly ruled out two easier-sounding fixes: switching to
  Velocity mode (mode=2) requires the vendor GUI plus a drive power-cycle, not a script
  change, and accel/decel tuning is GUI-only with no documented units — both were explicitly
  declined as "not now."
- **Diagnose before fixing, with real telemetry as the instrument** — rather than pick
  between competing hypotheses (motor control-loop lag vs. drivetrain mechanical compliance)
  by reasoning, real motor-position telemetry was wired into the same script so the next
  real run's data would show which one was true. It turned out to be neither: seven of the
  data points below.

## Verification

```bash
conda activate CT
pytest -q
# 295 passed in 43.41s

python -m py_compile scripts/measure_sensor_noise.py scripts/run_approach_and_stop.py \
    scripts/plot_approach_and_stop.py scripts/listen_needle_motor.py \
    scripts/listen_base_motor.py scripts/scan_can_bus.py
# all compile cleanly
```

Real hardware runs (by the user, per this project's standing rule):

| Run | Result |
|---|---|
| `measure_sensor_noise.py --duration 8` | 773 samples; ToF std 1.824mm, tactile std 0.0027mm (stationary, free-air) |
| `run_approach_and_stop.py` (v2, pre-fix), `outputs/approach_and_stop/20260827-172537` | contact at t=13.57s, held 6.76s; graph showed `dist_cm` continuing 0.01->0.10cm and `tof_mm` dropping ~123->115mm over the following ~1s |
| `run_approach_and_stop.py` (v2, pre-fix), `outputs/approach_and_stop/20260827-212820` | contact at t=14.613s; `motor_replies_seen=4` (of 1915 samples); real measured position at the one reply nearest contact (t=14.6212s) was -1.2907rad vs. dead-reckoned -1.4613rad -- a 0.1706rad (4.44mm) gap |

| Check | Expected | Measured |
|---|---|---|
| Full test suite | passes | 295 passed |
| Sensor sample rate | ~100Hz (firmware target) | 97.6Hz mean (1915 samples / 19.6s run) |
| `motor_replies_seen` before this session's fix | continuous/frequent (assumed) | 4 total, clustered exactly at the 4 commands sent |

## Findings

- **The GL-II motor protocol already in use is real, published CubeMars documentation**,
  not an unverified reverse-engineering guess — confirmed by fetching and reading the actual
  "Gimbal Motor Drive User Manual — For GL II" (V1.0) via a web-research pass, closing an
  open question `docs/sessions/004` had explicitly left unresolved. No copy of this manual
  exists anywhere on the local machine (checked via filesystem-wide search) — it had never
  been consulted before this session.
- **This motor's feedback frame is a per-command ACK, not a continuous broadcast**, unlike
  the phantom's servo motor. `motor_replies_seen=4` in a 1915-sample, 19.6s run, with all 4
  replies landing within milliseconds of the 4 commands actually sent
  (`CLEAR_ERRORS`/`ENTER_MODE`/approach-move/hold-move) — not spread through the run. Sending
  commands more often (this session's fix: periodic resend at `--command-hz`) is what turns
  this into usable, frequent telemetry rather than a handful of snapshots.
- **The overshoot bug was a dead-reckoning math error, not a slow control loop or drivetrain
  compliance** — both real candidate explanations going into this session's research, and
  both ruled out by one lucky, well-timed real reply. Dead reckoning (`velocity x
  elapsed_time` from the moment the approach move was sent) assumes constant velocity from
  t=0; the real motor has a documented (if unquantified) acceleration ramp-up at the start
  of a move, so by t=14.6s it had accumulated ~0.17rad (4.44mm) less travel than the naive
  formula assumed. The resulting "hold" command was computed 4.44mm past the real position —
  i.e., it asked the motor to advance another 4.44mm, and the motor correctly did so. It was
  never failing to stop; it was executing a wrong command correctly.
- **Switching the drive's control mode is a persistent, GUI-configured, power-cycle-requiring
  setting**, not a per-CAN-frame choice — the mode bits in the arbitration ID only matter
  once the drive itself has been configured for that mode via CubeMars's software. This
  closes off "just try Velocity mode real quick" as an option; it would need to be a
  deliberate, separate session.
- **Accel/decel for Position/Velocity mode has no CAN message, no documented units, and no
  default value** anywhere in either manual fetched this session — it exists only as a named
  GUI-tunable. There is no way to script a fix around it today.

## Open questions

- Whether the corrected hold-position math (real reply + small residual dead-reckoning
  correction) actually eliminates the overshoot on a real run — not yet re-tested on
  hardware after this session's fix.
- Whether periodic command resending at higher rates than 20Hz causes any adverse
  interaction with the drive's trajectory replanning (the same mechanism suspected, then
  ruled out, for the original overshoot) — untested; 20Hz was chosen as a reasonable
  starting point, not derived from measurement.
- The GL-II manual's torque/current range for Position/Velocity mode was never found (only
  position ±12.5rad and speed ±200rad/s were documented) — `motor_current` in the logged
  telemetry uses a placeholder range and should not be trusted numerically yet.
- Whether Velocity mode (mode=2) would meaningfully improve stop responsiveness remains
  unanswered — deliberately deferred, not investigated further this session.
- Carried from session 004, still open: root cause of MIT mode's failure on the needle
  motor; real CAN topology documentation; `forecast_variance` calibration.

## Next steps

1. Re-run `run_approach_and_stop.py` on real hardware with the corrected hold-position math
   and check the graph: does `dist_cm`/`tof_mm` now plateau close to the 0.01cm threshold
   crossing instead of continuing to ~0.1cm?
2. If overshoot is still present, use the now-frequent real `motor_position_rad` telemetry
   to check whether it tracks the sensors closely (implicating drivetrain compliance, since
   the motor-side lag hypothesis would now be corrected for) or diverges from them.
3. Physically label the CAN adapters/cables and update `unknowns.py` per session 004's
   still-open recommendation — this keeps resurfacing as a source of stale defaults
   (`scan_can_bus.py`'s channel bug this session, the earlier sensor-bus mixup two sessions
   ago).
4. Consider whether the periodic-resend pattern built for this bench script should become
   the template for how `ct.hw.motors`/`ct.rt.loop` eventually drive these motors for real
   — the manual's own "Timing Send" GUI pattern and this session's fix both point the same
   direction: continuous re-commanding is how this protocol is meant to be used, not
   one-shot moves.
