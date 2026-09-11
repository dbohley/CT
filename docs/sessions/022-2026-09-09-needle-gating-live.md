# 022 — 2026-09-09 — needle actuation on hardware, and four bugs in one abort rule

## Goal

Take the session-022 needle-actuation feature (breakthrough + one gated advance, built but never
run) to real hardware on the sinusoid phantom profile, and get the gated advance to reliably
complete. The goal expanded twice: first to repair the sensor decode after the Teensy was
reflashed mid-session, then — for most of the session — to chase why the gated advance kept
burning all five attempts on instant aborts.

## What changed

**The sensor wire format changed under us.** Collaborators reflashed the Teensy to trim the CAN
payload: `struct("<Hfh")` (uint16 ToF, float32 dist_cm, int16 angle) became `struct("<ff")`
(float32 ToF mm, float32 dist_cm), with the angle field dropped. Five independent decode sites
carried the old format; all five were updated. The angle field turned out to be dead downstream —
decoded and logged everywhere, read by nothing.

**Live-run tooling.** `generate_sinusoid_profile.py` writes a synthetic sinusoid in the phantom
driver's own `time_s,y_mm` format (`ct-generate` writes `t,y`, which the driver can't read).
`slice_breathing_profile.py` extracts the trailing N seconds of a real profile — necessary
because `run_breathing_profile.py` loops the *whole* file, so `--profile moira_normal_breathing`
would have played all 482s from the start rather than the last 300s the offline analysis used.
Runs go to `outputs/needle_gating_live/<profile>/<trial>/` rather than polluting
`outputs/approach_and_seat/`.

**Plot markers.** `plot_approach_and_seat.py` gained gate-fire vs. actual-drive-start vertical
lines and a shaded band wherever `needle_floating` is false, plus phase colours for the four
needle phases. Reading `abort_rule` off a plot is what eventually located the fourth bug in
minutes rather than hours.

**The re-arm sleeps were dead weight.** `reenter_needle_mode()` slept `0.1s` after `CLEAR_ERRORS`
and `0.5s` after `ENTER_MODE` — round numbers copied into every script that does this handshake,
never justified by a datasheet or a measurement. `--rearm-sleep-s 0.0` ran fine on hardware. The
same value now drives both the real sleep and the gate's horizon compensation, so they cannot
disagree.

**The gated advance: four bugs, found in sequence.** Each was real, each changed the symptom, and
only the last two mattered:

1. **Frozen tracker state.** The abort read `tracker.state`, but `tracker.step()` was paused for
   the whole drive (a settle-window guard added earlier this session), so the state never moved —
   the check re-evaluated one snapshot for the drive's entire duration.
2. **A units bug in that fix.** `tracker.predict(elapsed - tracker.t)` mixed frames: `elapsed` is
   rebased to run start, `tracker.t` is raw `time.monotonic()` (the CAN mailbox stamps arrival
   time). `dt` came out at `-1,192,231s`. Because `theta` only enters through `sin`/`cos`, the
   result stayed *inside the plausible amplitude range* while being phase-aliased nonsense.
3. **A structural mismatch, not a tuning problem.** The gate fires on `forecast(t + h)`; the abort
   tested `value(t)`. During a descent `value(t)` is always above `forecast(t + h)` — that gap
   *is* the horizon. So any `inhale_abort_frac` below wherever `value(t)` sat at fire time tripped
   instantly. At 0.85 it never tripped; at 0.25 it tripped in 6ms. **No threshold value fixes
   this.** Replaced with a model-free rule on the raw sensor: reference the reading at drive
   start, require it to descend `--abort-margin-mm` (arming), then float when it returns to that
   reference — symmetric about the trough by construction, with no waveform assumption.
4. **The replacement was preempted by its own backstop.** The retired model check was left wired
   as a parallel `or`, and the bench command line still carried `--inhale-abort-frac 0.25`, so it
   kept deciding every abort. Deleted from the logic; its one legitimate job (sensor never
   returns) became `--max-advance-drive-s`.

**And then the real one: the pause was destroying frequency lock.** With the abort fixed, run 7
completed — but fired its last drive at *peak inhale*. The tracker had collapsed to roughly half
the true rate and gone antiphase; the gate fired correctly against a wrong model. The premise
behind pausing the tracker ("actuation corrupts the sensor") turned out to be **untrue** — the raw
signal is clean straight through both drives. The pause was removed entirely, and
`omega_bounds_fraction: 0.1` — CLAUDE.md's own session-018 fix, which this script had never
adopted — was wired in via `ct.run.resolve_tracker_params()`.

## Files touched

| File | Change |
|---|---|
| `scripts/run_approach_and_seat.py` | Most of the session: sensor decode, `--rearm-sleep-s`, separate insertion servo limits, fixed advance reference across retries, post-drive settle, raw-sensor abort (`advance_abort_decision()`), continuous tracker feeding, `omega_bounds`, frequency-lock warning |
| `src/ct/cli/sensor_bench.py`, `scripts/measure_sensor_noise.py`, `scripts/run_approach_and_stop.py`, `scripts_reference/receive_sensor_can.py` | New `<ff>` wire format; angle field removed |
| `scripts/plot_approach_and_seat.py` | Gate-fire/activation markers, `needle_floating` shading, needle phase colours |
| `scripts/generate_sinusoid_profile.py` | New — synthetic sinusoid in the phantom driver's `time_s,y_mm` format |
| `scripts/slice_breathing_profile.py` | New — trailing-window slice of a real profile |
| `scripts/plot_needle_gating_timing.py` | New — offline gate-fire vs. activation diagnostic |
| `scripts/plot_approach_and_stop.py` | Stale `angle_deg` docstring column |

## Decisions and rationale

**The abort is raw-sensor, not model-based.** Decided by measurement: the EKF disagreed with the
raw tactile reading by up to 1.5mm during normal drives, and by 4.1mm once lock was lost. The
sensor is clean and needs no waveform assumption — which also matters for moira/emma, which are
not sinusoids.

**Breakthrough stays non-abortable.** Confirmed with the user rather than assumed. A needle
stopped mid-puncture and floated is a worse physical state than one already through the surface;
`advance_drive` is safe to interrupt because the needle is already embedded.

**The retired flag is accepted-and-warned, not removed.** Removing it would break a bench command
line mid-session; leaving it live is what caused bug 4. It now prints a warning and changes
nothing.

**The abort rule lives in a pure function.** `advance_abort_decision()` is called by both the live
loop and the offline replay. Bug 4 was "a different rule fired" — a replay that re-implements the
rule structurally cannot catch that. This one would have.

**`omega_bounds` kept despite a measured cost.** In replay the clamp binds at exactly ±10% and
ends ~6% off Stage-1 where the unclamped filter recovered to 0%. For a gate that fires on model
phase, bounding a 44%-off collapse is worth a few percent of rate accuracy.

## Verification

```bash
conda activate CT
pytest -q
# 326 passed in 27.21s
```

Offline replay of the shipped `advance_abort_decision()` against runs 5 and 6 — all ten attempts:

| | run 5 | run 6 |
|---|---|---|
| actual (model ceiling) | 0.006-0.988s | 0.006-1.117s |
| shipped rule | **1.510-1.739s**, all `symmetric_return` | **1.536-1.693s**, all `symmetric_return` |

Frequency lock, replayed against run 7's real samples with the fix (continuous feeding + bounds):

| Check | run 7 actual | Measured with fix |
|---|---|---|
| largest single `step()` dt | 6.09s | **0.045s** |
| tracked omega range | fell to 0.876 rad/s (8.4 bpm) | **1.4332-1.7279** (13.69-16.50 bpm) |
| worst deviation from Stage 1 | −44% | **10.0%** (clamp binding) |
| model vs. raw sensor | up to 4.1mm, antiphase | **mean 0.231mm, p95 0.567mm** |

Forecast horizon at the measured rate (`omega_r=1.5647`, period 4.016s):
`tau_s 0.0200 + tau_c 0.0050 + tau_cl 0.1516 + T_ins 0.1500 = h 0.3266s` — 8.1% of a breath, 29°
before the trough. Not the cause of the peak-inhale firing.

## Findings

- **The gate fires on `forecast(t + h)`; anything that tests `value(t)` against the same band is
  structurally inconsistent with it.** During a descent `value(t)` is always higher — that gap is
  the horizon. This is not tunable, and it silently looks like a threshold problem.
- **A wrong `dt` through this model produces plausible-looking numbers.** `theta` only enters via
  `sin`/`cos`, so even `dt = -1.2e6 s` returns a value inside the correct amplitude range. Bounded
  and believable is not the same as correct.
- **Pausing the tracker across a drive is worse than feeding it disturbed data — and the data
  wasn't disturbed anyway.** The raw sensor is clean through both breakthrough
  (`5.76 → 3.99 → 8.40`) and advance. The pause created gaps up to 6.1s (1.5 breaths), after which
  the tracker was seen at half rate and antiphase.
- **Frequency collapse is silent and total.** Every other number looked sane —
  `y_pred` fit, aborts had reasons, the drive completed. Only `tracked_omega_r`, sitting unread in
  the telemetry, showed it. Now warned on and recorded.
- **The re-arm sleeps (0.1s + 0.5s) were never needed.** `--rearm-sleep-s 0.0` works. The measured
  `drive_started_at − fired_at` gap in earlier runs was never independent evidence of anything —
  it was those same sleeps read back.
- **A backstop that can fire before the primary rule arms is not a backstop.**

## Open questions

- **Does the raw-sensor abort hold on non-sinusoidal profiles?** Untested — moira and emma are
  next. The rule makes no waveform assumption by design, which is the reason to expect it to, but
  that is a prediction, not a measurement.
- **Why did omega collapse to ~half after the pause?** The Q-scales-with-dt explanation was
  proposed and **not confirmed**: a faithful replay reproduced the drift up to 1.76 rad/s but not
  the collapse to 0.93. The fix (remove the gaps, clamp omega) bounds the failure regardless of
  mechanism, but the mechanism is unexplained.
- **`--max-advance-drive-s 3.0` and `--abort-margin-mm 0.2` are first guesses**, in the same
  status `exhale_band_frac` held before it had data.
- Resolved: ~~are the re-arm sleeps necessary~~ no, 0.0 runs clean.
- Resolved: ~~is the tactile signal usable during actuation~~ yes, clean through both drives.

## Next steps

1. Add the remaining script features the user has queued, then run moira and emma.
2. Watch `abort_reference_mm` per attempt: low (~4-5mm, near the trough) is correct; high (~8mm)
   means it fired near inhale and the model is wrong again.
3. Watch for the frequency-lock warning — it firing means the clamp is doing work, which is worth
   knowing on real subject profiles.
