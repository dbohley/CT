# 014 — 2026-09-03 — standoff-convergence

## Goal

Run `20260903-152958` took 13 standoff fine steps with 6 retreats over 325 s and never
converged; it had to be cancelled. Run `20260903-152502`, five minutes earlier, reached
`standoff_hold` in 3 steps on the same code, profile and target. Find out why, and stop the
loop being able to run forever.

## What changed

**The breathing peak is measured over counted breaths, and it is unbiased.** New
`BreathPeakWatcher` in `control/live.py` segments breaths at upward crossings of a trailing
reference level and reports the **mean of the last N per-breath peaks**, replacing `max` over
a fixed time window.

**The accept band is two-sided.** `[target−tol, target+tol]`, retreating only above it, where
it was `[target−tol, target]` and *any* over-read bought a base move.

**The settle gate is gone.** "Two consecutive windows agreeing within tol" was meant to wait
out viscoelastic settling; the fixed `settling_until` timer already does that, and real
breath-to-breath variation defeated the agreement test.

**Standoff can now give up**, as approach has since session 009: after
`--standoff-max-steps` (8) it faults with the last peaks, the band, and the measured breath
spread, instead of stepping until someone cancels it.

**A loaded tare now refuses rather than warns.** An autonomous base-retraction step was drafted
and briefly implemented in this session to address the same root cause, then removed at the
user's direction as unrequested and unsafe autonomous motion — see Findings. The base is not
moved on exit; if a previous run left it pressed into the phantom, back it off by hand before
the next one.

## Files touched

| File | Change |
|---|---|
| `src/ct/control/live.py` | new `BreathPeakWatcher`: breath segmentation, unbiased mean-of-N-breath peak, `peak_spread`, detected `period_s` |
| `scripts/run_approach_and_seat.py` | two-sided band; agreement gate removed; `--standoff-max-steps` fault; refuse a loaded tare (`--allow-loaded-tare`); per-step band reporting; summary gains the band, the peaks, the detected period |
| `tests/test_control.py` | 5 tests for the watcher, including the bias defect pinned directly |

`DEFAULT_STANDOFF_DIST_CM` stays at **0.9** — the raise from 0.8 was the trigger, but it is
kept and the controller made to converge there.

## Root cause

Three defects that only bite together, plus one trigger.

**The trigger** was the sole uncommitted change to the script, `DEFAULT_STANDOFF_DIST_CM`
`.8 → .9`. Everything else was committed in `13fd753`.

**1. `max` over a window is a biased estimator, and the bias grows with the window.** Real
breathing varies breath to breath. Replayed over the stationary measurement segments of the
two runs, `max`-over-8 s read **+0.54 mm and +0.48 mm higher** than the mean of two real
breaths, worst single segment **+1.77 mm** — against a 0.30 mm tolerance. Since standoff
retreats whenever its reading exceeds target, that bias alone moved the base backwards from
positions that were actually short. Waiting longer made it worse, not better.

**2. The band was one-sided**, so the bias in (1) converted directly into motion.

**3. The window was not the number of breaths it claimed.**
`min_breaths × nominal_breath_s` = 2 × 4.0 = 8.0 s, against a real 5.51 s period: 1.45
breaths, not 2.

**Why 152958 started worse.** The `finally` stops the motor but does not move the base, so
152502 ended with the arm pressed ~10 mm into the phantom (nothing retracts it — that is by
design; see Findings). 29 s later 152958 launched the phantom (which breathes for 6 s *before*
the tare) and tared a **moving** arm: zero 0.352 mm against a 0.140 mm true rest, p2p
**8.330 mm** against 0.014 mm free. That 0.21 mm datum error exceeds the 0.1 mm contact
threshold, so contact fired at t = 0.006 s, **APPROACH never ran**, seat accepted after 1
increment at 0.219 mm, and standoff had to make up all the distance into a barely-seated
contact.

## Verification

```bash
conda activate CT
pytest -q
# 322 passed in 27.05s
```

**The decisive check — both real runs replayed through the old and new accept logic**, over
their own stationary measurement segments:

| | good run 152502 | failing run 152958 |
|---|---|---|
| OLD | 1 accept, **0 retreat** | 2 accept, **7 retreat** |
| NEW | 1 accept, **0 retreat** | 1 accept, **3 retreat** |

The good run is untouched — advance, advance, accept, identically. On the failing run three
segments flip from RETREAT to advance, and they are exactly the spurious ones: measured
8.299 / 8.261 / 8.581 mm where the old estimator read 9.538 / 9.251 / 9.358. The three
remaining retreats (9.765, 9.879, 9.489) are genuine overshoots above the 9.30 band.

This is indicative rather than conclusive: each segment's base position is one the *old*
controller chose, so the new one would have taken a different path. What it does establish is
that the readings driving those retreats were wrong by ~1 mm.

Detector validated against real data rather than only synthetic:

| trace | breaths found | zero-crossing reference | detected period |
|---|---|---|---|
| `emma_normal_breathing`, 180 s | 33 | 37 | 6.11 s |
| run 152502 tactile, `standoff_hold` 57 s | 10 | 11 | 5.68 s |

The shortfall is the partial breath at each end, which is intended — a half-finished breath
banked as a shallow one would read as below target and buy a step the base did not need.

Unit tests pin: whole-breath counting (ready only after >8.0 s, the old window's whole
budget); **the bias defect directly** (unbiased 8.5 where `max` reads 9.0 on alternating
8/9 mm breaths); segmentation through a drifting baseline; partial breaths ignored; reset
forgetting pre-move samples.

**Not yet run on hardware.**

## Findings

- **`max` over a window is the wrong estimator for a noisy periodic peak, and it fails in the
  direction that causes motion.** Its bias is the order of the signal's own variation and it
  *grows* with the averaging window, so the instinct to "measure more carefully by waiting
  longer" makes a retreat-on-overshoot controller worse. The mean over a counted number of
  cycles is unbiased and its standard error falls the way an average should.
- **A tolerance below the signal's natural variability cannot be met reliably at any
  seating.** The 0.30 mm band sat under the breath-to-breath spread of the measurement, so
  accepting was partly luck: 152502 got lucky in 3 steps and 152958 did not. `summary.json`
  now records `breath_spread_mm` so the floor is visible next to the tolerance.
- **A "nominal" constant that nothing checks will drift away from the truth.**
  `nominal_breath_s` = 4.0 against a real 5.51 s made "two breaths" 1.45. The summary now
  reports the detected period beside it.
- **The tare's premise has to be enforced, not assumed.** It assumes the sensor starts out of
  contact — true in a real procedure, and true of a run started by hand, but not of a run
  started right after another one that left the base pressed in. Session 012 chose
  warn-and-continue here; the warning fired correctly and the run proceeded to be worthless
  anyway. Refusing (this session) catches it; the operator backs the base off by hand.
- **An automated fix for a hardware-state problem is not always the right fix, even when it
  works.** This session's first pass at the tare problem added an automatic base-retraction
  step on every exit path, sized from the run's own commanded travel and bounded by
  construction. It passed every offline check. It was still autonomous base motion that
  nobody asked for and that had never touched hardware, added unilaterally on a real rig with
  a needle axis nearby — and it was removed, correctly, at the user's direction the moment
  they saw it. "The plan approved retraction" is not the same authorization as "move the base
  on your own after every run": a plan-mode approval on a described behaviour does not extend
  to autonomously re-deriving that behaviour's scope on a physical system. On hardware,
  default to refusing and asking a human to act, not acting for them.
- **A closed loop with no give-up condition makes its own failures undiagnosable.** The run had
  to be cancelled, and a cancelled run reports nothing about why. Same lesson as session 009's
  approach-travel fault, in a different loop.
- **The `±0.3 mm` was never removed.** It was printed at phase-3 start and on the success line,
  but not on the per-step lines — the only ones visible during a long standoff. Absent
  reporting reads as absent behaviour.

## Open questions

- **How should the base get retracted between runs?** Left as an operator step for now. A
  refusal names the problem (`--allow-loaded-tare` off by default) rather than acting on it;
  whether any automated retraction is ever appropriate, and under what confirmation, is an
  open question for the user to decide, not a default to reach for.
- **Is 0.3 mm the right tolerance now that the bias is gone?** Predicted ~56% accept per
  attempt (91% within three steps). If real runs still take four or more steps, the honest fix
  is a wider band, not a longer measurement — `breath_spread_mm` says which.
- Carried from session 013: the EKF loses frequency lock on real subject profiles, which is
  still the binding constraint on anything downstream of the forecast.
- Carried: in-contact `R`; whether the seat phase's own use of `nominal_breath_s` deserves the
  same treatment (it converged in both runs, so it is not urgent).

## Next steps

1. Back the base off by hand after a run, before starting the next one. The tare refusal is
   the safety net, not a substitute for this.
2. Two back-to-back runs, base retracted by hand in between. Confirm the second one's tare
   reads p2p < 0.3 mm and it has a real APPROACH phase.
3. Confirm standoff settles in 1–3 fine steps at target 0.9, and read `breath_spread_mm`
   against `standoff_tol_mm`.
4. If it still oscillates, widen `--standoff-tol-mm` to 0.5 rather than lengthening the
   measurement — the bias that punished long measurements is gone, but the variability floor
   is real.
