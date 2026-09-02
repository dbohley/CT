# 009 — 2026-09-01 — approach-travel-exhaustion

## Goal

Diagnose and fix a run that faulted with `no contact within 45s of approach` on the emma
profile. The fault message was wrong about the cause, and the investigation turned up that the
rig had been operating with essentially no contact margin for its entire history.

Also, incidentally but importantly: this run is the first to record `measured_mm`, closing an
open question carried since session 007.

## What changed

**The approach phase no longer parks on its travel target.** `--travel-mm` is now the
*initial* target rather than a bound: if the base arrives there without touching anything, it
keeps walking the target forward at the approach velocity until it makes contact or reaches a
new hard cap, `--max-approach-mm` (default 100 mm). This reuses the mechanism the standoff
coarse stage already proves — walk the commanded target one tick's worth at a time, so the
drive tracks a smooth ramp with the target just ahead of actual position — rather than adding
a second, differently-behaved way of advancing the same axis.

**Arrival is detected from the motor's own replies, never dead-reckoned.** Dead reckoning is
what caused session 005's overshoot bug, and extending a travel limit on a guess is a worse
version of the same mistake. With no telemetry the detector returns False and the timeout is
the honest backstop.

**The failure modes now name distance instead of time.** Reaching the cap is its own fault,
and it reports how close the arm actually got:

```
FAULT: extended to the 100.0mm approach cap without contact (started at 60.0mm,
auto-extended 40.0mm). Peak deflection 0.0565mm = 57% of the 0.100mm contact
threshold, against a 0.0031mm pre-contact baseline -- the arm is grazing at best.
The phantom is further away than this cap reaches, or the tactile arm is not
aligned with it.
```

The surviving timeout also changed meaning: it now fires only when the base is *not reaching
its commanded target*, i.e. a stall or missing telemetry, and says so.

**`DEFAULT_TRAVEL_MM` 40 → 60 mm**, and `summary.json` gained an `approach` block recording
travelled/extended distance, peak and baseline deflection, and the peak as a fraction of the
threshold — the numbers that made this diagnosable in the first place.

## Files touched

| File | Change |
|---|---|
| `scripts/run_approach_and_seat.py` | Auto-extend past the initial target; `--max-approach-mm` cap and its fault; `arrived_at_target()` from motor replies; cap-derived timeout; `approach` block in `summary.json`; `DEFAULT_TRAVEL_MM` 60.0; guard against `--travel-mm > --max-approach-mm` |

Nothing else was implicated. `run_breathing_profile.py`, the plot script and the estimator are
all downstream of contact.

## Decisions and rationale

**Auto-extend rather than fail fast.** Failing fast with a clear message was the alternative
and would have been a smaller change, but it leaves the operator to guess a travel number and
re-run. Extending means `--travel-mm` stops being the safety bound and `--max-approach-mm`
becomes it, so the cap is now stated in the preamble, in the confirmation prompt, and in the
dry-run frame list rather than living in a default.

**The contact threshold was deliberately left at 0.1 mm.** Lowering it looks tempting — the
failed run reached 0.0565 mm, 57 % of the way — but the data forbids it. During that run the
pre-contact baseline was mean 0.0031 mm with std 0.0067, and its *maximum* was 0.0255 mm,
which already equals the post-contact settled mean of 0.0246 mm. A threshold low enough to
catch that contact would have fired before it. The arm was grazing, not seated; the fix is
travel, not sensitivity.

**Subject-profile baseline drift is left alone.** Emma's 30 s rolling mean wanders 1.54 mm
across the recording. That is real physiology, the rig will meet it on a patient, and
detrending it away would hide the requirement rather than meet it.

## Verification

Diagnosis, from `outputs/approach_and_seat/20260901-231950` and the run before it:

| Run | profile | motor at end of approach | peak \|dist_cm\| | threshold | outcome |
|---|---|---|---|---|---|
| 165415 | breathing_profile_1 | 40.01 mm (at the limit) | 0.00941 | 0.01 | contact, by **6 %** |
| 231950 | emma | 40.01 mm (at the limit) | 0.00565 | 0.01 | 57 % of the way, no contact |

Both consumed the entire 40 mm budget. The base **arrived** (−1.5387 rad against a −1.5385
command) rather than stalling, and then held for 32 s.

Offline validation of the new logic, replaying the real logs and the arithmetic:

```bash
conda activate CT
python scripts/run_approach_and_seat.py --dry-run
python scripts/run_approach_and_seat.py --dry-run --travel-mm 20 --max-approach-mm 25
pytest -q
# 313 passed in 43.41s
```

| Check | Expected | Measured |
|---|---|---|
| Extension step | matches approach velocity | 0.1300 mm/tick at 20 Hz = 2.60 mm/s, = 0.1 rad/s × 26 mm |
| Extension 60 → 100 mm | clamps exactly to the cap | 308 ticks, 15.4 s, final 100.000000 mm |
| Cap fault timing | before the timeout | 38.5 s vs 68.5 s timeout vs 900 s watchdog |
| `arrived_at_target()` on run 231950 | fires once the base parks | first True at t = 11.37 s, **0 spurious earlier triggers** |
| `arrived_at_target()` on run 165415 | same | first True at t = 12.58 s, 0 spurious |
| `--travel-mm 120 --max-approach-mm 100` | refused | exits 1 with an explanatory message |
| Test suite | 313 | 313 passed |

The `measured_mm` path, recorded for the first time in run 231950:

| Metric | Value |
|---|---|
| status frames | 2465 over 48.2 s = **51.1 Hz** |
| ticks with a real measurement | 4209 / 4209 |
| phantom tracking | 4.152 mm measured against 4.192 mm commanded (99.1 %) |

**Not yet run on hardware.** The extension path is validated by replaying real logs through
the new predicates and by checking the arithmetic and fault ordering, not by watching the base
move. The hardware check is `python scripts/run_approach_and_seat.py --profile
emma_normal_breathing`, and the number to read is `approach.travelled_mm` — anything above
40 mm is the fix working.

## Findings

- **The rig has been operating on a 6 % contact margin since the beginning, and it looked
  fine.** The one run that reached `standoff_hold` contacted at exactly its 40 mm travel limit
  with a peak reading of 0.00941 against a 0.01 threshold. Every conclusion drawn from bench
  runs so far rested on that margin holding. A limit that is only ever reached at the very end
  of the budget is not a limit with headroom; it is a coin flip that had been landing heads.
- **Real subject profiles carry ~1.5 mm of slow baseline wander that a short looping profile
  does not.** `breathing_profile_1` is 14.8 s, loops ~11 times in a run, and therefore presents
  a stationary mean. Emma is 618 s: a 180 s run completes **zero** loops
  (`loops_completed: 0`) and plays one non-representative slice, whose mean sat at −0.140 mm
  against breathing_profile_1's −1.098 mm. That ~1 mm difference is what consumed the 6 %
  margin. Seating and travel budgets tuned against short looping profiles will be wrong on real
  ones.
- **`measured_mm` is real: 51.1 Hz.** The AK60-6 broadcasts status densely while being
  commanded at 100 Hz — it is genuine ground truth, not a sparse hint, and every tick in the
  run had one. This closes the question sessions 007 and 008 both had to work around, and it
  means `ct-compare --truth measured_mm` can now separate the phantom's own tracking lag from
  the sensing chain's. The phantom tracks its own command to 99.1 %.
- **A "grazing" contact is distinguishable from a seated one, and 3.2 σ is not enough.** At
  the travel limit the deflection was 0.0246 mm mean against a 0.0031 ± 0.0067 mm baseline.
  The tactile arm saw 0.0417 mm peak-to-peak of breathing while the ToF saw 12 mm of the same
  motion — the arm was barely engaged. Worth remembering as the signature of under-travel
  rather than of a sensor fault.

## Open questions

- **How much travel does a seated contact actually need?** 40 mm grazes; 60 mm is a reasoned
  increase, not a measured one. The first hardware run with `approach.travelled_mm` recorded
  answers it directly.
- **Is the 1.5 mm baseline wander within the tactile arm's usable stroke once seated?** The
  existing finding that "the tactile sensor's usable stroke must exceed the full breathing
  excursion" now has a second term: excursion *plus* baseline wander.
- **Does the seat phase survive a drifting baseline?** SEAT accepts when amplitude stops
  growing, but a baseline drifting away mid-seat looks like amplitude shrinking. Untested
  against a long subject profile.
- Carried: in-contact `R`, whether `tau_s = 0.677 s` is a constant, and Q as the leading
  suspect for the NIS shortfall.

## Next steps

1. Run `run_approach_and_seat.py --profile emma_normal_breathing` on hardware and read
   `approach.travelled_mm` and `approach.peak_deflection_mm`.
2. Run the deliberate negative test once — `--max-approach-mm 45` should fault at the cap with
   the distance message rather than sitting for 30 s — since the fault path is the thing that
   was broken.
3. With `measured_mm` now real, re-run the session-007 comparison with
   `--truth measured_mm` and subtract the phantom's own lag from the 677 ms figure.
4. Watch whether SEAT copes with emma's drifting baseline, per the open question above.
