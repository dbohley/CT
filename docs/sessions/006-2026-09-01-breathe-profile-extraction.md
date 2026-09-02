# 006 — 2026-09-01 — breathe-profile-extraction

## Goal

Turn the seven raw OptiTrack takes dropped into `unfiltered_data/` into one usable breathing
trace per subject, in the `time_s,y_mm` form the rest of the pipeline reads. The reduction had
to be proven correct rather than assumed: `unfiltered_data/Moira_normal average marker
motion.csv` is a hand-produced reference for one subject over one window, and the new code had
to reproduce it exactly before being applied to the other six.

This is the first real (non-synthetic, non-bench) human breathing data in the repo. It answers
the long-standing open question *"Real sensor data: format, sample rate, availability
timeline"* for the optical modality.

## What changed

**The reduction was identified and proved, not guessed.** Each take is a 70-column OptiTrack
export at 120 Hz: two rigid bodies (`tube`, `crab`) whose position and rotation columns are
all-zero placeholders, their nine `Rigid Body Marker` children, and nine `Unlabeled NNNN`
markers. Averaging the Y position of those nine unlabeled markers reproduces the Moira
reference to `3.44e-06 mm` over all 1617 of its rows — pure 6-decimal rounding, i.e. an exact
match. Nothing else about the reference needed to be inferred: no filtering, no detrending, no
scaling, no offset.

**`scripts/build_breathe_profiles.py`** applies that reduction to every take. It selects the
marker columns *from the file's own header rows* (`Type == "Marker"` and axis `== "Y"`), never
by hard-coded index, and asserts exactly nine survive — which is what makes it survive a future
take with a different rigid-body count, and what makes the `tube`/`crab` garbage drop out
automatically rather than by a magic column list. `--verify` (on by default) re-derives the
Moira trace and re-checks it against the reference on every run, so the proof above is a
standing regression guard rather than a one-time observation.

**Seven traces landed in `breathe_profiles/`**, named by subject, at the takes' original
absolute timestamps. All seven were then run through `ct-identify` end to end; every one
yields a physiologically plausible rate and a mm-scale amplitude, so the files are genuinely
consumable by the existing `csv` source with no adapter.

## Files touched

| File | Change |
|---|---|
| `scripts/build_breathe_profiles.py` | New. Header-driven marker selection, nine-marker mean, short-gap interpolation, and the built-in reference check. |
| `breathe_profiles/{derek,emma,jake,junrong,moira,patient1,sara}_normal_breathing.csv` | New. One `time_s,y_mm` trace per subject, full take length, 120 Hz. |
| `docs/sessions/README.md` | Index line for this session. |
| `CLAUDE.md` | New findings and open-question updates (below). |

`unfiltered_data/` is read-only throughout, and `breathe_profiles/breathing_profile_1.csv` was
left untouched.

## Decisions and rationale

**Full recordings, not hand-picked windows.** Both pre-existing references are short excerpts
(the Moira reference 13.5 s of a 482 s take; `breathing_profile_1.csv` 14.8 s). Neither
documents how its window was chosen, so reproducing that choice would have meant inventing a
steady-breathing heuristic and silently discarding 97 % of the data. Exporting everything keeps
the windowing decision downstream, where `CSVSource.batch(duration_s, t0)` already implements
it. Cost is ~1.6 MB per subject.

**Original absolute timestamps, not rezeroed.** Both references keep them (238.03…, 303.07…),
and keeping them is what lets any row in an output be traced back to a frame in the raw take.

**Subject names, not `breathing_profile_N`.** The numbered scheme loses who is who, and
`breathing_profile_1.csv` turned out not to derive from any of these takes anyway (its best
match, Jake, correlates at r = 0.005 despite sitting at a similar DC level), so there was no
continuous numbering worth preserving.

**Short dropped-frame gaps are interpolated; long ones are an error.** Dropped frames are
all-marker-simultaneous, so a gap is a real hole rather than a partial average. The longest run
anywhere is 8 frames (67 ms) and nearly all sit in the last ~1 % of a take, where the subject is
getting up. Linear fill preserves the uniform 120 Hz grid that `CSVSource._infer_fs` requires
(it rejects >5 % jitter). Anything longer than `--max-gap-frames` (default 12) raises with the
file and timestamp named, rather than being smoothed over quietly.

## Verification

```bash
conda activate CT
python scripts/build_breathe_profiles.py
```

```
Verifying against Moira_normal average marker motion.csv
  1617 rows over t=[238.033333, 251.500000]s
  max |dt| = 1.00e-06 s    max |dy| = 3.44e-06 mm
  OK -- reduction reproduces the reference

Derek normal breathing.csv          -> derek_normal_breathing.csv        73372 rows, t=[0.000, 611.425]s, y=174.38..178.59 mm
Emma normal breathing.csv           -> emma_normal_breathing.csv         74154 rows, t=[0.000, 617.942]s, y=160.74..165.75 mm, 7 frame(s) in 5 gap(s) interpolated
Jake normal breathing.csv           -> jake_normal_breathing.csv         73902 rows, t=[0.000, 615.842]s, y=140.21..144.05 mm
Junrong normal breathing.csv        -> junrong_normal_breathing.csv      74323 rows, t=[0.000, 619.350]s, y=193.91..198.80 mm, 10 frame(s) in 8 gap(s) interpolated
Moira normal breathing.csv          -> moira_normal_breathing.csv        57851 rows, t=[0.000, 482.083]s, y=178.84..186.72 mm
Patient1 normal breathing.csv       -> patient1_normal_breathing.csv     73093 rows, t=[0.000, 609.100]s, y=180.01..185.85 mm, 15 frame(s) in 7 gap(s) interpolated
Sara normal breathing.csv           -> sara_normal_breathing.csv         77350 rows, t=[0.000, 644.575]s, y=169.24..178.26 mm, 1 frame(s) in 1 gap(s) interpolated
```

Reference row on disk, at the reference file's first timestamp:

```bash
grep -n '^238.033333,' breathe_profiles/moira_normal_breathing.csv
# 28566:238.033333,182.353226      (reference: 238.033333,182.353226)
```

Every produced trace through Stage 1 (`ct-identify`, `csv` source, `t_column=time_s`,
`y_column=y_mm`, 90 s calibration window, Kmax=8, 95 % energy rule):

| Subject | K used | rate [bpm] | `a0` [mm] | `A_1` [mm] |
|---|---|---|---|---|
| derek | 1 | 20.87 | 176.598 | 0.207 |
| emma | 1 | 10.23 | 163.137 | 0.987 |
| jake | 1 | 22.01 | 142.122 | 0.564 |
| junrong | 1 | 19.05 | 196.912 | 0.515 |
| moira | 1 | 13.22 | 181.835 | 1.083 |
| patient1 | 1 | 14.08 | 182.868 | 0.933 |
| sara | 1 | 15.29 | 173.458 | 0.599 |

```bash
pytest -q
# 295 passed in 40.93s
```

| Check | Expected | Measured |
|---|---|---|
| Moira reduction vs reference, `max \|dy\|` | < 1e-5 mm (6-dp rounding) | 3.44e-06 mm |
| Moira reduction vs reference, `max \|dt\|` | < 2e-6 s | 1.00e-06 s |
| Unlabeled markers per take | 9 in all 7 takes | 9 in all 7 takes |
| Sampling interval | uniform 120 Hz | `dt ∈ {0.008333, 0.008334}` in all 7 |
| Largest frame-to-frame step in the mean | no marker-swap jumps | 0.076–0.116 mm across subjects |
| Interpolated frames | few, short, near the take end | 33 frames total across 4 files; longest run 8 frames (67 ms) |
| Identified rate | 10–25 bpm | 10.23–22.01 bpm |
| Test suite | 295 passed | 295 passed |

## Findings

- **The nine useful markers are the `Unlabeled` ones, and the reduction is a plain mean of their
  Y position.** Confirmed to 3.44e-06 mm against a hand-produced reference. The rigid bodies
  `tube` and `crab` carry all-zero position/rotation and are the OptiTrack fixtures, not the
  subject; their `Rigid Body Marker` children are equally irrelevant. Selecting on
  `Type == "Marker"` separates the two cleanly.

- **This optical data is far cleaner than the synthetic configs assume.** Uniform 120 Hz with no
  jitter, at most 15 dropped frames in 73k, and a largest frame-to-frame step of 0.08–0.12 mm.
  There is effectively no high-frequency sensor noise to speak of at 120 Hz.

- **Real breathing amplitudes here are ~0.5–1.4 mm RMS on a 4–8 mm peak-to-peak excursion** —
  an order of magnitude smaller than the 10 mm amplitude `configs/sinusoid.yaml` uses and than
  the excursions the rig sizing discussion has assumed. This matters directly for the finding
  that *the tactile sensor's usable stroke must exceed the full breathing excursion*: the
  requirement is now a measured 4–8 mm, not a guess. Note these are chest-wall surface markers,
  not the target organ, so it does not settle the target's motion amplitude.

- **The 95 % energy rule selects K = 1 on all seven real subjects.** Consistent with the existing
  RC-piecewise finding — the rule under-fits anything whose Fourier series does not terminate.
  Real data now backs that up, so the threshold should be re-validated before the paper rather
  than carried over.

- **The breathing rate is not stationary within a take, and for at least one subject it moves
  enough to break a single fixed-`omega` calibration.** Derek's dominant peak measures 20.67 bpm
  over t ∈ [0,90), 14.67 bpm over [90,180), and 14.00 bpm over [300,390), while his signal std
  stays 0.52–0.67 mm. Stage 1 fitting the whole 90 s window at one `omega` recovers only
  `A_1 = 0.207 mm` of a 0.670 mm-std signal. This is exactly the `sinusoid_ramp.yaml` failure
  mode — a drifting fundamental inflating the residual — now observed on real data. Emma and
  Moira are far more stationary (10.0–11.3 and 13.3–14.7 bpm across the same windows), so this
  is subject-dependent, not universal.

## Open questions

- **Resolved: real optical sensor data format, sample rate, availability.** OptiTrack CSV export,
  120 Hz, millimetres, nine chest-wall markers, seven subjects, 8–11 minutes each, in hand.
  What is *not* resolved is the modality's **latency** (`tau_s`) — these takes carry no
  synchronised second clock, so `ct-compare` is still the only way to measure it.
- **Where did `breathe_profiles/breathing_profile_1.csv` come from?** It is not derived from any
  of these seven takes (best correlation r = 0.005, against Jake). It should be traced or
  retired before anything depends on it.
- **Is the 95 % energy threshold right for real data?** It selects K = 1 on all seven subjects.
  Carried forward from the RC-piecewise finding, now with real evidence.
- **Does the fixed-`omega` Stage-1 calibration need a stationarity guard?** Derek's within-take
  rate drift silently costs two-thirds of the recovered amplitude. A rate-stability check over
  the calibration window, or a shorter/adaptive window, would catch it.
- **Are the chest-wall marker excursions measured here (4–8 mm peak-to-peak) the right input to
  rig sizing,** or does the target organ move differently? Unchanged, but now quantified on one
  side.

## Next steps

1. Run `ct-pipeline` end to end on two or three of these subjects (a stationary one like Emma or
   Moira, and Derek as the hard case) and record real forecast RMSE against horizon `h` — the
   first forecast numbers on real data.
2. Use those runs to start calibrating `forecast_variance` against realised error. The firing
   gate's `max_forecast_std_mm` is still a guess, and it is load-bearing.
3. Re-validate the K threshold on these seven traces the way `ct-validate-k` does for the
   reference models.
4. Decide what to do about `breathing_profile_1.csv` — trace its origin or retire it.
