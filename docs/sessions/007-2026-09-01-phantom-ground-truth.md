# 007 — 2026-09-01 — phantom-ground-truth

## Goal

Be able to see, after an `approach_and_seat` run, what the phantom **actually did** next to
what the sensor **thought** it did — so that when the FFT/EKF stage lands there is a real
baseline to score reconstruction and forecasting against, rather than a synthetic one.

Two things were missing. The phantom log recorded only the position it was *commanded* to,
never the position its motor reported. And the comparison was neither plotted nor correctly
computed: `ct-compare` scored the entire run, mixing in approach and seat, where the base is
moving and the sensor is not yet seated.

## What changed

**The phantom now records what it did, not only what it was told.**
`run_breathing_profile.py` drains RX each playback tick, decodes the AK60-6's status-1
broadcast with the `CubeMarsServo` codec it already holds, and converts through the *same*
capstan constants as the outgoing command. The record is now
`{t, elapsed, commanded_mm, measured_mm, error_mm}` — deliberately the same five field names
`PhantomDriver.step()` already writes, so this is the existing convention rather than a new
one. The drain also fixes a real defect: this loop previously never called `recv`, so every
status frame was discarded and the adapter's receive buffer grew unbounded for the length of
the run. A new `summary.json` reports `replies_seen` and the observed feedback rate, because
how densely this motor broadcasts *while being commanded at 100 Hz* is still unmeasured;
everything downstream degrades to `commanded_mm` and says so when feedback is absent.

**`compare_logs` grew a phase filter, and that turned out to matter more than anything else
here.** Scored across a whole bench run the same data reports correlation 0.039; scored over
`standoff_hold` alone it reports 0.961. Approach and seat are not a weaker version of the
measurement, they are a different and meaningless one. `ct-compare` gained `--phase` and
`--truth`; the plot script defaults to `--phase standoff_hold`.

**A genuine bug in the amplitude metric was found by cross-checking against the estimator,
and fixed.** `amplitude_ratio` was computed by projecting the sensor onto the *unshifted*
phantom, which folds delay into scale: a 0.677 s lag against a ~4.9 s breath costs a factor
of `cos(2π·0.677/4.93) ≈ 0.63`. It reported 0.203 where the truth is 0.326. The
cross-check that caught it is independent of the whole correlation path — running Stage 1 on
`aligned.csv` fits `A_1 = 0.346 mm` to the sensor column and `1.063 mm` to the truth column,
a ratio of 0.3256 against the corrected 0.3259. `amplitude_ratio`, `rmse_mm`, `bias_mm` and
`correlation` are now all computed after removing the lag, with `correlation_unshifted` kept
and labelled. The RMSE this changes from 0.652 mm (mostly delay) to **0.072 mm** — the real
residual, and the number an estimator has to beat.

**The plot shows all of it**, and writes `aligned.csv` so the estimator can be pointed
straight at real bench data.

## Files touched

| File | Change |
|---|---|
| `scripts/run_breathing_profile.py` | Drain RX during playback; log `measured_mm`/`error_mm`; write `summary.json` with `replies_seen` and feedback rate |
| `src/ct/phantom/driver.py` | `compare_logs`: `phase`/`phantom_field` selectors, lag-corrected amplitude and RMSE, `correlation_unshifted`, `lag_at_search_edge`, seam counting; new `_apply_lag`, `count_seams` |
| `src/ct/cli/compare.py` | `--phase`, `--truth`; warnings for no-phase, search-edge lag, seams; corrected amplitude-ratio guidance |
| `scripts/plot_approach_and_seat.py` | Two ground-truth panels on the controller's time axis, metric report, `aligned.csv` export |
| `configs/bench_aligned.yaml` | NEW — run the estimator on a bench `aligned.csv` |
| `tests/test_phantom.py` | NEW — first coverage for `compare_logs` (16 tests) |

## Decisions and rationale

**Metrics are computed after removing the lag, not before.** Delay and attenuation are
separate physical effects and folding one into the other makes both wrong. The unshifted
correlation is still reported, because it is the honest answer to "how well would this track
if you ignored the delay", but it is labelled as such and is not what any threshold should
key on.

**`standoff_hold` is the default analysis window,** and `--phase all` prints a warning
rather than being quietly available. A default that silently produces a meaningless number
is worse than one that requires a flag.

**Seams are counted on the raw commanded series, before interpolation.** A loop restart is a
property of the command — the playback loop wrapping a finite CSV — so it is not something
`measured_mm` should be consulted about. Counting on the common grid double-counts, because
interpolation smears each one-sample jump across two: the real 11 restarts read as 18. The
threshold is `5 × p95(step)` rather than a multiple of the median, because `measured_mm` is a
zero-order hold between broadcasts and its median step is exactly zero, which made every
ordinary riser look like a seam (978 of them on the first attempt).

**`aligned.csv` keeps `truth_mm` unshifted and unscaled.** Dealing with the lag is the
estimator's job; pre-applying it would hand it the answer.

**A separate `configs/bench_aligned.yaml` rather than `--config sinusoid --set
source.name=csv`.** That form does not work — `sinusoid.yaml`'s `source.params` carry
sinusoid-only keys and `--set` can add keys but not remove them, so `CSVSource` rejects the
lot with a `TypeError`. The first version of this work printed that broken command as a hint.

## Verification

All numbers below are from the real hardware run `outputs/approach_and_seat/20260901-165415`
(157.6 s controller log at 127 Hz, 59.6 s of `standoff_hold`, phantom looping
`breathing_profile_1`), analysed offline.

```bash
conda activate CT
pytest -q
# 311 passed in 42.20s      (295 before, + 16 new in tests/test_phantom.py)
```

```bash
ct-compare outputs/approach_and_seat/20260901-165415/phantom/samples.jsonl \
           outputs/approach_and_seat/20260901-165415/samples.jsonl --phase standoff_hold
```

| Metric | Whole run (no `--phase`) | `standoff_hold` |
|---|---|---|
| correlation (lag removed) | 0.070 | **0.961** |
| correlation (unshifted) | 0.039 | 0.599 |
| amplitude ratio | 0.216 | **0.326** |
| lag | 0.685 s | **0.677 s** |
| residual RMSE | 2.374 mm | **0.072 mm** |
| seams | 10 | 4 |

```bash
python scripts/plot_approach_and_seat.py --run outputs/approach_and_seat/20260901-165415
ct-identify --config bench_aligned \
  --set source.params.path=outputs/approach_and_seat/20260901-165415/aligned.csv
```

| Stage-1 fit on `aligned.csv` | K | rate [bpm] | `A_1` [mm] | `R` |
|---|---|---|---|---|
| `sensor_mm` (what the rig sees) | 1 | 12.172 | 0.346 | 0.00943 |
| `truth_mm` (what the phantom did) | 1 | 12.192 | 1.063 | 0.03312 |

| Check | Expected | Measured |
|---|---|---|
| `A_1` ratio vs `compare_logs` amplitude_ratio | agree | 0.346/1.063 = 0.3256 vs 0.3259 |
| Seams in a 59.6 s window of a 14.83 s profile | 4 | 4 |
| Seams over the 157.6 s overlap | 10 | 10 |
| Seams on a zero-order-hold `measured_mm` series | same as commanded | 4 (was 978 before the p95 fix) |
| Faulted run with no `standoff_hold` | degrades, still plots | note printed, figure written, no `aligned.csv` |
| Synthetic `measured_mm` at 25 Hz with a 0.93 tracking shortfall | ratio rises by 1/0.93 | 0.326 → 0.350 |

The `measured_mm` path has **not** run on hardware. It was exercised by injecting a
synthetic 25 Hz zero-order-hold feedback stream with a deliberate 7 % tracking shortfall into
a copy of the real log; the plot, the field selection and the ratio correction all behaved,
and the recovered ratio moved by exactly the injected factor.

## Findings

- **The tactile chain sees only ~1/3 of the phantom's real excursion, 677 ms late.**
  2.64 mm of phantom motion reads as 0.86 mm of tactile deflection. Both numbers are
  load-bearing: the lag *is* `latency.tau_s` in the forecast horizon, and the attenuation
  sets what amplitude the estimator is actually working from.
- **Once those two are taken out, the sensor is very good.** Correlation 0.961, residual RMSE
  0.072 mm against a 2.64 mm signal. Whatever the sensing chain's problems are, waveform
  fidelity is not among them — which means the EKF has a real signal to work with and a
  demanding baseline to beat.
- **677 ms is a large latency, and it is probably not electronic.** It is ~14 % of a breath
  at 12 bpm. The likely mechanism is the viscoelastic skin/lever settling already documented
  in session 005 (a 1.39 mm rise over 5 s at a fixed base position), not sensor or bus delay.
  If so it is a property of the *contact*, not of the sensor, and it will move with seating
  depth and skin consistency — worth measuring across runs before treating it as a constant.
- **The estimator recovers the same fundamental from the sensor as from ground truth**
  (12.172 vs 12.192 bpm, 0.16 % apart) despite losing two thirds of the amplitude and gaining
  677 ms of delay. Frequency survives the sensing chain even where amplitude does not.
- **`R` from the sensor column is 3.5× *smaller* than from the truth column** (0.0094 vs
  0.0331). Stage 1's residual-variance fallback measures how well a fixed-frequency harmonic
  fit explains the trace, and the attenuated sensor signal simply has less variance to leave
  unexplained. It is a reminder that this `R` is a fit-quality number, not a noise
  measurement — consistent with the existing "upper bound only" finding.
- **The 95 % energy rule picks K = 1 here too**, on both the sensed and the true column —
  now on hardware, not just on the OptiTrack recordings.
- **`moira_normal_breathing.csv` cannot be played on the phantom as-is.** Its 7.88 mm
  peak-to-peak exceeds `run_breathing_profile.py --max-travel-mm`'s 5 mm default and the
  script refuses. That default is a deliberate safety clamp, so this is a decision to make
  (raise the clamp, or scale the profile), not a bug.

## Open questions

- **Does the AK60-6 broadcast status while it is being commanded at 100 Hz, and how fast?**
  Session 004 established only that it broadcasts when idle. The new `summary.json` reports
  `replies_seen`/`feedback_hz` on the first hardware run; until then `measured_mm` is
  plumbed but unproven. If the rate is low, `measured_mm` is a zero-order hold between real
  samples and should be treated as such.
- **Is the 677 ms lag a property of the contact or of the sensor?** If it is viscoelastic
  settling, it varies with seating depth and is not a constant `tau_s`. Measure it across
  several runs at different standoff depths.
- **Is the ~0.33 amplitude ratio stable?** Session 005 measured the related compliance ratio
  at 0.22–0.68 across runs. If the attenuation moves as much, the estimator sees a
  time-varying gain, which is a modelling question and not just a calibration one.
- **`forecast_variance` is still uncalibrated** — but `aligned.csv` now makes calibrating it
  against realised error a straightforward exercise, which it was not before.
- **What to do about `breathing_profile_1.csv`'s provenance** (carried from session 006) and
  whether the phantom should be driven from the real subject profiles instead, given the
  travel clamp above.

## Next steps

1. Run `run_approach_and_seat.py` on hardware and read `replies_seen`/`feedback_hz` off the
   phantom's new `summary.json` — the one number this session could not measure.
2. With `measured_mm` real, run `ct-compare --truth measured_mm` against
   `--truth commanded_mm` on the same run to separate the phantom's own tracking error from
   the sensing chain's.
3. Repeat at two or three standoff depths to see whether the 677 ms lag and the 0.33 ratio
   move with seating — that decides whether `tau_s` is a constant.
4. Run `ct-track` on `aligned.csv` and score the forecast against `truth_mm`; start
   calibrating `forecast_variance`, and with it `max_forecast_std_mm`.
