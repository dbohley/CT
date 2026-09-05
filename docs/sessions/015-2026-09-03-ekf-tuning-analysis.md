# 015 — 2026-09-03 — EKF tuning analysis

## Goal

Answer three questions raised against run `20260903-171153` (config `bench_aligned`, phantom
profile `emma_normal_breathing.csv`): is the reported 0.349s tactile-sensor delay believable,
why does the EKF track troughs worse than the rest of the waveform, and are there tuning
adjustments that measurably improve `NIS mean = 0.052` and `frequency lock 10.41 ± 0.92 bpm
vs 11.405 bpm`. All of it against this one already-recorded run — no new hardware motion.

## What changed

**Confirmed no ground-truth leakage.** Traced `src/ct/sources/csv_source.py` and
`src/ct/run.py`: `CSVSource` loads the estimator's `y` strictly from the configured
`y_column` (`sensor_mm` for this run) and loads `y_clean` only from a column literally named
`y_clean`, completely independent of `y_column` — confirmed against `aligned.csv`'s actual
header (`time_s,sensor_mm,truth_mm,y_clean`). `y_clean`/`truth` only ever reach
`truth_function` → `forecast_target`, used solely for scoring; `truth_mm` isn't even read by
`CSVSource`. `tests/test_sources.py::test_csv_has_no_truth` already covers the no-truth-column
case. The EKF beating the raw sensor in places is the intended effect of forecasting at the
measured lag (session 008), not a leak: the model output is deliberately time-advanced past
the late raw reading.

**The 0.349s delay is real, and now has a picture.** New `scripts/plot_lag_detail.py` plots
`tactile_mm` against `measured_mm` (phantom ground truth) over 3 breath periods, unshifted
and with the sensor trace shifted back by the measured lag. Unshifted, the sensor visibly
peaks and troughs ~0.35s after the truth every single cycle; shifted, the two traces land on
top of each other (correlation 0.912 → 0.988). This is steady-state viscoelastic settling in
the contact, present on every cycle of `standoff_hold`, not a one-time contact-transient
artifact — it is measured only over that phase, with APPROACH/SEAT already excluded.

**The trough-tracking theory I proposed was wrong; the real mechanism is different.** New
`scripts/analyze_observability.py` tested two candidate mechanisms against this run's tracked
history. Mechanism 1 (measurement Jacobian `H_theta` vanishes near *any* extremum, symmetric
between peaks and troughs) is **disconfirmed**: bucketing by `|H_theta|` quartile, the
*highest*-`|H_theta|` quartile has the *lowest* forecast RMSE (0.108mm), not the other way
round. Mechanism 2 (asymmetric contact loss at end-exhale, already named in
`ct.control.states.approach`'s docstring) is what the data actually shows: forecast RMSE is
0.224mm at the trough vs 0.149mm at the peak vs 0.142mm mid-slope — troughs are ~50% worse
than peaks, and peaks are not worse than mid-slope at all. This persisted after the Q fix
below, so it is a real, separate, still-unexplained asymmetry in the sensing chain, not an
EKF tuning artifact.

**Found and fixed a real, measured NIS/frequency-lock improvement — with a sharp caveat.**
Swept `identifier.params.q_scale` from 0.1 to 5.0 on this run holding everything else fixed.
Frequency lock is **not** a smooth function of `q_scale`: it is fully broken (tracked omega
collapses to 1-2bpm against an 11.4bpm truth) for `q_scale <= 0.40`, and recovers sharply at
`q_scale >= 0.42`. Inside the recovered band, `q_scale = 0.5` gives the best combination
measured: NIS 0.052 → 0.092, frequency-lock std 0.92 → 0.49bpm, forecast RMSE
0.2056 → 0.1747mm, lag error removed 33.9% → 43.9%. `tracker.params.omega_bounds` (±20% of
Stage 1's rate) alone gave a smaller, less clean win (std 0.92 → 0.60, but forecast RMSE
slightly *worse*, 0.2056 → 0.2148) and added nothing on top of `q_scale=0.5`. Adopted
`q_scale: 0.5` in `configs/bench_aligned.yaml`; did not add `omega_bounds`. The recovered band
is uneven — 0.55/0.6 score worse than 0.5 and 0.7 — which is evidence Q's structure needs
more attention, not proof that 0.5 is a general answer; see Open Questions.

**Surfaced Q's own calibration diagnostics.** `estimate_Q` (`ct.identification.noise`) already
computed `n_breaths` and `omega_source` and discarded them; `_report_estimator` in
`scripts/plot_approach_and_seat.py` now prints them. This run used 16 breaths (the 180s
`--record-s` default working as intended), which already rules out "too few breaths" as the
explanation for the residual NIS/lock imperfection — CLAUDE.md's standing suspicion that more
breaths was "the first thing to try" has now been tried, on this run, and did not by itself
fix it.

**Checked for an in-contact R opportunity; none exists in this run.** `configs/rig_bench.yaml`
sets `procedure.approach.breath_hold_s: 15.0`, but that belongs to `ct.control.states.approach`
(the full four-state `ct-rig` procedure) — `scripts/run_approach_and_seat.py`, which produced
this run, implements its own `approach/seat/standoff/standoff_hold` phase machine and has no
breath-hold step at all (`{'approach', 'seat', 'standoff', 'standoff_hold'}` are the only
phases logged). In-contact `R` is still genuinely unmeasured; getting it needs either a run
through the real `ct-rig` procedure or a deliberate stationary segment added to the bench
script.

## Files touched

| File | Change |
|---|---|
| `scripts/plot_lag_detail.py` | new — zoomed, few-breath sensor-vs-truth plot, unshifted and lag-shifted, so the 0.349s delay is visible by eye |
| `scripts/analyze_observability.py` | new — buckets a run's tracked history by `\|H_theta\|` and by peak/trough/mid-slope position to test which trough-degradation mechanism actually shows up |
| `scripts/plot_approach_and_seat.py` | added `_report_q_diagnostics`, printing `n_breaths`/`omega_source` from `ident.diagnostics['q']`, previously computed and never shown |
| `configs/bench_aligned.yaml` | `identifier.params.q_scale: 0.5`, cited to this session's sweep |
| `outputs/approach_and_seat/20260903-171153/lag_detail.png` | generated figure |

## Decisions and rationale

- **Adopted `q_scale=0.5` despite the sharp, non-monotonic transition, rather than waiting for
  a smoother story.** The improvement at 0.5 is real and measured on this run across every
  metric that matters (NIS, lock std, forecast RMSE), and the alternative — leaving `q_scale`
  at the implicit 1.0 — is not a safer default, just an untested one. The non-monotonicity is
  flagged in both the config comment and Open Questions rather than hidden, so a future session
  re-running this on a different bench run knows to re-check it rather than trust the number.
- **Did not add `omega_bounds`.** It moved the same metrics in the same direction as `q_scale`
  alone but by less, cost a small amount of forecast RMSE, and added nothing once `q_scale=0.5`
  was already applied (identical numbers with or without it). Two knobs doing the same job is
  worse than one.
- **Did not chase the trough asymmetry further this session.** The mechanism named in
  `approach.py`'s docstring (losing skin contact at end-exhale) is plausible but unconfirmed;
  viscoelastic loading/unloading hysteresis (different settling behavior on the way out of a
  breath vs the way in) is an equally plausible alternative not yet distinguished from it.
  Telling them apart needs either `tactile_raw_mm` inspected specifically around trough
  timestamps for flatlining/stiction, or comparing this asymmetry across seat depths — both
  are follow-up work, not answerable from this run alone.
- **Did not attempt an in-contact R measurement this session.** it would have required either
  running the real `ct-rig` procedure (which this session's plan explicitly avoided — no new
  hardware motion) or bolting a stationary segment onto `run_approach_and_seat.py`, which is a
  larger change than this session's scope.

## Verification

```bash
conda activate CT
pytest -q
# 322 passed in 27.60s
```

```bash
ct-compare outputs/approach_and_seat/20260903-171153/phantom/samples.jsonl \
           outputs/approach_and_seat/20260903-171153/samples.jsonl \
           --phase standoff_hold --sensor tactile_mm --truth measured_mm
#   lag_s                  0.348569
#   correlation            0.988022   (correlation_unshifted 0.911612)
#   lag_ambiguous          False
#   lag_at_search_edge     False

ct-compare outputs/approach_and_seat/20260903-171153/phantom/samples.jsonl \
           outputs/approach_and_seat/20260903-171153/samples.jsonl \
           --phase standoff_hold --sensor tof_mm --truth measured_mm
#   lag_s                  0.107643   (the non-contact floor)

python scripts/plot_lag_detail.py outputs/approach_and_seat/20260903-171153 --breaths 3
# lag_s=0.3486  breath_period_s=5.454  correlation(unshifted)=0.912  correlation(aligned)=0.988

python scripts/analyze_observability.py outputs/approach_and_seat/20260903-171153/aligned.csv \
  --config bench_aligned
#   |H_theta| in [0.935, 1.575)  n=2834  forecast RMSE 0.1084 mm   <- highest quartile, LOWEST error
#   trough (bottom 15%)     forecast RMSE 0.2239 mm
#   mid-slope                forecast RMSE 0.1415 mm
#   peak (top 15%)           forecast RMSE 0.1493 mm
#   trough/peak forecast RMSE ratio: 1.50

python scripts/plot_approach_and_seat.py --run outputs/approach_and_seat/20260903-171153
# before q_scale=0.5:  NIS 0.052   freq lock 10.41 +- 0.92 bpm   forecast RMSE 0.2056mm  lag error removed 33.9%
# after  q_scale=0.5:  NIS 0.092   freq lock 10.57 +- 0.49 bpm   forecast RMSE 0.1747mm  lag error removed 43.9%
```

| Check | Expected | Measured |
|---|---|---|
| No leakage: estimator input is `sensor_mm` only | `y_clean`/`truth` never reach Identifier/Tracker | Confirmed via code trace + `test_csv_has_no_truth` |
| Sensor delay is real, not a search artifact | `lag_ambiguous: False` | Confirmed, `lag_at_search_edge: False` too |
| Trough degradation mechanism | symmetric (Jacobian) or asymmetric (contact) | Asymmetric: trough 1.5x peak RMSE; highest-`\|H_theta\|` quartile is *best*, not worst |
| `q_scale=0.5` improves NIS/lock/forecast together | plausible from Q-suspicion in prior sessions | Confirmed: NIS 0.052→0.092, lock std 0.92→0.49, RMSE 0.2056→0.1747mm |

## Findings

Promoted to `CLAUDE.md`'s Known findings (session 015 entries) — see that file for the
final wording. Summary: (1) the 0.349s tactile lag is genuine steady-state viscoelastic
contact settling, visible by eye once shifted back onto truth, and unrelated to the sensor's
~100Hz sample rate; (2) trough-specific tracking error is real (~50% worse than peaks) but is
**not** explained by the measurement Jacobian's symmetric vanishing at extrema — the opposite
pattern shows up when bucketed by `\|H_theta\|` — so the cause is asymmetric, most likely
contact-related, and still open; (3) `q_scale` has a sharp, non-monotonic effect on whether the
EKF locks onto the correct frequency at all on real subject data, with a broken-lock regime
below ~0.40 and a recovered, improved regime at 0.42-0.7 that is itself uneven rather than
smoothly better; (4) 16 breaths of calibration (this run's actual count, from the 180s
`--record-s` default) was not by itself enough to fix the standing NIS/lock imperfection, so
"more breaths" is no longer the leading hypothesis for what's still wrong.

## Open questions

- **Why does `q_scale` have a sharp stability transition rather than a smooth trade-off, and
  is `q_scale=0.5` a good default or an artifact of this one run?** New this session. The
  bimodal lock behavior (broken below ~0.40, recovered but uneven from 0.42-0.7) suggests Q's
  per-state structure — not just its overall scale — is what actually needs fixing; scaling
  everything by one constant is a coarse probe that happened to land in a better basin on this
  run, not a principled fix. Needs re-running this sweep on the other six bench runs before
  `q_scale=0.5` is trusted as a general default.
- **What actually causes the trough-specific degradation?** New this session, refines the
  question the user raised. Ruled out: symmetric Jacobian-based observability loss. Still
  candidate: contact loss at end-exhale (named in `approach.py`'s docstring) vs viscoelastic
  loading/unloading hysteresis — indistinguishable from this run alone.
- **In-contact `R` is still unmeasured** (open since session 008). This session found the
  reason no bench run has one: `run_approach_and_seat.py` doesn't implement the breath-hold
  step that `rig_bench.yaml` already configures for the real procedure. Getting it needs either
  a `ct-rig` run or adding a stationary segment to the bench script.
- **"More calibration breaths" is resolved as insufficient, not as the answer.** This run
  already had 16 breaths (the 180s default working) and still showed NIS 0.052 before the
  `q_scale` fix. Remove this from the "first thing to try" list in CLAUDE.md; `q_scale`'s
  transition is the new leading thread.

## Next steps

1. Re-run the `q_scale` sweep (`/private/tmp/.../tuning_experiment.py`'s approach, not
   preserved as a repo script) against the other six bench runs on disk to see whether 0.5
   generalizes or whether each run has its own stable band — if the latter, `q_scale` needs to
   be computed from the calibration data rather than configured as a constant.
2. Distinguish the two trough-asymmetry candidates: pull `tactile_raw_mm` around trough
   timestamps on this and other runs and look for flatlining (contact loss) vs a smooth but
   slower return (hysteresis).
3. If a bench run through the real `ct-rig` procedure happens for another reason, capture its
   breath-hold segment and finally measure in-contact `R`.
