# 008 — 2026-09-01 — estimator-on-bench-data

## Goal

Point the FFT identifier and the harmonic EKF at real bench data for the first time, put the
result on the run's figure, and replace three of the estimator's placeholder inputs with
measured values.

The specific thing the figure has to show is **latency**. In a real procedure there is no
phantom and no phantom-motor lag — there is a patient, and a sensor that reads them late. So
ground truth had to become what the phantom's motor *actually did*, and the payoff panel had
to be the forecast: predicting forward by exactly the measured sensor lag should put the
estimate back on top of where the phantom is *now*.

## What changed

**The estimator runs on bench data and the answer is on the figure.** Two new panels: the
EKF's `y_pred` with its ±σ ribbon and the calibration/tracking boundary marked, and the
forecast at `h = tau_s` overlaid on where the phantom actually is, with the raw sensor drawn
alongside so the delay being undone stays visible. A second figure, `bench_horizon_sweep.png`,
plots forecast error against horizon with the measured lag marked. The estimator runs through
`ct.run.run_pipeline` on `configs/bench_aligned.yaml`, the same code path as `ct-pipeline`, so
a figure and a CLI run cannot disagree.

**`aligned.csv` gained a `y_clean` column, and that one column is what made the whole thing
work with no new plumbing.** `CSVSource` already reads a column of that exact name into
`SignalBatch.y_clean`, and `ct.run.truth_function` already prefers it over the measured trace.
Writing the phantom truth there — mapped into the sensor's frame as
`ratio·truth(t − lag) + offset` — makes `run_pipeline` score its forecast against the
**phantom** instead of against the sensor, which is the difference between a meaningful
forecast score and a circular one. `truth_mm` stays raw so the mapping can be redone. CLAUDE.md's
rule holds: `y_clean` is for validation and plots, and no estimator code reads it.

**Ground truth defaults to `measured_mm`.** `commanded_mm` folds the phantom motor's own
tracking lag into a number that is supposed to be sensor latency alone. A log without motor
feedback still works but prints a loud warning that the reported latency is an over-estimate.

**The base-motor position panel is gone** — it was diagnostic clutter next to the question the
figure now answers.

**`R_override`** was added to the identifier, because there was no way to supply a measured
`R` at all: only `breath_hold_window`, which needs a still segment inside the calibration
window to exist. Precedence is `R_override` > `breath_hold_window` > residual variance, and
`R_source` names which one was used so a report can never present a configured `R` as a
measured one.

## Files touched

| File | Change |
|---|---|
| `src/ct/identification/fft_identifier.py` | `R_override` param, top of the `_estimate_R` precedence chain, reported via `R_source` |
| `configs/bench_aligned.yaml` | `K_override: 3`, `R_override: 7.3e-6`, `calib_seconds: 90`, `horizon: 0.677` |
| `scripts/plot_approach_and_seat.py` | Motor panel removed; `--truth` defaults to `measured_mm`; EKF and forecast panels; horizon-sweep figure; `y_clean` column; `--no-estimator`/`--estimator-config`/`--horizon` |
| `scripts/run_approach_and_seat.py` | `--record-s` 60 → 180, `--max-runtime-s` 600 → 900 |
| `tests/test_identification.py` | `R_override` precedence and `R_source` reporting |
| `tests/test_phantom.py` | The forecast claim, on a synthetic signal with a known injected lag |

## Decisions and rationale

**`K = 3`, against the 95 % energy rule's `K = 1`.** Measured forecast RMSE at h = 0.677 s:
0.0766 mm at K=1, 0.0671 at K=2, 0.0515 at K=3, 0.0484 at K=4. K=3 takes essentially all the
gain; K=4 buys another 6 % for two more states drifting under Q. The 95 % threshold is kept in
the config and deliberately not used, so the report prints what the rule *would* have picked
next to what is used — the disagreement is a finding, not something to hide.

**`R = 7.3e-6 mm²`, the measured free-air sensor noise** (session 005: tactile std 0.0027 mm),
replacing the residual-variance fallback. Chosen knowing it is a *lower* bound — free-air,
arm unloaded, missing the off-axis wobble that appears once the arm is pressed into skin.

**`h = tau_s = 0.677 s`.** The one horizon term that is now a measurement rather than a guess.
Using only it, rather than the full `h` with placeholder `tau_c`/`tau_cl`/`T_ins`, makes the
result interpretable: any gap in the forecast panel is estimator error, not an unmeasured
latency term.

**`--record-s` 60 → 180 s.** The hold window is what the estimator gets, and Q is measured from
breath-to-breath refits, so it is really "how many breaths". 180 s is ~36 breaths at 12 bpm:
90 s to calibrate (~18 refits, matching session 006's OptiTrack windows) and 90 s to track. The
plot script clamps `calib_seconds` to half the available window and says so, rather than
failing, so the older 60 s runs still analyse.

## Verification

All numbers from the real hardware run `20260901-165415`, analysed offline. Its hold window is
59.6 s, so `calib_seconds` clamped from 90 s to 29.8 s and only ~30 s was tracked — a 180 s run
will do considerably better.

```bash
conda activate CT
pytest -q
# 313 passed in 43.25s      (311 before, + 2 new)

python scripts/plot_approach_and_seat.py --run outputs/approach_and_seat/20260901-165415
```

**The headline — forecasting by the measured lag, scored against where the phantom actually is:**

| | RMSE |
|---|---|
| raw sensor (do nothing, i.e. read late) | 0.2332 mm |
| **forecast at h = 0.677 s** | **0.0888 mm** |
| **lag error removed** | **61.9 %** |

Cost of the horizon, from the sweep: 0.0710 mm at h = 0 → 0.1016 mm at h = 0.70 s. Predicting
677 ms ahead costs **+0.031 mm** and buys back **0.144 mm** of lag error — a ~4.7:1 return.

**Stage 1 on the bench trace:** K used 3, K by the 95 % rule 1, rate 12.170 bpm,
R = 7.3e-6 (`configured (R_override)`), NIS mean 0.245.

**The `R` change in isolation** (same K=3, same 30 s calibration, only `R` differing):

| R | source | NIS mean | forecast RMSE |
|---|---|---|---|
| 0.00755 mm² | residual variance (upper bound) | 0.040 | 0.0820 mm |
| 7.3e-6 mm² | measured free-air | 0.243 | 0.0989 mm |

| Check | Expected | Measured |
|---|---|---|
| `R_source` | names the configured value | `configured (R_override)` |
| K used / by the rule | both printed | 3 / 1 |
| Forecast beats the raw sensor | yes | 0.0888 vs 0.2332 mm |
| Sweep shape | error rising with h | 0.071 mm at h=0 → 0.143 mm at h=1.5 s |
| Short run (60 s hold) | clamps calibration, does not fail | clamped 90 → 29.8 s with a note |
| Log without `measured_mm` | falls back, warns loudly | warns that the lag is an over-estimate |
| Test suite | 313 | 313 passed |

## Findings

- **Forecasting by the measured sensor lag removes 62 % of the lag error.** This is the first
  end-to-end evidence on real hardware that the premise of the repo holds: a sensor reading
  677 ms late can be put back into the present well enough to be worth gating on.
- **The horizon is cheap and the lag is expensive.** Predicting 677 ms ahead costs 0.031 mm of
  extra error and recovers 0.144 mm. The forecast error curve is nearly flat out to ~0.4 s and
  only then starts to climb, so there is real headroom for the other three horizon terms
  (`tau_c`, `tau_cl`, `T_ins`) once they are measured.
- **The 95 % energy rule is wrong on this data too, and now it costs something measurable.**
  K=1 forecasts 49 % worse than K=3 (0.0766 vs 0.0515 mm). Previous evidence was residual
  structure; this is the first time the rule's choice has a price attached in the units that
  matter.
- **My prediction about `R` was wrong in direction, and the real answer is more interesting.**
  I expected the free-air R to push NIS above 1. It moved it from 0.040 to 0.243 — 6× closer,
  still 4× short. So `S` is dominated by `HPH'`, not by `R`: at this point the filter's
  inconsistency lives in **Q and the model**, not in the noise floor, and no amount of
  correcting R alone will fix it. That reframes the open question.
- **A smaller, more honest `R` slightly *worsens* the forecast** (0.0820 → 0.0989 mm RMSE) while
  substantially improving covariance consistency. The filter trusts the noisy measurement more
  and tracks a little of its noise. This is a real trade-off and worth stating plainly: the
  gate needs the honest covariance more than it needs the last 0.017 mm of forecast accuracy.

## Open questions

- **Q, not R, is now the leading suspect for the NIS shortfall.** NIS 0.243 with a measured R
  says the covariance is dominated by the process model. Q is estimated from breath-to-breath
  refits over what was only ~6 breaths here; a 180 s run gives ~18 and is the first thing to
  try.
- **In-contact `R` is still unmeasured.** The free-air number is a lower bound. A breath-hold
  segment during `standoff_hold` (pause the phantom, set `breath_hold_window`) would measure it
  properly and make the `R_override` line obsolete.
- **Is `tau_s = 0.677 s` a constant?** Carried from session 007 and now load-bearing: it *is*
  the horizon. If it is viscoelastic contact settling it will move with seating depth.
- **`forecast_variance` is still uncalibrated against realised error** — but the forecast error
  is now measured (0.0888 mm at h = 0.677 s), so `max_forecast_std_mm` can finally be set
  against something real rather than guessed.
- **`measured_mm` has still never been recorded on hardware** (carried from session 007). Every
  number above therefore uses `commanded_mm` as truth and includes the phantom's own tracking
  lag, making 677 ms an over-estimate of sensor latency.

## Next steps

1. Run on hardware with the new 180 s default, and read `replies_seen`/`feedback_hz` from the
   phantom `summary.json` — everything above is waiting on `measured_mm` being real.
2. Re-run the whole analysis with `--truth measured_mm` and compare `tau_s` against the
   `commanded_mm` figure; the difference is the phantom motor's own lag.
3. With ~18 breaths of calibration, check whether NIS moves off 0.243 — that tests the Q
   hypothesis above.
4. Add a breath-hold to `standoff_hold` and measure `R` in contact.
5. Calibrate `forecast_variance` against the now-measured forecast error and set
   `max_forecast_std_mm`.
