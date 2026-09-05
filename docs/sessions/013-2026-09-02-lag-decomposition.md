# 013 — 2026-09-02 — lag-decomposition

## Goal

`tau_s = 0.677 s` was never credible for a sensor whose frames arrive every ~8 ms. Find out
what that number actually is, stop it being a frozen constant, and make the estimator panels
readable enough to tell the reconstruction, the EKF and the forecast apart.

## What changed

**The sensing lag is now decomposed and measured per run.** `compare_logs` gained a
`sensor_field` so the ToF can be scored as well as the tactile arm. The ToF is non-contact but
sits on the same CAN bus, in the same tick loop, watching the same motion — so it measures the
sensing chain alone, and everything above it is mechanical.

**The horizon is assembled from the run being plotted**, not read from a config. It used to be
`horizon: 0.677` in `configs/bench_aligned.yaml`, which is one run's figure reused for every
later run.

**Four defects in the lag measurement**, all of which mattered once the number started driving
the horizon:

- **No sub-sample resolution.** A raw `argmax` quantised the lag to one grid sample, 7.9 ms at
  the bench's 127 Hz. Now parabolic, reusing `_parabolic_offset_linear` from
  `identification/spectral.py`.
- **The correlation was unnormalised.** `np.correlate` is a bare dot product over `n-|k|`
  terms, so the shrinking overlap imposed a triangular taper pulling the peak toward zero.
  Normalising by the overlap *count* over-corrects and pushes it the other way — on a
  synthetic 0.235 s lag that landed 0.9 samples high, worse than not interpolating at all.
  Dividing by `sqrt(Ec*Ep)`, the per-shift correlation coefficient, is unbiased.
- **No periodicity guard.** Breathing is periodic, so the correlation surface is too: there is
  a sidelobe every `T_breath` and an argmax cannot prefer the true one. The search is now
  clamped to just inside `T/2` with a `lag_ambiguous` flag. Not hypothetical — a ±3 s scan of
  the ToF against this bench's ~5.8 s breathing returned **−2.77 s**.
- **Sensor polarity was assumed.** ToF *distance* shrinks as the surface advances, so it
  anti-correlates with phantom position. With signed search it pinned to ±1.998 s of a 2.0 s
  window hunting a positive lobe. `SENSOR_SIGN` orients each column up front; discovering it
  per run is not even possible, since for a near-sinusoid an inverted sensor is
  indistinguishable from a correctly-signed one half a breath away.

**The estimator panels are cropped, recoloured and separated**, and a frequency-lock
diagnostic was added — see Findings, because it exposed something serious.

## Files touched

| File | Change |
|---|---|
| `src/ct/phantom/driver.py` | `sensor_field` + `SENSOR_SIGN`; normalized correlation; sub-sample peak; breath-period clamp and `lag_ambiguous`; new result keys |
| `src/ct/cli/compare.py` | `--sensor` / `--no-split`; reports floor / total / contact excess; corrected the `latency.tau_s` advice, which named the contact-inclusive number |
| `scripts/plot_approach_and_seat.py` | per-run horizon via `horizon_from_components`; ToF reference; seating-quality and frequency-lock reports; panels cropped to the tracked window, recoloured, y-centred; sweep retargeted |
| `configs/bench_aligned.yaml` | frozen `0.677` removed, with the reasoning recorded |
| `src/ct/unknowns.py` | `latency.tau_s` now says how to measure the floor and why the tactile figure is not it |
| `tests/test_phantom.py` | 4 new tests; `_logs` can emit an inverted `tof_mm` |

## Verification

```bash
conda activate CT
pytest -q
# 317 passed in 27.72s
```

**The decomposition, over `standoff_hold` against `measured_mm`:**

| run | tactile lag | ToF lag | contact excess | r | amplitude |
|---|---|---|---|---|---|
| 20260902-160517 | 0.282 s | 0.098 s | 0.183 s | 0.992 | 0.760 |
| 20260902-140021 | 0.297 s | 0.046 s | 0.252 s | 0.918 | 0.298 |
| 20260902-141144 | 0.327 s | 0.011 s | 0.316 s | 0.942 | 0.302 |
| 20260902-142345 | 0.359 s | 0.054 s | 0.305 s | 0.868 | 0.160 |
| 20260902-144232 | 0.397 s | 0.030 s | 0.367 s | 0.897 | 0.396 |
| 20260901-234036 | 0.562 s | 0.023 s | 0.539 s | 0.806 | 0.163 |
| 20260901-165415 ‡ | 0.696 s | 0.074 s | 0.622 s | 0.961 | 0.326 |

‡ scored against `commanded_mm` — the only run on disk with no `measured_mm`, and the run
0.677 came from. Its figure therefore also carries the phantom motor's own ~0.05 s tracking
lag (0.330 vs 0.282 measured both ways on run 160517).

**Regression check:** run 165415 still reports 60.5% of lag error removed against session
008's 62%, and 0.0911 mm against 0.0888 mm. The small differences are the sub-sample lag
refinement (0.696 vs 0.677). The historical result reproduces.

**Frequency lock predicts forecast quality, across every run on disk.** Each is now scored at
its own measured lag, so the horizon is no longer a confound:

| run | Stage 1 | tracked | ratio | lag error removed |
|---|---|---|---|---|
| 20260901-165415 | 12.17 bpm | **11.96 ± 1.18** | 0.98 | **+60.5%** |
| 20260902-140021 | 11.43 bpm | 9.96 ± 2.07 | 0.87 | +1.3% |
| 20260902-144232 | 11.29 bpm | 7.43 ± 3.48 | 0.66 | −2.8% |
| 20260902-160517 | 10.52 bpm | 4.55 ± 3.60 | 0.43 | −9.0% |
| 20260902-142345 | 10.83 bpm | 3.92 ± 3.53 | 0.36 | −11.9% |
| 20260902-141144 | 11.22 bpm | 4.22 ± 3.43 | 0.38 | −19.0% |
| 20260901-234036 | 11.49 bpm | 2.42 ± 3.08 | 0.21 | −23.4% |

Monotonic in the ratio, with the one run that holds lock the only one that forecasts. That is
about as direct as evidence gets that the forecast failure *is* the frequency failure, and not
the lag measurement, the horizon, or the sensing chain.

## Findings

- **0.677 s was never sensor latency, and it is not a constant.** The ToF floor is
  0.011-0.098 s across seven runs — consistent with the ~8 ms frame period plus the ~11 ms
  tick — and the tactile arm reads 0.282-0.696 s. The 0.19-0.62 s difference is viscoelastic
  settling in the *contact*. This closes the open question carried since session 007.
- **Pressing harder makes the sensing worse, in both respects.** Correlation with the accepted
  seat peak is **+0.63 for the lag and −0.84 for the amplitude ratio**. The lightest seat
  (0.362 mm) gave 0.282 s / 0.760 / r=0.992; the heaviest (1.93 mm) gave 0.562 s / 0.163 /
  r=0.806. Deeper seating engages more material in a softer, more dissipative regime and loads
  the pivot harder. **Seating lighter buys more than any amount of forecasting.**
- **The lag explains almost none of the amplitude loss.** Splitting the measured gain by the
  attenuation the lag alone would cause (`1/sqrt(1+(ωτ)²)`, which is 0.83-0.95 for every run)
  leaves a *static* gain of 0.82 down to 0.19. Delay and attenuation are two effects that
  happen to degrade together, not one effect seen twice.
- **Stiction is part of it.** Samples with *exactly* zero change go from 6.8% at the lightest
  seat to 16.3% at the heaviest. A stuck arm does not move until force beats breakaway
  friction, delaying every reversal — and it worsens as the driving amplitude at the arm
  shrinks, which the static loss causes. The two compound.
- **The EKF loses frequency lock on real subject profiles, and `y_pred` hides it completely.**
  This is the serious one. On run 20260902-160517 Stage 1 identifies 10.52 bpm against a true
  10.34, and the tracker then runs at **4.55 ± 3.60 bpm** with `phi_1` spinning **9.70 rad** to
  absorb the error. `A_k`, `phi_k` and `(theta, omega_r)` are partially redundant, so a
  drifting `phi_1` mimics a frequency offset and the measurement still fits perfectly — the
  tracking panel looks flawless. Only the forecast suffers, because forecasting is the one
  operation that uses `omega_r` alone: asked to advance 0.279 s it advances ~0.031 s, leaving
  a **+0.248 s residual lag** and a −8.7% "improvement". Run 165415 holds 11.96 ± 1.18 against
  12.19, `phi_1` drifts 0.15 rad, and its forecast works.
- **Session 008's +62% headline was measured on the one synthetic looping profile.**
  `breathing_profile_1` is 14.8 s, loops 12× in a 180 s hold, and has **zero** baseline wander.
  `emma_normal_breathing` is 618 s, completes 0.29 loops, and carries 0.267 mm of slow wander
  on a 4.62 mm excursion. Every run on the real profile loses lock (NIS 0.007-0.020); the one
  on the looping profile does not (NIS 0.245). This is session 009's "anything tuned against a
  short looping profile should be re-checked against a real one", now showing up in the
  estimator rather than the approach.
- **`bus.recv()`-style staleness has a cousin in correlation search ranges.** Two of the four
  measurement defects above are the same mistake: assuming a search or a read is unambiguous
  when the signal's own structure says it is not.

## Open questions

- **Why does the tracker lose frequency lock, and what fixes it?** Q and the phi/omega
  observability trade are the suspects, not R — the filter is over-confident (NIS 0.008) while
  holding a badly wrong `omega_r`. Options worth trying: a tighter prior on `omega_r`,
  per-state Q rebalancing, or constraining `phi_k` drift. **Nothing downstream of the forecast
  should be trusted on a real subject profile until this is resolved.**
- **Is the contact lag a delay or a filter?** Phase lag per harmonic on run 160517 is 0.287 s
  at the fundamental but 0.407 s at harmonic 2, where a pure delay predicts a constant. If
  that holds, harmonic `k` needs a different advance than `k·ω·h` and CLAUDE.md's settled
  decision 4 is right for delay but incomplete for this contact. **Flagged, not concluded** —
  harmonic 2 carries 5.7% of the fundamental's energy and harmonic 3 is noise.
- **Would a non-contact sensor remove the problem entirely?** The ToF already sees 0.79-1.10 of
  real excursion at under 0.1 s, against the tactile arm's 0.16-0.76 at 0.28-0.70 s. It is
  quantised to 1 mm, but that is a resolution problem with known fixes, where the contact lag
  is physics.
- Carried: in-contact `R`; how much approach travel a seated contact needs; standoff having no
  distance bound.

## Next steps

1. **Chase the frequency lock.** It is now the binding constraint on everything the forecast
   feeds. Start by re-running `bench_aligned` with a tighter `omega_r` prior and watching the
   `frequency lock` line.
2. Re-check the seating recommendation on hardware: deliberately seat light (target a ~0.4 mm
   accepted peak) and confirm the lag and amplitude land near run 160517's.
3. Consider whether the ToF, dequantised or averaged, is a better estimator input than the
   tactile arm — it is 3-10× faster and sees 3× more of the motion.
