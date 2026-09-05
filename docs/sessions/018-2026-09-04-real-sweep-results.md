# 018 — 2026-09-04 — Real sweep results, cleanup, and the omega_bounds fix

## Goal

Act on the first real 10-run parameter sweep (5 profiles x 2 valid trials, collected via
`collect_sweep_trial.py`): clean up the 14 invalid trial folders cluttering
`outputs/param_sweep_runs/`, look at the sweep's own visuals, build a dedicated zoomed visual
of the EKF's frequency-lock fix and forecast (mirroring session 015's `plot_lag_detail.py`),
and investigate a "massive FFT vs autocorrelation disagreement" warning noticed while running
the sweep. Along the way, the real sweep showed the currently-configured `q_scale=0.5` (session
015, validated on one run) is only safe on 2 of these 10 real runs — fixing that properly is
also part of this session, at the user's explicit request.

## What changed

**Cleaned up 14 invalid trial folders**, all confirmed by `validate_trial()`: `derek trial_1,3`,
`emma trial_1,3`, `jake trial_1,2,3,5`, `junrong trial_1,3`, `patient1 trial_1,2,3,5` — every
odd-numbered trial faulted on a loaded tactile tare (arm swinging 6-9mm at tare time, i.e. still
in contact from the previous trial), consistent with the base not having been manually
repositioned between some of these collection runs. New `scripts/clean_bad_trials.py` lists
before deleting (`--delete` required to actually remove), so this is a reviewed command rather
than a repeat of the `rm -rf` that destroyed a real trial earlier this week.

**The real sweep result, confirmed at full scale**: `q_scale=0.5` + `omega_bounds` at ±10% of
Stage 1's own rate is the *only* setting (of 44 tested) that keeps frequency lock on all 10 real
runs — mean forecast RMSE 0.293mm, max 0.591mm. This is exactly session 016's n=2 lead,
generalizing cleanly to the full set. **The currently-configured `q_scale=0.5` with no
`omega_bounds` keeps lock on only 2 of these 10 runs** — `sweep_summary.csv`'s
`q_scale=0.5,omega_bounds=unset` row. Session 015's fix was real but incomplete; it is the pair,
not `q_scale` alone, that is validated.

**Fixed a real bug found while re-verifying this after wiring `omega_bounds_fraction` into the
config** (below): `sweep_ekf_params.py`'s per-grid-point tracker params stripped the resolved
`omega_bounds` key from the base config but not the new `omega_bounds_fraction` key, so once
`configs/bench_aligned.yaml` carried `omega_bounds_fraction: 0.1` as a baseline default, every
"unset" grid point silently inherited it anyway — the sweep briefly reported `omega_bounds=unset`
locking 10/10 with RMSE numbers bit-identical to the real `0.1` result, which is what gave it
away. Fixed by stripping both keys before adding the grid's own value; re-verified the corrected
numbers exactly reproduce the original (pre-config-edit) result: `unset` back to 2/10 locked,
`0.1` still 10/10 at the same RMSE.

**Wired `omega_bounds_fraction` into the real pipeline.** New
`ct.run.resolve_tracker_params(params, ident_bpm)` resolves a fractional bound into an absolute
`omega_bounds = bpm_to_omega(ident_bpm) * (1 +- frac)` at runtime — a fixed rad/s range in a
config can't generalize across the 10-22bpm spread of real subjects tested, only a bound
relative to *this run's own* Stage-1 rate can. Wired into the three call sites that actually
consume `bench_aligned.yaml`-style configs: `run_pipeline()`, `sweep_horizons()`
(`src/ct/run.py`), and `ct-track` (`src/ct/cli/track.py`). Not wired into `rig.py` /
`control/context.py` (the real `ct-rig` procedure, which consumes `rig_bench.yaml`/`rig_sim.yaml`
— untouched here, and per CLAUDE.md has never run on real hardware). `sweep_ekf_params.py` now
calls the same shared helper instead of its own copy of the fraction math.
`configs/bench_aligned.yaml` updated: `q_scale: 0.5` (kept) + `tracker.params.
omega_bounds_fraction: 0.1` (new), with the `q_scale` comment corrected to say plainly that
`q_scale` alone is not the validated fix.

**New `scripts/plot_ekf_detail.py`** — the zoomed, dedicated visual asked for. Two panels:
tracked `omega_r` over the whole tracking window for baseline (`q_scale=0.5`, no bounds) vs
tuned (`q_scale=0.5`, `omega_bounds_fraction=0.1`), with Stage 1's rate as a reference line; and
forecast vs raw sensor vs truth zoomed to a few breaths, RMSE annotated. On
`emma_normal_breathing/trial_2`: baseline collapses continuously from ~11.4bpm to under 1bpm
over 90s with no recovery; tuned holds 10.40+-0.17bpm the whole time. **Deliberately not
oversold**: on this same run, the tuned config's forecast RMSE (0.559-0.672mm, depending on
which horizon is used) is *worse* than just reading the raw sensor late (0.267-0.310mm) — fixing
lock is not the same as making the forecast trustworthy on every run, and the plot and this
write-up both say so rather than implying a uniform win.

**New `scripts/analyze_frequency_disagreement.py`** — the "massive FFT vs autocorrelation
differences" investigation. It's real, not a bug: `ct.identification.spectral.coarse_omega`
warns above 10% disagreement between the two frequency estimators. Checked against the 10 real
runs: Stage 1's own headline disagreement ranges 0.3%-10.8% (junrong 10.7-10.8%, derek
9.0-9.5%, jake trial_4 7.7%, the rest under 6%) — junrong and derek both close to the threshold,
and derek's result lines up with session 006's already-documented finding that *this exact
subject's* breathing rate drifts within a take. The "massive" impression during a real sweep
run is a volume effect, not a magnitude one: `sweep_ekf_params.py` calls `identify()` 11 times
per run (once per `q_scale`), and each of those calls `coarse_omega` again per Q's sliding
sub-window (0-6 more times, sometimes disagreeing far worse than the headline number — one
sub-window on this data disagreed by 69.8%) — a real sweep run fires this warning 100+ times,
each with a distinct embedded percentage so Python's default warning dedup never collapses them.

## Files touched

| File | Change |
|---|---|
| `scripts/clean_bad_trials.py` | new — list/delete invalid trial folders |
| `scripts/plot_ekf_detail.py` | new — zoomed frequency-lock + forecast visual |
| `scripts/analyze_frequency_disagreement.py` | new — per-run FFT-vs-autocorrelation summary |
| `src/ct/run.py` | new `resolve_tracker_params()`, wired into `run_pipeline`/`sweep_horizons` |
| `src/ct/cli/track.py` | `ct-track` now resolves `omega_bounds_fraction` too |
| `scripts/sweep_ekf_params.py` | uses the shared helper; fixed the leak bug above |
| `configs/bench_aligned.yaml` | added `tracker.params.omega_bounds_fraction: 0.1`, corrected the `q_scale` comment |
| `outputs/param_sweep_runs/` | 14 invalid trial folders removed |

## Decisions and rationale

- **`clean_bad_trials.py` defaults to listing, not deleting.** Directly motivated by deleting a
  real trial by hand earlier this week without checking its contents first — this tool makes
  the list explicit and requires `--delete` to act on it, and uses the exact same
  `validate_trial()` the sweep itself trusts, so nothing gets removed for a reason the sweep
  would not also have excluded it for.
- **`omega_bounds_fraction`, not a literal `omega_bounds`, in the config.** A fixed absolute
  range would have to cover every subject's rate (10-22bpm here) to avoid clipping some of them,
  which defeats the tight per-subject bound that is what fixed lock in the first place.
- **Did not wire the fix into `ct.rig`/`control/context.py`.** Those consume a different config
  family, never touched here, and that code path has never run on real hardware per CLAUDE.md —
  adding support for a key no config there uses yet would be unused code.
- **The EKF detail plot shows the honest RMSE, not just the lock fix.** It would have been easy
  to show only the frequency-lock panel (a clean, dramatic win) and leave out the forecast RMSE
  panel (a genuinely mixed result on this run) — the whole point of this project's session
  discipline is not doing that.

## Verification

```bash
conda activate CT
pytest -q
# 322 passed in 27.77s
```

```bash
python scripts/clean_bad_trials.py
# lists exactly the 14 folders named above
python scripts/clean_bad_trials.py --delete
# removes them; find outputs/param_sweep_runs -maxdepth 2 confirms only the 10 valid trials remain

python scripts/sweep_ekf_params.py outputs/param_sweep_runs
# BEST (safety-first): q_scale=0.5, omega_bounds=0.1 -- locked on 10/10 runs,
# mean forecast RMSE 0.2932mm
# (bug caught and fixed mid-session: before the fix this line incorrectly read
#  omega_bounds=unset, locked 10/10 -- reproducing 0.1's exact RMSE, the tell)

python scripts/plot_ekf_detail.py outputs/param_sweep_runs/emma_normal_breathing/trial_2
# baseline 5.12+-3.25bpm (collapsing to <1bpm over 90s) vs tuned 10.40+-0.17bpm (truth 11.46)
# forecast RMSE 0.5589mm vs raw sensor RMSE 0.2672mm -- worse, honestly reported

python scripts/analyze_frequency_disagreement.py outputs/param_sweep_runs
# junrong_normal_breathing/trial_4   10.8%  14.66bpm   2 sub-window warnings
# junrong_normal_breathing/trial_2   10.7%  14.67bpm   2
# derek_normal_breathing/trial_2      9.5%  12.02bpm   6
# derek_normal_breathing/trial_4      9.0%  12.02bpm   5
# ... (jake trial_6 lowest at 0.3%)
```

| Check | Expected | Measured |
|---|---|---|
| Cleanup removes exactly the invalid folders | 14 folders, all fault-explained | Confirmed |
| `q_scale=0.5`+`omega_bounds=0.1` generalizes to all 10 real runs | locked 10/10 | Confirmed, mean RMSE 0.293mm |
| Current config (`q_scale=0.5` alone) is unsafe | locked on some but not all | 2/10 |
| `omega_bounds_fraction` resolves correctly end-to-end | `ct-pipeline`/`plot_approach_and_seat.py` run without error | Confirmed, frequency lock matches `plot_ekf_detail.py`'s tuned side exactly (10.40+-0.17bpm) |
| FFT/autocorrelation "massive" difference explained | volume vs magnitude | Confirmed: headline disagreement 0.3-10.8%, sub-window warnings 0-6/run, ~100+ total per sweep |

## Findings

See "What changed" above; promoted to `CLAUDE.md`.

## Open questions

- **Forecast RMSE is not reliably improved by the lock fix, even though lock itself is.**
  `emma_normal_breathing/trial_2`'s forecast is worse than the raw sensor under the tuned
  config. The aggregate mean (0.293mm) is a reasonable number but individual runs vary a lot
  (max 0.591mm) — worth checking whether this correlates with seat depth or subject the same
  way session 013's lag/amplitude findings did, before trusting per-run forecast numbers.
- **Junrong and derek's real, moderate FFT/autocorrelation disagreement (9-11%) is not yet
  acted on** — session 006 already flagged derek as non-stationary; junrong is a new addition to
  that list. Neither has been checked for whether a shorter or stationarity-gated calibration
  window would help, the way session 006 speculated for derek.

## Next steps

1. If more trials get collected later, re-run `sweep_ekf_params.py` and check whether
   `omega_bounds_fraction=0.1` and `q_scale=0.5` remain the winners, or whether a larger sample
   moves them.
2. Investigate why forecast RMSE doesn't track lock quality per-run (open question above).
3. Consider a stationarity guard for calibration windows on subjects like derek/junrong whose
   FFT/autocorrelation disagreement sits near the 10% threshold.
