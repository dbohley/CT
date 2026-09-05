# 016 — 2026-09-03 — Multi-profile EKF parameter sweep tooling

## Goal

Session 015's `q_scale=0.5` fix was validated on one recorded run only. The user collected a
second trial of the same profile (`20260903-185614`) that looked worse by eye, and asked for a
systematic answer instead of another one-off comparison: run several breathing profiles,
several trials each, sweep the tuning parameters across all of them, and find the one setting
that works best everywhere, with results recorded and plotted. This session builds that
tooling and investigates the new run; it does not yet run the real 10-trial sweep, which needs
bench time.

## What changed

**`20260903-185614` really did lose frequency lock (6.53±2.40bpm vs 11.55) — but `q_scale=0.5`
didn't cause it.** Swept `q_scale` 0.3-1.0 on this exact run in isolation: every single value
gives a badly broken lock, including `q_scale=1.0` (the pre-session-015 default). This run has
a lock-failure mode that exists independent of `q_scale`, which is itself the important result:
`q_scale=0.5` is not a general fix, confirmed with a second real run rather than assumed from
the sweep's own non-monotonic shape.

**Extracted the frequency-lock threshold into a shared, testable function.**
`ct.diagnostics.metrics.is_frequency_locked(tracked_mean, tracked_std, ident_bpm, tolerance)`
replaces the inline check `scripts/plot_approach_and_seat.py::_report_frequency_lock` used to
hard-code, so the live report and the new offline sweep tool can never define "locked"
differently.

**`run_approach_and_seat.py` gained two small, bounded additions**, both explicitly requested
for this purpose (not autonomous/unrequested — see Decisions):
- `--retract-only-mm <mm>`: backs the base off by a given distance and exits. No sensors, no
  phantom, no approach/seat/standoff state machine — just clear-errors, enter-mode, read a
  *fresh* position (flushing queued frames first, the exact session-012 lesson applied to this
  file's own `CubeMarsMIT` codec instead of the phantom's `CubeMarsServo`), command
  `current - retract_mm`, wait for arrival, exit-mode, done. `--dry-run` prints the frames and
  opens no bus.
- `--yes`: skips the interactive `confirm("Proceed?")` before the normal four-phase run, so an
  orchestrator can call it as a subprocess without hanging on stdin (mirrors
  `run_breathing_profile.py`'s existing `--yes`).

**New `scripts/collect_param_sweep_runs.py`** — the bench orchestrator. Loops
`profile x trial` (default 5 profiles x 2 trials = 10 slots), and per slot: runs a trial,
validates its `summary.json` (`fault_reason is None`, `phase_reached == "standoff_hold"`,
`contact_t > 1.0s` — thresholds checked against the six real summary.json files on disk: four
good runs measured 2.36-6.43s, two bad ones measured 0.0063s and 0.0002s, the exact
near-instant-contact signature session 012 diagnosed by hand), retracts 20mm, and — if
invalid — retries the same slot up to `--max-retries` (default 3) rather than keeping bad data.
One confirmation before the whole batch, none per trial. If the retract step itself ever fails,
the batch stops rather than continuing from an unverified base position. I did not run this
against real hardware; verified with `--dry-run` and against the six real historical
`summary.json` files (see Verification).

**New `scripts/sweep_ekf_params.py`** — the offline sweep. Given a set of run directories (or a
collector manifest), builds `aligned.csv` for each (via
`plot_approach_and_seat.py --no-estimator`, not reimplemented), then for each run calls
`identify()` once per `q_scale` and `track()` once per `omega_bounds` fraction against that same
identification (only `q_scale` needs a fresh identification; `omega_bounds` is tracker-only).
Applies the agreed safety-first rule — disqualify any `(q_scale, omega_bounds)` that breaks
lock on *any* run, minimize mean forecast RMSE among survivors — and says plainly when nothing
survives, rather than picking the least-bad option and calling it safe. Writes
`sweep_results.csv` (every run x grid point), `sweep_summary.csv` (aggregated), `best_params.json`,
and two plots (a heatmap with disqualified cells marked and the winner highlighted; a
per-profile RMSE-vs-`q_scale` line plot at the winning `omega_bounds`).

**A real, unexpected finding from smoke-testing the sweep tool on the two existing runs
(`171153`, `185614`, sample size 2 — not the real 10-run answer, but already informative):**
`omega_bounds=±10%` keeps frequency lock on *both* runs at **every** `q_scale` value tested
(0.3 through 1.0) — something no `q_scale` value alone could do for `185614`. `omega_bounds`
directly clamps the state the redundancy problem corrupts, rather than indirectly discouraging
drift via `Q`; on this very small sample it looks like the more fundamental lever, with
`q_scale` then choosing the best point *within* the locked band (`q_scale=0.4` won on RMSE:
0.1883mm mean vs 0.1573/0.2192mm per-run). This needs the real 10-run sweep before it's
anything more than a lead worth prioritizing.

## Files touched

| File | Change |
|---|---|
| `src/ct/diagnostics/metrics.py` | added `is_frequency_locked()` |
| `scripts/plot_approach_and_seat.py` | `_report_frequency_lock` now calls the shared helper |
| `scripts/run_approach_and_seat.py` | added `--retract-only-mm` and `--yes` |
| `scripts/collect_param_sweep_runs.py` | new — bench orchestrator (user-run) |
| `scripts/sweep_ekf_params.py` | new — offline parameter sweep, aggregation, plots |

## Decisions and rationale

- **Automatic 20mm between-trial retraction is implemented despite session 014 refusing an
  auto-retract before.** That session's finding was specifically about an *unrequested*,
  unbounded retraction drafted as a side fix to a different problem — it named explicit
  authorization, at a specific bounded distance, for a specific purpose as the actual bar.
  This session clears that bar: the user asked for exactly this, bounded to 20mm (not the 60mm
  full retract), for the stated purpose of resetting between automated sweep trials. The
  loaded-tare refusal stays underneath it as the real backstop.
- **No per-trial confirmation in the collector.** The entire point of the auto-retract and
  auto-validate/retry was to let a batch run through failures unattended between trials; a
  prompt every trial would defeat that. One confirmation happens once, before the batch starts.
- **Discarded attempts are never deleted or moved**, only excluded from `accepted_dirs` in the
  manifest — matches this project's "keep the diagnostic, don't destroy it" pattern (session
  014 wished it had kept exactly this kind of data).
- **`configs/bench_aligned.yaml` is untouched this session.** The whole purpose of the sweep is
  to check whether `q_scale=0.5` generalizes; changing the config again before the real 10-run
  sweep has run would repeat exactly the mistake this session exists to avoid.

## Verification

```bash
conda activate CT
pytest -q
# 322 passed in 49.13s (after the metrics refactor, retract mode, and --yes flag)
```

```bash
python scripts/run_approach_and_seat.py --retract-only-mm 20 --dry-run
# retract-only: backing off 20.0mm at 0.100rad/s
# --dry-run: not opening any bus. (CLEAR_ERRORS, ENTER_MODE, EXIT_MODE frames printed)
# exit code: 0

python scripts/collect_param_sweep_runs.py --dry-run --yes --profiles emma --trials 1 --max-retries 1
# loop, retry, retract, and manifest writing all exercised correctly with no bus opened

python -c "... validate_trial(summary, 1.0) over the six real summary.json files ..."
# 20260902-160517: ACCEPT -- ok
# 20260903-152502: ACCEPT -- ok
# 20260903-152958: discard -- did not reach standoff_hold (stopped at 'standoff')
# 20260903-171153: ACCEPT -- ok
# 20260903-185535: discard -- faulted: the arm is swinging 7.442mm ...
# 20260903-185614: ACCEPT -- ok

python scripts/sweep_ekf_params.py outputs/approach_and_seat/20260903-171153 \
  outputs/approach_and_seat/20260903-185614
# BEST (safety-first): q_scale=0.4, omega_bounds=0.1 -- locked on 2/2 runs,
# mean forecast RMSE 0.1883mm
```

| Check | Expected | Measured |
|---|---|---|
| `is_frequency_locked` refactor changes no behavior | 322 tests still pass | Confirmed |
| Retract-only mode dry-run opens no bus | prints frames, exit 0 | Confirmed |
| Collector dry-run exercises full loop with no hardware | manifest correctly records discard/retry | Confirmed |
| Validator matches real historical data | 4 good accepted, 2 bad discarded, for the right reasons | Confirmed |
| Sweep tool runs end-to-end on real (if only 2) runs | produces CSVs, plots, a safety-first pick | Confirmed |

## Findings

See "What changed" above for the `185614` lock-failure result and the `omega_bounds` lead —
both promoted to `CLAUDE.md`.

## Open questions

- **Does `omega_bounds≈±10%` generalize as the primary fix, with `q_scale` secondary?** New
  this session, on a sample of 2 runs. The real 10-run sweep (5 profiles x 2 trials) is what
  answers this.
- Everything carried from session 015 (in-contact `R`, the trough-degradation mechanism,
  whether `q_scale=0.5` generalizes) is superseded by this question rather than separately
  open — the sweep tool now answers all of them at once once real data exists.

## Next steps

1. User runs `python scripts/collect_param_sweep_runs.py` at the bench (real hardware, ~10
   trials with automatic retry on bad data).
2. Run `python scripts/sweep_ekf_params.py outputs/param_sweep/<ts>/raw` on the result.
3. Only then: update `configs/bench_aligned.yaml`'s `q_scale` (and possibly add
   `tracker.params.omega_bounds`) from `best_params.json`, cited to that real sweep — not to
   this session's 2-run smoke test.
