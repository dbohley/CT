# 017 — 2026-09-04 — Single-trial sweep collector

## Goal

Session 016's `scripts/collect_param_sweep_runs.py` looped all 5 profiles x 2 trials
automatically with an internal discard/retry. The user asked for the opposite ergonomics: one
trial per invocation, so a bad trial is just "run the same command again" rather than something
an automatic loop has to detect and recover from, and so a human is watching every physical
move rather than an unattended batch chaining through failures.

## What changed

**Replaced `scripts/collect_param_sweep_runs.py` with `scripts/collect_sweep_trial.py`.** One
trial per invocation: `--profile <p>` auto-picks the next `trial_<n>` directory for that profile
under `outputs/param_sweep_runs/<profile>/` (scanning what already exists, +1 — no manual index
tracking), runs `run_approach_and_seat.py` normally (its own "Proceed?" confirmation is kept,
not skipped, since a human is watching this one trial), reads the resulting `summary.json`,
prints a plain `GOOD` / `DISCARD -- <reason>, re-run this same command` verdict, then retracts
the base 20mm automatically either way (session 016's `--retract-only-mm`, explicitly requested
and bounded). No automatic retry — the user decides whether to re-run.

**`--list` mode** counts, per profile, how many *valid* trials already exist under the output
root and prints exactly the remaining `python scripts/collect_sweep_trial.py --profile <p>`
commands needed to reach the target (default 2 per profile) — safe to re-run any time, since it
always reflects the current state of the folder rather than a static checklist.

**Output root moved to `outputs/param_sweep_runs/`**, never `outputs/approach_and_seat/` (that
directory is for one-off bench-development runs, not the sweep's systematic set) and never a
fresh timestamp per invocation — it accumulates across separate calls to the script, which is
what makes it "the folder of data" to hand to the sweep tool.

**Extracted `resolve_profile_name()` and `validate_trial()` into `scripts/_sweep_common.py`**
so `collect_sweep_trial.py` and `sweep_ekf_params.py` share one definition of "valid trial"
rather than two copies that could drift apart.

**`scripts/sweep_ekf_params.py::resolve_run_dirs()` no longer reads a manifest** (nothing
produces one now that collection is one-trial-at-a-time) — it recursively finds every directory
with a `samples.jsonl` under the given input path(s) and filters each through `validate_trial()`
against its own `summary.json`, printing why anything invalid was skipped. Everything else (the
grid sweep, safety-first selection, plots) is unchanged from session 016.

## Files touched

| File | Change |
|---|---|
| `scripts/_sweep_common.py` | new — shared `resolve_profile_name`, `validate_trial`, defaults |
| `scripts/collect_sweep_trial.py` | new — replaces `collect_param_sweep_runs.py` |
| `scripts/collect_param_sweep_runs.py` | removed |
| `scripts/sweep_ekf_params.py` | `resolve_run_dirs` drops the manifest path, validates directly |

## Verification

```bash
conda activate CT
pytest -q
# 322 passed in 43.82s

python scripts/collect_sweep_trial.py --list
# (empty outputs/param_sweep_runs/) prints all 10 commands, 5 profiles x 2 trials

python scripts/collect_sweep_trial.py --profile emma --dry-run
# runs the trial dry, prints DISCARD (no summary.json in dry-run, correctly), retracts dry

# validate_trial, relocated, against the six real historical summary.json files:
# same 4 accept / 2 discard result as session 016

python scripts/sweep_ekf_params.py outputs/approach_and_seat/20260903-171153 \
  outputs/approach_and_seat/20260903-185614 --quick
# identical result to session 016's manifest-based run:
# locked 1/2 at best, same top-5 candidates and RMSEs

# throwaway dir with a copied good + bad summary.json (no manifest present):
# resolve_run_dirs() correctly skips the bad one and names why
```

## Next steps

Unchanged from session 016: run `collect_sweep_trial.py` at the bench (one command per trial,
per its own `--list` output) until all 5 profiles have 2 valid trials, then
`python scripts/sweep_ekf_params.py outputs/param_sweep_runs`.
