# 020 — 2026-09-07 — in-tissue needle identification

## Goal

Session 019 measured `servo.needle.plant` entirely in free air — `src/ct/unknowns.py`'s own
`how_to_measure` text for that entry says "out of tissue." With the needle already seated in
the phantom, redo the measurement in the representative condition (in tissue), settle whether
that changes the identified plant, and — along the way — chase down session 019's unresolved
Finding 3 (only the first commanded step in any run fits cleanly). Both landed: the in-tissue
plant is now measured and confirmed consistent with free air, and Finding 3's two leading
hypotheses (tissue creep, stale mode-entry state) are each tested and ruled out.

## What changed

**`scripts/run_needle_step_response.py`**: net change is one new flag,
`--reenter-mode-per-rep` (resends `CLEAR_ERRORS`+`ENTER_MODE` before every rep's step phase,
not just once at the top of the run) — a diagnostic built to test one hypothesis for Finding 3.
A second change, `--tissue-margin-mm` (a safety guard capping `--max-step-travel-mm` against an
operator-judged safe-forward-push distance), was added and then explicitly reverted at the
user's request ("I don't want to change things unnecessarily") once it was clear the existing
15mm/20mm defaults already sat comfortably inside the confirmed ~30mm margin — verified byte-
for-byte via `--dry-run` output matching session 019's original exactly, and `pytest -q`.

**No config or `unknowns.py` changes.** The in-tissue measurement (see Findings) converged to
values statistically indistinguishable from the free-air ones already in
`configs/rig_bench.yaml`, so nothing needed updating — see Decisions for why a small nudge
wasn't worth making either.

**Considered and explicitly rejected**: a new `rig.geometry.phantom_forward_margin_mm`
`Unknown` entry for the confirmed travel margin. `tests/test_unknowns.py` enforces that every
`Unknown.key` resolves against a real key in `configs/rig_sim.yaml`, and config sections are
strictly schema-validated (`_unexpected()` rejects unrecognized keys) — so formalizing this
would have meant adding a real dataclass field to the geometry config and wiring it into both
shipped configs, for a number that is an operator's bench judgment call, not something any
simulation or the real procedure consumes. Disproportionate; the margin stays local to context
(this doc, and the conversation that produced it).

## Files touched

| File | Change |
|---|---|
| `scripts/run_needle_step_response.py` | added `--reenter-mode-per-rep` (kept); added then reverted `--tissue-margin-mm` (net no change) |

## Decisions and rationale

**Kept the step-response method rather than switching to impulse.** For an LTI system the
impulse response is the step response's derivative — the transient region already being fit
carries the same identifying information. An impulse would also fight the identical telemetry
resolution ceiling session 019 already found (position only updates once per command ACK,
~20-50Hz) with no specific reason to expect it resolves better, and would have changed two
things at once (signal shape *and* physical setup) when only one needed to change to test the
real question (does tissue change the plant?).

**Did not add a mass-derived cross-check to the plant, despite a measured mass being offered.**
Worked through this with the user directly: the model fit here (`K, wn, zeta` from a type-1
transfer function) is empirical/black-box — nothing downstream (`residual_lag()`, `bandwidth()`,
the lead-compensator design) ever needs mass, stiffness, or damping individually, only `wn`,
`zeta`, `K` evaluated as `G(jω)`. More importantly, if mass is used to solve the classical
`wn=sqrt(k/m)`, `zeta=b/(2·sqrt(km))` relations for `k` and `b` *from the already-measured*
`wn`/`zeta`, recombining them algebraically returns the identical `wn`/`zeta` — mass cancels
out exactly, so there is no independent second estimate to compare against the empirical one.
(Separately, this axis rotates, so the physically correct quantity would be moment of inertia,
not mass; and the type-1 structure — command-to-*velocity*, not force-to-position — means there
may be no literal spring for "k" to describe in the first place.) A genuine independent
cross-check would need mass *plus* something independently known (a stiffness number, or the
drive's own control-loop bandwidth spec) — neither was available.

**Chased Finding 3 with two cheap, falsifiable hypotheses rather than immediately falling back
to brute-force averaging.** Both were specific, testable, and wrong, which is progress:
1. *Tissue creep/relaxation* — ruled out because the identical only-the-first-rep-is-clean
   pattern showed up in free air (session 019) where there is no tissue to creep.
2. *Stale mode-entry state* — the hypothesis that only a fresh `CLEAR_ERRORS`+`ENTER_MODE`
   produces a clean response, since normally that pair is sent once per run, only before rep 0.
   Tested directly by resending it before every rep (`--reenter-mode-per-rep`); reps 1-4 were
   still degenerate afterward (see Verification) — ruled out.

**Fell back to collecting more independent single-rep trials rather than continuing to guess
causes.** With two plausible, cheap hypotheses eliminated and the specific *shape* of the
degeneracy changing between attempts (bounds-pegged high one run, bounds-pegged low the next),
this doesn't look like a single simple bug findable by more guessing. Rep 0 of any given
invocation has been reliably clean across every run this project has done; the practical fix is
more independent rep-0s, not fixing reps 1+.

**Dropped one of ten cold-start trials as an outlier, not as cherry-picking.** Trial 0 of the
10-trial batch (`wn=138.14`, `zeta=2.156`) sits roughly 30 standard deviations from the other
nine, which cluster tightly (`wn` 22.4-34.8, `zeta` 0.09-0.45). That is a categorical difference,
not a borderline judgment call — even the "reliable" first-command condition is not immune to
an occasional bad trial, which argues for always collecting several and screening for outliers
rather than trusting any single trial blindly.

**Left `configs/rig_bench.yaml` unchanged rather than nudging it toward the new numbers.** The
tightened in-tissue estimate (`K=0.900±0.025`, `wn=27.2±3.7`, `zeta=0.306±0.095`, n=10) is
statistically indistinguishable from the free-air one already in config
(`K=0.915±0.025`, `wn=27.1±0.8`, `zeta=0.35±0.11`, n=4) — the differences are well within each
estimate's own noise. Changing the config for a difference that small, in either direction,
would be cosmetic rather than a real update; the value of this session's work is the
*confirmation*, not a new number.

## Verification

```bash
conda activate CT && pytest -q
# 322 passed, throughout (checked after the --tissue-margin-mm add, the revert, and the
# --reenter-mode-per-rep add)
```

**Revert check**: `python scripts/run_needle_step_response.py --dry-run` output after reverting
matched session 019's original output byte-for-byte (same banner text, same frame sequence, no
tissue-margin mentions).

**In-tissue characterization, first attempt** (`outputs/needle_plant_characterization/
20260907-141700`, 3 cold-start + `v=0.15,0.30` warm batches): reproduced session 019's
only-rep-0-is-clean pattern exactly, in tissue this time. Clean (first-command) trials only:

| | value (n=4) |
|---|---|
| `K` | 0.893 ± 0.022 (2.5%) |
| `wn` | 39.2 ± 12.5 (32%) |
| `zeta` | 0.352 ± 0.168 (48%) |

**Mode-entry hypothesis test** (`outputs/needle_step_response/20260907-144125`,
`--reenter-mode-per-rep`, 5 reps @ 0.15rad/s): `retract_incomplete: 0` confirmed clean
retraction throughout. Fit result:

| rep | `wn` | `zeta` | `K` |
|---|---|---|---|
| 0 | 25.09 | 0.432 | 0.893 |
| 1 | 301.53 | 0.044 | 0.889 |
| 2 | 304.96 | 0.050 | 0.938 |
| 3 | 299.49 | 0.027 | 0.906 |
| 4 | 294.77 | 0.062 | 0.906 |

Reps 1-4 still degenerate — hypothesis rejected (see Findings/Decisions).

**10-trial cold-start collection** (`outputs/needle_plant_characterization/20260907-145116`,
`--cold-start-reps 10 --velocities 0.15 --reps 1`):

| trial | `K` | `wn` | `zeta` |
|---|---|---|---|
| 0 | 0.921 | **138.14** | **2.156** |
| 1 | 0.906 | 27.78 | 0.331 |
| 2 | 0.907 | 34.75 | 0.087 |
| 3 | 0.910 | 31.64 | 0.289 |
| 4 | 0.890 | 30.07 | 0.283 |
| 5 | 0.950 | 24.22 | 0.448 |
| 6 | 0.877 | 24.99 | 0.305 |
| 7 | 0.930 | 26.86 | 0.424 |
| 8 | 0.858 | 24.38 | 0.296 |
| 9 | 0.885 | 22.43 | 0.240 |
| warm rep0 | 0.884 | 24.93 | 0.359 |

Dropping trial 0 (outlier, see Decisions), n=10 (trials 1-9 + warm rep0):

| | value |
|---|---|
| `K` | 0.900 ± 0.025 (2.8%) |
| `wn` | 27.2 ± 3.7 (13.5%) |
| `zeta` | 0.306 ± 0.095 (31%) |

## Findings

**1. Only the very first commanded step in any invocation fits cleanly — confirmed in a second
medium and immune to two specific fixes.** Session 019 found this in free air; this session
reproduced it in tissue (Verification, first attempt) and found the specific *shape* of the
degeneracy is not even consistent between attempts — `zeta` pegged at its upper bound (5.0) in
one run, its lower bound (~0.03-0.06) with `wn` near 300 in another. Two plausible, cheap causes
were tested and eliminated (tissue creep — ruled out by reproducing the pattern in free air;
stale mode-entry state — ruled out by resending `CLEAR_ERRORS`+`ENTER_MODE` per rep and seeing
reps 1-4 stay broken). The cause remains unknown. The practical implication is settled, though:
treat every rep after the first, in any single invocation, as unusable, and get more data by
running more separate invocations, not more reps per invocation.

**2. Even "reliable" first-command trials aren't immune to an occasional bad one.** 1 of 10
cold-start trials (each independently the "clean" condition per Finding 1) came out a
30-standard-deviation outlier. A single trial, however clean its provenance, should not be
trusted without at least a few others to check it against.

**3. In-tissue and free-air plant estimates agree, once enough trials exist to see past the
noise.** With n=4, in-tissue `wn` (39.2±12.5) looked meaningfully different from free-air `wn`
(27.1±0.8). With n=10, it converges to 27.2±3.7 — matching free air closely. **The apparent
tissue effect was a small-sample noise artifact, not a real physical difference.** At the
velocity and excursion tested (0.15rad/s, ~15mm target offset), tissue contact does not
measurably change this axis's identified second-order dynamics. `K` (settled velocity as a
fraction of commanded limit) also agrees closely across every measurement this project has made
of this axis: 0.915 (free air, n=4), 0.893 (in-tissue, n=4), 0.900 (in-tissue, n=10) — all
within a few percent of each other and of nothing else.

**4. Mass alone cannot produce an independent second plant estimate from the same step-response
data**, regardless of how it's used — see Decisions for the full algebraic argument. Worth
remembering if this comes up again: the fix is to seek genuinely independent data (a second
measurement method, or an independently-known stiffness/bandwidth number), not to reprocess the
same numbers through a different parameterization.

## Open questions

- **What actually causes only the first commanded step in an invocation to fit cleanly?**
  Carried from session 019, now with two specific candidate explanations eliminated (tissue
  creep, stale mode-entry state). Remaining candidates are speculative: some other drive-internal
  state that a mode re-entry doesn't reset (adaptive gain, a velocity-loop integrator that only
  zeroes on power-up, thermal/current history), or something about the retract-then-immediately-
  step-again sequencing itself. Not chased further this session — diminishing returns on guessing
  without a new falsifiable hypothesis.
- Whether the plant is genuinely amplitude-dependent (wn possibly higher at 0.30rad/s than
  0.15rad/s) is still only supported by single, unreplicated clean trials at 0.30rad/s from
  session 019 and this session's first in-tissue attempt — not re-examined with the same
  n=10-trial rigor this session applied at 0.15rad/s.
- Whether `configs/rig_bench.yaml`'s `servo.needle.plant` should eventually be updated to a
  pooled free-air+in-tissue estimate is deliberately left as a non-decision — the two are close
  enough that it wouldn't change anything meaningful, but a future session with more data at
  more velocities might want to.

## Next steps

1. If Finding 1's root cause matters enough to chase further, the next falsifiable hypothesis
   would need to target something *other* than elapsed time, medium, or mode-entry state —
   candidates above are speculative and none are cheap to test yet.
2. Re-derive the lead compensator's design if the plant estimate ever changes materially — not
   needed now, since this session's numbers confirm rather than revise `configs/rig_bench.yaml`.
3. If a paper section discusses in-tissue vs. free-air actuator characterization, this session's
   Finding 3 (n=4 vs n=10 convergence) is a clean, citable illustration of why small-sample
   system-ID claims need replication before being trusted — worth keeping the raw run
   directories (`outputs/needle_step_response/20260907-144125`,
   `outputs/needle_plant_characterization/20260907-141700` and `-145116`) rather than only the
   summary numbers here.
