# 019 — 2026-09-05 — needle-step-response

## Goal

Get oriented on needle-axis actuation ahead of full-system integration: understand what
designing the needle's lead compensator actually requires, measure the real needle plant
(`servo.needle.plant` in `src/ct/unknowns.py`, a session-002-era placeholder) via a
step-response test, re-derive the lead compensator against it, and build a way to see the
difference compensation makes. All four landed by the end of the session, though the plant
estimate is explicitly provisional and one significant, previously-unknown discrepancy in the
project's core `tau_cl` formula was found along the way (see Findings 6).

## What changed

**Step-response measurement tooling** (`scripts/run_needle_step_response.py`,
`scripts/plot_needle_step_response.py`, `scripts/fit_needle_plant.py`): commands the needle to
a far, safety-capped position target at a fixed velocity limit (the real motor takes no direct
velocity command, so a velocity step is commanded indirectly — see Decisions), fits
`PlantModel(K, wn, zeta)` against the resulting transient via `scipy.optimize.curve_fit`. Built,
then run for real multiple times across the session, surfacing and fixing:
1. A crash (`scipy.signal.step` needs uniform timestamps; real ones aren't).
2. A sign/magnitude bug (the commanded velocity is a positive *limit*; measured velocity is
   signed by direction).
3. A wrong fit target (the reply's own `motor_velocity_rad_s` reads 6.7-8.5x too high and
   ~uncorrelated with real motion — switched to differentiated position).
4. **A real timing bug**: the retract phase shared the step phase's fixed duration regardless of
   how far it actually had to travel back, so whenever the step velocity exceeded the retract
   velocity's capacity, the *next* rep started mid-retract, reversing real residual motion
   instead of starting from rest. Fixed by sizing the retract phase's time budget from measured
   travel and exiting on measured arrival, not a blind timer; `fit_needle_plant.py` now reads
   the resulting `retract_arrived` flag and automatically excludes any step whose predecessor
   didn't arrive from its aggregate stats.

**`scripts/characterize_needle_plant.py`** (new): orchestrates the whole protocol — pre-flight,
several independent single-step "cold-start" trials, then multi-rep batches at a couple of
velocities — into one command, producing one `report.json`/`report.md`. Still requires operator
confirmation before every real motion (each stage just invokes the scripts above), per this
project's established preference for a human watching every physical move.

**`scripts/read_needle_position.py`** (fixed, not new): was still using MIT-mode CAN addressing
(`can_id = node_id`) left over from session 004's abandoned MIT-mode detour, so every probe sent
to a CAN ID nothing listens on and always reported "no reply received." Found because
`characterize_needle_plant.py`'s pre-flight stage called it and failed immediately. Now mirrors
the Gimbal position/velocity addressing every other current needle script uses.

**`scripts/design_needle_lead.py`** (new): numeric sweep over `(zero, alpha=pole/zero, gain)`
against the measured plant, using the already-tested frequency-response functions in
`src/ct/control/servo.py`, picking the combination with the lowest `residual_lag()` subject to a
phase-margin floor and a crossover ceiling (see Decisions for why not a single-target bisection).

**`scripts/simulate_needle_lead_tracking.py`** (new): discrete-time simulation of a sine
reference driving the measured plant two ways — direct/uncompensated and through the real
`LeadServo` class in closed loop — rendering all three (reference, uncompensated, compensated)
on one plot, plus a self-check against the closed-form frequency-response functions. That
self-check is what surfaced Finding 6.

**`configs/rig_bench.yaml`**: `servo.needle.plant` and `servo.needle.lead` replaced with the
measured/designed values (see Findings 4-5), with provenance comments. `ct-unknowns --check
rig_bench` now reports both as measured rather than placeholder.

## Files touched

| File | Change |
|---|---|
| `scripts/run_needle_step_response.py` | new, then fixed 4 times (see above) across the session |
| `scripts/plot_needle_step_response.py` | new — commanded-vs-measured plot, later updated to plot `\|d(position)/dt\|` instead of the untrustworthy raw reply velocity |
| `scripts/fit_needle_plant.py` | new, then fixed (uniform-grid crash, sign bug, fit-target swap, `started_from_rest` exclusion) |
| `scripts/characterize_needle_plant.py` | new — orchestrates cold-start + warm-batch data collection and reporting |
| `scripts/read_needle_position.py` | fixed — stale MIT-mode addressing replaced with the real Gimbal protocol |
| `scripts/design_needle_lead.py` | new — numeric lead-compensator sweep against a measured plant |
| `scripts/simulate_needle_lead_tracking.py` | new — sine-tracking simulation, with/without compensation |
| `configs/rig_bench.yaml` | `servo.needle.plant`/`servo.needle.lead` set from measurement/design, replacing session-002 placeholders |

## Decisions and rationale

**Fit the velocity channel (via differentiated position), not the reply's own velocity field.**
`PlantModel` is type-1 — the motor produces velocity and position is its free integral — so the
classic underdamped step-response landmarks live on velocity, not position. The reply's own
`motor_velocity_rad_s` turned out to be unusable (Finding 1), so differentiated position is used
instead, corroborated by every past session that has trusted position for real measurements.

**Numerical curve fit over closed-form overshoot/settling-time formulas**, since those need a
resolved overshoot peak and only work when underdamped, and the real axis's damping was unknown
going in (turned out to be `zeta≈0.35`, plausible either way in advance).

**No `--config`/`--set` machinery** in the bench scripts — matches their true siblings
(`run_needle_sine_tracking.py`, `test_needle_motor.py`), which are deliberately independent of
the simulation config layer.

**The retract phase judges completion from measured position, not a timer** (the fix for the
timing bug above) — the same "arrival is judged from the motor's own replies, never dead
reckoned" principle already established elsewhere in this project (session 009's travel-limit
fix, session 005's hold-position fix).

**`fit_needle_plant.py` defaults to *including* steps with unknown (pre-fix) provenance, and
only excludes an explicitly-confirmed-tainted step.** Treating "unknown" as guilty by default
would have discarded already-manually-validated data from before the `retract_arrived` field
existed; only a definite `False` means a step's predecessor demonstrably didn't arrive.

**Only "genuinely first command since connecting" step data was used for the final plant
estimate**, discarding every later rep in a back-to-back batch. Even after the retract-timing
bug was fixed (confirmed via `retract_arrived: True` on every rep), later reps *still*
consistently produced degenerate fits (first pegged at bounds one way, then pegged at `zeta=5.0`
a different way) while every independent "first-ever-command" trial gave sane, mutually
consistent numbers. The mechanism isn't understood — per the user's explicit steer this session
("we only need something that helps a little bit, it doesn't need to be perfect"), this was not
chased further; the data that *is* clean and repeatable was used instead of holding out for a
fully explained one.

**`design_needle_lead.py` searches a grid and picks the fastest candidate meeting a margin floor
and crossover ceiling, rather than bisecting for one exact target margin.** For this plant
(`zeta≈0.35`, a real resonance peak), phase margin is not monotonic in gain — a first attempt at
exact-target bisection either failed to bracket a solution or landed gain choices right at a
cliff where crossover jumps discontinuously past the resonance peak and margin collapses.
"Fastest within a safe zone" is both the actually-relevant design objective (it's what minimizes
`residual_lag()`, which is what the forecast horizon consumes) and immune to that cliff, since
candidates near it simply fail the crossover ceiling and are excluded.

**The sine-tracking simulation runs the real `LeadServo` class**, not a re-derivation of the
compensator math, specifically so a mismatch between the simulation and the analytical
`closed_loop_response()` would mean something (Finding 6), rather than just reflect two
independent implementations of the same intended formula.

## Verification

```bash
conda activate CT && pytest -q
# 322 passed, throughout the session (checked after every code change)
```

Real bench runs, in order (each preceded by `--dry-run`, `--help` smoke tests as applicable):

| Run | Settings | Headline result |
|---|---|---|
| `20260905-151246` | 0.15rad/s, 1 rep, 20Hz cmd | First real data; found the wrong-fit-target problem (Finding 1) |
| `20260905-173437` | 0.15rad/s, 1 rep, 50Hz cmd | Higher resend rate; velocity trace still noisy, inconclusive alone |
| `20260905-174230` | 0.15rad/s, 5 reps, 50Hz cmd | First repeatable signal: reps 1-4 clustered (`wn≈10.7±0.8`, `zeta≈0.76±0.03`), rep 0 an outlier |
| `20260905-180841` (`characterize_needle_plant.py`) | 3 cold-start + 0.15/0.30rad/s×5 reps | Found the retract-timing bug: `warm_v0.3` fits were fully degenerate (`wn≈300`, `zeta≈0.026`) because every retract after the first left the needle ~2.1mm short, still moving |
| `20260905-182042` (`characterize_needle_plant.py`, post-fix) | same | Retract bug confirmed fixed (`retract_incomplete: 0` everywhere); new pattern found instead — reps 1+ within any batch peg `zeta` at its 5.0 bound regardless |

Final plant/lead numbers (see Findings 4-5) verified via `ct-unknowns --check rig_bench`:
both `servo.needle.plant` and `servo.needle.lead` report `measured`, no longer `placeholder`.

`design_needle_lead.py`'s chosen design and `simulate_needle_lead_tracking.py`'s open-loop
self-check against it matched the closed-form prediction almost exactly (`|G|=0.587` both
ways, lag 1028.5ms vs. 1026.0ms predicted) — validating the discretization and fitting code.
The closed-loop self-check did *not* match (18% amplitude discrepancy) — see Finding 6, which
is a real architectural finding, not a bug in either function.

## Findings

**1. The reply's `motor_velocity_rad_s` field cannot be trusted at its documented scale.**
Reads 6.7-8.5x the commanded velocity limit across every run this session, consistently enough
(not randomly) to suggest a systematic units issue — `scripts/listen_needle_motor.py` had
already flagged the GL-II manual's internal "rad/s" vs "r/s" inconsistency as the likely cause.
Not fixed at the source (`MOTOR_REPLY_V_MIN/V_MAX`, shared by four scripts) since one session's
evidence isn't enough to safely change a value four other scripts depend on; worked around via
differentiated position instead.

**2. A real timing bug made every retract-incomplete rep's "step" fit meaningless, and it only
showed up at the higher of two tested velocities.** At 0.15rad/s the default retract velocity
(0.2rad/s) had enough capacity to finish in the same fixed window; at 0.30rad/s it didn't (needed
5.4mm, had budget for 3.6mm), silently leaving every subsequent rep starting mid-reversal. Fixed
by sizing retract duration from measured travel and confirming arrival from real telemetry.

**3. Even with that fixed, only the very first commanded step of any given invocation produces a
non-degenerate fit; every rep after it within the same back-to-back sequence does not — and the
*way* it's degenerate changed between runs** (bounds-pegged in one direction on one run, a
different bound on the next), which suggests noise/an unmodeled effect rather than a single
findable bug. Not resolved this session — see Open questions.

**4. Adopted plant estimate: `K=0.92, wn=27.0, zeta=0.35`.** From the mean of four independent
"genuinely first command since connecting" trials at 0.15rad/s (three cold-start trials plus one
warm-batch's own rep 0): `wn` and `K` agree to within ~3% across all four; `zeta` is noisier
(~33% spread) but consistently well below the `zeta=0.7` placeholder. `wn=27` is under half the
placeholder's `wn=60`. One less-certain point at 0.30rad/s (rep 0 only) suggested `wn` may
roughly double with amplitude — a single data point, not incorporated. This is explicitly a
provisional, "helps a little, not perfect" estimate per the user's own framing, not a finished
characterization.

**5. Adopted lead compensator: `zero=1.0, pole=5.0, gain=12.74`.** Chosen by
`design_needle_lead.py` as the fastest (lowest `residual_lag()`) option keeping the crossover at
or below half the measured plant's `wn` and phase margin above 40°: achieves 82.4° margin at a
13.16rad/s crossover (49% of `wn`) — safely conservative rather than pushed to the edge, since
the plant's `zeta=0.35` resonance makes the margin-vs-gain relationship non-monotonic near that
edge (see Decisions). Predicted `residual_lag` at a nominal 0.25Hz breathing rate: 151ms — worse
than the old placeholder implied, but honest (the real axis is slower than the placeholder
assumed).

**6. The forecast horizon's `tau_cl(omega_r)` may be computed against the wrong closed-loop
architecture.** `ct.control.states.insert._hold_standoff()` commands `reference_mm +
correction` (feedforward-plus-feedback), not `correction` alone — its own docstring is explicit
about this ("Feedforward from the forecast sets the reference; the lead compensator supplies the
correction for whatever the axis has not managed to follow"). But `ct.control.servo`'s
`closed_loop_response()`/`residual_lag()` compute the standard unity-feedback transfer function
`T=L/(1+L)`. Algebraically these are different: the real architecture's transfer function is
`G(1+C)/(1+GC)`, not `GC/(1+GC)`. `simulate_needle_lead_tracking.py`'s self-check — running the
real `LeadServo` in an actual feedforward+feedback loop against the plant and comparing to
`closed_loop_response()`'s prediction at the same frequency — measured an 18% amplitude
discrepancy and a lag of 228.7ms actual vs. 151.1ms predicted, consistent with this derivation
(estimated ratio `1+1/|C(jw)| ≈ 1.22` at this frequency, close to the observed 1.18). **Not
investigated or fixed further this session** — it's a bigger question than "tune a compensator a
little," and touches a formula used throughout the whole project's forecast-horizon history.
Flagged prominently as the most important open item, below.

## Open questions

- **Does `tau_cl(omega_r)`/`residual_lag()` need to account for the real feedforward+feedback
  architecture, and if so, by how much has every past session's `tau_cl` number been off?**
  (Finding 6 — new, and likely the most consequential open question in the project right now.)
  Needs a dedicated session: re-derive the correct transfer function for `u = r + C(r-y)`,
  decide whether `ct.control.servo` should expose it as the default, and check whether it changes
  any past forecast-horizon conclusion materially.
- Why does only the very first commanded step of a sequence produce a non-degenerate fit, with
  every later rep degenerate in a not-obviously-consistent way? (Finding 3.)
- Whether `MOTOR_REPLY_V_MIN/V_MAX = -200.0, 200.0` should actually be something else (Finding
  1) — still just suggestive, not conclusive, from this session's data alone.
- Whether the plant is genuinely amplitude-dependent (the single 0.30rad/s data point hinted
  `wn` roughly doubles) — would need a clean multi-rep measurement at a second velocity, which
  this session never obtained (every 0.30rad/s multi-rep batch was degenerate for the reasons
  above).
- Whether `zero=1.0, pole=5.0, gain=12.74` actually improves real tracking on hardware — verified
  only in simulation this session; `run_needle_sine_tracking.py` against the real needle would be
  the direct check.

## Next steps

1. Confirm the new lead compensator on real hardware: `run_needle_sine_tracking.py` at a
   breathing-relevant frequency, comparing measured lag/amplitude ratio to
   `simulate_needle_lead_tracking.py`'s predictions.
2. Open a dedicated investigation into Finding 6 (the `tau_cl` architecture question) — this is
   flagged as a new, separate, higher-priority item, not a continuation of this session's scope.
3. If motivated: chase Finding 3 (why only the first rep of a sequence fits cleanly) before
   trusting the plant estimate much further, or gather more reps at more velocities to at least
   characterize how it varies rather than explain it.
4. Promote Finding 6, and the retract-timing bug (Finding 2), into `CLAUDE.md`'s Known findings
   — both are durable, general lessons (respectively: check your control architecture actually
   matches your analysis formula; a phase's timing budget must be sized from what it needs to
   do, not copied from a different phase).
