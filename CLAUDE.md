# CT — respiratory-motion estimation for needle-insertion gating

## Project

Controller for a needle-insertion robot that must fire during specific points of the
breathing cycle. A 1-D sensor tracks the breathing signal; this repo identifies and then
tracks a parametric harmonic model of that signal so its value can be **forecast** at
`t + h`.

**Scope — estimator plus rig controller.** This repo was the estimator only until session
002. It now also contains the hardware layer and the insertion controller that consume
`(s_hat, P)`: CAN I/O, the base and needle axes, the four-state insertion procedure, the
needle servo, the firing gate, and the safety monitor. That was a deliberate amendment,
not drift — see [002](docs/sessions/002-2026-08-14-rig-control.md).

The estimator half is still separable and must stay that way. `ct.identification`,
`ct.tracking`, `ct.forecast` and `ct.sources` do not import anything from `ct.hw`,
`ct.rt`, `ct.control`, or `ct.plant`; the dependency runs one way only, and
`test_boundaries.py` enforces it.

## Settled decisions — do not reintroduce these errors

These were worked out at length before any code existed. They are settled.

1. **Feedforward and feedback are separate loops.** Breathing prediction (this repo) is
   open-loop disturbance forecasting. The needle servo is a separate closed loop. Keep
   them conceptually and architecturally apart.
2. **`tau_a` is not a horizon term.** The raw actuator delay is replaced by
   `tau_cl(omega_r)`, the *residual closed-loop tracking lag* of the lead-compensated
   servo. It is a servo-design output that varies with breathing rate, not a fixed
   physical constant. Since session 002 it is computed here, in
   [control/servo.py](src/ct/control/servo.py), as `-angle(T(j*omega_r)) / omega_r` from
   the closed loop — a *derived* quantity that changes with breathing rate, never a
   configured constant. The estimator still never sees it decomposed.
3. **The horizon is `h = tau_s + tau_c + tau_cl(omega_r) + T_ins`.** All four terms are
   consumed by one forecasting operation; none is solved individually. This pipeline's
   only job is to evaluate the model at whatever `h` it is handed. See
   [forecast.py](src/ct/forecast.py).
4. **Prediction is a time-advance, not a scalar phase shift.** Advancing by `h` means
   `theta -> theta + omega_r*h`; harmonic `k` then rotates by `k*omega_r*h`
   automatically. **Never** compute one delay-derived angle and add it to every
   harmonic's phase — that advances only the fundamental and under-rotates harmonic `k`
   by a factor of `k`, distorting waveform shape instead of shifting it in time.
   [tests/test_forecast.py](tests/test_forecast.py) is the guard; `naive_common_phase_forecast`
   in [forecast.py](src/ct/forecast.py) exists solely so that test can prove the bug is wrong.

## Architecture

Two stages behind three swappable boundaries, defined as `typing.Protocol` in
[interfaces.py](src/ct/interfaces.py) and resolved by name through
[registry.py](src/ct/registry.py):

| Boundary | Protocol | Built-in | Lives in |
|---|---|---|---|
| signal source | `SignalSource` | `sinusoid`, `lujan`, `rc_piecewise`, `csv` | [sources/](src/ct/sources/) |
| identification (Stage 1) | `Identifier` | `fft_harmonic` | [identification/](src/ct/identification/) |
| tracking (Stage 2) | `Tracker` | `harmonic_ekf` | [tracking/](src/ct/tracking/) |

Each stage speaks only in the dataclasses from [types.py](src/ct/types.py) — `SignalBatch`,
`IdentificationResult`, `TrackerStep`. A replacement (UKF, particle filter, IMM; an
alternative Stage-1 method; real sensor data) plugs in via `@register_*` and a config
string, with no change to the other two stages or the CLI. Both swaps are proven by tests
in [test_pipeline.py](tests/test_pipeline.py).

[run.py](src/ct/run.py) orchestrates config → source → identify → track → forecast →
metrics. The `ct-*` scripts are thin wrappers over it, so every entry point runs the same
code path.

### The rig controller

Added in session 002. Same idea one layer down: protocols in
[hw/interfaces.py](src/ct/hw/interfaces.py), resolved by name through the same registry.

| Boundary | Protocol | Built-in | Lives in |
|---|---|---|---|
| CAN transport | `Bus` | `loopback`, `rh02` | [hw/bus/](src/ct/hw/bus/) |
| motor wire protocol | `MotorCodec` | `cubemars_mit`, `cubemars_servo` | [hw/motors/](src/ct/hw/motors/) |
| moving joint | `Axis` | `can_axis`, `sim_axis` | [hw/motors/](src/ct/hw/motors/) |
| sensor | `Sensor` | `tof`, `tactile` | [hw/sensors/](src/ct/hw/sensors/) |
| clock | `Clock` | `real`, `sim` | [rt/clock.py](src/ct/rt/clock.py) |
| procedure state | `ProcedureStateImpl` | `approach`, `estimate`, `insert`, `advance`, … | [control/states/](src/ct/control/states/) |

[rt/loop.py](src/ct/rt/loop.py) ticks at a fixed rate: read mailbox → step tracker →
active state's `update` → write axis commands → emit telemetry. Swapping `Clock` from
`real` to `sim` is what makes the whole four-state procedure a fast deterministic test
with no hardware attached; swapping `Bus` from `rh02` to `loopback` is what makes it run
against the simulated plant. **Neither swap changes a line of control logic** — that is
the whole point, and `test_procedure.py` runs the same states both ways.

### The hard rules

**1. [layout.py](src/ct/layout.py) is the only place the state ordering is encoded.**

```
s = [a0, A_1, phi_1, ..., A_K, phi_K, theta, omega_r]      n = 2K + 3
```

Nothing else may hard-code an index into `s`. Use `StateLayout(K).A(k)`, `.phi(k)`,
`.theta`, `.omega`, `.amplitude_idx`, `.phase_idx`. This is what keeps the boundaries
genuinely swappable rather than swappable-looking.

**2. [geometry.py](src/ct/geometry.py) is the only place raw units become millimetres.**
No module outside it may multiply by `counts_per_mm`, apply a frame offset, or know a gear
ratio. Above the `Axis` boundary everything speaks mm, mm/s, and a named frame. This is
rule 1 applied to the hardware layer, and it is what makes recalibration a one-file
change. `test_boundaries.py` greps for violations.

**3. Nothing in `control/`, `hw/`, or `rt/` may import `plant/`.** `plant/` is the
simulated rig — the hardware-layer equivalent of `y_clean` and `truth`. A controller that
can see simulation ground truth proves nothing when you run it in sim. Enforced by an
import-graph test.

**4. The control tick never blocks on I/O.** CAN receive runs on its own thread into a
latest-frame-per-ID mailbox ([hw/bus/mailbox.py](src/ct/hw/bus/mailbox.py)); the tick
reads the mailbox and moves on. A stalled bus degrades to stale readings and a watchdog
fault, never to a cascade of missed deadlines.

### What we do not know yet

The rig's physical constants — counts per mm, where the needle tip sits relative to the
tactile face, the servo's characteristics, the real latency — are mostly unmeasured.
[unknowns.py](src/ct/unknowns.py) is the machine-readable list of them: each entry carries
the dotted config key it plugs into, its units, how to measure it, who owns it, a
placeholder that lets sim run today, and which procedure states it blocks. `ct-unknowns`
prints it for the team; `ct-unknowns --check <config>` says what is still a placeholder;
and a real-hardware run refuses to start on a placeholder that blocks a requested state.
**Add an entry there rather than a bare TODO** — the list is what keeps the gap visible.

### The model

```
process      theta_k = theta_{k-1} + omega_r*Ts     (exact, by definition)
             s_k(i)  = s_{k-1}(i)                   (everything else persists; Q carries the drift)

measurement  h(s) = a0 + sum_k A_k sin(k*theta + phi_k)
```

Jacobians are derived analytically and hard-coded in
[tracking/measurement.py](src/ct/tracking/measurement.py). **Never finite-difference at
runtime** — this runs at sensor rate, and numeric differencing would cost `K+3` extra
evaluations per step for a worse derivative. Finite differences appear only in
[tests/test_jacobians.py](tests/test_jacobians.py), as a check on the analytic forms.

## Conventions

- **Units are SI**: seconds, radians, rad/s. `omega_r` is *angular* frequency, not Hz;
  helpers `bpm_to_omega` / `omega_to_bpm` live in
  [identification/spectral.py](src/ct/identification/spectral.py).
- **Phases wrap to `(-pi, pi]`** after every update, via `wrap_angle`.
- **The measurement is scalar.** `R` and `S` are plain floats, so the Kalman gain step is
  a division, not a matrix inverse. Keep it that way.
- **Covariance uses the Joseph form** by default, for symmetry over long runs.
- **Math notation beats snake_case.** `P`, `Q`, `R`, `S`, `K`, `F`, `H`, `T_breath`,
  `P_diag` match the reference math and the literature. Linters will object; ignore them
  here — a reader checking the code against the paper matters more.
- `y_clean` and `truth` on a `SignalBatch` are for validation and plots only. **No
  estimator code may depend on them** — real sensor data has neither.
- **The word "phase" is taken.** It means `phi_k` or `theta`, and "Stage" means
  identification/tracking. The insertion state machine's states are therefore
  `ProcedureState`, never bare "phase" — in code, in comments, and in prose.
- **Rig distances are millimetres, not metres.** SI everywhere else, but the rig talks in
  mm and mm/s because that is what the mechanical drawings and the motor datasheets use,
  and silently-scaled-by-1000 bugs are the expensive kind. Time is still seconds and
  `omega_r` is still rad/s. Position frames are named and documented in `geometry.py`;
  `+x` always points *toward the phantom*.

## Environment and commands

```bash
conda env create -f environment.yml     # creates the CT env
conda activate CT
pip install -e .

pytest -q                               # 152 tests, ~13 s
```

### Scripts

Every script takes `--config <name>` (bare name resolves against `configs/`) plus
dedicated flags, and `--set dotted.key=value` for anything else. Artifacts and figures
land in `outputs/<name>/`, alongside the resolved config that produced them.

| Command | Purpose |
|---|---|
| `ct-generate` | write a synthetic trace to CSV (+ `--plot`) |
| `ct-identify` | Stage 1 only; prints K, the harmonic table, `R`, `Q`; saves `ident.npz` |
| `ct-track` | Stage 2 over a trace, optionally from a saved `--ident` |
| `ct-pipeline` | the headline script: generate/load → identify → track → forecast → all figures |
| `ct-sweep-horizon` | forecast error vs `h` |
| `ct-validate-k` | K-selection check across all reference models |
| `ct-unknowns` | the list of physical constants still unmeasured; `--check` a config, `--format md` for the team |
| `ct-rig` | run the four-state insertion procedure, simulated or on hardware |
| `ct-phantom` | drive the breathing phantom from a waveform; a separate process on its own bus |
| `ct-compare` | align a phantom log against a controller log; reports sensing RMSE, bias, and measured `tau_s` |

See [README.md](README.md) for worked examples of each.

Bench scripts for the live needle-insertion work (session 022) are plain
`python scripts/...` rather than `ct-*` entry points:

| Script | Purpose |
|---|---|
| `run_approach_and_seat.py --insert-needle` | the real thing: approach, seat, standoff, calibrate an EKF, then gated breakthrough + advance |
| `generate_sinusoid_profile.py` | synthetic sinusoid in the phantom driver's `time_s,y_mm` format (`ct-generate` writes `t,y`, which the driver cannot read) |
| `slice_breathing_profile.py` | trailing-window slice of a real profile — `run_breathing_profile.py` loops the *whole* file, so "the last 300s" needs a real slice |
| `plot_approach_and_seat.py` | run figure; marks gate-fire vs. actual drive start and shades where the needle is not floating |
| `plot_needle_gating_timing.py` | offline gate-fire vs. activation diagnostic across sinusoid/moira/emma |

### Real subject data

`unfiltered_data/` holds the raw OptiTrack takes (read-only; large). Each is a 70-column
export at 120 Hz in which only the **nine `Unlabeled NNNN` markers** carry signal — the
`tube` and `crab` rigid bodies are fixtures with all-zero position columns.
[scripts/build_breathe_profiles.py](scripts/build_breathe_profiles.py) reduces a take to one
`time_s,y_mm` trace by averaging those nine markers' Y position, and writes the result to
`breathe_profiles/`. It selects marker columns from each file's own header, never by index,
and its default `--verify` re-proves the reduction against
`unfiltered_data/Moira_normal average marker motion.csv` on every run. Consume the output
through the `csv` source with `t_column=time_s`, `y_column=y_mm`.

## Session workflow

**At the end of every working session**, write a session document:

1. Copy [docs/sessions/000-template.md](docs/sessions/000-template.md) to
   `docs/sessions/NNN-YYYY-MM-DD-slug.md` (next number, today's date).
2. Fill in: Goal · What changed · Files touched · Decisions & rationale · Verification run
   (exact commands and their **real** output numbers) · Open questions · Next steps.
3. Add a one-line entry to the index in [docs/sessions/README.md](docs/sessions/README.md).
4. Update the "Open questions" section below if any were answered or added.

Record what was actually measured, not what was expected. A session doc whose numbers were
not really produced is worse than no session doc.

## Known findings

- **The 95% energy rule is waveform-dependent.** It reproduces the documented Lujan
  answers (`n=1 -> K=1`, `n=2 -> K=2`, `n=3 -> K=2`), but on the RC-piecewise chest-wall
  model it selects `K=1` and leaves visible structure in the residual. Tracking RMSE:
  0.439 at 95%, 0.289 at 99%, 0.108 at 99.9%, against an injected noise std of 0.15.
  Unlike Lujan, whose Fourier series terminates exactly at `k=n`, the RC model's
  coefficients never reach zero — the kink. Re-validate the threshold on real data rather
  than assuming it. See [configs/rc_piecewise.yaml](configs/rc_piecewise.yaml) and
  [configs/rc_piecewise_k4.yaml](configs/rc_piecewise_k4.yaml).
- **The residual-variance fallback for `R` is genuinely only an upper bound.** With a
  drifting fundamental, Stage 1 fits the whole window at one fixed `omega`, which inflates
  the residual and makes the filter over-trust its model: NIS falls to ~0.027 against an
  expected 1.0. See [configs/sinusoid_ramp.yaml](configs/sinusoid_ramp.yaml). Prefer a
  breath-hold segment (`identifier.params.breath_hold_window`) once real data has one.
- **Two typos in the reference doc's RC-piecewise formulas** were found and corrected by
  re-deriving them; both are documented in
  [sources/rc_piecewise.py](src/ct/sources/rc_piecewise.py) and pinned by
  `test_rc_inhale_coefficients_solve_the_ode`.

### From the rig (session 002)

- **MIT mode's position range is only about two turns.** ±12.5 rad by default, and a
  command past it *saturates silently* — the axis stops short and feedback agrees with the
  clipped value. That is why the base runs servo mode (int32 degrees) and only the needle
  runs MIT. `CANAxis` now refuses to construct when `travel_mm × counts_per_mm` does not
  fit inside the codec's range.
- **Insertion under impedance control falls short by `load / kp`.** At `kp = 80` the needle
  stalled 4 mm short of a 10 mm command with the motor at full effort. `kp` must be high
  enough that the residual fits inside the placement tolerance, and arrival tests need a
  stall detector — a tolerance the physics forbids just becomes a timeout.
- **The tactile sensor's usable stroke must exceed the full breathing excursion**, or there
  is no seating depth at which the whole waveform is visible and APPROACH cannot succeed at
  any tuning. A hardware sizing requirement, not a tuning problem. Session 009 adds a second
  term: excursion **plus** the ~1.5mm of slow baseline wander a real subject profile carries.
- **A position axis is type-1.** Modelling the needle as a plain second-order lag gave a
  closed loop tracking 55% of its reference and *leading* rather than lagging, which
  silently clamped `tau_cl` to zero and dropped a term out of `h`.

### From the bench (session 005)

- **The Gimbal-protocol addressing needle/base already run is real, published CubeMars
  documentation** — confirmed by fetching CubeMars's actual "Gimbal Motor Drive User
  Manual — For GL II" (V1.0), closing an open question session 004 had left unresolved.
  `(mode<<8)|node_id` + little-endian float32 position/velocity matches exactly.
- **This motor's CAN feedback frame is a per-command ACK, not a continuous broadcast.**
  One 19.6 s real run produced exactly 4 replies, landing within milliseconds of the 4
  commands actually sent — not spread through the run. Sending commands more often (a
  periodic resend, not one-shot) is what turns this into usable telemetry.
- **A dead-reckoned position estimate (`velocity × elapsed_time` from move-start) drifts
  from the real position by an amount worth caring about.** One real reply near a contact
  event measured 0.17 rad (4.44 mm) less travel than dead reckoning assumed at t≈14.6 s —
  consistent with the real acceleration ramp-up at the start of a move (documented to
  exist, but with no quantified numbers) that constant-velocity dead reckoning ignores.
  Prefer a real, recent reply plus a small residual correction over dead-reckoning the
  whole elapsed time whenever real telemetry is available.
- **Switching a GL-II drive's control mode (MIT / Position-Velocity / Velocity) is a
  persistent, GUI-configured, power-cycle-requiring setting**, not a per-CAN-frame choice —
  the mode bits in the arbitration ID only take effect once the drive itself has been
  reconfigured via CubeMars's own software.

### From phantom relative playback (sessions 010-011)

- **The phantom breathes wherever it is parked, and the origin is COMMANDED, not read.**
  `run_breathing_profile.py` sends `codec.zero()` (`CMD_SET_ORIGIN`, temporary) at startup so
  the motor's present position *becomes* 0, then commands the profile's offsets literally.
  Session 010 tried referencing a *read* position (`initial_rad + profile_offset`) and it still
  traversed on hardware; commanding the origin leaves no reference number in the arithmetic to
  be wrong. `scripts/test_phantom_motor.py --zero` has used this on this motor since session 004.
- **The readback after zeroing is the safety check, and it is the right one.** The motor must
  report ~0 within `ZERO_READBACK_TOL_MM`, because that is exactly the property playback
  depends on: commanding position 0 must mean "stay put". If `SET_ORIGIN` silently fails the
  motor still reports its old position and the run refuses rather than dragging the phantom
  across the bench.
- **`load_profile` rebases about the profile's mean, not its first sample.** Every recorded
  profile starts near its top, so first-sample referencing would park the phantom 0.5-1.1mm
  behind where you left it — the same magnitude that consumed the base's contact margin in
  session 009.
- **Do not reference two logged series to the same unverified number.** Session 010 subtracted
  the read reference from both `commanded_mm` and `measured_mm`, so a wrong reference shifted
  them together and made a real traverse invisible in the log. The measured series must stay in
  the frame the hardware actually reports.
- **Diagnostics belong in the `finally`.** The phantom's `summary.json` used to be built after
  the `try/finally`, so any abnormal exit skipped it — and three runs on 2026-09-02 have none,
  each missing the one number that would have diagnosed the traverse. The phantom subprocess's
  stdout is likewise captured to `<out>/phantom/stdout.log` rather than lost to scrollback.
- **A subprocess dying mid-run used to be invisible.** `run_approach_and_seat.py` now checks
  `phantom_proc.poll()` every tick. Without it the base keeps seating and standing off against
  a stationary surface, and the run completes looking fine while measuring nothing.
### From the tactile tare (session 012)

- **The tactile firmware zero drifts ~0.008mm/hour and the signal itself is clean.** 0.0048mm
  at rest on 2026-09-01 16:54, 0.1967mm on 2026-09-02 15:46 — a 40x creep — while its std
  stayed at 0.002-0.014mm. Once it passed the fixed 0.1mm contact threshold, **every** run
  declared contact on its first sample and had no APPROACH phase at all.
  `run_approach_and_seat.py` now **tares per run**: 2s of stationary samples before any motion,
  subtracted from the *signed* `dist_cm` before the magnitude is taken (the sign is arbitrary,
  so subtracting after would fold a negative rest onto a positive one). Within-run drift is
  ~0.0004mm, so a per-run zero is enough. **Any absolute threshold on this signal has a shelf
  life** — this one lasted two days.
- **That failure looked like success.** A run declaring contact at t=0.0002s reports
  `phase_reached: seat` with no fault; only the missing `approach` phase gives it away.
  Everything derived from such a run was measured from a datum never established.
- **`bus.recv()` returns the OLDEST queued frame, so "command it, then read back to confirm"
  is a trap on this bus.** After a 0.5s settle at ~51Hz there are ~25 pre-command frames ahead
  of the one that matters. Session 011's SET_ORIGIN check refused on exactly such a stale
  frame. Flush before any confirmation read — `read_fresh_position_rad` does this.
- **Capturing a subprocess's stdout paid for itself on the first run after it was added.**
  Both session-011 phantom bugs were found in `phantom/stdout.log` and neither is visible in
  the JSONL or the summary.

### From the approach travel-exhaustion fault (session 009)

- **The rig ran on a 6% contact margin for its whole history and looked fine.** The one bench
  run that reached `standoff_hold` contacted at exactly its 40mm travel limit, with a peak
  tactile reading of 0.00941 against a 0.01 threshold. Every conclusion drawn from bench runs
  before session 009 rested on that margin holding. `--travel-mm` is now an *initial* target
  that auto-extends to a `--max-approach-mm` cap, so running out of room is a named fault
  rather than a timeout.
- **Real subject profiles carry ~1.5mm of slow baseline wander; short looping profiles do
  not.** `breathing_profile_1` is 14.8s and loops ~11 times per run, presenting a stationary
  mean. Emma is 618s, completes *zero* loops in a 180s run, and the slice that plays sat ~1mm
  further out (mean -0.140mm vs -1.098mm). That 1mm consumed the entire contact margin.
  **Anything tuned against a short looping profile — seating depth, travel budget, standoff —
  should be re-checked against a real one.**
- **A grazing contact is distinguishable from a seated one, and 3.2 sigma is not enough.** At
  the travel limit the deflection was 0.0246mm mean against a 0.0031 +- 0.0067mm baseline, and
  the tactile arm saw 0.0417mm p2p of breathing while the ToF saw 12mm of the same motion. The
  contact threshold was deliberately *not* lowered: the pre-contact maximum (0.0255mm) already
  equalled the post-contact settled mean, so a threshold low enough to catch it would fire
  first. Under-travel, not under-sensitivity.
- **Never dead-reckon a travel limit.** Arrival is judged from the motor's own replies
  (settled position error 0.006mm against ~0.87mm while moving). Extending a safety bound on a
  dead-reckoned guess is a worse version of session 005's overshoot bug.

### From the standoff convergence failure (session 014)

- **`max` over a window is a biased estimator of a noisy periodic peak, and the bias grows
  with the window.** Standoff measured the breathing peak that way and retreats whenever the
  reading exceeds target, so the bias moved the base backwards from positions that were
  actually short. Replayed over both 2026-09-03 runs' stationary measurement segments it read
  **+0.48 and +0.54 mm high, worst case +1.77 mm**, against a 0.30 mm tolerance. The perverse
  part: waiting longer to "measure more carefully" increases the bias. Use the **mean over a
  counted number of whole cycles** (`BreathPeakWatcher` in `control/live.py`) — unbiased, with
  a standard error that falls the way an average should.
- **A tolerance below the signal's own variability cannot be met reliably at any seating.**
  Accepting became luck: one run settled in 3 fine steps, the next took 13 with 6 retreats and
  had to be cancelled. `summary.json` now records `breath_spread_mm` so the floor is visible
  next to the tolerance — if runs still take many steps, widen the band rather than lengthen
  the measurement.
- **An acceptance band around a target should be two-sided.** One-sided
  `[target−tol, target]` turned every over-read into a base move.
- **A "nominal" constant that nothing checks drifts away from the truth.**
  `nominal_breath_s = 4.0` against a real 5.51 s made `min_breaths × nominal_breath_s` 1.45
  breaths, not 2. Count the real thing and report the measured period beside the nominal.
- **The tare's premise has to be enforced, not assumed.** It assumes the sensor starts out of
  contact — true in a real procedure, but not of a run started right after one that left the
  base pressed in, since nothing moves the base at exit. That run tared a *moving* arm
  (p2p 8.330 mm vs 0.014 mm free), giving a 0.352 mm zero against a 0.140 mm true rest:
  contact at t = 0.006 s, **no APPROACH phase at all**, seat accepted at 0.219 mm. A loaded
  tare now refuses instead of warning; the base is backed off by hand between runs, not by
  the script.
- **A closed loop needs a give-up condition or its failures are undiagnosable.** A cancelled
  run reports nothing about why it was cancelled. Same lesson as session 009's approach-travel
  fault, in a different loop.
- **Do not add autonomous physical motion as an unrequested fix, even a bounded one.** An
  automatic base-retraction step was drafted for the tare problem above, passed every offline
  check, and was removed on sight at the user's direction — it moved the rig on its own after
  every run, which nobody asked for and which had never touched hardware. A plan approving a
  described behaviour is not authorization to autonomously widen that behaviour's scope on a
  physical system with a needle axis nearby. Default to refusing and naming the problem; leave
  the physical correction to the operator unless explicitly asked to automate it.

### From the estimator on real bench data (session 008)

- **Forecasting by the measured sensor lag removes 62% of the lag error.** Scored against
  where the phantom actually is: 0.0888 mm RMSE for the forecast at `h = tau_s = 0.677 s`
  against 0.2332 mm for reading the sensor and treating it as current. First end-to-end
  evidence on hardware that a signal read 677 ms late can be put back into the present.
- **The horizon is cheap; the lag is expensive.** Forecast RMSE rises from 0.0710 mm at
  `h = 0` to 0.1016 mm at `h = 0.70 s` — +0.031 mm to buy back 0.144 mm. The curve is nearly
  flat to ~0.4 s, so there is real headroom for `tau_c`, `tau_cl` and `T_ins` once measured.
- **The 95% energy rule now has a price attached.** K=1 forecasts 49% worse than K=3
  (0.0766 vs 0.0515 mm) on real bench data. Previous evidence was residual structure; this is
  the cost in the units that matter. `configs/bench_aligned.yaml` uses `K_override: 3` and
  keeps the 95% threshold configured but unused, so the report prints both.
- **A measured `R` moved NIS from 0.040 to 0.243, not above 1.** So `S` is dominated by
  `HPH'`, not by `R`: the filter's remaining inconsistency lives in **Q and the model, not the
  noise floor**, and correcting `R` alone will not fix it. Use `identifier.params.R_override`
  to supply a measured value; `R_source` always names which of the three paths was used.
- **A smaller, more honest `R` slightly worsens the forecast** (0.0820 → 0.0989 mm) while
  substantially improving covariance consistency. The gate needs the honest covariance more
  than the last 0.017 mm of accuracy.
- **`aligned.csv`'s `y_clean` column is what makes a bench forecast score meaningful.**
  `CSVSource` reads that exact name into `SignalBatch.y_clean` and `ct.run.truth_function`
  prefers it, so writing the phantom truth there scores the forecast against the *phantom*
  rather than against the sensor. Without it the score is circular.

### From the phantom-vs-sensor comparison (session 007)

- **The tactile chain sees ~1/3 of the phantom's real excursion, 677 ms late — but
  reproduces the waveform almost exactly once both are removed.** 2.64 mm of phantom motion
  reads as 0.86 mm of deflection; correlation is 0.961 and the residual RMSE 0.072 mm after
  taking out the lag and the scale. The lag *is* `latency.tau_s`; the attenuation sets the
  amplitude the estimator actually works from. Neither was measured before.
- **Any amplitude or agreement metric must be computed after removing the lag.** Projecting
  the sensor onto an unshifted phantom folds delay into scale: at 677 ms against a ~4.9 s
  breath that costs `cos(2*pi*0.677/4.93) ~ 0.63`, and `compare_logs` reported 0.203 where
  the truth is 0.326. Fixed, and pinned by `test_amplitude_ratio_does_not_depend_on_the_lag`.
  The independent check is Stage 1 on `aligned.csv`: `A_1` = 0.346 mm sensed, 1.063 mm true.
- **Comparison metrics must be restricted to one procedure state.** Scored across a whole
  bench run, correlation reads 0.039; over `standoff_hold` alone, 0.961. Approach and seat —
  base moving, sensor unseated — do not dilute the answer, they replace it. Pass
  `ct-compare --phase standoff_hold`.
- **The estimator recovers the same fundamental from the sensor as from ground truth**
  (12.172 vs 12.192 bpm) despite the attenuation and the delay. Frequency survives the
  sensing chain where amplitude does not.
- **677 ms is probably contact physics, not electronics** — most likely the viscoelastic
  skin/lever settling session 005 measured at 1.39 mm over 5 s. If so `tau_s` varies with
  seating depth rather than being a constant. **Both halves confirmed in session 013 — see
  below.**

### From the lag decomposition (session 013)

- **The 677 ms was never sensor latency, and it is not a constant.** The ToF is non-contact but
  shares the CAN bus, the tick loop and the motion, so scoring it splits the chain: it lags
  **0.011–0.098 s** where the tactile arm lags **0.282–0.696 s** across seven runs. The
  difference is viscoelastic settling in the *contact*. The floor matches the ~8 ms frame
  period plus the ~11 ms tick, and is an upper bound (the ToF quantises to 1 mm on a 4.8 mm
  excursion). 0.677 also carried the phantom motor's own ~0.05 s tracking lag, having come from
  the one run on disk with no `measured_mm`. Measure the floor with `ct-compare --sensor tof_mm`.
- **Press harder and the sensing gets worse in both respects.** Correlation with the accepted
  seat peak is **+0.63 for the lag and −0.84 for the amplitude ratio**. Lightest seat
  (0.362 mm): 0.282 s, 0.760, r = 0.992. Heaviest (1.93 mm): 0.562 s, 0.163, r = 0.806. Deeper
  seating engages more material in a softer, more dissipative regime and loads the pivot
  harder. **Seat light. It buys more than any amount of forecasting.**
- **The lag explains almost none of the attenuation.** Dividing the measured gain by what the
  lag alone would cause (`1/sqrt(1+(ωτ)²)` — 0.83–0.95 for every run) leaves a *static* gain of
  0.82 down to 0.19. Delay and attenuation are separate effects that degrade together. Stiction
  compounds it: samples with exactly zero change go 6.8% → 16.3% with seating depth.
- **Never freeze a measured lag in a config.** `configs/bench_aligned.yaml` held `horizon: 0.677`
  and was still using it on runs whose real lag was 0.28 s, where it scored *worse than not
  forecasting* (0.4135 mm vs 0.3095 mm). `plot_approach_and_seat.py` now assembles `h` per run
  via `forecast.horizon_from_components`.
- **A cross-correlation over a periodic signal needs a periodicity guard.** There is a sidelobe
  every `T_breath` and `argmax` cannot prefer the true one — a ±3 s ToF scan against ~5.8 s
  breathing returned −2.77 s. `compare_logs` clamps to just inside `T/2` and flags
  `lag_ambiguous`. Likewise **sensor polarity must be declared, not discovered** (`SENSOR_SIGN`):
  for a near-sinusoid an inverted sensor is indistinguishable from a correct one half a breath
  away. And normalise by `sqrt(Ec*Ep)`, not by the overlap count — the latter over-corrects the
  triangular taper and biases the peak outward.
- **The EKF loses frequency lock on real subject profiles, and `y_pred` hides it completely.**
  On run 20260902-160517 Stage 1 finds 10.52 bpm against a true 10.34 and the tracker then runs
  at **4.55 ± 3.60 bpm**, with `phi_1` spinning **9.70 rad** to absorb the error. `A_k`, `phi_k`
  and `(theta, omega_r)` are partially redundant, so a drifting `phi_1` mimics a frequency
  offset and the measurement still fits perfectly — the tracking panel looks flawless. Only the
  forecast breaks, because it is the one operation using `omega_r` alone: asked to advance
  0.279 s it advances ~0.031 s. **Check the `frequency lock` line before trusting any forecast
  number.** Across all seven bench runs, each scored at its own measured lag, the ratio
  tracked/Stage-1 `omega_r` orders the forecast result monotonically — 0.98 → +60.5%,
  0.66 → −2.8%, 0.43 → −9.0%, 0.21 → −23.4% — so the forecast failure *is* the frequency
  failure, not the lag, the horizon, or the sensing chain.
- **Session 008's +62% headline holds only on the synthetic looping profile.**
  `breathing_profile_1` is 14.8 s, loops 12× in a 180 s hold and has *zero* baseline wander;
  `emma_normal_breathing` is 618 s with 0.267 mm of slow wander on a 4.62 mm excursion. Every
  emma run loses frequency lock (NIS 0.007–0.020); the looping one does not (NIS 0.245). This
  is session 009's warning about short looping profiles, now reaching the estimator.

### From the EKF tuning analysis (session 015)

- **The 0.349s tactile delay is real, steady-state, and now has a picture, not just a number.**
  `scripts/plot_lag_detail.py` plots the sensor against phantom ground truth over a few breath
  periods; unshifted, the sensor visibly peaks and troughs ~0.35s after truth on *every* cycle
  of `standoff_hold` (already excluding APPROACH/SEAT), and shifting it back by the measured lag
  snaps the two traces together (correlation 0.912 -> 0.988). This is viscoelastic contact
  settling happening continuously, not a one-time contact-transient artifact, and has nothing
  to do with the sensor's ~100Hz sample rate.
- **A plausible trough-degradation theory was wrong; the real pattern is asymmetric, not
  symmetric.** The obvious candidate — the measurement Jacobian's `dh/dtheta` vanishing at any
  extremum, peak or trough alike — is **disconfirmed** on real data
  (`scripts/analyze_observability.py`): bucketed by `|H_theta|` quartile, the *highest*
  quartile has the *lowest* forecast error, the opposite of the prediction. The real pattern is
  asymmetric: trough forecast RMSE is 1.5x peak RMSE (0.224 vs 0.149mm), while peaks are no
  worse than mid-slope. This points at something contact-specific happening only at end-exhale
  (loss of skin contact, per `approach.py`'s own docstring, or loading/unloading hysteresis in
  the viscoelastic material) rather than an information-theoretic property of the harmonic
  model — still unconfirmed which, and the two remain indistinguishable from one run.
- **`identifier.params.q_scale` has a sharp, non-monotonic effect on frequency lock, not a
  smooth trade-off.** Swept 0.1-5.0 on run `20260903-171153` holding R and everything else
  fixed: frequency lock is completely broken (tracked omega collapses to 1-2bpm against an
  11.4bpm truth) for `q_scale <= 0.40`, and recovers sharply at `q_scale >= 0.42` — but the
  recovered band is itself uneven (0.55/0.6 score worse than 0.5/0.7). At `q_scale = 0.5`:
  NIS 0.052 -> 0.092, frequency-lock std 0.92 -> 0.49bpm, forecast RMSE 0.2056 -> 0.1747mm, lag
  error removed 33.9% -> 43.9% — a real, simultaneous improvement on every metric, now the
  default in `configs/bench_aligned.yaml`, but on **one run only**; the bimodal transition
  itself is evidence Q's per-state structure needs attention, not proof this constant
  generalizes.
- **16 calibration breaths (the 180s `--record-s` default already working) was not, by itself,
  enough to fix the standing NIS/lock shortfall.** CLAUDE.md's prior "more breaths is the first
  thing to try" is now tried and answered: insufficient alone. Surfaced via a new
  `_report_q_diagnostics` line in `plot_approach_and_seat.py` that prints `n_breaths` /
  `omega_source` from `ident.diagnostics['q']`, which `estimate_Q` had always computed but
  never shown.
- **No bench run has an in-contact `R` opportunity, and now it's clear why.**
  `configs/rig_bench.yaml` sets `procedure.approach.breath_hold_s: 15.0`, but that belongs to
  `ct.control.states.approach` (the real `ct-rig` four-state procedure) —
  `scripts/run_approach_and_seat.py`, which produces every bench run analyzed so far,
  implements its own simpler phase machine (`approach`/`seat`/`standoff`/`standoff_hold`) with
  no breath-hold step at all. Measuring in-contact `R` needs either a `ct-rig` run or a
  deliberate stationary segment added to the bench script.
- **No ground-truth leakage into the estimator, confirmed by code trace.** `CSVSource`
  (`src/ct/sources/csv_source.py`) loads `y` strictly from the configured `y_column`
  (`sensor_mm`) and `y_clean` only from a column literally named `y_clean`, independent of
  `y_column`; `y_clean`/`truth` reach only `truth_function`/`forecast_target` for scoring, never
  `Identifier`/`Tracker`. The EKF beating the raw sensor in places is forecasting doing its job
  (session 008), not a leak.

### From the multi-profile parameter sweep tooling (session 016)

- **`q_scale=0.5` does not generalize even to a second trial of the same profile.** Run
  `20260903-185614` (also `emma_normal_breathing.csv`) loses frequency lock at **every**
  `q_scale` from 0.3 to 1.0, including the pre-session-015 default — a real property of that
  run, not something the session 015 config change caused. Confirms session 015's own caveat
  that one run cannot validate a tuning choice, now with a second real data point.
- **`omega_bounds`, not `q_scale`, may be the more fundamental lever — on a sample of 2 runs,
  not yet the real answer.** Sweeping `tracker.params.omega_bounds` at ±10% of Stage 1's rate
  kept **both** available runs locked at every `q_scale` tested, where no `q_scale` value alone
  could lock `185614` at all. `omega_bounds` clamps the state the `phi`/`omega` redundancy
  corrupts directly, rather than discouraging drift indirectly through `Q`. Needs the real
  5-profile x 2-trial sweep (`scripts/sweep_ekf_params.py`) before this is more than a lead.
- **Bench data collection is now systematic tooling, not one-off scripts.**
  `scripts/collect_param_sweep_runs.py` runs several profiles x several trials, auto-retracting
  20mm between attempts (explicitly requested and bounded — see the note on session 014 below)
  and auto-discarding/retrying an attempt whose `summary.json` shows near-instant contact or an
  incomplete run, calibrated against the six real runs on disk (good: `contact_t` 2.36-6.43s;
  bad: 0.0063s and 0.0002s, the session-012 signature). `scripts/sweep_ekf_params.py` then
  sweeps `q_scale x omega_bounds` over whatever it collected and picks a safety-first winner
  (disqualify anything that breaks lock on any run), or says plainly that nothing is safe.
- **Session 014's refused auto-retract and this session's implemented one are not in tension.**
  014 refused an *unrequested* auto-retract drafted as a side fix to a different problem; this
  session's 20mm retract between sweep trials was explicitly requested, for this purpose, at
  this specific bounded distance — exactly the authorization 014 said was the actual bar.

### From the real 10-run sweep (session 018)

- **`q_scale=0.5` alone locks only 2 of 10 real runs; the pair with `omega_bounds=±10%` locks
  all 10.** The first real 5-profile x 2-trial sweep confirms session 016's n=2 lead at full
  scale: mean forecast RMSE 0.293mm, max 0.591mm, and it is the only setting (of 44 tested)
  that avoids the catastrophic-lock failure mode on every collected run. Session 015's fix was
  real but incomplete — `configs/bench_aligned.yaml` now carries both
  (`tracker.params.omega_bounds_fraction: 0.1`), resolved at runtime from each run's own
  Stage-1 rate via `ct.run.resolve_tracker_params()` (a fixed rad/s range can't cover subjects
  at 10-22bpm; only a bound relative to *this run's* rate can).
- **Fixing frequency lock does not reliably improve forecast RMSE, even though it should be
  trusted more.** On `emma_normal_breathing/trial_2`, the tuned config's forecast RMSE
  (0.56-0.67mm depending on horizon) is *worse* than just reading the raw sensor late
  (0.27-0.31mm) — visible in `scripts/plot_ekf_detail.py`'s output. Lock and forecast accuracy
  are separate claims; fixing one is not evidence for the other.
- **"Massive FFT vs autocorrelation disagreement" (noticed running the sweep) is real but a
  volume effect, not a magnitude one.** `ct.identification.spectral.coarse_omega` warns above
  10% disagreement; headline disagreement across the 10 real runs is 0.3-10.8% (junrong
  10.7-10.8%, derek 9.0-9.5%, both close to the threshold and consistent with session 006's
  already-known finding that derek's rate drifts within a take). The alarming *volume* comes
  from `identify()` being called once per `q_scale` in a sweep (11x) with each call re-invoking
  `coarse_omega` per Q's sliding sub-window (0-6 more times, sometimes far worse than the
  headline — one sub-window disagreed by 69.8%) — 100+ warnings per sweep run, each with a
  distinct embedded percentage so Python's default dedup never collapses them.
  `scripts/analyze_frequency_disagreement.py` gives the clean per-run summary instead.
- **A parameter sweep that mixes a fractional and an absolute form of the same knob needs both
  forms stripped between grid points, not just one.** `sweep_ekf_params.py` stripped the
  resolved `omega_bounds` key from the base config's tracker params but not the newer
  `omega_bounds_fraction` key; once `bench_aligned.yaml` carried the latter as a baseline
  default, every "unset" grid point silently inherited it anyway. The tell was numeric, not
  logical: the "unset" and "0.1" rows came out bit-identical.

### From the first real subject data (session 006)

- **Real chest-wall breathing amplitude is ~0.5–1.4 mm RMS on a 4–8 mm peak-to-peak
  excursion**, across seven subjects. That is an order of magnitude below the 10 mm amplitude
  `configs/sinusoid.yaml` uses. It puts a measured number on the requirement that the tactile
  sensor's usable stroke exceed the breathing excursion — though these are surface markers,
  not the target organ, so the target's own amplitude is still unknown.
- **The optical data is far cleaner than the synthetic configs assume**: uniform 120 Hz with no
  jitter, at most 15 dropped frames in 73k, and a largest frame-to-frame step of 0.08–0.12 mm.
  Effectively no high-frequency sensor noise at 120 Hz.
- **The 95% energy rule selects `K=1` on all seven real subjects.** The RC-piecewise finding
  above now has real-data backing; re-validate the threshold rather than carrying it over.
- **The breathing rate is not stationary within a take, and for one subject it drifts enough to
  break a fixed-`omega` Stage 1.** Derek's dominant peak measures 20.67 bpm over t ∈ [0,90),
  14.67 over [90,180), 14.00 over [300,390); fitting the whole 90 s calibration window at one
  `omega` recovers only `A_1 = 0.207 mm` of a 0.670 mm-std signal. This is the
  `sinusoid_ramp.yaml` failure mode observed on real data. Emma and Moira are far more
  stationary, so it is subject-dependent — a calibration-window stationarity guard is worth
  considering.

### From the needle plant characterization (session 019)

- **The forecast horizon's `tau_cl(omega_r)` may be computed against the wrong closed-loop
  architecture.** `ct.control.states.insert._hold_standoff()` commands `reference_mm +
  correction` — feedforward plus feedback, by its own docstring — not `correction` alone, but
  `ct.control.servo`'s `closed_loop_response()`/`residual_lag()` compute the standard
  unity-feedback `T=L/(1+L)`. The real architecture's transfer function is algebraically
  `G(1+C)/(1+GC)`, not `GC/(1+GC)`. Simulating the actual `LeadServo` in its real loop
  (`scripts/simulate_needle_lead_tracking.py`) against the measured plant found an 18%
  amplitude and ~50% lag discrepancy from `closed_loop_response()`'s prediction at the same
  frequency, matching that derivation's estimated ratio (`1+1/|C(jw)| ≈ 1.22` there, observed
  1.18). Not yet fixed — this touches a formula every past session's `tau_cl` conclusion has
  used, and deserves its own dedicated investigation rather than a fix folded into an unrelated
  session.
- **A phase's timing budget must be sized from what it needs to do, not copied from a sibling
  phase.** `run_needle_step_response.py`'s retract phase reused the step phase's fixed
  duration; whenever the step's commanded velocity exceeded the retract velocity's capacity in
  that same window, the retract silently ran out of time and the *next* rep started mid-reversal
  instead of from rest — fully explaining a batch of fits that had degenerated to `wn≈300 rad/s,
  zeta≈0.026`. Fixed by sizing the budget from measured travel and confirming arrival from
  telemetry, the same "never dead-reckon, judge arrival from the motor's own replies" principle
  sessions 005 and 009 already established for other phases.
- **A GL-II motor's own `motor_velocity_rad_s` reply field cannot be trusted at its documented
  decode range.** Read 6.7-8.5x the commanded velocity limit across every needle-motor run this
  session, consistently enough to be systematic rather than noise — consistent with
  `listen_needle_motor.py`'s already-documented caution that the GL-II manual is internally
  inconsistent about "rad/s" vs "r/s" for this exact field. Not fixed at the source (the range
  constant is shared by four scripts); worked around by differentiating position instead.
- **Only the very first commanded step since a process connects fits cleanly; every later rep in
  a back-to-back sequence does not, and not always in the same way.** True even after the
  retract-timing bug above was fixed and confirmed (`retract_arrived: True` throughout) — one
  run's later reps pegged `zeta` against its upper bound, a different run's pegged it against
  the lower bound instead. Mechanism unknown; not chased further per this session's explicit
  "helps a little, doesn't need to be perfect" scope. The measured plant
  (`K=0.92, wn=27.0, zeta=0.35` in `configs/rig_bench.yaml`) is the mean of four independent
  clean ("first command") trials only.

### From in-tissue needle identification (session 020)

- **The only-first-rep-fits-cleanly mystery (session 019) is confirmed medium-independent, and
  two specific causes are now ruled out.** Reproduced identically in tissue, which rules out
  tissue creep/relaxation as the cause (nothing to creep in free air, where it was first seen).
  Resending `CLEAR_ERRORS`+`ENTER_MODE` before every rep (not just once per run) also failed to
  fix reps 1+ — they stayed degenerate, just with a different failure signature (`zeta` pegged
  low with `wn`~300 this time, vs. `zeta` pegged high with `wn`~80-230 in session 019) — so a
  stale mode-entry state is ruled out too. Root cause remains unknown; the practical fix is
  collecting more independent single-command trials, not more reps per invocation.
- **Even the "reliable" condition isn't immune to outliers, and small samples can look like real
  effects when they aren't.** 1 of 10 independent cold-start trials landed ~30 standard
  deviations from the other nine (`wn=138` vs. a 22-35 cluster) — a categorical outlier, not a
  borderline call. And the first in-tissue attempt (n=4) suggested `wn` was meaningfully higher
  in tissue than free air (39.2 vs 27.1) — collecting 10 trials instead of 4 converged the
  estimate back to 27.2±3.7, matching free air almost exactly. **The apparent tissue effect was
  a small-sample noise artifact.** At the tested velocity/excursion, tissue contact does not
  measurably change this axis's identified dynamics — `configs/rig_bench.yaml` was left
  unchanged.
- **A user-supplied mass cannot produce an independent second plant estimate from the same
  step-response data, and this generalizes**: solving `wn=sqrt(k/m)`, `zeta=b/(2sqrt(km))` for
  `k`/`b` using an already-measured `wn`/`zeta`, then recombining them, returns the identical
  `wn`/`zeta` — mass cancels out algebraically. Relabeling a measurement through an invertible
  parameter transform is not a second, independent measurement of anything, regardless of what
  physical quantity is used to do the relabeling. Worth remembering the next time someone
  proposes cross-checking a fit "a different way" using only information already used to
  produce it.

### From lead compensator hardware validation (session 021)

- **The session-019 lead compensator (gain=12.74, unmodified) gives a real, measured tracking
  improvement on real hardware — but only once tested at the rate it was actually designed for.**
  A live A/B test (`scripts/run_needle_lead_tracking_live.py`) at the bench script's default
  20Hz command rate oscillated badly; at 100Hz it was much better but the compensator still
  slightly hurt (0.0762 vs 0.0667mm RMSE, −14.3%); at 200Hz — `configs/rig_bench.yaml`'s actual
  `procedure.loop_rate_hz` — the same unmodified compensator gave a genuine improvement (0.0466
  vs 0.0502mm RMSE, +7.2%, zero saturation). **A bench test's command rate is not a neutral
  parameter**: real axis tracking lag against the same reference measured 0.139mm std at 20Hz,
  falling to an RMSE of 0.0667mm at 100Hz and 0.0502mm at 200Hz — an unrepresentative rate can
  dominate the very thing being measured, and can make a working design look broken.
- **A plausible root-cause theory that produces a number in the wrong ballpark should be
  dropped, not rationalized.** Suspected the compensator's ~12.7x high-frequency gain was
  amplifying real position-*sensor* noise; measured it directly
  (`scripts/measure_needle_position_noise.py`) and it came back tiny (0.005mm at rest,
  projecting to only ~0.096mm of correction noise) — nowhere near the multi-millimetre
  corrections observed. The real driver, confirmed from the *uncompensated* phase of the same
  run, was dynamic tracking lag (0.139mm std) from the axis chasing a reference that only
  updated 20 times a second — a different quantity than sensor noise, 30x larger, and the actual
  explanation.
- **A magnitude-only clamp on a compensator's correction is not sufficient protection against a
  sudden or coarse-rate-driven error; a genuine rate limit on the correction itself is a
  materially different safeguard, not a redundant one.** `LeadServo.update()` gained
  `rate_limit_mm_s`, chained after the existing magnitude clamp, both now defaulting from new
  `AxisServoConfig` fields rather than being passed in per call site — the same units confusion
  that caused the bug below happened specifically because the old design required each call
  site to remember which value meant what.
- **A units bug had been live in production control code since it was written**: `_hold_standoff()`
  reused `ctx.geometry.needle.v_max_mm_s` (mm/s, a genuine velocity ceiling used correctly
  elsewhere) as `LeadServo.update()`'s magnitude clamp (mm, a position). Never caught by
  simulation, which has no real measurement lag of the kind that exposed it here. Fixed, and the
  compensator extended from `_hold_standoff()` alone into `AdvanceState`'s increments and
  `InsertState._drive()` at the user's explicit direction (its job is general tracking fidelity,
  not standoff-specific) — not yet validated on real hardware, only in simulation via the full
  test suite.

### From needle actuation on hardware (session 022)

- **The gate fires on `forecast(t + h)`, so anything that tests `value(t)` against the same band
  is structurally inconsistent with it.** During a descent `value(t)` is always above
  `forecast(t + h)` — that gap *is* the horizon. So `advance_drive`'s old inhale abort tripped the
  instant a drive started whenever `inhale_abort_frac` sat below wherever `value(t)` happened to
  be at fire time: never at 0.85, in 6ms at 0.25. **No threshold value fixes this**, and it
  presents convincingly as a tuning problem. Replaced with a model-free rule on the raw sensor —
  reference the reading at drive start, require a descent of `--abort-margin-mm` to arm, float
  when it returns to that reference — which is symmetric about the trough by construction and
  assumes no waveform, in `advance_abort_decision()`.
- **Pausing the tracker across a needle drive is far worse than feeding it the drive's data, and
  the premise for pausing was never true.** The raw tactile signal is clean straight through both
  drives (breakthrough: `5.76 → 3.99` trough `→ 8.40` peak; advance: `5.02 → 3.68 → back`). The
  pause instead produced gaps up to **6.1s (1.5 breaths)**, after which tracked `omega_r` was
  observed at roughly half the true rate and antiphase — so the gate fired a drive at **peak
  inhale** while believing it was firing at end-exhale. Feeding continuously took the largest
  single `step()` dt from 6.09s to 0.045s and model-vs-sensor error from 4.1mm (antiphase) to
  0.231mm mean. **Why omega collapsed is still unexplained** — a Q-scales-with-dt explanation was
  proposed and *not* confirmed by replay.
- **`scripts/run_approach_and_seat.py` had never adopted `omega_bounds`**, despite session 018
  establishing it as the thing that holds frequency lock 10/10 on real runs. Now wired via
  `ct.run.resolve_tracker_params()`. It binds at exactly ±10% in replay and costs a few percent of
  rate accuracy against an unclamped filter that happened to recover — worth it, because the gate
  fires on model *phase* and a 44%-off collapse is unrecoverable.
- **Frequency collapse is silent and total.** Every other signal looked healthy — `y_pred` fit,
  aborts carried plausible reasons, the sequence completed. Only `tracked_omega_r`, sitting unread
  in the telemetry, showed it. There is now a warning and `omega_stage1`/`omega_tracked_min|max`
  in the summary.
- **A wrong `dt` through this model returns plausible, in-range numbers.** `theta` only enters via
  `sin`/`cos`, so `dt = -1.2e6 s` (from mixing rebased `elapsed` with raw-monotonic `tracker.t`,
  which is what the CAN mailbox stamps) still produced values inside the correct amplitude band —
  phase-aliased nonsense that looks like data. Bounded and believable is not correct.
- **The re-arm sleeps were dead weight.** `0.1s` after `CLEAR_ERRORS` and `0.5s` after
  `ENTER_MODE`, copied into every script doing that handshake, never justified by any datasheet or
  measurement. `--rearm-sleep-s 0.0` runs clean on hardware. The `drive_started_at − fired_at` gap
  measured in earlier runs was never independent evidence — it was those same sleeps read back.
- **A backstop that can fire before the primary rule arms is not a backstop.** The retired model
  check, left wired as a parallel `or`, went on deciding every abort because the bench command
  line still passed it a value. Retired flags should warn and do nothing, not stay live.
- **Replay the shipped function, not a re-implementation of it.** The failure above was "a
  different rule fired", which a hand-written replay structurally cannot detect. The abort rule is
  now a pure function called by both the control loop and the offline check.
- **The sensor's CAN payload changed** when the Teensy was reflashed: `struct("<Hfh")` (uint16
  ToF, float32 dist_cm, int16 angle) → `struct("<ff")` (float32 ToF mm, float32 dist_cm). The
  angle field was dead downstream — decoded and logged at five sites, read by none.

## Open questions

Carried forward; update rather than rediscover.

Most of the rig's physical constants now live in [unknowns.py](src/ct/unknowns.py) rather
than here — that list is machine-checked and generates
[docs/unknowns.md](docs/unknowns.md). What remains below is the part no config key can
capture.

- **Why did tracked `omega_r` collapse to roughly half the true rate after a paused-tracker gap?**
  (Session 022.) Observed directly in run 7 (1.7497 → 0.932 rad/s across one catch-up step, ending
  at 0.876 against a true 1.5647, antiphase, firing a drive at peak inhale). A
  Q-scales-with-`dt` explanation — a ~6s gap inflating `P` ~230x so one measurement rewrites
  `omega` rather than `theta` — was proposed and **not confirmed**: a faithful replay reproduced
  the drift up to 1.76 rad/s but not the collapse. The fix (no gaps + `omega_bounds`) bounds the
  failure regardless of mechanism, so this is not blocking, but the mechanism is unknown and may
  bite somewhere the clamp does not reach.
- **Does the raw-sensor symmetric-return abort hold on non-sinusoidal profiles?** (Session 022.)
  It assumes no waveform by design, which is the reason to expect it to — but it has only run
  against the synthetic sinusoid. Moira and emma are the test.
- **`--max-advance-drive-s 3.0` and `--abort-margin-mm 0.2` are unvalidated first guesses**,
  in the same status `exhale_band_frac` held before it had real data behind it.

- **Does `tau_cl(omega_r)`/`residual_lag()` need to account for the real feedforward+feedback
  architecture `_hold_standoff()` actually uses, and if so, by how much has every past
  session's `tau_cl` number been off?** (Session 019.) The current formula assumes standard
  unity feedback; the real control code commands `reference + correction`, which is
  algebraically a different closed loop. Needs a dedicated session: re-derive the transfer
  function for `u = r + C(r-y)`, decide whether `ct.control.servo` should expose it as the
  default, and check whether it changes any past forecast-horizon conclusion materially.
- **Whether the compensator's extension into `AdvanceState`'s increments and `InsertState
  ._drive()` (session 021) actually behaves on real hardware.** Validated only in simulation via
  the full test suite so far; the live A/B test that found and resolved the 20Hz-vs-200Hz
  oscillation only exercised `_hold_standoff()`'s continuous-tracking case. A fixed-target step
  command has a different initial-error shape (large and instantaneous, not building up
  gradually) that could plausibly trigger oscillation differently — needs its own `ct-rig`
  hardware check, not an inference from the standoff-hold result.
- **`configs/rig_bench.yaml`'s `servo.needle.correction_rate_limit_mm_s` placeholder (10.0)
  does not match the value actually validated on real hardware at 200Hz (5.0, session 021's
  bench-script default).** Both are still PLACEHOLDER in `unknowns.py`; needs a deliberate test
  (not a guess) before either is promoted to measured. Session 022 added a *separate*
  `NEEDLE_INSERTION_SERVO_CONFIG` (10.0mm / 40.0mm/s) for the large step insertions, because the
  standoff-tracking clamps saturated 77 and rate-limited 294 of ~600 drive ticks — those two are
  also first guesses, and are deliberately distinct from the tracking servo's.
- Sensor modality and its real measured latency (optical ~10-30 ms vs ultrasound ~50-150 ms).
  `ct-compare` measures it directly once both rigs run. The optical takes in
  `unfiltered_data/` carry no synchronised second clock, so they do **not** settle `tau_s`.
  **The tactile bench chain does, at 677 ms (session 007)** — far larger than either estimate
  above, which is itself the reason to suspect it is contact settling rather than sensing.
- ~~Whether the phantom motor broadcasts status densely enough while being commanded~~
  **resolved (session 009): 51.1 Hz**, 2465 frames over 48.2s, a real measurement on
  4209/4209 ticks. `measured_mm` is genuine ground truth, not a zero-order hold, and the
  phantom tracks its own command to 99.1%. `ct-compare --truth measured_mm` can now separate
  the phantom's own tracking lag from the sensing chain's — which no run has done yet, so
  every latency number so far still uses `commanded_mm` and includes both.
- ~~Whether the 677 ms lag and the 0.33 amplitude ratio are constants or move with seating
  depth~~ **resolved (session 013): they move, strongly.** Lag 0.282-0.696 s and amplitude
  0.16-0.76 across seven runs, correlating +0.63 and -0.84 with seat depth. The estimator does
  see a time-varying gain, and the horizon is now measured per run rather than configured.
- **Why the EKF loses frequency lock on real subject profiles** (session 013). Tracked
  `omega_r` collapses to 4.55 bpm against a true 10.34 while `y_pred` still fits, which breaks
  the forecast silently. **Resolved for lock itself (session 018):** the real 5-profile
  x 2-trial sweep found `q_scale=0.5` + `omega_bounds` at ±10% of Stage 1's rate keeps lock on
  all 10 real runs (`q_scale` alone locks only 2/10) — now the default in
  `configs/bench_aligned.yaml`, resolved at runtime via `ct.run.resolve_tracker_params()`.
  **Still open:** fixing lock does not reliably improve forecast RMSE per run (session 018
  found the tuned config's forecast *worse* than the raw sensor on one real run) — lock and
  forecast accuracy are separate claims, and nothing about the forecast should be assumed
  fixed just because lock now is.
- **Whether the contact is a delay or a filter.** Phase lag per harmonic on run 160517 is
  0.287 s at the fundamental and 0.407 s at harmonic 2, where a pure delay predicts a constant.
  If it holds, harmonic `k` needs a different advance than `k*omega_r*h` — settled decision 4
  would be right for delay but incomplete for this contact. Not yet a measurement worth acting
  on: harmonic 2 carries 5.7% of the fundamental's energy and harmonic 3 is noise.
- **Whether the ToF is simply a better estimator input than the tactile arm.** It sees
  0.79-1.10 of real excursion at under 0.1 s against the arm's 0.16-0.76 at 0.28-0.70 s. Its
  1 mm quantisation is a resolution problem with known fixes; the contact lag is physics.
- Target organ / expected motion amplitude
- Clinical tolerance epsilon
- Insertion depth / achievable needle velocity (sets `T_ins`)
- Breath-hold viability as a fallback — note it would also give the preferred `R`.
  `procedure.approach.breath_hold_s` implements it; `rig_bench.yaml` has it on at 15 s.
- ~~Real sensor data: format, sample rate, availability timeline~~ **resolved (session 006)**:
  OptiTrack CSV, 120 Hz, mm, nine chest-wall markers, seven subjects × 8–11 min, reduced into
  `breathe_profiles/`. Latency is the part still open, above.
- **Where `breathe_profiles/breathing_profile_1.csv` came from is unknown.** It is not derived
  from any of the seven takes in `unfiltered_data/` (best correlation r = 0.005, against Jake,
  despite a similar DC level). Trace it or retire it before anything depends on it.
- Confirmation that the team's intended chest-wall model is Singh et al. 2020
- **Which firmware the CubeMars motors are flashed with — resolved for needle/base as of
  session 004/005.** Both run the Gimbal Motor II Position/Velocity protocol (not MIT, not
  servo), now confirmed against the real vendor manual rather than empirically inferred.
  ~~ADVANCE's need for MIT-mode float on the needle is therefore not available as-is~~
  **resolved (session 004, itself — this was carried here stale until session 022 caught it):**
  the actual compliance requirement (needle must not stall between insertion increments) needs
  only zero-torque disable, which the Gimbal protocol already provides identically to MIT mode
  (the `EXIT_MODE` universal command every needle bench script already sends at cleanup). MIT
  mode's finer-grained `kp=0, kd=small` "soft float" was never actually needed. Why MIT mode
  never produced clean *position control* on this motor remains unexplained, but is now
  irrelevant to floating specifically.
- **What interface/channel the RH02 enumerates as** (`python -m can.detect_available_configs`).
  Nothing can be tried on hardware until this is settled.
- **`forecast_variance` is still uncalibrated against realised error.** It is what the
  firing gate thresholds on, so `max_forecast_std_mm` is a guess until it is checked.
  Carried from session 001 and now load-bearing. Session 008 measured the realised error
  (0.0888 mm at `h = 0.677 s`), so this can finally be set against something real.
- **Q, not R, is the leading suspect for the NIS shortfall** (session 008). NIS sits at 0.243
  with a measured `R`. ~~Q is estimated from breath-to-breath refits over only ~6 breaths in the
  runs so far; the new 180 s `--record-s` default gives ~18, which is the first thing to try.~~
  **Tried and insufficient alone (session 015):** run `20260903-171153` already used 16
  breaths and still showed NIS 0.052 before any other change. The next thread is `q_scale`'s
  sharp transition, above — Q's structure, not the amount of calibration data.
- **In-contact `R` is unmeasured.** The 7.3e-6 mm² in `bench_aligned.yaml` is free-air with the
  arm unloaded, and so a lower bound. A breath-hold during `standoff_hold` (pause the phantom,
  set `identifier.params.breath_hold_window`) would measure it properly. **Session 015 found
  why no bench run has this yet:** `procedure.approach.breath_hold_s` is implemented in
  `ct.control.states.approach` (the real `ct-rig` procedure), but every bench run so far went
  through `scripts/run_approach_and_seat.py`'s own simpler phase machine, which has no
  breath-hold step at all. Needs either a `ct-rig` run or a bench-script change.
- **Whether the corrected hold-position math actually eliminates the base motor's approach
  overshoot is untested on hardware** as of session 005 — the fix is reasoned from one real
  data point, not yet re-verified by a full run.
- **GL-II Position/Velocity mode's torque/current range is undocumented** — the manual
  gives position (±12.5 rad) and speed (±200 rad/s) but no current/torque scaling, so
  `motor_current` in bench telemetry uses a placeholder and should not be trusted yet.

## Positioning for the paper

- Describe the method as *"an extended Kalman filter matched to an adaptive multi-harmonic
  (Fourier) model with a time-varying fundamental frequency."*
- Nearest biomedical prior art: **WFLC** (Weighted-Frequency Fourier Linear Combiner) —
  Riviere, Thakral, Iordachita, Mitroi, Stoianovici, *"Predicting respiratory motion for
  active canceling during percutaneous needle insertion,"* IEEE EMBS 2001. They adapt the
  same model by LMS. Choosing an EKF is a real, statable methodological difference: it
  yields an explicit covariance the gate can consume directly; LMS does not.
- Nearest exact-math prior art (different domain): power-systems literature on
  *"extended Kalman filter frequency tracking"*, *"dynamic phasor estimation"*.

Full background: [docs/reference/fft_ekf_implementation_context.md](docs/reference/fft_ekf_implementation_context.md).
