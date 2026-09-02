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
  seating depth rather than being a constant. Unresolved.

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

## Open questions

Carried forward; update rather than rediscover.

Most of the rig's physical constants now live in [unknowns.py](src/ct/unknowns.py) rather
than here — that list is machine-checked and generates
[docs/unknowns.md](docs/unknowns.md). What remains below is the part no config key can
capture.

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
- **Whether the 677 ms lag and the 0.33 amplitude ratio are constants or move with seating
  depth.** Session 005 measured the related compliance ratio at 0.22-0.68 across runs. If the
  attenuation moves as much, the estimator sees a time-varying gain.
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
  ADVANCE's need for MIT-mode float on the needle is therefore not available as-is — worth
  revisiting when ADVANCE's actual compliance requirement is designed, per session 004's
  still-open question of why MIT mode never produced clean motion on this motor.
- **What interface/channel the RH02 enumerates as** (`python -m can.detect_available_configs`).
  Nothing can be tried on hardware until this is settled.
- **`forecast_variance` is still uncalibrated against realised error.** It is what the
  firing gate thresholds on, so `max_forecast_std_mm` is a guess until it is checked.
  Carried from session 001 and now load-bearing. Session 008 measured the realised error
  (0.0888 mm at `h = 0.677 s`), so this can finally be set against something real.
- **Q, not R, is the leading suspect for the NIS shortfall** (session 008). NIS sits at 0.243
  with a measured `R`. Q is estimated from breath-to-breath refits over only ~6 breaths in the
  runs so far; the new 180 s `--record-s` default gives ~18, which is the first thing to try.
- **In-contact `R` is unmeasured.** The 7.3e-6 mm² in `bench_aligned.yaml` is free-air with the
  arm unloaded, and so a lower bound. A breath-hold during `standoff_hold` (pause the phantom,
  set `identifier.params.breath_hold_window`) would measure it properly.
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
