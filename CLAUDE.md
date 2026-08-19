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
  any tuning. A hardware sizing requirement, not a tuning problem.
- **A position axis is type-1.** Modelling the needle as a plain second-order lag gave a
  closed loop tracking 55% of its reference and *leading* rather than lagging, which
  silently clamped `tau_cl` to zero and dropped a term out of `h`.

## Open questions

Carried forward; update rather than rediscover.

Most of the rig's physical constants now live in [unknowns.py](src/ct/unknowns.py) rather
than here — that list is machine-checked and generates
[docs/unknowns.md](docs/unknowns.md). What remains below is the part no config key can
capture.

- Sensor modality and its real measured latency (optical ~10-30 ms vs ultrasound ~50-150 ms).
  `ct-compare` measures it directly once both rigs run.
- Target organ / expected motion amplitude
- Clinical tolerance epsilon
- Insertion depth / achievable needle velocity (sets `T_ins`)
- Breath-hold viability as a fallback — note it would also give the preferred `R`.
  `procedure.approach.breath_hold_s` implements it; `rig_bench.yaml` has it on at 15 s.
- Real sensor data: format, sample rate, availability timeline (teammates sourcing)
- Confirmation that the team's intended chest-wall model is Singh et al. 2020
- **Which firmware the CubeMars motors are flashed with.** ADVANCE needs MIT-mode float on
  the needle; the base wants servo mode for its travel. If they are the other way round,
  that is a finding to act on rather than work around.
- **What interface/channel the RH02 enumerates as** (`python -m can.detect_available_configs`).
  Nothing can be tried on hardware until this is settled.
- **`forecast_variance` is still uncalibrated against realised error.** It is what the
  firing gate thresholds on, so `max_forecast_std_mm` is a guess until it is checked.
  Carried from session 001 and now load-bearing.

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
