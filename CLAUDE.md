# CT — respiratory-motion estimation for needle-insertion gating

## Project

Controller for a needle-insertion robot that must fire during specific points of the
breathing cycle. A 1-D sensor tracks the breathing signal; this repo identifies and then
tracks a parametric harmonic model of that signal so its value can be **forecast** at
`t + h`.

**Scope boundary — this repo is the estimator only.** The needle servo (a lead
compensator), the gating logic, and the safety/residual monitor consume `(s_hat, P)` but
are separate work and are not implemented here. Do not add them without being asked.

## Settled decisions — do not reintroduce these errors

These were worked out at length before any code existed. They are settled.

1. **Feedforward and feedback are separate loops.** Breathing prediction (this repo) is
   open-loop disturbance forecasting. The needle servo is a separate closed loop. Keep
   them conceptually and architecturally apart.
2. **`tau_a` is not a horizon term.** The raw actuator delay is replaced by
   `tau_cl(omega_r)`, the *residual closed-loop tracking lag* of the lead-compensated
   servo. It is a servo-design output that varies with breathing rate, not a fixed
   physical constant, and it does not belong to this repo.
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

### The one hard rule

**[layout.py](src/ct/layout.py) is the only place the state ordering is encoded.**

```
s = [a0, A_1, phi_1, ..., A_K, phi_K, theta, omega_r]      n = 2K + 3
```

Nothing else may hard-code an index into `s`. Use `StateLayout(K).A(k)`, `.phi(k)`,
`.theta`, `.omega`, `.amplitude_idx`, `.phase_idx`. This is what keeps the boundaries
genuinely swappable rather than swappable-looking.

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

## Open questions

Carried forward; update rather than rediscover.

- Sensor modality and its real measured latency (optical ~10-30 ms vs ultrasound ~50-150 ms)
- Target organ / expected motion amplitude
- Clinical tolerance epsilon
- Insertion depth / achievable needle velocity (sets `T_ins`)
- Breath-hold viability as a fallback — note it would also give the preferred `R`
- Real sensor data: format, sample rate, availability timeline (teammates sourcing)
- Confirmation that the team's intended chest-wall model is Singh et al. 2020

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
