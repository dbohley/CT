# CT

Given input signals of breathing, predict breathing for needle insertion.

A needle-insertion robot must fire during specific points of the breathing cycle. A 1-D
sensor tracks the breathing signal; this repo **identifies** a parametric harmonic model of
that signal, **tracks** it recursively, and **forecasts** its value at `t + h` so the rest
of the system can act ahead of the delays it faces.

Two stages:

```
Stage 1  identify   FFT + least-squares harmonic regression   ->  (K, s0, P0, Q, R)
Stage 2  track      extended Kalman filter                    ->  (s_hat, P) each sample
         forecast   theta -> theta + omega_r*h                ->  predicted value at t+h
```

The state is `s = [a0, A_1, phi_1, ..., A_K, phi_K, theta, omega_r]`, and the measurement
model is `h(s) = a0 + sum_k A_k sin(k*theta + phi_k)`.

This repo is the **estimator only**. The needle servo, the gating logic and the safety
monitor consume its output but live elsewhere.

## Quickstart

```bash
conda env create -f environment.yml
conda activate CT
pip install -e .

pytest -q                                    # 152 tests
ct-pipeline --config lujan_n2 --plot         # end-to-end run + figures in outputs/
```

## Scripts

All six accept `--config <name>` (a bare name resolves against `configs/`), dedicated flags
for the common fields, and `--set dotted.key=value` for anything else. Every run writes its
artifacts and the fully-resolved config to `outputs/<name>/`.

### `ct-pipeline` — the headline script

Generate or load a trace, identify on the first `--calib-seconds`, track the remainder,
forecast at `--horizon`, and report filter health.

```bash
ct-pipeline --config lujan_n2 --calib-seconds 60 --horizon 0.25 --plot
ct-pipeline --config rc_piecewise --set source.params.cardiac=true --plot
ct-pipeline --config sinusoid --fs 100 --noise-std 0.3 --name noisy_test
```

Figures written: `signal.png`, `identification.png`, `tracking.png`, `states.png`,
`innovations.png`, `forecast.png`.

### `ct-validate-k` — check the K-selection rule

Runs identification across every reference model and compares `K` against the documented
expectations. Exits non-zero on a mismatch, so it doubles as a regression check.

```bash
ct-validate-k --kmax 10
ct-validate-k --kmax 10 --noise-std 0.05 --duration 240
ct-validate-k --energy-threshold 0.99          # see how the answer moves
```

### `ct-generate` — write a synthetic trace

```bash
ct-generate --config lujan_n2 --duration 120 --noise-std 0.05 --plot
ct-generate --source rc_piecewise --set source.params.t1=1.2 --out outputs/fast_inhale.csv
ct-generate --source sinusoid --set source.params.omega_ramp=0.001 --duration 300
```

### `ct-identify` — Stage 1 only

Prints the harmonic table, the chosen `K`, `R`, per-state `Q`, and the initial state with
its uncertainties. Saves `ident.npz` for `ct-track`.

```bash
ct-identify --config lujan_n2 --plot
ct-identify --input outputs/gen_demo/signal.csv --kmax 8 --energy-threshold 0.95 --plot
ct-identify --input my_data.csv --set identifier.params.breath_hold_window="[100.0, 110.0]"
```

### `ct-track` — Stage 2 only

```bash
ct-track --input outputs/gen_demo/signal.csv --ident outputs/id_demo/ident.npz --plot
ct-track --config lujan_n2 --set tracker.params.clamp_amplitudes=true
```

With no `--ident` it identifies first, making it a single-trace `ct-pipeline`.

### `ct-sweep-horizon` — forecast error vs `h`

```bash
ct-sweep-horizon --config lujan_n2 --horizons 0.05,0.1,0.2,0.4,0.8 --plot
ct-sweep-horizon --config sinusoid_ramp --horizons 0.05,0.25,0.5,1,2,4 --plot
```

The curve must grow *smoothly*. A jump is the signature of forecasting by a common phase
rotation instead of a time-advance.

## Configs

| Config | What it is for |
|---|---|
| `sinusoid` | sanity check — the generator matches the estimator's model exactly |
| `sinusoid_ramp` | frequency tracking, with a drifting fundamental |
| `lujan_n2` | asymmetric breathing, `K=2` |
| `lujan_n3` | three true harmonics, but the 95% rule correctly picks `K=2` |
| `rc_piecewise` | Singh et al. 2020 chest-wall model at the settled 95% threshold |
| `rc_piecewise_k4` | the same signal at 99.9%, which is what actually tracks it |

## Swapping a stage

The three boundaries are `typing.Protocol`s resolved by name. To add a tracker:

```python
from ct.registry import register_tracker

@register_tracker("my_ukf")
class MyUKF:
    def init(self, result, t0=None): ...
    def step(self, t, y): ...          # -> TrackerStep
    def forecast(self, h): ...
    @property
    def state(self): ...               # -> (s_hat, P)
```

Then `--set tracker.name=my_ukf`. Nothing else changes. The same pattern applies to
sources (`@register_source`) and identifiers (`@register_identifier`).

## Repo layout

```
src/ct/
  layout.py           state ordering — the single source of truth
  types.py            SignalBatch, IdentificationResult, TrackerStep
  interfaces.py       the three Protocols
  registry.py         name -> class resolution
  config.py           YAML + CLI overrides
  run.py              orchestration; the CLI is a thin wrapper over this
  forecast.py         time-advance to t+h
  sources/            sinusoid, lujan, rc_piecewise, csv
  identification/     spectral, harmonic_ls, noise, fft_identifier
  tracking/           measurement (+ analytic Jacobians), harmonic_ekf
  diagnostics/        metrics, plots
  cli/                one module per ct-* script
configs/              experiment definitions
tests/                152 tests
docs/                 architecture, session log, reference material
```

## Documentation

- [CLAUDE.md](CLAUDE.md) — settled decisions, conventions, session workflow
- [docs/architecture.md](docs/architecture.md) — how the pieces fit and why
- [docs/sessions/](docs/sessions/) — what happened each working session
- [docs/reference/fft_ekf_implementation_context.md](docs/reference/fft_ekf_implementation_context.md) — the source design document
