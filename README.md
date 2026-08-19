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

Since session 002 the repo also contains the **rig controller** that consumes that
forecast: CAN transport, the base and needle axes, the four-state insertion procedure, the
needle servo, the firing gate and the safety monitor. The estimator half remains
independent — it does not import the hardware layer, and a test enforces it.

```
approach   drive the base in on ToF, seat the tactile sensor until the whole breath is
           visible, extend the needle to standoff
estimate   identify the harmonic model, run the EKF until it is consistent
insert     hold standoff against the receding skin, drive in at end-exhale
advance    float in tissue; one increment per breath at end-exhale until depth
```

## Quickstart

```bash
conda env create -f environment.yml
conda activate CT
pip install -e .                             # add ".[can]" only to talk to real hardware

pytest -q                                    # 294 tests
ct-pipeline --config lujan_n2 --plot         # estimator, end to end + figures
ct-rig --config rig_sim --plot               # the full insertion procedure, simulated
```

The simulated rig needs no hardware and no CAN driver: it runs the *same* control code,
the same CAN codecs and the same state machine against a simulated plant under a simulated
clock, so twenty minutes of breathing takes a couple of seconds and gives the same answer
every time.

## What we still need to measure

Most of the rig's physical constants are not known yet. They are not scattered through the
code as TODOs — each is an entry in [`src/ct/unknowns.py`](src/ct/unknowns.py) carrying the
config key it plugs into, its units, how to measure it, who owns it, and which procedure
states it blocks.

```bash
ct-unknowns                        # the table, grouped by owner
ct-unknowns --check rig_bench      # what is still a placeholder
ct-unknowns --format md            # writes docs/unknowns.md, to hand round the team
```

A run against real hardware **refuses to start** while a placeholder still blocks a state
it intends to enter. See [docs/unknowns.md](docs/unknowns.md).

## Scripts

All accept `--config <name>` (a bare name resolves against `configs/`), dedicated flags
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

### `ct-rig` — the insertion procedure

```bash
ct-rig --config rig_sim --plot                  # all four states, simulated
ct-rig --config rig_sim --stop-at estimate      # stop once the model has converged
ct-rig --config rig_bench --dry-run             # real bus, every motor command suppressed
ct-rig --config rig_sim --set procedure.advance.increment_mm=1.0
```

`--dry-run` is the first thing to run against real hardware: it exercises the bus, the
codecs, the geometry and the whole state machine without anything being able to move, so a
sign error in `rig.geometry` shows up as a log line rather than as motion.

Writes `controller.jsonl` (one record per tick), `rig_summary.json`, and with `--plot`:
`procedure.png`, `gate.png`, `sensing.png`.

### `ct-phantom` and `ct-compare` — how good is the sensing?

The phantom runs as a separate process on its own bus, so the controller cannot see what
it was commanded to do. Comparing the two logs afterwards is therefore a real measurement
rather than a circular one.

```bash
ct-phantom --config rig_bench --duration 120           # drive the phantom from a waveform
ct-phantom --config rig_bench --replay recordings/subject01.csv
ct-compare outputs/rig_bench/phantom.jsonl outputs/rig_bench/controller.jsonl
```

`ct-compare` reports sensing RMSE, bias, amplitude ratio, and the cross-correlation lag.
**That lag is `latency.tau_s`** — the sensor-latency term of the forecast horizon,
measured rather than assumed. Both processes must be live at the same time on the same
host; alignment is by `time.monotonic()`.

## Configs

| Config | What it is for |
|---|---|
| `sinusoid` | sanity check — the generator matches the estimator's model exactly |
| `sinusoid_ramp` | frequency tracking, with a drifting fundamental |
| `lujan_n2` | asymmetric breathing, `K=2` |
| `lujan_n3` | three true harmonics, but the 95% rule correctly picks `K=2` |
| `rc_piecewise` | Singh et al. 2020 chest-wall model at the settled 95% threshold |
| `rc_piecewise_k4` | the same signal at 99.9%, which is what actually tracks it |
| `rig_sim` | the whole rig, simulated — loopback buses, simulated plant, simulated clock |
| `rig_bench` | the real rig. **This is the file you edit as measurements come in.** |

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
sources (`@register_source`), identifiers (`@register_identifier`), and — one layer down —
buses, motor codecs, sensors and procedure states (`@register_bus`, `@register_codec`,
`@register_sensor`, `@register_state`).

## Repo layout

```
src/ct/
  layout.py           state ordering — the single source of truth
  geometry.py         rig frames and counts <-> mm — the other single source of truth
  unknowns.py         what we still need to measure, as data
  types.py            SignalBatch, IdentificationResult, TrackerStep
  interfaces.py       the three estimator Protocols
  registry.py         name -> class resolution, for every boundary
  config.py           YAML + CLI overrides
  run.py              estimator orchestration
  rig.py              rig orchestration; the only bridge between controller and plant
  forecast.py         time-advance to t+h
  sources/            sinusoid, lujan, rc_piecewise, csv
  identification/     spectral, harmonic_ls, noise, fft_identifier
  tracking/           measurement (+ analytic Jacobians), harmonic_ekf
  hw/                 CAN buses, CubeMars codecs, axes, sensors
  rt/                 clock, control loop, latency budget, telemetry
  control/            procedure states, servo, firing gate, safety monitor
  plant/              the simulated rig — off-limits to control/, hw/ and rt/
  phantom/            phantom driver and log comparison
  diagnostics/        metrics, plots, rig_plots
  cli/                one module per ct-* script
configs/              experiment and rig definitions
tests/                294 tests
docs/                 architecture, unknowns, session log, reference material
```

### The hard rules

Four, each enforced by a test in `tests/test_boundaries.py`:

1. `layout.py` is the only place the EKF state ordering is encoded.
2. `geometry.py` is the only place raw motor units become millimetres.
3. Nothing in `control/`, `hw/` or `rt/` may import `plant/` — a controller that can see
   simulation ground truth proves nothing when you run it in simulation.
4. The control tick never blocks on I/O; CAN receive runs on its own thread.

## Documentation

- [CLAUDE.md](CLAUDE.md) — settled decisions, conventions, session workflow
- [docs/unknowns.md](docs/unknowns.md) — **the list of physical constants still to measure**
- [docs/architecture.md](docs/architecture.md) — how the pieces fit and why
- [docs/sessions/](docs/sessions/) — what happened each working session
- [docs/reference/fft_ekf_implementation_context.md](docs/reference/fft_ekf_implementation_context.md) — the source design document
