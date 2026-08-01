# Architecture

How the pieces fit, and why they were separated where they were.

## The shape of the problem

The robot must act at a specific point in the breathing cycle, but by the time it has
sensed, computed, moved and inserted, the cycle has moved on. So the estimator's real
output is not "where is the chest now" but "where will it be at `t + h`", where

```
h = tau_s + tau_c + tau_cl(omega_r) + T_ins
```

is supplied from outside. Everything below exists to make that one evaluation trustworthy
and to attach an honest uncertainty to it.

## Data flow

```
              configs/*.yaml + CLI overrides
                          |
                     RunConfig                        (config.py)
                          |
        +-----------------+------------------+
        |                                    |
   SignalSource                              |                (sources/)
   sinusoid | lujan | rc_piecewise | csv     |
        |                                    |
   SignalBatch(t, y, fs, y_clean?, truth?)   |                (types.py)
        |                                    |
        +--> calibration window ---> Identifier                (identification/)
        |                                |
        |                    IdentificationResult(K, s0, P0, Q, R, t0, Ts)
        |                                |
        +--> tracking window ------> Tracker  <-----------------+  (tracking/)
                                         |
                                  TrackerStep(s, P, y_pred, innovation, S, nis)
                                         |
                            +------------+-------------+
                            |                          |
                     forecast(h)                  diagnostics       (forecast.py, diagnostics/)
                            |                          |
                  value at t+h, variance        metrics + figures
```

`run.py` wires this together; the `ct-*` scripts in `cli/` are thin wrappers over it, so a
result from the command line is reproducible in three lines from a notebook or a test.

## Why these three boundaries

The requirement was that each of the three could be replaced without disturbing the other
two. That maps onto the three things genuinely expected to change:

- **The signal source changes first and most often.** Real sensor data does not exist yet.
  Until it does, everything is validated against synthetic generators — and when the real
  data arrives it must slot in without touching a line of estimator code. Hence `CSVSource`
  exists now, tested, rather than being added later.
- **The tracker is the most likely algorithmic swap.** A UKF, particle filter or IMM are all
  plausible successors — the RC-piecewise model in particular is a genuine two-mode hybrid
  system that an IMM would suit. The `Tracker` protocol is shaped so those fit.
- **The identifier is the most likely methodological swap**, since Stage 1 is where the
  arbitrary choices live (the 95% threshold, the frequency estimator, the `Q` recipe).

They are `typing.Protocol`, not abstract base classes: a replacement needs matching methods
and nothing more — no inheritance, no import of this package's base classes. `registry.py`
maps names to classes so a config string is all it takes to select one.

## The state layout is load-bearing

```
s = [a0, A_1, phi_1, ..., A_K, phi_K, theta, omega_r]      n = 2K + 3
```

Protocol boundaries make the *interfaces* swappable, but every stage still has to agree on
what slot 5 of the state vector means. If the identifier writes `phi_2` where the tracker
reads `A_3`, nothing raises — the numbers just quietly become wrong.

So `StateLayout` owns the ordering and nothing else may hard-code an index. It is the
smallest piece of the codebase and the one most worth protecting; `tests/test_layout.py`
checks that the index sets form an exact permutation of `range(n)` for every `K`.

## Stage 1 — identification

Four steps, each in its own module so any one can be replaced:

1. **`spectral.py`** — coarse `omega_r`. Two independent estimators: an FFT peak with
   parabolic sub-bin interpolation, and an autocorrelation peak. They fail differently (FFT
   leaks and can lock onto a strong second harmonic; autocorrelation is sensitive to
   baseline drift), so disagreement beyond a tolerance is warned about. That warning is a
   cheap early signal that a recording is not clean periodic breathing.
2. **`harmonic_ls.py`** — with `omega` fixed, the model is *linear* in its coefficients, so
   this is ordinary least squares, not an optimisation. That is the whole reason Stage 1 is
   cheap and has a closed-form covariance.
3. **`select_K`** — Parseval: harmonic energy goes as `A_k^2`, so the smallest `K` capturing
   95% of `sum A_k^2`. DC is excluded; it carries no shape information.
4. **`noise.py`** — `R` and `Q`.

### Why `Q` is measured rather than tuned

The process model claims every state except `theta` persists. That is a convenient lie:
amplitudes and phases genuinely drift breath to breath. `Q` is where the lie is paid for,
so it is measured directly — refit each breath on its own at the same `omega` and `t_ref`
(so the fitted phases are comparable), take the spread *across* breaths, and scale from the
breath timescale to the sample timescale by `Ts / T_breath`.

Frequency is the exception: a single breath does not resolve `omega` well enough for its
scatter to mean anything, so frequency drift is measured over overlapping four-breath
windows instead and attributed to the same timescale.

Amplitude-, phase- and frequency-like states get separate values. There is no single scalar
that is simultaneously right for a 10 mm amplitude and a 0.01 rad phase.

## Stage 2 — tracking

A textbook EKF, with three details that matter:

- **The measurement is scalar.** `S = H P Hᵀ + R` is a float, so the gain is a division. No
  `solve`, no inverting an innovation covariance, no conditioning worry from that step.
- **Jacobians are analytic and hard-coded.** This runs at sensor rate; finite differencing
  would cost `K+3` extra model evaluations per step for a strictly worse derivative.
  `tests/test_jacobians.py` finite-differences them — that is the only place in the project
  that does.
- **Joseph-form covariance update.** Algebraically identical to the short form, but stays
  symmetric positive-definite over the ~10⁴-step runs these configs produce.

Phases wrap to `(-pi, pi]` after each update. Amplitude and frequency constraints exist but
are **off by default**: a filter that needs clamping to stay sane is telling you something
about `Q`, and that should be visible rather than hidden.

## Forecasting

The entire operation is `theta -> theta + omega_r*h`, then evaluate `h(s)`. Harmonic `k`
picks up `k*omega_r*h` for free, because its argument is `k*theta + phi_k`.

The tempting wrong version — compute one angle `omega_r*h` and add it to every `phi_k` —
advances the fundamental correctly and under-rotates harmonic `k` by a factor of `k`. It
distorts waveform *shape* rather than shifting it in *time*, and it is invisible on a pure
sinusoid, which is exactly how it survives casual testing. `naive_common_phase_forecast`
implements it deliberately so `tests/test_forecast.py` can assert it is wrong, sweeping a
full cycle of starting phases because at isolated phases the two agree by accident.

## Diagnostics

The filter's own consistency check is the NIS: for a correctly-specified filter the
normalised innovation squared is chi-square with one degree of freedom. Mean well above 1
means `Q` or `R` is too small, or the model is wrong; well below means they are too large
and the filter is ignoring data it should use. Both failure modes have been observed in
this repo and are documented in `CLAUDE.md`.

Innovation whiteness (autocorrelation plus Ljung-Box) catches what NIS alone does not: a
filter can have the right *scale* of residual while leaving obvious structure in it. That
is what exposed the RC-piecewise under-fitting at `K=1`.

## What is deliberately not here

The needle servo, the gating logic and the safety monitor. They consume `(s_hat, P)` and
supply `tau_cl` into `h`, but they are separate loops with separate design work. Keeping
them out is what lets this package be validated purely against synthetic ground truth.
