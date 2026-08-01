# Context: FFT + EKF respiratory signal estimator — implementation planning

> **Archived source document.** This is the design brief this repo was built from,
> preserved as given. Two coefficient transcriptions in the RC-piecewise section were
> found to be typos and are corrected in the implementation — see the note at the end of
> this file and `src/ct/sources/rc_piecewise.py`. Where this document and the code
> disagree elsewhere, the code plus `CLAUDE.md` is authoritative, and the difference
> should be recorded in a session doc.

## How to use this document
Upload this at the start of a new chat to continue directly into **implementation design**
for the FFT-identification + EKF-tracking pipeline described below. The prior chat worked
through the math and terminology in depth; this document carries forward only the
conclusions needed to design and build the thing. Do not re-derive what's below — it's
settled. The new chat's job is architecture, interfaces, and a Claude Code task breakdown.

## Project (carried over)
Controller for a needle-insertion robot that must fire/insert during specific points of a
breathing cycle. Sensor gives a 1-D position signal tracking the breathing cycle. This
document covers specifically the **signal estimation pipeline**: identifying and then
tracking a parametric model of the breathing signal, so its future value can be forecast.
The needle-servo (lead compensator) side and the gating/safety-monitor side were explored
in separate earlier threads and are NOT part of this implementation — see "Explicitly
deferred" at the bottom.

## Corrections established from the original framing (do not reintroduce these errors)
1. **Feedforward vs feedback are separate loops.** Breathing prediction (this pipeline) is
   open-loop disturbance forecasting. The needle servo is a separate closed loop. They were
   originally conflated; keep them conceptually and architecturally separate.
2. **tau_a is not a term in the prediction horizon.** The raw actuator delay from the
   original doc is replaced by `tau_cl(omega_r)` — the *residual* closed-loop tracking lag
   of the (lead-compensated) needle servo. It's a servo design output, not a fixed physical
   constant, and it does not belong to this pipeline's scope.
3. **Horizon:** `h = tau_s + tau_c + tau_cl(omega_r) + T_ins`. All four terms are consumed
   by one forecasting operation (below); none is "solved" individually. This pipeline's job
   is to be able to evaluate the model at `t + h` for whatever `h` the rest of the system
   supplies.
4. **"Prediction" means time-advance, not a scalar phase shift.** Advancing the harmonic
   model by `h` means `theta -> theta + omega_r*h`; every harmonic `k` then automatically
   gets `k*omega_r*h` of phase rotation. Do NOT compute one delay-derived angle and rotate
   every harmonic's phase by the same amount — that only correctly advances the fundamental
   and leaves higher harmonics under/over-rotated, distorting the waveform shape.

## The FFT + EKF pipeline (what gets implemented)
Two stages: **identify** (batch, offline/periodic) then **track** (recursive, online).

### Stage 1 — batch/FFT identification
Purpose: fix K (number of harmonics), produce an initial state + covariance for the EKF,
and estimate R and Q.

1. Coarse `omega_r_hat`: FFT or autocorrelation peak on a calibration recording (aim for
   ~15-30 breaths; long enough to average noise and to split into separate breaths later,
   short enough that breathing rate/depth stationarity is a reasonable assumption).
2. Least-squares harmonic regression at fixed `omega_r_hat` — linear in the coefficients,
   since `sin(k*omega_r_hat*t)` and `cos(k*omega_r_hat*t)` are known numbers once
   `omega_r_hat` and `t` are fixed:
   ```
   x(t) ~= a0 + sum_{k=1}^{Kmax} [alpha_k*sin(k*omega_r_hat*t) + beta_k*cos(k*omega_r_hat*t)]
   ```
   Solve via ordinary least squares. Also keep: residual variance `sigma_hat^2` and
   coefficient covariance `Cov(beta_hat) = sigma_hat^2 * (X^T X)^-1`, where `X` is the
   sin/cos design matrix — needed later for `R` and `P0`.
3. Convert to amplitude/phase: `A_k = sqrt(alpha_k^2 + beta_k^2)`,
   `phi_k = atan2(beta_k, alpha_k)`.
4. Select `K`: smallest `K` such that
   `sum_{k=1}^{K} A_k^2 / sum_{k=1}^{Kmax} A_k^2 >= 0.95` (Parseval-based energy fraction).
5. Validate `K` against synthetic ground-truth waveforms (see "Reference models" below)
   before trusting it on real data — do this for both reference models, not just one.

### Stage 2 — EKF tracking
State vector, dimension `n = 2K + 3`:
```
s = [a0, A_1, phi_1, A_2, phi_2, ..., A_K, phi_K, theta, omega_r]
```

Process model `f` — one kinematic row, everything else a random walk:
```
theta_k  = theta_{k-1} + omega_r * Ts     <- exact, from the definition of theta/omega_r
s_k(i)   = s_{k-1}(i)  for all other i    <- persistence assumption; Q carries the uncertainty
```

Measurement model `h` — one scalar output (sensor is 1-D):
```
h(s) = a0 + sum_{k=1}^{K} A_k * sin(k*theta + phi_k)
```

Jacobians (derive once analytically, hard-code — do not use numeric differencing for this):
```
dh/da0      = 1
dh/dA_k     = sin(k*theta + phi_k)
dh/dphi_k   = A_k * cos(k*theta + phi_k)
dh/dtheta   = sum_k [ k * A_k * cos(k*theta + phi_k) ]
F: identity everywhere, except the (theta row, omega_r column) entry = Ts
```

Standard predict/update recursion (5 equations). Because the measurement is scalar,
`R` and `S_k` are plain numbers — the "Kalman gain" step is a division, not a matrix
inversion. Keep the implementation aware of this; it simplifies the linear algebra
noticeably.

## Constructing the design-time quantities

| Quantity | Method |
|---|---|
| `K` | 95% energy criterion (Stage 1, step 4); cross-check against both reference models below |
| `s0` | `a0, A_k, phi_k` straight from Stage 1 regression; `omega_r` from Stage 1 step 1; `theta` = fitted phase at the *end* of the calibration window, so tracking picks up where identification left off |
| `P0` | `a0/A_k/phi_k` block: from OLS covariance `Cov(beta_hat)`, converted rectangular→amplitude/phase via a first-order (delta-method) Jacobian if needed. `omega_r`: ~ frequency-search resolution, `2*pi/(N*Ts)`. `theta`: similar scale to the phase terms. Inflate everything modestly as a model-mismatch safety margin. |
| `R` | Prefer: sample variance of sensor readings during a still/breath-hold segment. Fallback (treat as an upper bound only): Stage-1 residual variance. |
| `Q` (per-state) | Split the calibration recording into individual breaths, refit each *separately* with the same regression, take the variance of each state's fitted value *across* those per-breath fits, then scale from breath-timescale down to per-sample-timescale: `Q_i ~= var_breath_to_breath(i) * (Ts / T_breath)`. Amplitude-like and phase-like states typically need different `Q` values — don't use one global scalar. |

## Terminology for the paper (established, literature-checked)
- Overall description: *"an extended Kalman filter matched to an adaptive multi-harmonic
  (Fourier) model with a time-varying fundamental frequency."*
- Nearest biomedical prior art (LMS-adaptive, not Kalman): **Weighted-Frequency Fourier
  Linear Combiner (WFLC)**. Directly on-topic origin paper: Riviere, Thakral, Iordachita,
  Mitroi, Stoianovici, *"Predicting respiratory motion for active canceling during
  percutaneous needle insertion,"* IEEE EMBS 2001. Cite/position against this — choosing
  EKF over their LMS adaptation is a real, statable methodological difference (EKF gives an
  explicit covariance signal usable directly by the gate; LMS doesn't).
- Nearest exact-math prior art (EKF-based, different domain — power systems): search terms
  *"extended Kalman filter frequency tracking,"* *"dynamic phasor estimation,"* *"Kalman
  filter harmonic estimation."*

## Reference models for synthetic testing
Used to generate synthetic "ground truth" `x(t)` for validating `K` selection, EKF
convergence, and robustness. Kept separate from the general-purpose harmonic EKF, which
stays the *primary* method — it degrades gracefully when real breathing doesn't match any
closed-form family exactly; these models don't.

**1. Lujan model** — `x(t) = b - a*cos^{2n}(pi*t/T - phi)`. Single smooth closed form.
`n=1` → pure sinusoid; `n=2,3` add asymmetry. Already used to validate the 95%-energy `K`
rule: `n=1 -> K=1`; `n=2 -> K=2`; `n=3 -> K=2` despite 3 "true" harmonics being present,
since the 3rd carries under 0.4% of the energy. If ever baked directly into an EKF: fix `n`
**offline** (its measurement-Jacobian has a `log(cos(psi))` term that blows up once per
cycle if tracked online) and track only `[b, a, psi, omega_r]` (4 states) live.

**2. RC-circuit piecewise chest-wall model** — Singh, Rehman, Yongchareon, Chong,
*"Modelling of Chest Wall Motion for Cardiorespiratory Activity for Radar-Based NCVS
Systems,"* Sensors 2020, 20(18), 5094. Two-phase (inhale/exhale) piecewise solution of a
first-order RC-type respiratory-mechanics ODE, driven by a quadratic isometric pressure
pulse:
```
Inhale (0 <= t <= t1):
  V(t) = (tau_rs/R_rs) * [A1*t^2 + A2*t + A3*(1 - e^(-t/tau_rs))] + V0*e^(-t/tau_rs)

Exhale (t1 <= t <= t1+t2):
  V(t) = [P(t1) / (R_rs*(1/tau_rs - 1/tau))] * [e^((t-t1)/tau) - e^(-(t-t1)/tau_rs)]
         + V(t1)*e^(-(t-t1)/tau_rs)

where: tau_rs = R_rs*C_rs,  A1 = a2,  A2 = a2 - 2*a2*tau_rs,
       A3 = a0 - a1*tau_rs + 2*a2*tau_rs^2
Chest wall displacement x(t) is V(t), scaled (volume <-> displacement proportionality).
```
Parameters: `a0,a1,a2` (isometric pressure pulse shape), `R_rs`, `E_rs` (or `tau_rs`),
`tau` (separate exhale decay), `t1`, `t2` (phase durations) — ~7-8 total. Empirically beat a
plain sinusoid on real supine chest-belt data (correlation ~0.86-0.94 vs ~0.65-0.8; DTW
distance near 0 vs 2-15). Possible mild non-smoothness at the inhale/exhale transition —
re-validate the `K`-selection threshold against this model specifically, since slower
Fourier-coefficient decay from a kink could push the required `K` higher than what Lujan
alone suggests. **Not** a good candidate to bake directly into the online EKF — it's a
genuine two-mode/hybrid system (different equation per phase), which would need a
switched/IMM filter, a real scope increase rather than a state addition. Also includes a
separate, small-amplitude (~0.2-0.5mm vs 3-12mm respiration) cardiac component (Van der Pol
oscillator) — likely near the noise floor, but check it doesn't leak spurious high-frequency
content into the harmonic fit.

**Use both purely as data generators behind a common interface** (below), not as
competitors to the general harmonic EKF.

## New requirement driving the next chat: modular, swappable implementation
Build the pipeline as separable stages behind clean interfaces, so that:
- The **identification** method (Stage 1 above) can be swapped for an alternative later
  without touching the tracker.
- The **tracking** method (Stage 2 above) can be swapped for an alternative (UKF, particle
  filter, IMM, etc.) later without touching identification or the data source.
- The **signal source** can be swapped freely between: pure sinusoid → Lujan → RC-piecewise
  chest-wall model → real sensor data (once available), all behind one common interface, so
  none of the estimator code changes when the data source changes.

Concretely, at least three interface boundaries:
1. **signal source / data provider** — produces a stream (or batch) of `(t, y)` samples
2. **identifier** — consumes a calibration batch, produces `(K, s0, P0, Q, R)`
3. **tracker** — consumes `(K, s0, P0, Q, R)` plus the live sample stream, produces
   `s_hat_k, P_k` at every step

## Open questions (carried over, still unresolved)
- Sensor modality and its real measured latency (optical ~10-30ms vs ultrasound
  ~50-150ms, etc.)
- Target organ / expected motion amplitude
- Clinical tolerance epsilon
- Insertion depth / achievable needle velocity (sets `T_ins`)
- Breath-hold viability as a fallback
- Real sensor data: format, sample rate, availability timeline (teammates still sourcing)
- Exact chest-wall displacement formula the research team intends — likely the Singh et al.
  2020 RC-piecewise model above (matches the paper link/citation anchor already provided),
  but confirm once teammates send their reference directly

## What the next chat should do
Design details for a concrete implementation, in preparation for Claude Code sessions:
- Choose language/environment (Python + numpy/scipy is the natural default given the
  linear-algebra-heavy design — confirm or override)
- Concretely define the three interface boundaries above (function/class signatures)
- Plan the repo/folder structure and a synthetic-data test harness (sinusoid, Lujan,
  RC-piecewise generators behind the common interface)
- Define diagnostic/plotting needs (state trajectories, innovation/residual monitor,
  covariance bounds) for validating the filter once built
- Produce a task breakdown suitable for handing to Claude Code

## Explicitly deferred (separate future chats)
- Lead-compensator / needle-servo design (already explored in an earlier chat; not part of
  this implementation)
- Gating logic implementation
- Safety/residual monitor implementation

These consume the EKF's output (`s_hat`, `P`) but are not part of the estimator itself.

---

## Corrections applied during implementation (session 001)

The RC-piecewise formulas above contain two transcription errors, both found by
substituting the stated solution back into `V' + V/tau_rs = P/R_rs` and matching terms:

1. **`A2 = a2 - 2*a2*tau_rs` should be `A2 = a1 - 2*a2*tau_rs`.** The linear pressure
   coefficient `a1` must appear; `A3` in the same block already uses `a1` consistently
   with the corrected form.
2. **The exhale term `e^{+(t-t1)/tau}` should be `e^{-(t-t1)/tau}`.** A growing exponential
   would make exhalation diverge rather than relax.

With both corrections the inhale and exhale solutions are continuous at `t1` and the
particular solutions satisfy the ODE exactly. See `_inhale_coeffs` in
`src/ct/sources/rc_piecewise.py` and `test_rc_inhale_coefficients_solve_the_ode` in
`tests/test_sources.py`.
