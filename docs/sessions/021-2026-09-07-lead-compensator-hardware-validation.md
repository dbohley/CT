# 021 — 2026-09-07 — lead compensator hardware validation

## Goal

Session 019 measured the needle's plant and designed a lead compensator against it, validated
only in simulation. This session built a live hardware A/B test (compensated vs. uncompensated
tracking) to validate it for real, expecting a quick confirmation. Instead it surfaced a real
production-code bug, a genuine control-loop failure mode, and a false lead — before landing on
the actual cause and a hardware-confirmed result. The goal shifted from "confirm it works" to
"find out why it doesn't, and fix whichever of {bench script, compensator design, test
condition} is actually wrong" — all three turned out to be involved, in that order.

## What changed

**Two new bench scripts** (`scripts/run_needle_lead_tracking_live.py`,
`scripts/plot_needle_lead_tracking_live.py`) drive the real needle motor through a raised-cosine
reference twice — once commanded directly, once through `ct.control.servo.LeadServo` — and
report RMSE/amplitude-ratio/lag for both, the hardware counterpart of session 019's
`simulate_needle_lead_tracking.py`.

**The first real run overshot badly.** Root cause: `LeadServo.update()` was being called on
every ~1ms pass of the script's polling loop, not once per command tick — a filter built for
`Ts=1/command_hz` (50ms) was being driven ~50x faster than its own discretization assumed. Fixed
by gating the call to the same `elapsed >= next_command_at` block that sends the command.

**Fixing that exposed a real bug in production code, not just the bench script.**
`InsertState._hold_standoff()` was calling `ctx.servo.update(error, limit=ctx.geometry.needle
.v_max_mm_s)` — `v_max_mm_s` is a genuine, correctly-used velocity ceiling (mm/s) everywhere
else, reused here as a position-magnitude clamp (mm) on the correction. Two incompatible units,
same number. This led to a small production-code project of its own: `LeadServo.update()` gained
a genuine rate limit (separate from the existing magnitude clamp), both now default from two new
`AxisServoConfig` fields (`correction_limit_mm`, `correction_rate_limit_mm_s`) instead of being
passed in ad hoc per call site, and the compensator was extended from `_hold_standoff()` alone
into `InsertState._drive()` and `AdvanceState`'s increments (the user's call: the compensator's
job is general tracking fidelity, not standoff-specific), with a servo reset at each new step so
filter state doesn't leak across unrelated moves.

**Re-running with both fixes still oscillated.** A dead end followed: hypothesized the
compensator's ~12.7x high-frequency gain was amplifying real position-*sensor noise*, built
`scripts/measure_needle_position_noise.py` to measure it directly, and it came back tiny
(0.005mm at rest) — nowhere near large enough to explain multi-millimetre corrections. The
actual driver, confirmed from the *uncompensated* phase of the same run, was real dynamic
tracking lag (0.139mm std, 0.261mm max) from the axis chasing a position reference that only
updated 20 times a second — an artifact of the bench script's `--command-hz 20` default
(inherited from an earlier, unrelated script), not of sensor noise or the compensator's design.
`design_needle_lead.py` gained a new, permanent capability from chasing this lead: a
`--max-lag-std-mm` / `--max-projected-correction-mm` constraint that rejects any gain whose
worst-case amplification of a measured real-error number would exceed a cap — but the resulting
lower-gain design (gain=3.58 vs. 12.74) was never adopted, because the next experiment made it
unnecessary.

**Testing at 100Hz, then 200Hz — the real production loop rate — resolved it.** At 100Hz the
oscillation was already far smaller; at 200Hz (`configs/rig_bench.yaml`'s actual
`procedure.loop_rate_hz`), the *original*, unmodified compensator (gain=12.74) gave a real,
modest RMSE improvement over no compensation at all, with zero saturation. The bench script had
been testing the compensator at roughly 1/10th its real operating rate the whole time.

## Files touched

| File | Change |
|---|---|
| `scripts/run_needle_lead_tracking_live.py` | new — live compensated/uncompensated A/B test; fixed the Ts-gating bug; added `--correction-rate-limit-mm-s` |
| `scripts/plot_needle_lead_tracking_live.py` | new — scores both phases, prints measured-vs-predicted comparison |
| `scripts/measure_needle_position_noise.py` | new — at-rest position noise floor + projected correction-noise estimate |
| `scripts/design_needle_lead.py` | added the noise/lag-amplification constraint (`--max-lag-std-mm`, `--max-projected-correction-mm`) |
| `src/ct/control/servo.py` | `LeadServo.update()` gained `rate_limit_mm_s`, chained after the existing magnitude clamp; new `rate_limited` counter |
| `src/ct/hw/config.py` | `AxisServoConfig` gained `correction_limit_mm`, `correction_rate_limit_mm_s` |
| `src/ct/control/states/insert.py` | fixed the `v_max_mm_s` units bug in `_hold_standoff()`; routed `_drive()` through the servo; reset on firing |
| `src/ct/control/states/advance.py` | routed increments through the servo; reset on `enter()` and per increment |
| `configs/rig_bench.yaml`, `configs/rig_sim.yaml` | added `servo.needle.correction_limit_mm` (2.0), `correction_rate_limit_mm_s` (10.0) as placeholders |
| `src/ct/unknowns.py` | two new `Unknown` entries for the above, blocking INSERT and ADVANCE |
| `tests/test_control.py` | 4 new tests: rate-limit ramping, backward compatibility, config-driven defaults, validation |

## Decisions and rationale

- **Extended the compensator to step moves (ADVANCE, INSERT's post-fire drive), not just
  standoff-hold.** The user's correction: the compensator's job is to make the needle track
  *whatever it's commanded*, not specifically the breathing-rate forecast. Considered leaving
  step moves as plain velocity-limited `move_to()` calls (simpler, zero new risk) but the user
  chose the broader extension explicitly, after being walked through the real distinction
  between the existing velocity limit (bounds the motor's approach speed) and the new rate limit
  (bounds how fast the correction *itself* may change) — these are genuinely different
  quantities, not a duplicate safety mechanism.
- **Rejected the "sensor noise" theory before building on it.** Rather than assume the at-rest
  noise measurement explained the oscillation, checked it against the uncompensated phase's real
  tracking-error std (0.139mm) — 30x larger than the noise floor (0.005mm) — and concluded the
  noise theory was wrong. Worth recording as a methodology point: a plausible-sounding
  explanation that produces a number in the wrong ballpark should be abandoned, not rationalized.
- **Did not adopt the lower-gain compensator design (gain=3.58).** It was a well-reasoned
  response to the evidence available at the time (0.139mm real lag at 20Hz), but the 100Hz and
  200Hz results showed the real lag driving that number was itself an artifact of testing at an
  unrepresentative command rate. `design_needle_lead.py`'s new constraint is kept — it's the
  right tool if a future measurement at the real operating rate ever calls for it — but nothing
  in config was changed on the strength of a bench condition that didn't match production.
- **Left `configs/rig_bench.yaml`'s `correction_limit_mm`/`correction_rate_limit_mm_s` as
  placeholders**, despite the 200Hz run validating gain=12.74 with `--correction-limit-mm 2.0
  --correction-rate-limit-mm-s 5.0` working well. One condition (3mm, 0.25Hz) is not a
  characterization; also the config's placeholder `correction_rate_limit_mm_s` (10.0) doesn't
  match what was actually validated (5.0) — see Open questions.

## Verification

```bash
conda activate CT
pytest -q
# 326 passed
```

Real hardware, same reference (3mm amplitude, 0.25Hz, 2 cycles) across three command rates,
compensator gain=12.74 (unmodified from session 019) throughout except where noted:

| `--command-hz` | uncompensated RMSE | compensated RMSE | RMSE change | saturated_steps | clamped_commands |
|---|---|---|---|---|---|
| 20 (Ts bug present) | — | — | oscillating, ±2mm corrections | 46 | 343 |
| 20 (Ts fixed) | — | — | still oscillating | 57 | 35 |
| 20 (+ rate limit 5mm/s) | — | — | still oscillating, smaller amplitude | 21 | 11 |
| 100 | 0.0667mm | 0.0762mm | **−14.3%** (compensator hurts, slightly) | 0 | 43 |
| 200 | 0.0502mm | 0.0466mm | **+7.2%** (compensator helps) | 0 | 84 (all benign, at raised-cosine zero-crossings) |

At-rest position noise floor (`measure_needle_position_noise.py`, default 20Hz command rate):
raw std 0.0035mm, per-tick std 0.0048mm, tick-to-tick delta std 0.0075mm, projected correction
noise through gain=12.74: ~0.096mm — confirms this was never the driver.

`design_needle_lead.py --max-lag-std-mm 0.139 --max-projected-correction-mm 0.5`: chosen
zero=1.00, pole=5.00, gain=3.58 (vs. 12.74), phase margin 119.6° (vs. 82.4°), residual_lag
335.5ms (vs. 151.1ms) — computed, not adopted (see Decisions).

## Findings

1. **The lead compensator, as originally designed in session 019 (gain=12.74, unchanged), gives
   a real, measured tracking improvement on real hardware at the rate it was actually meant to
   run at.** 0.0466mm vs. 0.0502mm RMSE at 200Hz, zero saturation. This is the first hardware
   confirmation that this specific compensator design helps rather than just closes a
   theoretical phase-margin requirement.
2. **A compensator's real-world behavior can differ enormously from its own isolated frequency
   response depending on how fast it's actually driven, even when that frequency response
   (checked numerically) is nearly identical at the two rates.** The Tustin-discretized filter's
   low-frequency gain was confirmed nearly identical at Ts=0.05s and Ts=0.005s (both ≈2.5-12.7
   across 0.1-5Hz) — the oscillation was not a discretization-warping artifact of the compensator
   itself. It was the *plant's* real tracking lag at the coarser command rate (0.139mm at 20Hz)
   that the compensator was correctly, if aggressively, reacting to.
3. **Real axis tracking lag scales strongly and non-obviously with command rate**: 0.139mm std
   at 20Hz, down to an RMSE of 0.0667mm at 100Hz and 0.0502mm at 200Hz, for the exact same
   reference. A bench test's chosen command rate is not a neutral parameter — it can dominate
   the result being measured.
4. **A units bug (`v_max_mm_s` reused as a position-magnitude clamp) had been live in
   production `_hold_standoff()` code since session 019/session-002-era design**, allowing the
   correction up to ±60mm in principle. Never triggered in prior simulation-only testing because
   simulation has no real measurement lag of the kind that exposed it here.
5. **A magnitude-only clamp on a compensator's output is not sufficient protection against a
   sudden real error (e.g., a step's initial error, or coarse-rate tracking lag); a genuine
   rate limit on the correction itself is a materially different and necessary safeguard.**
   Confirmed both in the failing 20Hz runs (rate limit alone reduced but did not eliminate the
   oscillation — the underlying real lag was still too large for this gain) and in the new
   `LeadServo` unit tests.

## Open questions

- **`correction_rate_limit_mm_s` config placeholder (10.0) does not match the value actually
  validated on hardware (5.0).** Needs reconciling — likely by re-testing at 5.0 explicitly (or
  a small sweep) before treating either number as anything but a placeholder.
- **The step-move extension (ADVANCE increments, INSERT's `_drive()`) has not been tested on
  real hardware at all** — only via the full `pytest` suite in simulation. The live A/B test
  validated `_hold_standoff()`'s continuous-tracking case; a `ct-rig` run through a real
  INSERT→ADVANCE sequence is the next real-hardware check for the step-move case specifically.
- **Only one amplitude/frequency condition (3mm, 0.25Hz) has been tested at 200Hz.** Before
  calling gain=12.74 validated in general, worth checking at least one more amplitude (closer to
  session 006's real breathing amplitudes, 0.5-1.4mm RMS) and frequency.
- **Whether `configs/rig_bench.yaml`'s `correction_limit_mm`/`correction_rate_limit_mm_s`
  should move from PLACEHOLDER to measured in `src/ct/unknowns.py`** once the above two points
  are resolved.

## Next steps

1. Reconcile the `correction_rate_limit_mm_s` config/bench-default mismatch (5.0 vs. 10.0) with
   a deliberate test, not by picking one arbitrarily.
2. Run the live A/B test at 200Hz with at least one more amplitude/frequency pair representative
   of real breathing, to move past a single validated condition.
3. Validate the ADVANCE/`_drive()` step-move extension on real hardware via `ct-rig`
   (`--simulated` first, then real), watching specifically for oscillation near arrival — the
   same failure mode this session found, in a regime (fixed target, large initial error) that
   could plausibly trigger it differently than continuous tracking did.
4. Once 1-3 are done, promote `correction_limit_mm`/`correction_rate_limit_mm_s` from PLACEHOLDER
   to measured in `configs/rig_bench.yaml` and `src/ct/unknowns.py`.
