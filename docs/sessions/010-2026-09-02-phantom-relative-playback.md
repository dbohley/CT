# 010 — 2026-09-02 — phantom-relative-playback

## Goal

Make the phantom breathe wherever it is parked, instead of travelling to the motor's absolute
origin first. The bench had to be set up with the phantom motor already near absolute zero,
because every profile was played in the motor's absolute frame — park it anywhere else and it
would traverse across to that fixed spot before playback began.

## What changed

**Playback is now referenced to the motor's own current position.** `run_breathing_profile.py`
reads the status broadcast as before, but instead of ramping to absolute zero it commands
`initial_rad + profile_offset` throughout. The ramp-in shrinks from "however far the motor was
from absolute zero" to the profile's first sample relative to its mean — 0.5–1.1 mm.

**The datum is the profile's mean, not its first sample.** `load_profile` now subtracts
`positions_arr.mean()` rather than `positions_arr[0]`. Every recorded profile starts near its
top, so first-sample referencing would have left the phantom sitting 0.5–1.1 mm *behind* the
parked position on average. Session 009 found a shift of exactly that magnitude consumed the
base's entire contact margin, so this is a load-bearing choice rather than a cosmetic one.

**A failed position read is now fatal.** It used to warn and assume 0 rad — which traverses to
the absolute origin, i.e. precisely the behaviour this change removes, at the one moment the
operator has been told it will not happen. Session 009 measured the broadcast at 51.1 Hz on
every tick of a real run, so silence is a genuine fault (motor unpowered, wrong bus). The
motor is disabled on the way out.

**Log frame stays run-relative.** `commanded_mm` and `measured_mm` are both reported relative
to `initial_rad`, so `error_mm` remains a real tracking error, logs stay comparable across
runs that started at different physical positions, and `compare_logs` is unaffected (it
mean-removes both series anyway). `summary.json` gained `reference_position_rad` recording
where the run was centred.

**The parent script can now see the phantom die.** Refusing to run created a new way for the
subprocess to exit early that `run_approach_and_seat.py` could not detect: its startup check
was 2.0 s, but a refusal lands at roughly 3.5–4.5 s (interpreter start, loading a 74k-row
profile, opening the bus, `enable` + 0.5 s, then the 2.0 s position read). The check is now
6.0 s, and — more importantly — `phantom_proc.poll()` is checked every tick of the control
loop. Before this, a phantom that died mid-run went entirely unnoticed: the base kept seating
and standing off against a stationary surface and the run looked successful while measuring
nothing.

## Files touched

| File | Change |
|---|---|
| `scripts/run_breathing_profile.py` | Mean-rebase in `load_profile`; `target_rad = initial_rad + profile_rad`; refuse on a failed position read; run-relative log frame; `reference_position_rad` in the summary; docstring |
| `scripts/run_approach_and_seat.py` | `PHANTOM_STARTUP_CHECK_S` 2.0 → 6.0; per-tick `phantom_proc.poll()` fault |

## Decisions and rationale

**Mean-referenced rather than start-referenced.** Both readings of "start where the motor is
and normalize around that" were defensible, and they differ by the first-sample-to-mean offset
of each profile:

| profile | p2p | start-referenced | mean-referenced | offset |
|---|---|---|---|---|
| breathing_profile_1 | 2.65 | [−2.27, +0.38] | [−1.17, +1.48] | −1.10 |
| emma | 5.01 | [−2.67, +2.35] | [−1.65, +3.36] | −1.02 |
| moira | 7.88 | [−3.01, +4.87] | [−2.06, +5.83] | −0.96 |
| sara | 9.03 | [−3.40, +5.63] | [−2.88, +6.15] | −0.52 |
| derek | 4.21 | [−2.31, +1.90] | [−1.70, +2.52] | −0.62 |

Mean-referencing makes the position you park at the *average* standoff, which is what the
base's approach actually has to reach. Start-referencing costs no ramp-in but puts the
phantom ~1 mm behind where you left it.

**Whole-file mean, with a known limitation.** For a profile longer than the run, the
whole-file mean is not the mean of the slice that plays. Emma is 618 s, a 180 s run completes
zero loops, and her 30 s rolling mean wanders 1.54 mm — so the phantom can still drift that
far off centre. The script cannot know how long it will be left running, so the whole-file
mean is the only well-defined choice available to it. Documented rather than worked around.

**Refuse rather than fall back on a failed read.** See above; the fallback's behaviour is the
exact thing being removed.

## Verification

Offline — the framing change is arithmetic and checkable without hardware:

```bash
conda activate CT
python scripts/run_breathing_profile.py --profile emma_normal_breathing --dry-run
pytest -q
# 313 passed in 26.62s
```

`load_profile` on every profile, after the change:

| profile | mean | p2p | range | ramp-in |
|---|---|---|---|---|
| breathing_profile_1 | +1.7e-14 | 2.65 | [−1.17, +1.48] | +1.10 mm |
| emma | +7.7e-15 | 5.01 | [−1.65, +3.36] | +1.02 mm |
| moira | −2.3e-15 | 7.88 | [−2.06, +5.83] | +0.96 mm |
| sara | −2.8e-15 | 9.03 | [−2.88, +6.15] | +0.52 mm |
| derek | −1.4e-14 | 4.21 | [−1.70, +2.52] | +0.62 mm |

| Check | Expected | Measured |
|---|---|---|
| Profiles mean-centred | mean ≈ 0 | all < 1.4e-14 mm |
| Peak-to-peak unchanged | same as before the change | identical for all five |
| Ramp-in | ~1 mm, not a traverse | 0.52–1.10 mm |
| emma dry-run first sample | −0.0783 rad | −0.0783 rad = 1.02 mm on the 13 mm drum ✓ |
| `--max-travel-mm` clamp | unaffected (peak-to-peak) | moira still refused at 7.88 > 6.0; passes with `--max-travel-mm 8` |
| Early-return path | `finally` cleans up, summary skipped | `return 1` inside the `try`, summary never reached |
| Test suite | 313 | 313 passed |

**Not yet run on hardware.** Everything above is arithmetic and control flow. The hardware
check is the point of the change and has not been done:

```bash
# park the phantom deliberately away from its absolute zero, then:
python scripts/run_breathing_profile.py --profile emma_normal_breathing
```

Expect a ~1 mm ramp rather than a traverse, oscillation about the parked position, a return to
that position at the end, `measured_travel_mm` still ~99 % of `commanded_travel_mm` (was
4.152 / 4.192 in session 009), and `reference_position_rad` in the summary recording where it
was centred. Then a full `run_approach_and_seat.py --profile emma_normal_breathing` from an
off-zero position, which also exercises session 009's auto-extend since the base's required
travel now depends on where the phantom was parked.

## Findings

- **The phantom's playback datum was an absolute motor origin, and nothing required it to
  be.** No test referenced `load_profile`, and `compare_logs` mean-removes both series, so the
  absolute framing bought nothing and cost bench flexibility.
- **A phantom subprocess dying mid-run was invisible to the controller.** Not introduced by
  this change — it has been true since the subprocess launch was added — but this change made
  it reachable in a new way and therefore worth fixing. A run with a dead phantom completes
  normally and measures a stationary surface.

## Open questions

- **Does the ~1.5 mm baseline wander of a long profile matter once playback is mean-centred?**
  Centring fixes the profile-to-profile offset but not the within-profile drift. Whether the
  base's seating survives it is untested — carried from session 009.
- Carried unchanged: in-contact `R`, whether `tau_s = 0.677 s` is a constant, Q as the leading
  suspect for the NIS shortfall, and how much approach travel a seated contact actually needs.

## Next steps

1. Park the phantom off-zero and run `run_breathing_profile.py` alone — confirm the ramp is
   ~1 mm and it centres where parked.
2. Full `run_approach_and_seat.py --profile emma_normal_breathing` from that position, reading
   `approach.travelled_mm` (session 009) and `reference_position_rad` (this session).
3. Deliberately power the phantom down and confirm both the refusal and that the parent
   notices it.
