# 011 — 2026-09-02 — phantom-set-origin

## Goal

Session 010 was supposed to stop the phantom traversing to a fixed spot at startup. On
hardware it still did. This session finds out why the diagnosis was impossible, fixes that,
and removes the dependency that made the traverse possible at all.

## What changed

**Playback no longer depends on reading the motor's position — it commands the origin
instead.** At startup the script now sends `codec.zero()` (`CMD_SET_ORIGIN`, temporary) to
make the motor's present position 0, then commands the profile's offsets literally.
`scripts/test_phantom_motor.py --zero` has used exactly this on this motor since session 004.
`initial_rad` is gone from the motion arithmetic; there is no reference number left to be
wrong.

**The readback is the safety property, stated as a check.** After zeroing, the motor must
report ~0 (within `ZERO_READBACK_TOL_MM`, 1.0 mm). That is precisely what playback depends on:
commanding position 0 must mean *stay put*. If `SET_ORIGIN` silently fails, the motor still
reports its old position and the run refuses instead of ramping to 0 and dragging the phantom
across the bench.

**The observability regression I introduced in session 010 is undone.** Three separate gaps,
all of which had to be closed before this could ever have been diagnosed from files:

- `measured_mm` was made relative to `initial_rad`, so a wrong reference shifted the commanded
  and measured series *together* and a real traverse was invisible. After zeroing, the logged
  and motor frames coincide, so `measured_mm` is now the motor's own reported position with no
  conversion — the ambiguity stops existing rather than needing a second column.
- `summary.json` was built after the `try/finally`, so any abnormal exit skipped it. It is now
  written from the `finally`, including on the refusal path, and carries
  `pre_zero_position_rad`, `zero_readback_rad`, `zero_applied` and `refusal`.
- The phantom subprocess's stdout went to the shared terminal and was lost to scrollback.
  `run_approach_and_seat.py` now captures it to `<out-dir>/phantom/stdout.log`, and echoes it
  inline if the phantom exits before the run starts.

**One warning line in the controller**, no behaviour change: when the first tactile reading
already exceeds the contact threshold, say so.

## Files touched

| File | Change |
|---|---|
| `scripts/run_breathing_profile.py` | `zero()` + readback verification replacing the `initial_rad` arithmetic; summary written from `finally` with the position fields; log frame is the motor's own; docstring |
| `scripts/run_approach_and_seat.py` | Phantom stdout captured to `phantom/stdout.log` and echoed on early exit; warning when tactile already exceeds the contact threshold |

## What I could and could not determine

Only run `20260902-151454` used session 010's code (scripts changed 15:05, run started 15:14).

**Confirmed from the logs:**

- The mean-referencing arithmetic worked: `commanded_mm` starts at **+1.018 mm**, exactly
  emma's first sample about its mean, and `measured_mm` tracks it to ~0.05 mm.
- So whatever happened, happened during the **ramp**, before the first logged sample.
- Pre-change runs prove SET_POS and the status frame share a frame — commanding 0 gives status
  ≈ 0 over ±11° — so this was never a units or scale error.

**Could not determine: whether the phantom traversed, or what `initial_rad` was read as.** The
log could not show it (relative frame), the summary was never written, and the printed line
went to scrollback. A ToF-vs-base-position check across seven runs is inconclusive: the
post-change run sits on the same line as the pre-change ones, equally consistent with no
traverse or with a traverse to absolute zero.

That is the reason the fix is "remove the dependency" rather than "correct the reading" — the
reading still cannot be verified, and now nothing needs it to be.

## Verification

Offline:

```bash
conda activate CT
python scripts/run_breathing_profile.py --profile emma_normal_breathing --dry-run
python scripts/run_approach_and_seat.py --dry-run
pytest -q
# 313 passed in 26.63s
```

The dry-run now shows the origin frame in the sequence:

```
zero:    ID: 00000501  X Rx  DL: 1  00   <- 'here' becomes 0
```

Both new paths were exercised against a mock bus with the motor parked **500° (113 mm of
belt) from absolute zero** — the situation in which the old design would traverse:

| Scenario | Result |
|---|---|
| `SET_ORIGIN` takes | Proceeds. `pre_zero_position_rad` = 8.7266 recorded, ramp is +1.02 mm, no traverse |
| `SET_ORIGIN` silently fails | **Refuses**: "still reports 8.7266 rad (−113.45 mm) where it should report ~0. Commanding position 0 would therefore MOVE it by that much rather than hold it" |
| Ctrl+C mid-playback | `summary.json` written |
| Refusal path | `summary.json` written, with `refusal` set and `zero_applied: false` |

And the parent's handling, against a stub phantom that refuses and exits 1: `stdout.log`
written, contents echoed inline, parent exits 1 **without opening the base bus or moving the
base**.

**Not yet run on hardware.** The test that matters, and the one thing that would falsify this:

```bash
# park the phantom deliberately far from its absolute zero, somewhere a traverse is obvious
python scripts/run_breathing_profile.py --profile emma_normal_breathing
```

Expect a ~1 mm ramp. **Any traverse at all is a failure of this change.** Then read
`pre_zero_position_rad` from the summary — that number finally answers whether session 010's
reading was wrong, which is a question the previous runs cannot settle.

## Findings

- **A control input you cannot verify after the fact should not be load-bearing.** Session 010
  put an unverifiable reading at the centre of the motion arithmetic and then, in the same
  change, removed the only record that could have checked it. Commanding the origin is not
  merely more robust; it is *checkable* — the readback is a direct test of the property the
  motion depends on.
- **Referencing two logged series to the same suspect number hides errors in it.** Subtracting
  `initial_rad` from both commanded and measured made them agree beautifully while the motor
  was potentially in the wrong place entirely. The measured series must stay in the frame the
  hardware actually reports.
- **A summary written after `try/finally` is missing exactly when it is needed.** Three of the
  runs on 2026-09-02 have none, and the one number that would have diagnosed this was in it.
  Diagnostics belong in the `finally`.

Observed but deliberately not changed, per the decision to keep this session to the phantom:

- **The tactile zero drifts badly between runs** — 0.002 mm to 7.46 mm at startup across the
  runs of 2026-09-02. In run 151454 it read 0.196 mm, 1.96× the 0.1 mm threshold, so contact
  fired at t = 0.0003 s, the run had **no approach phase at all**, and the base then drove
  **42.7 mm** during standoff chasing a settled peak it never reached. Now warned about, not
  prevented.

## Open questions

- **Does `SET_ORIGIN` behave as documented on this firmware?** The readback check will say so
  on the first hardware run. If the origin does not take, the run refuses and
  `zero_readback_rad` records what it did report.
- **Was session 010's `initial_rad` actually wrong?** `pre_zero_position_rad` answers it on
  the next run; it cannot be recovered from the runs already taken.
- **The tactile firmware zero drift** above — whether to tare it in software, refuse on it, or
  power-cycle before each run. Deferred by choice.
- Carried: in-contact `R`, whether `tau_s = 0.677 s` is a constant, Q as the NIS suspect,
  approach travel needed for a seated contact, and standoff having no distance bound.

## Next steps

1. Park the phantom far from absolute zero and run `run_breathing_profile.py` alone. Confirm
   the ~1 mm ramp and read `pre_zero_position_rad`.
2. Power the phantom down and confirm the refusal fires and `phantom/stdout.log` captures it.
3. Only then a full `run_approach_and_seat.py --profile emma_normal_breathing`.
