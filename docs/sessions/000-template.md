# NNN — YYYY-MM-DD — <slug>

> Copy this file to `docs/sessions/NNN-YYYY-MM-DD-slug.md`, fill it in, and add a line to
> the index in `docs/sessions/README.md`.
>
> Record what was **actually measured**, not what was expected. Paste real command output.
> A session doc with invented numbers is worse than no session doc.

## Goal

What this session set out to do, in two or three sentences. If the goal changed partway
through, say so and say why.

## What changed

The substantive changes, in the order they matter to a reader — not the order they
happened. Prose, not a commit log.

## Files touched

| File | Change |
|---|---|
| `path/to/file.py` | one line on what and why |

## Decisions and rationale

Choices a future session might otherwise re-litigate. Include the alternatives considered
and why they lost. If something was decided by measurement, cite the number.

## Verification

Exact commands, with their real output.

```bash
conda activate CT
pytest -q
# paste result
```

| Check | Expected | Measured |
|---|---|---|
| | | |

## Findings

Anything learned about the *problem* rather than the code — a rule that does not hold, a
model that behaves unexpectedly, a number worth remembering. Promote the important ones to
the "Known findings" section of `CLAUDE.md`.

## Open questions

New or changed. Mark resolved ones as resolved and say what the answer was; move surviving
ones into `CLAUDE.md`'s open-questions list so they stay visible.

## Next steps

Concrete and ordered, so the next session can start without re-deriving context.
