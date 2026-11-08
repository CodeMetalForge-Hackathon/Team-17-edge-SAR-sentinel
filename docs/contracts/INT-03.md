# INT-03: Final docs, claims check, and freeze

Status: PROPOSED

| Field | Value |
|---|---|
| Objective | What we say matches what we measured. The repo is ready for judges |
| Owner / builder | D / builder agent for drafts, human for tag and push |
| Branch | `codex/INT-03` |
| Depends on | INT-02. Done by Sun 11:00 |
| Stage order | Build, stage review, human merge, tag |
| Budget | 1 hour. No verifier-gated code changes |
| Stronger review required? | **Yes**, for claims |

## Scope
1. README: replace the demo script, quick start and architecture with what is true now. Fix the claim that a tap fires SURVIVOR unless it was measured to.
2. `THIRD_PARTY.md`: separate rows for code, checkpoint, dataset. xBD is CC BY-NC-SA 4.0 with attribution. Say what ran where.
3. Judge answers: every number comes from an evidence file, with no "about 0.2% false alarms" from sim presented as field data. Label README's sim numbers.
4. Limits section: no routing, no live table flight, no speech, no stability claims, satellite model does not transfer indoors, staged zone link.
5. Check that the "written this weekend" statement is accurate. A human confirms.

## Explicit exclusions
No application changes.

## Allowed edits
`README.md`, `THIRD_PARTY.md`, `TODO.md`, `docs/evidence/INT-03/*`.

## Pre-approved commands and runs
Read-only checks. `tests/test_core.py`, invariant tests. The human tags and pushes.

## Acceptance
Each claim in README links to evidence, or is removed. Stronger reviewer approves. Tests still pass.

## Hardware and manual checks
A human reads the final README against the rehearsal record.

## Risks
Rushing at the end. Keep it short.

## Stop conditions
Common stop rule.

## Handoff and close
Human merges, tags and pushes. **Nothing merges after that without the whole team agreeing.**
