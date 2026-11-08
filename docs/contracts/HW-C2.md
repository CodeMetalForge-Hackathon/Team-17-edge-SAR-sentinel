# HW-C2: Supervised flight recording

Status: PROPOSED. **Conditional. Not authorized by this document.** This plan cannot grant flight permission.

| Field | Value |
|---|---|
| Objective | One recorded real flight under staff supervision, to show alongside the dry-run arrow |
| Owner | C, with named staff |
| Branch | Evidence through a human commit on `codex/HW-C2` |
| Depends on | HW-C1 done, SW-08 applied, **written or named staff authorization** |
| Budget | Set by staff. Agents have none: agents never fly |

## Scope
1. Staff agree the place, time and limits. Record the name.
2. First do bitcraze's own basic example, as the README advises.
3. Human edits `DRONE_DRY_RUN` temporarily, with staff present, on a throwaway branch. Pins will fail by design on that branch. **Do not merge that change.**
4. Fly a short straight leg, well clear of people. Record video.
5. Restore the setting. Confirm `git status` and `DRONE_DRY_RUN = True`.

## Explicit exclusions
No flight at the judging table. No agent-run flight. No flying without Flow deck confirmation.

## Allowed edits
`docs/evidence/HW-C2/*`. Video stored outside git, with a link or checksum.

## Acceptance (evidence file `docs/evidence/HW-C2/report.md`)
Supervisor's name, time, place, commit used, target leg, what happened including failures, and a video reference.

## Risks
Crashes. Flow deck drift on plain floors. Battery. All stated, not hidden, in the demo.

## Stop conditions
Staff say stop. Any injury risk. A failed flight is not retried without staff agreeing.

## Handoff and close
If not achieved, the demo says so and shows the dry-run arrow only.
