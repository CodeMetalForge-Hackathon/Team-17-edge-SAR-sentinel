# DEMO-1: Software spine for the first demo (campaign)

Status: PROPOSED. Approved by one human command, `guardrail_admin.py approve-campaign DEMO-1`, which records the SHA-256 of
[tools/campaigns/DEMO-1.json](../../tools/campaigns/DEMO-1.json) (the machine-readable copy of the numbers below).

| Field | Value |
|---|---|
| Objective | The simulated and bench-ready software path is reset-safe, stale-safe, hazard-correct, honestly worded and started by one launcher. Ready for the Pi bench (INT-01), not claimed to work on hardware |
| Owner / coordinator | One Sonnet session (the coordinator and builder), started by the human with the prompt pack's campaign prompt |
| Branch | `codex/DEMO-1`, from local `main` after BS-00. One branch, one guarded commit per stage |
| Stages, in order | SW-00, SW-01, SW-03, SW-04, SW-02, SW-05, SW-06. Each keeps its own contract file as the stage specification (scope, exclusions, allowed edits, acceptance) |
| Hard dependencies | SW-03 needs SW-01, SW-04 needs SW-03, SW-02 needs SW-04, SW-05 needs SW-02, SW-06 needs SW-05. SW-00 and SW-01 are independent |
| Stronger (Opus) review required | SW-01, SW-03, SW-04, SW-02, SW-05. Not SW-00 or SW-06 |
| Pre-approved by approving this campaign | See below |

## What approval pre-authorizes (and nothing else)
- Development test runs and scratch exports of drafts, any number within each stage's 3-hour guide.
- Verifier runs: up to the attempts in the campaign file per stage (SW-00 3, others 4), including retries after a FAIL. The stall rule (two identical failures) and budget stop still apply.
- Independent reviews started by the coordinator as subagents: `stage-reviewer` (Sonnet, read-only) per stage, `test-design-reviewer` (Opus, read-only) up to 8 sessions in total, `scout` (Haiku, read-only) up to 40 dispatches, 8 turns each, one retry per delegated task.
- A guarded commit per stage through `python tools/work_identity.py commit --contract <ID>`, only after verifier PASS and the required independent APPROVEs on the exact work identity.
- Moving on to the next stage whose hard dependencies are committed, without asking.
- Wall time: 14 hours from approval, then the approval lapses and a human must re-approve.

## Human gates that stay (the campaign stops or skips, records `docs/evidence/DEMO-1/BLOCKERS.md`, and continues independent stages)
1. **Applying protected-file drafts** (tests, tools, hooks, settings, contracts, `pi/drone_dispatch.py`). The agent drafts; a human applies. A stage whose acceptance needs one is BLOCKED.
2. **Merging or pushing.** Agents never do either. The branch is ready for a human to merge.
3. **Hardware actions**, flashing, flight, or anything INT-01 or an HW contract owns.
4. **Spending or budget increases**: more Opus sessions, attempts, dispatches, time, any download or paid API.
5. **Material scope change**: a stage's allowed edits, exclusions or acceptance, or a new stage.

## Explicit exclusions
SW-07 (Hailo), SW-08 (flight code), the ML contracts, HW and INT contracts. They are outside this campaign. A second campaign may cover SW-07 on its own branch.

## Evidence honesty
Stub, fake or simulated dependencies are diagnostic only. A stage passes only when `verify.py` runs the real required modules in the core `.venv`.
Teammate reports of Pi results stay claims until recorded evidence exists. Software PASS is not hardware validation; the physical checks stay listed as outstanding.

## Stop conditions
Common stop rule. On a stop the coordinator writes the blocker, skips to any stage whose dependencies are met, and when none is left writes `docs/evidence/DEMO-1/REPORT.md` and ends.

## Handoff and close
Final `REPORT.md`: per stage, the committed hash, the verifier line, the review files, and the outstanding physical checks. The human reviews and merges.
