# Rules for coding agents in this repo

Work happens only under an approved contract. Read `docs/contracts/TEMPLATE.md`, then the contract you were given.

- Confirm `git branch --show-current` matches the contract. Stop if not.
- Edit only the contract's allowed paths. Protected changes go to `docs/drafts/<ID>/` with an `APPLY.md`.
- Never commit by hand (in an approved campaign, `python tools/work_identity.py commit --contract <ID>` is the only commit path), push, merge, rebase, reset, stash, switch branches, install, download, flash, fly, or use `--no-verify`.
- Do not stage with `-A` or `.`. Run the verifier only when a human authorized it for this contract (an approved campaign counts, within its attempts).
- You cannot approve your own work. Approval reviewers are read-only agents in a fresh context: human-started, or, in an approved campaign, started by the coordinator and recorded by a hook. Other subagent reviews are advisory.
  A required Opus review is never replaced by another model. If blocked, stop and hand off. Never work around a guardrail.
- In an approved campaign you may start the `stage-reviewer` and `test-design-reviewer` agents yourself, within the budgets the gate enforces; a hook records their verdicts, and you may not write or edit a review. Record blockers in `docs/evidence/<campaign>/BLOCKERS.md` and continue independent authorized stages. Stubbed dependencies are diagnostic only.
- Models are explicit: Haiku scout, Sonnet builder and stage reviewer, Opus design reviewer only when required and approved.
- Token economy: delegate only bounded work worth the briefing cost; do trivial lookups yourself; one builder; at most two
  read-only scouts; no recursive delegation. Brief with task, files, contract requirements and every applicable guardrail, not the
  conversation. Scout reports are 300 words or fewer with evidence references; long logs go to files. Stay within the contract's
  dispatch, turn and retry budget. Record actual usage when shown; never invent savings. Do not re-read the repo or repeat research.
- Do not claim something works on hardware unless a recorded physical check says so.

Tools without Claude Code hooks, such as Codex, are not stopped by `.claude/` settings. For them these rules are
instructions, backed only by the git pre-commit hook and the verifier's manifest check.
