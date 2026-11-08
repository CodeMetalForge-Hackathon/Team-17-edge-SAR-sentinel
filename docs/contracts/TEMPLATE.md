# Contract template and common terms

No local template existed, so this one is new. A contract is PROPOSED until a human approves it. Once the
guardrails are INSTALLED, this folder is protected. Contracts cannot change themselves.

## Common terms (apply to every contract unless it says otherwise)

**Authority.** Contracts define authority. `TODO.md` is only an index and does not override them. Any expensive,
hardware, flight, download or paid run not named in a contract is **not pre-approved**.

**Baseline.** Contracts branch from local `main`, and the verifier measures scope against `main`. `main` must therefore already contain every
teammate commit the contract builds on (at this writing, `7a9e315`). A human fast-forwards local `main` to it first (BS-00 APPLY step 2). A teammate-reported
result such as "works on the Pi" is a claim until a recorded check (`docs/evidence/`) says so.

**Branch.** Each contract works on `codex/<ID>`, created by a human, preferably in its own worktree so nobody
switches branches over a teammate's uncommitted work:

```bash
git worktree add ../T17-<ID> -b codex/<ID> main
```

Before any work, the agent runs `git branch --show-current` and **stops if it differs**.

**Agents never** commit by hand (inside an approved campaign the only commit path is `work_identity.py commit`), push, merge, rebase, reset, stash, switch branches, install packages, stage with `-A` or `.`,
use `--no-verify`, fly, flash firmware, or edit protected paths.

**Protected paths.** The list is `tools/protected_paths.json` (DRAFTED in BS-00). Changes there go to
`docs/drafts/<ID>/` with an `APPLY.md`, and a human applies them. Applying a draft is the only route, unless
the additive test route below applies.

**Roles**

| Role | Model (explicit) | May | May not |
|---|---|---|---|
| Builder / coordinator (one session) | Sonnet | Edit a contract's allowed paths, write drafts and evidence, dispatch scouts and (in a campaign) the reviewers, run the verifier within its pre-approved attempts, commit through the guarded tool | Approve its own work. Change protected paths. Write or alter a review |
| Scout (read-only, optional) | Haiku | Narrow lookups with evidence references | Decide anything. Delegate. Edit |
| Stage reviewer (read-only) | Sonnet, a separate fresh session | Read diffs, evidence and contracts. Give a verdict | Edit anything |
| Test and design reviewer (read-only, stronger) | Opus, a separate fresh session, **only when the contract requires it and a human approves the spend** | Review test sufficiency and design changes | Edit anything |
| Human | n/a | Everything above, plus install, commit, flash, fly, approve, merge, push | |

**Model routing is explicit.** Every agent definition and every dispatch names its model. A dispatch never relies on
inheriting the parent's model, and the session that starts a role states the model it started it on. The agent
definitions in `.claude/agents/` carry `model:`. A `CLAUDE_CODE_SUBAGENT_MODEL` environment variable or a session
setting can override them, so the dispatching session reports the model it actually saw (or says it could not see it).

**Independent approval.** An approval reviewer is a read-only agent in a fresh context that did not build the work: either a **fresh human-started session**, or, inside an approved campaign, the `stage-reviewer` / `test-design-reviewer` agent that the coordinator starts. In a campaign the SubagentStop hook records the reviewer's verdict itself (`RECORDED_BY: hook`), so the builder cannot write or edit a review, and `gate` refuses a review that was only relayed. Other builder-spawned reviews are advisory. The human saves each verdict to
`docs/evidence/<ID>/review-<stage>.md`. A subagent report relayed by the builder does not count as approval.

**When the stronger reviewer is required.** Any new topic or payload field, any change touching hazard priority,
dispatch conditions or flight, any change of a `vib`-style field meaning, any test edit request, and any request
to lower a threshold. The contract's "Stronger review required?" field decides, and
`tools/verify_stages/<ID>.json` mirrors it as `stronger_review_required` (the contract wins on disagreement; a
missing key is treated as required). The requirement is met only by an **Opus** review of the current work identity.
Spending Opus tokens is a human decision: made when starting the session, or, in a campaign, once when approving its `opus_review_sessions` budget. **If the review is required and
absent, the contract waits at "awaiting design review". Nobody substitutes a Sonnet review, and no gate treats one
as equal.** This planning pass did not approve any future implementation.

**Token economy** (applies to every agent; the point is fewer wasted tokens, not fewer checks)

- Delegate only a bounded task where the benefit exceeds the cost of briefing and reading the result. Do a trivial
  lookup (one file, one symbol) directly.
- One builder at a time per contract by default. At most **two** concurrent read-only scouts. No recursive
  delegation: scouts and reviewers have no `Agent` tool.
- A brief contains the task, the relevant file paths, the contract requirements **and every guardrail that applies**
  (branch check, allowed paths, no commits, no installs, read-only), not the conversation.
- A scout report is normally 300 words or fewer, with `file:line` evidence references. Save long logs to a file
  under `docs/evidence/<ID>/` and return only the decisive excerpt and the path.
- Do not re-read the whole repository or repeat research another agent or an earlier stage already recorded. Read
  `docs/evidence/<ID>/` first.
- Each contract has a dispatch, turn and retry budget (see Default budgets). Record actual usage where the tool
  reports it. **Never estimate or claim a token saving that was not measured.**

**Verifier.** "Done" for software means: `python tools/verify.py --contract <ID>` from the core `.venv` reports
`SOFTWARE PASS`. It runs **only** after a human authorizes it:

```bash
python tools/guardrail_admin.py authorize-verify <ID> --attempts <budget>
```

Missing dependencies are `BLOCKED`, never a pass. Skips and xfails in required stages fail. Software pass does
not mean physical validation is done; the verifier prints outstanding physical checks every time.

**Scratch verification.** Protected drafts can be tried only in an isolated export:

```bash
python tools/scratch_export.py --contract <ID> --impl-paths <paths> --overlay docs/drafts/<ID>/OVERLAY.json --out <dir-outside-repo>
```

This starts from `git archive HEAD` with `core.autocrlf=false`. Since `git archive` excludes uncommitted work, it
then applies only your uncommitted changes under `--impl-paths` as a patch and copies untracked files under those
paths. Then it copies drafts to the targets listed in the overlay, and it writes `SCRATCH_RECORD.json` with the
base commit, patch hash and draft hashes (SHA-256 of bytes). Run the existing tests unchanged there. **Scratch success is evidence
for a draft, not live acceptance. Never run the verifier in scratch.** Budgets still apply.

**Additive test route.** A new test is installed only after all of these:
1. Builder drafts it in `docs/drafts/<ID>/tests/`.
2. An independent reviewer approves it, naming its sha256 (SHA-256 of the file's bytes, from `python tools/work_identity.py sha256 <file>`; not a git blob id).
3. A human runs `python tools/guardrail_admin.py add-test <draft> --contract <ID> --review docs/evidence/<ID>/review-tests.md`, which runs `tools/check_additive.py`.

The check verifies the destination is new and matches `tests/contract/test_<id>_*.py`, the review names this
file's hash, and existing manifest entries are unchanged. It also rejects skips, xfails, collection hooks,
`conftest`, module shadowing, `sys.modules` writes, subprocess and network use, repo writes, and threshold
changes at import. Editing or deleting an existing test, verifier or config changes, and lower floors always go
to the human through `APPLY.md`. A reviewer saying "approved" installs nothing.

**Close.** In order: **verifier `SOFTWARE PASS` → independent reviewer `APPROVE` → explicit-path commit.** There is
no pre-approval candidate commit. The work stays uncommitted until it has passed and been approved, and it is
identified by a **work identity** instead of a commit hash.

*Work identity* (`python tools/work_identity.py show --contract <ID>`, read-only). A SHA-256 over: the base commit
(`HEAD`) and branch; every changed path outside `docs/evidence/<ID>/` and `TODO.md`, each with its state
(`modified`, `new`, `deleted`), the SHA-256 of its **actual file bytes**, and its git blob id; and the SHA-256 of the
contract, its stage config, the manifest, the pins and the path list. The git blob id comes from
`git hash-object` and is **not** a file SHA-256; the two are recorded under separate names and never swapped. An
informational tracked-patch hash is printed for reviewers but is not part of the identity. The index must be clean
when the identity is computed.

1. The builder finishes, leaves everything uncommitted and unstaged, and records `work-identity.json` in
   `docs/evidence/<ID>/`.
2. A human authorizes the verifier (once per contract, or once for a whole campaign). The verifier computes the identity before and after the run and records both in
   `verify.json`. A difference is TAMPER. Only `SOFTWARE PASS` continues.
3. A human, or inside an approved campaign the coordinator, starts the fresh reviewer(s). Each recomputes the identity, reads `verify.json`, and ends with
   `VERDICT: APPROVE identity=<64 hex> base=<40 hex>` plus `REVIEWER_MODEL:` and `VERIFY_RECORD_SHA256:` lines.
   Any relevant change after this point (an edit to any identity path, a new or deleted file, a new base commit, a
   guardrail change) changes the identity and invalidates the verification and every review. Start again from
   step 1 or 2 as the change requires. Evidence files and `TODO.md` may change without invalidating anything.
4. `python tools/work_identity.py gate --contract <ID>` must pass: latest verify record is `SOFTWARE PASS` on this
   identity, and the required review files `APPROVE` this identity, with an Opus review when required.
5. The human stages exactly the identity's paths plus `docs/evidence/<ID>/` and `TODO.md` by explicit path, runs
   `python tools/work_identity.py commit-check --contract <ID>`, then commits one commit. Outside a campaign agents do not commit; inside one, `work_identity.py commit` does the gate, staging, check and commit itself.
   `python tools/work_identity.py post-commit --contract <ID>` confirms the commit's parent and blob ids afterward.

Then the human merges. Physical checks still outstanding are listed, not hidden. Writing or approving a contract
does not authorize a commit.

*Limits of enforcement.* The identity proves the bytes were the same when it was computed. It does not prove a
reviewer read them, that a session was independent, or that a review file was written by who it names; review files
are text a human saves. `gate` and `commit-check` run when someone runs them: no hook forces it, and the
pre-commit hook does not call them because ordinary work-in-progress commits are legitimate. Ignored files,
file-mode bits, symlink targets, line-ending conversion settings, installed packages, environment variables and
hardware state are outside the identity. A change between `commit-check` and `git commit` is not detected until
`post-commit`.

**Campaign mode (autonomous within one approval).** A *campaign* groups several contract stages on one branch (`tools/campaigns/<ID>.json`, described by
`docs/contracts/<ID>.md`). One human command, `python tools/guardrail_admin.py approve-campaign <ID>` (typed phrase), records the SHA-256 of the campaign file and an
expiry. It pre-authorizes, with counters enforced outside the agent: verifier attempts per stage and retries, reviewer sessions (Opus limited by a budget),
scout dispatches, and one guarded commit per stage. The coordinator then runs stage after stage: build, verify, stage review, design review if required, `work_identity.py commit`.
Scope for each stage is measured from the previous stage's commit. Editing the campaign file voids the approval; budgets rise only by a human edit through APPLY and a new approval.

Human gates that no campaign pre-approves: applying protected-file drafts, merging or pushing, any hardware action, any spend or budget increase, any material scope change.
When blocked, the coordinator records the blocker in `docs/evidence/<campaign>/BLOCKERS.md`, then continues any stage whose hard dependencies are met.
Stubbed or faked dependencies are diagnostic only; acceptance uses the real modules in the core `.venv`, which the verifier checks (a required module that resolves to a file inside the repository outside `.venv` is TAMPER).

**Local acceptance and publishing are separate.** A guardrail or contract set is *locally accepted* when a human has
installed it, run its acceptance checks, and merged it into local `main` with the evidence committed. Work may begin
from locally accepted state without a push. A human `git push` only *distributes* accepted guardrails to teammates,
who then pull and anchor them. Agents never push or merge.

**Stop rule.** Stop and write `docs/evidence/<ID>/HANDOFF.md` (see the prompt pack) if the budget is spent,
progress stalls, tampering is detected, a dependency is missing, or a rule blocks the work. Never work around
the block, and never mark the contract done.

**Handoff report at every stage.** Contract and stage. Actual verification result. Evidence location. Protected
drafts awaiting a human. Open judgment calls and hardware checks. The exact next prompt and who runs it.

**Default budgets.** 4 verifier attempts. Stall after 2 identical failures. No downloads and no paid API.
3 hours of wall time per software contract. Local tests only. **Agent budget per contract:** at most 6 scout
dispatches, 8 turns per scout, 1 retry per delegated task (a second failure is a handoff, not a third try), at most
2 scouts at once, 1 builder, 1 stage-review session and, only when required and approved, 1 Opus review session per
work identity. A contract may set tighter numbers in its Budget row, never silently looser ones.

---

## Copy from here for a new contract

```markdown
# <ID>: <title>

Status: PROPOSED

| Field | Value |
|---|---|
| Objective | |
| Owner / builder | |
| Branch | codex/<ID> |
| Depends on (merged first) | |
| Stage order | |
| Budget | Attempts, wall time, compute, downloads, paid API, agent budget (scout dispatches, turns, retries) |
| Stronger review required? | Yes or No, with the reason. Yes means an Opus review of the final work identity, spend approved by a human |

## Scope
## Explicit exclusions
## Allowed edits
## Protected paths touched (draft only)
## Pre-approved commands and runs
## Acceptance
Automated: exact commands and tests.
## Hardware and manual checks, with evidence required
## Risks
## Stop conditions
## Handoff and close
```
