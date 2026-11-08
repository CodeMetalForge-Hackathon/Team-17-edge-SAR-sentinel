# Execution prompt pack

Copy a prompt, then change only the two values at its top. `$CONTRACT` in a prompt means that value. Each one starts
from the current state and stops at human gates. **Elapsed time never counts as approval.** Evidence does.

**Status: DRAFTED.** Until BS-00 is installed, every rule in these prompts is an instruction only. Nothing enforces it.

Prompts 1 to 7 are the manual flow (bootstrap, and single contracts outside a campaign). Inside an approved campaign use prompt 0.

**Models are explicit, never inherited.** Start each session on the model in the table (the human chooses it when
starting the session). Subagents are dispatched only by name from `.claude/agents/`: `scout` is Haiku. Opus is used only
by prompt 4, only when the contract requires a stronger review and a human has approved the spend. If it is required and
not run, the contract waits: no Sonnet review stands in for it. Token-economy rules are in TEMPLATE.md and apply to every
prompt below: bounded delegation only, one builder, at most two read-only scouts, no recursive delegation, briefs carry
task, files, contract requirements and every applicable guardrail (not the conversation), scout reports are 300 words or
fewer with evidence references, long logs go to files, per-contract budgets, actual usage recorded, never invented savings.

## 0. Campaign coordinator (the normal way to work)

After the one-time `approve-campaign DEMO-1` (APPLY 6.2), start **one Sonnet session** (suggested effort: medium) in `..\T17-DEMO-1` and paste this. The coordinator builds, runs the verifier within its pre-approved attempts,
starts the independent reviewers itself, commits each passed stage through the guarded tool, and moves on. Prompts 2 to 5 below remain as the **manual fallback** for a single contract outside a campaign.

```text
CAMPAIGN=DEMO-1
BRANCH=codex/DEMO-1

You are the coordinator and builder for campaign DEMO-1, running on Sonnet. A human approved the campaign once; that approval, and nothing else, is your authority. Work until every stage is committed, or only human gates remain. Do not ask me to relay prompts or approve ordinary steps.

Setup (once)
1. git branch --show-current must equal BRANCH, else stop. Read AGENTS.md, docs/contracts/TEMPLATE.md (Campaign mode), docs/contracts/DEMO-1.md, tools/campaigns/DEMO-1.json, TODO.md, docs/evidence/DEMO-1/ if it exists. (tools/guardrail_admin.py is for humans; never run it.) Run git status --short and git log --oneline -5.
2. Confirm the approval is live: python tools/work_identity.py gate --contract SW-00 must fail only for ordinary reasons (no verify record yet), not "campaign" or "expired" reasons. If the campaign is not approved or expired, write the blocker and stop: that is a human gate.
3. Resume from evidence, not memory: for each stage in order, it is DONE if its guarded commit exists (git log --oneline --grep "^<ID>:").

For each stage in the order SW-00, SW-01, SW-03, SW-04, SW-02, SW-05, SW-06 whose hard dependencies are DONE:
A. Read only that stage's contract (docs/contracts/<ID>.md) and the code it names. Do trivial lookups yourself. Dispatch the Haiku scout only for a bounded question worth the briefing cost, with a brief of task, paths, contract requirements and every applicable guardrail; at most two at once; reports of 300 words or fewer with file:line evidence; long logs to files under docs/evidence/<ID>/.
B. Build within the stage's Allowed edits only. Run the stage's tests directly while developing (pytest in the core .venv, with the real dependencies). Stubbed or faked dependencies are diagnostic only and never count as acceptance.
C. If a protected file must change or a required test looks wrong, do not touch it and do not weaken it: write the draft with an APPLY.md under docs/drafts/<ID>/, record the stage as BLOCKED in docs/evidence/DEMO-1/BLOCKERS.md, and go on to the next stage whose dependencies are met.
D. When the work is ready, leave it uncommitted and unstaged and record its identity: python tools/work_identity.py show --contract <ID> --out docs/evidence/<ID>/work-identity.json
E. Verify: .venv/Scripts/python tools/verify.py --contract <ID>. Campaign approval pre-authorizes the stage's attempts. On FAIL, fix and retry (new identity each time) up to the builder fix cycles in the campaign file. STALLED, BUDGET EXHAUSTED, TAMPER, BLOCKED or WRONG BRANCH: stop that stage, record the exact message in BLOCKERS.md, do not retry for a different answer, continue elsewhere.
F. On SOFTWARE PASS, start the independent reviewers as subagents, in this order, each with a brief that contains ONLY the contract ID, the work identity and base from the verify record, and the changed paths (never your reasoning or your summary):
   - stage-reviewer (Sonnet) always.
   - test-design-reviewer (Opus) when the stage requires it. If the gate refuses it (budget spent, expired), the stage waits: record it, do not substitute another model, do not review yourself.
   A hook records their verdicts. You may not write, edit or paraphrase a review. If a verdict is REJECT, fix the reasons, which changes the work identity, then go back to E.
G. Update TODO.md for the stage (SW PASS, with physical checks outstanding). Then: python tools/work_identity.py commit --contract <ID>. It gates, stages explicit paths, commits and advances the campaign. Never git commit by hand.
H. Move on to the next stage. After each stage append a three-line note to docs/evidence/DEMO-1/progress.md (stage, commit hash, verifier line, reviews, usage you actually saw; no estimated savings).

Rules: never push, merge, switch branches, install, download, flash, fly, stage with -A, use --no-verify or touch protected paths. Human gates are only: protected-file application, merging/pushing, hardware, spend or budget increases, material scope change. If a guardrail blocks you, record it and move on; do not look for another way. Software PASS is not hardware validation.

Finish by writing docs/evidence/DEMO-1/REPORT.md (per stage: commit hash or BLOCKED reason, verifier result, review files, outstanding physical checks, drafts awaiting a human) and end with the HANDOFF block.
```

---

Who runs what:

| # | Prompt | Run by | Suggested model | Starts when |
|---|---|---|---|---|
| 1 | Bootstrap helper | Human, on `codex/BS-00` | Sonnet (scout: Haiku, optional) | Human chooses to install |
| 2 | Builder: build and scratch (no commit) | Human, in the contract's worktree | Sonnet (scout: Haiku, optional) | Dependencies closed locally |
| 3 | Stage reviewer | Human, in a **fresh** session | Sonnet | Verifier `SOFTWARE PASS` recorded |
| 4 | Test and design reviewer | Human, in a **fresh** session | **Opus**, only if the contract requires it and the human approves the spend | Verifier `SOFTWARE PASS` recorded |
| 5a | Verify | Human, after authorizing | Sonnet | Builder reports ready |
| 5b | Gate and prepare commit | Human | Sonnet | Required reviews recorded |
| 6 | Resume | Human | Same as interrupted | After an interruption |
| 7 | Blocked or stalled handoff | Human | Same as stopped | When a stop rule fires |

Suggested effort: prompt 1 low to medium, prompt 2 medium, prompts 3 and 5a/5b low to medium, prompt 4 high. Higher effort buys depth, not permission.

Order of a contract: 2 (build) → 5a (verify) → 3 (stage review) → 4 (design review, if required) → 5b (gate; the human
commits). The verifier runs before the reviewers so they can check its record. Reviews bind to a **work identity**
(see TEMPLATE.md, Close), not to a commit, because nothing is committed until the end.

Every report ends with this block, filled in:

```text
HANDOFF
Contract and stage:
Actual verification result (quote it, or say "not run"):
Evidence location:
Protected drafts awaiting a human:
Open judgment calls:
Hardware checks outstanding:
Next prompt, and who runs it:
```

---

## 1. Bootstrap helper

```text
CONTRACT=BS-00
BRANCH=codex/BS-00

You are helping a human install guardrails. You do not install them. The human runs every command.

1. Run: git branch --show-current. It should be codex/BS-00. It may also be main or the throwaway codex/SW-01 once APPLY.md section 5.3 has started, if the merge shows in git log. Local acceptance does not need a push; do not tell me to push to continue. Otherwise stop and tell me how to get to codex/BS-00. Do not switch or create branches yourself.
2. Read docs/contracts/BS-00.md, docs/drafts/BS-00/APPLY.md and docs/drafts/BS-00/DRAFT_CHECKS.md.
3. Run read-only checks only: git status --short, git log --oneline -5, and git merge-base --is-ancestor 7a9e315 main (reports whether local main has the teammates' baseline; do not fix it yourself), and test whether .venv, tools/verify.py, tools/protected_manifest.json, .claude/settings.json and .git/hooks/pre-commit exist.
4. From that state, tell me which APPLY.md step I am on, and give me only the next step's command, with what to expect.
5. After I paste a result, compare it with APPLY.md's expected result. If it differs, say so plainly. Do not reword the expectation. A difference is a draft defect for me to resolve.

Rules: use the Haiku scout only for a bounded lookup that costs less than doing it yourself (normally none are needed). Do not edit protected paths, do not commit, push, merge, install, or run guardrail_admin.py or tools/verify.py yourself. Do not say a guardrail is active unless my pasted result shows it. Do not start SW-01.

When all of APPLY.md section 5 has recorded results and I have shown the local acceptance commit in section 6, write the HANDOFF block, naming prompt 2 for SW-01 as next, run by a human in a new codex/SW-01 worktree created from local main. Say plainly that the push in section 7 is a separate, optional distribution step for teammates and that no agent does it.
```

---

## 2. Builder: build and scratch (does not commit)

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

You are the builder and coordinator for this one contract, running on Sonnet. Check the current state before you assume anything.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read AGENTS.md, docs/contracts/TEMPLATE.md, docs/contracts/$CONTRACT.md, and PLAN.md's sections on this area. Then read TODO.md (an index, not authority), git log --oneline -10, git status --short, and docs/evidence/$CONTRACT/ if it exists. Do not re-read the whole repository or repeat research already recorded there.
3. Confirm local acceptance: tools/protected_manifest.json and docs/evidence/BS-00/install-log.md both exist, and for each contract this one depends on, its evidence commit is in local main: git log --oneline main -- docs/evidence/<dependency>/. A push is not required. Also confirm local main contains the teammates' baseline: git merge-base --is-ancestor 7a9e315 main. If any of these is missing, stop with a HANDOFF. Do not run guardrail_admin.py; it is for humans.
4. Implement only the Scope, editing only Allowed edits. Excluded work stays excluded even if it looks useful.
5. Delegation: do trivial lookups yourself. Dispatch the `scout` agent (Haiku) only for a bounded question that costs less than doing it yourself. At most two scouts at once, no scout may delegate, and stay inside the contract's agent budget (default: 6 dispatches, 8 turns each, 1 retry per task). Each brief carries the task, the relevant paths, the contract requirements and every applicable guardrail (read-only, no commits, no installs, branch check), not this conversation. Ask for 300 words or fewer with file:line evidence. Save any long log under docs/evidence/$CONTRACT/ and keep only the decisive excerpt. Anything a subagent says is evidence to check or an advisory opinion, never an approval. Keep a short dispatch note in docs/evidence/$CONTRACT/dispatch.md: what was asked, which model, turns used if shown. Record actual token usage only if the tool shows it, and never estimate a saving.
6. Run the acceptance tests directly while developing, for example: .venv/Scripts/python -m pytest -q <test files named in the contract>. Those are not the verifier. Do not run tools/verify.py in this stage.
7. If protected content must change, do not touch the live path. Write the change in docs/drafts/$CONTRACT/ with an APPLY.md of ordered human steps and validation commands, and an OVERLAY.json if you need to test it. You may test only in a scratch copy:
   python tools/scratch_export.py --contract $CONTRACT --impl-paths <your paths> --overlay docs/drafts/$CONTRACT/OVERLAY.json --out <new folder outside the repo>
   Run existing tests unchanged there, within budget. Report it as evidence for a draft, not acceptance.
8. If a required test looks wrong, do not change, skip or weaken it. Write up the evidence in docs/drafts/$CONTRACT/ and hand off.
9. When the work is ready for verification and no protected draft is waiting on a human, do NOT commit and do NOT stage. Leave the changes uncommitted and unstaged, then run:
   python tools/work_identity.py show --contract $CONTRACT --out docs/evidence/$CONTRACT/work-identity.json
   If a draft is waiting, stop and name the human APPLY step instead.

Never: commit, push, merge, switch branches, stage with -A or ., use --no-verify, install, download, flash, or fly. If a guardrail blocks you, stop. Do not find another way. Any edit to the work after step 9 changes its identity and voids any verification or review; if you must edit, say so and rerun step 9.

Finish with: the work identity (identity_sha256 and base_commit), the list of changed paths, test results as actually run (counts, not adjectives), the dispatch summary (number of dispatches and the models used), then the HANDOFF block. Name prompt 5a as next, run by a human after they authorize the verifier.
```

---

## 3. Stage reviewer

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

You are the read-only stage reviewer, running on Sonnet in a fresh session started by a human. You did not build this and nothing here was delegated to you by a builder. If that is untrue, say so and mark all of this ADVISORY ONLY. Change nothing: not code, tests, contracts, evidence or approvals.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read docs/contracts/TEMPLATE.md and docs/contracts/$CONTRACT.md. Do not take the builder's summary as fact.
3. Recompute the identity of the uncommitted work yourself: python tools/work_identity.py show --contract $CONTRACT. Compare identity_sha256 and base_commit with docs/evidence/$CONTRACT/work-identity.json and with work_identity in the newest docs/evidence/$CONTRACT/verify-*/verify.json. If any differ, or the verify result is not SOFTWARE PASS, REJECT and say why. Compute the verify record's bytes hash: python tools/work_identity.py sha256 <path to that verify.json>.
4. Inspect the real change: git diff HEAD, git status --short, and the untracked files listed in the identity.
5. Check, with file:line references:
   - every changed path is an Allowed edit, and no protected path changed
   - the change does the Scope and none of the Exclusions
   - existing topics and required payload fields are unchanged, and any new one is named in this contract
   - hazard priority is unchanged
   - evidence exists and matches what was claimed; any subagent finding the builder relied on was checked, not trusted
   - physical checks appear as outstanding, not implied as done
6. Run tests only if the contract's pre-approved commands allow it. Do not run tools/verify.py. Do not start hardware.

End with exactly these three lines, in this order, then numbered reasons and the HANDOFF block:
VERDICT: APPROVE identity=<64 hex identity_sha256> base=<40 hex base_commit>      (or REJECT with the same fields)
REVIEWER_MODEL: <the exact model you are actually running as>
VERIFY_RECORD_SHA256: <64 hex SHA-256 of the verify.json bytes you read>

The human saves your output as docs/evidence/$CONTRACT/review-stage.md. Your verdict covers only the identity named. Any later change to any reviewed path voids it. If the contract requires the stronger reviewer, name prompt 4 next (a human decides on the Opus spend; this review does not replace it), otherwise prompt 5b.
```

---

## 4. Test and design reviewer (Opus, only when required)

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

You are the stronger read-only reviewer for test sufficiency and design, in a fresh session started by a human on an Opus model. The contract requires this review and the human approved the spend. If you are not actually running on Opus, say so in REVIEWER_MODEL and stop: a required Opus review is never replaced by another model. Change nothing. If a builder dispatched you, everything you write is ADVISORY ONLY.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read docs/contracts/$CONTRACT.md, PLAN.md, the contract's tests, the uncommitted change (git diff HEAD, plus the untracked files in the identity), docs/evidence/$CONTRACT/review-stage.md, and any drafts in docs/drafts/$CONTRACT/. Read only what the questions below need.
3. Recompute the identity: python tools/work_identity.py show --contract $CONTRACT. It must equal the identity in work-identity.json, the newest verify.json (which must be SOFTWARE PASS) and review-stage.md. If not, REJECT. Compute the verify record hash: python tools/work_identity.py sha256 <that verify.json>.
4. Answer with evidence:
   - For each key behaviour: name a mutation the tests would catch, and one they would miss. Is any gap serious enough to matter?
   - Does anything weaken, skip, shadow or bypass a check, or rely on sim or fakes while claiming more?
   - Can a survivor signal now outrank a hazard? Can cached context affect avoid zones, decide() or dispatch?
   - Any changed field meaning: is it documented, and are old artifacts handled?
   - Are the words honest? Software pass is not hardware validation, and selected examples are not accuracy.
5. For a new test proposed on the additive route: run python tools/check_additive.py <file> --contract $CONTRACT, then state its SHA-256 from python tools/work_identity.py sha256 <file>. That is the hash of the file bytes. Do not use git hash-object output; it is a different value that guardrail_admin.py will not accept. Check it is not shadowing, disabling or lowering anything.

End with exactly these lines, then numbered reasons and the HANDOFF block:
VERDICT: APPROVE identity=<64 hex> base=<40 hex>      (or REJECT)
REVIEWER_MODEL: <exact model id>
VERIFY_RECORD_SHA256: <64 hex>
TEST_SHA256: <64 hex>      (only for an additive test review)

The human saves this as docs/evidence/$CONTRACT/review-design.md (or review-tests.md for an additive test, which must contain the TEST_SHA256 value). You approved nothing beyond the identity or file you name, and nothing is installed by your saying so. Next is prompt 5b if approved, or prompt 2 with your reasons if not.
```

---

## 5a. Verify (after the human authorizes)

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

You run the verifier once, on Sonnet. You do not close anything and you do not commit. Only proceed on evidence.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read docs/contracts/$CONTRACT.md. Run git status --short and python tools/work_identity.py show --contract $CONTRACT. Its identity_sha256 must equal docs/evidence/$CONTRACT/work-identity.json. If not, the work changed since the builder reported it: stop and ask for prompt 2 step 9 to be redone. Nothing may be staged; if the tool refuses because of staged paths, stop.
3. Run once, from the core virtual environment: .venv/Scripts/python tools/verify.py --contract $CONTRACT
   It will refuse without human authorization. If so, stop and ask me to run guardrail_admin.py authorize-verify. Do not authorize it yourself.
4. Act on the printed result:
   - SOFTWARE PASS: report it. Do not update TODO.md yet.
   - FAIL: stop and report. A fix belongs to prompt 2; the work identity will change, so verification repeats.
   - BLOCKED, STALLED, BUDGET EXHAUSTED, TAMPER, WRONG BRANCH: stop. Use prompt 7. Do not retry to get a different answer. TAMPER can mean the work changed during the run.

Report exactly: the verifier result line, the work identity before and after (from verify.json), the evidence folder, the printed list of physical checks still outstanding, then the HANDOFF block. Next is prompt 3, and prompt 4 if the contract requires the stronger reviewer, each run by a human in a fresh session. Do not say "done" or "working" where the result covers only software.
```

---

## 5b. Gate and prepare the commit

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

You check the close gate and prepare, but do not make, the commit. Sonnet. Only proceed on evidence.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read docs/contracts/$CONTRACT.md and note whether the stronger review is required.
3. Run: python tools/work_identity.py gate --contract $CONTRACT
   It passes only if the newest verify record is SOFTWARE PASS on the current work identity, review-stage.md APPROVES that identity and base, and, if the contract requires it, review-design.md APPROVES it and names an Opus reviewer model. If it fails, stop and report each problem. A missing required Opus review is a gate, not an inconvenience: say that the contract is waiting for it and do not suggest a substitute. Any problem about identity means the work changed after verification or review, so verification and every review are void and must be redone.
4. If the gate passes, update TODO.md to SW PASS (not CLOSED, if physical checks remain). TODO.md and docs/evidence/$CONTRACT/ are exempt from the identity.
5. Stage by explicit path only: the paths listed in work_identity (git add <each path>), docs/evidence/$CONTRACT/ and TODO.md. Never -A or ".". Then run: python tools/work_identity.py commit-check --contract $CONTRACT
6. Do not commit. Give me the exact commit command for the human, with the contract ID in the message, and tell me to run python tools/work_identity.py post-commit --contract $CONTRACT right afterward. Agents never commit, push or merge.

Report: identity, gate result, commit-check result, the staged paths, the command for me, the outstanding physical checks, then the HANDOFF block. Next is: the human commits, runs post-commit, reviews and merges, then the owner does the named physical checks.
```

---

## 6. Resume after interruption

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

Work on this contract was interrupted. Rebuild the state from the repo, not from memory or any earlier summary.

1. Run: git branch --show-current. Stop if it differs from BRANCH.
2. Read docs/contracts/TEMPLATE.md, docs/contracts/$CONTRACT.md, TODO.md.
3. Run: git status --short, git diff --stat, git log --oneline main..HEAD. List docs/evidence/$CONTRACT/ and docs/drafts/$CONTRACT/, and read the newest verify.json and any HANDOFF.md.
4. Determine and state which stage this is in: build, awaiting authorization, verify, awaiting stage review, awaiting stronger (Opus) review, awaiting human apply, ready for gate, awaiting human commit, closed, or blocked. Recompute python tools/work_identity.py show --contract $CONTRACT and compare it with the identities in work-identity.json, the newest verify.json and each review: a mismatch means that verification or review is void.
5. Check gates against evidence. A human gate counts as met only if its evidence exists: for example a live file matching a draft and a manifest change, or a recorded verdict naming the current work identity. If evidence is missing, assume the gate is not met, however much time has passed.
6. Uncommitted changes may be yours or someone else's. Do not discard them. If you cannot explain a change, stop and ask.

Report the stage, your evidence for it, what is unverified, and the HANDOFF block. Then continue only if the next step is one you are allowed to do, following the rules of that stage's prompt. Otherwise stop and name the next prompt and who runs it.
```

---

## 7. Blocked or stalled handoff

```text
CONTRACT=SW-01
BRANCH=codex/SW-01

This contract has stopped: the budget is spent, progress stalled, tampering was reported, or a rule blocks the work. Do not try again and do not work around it. Your only job is a precise handoff.

1. Run: git branch --show-current. Report it, even if it differs, and do not switch.
2. Read the contract, the newest docs/evidence/$CONTRACT/verify-*/verify.json, and git status --short, git diff --stat.
3. Write docs/evidence/$CONTRACT/HANDOFF.md containing:
   - Why it stopped, quoting the exact message
   - Attempts used, and the failure fingerprints
   - What was tried and what each attempt showed
   - Exactly what state the tree is in, with committed and uncommitted paths
   - The smallest human action that would unblock it. Name one decision, or the APPLY steps, and how to validate them
   - What you do not know
4. Set this contract to BLOCKED in TODO.md, with a one-line reason.

Do not edit other files. Do not commit; agents never do. Do not mark it done. Do not suggest lowering a threshold, skipping a test, or regenerating the manifest unless the evidence shows that is truly right, and if so, present it as a decision for me, not as a fix.

Finish with the HANDOFF block.
```
