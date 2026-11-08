# BS-00 APPLY: install guardrails

**For a human.** About 30 to 45 minutes. An inspection found no verifier, manifest, hook, deny rules, agent definitions or
template in this checkout, so everything here is new.

**Baseline: teammate commit `7a9e315`** (`run_pi.sh`, `pi/reset.py`, reset handlers, `FORGE_MQTT_HOST`). Teammates report it works on the Pi; this file does not rely on that.

**Current state: DRAFTED, not installed.** See [DRAFT_CHECKS.md](DRAFT_CHECKS.md) for what was and was not tried.
`tools/verify.py` has **never been executed**, by rule. Section 5 is its first run.

**Minimal path (everything else in this file is detail or optional hardening).** (1) 3.1 to 3.7 install and commit. (2) 4.1 to 4.4 baseline, hook, restart.
(3) Section 5: 5.1, 5.2, and 5.3 rows a to e, l, m and p, plus 5.4, as the acceptance record; the other rows are worth doing if time allows and are named in `DRAFT_CHECKS.md`.
(4) Section 6: local acceptance commit, then the **one-time campaign approval** (6.2). After that the coordinator works without you except at the gates listed in
`docs/contracts/DEMO-1.md`. You make no application edits by hand.

**Two separate things.** *Local acceptance* (sections 3 to 6) installs and checks the guardrails and merges them into
your **local** `main`. You can start real work from there without pushing. *Publishing* (section 7) is a later,
optional human `git push` that distributes the accepted guardrails to teammates. Agents never push or merge.

Commands below are for PowerShell at the repo root, on Windows. Use `.venv/bin/python` elsewhere.

## 1. What each piece actually enforces

| Mechanism | Stops | Does not stop |
|---|---|---|
| **Deny rules** (`.claude/settings.json`) | Claude Code's file tools on listed paths; listed shell commands; recognized file commands and redirections | A Python or other script writing files indirectly (per Claude Code's own docs). Other tools, like Codex. You |
| **PreToolUse hook** (`protect_paths.py`) | The same again, plus many mutating shell patterns and git-admin commands. Exit 2 blocks. `onFailure: block` means a hook that cannot run blocks instead of allowing | Obfuscated or indirect commands. It is pattern matching. Other tools. You |
| **Agent definitions** | Limits a subagent's tool list when that subagent is used | Anything if a session does not use them. Reviewers' `Bash` is limited by plan mode, not by a sandbox |
| **Git pre-commit** | Committing a protected change that a human did not anchor, from any tool | `--no-verify` (the hook refuses it for Claude, not for others). Nothing is stopped before commit |
| **Verifier + manifest** | Calls a change TAMPER after the fact, before and after the run, whichever tool made it | Prevention. It runs only when called. An attacker with shell access could edit `.git/forge-guardrails` |
| **Work identity** (`work_identity.py`) | Lets `gate` and `commit-check` refuse a commit unless verify PASS and the required APPROVEs name the exact uncommitted bytes. Deny rules and the hook also stop Claude's `git commit` | A reviewer who did not really review, a forged review file, anyone running `git commit` in a terminal without running `commit-check`, ignored files, file modes, the environment. Nothing runs it automatically |
| **`guardrail_admin` terminal check** | Scripts and non-interactive agents | An agent given a real terminal, or a person |
| **AGENTS.md** | Nothing by force. Instructions | A tool that ignores it |

These are **practical guardrails against mistakes and shortcuts, not perfect isolation.** Claude Code's OS-level
sandbox would be stronger; check whether it supports your platform before relying on it (it is not part of this plan).
For Codex, only pre-commit and the verifier apply.

## 2. Protected zone and why

| Path | Why |
|---|---|
| `tests/test_core.py`, `tests/test_guardrail_invariants.py`, `tests/contract/*` | They define "done" |
| `tools/*`, including `verify.py`, the manifest, pins, stage configs | The checks must not edit themselves |
| `.claude/settings.json`, `.claude/hooks/*`, `.claude/agents/*`, `CLAUDE.md`, `AGENTS.md` | Agents must not grant themselves authority |
| `docs/contracts/*` | Contracts cannot expand their own scope |
| `requirements*.txt`, `.gitignore` | Stops heavy dependencies entering core, or files hiding from the check |
| `pi/drone_dispatch.py` | Whole file, because `--force`, arming and motion live here |
| `conftest.py`, `pytest.ini`, `tox.ini`, `setup.cfg`, `pyproject.toml` | Forbidden anywhere: they could change test collection |

**Deliberately not whole-file protected:**

| Path | Protection | Why |
|---|---|---|
| `pi/config.py` | **Value pins** for dry-run, height, max leg, hover, radii, cooldown, change thresholds, confidence | Measured positions and offsets change constantly on the bench. Whole-file locking would create friction for no safety gain |
| `pi/policy.py` | Value pins for `LOUD_SPIKE`, `VIB_STRUCTURAL`, plus invariant tests for hazard priority | Contracts legitimately change reason text |
| `pi/anomaly.py` | Value pins for `Z_THRESH`, `IF_QUANTILE`, `IF_MARGIN` | SW-01 and SW-06 must edit it |
| `pi/features.py` | Value pin for `FEATURE_NAMES` | SW-01 adds a version constant |
| `run_pi.sh`, `pi/reset.py`, `mac/*` | Contract scope check only. The launcher's flight flag is guarded by SW-00's contract test (no `--force`), and SW-00 makes the one-line fix | Extended by SW-03 and SW-06 |
| Rest of `pi/`, `arduino/`, README | Contract scope check only | This is where the work is |

**Retuning a pinned value on the bench** is normal: a human edits it, edits `tools/protected_pins.json`, and runs
`guardrail_admin.py apply-protected tools/protected_pins.json --reason "..."`. Pins are checked by the verifier only,
not by the hook.

## 3. Install

**3.1** Confirm you have read `PLAN.md`, and that nobody has uncommitted work you would disturb. This pass's files are
uncommitted on `Rohan`. `Rohan` is at the teammates' commit `7a9e315`; local `main` is still at `69cf3a7`.

```bash
git status --short
```

```bash
git log -3 --oneline
```

Bring local `main` up to the integrated baseline. The verifier measures scope against `main`, so without this the teammates' files
would look out of scope. This only moves a branch pointer forward; nothing is pushed.

```bash
git merge-base --is-ancestor main 7a9e315
```

```bash
git branch -f main 7a9e315
```

The first command prints nothing and succeeds only if `main` is an ancestor of `7a9e315`. If it fails, stop.

**3.2** Create the branch. Your uncommitted planning files come with you.

```bash
git switch -c codex/BS-00
```

**3.3** Copy the drafts.

```bash
python docs/drafts/BS-00/install_drafts.py
```

**3.4** Make the core environment. This downloads ordinary PyPI packages.

```bash
python -m venv .venv
```

```bash
.venv\Scripts\python -m pip install -r requirements-dev.txt
```

**3.5** Check the baseline tests with what you just installed.

```bash
.venv\Scripts\python -m pytest -q tests/test_core.py tests/test_guardrail_invariants.py
```

Expect 17 passed (6 core and 11 invariants), with the **real** scikit-learn and joblib. A stubbed run is diagnostic only and does not count. **If not, stop.** Do not baseline a failing state.

**3.6** Read what you are about to trust.

```bash
git status --short
```

```bash
git diff -- .gitignore
```

**3.7** Commit it, with explicit paths.

```bash
git add PLAN.md TODO.md docs tools tests/test_guardrail_invariants.py tests/contract .claude CLAUDE.md AGENTS.md requirements-dev.txt .gitignore
```

```bash
git commit -m "BS-00: planning package and guardrails (human reviewed)"
```

## 4. Establish the baseline

The rule: **a baseline is only created on a state you have just reviewed, and never over an unexplained change.**

**4.1** Create it. This asks you to type a phrase, so use PowerShell or Windows Terminal, not Git Bash.

```bash
.venv\Scripts\python tools\guardrail_admin.py init
```

**4.2** Install the git hook. This file is not tracked by git.

```bash
Copy-Item docs\drafts\BS-00\git-hooks\pre-commit .git\hooks\pre-commit
```

**4.3** Commit the manifest.

```bash
git add tools/protected_manifest.json
```

```bash
git commit -m "BS-00: protected baseline"
```

**4.4** Restart your Claude Code session so it loads the new settings. The `/hooks` menu lists configured hooks.

## 5. Validate (the verifier's first real run)

These are pre-approved for BS-00 only. **Keep results in a note outside the repo for now.** A new file in the tree
during 5.3 would count as out of scope. Section 6 turns the note into `docs/evidence/BS-00/install-log.md`.

**5.1 Hook, directly.** Expect exit code 2 with a message for the first, and 0 for the second.

```bash
'{"tool_name":"Edit","tool_input":{"file_path":"tests/test_core.py"}}' | python .claude\hooks\protect_paths.py; $LASTEXITCODE
```

```bash
'{"tool_name":"Edit","tool_input":{"file_path":"pi/brain.py"}}' | python .claude\hooks\protect_paths.py; $LASTEXITCODE
```

**5.2 Hook, in real use.** In a new Claude Code session, ask it to add a comment to `tests/test_core.py`, and then
to run `git push`, and then `git commit`. All three must be blocked. **If they are not, the hook is not active. Stop and fix it.**

**5.3 Verifier.** The verifier measures scope against `main`, so BS-00 must be in **local** `main` first, or every
BS-00 file counts as out of scope. Merge locally and **do not push**. Section 9 undoes this if validation fails.
First write down where `main` is now, for that undo:

```bash
git rev-parse main
```

```bash
git switch main
```

```bash
git merge --no-ff codex/BS-00 -m "Merge BS-00 for validation"
```

```bash
git switch -c codex/SW-01
```

Below, **V** means `.venv\Scripts\python tools\verify.py --contract SW-01`. Run the rows in order.

| # | Do | Expect |
|---|---|---|
| a | V | `BLOCKED`, not authorized |
| b | `.venv\Scripts\python tools\guardrail_admin.py authorize-verify SW-01 --attempts 4` | Asks for a phrase, then confirms |
| c | `python tools\verify.py --contract SW-01`, using **global** Python, which lacks pytest and sklearn on this laptop | `BLOCKED`, missing modules. Not PASS. No attempt counted |
| d | V | `FAIL`: `regression` ok, `acceptance` red, because SW-01 is not built. Attempt 1 |
| e | Add a comment to `tests/test_core.py`, then V | `TAMPER`. Then `git restore tests/test_core.py` |
| f | Change `EVENT_COOLDOWN_S` in `pi/config.py`, then V | `TAMPER`, pinned value. Then `git restore pi/config.py` |
| g | Create an empty `pi/zzz.py`, then V | `FAIL`, outside allowed paths. No attempt counted. Then delete it |
| h | Create an empty `conftest.py`, then V | `TAMPER`, forbidden file. Then delete it |
| i | `git switch main`, then V, then `git switch codex/SW-01` | `WRONG BRANCH` |
| j | V | `STALLED`: same failure as (d). Attempt 2 |
| k | V, three more times | `STALLED` for attempts 3 and 4, then `BUDGET EXHAUSTED` |
| l | Open the newest `docs\evidence\SW-01\verify-*\verify.json` | `work_identity.before` equals `work_identity.after`, 64 hex characters |
| m | `.venv\Scripts\python tools\work_identity.py show --contract SW-01` | JSON with `"files": []` and an `identity_sha256` equal to row l's |
| n | Create `pi\zzz.py` with one line, run m again, change the line, run m again, delete the file, run m again | Three runs differ from each other and from the empty identity (the first two list `pi/zzz.py` as `new`); after deletion it equals the empty identity again |
| o | `.venv\Scripts\python tools\work_identity.py sha256 tools\verify.py` and `git hash-object tools/verify.py` | Two different 64-hex and 40-hex values. Never interchangeable |
| p | `.venv\Scripts\python tools\work_identity.py gate --contract SW-01` | `GATE FAILED`, exit 1: latest verify result is not SOFTWARE PASS, and review files missing (including the required Opus review) |

Then clean up, and remove what the test created:

```bash
git switch main
```

```bash
git branch -D codex/SW-01
```

```bash
Remove-Item -Recurse -Force .git\forge-guardrails\state, .git\forge-guardrails\verify_auth, docs\evidence\SW-01
```

**5.4 Pre-commit.** Add a comment to `tests/test_core.py`, try `git add` and `git commit` on it. Expect refusal.
Then unstage and restore:

```bash
git restore --staged --worktree tests/test_core.py
```

**If a check behaves differently, that is a defect in the draft.** Fix it by hand, record it, then
`guardrail_admin.py apply-protected <file> --reason "..."`. Never reword the expected result to match.

## 6. Finish

**Only a human does this.** You are on local `main`. Save your results note as
`docs/evidence/BS-00/install-log.md` and commit it. **That commit is local acceptance. You do not need to push to
begin work.** Worktrees made from local `main` have everything.

```bash
git add docs/evidence/BS-00/install-log.md
```

```bash
git commit -m "BS-00: install evidence"
```

Your own anchor and hook already exist in this clone (sections 4.1 and 4.2).

### 6.2 Approve the DEMO-1 campaign (once)

The campaign runs on its own branch in its own worktree. A worktree needs its own `.venv`. Use PowerShell or Windows Terminal for the typed phrase.

```bash
git worktree add ..\T17-DEMO-1 -b codex/DEMO-1 main
```

```bash
python -m venv ..\T17-DEMO-1\.venv
```

```bash
..\T17-DEMO-1\.venv\Scripts\python -m pip install -r ..\T17-DEMO-1\requirements-dev.txt
```

```bash
cd ..\T17-DEMO-1
```

```bash
.venv\Scripts\python tools\guardrail_admin.py approve-campaign DEMO-1
```

It prints every stage, its attempts, the Opus review budget, the time limit and the gates that stay with you, then asks you to type `approve campaign DEMO-1`.
Check it with `.venv\Scripts\python tools\guardrail_admin.py status`: it must list DEMO-1 with an expiry about 14 hours ahead. Start Claude Code in `..\T17-DEMO-1`
(model Sonnet) and use the campaign prompt in `docs/prompts/PROMPT_PACK.md`. Changing the campaign file later voids this approval and needs a new one.

**Starting any other single contract by hand.** A worktree does not share `.venv`, so it gets its own. The prompts expect it there.

```bash
git worktree add ..\T17-SW-01 -b codex/SW-01 main
```

```bash
python -m venv ..\T17-SW-01\.venv
```

```bash
..\T17-SW-01\.venv\Scripts\python -m pip install -r ..\T17-SW-01\requirements-dev.txt
```

**Authorizing a verifier run** for a contract outside a campaign:

```bash
.venv\Scripts\python tools\guardrail_admin.py authorize-verify SW-01 --attempts 4
```

**Committing finished work outside a campaign** (a human; inside a campaign the coordinator uses `work_identity.py commit`). After verify PASS and the independent APPROVEs, and after
`work_identity.py gate` and `commit-check` pass (prompt 5b), run the commit command that prompt gives you, then:

```bash
.venv\Scripts\python tools\work_identity.py post-commit --contract SW-01
```

## 7. Publish to teammates (optional, later, human only)

Not needed for you to start work. Do this when the team agrees to receive the guardrails. It pushes everything on
local `main` that is not on `origin/main`, so check that first.

```bash
git log --oneline origin/main..main
```

```bash
git push origin main
```

**Every teammate, in their own clone,** after finishing or committing their own work:

```bash
git pull origin main
```

```bash
python -m venv .venv
```

```bash
.venv\Scripts\python -m pip install -r requirements-dev.txt
```

```bash
.venv\Scripts\python tools\guardrail_admin.py anchor
```

```bash
Copy-Item docs\drafts\BS-00\git-hooks\pre-commit .git\hooks\pre-commit
```

Anchors and the hook live in `.git`, per clone, so each person trusts the baseline for themselves. Worktrees of a
clone share them.

## 8. If you are running out of time

At 45 minutes, keep sections 3 and 4 plus deny rules, and from section 5 keep only the local merge and 5.3 (b), (d)
and (e), then commit locally as in section 6. Write what you skipped in `TODO.md`. Do not start tuning the hook.

## 9. Recovery

| Problem | Do |
|---|---|
| Section 5 failed after the local merge | Nothing was pushed. Save your note, then `git switch main` and `git reset --hard <hash you wrote down before the merge in 5.3>`, which drops only the local merge since `codex/BS-00` keeps its commits. Do not use `origin/main` for this once local `main` has accepted work that was never pushed. Fix on `codex/BS-00`, then repeat from 4.4. Check `git status` first, because this discards uncommitted edits to tracked files |
| Hook blocks everything | A human renames `.claude/settings.json`, fixes it, renames it back, restarts the session. Record it |
| Python not found by the hook on another machine | `onFailure: block` will block. Change `"command"` to the right interpreter for that clone, by hand. On Linux use `python3` |
| Verifier says TAMPER and you know why | Review the diff. `git restore` the file, or if intended, `guardrail_admin.py apply-protected <path> --reason "..."` |
| Verifier says TAMPER and you **don't** know why | **Stop.** `git diff`, `git log`. Ask the team. Never run `anchor` or `apply-protected` to make it go away |
| Fresh clone says no anchor | `guardrail_admin.py anchor`, after looking at what you pulled |
| Merged a guardrail change from a teammate | Review it, then `anchor` |
| Attempts used up | The contract stops. A human decides whether to reset: delete `.git\forge-guardrails\state\<ID>.json`, and record why in `TODO.md` |
| Remove guardrails entirely | A human deletes `.claude/settings.json` and `.git\hooks\pre-commit`. Say so in `TODO.md`. Nothing checked afterwards counts as verified by it |
