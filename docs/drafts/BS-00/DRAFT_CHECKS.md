# What was checked on these drafts

From the planning pass, Sat 2026-11-07, at commit `69cf3a7`. **This is evidence for drafts, not live acceptance.**
Nothing here is installed. All runs used a `git archive` export or a throwaway clone outside the repo.

## Run, with results

| Check | Result |
|---|---|
| All 17 draft Python files parse. All 12 JSON files load | Pass |
| Pinned values match the code at HEAD, read with a separate copy of the logic | 0 mismatches |
| `protect_paths.py` on synthetic tool calls, including malformed input | 40 of 40 as expected |
| `check_additive.py` on good and bad test files | 13 of 13 as expected |
| `scratch_export.py` in a clone: base recorded, patch and untracked files carried over, overlay placed, refuses output inside the repo | Pass |
| `install_drafts.py`: refuses the wrong branch, copies 36 files, refuses to overwrite | Pass |
| APPLY's exact `git add` list leaves nothing uncommitted | Pass |
| Manifest build: 64 entries, detects a protected edit, ignores a CRLF-only difference | Pass |
| Real `pre-commit` hook: allows the manifest commit and an ordinary commit; refuses a protected edit and `conftest.py` | Pass |
| APPLY 5.1's hook commands, in Windows PowerShell | Printed 2, then 0 |
| Scope logic in merge-first order, using the verifier's git commands | 0 files out of scope, and `pi/zzz.py` detected |
| `test_core.py` and invariants | 6 and 11 pass, **with fake sklearn and joblib and a small runner** |
| Contract tests at HEAD | Counts match the verifier's floors. Red tests fail on missing features, as intended. Some tests already pass: SW-01 3, SW-03 5, SW-02 3, SW-07 1, ML-03 1 |

## Defects these checks found, now fixed in the drafts

| Defect | Fix |
|---|---|
| Validating before BS-00 reached `main` put 105 files out of scope, so APPLY's expected results were wrong | Merge locally first. Push only after validation |
| An install log created during 5.3 would have counted as out of scope | Keep the note outside the repo until section 6 |
| Builders were told not to commit, yet reviewers must name a commit | **Superseded by revision 2.** Candidate commits are gone: reviews name a work identity (hash of the uncommitted bytes), and a human makes one explicit-path commit after verify PASS and APPROVE |
| `python3` on this laptop is a Microsoft Store stub, and pre-commit tried it first | `python` is tried first |
| Worktrees have no `.venv` for the prompts' commands | APPLY creates one per worktree |
| A non-ASCII byte in test output could crash the verifier on Windows's default encoding | Decode as UTF-8 with replacement |
| A null stdin passes the terminal check on Windows and ended in a raw `EOFError` | Clean refusal. Nothing changes either way |

## Revision 2 (model routing, token economy, work identity, local acceptance)

Scratch checks only; nothing installed, no application code changed.

| Check | Result |
|---|---|
| `work_identity.py` in a throwaway git repo (28 assertions): deterministic; edit, new, delete and stage-config change each alter the identity; evidence and `TODO.md` are excluded; staged index refused; SHA-256 of bytes differs from git blob id; gate fails without verify or reviews, fails when the required Opus review is absent, rejects a Sonnet model for it, rejects a REJECT, another identity, another verify record, a non-PASS verify, and a post-review edit; `commit-check` accepts exact explicit staging and rejects a stray file and a post-staging edit; `post-commit` passes | 28 of 28 |
| `protect_paths.py` with `git commit -m x`, `git status`, `work_identity.py show` | blocked, allowed, allowed |
| All draft JSON and Python still parse; stage configs carry `stronger_review_required` matching the contracts | Pass |

**Not run for revision 2:** `verify.py` with the new identity calls (never executed, by rule; APPLY 5.3 rows l to p cover it), `maxTurns` and
`Agent(scout)` agent frontmatter inside a real Claude Code session, the `git commit` deny rule in a real session (APPLY 5.2),
and any enforcement of model routing: environment overrides can change a subagent's model and nothing here stops that.

## Revision 3 (reconciliation with teammate commit `7a9e315`)

Scratch checks on a `git archive` export of `7a9e315` with the draft tests overlaid, `sklearn`, `joblib` and `pytest` replaced by stubs and a plain runner
(draft evidence, not live acceptance; model-dependent tests cannot be judged under the stub).

| Check | Result |
|---|---|
| `tests/test_core.py` and the invariants on the integrated code | 6 and 11 pass (before the new launcher invariant) |
| SW-03 draft tests against the integrated reset | 7 pass, 5 fail. The failures are the intended open work (dedupe, `forge/status`, detector `reset()`) plus two test defects now fixed: the dashboard keeps a `(reset)` log marker, and the sender is `pi/reset.py`, not `demo_ctl.py` |
| New launcher invariant against the unmodified `run_pi.sh` | Fails by design until APPLY 3.2b. The text check was not run on a real clone |
| Stage floors recomputed from the real test counts | Existing floors reproduced exactly before the changes; now 18 invariants+core and the SW-03 and SW-06 additions |

**Not run for revision 3:** pytest itself, `bash -n run_pi.sh`, the launcher, the reset on MQTT, the laptop workflow, anything on the Pi.

## Revision 4 (campaign autonomy)

Scratch only, in throwaway git repos; nothing installed, `verify.py` not executed.

| Check | Result |
|---|---|
| Identity, gate, commit-check, post-commit (earlier suite, unchanged) | 28 of 28 |
| Campaign suite: agent gate allows only named agents on their own models, counts scout and Opus use, refuses past the budget, refuses on an expired or edited campaign file; SubagentStop hook records reviewer verdicts and ignores other agents; `gate` refuses relayed reviews and a Sonnet-recorded design review; guarded `commit` makes one explicit-path commit, advances state, leaves a clean tree, and refuses (leaving nothing staged) when work changed or no campaign covers the stage | 22 of 22 |
| Campaign stage list against the stage configs and `stronger_review_required` | consistent (checked by script) |
| Stage floors recomputed from real test counts (17 baseline; SW-00 adds 2 tests; SW-03 14; SW-06 10) | done |

**Not run for revision 4:** `verify.py` (campaign auth, scope base, stub rejection are read and parsed only), `approve-campaign` (needs a typed phrase at a terminal),
the Agent-tool and SubagentStop hook payload fields in a real Claude Code session (the field names used are as documented, unverified here), pytest, anything on hardware.
If the SubagentStop payload lacks the reviewer's final message, no review is recorded and the campaign gate stops the stage (fail closed).

## Not run

| Not run | Why | What covers it |
|---|---|---|
| **`tools/verify.py`** | Forbidden without pre-approval. It was parsed and read only, and has never executed | APPLY 5.3 |
| **pytest itself** | Not installed here, and no installs were allowed. Collection, the `-c` config and the test-count floors are unproven | APPLY 3.5, 5.3 (d) |
| **Real scikit-learn and joblib** | Same | APPLY 3.5, 5.3 (d) |
| Hook and deny rules inside a real Claude Code session | Installing them was forbidden | APPLY 5.2 |
| The typed-phrase prompts at a real terminal | Needs a person. Internals were called directly instead | APPLY 4.1, 5.3 (b) |
| Stall detection on real pytest output | Needs the verifier | APPLY 5.3 (j). If it prints `FAIL` rather than `STALLED`, that is a draft defect |
| Linux or Raspberry Pi behaviour | Windows laptop only | Check before relying on the hook there |
| Anything on hardware | Out of scope | HW contracts |
