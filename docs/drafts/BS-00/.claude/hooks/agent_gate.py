#!/usr/bin/env python3
"""PreToolUse guardrail for the Agent tool. DRAFT. Exit 2 blocks.

Keeps model routing and budgets enforceable for a coordinator that dispatches subagents on its own:
only the named agents, each with its expected model, within the approved campaign's counters.
Fails closed. The counters live in .git/forge-guardrails/campaign_state (a human-approved campaign owns the limits).
"""
import json
import os
import sys

EXPECTED = {"scout": "haiku", "stage-reviewer": "sonnet", "test-design-reviewer": "opus"}


def block(msg):
    print(f"Blocked by project guardrail: {msg}", file=sys.stderr)
    return 2


def main():
    try:
        data = json.load(sys.stdin)
    except ValueError:
        return block("agent gate could not read its input (failing closed)")
    ti = data.get("tool_input") or {}
    st = (ti.get("subagent_type") or "").strip()
    if st not in EXPECTED:
        return block(f"subagent {st!r} is not allowed. Allowed: {sorted(EXPECTED)}. The coordinator builds directly; there is no builder subagent")
    model = (ti.get("model") or "").lower()
    if model and EXPECTED[st] not in model:
        return block(f"{st} must run on {EXPECTED[st]}, not {model!r}. Models are explicit")
    root = os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or os.getcwd()
    os.chdir(root)
    sys.path.insert(0, os.path.join(root, "tools"))
    try:
        import work_identity as wi
        camp = wi.active_campaign()
    except Exception as e:  # noqa: BLE001 - fail closed
        return block(f"cannot read campaign state ({e!r}). Failing closed")
    if not camp:
        if st == "scout":
            return 0
        return block(f"{st} needs an approved campaign on this branch. Without one, a human starts reviewers in a fresh session")
    rec, spec = camp
    if wi.expired(rec):
        return block("campaign authorization expired. Human gate: re-approve the campaign (time budget)")
    state = wi.load_state(spec["campaign"])
    use = state.setdefault("usage", {})
    b = spec["budgets"]
    key, limit = {"scout": ("scout_dispatches", b["scout_dispatches"]),
                  "test-design-reviewer": ("opus_reviews", b["opus_review_sessions"]),
                  "stage-reviewer": ("stage_reviews", b.get("stage_review_sessions", 20))}[st]
    if use.get(key, 0) >= limit:
        return block(f"{key} budget of {limit} is spent. Human gate: a budget increase needs re-approval. Record the blocker and continue other authorized work")
    use[key] = use.get(key, 0) + 1
    wi.save_state(spec["campaign"], state)
    return 0


if __name__ == "__main__":
    sys.exit(main())
