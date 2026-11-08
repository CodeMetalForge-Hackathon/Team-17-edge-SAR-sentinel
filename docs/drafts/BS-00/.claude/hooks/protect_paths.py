#!/usr/bin/env python3
"""PreToolUse guardrail for Claude Code. DRAFT. Exit 2 blocks the tool call.

What it enforces: Claude's file-edit tools cannot change protected paths, and shell commands that plainly
mutate them or do git-admin work are refused.

What it cannot enforce: a script that writes files indirectly, encoded commands, a human's actions, or any
other agent tool such as Codex. The verifier's manifest check and the git pre-commit hook catch those later.
"""
import fnmatch
import json
import os
import re
import sys

FILE_TOOLS = {"Write", "Edit", "MultiEdit", "NotebookEdit"}
SHELL_TOOLS = {"Bash", "PowerShell"}

# Mirrors tools/protected_paths.json, kept here so this still works if that file is damaged.
PROTECTED = ["tests/test_core.py", "tests/test_guardrail_invariants.py", "tests/contract/*", "tools/*", "tools/verify_stages/*",
             ".claude/*", ".claude/hooks/*", ".claude/agents/*", "CLAUDE.md", "AGENTS.md", "docs/contracts/*",
             "requirements.txt", "requirements-dev.txt", ".gitignore", "pi/drone_dispatch.py",
             "conftest.py", "*/conftest.py", "pytest.ini", "tox.ini", "setup.cfg", "pyproject.toml", ".coveragerc",
             ".git/*"]

PATH_TOKENS = ["tests/test_core", "tests/test_guardrail", "tests/contract", "tools/", ".claude/", "CLAUDE.md", "AGENTS.md",
               "docs/contracts", "requirements", ".gitignore", "drone_dispatch", "conftest", "pytest.ini", "pyproject.toml",
               ".git/", "forge-guardrails"]

MUTATORS = [r">>?", r"\brm\b", r"\bmv\b", r"\bcp\b", r"\bsed\s+-i", r"\btee\b", r"\bdd\b", r"\btruncate\b", r"\bchmod\b",
            r"\bln\b", r"\btouch\b", r"\bgit\s+(checkout|restore|apply|am|rm|mv)\b", r"Remove-Item", r"Set-Content", r"Add-Content",
            r"Out-File", r"Move-Item", r"Copy-Item", r"New-Item", r"Clear-Content", r"Rename-Item", r"\.write\(", r"write_text",
            r"write_bytes", r"\bopen\(.*['\"][wax]"]

ALWAYS_BLOCK = [
    (r"\bgit\s+push\b", "agents never push"),
    (r"\bgit\s+commit\b", "agents never commit: a human commits after verify, review and work_identity.py commit-check"),
    (r"\bgit\s+(merge|rebase|cherry-pick|revert|pull)\b", "the human merges"),
    (r"\bgit\s+reset\b", "no resets"),
    (r"\bgit\s+stash\b", "no stashing teammates' work"),
    (r"\bgit\s+(switch|checkout|branch\s+-[dDmM]|worktree)\b", "the human manages branches"),
    (r"\bgit\s+add\s+(-A|--all|\.|\*|-u)(\s|$)", "stage explicit paths only"),
    (r"--no-verify|\bgit\s+commit\b.*\s-n(\s|$)", "hooks cannot be skipped"),
    (r"\bgit\s+config\b|core\.hooksPath|\bgit\s+update-ref\b|\bgit\s+filter", "no git config or ref edits"),
    (r"guardrail_admin", "human-only tool"),
    (r"--fly\b|--force\b|DRONE_DRY_RUN\s*=\s*False", "flight and force flags are human-only"),
    (r"\b(pip|pip3|uv|conda)\s+(install|uninstall)|-m\s+pip\b|\bapt(-get)?\s+install", "installs are human-only"),
    (r"\b(curl|wget|Invoke-WebRequest|iwr|huggingface-cli|hf\s+download|hf_hub_download|snapshot_download)\b", "downloads are human-only"),
    (r"\b(avrdude|arduino-cli\s+upload)\b", "flashing is human-only"),
    (r"disableAllHooks", "hooks cannot be disabled"),
]


def block(msg):
    print(f"Blocked by project guardrail: {msg}. See docs/contracts/TEMPLATE.md. Write a draft under docs/drafts/<ID>/ or hand off.",
          file=sys.stderr)
    return 2


def rel_path(path, root):
    p = os.path.normcase(os.path.abspath(path if os.path.isabs(path) else os.path.join(root, path)))
    r = os.path.normcase(os.path.abspath(root))
    try:
        rel = os.path.relpath(p, r)
    except ValueError:
        return None
    return None if rel.startswith("..") else rel.replace("\\", "/")


def main():
    try:
        data = json.load(sys.stdin)
    except ValueError:
        return block("hook could not read its input (failing closed)")

    tool = data.get("tool_name", "")
    ti = data.get("tool_input") or {}
    root = os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or os.getcwd()

    if tool in FILE_TOOLS:
        target = ti.get("file_path") or ti.get("notebook_path") or ""
        rel = rel_path(target, root)
        if rel is None:
            return 0  # outside this project, for example the scratchpad
        low = rel.lower()
        for pat in PROTECTED:
            if fnmatch.fnmatch(low, pat.lower()):
                return block(f"{rel} is protected")
        name = low.rsplit("/", 1)[-1]
        if low.startswith(("pi/", "tests/")) and name in {"numpy.py", "pytest.py", "sklearn.py", "joblib.py", "json.py", "os.py", "sys.py"}:
            return block(f"{rel} would shadow a module")
        return 0

    if tool in SHELL_TOOLS:
        cmd = ti.get("command") or ""
        for pat, why in ALWAYS_BLOCK:
            if re.search(pat, cmd, re.I):
                return block(why)
        # tools/verify.py is not listed here on purpose: it refuses to run without human authorization.
        if any(t.lower() in cmd.lower() for t in PATH_TOKENS) and any(re.search(m, cmd) for m in MUTATORS):
            return block("this command looks like it changes a protected path")
    return 0


if __name__ == "__main__":
    sys.exit(main())
