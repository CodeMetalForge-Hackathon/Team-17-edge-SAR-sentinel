#!/usr/bin/env python3
"""HUMAN-RUN. Copies the BS-00 drafts to their live paths. Run from the repo root on branch codex/BS-00.

It refuses to overwrite an existing file, prints every action, and appends to .gitignore once.
It does not commit, baseline, or install the git hook; APPLY.md does those as separate steps.
"""
import shutil
import subprocess
import sys
from pathlib import Path

DRAFT = Path(__file__).resolve().parent
ROOT = Path(subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True, check=True).stdout.strip())
BRANCH = subprocess.run(["git", "branch", "--show-current"], capture_output=True, text=True, check=True).stdout.strip()

SKIP = {"APPLY.md", "DRAFT_CHECKS.md", "install_drafts.py", "gitignore.additions"}


def main():
    if BRANCH != "codex/BS-00":
        print(f"Refused: on {BRANCH!r}. Create and switch to codex/BS-00 first.")
        return 1
    items = [p for p in sorted(DRAFT.rglob("*")) if p.is_file() and "__pycache__" not in p.parts
             and p.name not in SKIP and p.relative_to(DRAFT).parts[0] != "git-hooks"]
    conflicts = [p for p in items if (ROOT / p.relative_to(DRAFT)).exists()]
    if conflicts:
        print("Refused. These live files already exist:")
        for p in conflicts:
            print("  ", p.relative_to(DRAFT))
        return 1
    for p in items:
        dst = ROOT / p.relative_to(DRAFT)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(p, dst)
        print("installed", p.relative_to(DRAFT).as_posix())
    gi = ROOT / ".gitignore"
    add = (DRAFT / "gitignore.additions").read_text(encoding="utf-8")
    cur = gi.read_text(encoding="utf-8")
    if "# Added by BS-00" not in cur:
        gi.write_text(cur.rstrip() + "\n" + add, encoding="utf-8", newline="\n")
        print("appended .gitignore")
    print(f"\n{len(items)} files copied. Nothing is committed or active as a baseline yet. Continue with APPLY.md.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
