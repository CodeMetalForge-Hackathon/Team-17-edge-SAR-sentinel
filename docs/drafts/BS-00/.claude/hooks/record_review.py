#!/usr/bin/env python3
"""SubagentStop hook. DRAFT. Records a reviewer subagent's final message into docs/evidence/<ID>/ itself, so the coordinator
cannot write or alter a review. Never blocks (exit 0). Written against the hook payload as documented
(agent_type, agent_id, last_assistant_message); NOT verified against the installed Claude Code version. If the fields are
absent the review is simply not recorded, and the campaign gate then refuses the stage (fail closed, human gate).

A reviewer must include a line `CONTRACT: <ID>`. The file begins with `RECORDED_BY: hook ...`, which work_identity.py gate requires.
"""
import datetime as dt
import json
import os
import re
import sys
from pathlib import Path

TARGET = {"stage-reviewer": "review-stage.md", "test-design-reviewer": "review-design.md"}


def main():
    try:
        data = json.load(sys.stdin)
    except ValueError:
        return 0
    at = data.get("agent_type") or ""
    if at not in TARGET:
        return 0
    msg = data.get("last_assistant_message") or ""
    if not msg and data.get("agent_transcript_path"):
        try:   # best effort: last assistant text block in the transcript
            for line in Path(data["agent_transcript_path"]).read_text(encoding="utf-8").splitlines()[::-1]:
                rec = json.loads(line)
                content = (rec.get("message") or {}).get("content")
                if rec.get("type") == "assistant" and isinstance(content, list):
                    msg = "".join(c.get("text", "") for c in content if c.get("type") == "text")
                    if msg:
                        break
        except (OSError, ValueError):
            pass
    m = re.search(r"^CONTRACT:\s*([A-Z]{2,3}-[0-9A-Z]{2})\s*$", msg, re.M)
    if not m or not re.search(r"^VERDICT:", msg, re.M):
        return 0
    root = Path(os.environ.get("CLAUDE_PROJECT_DIR") or data.get("cwd") or os.getcwd())
    ev = root / "docs" / "evidence" / m.group(1)
    ev.mkdir(parents=True, exist_ok=True)
    now = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    body = f"RECORDED_BY: hook agent_type={at} agent_id={data.get('agent_id', 'unknown')} at={now}\n{msg.rstrip()}\n"
    (ev / "reviews").mkdir(exist_ok=True)
    (ev / "reviews" / f"{at}-{now}.md").write_text(body, encoding="utf-8", newline="\n")   # history
    (ev / TARGET[at]).write_text(body, encoding="utf-8", newline="\n")                      # latest wins
    return 0


if __name__ == "__main__":
    sys.exit(main())
