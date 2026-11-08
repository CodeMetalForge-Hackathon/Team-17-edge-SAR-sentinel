#!/usr/bin/env python3
"""HUMAN-ONLY guardrail administration. Agents are blocked from this by the hook and, for commands that
change trust, by a terminal check and a typed confirmation. Those are guardrails, not perfect isolation.

    python tools/guardrail_admin.py status
    python tools/guardrail_admin.py init                  # first baseline, on a reviewed clean tree
    python tools/guardrail_admin.py anchor                # trust a manifest that arrived through a reviewed merge
    python tools/guardrail_admin.py apply-protected PATH... --reason TEXT   # after a human changed protected files
    python tools/guardrail_admin.py add-test DRAFT --contract ID --review FILE
    python tools/guardrail_admin.py authorize-verify ID --attempts N   # one contract, one run of attempts
    python tools/guardrail_admin.py approve-campaign ID                # a whole campaign's pre-approved attempts, reviews, time
    python tools/guardrail_admin.py revoke-campaign ID
    python tools/guardrail_admin.py precommit             # run by .git/hooks/pre-commit, no terminal needed

Git Bash's mintty may report "no terminal". Use PowerShell or Windows Terminal for this tool.
"""
import argparse
import datetime as dt
import fnmatch
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

ID_RE = re.compile(r"^[A-Z]{2,3}-[0-9A-Z]{2}$")


def git(*args, check=True):
    r = subprocess.run(["git", *args], capture_output=True, text=True, encoding="utf-8", errors="replace")
    if check and r.returncode:
        raise SystemExit(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


ROOT = Path(git("rev-parse", "--show-toplevel"))
COMMON = Path(git("rev-parse", "--path-format=absolute", "--git-common-dir"))
GUARD = COMMON / "forge-guardrails"
MANIFEST = ROOT / "tools/protected_manifest.json"
PATHS = ROOT / "tools/protected_paths.json"
TRUST_FILES = {"manifest": "tools/protected_manifest.json", "pins": "tools/protected_pins.json",
               "paths": "tools/protected_paths.json"}


def blob(rel):
    return git("hash-object", f"--path={rel}", rel)


def load(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def save(p, data):
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")


def spec():
    return load(PATHS)


def protected_files(s=None):
    s = s or spec()
    files = [f for f in git("ls-files", "-co", "--exclude-standard").splitlines() if f]
    return sorted(f for f in files if any(fnmatch.fnmatchcase(f, g) for g in s["protected_globs"]))


def build_manifest():
    files = protected_files()
    return {"note": "Created by guardrail_admin.py. Do not edit by hand.",
            "files": {f: blob(f) for f in files if f != "tools/protected_manifest.json"}}


def require_human(phrase):
    """A person at a terminal types the phrase. Non-terminal callers are refused."""
    if not sys.stdin.isatty():
        raise SystemExit("Refused: needs an interactive terminal. Use PowerShell or Windows Terminal.")
    try:
        typed = input(f'Type exactly "{phrase}" to continue: ').strip()
    except EOFError:  # On Windows the null device passes isatty(), then gives end-of-file here.
        raise SystemExit("Refused: no one typed a confirmation. Nothing changed.")
    if typed != phrase:
        raise SystemExit("Not confirmed. Nothing changed.")


def write_anchor():
    GUARD.mkdir(parents=True, exist_ok=True)
    anchor = {k: blob(v) for k, v in TRUST_FILES.items()}
    anchor["anchored_at"] = dt.datetime.now(dt.timezone.utc).isoformat()
    anchor["head"] = git("rev-parse", "HEAD")
    save(GUARD / "anchor.json", anchor)
    return anchor


def log_change(line):
    p = GUARD / "changes.log"
    with open(p, "a", encoding="utf-8") as f:
        f.write(f"{dt.datetime.now(dt.timezone.utc).isoformat()} {line}\n")


def mismatches():
    m = load(MANIFEST)["files"]
    return [f for f, h in m.items() if not (ROOT / f).exists() or blob(f) != h]


def cmd_status(_):
    print("anchor:", "present" if (GUARD / "anchor.json").exists() else "MISSING")
    if MANIFEST.exists():
        bad = mismatches()
        print("manifest entries:", len(load(MANIFEST)["files"]), "| mismatched:", bad or "none")
    else:
        print("manifest: MISSING")
    auth = sorted(p.stem for p in (GUARD / "verify_auth").glob("*.json")) if (GUARD / "verify_auth").exists() else []
    print("verifier authorized for:", auth or "nothing")
    camps = sorted((GUARD / "campaign").glob("*.json")) if (GUARD / "campaign").exists() else []
    for c in camps:
        r = load(c)
        print(f"campaign {r['campaign']}: approved, expires {r['expires_at']}")
    if not camps:
        print("campaigns: none approved")
    print("branch:", git("branch", "--show-current"))
    return 0


def cmd_init(_):
    """First baseline. Refuses a dirty protected tree so an unexplained change cannot become the baseline."""
    if MANIFEST.exists():
        raise SystemExit("Manifest already exists. Use 'anchor' or 'apply-protected'.")
    files = protected_files()
    print(f"{len(files)} protected files will be baselined at HEAD {git('rev-parse', '--short', 'HEAD')}:")
    for f in files:
        print("  ", f)
    dirty = git("status", "--porcelain", "--", *files).splitlines()
    unstaged = [d for d in dirty if not d.startswith("A ") and not d.startswith("M ")]
    if unstaged:
        print("Not committed or staged yet:", *unstaged, sep="\n  ")
    require_human("baseline reviewed")
    save(MANIFEST, build_manifest())
    print("Anchored:", write_anchor())
    print("Now run: git add tools/protected_manifest.json, then commit by explicit path.")
    return 0


def cmd_anchor(_):
    """Use after pulling a merge that changed guardrails. Refuses if files disagree with the manifest."""
    bad = mismatches()
    if bad:
        raise SystemExit("Refused. Protected files differ from the manifest:\n  " + "\n  ".join(bad))
    prev = GUARD / "anchor.json"
    if prev.exists():
        old = load(prev).get("head")
        if old:
            print(git("diff", "--stat", old, "HEAD", "--", "tools/", ".claude/", "tests/", "docs/contracts/", check=False))
    require_human("anchor reviewed")
    print("Anchored:", write_anchor())
    log_change(f"anchor head={git('rev-parse', 'HEAD')}")
    return 0


def cmd_apply_protected(a):
    """After a human edited protected files. Updates only the named entries, never a blanket regenerate."""
    s = spec()
    m = load(MANIFEST)
    print(git("diff", "HEAD", "--", *a.paths, check=False) or "(no diff against HEAD; new or already committed)")
    for p in a.paths:
        if not any(fnmatch.fnmatchcase(p, g) for g in s["protected_globs"]):
            raise SystemExit(f"{p} is not a protected path")
        if not (ROOT / p).exists():
            raise SystemExit(f"{p} does not exist")
    others = [f for f in mismatches() if f not in a.paths]
    if others:
        raise SystemExit("Refused. Other protected files changed without explanation:\n  " + "\n  ".join(others))
    require_human("protected change reviewed")
    for p in a.paths:
        m["files"][p] = blob(p)
    save(MANIFEST, m)
    print("Anchored:", write_anchor())
    log_change(f"apply-protected {a.paths} reason={a.reason!r}")
    print("Commit these paths and tools/protected_manifest.json explicitly.")
    return 0


def cmd_add_test(a):
    """The additive route. Only adds. Fails if any existing manifest entry changed."""
    if not ID_RE.match(a.contract):
        raise SystemExit("bad contract id")
    draft = Path(a.draft)
    if not draft.exists():
        raise SystemExit(f"no such draft: {draft}")
    bad = mismatches()
    if bad:
        raise SystemExit("Refused. Existing protected files changed:\n  " + "\n  ".join(bad))
    sha = hashlib.sha256(draft.read_bytes()).hexdigest()
    review = Path(a.review)
    if not review.exists():
        raise SystemExit("No review record. An independent reviewer must approve first.")
    text = review.read_text(encoding="utf-8")
    if sha not in text or "APPROVE" not in text:
        raise SystemExit(f"Review record must contain APPROVE and this file's sha256: {sha}")
    dest = f"tests/contract/{draft.name}"
    if (ROOT / dest).exists() or dest in load(MANIFEST)["files"]:
        raise SystemExit(f"{dest} already exists. Editing an existing test goes through APPLY.md.")
    r = subprocess.run([sys.executable, str(ROOT / "tools/check_additive.py"), str(draft), "--contract", a.contract])
    if r.returncode:
        raise SystemExit("Mechanical checks failed. Not installed.")
    require_human("add reviewed test")
    (ROOT / dest).parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(draft, ROOT / dest)
    m = load(MANIFEST)
    m["files"][dest] = blob(dest)
    save(MANIFEST, m)
    print("Anchored:", write_anchor())
    log_change(f"add-test {dest} sha256={sha}")
    print(f"Installed {dest}. Add it to this contract's tools/verify_stages file through apply-protected if it must be required.")
    return 0


def cmd_authorize(a):
    if not ID_RE.match(a.contract):
        raise SystemExit("bad contract id")
    if not 1 <= a.attempts <= 8:
        raise SystemExit("attempts must be from 1 to 8")
    require_human(f"authorize {a.contract}")
    save(GUARD / "verify_auth" / f"{a.contract}.json", {"contract": a.contract, "attempts": a.attempts,
         "at": dt.datetime.now(dt.timezone.utc).isoformat()})
    log_change(f"authorize-verify {a.contract} attempts={a.attempts}")
    print(f"Verifier authorized for {a.contract}, up to {a.attempts} attempts (lower of this and the contract's budget).")
    return 0


def cmd_approve_campaign(a):
    """One human approval for a whole campaign: bounded verifier attempts, reviewer sessions, dispatches and time.
    Needed again after any change to the campaign file (its SHA-256 is recorded) or after the time budget expires."""
    cf = ROOT / "tools" / "campaigns" / f"{a.campaign}.json"
    if not cf.exists():
        raise SystemExit(f"no campaign file {cf}")
    spec = load(cf)
    bad = mismatches()
    if bad:
        raise SystemExit("Refused. Protected files differ from the manifest:\n  " + "\n  ".join(bad))
    branch = git("branch", "--show-current")
    if branch != spec["branch"]:
        raise SystemExit(f"on {branch!r}; the campaign runs on {spec['branch']!r}")
    for s in spec["stages"]:
        p = ROOT / "tools" / "verify_stages" / f"{s['id']}.json"
        if not p.exists():
            raise SystemExit(f"stage {s['id']} has no tools/verify_stages file")
        if bool(load(p).get("stronger_review_required", True)) != bool(s["stronger_review"]):
            raise SystemExit(f"stage {s['id']}: stronger_review disagrees with its stage config")
    b = spec["budgets"]
    print(f"Campaign {a.campaign} on {spec['branch']}, base {git('rev-parse', '--short', 'HEAD')}")
    for s in spec["stages"]:
        print(f"  {s['id']}: up to {s['attempts']} verifier attempts, Opus review {'REQUIRED' if s['stronger_review'] else 'not required'}, needs {s.get('needs') or 'nothing'}")
    print(f"Budgets: {b['wall_hours']} h wall time, {b['opus_review_sessions']} Opus review sessions, {b['scout_dispatches']} Haiku scout dispatches, "
          f"{b['retries_per_delegated_task']} retry per delegated task. Reviews must be {spec.get('review_provenance', 'hook')}-recorded.")
    print("Human gates that stay yours:", "; ".join(spec["human_gates"]))
    require_human(f"approve campaign {a.campaign}")
    now = dt.datetime.now(dt.timezone.utc)
    rec = {"campaign": a.campaign, "campaign_sha256": hashlib.sha256(cf.read_bytes()).hexdigest(), "approved_at": now.isoformat(),
           "expires_at": (now + dt.timedelta(hours=b["wall_hours"])).isoformat(), "head_at_approval": git("rev-parse", "HEAD")}
    save(GUARD / "campaign" / f"{a.campaign}.json", rec)
    sp = GUARD / "campaign_state" / f"{a.campaign}.json"
    if not sp.exists():   # a re-approval keeps usage and progress; raising a budget is the campaign file's job
        save(sp, {"base": rec["head_at_approval"], "done": [], "usage": {}})
    log_change(f"approve-campaign {a.campaign} sha256={rec['campaign_sha256']} expires={rec['expires_at']}")
    print(f"Approved until {rec['expires_at']}.")
    return 0


def cmd_revoke_campaign(a):
    require_human(f"revoke campaign {a.campaign}")
    p = GUARD / "campaign" / f"{a.campaign}.json"
    if p.exists():
        p.unlink()
    log_change(f"revoke-campaign {a.campaign}")
    print("Revoked. Verifier authorization for its stages falls back to authorize-verify.")
    return 0


def cmd_precommit(_):
    """Git-level check that also covers tools without Claude Code hooks. --no-verify bypasses it."""
    if not MANIFEST.exists():
        print("[guardrail] no manifest yet, not enforcing")
        return 0
    m = load(MANIFEST)["files"]
    s = spec()
    staged = git("diff", "--cached", "--name-only", "--diff-filter=ACMRD").splitlines()
    anchor = load(GUARD / "anchor.json") if (GUARD / "anchor.json").exists() else {}
    problems = []
    for f in staged:
        name = f.rsplit("/", 1)[-1]
        if name in s["forbidden_names"]:
            problems.append(f"forbidden file staged: {f}")
        if not any(fnmatch.fnmatchcase(f, g) for g in s["protected_globs"]):
            continue
        staged_blob = git("rev-parse", f":{f}", check=False)
        if f in TRUST_FILES.values():
            key = [k for k, v in TRUST_FILES.items() if v == f][0]
            if staged_blob != anchor.get(key):
                problems.append(f"{f} staged but not anchored by a human")
        elif m.get(f) != staged_blob:
            problems.append(f"protected file staged without human apply: {f}")
    if problems:
        print("[guardrail] commit refused:\n  " + "\n  ".join(problems), file=sys.stderr)
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status").set_defaults(fn=cmd_status)
    sub.add_parser("init").set_defaults(fn=cmd_init)
    sub.add_parser("anchor").set_defaults(fn=cmd_anchor)
    p = sub.add_parser("apply-protected")
    p.add_argument("paths", nargs="+")
    p.add_argument("--reason", required=True)
    p.set_defaults(fn=cmd_apply_protected)
    p = sub.add_parser("add-test")
    p.add_argument("draft")
    p.add_argument("--contract", required=True)
    p.add_argument("--review", required=True)
    p.set_defaults(fn=cmd_add_test)
    p = sub.add_parser("authorize-verify")
    p.add_argument("contract")
    p.add_argument("--attempts", type=int, required=True)
    p.set_defaults(fn=cmd_authorize)
    for name, fn in (("approve-campaign", cmd_approve_campaign), ("revoke-campaign", cmd_revoke_campaign)):
        p = sub.add_parser(name)
        p.add_argument("campaign")
        p.set_defaults(fn=fn)
    sub.add_parser("precommit").set_defaults(fn=cmd_precommit)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
