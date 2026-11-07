#!/usr/bin/env python3
"""Identity of a contract's UNCOMMITTED work, plus the pre-commit gate. Read-only except `show --out`.

    python tools/work_identity.py show --contract SW-01 [--out docs/evidence/SW-01/work-identity.json]
    python tools/work_identity.py sha256 FILE...           # SHA-256 of the file bytes (not a git blob id)
    python tools/work_identity.py gate --contract SW-01    # verify PASS + required APPROVEs name this identity
    python tools/work_identity.py commit-check --contract SW-01   # after staging explicit paths
    python tools/work_identity.py post-commit --contract SW-01    # after the commit
    python tools/work_identity.py commit --contract SW-01         # campaign only: gate, stage, check, commit, advance

DRAFT. Never executed against a real contract during planning.

The identity covers: base commit and branch; every changed path except docs/evidence/<ID>/ and TODO.md, each with
state, SHA-256 of its file bytes and its git blob id (`git hash-object`, a DIFFERENT value, kept under its own name);
and SHA-256 of the contract, stage config, manifest, pins and path list. It does NOT cover ignored files, file mode
bits, installed packages, environment variables or hardware. A matching identity proves equal bytes at two moments.
It does not prove a reviewer read them or was independent.

Exit codes: 0 ok, 1 gate/check problems, 2 cannot compute (for example the index is not clean).
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ID_RE = re.compile(r"^[A-Z]{2,3}-[0-9A-Z]{2}$")
VERDICT_RE = re.compile(r"^VERDICT:\s*(APPROVE|REJECT)\s+identity=([0-9a-f]{64})\s+base=([0-9a-f]{40})\s*$", re.M)
MODEL_RE = re.compile(r"^REVIEWER_MODEL:\s*(\S.*?)\s*$", re.M)
VERIFY_SHA_RE = re.compile(r"^VERIFY_RECORD_SHA256:\s*([0-9a-f]{64})\s*$", re.M)


class IdentityError(Exception):
    pass


def _git(*args, check=True):
    r = subprocess.run(["git", *args], capture_output=True, cwd=ROOT)
    if check and r.returncode:
        raise IdentityError(f"git {' '.join(args)}: {r.stderr.decode('utf-8', 'replace').strip()}")
    return r.stdout


def _git_text(*args, check=True):
    return _git(*args, check=check).decode("utf-8", "replace").strip()


_r = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True, encoding="utf-8")
ROOT = Path(_r.stdout.strip()) if _r.returncode == 0 else Path.cwd()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sha256_or_absent(rel):
    p = ROOT / rel
    return sha256_file(p) if p.is_file() else "absent"


def excluded(rel, cid):
    return rel == "TODO.md" or rel.startswith(f"docs/evidence/{cid}/")


def _entries():
    """(XY, path) for each changed path, ignoring ignored files. Staged renames also yield the old path as deleted."""
    out = _git("status", "--porcelain=v1", "-z", "--untracked-files=all").decode("utf-8", "surrogateescape")
    parts = out.split("\0")
    entries, i = [], 0
    while i < len(parts):
        e = parts[i]
        i += 1
        if len(e) < 4:
            continue
        xy, path = e[:2], e[3:]
        entries.append((xy, path))
        if xy[0] in "RC" and i < len(parts):
            orig = parts[i]
            i += 1
            if xy[0] == "R" and orig:
                entries.append(("D ", orig))
    return entries


def _file_record(rel):
    p = ROOT / rel
    in_head = subprocess.run(["git", "cat-file", "-e", f"HEAD:{rel}"], cwd=ROOT, capture_output=True).returncode == 0
    if os.path.islink(p):
        target = os.readlink(p)
        return {"path": rel, "state": "modified" if in_head else "new", "symlink_target": target,
                "sha256": hashlib.sha256(target.encode()).hexdigest(), "git_blob_oid": None}
    if not p.exists():
        return {"path": rel, "state": "deleted", "sha256": None, "git_blob_oid": None}
    return {"path": rel, "state": "modified" if in_head else "new", "sha256": sha256_file(p),
            "git_blob_oid": _git_text("hash-object", f"--path={rel}", rel)}


def compute(cid, staged_ok=False):
    if not ID_RE.match(cid):
        raise IdentityError("bad contract id")
    entries = [(xy, p) for xy, p in _entries() if not excluded(p, cid)]
    staged = sorted({p for xy, p in entries if xy[0] not in " ?!"})
    if staged and not staged_ok:
        raise IdentityError("the index is not clean for reviewed paths. Unstage them (git restore --staged <path>) "
                            "before verification or review: " + ", ".join(staged))
    paths = sorted({p for _, p in entries})
    files = [_file_record(p) for p in paths]
    tracked = [f["path"] for f in files if f["state"] in ("modified", "deleted")]
    patch = b""
    if tracked:
        patch = _git("-c", "core.autocrlf=false", "diff", "--binary", "--no-color", "HEAD", "--", *tracked)
    body = {
        "schema": 1,
        "contract": cid,
        "branch": _git_text("branch", "--show-current"),
        "base_commit": _git_text("rev-parse", "HEAD"),
        "files": files,
        "context": {
            "contract_md_sha256": sha256_or_absent(f"docs/contracts/{cid}.md"),
            "stage_config_sha256": sha256_or_absent(f"tools/verify_stages/{cid}.json"),
            "manifest_sha256": sha256_or_absent("tools/protected_manifest.json"),
            "pins_sha256": sha256_or_absent("tools/protected_pins.json"),
            "paths_sha256": sha256_or_absent("tools/protected_paths.json"),
        },
    }
    canon = json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    out = dict(body)
    out["identity_sha256"] = hashlib.sha256(canon).hexdigest()
    out["tracked_patch_sha256_informational"] = hashlib.sha256(patch).hexdigest()
    out["staged_paths"] = staged
    return out


HOOK_RE = re.compile(r"^RECORDED_BY:\s*hook\b", re.M)


def guard_dir():
    return Path(_git_text("rev-parse", "--path-format=absolute", "--git-common-dir")) / "forge-guardrails"


def active_campaign():
    """(approval record, campaign spec) for an approved, unmodified campaign whose branch is the current one, else None."""
    d = guard_dir() / "campaign"
    if not d.is_dir():
        return None
    branch = _git_text("branch", "--show-current")
    for f in sorted(d.glob("*.json")):
        try:
            rec = json.loads(f.read_text(encoding="utf-8"))
            cf = ROOT / "tools" / "campaigns" / f"{rec['campaign']}.json"
            if not cf.is_file() or sha256_file(cf) != rec.get("campaign_sha256"):
                continue   # the campaign file changed after approval: not approved any more
            spec = json.loads(cf.read_text(encoding="utf-8"))
        except (OSError, ValueError, KeyError):
            continue
        if spec.get("branch") == branch:
            return rec, spec
    return None


def campaign_for(cid):
    c = active_campaign()
    return c if c and any(s["id"] == cid for s in c[1]["stages"]) else None


def expired(rec):
    import datetime as dt
    return dt.datetime.now(dt.timezone.utc) > dt.datetime.fromisoformat(rec["expires_at"])


def state_path(campaign):
    return guard_dir() / "campaign_state" / f"{campaign}.json"


def load_state(campaign):
    p = state_path(campaign)
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {"base": None, "done": [], "usage": {}}


def save_state(campaign, st):
    p = state_path(campaign)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(st, indent=2, sort_keys=True), encoding="utf-8")


def stronger_required(cid):
    p = ROOT / "tools" / "verify_stages" / f"{cid}.json"
    try:
        cfg = json.loads(p.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return True  # unknown means required
    return cfg.get("stronger_review_required", True) is not False


def latest_verify(cid):
    found = sorted((ROOT / "docs" / "evidence" / cid).glob("verify-*/verify.json"))
    return found[-1] if found else None


def _check_review(cid, fname, ident, vsha, need_opus):
    problems = []
    p = ROOT / "docs" / "evidence" / cid / fname
    if not p.exists():
        return [f"{fname} is missing" + (" (a required Opus review cannot be replaced by another model)" if need_opus else "")]
    text = p.read_text(encoding="utf-8", errors="replace")
    verdicts = VERDICT_RE.findall(text)
    if not verdicts:
        return [f"{fname}: no 'VERDICT: APPROVE|REJECT identity=<sha256> base=<commit>' line"]
    verdict, wid, base = verdicts[-1]
    if verdict != "APPROVE":
        problems.append(f"{fname}: verdict is {verdict}")
    if wid != ident["identity_sha256"]:
        problems.append(f"{fname}: approves identity {wid[:12]}..., current work is {ident['identity_sha256'][:12]}...")
    if base != ident["base_commit"]:
        problems.append(f"{fname}: approves base {base[:12]}, HEAD is {ident['base_commit'][:12]}")
    m = MODEL_RE.search(text)
    if not m:
        problems.append(f"{fname}: no REVIEWER_MODEL line")
    elif need_opus and "opus" not in m.group(1).lower():
        problems.append(f"{fname}: REVIEWER_MODEL is {m.group(1)!r}; this review must be Opus")
    camp = campaign_for(cid)
    if camp and camp[1].get("review_provenance", "hook") == "hook" and not HOOK_RE.search(text):
        problems.append(f"{fname}: not recorded by the SubagentStop hook (a relayed review is not accepted in a campaign)")
    v = VERIFY_SHA_RE.search(text)
    if not v:
        problems.append(f"{fname}: no VERIFY_RECORD_SHA256 line (review must follow the verifier)")
    elif v.group(1) != vsha:
        problems.append(f"{fname}: cites a different verify record than the latest one")
    return problems


def gate(cid, staged_ok=False):
    ident = compute(cid, staged_ok=staged_ok)
    problems = []
    camp = campaign_for(cid)
    if camp and expired(camp[0]):
        problems.append("campaign authorization has expired. A human must re-approve (time budget)")
    vpath = latest_verify(cid)
    vsha = None
    if not vpath:
        problems.append("no verify record under docs/evidence/%s/verify-*/" % cid)
    else:
        vsha = sha256_file(vpath)
        rec = json.loads(vpath.read_text(encoding="utf-8"))
        wid = rec.get("work_identity") or {}
        if rec.get("result") != "SOFTWARE PASS":
            problems.append(f"latest verify result is {rec.get('result')!r}, not SOFTWARE PASS")
        if wid.get("before") != ident["identity_sha256"] or wid.get("after") != ident["identity_sha256"]:
            problems.append("verified work identity differs from the current work (changed since verification)")
        if rec.get("head") != ident["base_commit"]:
            problems.append("HEAD moved since verification")
    problems += _check_review(cid, "review-stage.md", ident, vsha, need_opus=False)
    if stronger_required(cid):
        problems += _check_review(cid, "review-design.md", ident, vsha, need_opus=True)
    return ident, problems


def _lines(raw):
    return [x for x in raw.decode("utf-8", "surrogateescape").split("\0") if x]


def commit_check(cid):
    ident, problems = gate(cid, staged_ok=True)
    expected = {f["path"] for f in ident["files"]}
    staged = set(_lines(_git("diff", "--cached", "--name-only", "-z")))
    for p in sorted(expected - staged):
        problems.append(f"reviewed path not staged: {p}")
    for p in sorted(staged - expected):
        if not excluded(p, cid):
            problems.append(f"staged but not part of the reviewed work: {p}")
    unstaged = set(_lines(_git("diff", "--name-only", "-z")))
    for p in sorted(unstaged):
        if not excluded(p, cid):
            problems.append(f"changed after staging (differs from index): {p}")
    for f in ident["files"]:
        if f["git_blob_oid"] and _git_text("rev-parse", f":{f['path']}", check=False) != f["git_blob_oid"]:
            problems.append(f"staged blob differs from reviewed content: {f['path']}")
    return ident, problems


def post_commit(cid):
    problems = []
    review = ROOT / "docs" / "evidence" / cid / "review-stage.md"
    rec_path = ROOT / "docs" / "evidence" / cid / "work-identity.json"
    v = VERDICT_RE.findall(review.read_text(encoding="utf-8", errors="replace")) if review.exists() else []
    if not v:
        raise IdentityError("cannot read the approved identity and base from review-stage.md")
    want_id, base = v[-1][1], v[-1][2]
    if not rec_path.exists():
        raise IdentityError(f"docs/evidence/{cid}/work-identity.json is missing from the commit")
    recorded = json.loads(rec_path.read_text(encoding="utf-8"))
    if recorded.get("identity_sha256") != want_id:
        problems.append("work-identity.json does not match the approved identity")
    if _git_text("rev-parse", "HEAD^") != base:
        problems.append("HEAD's parent is not the approved base commit")
    for f in recorded["files"]:
        present = subprocess.run(["git", "cat-file", "-e", f"HEAD:{f['path']}"], cwd=ROOT, capture_output=True).returncode == 0
        if f["state"] == "deleted":
            if present:
                problems.append(f"deleted path still present in the commit: {f['path']}")
        elif _git_text("rev-parse", f"HEAD:{f['path']}", check=False) != f["git_blob_oid"]:
            problems.append(f"committed blob differs from reviewed content: {f['path']}")
    reviewed = {f["path"] for f in recorded["files"]}
    for p in sorted(_lines(_git("diff-tree", "--no-commit-id", "--name-only", "-r", "-z", "HEAD"))):
        if p not in reviewed and not excluded(p, cid):
            problems.append(f"commit contains an unreviewed path: {p}")
    return {"identity_sha256": want_id, "base_commit": base, "files": recorded["files"]}, problems


def do_commit(cid):
    """Campaign-only guarded commit: gate, stage explicit paths, commit-check, commit (hooks run), post-commit, advance state."""
    camp = campaign_for(cid)
    if not camp:
        raise IdentityError("no approved campaign covers this stage on this branch. A human commits (see TEMPLATE.md)")
    rec, spec = camp
    st = load_state(spec["campaign"])
    needs = next(s for s in spec["stages"] if s["id"] == cid).get("needs", [])
    missing = [n for n in needs if n not in st["done"]]
    if missing:
        raise IdentityError(f"stage needs {missing} committed first")
    ident, problems = gate(cid)
    if problems:
        return ident, problems
    paths = [f["path"] for f in ident["files"]]
    extra = [p for p in (f"docs/evidence/{cid}", "TODO.md") if (ROOT / p).exists()]
    _git("add", "--", *paths, *extra)
    ident, problems = commit_check(cid)
    if problems:
        _git("reset", "-q", check=False)
        return ident, problems
    msg = (f"{cid}: verified and independently reviewed\n\nWork identity {ident['identity_sha256']}\nBase {ident['base_commit']}\n\n"
           "Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>")
    r = subprocess.run(["git", "commit", "-q", "-m", msg], cwd=ROOT, capture_output=True, text=True)
    if r.returncode:
        _git("reset", "-q", check=False)
        return ident, ["git commit refused: " + (r.stderr or r.stdout).strip()[:400]]
    st["done"].append(cid)                     # the commit exists now, so state advances even if the audit below complains
    st["base"] = _git_text("rev-parse", "HEAD")
    save_state(spec["campaign"], st)
    try:
        _, problems = post_commit(cid)
    except IdentityError as e:
        problems = [f"committed, but the post-commit audit could not run: {e}"]
    return ident, problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("show")
    p.add_argument("--contract", required=True)
    p.add_argument("--out")
    p = sub.add_parser("sha256")
    p.add_argument("files", nargs="+")
    for name in ("gate", "commit-check", "post-commit", "commit"):
        sub.add_parser(name).add_argument("--contract", required=True)
    a = ap.parse_args()
    try:
        if a.cmd == "sha256":
            for f in a.files:
                print(f"{sha256_file(f)}  {f}")
            return 0
        if a.cmd == "show":
            ident = compute(a.contract)
            text = json.dumps(ident, indent=2) + "\n"
            if a.out:
                Path(a.out).parent.mkdir(parents=True, exist_ok=True)
                Path(a.out).write_text(text, encoding="utf-8", newline="\n")
            print(text, end="")
            return 0
        fn = {"gate": gate, "commit-check": commit_check, "post-commit": post_commit, "commit": do_commit}[a.cmd]
        ident, problems = fn(a.contract)
    except IdentityError as e:
        print(f"CANNOT COMPUTE: {e}", file=sys.stderr)
        return 2
    print(f"identity {ident['identity_sha256']}  base {ident['base_commit']}  files {len(ident['files'])}")
    if problems:
        print(f"{a.cmd.upper()} FAILED:")
        for x in problems:
            print("  -", x)
        return 1
    print(f"{a.cmd.upper()} OK (mechanical only; it proves matching bytes, not review quality)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
