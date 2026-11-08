#!/usr/bin/env python3
"""Contract verifier. One command defines automated "done" for a software contract.

    python tools/verify.py --contract SW-01

DRAFT. Never executed during planning. BS-00's APPLY.md runs it first against deliberate failures.

It will not rewrite its manifest, lower a threshold, repair a check, or treat a missing dependency as a pass.
There is intentionally no flag for any of those. SOFTWARE PASS is not physical validation.

Exit codes: 0 SOFTWARE PASS, 1 FAIL, 2 BLOCKED (missing dependency/config/authorization), 3 STALLED,
            4 BUDGET EXHAUSTED, 5 TAMPER (protected change), 6 WRONG BRANCH
"""
import argparse
import ast
import datetime as dt
import fnmatch
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ID_RE = re.compile(r"^[A-Z]{2,3}-[0-9A-Z]{2}$")
PASS, FAIL, BLOCKED, STALLED, BUDGET, TAMPER, WRONG_BRANCH = 0, 1, 2, 3, 4, 5, 6
NAMES = {0: "SOFTWARE PASS", 1: "FAIL", 2: "BLOCKED", 3: "STALLED", 4: "BUDGET EXHAUSTED", 5: "TAMPER", 6: "WRONG BRANCH"}


def git(*args, check=True):
    r = subprocess.run(["git", *args], capture_output=True, text=True, encoding="utf-8", errors="replace")
    if check and r.returncode:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


ROOT = Path(git("rev-parse", "--show-toplevel"))
COMMON = Path(git("rev-parse", "--path-format=absolute", "--git-common-dir"))
GUARD = COMMON / "forge-guardrails"


def blob_hash(rel):
    """Hash as git would store it. CRLF checkouts on Windows then match Linux and the commit."""
    return git("hash-object", f"--path={rel}", rel)


def all_files():
    out = git("ls-files", "-co", "--exclude-standard")
    return [p for p in out.splitlines() if p]


def glob_match(path, patterns):
    return any(fnmatch.fnmatchcase(path, p) for p in patterns)


class Stop(Exception):
    def __init__(self, code, msg):
        super().__init__(msg)
        self.code, self.msg = code, msg


def load_json(p):
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def check_anchor():
    """The manifest is trusted only if a human anchored this exact file in this clone."""
    anchor_file = GUARD / "anchor.json"
    if not anchor_file.exists():
        raise Stop(BLOCKED, "no anchor. A human must run: python tools/guardrail_admin.py init (or anchor)")
    anchor = load_json(anchor_file)
    for key, rel in (("manifest", "tools/protected_manifest.json"), ("pins", "tools/protected_pins.json"),
                     ("paths", "tools/protected_paths.json")):
        if not (ROOT / rel).exists():
            raise Stop(TAMPER, f"{rel} is missing")
        if blob_hash(rel) != anchor.get(key):
            raise Stop(TAMPER, f"{rel} differs from the human-anchored version. A human must review it.")


def check_manifest():
    """Every listed file unchanged, nothing unlisted in protected areas, nothing shadowing."""
    manifest = load_json(ROOT / "tools/protected_manifest.json")
    spec = load_json(ROOT / "tools/protected_paths.json")
    problems = []
    for rel, want in manifest["files"].items():
        if not (ROOT / rel).exists():
            problems.append(f"missing: {rel}")
        elif blob_hash(rel) != want:
            problems.append(f"changed: {rel}")
    files = all_files()
    for f in files:
        name = f.rsplit("/", 1)[-1]
        if f != "tools/protected_manifest.json" and f not in manifest["files"] and glob_match(f, spec["protected_globs"]):
            problems.append(f"unlisted file in protected area: {f}")
        if name in spec["forbidden_names"]:
            problems.append(f"forbidden file: {f}")
        if name.endswith(".py") and name[:-3] in spec["forbidden_module_shadows"] and f.startswith(("pi/", "tests/")):
            problems.append(f"shadows a module: {f}")
    if problems:
        raise Stop(TAMPER, "protected files:\n  " + "\n  ".join(problems))


def literal_constant(path, name):
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == name:
                    return ast.literal_eval(node.value)
    raise KeyError(name)


def check_pins():
    """Value-level protection: measured values in config.py can change, these cannot."""
    pins = load_json(ROOT / "tools/protected_pins.json")
    problems = []
    for rel, values in pins["pins"].items():
        for name, want in values.items():
            try:
                got = literal_constant(rel, name)
            except (KeyError, ValueError, SyntaxError, OSError) as e:
                problems.append(f"{rel}:{name} unreadable ({e!r})")
                continue
            if got != want:
                problems.append(f"{rel}:{name} is {got!r}, pinned {want!r}")
    if problems:
        raise Stop(TAMPER, "pinned values:\n  " + "\n  ".join(problems))


def check_scope(stage_cfg, base_ref=None):
    base = git("merge-base", base_ref or stage_cfg["base"], "HEAD")
    committed = git("diff", "--name-only", base, "HEAD").splitlines()
    dirty = git("status", "--porcelain", "--untracked-files=all").splitlines()
    touched = set(committed) | {re.sub(r'^"|"$', "", d[3:].split(" -> ")[-1]) for d in dirty}
    outside = sorted(p for p in touched if p and not glob_match(p, stage_cfg["allowed_paths"]))
    if outside:
        raise Stop(FAIL, "outside this contract's allowed paths:\n  " + "\n  ".join(outside))
    return sorted(touched)


def junit_counts(path):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == "testsuite" else list(root)
    t = {k: sum(int(s.get(k, 0)) for s in suites) for k in ("tests", "failures", "errors", "skipped")}
    xfail = 0
    for s in suites:
        for case in s.iter("testcase"):
            for sk in case.findall("skipped"):
                if "xfail" in (sk.get("type") or "") or "xfail" in (sk.get("message") or "").lower():
                    xfail += 1
    t["xfail"] = xfail
    return t


def clean_env():
    env = dict(os.environ)
    for k in ("PYTHONPATH", "PYTEST_ADDOPTS", "PYTEST_PLUGINS", "PYTHONSTARTUP", "PYTHONHOME"):
        env.pop(k, None)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    return env


def run_stage(stage, evidence):
    """A stage passes only with exit 0, at least min_tests tests, and zero skips, xfails or errors."""
    xml = evidence / f"{stage['name']}.xml"
    log = evidence / f"{stage['name']}.log"
    tests = stage["tests"]
    for t in tests:
        if not (ROOT / t).exists():
            return {"name": stage["name"], "ok": False, "why": f"required test file missing: {t}", "tail": ""}
    cmd = [sys.executable, "-m", "pytest", "-q", "-c", "tools/pytest_verify.ini", "--rootdir", ".", "--noconftest",
           "-p", "no:cacheprovider", f"--junitxml={xml}", *tests]
    try:
        r = subprocess.run(cmd, cwd=ROOT, env=clean_env(), capture_output=True, text=True, encoding="utf-8",
                           errors="replace", timeout=stage.get("timeout_s", 180))
    except subprocess.TimeoutExpired:
        return {"name": stage["name"], "ok": False, "why": "timeout", "tail": ""}
    log.write_text(r.stdout + "\n" + r.stderr, encoding="utf-8")
    tail = "\n".join((r.stdout + r.stderr).splitlines()[-40:])
    if not xml.exists():
        return {"name": stage["name"], "ok": False, "why": f"no junit output, rc={r.returncode}", "tail": tail}
    c = junit_counts(xml)
    why = []
    if r.returncode:
        why.append(f"pytest rc={r.returncode}")
    if c["tests"] < stage.get("min_tests", 1):
        why.append(f"only {c['tests']} tests, floor {stage.get('min_tests', 1)}")
    if c["skipped"] or c["xfail"]:
        why.append(f"skips or xfails in a required stage ({c['skipped']})")
    if c["errors"]:
        why.append(f"{c['errors']} errors")
    return {"name": stage["name"], "ok": not why, "why": "; ".join(why), "counts": c, "tail": tail}


def fingerprint(results):
    """Normalized so timing, paths and addresses do not hide a repeat of the same failure."""
    parts = []
    for r in results:
        if not r["ok"]:
            t = re.sub(r"0x[0-9a-fA-F]+|\d+(\.\d+)?|[A-Za-z]:\\\S+|/\S+", "#", r.get("tail", ""))
            parts.append(r["name"] + "|" + r.get("why", "") + "|" + t)
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()[:16]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--contract", required=True)
    a = ap.parse_args()
    cid = a.contract
    if not ID_RE.match(cid):
        print("bad contract id")
        return BLOCKED

    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    evidence = ROOT / "docs" / "evidence" / cid / f"verify-{stamp}"
    record = {"contract": cid, "time_utc": stamp, "python": sys.version.split()[0], "executable": sys.executable,
              "stages": [], "result": None, "physical_outstanding": []}
    code, msg, results = BLOCKED, "", []
    state = None
    state_file = GUARD / "state" / f"{cid}.json"
    try:
        cfg_path = ROOT / "tools" / "verify_stages" / f"{cid}.json"
        if not cfg_path.exists():
            raise Stop(BLOCKED, f"no stage config {cfg_path.name}: this contract has no automated acceptance")
        cfg = load_json(cfg_path)
        record["physical_outstanding"] = cfg.get("physical_outstanding", [])
        record["head"] = git("rev-parse", "HEAD")

        import work_identity   # tools/ is sys.path[0] when this file is run as a script
        camp = work_identity.campaign_for(cid)   # an approved, unmodified campaign on this branch, or None
        branch = git("branch", "--show-current")
        record["branch"] = branch
        want_branch = camp[1]["branch"] if camp else cfg["branch"]
        if branch != want_branch:
            raise Stop(WRONG_BRANCH, f"on {branch!r}, contract requires {want_branch!r}")

        scope_base = None
        if camp:
            rec_c, spec_c = camp
            if work_identity.expired(rec_c):
                raise Stop(BLOCKED, "campaign approval expired. A human must re-approve (time budget).")
            stage_c = next(s for s in spec_c["stages"] if s["id"] == cid)
            auth = {"attempts": stage_c["attempts"]}
            scope_base = work_identity.load_state(spec_c["campaign"]).get("base")
            record["campaign"] = spec_c["campaign"]
        else:
            auth_file = GUARD / "verify_auth" / f"{cid}.json"
            if not auth_file.exists():
                raise Stop(BLOCKED, "not authorized. A human must run: "
                                    f"python tools/guardrail_admin.py authorize-verify {cid} --attempts N (or approve-campaign)")
            auth = load_json(auth_file)

        state = load_json(state_file) if state_file.exists() else {"attempts": 0, "fingerprints": []}
        budget = min(int(auth["attempts"]), int(cfg.get("attempt_budget", 4)))
        record["attempt"] = state["attempts"] + 1
        record["attempt_budget"] = budget
        if state["attempts"] >= budget:
            raise Stop(BUDGET, f"{state['attempts']} of {budget} attempts used. Stop and hand off.")

        check_anchor()
        check_manifest()   # before the run
        check_pins()
        record["touched"] = check_scope(cfg, scope_base)

        try:
            wid0 = work_identity.compute(cid)
        except work_identity.IdentityError as e:
            raise Stop(BLOCKED, f"cannot identify the work under test: {e}")
        record["work_identity"] = {"before": wid0["identity_sha256"], "after": None,
                                   "base_commit": wid0["base_commit"], "files": [f["path"] for f in wid0["files"]]}

        missing = [m for m in cfg.get("required_modules", []) if importlib.util.find_spec(m) is None]
        if missing:
            raise Stop(BLOCKED, f"missing modules {missing}. Not a pass. A human installs them into the core .venv.")
        stubs = []
        for m in cfg.get("required_modules", []):
            origin = getattr(importlib.util.find_spec(m), "origin", None)
            if origin and origin not in ("built-in", "frozen"):
                op = Path(origin).resolve()
                if (op == ROOT or ROOT in op.parents) and ".venv" not in op.parts:
                    stubs.append(f"{m} -> {op}")
        if stubs:
            raise Stop(TAMPER, "required modules resolve to files inside the repository (stubs are diagnostic only, never acceptance): " + "; ".join(stubs))

        evidence.mkdir(parents=True, exist_ok=True)
        state["attempts"] += 1  # counted once it can actually run
        state_file.parent.mkdir(parents=True, exist_ok=True)
        state_file.write_text(json.dumps(state), encoding="utf-8")

        for stage in cfg["stages"]:
            res = run_stage(stage, evidence)
            results.append(res)
            print(f"[{'ok' if res['ok'] else 'FAIL'}] {stage['name']} {res.get('counts', '')} {res.get('why', '')}")
        record["stages"] = results

        check_manifest()   # after the run: tests must not alter protected files
        check_pins()
        check_scope(cfg, scope_base)
        try:
            wid1 = work_identity.compute(cid)
        except work_identity.IdentityError as e:
            raise Stop(TAMPER, f"work could not be re-identified after the run: {e}")
        record["work_identity"]["after"] = wid1["identity_sha256"]
        if wid1["identity_sha256"] != wid0["identity_sha256"]:
            raise Stop(TAMPER, "the work under test changed during verification. Any result is void.")

        if all(r["ok"] for r in results):
            code, msg = PASS, "all acceptance stages passed"
        else:
            fp = fingerprint(results)
            record["fingerprint"] = fp
            state["fingerprints"].append(fp)
            state_file.write_text(json.dumps(state), encoding="utf-8")
            window = int(cfg.get("stall_window", 2))
            recent = state["fingerprints"][-window:]
            if len(recent) == window and len(set(recent)) == 1:
                code, msg = STALLED, f"same failure {window} times in a row. Stop and hand off."
            else:
                code, msg = FAIL, "acceptance failed"
    except Stop as s:
        code, msg = s.code, s.msg
    except Exception as e:  # fail closed
        code, msg = BLOCKED, f"verifier error, not a pass: {e!r}"

    record["result"] = NAMES[code]
    record["message"] = msg
    evidence.mkdir(parents=True, exist_ok=True)
    (evidence / "verify.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(f"\n{NAMES[code]}: {msg}")
    if record["physical_outstanding"]:
        print("PHYSICAL CHECKS STILL OUTSTANDING (not covered by this result):")
        for p in record["physical_outstanding"]:
            print(f"  - {p}")
    print(f"Evidence: {evidence.relative_to(ROOT)}")
    return code


if __name__ == "__main__":
    sys.exit(main())
