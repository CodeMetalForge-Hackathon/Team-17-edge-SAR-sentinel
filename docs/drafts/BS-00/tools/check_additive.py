#!/usr/bin/env python3
"""Mechanical checks for one NEW test file on the additive route. Read-only; anyone may run it.

    python tools/check_additive.py docs/drafts/SW-02/tests/test_sw02_extra.py --contract SW-02

Passing means the file looks safe to add. It does not mean the test is good: that needs a reviewer.
AST checks can be evaded by a determined author, so a human still reads the file.
"""
import argparse
import ast
import re
import sys
from pathlib import Path

BANNED_IMPORTS = {"subprocess", "socket", "urllib", "requests", "http", "shutil", "ctypes", "multiprocessing", "importlib"}
BANNED_CALLS = {"exec", "eval", "compile", "__import__", "system", "popen", "remove", "unlink", "rmtree", "rename",
                "replace", "setattr", "delattr"}
BANNED_PYTEST = {"skip", "xfail", "importorskip", "exit", "skipif"}
BANNED_NAMES = {"collect_ignore", "collect_ignore_glob", "pytest_plugins", "pytestmark", "__test__"}
# Modules whose names a test file must not take, because it could replace them on import.
RESERVED = {"config", "bus", "brain", "policy", "features", "anomaly", "dashboard", "simulate", "lidar_change", "lidar_node",
            "node_bridge", "hailo_hook", "drone_dispatch", "numpy", "pytest", "sklearn", "joblib", "conftest"}


def check(path, contract):
    problems = []
    p = Path(path)
    want = rf"^test_{contract.lower().replace('-', '')}_[a-z0-9_]+\.py$"
    if not re.match(want, p.name):
        problems.append(f"file name must match {want}")
    if p.stem in RESERVED:
        problems.append(f"file name shadows module {p.stem}")
    try:
        tree = ast.parse(p.read_text(encoding="utf-8"))
    except (SyntaxError, OSError, UnicodeError) as e:
        return [f"cannot parse: {e}"]

    tests = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name.startswith("test_")]
    if not tests:
        problems.append("no test functions at module level")
    for t in tests:
        if not any(isinstance(n, (ast.Assert, ast.Raise)) or (isinstance(n, ast.Attribute) and n.attr == "raises") for n in ast.walk(t)):
            problems.append(f"{t.name} has no assert")
        body = [n for n in t.body if not (isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant))]
        if all(isinstance(n, ast.Pass) for n in body):
            problems.append(f"{t.name} is empty")
        if any(isinstance(n, ast.Assert) and isinstance(n.test, ast.Constant) and n.test.value for n in ast.walk(t)):
            problems.append(f"{t.name} asserts a constant True")

    for n in tree.body:   # module-level writes and hooks run at collection time
        if isinstance(n, ast.FunctionDef) and n.name.startswith("pytest_"):
            problems.append(f"defines pytest hook {n.name}")
        if isinstance(n, ast.Assign):
            for t in n.targets:
                if isinstance(t, ast.Attribute):
                    problems.append(f"module-level attribute write at line {n.lineno} (could change a threshold at import)")
                if isinstance(t, ast.Name) and t.id in BANNED_NAMES:
                    problems.append(f"sets {t.id}")
        if isinstance(n, ast.AugAssign) and isinstance(n.target, ast.Attribute):
            problems.append(f"module-level attribute change at line {n.lineno}")

    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            for a in n.names:
                if a.name.split(".")[0] in BANNED_IMPORTS:
                    problems.append(f"imports {a.name}")
        elif isinstance(n, ast.ImportFrom):
            if (n.module or "").split(".")[0] in BANNED_IMPORTS:
                problems.append(f"imports from {n.module}")
        elif isinstance(n, ast.Call):
            f = n.func
            name = f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else "")
            if name in BANNED_CALLS:
                problems.append(f"calls {name}() at line {n.lineno}")
            if name == "open" and len(n.args) >= 2:
                mode = n.args[1]
                if not (isinstance(mode, ast.Constant) and set(str(mode.value)) <= set("rbt")):
                    problems.append(f"open() for writing at line {n.lineno}")
            if name in ("write_text", "write_bytes", "mkdir", "touch"):
                problems.append(f"file write {name}() at line {n.lineno}")
        elif isinstance(n, ast.Attribute):
            base = n.value
            if isinstance(base, ast.Name) and base.id == "pytest" and n.attr in BANNED_PYTEST:
                problems.append(f"uses pytest.{n.attr} at line {n.lineno}")
            if isinstance(base, ast.Attribute) and base.attr == "mark" and n.attr in BANNED_PYTEST:
                problems.append(f"uses a skip or xfail mark at line {n.lineno}")
            if isinstance(base, ast.Name) and base.id == "sys" and n.attr == "modules":
                problems.append(f"touches sys.modules at line {n.lineno}")
            if isinstance(base, ast.Name) and base.id == "sys" and n.attr == "path":
                pass  # adding pi/ to the path is how existing tests import
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--contract", required=True)
    a = ap.parse_args()
    problems = check(a.file, a.contract)
    if problems:
        print("ADDITIVE CHECK FAILED:")
        for p in problems:
            print(" -", p)
        return 1
    print("ADDITIVE CHECK PASSED (mechanical checks only; a reviewer must still approve)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
