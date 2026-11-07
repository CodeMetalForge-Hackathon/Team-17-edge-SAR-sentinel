#!/usr/bin/env python3
"""Build an isolated scratch copy to try protected drafts without touching live files.

    python tools/scratch_export.py --contract SW-01 --impl-paths pi/simulate.py pi/anomaly.py \
        --overlay docs/drafts/SW-01/OVERLAY.json --out C:/temp/scratch-SW-01

1. git archive HEAD (with autocrlf off, so patches apply cleanly). HEAD excludes uncommitted work, so:
2. Apply your uncommitted changes under --impl-paths as a patch, and copy untracked files under those paths.
3. Copy protected drafts onto their target paths from OVERLAY.json: {"docs/drafts/<ID>/x.py": "tests/x.py"}.
4. Write SCRATCH_RECORD.json with the base commit and the identity of the patch and each draft.

It does not run anything. Run the existing tests unchanged inside the output folder, within the contract's budget.
Never run tools/verify.py there. Scratch success is evidence for a draft, not live acceptance.
"""
import argparse
import datetime as dt
import hashlib
import io
import json
import shutil
import subprocess
import sys
import tarfile
from pathlib import Path, PurePosixPath


def run(*args, **kw):
    return subprocess.run(args, capture_output=True, check=True, **kw).stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--contract", required=True)
    ap.add_argument("--impl-paths", nargs="*", default=[])
    ap.add_argument("--overlay")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    root = Path(run("git", "rev-parse", "--show-toplevel").decode().strip())
    out = Path(a.out).resolve()
    if out == root or root in out.parents:
        sys.exit("--out must be outside the repository")
    if out.exists() and any(out.iterdir()):
        sys.exit(f"{out} is not empty; use a new folder")
    out.mkdir(parents=True, exist_ok=True)

    base = run("git", "rev-parse", "HEAD").decode().strip()
    data = run("git", "-c", "core.autocrlf=false", "archive", "--format=tar", "HEAD", cwd=root)
    with tarfile.open(fileobj=io.BytesIO(data)) as t:
        t.extractall(out, filter="data")

    record = {"contract": a.contract, "base_commit": base, "time_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "impl_paths": a.impl_paths, "patch_sha256": None, "untracked_copied": {}, "overlay": {}}

    if a.impl_paths:
        patch = run("git", "-c", "core.autocrlf=false", "diff", "--binary", "HEAD", "--", *a.impl_paths, cwd=root)
        record["patch_sha256"] = hashlib.sha256(patch).hexdigest()
        if patch.strip():
            (out / "impl.patch").write_bytes(patch)
            r = subprocess.run(["git", "-c", "core.autocrlf=false", "apply", "--whitespace=nowarn", "impl.patch"],
                               cwd=out, capture_output=True, text=True)
            if r.returncode:
                sys.exit(f"patch failed; scratch is unusable:\n{r.stderr}")
        untracked = run("git", "ls-files", "-o", "--exclude-standard", "--", *a.impl_paths, cwd=root).decode().splitlines()
        for rel in untracked:
            dst = out / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root / rel, dst)
            record["untracked_copied"][rel] = hashlib.sha256(dst.read_bytes()).hexdigest()

    if a.overlay:
        with open(root / a.overlay, encoding="utf-8") as f:
            mapping = json.load(f)
        for src, target in mapping.items():
            tp = PurePosixPath(target)
            if tp.is_absolute() or ".." in tp.parts:
                sys.exit(f"bad overlay target: {target}")
            dst = out / target
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root / src, dst)
            record["overlay"][target] = {"from": src, "sha256": hashlib.sha256(dst.read_bytes()).hexdigest()}

    (out / "SCRATCH_RECORD.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print(f"Scratch ready at {out}\nBase {base}\nPatch {record['patch_sha256']}\nOverlaid {list(record['overlay'])}")
    print("This is evidence for a draft, not live acceptance. Do not run tools/verify.py here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
