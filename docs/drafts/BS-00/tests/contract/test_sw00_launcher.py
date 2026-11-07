"""SW-00 acceptance: the launcher cannot force a live flight."""
import os

from forge_test_support import REPO


def launcher_code():
    with open(os.path.join(REPO, "run_pi.sh"), encoding="utf-8") as f:
        return [ln for ln in f.read().splitlines() if not ln.lstrip().startswith("#")]


def test_launcher_never_passes_force():
    assert not any("--force" in ln for ln in launcher_code())


def test_fly_flag_only_inside_the_fly_branch():
    code = launcher_code()
    fly_lines = [i for i, ln in enumerate(code) if "--fly" in ln]
    branch = [i for i, ln in enumerate(code) if '"$MODE" = "fly"' in ln]
    assert len(fly_lines) == 1 and branch and fly_lines[0] > branch[0]
