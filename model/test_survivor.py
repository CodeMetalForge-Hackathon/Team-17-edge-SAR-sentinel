"""Scenario checks for the chance-of-life model.
Run: python -m pytest -q test_survivor.py   (or: python test_survivor.py)
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_stops as sim  # noqa: E402
import survivor as sv    # noqa: E402

# scenario -> (min p, max p)
TARGETS = {
    "empty":            (0.0, 0.15),
    "empty_void":       (0.0, 0.15),
    "false_trigger":    (0.0, 0.25),
    "fire":             (0.0, 0.25),
    "unconscious_void": (0.50, 1.0),
    "conscious":        (0.70, 1.0),
    "conscious_void":   (0.90, 1.0),
}
SEEDS = range(30)


def _base(seed):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "calib.csv")
        sim.write_csv(sim.simulate(sim.parse_plan("A1:empty,B1:empty,C1:empty,D1:empty"), seed + 1000), p)
        return sv.calibrate(sv.load_stops(p))


def _score(plan, seed):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "run.csv")
        sim.write_csv(sim.simulate(sim.parse_plan(plan), seed), p)
        stops = sv.load_stops(p)
    base = _base(seed)
    out = []
    for s in stops.values():
        prob, reasons, f = sv.score_stop(s, base)
        sv.update_room(base, f)
        out.append((prob, reasons))
    return out


def test_each_scenario_after_empty():
    """Each scenario on its own, preceded by an empty square."""
    for scen, (lo, hi) in TARGETS.items():
        for seed in SEEDS:
            prob, reasons = _score(f"A1:empty,B1:{scen}", seed)[1]
            assert lo <= prob <= hi, f"{scen} seed={seed}: {prob:.2f} {reasons}"


def test_co2_leftover_does_not_leak():
    """Square right after a CO2 source must not inherit it."""
    for seed in SEEDS:
        for src in ("unconscious_void", "fire"):
            for nxt in ("empty", "empty_void"):
                prob, reasons = _score(f"A1:empty,B1:{src},C1:{nxt}", seed)[2]
                assert prob <= 0.15, f"{src}->{nxt} seed={seed}: {prob:.2f} {reasons}"


def test_demo_plan_runs():
    res = _score(sim.DEMO_PLAN, 0)
    assert len(res) == len(sim.parse_plan(sim.DEMO_PLAN))


if __name__ == "__main__":
    test_each_scenario_after_empty()
    test_co2_leftover_does_not_leak()
    test_demo_plan_runs()
    print("all scenario checks passed")
