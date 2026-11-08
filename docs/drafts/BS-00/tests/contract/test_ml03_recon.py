"""ML-03 acceptance: fresh local evidence updates the list. Priors never become hazards or dispatches."""
import copy
import inspect

import forge_test_support  # noqa: F401

from bus import LocalBus

Z1 = {"zone": "Z1", "name": "store", "polygon": [[1, 0], [2, 0], [2, 1], [1, 1]], "sample_id": "s1", "summary_class": "major",
      "kind": "prediction", "status": "CACHED", "association": "STAGED_MANUAL"}
Z2 = {"zone": "Z2", "name": "office", "polygon": [[1, -2], [2, -2], [2, -1], [1, -1]], "sample_id": "s2", "summary_class": "no_damage",
      "kind": "prediction", "status": "CACHED", "association": "STAGED_MANUAL"}
NOW = 1000.0


def rec(events=(), avoid=(), zones=(Z1, Z2), now=NOW):
    import recon
    return recon.recommend(list(zones), list(events), list(avoid), now=now, person_max_age_s=20.0)


def person(x, y, ts=NOW):
    return {"kind": "SURVIVOR", "x": x, "y": y, "ts": ts, "id": 7}


def by_zone(out):
    return {r["zone"]: r for r in out}


def test_prior_only_is_a_candidate_never_a_dispatch():
    r = by_zone(rec())
    assert r["Z1"]["recommendation"] == "RECON_CANDIDATE"
    assert r["Z2"]["recommendation"] == "NO_PRIORITY"


def test_fresh_person_outranks_a_prior():
    out = rec(events=[person(1.5, -1.5)])
    assert out[0]["zone"] == "Z2" and out[0]["recommendation"] == "CONFIRM_SURVIVOR"


def test_person_in_hazard_comes_first_and_is_ground_team_only():
    out = rec(events=[person(1.5, 0.5), person(1.5, -1.5)], avoid=[{"x": 1.5, "y": 0.5, "r": 0.7, "kind": "FIRE"}])
    assert out[0]["zone"] == "Z1" and out[0]["recommendation"] == "GROUND_TEAM_ONLY"


def test_hazard_without_person_says_avoid():
    assert by_zone(rec(avoid=[{"x": 1.5, "y": -1.5, "r": 0.3, "kind": "FLOOD"}]))["Z2"]["recommendation"] == "AVOID_HAZARD"


def test_stale_person_no_longer_counts():
    assert by_zone(rec(events=[person(1.5, -1.5, ts=NOW - 60)]))["Z2"]["recommendation"] == "NO_PRIORITY"


def test_obstruction_says_check_access():
    ev = {"kind": "OBSTRUCTION", "x": 1.5, "y": 0.5, "ts": NOW, "id": 8}
    assert by_zone(rec(events=[ev]))["Z1"]["recommendation"] == "CHECK_ACCESS"


def test_evidence_says_where_each_reason_came_from():
    r = by_zone(rec(events=[person(1.5, 0.5)]))["Z1"]
    sources = [e["source"] for e in r["evidence"]]
    assert "live:person" in sources and "cached:prediction" in sources
    assert any(e.get("sample_id") == "s1" for e in r["evidence"])


def test_ranks_are_consecutive():
    assert [r["rank"] for r in rec()] == [1, 2]


def test_recommend_does_not_change_its_inputs():
    avoid = [{"x": 1.5, "y": 0.5, "r": 0.7, "kind": "FIRE"}]
    zones, before_a, before_z = [dict(Z1), dict(Z2)], copy.deepcopy(avoid), [dict(Z1), dict(Z2)]
    rec(avoid=avoid, zones=zones)
    assert avoid == before_a and zones == before_z


def test_works_with_no_context_at_all():
    assert rec(zones=()) == []


def test_service_publishes_recommendations_and_no_actions():
    import recon
    bus = LocalBus()
    recon.ReconService(bus, clock=lambda: NOW)
    bus.publish("forge/context/damage", {"zones": [Z1, Z2]})
    bus.publish("forge/event", person(1.5, -1.5))
    msgs = [p for t, p in bus.log if t == "forge/recon/recommendation"]
    assert msgs and msgs[-1]["recommendations"][0]["zone"] == "Z2"
    assert not [1 for t, _ in bus.log if t == "forge/action"]


def test_service_reset_forgets_live_evidence_keeps_prior():
    import recon
    bus = LocalBus()
    recon.ReconService(bus, clock=lambda: NOW)
    bus.publish("forge/context/damage", {"zones": [Z1, Z2]})
    bus.publish("forge/event", person(1.5, -1.5))
    bus.publish("forge/control/reset", {"ts": NOW, "reason": "t", "operator": "t"})
    last = [p for t, p in bus.log if t == "forge/recon/recommendation"][-1]
    assert by_zone(last["recommendations"])["Z2"]["recommendation"] == "NO_PRIORITY"
    assert by_zone(last["recommendations"])["Z1"]["recommendation"] == "RECON_CANDIDATE"


def test_policy_and_brain_do_not_read_context():
    import brain
    import policy
    for mod in (brain, policy):
        src = inspect.getsource(mod)
        assert "forge/context" not in src and "recon" not in src and "damage_context" not in src
