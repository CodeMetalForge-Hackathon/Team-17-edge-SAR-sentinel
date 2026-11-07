"""SW-05 acceptance: the crowd outside the demo region is ignored, hazards never are, and a person walking
in is not first reported as debris."""
import time

from forge_test_support import cfg, brain_with, kinds, statuses, node_msg, feat, fresh_scan, person_msg

REGION = [(0.5, -2.0), (4.0, -2.0), (4.0, 2.0), (0.5, 2.0)]


def change(x, y, ts):
    return {"ts": ts, "x": x, "y": y, "bearing_deg": 0.0, "range_m": 1.0, "width_deg": 8, "size_m": 0.2}


def test_person_outside_region_is_ignored_and_reported():
    with cfg(DEMO_REGION=REGION):
        bus, b = brain_with(nodes={})
        b.on_scan("t", fresh_scan([(-0.8, 0.0, 0.1)]))
        b.on_person("t", person_msg(-0.8, 0.0))
        assert kinds(bus) == []
        assert any(s["level"] == "INFO" for s in statuses(bus))


def test_person_inside_region_fires():
    with cfg(DEMO_REGION=REGION):
        bus, b = brain_with(nodes={})
        b.on_scan("t", fresh_scan([(2.0, 0.5, 0.2)]))
        b.on_person("t", person_msg(2.0, 0.5))
        assert kinds(bus) == ["SURVIVOR"]


def test_lidar_change_outside_region_is_ignored():
    with cfg(DEMO_REGION=REGION, OBSTRUCTION_HOLD_S=0):
        bus, b = brain_with(nodes={})
        b.on_change("t", change(-0.8, 0.0, time.time()))
        assert kinds(bus) == []


def test_hazard_at_a_node_outside_region_still_fires():
    with cfg(DEMO_REGION=REGION):
        bus, b = brain_with(nodes={})
        b.on_node("t", node_msg(feat("flame"), "n9", -0.8, 0.0))
        assert kinds(bus) == ["FIRE"]


def test_no_region_means_no_filter():
    with cfg(DEMO_REGION=None):
        bus, b = brain_with(nodes={})
        b.on_scan("t", fresh_scan([(-0.8, 0.0, 0.1)]))
        b.on_person("t", person_msg(-0.8, 0.0))
        assert kinds(bus) == ["SURVIVOR"]


def test_obstruction_waits_for_the_hold_time():
    with cfg(OBSTRUCTION_HOLD_S=2.0, DEMO_REGION=None):
        t0 = time.time()
        bus, b = brain_with(nodes={})
        b.on_change("t", change(3.0, 1.8, t0))
        assert kinds(bus) == []
        b.on_scan("t", fresh_scan(ts=t0 + 1.0))
        assert kinds(bus) == []
        b.on_scan("t", fresh_scan(ts=t0 + 2.5))
        assert kinds(bus) == ["OBSTRUCTION"]


def test_person_during_hold_cancels_the_obstruction():
    with cfg(OBSTRUCTION_HOLD_S=2.0, DEMO_REGION=None):
        t0 = time.time()
        bus, b = brain_with(nodes={})
        b.on_scan("t", fresh_scan([(2.0, 0.5, 0.2)], ts=t0))
        b.on_change("t", change(1.85, 0.46, t0))
        b.on_person("t", person_msg(2.0, 0.5, ts=t0 + 0.5))
        b.on_scan("t", fresh_scan(ts=t0 + 3.0))
        assert kinds(bus) == ["SURVIVOR"]


def test_zero_hold_fires_immediately_for_the_fast_simulator():
    with cfg(OBSTRUCTION_HOLD_S=0, DEMO_REGION=None):
        bus, b = brain_with(nodes={})
        b.on_change("t", change(3.0, 1.8, time.time()))
        assert kinds(bus) == ["OBSTRUCTION"]


def test_reset_clears_pending_obstruction():
    with cfg(OBSTRUCTION_HOLD_S=2.0, DEMO_REGION=None):
        t0 = time.time()
        bus, b = brain_with(nodes={})
        b.on_change("t", change(3.0, 1.8, t0))
        bus.publish("forge/control/reset", {"ts": t0, "reason": "t", "operator": "t"})
        b.on_scan("t", fresh_scan(ts=t0 + 5.0))
        assert kinds(bus) == []


def test_wording_does_not_claim_more_than_we_know():
    from policy import decide
    _, acts = decide("node", 1.0, 1.0, [], feat={"vib_hits": 50.0}, anomalous=True)
    assert all("collapse" not in reason.lower() for _, reason in acts)
    assert any("not assessed" in reason.lower() for _, reason in acts)
