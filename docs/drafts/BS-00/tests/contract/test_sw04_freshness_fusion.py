"""SW-04 acceptance: freshness, nearest beacon, and a cooldown that cannot hide a changed decision."""
import time

from forge_test_support import cfg, brain_with, kinds, actions, statuses, node_msg, feat, fresh_scan, person_msg

NODES = {"n1": (1.5, 0.8)}
AT_N1 = [(1.5, 0.8, 0.2)]


def test_fresh_scan_places_person():
    bus, b = brain_with(nodes=NODES)
    b.on_scan("t", fresh_scan(AT_N1))
    b.on_person("t", person_msg(1.5, 0.8))
    assert kinds(bus) == ["SURVIVOR"]


def test_stale_scan_does_not_place_person_and_says_why():
    with cfg(SCAN_MAX_AGE_S=2.0):
        bus, b = brain_with(nodes=NODES)
        b.on_scan("t", fresh_scan(AT_N1, ts=time.time() - 60))
        b.on_person("t", person_msg(1.5, 0.8))
        assert kinds(bus) == []
        assert any(s["level"] == "WARN" for s in statuses(bus))


def test_no_scan_says_why():
    bus, b = brain_with(nodes=NODES)
    b.on_person("t", person_msg(1.5, 0.8))
    assert kinds(bus) == []
    assert any(s["level"] == "WARN" for s in statuses(bus))


def test_warning_is_rate_limited():
    bus, b = brain_with(nodes=NODES)
    for _ in range(10):
        b.on_person("t", person_msg(1.5, 0.8))
    assert len(statuses(bus)) <= 2


def test_camera_person_beacons_nearest_node():
    bus, b = brain_with(nodes=NODES)
    b.on_scan("t", fresh_scan(AT_N1))
    b.on_person("t", person_msg(1.5, 0.8))
    assert "BEACON" in actions(bus)
    assert [t for t, _ in bus.log if t == "forge/node/n1/cmd"]


def test_camera_person_far_from_node_gets_no_beacon():
    bus, b = brain_with(nodes=NODES)
    b.on_scan("t", fresh_scan([(3.0, -1.5, 0.2)]))
    b.on_person("t", person_msg(3.0, -1.5))
    assert kinds(bus) == ["SURVIVOR"] and "BEACON" not in actions(bus)


def test_nodes_default_to_config_format():
    with cfg(NODES={"n1": {"x": 1.5, "y": 0.8, "port": "x"}}):
        bus, b = brain_with()
        b.on_scan("t", fresh_scan(AT_N1))
        b.on_person("t", person_msg(1.5, 0.8))
        assert "BEACON" in actions(bus)


def test_person_in_hazard_still_beacons_but_drone_is_held():
    with cfg(EVENT_COOLDOWN_S=0):
        bus, b = brain_with(nodes=NODES)
        b.on_scan("t", fresh_scan(AT_N1))
        b.on_node("t", node_msg(feat("flame")))
        b.on_person("t", person_msg(1.5, 0.8))
        assert "DISPATCH_DRONE" not in actions(bus)
        assert "BEACON" in actions(bus) and actions(bus).count("ALERT_GROUND") == 2


def test_changed_decision_is_not_hidden_by_cooldown():
    """Dispatch, then a hazard appears, then the same person inside it, within the default cooldown."""
    bus, b = brain_with(nodes={})
    b.on_scan("t", fresh_scan([(2.1, -0.9, 0.2)]))
    b.on_person("t", person_msg(2.1, -0.9))
    assert "DISPATCH_DRONE" in actions(bus)
    b.on_node("t", node_msg(feat("flame"), "n2", 2.2, -1.0))
    before = actions(bus).count("ALERT_GROUND")
    b.on_person("t", person_msg(2.1, -0.9))
    assert actions(bus).count("ALERT_GROUND") == before + 1
    assert actions(bus).count("DISPATCH_DRONE") == 1


def test_identical_decision_is_still_suppressed():
    bus, b = brain_with(nodes={})
    b.on_scan("t", fresh_scan([(2.1, -0.9, 0.2)]))
    for _ in range(5):
        b.on_person("t", person_msg(2.1, -0.9))
    assert actions(bus).count("DISPATCH_DRONE") == 1
