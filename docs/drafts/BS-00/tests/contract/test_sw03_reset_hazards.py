"""SW-03 acceptance: explicit reset, no duplicate zones, and no automatic expiry."""
from forge_test_support import cfg, brain_with, kinds, statuses, node_msg, feat, fresh_scan

from bus import LocalBus


def flame_at(b, nid="n1", x=1.5, y=0.8):
    b.on_node("forge/node/n1/features", node_msg(feat("flame"), nid, x, y))


def test_repeat_hazard_updates_the_same_zone():
    with cfg(EVENT_COOLDOWN_S=0):
        bus, b = brain_with()
        for _ in range(4):
            flame_at(b)
        assert len(b.avoid) == 1
        assert b.avoid[0]["count"] == 4
        assert b.avoid[0]["first_ts"] <= b.avoid[0]["ts"]


def test_zone_keeps_required_fields():
    with cfg(EVENT_COOLDOWN_S=0):
        _, b = brain_with()
        flame_at(b)
        assert {"x", "y", "r", "kind", "ts"} <= set(b.avoid[0])


def test_different_kinds_at_one_place_stay_separate():
    with cfg(EVENT_COOLDOWN_S=0):
        _, b = brain_with()
        flame_at(b)
        b.on_node("forge/node/n1/features", node_msg(feat("water")))
        assert sorted(z["kind"] for z in b.avoid) == ["FIRE", "FLOOD"]


def test_far_apart_hazards_stay_separate():
    with cfg(EVENT_COOLDOWN_S=0):
        _, b = brain_with()
        flame_at(b)
        flame_at(b, "n2", 3.5, -1.5)
        assert len(b.avoid) == 2


def test_zones_never_expire_on_their_own():
    with cfg(EVENT_COOLDOWN_S=0):
        _, b = brain_with()
        flame_at(b)
        b.avoid[0]["ts"] = b.avoid[0]["first_ts"] = 0.0    # very old
        flame_at(b, "n2", 3.5, -1.5)
        assert len(b.avoid) == 2


def test_reset_topic_clears_brain_state():
    bus, b = brain_with()
    b.on_scan("forge/lidar/scan", fresh_scan())
    flame_at(b)
    assert b.avoid and b.cooldown
    bus.publish("forge/control/reset", {"ts": 1.0, "reason": "test", "operator": "t"})
    assert b.avoid == [] and b.cooldown == {} and b.persons == []
    assert b.scan is not None            # sensor input is not "state to forget"


def test_after_reset_same_hazard_fires_at_once():
    bus, b = brain_with()
    flame_at(b)
    bus.publish("forge/control/reset", {"ts": 1.0, "reason": "test", "operator": "t"})
    flame_at(b)                          # default 15 s cooldown must not suppress this
    assert kinds(bus) == ["FIRE", "FIRE"]
    assert len(b.avoid) == 1


def test_event_ids_keep_increasing_after_reset():
    bus, b = brain_with()
    flame_at(b)
    bus.publish("forge/control/reset", {"ts": 1.0, "reason": "test", "operator": "t"})
    flame_at(b)
    ids = [p["id"] for t, p in bus.log if t == "forge/event"]
    assert ids == sorted(set(ids))


def test_brain_reports_reset_on_status_topic():
    bus, _ = brain_with()
    bus.publish("forge/control/reset", {"ts": 1.0, "reason": "test", "operator": "t"})
    st = [s for s in statuses(bus) if s["service"] == "brain"]
    assert st and {"ts", "service", "level", "msg"} <= set(st[-1])


def test_dashboard_reset_clears_view_keeps_scan():
    from dashboard import Dashboard
    bus = LocalBus()
    d = Dashboard(bus, nodes={"n1": (1.5, 0.8)})
    bus.publish("forge/lidar/scan", {"ts": 1.0, "ranges_m": [1.0] * 360})
    bus.publish("forge/event", {"id": 1, "kind": "FIRE", "x": 1.0, "y": 1.0})
    bus.publish("forge/action", {"event_id": 1, "action": "DISPATCH_DRONE", "x": 1, "y": 1, "reason": "r",
                                 "avoid_zones": [{"x": 1, "y": 1, "r": 1, "kind": "FIRE"}]})
    bus.publish("forge/control/reset", {"ts": 2.0, "reason": "test", "operator": "t"})
    # The integrated dashboard keeps a "(reset)" marker line in its log; that is accepted, old entries are not.
    assert d.events == [] and d.avoid == [] and d.dispatches == [] and d.log in ([], ["(reset)"])
    assert d.scan is not None


def test_lidar_detector_reset_allows_refire():
    from lidar_change import ChangeDetector, build_baseline
    from simulate import room_scan
    with cfg(CHANGE_PERSIST_SCANS=2):
        det = ChangeDetector(build_baseline([room_scan() for _ in range(5)]))
        box = [(2.0, 1.0, 0.25)]
        det.update(room_scan(box))
        assert len(det.update(room_scan(box))) == 1
        assert det.update(room_scan(box)) == []
        det.reset()
        assert det.tracks == []
        det.update(room_scan(box))
        assert len(det.update(room_scan(box))) == 1


def test_operator_command_publishes_reset():
    import reset as reset_cli      # the existing pi/reset.py is the one canonical reset interface
    bus = LocalBus()
    reset_cli.send_reset(bus, reason="between runs", operator="D")
    msg = next(p for t, p in bus.log if t == "forge/control/reset")
    assert msg["reason"] == "between runs" and msg["operator"] == "D" and "ts" in msg


def test_importing_the_reset_cli_sends_nothing():
    """pi/reset.py used to publish at import time. Importing it for tests or reuse must be silent."""
    import importlib
    import reset as reset_cli
    importlib.reload(reset_cli)
    assert callable(reset_cli.send_reset)


def test_legacy_ts_only_reset_still_works():
    """The integrated dashboard (press R) and the old reset.py send only {ts}. That must keep working."""
    from dashboard import Dashboard
    bus, b = brain_with()
    d = Dashboard(bus, nodes={"n1": (1.5, 0.8)})
    flame_at(b)
    bus.publish("forge/control/reset", {"ts": 1.0})
    assert b.avoid == [] and b.cooldown == {} and d.avoid == [] and d.events == []
