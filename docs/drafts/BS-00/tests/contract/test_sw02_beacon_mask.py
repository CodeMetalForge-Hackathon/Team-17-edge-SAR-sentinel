"""SW-02 acceptance: our own beacon cannot look like an event, and it can never hide a hazard.
Whether the buzzer really couples into the sensors is a bench question (HW-B1)."""
from forge_test_support import brain_with, kinds, node_msg, feat, trained_model


def test_mask_window_timing():
    from node_bridge import BeaconMask
    now = [100.0]
    m = BeaconMask(beacon_s=2.0, guard_s=0.3, clock=lambda: now[0])
    assert not m.overlaps(99.0, 99.9)
    m.on_beacon()
    assert m.overlaps(99.5, 100.5)
    assert m.overlaps(102.2, 103.0)     # inside the guard after the 2 s beacon
    assert not m.overlaps(102.31, 103.0)
    assert not m.overlaps(98.0, 99.9)


def test_message_field_only_when_masked():
    from node_bridge import build_message
    f = feat("normal")
    plain = build_message("n1", {"x": 1.0, "y": 2.0}, f, 5.0, masked=False)
    assert set(plain) == {"node", "x", "y", "ts", "features"}
    assert build_message("n1", {"x": 1.0, "y": 2.0}, f, 5.0, masked=True)["beacon_masked"] is True


def test_masked_loud_window_makes_no_event():
    bus, b = brain_with(trained_model())
    b.on_node("t", node_msg(feat("tap"), beacon_masked=True))
    assert kinds(bus) == []


def test_masked_flame_still_fires():
    bus, b = brain_with(trained_model())
    b.on_node("t", node_msg(feat("flame"), beacon_masked=True))
    assert kinds(bus) == ["FIRE"]


def test_masked_water_still_fires():
    bus, b = brain_with(trained_model())
    b.on_node("t", node_msg(feat("water"), beacon_masked=True))
    assert kinds(bus) == ["FLOOD"]


def test_unmasked_tap_still_works():
    bus, b = brain_with(trained_model())
    b.on_node("t", node_msg(feat("tap")))
    assert kinds(bus) == ["SURVIVOR"]
