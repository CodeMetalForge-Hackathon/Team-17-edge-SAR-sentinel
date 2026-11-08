"""Safety and compatibility invariants. These must pass now and stay passing through every contract.
Run from repo root:  python -m pytest -q tests/test_guardrail_invariants.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "pi"))

import config
import drone_dispatch
from brain import Brain
from bus import LocalBus
from features import FEATURE_NAMES
from policy import decide
from simulate import node_window, room_scan, scan_msg
from features import window_features


class AlwaysAnomalous:
    def assess(self, x):
        return 9.0, True, "stub"


def actions_of(acts):
    return [a for a, _ in acts]


def test_flame_outranks_survivor_signals():
    kind, acts = decide("node", 1.0, 1.0, [], feat={"flame_any": 1.0, "pir_frac": 1.0, "loud_max": 900.0}, anomalous=True, person_near=True, node="n1")
    assert kind == "FIRE" and "DISPATCH_DRONE" not in actions_of(acts)


def test_water_outranks_survivor_signals():
    kind, acts = decide("node", 1.0, 1.0, [], feat={"water_any": 1.0, "pir_frac": 1.0}, anomalous=True, person_near=True, node="n1")
    assert kind == "FLOOD" and "DISPATCH_DRONE" not in actions_of(acts)


def test_person_inside_hazard_never_dispatches():
    z = [{"x": 2.0, "y": 0.0, "r": 0.7, "kind": "FIRE"}]
    kind, acts = decide("person", 2.1, 0.1, z)
    assert kind == "SURVIVOR" and "DISPATCH_DRONE" not in actions_of(acts) and "ALERT_GROUND" in actions_of(acts)


def test_path_through_hazard_never_dispatches():
    pad = config.DRONE_PAD
    tx, ty = pad["x"] + 3.0, pad["y"]
    z = [{"x": pad["x"] + 1.5, "y": pad["y"], "r": 0.5, "kind": "FIRE"}]
    assert "DISPATCH_DRONE" not in actions_of(decide("person", tx, ty, z)[1])


def test_obstruction_and_unknown_never_dispatch():
    assert "DISPATCH_DRONE" not in actions_of(decide("lidar", 1.0, 1.0, [], anomalous=True)[1])
    assert "DISPATCH_DRONE" not in actions_of(decide("node", 1.0, 1.0, [], feat={}, anomalous=True)[1])


def test_dry_run_is_the_default():
    assert config.DRONE_DRY_RUN is True
    bus = LocalBus()
    assert drone_dispatch.Dispatcher(bus).dry_run is True


def test_dry_run_loads_no_flight_library():
    sys.modules.pop("cflib", None)
    assert drone_dispatch.fly_to(0.1, 0.0, dry_run=True) is True
    assert "cflib" not in sys.modules


def test_target_past_max_leg_is_refused():
    assert drone_dispatch.fly_to(config.DRONE_MAX_LEG_M + 0.5, 0.0, dry_run=True) is False


def test_feature_schema_is_unchanged():
    assert FEATURE_NAMES == ["loud_max", "loud_mean", "loud_std", "light_mean", "light_delta", "pir_frac", "vib_hits",
                             "flame_any", "water_any", "temp", "hum"]


def test_event_and_action_keep_required_fields():
    bus = LocalBus()
    b = Brain(bus, AlwaysAnomalous())
    b.on_scan("forge/lidar/scan", scan_msg(room_scan([(2.0, 0.5, 0.2)])))
    b.on_person("forge/vision/person", {"ts": __import__("time").time(), "bearing_deg": math.degrees(math.atan2(0.5, 2.0)), "conf": 0.9})
    ev = next(p for t, p in bus.log if t == "forge/event")
    assert {"id", "ts", "kind", "x", "y", "score", "anomalous", "source", "detail"} <= set(ev)
    act = next(p for t, p in bus.log if t == "forge/action")
    assert {"event_id", "ts", "action", "x", "y", "reason", "avoid_zones"} <= set(act)


def test_node_features_payload_still_accepted():
    bus = LocalBus()
    b = Brain(bus, AlwaysAnomalous())
    f = window_features(node_window("flame"), 600.0)
    b.on_node("forge/node/n1/features", {"node": "n1", "x": 1.5, "y": 0.8, "ts": __import__("time").time(), "features": f})
    assert [p["kind"] for t, p in bus.log if t == "forge/event"] == ["FIRE"]
