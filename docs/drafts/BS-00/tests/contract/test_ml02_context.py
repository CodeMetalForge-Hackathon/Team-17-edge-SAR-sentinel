"""ML-02 acceptance. Uses a labelled synthetic fixture only: no dataset, no model and no network.
Passing says the plumbing and the labels work, not that any damage prediction is good."""
import json
import os
import socket
import tempfile

from forge_test_support import brain_with, kinds, actions, fresh_scan, person_msg

from bus import LocalBus

GOOD_CTX = {"schema": "forge.damage_context/1", "status": "CACHED", "kind": "simulated", "generated_at": "2026-11-07T00:00:00Z",
            "runtime": {"device": "fixture"},
            "dataset": {"name": "fixture", "split": "n/a", "source": "test", "license": "n/a"},
            "checkpoint": {"repo": "n/a", "revision": "n/a", "sha256": "n/a", "license_declared": "n/a", "training_data_license": "n/a"},
            "preprocessing": "none",
            "samples": [{"sample_id": "s1", "event": "fixture", "class_pixels": {"major": 10}, "summary_class": "major", "overlay": "", "selection": "fixture"}]}
GOOD_ASSOC = {"association": "STAGED_MANUAL", "operator": "t", "note": "illustration only",
              "zones": [{"zone": "Z1", "name": "store", "polygon": [[1, 0], [2, 0], [2, 1], [1, 1]], "sample_id": "s1"}]}


def write(d, name, obj):
    p = os.path.join(d, name)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(obj, f)
    return p


def rejected(ctx=GOOD_CTX, assoc=GOOD_ASSOC):
    import damage_context
    with tempfile.TemporaryDirectory() as d:
        try:
            damage_context.load_context(write(d, "c.json", ctx), write(d, "a.json", assoc))
        except ValueError:
            return True
    return False


def test_good_files_load():
    assert not rejected()


def test_wrong_status_is_rejected():
    assert rejected(ctx={**GOOD_CTX, "status": "LIVE"})


def test_unknown_kind_is_rejected():
    assert rejected(ctx={**GOOD_CTX, "kind": "truth"})


def test_prediction_without_checkpoint_provenance_is_rejected():
    c = {**GOOD_CTX, "kind": "prediction"}
    c.pop("checkpoint")
    assert rejected(ctx=c)


def test_missing_dataset_provenance_is_rejected():
    c = dict(GOOD_CTX)
    c.pop("dataset")
    assert rejected(ctx=c)


def test_association_must_say_it_is_staged():
    assert rejected(assoc={**GOOD_ASSOC, "association": "SURVEYED"})


def test_zone_with_unknown_sample_is_rejected():
    z = {**GOOD_ASSOC["zones"][0], "sample_id": "nope"}
    assert rejected(assoc={**GOOD_ASSOC, "zones": [z]})


def message():
    import damage_context
    with tempfile.TemporaryDirectory() as d:
        return damage_context.context_message(*damage_context.load_context(write(d, "c.json", GOOD_CTX), write(d, "a.json", GOOD_ASSOC)))


def test_message_labels_every_zone():
    z = message()["zones"][0]
    assert z["kind"] == "simulated" and z["status"] == "CACHED" and z["association"] == "STAGED_MANUAL"
    assert z["sample_id"] == "s1" and z["summary_class"] == "major"


def test_loading_touches_no_network():
    real = socket.socket
    def refuse(*a, **k):
        raise AssertionError("network used")
    socket.socket = refuse
    try:
        message()
    finally:
        socket.socket = real


def test_context_cannot_create_hazards_or_actions():
    bus, b = brain_with(nodes={})
    bus.publish("forge/context/damage", message())
    assert b.avoid == [] and kinds(bus) == [] and actions(bus) == []


def test_person_in_a_major_damage_zone_decides_as_before():
    bus, b = brain_with(nodes={})
    bus.publish("forge/context/damage", message())
    b.on_scan("t", fresh_scan([(1.5, 0.5, 0.2)]))
    b.on_person("t", person_msg(1.5, 0.5))
    assert "DISPATCH_DRONE" in actions(bus) and b.avoid == []


def test_dashboard_stores_context_and_keeps_it_on_reset():
    from dashboard import Dashboard
    bus = LocalBus()
    d = Dashboard(bus, nodes={})
    bus.publish("forge/context/damage", message())
    assert d.context["zones"][0]["zone"] == "Z1"
    bus.publish("forge/control/reset", {"ts": 1.0, "reason": "t", "operator": "t"})
    assert d.context is not None     # the prior is reference data, not run state
