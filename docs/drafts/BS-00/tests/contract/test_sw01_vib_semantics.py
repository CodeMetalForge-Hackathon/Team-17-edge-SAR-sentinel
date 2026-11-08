"""SW-01 acceptance. Measurement meaning first; classification rules are untouched.
These tests cover the simulator, model and static firmware text. Real firmware behaviour is a bench check (HW-B1)."""
import os
import re
import tempfile

from forge_test_support import REPO, brain_with, kinds, node_msg, feat, trained_model

import policy
import features
from simulate import node_window


def test_sim_tap_has_few_distinct_events():
    for _ in range(50):
        total = sum(r["vib"] for r in node_window("tap"))
        assert 1 <= total < policy.VIB_STRUCTURAL


def test_sim_shake_respects_refractory_limit_and_passes_threshold():
    for _ in range(50):
        rs = node_window("shake")
        assert all(r["vib"] <= 2 for r in rs)  # 50 ms refractory allows at most 2 per 100 ms report
        assert sum(r["vib"] for r in rs) >= policy.VIB_STRUCTURAL


def test_tap_is_survivor_not_structural():
    bus, b = brain_with(trained_model())
    b.on_node("forge/node/n1/features", node_msg(feat("tap")))
    assert kinds(bus) == ["SURVIVOR"]
    assert b.avoid == []


def test_shake_is_structural():
    bus, b = brain_with(trained_model())
    b.on_node("forge/node/n1/features", node_msg(feat("shake")))
    assert kinds(bus) == ["STRUCTURAL"]


def test_hazard_still_outranks_tap():
    f = feat("tap")
    f["flame_any"] = 1.0
    bus, b = brain_with(trained_model())
    b.on_node("forge/node/n1/features", node_msg(f))
    assert kinds(bus) == ["FIRE"]


def test_feature_semantics_version_exists():
    assert features.FEATURE_SEMANTICS_VERSION >= 2


def test_model_saved_records_semantics_and_old_model_is_rejected():
    import joblib
    import anomaly
    m = trained_model()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "m.joblib")
        m.save(path)
        assert joblib.load(path)["feature_semantics"] == features.FEATURE_SEMANTICS_VERSION
        anomaly.NodeAnomalyModel.load(path)  # a current model loads
        old = joblib.load(path)
        del old["feature_semantics"]         # a model from before this change
        joblib.dump(old, path)
        try:
            anomaly.NodeAnomalyModel.load(path)
            raised = False
        except anomaly.ModelSchemaError:
            raised = True
        assert raised


def test_brain_starts_degraded_on_old_model():
    import anomaly
    import brain
    from bus import LocalBus
    real = anomaly.NodeAnomalyModel.load
    def stale(*a, **k):
        raise anomaly.ModelSchemaError("old model")
    brain.NodeAnomalyModel.load = stale
    try:
        assert brain.main(local_bus=LocalBus()).model is None
    finally:
        brain.NodeAnomalyModel.load = real


def test_firmware_text_no_longer_counts_active_samples():
    """Static only. It cannot show the firmware behaves; HW-B1 does that on a bench."""
    with open(os.path.join(REPO, "arduino", "sensor_node", "sensor_node.ino"), encoding="utf-8") as f:
        src = f.read()
    assert not re.search(r"digitalRead\(PIN_VIB\)\s*==\s*VIB_ACTIVE\)\s*vibHits\+\+", src)
    assert "VIB_REFRACTORY_MS" in src


def test_changed_meaning_is_documented():
    with open(os.path.join(REPO, "pi", "bus.py"), encoding="utf-8") as f:
        src = f.read()
    assert "distinct" in src.lower() and "vib" in src.lower()
