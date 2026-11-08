"""SW-06 acceptance: startup problems are visible, and wrong working directories cannot silently lose files."""
import os
import subprocess
import sys

from forge_test_support import PI, REPO

import config


def env():
    e = dict(os.environ)
    e.pop("PYTHONPATH", None)
    e["PYTHONDONTWRITEBYTECODE"] = "1"
    return e


def test_data_paths_do_not_depend_on_working_directory():
    for p in (config.MODEL_FILE, config.BASELINE_FILE, config.NORMAL_LOG):
        assert os.path.isabs(p) and os.path.dirname(os.path.abspath(p)) == os.path.abspath(PI)


def test_mqtt_host_can_be_set_by_environment():
    e = env()
    e["FORGE_MQTT_HOST"] = "10.1.2.3"
    out = subprocess.run([sys.executable, "-c", "import config; print(config.MQTT_HOST)"], cwd=PI, env=e, capture_output=True, text=True)
    assert out.stdout.strip() == "10.1.2.3"


def by_name(checks):
    return {c.name: c for c in checks}


def test_live_missing_files_are_reported(tmp_path=None):
    import preflight
    checks = by_name(preflight.run_checks(live=True, model_file="does-not-exist.joblib", baseline_file="does-not-exist.npy",
                                          broker_probe=lambda: (True, "ok"), serial_ports=[]))
    assert checks["lidar_baseline"].status == "FAIL" and checks["lidar_baseline"].detail
    assert checks["node_model"].status == "WARN" and checks["node_model"].detail


def test_unreachable_broker_fails_with_a_reason():
    import preflight
    checks = by_name(preflight.run_checks(live=True, model_file="x", baseline_file="x",
                                          broker_probe=lambda: (False, "connection refused"), serial_ports=[]))
    assert checks["mqtt_broker"].status == "FAIL" and "refused" in checks["mqtt_broker"].detail


def test_sim_mode_needs_no_hardware():
    import preflight
    checks = preflight.run_checks(live=False, model_file="x", baseline_file="x", broker_probe=lambda: (False, "off"), serial_ports=[])
    assert all(c.status != "FAIL" for c in checks)


def test_exit_code_reflects_failures():
    import preflight
    ok = preflight.Check("a", "OK", "fine")
    warn = preflight.Check("b", "WARN", "degraded")
    bad = preflight.Check("c", "FAIL", "broken")
    assert preflight.exit_code([ok, warn]) == 0
    assert preflight.exit_code([ok, warn, bad]) == 1


def test_preflight_check_runs_from_any_folder():
    r = subprocess.run([sys.executable, os.path.join(PI, "preflight.py"), "--check", "--sim"], cwd=REPO, env=env(), capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr


def test_live_check_without_broker_is_nonzero():
    e = env()
    e["FORGE_MQTT_HOST"] = "127.0.0.1"
    e["FORGE_MQTT_PORT"] = "1"
    r = subprocess.run([sys.executable, os.path.join(PI, "preflight.py"), "--check", "--live"], cwd=REPO, env=e, capture_output=True, text=True, timeout=60)
    assert r.returncode != 0
    assert "mqtt_broker" in r.stdout


def test_brain_survives_any_model_load_failure():
    import brain
    from bus import LocalBus
    real = brain.NodeAnomalyModel.load
    def broken(*a, **k):
        raise ModuleNotFoundError("joblib")
    brain.NodeAnomalyModel.load = broken
    try:
        assert brain.main(local_bus=LocalBus()).model is None
    finally:
        brain.NodeAnomalyModel.load = real


def test_launcher_is_the_single_one_and_text_checks_readiness():
    """Text-level only: proves the launcher calls preflight and watches its children. Real behaviour is INT-01 on the Pi."""
    assert not os.path.exists(os.path.join(PI, "run_demo.py")), "run_pi.sh is the one canonical launcher"
    code = open(os.path.join(REPO, "run_pi.sh"), encoding="utf-8").read()
    assert "preflight.py" in code and "kill -0" in code
    assert "--force" not in [w for ln in code.splitlines() if not ln.lstrip().startswith("#") for w in ln.split()]
