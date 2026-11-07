"""Run from repo root:  python -m pytest -q tests   (or: python tests/test_core.py)"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "pi"))

import numpy as np
import config
from hailo_hook import pixel_to_bearing
from drone_dispatch import map_to_drone
from lidar_change import _circular_runs, ChangeDetector, build_baseline
from lidar_node import LINE_RE, read_revolutions
from policy import decide
from simulate import room_scan


def test_pixel_to_bearing():
    assert abs(pixel_to_bearing(0.5)) < 1e-9
    assert abs(pixel_to_bearing(0.0) - config.CAMERA_HFOV_DEG / 2) < 1e-6        # left edge = +HFOV/2
    assert abs(pixel_to_bearing(1.0) - (360 - config.CAMERA_HFOV_DEG / 2)) < 1e-6  # right edge = -HFOV/2


def test_map_to_drone_yaw():
    pad = {"x": 1.0, "y": 1.0, "yaw_deg": 90.0}  # nose points along map +y
    fwd, left = map_to_drone(1.0, 2.0, pad)
    assert abs(fwd - 1.0) < 1e-9 and abs(left) < 1e-9
    fwd, left = map_to_drone(0.0, 1.0, pad)       # map -x is the drone's left when facing +y
    assert abs(fwd) < 1e-9 and abs(left - 1.0) < 1e-9


def test_circular_runs_wrap():
    m = np.zeros(360, bool)
    m[[358, 359, 0, 1]] = True
    m[[100, 101]] = True
    runs = _circular_runs(m)
    assert sorted(len(r) for r in runs) == [2, 4]


def test_change_detector_finds_box():
    config.CHANGE_PERSIST_SCANS = 2
    det = ChangeDetector(build_baseline([room_scan() for _ in range(5)]))
    assert det.update(room_scan([(2.0, 1.0, 0.25)])) == []
    out = det.update(room_scan([(2.0, 1.0, 0.25)]))
    assert len(out) == 1 and math.hypot(out[0]["x"] - 2.0, out[0]["y"] - 1.0) < 0.35
    assert det.update(room_scan([(2.0, 1.0, 0.25)])) == []  # emitted once


def test_ultra_simple_parsing():
    lines = ["S  theta: 0.50 Dist: 01000.00 Q: 47 \n", "   theta: 1.50 Dist: 01010.00 Q: 47 \n",
             "   theta: 2.50 Dist: 00000.00 Q: 0 \n", "S  theta: 0.40 Dist: 01000.00 Q: 47 \n"]
    assert LINE_RE.match(lines[0])
    revs = list(read_revolutions(lines))
    assert revs == [[(0.5, 1000.0), (1.5, 1010.0)]]


def test_policy_blocks_path_through_hazard():
    zones = [{"x": 1.5, "y": 0.0, "r": 0.5, "kind": "FIRE"}]
    old = config.DRONE_PAD
    config.DRONE_PAD = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    kind, acts = decide("person", 3.0, 0.0, zones)
    config.DRONE_PAD = old
    assert kind == "SURVIVOR" and acts[0][0] == "ALERT_GROUND"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
