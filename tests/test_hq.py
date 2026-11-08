"""End-to-end test of the HQ overhead loop on the simulator: corners, rubble, route, calibration, autonomous drive.

    python tests/test_hq.py      (needs opencv + numpy)
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hq"))

import autopilot
from autopilot import HQ
from planner import Grid
from sim import SimCamera, SimRover, SimWorld
from vision import corners_from_markers, detect_markers


def make(seed=0):
    random.seed(seed)
    autopilot.SETTLE_S = 0
    world = SimWorld(seed=seed)
    hq = HQ(SimCamera(world), SimRover(world), motion_allowed=True)
    return world, hq


def test_corner_markers_any_ids():
    world, _ = make()
    m = detect_markers(world.photo())
    assert set(m) == {4, 7, 9, 11, 10}, sorted(m)
    assert corners_from_markers(m, rover_id=10) is not None


def test_pose_and_rubble():
    world, hq = make()
    hq.save_before()
    x, y, th = hq.pose
    assert math.hypot(x - world.x, y - world.y) < 0.03 and abs(math.degrees(th - world.th)) < 4, hq.pose
    world.boxes = [(0.55, 0.0, 0.75, 0.6), (1.0, 0.45, 1.2, 1.0)]
    hq.scan()
    assert len(hq.obstacles) == 2, hq.obstacles
    names = {hq.grid.name(*c) for c in hq.blocked}
    assert {"C1", "C2", "E3", "E4"} <= names, names


def test_route_avoids_blocked():
    g = Grid(1.5, 1.0, 6, 4)
    blocked = {(2, 0), (2, 1), (2, 2)}
    path = g.route((0, 0), (5, 0), blocked)
    assert path and not (set(path) & blocked) and path[0] == (0, 0) and path[-1] == (5, 0)


def test_drives_to_target_around_rubble():
    world, hq = make(seed=3)
    hq.save_before()
    world.boxes = [(0.55, 0.0, 0.70, 0.55), (1.05, 0.45, 1.2, 1.0)]   # leaves a ~35 cm gap between them
    hq.set_target("F1")
    hq.running = True
    trail = []
    orig = hq.snapshot
    def snap():
        p = orig()
        trail.append((world.x, world.y))
        return p
    hq.snapshot = snap
    hq._run()
    assert hq.status.startswith("arrived F1"), (hq.status, list(hq.log)[:8])
    assert not any(world.inside_box(x, y, pad=0.02) for x, y in trail), "drove through rubble"
    print("   arrived in", len(hq.rover.log), "bursts:", hq.rover.log)


def test_stops_when_marker_lost():
    world, hq = make(seed=5)
    hq.save_before()
    hq.set_target("D3")
    world.rover_id = 12            # the rover's marker "disappears"
    hq.running = True
    hq._run()
    assert "not visible" in hq.status, hq.status


def test_blob_pilot_no_rover_marker():
    """White board, rover WITHOUT a marker, sandbox = outer corners of the 4 markers (like the real table)."""
    import blob_pilot
    from sim import SimCamera as Cam
    random.seed(7)
    blob_pilot.time.sleep = lambda s: None
    world = SimWorld(seed=7, white_board=True, rover_id=None, rover=(0.2, 0.75, math.radians(90)))
    world.boxes = [(0.62, 0.72, 0.77, 1.0), (0.48, -0.08, 0.68, 0.12)]   # like the Crazyflie box + yellow box
    pilot = blob_pilot.BlobPilot(Cam(world), SimRover(world), size=(world.w + 0.2, world.h + 0.2), grid=(6, 4),
                                 log=lambda m: None)
    pilot.look()
    pilot.set_target("F1")
    trail = []
    orig = pilot.look
    def look(**kw):
        r = orig(**kw); trail.append((world.x, world.y)); return r
    pilot.look = look
    assert pilot.drive() == "F1"
    assert not any(world.inside_box(x, y, pad=0.03) for x, y in trail), "drove through rubble"
    print("   bursts:", world and len(pilot.rover.log), pilot.rover.log)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
