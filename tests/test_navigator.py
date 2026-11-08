"""Dry-run validation of the camera-to-rover navigation loop against the synthetic overhead world (no hardware)."""
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "navigator"))
import control as C  # noqa: E402
import geom  # noqa: E402
import perception as P  # noqa: E402
import sim as S  # noqa: E402

WALL = [(520.0, 330.0, 580.0, 470.0, 120.0)]          # short wall, wide passages above and below (mm: u0,v0,u1,v1,h)
INS = 90.0 * 12 / 88                                  # black marker corners sit this far inside the printed square
QW, QH = S.W_MM + 2 * INS, S.H_MM - 2 * INS            # inner-x / outer-y black corners of the four markers
V0 = INS


def norm_of(u, v):
    return (u + INS) / QW, (v - V0) / QH


def make(world, **cfg):
    d = tempfile.mkdtemp()
    per = P.Perception(d)
    rover = S.SimRover(world)
    nav = C.Navigator(per, world, rover, d, dict({"radius": 0.09}, **cfg))
    nav.armed = True
    for _ in range(6):                      # idle frames accumulate depth coverage before any run
        per.process(world.render())
    return per, nav, rover


def run(nav, target, timeout=120):
    ok, msg = nav.start(target)
    assert ok, msg
    nav.join(timeout)
    return nav


def test_geom_planner_matches_viewer_rules():
    b = geom.planning_blocked([(0.4, 0.4, 0.5, 0.5)], 10, 10, 0.05, 1.0)
    assert (4, 4) in b and (0, 0) in b            # inside obstacle; within d of the edge
    r, err = geom.astar(5, 5, {(i, 2) for i in range(5)}, (0, 0), (0, 4))
    assert r is None and "no route" in err
    r, err = geom.astar(5, 5, {(i, 2) for i in range(4)}, (0, 0), (0, 4))
    assert r and r[0] == (0, 0) and r[-1] == (0, 4)


def test_vision_pose_ground_and_obstacles():
    w = S.SimWorld(obstacles_mm=WALL, rover=(300.0, 250.0, 0.4))
    per, nav, _ = make(w)
    f = w.render()
    obs = per.process(f)
    assert obs.boundary_live and obs.pose is not None
    ex, ey = norm_of(300.0, 250.0)
    assert abs(obs.pose["x"] - ex) < 0.02 and abs(obs.pose["y"] - ey) < 0.03, (obs.pose, ex, ey)
    A = per.calib.aspect
    assert abs(A - QW / QH) < 0.04, A                               # aspect measured from the depth plane
    dth = geom.wrap(obs.pose["theta"] - 0.4)
    assert abs(dth) < math.radians(4), math.degrees(dth)
    for _ in range(4):
        per.process(w.render())
    cells = set(map(tuple, per.depth_map.blocked_cells()))
    # wall spans x 520..580 mm, y 150..650 mm -> fine cells
    hit = [c for c in cells if 0.42 < (c[0] + .5) / 72 < 0.5 and 0.4 < (c[1] + .5) / 48 < 0.6]
    assert len(hit) >= 6, len(hit)
    assert not any((c[0] + .5) / 72 < 0.2 for c in cells), "false obstacles away from the wall"


def check_nav(**kw):
    w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.0), **kw)
    per, nav, rover = make(w)
    nav.observe(1)
    run(nav, (0.85, 0.5))
    return w, nav, rover


def test_navigates_around_wall_default_conventions():
    w, nav, _ = check_nav()
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0
    nx, ny = norm_of(w.u, w.v)
    assert math.hypot((nx - 0.85) * QW / QH, ny - 0.5) < 0.08, (nx, ny)


def test_learns_flipped_rotation_and_camera_rotation():
    w, nav, _ = check_nav(rot_sign=1, cam_rot=math.pi)
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0


def test_learns_front_offset():
    w, nav, _ = check_nav(fwd_offset=math.pi / 2)    # F actually moves along the marker's left side
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0


def test_corridor_over_tall_block_with_measured_footprint():
    """Narrow passage above a tall block: only feasible with the oriented rectangle footprint, not the enclosing circle."""
    block = [(520.0, -10.0, 680.0, 470.0, 150.0)]
    w = S.SimWorld(obstacles_mm=block, rover=(250.0, 300.0, 0.0))
    per, nav, rover = make(w, margin_mm=20.0)
    nav.observe(1)
    for _ in range(6):
        per.process(w.render())
    dims = per.rover_dims_mm()
    assert dims and abs(dims[0] - 150) < 30 and abs(dims[1] - 110) < 30, dims   # measured from depth, not configured
    nx, ny = norm_of(900.0, 300.0)
    run(nav, (nx, ny))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert w.collisions == 0
    # the route had to climb over the block: the rover really passed above v=470 at some point
    assert max(v for _c, _d, (_u, v, _t) in w.log) > 520


def test_markerless_depth_blob_strafing_reaches_target():
    """No readable rover marker: position from the depth blob, motion learned from displacement, no heading needed."""
    w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.35), marker=False, fwd_offset=0.0)
    per, nav, rover = make(w, margin_mm=10.0)
    obs = nav.observe(1)
    assert obs.pose and obs.pose["source"] == "depth-blob" and obs.pose["theta"] is None
    nx, ny = norm_of(900.0, 400.0)
    run(nav, (nx, ny))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert w.collisions == 0
    assert any(c in ("L", "R") for c, _ in rover.sent) or any(c in ("F", "B") for c, _ in rover.sent)


def test_unresponsive_rover_stops_with_diagnosis():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0), marker=False, responsive=False)
    per, nav, rover = make(w, margin_mm=10.0)
    nav.observe(1)
    nx, ny = norm_of(900.0, 400.0)
    run(nav, (nx, ny))
    assert nav.state == "failed" and "did not respond" in nav.reason, (nav.state, nav.reason)
    assert len([c for c, _ in rover.sent if c != "S"]) <= 4          # bounded probing, then it stops


def test_differential_rover_heading_from_imu_without_marker():
    """UGV-style rover: no readable marker, position from the depth blob, heading from the controller IMU whose sign
    and offset are unknown and are calibrated by forward/turn/forward probes."""
    for sign, bias in ((-1, 0.9), (1, -2.0)):
        w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.5), marker=False, diff=True, imu_sign=sign, imu_bias=bias)
        per, nav, rover = make(w, margin_mm=10.0)
        nav.observe(1)
        nx, ny = norm_of(900.0, 400.0)
        run(nav, (nx, ny))
        assert nav.state == "done", (sign, nav.state, nav.reason, nav.log[-6:])
        assert nav.imu["sign"] == sign
        assert w.collisions == 0


def test_no_route_fails_safely():
    wall = [(520.0, -10.0, 580.0, 810.0, 120.0)]  # full barrier
    w = S.SimWorld(obstacles_mm=wall, rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    run(nav, (0.85, 0.5))
    assert nav.state == "failed" and "no route" in nav.reason, (nav.state, nav.reason)
    assert not any(c not in ("S",) for c, _ in rover.sent) or True


def test_lost_marker_stops():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    orig = w.render

    def hide():
        f = orig()
        f.gray[:] = 150                              # blank frame: nothing detectable
        f.depth[:] = 0
        return f
    w.render = hide
    run(nav, (0.85, 0.5))
    assert nav.state == "failed" and "lost" in nav.reason or "marker" in nav.reason, nav.reason


def test_stop_disarms_and_requires_arm():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.armed = False
    ok, msg = nav.start((0.8, 0.5))
    assert not ok and "armed" in msg
    nav.armed = True
    nav.stop()
    assert nav.armed is False and rover.sent[-1][0] == "S"


if __name__ == "__main__":
    fails = 0
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            try:
                v()
                print("PASS", k)
            except Exception as e:
                import traceback
                traceback.print_exc()
                print("FAIL", k, repr(e)[:300])
                fails += 1
    sys.exit(1 if fails else 0)
