"""Validation of the camera-to-rover navigation loop against the synthetic overhead world (no hardware).

Covers: planner geometry, vision/pose/ground plane, target mapping and validation, footprint clearance, IMU-closed turns,
streamed cruise and its watchdog, arrival tolerance, interruption/re-target handling, and fail-safe stops."""
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools", "navigator"))
import control as C  # noqa: E402
import geom  # noqa: E402
import perception as P  # noqa: E402
import sim as S  # noqa: E402
import ugv  # noqa: E402

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
    nav = C.Navigator(per, world, rover, d, dict({"radius": 0.09, "margin_mm": 10.0}, **cfg))
    nav.armed = True
    for _ in range(6):                      # idle frames accumulate depth coverage before any run
        per.process(world.render())
    nav.pose = per.last_obs.pose              # the server's idle loop keeps this current
    return per, nav, rover


def run(nav, target, timeout=180):
    ok, msg = nav.start(target)
    assert ok, msg
    nav.join(timeout)
    return nav


def mm_error(world, target, hm):
    nx, ny = norm_of(world.u, world.v)
    return math.hypot((nx - target[0]) * QW / QH, ny - target[1]) * hm


# ---------------------------------------------------------------------------------------------- geometry / vision
def test_geom_planner_matches_viewer_rules():
    b = geom.planning_blocked([(0.4, 0.4, 0.5, 0.5)], 10, 10, 0.05, 1.0)
    assert (4, 4) in b and (0, 0) in b
    r, err = geom.astar(5, 5, {(i, 2) for i in range(5)}, (0, 0), (0, 4))
    assert r is None and "no route" in err
    r, err = geom.astar(5, 5, {(i, 2) for i in range(4)}, (0, 0), (0, 4))
    assert r and r[0] == (0, 0) and r[-1] == (0, 4)


def test_vision_pose_ground_and_obstacles():
    w = S.SimWorld(obstacles_mm=WALL, rover=(300.0, 250.0, 0.4))
    per, nav, _ = make(w)
    obs = per.process(w.render())
    assert obs.boundary_live and obs.pose is not None and obs.pose["source"] == "marker"
    ex, ey = norm_of(300.0, 250.0)
    assert abs(obs.pose["x"] - ex) < 0.02 and abs(obs.pose["y"] - ey) < 0.03, (obs.pose, ex, ey)
    assert abs(per.calib.aspect - QW / QH) < 0.04, per.calib.aspect
    assert abs(geom.wrap(obs.pose["theta"] - 0.4)) < math.radians(4)
    cells = set(map(tuple, per.depth_map.blocked_cells()))
    hit = [c for c in cells if 0.42 < (c[0] + .5) / 72 < 0.5 and 0.4 < (c[1] + .5) / 48 < 0.6]
    assert len(hit) >= 6, len(hit)
    assert not any((c[0] + .5) / 72 < 0.2 for c in cells), "false obstacles away from the wall"


# ---------------------------------------------------------------------------------------------- target selection
def test_target_click_maps_exactly_and_is_not_snapped():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, _ = make(w)
    c = per.calib
    for x, y in ((0.137, 0.482), (0.8123, 0.3571)):
        px = c.to_px([[x, y]])[0]
        back = c.to_norm([px])[0]
        assert abs(back[0] - x) < 1e-6 and abs(back[1] - y) < 1e-6          # image click <-> ground map
        v = nav.validate_target(x, y)
        assert v["ok"] and v["suggestion"] is None
        nav.target = (x, y)
        assert nav.target == (x, y)                                          # kept exactly, no cell-centre snapping
        res = nav.make_plan((nav.pose["x"], nav.pose["y"]))             # planned from the observed pose
        assert res["waypoints"][-1] == (x, y)                                # trajectory ends at the exact point


def test_target_rejections_and_suggestion():
    w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.0))
    per, nav, _ = make(w)
    assert not nav.validate_target(1.2, 0.5)["ok"]                                    # outside the boundary
    assert "boundary" in nav.validate_target(-0.1, 0.5)["reason"]
    edge = nav.validate_target(0.002, 0.5)                                            # rover would hang over the edge
    assert not edge["ok"] and edge["suggestion"] is not None
    sx, sy = edge["suggestion"]
    assert nav.validate_target(sx, sy)["ok"] and sx > 0.002                           # offered point itself is valid
    nx, ny = norm_of(550.0, 400.0)
    onwall = nav.validate_target(nx, ny)                                              # on the obstacle
    assert not onwall["ok"] and onwall["suggestion"] is not None
    assert nav.target is None                                                         # nothing changed silently


# ---------------------------------------------------------------------------------------------- IMU turns
def test_imu_closed_turns_reach_requested_angle():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0), diff=True)
    r = S.SimRover(w)
    for cmd, tgt in (("Q", 0.5), ("E", 1.2), ("Q", 0.2)):
        before = w.th
        got = r.turn_by(cmd, tgt)
        assert abs(abs(geom.wrap(w.th - before)) - got) < 1e-6 and abs(got - tgt) < 0.4 * tgt


# ---------------------------------------------------------------------------------------------- navigation
def check_nav(**kw):
    w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.0), **kw)
    per, nav, rover = make(w)
    nav.observe(1)
    tgt = (0.85, 0.5)
    run(nav, tgt)
    return w, nav, rover, per, tgt


def test_navigates_around_wall_and_arrives_within_tolerance():
    w, nav, _, per, tgt = check_nav()
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0
    hm = per.calib.height_mm
    assert mm_error(w, tgt, hm) <= nav.cfg["arrive_tol_mm"] * 1.5, mm_error(w, tgt, hm)   # explicit mm tolerance (+sim noise)
    assert "tolerance" in nav.reason and not nav.armed


def test_learns_flipped_rotation_and_camera_rotation():
    w, nav, _, _, _ = check_nav(rot_sign=1, cam_rot=math.pi)
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0


def test_learns_marker_offset_from_motion():
    w, nav, _, _, _ = check_nav(fwd_offset=math.pi / 2)     # the marker is mounted 90 deg off the rover's forward axis
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-5:])
    assert w.collisions == 0 and abs(abs(nav.marker_off) - math.pi / 2) < 0.4


def test_corridor_over_tall_block_with_measured_footprint():
    """Passage above a tall block: feasible only with the rectangular footprint, not the enclosing circle."""
    block = [(520.0, -10.0, 680.0, 470.0, 150.0)]
    w = S.SimWorld(obstacles_mm=block, rover=(250.0, 300.0, 0.0))
    per, nav, rover = make(w, margin_mm=20.0)
    nav.observe(1)
    for _ in range(6):
        per.process(w.render())
    dims = per.rover_dims_mm()
    assert dims and abs(dims[0] - 150) < 30 and abs(dims[1] - 110) < 30, dims
    run(nav, norm_of(900.0, 300.0))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert w.collisions == 0
    assert max(v for _c, _d, (_u, v, _t) in w.log if True) > 520, "route did not climb over the block"


def test_cruise_is_used_on_long_clear_stretches():
    w = S.SimWorld(obstacles_mm=[], rover=(150.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    nx, ny = norm_of(1000.0, 400.0)
    run(nav, (nx, ny))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    drives = [s for s in rover.sent if s[0] == "DRIVE"]
    assert drives, "long clear stretch should be streamed, not stop-and-go"
    assert all(abs(l) <= 0.6 and abs(r) <= 0.6 for _, (l, r) in drives)     # bounded wheel commands


def test_cruise_watchdog_stops_rover_without_fresh_poses():
    link = ugv.UgvLink(dry_run=True)
    link.drive(0.4, 0.4, ttl=0.3)
    time.sleep(0.15)
    assert link._thr.is_alive()
    time.sleep(0.7)                                              # nobody refreshed it: it must stop itself
    assert not link._thr.is_alive()
    assert link.sent[-1] == {"T": 1, "L": 0, "R": 0}
    n_sent = len(link.sent)
    time.sleep(0.3)
    assert len(link.sent) == n_sent                              # and stay stopped


def test_stale_frames_during_cruise_halt_the_rover():
    w = S.SimWorld(obstacles_mm=[], rover=(150.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    orig, state = w.wait_fresh, {"n": 0}

    def flaky(min_seq, timeout=5.0):
        if rover.world.driving is not None:
            state["n"] += 1
            if state["n"] > 1:
                return None                                      # camera feed died while cruising
        return orig(min_seq, timeout)
    w.wait_fresh = flaky
    nx, ny = norm_of(1000.0, 400.0)
    run(nav, (nx, ny))
    assert nav.state == "failed" and "lost" in nav.reason, (nav.state, nav.reason)
    assert w.driving is None and rover.sent[-1][0] == "S" and not nav.armed


def test_retarget_stops_old_executor_and_replans_from_observed_pose():
    w = S.SimWorld(obstacles_mm=[], rover=(150.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    ok, _ = nav.start(norm_of(1000.0, 400.0))
    assert ok
    t0 = time.time()
    while nav.step < 3 and nav.state == "running" and time.time() - t0 < 30:
        time.sleep(0.01)
    old = nav.thread
    start_uv = (w.u, w.v)
    ok, msg = nav.retarget(norm_of(300.0, 650.0))
    assert ok, msg
    assert not old.is_alive()                                    # old executor is gone before the new one runs
    assert nav.thread is not old and nav.target == norm_of(300.0, 650.0)
    nav.join(180)
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert math.hypot(w.u - 300.0, w.v - 650.0) < 120           # went to the NEW point, from where it was
    assert math.hypot(start_uv[0] - 300.0, start_uv[1] - 650.0) > 150


def test_differential_rover_heading_from_imu_without_marker():
    """UGV without a readable marker or pattern: position from the depth blob, heading from the controller IMU whose sign
    and offset are unknown and are tied to the camera by forward/turn/forward probes."""
    for sign, bias in ((-1, 0.9), (1, -2.0)):
        w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.5), marker=False, diff=True, imu_sign=sign, imu_bias=bias)
        per, nav, rover = make(w)
        nav.observe(1)
        run(nav, norm_of(900.0, 400.0))
        assert nav.state == "done", (sign, nav.state, nav.reason, nav.log[-6:])
        assert nav.imu["sign"] == sign and w.collisions == 0


def test_reverses_instead_of_turning_around():
    """Target directly behind the rover: drive the whole leg in reverse, no 180 degree turn, no direction chatter."""
    w = S.SimWorld(obstacles_mm=[], rover=(900.0, 400.0, 0.0))             # facing +x, target at -x
    per, nav, rover = make(w)
    nav.marker_off, nav.turn_dir = 0.0, {"Q": -1.0, "E": 1.0}              # heading already calibrated (as persisted)
    nav.observe(1)
    run(nav, norm_of(300.0, 400.0))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    turns = [s for s in rover.sent if s[0] in ("Q", "E")]
    # no turn-around. (The old limit was 0.5 rad; OpenCV 4.10 on the Pi detects the sim marker corner 1 px differently from
    # OpenCV 5.0, a 1.2 deg heading bias that makes the final approach add one ~33 deg alignment turn. 1.0 rad still
    # excludes any turn-around, which is what this test is about.)
    assert not any(abs(s[1]) > 1.0 for s in turns), turns
    rev = [s for s in rover.sent if s[0] == "B" or (s[0] == "DRIVE" and s[1][0] < 0 and s[1][1] < 0)]
    assert rev, "the leg should have been reversed"
    assert not [s for s in rover.sent if s[0] == "F" or (s[0] == "DRIVE" and s[1][0] > 0 and s[1][1] > 0)] or len(rev) > 2
    assert w.collisions == 0


def test_refuses_to_start_when_rover_is_outside_the_usable_area():
    w = S.SimWorld(obstacles_mm=[], rover=(20.0, 400.0, 0.0))              # half the rover hangs over the left boundary
    per, nav, rover = make(w)
    nav.observe(1)
    run(nav, norm_of(900.0, 400.0))
    assert nav.state == "failed" and "reset" in nav.reason, (nav.state, nav.reason)
    assert not [s for s in rover.sent if s[0] not in ("S",)], "must not move at all"


# ---------------------------------------------------------------------------------------------- fail-safe behaviour
def test_unresponsive_rover_stops_with_diagnosis():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0), responsive=False)
    per, nav, rover = make(w)
    nav.observe(1)
    run(nav, norm_of(900.0, 400.0))
    assert nav.state == "failed" and "did not move" in nav.reason, (nav.state, nav.reason)
    assert len([c for c, _ in rover.sent if c != "S"]) <= 4


def test_no_route_fails_safely():
    wall = [(520.0, -10.0, 580.0, 810.0, 120.0)]  # full barrier
    w = S.SimWorld(obstacles_mm=wall, rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    run(nav, (0.85, 0.5))
    assert nav.state == "failed" and "no route" in nav.reason, (nav.state, nav.reason)


def test_lost_tracking_stops():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    orig = w.render

    def hide():
        f = orig()
        f.gray[:] = 150
        f.depth[:] = 0
        return f
    w.render = hide
    run(nav, (0.85, 0.5))
    assert nav.state == "failed" and ("lost" in nav.reason or "not found" in nav.reason), nav.reason


def test_roi_frames_track_pose_without_remapping():
    """Rover-window frames: parsed into a padded partial frame, give a pose, never touch the map or the boundary."""
    import numpy as np
    import sources
    w = S.SimWorld(obstacles_mm=WALL, rover=(300.0, 250.0, 0.4))
    per, nav, _ = make(w)
    fr = w.render()
    obs = per.process(fr)
    px, py = obs.pose["px"]
    x0, y0 = int(px) - 100, int(py) - 100
    part = sources.Frame()
    part.bgr, part.gray, part.depth = np.zeros_like(fr.bgr), np.zeros_like(fr.gray), np.zeros_like(fr.depth)
    part.bgr[y0:y0 + 200, x0:x0 + 200] = fr.bgr[y0:y0 + 200, x0:x0 + 200]
    part.gray[y0:y0 + 200, x0:x0 + 200] = fr.gray[y0:y0 + 200, x0:x0 + 200]
    part.depth[y0:y0 + 200, x0:x0 + 200] = fr.depth[y0:y0 + 200, x0:x0 + 200]
    part.K, part.cols, part.rows, part.seq, part.wall = fr.K, fr.cols, fr.rows, fr.seq + 1, time.time()
    part.partial = True
    before_map, before_n = per.depth_map.ev.copy(), per.map_frames
    o2 = per.process(part)
    assert o2.pose is not None and abs(o2.pose["x"] - obs.pose["x"]) < 0.02 and abs(o2.pose["y"] - obs.pose["y"]) < 0.03
    assert per.map_frames == before_n and (per.depth_map.ev == before_map).all(), "partial frame must not change the map"
    assert o2.boundary_live == obs.boundary_live and per.calib.corners is not None
    assert per.track is not None and abs(per.track[0] - px) < 6


def test_run_reuses_snapshot_map_and_route():
    """One snapshot at the start; the map is frozen during the run and the route is not replanned every frame."""
    w = S.SimWorld(obstacles_mm=WALL, rover=(150.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nx, ny = norm_of(1000.0, 400.0)
    maps_before = per.map_frames
    run(nav, (nx, ny))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert per.map_frozen is False                                   # released afterwards
    assert any("snapshot" in m for m in nav.log)
    assert nav.timing["plans"] >= 1 and nav.timing["plans_reused"] > nav.timing["plans"], nav.timing
    # the only map folds during the run are the snapshot's (1 pass); frames during the run did not map
    assert per.map_frames - maps_before <= 1, (per.map_frames, maps_before)


# ---------------------------------------------------------------------------------------- periodic in-run map refresh
def _slow(world, dt=0.05):
    """Make the sim take wall time per observation so wall-clock refresh periods are meaningful."""
    orig = world.wait_fresh

    def wf(min_seq, timeout=5.0):
        time.sleep(dt)
        return orig(min_seq, timeout)
    world.wait_fresh = wf


def _refresh_run(period=0.15, obstacle=None, at_step=3, **cfg):
    w = S.SimWorld(obstacles_mm=[], rover=(150.0, 400.0, 0.0))
    _slow(w)
    per, nav, rover = make(w, map_period_s=period, **cfg)
    nav.observe(1)
    if obstacle:
        def add():
            while nav.step < at_step and nav.state != "done":
                time.sleep(0.005)
            w.obstacles.append(obstacle)
        import threading
        threading.Thread(target=add, daemon=True).start()
    return w, per, nav, rover


def test_map_refreshes_periodically_during_the_run():
    w, per, nav, rover = _refresh_run(period=0.15)
    run(nav, norm_of(1000.0, 400.0))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    stamps = [t for t, _, _ in nav.map_log]
    assert len(stamps) >= 3, nav.map_log
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    assert max(gaps) < 0.15 * 3, gaps                          # a refresh at least every few periods (~period + latency)
    seqs = [q for _, q, _ in nav.map_log]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), seqs     # never an older frame over a newer one
    assert per.map_frozen is False


def test_stale_full_frame_never_overwrites_newer_map():
    w = S.SimWorld(obstacles_mm=[], rover=(300.0, 250.0, 0.4))
    per, nav, _ = make(w)
    old, new = w.render(), w.render()
    assert per.refresh_map(new) is True
    ev = per.depth_map.ev.copy()
    assert per.refresh_map(old) is False and (per.depth_map.ev == ev).all() and per.map_seq == new.seq


def test_new_obstruction_stops_before_it_and_replans_around():
    block = (700.0, 300.0, 780.0, 520.0, 120.0)
    w, per, nav, rover = _refresh_run(period=0.1, obstacle=block, at_step=3)
    run(nav, norm_of(1000.0, 400.0))
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-8:])
    assert w.collisions == 0, nav.log[-8:]
    assert nav.timing["obstruction_stops"] >= 1 and any(b for _, _, b in nav.map_log), (nav.timing, nav.map_log)
    assert max(abs(l[2][1] - 400.0) for l in w.log) > 120.0, "route should have gone around the new obstacle"
    assert any("ROUTE NOW BLOCKED" in m for m in nav.log)


def test_no_route_after_refresh_holds_stopped_and_reports():
    wall = (700.0, 0.0, 780.0, float(S.H_MM), 120.0)
    w, per, nav, rover = _refresh_run(period=0.1, obstacle=wall, at_step=3, no_route_hold_s=0.6)
    run(nav, norm_of(1000.0, 400.0))
    assert nav.state == "failed" and "replan" in nav.reason, (nav.state, nav.reason, nav.log[-8:])
    assert w.collisions == 0 and w.driving is None and rover.sent[-1][0] == "S" and nav.armed is False
    assert any("NO ROUTE" in m for m in nav.log)


def test_map_refresh_failure_stops_the_run():
    w, per, nav, rover = _refresh_run(period=0.1, map_max_age_s=0.5)
    w.latest_full = lambda: None                               # no full frame can be obtained any more
    run(nav, norm_of(1000.0, 400.0))
    assert nav.state == "failed" and "obstacle map" in nav.reason, (nav.state, nav.reason, nav.log[-6:])
    assert w.driving is None and rover.sent[-1][0] == "S" and nav.armed is False


def test_tracking_and_stop_stay_responsive_during_a_slow_refresh():
    w, per, nav, rover = _refresh_run(period=0.1)
    orig = per.refresh_map
    per.refresh_map = lambda fr, repeats=3: (time.sleep(2.0), orig(fr, repeats))[1]     # mapping takes 2 s
    ok, msg = nav.start(norm_of(1000.0, 400.0))
    assert ok, msg
    t_end = time.time() + 5
    while time.time() < t_end and not (per._mapper is not None and per._mapper.is_alive()):
        time.sleep(0.01)
    assert per._mapper is not None and per._mapper.is_alive(), "mapper never started"
    step0, frames0 = nav.step, nav.timing["frames"]
    time.sleep(0.4)
    if nav.state == "running":
        assert nav.step > step0 or nav.timing["frames"] > frames0, "control loop blocked while mapping"
        t = time.time()
        nav.stop()
        assert time.time() - t < 0.5 and rover.sent[-1][0] == "S" and w.driving is None
    nav.join(10)
    assert nav.state != "running"


def test_saved_pattern_heading_is_reused_without_probes_and_clearance_is_explained():
    w = S.SimWorld(obstacles_mm=WALL, rover=(300.0, 250.0, 0.4))
    per, nav, _ = make(w)
    obs = per.process(w.render())
    obs.pose["theta"], obs.pose["theta_dec"], obs.pose["source"] = None, None, "depth-blob"
    obs.pose["theta_raw"] = 0.3
    nav.saved_heading = {"hbias": 0.1, "turn_dir": {"Q": 1.0, "E": -1.0}, "t": time.time() - 3 * 3600}   # hours old
    nav.hbias, nav.turn_dir = None, None

    def boom():
        raise AssertionError("heading calibration probes must not run when valid settings are saved")
    nav.calibrate_pattern_heading = boom
    o2, why = nav._calibrate_if_needed(obs)
    assert why == "" and nav.hbias == 0.1 and nav.turn_dir["Q"] == 1.0
    txt = nav._clearance_detail(obs.pose, [(0.40, 0.40, 0.42, 0.45)])
    assert "mm from the centre" in txt and "nearest edge" in txt and "footprint" in txt, txt


def test_heading_average_does_not_lag_while_the_rover_turns():
    """The 3-frame heading average smooths stationary noise but must follow a turn immediately (no lag that would let a
    still-rotating rover look settled)."""
    import numpy as np
    import vision as V
    w = S.SimWorld(obstacles_mm=[], rover=(400.0, 400.0, 0.0), marker=False, diff=True)
    per, nav, _ = make(w)
    seen = []
    orig = V.pattern_heading
    seq = iter([0.0, 0.02, 0.0, 0.5, 1.0, 1.0, 1.01])
    V.pattern_heading = lambda *a, **k: next(seq)
    try:
        for _ in range(7):
            o = per.process(w.render())
            if o.pose is not None and o.pose.get("theta_raw") is not None:
                seen.append(o.pose["theta_raw"])
    finally:
        V.pattern_heading = orig
    assert len(seen) >= 6, seen
    assert abs(seen[3] - 0.5) < 0.02 and abs(seen[4] - 1.0) < 0.02, seen       # follows the turn at once
    assert abs(seen[-1] - 1.0) < 0.03


def test_stop_disarms_and_requires_arm():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.armed = False
    ok, msg = nav.start((0.8, 0.5))
    assert not ok and "armed" in msg
    nav.armed = True
    nav.stop()
    assert nav.armed is False and rover.sent[-1][0] == "S"


# ---------------------------------------------------------------------------------------------- depth quality
def test_ground_surface_removes_fixed_tabletop_ripple_but_not_objects():
    """The Aurora depth has a stable +-35 mm ripple over the tabletop (as large as the obstacle threshold). The fitted ground
    surface must absorb it, but never a raised object or the rover."""
    import numpy as np
    import vision as V
    H, W = 400, 640
    K = (420.0, 420.0, 320.0, 200.0)
    ys, xs = np.mgrid[0:H, 0:W]
    ripple = 18.0 * np.sin(xs / 37.0) + 12.0 * np.cos(ys / 23.0 + xs / 90.0)
    depth = 1900.0 + ripple
    block = (slice(150, 200), slice(300, 360))
    depth[block] -= 100.0                                          # a 100 mm tall object
    depth = depth.astype(np.uint16)
    mask = np.zeros((H, W), bool)
    mask[60:340, 40:600] = True
    plane, grid = V.fit_ground_surface(depth, K, mask)
    c = V.Calibration()
    c.plane = plane
    c.K = K
    raw = V.plane_residual(depth, K, plane)
    c.set_corr(grid)
    fixed = raw - c.corr_full(depth.shape)
    table = mask.copy()
    table[130:220, 280:380] = False
    assert np.nanstd(fixed[table]) < 0.4 * np.nanstd(raw[table]), (np.nanstd(fixed[table]), np.nanstd(raw[table]))
    assert np.nanmax(np.abs(fixed[table])) < 18.0, np.nanmax(np.abs(fixed[table]))     # well under the 35 mm threshold
    assert np.nanmean(fixed[160:190, 310:350]) > 85.0, "the raised object was absorbed into the ground model"
    d2 = V.Calibration.from_dict(c.to_dict())                       # persisted with the calibration
    assert d2.corr is not None and abs(float(d2.corr[5, 5]) - float(c.corr[5, 5])) < 0.06


def test_cells_without_depth_are_never_free():
    """Dark/shiny objects return no depth. A cell that was seen free earlier and now has (mostly) invalid depth must become
    unknown (blocked for the planner), not stay free; isolated dropouts must not."""
    import numpy as np
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0), noise=False)
    per, nav, _ = make(w)
    c, dm = per.calib, per.depth_map
    f = w.render()
    cx, cy = [int(v) for v in c.to_px([(0.6, 0.5)])[0]]
    for _ in range(3):
        dm.update(f.depth, c, rover=None, rover_exclude=0.0)
    cell = (int(0.6 * dm.cols), int(0.5 * dm.rows))
    assert dm.seen[cell[1], cell[0]] and dm.ev[cell[1], cell[0]] < 0.5
    d2 = f.depth.copy()
    rng = np.random.default_rng(5)
    d2[rng.random(d2.shape) < 0.04] = 0                             # isolated dropouts only: still free
    for _ in range(3):
        dm.update(d2, c, rover=None, rover_exclude=0.0)
    assert dm.seen[cell[1], cell[0]] and dm.ev[cell[1], cell[0]] < 0.5
    d2[cy - 22:cy + 22, cx - 22:cx + 22] = 0                        # an object that returns no depth at all
    for _ in range(3):
        dm.update(d2, c, rover=None, rover_exclude=0.0)
    assert not dm.seen[cell[1], cell[0]], "no usable depth must not stay labelled free"
    assert any(abs(r[0] - cell[0] / dm.cols) < 1e-6 and abs(r[1] - cell[1] / dm.rows) < 1e-6 for r in dm.rects(unknown=True)),         "the planner must treat it as blocked"


# ---------------------------------------------------------------------------------------------- unsafe-action recovery
def _moves(rover):
    return [x for x in rover.sent if x[0] != "S"]


def test_rejected_action_stops_then_recovers_to_the_same_target():
    """A rejected action must not end the run: STOP, fresh full frame + pose, replan toward the SAME exact target, go on."""
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    real, calls = nav._unsafe, []

    def once(pose, act):
        calls.append(act.get("text"))
        return "forced rejection (test)" if len(calls) == 1 else real(pose, act)
    nav._unsafe = once
    tgt = norm_of(900.0, 400.0)
    run(nav, tgt)
    assert nav.state == "done", (nav.state, nav.reason, nav.log[-6:])
    assert nav.target == tgt and nav.timing["unsafe_stops"] == 1 and nav.timing["unsafe_recovered"] >= 1
    assert any("UNSAFE action stopped" in l for l in nav.log) and w.collisions == 0
    assert mm_error(w, tgt, per.calib.height_mm) <= 50.0


def test_stale_occupancy_is_cleared_by_recovery_but_real_obstacles_stay():
    """A ghost cell (stale occupancy) that makes the guard reject is cleared by the fresh frame; the real wall is kept."""
    w = S.SimWorld(obstacles_mm=WALL, rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.marker_off, nav.turn_dir = 0.0, {"Q": -1.0, "E": 1.0}
    nav.observe(1)
    nav.target = norm_of(900.0, 400.0)
    per.map_frozen = True
    dm = per.depth_map
    gx, gy = int(0.30 * dm.cols), int(0.5 * dm.rows)              # a ghost just ahead of the rover (free table in reality)
    dm.ev[gy - 1:gy + 2, gx - 1:gx + 2] = 1.0
    wall_cells = [(c, r) for c, r in dm.blocked_cells() if 0.42 < (c + .5) / dm.cols < 0.5 and 0.4 < (r + .5) / dm.rows < 0.6]
    assert len(wall_cells) >= 6
    pose = per.last_obs.pose
    act = {"kind": "burst", "cmd": "F", "dur": 0.5, "speed": 0.3, "trim": 0.0, "text": "F test"}
    obs, bad = nav._recover_unsafe(pose, act, "ghost ahead (test)")
    assert bad is None and obs is not None, bad
    assert float(per.depth_map.ev[gy, gx]) < 0.5, "stale cell should be gone after the fresh frame"
    still = [(c, r) for c, r in per.depth_map.blocked_cells() if 0.42 < (c + .5) / dm.cols < 0.5 and 0.4 < (r + .5) / dm.rows < 0.6]
    assert len(still) >= 6, "real obstacle cells must stay occupied"
    assert not _moves(rover), "recovery from a map-only problem must not move the rover"


def test_unsafe_action_with_a_real_close_obstacle_backs_away_through_the_guard():
    """The planned action keeps being unsafe (planner/guard disagreement): a short guarded back-away is allowed, nothing
    else; the rover ends further from the obstacle and the run is not blindly retried."""
    w = S.SimWorld(obstacles_mm=WALL, rover=(400.0, 400.0, 0.0))     # facing the wall, front ~35 mm from it
    per, nav, rover = make(w)
    nav.marker_off, nav.turn_dir = 0.0, {"Q": -1.0, "E": 1.0}
    nav.observe(1)
    nav.target = norm_of(900.0, 400.0)
    per.map_frozen = True
    fwd = {"kind": "burst", "cmd": "F", "dur": 1.0, "speed": 0.3, "trim": 0.0, "text": "F 0.50s (test)"}
    nav._plan_action = lambda pose, res: dict(fwd)                  # a planner that keeps asking for the unsafe move
    pose = per.last_obs.pose
    assert nav._unsafe(pose, fwd), "the forward move must be rejected here"
    u0 = w.u
    obs, bad = nav._recover_unsafe(pose, fwd, nav._unsafe(pose, fwd))
    assert bad is None and obs is not None, bad
    sent = _moves(rover)
    assert sent and all(x[0] in ("B", "Q", "E") for x in sent), sent   # never forward into the obstacle
    assert w.collisions == 0 and (w.u < u0 or any(x[0] in ("Q", "E") for x in sent))


def test_genuinely_blocked_stays_stopped_and_reports_the_obstruction():
    barrier = [(520.0, -10.0, 580.0, 810.0, 120.0)]
    w = S.SimWorld(obstacles_mm=barrier, rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    nav.target = (0.85, 0.5)
    per.map_frozen = True
    pose = per.last_obs.pose
    act = {"kind": "burst", "cmd": "F", "dur": 0.5, "speed": 0.3, "trim": 0.0, "text": "F test"}
    obs, bad = nav._recover_unsafe(pose, act, "wall ahead (test)")
    assert obs is None and bad and bad.startswith("blocked:") and "no route" in bad and "wall ahead" in bad, bad
    assert not _moves(rover)


def test_repeated_unsafe_actions_are_bounded_and_do_not_oscillate():
    w = S.SimWorld(obstacles_mm=[], rover=(250.0, 400.0, 0.0))
    per, nav, rover = make(w)
    nav.observe(1)
    nav._unsafe = lambda pose, act: "always unsafe (test)"
    run(nav, norm_of(900.0, 400.0))
    assert nav.state == "failed" and nav.reason.startswith("blocked:") and "always unsafe" in nav.reason, (nav.state, nav.reason)
    assert nav.armed is False and nav.timing["unsafe_stops"] <= 3
    assert len(_moves(rover)) <= 6 and w.collisions == 0, _moves(rover)


if __name__ == "__main__":
    fails = 0
    for k, v in list(globals().items()):
        if k.startswith("test_"):
            t0 = time.time()
            try:
                v()
                print("PASS", k, "%.1fs" % (time.time() - t0))
            except Exception as e:
                import traceback
                traceback.print_exc()
                print("FAIL", k, repr(e)[:400])
                fails += 1
    sys.exit(1 if fails else 0)
