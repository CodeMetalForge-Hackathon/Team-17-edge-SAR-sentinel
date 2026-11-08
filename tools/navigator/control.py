"""Closed-loop navigation: plan -> one bounded burst -> STOP -> fresh frame + pose -> replan, until the target.

The rover's real motion conventions (which of Q/E turns left, whether F moves along the marker 'up' edge, speeds) are
unverified, so they are learned online from the observed pose change after each burst and saved. Nothing here sends
continuous commands: every burst is short, followed by STOP, and the next burst requires a frame captured after it.
"""
import json
import math
import os
import threading
import time

import geom

MAX_BURST_S = 0.5            # navigator cap (the link itself caps at 1.0 s)
POS_TOL = 0.12               # reached the target within 5% of sandbox height (square frame)
ANG_TOL = math.radians(18)   # heading error below which we drive forward
SETTLE_POS, SETTLE_ANG = 0.012, math.radians(4)


class MotionModel:
    """Per-command effect per second: F/B -> (forward, left) speed in the marker frame (square-frame units/s);
    Q/E -> signed yaw rate (rad/s). Priors are guesses; observations replace them."""

    def __init__(self):
        self.v = {"F": [0.12, 0.0], "B": [-0.12, 0.0]}
        self.w = {"Q": math.radians(45), "E": -math.radians(45)}
        self.n = {"F": 0, "B": 0, "Q": 0, "E": 0}

    def to_dict(self):
        return {"v": self.v, "w": self.w, "n": self.n}

    def load(self, d):
        try:
            self.v, self.w, self.n = d["v"], d["w"], d["n"]
        except Exception:
            pass

    def observe_move(self, cmd, dur, d_fwd, d_left):
        if dur < 0.05:
            return
        k = 1.0 if self.n[cmd] == 0 else 0.5
        self.v[cmd] = [(1 - k) * self.v[cmd][0] + k * d_fwd / dur, (1 - k) * self.v[cmd][1] + k * d_left / dur]
        self.n[cmd] += 1
        opp = {"F": "B", "B": "F"}[cmd]   # opposite command should do the opposite; seed it if never observed
        if self.n[opp] == 0:
            self.v[opp] = [-self.v[cmd][0], -self.v[cmd][1]]

    def observe_turn(self, cmd, dur, dth):
        if dur < 0.05:
            return
        k = 1.0 if self.n[cmd] == 0 else 0.5
        self.w[cmd] = (1 - k) * self.w[cmd] + k * dth / dur
        self.n[cmd] += 1
        opp = {"Q": "E", "E": "Q"}[cmd]
        if self.n[opp] == 0:
            self.w[opp] = -self.w[cmd]


class Navigator:
    def __init__(self, perception, source, rover, store_dir, cfg=None, now=time.time):
        self.p, self.src, self.rover = perception, source, rover
        self.dir = store_dir
        self.cfg = {"cols": 120, "rows": 40, "radius": 0.07, "margin": 0.02, "max_steps": 160, "max_seconds": 600,
                    "fwd_burst": 0.30, "no_progress": 10,
                    # physical rover size / clearance (mm); converted with the measured sandbox height when known
                    "rover_w_mm": None, "rover_l_mm": None, "margin_mm": None}
        self.cfg.update(cfg or {})
        self.model = MotionModel()
        self.holonomic = False        # True while the pose has no heading (depth-blob tracking): strafe, never turn
        self.probe_disp = {}
        self._factor = 1.0
        self._turn_target = None
        self._calib_t = 0.0
        self._pattern_mode = False
        self.hbias = None             # rad: heading = pattern_heading + hbias (marker/rover offset, learned)
        self.turn_dir = None          # {'Q': +-1, 'E': +-1}: heading change (CCW +) per rotation burst, IMU mode
        self.turn_bad = 0
        self.imu = None               # {'sign': +-1, 'offset': rad}: world heading = sign*IMU_yaw + offset
        self.hv = {}                  # learned WORLD-frame velocity (square-frame units/s) per command F/B/L/R
        self.now = now
        self.saved_heading = None
        try:
            md = json.load(open(os.path.join(store_dir, "model.json")))
            self.model.load(md)
            self.saved_heading = md.get("heading")
        except Exception:
            pass
        self.target = None            # normalized (x, y)
        self.state = "idle"           # idle | running | done | failed
        self.reason = ""
        self.step = 0
        self.log = []
        self.path = []
        self.pose = None
        self.dist = None
        self.armed = False
        self.stop_flag = threading.Event()
        self.thread = None
        self.lock = threading.Lock()

    # ---- helpers
    def _log(self, msg):
        self.log.append("%s %s" % (time.strftime("%H:%M:%S"), msg))
        self.log = self.log[-60:]

    def save_model(self):
        d = self.model.to_dict()
        if self.hbias is not None and self.turn_dir is not None:
            d["heading"] = {"hbias": self.hbias, "turn_dir": self.turn_dir, "t": time.time()}
        json.dump(d, open(os.path.join(self.dir, "model.json"), "w"))

    def _dist_sq(self, a, b):
        A = self.p.calib.aspect
        return math.hypot((a[0] - b[0]) * A, a[1] - b[1])

    def footprint(self):
        """(L, W, margin) in sandbox-height units: L along the rover heading, W across. Taken from the depth-measured
        rover (once its marker pose is known), else the configured mm size, else the dimensionless fallback."""
        c, cal = self.cfg, self.p.calib
        hm = cal.height_mm
        if self.holonomic and hm:
            a = self.p.rover_aabb_mm()
            if a:
                return a[0] / hm, a[1] / hm, (c["margin_mm"] or 0) / hm
        dims = self.p.rover_dims_mm()
        if hm and dims:
            L, W = dims
        elif hm and c["rover_w_mm"] and c["rover_l_mm"]:
            L, W = c["rover_l_mm"], c["rover_w_mm"]
        else:
            r = c["radius"]
            return r * 1.414, r * 1.414, c["margin"]
        vf, vl = self.model.v["F"]
        if self.model.n["F"] and abs(math.atan2(vl, vf)) > math.pi / 4:   # F moves sideways relative to the marker axes
            L, W = W, L
        return L / hm, W / hm, (c["margin_mm"] or 0) / hm

    def clearance(self):
        """(enclosing radius, margin) for display."""
        L, W, m = self.footprint()
        return math.hypot(L, W) / 2, m

    def make_plan(self, pos):
        c = self.cfg
        L, W, m = self.footprint()
        return geom.plan_oriented(pos, self.target, self.p.obstacle_rects(near=pos), c["cols"], c["rows"], L, W, m,
                                  self.p.calib.aspect, fixed=self.holonomic)

    def preview(self):
        """Plan from the last known pose for display (no motion)."""
        if self.target is None or not self.pose:
            return None
        return self.make_plan((self.pose["x"], self.pose["y"]))

    # ---- heading from the controller's IMU when no marker is readable (differential-drive rover)
    def _yaw_rad(self):
        f = getattr(self.rover, "yaw_deg", None)
        if f is None:
            return None
        try:
            y = f()
        except Exception:
            return None
        return None if y is None else math.radians(y)

    def _apply_heading(self, obs):
        p = obs.pose
        if p is None or p.get("theta") is not None:
            return
        if self.hbias is not None and p.get("theta_raw") is not None:
            p["theta"] = geom.wrap(p["theta_raw"] + self.hbias)
            p["source"] = "depth-blob + marker-pattern heading"
            return
        if self.imu is None:
            return
        y = self._yaw_rad()
        if y is None:
            obs.pose = None                      # heading source lost: treat as lost tracking
            return
        p["theta"] = geom.wrap(self.imu["sign"] * y + self.imu["offset"])
        p["source"] = "depth-blob + IMU heading"

    def _move(self, cmd, dur, speed=None):
        self.p.reset_heading()
        if cmd in ("Q", "E") and self._turn_target and getattr(self.rover, "turn_by", None) is not None:
            tgt, self._turn_target = self._turn_target, None
            got = self.rover.turn_by(cmd, tgt)
            self._log("IMU-closed turn %s: target %.0f deg, turned %.0f deg" % (cmd, math.degrees(tgt), math.degrees(got)))
            return
        if speed is not None:
            try:
                return self.rover.move(cmd, dur, speed=speed)
            except TypeError:
                pass
        return self.rover.move(cmd, dur)

    def _burst_and_observe(self, cmd, dur, quick=False, speed=None):
        self._move(cmd, dur, speed)
        return self.observe(self.src.current_seq() + 2, quick=quick)

    def calibrate_pattern_heading(self):
        """Tie the (few-pixel) marker pattern to the direction of travel and find which of Q/E turns which way:
        forward probe -> bias; two turn pulses -> direction. Returns '' on success else the reason."""
        A = self.p.calib.aspect
        self._log("heading calibration (marker pattern): forward and turn probes")
        o0 = self.observe(self.src.current_seq() + 1)
        if o0 is None or o0.pose is None or o0.pose.get("theta_raw") is None:
            return "marker pattern heading not readable"
        raw0, p0 = o0.pose["theta_raw"], o0.pose
        for dur in (0.3, 0.5):
            o1 = self._burst_and_observe("F", dur, quick=True, speed=0.4)
            if o1 is None or o1.pose is None:
                return "pose lost during the forward probe"
            d = ((o1.pose["x"] - p0["x"]) * A, o1.pose["y"] - p0["y"])
            if math.hypot(*d) >= 0.015:
                break
            p0 = o1.pose
        else:
            return "rover did not move on the forward probes (no measurable displacement)"
        phi = math.atan2(d[1], d[0])
        self.hbias = geom.wrap(phi - raw0)
        self.model.observe_move("F", dur, math.hypot(*d) / (0.4 / 0.35), 0.0)
        self._log("forward probe: moved %.3f in %.2fs; marker-pattern offset %.0f deg" % (math.hypot(*d), dur, math.degrees(self.hbias)))
        o1 = self.observe(self.src.current_seq() + 1, quick=True)
        th_a = geom.wrap(o1.pose["theta_raw"] + self.hbias) if o1 and o1.pose and o1.pose.get("theta_raw") is not None else None
        if th_a is None:
            return "marker pattern lost after the forward probe"
        for k in range(3):
            o2 = self._burst_and_observe("Q", 0.5, quick=False)
            if o2 is None or o2.pose is None or o2.pose.get("theta_raw") is None:
                return "marker pattern lost during the turn probe"
            th_b = geom.wrap(o2.pose["theta_raw"] + self.hbias)
            dth = geom.wrap(th_b - th_a)
            if abs(dth) >= 0.2:
                self.turn_dir = {"Q": 1.0 if dth > 0 else -1.0, "E": -1.0 if dth > 0 else 1.0}
                self.turn_bad = 0
                self._log("turn probe: Q rotated %.0f deg (%d pulse%s) -> Q is %s" % (math.degrees(dth), k + 1, "" if k == 0 else "s", "CCW" if dth > 0 else "CW"))
                return ""
        return "rover did not rotate on three 0.5 s turn pulses"

    def calibrate_heading(self):
        """Bounded probes that tie the IMU yaw to the camera frame: F, turn, F. Returns '' on success else the reason."""
        A = self.p.calib.aspect
        self._log("heading calibration: forward / turn / forward probes")
        o0 = self.observe(self.src.current_seq() + 1)
        if o0 is None or o0.pose is None:
            return "no pose for heading calibration"
        p0 = o0.pose

        def forward(pb):
            for dur in (0.25, 0.45):
                o = self._burst_and_observe("F", dur)
                if o is None or o.pose is None:
                    return None, None, None, "pose lost during the forward probe"
                d = ((o.pose["x"] - pb["x"]) * A, o.pose["y"] - pb["y"])
                if math.hypot(*d) >= 0.012:
                    return o, d, dur, ""
                pb = o.pose
            return None, None, None, "rover did not move on the forward probes (no measurable displacement)"
        o1, d1, dur1, why = forward(p0)
        if why:
            return why
        phi1, y1 = math.atan2(d1[1], d1[0]), self._yaw_rad()
        self.model.observe_move("F", dur1, math.hypot(*d1), 0.0)
        dpsi, dur_t = 0.0, 0.35
        for dur_t in (0.08, 0.12, 0.2):
            o2 = self._burst_and_observe("Q", dur_t)
            y2 = self._yaw_rad()
            if o2 is None or o2.pose is None:
                o2 = self.observe(self.src.current_seq() + 2)          # one retry for a missed blob frame
            if o2 is None or o2.pose is None:
                return "rover position lost after the turn probe (burst %.2fs)" % dur_t
            if y1 is None or y2 is None:
                return "IMU yaw unavailable (before=%s after=%s)" % (y1, y2)
            dpsi = geom.wrap(y2 - y1)
            if abs(dpsi) >= 0.06:
                break
            y1 = y2
        if abs(dpsi) < 0.06:
            return "IMU yaw did not change during the turn probe"
        o3, d3, dur3, why = forward(o2.pose)
        if why:
            return why
        phi2, y3 = math.atan2(d3[1], d3[0]), self._yaw_rad()
        dphi = geom.wrap(phi2 - phi1)
        sign = 1.0 if dphi * dpsi > 0 else -1.0
        if abs(abs(dphi) - abs(dpsi)) > max(0.35, 0.5 * abs(dpsi)):
            self._log("WARNING: camera-measured turn %.0f deg vs IMU %.0f deg disagree" % (math.degrees(dphi), math.degrees(dpsi)))
        self.imu = {"sign": sign, "offset": geom.wrap(phi2 - sign * y3)}
        self.model.observe_turn("Q", dur_t, sign * dpsi)
        self.turn_dir = {"Q": 1.0 if sign * dpsi > 0 else -1.0, "E": -1.0 if sign * dpsi > 0 else 1.0}
        self.turn_bad = 0
        self._log("heading calibrated: IMU sign %+d, offset %.0f deg; Q turns %.0f deg/s, F moves %.3f/s"
                  % (sign, math.degrees(self.imu["offset"]), math.degrees(sign * dpsi / dur_t), math.hypot(*d1) / dur1))
        return ""

    # ---- fresh, settled observation
    def observe(self, min_seq, timeout=6.0, quick=False):
        """Return an Obs with a rover pose from frames captured at/after min_seq and consistent across two frames."""
        prev, tries, t_end = None, 0, self.now() + timeout
        seq = min_seq
        while self.now() < t_end and not self.stop_flag.is_set():
            fr = self.src.wait_fresh(seq, timeout=max(0.5, t_end - self.now()))
            if fr is None:
                continue
            obs = self.p.process(fr)
            self._apply_heading(obs)
            seq = fr.seq + 1
            tries += 1
            if obs.pose is None:
                prev = None
                if tries >= 8:
                    return obs
                continue
            if obs.pose is not None and obs.pose.get("theta") is None and obs.pose.get("theta_raw") is None                     and (self.hbias is not None or self._pattern_mode) and tries < 6:
                prev = None                      # pattern not readable in this frame: try the next one
                continue
            if quick and obs.pose is not None:
                return obs
            if prev is not None:
                dp = math.hypot((obs.pose["x"] - prev["x"]) * self.p.calib.aspect, obs.pose["y"] - prev["y"])
                dt = 0.0 if obs.pose.get("theta") is None or prev.get("theta") is None else abs(geom.wrap(obs.pose["theta"] - prev["theta"]))
                if dp < SETTLE_POS and dt < SETTLE_ANG:
                    return obs
            prev = obs.pose
            if tries >= 5:
                return obs
        return None

    # ---- run control
    def start(self, target):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False, "already running"
            if not self.armed:
                return False, "motion is not armed"
            self.target = tuple(target)
            self.stop_flag.clear()
            self.state, self.reason, self.step = "running", "", 0
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True, "started"

    def stop(self):
        """Immediate: signal the loop and send STOP. Calibration, map and model are kept."""
        self.stop_flag.set()
        ok = self.rover.stop()
        self.armed = False
        if self.state == "running":
            self.state, self.reason = "idle", "stopped by operator"
        self._log("STOP %s; disarmed" % ("sent" if ok else "FAILED"))
        return ok

    def join(self, t=30):
        if self.thread:
            self.thread.join(t)

    def _fail(self, why):
        self.rover.stop()
        self.armed = False
        self.state, self.reason = "failed", why
        self._log("FAILED: " + why)
        self.save_model()
        self.p.save()

    def _run(self):
        try:
            self._run_inner()
        except Exception as e:   # never leave the rover in an undefined state: STOP, disarm, report
            import traceback
            self._log("internal error: " + traceback.format_exc().splitlines()[-1])
            self._fail("internal error: %s" % e)

    def _run_inner(self):
        c = self.cfg
        t0 = self.now()
        self._log("run started; reacquiring pose")
        obs = self.observe(self.src.current_seq() + 1)
        if obs is not None and obs.pose is not None and obs.pose.get("theta") is None and obs.pose.get("theta_raw") is not None:
            sh = self.saved_heading or ({"hbias": self.hbias, "turn_dir": self.turn_dir, "t": self._calib_t} if self.hbias is not None and self.turn_dir else None)
            if sh and time.time() - sh["t"] < 900:
                self.hbias, self.turn_dir = sh["hbias"], sh["turn_dir"]
                self._log("reusing heading calibration (offset %.0f deg); verified by the next forward bursts" % math.degrees(self.hbias))
                self.imu = None
                obs = self.observe(self.src.current_seq() + 1, quick=True)
            else:
                self.hbias, self.turn_dir, self.imu = None, None, None
                self._pattern_mode = True
                why = self.calibrate_pattern_heading()
                if why:
                    return self._fail("heading calibration failed: " + why)
                self._calib_t = time.time()
                obs = self.observe(self.src.current_seq() + 1, quick=True)
        elif obs is not None and obs.pose is not None and obs.pose.get("theta") is None and getattr(self.rover, "yaw_deg", None) is not None:
            self.imu = None
            why = self.calibrate_heading()
            if why:
                return self._fail("heading calibration failed: " + why)
            obs = self.observe(self.src.current_seq() + 1)
        best_rem, bad_dist, last_cmd, turns_in_row = None, 0, None, 0
        self.remaining = None
        pending = None   # (cmd, dur, pose_before) awaiting its effect measurement
        try:
            while not self.stop_flag.is_set():
                if obs is None or obs.pose is None:
                    return self._fail("rover marker not found in a fresh frame (pose lost)")
                pose = obs.pose
                self.pose = pose
                self.holonomic = pose.get("theta") is None
                if not pose["inside"]:
                    return self._fail("rover is outside the sandbox boundary")
                # learn from the previous burst
                if pending:
                    cmd, dur, pb = pending
                    self._learn(cmd, dur, pb, pose)
                    pending = None
                pos = (pose["x"], pose["y"])
                self.dist = self._dist_sq(pos, self.target)
                if self.dist <= POS_TOL:
                    self.state, self.reason = "done", "reached target (%.3f from it)" % self.dist
                    self._log(self.reason)
                    self.rover.stop()
                    self.armed = False
                    self.save_model()
                    self.p.save()
                    return
                if self.step >= c["max_steps"]:
                    return self._fail("step limit reached")
                if self.now() - t0 > c["max_seconds"]:
                    return self._fail("time limit reached")
                res = self.make_plan(pos)
                self.path = res["waypoints"]
                if res["error"]:
                    return self._fail("replan: " + res["error"])
                # no-progress watchdog on the REMAINING ROUTE LENGTH (the route may first lead away from the target)
                rem = geom.path_length_sq(res["waypoints"], self.p.calib.aspect)
                self.remaining = rem
                if last_cmd in ("F", "B", "L", "R") and not (self.holonomic and len(self.hv) < 2):
                    if best_rem is not None and rem > best_rem - 0.004:
                        bad_dist += 1
                    else:
                        bad_dist = 0
                if best_rem is None or rem < best_rem:
                    best_rem = rem
                if bad_dist >= c["no_progress"]:
                    return self._fail("no progress along the route for %d translation bursts" % bad_dist)
                if turns_in_row >= 12:
                    return self._fail("cannot align the heading (12 rotation bursts without reaching it)")
                cmd, dur = (self._choose_holo(pose, res["waypoints"]) if self.holonomic else self._choose(pose, res["waypoints"]))
                if cmd is None:
                    return self._fail(dur)
                # safety (margin 0, the rover's true footprint): a translation burst must stay clear for the footprint
                # at the current heading; a large rotation must not sweep the enclosing circle into an obstacle
                speed = self._speed_for(cmd, pose)
                self._factor = (speed / 0.35) if speed else 1.0
                if speed and cmd in ("F", "B"):
                    dur = round(max(0.10, min(0.5, dur / self._factor)), 2)
                why = self._unsafe(pose, cmd, dur)
                if why:
                    return self._fail(why)
                self.step += 1
                self._log("step %d: %s %.2fs  dist=%.3f  %s" % (self.step, cmd, dur, self.dist, "heading=%.0f deg" % math.degrees(pose["theta"]) if pose.get("theta") is not None else "position-only (" + pose.get("source", "?") + ")"))
                if self.stop_flag.is_set() or not self.armed:
                    break
                try:
                    self._move(cmd, dur, speed)
                except Exception as e:  # comm error: stop, disarm
                    return self._fail("rover comm error: %s" % e)
                pending = (cmd, dur, pose)
                last_cmd = cmd
                turns_in_row = turns_in_row + 1 if cmd in ("Q", "E") else 0
                obs = self.observe(self.src.current_seq() + 2, quick=cmd in ("F", "B"))
        finally:
            if self.state == "running":
                self.state, self.reason = "idle", "stopped"
            self.save_model()
            self.p.save()

    def _aim(self, pos, wps):
        """Follow the planned polyline without cutting corners: aim at a point 0.10 ahead on the current segment."""
        A = self.p.calib.aspect
        sq = lambda p: (p[0] * A, p[1])
        P = sq(pos)
        seg = len(wps) - 2
        for i in range(len(wps) - 1):
            if math.hypot(*(a - b for a, b in zip(sq(wps[i + 1]), P))) > 0.04:
                seg = i
                break
        s0, s1 = sq(wps[seg]), sq(wps[seg + 1])
        dx, dy = s1[0] - s0[0], s1[1] - s0[1]
        n = math.hypot(dx, dy) or 1e-9
        ux, uy = dx / n, dy / n
        t = max(0.0, min(n, (P[0] - s0[0]) * ux + (P[1] - s0[1]) * uy))
        t2 = min(n, t + 0.10)
        aim = (s0[0] + ux * t2, s0[1] + uy * t2)
        return (aim[0] / A, aim[1]), seg

    def _unsafe(self, pose, cmd, dur):
        c, A = self.cfg, self.p.calib.aspect
        L, W, _m = self.footprint()
        pos = (pose["x"], pose["y"])
        obs = self.p.obstacle_rects(near=pos)
        bt = geom.planning_blocked(obs, c["cols"], c["rows"], math.hypot(L, W) / 2, A)
        here = geom.cell_of(pos[0], pos[1], c["cols"], c["rows"])
        if self.holonomic:
            v = self.hv.get(cmd)
            if not v:
                return ""
            nu, nv = pose["x"] * A + v[0] * dur, pose["y"] + v[1] * dur
            cell = geom.cell_of(nu / A, nv, c["cols"], c["rows"])
            hm = self.p.calib.height_mm or 700.0
            a = self.p.rover_aabb_mm() or (200.0, 200.0)
            blk = geom.rect_blocked(obs, c["cols"], c["rows"], 0.7 * a[0] / 2 / hm, 0.7 * a[1] / 2 / hm, A)   # light contact tolerated
            if cell in blk and cell != here:
                return "next burst would bring the rover footprint into an obstacle or the table edge"
            return ""
        th = pose["theta"]
        if cmd in ("F", "B"):
            vf, vl = self.model.v[cmd]
            pu = pose["x"] * A + (math.cos(th) * vf - math.sin(th) * vl) * dur * self._factor
            pv = pose["y"] + (math.sin(th) * vf + math.cos(th) * vl) * dur * self._factor
            cell = geom.cell_of(pu / A, pv, c["cols"], c["rows"])
            ex = max((L * abs(math.cos(th + d)) + W * abs(math.sin(th + d))) / 2 for d in (-0.14, 0.0, 0.14))
            ey = max((L * abs(math.sin(th + d)) + W * abs(math.cos(th + d))) / 2 for d in (-0.14, 0.0, 0.14))
            blk = geom.rect_blocked(obs, c["cols"], c["rows"], ex, ey, A)   # exact AABB of the rotated footprint (+-8 deg)
            if cell in blk and cell != here:
                return "next burst would bring the rover footprint into an obstacle or the table edge"
        elif here in bt:
            return "a turn here would sweep the rover footprint into an obstacle or the table edge"
        return ""

    def _choose(self, pose, wps):
        A = self.p.calib.aspect
        pos = (pose["x"], pose["y"])
        wp, _seg = self._aim(pos, wps)
        du, dv = (wp[0] - pos[0]) * A, wp[1] - pos[1]
        want = math.atan2(dv, du)
        dist = math.hypot(du, dv)
        # translation command: the one whose learned forward component is largest; handle the marker/rover offset
        # by aiming the learned motion direction (not the marker's up edge) at the waypoint
        cands = []
        for cmd in ("F", "B"):
            vf, vl = self.model.v[cmd]
            sp = math.hypot(vf, vl)
            if sp < 1e-3:
                continue
            off = math.atan2(vl, vf)
            e = abs(geom.wrap(want - (pose["theta"] + off)))
            cands.append((e + (0.0 if cmd == "F" else 0.3), cmd, off, sp))   # prefer F unless B saves a big turn
        if cands:
            _, best, ang_off, speed = min(cands)
        else:
            best, ang_off, speed = "F", 0.0, 0.02
        err = geom.wrap(want - (pose["theta"] + ang_off))
        if abs(err) > ANG_TOL:
            if self.turn_dir is not None:
                # skid-steer rover: rotation response is non-linear (dead time, ramp), so use fixed pulses and let the
                # fresh pose/heading decide the next one instead of trusting a rate model
                cmd = "Q" if self.turn_dir["Q"] * err > 0 else "E"
                dur = 0.5 if abs(err) > 0.7 else (0.35 if abs(err) > 0.4 else 0.22)
                self._turn_target = 0.8 * abs(err)           # leave margin for coasting; the next frame finishes the job
                return cmd, dur
            # rotate: choose the command whose learned yaw rate has the sign of err
            cand = [(cmd, r) for cmd, r in self.model.w.items() if r * err > 0]
            if not cand:
                cand = [max(self.model.w.items(), key=lambda kv: abs(kv[1]))]
            cmd, rate = max(cand, key=lambda kv: abs(kv[1]))
            dur = min(0.30, max(0.05, 0.8 * abs(err) / max(abs(rate), 1e-3)))
            if self.model.n[cmd] == 0:
                dur = 0.08   # first use: small probe burst
            return cmd, round(dur, 2)
        sp = max(speed, 0.02)
        dur = min(self.cfg["fwd_burst"], max(0.12, 0.8 * min(dist, 0.30) / sp))
        if self.model.n[best] == 0:
            dur = 0.15       # first use: small probe burst
        return best, round(dur, 2)

    def _speed_for(self, cmd, pose):
        """Command speed for F/B bursts: fast in open space far from the target, slow near obstacles/edges/target."""
        if cmd not in ("F", "B") or not hasattr(self.rover, "speed"):
            return None
        A = self.p.calib.aspect
        pos = (pose["x"], pose["y"])
        rects = self.p.obstacle_rects(near=pos)
        d = 9.0
        if rects:
            import numpy as np
            O = np.asarray(rects, np.float64)
            dx = np.maximum(0, np.maximum(pos[0] * A - O[:, 2] * A, O[:, 0] * A - pos[0] * A))
            dy = np.maximum(0, np.maximum(pos[1] - O[:, 3], O[:, 1] - pos[1]))
            d = float(np.hypot(dx, dy).min())
        edge = min(pos[0] * A, (1 - pos[0]) * A, pos[1], 1 - pos[1])
        m = min(d, edge + 0.1, self.dist if self.dist is not None else 9.0)
        return 0.30 if m < 0.30 else (0.40 if m < 0.55 else 0.50)

    def _choose_holo(self, pose, wps):
        """Position-only control for a mecanum rover: learn each command's world-frame velocity from observed
        displacement; probe F then L when unknown; then pick the command best aligned with the aim direction."""
        A = self.p.calib.aspect
        pos = (pose["x"], pose["y"])
        aim, _ = self._aim(pos, wps)
        s = ((aim[0] - pos[0]) * A, aim[1] - pos[1])
        for probe in ("F", "L"):
            if probe not in self.hv:
                return probe, 0.15
        if all(self.probe_disp.get(k, 1.0) < 0.006 for k in ("F", "L")):
            return None, "rover did not respond to the F and L probes (no measurable displacement)"
        n = math.hypot(*s) or 1e-9
        best, score = None, 0.0
        for cmd, v in self.hv.items():
            sp = math.hypot(*v)
            if sp < 1e-3:
                continue
            cosang = (v[0] * s[0] + v[1] * s[1]) / (sp * n)
            if cosang * sp > score:
                best, score = cmd, cosang * sp
        if best is None or score <= 0:
            return None, "rover did not respond to the F and L probes (no measurable displacement)"
        sp = math.hypot(*self.hv[best])
        dur = min(self.cfg["fwd_burst"], max(0.12, 0.8 * min(n, 0.12) / max(score, 0.02)))
        return best, round(dur, 2)

    def _learn_holo(self, cmd, dur, pb, pa):
        A = self.p.calib.aspect
        d = ((pa["x"] - pb["x"]) * A / dur, (pa["y"] - pb["y"]) / dur)
        self.probe_disp.setdefault(cmd, math.hypot(*d) * dur)
        old = self.hv.get(cmd)
        self.hv[cmd] = d if old is None else (0.5 * old[0] + 0.5 * d[0], 0.5 * old[1] + 0.5 * d[1])
        opp = {"F": "B", "B": "F", "L": "R", "R": "L"}[cmd]
        if opp not in self.hv:
            self.hv[opp] = (-self.hv[cmd][0], -self.hv[cmd][1])
        if math.hypot(*d) * dur < 0.004:
            self._log("WARNING: %s %.2fs produced no measurable displacement (%.4f)" % (cmd, dur, math.hypot(*d) * dur))

    def _learn(self, cmd, dur, pb, pa):
        if self.holonomic:
            return self._learn_holo(cmd, dur, pb, pa)
        A = self.p.calib.aspect
        if (self.imu is not None or self.hbias is not None) and cmd in ("Q", "E") and self.turn_dir is not None and pb.get("theta") is not None and pa.get("theta") is not None:
            dth = geom.wrap(pa["theta"] - pb["theta"])
            if abs(dth) > 0.08 and abs(dth) < 1.2:
                self.turn_bad = self.turn_bad + 1 if dth * self.turn_dir[cmd] < 0 else 0
                if self.turn_bad >= 2:                      # the IMU says this command turns the other way: swap
                    self.turn_dir = {k: -v for k, v in self.turn_dir.items()}
                    self.turn_bad = 0
                    self._log("rotation direction corrected (Q/E swapped)")
            return
        if self.hbias is not None and cmd in ("F", "B") and pb.get("theta_raw") is not None:
            d = ((pa["x"] - pb["x"]) * A, pa["y"] - pb["y"])
            if math.hypot(*d) >= 0.02:
                phi = math.atan2(d[1], d[0]) + (math.pi if cmd == "B" else 0.0)
                nb = geom.wrap(phi - pb["theta_raw"])
                self.hbias = geom.wrap(self.hbias + 0.5 * geom.wrap(nb - self.hbias))
        if self.imu is not None and cmd in ("F", "B") and pb.get("theta") is not None:
            d = ((pa["x"] - pb["x"]) * A, pa["y"] - pb["y"])
            if math.hypot(*d) >= 0.02:                      # camera displacement direction is the heading truth: re-anchor
                phi = math.atan2(d[1], d[0]) + (math.pi if cmd == "B" else 0.0)
                y = self._yaw_rad()
                if y is not None:
                    new_off = geom.wrap(phi - self.imu["sign"] * y)
                    if abs(geom.wrap(new_off - self.imu["offset"])) > 0.5:
                        self._log("heading re-anchored by %.0f deg from the camera displacement" % math.degrees(geom.wrap(new_off - self.imu["offset"])))
                    self.imu["offset"] = new_off
        if cmd in ("Q", "E"):
            self.model.observe_turn(cmd, dur, geom.wrap(pa["theta"] - pb["theta"]))
        else:
            du, dv = (pa["x"] - pb["x"]) * A, pa["y"] - pb["y"]
            th = pb["theta"]
            fwd = math.cos(th) * du + math.sin(th) * dv
            left = -math.sin(th) * du + math.cos(th) * dv
            self.model.observe_move(cmd, dur, fwd / self._factor, left / self._factor)
