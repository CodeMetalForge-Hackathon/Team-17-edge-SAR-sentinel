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
POS_TOL = 0.05               # reached the target within 5% of sandbox height (square frame)
ANG_TOL = math.radians(14)   # heading error below which we drive forward
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
        self.cfg = {"cols": 120, "rows": 40, "radius": 0.07, "margin": 0.02, "max_steps": 90, "max_seconds": 240,
                    "fwd_burst": 0.30, "no_progress": 8,
                    # physical rover size / clearance (mm); converted with the measured sandbox height when known
                    "rover_w_mm": None, "rover_l_mm": None, "margin_mm": None}
        self.cfg.update(cfg or {})
        self.model = MotionModel()
        self.now = now
        try:
            self.model.load(json.load(open(os.path.join(store_dir, "model.json"))))
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
        json.dump(self.model.to_dict(), open(os.path.join(self.dir, "model.json"), "w"))

    def _dist_sq(self, a, b):
        A = self.p.calib.aspect
        return math.hypot((a[0] - b[0]) * A, a[1] - b[1])

    def footprint(self):
        """(L, W, margin) in sandbox-height units: L along the rover heading, W across. Taken from the depth-measured
        rover (once its marker pose is known), else the configured mm size, else the dimensionless fallback."""
        c, cal = self.cfg, self.p.calib
        hm = cal.height_mm
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
                                  self.p.calib.aspect)

    def preview(self):
        """Plan from the last known pose for display (no motion)."""
        if self.target is None or not self.pose:
            return None
        return self.make_plan((self.pose["x"], self.pose["y"]))

    # ---- fresh, settled observation
    def observe(self, min_seq, timeout=6.0):
        """Return an Obs with a rover pose from frames captured at/after min_seq and consistent across two frames."""
        prev, tries, t_end = None, 0, self.now() + timeout
        seq = min_seq
        while self.now() < t_end and not self.stop_flag.is_set():
            fr = self.src.wait_fresh(seq, timeout=max(0.5, t_end - self.now()))
            if fr is None:
                continue
            obs = self.p.process(fr)
            seq = fr.seq + 1
            tries += 1
            if obs.pose is None:
                prev = None
                if tries >= 4:
                    return obs
                continue
            if prev is not None:
                dp = math.hypot((obs.pose["x"] - prev["x"]) * self.p.calib.aspect, obs.pose["y"] - prev["y"])
                dt = abs(geom.wrap(obs.pose["theta"] - prev["theta"]))
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
        c = self.cfg
        t0 = self.now()
        self._log("run started; reacquiring pose")
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
                if last_cmd in ("F", "B", "L", "R"):
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
                cmd, dur = self._choose(pose, res["waypoints"])
                # safety (margin 0, the rover's true footprint): a translation burst must stay clear for the footprint
                # at the current heading; a large rotation must not sweep the enclosing circle into an obstacle
                why = self._unsafe(pose, cmd, dur)
                if why:
                    return self._fail(why)
                self.step += 1
                self._log("step %d: %s %.2fs  dist=%.3f  heading=%.0f deg" % (self.step, cmd, dur, self.dist, math.degrees(pose["theta"])))
                if self.stop_flag.is_set() or not self.armed:
                    break
                try:
                    self.rover.move(cmd, dur)
                except Exception as e:  # comm error: stop, disarm
                    return self._fail("rover comm error: %s" % e)
                pending = (cmd, dur, pose)
                last_cmd = cmd
                turns_in_row = turns_in_row + 1 if cmd in ("Q", "E") else 0
                obs = self.observe(self.src.current_seq() + 2)
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
        th = pose["theta"]
        if cmd in ("F", "B"):
            vf, vl = self.model.v[cmd]
            pu = pose["x"] * A + (math.cos(th) * vf - math.sin(th) * vl) * dur
            pv = pose["y"] + (math.sin(th) * vf + math.cos(th) * vl) * dur
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
            # rotate: choose the command whose learned yaw rate has the sign of err
            cand = [(cmd, r) for cmd, r in self.model.w.items() if r * err > 0]
            if not cand:
                cand = [max(self.model.w.items(), key=lambda kv: abs(kv[1]))]
            cmd, rate = max(cand, key=lambda kv: abs(kv[1]))
            dur = min(0.30, max(0.10, 0.8 * abs(err) / max(abs(rate), 1e-3)))
            if self.model.n[cmd] == 0:
                dur = 0.12   # first use: small probe burst
            return cmd, round(dur, 2)
        sp = max(speed, 0.02)
        dur = min(self.cfg["fwd_burst"], max(0.12, 0.8 * min(dist, 0.15) / sp))
        if self.model.n[best] == 0:
            dur = 0.15       # first use: small probe burst
        return best, round(dur, 2)

    def _learn(self, cmd, dur, pb, pa):
        A = self.p.calib.aspect
        if cmd in ("Q", "E"):
            self.model.observe_turn(cmd, dur, geom.wrap(pa["theta"] - pb["theta"]))
        else:
            du, dv = (pa["x"] - pb["x"]) * A, pa["y"] - pb["y"]
            th = pb["theta"]
            fwd = math.cos(th) * du + math.sin(th) * dv
            left = -math.sin(th) * du + math.cos(th) * dv
            self.model.observe_move(cmd, dur, fwd, left)
