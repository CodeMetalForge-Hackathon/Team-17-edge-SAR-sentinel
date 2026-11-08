"""Closed-loop navigation for the Waveshare UGV01 (differential drive, HTTP JSON, IMU-assisted turns).

Loop: fresh camera pose -> replan from the OBSERVED pose -> act -> fresh pose. Actions:
  cruise  long clear straight: wheel commands streamed with a 0.9 s watchdog and a heading trim, refreshed by every new
          camera pose; stale tracking stops the rover (the rover firmware additionally stops after ~3 s of silence)
  burst   short bounded drive (optionally an arc), then STOP, then a fresh pose  (turns, narrow passages, arrival)
  turn    in-place turn closed on the controller IMU (stops at the requested angle), then a fresh pose
Heading comes from the decoded rover marker (ID 5) when readable; otherwise from the marker's bright pattern at the rover
front (a few pixels, NOT decodable at this camera height) anchored to the direction of travel; otherwise from the
controller IMU anchored the same way. The rover's own turn/forward response is learned and saved.
"""
import json
import math
import os
import threading
import time

import geom

ANG_TOL = math.radians(18)        # heading error below which we drive straight
ARC_MAX = math.radians(40)        # up to this error and close to the target we arc instead of turning in place
CRUISE_CMD = 0.40                 # wheel command for cruising (UGV units)
CRUISE_REV = 0.30                 # reverse cruising command (slower)
BURST_CMD = 0.30
REF_CMD = 0.35                    # command at which the forward model is expressed
CRUISE_TTL = 0.9                  # s: the link stops the rover if no new pose refreshes the cruise within this time
SETTLE_POS, SETTLE_ANG = 0.012, math.radians(4)
DEV_TOL = 0.04                    # sandbox heights: route deviation that triggers a replan (otherwise the route is reused)


class MotionModel:
    """Forward response of the rover: speed (square-frame units/s) at REF_CMD for F and B."""

    def __init__(self):
        self.v = {"F": [0.12, 0.0], "B": [-0.12, 0.0]}
        self.n = {"F": 0, "B": 0}

    def to_dict(self):
        return {"v": self.v, "n": self.n}

    def load(self, d):
        try:
            self.v = {k: d["v"][k] for k in ("F", "B")}
            self.n = {k: d["n"][k] for k in ("F", "B")}
        except Exception:
            pass

    def observe_move(self, cmd, dur, d_fwd, d_left):
        if dur < 0.05:
            return
        k = 1.0 if self.n[cmd] == 0 else 0.5
        self.v[cmd] = [(1 - k) * self.v[cmd][0] + k * d_fwd / dur, (1 - k) * self.v[cmd][1] + k * d_left / dur]
        self.n[cmd] += 1
        opp = {"F": "B", "B": "F"}[cmd]
        if self.n[opp] == 0:
            self.v[opp] = [-self.v[cmd][0], -self.v[cmd][1]]


class Navigator:
    def __init__(self, perception, source, rover, store_dir, cfg=None, now=time.time):
        self.p, self.src, self.rover = perception, source, rover
        self.dir = store_dir
        self.cfg = {"cols": 120, "rows": 40, "radius": 0.07, "margin": 0.02, "max_steps": 200, "max_seconds": 600,
                    "no_progress": 10,
                    # physical rover size / clearance (mm); converted with the measured sandbox height when known
                    "rover_w_mm": None, "rover_l_mm": None, "margin_mm": None,
                    # arrival: centre within this many mm of the selected point (~1/3 of the rover half-width, ~6x the
                    # measured tracking noise, ~1.5x the shortest reliable step)
                    "arrive_tol_mm": 50.0, "cruise": True}
        self.cfg.update(cfg or {})
        self.model = MotionModel()
        self.now = now
        self.hbias = None             # rad: heading = pattern_heading + hbias (marker/rover offset, learned)
        self.turn_dir = None          # {'Q': +-1, 'E': +-1}: heading change (CCW +) per in-place turn command
        self.turn_bad = 0
        self.marker_off = None        # rad: rover forward direction minus decoded-marker heading (learned from motion)
        self.imu = None               # {'sign': +-1, 'offset': rad}: world heading = sign*IMU_yaw + offset
        self.saved_heading = None
        try:
            md = json.load(open(os.path.join(store_dir, "model.json")))
            self.model.load(md)
            self.saved_heading = md.get("heading")
        except Exception:
            pass
        self._calib_t = 0.0
        self._pattern_mode = False
        self._turn_target = None
        self._factor = 1.0
        self._leg_dir = {}
        self.latency_s = 1.0          # camera-to-decision latency estimate (updated from measured fetch times)
        self.timing = {"frames": 0, "fetch_ms": 0.0, "cruise_s": 0.0, "bursts": 0, "turns": 0, "plans": 0, "plans_reused": 0,
                       "recoveries": 0, "snapshot_ms": 0.0}
        self._cached = None           # {"target", "wps"}: the route planned from the target snapshot, reused while on it
        self.cruise_ttl = CRUISE_TTL
        self.target = None            # normalized (x, y), exactly as selected (never snapped)
        self.state = "idle"           # idle | running | done | failed
        self.reason = ""
        self.step = 0
        self.log = []
        self.path = []
        self.pose = None
        self.dist = None
        self.remaining = None
        self.armed = False
        self.cruising = False
        self.run_info = {}
        self.stop_flag = threading.Event()
        self.thread = None
        self.lock = threading.Lock()

    # ------------------------------------------------------------------ geometry / planning
    def _log(self, msg):
        self.log.append("%s %s" % (time.strftime("%H:%M:%S"), msg))
        self.log = self.log[-80:]

    def save_model(self):
        d = self.model.to_dict()
        if self.hbias is not None and self.turn_dir is not None:
            d["heading"] = {"hbias": self.hbias, "turn_dir": self.turn_dir, "t": time.time()}
        json.dump(d, open(os.path.join(self.dir, "model.json"), "w"))

    def _dist_sq(self, a, b):
        A = self.p.calib.aspect
        return math.hypot((a[0] - b[0]) * A, a[1] - b[1])

    def arrive_tol(self):
        """Arrival tolerance in sandbox-height units (explicit, from mm)."""
        hm = self.p.calib.height_mm
        return self.cfg["arrive_tol_mm"] / hm if hm else 0.07

    def footprint(self):
        """(L, W, margin) in sandbox-height units: L along the rover heading, W across."""
        c, hm = self.cfg, self.p.calib.height_mm
        dims = self.p.rover_dims_mm()
        if hm and dims:
            L, W = dims
        elif hm and c["rover_w_mm"] and c["rover_l_mm"]:
            L, W = c["rover_l_mm"], c["rover_w_mm"]
        else:
            r = c["radius"]
            return r * 1.414, r * 1.414, c["margin"]
        if self.marker_off is not None and abs(abs(geom.wrap(self.marker_off)) - math.pi / 2) < math.pi / 4:
            L, W = W, L          # the decoded marker is mounted ~90 degrees off the rover's forward axis
        return L / hm, W / hm, (c["margin_mm"] or 0) / hm

    def clearance(self):
        L, W, m = self.footprint()
        return math.hypot(L, W) / 2, m

    def make_plan(self, pos, target=None, heading=None):
        c = self.cfg
        L, W, m = self.footprint()
        # allow for +-8 degrees of heading error while driving a leg (the same allowance the hard check uses)
        L, W = L + 0.14 * W, W + 0.14 * L
        if heading is None and self.pose and self.pose.get("theta") is not None:
            heading = self.pose["theta"]
        return geom.plan_oriented(pos, target or self.target, self.p.obstacle_rects(near=pos), c["cols"], c["rows"], L, W,
                                  m, self.p.calib.aspect, heading=heading)

    def footprint_outside(self, pose, tol=0.01):
        """None if the WHOLE rover (rotated footprint) is inside the usable boundary, else a description."""
        A = self.p.calib.aspect
        L, W, _m = self.footprint()
        th = pose.get("theta")
        th = 0.0 if th is None else th
        ex = (L * abs(math.cos(th)) + W * abs(math.sin(th))) / 2
        ey = (L * abs(math.sin(th)) + W * abs(math.cos(th))) / 2
        u, v = pose["x"] * A, pose["y"]
        hm = self.p.calib.height_mm or 0
        worst = max(ex - u, u + ex - A, ey - v, v + ey - 1.0)
        if worst > tol:
            return "rover footprint extends %.0f mm beyond the usable boundary" % (worst * hm)
        return None

    def _route(self, pos):
        """The route from the target snapshot, reused while the rover stays on it and the (frozen) map is unchanged.
        A replan happens only on first use, route deviation, or a changed target/map."""
        cp = self._cached
        if cp is not None and cp["target"] == self.target and self.p.map_frozen and cp["wps"] and len(cp["wps"]) >= 2:
            A = self.p.calib.aspect
            wps = cp["wps"]
            best_d, best_i = 9.0, 0
            for i in range(len(wps) - 1):
                a, b = (wps[i][0] * A, wps[i][1]), (wps[i + 1][0] * A, wps[i + 1][1])
                dx, dy = b[0] - a[0], b[1] - a[1]
                n2 = dx * dx + dy * dy
                t = 0.0 if n2 < 1e-12 else max(0.0, min(1.0, ((pos[0] * A - a[0]) * dx + (pos[1] - a[1]) * dy) / n2))
                d = math.hypot(pos[0] * A - (a[0] + t * dx), pos[1] - (a[1] + t * dy))
                if d < best_d:
                    best_d, best_i = d, i
            if best_d <= DEV_TOL:
                self.timing["plans_reused"] += 1
                return {"waypoints": [tuple(pos)] + [tuple(w) for w in wps[best_i + 1:]], "error": ""}
            self._log("route deviation %.0f mm: replanning" % (best_d * (self.p.calib.height_mm or 0)))
        res = self.make_plan(pos)
        self.timing["plans"] += 1
        self._cached = None if res["error"] else {"target": self.target, "wps": [tuple(w) for w in res["waypoints"]]}
        return res

    def preview(self):
        if self.target is None or not self.pose:
            return None
        return self.make_plan((self.pose["x"], self.pose["y"]))

    def validate_target(self, x, y):
        """Is (x, y) a usable destination for the WHOLE rover? Returns dict(ok, reason, suggestion). Never alters the
        point; a reachable neighbour is only offered as a suggestion."""
        c, A = self.cfg, self.p.calib.aspect
        if self.p.calib.H is None or self.p.calib.plane is None:
            return {"ok": False, "reason": "calibration incomplete (boundary markers / ground plane)", "suggestion": None}
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
            return {"ok": False, "reason": "outside the usable sandbox boundary", "suggestion": None}
        L, W, m = self.footprint()
        pos = (self.pose["x"], self.pose["y"]) if self.pose else None
        obs = self.p.obstacle_rects(near=pos)
        bx, by, bt = geom.clearance_sets(obs, c["cols"], c["rows"], L, W, m, A)
        cell = geom.cell_of(x, y, c["cols"], c["rows"])
        free = lambda cl: cl not in bx or cl not in by          # fits in at least one orientation
        # sub-cell exactness: test the footprint around the exact point, not just its cell
        ex, ey = L / 2 + m, W / 2 + m
        fits = self._fits_at(x, y, obs, (ex, ey), (ey, ex), A)
        if fits:
            return {"ok": True, "reason": "", "suggestion": None}
        sug = None
        best = 1e9
        for dc in range(-40, 41):
            for dr in range(-14, 15):
                cc, rr = cell[0] + dc, cell[1] + dr
                if not (0 <= cc < c["cols"] and 0 <= rr < c["rows"]):
                    continue
                px, py = (cc + 0.5) / c["cols"], (rr + 0.5) / c["rows"]
                d = math.hypot((px - x) * A, py - y)
                if d < best and d < 0.6 and self._fits_at(px, py, obs, (ex, ey), (ey, ex), A):
                    best, sug = d, (px, py)
        why = "the whole rover (%.0f x %.0f mm + clearance) does not fit there" % (
            L * (self.p.calib.height_mm or 0), W * (self.p.calib.height_mm or 0)) if self.p.calib.height_mm else "the whole rover does not fit there"
        return {"ok": False, "reason": why, "suggestion": sug}

    @staticmethod
    def _fits_at(x, y, obstacles, ex_ey_a, ex_ey_b, A):
        """Footprint (axis-aligned half extents, either orientation) centred exactly at (x, y) inside [0,1]^2 and clear
        of every obstacle rect (normalized x0,y0,x1,y1)."""
        for ex, ey in (ex_ey_a, ex_ey_b):
            u, v = x * A, y
            if u - ex < 0 or u + ex > A or v - ey < 0 or v + ey > 1:
                continue
            ok = True
            for o in obstacles:
                if u - ex < o[2] * A and o[0] * A < u + ex and v - ey < o[3] and o[1] < v + ey:
                    ok = False
                    break
            if ok:
                return True
        return False

    # ------------------------------------------------------------------ heading sources
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
        if p is None:
            return
        if p.get("theta") is not None and p.get("source") == "marker":      # decoded ID 5: heading + learned offset
            p["theta_dec"] = p["theta"]
            if self.marker_off is not None:
                p["theta"] = geom.wrap(p["theta"] + self.marker_off)
            return
        if p.get("theta") is not None:
            return
        if self.hbias is not None and p.get("theta_raw") is not None:
            p["theta"] = geom.wrap(p["theta_raw"] + self.hbias)
            p["source"] = "blob + marker-pattern heading"
            return
        if self.imu is None:
            return
        y = self._yaw_rad()
        if y is None:
            obs.pose = None
            return
        p["theta"] = geom.wrap(self.imu["sign"] * y + self.imu["offset"])
        p["source"] = "blob + IMU heading"

    # ------------------------------------------------------------------ rover actions
    def _seq_after_stop(self):
        f = getattr(self.src, "seq_est", None)
        return f() if f else self.src.current_seq()

    def _halt(self):
        if self.cruising:
            self.cruising = False
        h = getattr(self.rover, "halt", None)
        (h or self.rover.stop)()

    def _move(self, cmd, dur, speed=None, trim=0.0):
        self.p.reset_heading()
        self._halt()
        if cmd in ("Q", "E") and self._turn_target and getattr(self.rover, "turn_by", None) is not None:
            tgt, self._turn_target = self._turn_target, None
            got = self.rover.turn_by(cmd, tgt)
            self.timing["turns"] += 1
            self._log("IMU turn %s: target %.0f deg, turned %.0f deg" % (cmd, math.degrees(tgt), math.degrees(got)))
            return
        self.timing["bursts"] += 1
        try:
            return self.rover.move(cmd, dur, speed=speed, trim=trim)
        except TypeError:
            return self.rover.move(cmd, dur)

    def _burst_and_observe(self, cmd, dur, quick=False, speed=None):
        self._move(cmd, dur, speed)
        return self.observe(self._seq_after_stop() + 2, quick=quick)

    def calibrate_pattern_heading(self):
        """Forward probe ties the (few-pixel) marker pattern to the direction of travel; turn pulses find which of Q/E
        turns which way. Returns '' on success else the reason."""
        A = self.p.calib.aspect
        self._log("heading calibration (marker pattern): forward and turn probes")
        o0 = self.observe(self._seq_after_stop() + 1)
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
        self.model.observe_move("F", dur, math.hypot(*d) / (0.4 / REF_CMD), 0.0)
        self._log("forward probe: moved %.3f in %.2fs; marker-pattern offset %.0f deg" % (math.hypot(*d), dur, math.degrees(self.hbias)))
        o1 = self.observe(self._seq_after_stop() + 1, quick=True)
        th_a = geom.wrap(o1.pose["theta_raw"] + self.hbias) if o1 and o1.pose and o1.pose.get("theta_raw") is not None else None
        if th_a is None:
            return "marker pattern lost after the forward probe"
        for k in range(3):
            self._turn_target = 0.5
            o2 = self._burst_and_observe("Q", 0.5)
            if o2 is None or o2.pose is None or o2.pose.get("theta_raw") is None:
                return "marker pattern lost during the turn probe"
            th_b = geom.wrap(o2.pose["theta_raw"] + self.hbias)
            dth = geom.wrap(th_b - th_a)
            if abs(dth) >= 0.2:
                self.turn_dir = {"Q": 1.0 if dth > 0 else -1.0, "E": -1.0 if dth > 0 else 1.0}
                self.turn_bad = 0
                self._log("turn probe: Q rotated %.0f deg -> Q is %s" % (math.degrees(dth), "CCW" if dth > 0 else "CW"))
                return ""
        return "rover did not rotate on three turn probes"

    def calibrate_marker(self):
        """Decoded marker: forward probe -> marker/rover offset; turn probe -> which of Q/E is CCW."""
        A = self.p.calib.aspect
        self._log("heading calibration (decoded marker): forward and turn probes")
        o0 = self.observe(self._seq_after_stop() + 1)
        if o0 is None or o0.pose is None or o0.pose.get("theta_dec") is None:
            return "decoded marker heading not available"
        p0 = o0.pose
        for dur in (0.25, 0.45):
            o1 = self._burst_and_observe("F", dur)
            if o1 is None or o1.pose is None:
                return "pose lost during the forward probe"
            d = ((o1.pose["x"] - p0["x"]) * A, o1.pose["y"] - p0["y"])
            if math.hypot(*d) >= 0.012:
                break
            p0 = o1.pose
        else:
            return "rover did not move on the forward probes (no measurable displacement)"
        self.marker_off = geom.wrap(math.atan2(d[1], d[0]) - p0["theta_dec"])
        self.model.observe_move("F", dur, math.hypot(*d), 0.0)
        o1 = self.observe(self._seq_after_stop() + 1, quick=True)
        th_a = o1.pose["theta_dec"] if o1 and o1.pose else None
        if th_a is None:
            return "pose lost after the forward probe"
        for tgt in (0.4, 0.7):
            self._turn_target = tgt
            o2 = self._burst_and_observe("Q", 0.3)
            if o2 is None or o2.pose is None:
                return "pose lost during the turn probe"
            dth = geom.wrap(o2.pose["theta_dec"] - th_a)
            if abs(dth) >= 0.15:
                self.turn_dir = {"Q": 1.0 if dth > 0 else -1.0, "E": -1.0 if dth > 0 else 1.0}
                self._log("decoded-marker offset %.0f deg; Q is %s" % (math.degrees(self.marker_off), "CCW" if dth > 0 else "CW"))
                return ""
        return "rover did not rotate on the turn probes"

    def calibrate_heading(self):
        """IMU path (no readable pattern): F, turn, F probes tie the IMU yaw to the camera frame. '' on success."""
        A = self.p.calib.aspect
        self._log("heading calibration (IMU): forward / turn / forward probes")
        o0 = self.observe(self._seq_after_stop() + 1)
        if o0 is None or o0.pose is None:
            return "no pose for heading calibration"

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
        o1, d1, dur1, why = forward(o0.pose)
        if why:
            return why
        phi1, y1 = math.atan2(d1[1], d1[0]), self._yaw_rad()
        self.model.observe_move("F", dur1, math.hypot(*d1), 0.0)
        dpsi = 0.0
        for k, tgt in enumerate((0.3, 0.5, 0.8)):
            self._turn_target = tgt
            o2 = self._burst_and_observe("Q", 0.3)
            y2 = self._yaw_rad()
            if o2 is None or o2.pose is None:
                return "rover position lost after the turn probe"
            if y1 is None or y2 is None:
                return "IMU yaw unavailable"
            dpsi = geom.wrap(y2 - y1)
            if abs(dpsi) >= 0.15:
                break
            y1 = y2
        if abs(dpsi) < 0.15:
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
        self.turn_dir = {"Q": 1.0 if sign * dpsi > 0 else -1.0, "E": -1.0 if sign * dpsi > 0 else 1.0}
        self.turn_bad = 0
        self._log("heading calibrated (IMU): sign %+d, offset %.0f deg" % (sign, math.degrees(self.imu["offset"])))
        return ""

    # ------------------------------------------------------------------ observation
    def observe(self, min_seq, timeout=6.0, quick=False):
        """Return an Obs with a rover pose from a frame captured at/after min_seq. quick: first frame with a pose;
        otherwise two consecutive frames must agree (rover settled)."""
        prev, tries, t_end = None, 0, self.now() + timeout
        seq = min_seq
        while self.now() < t_end and not self.stop_flag.is_set():
            fr = self.src.wait_fresh(seq, timeout=max(0.5, t_end - self.now()))
            if fr is None:
                continue
            obs = self.p.process(fr)
            self._apply_heading(obs)
            self.timing["frames"] += 1
            self.timing["fetch_ms"] += getattr(self.src, "fetch_ms", 0)
            self._update_latency(fr)
            seq = fr.seq + 1
            tries += 1
            if obs.pose is None:
                prev = None
                if tries >= 8:
                    return obs
                continue
            if obs.pose.get("theta") is None and obs.pose.get("theta_raw") is None and \
                    (self.hbias is not None or self._pattern_mode) and tries < 6:
                prev = None                      # pattern not readable in this frame: try the next one
                continue
            if quick:
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

    # ------------------------------------------------------------------ run control (exactly one executor)
    def start(self, target):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False, "already running"
            if not self.armed:
                return False, "motion is not armed"
            self.target = tuple(target)
            self.stop_flag.clear()
            self.state, self.reason, self.step = "running", "", 0
            self.run_info = {"t_start": time.time(), "target": self.target}
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True, "started"

    def retarget(self, target):
        """New destination while running: stop the executor (rover halted), replan from the observed pose, start again.
        Never two executors: the old thread must be dead before the new one starts."""
        with self.lock:
            t = self.thread
        if t and t.is_alive():
            self.stop_flag.set()
            self._halt()
            t.join(8.0)
            if t.is_alive():
                self.rover.stop()
                return False, "previous route executor did not stop; not starting another"
        self._log("new target selected: replanning from the observed pose")
        return self.start(target)

    def stop(self):
        """Operator STOP: signal the loop, stop the rover, disarm. Calibration, map and model are kept."""
        self.stop_flag.set()
        self._halt()
        ok = self.rover.stop()
        self.armed = False
        if self.state == "running":
            self.state, self.reason = "idle", "stopped by operator"
        self._log("STOP %s; disarmed" % ("sent" if ok else "FAILED"))
        return ok

    def join(self, t=60):
        if self.thread:
            self.thread.join(t)

    def _fail(self, why):
        try:
            self._halt()
        finally:
            self.rover.stop()
        self.armed = False
        self.state, self.reason = "failed", why
        self._log("FAILED: " + why)
        self.save_model()
        self.p.save()

    def _finish(self, reason):
        self._halt()
        self.rover.stop()
        self.armed = False
        self.state, self.reason = "done", reason
        self.run_info["t_end"] = time.time()
        self._log(reason)
        self.save_model()
        self.p.save()

    def _run(self):
        try:
            self._run_inner()
        except Exception as e:   # never leave the rover in an undefined state: STOP, disarm, report
            import traceback
            self._log("internal error: " + traceback.format_exc().splitlines()[-1])
            self._fail("internal error: %s" % e)

    def _snapshot(self):
        """One fresh full RGB+depth frame -> obstacle map (+ pose). Returns the Obs, None if no pose, '' if no frame."""
        t = time.time()
        fresh = getattr(self.src, "fresh_full", None)
        fr = fresh(10.0) if fresh else self.src.latest()
        if fr is None:
            return ""
        obs = self.p.snapshot(fr)
        self._apply_heading(obs)
        self.timing["snapshot_ms"] = (time.time() - t) * 1000
        dm = self.p.depth_map
        self._log("snapshot seq %s: %d blocked / %d unknown map cells, %.0f ms" % (
            fr.seq, len(dm.blocked_cells()), len(dm.unknown_cells()), self.timing["snapshot_ms"]))
        return obs

    def _calibrate_if_needed(self, obs):
        if obs is None or obs.pose is None:
            return obs, ""
        if obs.pose.get("theta_dec") is not None:
            if self.marker_off is None or self.turn_dir is None:
                why = self.calibrate_marker()
                if why:
                    return obs, "heading calibration failed: " + why
                return self.observe(self._seq_after_stop() + 1, quick=True), ""
            return obs, ""
        if obs.pose.get("theta") is not None:
            return obs, ""
        if obs.pose.get("theta_raw") is not None:
            sh = self.saved_heading or ({"hbias": self.hbias, "turn_dir": self.turn_dir, "t": self._calib_t}
                                        if self.hbias is not None and self.turn_dir else None)
            if sh and time.time() - sh["t"] < 900:
                self.hbias, self.turn_dir, self.imu = sh["hbias"], sh["turn_dir"], None
                self._log("reusing heading calibration (offset %.0f deg); re-anchored by forward motion" % math.degrees(self.hbias))
            else:
                self.hbias, self.turn_dir, self.imu = None, None, None
                self._pattern_mode = True
                why = self.calibrate_pattern_heading()
                if why:
                    return obs, "heading calibration failed: " + why
                self._calib_t = time.time()
        elif getattr(self.rover, "yaw_deg", None) is not None:
            self.imu = None
            why = self.calibrate_heading()
            if why:
                return obs, "heading calibration failed: " + why
        else:
            return obs, "no heading source (marker, pattern or IMU)"
        return self.observe(self._seq_after_stop() + 1, quick=True), ""

    def _run_inner(self):
        c = self.cfg
        t0 = self.now()
        self._leg_dir = {}
        self._cached = None
        self._log("run started; capturing the target snapshot (one full RGB+depth frame)")
        self.p.map_frozen = False
        snap = self._snapshot()
        if snap == "":
            return self._fail("no fresh full frame for the target snapshot")
        self.p.map_frozen = True                # the map from this snapshot is reused for the whole run
        if hasattr(self.src, "hold_full"):
            self.src.hold_full = True
        obs = snap if (snap is not None and snap.pose is not None) else self.observe(self._seq_after_stop() + 1)
        if obs is not None and obs.pose is not None:       # never move (not even a calibration probe) from outside
            if not obs.pose["inside"]:
                return self._fail("rover is outside the sandbox boundary: physical reset needed")
            out = self.footprint_outside(obs.pose)
            if out:
                return self._fail(out + ": physical reset needed")
        obs, why = self._calibrate_if_needed(obs)
        if why:
            return self._fail(why)
        best_rem, bad_dist, last_cmd, turns_in_row = None, 0, None, 0
        approach_best, approach_bad = 1e9, 0
        pending = None
        try:
            while not self.stop_flag.is_set():
                if obs is None or obs.pose is None:
                    obs = self._recover()            # rover is halted here: look for it in full frames while stationary
                    if obs is None or obs.pose is None:
                        return self._fail("rover not found in a fresh frame (tracking lost)")
                pose = obs.pose
                if pose.get("theta") is None:
                    return self._fail("rover heading unavailable in this frame")
                self.pose = pose
                if not pose["inside"]:
                    return self._fail("rover is outside the sandbox boundary: physical reset needed")
                out = self.footprint_outside(pose)
                if out:
                    return self._fail(out + ": physical reset needed" if self.step == 0 else out)
                if pending:
                    cmd, dur, pb = pending
                    self._learn(cmd, dur, pb, pose)
                    pending = None
                pos = (pose["x"], pose["y"])
                self.dist = self._dist_sq(pos, self.target)
                if self.dist <= self.arrive_tol():
                    hm = self.p.calib.height_mm or 0
                    return self._finish("reached target: %.0f mm from it (tolerance %.0f mm)" % (self.dist * hm, c["arrive_tol_mm"]))
                if self.step >= c["max_steps"]:
                    return self._fail("step limit reached")
                if self.now() - t0 > c["max_seconds"]:
                    return self._fail("time limit reached")
                res = self._route(pos)
                self.path = res["waypoints"]
                if res["error"]:
                    return self._fail("replan: " + res["error"])
                rem = geom.path_length_sq(res["waypoints"], self.p.calib.aspect)
                self.remaining = rem
                if last_cmd in ("F", "B", "CRUISE"):
                    bad_dist = bad_dist + 1 if (best_rem is not None and rem > best_rem - 0.004) else 0
                if best_rem is None or rem < best_rem:
                    best_rem = rem
                if bad_dist >= c["no_progress"]:
                    return self._fail("no progress along the route for %d translation bursts" % bad_dist)
                if turns_in_row >= 12:
                    return self._fail("cannot align the heading (12 turns without reaching it)")
                # final approach must converge, not oscillate
                if self.dist < 3 * self.arrive_tol() + 0.2:
                    if self.dist < approach_best - 0.004:
                        approach_best, approach_bad = self.dist, 0
                    else:
                        approach_bad += 1
                    if approach_bad >= 7:
                        hm = self.p.calib.height_mm or 0
                        return self._fail("could not settle within %.0f mm of the target (best %.0f mm)" % (c["arrive_tol_mm"], approach_best * hm))
                act = self._plan_action(pose, res)
                if act["kind"] == "fail":
                    return self._fail(act["why"])
                why = self._unsafe(pose, act)
                if why:
                    return self._fail(why)
                self.step += 1
                self._last_speed = act.get("speed") or REF_CMD
                self._log("step %d: %s  dist=%.3f  heading=%.0f deg" % (self.step, act["text"], self.dist, math.degrees(pose["theta"])))
                if self.stop_flag.is_set() or not self.armed:
                    break
                try:
                    if act["kind"] == "cruise":
                        self._cruise(act)
                        last_cmd, turns_in_row = "CRUISE", 0
                        obs = self.observe_next(pose)
                        continue
                    self._move(act["cmd"], act.get("dur", 0.2), act.get("speed"), act.get("trim", 0.0))
                except Exception as e:      # comm error: stop, disarm
                    return self._fail("rover comm error: %s" % e)
                pending = (act["cmd"], act.get("dur", 0.2), pose)
                last_cmd = act["cmd"]
                turns_in_row = turns_in_row + 1 if act["cmd"] in ("Q", "E") else 0
                obs = self.observe(self._seq_after_stop() + 2, quick=act["cmd"] in ("F", "B"))
        finally:
            try:
                self._halt()
            except Exception:
                pass
            self.p.map_frozen = False
            if hasattr(self.src, "hold_full"):
                self.src.hold_full = False
            if self.state == "running":
                self.state, self.reason = "idle", "stopped"
            self.save_model()
            self.p.save()

    def _recover(self):
        """Stationary diagnosis after stale/lost tracking: STOP, then full frames until the pose is back.
        At most 2 attempts per call and 4 per run."""
        self._halt()
        if self.timing["recoveries"] >= 4 or self.stop_flag.is_set():
            return None
        for k in range(2):
            self.timing["recoveries"] += 1
            self._log("tracking lost: rover stopped, searching the full frame (attempt %d)" % (k + 1))
            fresh = getattr(self.src, "fresh_full", None)
            fr = fresh(8.0) if fresh else None
            if fr is None:
                continue
            obs = self.p.process(fr, update_map=False)
            self._apply_heading(obs)
            if obs.pose is not None and obs.pose.get("theta") is not None:
                self._log("rover reacquired at (%.3f, %.3f)" % (obs.pose["x"], obs.pose["y"]))
                return obs
        return None

    # ------------------------------------------------------------------ cruise
    def _cruise(self, act):
        a = self.rover
        l, r = act["l"], act["r"]
        a.drive(l, r, ttl=self.cruise_ttl)
        self.cruising = True
        self.timing["cruise_s"] += 0.0

    def _update_latency(self, fr):
        """Camera-to-decision latency from the measured fetch time. The cruise watchdog is ~2 observation periods:
        long enough to ride out one slow frame, short enough that the rover never travels blind for more than ~1 s."""
        fm = getattr(self.src, "fetch_ms", 0) / 1000.0
        self.latency_s = min(2.0, max(0.35, 0.30 + fm))
        self.cruise_ttl = min(CRUISE_TTL, max(0.6, 0.35 + 2.0 * fm))

    def observe_next(self, last_pose):
        """While cruising: next frame with a pose (no stop between observations). Stale tracking stops the rover."""
        t_end = self.now() + self.cruise_ttl + 0.3
        seq = getattr(self.src, "latest_seq", lambda: 0)() + 1
        while self.now() < t_end and not self.stop_flag.is_set():
            fr = self.src.wait_fresh(seq, timeout=max(0.1, t_end - self.now()))
            if fr is None:
                break
            obs = self.p.process(fr)
            self._apply_heading(obs)
            seq = fr.seq + 1
            if obs.pose is not None and obs.pose.get("theta") is not None:
                self.timing["frames"] += 1
                self.timing["fetch_ms"] += getattr(self.src, "fetch_ms", 0)
                self._update_latency(fr)
                return obs
        self._halt()
        return None

    # ------------------------------------------------------------------ action selection
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
        seg_rem = n - t
        return (aim[0] / A, aim[1]), seg, seg_rem

    def _ccw(self):
        return self.turn_dir["Q"] if self.turn_dir else 1.0

    def _fwd_speed(self, cmd_speed):
        """Learned forward speed (square-frame units/s) at a wheel command."""
        return math.hypot(*self.model.v["F"]) * cmd_speed / REF_CMD

    def _near_obstacle(self, pos):
        rects = self.p.obstacle_rects(near=pos)
        A = self.p.calib.aspect
        d = 9.0
        if rects:
            import numpy as np
            O = np.asarray(rects, np.float64)
            dx = np.maximum(0, np.maximum(pos[0] * A - O[:, 2] * A, O[:, 0] * A - pos[0] * A))
            dy = np.maximum(0, np.maximum(pos[1] - O[:, 3], O[:, 1] - pos[1]))
            d = float(np.hypot(dx, dy).min())
        edge = min(pos[0] * A, (A - pos[0] * A), pos[1], 1 - pos[1])
        L, W, _m = self.footprint()
        r = math.hypot(L, W) / 2                      # enclosing radius: gap from the rover body, not from its centre
        return min(d, edge) - r

    def _plan_action(self, pose, res):
        A = self.p.calib.aspect
        pos = (pose["x"], pose["y"])
        aim, seg, seg_rem = self._aim(pos, res["waypoints"])
        du, dv = (aim[0] - pos[0]) * A, aim[1] - pos[1]
        want = math.atan2(dv, du)
        n = math.hypot(du, dv)
        th = pose["theta"]
        # forward or reverse per leg: whichever needs less turning; a whole straight reverse leg is fine, a 180 degree turn
        # is not. The choice is kept for the leg (no direction chattering).
        errF = geom.wrap(want - th)
        errB = geom.wrap(want - th - math.pi)
        key = (round(res["waypoints"][seg + 1][0], 2), round(res["waypoints"][seg + 1][1], 2))
        prev = self._leg_dir.get(key)
        fresh = abs(errB) + 0.2 < abs(errF)
        if prev is None:
            use_b = fresh
        else:                       # keep the leg's direction unless the other one now needs 0.5 rad less turning
            use_b = prev if abs((errB if prev else errF)) < abs((errF if prev else errB)) + 0.5 else fresh
        self._leg_dir[key] = use_b
        err = errB if use_b else errF
        ccw = self._ccw()
        if abs(err) > ANG_TOL:
            if abs(err) <= ARC_MAX and self.dist < 0.6:
                trim = max(-0.5, min(0.5, 1.1 * err)) * ccw        # trim > 0 yaws CCW whether driving forward or reversing
                s = BURST_CMD
                dur = min(0.30, max(0.12, 0.8 * min(n, 0.25) / max(self._fwd_speed(s), 0.05)))
                cmd = "B" if use_b else "F"
                return {"kind": "burst", "cmd": cmd, "dur": round(dur, 2), "speed": s, "trim": trim,
                        "text": "%s arc %.2fs trim %+.2f (err %.0f deg)" % (cmd, dur, trim, math.degrees(err))}
            if self.turn_dir is None:
                return {"kind": "fail", "why": "turn direction unknown"}
            cmd = "Q" if self.turn_dir["Q"] * err > 0 else "E"
            self._turn_target = 0.8 * abs(err)
            return {"kind": "burst", "cmd": cmd, "dur": 0.3, "text": "%s turn %.0f deg" % (cmd, math.degrees(self._turn_target))}
        # straight: cruise on long clear stretches, bounded bursts otherwise
        cc = CRUISE_REV if use_b else CRUISE_CMD                 # reverse is driven slower
        v_c = self._fwd_speed(cc)
        lead = v_c * self.latency_s                   # distance the rover covers while the observation is in flight
        clear = self._near_obstacle(pos)              # footprint-to-obstacle/edge gap, sandbox heights
        end_dist = min(seg_rem, self.dist)
        if self.cfg.get("cruise") and getattr(self.rover, "drive", None) is not None                 and end_dist > lead + 0.45 and clear > 0.30 and abs(err) < math.radians(12):
            trim = max(-0.15, min(0.15, 0.35 * err)) * ccw
            if use_b:
                l, r = -cc * (1 + trim), -cc * (1 - trim)
            else:
                l, r = cc * (1 - trim), cc * (1 + trim)
            return {"kind": "cruise", "l": l, "r": r, "cmd": "B" if use_b else "F", "rev": use_b, "cc": cc,
                    "lead": lead, "text": "CRUISE%s %.2f cmd (seg %.2f to go, trim %+.2f)" % (" REVERSE" if use_b else "", cc, seg_rem, trim)}
        s = 0.5 if (end_dist > 0.9 and clear > 0.5 and not use_b) else (0.4 if end_dist > 0.4 and clear > 0.25 else BURST_CMD)
        cmd = "B" if use_b else "F"
        v = max(self._fwd_speed(s), 0.05)
        step = min(end_dist, 0.45 if s >= 0.5 else 0.30)
        dur = min(0.5, max(0.10, 0.85 * step / v))
        trim = max(-0.2, min(0.2, 0.5 * err)) * ccw
        return {"kind": "burst", "cmd": cmd, "dur": round(dur, 2), "speed": s, "trim": trim,
                "text": "%s %.2fs @%.2f (%.2f to segment end)" % (cmd, dur, s, seg_rem)}

    def _unsafe(self, pose, act):
        """Hard safety (footprint with NO margin): a translation must keep the rotated footprint clear of obstacles and
        the table edge over the distance covered before the next observation; a turn must not sweep the enclosing circle
        into anything."""
        c, A = self.cfg, self.p.calib.aspect
        L, W, _m = self.footprint()
        pos = (pose["x"], pose["y"])
        obs = self.p.obstacle_rects(near=pos)
        here = geom.cell_of(pos[0], pos[1], c["cols"], c["rows"])
        th = pose["theta"]
        if act["kind"] == "cruise" or act["cmd"] in ("F", "B"):
            if act["kind"] == "cruise":
                dist = act["lead"] + self._fwd_speed(act["cc"]) * 0.6        # in-flight latency + coasting after STOP
                sgn = -1.0 if act["rev"] else 1.0
            else:
                dist = self._fwd_speed(act["speed"]) * act["dur"]
                sgn = 1.0 if act["cmd"] == "F" else -1.0
            ex = max((L * abs(math.cos(th + d)) + W * abs(math.sin(th + d))) / 2 for d in (-0.14, 0.0, 0.14))
            ey = max((L * abs(math.sin(th + d)) + W * abs(math.cos(th + d))) / 2 for d in (-0.14, 0.0, 0.14))
            blk = geom.rect_blocked(obs, c["cols"], c["rows"], ex, ey, A)
            for f in (0.5, 1.0):
                pu = pos[0] * A + sgn * math.cos(th) * dist * f
                pv = pos[1] + sgn * math.sin(th) * dist * f
                cell = geom.cell_of(pu / A, pv, c["cols"], c["rows"])
                if cell in blk and cell != here:
                    return "next move would bring the rover footprint into an obstacle or the table edge"
        elif act["cmd"] in ("Q", "E"):
            bt = geom.planning_blocked(obs, c["cols"], c["rows"], math.hypot(L, W) / 2, A)
            if here in bt:
                return "a turn here would sweep the rover footprint into an obstacle or the table edge"
        return ""

    # ------------------------------------------------------------------ learning
    def _learn(self, cmd, dur, pb, pa):
        A = self.p.calib.aspect
        if cmd in ("Q", "E"):
            if self.turn_dir is not None and pb.get("theta") is not None and pa.get("theta") is not None:
                dth = geom.wrap(pa["theta"] - pb["theta"])
                if 0.08 < abs(dth) < 1.2:
                    self.turn_bad = self.turn_bad + 1 if dth * self.turn_dir[cmd] < 0 else 0
                    if self.turn_bad >= 2:
                        self.turn_dir = {k: -v for k, v in self.turn_dir.items()}
                        self.turn_bad = 0
                        self._log("turn direction corrected (Q/E swapped)")
            return
        d = ((pa["x"] - pb["x"]) * A, pa["y"] - pb["y"])
        th = pb["theta"]
        fwd = math.cos(th) * d[0] + math.sin(th) * d[1]
        left = -math.sin(th) * d[0] + math.cos(th) * d[1]
        if dur >= 0.12:
            self.model.observe_move(cmd, dur, fwd / (self._last_speed / REF_CMD), left / (self._last_speed / REF_CMD))
        if math.hypot(*d) >= 0.02 and pb.get("theta_raw") is not None and self.hbias is not None:
            phi = math.atan2(d[1], d[0]) + (math.pi if cmd == "B" else 0.0)
            nb = geom.wrap(phi - pb["theta_raw"])
            self.hbias = geom.wrap(self.hbias + 0.5 * geom.wrap(nb - self.hbias))
        if self.marker_off is not None and pb.get("theta_dec") is not None and math.hypot(*d) >= 0.02:
            phi = math.atan2(d[1], d[0]) + (math.pi if cmd == "B" else 0.0)
            nb = geom.wrap(phi - pb["theta_dec"])
            self.marker_off = geom.wrap(self.marker_off + 0.5 * geom.wrap(nb - self.marker_off))
        if self.imu is not None and math.hypot(*d) >= 0.02:
            phi = math.atan2(d[1], d[0]) + (math.pi if cmd == "B" else 0.0)
            y = self._yaw_rad()
            if y is not None:
                self.imu["offset"] = geom.wrap(phi - self.imu["sign"] * y)

    _last_speed = REF_CMD
