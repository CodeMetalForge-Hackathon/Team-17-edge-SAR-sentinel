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
UNSAFE_MAX = 4                    # unsafe-action stops (each followed by bounded recovery) tolerated per run
ESCAPE_DUR = 0.20                 # s: the short, guarded back-away burst used by recovery
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
                    "arrive_tol_mm": 50.0, "cruise": True,
                    # periodic obstacle detection DURING a run: a fresh full frame is folded into the map about every
                    # map_period_s; a map older than map_max_age_s stops the run; no route is held (stopped) up to hold_s
                    "map_period_s": 6.0, "map_max_age_s": 10.0, "no_route_hold_s": 30.0}
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
                       "recoveries": 0, "snapshot_ms": 0.0, "map_refreshes": 0, "obstruction_stops": 0, "unsafe_stops": 0,
                       "unsafe_recovered": 0}
        self._unsafe_dist, self._unsafe_budget = None, 0
        self._aligned_legs = set()
        self._noroute_sig, self._noroute_res = None, None     # no identical replans while nothing changed
        self._room_moves = 0
        self._unsafe_sigs = []        # signatures of rejected actions this run (the same one twice = stop and report)
        self.map_log = []             # (wall time, frame seq, route_blocked) of every in-run map refresh
        self._map_seen = None
        self._hold_t0 = None
        self._had_route = False
        self._cached = None           # {"target", "wps"}: the route planned from the target snapshot, reused while on it
        self.cruise_ttl = CRUISE_TTL
        self.soft_failed = False
        self.keep_armed = False       # mission mode: a finished leg does not disarm (STOP / a failure still do)
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
        self.preempt = threading.Event()   # mission-only: a scene change interrupts this leg. NOT a STOP: the rover is halted, the
                                           # executor ends, authorisation (armed) is untouched and the mission decides what is next
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
        # can the rover rotate in place where it stands? (the SAME test the execution guard applies to a turn, plus 8 mm)
        hm0 = self.p.calib.height_mm or 1.0
        obs0 = self.p.obstacle_rects(near=pos)
        stb = geom.cell_of(pos[0], pos[1], c["cols"], c["rows"]) in geom.planning_blocked(
            obs0, c["cols"], c["rows"], math.hypot(L, W) / 2 + 8.0 / hm0, self.p.calib.aspect)
        # allow for +-8 degrees of heading error while driving a leg (the same allowance the hard check uses)
        L, W = L + 0.14 * W, W + 0.14 * L
        if heading is None and self.pose and self.pose.get("theta") is not None:
            heading = self.pose["theta"]
        return geom.plan_oriented(pos, target or self.target, self.p.obstacle_rects(near=pos), c["cols"], c["rows"], L, W,
                                  m, self.p.calib.aspect, heading=heading, start_arc_max=ARC_MAX, start_turn_blocked=stb)

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

    def _route_blocked(self, wps):
        """Is any part of the remaining route (from the rover's position) now blocked for the rover footprint (rotated along
        each segment, +-8 deg allowance, plus the clearance margin) on the CURRENT map?"""
        c, A = self.cfg, self.p.calib.aspect
        L, W, m = self.footprint()
        L, W = L + 0.14 * W, W + 0.14 * L
        pos = wps[0]
        obs = self.p.obstacle_rects(near=pos)
        for a, b in zip(wps, wps[1:]):
            du, dv = (b[0] - a[0]) * A, b[1] - a[1]
            ln = math.hypot(du, dv)
            if ln < 1e-6:
                continue
            th = math.atan2(dv, du)
            ex = (L * abs(math.cos(th)) + W * abs(math.sin(th))) / 2 + m
            ey = (L * abs(math.sin(th)) + W * abs(math.cos(th))) / 2 + m
            blk = geom.rect_blocked(obs, c["cols"], c["rows"], ex, ey, A)
            k = max(2, int(ln / 0.01))
            for i in range(k + 1):
                f = i / k
                u, v = a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
                if math.hypot((u - pos[0]) * A, v - pos[1]) < 0.06:
                    continue                                   # the rover's own cell
                if geom.cell_of(u, v, c["cols"], c["rows"]) in blk:
                    return True
        return False

    def _map_tick(self, pose):
        """Cheap per-pose check of the periodic obstacle refresh. Starts an off-thread refresh when a newer full frame is
        available, reacts to a refreshed map (route blocked -> STOP and replan), and stops the run if the map is too old.
        Returns '' to carry on, 'blocked' if the rover was stopped for a new obstruction, else a failure reason."""
        per, c = self.p, self.cfg
        now = self.now()
        if not self.p.map_frozen:
            return ""
        if now - per.map_time >= 0.5 * c["map_period_s"]:
            full = getattr(self.src, "latest_full", None)
            fr = full() if full else self.src.latest()
            if fr is not None and fr.seq > per.map_seq:
                per.refresh_async(fr)
        res = ""
        if per.map_seq != self._map_seen:                        # a newer map was swapped in by the mapper thread
            first = self._map_seen is None
            self._map_seen = per.map_seq
            if not first:
                wps = [(pose["x"], pose["y"])] + [tuple(w) for w in self.path[1:]] if self.path else None
                blocked = bool(wps and len(wps) >= 2 and self._route_blocked(wps))
                self.map_log.append((time.time(), per.map_seq, blocked))
                self.timing["map_refreshes"] += 1
                self._log("map refreshed from full frame seq %s%s" % (per.map_seq, ": ROUTE NOW BLOCKED" if blocked else ""))
                if blocked:
                    self._halt()                                   # stop before entering it, replan from a fresh pose
                    self._cached, self.path = None, []              # the old route is void: replan, do not re-test it
                    self.timing["obstruction_stops"] += 1
                    res = "blocked"
        age = now - per.map_time
        if age > c["map_period_s"] and hasattr(self.src, "request_full"):
            self.src.request_full()
        if age > c["map_max_age_s"]:
            self._halt()
            return "obstacle map is %.0f s old (refresh failed: %s): stopped" % (age, per.map_err or "no fresh full frame")
        return res

    def _diagnose_route(self, pos, target):
        """Why is there no route? Concrete geometry for the report (boxed-in start, blocked target, nearest obstacle)."""
        try:
            c, A, hm = self.cfg, self.p.calib.aspect, self.p.calib.height_mm or 0
            L, W, m = self.footprint()
            L, W = L + 0.14 * W, W + 0.14 * L
            obs = self.p.obstacle_rects(near=pos)
            bx, by, bt = geom.clearance_sets(obs, c["cols"], c["rows"], L, W, m, A)
            sc = geom.cell_of(pos[0], pos[1], c["cols"], c["rows"])
            tc = geom.cell_of(target[0], target[1], c["cols"], c["rows"])
            free = 0
            for nd, (dc, dr) in enumerate(geom.DIRS):
                n = (sc[0] + dc, sc[1] + dr)
                if 0 <= n[0] < c["cols"] and 0 <= n[1] < c["rows"] and n not in (bx if nd < 2 else by):
                    free += 1
            txt = "rover cell %d/4 neighbours free%s; target cell %s; " % (
                free, " (start cell is inside the turn-clearance halo)" if sc in bt else "",
                "BLOCKED for the footprint" if (tc in bx and tc in by) else "free")
            return txt + self._clearance_detail({"x": pos[0], "y": pos[1]}, obs)
        except Exception as e:
            return "diagnosis failed: %s" % e

    def _dump_evidence(self, why):
        """Keep the failure evidence on disk (pose, target, route, obstacle cells near both): survives restarts."""
        try:
            import glob
            pose, tgt = self.pose, self.target
            near = []
            if pose and tgt:
                for r in self.p.obstacle_rects(near=(pose["x"], pose["y"])):
                    cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
                    if min(math.hypot((cx - q[0]) * self.p.calib.aspect, cy - q[1]) for q in ((pose["x"], pose["y"]), tgt)) < 0.3:
                        near.append([round(v, 4) for v in r])
            d = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "why": why, "pose": pose, "target": tgt, "path": [list(p) for p in self.path],
                 "step": self.step, "log": self.log[-30:], "map_seq": self.p.map_seq, "map_age_s": round(time.time() - self.p.map_time, 1) if self.p.map_time else None,
                 "obstacle_rects_near": near[:400], "map": self.p.depth_map.to_dict(),
                 "calib": {"aspect": self.p.calib.aspect, "height_mm": self.p.calib.height_mm}}
            fn = os.path.join(self.dir, "fail_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
            json.dump(d, open(fn, "w"), default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
            for old in sorted(glob.glob(os.path.join(self.dir, "fail_*.json")))[:-10]:
                os.remove(old)
        except Exception as e:
            self._log("evidence dump failed: %s" % e)

    def _make_room(self, pose, why_tag):
        """The rover is stuck INSIDE the clearance halo of something (no route from here, a turn rejected here). Find the
        shortest short move along its own heading (reverse preferred when equal) that is guarded-safe and ends where a route
        / a turn exists, and do it. Bounded per run. Returns the new Obs or None."""
        if self._room_moves >= 3 or self.cancelled():
            return None
        A, hm = self.p.calib.aspect, self.p.calib.height_mm or 1.0
        pos, th = (pose["x"], pose["y"]), pose["theta"]
        cfg = self.cfg
        L, W, m = self.footprint()
        R = math.hypot(L, W) / 2
        obs = self.p.obstacle_rects(near=pos)
        bt = geom.planning_blocked(obs, cfg["cols"], cfg["rows"], R + m, A)
        best = None
        for dur in (0.2, 0.35, 0.5):
            for cmd, sgn in (("B", -1.0), ("F", 1.0)):
                act = {"kind": "burst", "cmd": cmd, "dur": dur, "speed": BURST_CMD, "trim": 0.0,
                       "text": "MAKE ROOM %s %.2fs (%s)" % (cmd, dur, why_tag)}
                if self._unsafe(pose, act):
                    continue
                d = self._fwd_speed(BURST_CMD) * dur
                end = ((pos[0] * A + sgn * math.cos(th) * d) / A, pos[1] + sgn * math.sin(th) * d)
                if not (0 < end[0] < 1 and 0 < end[1] < 1):
                    continue
                if geom.cell_of(end[0], end[1], cfg["cols"], cfg["rows"]) in bt:
                    continue                                   # must end where the rover can also turn
                if self.target is not None and self.make_plan(end, heading=th)["error"]:
                    continue
                if best is None:
                    best = act
            if best:
                break
        if best is None:
            return None
        self._room_moves += 1
        self.step += 1
        self._log("step %d: %s" % (self.step, best["text"]))
        self._move(best["cmd"], best["dur"], best["speed"], 0.0)
        if self.cancelled():
            return None
        return self.observe(self._seq_after_stop() + 2, quick=True)

    def preview(self):
        if self.target is None or not self.pose:
            return None
        return self.make_plan((self.pose["x"], self.pose["y"]))

    def first_move_check(self, pose, res):
        """Display/diagnosis only: would the execution guard accept the FIRST action of this route from this pose? Uses the
        same _plan_action/_unsafe as the run, so the drawn trajectory and the guard cannot silently disagree. Returns '' if
        yes (or not evaluable), else the guard's reason."""
        try:
            if not pose or pose.get("theta") is None or not res or res.get("error") or len(res.get("waypoints", [])) < 2:
                return ""
            saved = (self._leg_dir, self._turn_target)
            self._leg_dir = dict(self._leg_dir)
            try:
                act = self._plan_action(pose, res)
                return "" if act["kind"] == "fail" else self._unsafe(pose, act)
            finally:
                self._leg_dir, self._turn_target = saved
        except Exception:
            return ""

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

    def cancelled(self):
        """STOP or disarm: nothing may move the rover (or resume a run/mission) after this."""
        return self.stop_flag.is_set() or not self.armed

    def _move(self, cmd, dur, speed=None, trim=0.0):
        if self.cancelled():
            self._halt()
            return None
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
            self.soft_failed = False
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

    SOFT = ("replan:", "no progress", "cannot align", "could not settle")
    SOFT_BLOCKED = ("no safe alternative", "recovery exhausted", "fresh map:")

    def _soft_reason(self, why):
        """A ROUTE-class failure (no route / no safe alternative / not converging). Terminal safety failures (tracking, comms,
        boundary, stale map, internal errors, STOP) are never soft."""
        if why.startswith(self.SOFT):
            return True
        return why.startswith("blocked:") and any(k in why for k in self.SOFT_BLOCKED)

    def _fail(self, why, soft=None):
        """Stop the rover and report. Normally also disarms. During a MISSION a soft (route-class) failure leaves the rover
        stopped but keeps the operator's single authorisation, so the mission can pick another inspection position; it never
        re-arms anything, and STOP/disarm/terminal failures still cancel everything."""
        if soft is None:
            soft = self._soft_reason(why)
        try:
            self._halt()
        finally:
            self.rover.stop()
        if self.preempt.is_set() and self.armed and self.keep_armed:
            # the mission interrupted this leg (scene change): whatever the executor stumbled over while being stopped is not a
            # terminal failure; the mission re-observes the (stopped) rover itself and fails terminally if it cannot track it
            self.state, self.reason, self.soft_failed = "idle", "interrupted: " + why, False
            self._log("leg interrupted by the mission (%s)" % why)
            return
        if soft and self.keep_armed and not self.stop_flag.is_set():
            self.state, self.reason, self.soft_failed = "failed", why, True
            self._log("FAILED (route-class; still armed for the mission): " + why)
            self._dump_evidence(why)
            self.save_model()
            self.p.save()
            return
        self.soft_failed = False
        if why.startswith(("replan:", "blocked:")):
            self._dump_evidence(why)
        self.armed = False
        self.state, self.reason = "failed", why
        self._log("FAILED: " + why)
        self.save_model()
        self.p.save()

    def _finish(self, reason):
        self._halt()
        self.rover.stop()
        if not self.keep_armed:
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
            if sh and time.time() - sh["t"] < 12 * 3600:
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
        self._unsafe_sigs = []
        self._noroute_sig, self._noroute_res, self._room_moves = None, None, 0
        self._unsafe_dist, self._unsafe_budget = None, 0
        self._aligned_legs = set()
        self.timing["unsafe_stops"] = self.timing["unsafe_recovered"] = 0
        self._log("run started; capturing the target snapshot (one full RGB+depth frame)")
        self.p.map_frozen = False
        snap = self._snapshot()
        if snap == "":
            return self._fail("no fresh full frame for the target snapshot")
        self.p.map_frozen = True                # in-run maps come only from refresh_map (full frames, off-thread)
        self._map_seen, self._hold_t0, self._had_route = self.p.map_seq, None, False
        self.map_log = []
        if hasattr(self.src, "hold_full"):
            self.src.hold_full = True               # no idle throttling of tracking frames
        if hasattr(self.src, "full_period"):
            self.src.full_period = 0.8 * self.cfg["map_period_s"]     # a full frame at least every ~8 s (<= 10 s incl. transfer)
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
                why = self._map_tick(pose)
                if why == "blocked":                       # rover halted: fresh pose, then replan on the new map
                    obs = self.observe(self._seq_after_stop() + 1, quick=True)
                    continue
                if why:
                    return self._fail(why)
                sig = (self.p.map_seq, round(pos[0], 3), round(pos[1], 3), round(pose["theta"], 1))
                if self._hold_t0 is not None and sig == self._noroute_sig:
                    res = self._noroute_res                          # nothing changed: do not replan the same thing again
                else:
                    res = self._route(pos)
                    if res["error"]:
                        self._noroute_sig, self._noroute_res = sig, res
                if res["error"]:
                    if self._hold_t0 is None:
                        self._log("NO ROUTE from here: %s" % self._diagnose_route(pos, self.target))
                        nobs = self._make_room(pose, "no route")      # stuck inside a clearance halo? try one guarded relocation
                        if nobs is not None and nobs.pose is not None:
                            obs = nobs
                            self._cached, self._leg_dir = None, {}
                            continue
                    if self._had_route and self.p.map_frozen:       # new obstruction left no route: stay stopped, report, wait
                        if self._hold_t0 is None:
                            self._hold_t0 = self.now()
                            self._log("NO ROUTE after the map refresh: rover stopped, holding (%s)" % res["error"])
                            self.reason = "no route: holding stopped"
                        self._halt()
                        if self.now() - self._hold_t0 > c["no_route_hold_s"]:
                            return self._fail("replan: %s (held stopped %.0f s) [%s]" % (
                                res["error"], c["no_route_hold_s"], self._diagnose_route(pos, self.target)))
                        time.sleep(0.3)
                        obs = self.observe(self._seq_after_stop() + 1, quick=True)
                        continue
                    return self._fail("replan: %s [%s]" % (res["error"], self._diagnose_route(pos, self.target)))
                if self._hold_t0 is not None:
                    self._log("route found again: resuming")
                    self._hold_t0, self.reason = None, ""
                self._had_route = True
                self.path = res["waypoints"]
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
                    obs, bad = self._recover_unsafe(pose, act, why)
                    if bad:
                        return self._fail(bad)
                    last_cmd, pending = None, None
                    continue
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
            if not self.keep_armed:                  # a mission owns the frame schedule for its whole duration
                if hasattr(self.src, "hold_full"):
                    self.src.hold_full = False
                if hasattr(self.src, "full_period"):
                    self.src.full_period = getattr(self.src, "FULL_PERIOD_S", 8.0)
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
        if self.cancelled():
            self._halt()
            return
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
        akey = (round(res["waypoints"][seg + 1][0], 2), round(res["waypoints"][seg + 1][1], 2))
        if akey in self._aligned_legs:
            # recovery had to line the rover up with this leg (a diagonal approach swept a bigger box than the aligned
            # footprint the planner assumed): keep driving ALONG the leg, do not turn back toward the diagonal aim point
            a, b = res["waypoints"][seg], res["waypoints"][seg + 1]
            want = math.atan2(b[1] - a[1], (b[0] - a[0]) * A)
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

    def _clearance_detail(self, pose, obs):
        """Specific geometry for a clearance rejection: the nearest obstacle cell and the four table edges, in mm, against
        the distance the footprint needs (enclosing radius of the measured/configured footprint)."""
        A, hm = self.p.calib.aspect, self.p.calib.height_mm or 0
        L, W, m = self.footprint()
        r = math.hypot(L, W) / 2
        u, v = pose["x"] * A, pose["y"]
        edges = {"left": u, "right": A - u, "bottom": v, "top": 1 - v}
        best = None
        for o in obs:
            d = math.hypot(max(0.0, o[0] * A - u, u - o[2] * A), max(0.0, o[1] - v, v - o[3]))
            if best is None or d < best[0]:
                best = (d, o)
        e = min(edges, key=edges.get)
        txt = "rover at (%.3f, %.3f), footprint %.0fx%.0f mm (turning radius %.0f mm); " % (pose["x"], pose["y"], L * hm, W * hm, r * hm)
        if best:
            o = best[1]
            txt += "nearest obstacle cell x %.3f-%.3f y %.3f-%.3f is %.0f mm from the centre; " % (o[0], o[2], o[1], o[3], best[0] * hm)
        return txt + "nearest edge %s %.0f mm" % (e, edges[e] * hm)

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
                    return "next move (%s) would bring the rover footprint into an obstacle or the table edge: %s" % (
                        act.get("text", act["cmd"]), self._clearance_detail(pose, obs))
        elif act["cmd"] in ("Q", "E"):
            bt = geom.planning_blocked(obs, c["cols"], c["rows"], math.hypot(L, W) / 2, A)
            if here in bt:
                return "a turn here (%s) would sweep the rover footprint into an obstacle or the table edge: %s" % (
                    act.get("text", act["cmd"]), self._clearance_detail(pose, obs))
        return ""

    # ------------------------------------------------------------------ unsafe-action recovery
    def _escape_actions(self, pose, res, rejected=None):
        """Short candidate moves for a stationary rover whose planned action was rejected: back away (reverse or forward,
        whichever leaves more room) and a turn toward the route. Every candidate must still pass _unsafe() (the same
        guarded path as any other action); translations must also END with more footprint-to-obstacle gap than now."""
        A = self.p.calib.aspect
        pos = (pose["x"], pose["y"])
        th = pose["theta"]
        gap0 = self._near_obstacle(pos)
        out = []
        for cmd, sgn in (("B", -1.0), ("F", 1.0)):
            speed = BURST_CMD
            dist = self._fwd_speed(speed) * ESCAPE_DUR
            end = ((pos[0] * A + sgn * math.cos(th) * dist) / A, pos[1] + sgn * math.sin(th) * dist)
            gain = self._near_obstacle(end) - gap0
            if gain > 0.005:
                out.append((gain, {"kind": "burst", "cmd": cmd, "dur": ESCAPE_DUR, "speed": speed, "trim": 0.0,
                                   "text": "RECOVERY %s %.2fs @%.2f (gap +%.0f mm)" % (
                                       cmd, ESCAPE_DUR, speed, gain * (self.p.calib.height_mm or 0))}))
        out.sort(key=lambda t: -t[0])
        acts = [a for _, a in out]
        shorter = []
        if rejected is not None and rejected.get("cmd") in ("F", "B"):
            # the SAME move, shorter: a planned move that misses the guard by a hair is usually fine over a shorter distance
            # (the guard checks the swept footprint at half and full distance), so progress continues toward the same route
            d0 = rejected.get("dur") if rejected["kind"] == "burst" else 0.3
            for k in (0.5, 0.25):
                dur = round(max(0.1, d0 * k), 2)
                if dur < d0:
                    shorter.append({"kind": "burst", "cmd": rejected["cmd"], "dur": dur, "speed": rejected.get("speed") or BURST_CMD,
                                    "trim": rejected.get("trim", 0.0), "text": "RECOVERY %s %.2fs (shorter than the rejected move)" % (rejected["cmd"], dur)})
        acts = shorter + acts
        if rejected is not None and rejected.get("cmd") in ("F", "B") and self.turn_dir is not None:
            # a MISALIGNED rover has a bigger swept box than the aligned one the planner assumed: if turning to line up with
            # the leg is itself safe, do that first (the turn is guarded by the circle check like every other turn)
            _aim, seg, _rem = self._aim(pos, res["waypoints"])
            a, b = res["waypoints"][seg], res["waypoints"][seg + 1]
            want = math.atan2(b[1] - a[1], (b[0] - a[0]) * A)      # the LEG's direction (the planner's axis-aligned assumption)
            err = min(geom.wrap(want - th), geom.wrap(want - th - math.pi), key=abs)
            if abs(err) > math.radians(6):
                tt = 0.8 * min(abs(err), math.radians(75))
                cmd = "Q" if self.turn_dir["Q"] * err > 0 else "E"
                acts = [{"kind": "burst", "cmd": cmd, "dur": 0.3, "turn_target": tt,
                         "align_key": (round(b[0], 2), round(b[1], 2)),
                         "text": "RECOVERY %s turn %.0f deg (line up with the leg first)" % (cmd, math.degrees(tt))}] + acts
        if rejected is not None and rejected.get("cmd") in ("Q", "E"):
            # a rejected TURN: relocate along the heading to the nearest cell where the turn is clear (and a route exists)
            L, W, m = self.footprint()
            bt = geom.planning_blocked(self.p.obstacle_rects(near=pos), self.cfg["cols"], self.cfg["rows"], math.hypot(L, W) / 2 + m, A)
            room = []
            for dur in (0.2, 0.35, 0.5):
                for cmd, sgn in (("B", -1.0), ("F", 1.0)):
                    d = self._fwd_speed(BURST_CMD) * dur
                    end = ((pos[0] * A + sgn * math.cos(th) * d) / A, pos[1] + sgn * math.sin(th) * d)
                    if 0 < end[0] < 1 and 0 < end[1] < 1 and geom.cell_of(end[0], end[1], self.cfg["cols"], self.cfg["rows"]) not in bt:
                        room.append((dur, {"kind": "burst", "cmd": cmd, "dur": dur, "speed": BURST_CMD, "trim": 0.0,
                                           "text": "RECOVERY %s %.2fs (to a cell with turning room)" % (cmd, dur)}))
            room.sort(key=lambda t: t[0])
            acts = [a for _, a in room] + acts
        if self.turn_dir is not None:
            aim, _seg, _rem = self._aim(pos, res["waypoints"])
            want = math.atan2(aim[1] - pos[1], (aim[0] - pos[0]) * A)
            err = min(geom.wrap(want - th), geom.wrap(want - th - math.pi), key=abs)
            if abs(err) > math.radians(4):
                tt = 0.8 * min(abs(err), math.radians(30))
                cmd = "Q" if self.turn_dir["Q"] * err > 0 else "E"
                acts.append({"kind": "burst", "cmd": cmd, "dur": 0.3, "turn_target": tt,
                             "text": "RECOVERY %s turn %.0f deg" % (cmd, math.degrees(tt))})
        return acts

    def _recover_unsafe(self, pose, act, why):
        """An action failed the hard footprint check. STOP at once, then bounded recovery from FRESH observations: a new
        full frame refreshes the occupancy map (stale cells clear, real ones stay) and the pose; replan toward the SAME
        target; if that action is still unsafe try one guarded back-away/turn; else report the actual obstruction.
        Returns (obs, None) to continue the run, or (None, reason) to stop. Never retries the same rejected action: the
        same (position, heading, action) twice, or UNSAFE_MAX stops in a run, ends the run stopped with the reason."""
        self._halt()
        self.timing["unsafe_stops"] += 1
        if self.dist is not None and (self._unsafe_dist is None or self.dist < self._unsafe_dist - 0.04):
            self._unsafe_dist, self._unsafe_budget, self._unsafe_sigs = self.dist, 0, []   # real progress (>~30 mm): fresh budget
        self._unsafe_budget += 1
        sig = (round(pose["x"], 2), round(pose["y"], 2), int(round(math.degrees(pose["theta"]) / 15.0)), act["kind"], act.get("cmd"))
        self._unsafe_sigs.append(sig)
        detail = "%s rejected: %s" % (act.get("text", act.get("cmd")), why)
        self._log("UNSAFE action stopped before moving; recovering (%d of %d): %s" % (self._unsafe_budget, UNSAFE_MAX, detail))
        if self._unsafe_budget > UNSAFE_MAX or self._unsafe_sigs.count(sig) >= 2:
            return None, "blocked: " + detail + " [recovery exhausted: the same unsafe action recurred or %d stops without progress]" % UNSAFE_MAX
        if self.stop_flag.is_set() or not self.armed:
            return None, "stopped"
        fresh = getattr(self.src, "fresh_full", None)
        fr = fresh(8.0) if fresh else self.src.wait_fresh(self._seq_after_stop() + 1, 5.0)
        if self.cancelled():                           # STOP/disarm arrived while waiting for the frame
            self._halt()
            return None, "stopped"
        if fr is None:
            return None, "blocked: %s [no fresh full frame to recover with]" % detail
        rx = getattr(fr, "rx", None)
        if getattr(fr, "partial", False) or (rx is not None and time.time() - rx > 3.0):
            return None, "blocked: %s [recovery needs a fresh FULL frame, got %s]" % (
                detail, "a partial frame" if getattr(fr, "partial", False) else "one %.1f s old" % (time.time() - rx))
        try:
            self.p.refresh_map(fr)                     # current-scene occupancy (an older frame never overwrites a newer map)
        except Exception as e:
            return None, "blocked: %s [map refresh failed, not moving without a valid map: %s]" % (detail, e)
        if self.p.map_seq < fr.seq or self.now() - self.p.map_time > self.cfg["map_max_age_s"]:
            return None, "blocked: %s [obstacle map is not current after the refresh (%s)]" % (detail, self.p.map_err or "no newer map")
        obs = self.p.process(fr, update_map=False)
        self._apply_heading(obs)
        p2 = obs.pose
        if p2 is None or p2.get("theta") is None:
            return None, "blocked: %s [rover not tracked in the fresh frame]" % detail
        if not p2["inside"]:
            return None, "blocked: %s [rover is outside the sandbox boundary: physical reset needed]" % detail
        out = self.footprint_outside(p2)
        if out:
            return None, "blocked: %s [%s: physical reset needed]" % (detail, out)
        self._cached, self._leg_dir = None, {}
        pos = (p2["x"], p2["y"])
        res = self.make_plan(pos, heading=p2["theta"])
        if res["error"]:
            return None, "blocked: %s [fresh map: %s]" % (detail, res["error"])
        a2 = self._plan_action(p2, res)
        if a2["kind"] != "fail" and not self._unsafe(p2, a2):
            self.timing["unsafe_recovered"] += 1
            self._log("recovered: the fresh map/pose makes the next action safe; continuing to the same target")
            return obs, None
        why2 = a2.get("why") if a2["kind"] == "fail" else self._unsafe(p2, a2)
        for esc in self._escape_actions(p2, res, rejected=act):
            if self._unsafe(p2, esc):
                continue
            if self.step >= self.cfg["max_steps"]:
                break
            if self.cancelled():
                self._halt()
                return None, "stopped"
            self.step += 1
            if esc.get("align_key"):
                self._aligned_legs.add(esc["align_key"])
            self._log("step %d: %s  (replacing the rejected action)" % (self.step, esc["text"]))
            if esc.get("turn_target"):
                self._turn_target = esc["turn_target"]
            try:
                self._move(esc["cmd"], esc.get("dur", 0.2), esc.get("speed"), esc.get("trim", 0.0))
            except Exception as e:
                return None, "rover comm error: %s" % e
            if self.cancelled():
                self._halt()
                return None, "stopped"
            nobs = self.observe(self._seq_after_stop() + 2, quick=esc["cmd"] in ("F", "B"))
            if nobs is None or nobs.pose is None:
                return None, "blocked: tracking lost after the recovery move (%s)" % detail
            self.timing["unsafe_recovered"] += 1
            return nobs, None
        return None, "blocked: %s [no safe alternative: %s]" % (detail, why2 or "no escape move improves clearance")

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
