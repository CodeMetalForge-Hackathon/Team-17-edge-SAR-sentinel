"""Structure visits with a timed placeholder collection (no sensors in this phase: nothing is measured or reported).

One operator Arm + START MISSION authorises the whole sequence. For each detected structure the mission picks a reachable
inspection position NEAR it (outside its occupied/unknown footprint, with room for the whole rover, a sensor mounting
allowance, the 50 mm arrival tolerance, tracking uncertainty and stopping distance), drives there with the existing guarded
Navigator, verifies the ACTUAL clearance at a settled stop, then dwells (default 15 s) while tracking, boundary, the map and
the structure itself are monitored. Any invalid observation or a moved structure aborts the dwell (not counted as a visit).
STOP / disarm / a terminal navigator failure cancels the mission for good: no leg or dwell resumes afterwards.
"""
import math
import threading
import time

import cv2
import numpy as np

import geom

DWELL_S = 15.0                 # placeholder collection time
GAP_MM = 110.0                 # rover enclosing circle -> structure edge at the inspection position (sensor space + tolerance)
EDGE_MM = 25.0                 # map cells this close to the boundary are markers/edge artefacts, not structures
MERGE_MM = 50.0                # occupied/unknown fragments closer than this belong to one structure
MATCH_MM = 90.0                # a detection this close to a known structure is that structure (no duplicate visits)
MIN_CLEAR_MM = 25.0            # hard floor for the real rover-to-structure gap at arrival (margin + tracking uncertainty)
DRIFT_MM = 40.0                # the rover moving this far during the dwell invalidates it
MOVE_MM = 45.0                 # the structure's centre moving this far during the dwell invalidates it
LOST_S = 1.5                   # tracking invalid for this long aborts the dwell
MAX_ATTEMPTS = 3
MIN_UNKNOWN_CELLS = 5          # a no-depth-only structure (dark/shiny object) needs at least this many cells
RING_FRAC = 0.06               # width of the candidate ring (sandbox heights) beyond the planned gap
STATUS_TEXT = {"pending": "waiting", "visiting": "driving there", "collecting": "collecting (placeholder)",
               "complete": "visited", "unreachable": "unreachable", "cancelled": "not visited (mission cancelled)",
               "failed": "not visited (failure)"}


# ------------------------------------------------------------------------------------------- detection
def detect_structures(per, nav):
    """Distinct structures from the obstacle map. Occupied cells AND no-depth (unknown) cells count, fragments within
    MERGE_MM are one structure. Excluded: the confidently identified rover (only its own footprint circle), cells at the
    boundary edge (markers / edge artefacts), specks (< 3 cells, or unknown-only < 5 cells), marker sheets. Unknown space stays blocked in
    the planner regardless of whether it is reported as a structure."""
    c, dm = per.calib, per.depth_map
    if c.height_mm is None or c.H is None or c.plane is None:
        return []
    A, hm = c.aspect, c.height_mm
    cols, rows = dm.cols, dm.rows
    cw, ch = A * hm / cols, hm / rows
    cc, rr = np.meshgrid(np.arange(cols), np.arange(rows))
    u, v = (cc + 0.5) * cw, (rr + 0.5) * ch
    occ = dm.ev >= 0.5
    unk = ~dm.seen
    mask = occ | unk
    mask &= np.minimum.reduce([u, A * hm - u, v, hm - v]) >= EDGE_MM
    # boundary markers and their paper sheets are not structures: mask them (marker quad scaled 1.8x about its centre)
    try:
        for mid in (0, 1, 2, 3):
            q = per.last_markers.get(mid)
            if q is None:
                continue
            ctr = q.mean(0)
            poly = c.to_norm(ctr + 1.8 * (q - ctr)) * np.array([cols, rows])
            mk = np.zeros((rows, cols), np.uint8)
            cv2.fillPoly(mk, [np.round(poly - 0.5).astype(np.int32)], 1)
            mask &= ~mk.astype(bool)
    except Exception:
        pass
    rv = per.last_pose
    if rv is not None and rv.get("x") is not None:
        L, W, _m = nav.footprint()
        rad = math.hypot(L, W) / 2 * hm + 10.0
        mask &= np.hypot(u - rv["x"] * A * hm, v - rv["y"] * hm) > rad
    if not mask.any():
        return []
    dx, dy = max(1, int(math.ceil(MERGE_MM / 2 / cw))), max(1, int(math.ceil(MERGE_MM / 2 / ch)))
    grown = cv2.dilate(mask.astype(np.uint8), np.ones((2 * dy + 1, 2 * dx + 1), np.uint8))
    n, lab = cv2.connectedComponents(grown, connectivity=8)
    out = []
    for i in range(1, n):
        m = mask & (lab == i)
        k, ko = int(m.sum()), int((m & occ).sum())
        if k < 3 or (ko == 0 and k < MIN_UNKNOWN_CELLS):
            continue
        rs, cs = np.nonzero(m)
        if rv is not None and rv.get("x") is not None:
            # a remnant of the rover's own no-depth body/fringe, centred where only the rover can be, is not a structure
            L, W, _m = nav.footprint()
            if math.hypot(((cs + 0.5).mean() * cw) - rv["x"] * A * hm, ((rs + 0.5).mean() * ch) - rv["y"] * hm) < math.hypot(L, W) / 2 * hm + 15.0:
                continue
        out.append({"cells": list(zip(cs.tolist(), rs.tolist())), "occ": ko, "unk": k - ko,
                    "cx": float(((cs + 0.5) / cols).mean()), "cy": float(((rs + 0.5) / rows).mean()),
                    "bbox": (float(cs.min() / cols), float(rs.min() / rows), float((cs.max() + 1) / cols), float((rs.max() + 1) / rows))})
    return out


def cell_rects(cells, cols, rows):
    return [(c / cols, r / rows, (c + 1) / cols, (r + 1) / rows) for c, r in cells]


class Structure:
    def __init__(self, sid, cl):
        self.id, self.label = sid, "S%d" % sid
        self.status, self.reason = "pending", ""
        self.attempts, self.missed = 0, 0
        self.tried = []                    # inspection points that failed verification (not reused)
        self.visited_at = None
        self.inspect = None                # the inspection point used / planned (normalized)
        self.set(cl, first=True)

    def set(self, cl, first=False):
        k = 1.0 if first else 0.5          # smooth the centre (detection jitter)
        self.cx = cl["cx"] if first else (1 - k) * self.cx + k * cl["cx"]
        self.cy = cl["cy"] if first else (1 - k) * self.cy + k * cl["cy"]
        self.cells, self.bbox, self.occ, self.unk = cl["cells"], cl["bbox"], cl["occ"], cl["unk"]
        self.missed = 0

    def public(self, calib):
        d = {"id": self.id, "label": self.label, "status": self.status, "status_text": STATUS_TEXT.get(self.status, self.status),
             "reason": self.reason, "x": round(self.cx, 4), "y": round(self.cy, 4), "cells": len(self.cells),
             "occupied": self.occ, "no_depth": self.unk, "attempts": self.attempts,
             "inspect": None if self.inspect is None else [round(self.inspect[0], 4), round(self.inspect[1], 4)]}
        if calib.H is not None:
            d["px"] = [round(float(v), 1) for v in calib.to_px([(self.cx, self.cy)])[0]]
        return d


class StructureBook:
    """Stable identities: a detection within MATCH_MM of a known structure IS that structure (status kept), so jitter and map
    refreshes never create duplicates or re-visits. Unmatched pending structures disappear only after 3 misses; completed or
    reported ones are kept forever (their label and result stay on screen)."""

    def __init__(self):
        self.items, self._next = [], 1

    def update(self, clusters, calib, drop_after=3):
        A, hm = calib.aspect, calib.height_mm or 1.0
        pairs = []
        for i, st in enumerate(self.items):
            for j, cl in enumerate(clusters):
                d = math.hypot((st.cx - cl["cx"]) * A * hm, (st.cy - cl["cy"]) * hm)
                if d <= MATCH_MM:
                    pairs.append((d, i, j))
        pairs.sort()
        used_i, used_j = set(), set()
        for d, i, j in pairs:
            if i in used_i or j in used_j:
                continue
            used_i.add(i)
            used_j.add(j)
            self.items[i].set(clusters[j])
        for i, st in enumerate(self.items):
            if i not in used_i:
                st.missed += 1
        self.items = [s for s in self.items if not (s.status == "pending" and s.missed >= drop_after)]
        for j, cl in enumerate(clusters):
            if j not in used_j:
                self.items.append(Structure(self._next, cl))
                self._next += 1
        return self.items


# ------------------------------------------------------------------------------------------- mission
class Mission:
    def __init__(self, nav, dwell_s=DWELL_S, gap_mm=GAP_MM, poll_s=0.4):
        self.nav, self.per = nav, nav.p
        self.dwell_s, self.gap_mm, self.poll_s = float(dwell_s), float(gap_mm), poll_s
        self.map_every = 3.5                           # s between scene refreshes / structure re-checks during the dwell (map <= 10 s old)
        self.book = StructureBook()
        self.state, self.message = "idle", ""          # idle | running | complete | cancelled | failed
        self.active, self.countdown = None, None
        self.thread, self.cancel_reason = None, ""
        self.lock = threading.Lock()
        self.log = []
        self._key = None

    # ---- operator-visible
    def _log(self, m):
        self.log = (self.log + ["%s %s" % (time.strftime("%H:%M:%S"), m)])[-40:]
        self.nav._log("mission: " + m)

    def refresh_preview(self):
        """Idle: keep the structure list current (labels stay stable across scene refreshes). Not while a mission runs."""
        if self.state == "running":
            return
        key = (self.per.map_frames, self.per.map_seq)
        ob = self.per.last_obs
        if not (ob and ob.pose and (ob.pose.get("verified") or ob.pose.get("source") == "marker")):
            return                                  # rover not yet identified (e.g. just after a restart): its own cells could pose as structures
        if key != self._key and self.per.calib.height_mm:
            self._key = key
            self.book.update(detect_structures(self.per, self.nav), self.per.calib, drop_after=2)

    def snapshot(self):
        c = self.per.calib
        return {"state": self.state, "message": self.message, "active": self.active,
                "countdown": None if self.countdown is None else round(self.countdown, 1),
                "dwell_s": self.dwell_s, "gap_mm": self.gap_mm, "log": self.log[-8:],
                "structures": [s.public(c) for s in self.book.items]}

    def overlay_key(self):
        return (self.state, self.active, None if self.countdown is None else int(math.ceil(self.countdown)),
                tuple((s.id, s.status) for s in self.book.items))

    # ---- control
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, dwell_s=None, gap_mm=None):
        """Explicit operator START only (never from page load). Requires the rover armed and tracked."""
        nav = self.nav
        with self.lock:
            if self.running():
                return False, "a mission is already running"
            if nav.state == "running":
                return False, "a route is running: stop it first"
            if not nav.armed:
                return False, "motion is not armed (Arm first)"
            if not nav.pose or (nav.pose.get("theta") is None and nav.pose.get("theta_raw") is None):
                return False, "rover is not tracked"
            if dwell_s is not None:
                self.dwell_s = max(1.0, min(120.0, float(dwell_s)))
            if gap_mm is not None:
                self.gap_mm = max(60.0, min(250.0, float(gap_mm)))
            self.state, self.message, self.cancel_reason = "running", "", ""
            self.active = self.countdown = None
            for s in self.book.items:          # a new mission starts a fresh visit list (labels stay stable)
                if s.status != "complete":
                    s.status, s.reason, s.attempts, s.tried = "pending", "", 0, []
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True, "mission started"

    def cancel(self, reason="stopped by operator"):
        """STOP/disarm: cancel the whole mission and the current leg. Nothing resumes afterwards."""
        self.cancel_reason = self.cancel_reason or reason
        if self.running():
            self.nav.stop_flag.set()
            self.nav._halt()

    def _cancelled(self):
        if self.nav.cancelled() and not self.cancel_reason:
            self.cancel_reason = "stopped / disarmed"
        return bool(self.cancel_reason)

    def join(self, t=120):
        if self.thread:
            self.thread.join(t)

    # ---- helpers
    def _fresh_full(self):
        src = self.nav.src
        f = getattr(src, "fresh_full", None)
        fr = f(8.0) if f else src.wait_fresh(self.nav._seq_after_stop() + 1, 5.0)
        if fr is None or getattr(fr, "partial", False):
            return None
        rx = getattr(fr, "rx", None)
        if rx is not None and time.time() - rx > 3.0:
            return None
        return fr

    def _refresh_scene(self):
        """Fresh full frame -> current obstacle map. Returns (obs or None, why)."""
        fr = self._fresh_full()
        if fr is None:
            return None, "no fresh full frame"
        try:
            self.per.refresh_map(fr)
        except Exception as e:
            return None, "map refresh failed: %s" % e
        if self.per.map_seq < fr.seq:
            return None, "obstacle map not current"
        obs = self.per.process(fr, update_map=False)
        self.nav._apply_heading(obs)
        return obs, ""

    def _geometry(self):
        L, W, m = self.nav.footprint()
        return math.hypot(L, W) / 2, m

    def _gap_mm(self, pose, st_cells):
        """Real clearance: enclosing circle of the whole rover (heading-independent, conservative) to the structure cells."""
        c = self.per.calib
        A, hm = c.aspect, c.height_mm
        dm = self.per.depth_map
        R, _m = self._geometry()
        u, v = pose["x"] * A, pose["y"]
        best = 9.0
        for r in cell_rects(st_cells, dm.cols, dm.rows):
            best = min(best, math.hypot(max(0.0, r[0] * A - u, u - r[2] * A), max(0.0, r[1] - v, v - r[3])))
        return (best - R) * hm

    def plan_inspection(self, st, exclude=()):
        """Reachable inspection point near `st`: ring of whole-rover-valid points at enclosing radius + gap from its footprint,
        cheapest guarded route first. Returns (point, path_len, '') or (None, None, reason)."""
        nav, c = self.nav, self.per.calib
        A, hm = c.aspect, c.height_mm
        cfg = nav.cfg
        R, m = self._geometry()
        g = self.gap_mm / hm
        pose = nav.pose
        if not pose:
            return None, None, "rover pose unknown"
        pos = (pose["x"], pose["y"])
        rects = cell_rects(st.cells, self.per.depth_map.cols, self.per.depth_map.rows)
        cols, rows = cfg["cols"], cfg["rows"]
        ci, ri = np.meshgrid(np.arange(cols), np.arange(rows), indexing="ij")
        gx, gy = ((ci + 0.5) / cols).ravel(), ((ri + 0.5) / rows).ravel()
        O = np.asarray(rects, np.float64)
        u, v = gx[:, None] * A, gy[:, None]
        dist = np.hypot(np.maximum(0.0, np.maximum(O[None, :, 0] * A - u, u - O[None, :, 2] * A)),
                        np.maximum(0.0, np.maximum(O[None, :, 1] - v, v - O[None, :, 3]))).min(1)
        ok = (dist >= R + g) & (dist <= R + g + RING_FRAC)
        cand = []
        for px, py in zip(gx[ok], gy[ok]):
            if any(math.hypot((px - q[0]) * A, py - q[1]) * hm < 60.0 for q in exclude):
                continue
            cand.append((math.hypot((px - pos[0]) * A, py - pos[1]), float(px), float(py)))
        if not cand:
            return None, None, "no free position at a safe inspection distance"
        cand.sort()
        step = max(1, len(cand) // 14)
        best, tried, why = None, 0, "no guarded route to any inspection position"
        for _d, px, py in cand[::step]:
            if not nav.validate_target(px, py)["ok"]:
                continue
            tried += 1
            res = nav.make_plan(pos, target=(px, py))
            if res["error"]:
                why = res["error"]
                continue
            ln = geom.path_length_sq(res["waypoints"], A)
            if best is None or ln < best[2]:
                best = (px, py, ln)
        if best is None:
            return None, None, why if tried else "the whole rover (plus clearance) fits at no inspection position"
        return (best[0], best[1]), best[2], ""

    def _settled_pose(self):
        nav = self.nav
        obs = nav.observe(nav._seq_after_stop() + 1)          # two consecutive agreeing frames: the rover has stopped
        if obs is None or obs.pose is None or obs.pose.get("theta") is None:
            return None, "rover not tracked / not settled"
        p = obs.pose
        if not p["inside"]:
            return None, "rover outside the boundary"
        out = nav.footprint_outside(p)
        if out:
            return None, out
        return p, ""

    def _matching(self, st):
        """Current detection that is this structure (None if it vanished or moved too far)."""
        c = self.per.calib
        A, hm = c.aspect, c.height_mm
        best = None
        for cl in detect_structures(self.per, self.nav):
            d = math.hypot((st.cx - cl["cx"]) * A * hm, (st.cy - cl["cy"]) * hm)
            if d <= MOVE_MM and (best is None or d < best[0]):
                best = (d, cl)
        return None if best is None else best[1]

    def verify_arrival(self, st, target):
        """Centre tolerance alone is not enough: settled stop, valid pose + whole footprint inside the boundary, and the
        actual gap to the structure on a freshly refreshed map within [MIN_CLEAR_MM, planned gap + tolerance + allowance]."""
        obs, why = self._refresh_scene()
        if obs is None:
            return None, why
        pose, why = self._settled_pose()
        if pose is None:
            return None, why
        cl = self._matching(st)
        if cl is None:
            return None, "structure not found where expected at arrival"
        gap = self._gap_mm(pose, cl["cells"])
        hi = self.gap_mm + RING_FRAC * self.per.calib.height_mm + self.nav.cfg["arrive_tol_mm"] + 40.0     # planned ring + tolerance + allowance
        if gap < MIN_CLEAR_MM:
            return None, "too close to the structure at arrival (%.0f mm clear, need %.0f)" % (gap, MIN_CLEAR_MM)
        if gap > hi:
            return None, "not near enough to the structure (%.0f mm clear, expected <= %.0f)" % (gap, hi)
        return {"pose": pose, "gap": gap, "cells": cl["cells"], "centre": (cl["cx"], cl["cy"])}, ""

    def dwell(self, st, arrival):
        """Timed placeholder collection with continuous monitoring. Returns '' if the full dwell completed with every
        observation valid, else the reason it was aborted (then it is NOT a visit)."""
        nav, c = self.nav, self.per.calib
        A, hm = c.aspect, c.height_mm
        p0 = arrival["pose"]
        c0 = arrival["centre"]
        st.status = "collecting"
        self.active = st.id
        t0 = time.time()
        last_map = time.time()
        bad_since = None
        while True:
            if self._cancelled():
                return "cancelled"
            left = self.dwell_s - (time.time() - t0)
            self.countdown = max(0.0, left)
            if left <= 0:
                return ""
            obs = nav.observe(nav._seq_after_stop() + 1, quick=True, timeout=3.0)
            pose = None if obs is None else obs.pose
            if self._cancelled():
                return "cancelled"
            ok = pose is not None and pose.get("theta") is not None and pose["inside"] and not nav.footprint_outside(pose)
            if not ok:
                bad_since = bad_since or time.time()
                if time.time() - bad_since > LOST_S:
                    return "tracking/boundary invalid for %.1f s" % LOST_S
            else:
                bad_since = None
                moved = math.hypot((pose["x"] - p0["x"]) * A, pose["y"] - p0["y"]) * hm
                if moved > DRIFT_MM:
                    return "the rover moved %.0f mm during collection" % moved
            if time.time() - last_map >= self.map_every:           # keep the map <= 10 s old and re-check the structure
                last_map = time.time()
                o2, why = self._refresh_scene()
                if o2 is None:
                    return "scene update failed: " + why
                cl = self._matching(st)
                if cl is None:
                    return "the structure moved or vanished"
                if math.hypot((cl["cx"] - c0[0]) * A, cl["cy"] - c0[1]) * hm > MOVE_MM:
                    return "the structure moved"
                if ok:
                    gap = self._gap_mm(pose, cl["cells"])
                    if gap < MIN_CLEAR_MM:
                        return "clearance to the structure dropped to %.0f mm" % gap
            time.sleep(self.poll_s)

    # ---- main sequence
    def _run(self):
        nav = self.nav
        nav.keep_armed = True
        try:
            self._sequence()
        except Exception as e:
            import traceback
            self._log("internal error: " + traceback.format_exc().splitlines()[-1])
            self.state, self.message = "failed", "internal error: %s" % e
            self.cancel_reason = self.cancel_reason or "internal error"
        finally:
            nav.keep_armed = False
            self.active = self.countdown = None
            try:
                nav._halt()
                nav.rover.stop()
            except Exception:
                pass
            nav.armed = False                          # one authorisation: the mission never leaves the rover armed
            for s in self.book.items:
                if s.status in ("pending", "visiting", "collecting"):
                    s.status = "cancelled" if self.state != "failed" else "failed"
                    s.reason = s.reason or self.cancel_reason or self.message
            if self.state == "running":
                self.state = "cancelled" if self.cancel_reason else "complete"
                self.message = self.cancel_reason or self._summary()

    def _summary(self):
        n = lambda k: sum(1 for s in self.book.items if s.status == k)
        return "%d visited, %d unreachable, %d not visited" % (n("complete"), n("unreachable"), len(self.book.items) - n("complete") - n("unreachable"))

    def _sequence(self):
        nav = self.nav
        self._log("started: scanning the scene (dwell %.0f s, gap %.0f mm)" % (self.dwell_s, self.gap_mm))
        obs, why = self._refresh_scene()
        if obs is None:
            self.state, self.message = "failed", "could not start: " + why
            return
        self.book.update(detect_structures(self.per, nav), self.per.calib, drop_after=1)    # a fresh scan is authoritative for pending ones
        self._log("%d structure(s): %s" % (len(self.book.items), ", ".join(s.label for s in self.book.items) or "none"))
        while not self._cancelled():
            pend = [s for s in self.book.items if s.status == "pending"]
            if not pend:
                break
            nav.pose = self.per.last_obs.pose if self.per.last_obs and self.per.last_obs.pose else nav.pose
            choice = None
            for s in pend:
                pt, ln, why = self.plan_inspection(s, s.tried)
                if pt is None:
                    s.status, s.reason = "unreachable", why
                    self._log("%s unreachable: %s" % (s.label, why))
                    continue
                if choice is None or ln < choice[2]:
                    choice = (s, pt, ln)
            if choice is None:
                break
            st, pt, _ln = choice
            st.status, st.inspect, st.attempts = "visiting", pt, st.attempts + 1
            self.active = st.id
            self._log("%s: driving to its inspection position (%.3f, %.3f)" % (st.label, pt[0], pt[1]))
            res = self._leg(pt)
            if self._cancelled():
                break
            if res != "done" and nav.soft_failed and nav.armed and not self._cancelled():
                # route-class failure (no route / no safe alternative): the rover is stopped, authorisation is intact.
                # Try ANOTHER inspection position near the SAME structure; report it unreachable when they run out.
                self._retry_or_report(st, pt, "inspection position became inaccessible: %s" % nav.reason[:160])
                continue
            if res != "done":
                self.state, self.message = "failed", "%s: navigation stopped: %s" % (st.label, nav.reason)
                st.status, st.reason = "failed", nav.reason
                self.cancel_reason = self.message
                self._log(self.message)
                break
            arr, why = self.verify_arrival(st, pt)
            if self._cancelled():
                break
            if arr is None:
                self._retry_or_report(st, pt, "arrival check failed: " + why)
                continue
            self._log("%s: arrived, %.0f mm clear; collecting (placeholder)" % (st.label, arr["gap"]))
            why = self.dwell(st, arr)
            self.countdown = None
            if self._cancelled():
                break
            if why:
                self._retry_or_report(st, pt, "collection not counted: " + why)
                continue
            st.status, st.visited_at, st.reason = "complete", time.time(), ""
            self.active = None
            self._log("%s: complete" % st.label)
            self._rescan()
        self.active = None

    def _rescan(self):
        obs, why = self._refresh_scene()
        if obs is not None:
            if obs.pose is not None:
                self.nav.pose = obs.pose
            self.book.update(detect_structures(self.per, self.nav), self.per.calib)

    def _retry_or_report(self, st, pt, why):
        st.tried.append(pt)
        st.reason = why
        self._log("%s: %s" % (st.label, why))
        if st.attempts >= MAX_ATTEMPTS:
            st.status = "unreachable"
            st.reason = "no verified safe inspection after %d attempts: %s" % (st.attempts, why)
        else:
            st.status = "pending"
        self.active = None
        self._rescan()

    def _leg(self, target):
        """One guarded Navigator run to the inspection point; returns the Navigator's final state."""
        nav = self.nav
        ok, msg = nav.start(target)
        if not ok:
            self.cancel_reason = self.cancel_reason or msg
            return "refused"
        while True:
            t = nav.thread
            if t is not None:
                t.join(0.3)
            if nav.state != "running" and (t is None or not t.is_alive()):
                break
            if self._cancelled():
                nav.stop()
                if t is not None:
                    t.join(5.0)
                return "cancelled"
        return nav.state
