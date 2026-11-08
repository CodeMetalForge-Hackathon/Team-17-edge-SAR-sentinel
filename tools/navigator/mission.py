"""Structure visits with a timed placeholder collection (no sensors in this phase: nothing is measured or reported).

One operator Arm + START MISSION authorises the whole sequence. For each detected structure the mission picks a reachable
inspection position NEAR it (outside its occupied/unknown footprint, with room for the whole rover, a sensor mounting
allowance, the 50 mm arrival tolerance, tracking uncertainty and stopping distance), drives there with the existing guarded
Navigator, verifies the ACTUAL clearance at a settled stop, then dwells (default 15 s) while tracking, boundary, the map and
the structure itself are monitored. Any invalid observation or a moved structure aborts the dwell (not counted as a visit).
STOP / disarm / a terminal navigator failure cancels the mission for good: no leg or dwell resumes afterwards.

SCENE CHANGES. At START a fresh full scene is captured as the baseline (scene.py). A monitor thread then refreshes the full
RGB+depth scene and the obstacle map every SCENE_PERIOD_S (well inside the 10 s map-age limit) and compares it with the previous
valid scene. A moved structure or a newly appearing object becomes an EVENT with a stable identity. Detection happens only on
those scene updates; the moment one reports a change the current navigation (or structure collection) is stopped, valid
observations are re-acquired, and the event is inspected at a safe position near its NEW location with the same monitored
placeholder dwell. Interrupted collections are not complete; unfinished work resumes afterwards. STOP / disarm / terminal safety
failures (incl. a calibration problem) cancel everything: nothing resumes or re-arms.
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

import geom
import scene as SC

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
SCENE_PERIOD_S = 5.0           # full-scene acquisition period (start to start); the map-age limit is 10 s
INVALID_MAX_S = 12.0           # no valid (rover-identified) scene for this long is a terminal tracking failure
MAX_EVENTS = 12                # per mission: a scene that keeps changing cannot starve the rest of the work forever
MAX_MOVES_CHASED = 3           # an inspected object that keeps moving is reported, not chased for ever
LIVE = ("pending", "visiting", "collecting")
STATUS_TEXT = {"pending": "waiting", "visiting": "driving there", "collecting": "collecting (placeholder)",
               "complete": "visited", "unreachable": "unreachable", "cancelled": "not visited (mission cancelled)",
               "failed": "not visited (failure)", "gone": "gone (removed before inspection)"}


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
        self.gone = False                  # removed from the scene (no longer a place to visit)
        self.interrupted = 0
        self.set(cl, first=True)

    def set(self, cl, first=False):
        k = 1.0 if first else 0.5          # smooth the centre (detection jitter)
        self.cx = cl["cx"] if first else (1 - k) * self.cx + k * cl["cx"]
        self.cy = cl["cy"] if first else (1 - k) * self.cy + k * cl["cy"]
        self.cells, self.bbox, self.occ, self.unk = cl["cells"], cl["bbox"], cl["occ"], cl["unk"]
        self.missed = 0

    def relocate(self, cx, cy, cells):
        """The structure was moved (scene change): same identity, new place."""
        self.cx, self.cy, self.cells, self.missed = cx, cy, list(cells), 0

    def public(self, calib):
        d = {"id": self.id, "label": self.label, "gone": self.gone, "status": self.status, "status_text": STATUS_TEXT.get(self.status, self.status),
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

    def update(self, clusters, calib, drop_after=3, create=True):
        """create=False (during a mission): detections only refresh known structures. A new object is an EVENT, not a structure."""
        A, hm = calib.aspect, calib.height_mm or 1.0
        pairs = []
        for i, st in enumerate(self.items):
            if st.gone:
                continue
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
            if i not in used_i and not st.gone:
                st.missed += 1
        self.items = [s for s in self.items if not (s.status == "pending" and s.missed >= drop_after)]
        if not create:
            for s in self.items:       # a visited structure that has been absent from the refreshed map for several scenes was removed
                if s.status == "complete" and not s.gone and s.missed >= drop_after:
                    s.gone, s.status, s.reason = True, "gone", "removed from the scene (it had been visited)"
        if create:
            for j, cl in enumerate(clusters):
                if j not in used_j:
                    self.items.append(Structure(self._next, cl))
                    self._next += 1
        return self.items

    def get(self, sid):
        return next((s for s in self.items if s.id == sid), None)


# ------------------------------------------------------------------------------------------- scene-change events
EVENT_TEXT = {"moved": "moved", "new": "new object"}


class Event:
    """One scene change to inspect. Stable identity (E1, E2, ...) for the whole mission: re-detections of the same change, jitter
    and refreshes never create another one; the same object moving again BEFORE inspection updates this event in place."""

    def __init__(self, eid, kind, to, cur, prev_seq, prev_rx, stats, struct=None, frm=None, parent=None):
        self.id, self.label, self.kind = eid, "E%d" % eid, kind
        self.status, self.reason = "pending", ""
        self.attempts, self.tried, self.inspect = 0, [], None
        self.interrupted, self.moves_chased, self.revision = 0, 0, 0
        self.visited_at, self.parent = None, parent
        self.struct_id = None if struct is None else struct.id
        self.struct_label = None if struct is None else struct.label
        self.detected_wall = time.time()
        self.present_frac = None
        self.image = b""
        self.history = []
        self.set_location(to, frm)
        self.evidence = self._evidence(to, frm, cur, prev_seq, prev_rx, stats)
        self.sig_from = None if frm is None else {"dil": cv2.dilate(frm["mask"].astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)}

    def set_location(self, to, frm=None):
        self.cx, self.cy = to["xy"]
        self.cells = list(to["cells"])
        self.sig = to.get("sig")
        self.support = to["support"]
        self.from_xy = None if frm is None else tuple(frm["xy"])

    def _evidence(self, to, frm, cur, prev_seq, prev_rx, stats):
        e = {"scene_seq": cur.seq, "scene_rx": round(cur.rx, 3), "scene_rx_time": time.strftime("%H:%M:%S", time.localtime(cur.rx)),
             "prev_scene_seq": prev_seq, "prev_scene_rx": round(prev_rx, 3), "detected_at": time.strftime("%H:%M:%S", time.localtime(self.detected_wall)),
             "detection_delay_s": round(self.detected_wall - cur.rx, 2), "support": to["support"], "area_px": to["area"],
             "height_mm": None if to["h_cur"] is None else round(to["h_cur"], 1),
             "height_before_mm": None if to["h_ref"] is None else round(to["h_ref"], 1),
             "baseline_height_mm": None if to["h_base"] is None else round(to["h_base"], 1),
             "dh_mm": round(to["dh_mm"], 1), "rgb_dL": round(to["dL"], 1), "rgb_dC": round(to["dC"], 1),
             "depth_threshold_mm": stats.get("T_mm"), "noise_p999_mm": stats.get("noise_p999_mm"),
             "x": round(self.cx, 4), "y": round(self.cy, 4)}
        e["vs_baseline"] = ("not in the baseline (baseline height there %s mm)" % e["baseline_height_mm"]) if (
            e["baseline_height_mm"] is None or abs((e["height_mm"] or 0) - e["baseline_height_mm"]) > 20 or to["support"] != "depth") else "differs from the baseline"
        if frm is not None:
            e["from_x"], e["from_y"] = round(frm["xy"][0], 4), round(frm["xy"][1], 4)
            e["from_area_px"], e["from_height_mm"] = frm["area"], None if frm["h_ref"] is None else round(frm["h_ref"], 1)
        return e

    @property
    def text(self):
        t = EVENT_TEXT[self.kind]
        if self.kind == "moved":
            t = ("%s moved" % self.struct_label) if self.struct_label else "object moved"
        return t

    def public(self, calib):
        d = {"id": self.id, "label": self.label, "kind": self.kind, "text": self.text, "status": self.status,
             "status_text": STATUS_TEXT.get(self.status, self.status), "reason": self.reason, "x": round(self.cx, 4), "y": round(self.cy, 4),
             "attempts": self.attempts, "interrupted": self.interrupted, "revision": self.revision, "struct": self.struct_label,
             "parent": None if self.parent is None else "E%d" % self.parent, "evidence": self.evidence, "history": self.history[-6:],
             "present": None if self.present_frac is None else round(self.present_frac, 2),
             "inspect": None if self.inspect is None else [round(self.inspect[0], 4), round(self.inspect[1], 4)],
             "from": None if self.from_xy is None else [round(self.from_xy[0], 4), round(self.from_xy[1], 4)],
             "has_image": bool(self.image)}
        if calib.H is not None:
            d["px"] = [round(float(v), 1) for v in calib.to_px([(self.cx, self.cy)])[0]]
            if self.from_xy is not None:
                d["from_px"] = [round(float(v), 1) for v in calib.to_px([self.from_xy])[0]]
        return d


class EventBook:
    def __init__(self):
        self.items, self._next = [], 1

    def add(self, *a, **k):
        ev = Event(self._next, *a, **k)
        self._next += 1
        self.items.append(ev)
        return ev

    def live(self):
        return [e for e in self.items if e.status in LIVE]

    def pending(self):
        return sorted((e for e in self.items if e.status == "pending"), key=lambda e: e.id)      # FIFO: oldest change first


class SceneMonitor:
    """Owns full-scene acquisition for the whole mission. Every SCENE_PERIOD_S (start to start; or on demand) it fetches a fresh
    full frame, refreshes the obstacle map from it, compares it with the previous valid scene and registers the changes. Rover
    tracking, the live display and STOP never wait for it (they run on their own threads at frame rate)."""

    def __init__(self, mission, period_s):
        self.m, self.per, self.nav = mission, mission.per, mission.nav
        self.watch = SC.SceneWatch(self.per)
        self.period = float(period_s)
        self.updates, self.count = [], 0
        self.fatal, self.last_why, self.last_ok = "", "", False
        self.last_rx, self.last_valid_rx, self.last_valid_wall = 0.0, None, 0.0
        self.max_interval = 0.0
        self.baseline_info = None
        self.invalid_since = None
        self.errors = 0
        self.cond = threading.Condition()
        self._wake, self._stop = threading.Event(), threading.Event()
        self.thread = None
        self.next_due = 0.0
        self.log_path = os.path.join(self.nav.dir, "scene_log.jsonl")

    # ---- capture
    def _grab(self):
        m, per, nav = self.m, self.per, self.nav
        fr, why = m._scene_frame()
        if fr is None:
            return None, None, False, why
        ob = per.process(fr, update_map=False)
        nav._apply_heading(ob)
        pose = ob.pose
        conf = pose is not None and (pose.get("source") == "marker" or pose.get("verified", True) or not per._pattern_seen)
        return fr, pose, conf, ""

    def _footprint_mm(self):
        L, W, _m = self.nav.footprint()
        hm = self.per.calib.height_mm or 0.0
        return (L * hm, W * hm)

    def baseline(self):
        """Fresh full scene at mission START: the initial environment. Returns (ok, why)."""
        per = self.per
        fr, pose, conf, why = self._grab()
        if fr is None:
            return False, why
        try:
            per.refresh_map(fr)
        except Exception as e:
            return False, "map refresh failed: %s" % e
        if per.map_seq < fr.seq:
            return False, "obstacle map not current"
        cur, res = self.watch.observe(fr, pose, conf, self._footprint_mm())
        if res.status != "baseline":
            return False, ("the baseline scene is not usable: " + (res.why or res.status))
        self.baseline_info = {"seq": fr.seq, "rx": round(cur.rx, 3), "rx_time": time.strftime("%H:%M:%S", time.localtime(cur.rx)),
                              "rover": [round(pose["x"], 4), round(pose["y"], 4)], "comparable_px": res.stats.get("area_px")}
        self.last_rx, self.last_valid_rx, self.last_valid_wall = cur.rx, cur.rx, time.time()
        self.last_ok = True
        self.next_due = time.time() + self.period
        self._append({"n": 0, "kind": "baseline", "seq": fr.seq, "rx": round(cur.rx, 3), "rx_time": self.baseline_info["rx_time"]})
        return True, ""

    def begin(self):
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self._stop.set()
        self._wake.set()
        t = self.thread
        if t is not None and t is not threading.current_thread():
            t.join(5.0)

    def age(self):
        return time.time() - self.per.map_time if self.per.map_time else 1e9

    # ---- schedule
    def _loop(self):
        m = self.m
        while not self._stop.is_set():
            self._wake.wait(timeout=max(0.02, min(0.25, self.next_due - time.time())))
            if self._stop.is_set() or m._cancelled():
                break
            if not self._wake.is_set() and time.time() < self.next_due:
                continue
            self._wake.clear()
            t0 = time.time()
            try:
                self._update()
                self.errors = 0
            except Exception as e:
                import traceback
                self.errors += 1
                m._log("scene update error: " + traceback.format_exc().splitlines()[-1])
                self.last_rx = time.time()
                self._finish_update({"valid": False, "status": "error", "why": str(e)[:120]}, ok=False)
                if self.errors >= 3:
                    self._fatal("scene update keeps failing: %s" % e)
            self.next_due = t0 + self.period              # start-to-start period (an on-demand update re-times the schedule)
            age = self.age()
            if age > self.nav.cfg["map_max_age_s"] - 0.5 and not self.fatal:
                self._fatal("obstacle map is %.0f s old (scene refresh failed: %s)" % (age, self.last_why or self.per.map_err or "no fresh full frame"))
        with self.cond:
            self.cond.notify_all()

    def update_now(self, timeout=15.0):
        """Wait for a scene update whose frame was captured AFTER this call. Returns (record, '') or (None, why)."""
        t = time.time()
        end = t + timeout
        self._wake.set()
        seen = self.count
        with self.cond:
            while True:
                if self.fatal or self.m._cancelled():
                    return None, self.fatal or "cancelled"
                if self.last_rx > t:
                    ok = self.last_ok
                    return (self.updates[-1], "") if ok else (None, self.last_why or "scene not valid")
                left = end - time.time()
                if left <= 0:
                    return None, "no fresh scene update within %.0f s" % timeout
                if self.thread is None or not self.thread.is_alive():
                    return None, "scene monitor is not running"
                self.cond.wait(min(left, 0.2))
                if self.count != seen:                      # the update in progress when we asked was too old: ask for another
                    seen = self.count
                    self._wake.set()

    def _fatal(self, why):
        if not self.fatal:
            self.fatal = why
            self.m._terminal(why)
        with self.cond:
            self.cond.notify_all()

    def _append(self, rec):
        self.updates = (self.updates + [rec])[-60:]
        try:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(rec, default=str) + "\n")
        except Exception:
            pass

    def _finish_update(self, rec, ok):
        self.count += 1
        rec["n"] = self.count
        rec["done"] = time.strftime("%H:%M:%S", time.localtime())
        rec["done_wall"] = round(time.time(), 3)
        rec["map_seq"], rec["map_age_s"] = self.per.map_seq, round(self.age(), 2)
        self.last_ok = ok
        self.last_why = "" if ok else (rec.get("why") or rec.get("status", ""))
        self._append(rec)
        with self.cond:
            self.cond.notify_all()

    # ---- one scene update
    def _update(self):
        m, per = self.m, self.per
        t0 = time.time()
        fr, pose, conf, why = self._grab()
        if fr is None:
            self.last_rx = time.time()               # an attempt was made (wakes update_now with the failure)
            self._finish_update({"valid": False, "status": "no-frame", "why": why}, ok=False)
            return
        rec = {"seq": fr.seq, "rx": round(getattr(fr, "rx", t0), 3), "rx_time": time.strftime("%H:%M:%S", time.localtime(getattr(fr, "rx", t0)))}
        map_ok = True
        try:
            per.refresh_map(fr)                                  # occupancy at old AND new locations, from this scene's depth
        except Exception as e:
            map_ok = False
            rec["map_error"] = str(e)[:100]
        prev_ref_seq = self.watch.ref.seq if self.watch.ref else None
        prev_ref_rx = self.watch.ref.rx if self.watch.ref else 0.0
        cur, res = self.watch.observe(fr, pose, conf, self._footprint_mm())
        rec["rover"] = bool(conf)
        rec["status"], rec["prev_seq"] = res.status, prev_ref_seq
        rec["stats"] = {k: v for k, v in res.stats.items()}
        rx = getattr(fr, "rx", t0)
        if res.status == "calibration":
            rec.update(valid=False, why=res.why)
            self.last_rx = rx
            self._finish_update(rec, ok=False)
            self._fatal("scene comparison invalid - calibration problem: " + res.why)
            return
        if res.status == "invalid":
            self.invalid_since = self.invalid_since or time.time()
            rec.update(valid=False, why=res.why)
            self.last_rx = rx
            self._finish_update(rec, ok=False)
            if time.time() - self.invalid_since > INVALID_MAX_S:
                self._fatal("no valid scene observation for %.0f s: %s" % (INVALID_MAX_S, res.why))
            return
        self.invalid_since = None
        if self.last_valid_rx is not None:
            iv = rx - self.last_valid_rx
            rec["interval_s"] = round(iv, 2)
            self.max_interval = max(self.max_interval, iv)
        self.last_valid_rx, self.last_valid_wall = rx, time.time()
        new, active = self._register(res, cur, fr, prev_ref_seq, prev_ref_rx, rec)
        rec["valid"] = True
        rec["new_events"] = [e.label for e in new]
        rec["ms"] = round((time.time() - t0) * 1000)
        self.last_rx = rx
        if not map_ok:
            rec["why"] = "map refresh failed: " + rec["map_error"]
            self._finish_update(rec, ok=False)
        else:
            self._finish_update(rec, ok=True)
        m._on_changes(new, active)

    # ---- registry
    def _struct_at(self, blob, strict):
        cells = set(blob["cells"])
        A, hm = self.per.calib.aspect, self.per.calib.height_mm or 1.0
        for st in self.m.book.items:
            if st.gone:
                continue
            sc = set(st.cells)
            if sc and cells and len(cells & sc) >= (0.5 if strict else 0.3) * min(len(sc), len(cells)):
                return st
            if not strict and math.hypot((blob["xy"][0] - st.cx) * A * hm, (blob["xy"][1] - st.cy) * hm) <= 60.0:
                return st
        return None

    @staticmethod
    def _explained(blob, events, was_here=False):
        """The event whose object this blob is: where it is NOW (a destination) or, for a vacated place, also where it was."""
        n = max(1, int(blob["mask"].sum()))
        for ev in events:
            if ev.status == "gone":
                continue
            for dil in (ev.sig["dil"] if ev.sig else None, ev.sig_from["dil"] if (was_here and ev.sig_from) else None):
                if dil is not None and float((blob["mask"] & dil).sum()) / n >= 0.4:
                    return ev
        return None

    def _register(self, res, cur, fr, prev_seq, prev_rx, rec):
        m, w = self.m, self.watch
        evs = m.events
        new, active = [], []
        removed = 0
        with m.elock:
            rev0 = {ev.id: ev.revision for ev in evs.live()}
            for cand in res.cands:
                t, to, frm = cand["type"], cand.get("to"), cand.get("from")
                ev_to = self._explained(to, evs.items) if to else None
                ev_from = self._explained(frm, evs.items, was_here=True) if frm else None
                if t == "removed":
                    removed += 1
                    if ev_from is not None and ev_from.status in LIVE:
                        self._gone(ev_from, "removed before inspection", active)
                    continue                                       # (structures are only retired after 3 scenes of absence: see StructureBook)
                if t == "moved" and ev_from is not None and ev_from.status in LIVE:
                    ev = ev_from                               # the pending object moved again: same event, new place
                    if ev.kind == "moved" and ev.from_xy is None:
                        ev.from_xy = (ev.cx, ev.cy)
                    old = (ev.cx, ev.cy)
                    ev.set_location(to, frm)
                    ev.sig_from = {"dil": cv2.dilate(frm["mask"].astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)}
                    ev.revision += 1
                    ev.moves_chased += 1
                    ev.evidence.update(x=round(ev.cx, 4), y=round(ev.cy, 4), scene_seq=cur.seq, moved_again_at=time.strftime("%H:%M:%S"))
                    ev.history.append("moved again in scene %s: (%.3f,%.3f) -> (%.3f,%.3f)" % (cur.seq, old[0], old[1], ev.cx, ev.cy))
                    if ev.moves_chased > MAX_MOVES_CHASED:
                        ev.status, ev.reason = "unreachable", "the object keeps moving (%d moves before it could be inspected)" % ev.moves_chased
                    if m.active == ev.label:
                        active.append(ev)
                    m._log("%s moved again before inspection: now (%.3f, %.3f)" % (ev.label, ev.cx, ev.cy))
                    continue
                if ev_to is not None:
                    continue                                   # already an event (same object seen again): no duplicate
                if t == "new" and self._struct_at(to, strict=True) is not None:
                    continue                                   # a known structure seen again
                if len(evs.items) >= MAX_EVENTS:
                    m._log("event limit (%d) reached: further scene changes are not inspected" % MAX_EVENTS)
                    continue
                st = self._struct_at(frm, strict=False) if (t == "moved") else None
                parent = ev_from.id if (t == "moved" and ev_from is not None) else None
                ev = evs.add("moved" if t == "moved" else "new", to, cur, prev_seq, prev_rx, res.stats, struct=st, frm=frm, parent=parent)
                ev.image = self._evidence_image(fr, to, frm, ev)
                if st is not None:                             # same structure identity, new place; its occupancy is already refreshed
                    cl = self._cluster_near(to)
                    st.relocate(cl["cx"] if cl else ev.cx, cl["cy"] if cl else ev.cy, cl["cells"] if cl else ev.cells)
                new.append(ev)
                m._log("%s detected (scene %s vs %s, %.1fs after capture): %s at (%.3f, %.3f)%s" % (
                    ev.label, cur.seq, prev_seq, ev.evidence["detection_delay_s"], ev.text, ev.cx, ev.cy,
                    "" if ev.from_xy is None else ", was (%.3f, %.3f)" % ev.from_xy))
            for ev in evs.live():                              # is each pending object still there? (candidates above come first:
                if ev.sig is None or ev.revision != rev0.get(ev.id, ev.revision):      # an object that MOVED is retargeted, not gone)
                    continue
                pf = w.presence(ev.sig, cur)
                ev.present_frac = pf
                if pf is not None and pf < 0.5:
                    self._gone(ev, "removed before inspection (%.0f%% of it still visible)" % (100 * pf), active)
            rec["removed"] = removed
            try:
                before = {s.id for s in m.book.items if not s.gone}
                m.book.update(detect_structures(self.per, self.nav), self.per.calib, drop_after=3, create=False)
                for sid in before - {s.id for s in m.book.items if not s.gone}:
                    m._log("S%d removed from the scene" % sid)
            except Exception as e:
                m._log("structure refresh error: %s" % e)
        return new, active

    def _gone(self, ev, why, active):
        ev.status, ev.reason = "gone", why
        ev.revision += 1
        ev.history.append(why)
        self.m._log("%s: %s" % (ev.label, why))
        if self.m.active == ev.label:
            active.append(ev)

    def _cluster_near(self, blob, max_mm=150.0):
        A, hm = self.per.calib.aspect, self.per.calib.height_mm or 1.0
        best = None
        for cl in detect_structures(self.per, self.nav):
            d = math.hypot((cl["cx"] - blob["xy"][0]) * A * hm, (cl["cy"] - blob["xy"][1]) * hm)
            if d <= max_mm and (best is None or d < best[0]):
                best = (d, cl)
        return None if best is None else best[1]

    def _evidence_image(self, fr, to, frm, ev):
        try:
            img = fr.bgr.copy()
            for b, col in ((to, (0, 255, 0)), (frm, (0, 0, 255))):
                if b is None:
                    continue
                cs, _ = cv2.findContours(b["mask"].astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(img, cs, -1, col, 2)
            cv2.putText(img, "%s %s  scene %s  (green=now, red=was)" % (ev.label, ev.text, fr.seq), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
        except Exception:
            return b""


# ------------------------------------------------------------------------------------------- mission
class Mission:
    def __init__(self, nav, dwell_s=DWELL_S, gap_mm=GAP_MM, poll_s=0.4, scene_period=SCENE_PERIOD_S):
        self.nav, self.per = nav, nav.p
        self.dwell_s, self.gap_mm, self.poll_s = float(dwell_s), float(gap_mm), poll_s
        self.scene_period = float(scene_period)
        self.book = StructureBook()
        self.events = EventBook()
        self.monitor = None
        self.state, self.message = "idle", ""          # idle | running | complete | cancelled | failed
        self.active, self.countdown = None, None
        self.thread, self.cancel_reason = None, ""
        self.lock = threading.Lock()
        self.elock = threading.RLock()                 # guards the event registry / interrupt decisions
        self.event_gen, self.seen_gen = 0, 0           # event_gen moves when a change is registered; seen_gen = what planning has seen
        self.phase = "idle"                            # idle | leg | dwell-S | dwell-E : what an interrupt would stop
        self._preempt = threading.Event()
        self.preempt_why = ""
        self.interruptions = 0
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

    def scene_info(self):
        mon = self.monitor
        if mon is None:
            return {"period_s": self.scene_period, "baseline": None, "updates": [], "count": 0}
        return {"period_s": mon.period, "baseline": mon.baseline_info, "count": mon.count, "max_interval_s": round(mon.max_interval, 2),
                "map_age_s": round(mon.age(), 1) if mon.age() < 1e8 else None, "fatal": mon.fatal, "updates": mon.updates[-8:],
                "last_valid": None if mon.last_valid_rx is None else {"rx": round(mon.last_valid_rx, 3),
                                                                     "rx_time": time.strftime("%H:%M:%S", time.localtime(mon.last_valid_rx)),
                                                                     "age_s": round(time.time() - mon.last_valid_wall, 1)}}

    def snapshot(self):
        c = self.per.calib
        return {"state": self.state, "message": self.message, "active": self.active,
                "countdown": None if self.countdown is None else round(self.countdown, 1),
                "dwell_s": self.dwell_s, "gap_mm": self.gap_mm, "log": self.log[-8:],
                "structures": [s.public(c) for s in self.book.items if not (s.gone and s.status == "gone")],
                "events": [e.public(c) for e in list(self.events.items)], "scene": self.scene_info(),
                "phase": self.phase, "interruptions": self.interruptions}

    def overlay_key(self):
        return (self.state, self.active, None if self.countdown is None else int(math.ceil(self.countdown)),
                tuple((s.id, s.status) for s in self.book.items), tuple((e.id, e.status, e.revision) for e in self.events.items),
                0 if self.monitor is None else self.monitor.count)

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
            self.phase, self.interruptions, self.preempt_why = "idle", 0, ""
            self._preempt.clear()
            nav.preempt.clear()
            nav.stop_flag.clear()                      # this START follows a fresh Arm: an old STOP must not cancel the new mission
            if not nav.armed:                          # ...but a STOP/disarm racing with this START still wins
                nav.stop_flag.set()
                self.state, self.message = "idle", "stopped"
                return False, "motion is not armed (Arm first)"
            self.events = EventBook()
            self.monitor = SceneMonitor(self, self.scene_period)
            self.event_gen = self.seen_gen = 0
            for s in self.book.items:          # a new mission starts a fresh visit list (labels stay stable)
                if s.status != "complete":
                    s.status, s.reason, s.attempts, s.tried = "pending", "", 0, []
                    s.gone = False
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True, "mission started"

    def cancel(self, reason="stopped by operator"):
        """STOP/disarm: cancel the whole mission and the current leg. Nothing resumes afterwards."""
        self.cancel_reason = self.cancel_reason or reason
        if self.running():
            self.nav.stop_flag.set()
            self.nav._halt()

    def _terminal(self, reason):
        """A terminal safety failure found off the mission thread (scene monitor): stop everything for good."""
        with self.elock:
            if not self.cancel_reason:
                self.cancel_reason = reason
                self.state, self.message = "failed", reason
        self._log("TERMINAL: " + reason)
        try:
            self.nav.stop_flag.set()
            self.nav._halt()
            self.nav.rover.stop()
        except Exception:
            pass

    def _cancelled(self):
        nav, mon = self.nav, self.monitor
        if not self.cancel_reason and mon is not None and mon.fatal:
            self.cancel_reason = mon.fatal
        if not self.cancel_reason:
            if not nav.armed:
                if nav.state == "failed" and nav.reason and not nav.soft_failed:
                    self.cancel_reason = "navigation stopped: " + nav.reason
                    self.state, self.message = "failed", self.cancel_reason
                else:
                    self.cancel_reason = "stopped / disarmed"
            elif nav.stop_flag.is_set() and not (nav.preempt.is_set() or self._preempt.is_set()):
                self.cancel_reason = "stopped / disarmed"
        return bool(self.cancel_reason)

    def join(self, t=120):
        if self.thread:
            self.thread.join(t)

    # ---- helpers
    def _scene_frame(self):
        """A fresh COMPLETE frame captured after this call (None, why) otherwise."""
        src = self.nav.src
        f = getattr(src, "scene_frame", None)
        if f is not None:
            fr = f()
        else:
            ff = getattr(src, "fresh_full", None)
            fr = ff(8.0) if ff else src.wait_fresh(self.nav._seq_after_stop() + 1, 5.0)
        if fr is None:
            return None, "no fresh full frame"
        if getattr(fr, "partial", False):
            return None, "only a partial frame"
        rx = getattr(fr, "rx", None)
        if rx is not None and time.time() - rx > 3.0:
            return None, "the full frame is %.1f s old" % (time.time() - rx)
        return fr, ""

    def _refresh_scene(self):
        """A scene update that completed after this call (map + comparison + events). (record or None, why)."""
        if self.monitor is None:
            return None, "no scene monitor"
        return self.monitor.update_now()

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

    def _locate(self, item):
        """Current evidence for `item` (a structure from the obstacle map, an event from the scene comparison), or None."""
        if isinstance(item, Event):
            if item.status == "gone" or (item.present_frac is not None and item.present_frac < 0.5):
                return None
            return {"cells": item.cells, "cx": item.cx, "cy": item.cy}
        return self._matching(item)

    def verify_arrival(self, st, target):
        """Centre tolerance alone is not enough: settled stop, valid pose + whole footprint inside the boundary, and the
        actual gap to the structure on a freshly refreshed map within [MIN_CLEAR_MM, planned gap + tolerance + allowance]."""
        rec, why = self._refresh_scene()
        if rec is None:
            return None, why
        pose, why = self._settled_pose()
        if pose is None:
            return None, why
        cl = self._locate(st)
        if cl is None:
            return None, "%s not found where expected at arrival" % ("the object" if isinstance(st, Event) else "structure")
        gap = self._gap_mm(pose, cl["cells"])
        hi = self.gap_mm + RING_FRAC * self.per.calib.height_mm + self.nav.cfg["arrive_tol_mm"] + 40.0     # planned ring + tolerance + allowance
        if gap < MIN_CLEAR_MM:
            return None, "too close to the structure at arrival (%.0f mm clear, need %.0f)" % (gap, MIN_CLEAR_MM)
        if gap > hi:
            return None, "not near enough to the structure (%.0f mm clear, expected <= %.0f)" % (gap, hi)
        return {"pose": pose, "gap": gap, "cells": cl["cells"], "centre": (cl["cx"], cl["cy"])}, ""

    def dwell(self, st, arrival):
        """Timed placeholder collection with continuous monitoring. Returns '' if the full dwell completed with every
        observation valid, 'interrupted' if a scene change preempted it, else the reason it was aborted (never a visit)."""
        nav, c, mon = self.nav, self.per.calib, self.monitor
        A, hm = c.aspect, c.height_mm
        is_ev = isinstance(st, Event)
        with self.elock:
            if self._preempt.is_set() or (not is_ev and self.event_gen != self.seen_gen):
                if not self._preempt.is_set():
                    self.interruptions += 1
                return "interrupted"                   # a change was registered before collection could start
            self.phase = "dwell-E" if is_ev else "dwell-S"
            st.status = "collecting"
            self._log("%s: collecting (placeholder, %.0f s)" % (st.label, self.dwell_s))
        try:
            p0 = arrival["pose"]
            c0 = arrival["centre"]
            rev0 = st.revision if is_ev else None
            self.active = st.label if is_ev else st.id
            t0 = time.time()
            ver = mon.count
            bad_since = None
            while True:
                if self._cancelled():
                    return "cancelled"
                if self._preempt.is_set():
                    return "interrupted"
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
                if mon.count != ver:                                   # a new full-scene update: re-check the object and the map age
                    ver = mon.count
                    if not mon.last_ok:
                        return "scene update failed: " + (mon.last_why or "invalid")
                    if is_ev and st.revision != rev0:
                        return "the object moved" if st.status != "gone" else "the object was removed"
                    cl = self._locate(st)
                    if cl is None:
                        return "the object moved or vanished" if is_ev else "the structure moved or vanished"
                    if math.hypot((cl["cx"] - c0[0]) * A, cl["cy"] - c0[1]) * hm > MOVE_MM:
                        return "the object moved" if is_ev else "the structure moved"
                    if ok:
                        gap = self._gap_mm(pose, cl["cells"])
                        if gap < MIN_CLEAR_MM:
                            return "clearance to the object dropped to %.0f mm" % gap
                elif mon.age() > nav.cfg["map_max_age_s"]:
                    return "scene updates stalled (map %.0f s old)" % mon.age()
                time.sleep(self.poll_s)
        finally:
            with self.elock:
                self.phase = "idle"

    # ---- scene changes -> interruption (called from the monitor thread right after it registered them)
    def _on_changes(self, new, active):
        with self.elock:
            if new:
                self.event_gen += 1
                self.preempt_why = ", ".join("%s (%s)" % (e.label, e.text) for e in new)
            if self.cancel_reason or self.state != "running":
                return
            why = ""
            if new and self.phase in ("leg", "dwell-S"):
                why = ", ".join("%s (%s)" % (e.label, e.text) for e in new)
            elif active and self.phase in ("leg", "dwell-E"):
                why = ", ".join("%s (%s)" % (e.label, e.status if e.status == "gone" else "moved") for e in active)
            if not why:
                return
            self.preempt_why = why
            self.interruptions += 1
            self._preempt.set()
            self.nav.preempt.set()                      # before stop_flag: the executor must see an interruption, not a STOP
            self.nav.stop_flag.set()
        # immediately after detection: stop the rover now (the mission thread follows up)
        try:
            self.nav._halt()
        except Exception:
            pass
        self._log("scene change %s: stopping the current %s" % (why, "navigation" if self.phase == "leg" else "collection"))

    def _quiesce(self):
        """After an interruption: the executor is gone, the rover halted, flags cleared. Cancellation still wins."""
        nav = self.nav
        try:
            nav._halt()
            nav.rover.stop()
        except Exception:
            pass
        t = nav.thread
        if t is not None and t.is_alive():
            t.join(8.0)
            if t.is_alive():
                self._terminal("the navigator did not stop after the scene-change interruption")
                return False
        with self.elock:
            if not self._cancelled():
                nav.stop_flag.clear()                   # first the STOP flag (while the interruption flags still explain it) ...
                if self._cancelled():                   # STOP/disarm raced with the clear: it wins
                    nav.stop_flag.set()
            nav.preempt.clear()                         # ... then the interruption flags
            self._preempt.clear()
        return not self._cancelled()

    def _reobserve(self):
        """Valid observations after a stop: a fresh full scene (map + events) and a settled, tracked, inside-the-boundary pose."""
        why = ""
        for _k in range(3):
            if self._cancelled():
                return False
            rec, why = self._refresh_scene()
            if rec is not None:
                pose, why = self._settled_pose()
                if pose is not None:
                    self.nav.pose = pose
                    return True
        if not self._cancelled():
            self._terminal("no valid observation after the scene-change interruption: " + why)
            self.state = "failed"
        return False

    def _after_preempt(self, item):
        self.countdown = None
        ok = self._quiesce()
        if self._cancelled():
            return
        if item.status in ("visiting", "collecting"):
            item.status, item.reason = "pending", "interrupted: " + self.preempt_why
            item.attempts = max(0, item.attempts - 1)       # an interruption is not a failed attempt
        item.interrupted += 1
        self.active = None
        self._log("%s: interrupted (%s): rover stopped, re-observing; not counted as collected" % (item.label, self.preempt_why))
        if ok:
            self._reobserve()

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
            mon = self.monitor
            self.cancel_reason = self.cancel_reason or (mon.fatal if mon is not None else "")
            nav.keep_armed = False
            self.active = self.countdown = None
            self.phase = "idle"
            if mon is not None:
                mon.stop()
            try:
                nav._halt()
                nav.rover.stop()
            except Exception:
                pass
            nav.armed = False                          # one authorisation: the mission never leaves the rover armed
            src = nav.src
            if hasattr(src, "hold_full"):
                src.hold_full = False
            if hasattr(src, "full_period"):
                src.full_period = getattr(src, "FULL_PERIOD_S", 8.0)
            bad = "failed" if self.state == "failed" else "cancelled"
            for s in self.book.items:
                if s.status in ("pending", "visiting", "collecting"):
                    s.status = bad
                    s.reason = s.reason or self.cancel_reason or self.message
            for e in self.events.items:
                if e.status in LIVE:
                    e.status = bad
                    e.reason = e.reason or self.cancel_reason or self.message
            if self.state == "running":
                self.state = "cancelled" if self.cancel_reason else "complete"
                self.message = self.cancel_reason or self._summary()
            elif not self.message:
                self.message = self.cancel_reason

    def _summary(self):
        n = lambda k: sum(1 for s in self.book.items if s.status == k)
        ne = lambda k: sum(1 for e in self.events.items if e.status == k)
        txt = "%d visited, %d unreachable, %d not visited" % (n("complete"), n("unreachable"), len([s for s in self.book.items if not s.gone]) - n("complete") - n("unreachable"))
        if self.events.items:
            txt += "; scene changes: %d inspected, %d unreachable, %d gone, %d interruptions" % (ne("complete"), ne("unreachable"), ne("gone"), self.interruptions)
        return txt

    def _pick(self):
        """Next piece of work: scene-change events first (oldest first, so a stream of changes cannot starve an older one), then
        structures (cheapest route first). Items with no safe reachable inspection position are reported, not retried here."""
        nav = self.nav
        nav.pose = self.per.last_obs.pose if self.per.last_obs and self.per.last_obs.pose else nav.pose
        if self.monitor is not None and self.monitor.age() > nav.cfg["map_max_age_s"] - 1.0:
            self.monitor.update_now()
        for ev in self.events.pending():
            if self._cancelled():
                return None
            pt, ln, why = self.plan_inspection(ev, ev.tried)
            if pt is None:
                ev.status, ev.reason = "unreachable", why
                self._log("%s unreachable: %s" % (ev.label, why))
                continue
            return ev, pt
        choice = None
        for s in [s for s in self.book.items if s.status == "pending" and not s.gone]:
            pt, ln, why = self.plan_inspection(s, s.tried)
            if pt is None:
                s.status, s.reason = "unreachable", why
                self._log("%s unreachable: %s" % (s.label, why))
                continue
            if choice is None or ln < choice[2]:
                choice = (s, pt, ln)
        return None if choice is None else (choice[0], choice[1])

    def _sequence(self):
        nav, mon = self.nav, self.monitor
        self._log("started: capturing the baseline scene (dwell %.0f s, gap %.0f mm, scene update every %.0f s)" % (self.dwell_s, self.gap_mm, mon.period))
        if hasattr(nav.src, "hold_full"):
            nav.src.hold_full = True                    # no idle throttling of tracking frames for the whole mission
        if hasattr(nav.src, "full_period"):
            nav.src.full_period = mon.period
        ok, why = mon.baseline()
        if not ok:
            self.state, self.message = "failed", "could not start: " + why
            return
        self.book.update(detect_structures(self.per, nav), self.per.calib, drop_after=1)    # a fresh scan is authoritative for pending ones
        self._log("baseline scene %s captured; %d structure(s): %s" % (mon.baseline_info["seq"], len(self.book.items), ", ".join(s.label for s in self.book.items) or "none"))
        mon.begin()
        while not self._cancelled():
            with self.elock:
                self.seen_gen = self.event_gen
            choice = self._pick()
            if self._cancelled():
                break
            if choice is None:
                if self._final_scan():
                    continue
                break
            self._work(*choice)
        self.active = None

    def _final_scan(self):
        """Before declaring the mission complete: one last scene update, so a change made during the final move is not missed.
        True if there is more to do."""
        rec, why = self._refresh_scene()
        if rec is None and not self._cancelled():
            self._log("final scene check failed: %s" % why)
            return False
        return self.event_gen != self.seen_gen or bool(self.events.pending())

    def _work(self, item, pt):
        nav = self.nav
        is_ev = isinstance(item, Event)
        item.status, item.inspect, item.attempts = "visiting", pt, item.attempts + 1
        self.active = item.label if is_ev else item.id
        self._log("%s: driving to its inspection position (%.3f, %.3f)" % (item.label, pt[0], pt[1]))
        res = self._leg(pt)
        if self._cancelled():
            return
        if res == "preempted":
            return self._after_preempt(item)
        if res != "done" and nav.soft_failed and nav.armed and not self._cancelled():
            # route-class failure (no route / no safe alternative): the rover is stopped, authorisation is intact.
            # Try ANOTHER inspection position near the SAME item; report it unreachable when they run out.
            self._retry_or_report(item, pt, "inspection position became inaccessible: %s" % nav.reason[:160])
            return
        if res != "done":
            self.state, self.message = "failed", "%s: navigation stopped: %s" % (item.label, nav.reason)
            item.status, item.reason = "failed", nav.reason
            self.cancel_reason = self.message
            self._log(self.message)
            return
        arr, why = self.verify_arrival(item, pt)
        if self._cancelled():
            return
        if self._preempt.is_set():
            return self._after_preempt(item)
        if arr is None:
            self._retry_or_report(item, pt, "arrival check failed: " + why)
            return
        self._log("%s: arrived, %.0f mm clear" % (item.label, arr["gap"]))
        why = self.dwell(item, arr)
        self.countdown = None
        if self._cancelled():
            return
        if why == "interrupted":
            return self._after_preempt(item)
        if why:
            self._retry_or_report(item, pt, "collection not counted: " + why)
            return
        item.status, item.visited_at, item.reason = "complete", time.time(), ""
        if is_ev and item.struct_id is not None:                     # the moved structure was just inspected at its new place
            st = self.book.get(item.struct_id)
            if st is not None and not st.gone:
                st.status, st.visited_at, st.reason = "complete", time.time(), "inspected after it moved (%s)" % item.label
        self.active = None
        self._log("%s: complete" % item.label)

    def _retry_or_report(self, st, pt, why):
        st.tried.append(pt)
        st.reason = why
        self._log("%s: %s" % (st.label, why))
        if st.status == "gone":
            self.active = None
            return
        if st.attempts >= MAX_ATTEMPTS:
            st.status = "unreachable"
            st.reason = "no verified safe inspection after %d attempts: %s" % (st.attempts, why)
        else:
            st.status = "pending"
        self.active = None
        self._refresh_scene()

    def _leg(self, target):
        """One guarded Navigator run to the inspection point; returns the Navigator's final state ('preempted' if a scene change
        stopped it, 'cancelled' on STOP/disarm)."""
        nav = self.nav
        with self.elock:
            if self._preempt.is_set() or self.event_gen != self.seen_gen:
                if not self._preempt.is_set():
                    self.interruptions += 1
                return "preempted"                      # something changed while this leg was being planned: plan again
            self.phase = "leg"
        try:
            ok, msg = nav.start(target)
            if not ok:
                self.cancel_reason = self.cancel_reason or msg
                return "refused"
            while True:
                t = nav.thread
                if t is not None:
                    t.join(0.2)
                if nav.state != "running" and (t is None or not t.is_alive()):
                    break
                if self._cancelled():
                    nav.stop()
                    if t is not None:
                        t.join(5.0)
                    return "cancelled"
                if self._preempt.is_set() and not nav.preempt.is_set():     # nav.start cleared the monitor's stop: re-assert
                    nav.preempt.set()
                    nav.stop_flag.set()
                    nav._halt()
            if self._preempt.is_set() or nav.preempt.is_set():
                return "preempted"
            return nav.state
        finally:
            with self.elock:
                self.phase = "idle"
