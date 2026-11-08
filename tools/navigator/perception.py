"""Turns a camera frame into: calibration (kept across STOP/restart), rover pose, and the obstacle map."""
import json
import math
import os
import threading
import time

import cv2
import numpy as np

import vision as V


class Obs:
    def __init__(self, frame, markers, pose, boundary_live):
        self.frame, self.markers, self.pose, self.boundary_live = frame, markers, pose, boundary_live
        self.seq = frame.seq


class Perception:
    def __init__(self, store_dir):
        self.dir = store_dir
        os.makedirs(store_dir, exist_ok=True)
        self.calib = V.Calibration()
        self.depth_map = V.ObstacleMap()
        self.manual_markers = {}      # id -> 4x2 corners for a boundary marker the detector cannot read (static)
        self.manual = []              # manual footprints, normalized (x0,y0,x1,y1) -- always honoured
        self.warnings = []
        self.want_ground_refit = False
        self.last_markers = {}
        self.last_obs = None
        self.map_frames = 0
        self.map_seq = -1            # sequence of the full frame the current obstacle map was built from
        self.map_time = 0.0          # when that frame was received
        self._mapper = None
        self.map_err = ""
        self._pattern_seen = False   # the rover's marker pattern has been read at least once (identity check enabled)
        self.map_frozen = False      # True during a run: the obstacle map built from the target snapshot is reused
        self.track = None            # (px, py, t): last tracked rover pixel, used by the frame source for its window
        self.stats = {"frames": 0, "partial": 0, "ms": 0.0}
        self.allow_blob = True      # depth-blob pose when the rover marker is unreadable (labelled in the UI)
        self.last_pose = None
        self._raw_hist = []
        self._recent = []            # recent rover poses (time, pose): the swept path is not an obstacle
        self.dims_hist = []
        self.aabb_hist = []          # recent world-aligned blob extents (wx, wy) in mm
          # recent (L, W) rover extents in mm measured from depth
        self.lock = threading.RLock()
        self._load()

    # --- persistence
    def _p(self, name):
        return os.path.join(self.dir, name)

    def _load(self):
        try:
            self.calib = V.Calibration.from_dict(json.load(open(self._p("calib.json"))))
        except Exception:
            pass
        try:
            self.depth_map.load(json.load(open(self._p("depthmap.json"))))
        except Exception:
            pass
        try:
            self.manual_markers = {int(k): np.array(v, np.float64) for k, v in json.load(open(self._p("manual_markers.json"))).items()}
        except Exception:
            pass
        try:
            self.manual = [tuple(r) for r in json.load(open(self._p("manual.json")))]
        except Exception:
            pass

    def save(self):
        json.dump(self.calib.to_dict(), open(self._p("calib.json"), "w"))
        json.dump(self.depth_map.to_dict(), open(self._p("depthmap.json"), "w"))
        json.dump([list(r) for r in self.manual], open(self._p("manual.json"), "w"))

    # --- obstacles for planning: depth-derived cells + manual footprints
    def obstacle_rects(self, near=None, self_r=0.13):
        """Planner obstacles. Cells within self_r (square-frame units) of the rover are its own body/shadow, not obstacles."""
        rects = self.depth_map.rects(near=near, aspect=self.calib.aspect)
        if near is not None:
            A = self.calib.aspect
            rects = [r for r in rects if math.hypot((((r[0] + r[2]) / 2) - near[0]) * A, ((r[1] + r[3]) / 2) - near[1]) > self_r]
        return rects + list(self.manual)

    def reset_heading(self):
        self._raw_hist = []

    def clear_depth_map(self):
        self.depth_map.clear()
        self.save()

    # --- main entry
    def process(self, frame, update_map=True):
        with self.lock:
            return self._process(frame, update_map)

    def snapshot(self, frame, repeats=3):
        """Obstacle map from ONE full frame (target selection): the EMA needs a few passes over the same frame to converge,
        so cells found/cleared by this snapshot settle without waiting for more frames. Rover cells are excluded."""
        with self.lock:
            obs = self._process(frame, True)
            if obs.pose is not None and self.calib.plane is not None and not getattr(frame, "partial", False):
                for _ in range(repeats - 1):
                    self.depth_map.update(frame.depth, self.calib, rover=[obs.pose], rover_exclude=self._rover_exclude())
            if not getattr(frame, "partial", False):
                self.map_seq = frame.seq
                self.map_time = getattr(frame, "rx", None) or time.time()
            return obs

    def refresh_map(self, frame, repeats=3):
        """Recompute obstacle occupancy from the NUMERICAL depth of one full frame, off the control thread. Works on a copy
        of the map and swaps it in atomically, only if no newer frame has been applied (an older frame never overwrites a
        newer map). Manual boxes (self.manual) and the validated calibration are untouched. Returns True if applied."""
        c = self.calib
        if getattr(frame, "partial", False) or c.plane is None or c.H is None or frame.depth is None:
            return False
        with self.lock:
            if frame.seq <= self.map_seq:
                return False
            base = self.depth_map
            new = V.ObstacleMap(base.cols, base.rows, base.h_min, base.h_max)
            new.ev, new.seen, new.height = base.ev.copy(), base.seen.copy(), base.height.copy()
            near = (self.last_pose["x"], self.last_pose["y"]) if self.last_pose else None
            recent = [q for _, q in self._recent]
            lastp = self.last_pose
        pose = V.blob_pose(frame.depth, c, near=near, known=base)       # the rover as it was in THIS frame
        if pose is not None and self._pattern_seen and V.pattern_heading(frame.bgr, frame.gray, c, frame.depth, pose["px"]) is None:
            pose = None                                                  # not confidently the rover: exclude nothing for it
        rovers = ([pose] if pose else []) + recent + ([lastp] if lastp else [])
        for _ in range(repeats):
            new.update(frame.depth, c, rover=rovers or None, rover_exclude=self._rover_exclude())
        with self.lock:
            if frame.seq <= self.map_seq:
                return False
            self.depth_map, self.map_seq = new, frame.seq
            self.map_time = getattr(frame, "rx", None) or time.time()
            self.map_frames += 1
        return True

    def refresh_async(self, frame):
        """Start refresh_map in a worker thread unless one is already running (single mapper, no queue build-up)."""
        if self._mapper is not None and self._mapper.is_alive():
            return False

        def work():
            try:
                self.refresh_map(frame)
                self.map_err = ""
            except Exception as e:
                self.map_err = "map refresh failed: %s" % e
        self._mapper = threading.Thread(target=work, daemon=True)
        self._mapper.start()
        return True

    def _process(self, frame, update_map):
        t_start = time.time()
        c = self.calib
        partial = bool(getattr(frame, "partial", False))
        if c.K is None or tuple(c.K) != tuple(frame.K):
            c.K = list(frame.K)
        if partial:                   # rover-region frame: no boundary markers / mapping, pose only
            markers, live = self.last_markers, bool(self.last_obs and self.last_obs.boundary_live)
            corners = None
        else:
            markers = V.detect_markers(frame.gray)
            for k, v in self.manual_markers.items():
                markers.setdefault(k, v)
            corners = V.boundary_corners(markers)
            live = corners is not None
        if live and not partial:
            if c.corners is None:
                c.set_corners(corners)
                self.save()
            elif c.drifted(corners):
                if self.manual_markers:                     # a static hand-placed marker is invalid once the camera moves
                    self.manual_markers = {}
                    try:
                        os.remove(self._p("manual_markers.json"))
                    except OSError:
                        pass
                self.warnings = ["camera or markers moved: boundary updated, ground plane will be refit, depth map cleared"]
                c.set_corners(corners)
                c.plane, c.aspect_measured, c.height_mm = None, False, None
                self.depth_map.clear()
                self.save()
        if c.corners is not None and not partial and (c.plane is None or self.want_ground_refit):
            self._fit_ground(frame)
        pose = V.rover_pose({} if partial else markers, c, frame.depth) if c.H is not None else None
        if pose is not None:
            pose["source"] = "marker"
        elif c.H is not None and c.plane is not None and self.allow_blob:
            near = (self.last_pose["x"], self.last_pose["y"]) if self.last_pose else None
            pose = V.blob_pose(frame.depth, c, near=near, known=self.depth_map)
            raw = V.pattern_heading(frame.bgr, frame.gray, c, frame.depth, pose["px"]) if pose is not None else None
            if raw is None and self._pattern_seen and (near is not None or self.depth_map.blocked_cells()):
                # identity check: the rover is the blob that carries the marker pattern. A latched or map-filtered blob
                # (cans, other objects) has none, so look for the rover globally, without the map filter
                alt = V.blob_pose(frame.depth, c, near=None, known=None)
                araw = V.pattern_heading(frame.bgr, frame.gray, c, frame.depth, alt["px"]) if alt is not None else None
                if araw is not None:
                    pose, raw = alt, araw
            if pose is not None:
                pose["verified"] = raw is not None
                if raw is not None:
                    self._pattern_seen = True
                    now = time.time()
                    hist = [(t, r) for t, r in self._raw_hist if now - t < 2.5][-2:]
                    if hist:
                        m = math.atan2(sum(math.sin(r) for _, r in hist), sum(math.cos(r) for _, r in hist))
                        if abs((raw - m + math.pi) % (2 * math.pi) - math.pi) > math.radians(8):
                            hist = []            # the rover is turning (or the reading jumped): no lagging average
                    self._raw_hist = hist + [(now, raw)]
                    raw = math.atan2(sum(math.sin(r) for _, r in self._raw_hist), sum(math.cos(r) for _, r in self._raw_hist))
                pose["theta_raw"] = raw
        # only a confidently identified rover is tracked / excluded from occupancy: a marker, a blob carrying the marker
        # pattern, or (when no pattern has ever been readable, e.g. IMU-heading rovers) the blob itself
        conf = pose is not None and (pose.get("source") == "marker" or pose.get("verified", True) or not self._pattern_seen)
        if conf:
            self.last_pose = pose
            if pose.get("source") == "depth-blob":
                self.aabb_hist = (self.aabb_hist + [tuple(pose["dims_mm"])])[-9:]
        do_map = update_map and not partial and not self.map_frozen
        if conf and pose.get("px") is not None:
            self.track = (float(pose["px"][0]), float(pose["px"][1]), time.time())
        if do_map and c.plane is not None and c.H is not None and frame.depth is not None:
            now = time.time()
            self._recent = [(t, q) for t, q in self._recent if now - t < 4.0][-4:]
            if conf:
                self._recent.append((now, pose))
            used = self.depth_map.update(frame.depth, c, rover=[q for _, q in self._recent] or None, rover_exclude=self._rover_exclude())
            self.map_frames += 1
        if do_map and pose is not None and pose.get("theta") is not None and c.plane is not None and frame.depth is not None:
            self._measure_rover(frame, pose)
        if not partial:
            self.last_markers = markers
        self.last_obs = Obs(frame, markers, pose, live)
        self.stats["frames"] += 1
        self.stats["partial"] += partial
        self.stats["ms"] += (time.time() - t_start) * 1000
        return self.last_obs

    def _fit_ground(self, frame):
        c = self.calib
        mask = np.zeros(frame.depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, c.corners.astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((9, 9), np.uint8)).astype(bool)
        plane = V.fit_ground_plane(frame.depth, c.K, mask)
        if plane is None:
            self.warnings = ["ground plane fit failed (not enough valid depth inside the boundary)"]
            return
        c.plane = plane
        self.depth_map.clear()
        self.want_ground_refit = False
        try:
            c.aspect, c.height_mm = V.measure_aspect(c.corners, c.K, plane)
            c.aspect_measured = True
        except Exception:
            pass
        self.warnings = []
        self.save()

    # --- rover size from depth: extents of the raised pixels around the rover along/across its heading
    def _measure_rover(self, frame, pose):
        c = self.calib
        if not c.height_mm:
            return
        near = any(math.hypot(((cc + .5) / self.depth_map.cols - pose["x"]) * c.aspect, (rr + .5) / self.depth_map.rows - pose["y"]) < 0.26
                   for cc, rr in self.depth_map.blocked_cells())
        if near:
            return                      # something else is close: do not contaminate the measurement
        K, (n, d) = c.K, c.plane
        ys, xs = np.mgrid[0:frame.depth.shape[0]:1, 0:frame.depth.shape[1]:1]
        ys, xs = ys.ravel(), xs.ravel()
        z = frame.depth[ys, xs].astype(np.float64)
        ok = z > 0
        ys, xs, z = ys[ok], xs[ok], z[ok]
        fx, fy, cx, cy = K
        Pt = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)
        h = d - Pt @ n
        keep = h > 35.0
        if keep.sum() < 50:
            return
        Pf = Pt[keep] + n[None, :] * h[keep][:, None]
        fpx = np.stack([fx * Pf[:, 0] / Pf[:, 2] + cx, fy * Pf[:, 1] / Pf[:, 2] + cy], 1)
        nr = c.to_norm(fpx)
        du = (nr[:, 0] - pose["x"]) * c.aspect * c.height_mm
        dv = (nr[:, 1] - pose["y"]) * c.height_mm
        r = np.hypot(du, dv)
        sel = r < 0.22 * c.height_mm
        if sel.sum() < 50:
            return
        du, dv = du[sel], dv[sel]
        th = pose["theta"]
        along = math.cos(th) * du + math.sin(th) * dv
        across = -math.sin(th) * du + math.cos(th) * dv
        L = float(np.percentile(along, 99) - np.percentile(along, 1))
        W = float(np.percentile(across, 99) - np.percentile(across, 1))
        if 80 <= L <= 400 and 80 <= W <= 400:
            self.dims_hist = (self.dims_hist + [(L, W)])[-9:]

    def _rover_exclude(self):
        """Radius (height units) around the rover marker whose raised pixels are the rover itself, not obstacles."""
        dims, hm = self.rover_dims_mm(), self.calib.height_mm
        if dims and hm:
            return math.hypot(*dims) / 2 / hm + 0.015
        return 0.22

    def rover_aabb_mm(self):
        if len(self.aabb_hist) < 3:
            return None
        a = np.array(self.aabb_hist)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1]))

    def rover_dims_mm(self):
        if len(self.dims_hist) < 3:
            return None
        a = np.array(self.dims_hist)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1]))
