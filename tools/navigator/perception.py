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
        self.allow_blob = True      # depth-blob pose when the rover marker is unreadable (labelled in the UI)
        self.last_pose = None
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
    def obstacle_rects(self, near=None):
        return self.depth_map.rects(near=near, aspect=self.calib.aspect) + list(self.manual)

    def clear_depth_map(self):
        self.depth_map.clear()
        self.save()

    # --- main entry
    def process(self, frame, update_map=True):
        with self.lock:
            return self._process(frame, update_map)

    def _process(self, frame, update_map):
        c = self.calib
        if c.K is None or tuple(c.K) != tuple(frame.K):
            c.K = list(frame.K)
        markers = V.detect_markers(frame.gray)
        for k, v in self.manual_markers.items():
            markers.setdefault(k, v)
        corners = V.boundary_corners(markers)
        live = corners is not None
        if live:
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
        if c.corners is not None and (c.plane is None or self.want_ground_refit):
            self._fit_ground(frame)
        pose = V.rover_pose(markers, c, frame.depth) if c.H is not None else None
        if pose is not None:
            pose["source"] = "marker"
        elif c.H is not None and c.plane is not None and self.allow_blob:
            near = (self.last_pose["x"], self.last_pose["y"]) if self.last_pose else None
            pose = V.blob_pose(frame.depth, c, near=near)
        if pose is not None:
            self.last_pose = pose
            if pose.get("source") == "depth-blob":
                self.aabb_hist = (self.aabb_hist + [tuple(pose["dims_mm"])])[-9:]
        if update_map and c.plane is not None and c.H is not None and frame.depth is not None:
            used = self.depth_map.update(frame.depth, c, rover=pose, rover_exclude=self._rover_exclude())
            self.map_frames += 1
        if pose is not None and pose.get("theta") is not None and c.plane is not None and frame.depth is not None:
            self._measure_rover(frame, pose)
        self.last_markers = markers
        self.last_obs = Obs(frame, markers, pose, live)
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
            return math.hypot(*dims) / 2 / hm + 0.03
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
