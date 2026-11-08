"""Overhead vision: ArUco boundary calibration, rover pose + heading, depth ground plane and obstacle map.

Frames come from the Aurora930 (aligned RGB NV12 + 16-bit depth in millimetres, 0 = invalid).
Boundary markers: DICT_4X4_50 ids BL=0, TL=1, BR=2, TR=3. Rover marker: id 4 (its printed 'up' edge is the rover front,
any offset is absorbed by the learned motion model).
"""
import math

import cv2
import numpy as np

from geom import wrap

ID_BL, ID_TL, ID_BR, ID_TR = 0, 1, 2, 3
ID_ROVER = 5
HIT_FRAC = 0.4          # share of a cell's valid depth samples that must be raised for the cell to count as an obstacle
HOLE_FRAC = 0.30        # a map cell with at least this share of invalid-depth pixels has no usable depth
CORR_STEP = 16          # px per cell of the tabletop-ripple correction grid
H_MIN_MM = 35.0         # obstacle height threshold above the tabletop (unchanged)
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_DET = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())


def detect_markers(gray):
    c, i, _ = _DET.detectMarkers(gray)
    out = {}
    if i is not None:
        for cc, ii in zip(c, i.flatten()):
            out[int(ii)] = cc.reshape(4, 2).astype(np.float64)
    return out


def boundary_corners(markers):
    """Image points of the arena corners in BL,BR,TR,TL order from the four boundary markers (ids 0-3, any layout).
    Roles come from the image geometry (which marker is bottom-left, ...), so the flattened view is upright with
    respect to the camera image and "left-centre" means image left. Corner convention (verified on the live tabletop,
    where the markers sit at the left and right ends of the table): from each marker take the black-square corner
    that is INNER along the sandbox x axis and OUTER along y, i.e. the arena spans between the left and right marker
    columns and from the outer top edge to the outer bottom edge of the markers (the table edge).
    None unless exactly four boundary markers form a convex quad."""
    ids = [k for k in (ID_BL, ID_TL, ID_BR, ID_TR) if k in markers]
    if len(ids) != 4:
        return None
    ctr = {k: markers[k].mean(0) for k in ids}
    c0 = np.mean(list(ctr.values()), 0)
    role = {}
    for k in ids:
        left, top = ctr[k][0] < c0[0], ctr[k][1] < c0[1]       # image y grows downward
        role[("T" if top else "B") + ("L" if left else "R")] = k
    if len(role) != 4:
        return None
    ex = (ctr[role["BR"]] - ctr[role["BL"]] + ctr[role["TR"]] - ctr[role["TL"]]) / 2
    ey = (ctr[role["TL"]] - ctr[role["BL"]] + ctr[role["TR"]] - ctr[role["BR"]]) / 2
    ex, ey = ex / np.linalg.norm(ex), ey / np.linalg.norm(ey)
    signs = {"BL": (1, -1), "BR": (-1, -1), "TR": (-1, 1), "TL": (1, 1)}   # (inner-x, outer-y)
    out = []
    for r in ("BL", "BR", "TR", "TL"):
        m = markers[role[r]]
        sx, sy = signs[r]
        out.append(m[int(np.argmax(m @ (sx * ex) + m @ (sy * ey)))])
    q = np.array(out, np.float64)
    cr = [np.cross(q[(i + 1) % 4] - q[i], q[(i + 2) % 4] - q[(i + 1) % 4]) for i in range(4)]
    if not (all(c > 0 for c in cr) or all(c < 0 for c in cr)):
        return None
    return q


class Calibration:
    """Image -> normalized sandbox homography plus ground plane, aspect and intrinsics. Kept across STOP/restart."""

    def __init__(self):
        self.corners = None      # 4x2 image points BL,BR,TR,TL
        self.H = None            # image px -> normalized (x,y)
        self.Hinv = None
        self.plane = None        # (n[3], d) with n pointing away from the camera, height = d - n.p
        self.aspect = 1.5        # sandbox width/height; replaced by the measured value when the plane is known
        self.aspect_measured = False
        self.height_mm = None    # sandbox height in millimetres (from the depth plane), for mm-based rover sizes
        self.K = None            # (fx, fy, cx, cy) of the aligned RGB/depth image
        self.corr = None         # coarse image-space grid (mm) of the FIXED tabletop deviation from the plane (depth-sensor ripple)
        self.corr_step = CORR_STEP
        self._corr_full = None

    def set_corr(self, grid):
        self.corr = None if grid is None else np.asarray(grid, np.float32)
        self._corr_full = None

    def corr_full(self, shape):
        """Per-pixel tabletop deviation (mm) for a depth image of `shape`; zeros when no correction has been fitted."""
        if self.corr is None:
            return np.zeros(shape, np.float32)
        if self._corr_full is None or self._corr_full.shape != tuple(shape):
            s = self.corr_step
            big = cv2.resize(self.corr, (self.corr.shape[1] * s, self.corr.shape[0] * s), interpolation=cv2.INTER_LINEAR)
            self._corr_full = np.ascontiguousarray(big[:shape[0], :shape[1]], np.float32)
        return self._corr_full

    def set_corners(self, corners):
        dst = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float64)
        self.corners = np.asarray(corners, np.float64)
        self.H = cv2.getPerspectiveTransform(self.corners.astype(np.float32), dst.astype(np.float32)).astype(np.float64)
        self.Hinv = np.linalg.inv(self.H)

    def to_norm(self, px):
        p = np.asarray(px, np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.H).reshape(-1, 2)

    def to_px(self, xy):
        p = np.asarray(xy, np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.Hinv).reshape(-1, 2)

    def drifted(self, corners, tol_px=6.0):
        return self.corners is not None and float(np.max(np.linalg.norm(self.corners - np.asarray(corners), axis=1))) > tol_px

    def to_dict(self):
        return {"corners": None if self.corners is None else self.corners.tolist(),
                "plane": None if self.plane is None else [list(map(float, self.plane[0])), float(self.plane[1])],
                "aspect": self.aspect, "aspect_measured": self.aspect_measured, "height_mm": self.height_mm, "K": self.K,
                "corr": None if self.corr is None else {"step": self.corr_step, "data": np.round(self.corr, 1).tolist()}}

    @classmethod
    def from_dict(cls, d):
        c = cls()
        if d.get("corners"):
            c.set_corners(d["corners"])
        if d.get("plane"):
            c.plane = (np.array(d["plane"][0], np.float64), float(d["plane"][1]))
        c.aspect = d.get("aspect", 1.5)
        c.aspect_measured = d.get("aspect_measured", False)
        c.height_mm = d.get("height_mm")
        c.K = d.get("K")
        if d.get("corr"):
            c.corr_step = int(d["corr"].get("step", CORR_STEP))
            c.set_corr(d["corr"]["data"])
        return c


def nv12_to_bgr(buf, cols, rows):
    yuv = np.frombuffer(buf, np.uint8, cols * rows * 3 // 2).reshape(rows * 3 // 2, cols)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def gray_of(buf, cols, rows):
    return np.frombuffer(buf, np.uint8, cols * rows).reshape(rows, cols)


def foot_px(pts_px, depth, K, plane, win=2):
    """Pixels where the points' vertical projections onto the ground plane appear. Removes parallax for anything
    raised above the floor (rover marker, obstacle tops). Falls back to the input pixel when depth is invalid."""
    out = []
    fx, fy, cx, cy = K
    n, d = plane
    H, W = depth.shape
    for x, y in pts_px:
        xi, yi = int(round(x)), int(round(y))
        z = 0.0
        if 0 <= xi < W and 0 <= yi < H:
            w = depth[max(0, yi - win):yi + win + 1, max(0, xi - win):xi + win + 1]
            w = w[w > 0]
            z = float(np.median(w)) if w.size >= 3 else 0.0
        if z <= 0:
            out.append((x, y))
            continue
        P = np.array([(x - cx) * z / fx, (y - cy) * z / fy, z])
        Pf = P + n * (d - n @ P)
        out.append((fx * Pf[0] / Pf[2] + cx, fy * Pf[1] / Pf[2] + cy))
    return np.array(out, np.float64)


def foot_at_height(px, K, plane, h_mm):
    """Ground-plane footprint pixel of the image point `px` seen on a surface h_mm above the plane (no depth needed)."""
    fx, fy, cx, cy = K
    n, d = plane
    ray = np.array([(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0])
    P = ray * ((d - h_mm) / (n @ ray))
    Pf = P + n * h_mm
    return np.array([fx * Pf[0] / Pf[2] + cx, fy * Pf[1] / Pf[2] + cy])


def rover_pose(markers, calib, depth=None):
    """Rover (x, y) normalized and heading (radians, CCW in the square frame, 0 = +x) from marker ID_ROVER."""
    m = markers.get(ID_ROVER)
    if m is None or calib.H is None:
        return None
    ctr = m.mean(0)
    front = (m[0] + m[1]) / 2           # marker 'up' edge midpoint
    pts = np.array([ctr, front])
    if depth is not None and calib.plane is not None and calib.K is not None:
        pts = foot_px(pts, depth, calib.K, calib.plane)   # ground footprint below the (raised) marker
    a, b = calib.to_norm(pts)
    du, dv = (b[0] - a[0]) * calib.aspect, b[1] - a[1]
    if math.hypot(du, dv) < 1e-6:
        return None
    return {"x": float(a[0]), "y": float(a[1]), "theta": float(math.atan2(dv, du)),
            "px": ctr.tolist(), "inside": bool(-0.02 <= a[0] <= 1.02 and -0.02 <= a[1] <= 1.02)}


# ---------------------------------------------------------------- depth ----------------------------------------
def backproject(depth, K, ys, xs):
    z = depth[ys, xs].astype(np.float64)
    fx, fy, cx, cy = K
    return np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)


def fit_ground_plane(depth, K, mask, iters=200, thr_mm=15.0, rng=None):
    """RANSAC plane through valid depth inside mask. Returns (n, d) with n.p = d for floor points, n pointing away
    from the camera (so height above floor = d - n.p), or None."""
    rng = rng or np.random.default_rng(1)
    ys, xs = np.nonzero((depth > 0) & mask)
    if len(ys) < 500:
        return None
    sel = rng.choice(len(ys), min(4000, len(ys)), replace=False)
    P = backproject(depth, K, ys[sel], xs[sel])
    best, best_n = None, 0
    for _ in range(iters):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n)
        if nn < 1e-6:
            continue
        n /= nn
        d = n @ a
        cnt = int(np.sum(np.abs(P @ n - d) < thr_mm))
        if cnt > best_n:
            best_n, best = cnt, (n, d)
    if best is None or best_n < 0.3 * len(P):
        return None
    n, d = best
    inl = P[np.abs(P @ n - d) < thr_mm]
    cen = inl.mean(0)
    _, _, vt = np.linalg.svd(inl - cen, full_matrices=False)
    n = vt[2]
    d = float(n @ cen)
    if d < 0:
        n, d = -n, -d
    return n, d


def plane_residual(depth, K, plane):
    """Signed height (mm) of every pixel above `plane` (nan where depth is invalid)."""
    n, d = plane
    H, W = depth.shape
    ys, xs = np.mgrid[0:H, 0:W]
    z = depth.astype(np.float64)
    h = d - (((xs - K[2]) * z / K[0]) * n[0] + ((ys - K[3]) * z / K[1]) * n[1] + z * n[2])
    h[depth == 0] = np.nan
    return h


def fit_ground_surface(depth, K, mask, plane=None, step=CORR_STEP, hi=28.0, lo=-70.0):
    """Tabletop model = plane + smooth fixed-pattern correction. The depth sensor has a stable (frame-to-frame r=0.98)
    +-35 mm ripple over the table, as large as the obstacle threshold, so a plane alone mislabels clear table as raised and
    flattens low objects. Fit: seed plane (RANSAC), then repeatedly re-fit it on tabletop pixels only (residual inside
    [lo, hi] and away from invalid/raised pixels, so objects and the rover never enter the model), then take a block median
    of the remaining residual on a coarse grid, fill empty blocks by diffusion, smooth. Returns (plane, grid_mm) or None."""
    if plane is None:
        plane = fit_ground_plane(depth, K, mask)
    if plane is None:
        return None
    ok = None
    for _ in range(3):
        h = plane_residual(depth, K, plane)
        bad = ~np.isfinite(h) | (h > hi) | (h < lo)
        bad = cv2.dilate(bad.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
        ok = mask & ~bad
        ys, xs = np.nonzero(ok)
        if len(ys) < 2000:
            return None
        z = depth[ys, xs].astype(np.float64)
        P = np.stack([(xs - K[2]) * z / K[0], (ys - K[3]) * z / K[1], z], 1)
        cen = P.mean(0)
        n = np.linalg.svd(P - cen, full_matrices=False)[2][2]
        if n[2] < 0:
            n = -n
        plane = (n, float(n @ cen))
    h = plane_residual(depth, K, plane)
    Hh, Ww = depth.shape
    gh, gw = -(-Hh // step), -(-Ww // step)
    grid = np.full((gh, gw), np.nan, np.float32)
    for gy in range(gh):
        for gx in range(gw):
            sl = (slice(gy * step, (gy + 1) * step), slice(gx * step, (gx + 1) * step))
            m = ok[sl] & np.isfinite(h[sl])
            if m.sum() >= 40:
                grid[gy, gx] = np.median(h[sl][m])
    known = np.isfinite(grid)
    if known.sum() < 0.15 * grid.size:
        return None
    v0 = np.where(known, grid, 0).astype(np.float32)
    v, w = v0.copy(), known.astype(np.float32)
    k3 = np.ones((3, 3), np.float32) / 9
    for _ in range(80):                                   # normalized-convolution diffusion into unmeasured blocks
        vs, ws = cv2.filter2D(v * w, -1, k3), cv2.filter2D(w, -1, k3)
        new = np.where(ws > 1e-6, vs / np.maximum(ws, 1e-6), 0)
        v = np.where(known, v0, new)
        w = np.maximum(known.astype(np.float32), (ws > 1e-6).astype(np.float32))
    grid = np.clip(cv2.GaussianBlur(v, (0, 0), 1.0), -50.0, 50.0)
    return plane, grid


def ray_plane_point(px, K, plane):
    fx, fy, cx, cy = K
    n, d = plane
    ray = np.array([(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0])
    t = d / (n @ ray)
    return ray * t


def measure_aspect(corners, K, plane):
    P = [ray_plane_point(c, K, plane) for c in corners]  # BL,BR,TR,TL
    w = (np.linalg.norm(P[1] - P[0]) + np.linalg.norm(P[2] - P[3])) / 2
    h = (np.linalg.norm(P[3] - P[0]) + np.linalg.norm(P[2] - P[1])) / 2
    return float(w / h), float(h)  # aspect, sandbox height in mm


class ObstacleMap:
    """Persistent obstacle evidence on a fine grid over the normalized sandbox, from depth height above the plane."""

    def __init__(self, cols=72, rows=48, h_min=H_MIN_MM, h_max=900.0):
        self.cols, self.rows, self.h_min, self.h_max = cols, rows, h_min, h_max
        self.ev = np.zeros((rows, cols), np.float32)   # 0 free .. 1 obstacle
        self.seen = np.zeros((rows, cols), bool)
        self.height = np.zeros((rows, cols), np.float32)
        self.holey = np.zeros((rows, cols), bool)      # cells that had mostly invalid depth in the last update

    def update(self, depth, calib, rover=None, rover_exclude=0.12, stride=2):
        """Fold one depth frame into the map. Returns number of obstacle pixels used."""
        if calib.plane is None or calib.H is None or calib.K is None:
            return 0
        K, (n, d) = calib.K, calib.plane
        H, W = depth.shape
        ys, xs = np.mgrid[0:H:stride, 0:W:stride]
        ys, xs = ys.ravel(), xs.ravel()
        z = depth[ys, xs].astype(np.float64)
        ok = z > 0
        # pixels WITHOUT depth (dark/shiny objects return none) are counted per map cell through the ground homography:
        # a cell that is mostly holes has no usable depth and must never be reported as free
        hole_cnt = np.zeros((self.rows, self.cols), int)
        if (~ok).any() and calib.H is not None:
            hn = calib.to_norm(np.stack([xs[~ok], ys[~ok]], 1).astype(np.float64))
            hin = (hn[:, 0] >= 0) & (hn[:, 0] < 1) & (hn[:, 1] >= 0) & (hn[:, 1] < 1)
            hc = np.minimum((hn[hin, 0] * self.cols).astype(int), self.cols - 1)
            hr = np.minimum((hn[hin, 1] * self.rows).astype(int), self.rows - 1)
            hole_cnt = np.bincount(hr * self.cols + hc, minlength=self.rows * self.cols).reshape(self.rows, self.cols)
        ys, xs, z = ys[ok], xs[ok], z[ok]
        if len(z) == 0:
            return 0
        fx, fy, cx, cy = K
        P = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1)
        h = d - P @ n - calib.corr_full(depth.shape)[ys, xs]      # height above the tabletop (plane + fixed ripple correction)
        Pf = P + n[None, :] * h[:, None]                      # drop each point vertically onto the floor plane
        fpx = np.stack([fx * Pf[:, 0] / Pf[:, 2] + cx, fy * Pf[:, 1] / Pf[:, 2] + cy], 1)
        nrm = calib.to_norm(fpx)
        inside = (nrm[:, 0] >= 0) & (nrm[:, 0] < 1) & (nrm[:, 1] >= 0) & (nrm[:, 1] < 1)
        in_rover = np.zeros(len(h), bool)
        rovers = [] if rover is None else (rover if isinstance(rover, list) else [rover])
        for rv in rovers:      # the rover (now and where it was a moment ago) is not an obstacle
            du = (nrm[:, 0] - rv["x"]) * calib.aspect
            dv = nrm[:, 1] - rv["y"]
            in_rover |= np.hypot(du, dv) <= rover_exclude
        ci = np.minimum((nrm[:, 0] * self.cols).astype(int), self.cols - 1)
        ri = np.minimum((nrm[:, 1] * self.rows).astype(int), self.rows - 1)
        flat = ri * self.cols + ci
        valid_cnt = np.bincount(flat[inside], minlength=self.rows * self.cols).reshape(self.rows, self.cols)
        hit = inside & ~in_rover & (h > self.h_min) & (h < self.h_max)
        hit_cnt = np.bincount(flat[hit], minlength=self.rows * self.cols).reshape(self.rows, self.cols)
        hsum = np.bincount(flat[hit], weights=h[hit], minlength=self.rows * self.cols).reshape(self.rows, self.cols)
        observed = valid_cnt >= 3
        # judged over a 3x3 cell neighbourhood (a cell holds only ~6 depth samples): isolated dropouts do not count, an
        # object-sized hole region does
        k3 = np.ones((3, 3), np.float32)
        hs = cv2.filter2D(hole_cnt.astype(np.float32), -1, k3, borderType=cv2.BORDER_CONSTANT)
        ts = cv2.filter2D((hole_cnt + valid_cnt).astype(np.float32), -1, k3, borderType=cv2.BORDER_CONSTANT)
        holey = (hole_cnt >= 1) & (hs >= 4) & (hs >= HOLE_FRAC * np.maximum(ts, 1))
        observed &= ~holey
        obstacle = (hit_cnt >= 3) & (hit_cnt >= HIT_FRAC * np.maximum(valid_cnt, 1))
        obstacle = cv2.morphologyEx(obstacle.astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8)).astype(bool)
        self.ev[observed] = 0.6 * self.ev[observed] + 0.4 * obstacle[observed]
        if rovers:
            cc, rr = np.meshgrid(np.arange(self.cols), np.arange(self.rows))
            for rv in rovers:                      # cells under the rover cannot be obstacles: clear stale evidence
                zone = np.hypot(((cc + .5) / self.cols - rv["x"]) * calib.aspect, (rr + .5) / self.rows - rv["y"]) <= rover_exclude * 0.8
                self.ev[zone & ~obstacle] *= 0.3
        self.seen |= observed
        self.seen &= ~holey                       # no usable depth now: unknown (planner: blocked), not free
        self.holey = holey
        with np.errstate(invalid="ignore", divide="ignore"):
            hm = np.where(hit_cnt > 0, hsum / np.maximum(hit_cnt, 1), 0)
        self.height[obstacle] = hm[obstacle]
        return int(hit.sum())

    def blocked_cells(self, thr=0.5):
        rr, cc = np.nonzero(self.ev >= thr)
        return list(zip(cc.tolist(), rr.tolist()))

    def blocked_at(self, nr, thr=0.5, grow=1):
        """Vectorised: are the normalized points (N,2) inside a mapped obstacle cell (grown by `grow` cells)?"""
        ci = np.clip((nr[:, 0] * self.cols).astype(int), 0, self.cols - 1)
        ri = np.clip((nr[:, 1] * self.rows).astype(int), 0, self.rows - 1)
        b = (self.ev >= thr).astype(np.uint8)
        if grow:
            b = cv2.dilate(b, np.ones((2 * grow + 1, 2 * grow + 1), np.uint8))
        return b[ri, ci].astype(bool)

    def unknown_cells(self):
        """Cells never observed with valid depth: unknown is NOT free space."""
        rr, cc = np.nonzero(~self.seen)
        return list(zip(cc.tolist(), rr.tolist()))

    def rects(self, thr=0.5, unknown=True, near=None, near_r=0.2, aspect=1.5):
        """Blocked (and unknown) fine cells as normalized rects (x0,y0,x1,y1) for the planner. Unknown cells within
        near_r (square-frame units) of `near` (the rover) are its own occlusion shadow and are not counted."""
        unk = self.unknown_cells() if unknown else []
        if near is not None and unk:
            unk = [(c, r) for c, r in unk
                   if math.hypot(((c + .5) / self.cols - near[0]) * aspect, (r + .5) / self.rows - near[1]) > near_r]
        cells = self.blocked_cells(thr) + unk
        return [(c / self.cols, r / self.rows, (c + 1) / self.cols, (r + 1) / self.rows) for c, r in cells]

    def to_dict(self):
        return {"cols": self.cols, "rows": self.rows, "cells": [[c, r, round(float(self.ev[r, c]), 2)] for c, r in self.blocked_cells(0.2)],
                "seen": [[int(c), int(r)] for r, c in zip(*np.nonzero(self.seen))], "heights": {"%d,%d" % (c, r): round(float(self.height[r, c]), 0) for c, r in self.blocked_cells(0.5)}}

    def load(self, d):
        if not d or d.get("cols") != self.cols or d.get("rows") != self.rows:
            return
        self.ev[:] = 0
        for c, r, v in d.get("cells", []):
            self.ev[r, c] = v
        self.seen[:] = False
        for c, r in d.get("seen", []):
            self.seen[r, c] = True
        for k, v in d.get("heights", {}).items():
            c, r = map(int, k.split(","))
            self.height[r, c] = v

    def clear(self):
        self.ev[:] = 0
        self.seen[:] = False
        self.height[:] = 0


def flatten_view(bgr, calib, out_w=720):
    """Top-down (flattened) colour view of the sandbox, aspect-corrected."""
    out_h = max(2, int(round(out_w / calib.aspect)))
    dst = np.array([[0, out_h], [out_w, out_h], [out_w, 0], [0, 0]], np.float32)  # BL,BR,TR,TL with y up -> image rows down
    M = cv2.getPerspectiveTransform(calib.corners.astype(np.float32), dst)
    return cv2.warpPerspective(bgr, M, (out_w, out_h), flags=cv2.INTER_LINEAR)


def blob_pose(depth, calib, near=None, near_r=0.3, expect_mm=(200.0, 170.0), known=None, dbg=None):
    """Position-only rover pose from depth: the raised blob (40..135 mm above the floor plane) of rover size, not
    touching taller obstacles. Used when the rover marker is not readable. Returns pose dict (theta None) with the
    world-aligned footprint extents in mm, or None. `near` (x, y) restricts to the blob closest to the last pose."""
    if calib.plane is None or calib.H is None or calib.K is None or calib.height_mm is None:
        return None
    fx, fy, cx, cy = calib.K
    n, d = calib.plane
    Hh, Ww = depth.shape
    ys, xs = np.mgrid[0:Hh, 0:Ww]
    z = depth.astype(np.float32)
    X, Y = (xs - cx) * z / fx, (ys - cy) * z / fy
    h = d - (X * n[0] + Y * n[1] + z * n[2]) - calib.corr_full(depth.shape)
    valid = z > 0
    quad = np.zeros((Hh, Ww), np.uint8)
    cv2.fillConvexPoly(quad, calib.corners.astype(np.int32), 1)
    quad = cv2.dilate(quad, np.ones((41, 41), np.uint8)).astype(bool)      # the rover may overhang the boundary a little
    mid = valid & quad & (h > 35) & (h < 175)
    real = mid.copy()                                  # pixels with real depth (closing below may fill holes)
    tall = cv2.dilate((valid & (h >= 175)).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    mid = cv2.morphologyEx(mid.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mid = cv2.morphologyEx(mid, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))   # merge split halves
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(mid, connectivity=8)
    best, best_score = None, 1e9
    for i in range(1, nlab):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 300:
            continue
        m = lab == i
        if (m & tall).sum() > 0.20 * area:
            if dbg is not None: dbg.append(("tall", int(area), int((m & tall).sum())))
            continue                                   # a fringe of a taller obstacle, not the rover
        m = m & real
        if m.sum() < 150:
            continue
        py, px = np.nonzero(m)
        P = np.stack([X[py, px], Y[py, px], z[py, px]], 1).astype(np.float64)
        hh = h[py, px].astype(np.float64)
        Pf = P + n[None, :] * hh[:, None]
        fpx = np.stack([fx * Pf[:, 0] / Pf[:, 2] + cx, fy * Pf[:, 1] / Pf[:, 2] + cy], 1)
        nr = calib.to_norm(fpx)
        if known is not None:          # drop pixels that belong to already-mapped obstacles touching the rover's blob
            keep = ~known.blocked_at(nr)
            if 150 <= keep.sum() and keep.mean() > 0.4:      # if most of the blob is "known obstacle", it is the rover itself
                nr, hh = nr[keep], hh[keep]
                px, py = px[keep], py[keep]
        u = nr[:, 0] * calib.aspect * calib.height_mm
        v = nr[:, 1] * calib.height_mm
        wx = float(np.percentile(u, 93) - np.percentile(u, 7)) * 1.15      # robust to flying-pixel outliers
        wy = float(np.percentile(v, 93) - np.percentile(v, 7)) * 1.15
        if not (80 <= wx <= 340 and 80 <= wy <= 340):
            if dbg is not None: dbg.append(("dims", int(area), round(wx), round(wy)))
            continue
        cxn, cyn = float(np.median(nr[:, 0])), float(np.median(nr[:, 1]))
        if near is not None:
            dist = math.hypot((cxn - near[0]) * calib.aspect, cyn - near[1])
            if dist > near_r:
                if dbg is not None: dbg.append(("far", int(area), round(dist, 2)))
                continue
            score = dist
        else:
            score = abs(wx - expect_mm[0]) + abs(wy - expect_mm[1]) + abs(float(np.median(hh)) - 80)
        if score < best_score:
            best_score = score
            best = {"x": cxn, "y": cyn, "theta": None, "source": "depth-blob", "dims_mm": [wx, wy],
                    "px": [float(px.mean()), float(py.mean())], "inside": True, "height_mm": float(np.median(hh))}
    return best


def _pattern_ex(bgr, gray, calib, depth, centre_px, half=34):
    """Rover heading from the (too small to decode) marker: its bright pattern sits at the rover FRONT on the dark body,
    so heading = direction from the body centre to the pattern centroid. Returns heading in the square frame
    (radians, CCW, 0 = +x), or None. The marker/body offset is only a few pixels, so the caller anchors it with
    the camera-measured direction of travel. Returns (heading, dark_body_centroid_px); the dark body (wheels included) is
    a far steadier position than the depth blob, whose fragments move when the dark body returns no depth."""
    if calib.H is None:
        return None, None
    H, W = gray.shape
    cx, cy = int(round(centre_px[0])), int(round(centre_px[1]))
    x0, x1, y0, y1 = max(0, cx - half), min(W, cx + half), max(0, cy - half), min(H, cy + half)
    g = gray[y0:y1, x0:x1]
    b = bgr[y0:y1, x0:x1].astype(np.int16)
    dark = (g < 80).astype(np.uint8)
    n, lab, st, ce = cv2.connectedComponentsWithStats(dark, connectivity=8)
    if n < 2:
        return None, None
    k = 1 + int(np.argmax([st[i, cv2.CC_STAT_AREA] - 0.5 * np.hypot(*(ce[i] - [cx - x0, cy - y0])) * 10 for i in range(1, n)]))
    if st[k, cv2.CC_STAT_AREA] < 400:
        return None, None
    body = (lab == k).astype(np.uint8)
    cnts, _ = cv2.findContours(body, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(body)
    cv2.drawContours(filled, cnts, -1, 1, -1)
    inner = cv2.erode(filled, np.ones((5, 5), np.uint8)).astype(bool)
    bright = inner & (~body.astype(bool)) & (b.min(axis=2) > 105)
    if bright.sum() < 6:
        return None, None
    # the marker pattern is the largest bright cluster; ignore stray highlights (LEDs, reflections)
    grown = cv2.dilate(bright.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=2)
    nb, lb, sb, _ = cv2.connectedComponentsWithStats(grown, connectivity=8)
    if nb < 2:
        return None, None
    kb = 1 + int(np.argmax(sb[1:, cv2.CC_STAT_AREA]))
    bright = bright & (lb == kb)
    by, bx = np.nonzero(bright)
    if len(by) < 4:
        return None, None
    m = np.array([bx.mean() + x0, by.mean() + y0])
    M = cv2.moments(filled)
    c = np.array([M["m10"] / M["m00"] + x0, M["m01"] / M["m00"] + y0])
    if np.hypot(*(m - c)) < 2.0:
        return None, None
    pts = np.array([c, m])
    if depth is not None and calib.plane is not None and calib.K is not None:
        pts = foot_px(pts, depth, calib.K, calib.plane)
    a, bq = calib.to_norm(pts)
    du, dv = (bq[0] - a[0]) * calib.aspect, bq[1] - a[1]
    hd = float(math.atan2(dv, du))
    return hd, c.tolist()


def pattern_heading(bgr, gray, calib, depth, centre_px, half=34):
    """Rover heading (radians) from the marker pattern on its dark body, or None (see _pattern_ex)."""
    return _pattern_ex(bgr, gray, calib, depth, centre_px, half)[0]


def dark_body_px(bgr, gray, calib, depth, centre_px, half=34):
    """Image centroid of the rover's dark body (wheels included) where the marker pattern was read, else None."""
    return _pattern_ex(bgr, gray, calib, depth, centre_px, half)[1]
