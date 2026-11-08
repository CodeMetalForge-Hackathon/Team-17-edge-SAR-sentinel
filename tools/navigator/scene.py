"""Full-scene change detection (RGB + numerical depth) for the mission.

At mission START a fresh full scene is captured as the BASELINE (the initial environment). Every later full scene is compared
with the PREVIOUS VALID scene (a composite reference, see SceneWatch.ref); the baseline is kept unchanged for context only.

What counts as a change (everything else is ignored):
  * depth: the height above the tabletop changed by more than max(30 mm, 5 sigma of the measured noise) over an object-sized
    blob, where BOTH scenes have valid depth. Pixels with a depth dropout in either scene are never compared, so dropouts cannot
    create changes. A blob that got higher is an appearance, one that got lower a vacated place; a vacated + appeared pair of
    the same size / height / colour is a MOVE.
  * RGB (objects with little height or unusable depth): a local colour/brightness change after removing the global gain and
    every smooth illumination trend, over a solid object-sized blob. A blob where depth was valid before and is now missing is
    only accepted when RGB confirms it (a dark/shiny object), never from the dropout alone.
  * the rover is excluded by a mask built from its confidently identified pose (rotated footprint + a heading allowance + a small
    imaging margin, from the floor up to the rover's height, so its parallax, shadow and no-depth body are covered) in BOTH
    scenes. Only that region is masked: nothing else near it, and not its travel corridor.
  * the reference keeps OLD pixels wherever the rover covered them, so something that appeared under the rover's old position is
    still compared once the rover has left.
  * a camera or boundary movement (corners / plane / markers moved, the table level shifted, or most of the scene changing) is
    reported as a calibration problem, never as many new objects.
"""
import math
import time

import cv2
import numpy as np

import vision as V

DEPTH_T_MM = 45.0         # minimum height change that counts. Measured on the real camera at this range: median |change| 4 mm but a
                          # heavy tail (p99.9 35 mm, p99.99 54 mm; whole tabletop patches flip by one ~40 mm quantisation step)
DEPTH_T_MAX = 90.0
DEPTH_TAIL_K = 1.5        # threshold >= K x the measured p99.9 of the frame-to-frame |change| on bare table
DEPTH_MEAN_K = 1.4        # a blob's MEAN change must exceed K x threshold (real objects change by their whole height). 0 false
                          # candidates in 60 real scene pairs (1.5 s and 5 s apart); finds >= 80 mm objects 8/8, 60 mm 6/8 by depth alone
MIN_DEPTH_PX = 30         # object-sized blobs only (a 30 mm object is ~10 px across)
MIN_RGB_PX = 70
MIN_HOLE_PX = 120
RGB_L_MIN = 28.0          # luminance change (after removing gain + smooth trends), 0..255
RGB_C_MIN = 14.0          # chroma change (Lab a/b), 8-bit units
RGB_MAD_K = 6.0
GLOBAL_FRAC = 0.20        # more than this share of the comparable pixels changing is a disturbance, not objects
LEVEL_SHIFT_MM = 12.0     # tabletop level moving by this much = camera / plane problem
MOVE_EVENT_MM = 40.0      # a pair displaced less than this is jitter, not a move
PAIR_AREA_RATIO = 0.4
PAIR_H_MM = 45.0
PAIR_COLOR = 45.0
TABLE_LIKE = 22.0         # Lab distance to the table colour that still counts as bare table
CORNER_SHIFT_PX = 4.0
MARKER_SHIFT_PX = 3.0
ROVER_HEIGHT_MM = 130.0   # the rover mask reaches up to here (parallax of its top)
ROVER_MARGIN_MM = 20.0    # imaging halo / shadow allowance around the footprint
ROVER_YAW_ALLOW = 0.17    # rad, heading uncertainty of the footprint
SHADOW_PX = 15            # RGB-only candidates this close to the rover are deferred (its shadow / reflection), not reported

_GRIDS = {}


def _grid(shape):
    g = _GRIDS.get(shape)
    if g is None:
        ys, xs = np.mgrid[0:shape[0], 0:shape[1]]
        g = _GRIDS[shape] = (xs.astype(np.float32), ys.astype(np.float32))
    return g


def height_map(depth, calib):
    """Height (mm) above the calibrated tabletop of every pixel; NaN where depth is invalid."""
    xs, ys = _grid(depth.shape)
    fx, fy, cx, cy = calib.K
    n, d = calib.plane
    z = depth.astype(np.float32)
    h = d - (((xs - cx) * z / fx) * n[0] + ((ys - cy) * z / fy) * n[1] + z * n[2]) - calib.corr_full(depth.shape)
    h = h.astype(np.float32)
    h[depth == 0] = np.nan
    return h


def roi_mask(shape, calib):
    m = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(m, calib.corners.astype(np.int32), 1)
    return cv2.erode(m, np.ones((9, 9), np.uint8)).astype(bool)


def _lift_px(calib, px, hgt):
    fx, fy, cx, cy = calib.K
    n, d = calib.plane
    out = []
    for x, y in px:
        ray = np.array([(x - cx) / fx, (y - cy) / fy, 1.0])
        P = ray * (d / (n @ ray)) - n * hgt
        out.append((fx * P[0] / P[2] + cx, fy * P[1] / P[2] + cy))
    return np.array(out, np.float64)


def rover_mask(shape, calib, pose, L_mm, W_mm, margin_mm=ROVER_MARGIN_MM):
    """Image mask of the identified rover: hull of its (heading-uncertain) footprint at the floor and at ROVER_HEIGHT_MM."""
    A, hm = calib.aspect, calib.height_mm
    ux, vy = pose["x"] * A * hm, pose["y"] * hm
    hl, hw = L_mm / 2 + margin_mm, W_mm / 2 + margin_mm
    th = pose.get("theta")
    pts = []
    if th is None:                                     # heading unknown: the enclosing circle
        r = math.hypot(hl, hw)
        pts = [(ux + r * math.cos(a), vy + r * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
    else:
        for t in (th - ROVER_YAW_ALLOW, th, th + ROVER_YAW_ALLOW):
            c, s = math.cos(t), math.sin(t)
            for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
                pts.append((ux + c * a - s * b, vy + s * a + c * b))
    nrm = np.array([(u / (A * hm), v / hm) for u, v in pts])
    g = calib.to_px(nrm)
    allp = np.vstack([g, _lift_px(calib, g, ROVER_HEIGHT_MM)]).astype(np.float32)
    m = np.zeros(shape, np.uint8)
    cv2.fillConvexPoly(m, cv2.convexHull(allp).astype(np.int32), 1)
    return cv2.dilate(m, np.ones((5, 5), np.uint8)).astype(bool)


def foot_norm(calib, xs, ys, z, h):
    """Normalized sandbox positions of pixels dropped vertically onto the tabletop."""
    fx, fy, cx, cy = calib.K
    n, _d = calib.plane
    P = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], 1).astype(np.float64)
    Pf = P + n[None, :] * h[:, None]
    return calib.to_norm(np.stack([fx * Pf[:, 0] / Pf[:, 2] + cx, fy * Pf[:, 1] / Pf[:, 2] + cy], 1))


def _wblur(d, w, sigma):
    return cv2.GaussianBlur(d * w, (0, 0), sigma) / np.maximum(cv2.GaussianBlur(w, (0, 0), sigma), 1e-3)


def _comps(mask, min_area, open_k=3, close_k=0):
    m = mask.astype(np.uint8)
    if open_k:
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((open_k, open_k), np.uint8))
    if close_k:
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8))
    n, lab, st, _ce = cv2.connectedComponentsWithStats(m, connectivity=8)
    return [(lab == i, st[i]) for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= min_area]


class SceneObs:
    """One valid full-scene observation (depth heights, Lab colour, rover mask) plus where/when it was captured."""

    def __init__(self, frame, pose, conf, calib, per, footprint_mm):
        depth = frame.depth
        self.seq = frame.seq
        self.rx = getattr(frame, "rx", None) or time.time()
        self.wall = time.time()
        self.depth = depth
        self.h = height_map(depth, calib)
        self.lab = cv2.cvtColor(frame.bgr, cv2.COLOR_BGR2LAB)
        self.roi = roi_mask(depth.shape, calib)
        self.pose, self.conf = pose, conf
        self.mask = np.zeros(depth.shape, bool)
        if conf and pose is not None and pose.get("x") is not None and footprint_mm:
            self.mask = rover_mask(depth.shape, calib, pose, footprint_mm[0], footprint_mm[1])
        mk = V.detect_markers(frame.gray)
        self.markers = {k: v for k, v in mk.items() if k in (V.ID_BL, V.ID_TL, V.ID_BR, V.ID_TR) and k not in per.manual_markers}   # boundary only, never the rover's
        self.sig = (None if calib.corners is None else calib.corners.copy(), None if calib.plane is None else (calib.plane[0].copy(), calib.plane[1]),
                    calib.aspect, calib.height_mm)


class Ref:
    """Previous-valid-scene reference: pixels the rover covered keep their older values."""

    def __init__(self, cur):
        self.z, self.h, self.lab = cur.depth.copy(), cur.h.copy(), cur.lab.copy()
        self.dk = np.isfinite(cur.h) & ~cur.mask
        self.ck = ~cur.mask
        self.h[~self.dk] = np.nan
        self.seq, self.rx = cur.seq, cur.rx

    def advance(self, cur, hold=None):
        """Fold the new scene in, except under the rover and in `hold` (deferred candidates stay UNABSORBED so that the next
        scene judges them again)."""
        keep = ~cur.mask if hold is None else (~cur.mask & ~hold)
        upd = np.isfinite(cur.h) & keep
        self.h[upd], self.z[upd] = cur.h[upd], cur.depth[upd]
        self.dk |= upd
        uc = keep
        self.lab[uc] = cur.lab[uc]
        self.ck |= uc
        self.seq, self.rx = cur.seq, cur.rx


class Result:
    def __init__(self, status, why="", cands=None, stats=None):
        self.status, self.why, self.cands, self.stats = status, why, cands or [], stats or {}


class SceneWatch:
    def __init__(self, per):
        self.per = per
        self.baseline = None
        self.ref = None
        self.table_lab = None

    # ------------------------------------------------------------------ one scene in
    def observe(self, frame, pose, conf, footprint_mm):
        c = self.per.calib
        if getattr(frame, "partial", False) or frame.depth is None:
            return None, Result("invalid", "partial frame")
        if c.H is None or c.plane is None or c.K is None or c.height_mm is None:
            return None, Result("calibration", "calibration incomplete (boundary / ground plane)")
        if not conf or pose is None:
            return None, Result("invalid", "the rover is not confidently identified in this scene")
        cur = SceneObs(frame, pose, conf, c, self.per, footprint_mm)
        if self.baseline is None:
            self.baseline, self.ref = cur, Ref(cur)
            m = cur.roi & ~cur.mask
            self.table_lab = np.median(cur.lab[m].reshape(-1, 3).astype(np.float32), 0) if m.any() else None
            return cur, Result("baseline", stats={"area_px": int(m.sum())})
        prob = self.calibration_problem(cur)
        if prob:
            return cur, Result("calibration", prob)
        res = self.compare(cur)
        if res.status == "ok":
            hold = res.stats.pop("_hold", None)
            res.stats["prev_seq"], res.stats["prev_rx"] = self.ref.seq, self.ref.rx
            for cand in res.cands:                      # the object's signature needs the background it replaced: before advancing
                if cand.get("to") is not None:
                    cand["to"]["sig"] = self.signature(cand["to"], cur, self.ref)
            self.ref.advance(cur, hold)
        return cur, res

    def calibration_problem(self, cur):
        b = self.baseline
        cs, bs = cur.sig, b.sig
        if cs[0] is None or cs[1] is None:
            return "boundary / ground plane calibration was lost"
        shift = float(np.max(np.linalg.norm(cs[0] - bs[0], axis=1)))
        if shift > CORNER_SHIFT_PX:
            return "the boundary moved %.1f px since the baseline (camera or markers moved)" % shift
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(cs[1][0] @ bs[1][0])))))
        if ang > 0.6 or abs(cs[1][1] - bs[1][1]) > 12.0 or abs(cs[2] - bs[2]) > 0.03 * bs[2]:
            return "the ground plane / aspect changed since the baseline (camera moved: %.2f deg, %.0f mm)" % (ang, cs[1][1] - bs[1][1])
        sh = [float(np.max(np.linalg.norm(cur.markers[k] - b.markers[k], axis=1))) for k in cur.markers if k in b.markers]
        if sh and max(sh) > MARKER_SHIFT_PX:
            return "a boundary marker moved %.1f px since the baseline" % max(sh)
        return ""

    # ------------------------------------------------------------------ compare with the previous valid scene
    def compare(self, cur):
        ref, c = self.ref, self.per.calib
        roi = cur.roi
        dok = ref.dk & np.isfinite(cur.h) & ~cur.mask & roi
        cok = ref.ck & ~cur.mask & roi
        if int(dok.sum()) < 2000 or int(cok.sum()) < 2000:
            return Result("invalid", "too little of the scene is comparable (%d px)" % int(dok.sum()))
        dh = np.where(dok, cur.h - ref.h, 0.0).astype(np.float32)
        flat = dok & (np.abs(ref.h) < 25) & (np.abs(cur.h) < 25)
        if flat.sum() > 500:
            med = float(np.median(dh[flat]))
            sig = float(np.percentile(np.abs(dh[flat] - med), 99.9))        # tail of the noise, not its (misleadingly tiny) median
        else:
            med, sig = 0.0, 15.0
        T = float(np.clip(DEPTH_TAIL_K * sig, DEPTH_T_MM, DEPTH_T_MAX))
        stats = {"T_mm": round(T, 1), "noise_p999_mm": round(sig, 2), "level_shift_mm": round(med, 1), "comparable_px": int(dok.sum())}
        if abs(med) > LEVEL_SHIFT_MM:
            return Result("calibration", "the tabletop level shifted %.0f mm between scenes (camera moved)" % med, stats=stats)
        raised, vacated = dok & (dh > T), dok & (dh < -T)
        fd = float((raised | vacated).sum()) / max(int(dok.sum()), 1)
        stats["depth_changed_frac"] = round(fd, 4)
        if fd > GLOBAL_FRAC:
            return Result("calibration", "%.0f%% of the scene changed in depth at once (camera or boundary moved)" % (100 * fd), stats=stats)
        # ---- RGB: remove gain and every smooth illumination trend, keep local object-sized changes
        L0, L1 = ref.lab[..., 0].astype(np.float32), cur.lab[..., 0].astype(np.float32)
        gm = float(np.clip(np.median(L0[cok]) / max(float(np.median(L1[cok])), 1.0), 0.5, 2.0))
        w = cok.astype(np.float32)
        d = (L1 * gm - L0) * w
        dl = np.abs(cv2.GaussianBlur(d - _wblur(d, w, 30.0), (0, 0), 1.2)) * w
        dab = (cur.lab[..., 1:].astype(np.float32) - ref.lab[..., 1:].astype(np.float32))
        for k in (0, 1):
            dab[..., k] -= float(np.median(dab[..., k][cok]))
        dc = cv2.GaussianBlur(np.hypot(dab[..., 0], dab[..., 1]), (0, 0), 1.2) * w
        TL = max(RGB_L_MIN, RGB_MAD_K * 1.4826 * float(np.median(dl[cok])))
        TC = max(RGB_C_MIN, RGB_MAD_K * 1.4826 * float(np.median(dc[cok])))
        rgbm = cok & ((dl > TL) | (dc > TC))
        fc = float(rgbm.sum()) / max(int(cok.sum()), 1)
        suppressed = fc > GLOBAL_FRAC
        stats.update({"rgb_L_thr": round(TL, 1), "rgb_C_thr": round(TC, 1), "rgb_changed_frac": round(fc, 4), "gain": round(gm, 3),
                      "rgb_suppressed": bool(suppressed)})
        if suppressed:
            rgbm = np.zeros_like(rgbm)             # illumination / exposure disturbance: depth evidence only this time
        # ---- candidate blobs
        blobs, dep_union = [], np.zeros(cur.h.shape, bool)
        for kind, m0, srch, srcz in (("raised", raised, cur.h, cur.depth), ("vacated", vacated, ref.h, ref.z)):
            for m, st in _comps(m0, MIN_DEPTH_PX):
                if float(np.mean(np.abs(dh[m]))) < DEPTH_MEAN_K * T:
                    continue
                dep_union |= m
                blobs.append(self._blob(kind, m, srch, srcz, cur, ref, dh, dl, dc, "depth"))
        dep_dil = cv2.dilate(dep_union.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
        holes = ref.dk & ~np.isfinite(cur.h) & ~cur.mask & roi & (cur.depth == 0) & rgbm
        for m, st in _comps(holes, MIN_HOLE_PX, 3, 5):
            if (m & dep_dil).sum() > 0.4 * m.sum():
                continue
            if float(rgbm[m].mean()) < 0.4:
                continue
            blobs.append(self._blob("rgb", m, None, None, cur, ref, dh, dl, dc, "no-depth"))
        used = dep_dil | (holes & rgbm)
        for m, st in _comps(rgbm & ~used, MIN_RGB_PX, 3, 5):
            bw, bh = st[cv2.CC_STAT_WIDTH], st[cv2.CC_STAT_HEIGHT]
            if min(bw, bh) < 6 or st[cv2.CC_STAT_AREA] < 0.35 * bw * bh:      # thin edges / shadow lines are not objects
                continue
            if float(np.mean(dl[m])) < 1.3 * TL and float(np.mean(dc[m])) < 1.3 * TC:
                continue
            blobs.append(self._blob("rgb", m, None, None, cur, ref, dh, dl, dc, "rgb"))
        # RGB-only candidates touching the rover's surroundings are most likely its shadow / reflection: depth is immune to those,
        # so they are held back (and left out of the reference) until the rover has moved on and the scene can be judged again
        near = cv2.dilate(cur.mask.astype(np.uint8), np.ones((2 * SHADOW_PX + 1, 2 * SHADOW_PX + 1), np.uint8)).astype(bool)
        hold, kept = np.zeros(cur.h.shape, bool), []
        for b in blobs:
            if b["support"] != "depth" and (b["mask"] & near).any():
                hold |= b["mask"]
            else:
                kept.append(b)
        blobs = kept
        stats["deferred_px"] = int(hold.sum())
        stats["_hold"] = hold if hold.any() else None
        stats["blobs"] = len(blobs)
        return Result("ok", cands=self._classify(blobs, cur), stats=stats)

    def _blob(self, kind, m, srch, srcz, cur, ref, dh, dl, dc, support):
        ys, xs = np.nonzero(m)
        c = self.per.calib
        dm = self.per.depth_map
        if srch is not None:
            hh = srch[ys, xs]
            fin = np.isfinite(hh) & (hh > 10)
        else:
            fin = np.zeros(len(ys), bool)
        if fin.sum() >= 5:
            foot = foot_norm(c, xs[fin].astype(np.float64), ys[fin].astype(np.float64), srcz[ys[fin], xs[fin]].astype(np.float64), hh[fin].astype(np.float64))
        else:
            foot = c.to_norm(np.stack([xs, ys], 1).astype(np.float64))
        ci = np.clip((foot[:, 0] * dm.cols).astype(int), 0, dm.cols - 1)
        ri = np.clip((foot[:, 1] * dm.rows).astype(int), 0, dm.rows - 1)
        g = np.zeros((dm.rows, dm.cols), np.uint8)
        g[ri, ci] = 1
        g = cv2.dilate(g, np.ones((3, 3), np.uint8))
        rr, cc = np.nonzero(g)
        bh = self.baseline.h[m]
        return {"kind": kind, "support": support, "mask": m, "area": int(len(ys)),
                "px": (float(xs.mean()), float(ys.mean())), "xy": (float(np.median(foot[:, 0])), float(np.median(foot[:, 1]))),
                "cells": list(zip(cc.tolist(), rr.tolist())),
                "dh_mm": float(np.nanmean(dh[m])) if kind in ("raised", "vacated") else 0.0,
                "h_cur": float(np.nanmedian(cur.h[m])) if np.isfinite(cur.h[m]).any() else None,
                "h_ref": float(np.nanmedian(ref.h[m])) if np.isfinite(ref.h[m]).any() else None,
                "h_base": float(np.nanmedian(bh)) if np.isfinite(bh).any() else None,
                "lab_cur": cur.lab[m].reshape(-1, 3).mean(0), "lab_ref": ref.lab[m].reshape(-1, 3).mean(0),
                "dL": float(np.mean(dl[m])), "dC": float(np.mean(dc[m]))}

    def table_like(self, lab):
        return self.table_lab is not None and float(np.linalg.norm(np.asarray(lab) - self.table_lab)) < TABLE_LIKE

    def _classify(self, blobs, cur):
        A, hm = self.per.calib.aspect, self.per.calib.height_mm
        dist = lambda a, b: math.hypot((a["xy"][0] - b["xy"][0]) * A * hm, (a["xy"][1] - b["xy"][1]) * hm)
        vac = [b for b in blobs if b["kind"] == "vacated"]
        app = [b for b in blobs if b["kind"] == "raised"]
        pairs = []
        for v in vac:
            for a in app:
                r = min(v["area"], a["area"]) / max(v["area"], a["area"])
                hd = abs((v["h_ref"] or 0.0) - (a["h_cur"] or 0.0))
                cd = float(np.linalg.norm(v["lab_ref"] - a["lab_cur"]))
                if r >= PAIR_AREA_RATIO and hd <= PAIR_H_MM and cd <= PAIR_COLOR:
                    pairs.append((hd / PAIR_H_MM + cd / PAIR_COLOR + (1 - r), id(v), id(a), v, a))
        pairs.sort(key=lambda t: t[0])
        out, usedv, useda = [], set(), set()
        for _s, iv, ia, v, a in pairs:
            if iv in usedv or ia in useda:
                continue
            usedv.add(iv)
            useda.add(ia)
            d = dist(v, a)
            if d < MOVE_EVENT_MM:
                continue                                   # a small shift of the same object: jitter, not a move
            out.append({"type": "moved", "to": a, "from": v, "displacement_mm": d})
        for a in app:
            if id(a) not in useda:
                out.append({"type": "new", "to": a})
        for v in vac:
            if id(v) not in usedv:
                out.append({"type": "removed", "from": v})
        for b in blobs:
            if b["kind"] != "rgb":
                continue
            # an RGB-only change: a new object unless the place now looks like bare table (something was taken away)
            if self.table_like(b["lab_cur"]) and not self.table_like(b["lab_ref"]):
                out.append({"type": "removed", "from": b})
            else:
                out.append({"type": "new", "to": b})
        return out

    # ------------------------------------------------------------------ object presence (for events not yet inspected)
    @staticmethod
    def signature(blob, cur, ref):
        m = blob["mask"]
        ys, xs = np.nonzero(m)
        return {"mask": m, "dil": cv2.dilate(m.astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool), "ys": ys, "xs": xs,
                "obj_h": cur.h[ys, xs].copy(), "bg_h": ref.h[ys, xs].copy(),
                "obj_lab": cur.lab[ys, xs].astype(np.float32), "bg_lab": ref.lab[ys, xs].astype(np.float32)}

    @staticmethod
    def presence(sig, cur):
        """Fraction of the object's pixels that still look like the object (not like the background it replaced), or None
        when too little of it is visible (rover in front of it, depth dropouts)."""
        ys, xs = sig["ys"], sig["xs"]
        vis = cur.roi[ys, xs] & ~cur.mask[ys, xs]
        if vis.sum() < max(10, 0.4 * len(ys)):
            return None
        votes = []
        ch = cur.h[ys, xs]
        use_h = vis & np.isfinite(ch) & np.isfinite(sig["obj_h"]) & np.isfinite(sig["bg_h"]) & (np.abs(sig["obj_h"] - sig["bg_h"]) > 20)
        if use_h.any():
            votes.append(np.abs(ch[use_h] - sig["obj_h"][use_h]) < np.abs(ch[use_h] - sig["bg_h"][use_h]))
        cl = cur.lab[ys, xs].astype(np.float32)
        sep = np.linalg.norm(sig["obj_lab"] - sig["bg_lab"], axis=1)
        use_c = vis & ~use_h & (sep > 20)
        if use_c.any():
            votes.append(np.linalg.norm(cl[use_c] - sig["obj_lab"][use_c], axis=1) < np.linalg.norm(cl[use_c] - sig["bg_lab"][use_c], axis=1))
        if not votes:
            return None
        v = np.concatenate(votes)
        return float(v.mean()) if len(v) >= 8 else None
