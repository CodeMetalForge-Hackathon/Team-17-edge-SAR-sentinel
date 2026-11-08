"""HQ vision: overhead photo -> sandbox frame, rubble, rover pose.

Frame used everywhere in hq/: metres, origin at the sandbox's bottom-left corner as seen in the photo,
+x to the right, +y up (same orientation as the overhead viewer: A1 is the bottom-left square).
The flattened "map image" is the sandbox seen straight down at PX_PER_M pixels per metre (image rows go down).
"""
import math

import cv2
import numpy as np

PX_PER_M = 400
DICTS = ["DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50", "DICT_ARUCO_ORIGINAL"]

# rubble detection (see tools/overhead_viewer/detect.py for the reasoning)
DIFF_THRESH = 28
L_WEIGHT = 0.5
MIN_AREA_M2 = 0.004      # ignore changed blobs smaller than ~6 x 6 cm
EDGE_TRIM_M = 0.02


class Sandbox:
    """Size of the sandbox in metres + the 4 corner pixels [BL, BR, TR, TL] in the camera photo."""

    def __init__(self, width_m, height_m, corners_px):
        self.w, self.h = float(width_m), float(height_m)
        self.corners = np.float32(corners_px)
        self.W, self.H = int(round(self.w * PX_PER_M)), int(round(self.h * PX_PER_M))
        bl, br, tr, tl = self.corners
        # photo pixels -> map pixels (flattened, rows down)
        self.to_map = cv2.getPerspectiveTransform(np.float32([tl, tr, br, bl]),
                                                  np.float32([[0, 0], [self.W, 0], [self.W, self.H], [0, self.H]]))

    def flatten(self, photo):
        return cv2.warpPerspective(photo, self.to_map, (self.W, self.H), flags=cv2.INTER_LINEAR)

    def photo_to_m(self, pts):
        """Nx2 photo pixels -> Nx2 metres (sandbox frame, y up)."""
        p = cv2.perspectiveTransform(np.float32(pts).reshape(-1, 1, 2), self.to_map).reshape(-1, 2)
        return np.stack([p[:, 0] / PX_PER_M, self.h - p[:, 1] / PX_PER_M], axis=1)

    def m_to_map_px(self, x, y):
        return int(round(x * PX_PER_M)), int(round((self.h - y) * PX_PER_M))


def detect_markers(photo):
    """-> {id: 4x2 corner pixels (TL, TR, BR, BL of the marker)}, trying a few common dictionaries."""
    gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY) if photo.ndim == 3 else photo
    params = cv2.aruco.DetectorParameters()
    best = {}
    for name in DICTS:
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)), params)
        corners, ids, _ = det.detectMarkers(gray)
        found = {} if ids is None else {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}
        if len(found) > len(best):
            best = found
        if len(found) >= 5:
            break
    return best


def find_markers(photo):
    """detect_markers + a second look for a corner marker missed because nothing white surrounds it
    (cardboard or a dark mat touching the paper). If exactly 3 are seen, the 4th is searched for around where
    a parallelogram of the other 3 puts it, with tight marker-sized crops given an artificial white margin."""
    markers = detect_markers(photo)
    if len(markers) != 3:
        return markers
    gray = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY) if photo.ndim == 3 else photo
    side = float(np.mean([np.linalg.norm(c[0] - c[1]) for c in markers.values()]))
    cents = [c.mean(axis=0) for c in markers.values()]
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())
    # the missing corner is opposite the known marker that has the other two as neighbours: try all 3 guesses
    guesses = [cents[(k + 1) % 3] + cents[(k + 2) % 3] - cents[k] for k in range(3)]
    reach, step = int(1.6 * side), max(8, int(side / 5))
    for scale in (0.85, 1.05, 1.25):
        S = int(scale * side)
        pad = S // 4
        for gx, gy in guesses:
            for dy in range(-reach, reach + 1, step):
                for dx in range(-reach, reach + 1, step):
                    x0, y0 = int(gx + dx) - S // 2, int(gy + dy) - S // 2
                    if x0 < 0 or y0 < 0 or x0 + S > gray.shape[1] or y0 + S > gray.shape[0]:
                        continue
                    crop = cv2.copyMakeBorder(gray[y0:y0 + S, x0:x0 + S], pad, pad, pad, pad,
                                              cv2.BORDER_CONSTANT, value=255)
                    corners, ids, _ = det.detectMarkers(crop)
                    if ids is None:
                        continue
                    for i, c in zip(ids.flatten(), corners):
                        if int(i) not in markers:
                            markers[int(i)] = c.reshape(4, 2) + np.float32([x0 - pad, y0 - pad])
                            return markers
    return markers


def outer_corners(markers, rover_id=None):
    """Like corners_from_markers but each corner is the marker corner FARTHEST from the middle, so the
    sandbox includes the markers themselves (the setup where objects sit between the 4 markers)."""
    cand = {i: c for i, c in markers.items() if i != rover_id}
    if len(cand) != 4:
        return None
    cents = {i: c.mean(axis=0) for i, c in cand.items()}
    mid = np.mean(list(cents.values()), axis=0)
    s = {i: p[0] + p[1] for i, p in cents.items()}
    d = {i: p[0] - p[1] for i, p in cents.items()}
    tl, br, tr, bl = min(s, key=s.get), max(s, key=s.get), max(d, key=d.get), min(d, key=d.get)
    if len({tl, br, tr, bl}) != 4:
        return None
    far = lambda i: cand[i][np.argmax(np.linalg.norm(cand[i] - mid, axis=1))].tolist()
    return [far(bl), far(br), far(tr), far(tl)]


def objects_on_white(top, sandbox, markers, min_area_m2=0.004):
    """Objects on a plain white board: anything coloured or dark, except the corner markers' paper.
    -> [{x0, x1, y0, y1, cx, cy, area_m2}] in metres (y up)."""
    H = top.shape[0]
    skip = np.zeros(top.shape[:2], np.uint8)
    for q in markers.values():
        p = cv2.perspectiveTransform(np.float32(q).reshape(-1, 1, 2), sandbox.to_map).reshape(-1, 2)
        c = p.mean(axis=0)
        cv2.fillConvexPoly(skip, np.int32(c + (p - c) * 1.4), 255)
    hsv = cv2.cvtColor(cv2.GaussianBlur(top, (5, 5), 0), cv2.COLOR_BGR2HSV)
    v = hsv[..., 2].astype(np.float32)
    dark = v < 0.72 * np.median(v)                   # relative to the board's own brightness
    mask = ((hsv[..., 1] > 60) | dark).astype(np.uint8) * 255
    mask[skip > 0] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8), iterations=2)
    n, _, st, cen = cv2.connectedComponentsWithStats(mask)
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_area_m2 * PX_PER_M * PX_PER_M:
            continue
        out.append({"x0": x / PX_PER_M, "x1": (x + w) / PX_PER_M, "y0": (H - y - h) / PX_PER_M,
                    "y1": (H - y) / PX_PER_M, "cx": float(cen[i][0]) / PX_PER_M,
                    "cy": (H - float(cen[i][1])) / PX_PER_M, "area_m2": a / PX_PER_M ** 2})
    return out


def corners_from_markers(markers, rover_id=None):
    """Find the sandbox corners from the 4 corner markers, whatever their IDs.

    Uses every marker except the rover's. The 4 are assigned BL/BR/TR/TL by where they sit in the photo,
    and each corner is the marker corner nearest the middle of the 4 (its inner corner, the sandbox edge),
    like the handles in the overhead viewer. -> [BL, BR, TR, TL] pixels, or None unless exactly 4 are seen."""
    cand = {i: c for i, c in markers.items() if i != rover_id}
    if len(cand) != 4:
        return None
    cents = {i: c.mean(axis=0) for i, c in cand.items()}
    mid = np.mean(list(cents.values()), axis=0)
    s = {i: p[0] + p[1] for i, p in cents.items()}      # image coords: x right, y DOWN
    d = {i: p[0] - p[1] for i, p in cents.items()}
    tl, br = min(s, key=s.get), max(s, key=s.get)
    tr, bl = max(d, key=d.get), min(d, key=d.get)
    if len({tl, br, tr, bl}) != 4:
        return None
    inner = lambda i: cand[i][np.argmin(np.linalg.norm(cand[i] - mid, axis=1))].tolist()
    return [inner(bl), inner(br), inner(tr), inner(tl)]


def rover_pose(markers, sandbox, rover_id):
    """-> (x, y, heading_rad) in metres, heading = direction of the marker's top edge normal (its 'up'),
    or None if the rover marker is not visible."""
    if rover_id not in markers:
        return None
    m = sandbox.photo_to_m(markers[rover_id])  # TL, TR, BR, BL in metres
    c = m.mean(axis=0)
    up = (m[0] + m[1]) / 2 - (m[3] + m[2]) / 2   # bottom edge midpoint -> top edge midpoint
    return float(c[0]), float(c[1]), math.atan2(up[1], up[0])


def change_mask(before_map, after_map, ignore_circles=()):
    """0/255 mask of what changed between two flattened views. ignore_circles: [(x_m, y_m, r_m)]."""
    b = cv2.GaussianBlur(before_map, (7, 7), 0)
    a = cv2.GaussianBlur(after_map, (7, 7), 0)
    bl = cv2.cvtColor(b, cv2.COLOR_BGR2LAB).astype(np.float32)
    al = cv2.cvtColor(a, cv2.COLOR_BGR2LAB).astype(np.float32)
    al[..., 0] += bl[..., 0].mean() - al[..., 0].mean()   # cancel a global exposure change
    d = np.abs(al - bl)
    mask = ((L_WEIGHT * d[..., 0] + d[..., 1] + d[..., 2]) > DIFF_THRESH).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    e = int(EDGE_TRIM_M * PX_PER_M)
    if e:
        mask[:e, :] = 0; mask[-e:, :] = 0; mask[:, :e] = 0; mask[:, -e:] = 0
    H = mask.shape[0]
    for x, y, r in ignore_circles:
        cv2.circle(mask, (int(x * PX_PER_M), int(H - y * PX_PER_M)), int(r * PX_PER_M), 0, -1)
    return mask


def rubble(before_map, after_map, ignore_circles=()):
    """-> (obstacles, mask). obstacles: [{x0, y0, x1, y1}] in metres (y up)."""
    mask = change_mask(before_map, after_map, ignore_circles)
    H, W = mask.shape
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    min_px = MIN_AREA_M2 * PX_PER_M * PX_PER_M
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_px:
            continue
        out.append({"x0": round(x / PX_PER_M, 3), "x1": round((x + w) / PX_PER_M, 3),
                    "y0": round((H - y - h) / PX_PER_M, 3), "y1": round((H - y) / PX_PER_M, 3)})
    return out, mask
