"""Automatic rubble detection for the overhead viewer: before photo vs after photo -> obstacle rectangles.

Uses the same frame as homography.js: corners are image pixels in the order [BL, BR, TR, TL] as seen in the
photo; the normalized sandbox frame has BL = (0, 0), BR = (1, 0), TR = (1, 1), TL = (0, 1) (y points UP).
Obstacles come back as {x0, y0, x1, y1} in that frame, ready for the viewer's footprint planner.

    python detect.py --before before.jpg --after after.jpg --corners corners.json [--cols 6 --rows 4]
                     [--exclude x0,y0,x1,y1] [--debug detect_debug.png]

How it works: both photos are flattened to the same top-down view, compared in LAB colour (lightness is
down-weighted so soft shadows and exposure drift count less than a real object), thresholded, cleaned up,
and every changed blob bigger than MIN_AREA_FRAC of the sandbox becomes an obstacle rectangle.
Needs: opencv (python3-opencv or opencv-python-headless) and numpy.
"""
import argparse
import json
import sys

import cv2
import numpy as np

MAP_W, MAP_H = 600, 400        # flattened view size (3:2, like the 1.5 m x 1.0 m sandbox)
DIFF_THRESH = 28               # per-pixel change score that counts as "changed"
L_WEIGHT = 0.5                 # lightness counts half: shadows and exposure drift mostly change L
MIN_AREA_FRAC = 0.004          # ignore blobs smaller than 0.4% of the sandbox (noise, specks)
EDGE_TRIM_PX = 6               # ignore a thin band at the sandbox border (tape, corner markers)


def load_corners(path):
    """corners.json from the viewer -> [[x, y] * 4] in image pixels, order BL, BR, TR, TL."""
    with open(path) as f:
        d = json.load(f)
    c = d["corners"] if isinstance(d, dict) else d
    if len(c) != 4:
        raise ValueError("need 4 corners (BL, BR, TR, TL)")
    return [[float(x), float(y)] for x, y in c]


def flatten(img, corners, w=MAP_W, h=MAP_H):
    """Warp the photo so the sandbox fills a w x h top-down image (image rows go DOWN, so TL is (0, 0))."""
    bl, br, tr, tl = corners
    src = np.float32([tl, tr, br, bl])
    dst = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    M = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)


def change_mask(before_top, after_top):
    """0/255 mask of pixels that changed between two flattened views."""
    b = cv2.GaussianBlur(before_top, (7, 7), 0)
    a = cv2.GaussianBlur(after_top, (7, 7), 0)
    bl = cv2.cvtColor(b, cv2.COLOR_BGR2LAB).astype(np.float32)
    al = cv2.cvtColor(a, cv2.COLOR_BGR2LAB).astype(np.float32)
    # match overall brightness first, so a global exposure change is not "rubble"
    al[..., 0] += bl[..., 0].mean() - al[..., 0].mean()
    d = np.abs(al - bl)
    score = L_WEIGHT * d[..., 0] + d[..., 1] + d[..., 2]
    mask = (score > DIFF_THRESH).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)           # remove specks
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)  # fill holes in objects
    if EDGE_TRIM_PX:
        e = EDGE_TRIM_PX
        mask[:e, :] = 0; mask[-e:, :] = 0; mask[:, :e] = 0; mask[:, -e:] = 0
    return mask, score


def to_norm_rect(x, y, w, h, W=MAP_W, H=MAP_H):
    """Pixel box in the flattened view (y down) -> normalized rect (y up)."""
    return {"x0": round(x / W, 4), "x1": round((x + w) / W, 4),
            "y0": round(1 - (y + h) / H, 4), "y1": round(1 - y / H, 4)}


def detect(before_img, after_img, corners, exclude=None):
    """-> (obstacles, mask, before_top, after_top). exclude: list of normalized rects to ignore (the rover)."""
    bt, at = flatten(before_img, corners), flatten(after_img, corners)
    mask, _ = change_mask(bt, at)
    for r in exclude or []:  # blank out e.g. the rover so it is not treated as rubble
        x0, x1 = int(r["x0"] * MAP_W), int(r["x1"] * MAP_W)
        y0, y1 = int((1 - r["y1"]) * MAP_H), int((1 - r["y0"]) * MAP_H)
        mask[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = 0
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    min_px = MIN_AREA_FRAC * MAP_W * MAP_H
    obstacles = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_px:
            continue
        rect = to_norm_rect(x, y, w, h)
        rect["area_frac"] = round(area / (MAP_W * MAP_H), 4)
        rect["source"] = "detected"
        obstacles.append(rect)
    obstacles.sort(key=lambda r: -r["area_frac"])
    return obstacles, mask, bt, at


def blocked_cells(obstacles, cols, rows):
    """Cells (col, row; row 0 = bottom) that any obstacle overlaps. Names like the viewer: A1 = bottom-left."""
    out = []
    for c in range(cols):
        for r in range(rows):
            cx0, cx1, cy0, cy1 = c / cols, (c + 1) / cols, r / rows, (r + 1) / rows
            if any(o["x0"] < cx1 and cx0 < o["x1"] and o["y0"] < cy1 and cy0 < o["y1"] for o in obstacles):
                out.append(chr(65 + c) + str(r + 1))
    return out


def debug_image(after_top, mask, obstacles, cols, rows):
    """Flattened 'after' view with the change mask tinted red, obstacle boxes and the grid drawn on."""
    img = after_top.copy()
    red = np.zeros_like(img); red[..., 2] = 255
    img = np.where(mask[..., None] > 0, (0.55 * img + 0.45 * red).astype(np.uint8), img)
    for c in range(1, cols):
        x = int(c * MAP_W / cols); cv2.line(img, (x, 0), (x, MAP_H), (255, 255, 255), 1)
    for r in range(1, rows):
        y = int(r * MAP_H / rows); cv2.line(img, (0, y), (MAP_W, y), (255, 255, 255), 1)
    for c in range(cols):
        for r in range(rows):
            x, y = int(c * MAP_W / cols) + 3, int(MAP_H - (r + 1) * MAP_H / rows) + 14
            cv2.putText(img, chr(65 + c) + str(r + 1), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    for i, o in enumerate(obstacles):
        p0 = (int(o["x0"] * MAP_W), int((1 - o["y1"]) * MAP_H))
        p1 = (int(o["x1"] * MAP_W), int((1 - o["y0"]) * MAP_H))
        cv2.rectangle(img, p0, p1, (0, 0, 255), 2)
        cv2.putText(img, "#%d" % (i + 1), (p0[0] + 2, p0[1] + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--corners", required=True, help="corners.json saved by the viewer")
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--exclude", action="append", default=[], help="normalized x0,y0,x1,y1 to ignore (the rover)")
    ap.add_argument("--debug", help="write an overlay PNG here")
    a = ap.parse_args()
    before, after = cv2.imread(a.before), cv2.imread(a.after)
    if before is None or after is None:
        sys.exit("could not read the before/after images")
    if before.shape != after.shape:
        sys.exit("before and after photos must be the same size (same camera, same mode)")
    excl = []
    for s in a.exclude:
        x0, y0, x1, y1 = (float(v) for v in s.split(","))
        excl.append({"x0": x0, "y0": y0, "x1": x1, "y1": y1})
    obstacles, mask, _, at = detect(before, after, load_corners(a.corners), excl)
    if a.debug:
        cv2.imwrite(a.debug, debug_image(at, mask, obstacles, a.cols, a.rows))
    print(json.dumps({"obstacles": obstacles, "blocked": blocked_cells(obstacles, a.cols, a.rows)}, indent=1))


if __name__ == "__main__":
    main()
