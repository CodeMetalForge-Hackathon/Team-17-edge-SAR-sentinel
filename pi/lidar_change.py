"""Static-sentinel LiDAR change detection. No SLAM: the LiDAR doesn't move, so its frame IS the map.

1. Record a baseline of the empty scene (median of N scans per 1-degree bin).
2. Each live scan: a bin is "changed" if something is CHANGE_THRESH_M closer than baseline
   (or something appears where baseline saw nothing).
3. Group adjacent changed bins into blobs, require them to persist a few scans, emit one event each.
"""
import math
import numpy as np

import config


def points_to_bins(points):
    """points: iterable of (raw_angle_deg, dist_mm) from the driver -> 360 ranges in meters, map frame.
    Bin i = map bearing i degrees (CCW from LiDAR forward). NaN = no valid return."""
    bins = np.full(360, np.nan)
    for ang, dist_mm in points:
        d = dist_mm / 1000.0
        if not (config.LIDAR_MIN_RANGE_M <= d <= config.LIDAR_MAX_RANGE_M):
            continue
        a = (config.LIDAR_ANGLE_SIGN * ang + config.LIDAR_ANGLE_OFFSET_DEG) % 360
        i = int(round(a)) % 360
        if np.isnan(bins[i]) or d < bins[i]:
            bins[i] = d
    return bins


def build_baseline(scans):
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN bins stay NaN, that's fine
        return np.nanmedian(np.vstack(scans), axis=0)


def polar_to_xy(bearing_deg, r):
    a = math.radians(bearing_deg)
    return r * math.cos(a), r * math.sin(a)


def _circular_runs(mask):
    """Indices of contiguous True runs on a circular 360 array."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    if len(idx) == 360:
        return [list(range(360))]
    runs, cur = [], [idx[0]]
    for i in idx[1:]:
        if i == cur[-1] + 1:
            cur.append(i)
        else:
            runs.append(cur)
            cur = [i]
    runs.append(cur)
    if len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == 359:
        runs[0] = runs[-1] + runs[0]
        runs.pop()
    return runs


def _circ_mean_deg(degs):
    rad = np.radians(degs)
    return math.degrees(math.atan2(np.sin(rad).mean(), np.cos(rad).mean())) % 360


def _ang_diff(a, b):
    return abs((a - b + 180) % 360 - 180)


class ChangeDetector:
    def __init__(self, baseline):
        self.baseline = baseline
        self.tracks = []  # each: {bearing, count, emitted}

    def blobs(self, ranges):
        base = self.baseline
        valid = ~np.isnan(ranges)
        closer = valid & ~np.isnan(base) & (ranges < base - config.CHANGE_THRESH_M)
        new_obj = valid & np.isnan(base)
        mask = closer | new_obj
        out = []
        for run in _circular_runs(mask):
            if len(run) < config.CHANGE_MIN_BINS:
                continue
            r = float(np.nanmedian(ranges[run]))
            b = _circ_mean_deg(run)
            x, y = polar_to_xy(b, r)
            width = len(run)
            out.append({"bearing_deg": b, "range_m": r, "x": x, "y": y,
                        "width_deg": width, "size_m": r * math.radians(width)})
        return out

    def update(self, ranges):
        """Returns blobs that just became persistent (emit each once while it stays)."""
        found = self.blobs(ranges)
        new_tracks, emit = [], []
        for blob in found:
            match = next((t for t in self.tracks if _ang_diff(t["bearing"], blob["bearing_deg"]) < 6), None)
            t = match or {"bearing": blob["bearing_deg"], "count": 0, "emitted": False}
            t["bearing"] = blob["bearing_deg"]
            t["count"] += 1
            if t["count"] >= config.CHANGE_PERSIST_SCANS and not t["emitted"]:
                t["emitted"] = True
                emit.append(blob)
            new_tracks.append(t)
        self.tracks = new_tracks  # blobs that vanished are forgotten, so they can re-trigger later
        return emit
