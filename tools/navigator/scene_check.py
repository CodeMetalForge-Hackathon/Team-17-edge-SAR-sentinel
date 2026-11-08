#!/usr/bin/env python3
"""READ-ONLY check of the scene-change detector against the REAL camera (run on the controller Pi, rover untouched).

  python3 scene_check.py [N=8] [gap_s=3] [--zero http://100.96.176.12:8780] [--state ~/forge-nav]

Fetches N consecutive full scenes, compares each with the previous valid one using a COPY of the saved calibration (the live
state directory is never written), prints per-scene timing / noise / candidates, and then injects a synthetic object into a real
frame (depth + colour) to confirm that real sensor noise does not hide a genuine change. Commands to the rover: none."""
import argparse
import os
import shutil
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import perception as P  # noqa: E402
import scene as SC  # noqa: E402
import sources  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("n", nargs="?", type=int, default=8)
    ap.add_argument("gap", nargs="?", type=float, default=3.0)
    ap.add_argument("--zero", default="http://100.96.176.12:8780")
    ap.add_argument("--state", default="~/forge-nav")
    ap.add_argument("--rover-mm", default="170x170")
    a = ap.parse_args()
    d = tempfile.mkdtemp()
    for f in ("calib.json", "manual_markers.json", "manual.json", "depthmap.json"):
        p = os.path.join(os.path.expanduser(a.state), f)
        if os.path.exists(p):
            shutil.copy(p, d)
    per = P.Perception(d)
    L, W = [float(x) for x in a.rover_mm.lower().split("x")]
    src = sources.RemoteFrames(a.zero)
    watch = SC.SceneWatch(per)
    bad, rows, last = 0, [], None
    for i in range(a.n + 3):
        t_req = time.time()
        fr = src.fresh_full(12.0)
        if fr is None:
            print("scene %d: no full frame (%s)" % (i, src.err))
            bad += 1
            continue
        t_rx = fr.rx
        ob = per.process(fr, update_map=False)
        pose = ob.pose
        conf = pose is not None and (pose.get("source") == "marker" or pose.get("verified", True) or not per._pattern_seen)
        injected = ""
        if i >= a.n and watch.baseline is not None:           # synthetic object in a REAL frame (last 3 scenes)
            spot = getattr(main, "spot", None)
            if spot is None:
                spot = main.spot = free_spot(watch.baseline, per)
            if spot is not None and i >= a.n + 1:
                x, y = spot
                fr.depth[y - 12:y + 12, x - 12:x + 12] = np.maximum(fr.depth[y - 12:y + 12, x - 12:x + 12].astype(np.int32) - 90, 1).astype(np.uint16)
                fr.bgr[y - 12:y + 12, x - 12:x + 12] = (fr.bgr[y - 12:y + 12, x - 12:x + 12] * 0.4).astype(np.uint8)
                injected = " [SYNTHETIC object at px %s]" % (spot,)
        t0 = time.time()
        cur, res = watch.observe(fr, pose, conf, (L, W))
        dt = (time.time() - t0) * 1000
        gap = "" if last is None else " +%.1fs" % (t_rx - last)
        last = t_rx
        print("scene %d seq %s rx %s%s rover=%s status=%s%s  cands=%s  %.0f ms%s" % (
            i, fr.seq, time.strftime("%H:%M:%S", time.localtime(t_rx)), gap, bool(conf), res.status, (" (" + res.why + ")") if res.why else "",
            [(c["type"], "(%.3f,%.3f) area=%d dh=%.0f h_ref=%s" % (tuple((c.get("to") or c.get("from"))["xy"]) + ((c.get("to") or c.get("from"))["area"], (c.get("to") or c.get("from"))["dh_mm"], (c.get("to") or c.get("from"))["h_ref"] and round((c.get("to") or c.get("from"))["h_ref"])))) for c in res.cands], dt, injected))
        if res.status == "ok":
            s = {k: v for k, v in res.stats.items() if k in ("T_mm", "noise_p999_mm", "level_shift_mm", "rgb_L_thr", "rgb_C_thr", "depth_changed_frac", "rgb_changed_frac", "gain", "deferred_px")}
            print("         ", s)
        rows.append((i, res, bool(injected)))
        if i < a.n + 2:
            time.sleep(max(0.0, a.gap - (time.time() - t_req)) if i < a.n - 1 or True else 0)
    clean = [r for i, r, inj in rows if not inj and r.status == "ok"]
    false_ev = sum(len([c for c in r.cands if c["type"] != "removed"]) for r in clean)
    inj = [r for i, r, inj in rows if inj and r.status == "ok"]
    print("\nclean scenes compared: %d, false events: %d, calibration/invalid scenes: %d" % (len(clean), false_ev, sum(1 for _, r, x in rows if not x and r.status != "ok")))
    if inj:
        print("synthetic object: first scene -> %s" % [c["type"] for c in inj[0].cands], "; unchanged next scene -> %s" % ([c["type"] for c in inj[1].cands] if len(inj) > 1 else "n/a"))
    return 0 if false_ev == 0 else 1


def free_spot(base, per, win=14):
    """A patch of bare table (|h| small, valid depth, away from the rover mask and from anything raised)."""
    h = base.h
    ok = np.isfinite(h) & (np.abs(h) < 12) & base.roi & ~base.mask
    import cv2
    ok = cv2.erode(ok.astype(np.uint8), np.ones((2 * win + 1, 2 * win + 1), np.uint8)).astype(bool)
    ys, xs = np.nonzero(ok)
    if len(ys) == 0:
        return None
    k = len(ys) // 2
    return int(xs[k]), int(ys[k])


if __name__ == "__main__":
    sys.exit(main())
