#!/usr/bin/env python3
"""Camera-to-rover navigation server (controller Pi). One page: select target, Arm, Start, STOP, live progress.

  python3 server.py --zero http://100.96.176.12:8780 --dry-run                (no rover packets)
  python3 server.py --zero http://100.96.176.12:8780 --enable-motion          (real rover; still needs Arm in the UI)
  python3 server.py --sim --dry-run                                           (synthetic world, no hardware)

Binds 127.0.0.1 only; reach it through an SSH tunnel. POSTs need the X-Requested-With: nav header.
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "rover"))
import control as C  # noqa: E402
import perception as P  # noqa: E402
import vision as V  # noqa: E402
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # noqa: E402


class App:
    def __init__(self, a):
        self.a = a
        self.dir = os.path.expanduser(a.state_dir)
        os.makedirs(self.dir, exist_ok=True)
        self.per = P.Perception(self.dir)
        if a.sim:
            import sim
            self.world = sim.SimWorld(obstacles_mm=[(520.0, 250.0, 580.0, 550.0, 120.0)], rover=(250.0, 400.0, 0.0),
                                      rot_sign=-1)
            self.src, rover = self.world, sim.SimRover(self.world)
            self.sim_dir = self.dir
        else:
            from sources import RemoteFrames
            import rover_control as rc
            self.src = RemoteFrames(a.zero)
            rover = rc.RoverLink(dry_run=a.dry_run)
        self.rover = rover
        cfg = {"radius": a.radius, "margin": a.margin, "margin_mm": a.margin_mm}
        if a.rover_mm:
            cfg["rover_w_mm"], cfg["rover_l_mm"] = [float(x) for x in a.rover_mm.lower().split("x")]
        self.nav = C.Navigator(self.per, self.src, rover, self.dir, cfg)
        self.motion_enabled = a.enable_motion
        self.dry = a.dry_run or a.sim
        self.view_jpg, self.view_seq = b"", -1
        self.cam_jpg, self.cam_seq = b"", -1
        self.rec = None
        self.lock = threading.Lock()
        threading.Thread(target=self._bg, daemon=True).start()

    # keeps calibration/map/pose display fresh while idle (no motion is ever commanded from here)
    def _bg(self):
        last = -1
        while True:
            try:
                if self.nav.state != "running":
                    fr = self.src.latest()
                    if fr is not None and fr.seq != last:
                        last = fr.seq
                        obs = self.per.process(fr)
                        if obs.pose:
                            self.nav.pose = obs.pose
                        if self.rec:
                            self._record(fr)
                    elif fr is None or fr.seq == last:
                        time.sleep(0.25)
                else:
                    time.sleep(0.3)
                if self.a.sim:
                    time.sleep(0.5)
            except Exception as e:  # never let the display thread die
                self.nav._log("bg error: %s" % e)
                time.sleep(1)

    def _record(self, fr):
        if self.rec["last"] == fr.seq or fr.seq % self.rec["every"]:
            return
        self.rec["last"] = fr.seq
        np.savez_compressed(os.path.join(self.rec["dir"], "f%08d.npz" % fr.seq), depth_mm=fr.depth, rgb_bgr=fr.bgr,
                            wall=fr.wall, K=np.array(fr.K))
        self.rec["n"] += 1

    # ---------------------------------------------------------------- state
    def state(self):
        n, per = self.nav, self.per
        c = per.calib
        fr = self.src.latest()
        obs = per.last_obs
        pose = obs.pose if obs else None
        res = n.preview() if (n.target and pose and c.H is not None) else None
        age = None if fr is None else round(time.time() - getattr(fr, "rx", fr.wall), 1) if not self.a.sim else 0.0
        return {
            "frame": {"seq": fr.seq if fr else None, "age_s": age, "err": getattr(self.src, "err", ""),
                      "fetch_ms": getattr(self.src, "fetch_ms", 0), "size": [fr.cols, fr.rows] if fr else None},
            "calib": {"boundary_live": bool(obs and obs.boundary_live), "boundary_stored": c.corners is not None,
                      "plane": c.plane is not None, "aspect": round(c.aspect, 3), "aspect_measured": c.aspect_measured,
                      "plane_d_mm": None if c.plane is None else round(c.plane[1], 1)},
            "markers": sorted(per.last_markers.keys()), "manual_markers": sorted(per.manual_markers.keys()),
            "pose": pose,
            "target": n.target,
            "path": [list(p) for p in (n.path if n.state == "running" else (res["waypoints"] if res else []))],
            "plan_error": (res or {}).get("error", ""),
            "nav": {"state": n.state, "reason": n.reason, "step": n.step, "dist": n.dist, "armed": n.armed,
                    "model": n.model.to_dict()},
            "motion_enabled": self.motion_enabled, "dry_run": self.dry,
            "cfg": dict({k: n.cfg[k] for k in ("cols", "rows", "rover_w_mm", "rover_l_mm", "margin_mm")},
                        radius=n.clearance()[0], margin=n.clearance()[1], height_mm=c.height_mm),
            "depth_cells": self._cells(),
            "manual": [list(r) for r in per.manual],
            "warnings": per.warnings,
            "log": n.log[-14:],
            "recording": None if not self.rec else {"n": self.rec["n"], "dir": self.rec["dir"]},
            "map_frames": per.map_frames,
        }

    def _cells(self):
        dm = self.per.depth_map
        if self.per.calib.plane is None:
            return {"cols": dm.cols, "rows": dm.rows, "cells": [], "unknown": []}
        d = dm.to_dict()
        d.pop("seen", None)
        d["unknown"] = [[c, r] for c, r in dm.unknown_cells()]
        return d

    def view(self):
        fr = self.src.latest()
        if fr is None or self.per.calib.H is None:
            return None
        if fr.seq != self.view_seq:
            flat = V.flatten_view(fr.bgr, self.per.calib)
            self.view_jpg = cv2.imencode(".jpg", flat, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.view_seq = fr.seq
        return self.view_jpg

    def camera(self):
        """Raw RGB beside a COLOURISED depth image (display only; numeric depth is never taken from this)."""
        fr = self.src.latest()
        if fr is None:
            return None
        if fr.seq != self.cam_seq:
            d = fr.depth.astype(np.float32)
            vis = np.zeros(d.shape, np.uint8)
            ok = d > 0
            if ok.any():
                lo, hi = np.percentile(d[ok], 2), np.percentile(d[ok], 98)
                vis[ok] = np.clip((d[ok] - lo) / max(hi - lo, 1) * 255, 0, 255)
            col = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
            col[~ok] = 0
            img = np.hstack([fr.bgr, col])
            for k, m in self.per.last_markers.items():
                cv2.polylines(img, [m.astype(np.int32)], True, (0, 255, 0), 1)
            self.cam_jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
            self.cam_seq = fr.seq
        return self.cam_jpg

    # ---------------------------------------------------------------- actions
    def post(self, path, d):
        n, per = self.nav, self.per
        if path == "/api/target":
            if n.state == "running":
                return 409, {"error": "stop before changing the target"}
            n.target = (float(d["x"]), float(d["y"]))
            n.state, n.reason = ("idle", "") if n.state in ("done", "failed") else (n.state, n.reason)
            return 200, {"ok": True}
        if path == "/api/arm":
            if d.get("armed") and not self.motion_enabled:
                return 403, {"error": "server started without --enable-motion"}
            if d.get("armed") and (not per.last_obs or not per.last_obs.pose):
                return 409, {"error": "cannot arm: rover marker not currently seen"}
            n.armed = bool(d.get("armed"))
            n._log("armed" if n.armed else "disarmed")
            return 200, {"armed": n.armed}
        if path == "/api/start":
            if n.target is None:
                return 409, {"error": "select a target first"}
            if per.calib.plane is None or per.calib.H is None:
                return 409, {"error": "calibration incomplete (boundary markers / ground plane)"}
            ok, msg = n.start(n.target)
            return (200 if ok else 409), {"ok": ok, "msg": msg}
        if path == "/api/stop":
            ok = n.stop()
            return (200 if ok else 502), {"stop_sent": ok}
        if path == "/api/manual":
            if n.state == "running":
                return 409, {"error": "stop first"}
            per.manual = [tuple(max(0.0, min(1.0, float(v))) for v in r) for r in d.get("rects", [])][:100]
            per.save()
            return 200, {"ok": True}
        if path == "/api/config":
            if n.state == "running":
                return 409, {"error": "stop first"}
            for k, hi in (("radius", 0.5), ("margin", 0.5), ("rover_w_mm", 600), ("rover_l_mm", 600), ("margin_mm", 200)):
                if k in d and d[k] is not None:
                    n.cfg[k] = max(0.0, min(hi, float(d[k])))
            return 200, {"ok": True}
        if path == "/api/recal_ground":
            if n.state == "running":
                return 409, {"error": "stop first"}
            per.want_ground_refit = True
            return 200, {"ok": True}
        if path == "/api/clear_map":
            if n.state == "running":
                return 409, {"error": "stop first"}
            per.clear_depth_map()
            return 200, {"ok": True}
        if path == "/api/record":
            if d.get("on"):
                rd = os.path.join(self.dir, "rec_" + time.strftime("%Y%m%d_%H%M%S"))
                os.makedirs(rd, exist_ok=True)
                json.dump(self.per.calib.to_dict(), open(os.path.join(rd, "calibration.json"), "w"))
                self.rec = {"dir": rd, "n": 0, "last": -1, "every": max(1, int(d.get("every", 1)))}
            else:
                self.rec = None
            return 200, {"ok": True}
        return 404, {"error": "not found"}


def make_handler(app):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ct="application/json"):
            if not isinstance(body, (bytes, bytearray)):
                body = json.dumps(body, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)).encode()
            self.send_response(code)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split("?")[0]
            if p in ("/", "/index.html"):
                return self._send(200, open(os.path.join(HERE, "nav.html"), "rb").read(), "text/html; charset=utf-8")
            if p == "/api/state":
                return self._send(200, app.state())
            if p == "/api/view.jpg":
                b = app.view()
                return self._send(200, b, "image/jpeg") if b else self._send(503, {"error": "no frame/calibration"})
            if p == "/api/camera.jpg":
                b = app.camera()
                return self._send(200, b, "image/jpeg") if b else self._send(503, {"error": "no frame"})
            return self._send(404, {"error": "not found"})

        def do_POST(self):
            if self.headers.get("X-Requested-With") != "nav":
                return self._send(403, {"error": "forbidden"})
            n = int(self.headers.get("Content-Length", 0))
            if n > 200000:
                return self._send(413, {"error": "too large"})
            try:
                d = json.loads(self.rfile.read(n) or b"{}")
                code, body = app.post(self.path.split("?")[0], d)
            except Exception as e:
                code, body = 400, {"error": str(e)}
            self._send(code, body)

    return H


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--zero", default="http://100.96.176.12:8780")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--state-dir", default="~/forge-nav")
    ap.add_argument("--enable-motion", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--radius", type=float, default=0.09, help="rover enclosing radius, sandbox-height units")
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--rover-mm", default=None, help="rover footprint WxL in mm, e.g. 200x170")
    ap.add_argument("--margin-mm", type=float, default=None, help="extra clearance in mm")
    a = ap.parse_args()
    app = App(a)
    print("navigator: http://127.0.0.1:%d  sim=%s dry_run=%s motion=%s" % (a.port, a.sim, app.dry, "ENABLED-for-arming" if a.enable_motion else "OFF"), flush=True)
    try:
        ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(app)).serve_forever()
    finally:
        app.nav.stop()
