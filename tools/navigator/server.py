#!/usr/bin/env python3
"""Camera-to-rover navigation server (controller Pi). One page: click a destination on the live image or the ground
map, see the trajectory, Arm, Start, STOP, and watch progress.

  python3 server.py --zero http://100.96.176.12:8780 --enable-motion --rover-mm 170x170 --margin-mm 15
  python3 server.py --zero ... --dry-run          (UGV commands logged, not sent)
  python3 server.py --sim --dry-run               (synthetic world, no hardware)

Binds 127.0.0.1 only; reach it through an SSH tunnel. POSTs need the X-Requested-With: nav header.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import control as C  # noqa: E402
import geom  # noqa: E402
import mission as M  # noqa: E402
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
                                      rot_sign=-1, diff=True)
            self.src, rover = self.world, sim.SimRover(self.world)
        else:
            from sources import RemoteFrames
            from ugv import UgvLink
            self.src = RemoteFrames(a.zero)
            self.src.hint = lambda: self.per.track          # small rover-window frames while the rover is tracked
            rover = UgvLink(a.ugv_host, a.local_ip, a.ugv_speed, a.ugv_turn, dry_run=a.dry_run)
        self.rover = rover
        cfg = {"radius": a.radius, "margin": a.margin, "margin_mm": a.margin_mm, "arrive_tol_mm": a.arrive_tol_mm,
               "cruise": not a.no_cruise}
        if a.rover_mm:
            cfg["rover_w_mm"], cfg["rover_l_mm"] = [float(x) for x in a.rover_mm.lower().split("x")]
        self.nav = C.Navigator(self.per, self.src, rover, self.dir, cfg)
        self.motion_enabled = a.enable_motion
        self.mission = M.Mission(self.nav)
        self.dry = a.dry_run or a.sim
        self.view_jpg, self.view_seq = b"", -1
        self.cam_jpg, self.cam_seq = b"", -1
        self.live_jpg, self.live_key = b"", None
        self._prev = (None, None)                       # (key, result): preview route cache
        self._cells_cache = (None, None)
        self._state_cache = (0.0, None)
        self.rec = None
        threading.Thread(target=self._bg, daemon=True).start()

    # keeps calibration/map/pose display fresh while idle (no motion is ever commanded from here)
    def _bg(self):
        last = -1
        last_full = -1
        while True:
            try:
                if self.nav.state != "running":
                    fr = self.src.latest()
                    # The map is built from the latest FULL frame, looked up separately: a small rover-window frame usually
                    # arrives ~0.1 s after a full one, so keying on latest() alone missed full frames (an explicit refresh
                    # or the once-a-minute update could be skipped until the next full frame, a minute later).
                    full = self._full()
                    if full is not None and not getattr(full, "partial", False) and full.seq != last_full and full.seq != last \
                            and self._map_due():
                        last_full = last = full.seq
                        obs = self.per.snapshot(full)            # overview refresh: map from this full frame
                        if obs.pose:
                            self.nav.pose = obs.pose
                        if self.rec:
                            self._record(full)
                    elif fr is not None and fr.seq != last:
                        last = fr.seq
                        if not getattr(fr, "partial", False):
                            last_full = fr.seq
                        obs = self.per.process(fr, update_map=False)
                        if obs.pose:
                            self.nav.pose = obs.pose
                        if self.rec:
                            self._record(fr)
                    else:
                        time.sleep(0.25)
                else:
                    time.sleep(0.3)
                if self.a.sim:
                    time.sleep(0.5)
            except Exception as e:  # never let the display thread die
                self.nav._log("bg error: %s" % e)
                time.sleep(1)

    def _map_due(self):
        """Idle display: fold a full frame into the obstacle map at most about once a minute (not per frame)."""
        now = time.time()
        if now - getattr(self, "_map_t", 0.0) >= 55.0 or self.per.map_frames == 0:
            self._map_t = now
            return True
        return False

    def _record(self, fr):
        if self.rec["last"] == fr.seq or fr.seq % self.rec["every"]:
            return
        self.rec["last"] = fr.seq
        np.savez_compressed(os.path.join(self.rec["dir"], "f%08d.npz" % fr.seq), depth_mm=fr.depth, rgb_bgr=fr.bgr,
                            wall=fr.wall, K=np.array(fr.K))
        self.rec["n"] += 1

    # ---------------------------------------------------------------- state
    def sweep(self, path):
        """Footprint outlines along the trajectory (rectangle aligned with each segment) and the turning circle at each
        vertex, in normalized sandbox coordinates, for display."""
        n, c = self.nav, self.per.calib
        if len(path) < 2 or not c.height_mm:
            return {"rects": [], "turns": []}
        L, W, m = n.footprint()
        A = c.aspect
        rects, turns = [], []
        for a, b in zip(path, path[1:]):
            du, dv = (b[0] - a[0]) * A, b[1] - a[1]
            ln = math.hypot(du, dv)
            if ln < 1e-6:
                continue
            th = math.atan2(dv, du)
            k = max(1, int(ln / 0.3))
            for i in range(k + 1):
                f = i / k
                rects.append([a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, th])
        for p in path[1:-1]:
            turns.append([p[0], p[1]])
        return {"rects": rects, "turns": turns, "L": L, "W": W, "margin": m, "turn_r": math.hypot(L, W) / 2 + m}

    def state(self):
        n, per = self.nav, self.per
        c = per.calib
        fr = self.src.latest()
        obs = per.last_obs
        pose = obs.pose if obs else None
        res = self._preview(pose) if (n.target and pose and c.H is not None) else None
        path = [list(p) for p in (n.path if n.state == "running" else (res["waypoints"] if res else []))]
        age = None if fr is None else round(time.time() - getattr(fr, "rx", fr.wall), 1) if not self.a.sim else 0.0
        hm = c.height_mm
        t = n.target
        tinfo = None
        if t and c.aspect and hm:
            tinfo = {"x": t[0], "y": t[1], "mm_from_left": round(t[0] * c.aspect * hm), "mm_from_bottom": round(t[1] * hm)}
        tm = n.timing
        return {
            "frame": {"seq": fr.seq if fr else None, "age_s": age, "err": getattr(self.src, "err", ""),
                      "fetch_ms": getattr(self.src, "fetch_ms", 0), "size": [fr.cols, fr.rows] if fr else None},
            "map": {"seq": per.map_seq, "age_s": round(time.time() - per.map_time, 1) if per.map_time else None,
                    "updates": per.map_frames, "err": per.map_err, "frame_seq": fr.seq if fr else None},
            "calib": {"boundary_live": bool(obs and obs.boundary_live), "boundary_stored": c.corners is not None,
                      "plane": c.plane is not None, "aspect": round(c.aspect, 3), "aspect_measured": c.aspect_measured,
                      "height_mm": hm},
            "markers": sorted(per.last_markers.keys()), "manual_markers": sorted(per.manual_markers.keys()),
            "pose": pose, "pose_source": (pose or {}).get("source"),
            "target": t, "target_info": tinfo,
            "path": path, "sweep": self.sweep(path),
            "mission": self._mission_state(),
            "plan_error": (res or {}).get("error", ""),
            "plan_warning": ("" if n.state == "running" or not res or res.get("error") else
                             (lambda g: ("route drawn, but the guard would stop its first move: " + g + " (the run would stop, refresh the map "
                                         "and try one guarded alternative)") if g else "")(n.first_move_check(pose, res))),
            "nav": {"state": n.state, "reason": n.reason, "step": n.step, "dist": n.dist, "armed": n.armed,
                    "cruising": n.cruising, "remaining": n.remaining, "arrive_tol_mm": n.cfg["arrive_tol_mm"],
                    "latency_s": n.latency_s,
                    "avg_fetch_ms": round(tm["fetch_ms"] / tm["frames"]) if tm["frames"] else None,
                    "run_s": (round(time.time() - n.run_info["t_start"], 1) if n.state == "running" and n.run_info else
                              (round(n.run_info["t_end"] - n.run_info["t_start"], 1) if n.run_info.get("t_end") else None))},
            "motion_enabled": self.motion_enabled, "dry_run": self.dry,
            "cfg": dict({k: n.cfg[k] for k in ("cols", "rows", "rover_w_mm", "rover_l_mm", "margin_mm")},
                        radius=n.clearance()[0], margin=n.clearance()[1], height_mm=hm),
            "depth_cells": self._cells(),
            "manual": [list(r) for r in per.manual],
            "warnings": per.warnings,
            "log": n.log[-16:],
            "recording": None if not self.rec else {"n": self.rec["n"], "dir": self.rec["dir"]},
            "perf": self.perf(),
        }

    def _mission_state(self):
        try:
            self.mission.refresh_preview()          # idle: keep the structure list (stable labels) current
        except Exception as e:
            self.nav._log("structure preview error: %s" % e)
        return self.mission.snapshot()

    def perf(self):
        st = getattr(self.src, "stats", None)
        out = {"timing": {k: (round(v, 1) if isinstance(v, float) else v) for k, v in self.nav.timing.items()},
               "perception_ms": round(self.per.stats["ms"] / self.per.stats["frames"], 1) if self.per.stats["frames"] else None}
        out["map_refreshes"] = [[time.strftime("%H:%M:%S", time.localtime(t)), q, bool(b)] for t, q, b in self.nav.map_log]
        if st:
            out["frames"] = {"full": st["full"], "roi": st["roi"], "kbytes": round(st["bytes"] / 1024),
                             "full_ms": round(st["full_ms"] / st["full"]) if st["full"] else None,
                             "roi_ms": round(st["roi_ms"] / st["roi"]) if st["roi"] else None,
                             "last_kb": round(getattr(self.src, "nbytes", 0) / 1024, 1)}
        return out

    def _preview(self, pose):
        n = self.nav
        key = (n.target, round(pose["x"], 2), round(pose["y"], 2), round(pose.get("theta") or 0, 1), self.per.map_frames,
               n.cfg["rover_w_mm"], n.cfg["margin_mm"], n.p.calib.height_mm)
        if self._prev[0] != key:
            self._prev = (key, n.preview())
        return self._prev[1]

    def _cells(self):
        dm = self.per.depth_map
        if self.per.calib.plane is None:
            return {"cols": dm.cols, "rows": dm.rows, "cells": [], "unknown": []}
        key = (self.per.map_frames, self.per.calib.plane is not None)
        if self._cells_cache[0] == key:
            return self._cells_cache[1]
        d = dm.to_dict()
        d.pop("seen", None)
        d["unknown"] = [[c, r] for c, r in dm.unknown_cells()]
        self._cells_cache = (key, d)
        return d

    # ---------------------------------------------------------------- images
    def _full(self):
        f = getattr(self.src, "latest_full", None)
        return f() if f else self.src.latest()

    def view(self):
        fr = self._full()
        if fr is None or self.per.calib.H is None:
            return None
        if fr.seq != self.view_seq:
            flat = V.flatten_view(fr.bgr, self.per.calib)
            self.view_jpg = cv2.imencode(".jpg", flat, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.view_seq = fr.seq
        return self.view_jpg

    def live(self):
        """Raw RGB with the boundary, route, target and rover drawn on it, in IMAGE pixel coordinates so a click maps
        straight through the ground homography."""
        fr = self._full()
        if fr is None:
            return None
        n, c = self.nav, self.per.calib
        key = (fr.seq, n.target, len(n.path), n.state, self.mission.overlay_key())
        if key != self.live_key:
            img = fr.bgr.copy()
            if c.corners is not None:
                cv2.polylines(img, [c.corners.astype(np.int32)], True, (0, 255, 0), 1)
            if c.H is not None:
                path = n.path if n.state == "running" else ((self._preview(n.pose) or {}).get("waypoints") if n.target and n.pose else [])
                if path:
                    pts = c.to_px(path).astype(np.int32)
                    cv2.polylines(img, [pts], False, (255, 200, 0), 2)
                if n.target:
                    tp = c.to_px([n.target])[0]
                    cv2.circle(img, (int(tp[0]), int(tp[1])), 7, (0, 255, 0), 2)
                    cv2.line(img, (int(tp[0]) - 11, int(tp[1])), (int(tp[0]) + 11, int(tp[1])), (0, 255, 0), 1)
                    cv2.line(img, (int(tp[0]), int(tp[1]) - 11), (int(tp[0]), int(tp[1]) + 11), (0, 255, 0), 1)
            self._draw_structures(img)
            self.live_jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
            self.live_key = key
        return self.live_jpg

    COL = {"pending": (255, 255, 255), "visiting": (255, 200, 0), "collecting": (0, 200, 255), "complete": (0, 200, 0),
           "unreachable": (0, 0, 255), "cancelled": (150, 150, 150), "failed": (0, 0, 255)}

    def _draw_structures(self, img):
        """Structure labels + status on the live feed; the active visit gets the placeholder banner with the countdown.
        Text only: no readings and no detection result exist in this phase."""
        m, c = self.mission, self.per.calib
        if c.H is None:
            return
        for s in list(m.book.items):
            col = self.COL.get(s.status, (255, 255, 255))
            x, y = [int(v) for v in c.to_px([(s.cx, s.cy)])[0]]
            cv2.circle(img, (x, y), 15, col, 2)
            for t, th, cc in ((s.label + " " + {"pending": "", "visiting": "-> going", "collecting": "collecting", "complete": "visited",
                               "unreachable": "unreachable", "cancelled": "", "failed": "failed"}.get(s.status, ""), 1, col),):
                cv2.putText(img, t.strip(), (x - 18, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                cv2.putText(img, t.strip(), (x - 18, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cc, 1)
        act = next((s for s in m.book.items if s.id == m.active and s.status == "collecting"), None)
        if act is not None and m.countdown is not None:
            t = "%s - Collecting - placeholder - %d s" % (act.label, int(math.ceil(m.countdown)))
            (w, h), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.rectangle(img, (6, 6), (w + 18, h + 22), (0, 0, 0), -1)
            cv2.putText(img, t, (12, h + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    def camera(self):
        """Raw RGB beside a COLOURISED depth image (display only; numeric depth is never taken from this)."""
        fr = self._full()
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
            self.cam_jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])[1].tobytes()
            self.cam_seq = fr.seq
        return self.cam_jpg

    # ---------------------------------------------------------------- actions
    def set_target(self, d):
        """Exact point from the map ({x,y} normalized) or the live image ({px,py} image pixels). Validated for the whole
        rover; never silently moved. While running this is a re-target: stop, replan from the observed pose, go."""
        n, c = self.nav, self.per.calib
        if self.mission.running():
            return 409, {"ok": False, "reason": "a mission is running: STOP it first"}
        if "px" in d:
            if c.H is None:
                return 409, {"ok": False, "reason": "calibration incomplete"}
            x, y = [float(v) for v in c.to_norm([[float(d["px"]), float(d["py"])]])[0]]
        else:
            x, y = float(d["x"]), float(d["y"])
        v = n.validate_target(x, y)
        info = dict(x=x, y=y, **{k: v[k] for k in ("reason", "suggestion")})
        if not v["ok"]:
            return 409, dict(ok=False, **info)
        # the route must exist from the observed pose
        if n.pose and c.H is not None:
            res = n.make_plan((n.pose["x"], n.pose["y"]), (x, y))
            if res["error"]:
                return 409, dict(ok=False, x=x, y=y, reason=res["error"], suggestion=None)
        if n.state == "running":
            if not n.armed:
                return 409, {"ok": False, "reason": "not armed"}
            ok, msg = n.retarget((x, y))
            return (200 if ok else 409), {"ok": ok, "x": x, "y": y, "msg": msg, "retargeted": True}
        n.target = (x, y)
        if hasattr(self.src, "request_full"):
            self.src.request_full()                # fresh overhead snapshot for the preview
            self._map_t = 0.0
        if n.state in ("done", "failed"):
            n.state, n.reason = "idle", ""
        return 200, {"ok": True, "x": x, "y": y, "reason": "", "suggestion": None}

    def post(self, path, d):
        n, per = self.nav, self.per
        if path == "/api/target":
            return self.set_target(d)
        if path == "/api/arm":
            if d.get("armed") and not self.motion_enabled:
                return 403, {"error": "server started without --enable-motion"}
            if d.get("armed") and (not per.last_obs or not per.last_obs.pose):
                return 409, {"error": "cannot arm: rover not currently tracked"}
            n.armed = bool(d.get("armed"))
            if not n.armed:
                self.mission.cancel("disarmed by operator")
            n._log("armed" if n.armed else "disarmed")
            return 200, {"armed": n.armed}
        if path == "/api/start":
            if self.mission.running():
                return 409, {"error": "a mission is running"}
            if n.target is None:
                return 409, {"error": "select a target first"}
            if per.calib.plane is None or per.calib.H is None:
                return 409, {"error": "calibration incomplete (boundary markers / ground plane)"}
            ok, msg = n.start(n.target)
            return (200 if ok else 409), {"ok": ok, "msg": msg}
        if path == "/api/mission/start":
            ok, msg = self.mission.start(d.get("dwell_s"), d.get("gap_mm"))
            return (200 if ok else 409), {"ok": ok, "msg": msg}
        if path == "/api/stop" or path == "/api/mission/stop":
            self.mission.cancel("stopped by operator")
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
        if path == "/api/refresh":
            if hasattr(self.src, "request_full"):
                self.src.request_full()
            self._map_t = 0.0
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
            for name, fn in (("/api/view.jpg", app.view), ("/api/live.jpg", app.live), ("/api/camera.jpg", app.camera)):
                if p == name:
                    b = fn()
                    return self._send(200, b, "image/jpeg") if b else self._send(503, {"error": "no frame/calibration"})
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
    ap.add_argument("--ugv-host", default="192.168.4.1")
    ap.add_argument("--local-ip", default="192.168.4.2")
    ap.add_argument("--ugv-speed", type=float, default=0.35)
    ap.add_argument("--ugv-turn", type=float, default=0.5)
    ap.add_argument("--radius", type=float, default=0.09, help="fallback rover enclosing radius, sandbox-height units")
    ap.add_argument("--margin", type=float, default=0.02)
    ap.add_argument("--rover-mm", default=None, help="rover footprint WxL in mm, e.g. 170x170")
    ap.add_argument("--margin-mm", type=float, default=None, help="extra clearance in mm")
    ap.add_argument("--arrive-tol-mm", type=float, default=50.0, help="arrival tolerance (centre to target), mm")
    ap.add_argument("--no-cruise", action="store_true", help="disable streamed cruise (bounded bursts only)")
    a = ap.parse_args()
    app = App(a)
    print("navigator: http://127.0.0.1:%d  sim=%s dry_run=%s motion=%s" % (a.port, a.sim, app.dry, "ENABLED-for-arming" if a.enable_motion else "OFF"), flush=True)
    try:
        ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(app)).serve_forever()
    finally:
        app.nav.stop()
