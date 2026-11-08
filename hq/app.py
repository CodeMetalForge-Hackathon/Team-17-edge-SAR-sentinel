#!/usr/bin/env python3
"""HQ app (runs on the khoj Pi): one page to take the aerial photo, see the map + route, and send the rover.

    python3 hq/app.py --enable-motion                  # real: Zero camera server + rover
    python3 hq/app.py --sim                            # simulated camera + rover (testing / demo fallback)

Open http://<khoj pi>:8080  (e.g. http://100.119.240.31:8080 over Tailscale, also from the iPad).

Page: Get photo (instant, from the Zero's camera server) -> objects, rover, blocked squares and route are drawn.
Click a square to set the TARGET (or switch to "rover is here" and click the rover). GO drives there on its own,
taking a new photo after every burst. STOP (button, Space or Esc) stops at once.
"""
import argparse
import collections
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from blob_pilot import BlobPilot, Stop
from vision import PX_PER_M


class App:
    def __init__(self, camera, rover, size, grid, motion_allowed, rover_radius=0.07, margin=0.03):
        self.rover = rover
        self.motion_allowed = motion_allowed
        self.logs = collections.deque(maxlen=80)
        self.lock = threading.Lock()          # one photo/plan action at a time
        self.running, self.stop_flag = False, False
        self.status = "idle"
        self.pilot = BlobPilot(camera, rover, size=size, grid=grid, rover_radius=rover_radius, margin=margin,
                               log=self.log, should_stop=lambda: self.stop_flag)
        self.last_photo_t = None

    def log(self, msg):
        line = time.strftime("%H:%M:%S ") + msg
        self.logs.appendleft(line)
        print("[hq]", msg, flush=True)

    # ------------------------------------------------------------------ actions
    def get_photo(self):
        if self.running:
            raise RuntimeError("the rover is driving; photos are being taken automatically")
        with self.lock:
            t0 = time.time()
            self.pilot.look(lenient=True)
            self.last_photo_t = time.time()
            s = self.pilot.summary()
        self.log("photo (%.1f s): %d object(s), rover %s, blocked %s" % (
            time.time() - t0, s["objects"], s["rover"] or "unknown", " ".join(s["blocked"]) or "none"))

    def _need_map(self):
        if self.running:
            raise RuntimeError("the rover is driving: press STOP first")
        if self.pilot.top is None:
            raise RuntimeError("take a photo first")

    def set_target(self, cell):
        self._need_map()
        with self.lock:
            self.pilot.set_target(cell)
        self.log("target %s, route: %s" % (cell, " > ".join(self.pilot.summary()["path"] or ["none yet"])))

    def set_rover(self, cell):
        self._need_map()
        with self.lock:
            self.pilot.set_rover_hint(cell)
        self.log("rover is at %s" % cell)

    def recalibrate(self):
        self.pilot.vel = {}
        self.log("motion calibration cleared: GO will do the 2 test bursts again")

    def go(self):
        if not self.motion_allowed:
            raise RuntimeError("motion is OFF (start the app with --enable-motion)")
        if self.running:
            raise RuntimeError("already driving")
        self._need_map()
        if self.pilot.target is None:
            raise RuntimeError("click a target square first")
        self.stop_flag = False
        self.running = True
        self.status = "driving"
        threading.Thread(target=self._drive, daemon=True).start()

    def _drive(self):
        try:
            with self.lock:
                where = self.pilot.drive()
            self.status = "arrived " + where
        except Stop:
            self.status = "stopped"
            self.log("stopped")
        except Exception as e:
            self.status = "stopped: " + str(e)
            self.log("STOPPED: " + str(e))
        finally:
            self.rover.stop()
            self.running = False

    def stop(self):
        self.stop_flag = True
        ok = self.rover.stop()
        self.log("STOP sent" if ok is not False else "STOP could NOT be sent: use the rover's button")

    # ------------------------------------------------------------------ views
    def view_png(self):
        img = self.pilot.render()
        if img is None:
            return None
        return cv2.imencode(".png", img)[1].tobytes()

    def photo_jpg(self):
        p = self.pilot.photo
        if p is None:
            return None
        h, w = p.shape[:2]
        small = cv2.resize(p, (960, int(960 * h / w))) if w > 960 else p
        return cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()

    def state(self):
        s = self.pilot.summary()
        g = self.pilot.grid
        top = self.pilot.top
        return dict(s, status=self.status, running=self.running, motion_allowed=self.motion_allowed,
                    calibrated=len(self.pilot.vel) >= 4, cols=g.cols, rows=g.rows, size_m=[g.w, g.h],
                    px_per_m=PX_PER_M, map_px=[top.shape[1], top.shape[0]] if top is not None else None,
                    photo_age_s=round(time.time() - self.last_photo_t, 1) if self.last_photo_t else None,
                    log=list(self.logs)[:30])


def make_handler(app, page):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(200, page, "text/html; charset=utf-8")
            if path == "/api/state":
                return self._json(200, app.state())
            if path in ("/view.png", "/photo.jpg"):
                try:
                    body = app.view_png() if path == "/view.png" else app.photo_jpg()
                except Exception as e:  # drawing while the drive thread updates: try again next poll
                    return self._send(503, str(e).encode(), "text/plain")
                if body is None:
                    return self._send(404, b"no photo yet", "text/plain")
                return self._send(200, body, "image/png" if path.endswith("png") else "image/jpeg")
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if self.headers.get("X-Requested-With") != "hq":
                return self._send(403, b"forbidden", "text/plain")
            n = int(self.headers.get("Content-Length", 0) or 0)
            try:
                d = json.loads(self.rfile.read(min(n, 4096)) or b"{}")
            except ValueError:
                return self._json(400, {"error": "bad json"})
            path = self.path.split("?")[0]
            try:
                if path == "/api/stop":
                    app.stop()
                elif path == "/api/photo":
                    app.get_photo()
                elif path == "/api/target":
                    app.set_target(str(d["cell"]))
                elif path == "/api/rover":
                    app.set_rover(str(d["cell"]))
                elif path == "/api/go":
                    app.go()
                elif path == "/api/recal":
                    app.recalibrate()
                else:
                    return self._json(404, {"error": "not found"})
            except Exception as e:
                app.log("error: %s" % e)
                return self._json(409, {"error": str(e), "state": app.state()})
            return self._json(200, app.state())

        def log_message(self, *a):
            pass

    return H


def main():
    ap = argparse.ArgumentParser(description="HQ app: aerial photo -> route -> autonomous rover")
    ap.add_argument("--camera-url", default="http://100.96.176.12:8090/photo.jpg", help="Zero camera server")
    ap.add_argument("--save-photo", default=os.path.expanduser("~/overhead_latest.jpg"))
    ap.add_argument("--size", default="1.18x0.62", help="board WxH in metres between the markers' outer corners")
    ap.add_argument("--grid", default="6x4", help="COLSxROWS")
    ap.add_argument("--rover-radius", type=float, default=0.07)
    ap.add_argument("--margin", type=float, default=0.03)
    ap.add_argument("--enable-motion", action="store_true", help="allow GO to drive the real rover")
    ap.add_argument("--dry-run", action="store_true", help="log rover commands instead of sending them")
    ap.add_argument("--sim", action="store_true", help="simulated camera and rover")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args()
    size = tuple(float(v) for v in a.size.lower().split("x"))
    grid = tuple(int(v) for v in a.grid.lower().split("x"))

    if a.sim:
        import math
        from sim import SimCamera, SimRover, SimWorld
        world = SimWorld(seed=1, white_board=True, rover_id=None, rover=(0.2, 0.75, math.radians(90)))
        world.boxes = [(0.62, 0.72, 0.77, 1.0), (0.48, -0.08, 0.68, 0.12)]
        camera, rover = SimCamera(world), SimRover(world)
        size, motion = (world.w + 0.2, world.h + 0.2), True
    else:
        from devices import HttpCamera, Rover
        camera = HttpCamera(a.camera_url, save_path=a.save_photo)
        rover = Rover(dry_run=a.dry_run or not a.enable_motion)
        motion = a.enable_motion

    app = App(camera, rover, size, grid, motion, a.rover_radius, a.margin)
    page = open(os.path.join(HERE, "app.html"), "rb").read()
    print("HQ app on http://%s:%d   motion=%s%s" % (a.host, a.port, "ON" if motion else "OFF",
                                                     "  (SIM)" if a.sim else ""), flush=True)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(app, page))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.stop()


if __name__ == "__main__":
    main()
