#!/usr/bin/env python3
"""Tiny stdlib static server for the overhead viewer (static-photo mode). No cloud, no extra deps.

  python serve.py --image C:\\Users\\rohan\\Pictures\\sandbox_setup.jpg [--port 8765]

Binds to 127.0.0.1 only. Serves index.html, homography.js, the chosen image at /image,
and saves/loads boundary corners at /corners (corners.json next to this file).
"""
import argparse, json, os, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "rover"))
import rover_control as rc
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
CORNERS = os.path.join(HERE, "corners.json")


def corners_path(image):
    """One scenario file per photo. The original sandbox_setup.jpg keeps corners.json."""
    base = os.path.basename(image)
    if base == "sandbox_setup.jpg":
        return os.path.join(HERE, "corners.json")
    return os.path.join(HERE, "corners_" + os.path.splitext(base)[0] + ".json")


# ---- automatic rubble detection (before photo vs current photo), see detect.py ----
DEBUG_PNG = os.path.join(HERE, "detect_debug.png")


def before_path(image):
    """The saved BEFORE photo for this image path (kept next to this file, one per photo name)."""
    return os.path.join(HERE, "before_" + os.path.basename(image))


def save_before(image):
    import shutil
    if not os.path.exists(image):
        return 404, {"error": "no current image on disk"}
    shutil.copyfile(image, before_path(image))
    return 200, {"before": os.path.basename(before_path(image)), "before_mtime": os.stat(before_path(image)).st_mtime}


def run_detect(image, d):
    """Body: {corners:[[x,y]*4] BL,BR,TR,TL, exclude:[{x0,y0,x1,y1}], cols, rows, image_mtime}."""
    try:
        import cv2
        import detect as det
    except ImportError as e:
        return 500, {"error": "detection needs OpenCV on this machine (sudo apt install python3-opencv): %s" % e}
    bp = before_path(image)
    if not os.path.exists(bp):
        return 409, {"error": "no BEFORE photo yet: photograph the empty sandbox, Reload image, press Save as BEFORE"}
    try:
        m = os.stat(image).st_mtime
    except OSError as e:
        return 404, {"error": str(e)}
    if "image_mtime" in d and abs(float(d["image_mtime"]) - m) > 1e-3:
        return 409, {"error": "a newer photo is on disk: press Reload image first"}
    corners = d.get("corners")
    if not (isinstance(corners, list) and len(corners) == 4):
        return 400, {"error": "need 4 corners"}
    before, after = cv2.imread(bp), cv2.imread(image)
    if before is None or after is None:
        return 500, {"error": "could not read the before/current photo"}
    if before.shape != after.shape:
        return 409, {"error": "BEFORE photo has a different size: save a new BEFORE with the same camera settings"}
    cols, rows = int(d.get("cols", 6)), int(d.get("rows", 4))
    exclude = [r for r in d.get("exclude", []) if isinstance(r, dict) and all(k in r for k in ("x0", "y0", "x1", "y1"))]
    obstacles, mask, _, after_top = det.detect(before, after, corners, exclude)
    cv2.imwrite(DEBUG_PNG, det.debug_image(after_top, mask, obstacles, cols, rows))
    return 200, {"obstacles": obstacles, "blocked": det.blocked_cells(obstacles, cols, rows),
                 "before_age_s": round(m - os.stat(bp).st_mtime, 1)}


class Rover:
    """Gatekeeper between the web UI and RoverLink. Motion is OFF unless the server was started with
    --enable-motion AND the operator arms it in the UI. Every move requires a fresh observation: the
    image file must be newer than the previous command, and the pose must come from that exact image.
    STOP is always allowed and disarms."""

    def __init__(self, image_path, enable_motion, dry_run):
        self.image_path, self.enable_motion, self.dry_run = image_path, enable_motion, dry_run
        self.link = rc.RoverLink(dry_run=dry_run)
        self.armed = False
        self.last_cmd_end = 0.0
        self.last_event = ""
        self.lock = threading.Lock()

    def image_mtime(self):
        try:
            return os.stat(self.image_path).st_mtime
        except OSError:
            return 0.0

    def state(self):
        m = self.image_mtime()
        return {"motion_enabled": self.enable_motion, "armed": self.armed, "dry_run": self.dry_run,
                "image_mtime": m, "fresh_observation": m > self.last_cmd_end, "last_event": self.last_event,
                "max_burst_s": rc.MAX_BURST_S}

    def arm(self, on):
        if on and not self.enable_motion:
            return 403, {"error": "server started without --enable-motion"}
        self.armed = bool(on)
        self.last_event = "armed" if on else "disarmed"
        return 200, self.state()

    def stop(self):
        with self.lock:  # a move in progress ends within MAX_BURST_S; STOP is sent regardless of armed state
            pass
        ok = self.link.stop()
        self.armed = False
        self.last_cmd_end = time.time()
        self.last_event = "STOP sent" if ok else "STOP FAILED to send"
        return (200 if ok else 502), dict(self.state(), stop_sent=ok)

    def move(self, d):
        if not self.enable_motion or not self.armed:
            return 403, {"error": "motion disabled/not armed"}
        cmd, dur = d.get("cmd"), d.get("duration")
        if cmd not in rc.MOVE_CMDS or not isinstance(dur, (int, float)) or not (0 < dur <= rc.MAX_BURST_S):
            return 400, {"error": "bad cmd/duration"}
        if not self.lock.acquire(blocking=False):
            return 409, {"error": "a move is already in progress"}
        try:
            m = self.image_mtime()
            if m <= self.last_cmd_end:
                return 409, {"error": "need a new photo taken after the last command"}
            if abs(float(d.get("image_mtime", 0)) - m) > 1e-3:
                return 409, {"error": "pose was set on an older image than the one on disk"}
            self.last_event = "move %s %.2fs" % (cmd, dur)
            try:
                self.link.move(cmd, dur)
            except rc.RoverCommError as e:
                self.armed = False
                self.last_cmd_end = time.time()
                self.last_event = "COMM ERROR: %s (disarmed; STOP attempted)" % e
                return 502, {"error": self.last_event}
            self.last_cmd_end = time.time()
            self.last_event = "moved %s %.2fs then STOP; need new photo" % (cmd, dur)
            return 200, self.state()
        finally:
            self.lock.release()


def make_handler(image_path, rover):
    class H(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                return self._send(200, open(os.path.join(HERE, "index.html"), "rb").read(), "text/html; charset=utf-8")
            if path == "/homography.js":
                return self._send(200, open(os.path.join(HERE, "homography.js"), "rb").read(), "text/javascript")
            if path == "/image":
                try:
                    st = os.stat(image_path)
                    data = open(image_path, "rb").read()
                except OSError as e:
                    return self._send(404, str(e).encode(), "text/plain")
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("X-Image-Mtime", str(st.st_mtime))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
                return
            if path == "/detect_debug.png":
                if not os.path.exists(DEBUG_PNG):
                    return self._send(404, b"no detection run yet", "text/plain")
                return self._send(200, open(DEBUG_PNG, "rb").read(), "image/png")
            if path == "/api/state":
                return self._send(200, json.dumps(rover.state()).encode(), "application/json")
            if path == "/corners":
                if os.path.exists(CORNERS):
                    return self._send(200, open(CORNERS, "rb").read(), "application/json")
                return self._send(404, b"{}", "application/json")
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            path = self.path.split("?")[0]
            if path.startswith("/api/"):
                # custom header blocks cross-site form posts from other browser tabs
                if self.headers.get("X-Requested-With") != "overhead-viewer":
                    return self._send(403, b"forbidden", "text/plain")
                n = int(self.headers.get("Content-Length", 0))
                try:
                    d = json.loads(self.rfile.read(min(n, 8192)) or b"{}")
                except ValueError:
                    return self._send(400, b"bad json", "text/plain")
                if path == "/api/stop":
                    code, body = rover.stop()
                elif path == "/api/arm":
                    code, body = rover.arm(d.get("armed"))
                elif path == "/api/move":
                    code, body = rover.move(d)
                elif path == "/api/before":
                    code, body = save_before(image_path)
                elif path == "/api/detect":
                    code, body = run_detect(image_path, d)
                else:
                    code, body = 404, {"error": "not found"}
                return self._send(code, json.dumps(body).encode(), "application/json")
            if path != "/corners":
                return self._send(404, b"not found", "text/plain")
            n = int(self.headers.get("Content-Length", 0))
            if n > 16384:
                return self._send(413, b"too large", "text/plain")
            try:
                d = json.loads(self.rfile.read(n))
                c = d["corners"]
                assert len(c) == 4 and all(len(p) == 2 and all(isinstance(v, (int, float)) for v in p) for p in c)
                sc = d.get("scenario")
                if sc is not None:
                    assert isinstance(sc, dict) and isinstance(sc["cols"], int) and isinstance(sc["rows"], int)
                    assert isinstance(sc["obstacles"], list) and len(sc["obstacles"]) <= 200 and sc.get("version") == 2
                json.dump({"corners": c, "image_size": d.get("image_size"), "scenario": sc, "hints": d.get("hints")}, open(CORNERS, "w"))
            except Exception as e:
                return self._send(400, ("bad corners: %s" % e).encode(), "text/plain")
            self._send(200, b'{"saved":true}', "application/json")

        def log_message(self, *a):
            pass

    return H


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--enable-motion", action="store_true", help="allow the UI to arm and send motion commands (default OFF)")
    ap.add_argument("--dry-run", action="store_true", help="rover commands are logged, not sent")
    a = ap.parse_args()
    if not os.path.exists(a.image):
        raise SystemExit("image not found: " + a.image)
    CORNERS = corners_path(a.image)
    rover = Rover(a.image, a.enable_motion, a.dry_run)
    print("scenario file:", CORNERS)
    print("Overhead viewer: http://127.0.0.1:%d  (image: %s) motion=%s dry_run=%s" % (a.port, a.image, "ENABLED-for-arming" if a.enable_motion else "OFF", a.dry_run))
    try:
        ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(a.image, rover)).serve_forever()
    finally:
        rover.link.stop(); rover.link.close()
