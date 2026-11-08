#!/usr/bin/env python3
"""Aerial camera server for the Pi Zero: keeps the camera open and focused so a photo is instant.

    python3 camserver.py                 # http://<zero>:8090/photo.jpg
    python3 camserver.py --size 2304x1296 --port 8090

GET /photo.jpg  -> JPEG of the current frame (taken at the moment of the request)
GET /health     -> "ok <width>x<height>"

Uses picamera2 (preinstalled on Raspberry Pi OS; else: sudo apt install -y python3-picamera2).
Autofocus runs continuously, so frames are already sharp when asked for.
"""
import argparse
import io
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from picamera2 import Picamera2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", default="2304x1296")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--quality", type=int, default=90)
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.lower().split("x"))

    cam = Picamera2()
    cam.configure(cam.create_still_configuration(main={"size": (w, h)}, buffer_count=2))
    cam.options["quality"] = a.quality
    cam.start()
    try:
        from libcamera import controls
        cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
    except Exception as e:  # camera without autofocus, or older libcamera: fine
        print("autofocus not set:", e)
    lock = threading.Lock()

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/health":
                body = ("ok %dx%d" % (w, h)).encode()
                ctype = "text/plain"
            elif path == "/photo.jpg":
                buf = io.BytesIO()
                with lock:
                    cam.capture_file(buf, format="jpeg")
                body, ctype = buf.getvalue(), "image/jpeg"
            else:
                self.send_response(404); self.end_headers(); return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    print("camera server on :%d  (%dx%d)  GET /photo.jpg" % (a.port, w, h), flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
