#!/usr/bin/env python3
"""Read-only frame server for the Zero. Serves the latest atomically-written frame.bin from aurora_cap.
GET /frame            -> raw frame.bin   (u32 metalen | meta json | rgb NV12 | depth u16 mm)
GET /frame?z=1        -> same with the depth block zlib-compressed (meta gets depth_zlib_bytes)
GET /meta             -> meta json only
No control endpoints. Bind to the Tailscale address only (not 0.0.0.0).
"""
import argparse, json, os, struct, zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ap = argparse.ArgumentParser()
ap.add_argument("--bind", default="127.0.0.1")
ap.add_argument("--port", type=int, default=8780)
ap.add_argument("--dir", default="/tmp/aurora")
A = ap.parse_args()


def load():
    try:
        with open(os.path.join(A.dir, "frame.bin"), "rb") as f:
            b = f.read()
        ml = struct.unpack("<I", b[:4])[0]
        meta = json.loads(b[4:4 + ml])
        return meta, b[4 + ml:4 + ml + meta["rgb"]["bytes"]], b[4 + ml + meta["rgb"]["bytes"]:]
    except Exception:
        return None


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        path, _, q = self.path.partition("?")
        fr = load()
        if not fr:
            self.send_response(503); self.end_headers(); return
        meta, rgb, dep = fr
        if path == "/meta":
            body, ct = json.dumps(meta).encode(), "application/json"
        elif path == "/frame":
            if "z=1" in q:
                dep = zlib.compress(dep, 1)
                meta["depth_zlib_bytes"] = len(dep)
            m = json.dumps(meta).encode()
            body, ct = struct.pack("<I", len(m)) + m + rgb + dep, "application/octet-stream"
        else:
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", ct); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)


ThreadingHTTPServer((A.bind, A.port), H).serve_forever()
