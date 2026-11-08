#!/usr/bin/env python3
"""Read-only frame server for the Zero. Serves the latest atomically-written frame.bin from aurora_cap.
GET /frame            -> raw frame.bin   (u32 metalen | meta json | rgb NV12 | depth u16 mm)
GET /frame?z=1        -> same with the depth block zlib-compressed (meta gets depth_zlib_bytes)
GET /frame?z=1&jpg=1  -> additionally RGB as JPEG instead of raw NV12 (meta gets rgb_jpeg_bytes): ~3x smaller
GET /frame?z=1&jpg=1&roi=x0,y0,x1,y1 -> only that image rectangle (rover tracking: ~5x fewer bytes; meta gets "roi")
GET /meta             -> meta json only
No control endpoints. Bind to the Tailscale address only (not 0.0.0.0).
"""
import argparse, io, json, os, struct, zlib
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


_cache = {"seq": None, "q": None, "body": None}


def nv12_to_jpeg(rgb, cols, rows, quality=82, roi=None):
    import numpy as np
    from PIL import Image
    y = np.frombuffer(rgb, np.uint8, cols * rows).reshape(rows, cols)
    uv = np.frombuffer(rgb, np.uint8, cols * rows // 2, cols * rows).reshape(rows // 2, cols // 2, 2)
    if roi:                                   # even-aligned crop (chroma is subsampled 2x2)
        x0, y0, x1, y1 = roi
        y = y[y0:y1, x0:x1]
        uv = uv[y0 // 2:y1 // 2, x0 // 2:x1 // 2]
    y = y.astype(np.float32)
    uv = uv.astype(np.float32) - 128.0
    u = np.repeat(np.repeat(uv[:, :, 0], 2, 0), 2, 1)
    v = np.repeat(np.repeat(uv[:, :, 1], 2, 0), 2, 1)
    img = np.clip(np.stack([y + 1.402 * v, y - 0.344 * u - 0.714 * v, y + 1.772 * u], 2), 0, 255).astype(np.uint8)
    b = io.BytesIO()
    Image.fromarray(img).save(b, "JPEG", quality=quality)
    return b.getvalue()


def parse_roi(q, cols, rows):
    for kv in q.split("&"):
        if kv.startswith("roi="):
            try:
                x0, y0, x1, y1 = [int(float(v)) for v in kv[4:].split(",")]
            except ValueError:
                return None
            x0, y0 = max(0, x0) & ~1, max(0, y0) & ~1
            x1, y1 = min(cols, x1 + 1) & ~1, min(rows, y1 + 1) & ~1
            return (x0, y0, x1, y1) if x1 - x0 >= 32 and y1 - y0 >= 32 else None
    return None


class H(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"             # keep-alive: no TCP setup round trip through the relay per frame

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
            roi = parse_roi(q, meta["rgb"]["cols"], meta["rgb"]["rows"])
            key = (meta["seq"], q)
            if _cache["q"] == key:
                body, ct = _cache["body"], "application/octet-stream"
            else:
                if roi:
                    import numpy as np
                    x0, y0, x1, y1 = roi
                    d = np.frombuffer(dep, "<u2").reshape(meta["depth"]["rows"], meta["depth"]["cols"])[y0:y1, x0:x1]
                    dep = d.tobytes()
                    meta["roi"] = [x0, y0, x1, y1]
                if "z=1" in q:
                    if "jpg=1" in q:        # 4 mm depth quantisation (sensor noise is larger) shrinks the frame ~40 %
                        import numpy as np
                        dep = ((np.frombuffer(dep, "<u2") >> 2) << 2).astype("<u2").tobytes()
                        dep = zlib.compress(dep, 6)
                    else:
                        dep = zlib.compress(dep, 1)
                    meta["depth_zlib_bytes"] = len(dep)
                if "jpg=1" in q:
                    rgb = nv12_to_jpeg(rgb, meta["rgb"]["cols"], meta["rgb"]["rows"], roi=roi)
                    meta["rgb_jpeg_bytes"] = len(rgb)
                m = json.dumps(meta).encode()
                body, ct = struct.pack("<I", len(m)) + m + rgb + dep, "application/octet-stream"
                _cache.update(q=key, body=body)
        else:
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", ct); self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers(); self.wfile.write(body)


ThreadingHTTPServer((A.bind, A.port), H).serve_forever()
