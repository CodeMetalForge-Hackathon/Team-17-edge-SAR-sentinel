"""Frame sources: the Aurora930 on the Zero (HTTP, read-only) with the same interface as sim.SimWorld."""
import json
import struct
import threading
import time
import urllib.request
import zlib

import numpy as np

import vision as V


class Frame:
    pass


def parse_frame(buf):
    ml = struct.unpack("<I", buf[:4])[0]
    meta = json.loads(buf[4:4 + ml])
    r, d = meta["rgb"], meta["depth"]
    off = 4 + ml
    rgb = buf[off:off + r["bytes"]]
    off += r["bytes"]
    dep = buf[off:]
    if "depth_zlib_bytes" in meta:
        dep = zlib.decompress(dep)
    f = Frame()
    f.bgr = V.nv12_to_bgr(rgb, r["cols"], r["rows"])
    f.gray = V.gray_of(rgb, r["cols"], r["rows"])
    f.depth = np.frombuffer(dep, "<u2").reshape(d["rows"], d["cols"]).copy()   # millimetres, 0 = invalid
    k = meta["intrinsics"]["rgb"] if meta.get("aligned") else meta["intrinsics"]["ir"]
    f.K = (k["fx"], k["fy"], k["cx"], k["cy"])
    f.cols, f.rows = r["cols"], r["rows"]
    f.seq, f.wall, f.meta = meta["seq"], meta["wall"], meta
    return f


class RemoteFrames:
    """Pulls frames from the Zero's frame_server. 'seq' is made monotonic across capture restarts."""

    def __init__(self, url="http://100.96.176.12:8780"):
        self.url = url.rstrip("/")
        self.cv = threading.Condition()
        self.frame = None
        self.epoch, self.last_raw = 0, 0
        self.err = ""
        self.fetch_ms = 0
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _vseq(self, raw):
        if raw < self.last_raw - 3:       # capture restarted on the Zero
            self.epoch += 1
        self.last_raw = raw
        return self.epoch * 1_000_000 + raw

    def _loop(self):
        while self.running:
            t = time.time()
            try:
                with urllib.request.urlopen(self.url + "/frame?z=1", timeout=8) as r:
                    f = parse_frame(r.read())
                f.seq = self._vseq(f.seq)
                f.rx = time.time()
                with self.cv:
                    if self.frame is None or f.seq != self.frame.seq:
                        self.frame = f
                        self.cv.notify_all()
                self.err = ""
                self.fetch_ms = int((time.time() - t) * 1000)
            except Exception as e:
                self.err = str(e)[:120]
                time.sleep(1.0)
            time.sleep(0.05)

    def current_seq(self):
        try:
            with urllib.request.urlopen(self.url + "/meta", timeout=4) as r:
                return self._vseq(json.loads(r.read())["seq"])
        except Exception:
            with self.cv:
                return self.frame.seq if self.frame else 0

    def wait_fresh(self, min_seq, timeout=5.0):
        end = time.time() + timeout
        with self.cv:
            while True:
                if self.frame is not None and self.frame.seq >= min_seq:
                    return self.frame
                left = end - time.time()
                if left <= 0:
                    return None
                self.cv.wait(min(left, 0.5))

    def latest(self):
        with self.cv:
            return self.frame

    def age(self):
        f = self.latest()
        return None if f is None else time.time() - f.rx
