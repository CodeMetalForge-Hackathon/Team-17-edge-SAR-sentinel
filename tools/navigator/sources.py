"""Frame sources: the Aurora930 on the Zero (HTTP, read-only) with the same interface as sim.SimWorld."""
import http.client
import json
import struct
import threading
import time
import urllib.parse
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
    nrgb = meta.get("rgb_jpeg_bytes", r["bytes"])
    rgb = buf[off:off + nrgb]
    off += nrgb
    dep = buf[off:]
    if "depth_zlib_bytes" in meta:
        dep = zlib.decompress(dep)
    f = Frame()
    import cv2
    if "rgb_jpeg_bytes" in meta:
        f.bgr = cv2.imdecode(np.frombuffer(rgb, np.uint8), cv2.IMREAD_COLOR)
        f.gray = cv2.cvtColor(f.bgr, cv2.COLOR_BGR2GRAY)
    else:
        f.bgr = V.nv12_to_bgr(rgb, r["cols"], r["rows"])
        f.gray = V.gray_of(rgb, r["cols"], r["rows"])
    roi = meta.get("roi")
    if roi:                                       # rover-region frame: paste into an empty canvas, flag it partial
        x0, y0, x1, y1 = roi
        full_d = np.zeros((d["rows"], d["cols"]), np.uint16)
        full_d[y0:y1, x0:x1] = np.frombuffer(dep, "<u2").reshape(y1 - y0, x1 - x0)
        full_b = np.zeros((r["rows"], r["cols"], 3), np.uint8)
        full_b[y0:y1, x0:x1] = f.bgr
        f.bgr, f.gray, f.depth = full_b, cv2.cvtColor(full_b, cv2.COLOR_BGR2GRAY), full_d
        f.partial, f.roi = True, (x0, y0, x1, y1)
    else:
        f.depth = np.frombuffer(dep, "<u2").reshape(d["rows"], d["cols"]).copy()   # millimetres, 0 = invalid
        f.partial, f.roi = False, None
    k = meta["intrinsics"]["rgb"] if meta.get("aligned") else meta["intrinsics"]["ir"]
    f.K = (k["fx"], k["fy"], k["cx"], k["cy"])
    f.cols, f.rows = r["cols"], r["rows"]
    f.seq, f.wall, f.meta = meta["seq"], meta["wall"], meta
    return f


class RemoteFrames:
    """Pulls frames from the Zero's frame_server over ONE keep-alive connection. 'seq' is made monotonic across capture
    restarts. Two kinds of frame: full (overview, obstacle map, boundary) and rover-region "track" frames (small, for pose
    only). While a pose is being tracked only track frames are fetched; a full frame is fetched when there is no recent
    pose, on request (request_full), or about once a minute while idle (never during a run: the map is reused)."""

    FULL_PERIOD_S = 60.0
    IDLE_INTERVAL_S = 0.5
    ROI_HALF = 120                    # px half-width of the track window (rover ~70 px; < 1 observation of travel)
    HINT_MAX_AGE_S = 1.5

    def __init__(self, url="http://100.96.176.12:8780"):
        self.url = url.rstrip("/")
        u = urllib.parse.urlparse(self.url)
        self._host, self._port = u.hostname, u.port or 80
        self._conn = None
        self.cv = threading.Condition()
        self.frame = None
        self.full = None                  # latest FULL frame (display / mapping)
        self.epoch, self.last_raw = 0, 0
        self.err = ""
        self.fetch_ms = 0
        self.nbytes = 0
        self.hint = None                  # callable -> (px, py, t) of the last tracked rover, or None
        self.hold_full = False            # True during a run: no periodic full frames
        self.want_full = False
        self.full_period = self.FULL_PERIOD_S     # s between full frames; the navigator sets ~8 s while a run is active
        self.last_full_t = 0.0
        self.stats = {"full": 0, "roi": 0, "bytes": 0, "full_ms": 0.0, "roi_ms": 0.0}
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _vseq(self, raw):
        if raw < self.last_raw - 3:       # capture restarted on the Zero
            self.epoch += 1
        self.last_raw = raw
        return self.epoch * 1_000_000 + raw

    def request_full(self):
        self.want_full = True

    def fresh_full(self, timeout=10.0):
        """A full frame captured/received after this call (the target-selection snapshot)."""
        t = time.time()
        self.want_full = True
        end = t + timeout
        with self.cv:
            while True:
                if self.full is not None and self.full.rx > t:
                    return self.full
                left = end - time.time()
                if left <= 0:
                    return None
                self.cv.wait(min(left, 0.5))

    def _roi(self):
        h = self.hint() if self.hint else None
        if not h or time.time() - h[2] > self.HINT_MAX_AGE_S:
            return None
        cx, cy = h[0], h[1]
        f = self.full or self.frame
        W, H = (f.cols, f.rows) if f is not None else (640, 400)
        x0, y0 = int(max(0, min(W - 2 * self.ROI_HALF, cx - self.ROI_HALF))), int(max(0, min(H - 2 * self.ROI_HALF, cy - self.ROI_HALF)))
        return x0, y0, min(W, x0 + 2 * self.ROI_HALF), min(H, y0 + 2 * self.ROI_HALF)

    def _plan(self):
        """('full'|'roi', roi). Full when nothing is tracked, on request, or (idle only) once a minute."""
        now = time.time()
        roi = self._roi()
        if self.full is None or roi is None or self.want_full:
            return "full", None
        if now - self.last_full_t > self.full_period:
            return "full", None
        return "roi", roi

    def _fetch(self, q):
        for attempt in range(2):
            try:
                if self._conn is None:
                    self._conn = http.client.HTTPConnection(self._host, self._port, timeout=8)
                self._conn.request("GET", "/frame?" + q)
                r = self._conn.getresponse()
                body = r.read()
                if r.status != 200:
                    raise OSError("HTTP %d" % r.status)
                return body
            except Exception:
                try:
                    self._conn.close()
                except Exception:
                    pass
                self._conn = None
                if attempt:
                    raise

    def _loop(self):
        while self.running:
            t = time.time()
            try:
                kind, roi = self._plan()
                q = "z=1&jpg=1" + ("&roi=%d,%d,%d,%d" % roi if roi else "")
                body = self._fetch(q)
                f = parse_frame(body)
                f.seq = self._vseq(f.seq)
                f.rx = time.time()
                dt = f.rx - t
                with self.cv:
                    if self.frame is None or f.seq != self.frame.seq:
                        self.frame = f
                        if not f.partial:
                            self.full, self.last_full_t, self.want_full = f, f.rx, False
                        self.cv.notify_all()
                self.err = ""
                self.fetch_ms, self.nbytes = int(dt * 1000), len(body)
                k = "full" if kind == "full" else "roi"
                self.stats[k] += 1
                self.stats["bytes"] += len(body)
                self.stats[k + "_ms"] += dt * 1000
                if kind == "roi" and self.hold_full is False and self.IDLE_INTERVAL_S > dt:
                    time.sleep(self.IDLE_INTERVAL_S - dt)       # idle: tracking display only, no need to hammer the link
            except Exception as e:
                self.err = str(e)[:120]
                time.sleep(1.0)
            time.sleep(0.01)

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

    def seq_est(self):
        """Estimate of the Zero's current sequence number without a network round trip (5 fps)."""
        with self.cv:
            f = self.frame
        if f is None:
            return 0
        # frame f was captured about (transfer + serve) seconds before it arrived; the Zero keeps capturing at 5 fps
        return f.seq + int(((time.time() - f.rx) + self.fetch_ms / 1000.0 + 0.4) * 5.0)

    def latest_seq(self):
        with self.cv:
            return self.frame.seq if self.frame else 0

    def latest(self):
        with self.cv:
            return self.frame

    def latest_full(self):
        with self.cv:
            return self.full or self.frame

    def age(self):
        f = self.latest()
        return None if f is None else time.time() - f.rx
