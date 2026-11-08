#!/usr/bin/env python3
"""Measure camera latency, tracking noise, command-to-motion latency and stopping response on the real rover.
Moves the rover about 10-15 cm once (streamed drive for ~0.5 s, then STOP). Run on the controller Pi."""
import math, os, sys, time, statistics as st
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import perception as P, sources as S, ugv

src = S.RemoteFrames("http://100.96.176.12:8780"); time.sleep(3)
per = P.Perception(os.path.expanduser("~/forge-nav")); link = ugv.UgvLink()
A = None


def pose_of(fr):
    o = per.process(fr)
    return o.pose, time.time()


# A. frame cadence, transfer and tracking noise (stationary)
t0 = time.time(); seqs = []; fetch = []; pts = []; hd = []
while time.time() - t0 < 12:
    fr = src.wait_fresh((seqs[-1] + 1) if seqs else 0, 3)
    if fr is None: continue
    seqs.append(fr.seq); fetch.append(src.fetch_ms)
    p, _ = pose_of(fr)
    if p: pts.append((p["x"], p["y"])); hd.append(p.get("theta_raw"))
A = per.calib.aspect; hm = per.calib.height_mm
meta_rtt = []
for _ in range(5):
    t = time.time(); src.current_seq(); meta_rtt.append((time.time() - t) * 1000)
dts = [b - a for a, b in zip(seqs, seqs[1:])]
xs = [p[0] * A * hm for p in pts]; ys = [p[1] * hm for p in pts]
print("frames received: %d in 12 s (%.2f fps delivered, %.1f frames skipped on average)" % (len(seqs), len(seqs) / 12, st.mean(dts) - 1 if dts else 0))
print("frame transfer: mean %.0f ms, max %.0f ms; /meta round trip %.0f ms" % (st.mean(fetch), max(fetch), st.mean(meta_rtt)))
print("position noise (stationary): sigma x %.1f mm, y %.1f mm  (n=%d)" % (st.pstdev(xs), st.pstdev(ys), len(xs)))
hh = [h for h in hd if h is not None]
if len(hh) > 3:
    m = math.atan2(sum(math.sin(h) for h in hh), sum(math.cos(h) for h in hh))
    d = [math.degrees(math.atan2(math.sin(h - m), math.cos(h - m))) for h in hh]
    print("pattern-heading noise (single frame): sigma %.1f deg" % st.pstdev(d))

# B/C. command -> first observed motion, and stop response
base = pts[-1]
frames = []
def watch(until, tag):
    seq = src.latest_seq() + 1
    while time.time() < until:
        fr = src.wait_fresh(seq, 1.0)
        if fr is None: continue
        seq = fr.seq + 1
        p, trx = pose_of(fr)
        if p: frames.append((trx, p["x"] * A * hm, p["y"] * hm, tag))
t_cmd = time.time(); link.drive(0.35, 0.35, ttl=0.9)
end = t_cmd + 0.5
while time.time() < end:
    link.drive(0.35, 0.35, ttl=0.9); time.sleep(0.08)
t_stop = time.time(); link.halt(); stop_ms = (time.time() - t_stop) * 1000
watch(time.time() + 4.0, "after")
x0, y0 = base[0] * A * hm, base[1] * hm
mov = [(t, math.hypot(x - x0, y - y0)) for t, x, y, _ in frames]
first = next((t for t, d in mov if d > 8.0), None)
print("command -> first frame showing >8 mm motion: %.2f s" % ((first - t_cmd) if first else float('nan')))
print("STOP command round trip: %.0f ms" % stop_ms)
final = mov[-1][1] if mov else 0
tstab = next((t for t, d in mov if abs(d - final) < 6.0), None)
print("total travel %.0f mm; position stopped changing %.2f s after the STOP command" % (final, (tstab - t_stop) if tstab else float('nan')))
print("distance travelled after STOP was sent (as seen): %.0f mm of %.0f mm" % (final - min([d for t, d in mov if t <= t_stop + 0.3] or [0]), final))
