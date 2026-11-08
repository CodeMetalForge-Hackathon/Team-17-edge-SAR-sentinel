"""LiDAR driver -> scans -> change events on MQTT.

Driver: Slamtec's official SDK demo `ultra_simple` (supports C1 and S2). Build it once:
    git clone https://github.com/Slamtec/rplidar_sdk && cd rplidar_sdk && make
C1 runs at 460800 baud, S2 at 1000000. Then pipe it in:

    # 1) record a baseline of the EMPTY scene (~10 s), saves baseline_scan.npy
    ./output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB0 460800 | python lidar_node.py --baseline
    # 2) run live
    ./output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB0 460800 | python lidar_node.py

ultra_simple prints one line per point, roughly:  "S  theta: 12.34 Dist: 01234.00 Q: 47"
("S" marks the first point of a new revolution). If your build prints something different,
fix LINE_RE. If you'd rather use the `rplidarc1` pip package, just replace read_revolutions().
"""
import re
import sys
import time

import numpy as np

import config
from bus import make_bus
from lidar_change import points_to_bins, build_baseline, ChangeDetector

LINE_RE = re.compile(r"^\s*(S)?\s*theta:\s*([\d.]+)\s+Dist:\s*([\d.]+)\s+Q:\s*(\d+)")


def read_revolutions(stream):
    """Yield one list of (angle_deg, dist_mm) per full revolution."""
    pts = []
    for line in stream:
        m = LINE_RE.match(line)
        if not m:
            continue
        sync, ang, dist, q = m.groups()
        if sync and pts:
            yield pts
            pts = []
        if int(q) > 0 and float(dist) > 0:
            pts.append((float(ang), float(dist)))


def main():
    if "--baseline" in sys.argv:
        scans, t0 = [], time.time()
        for pts in read_revolutions(sys.stdin):
            scans.append(points_to_bins(pts))
            if time.time() - t0 > 10:
                break
        base = build_baseline(scans)
        np.save(config.BASELINE_FILE, base)
        print(f"baseline saved from {len(scans)} scans, {np.sum(~np.isnan(base))}/360 bins valid")
        return

    det = ChangeDetector(np.load(config.BASELINE_FILE))
    bus = make_bus(client_id="lidar")
    last_scan_pub = 0
    for pts in read_revolutions(sys.stdin):
        ranges = points_to_bins(pts)
        now = time.time()
        if now - last_scan_pub > 0.5:  # 2 Hz is plenty for the brain + dashboard
            bus.publish("forge/lidar/scan", {"ts": now, "ranges_m": [None if np.isnan(r) else round(float(r), 3) for r in ranges]})
            last_scan_pub = now
        for blob in det.update(ranges):
            blob["ts"] = now
            bus.publish("forge/lidar/change", blob)
            print(f"[lidar] change at ({blob['x']:.2f}, {blob['y']:.2f}) width {blob['width_deg']} deg")


if __name__ == "__main__":
    main()
