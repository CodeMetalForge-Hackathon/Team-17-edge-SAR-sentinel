"""Simulated sensors: test everything without hardware, and a fallback if a sensor dies mid-demo.

    python simulate.py --local     # runs sim + brain + drone (dry run) in ONE process, no broker needed
    python simulate.py             # publishes to the MQTT broker; run brain.py / drone_dispatch.py / dashboard.py separately
"""
import math
import sys
import time

import numpy as np

from features import window_features

SIM_NODES = {"n1": (1.5, 0.8), "n2": (2.2, -1.0)}
ROOM = (-1.0, 4.0, -2.5, 2.5)  # xmin, xmax, ymin, ymax of a rectangular room around the LiDAR
rng = np.random.default_rng(0)


# ---------------- node readings ----------------
def normal_reading(t_ms):
    return {"pir": 0, "vib": int(rng.random() < 0.02), "flame": 0, "water": 0,
            "loud_max": int(rng.normal(180, 25)), "loud_mean": int(rng.normal(120, 10)),
            "light": int(rng.normal(600, 8)), "ms": t_ms}


def node_window(kind="normal", n=10):
    rs = [normal_reading(i * 100) for i in range(n)]
    if kind == "tap":        # survivor tapping / calling + some motion
        for r in rs[3:8]:
            r["loud_max"] = int(rng.normal(650, 40))
        for r in rs:
            r["pir"] = 1
    elif kind == "flame":
        for r in rs:
            r["flame"] = 1
    elif kind == "shake":
        for r in rs:
            r["vib"] = 1   # the sketch reports vib as 0/1 per 100 ms period
    elif kind == "water":
        for r in rs:
            r["water"] = 1
    return rs


def synthetic_normal_windows(n):
    from features import to_vector
    X, prev = [], None
    for _ in range(n):
        f = window_features(node_window("normal"), prev)
        prev = f["light_mean"]
        X.append(to_vector(f))
    return np.array(X)


# ---------------- lidar ----------------
def room_scan(objects=()):
    """360 ranges for a rectangular room + circular objects [(x, y, radius)]."""
    xmin, xmax, ymin, ymax = ROOM
    out = np.empty(360)
    for b in range(360):
        c, s = math.cos(math.radians(b)), math.sin(math.radians(b))
        ts = []
        if c > 1e-9: ts.append(xmax / c)
        if c < -1e-9: ts.append(xmin / c)
        if s > 1e-9: ts.append(ymax / s)
        if s < -1e-9: ts.append(ymin / s)
        r = min(ts)
        for ox, oy, rad in objects:  # ray-circle intersection
            bq = ox * c + oy * s
            disc = bq * bq - (ox * ox + oy * oy - rad * rad)
            if disc >= 0 and bq - math.sqrt(disc) > 0:
                r = min(r, bq - math.sqrt(disc))
        out[b] = r + rng.normal(0, 0.01)
    return out


def scan_msg(ranges):
    return {"ts": time.time(), "ranges_m": [round(float(r), 3) for r in ranges]}


# ---------------- scenario ----------------
def run_scenario(bus, step_s=1.0):
    from lidar_change import ChangeDetector, build_baseline
    import config
    config.CHANGE_PERSIST_SCANS = 2
    base = build_baseline([room_scan() for _ in range(5)])
    det = ChangeDetector(base)
    objects = []
    prev_light = {k: None for k in SIM_NODES}

    def tick(node_kinds=None, person_bearing=None):
        node_kinds = node_kinds or {}
        scan = room_scan(objects)
        bus.publish("forge/lidar/scan", scan_msg(scan))
        for blob in det.update(scan):
            blob["ts"] = time.time()
            bus.publish("forge/lidar/change", blob)
        if person_bearing is not None:
            bus.publish("forge/vision/person", {"ts": time.time(), "bearing_deg": person_bearing, "conf": 0.86})
        for nid, (x, y) in SIM_NODES.items():
            f = window_features(node_window(node_kinds.get(nid, "normal")), prev_light[nid])
            prev_light[nid] = f["light_mean"]
            bus.publish(f"forge/node/{nid}/features", {"node": nid, "x": x, "y": y, "ts": time.time(), "features": f})
        time.sleep(step_s)

    print("--- 1. quiet room")
    for _ in range(3): tick()
    print("--- 2. tapping + motion at n1 (survivor)")
    tick({"n1": "tap"})
    for _ in range(2): tick()
    print("--- 3. flame at n2 (fire hazard)")
    tick({"n2": "flame"})
    print("--- 4. person walks in near n2, camera sees them; inside the fire zone")
    objects.append((2.1, -0.9, 0.2))
    b = math.degrees(math.atan2(-0.9, 2.1)) % 360
    for _ in range(3): tick(person_bearing=b)
    objects.clear()
    print("--- 5. debris appears at (3.0, 1.8), nobody there")
    objects.append((3.0, 1.8, 0.25))
    for _ in range(3): tick()
    print("--- 6. n1 gets shaken (structural)")
    tick({"n1": "shake"})
    print("--- 7. another survivor taps at n1, but the path is now blocked? (n1 itself is a hazard zone)")
    tick({"n1": "tap"})
    print("--- done")


if __name__ == "__main__":
    from bus import make_bus
    local = "--local" in sys.argv
    bus = make_bus(local=local, client_id="sim")
    if local:
        import config
        import brain
        from anomaly import NodeAnomalyModel
        from drone_dispatch import Dispatcher
        config.EVENT_COOLDOWN_S = 0  # the fast sim has no real time between steps
        b = brain.main(local_bus=bus)
        if b.model is None:
            print("[sim] training a throwaway model on synthetic normal data")
            b.model = NodeAnomalyModel.train(synthetic_normal_windows(1500))
        Dispatcher(bus, dry_run=True)
        run_scenario(bus, step_s=0.0)
    else:
        run_scenario(bus, step_s=1.0)
