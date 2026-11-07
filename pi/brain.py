"""The edge brain: subscribes to everything, fuses, scores, decides, publishes events + actions.

    python brain.py            # real run against the MQTT broker
"""
import math
import time
import itertools

import numpy as np

import config
from bus import make_bus
from features import to_vector
from anomaly import NodeAnomalyModel
from policy import decide


class Brain:
    def __init__(self, bus, model):
        self.bus = bus
        self.model = model
        self.scan = None
        self.persons = []        # recent person fixes {ts, x, y}
        self.avoid = []          # {x, y, r, kind, ts}
        self.cooldown = {}       # (kind, gx, gy) -> ts
        self.ids = itertools.count(1)
        bus.subscribe("forge/lidar/scan", self.on_scan)
        bus.subscribe("forge/lidar/change", self.on_change)
        bus.subscribe("forge/vision/person", self.on_person)
        bus.subscribe("forge/node/+/features", self.on_node)

    # ---------- inputs ----------
    def on_scan(self, topic, p):
        self.scan = np.array([np.nan if r is None else r for r in p["ranges_m"]], float)

    def range_at(self, bearing_deg, half_width=2):
        if self.scan is None:
            return None
        idx = [int(round(bearing_deg + k)) % 360 for k in range(-half_width, half_width + 1)]
        vals = self.scan[idx]
        vals = vals[~np.isnan(vals)]
        return float(vals.min()) if len(vals) else None  # nearest return = the person, not the wall behind

    def on_person(self, topic, p):
        r = self.range_at(p["bearing_deg"])
        if r is None:
            return  # no range at that bearing yet; can't place them
        x, y = r * math.cos(math.radians(p["bearing_deg"])), r * math.sin(math.radians(p["bearing_deg"]))
        now = p.get("ts", time.time())
        self.persons = [q for q in self.persons if now - q["ts"] < 5] + [{"ts": now, "x": x, "y": y}]
        self.handle("person", x, y, score=p["conf"], anomalous=True,
                    detail={"bearing_deg": p["bearing_deg"], "range_m": r, "conf": p["conf"]})

    def on_change(self, topic, p):
        now = p.get("ts", time.time())
        if self.person_near(p["x"], p["y"], now):
            return  # the person event already covers it
        self.handle("lidar", p["x"], p["y"], score=p["size_m"], anomalous=True, detail=p)

    def on_node(self, topic, p):
        feat = p["features"]
        score, anomalous, top = self.model.assess(to_vector(feat)) if self.model else (0.0, False, None)
        if not (anomalous or feat.get("flame_any") or feat.get("water_any")):
            return
        self.handle("node", p["x"], p["y"], score=score, anomalous=anomalous, feat=feat, node=p["node"],
                    person_near=self.person_near(p["x"], p["y"], p.get("ts", time.time())),
                    detail={"node": p["node"], "trigger": top, "features": feat})

    # ---------- fusion + decision ----------
    def person_near(self, x, y, now):
        return any(now - q["ts"] < 5 and math.hypot(q["x"] - x, q["y"] - y) < config.NEAR_RADIUS_M for q in self.persons)

    def handle(self, source, x, y, score, anomalous, feat=None, node=None, person_near=False, detail=None):
        now = time.time()
        kind, actions = decide(source, x, y, self.avoid, feat=feat, anomalous=anomalous,
                               person_near=person_near, node=node)
        if kind == "NORMAL":
            return
        key = (kind, round(x * 2), round(y * 2))  # 0.5 m grid
        if now - self.cooldown.get(key, 0) < config.EVENT_COOLDOWN_S:
            return
        self.cooldown[key] = now

        eid = next(self.ids)
        ev = {"id": eid, "ts": now, "kind": kind, "x": round(x, 3), "y": round(y, 3), "score": round(float(score), 3),
              "anomalous": bool(anomalous), "source": source, "detail": detail or {}}
        self.bus.publish("forge/event", ev)
        trig = (detail or {}).get("trigger")
        print(f"[brain] #{eid} {kind:<11} at ({x:+.2f}, {y:+.2f}) score {score:.2f} via {source}"
              + (f" (trigger: {trig})" if trig else ""))

        for action, reason in actions:
            if action == "MARK_AVOID":
                self.avoid.append({"x": x, "y": y, "r": config.AVOID_RADIUS_M, "kind": kind, "ts": now})
            if action == "BEACON" and node:
                self.bus.publish(f"forge/node/{node}/cmd", {"cmd": "beacon"})
            self.bus.publish("forge/action", {"event_id": eid, "ts": now, "action": action,
                                              "x": round(x, 3), "y": round(y, 3), "reason": reason,
                                              "avoid_zones": self.avoid})
            print(f"         -> {action}: {reason}")


def main(local_bus=None):
    try:
        model = NodeAnomalyModel.load()
    except FileNotFoundError:
        print("[brain] no model file, node anomalies limited to flame/water rules. Run: python anomaly.py train")
        model = None
    bus = local_bus or make_bus(client_id="brain")
    brain = Brain(bus, model)
    if local_bus is None:
        print("[brain] running")
        bus.loop_forever()
    return brain


if __name__ == "__main__":
    main()
