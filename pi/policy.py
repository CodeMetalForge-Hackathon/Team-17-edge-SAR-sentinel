"""Response policy: situation -> kind of event + list of actions. Plain rules on purpose:
easy to explain to a judge, easy to tune with operator feedback.

Actions:
  DISPATCH_DRONE   fly the Crazyflie to (x, y), hover, return
  BEACON           buzz/flash the nearest node so a survivor knows help is coming
  MARK_AVOID       add a hazard zone to the shared map; drone paths must not cross it
  ALERT_GROUND     tell human responders (drone can't or shouldn't go)
  UPDATE_MAP       record an obstruction / changed terrain
  LOG              anomalous but low confidence, just record it
"""
import math

import config

LOUD_SPIKE = 400        # loud_max on a 0..1023 ADC; tune on site
VIB_STRUCTURAL = 5      # vibration hits per window


def seg_point_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def path_blocked(x, y, avoid_zones):
    pad = config.DRONE_PAD
    for z in avoid_zones:
        if seg_point_dist(pad["x"], pad["y"], x, y, z["x"], z["y"]) < z["r"]:
            return z
    return None


def in_zone(x, y, avoid_zones):
    return next((z for z in avoid_zones if math.hypot(x - z["x"], y - z["y"]) < z["r"]), None)


def decide(source, x, y, avoid_zones, feat=None, anomalous=False, person_near=False, node=None):
    """Returns (kind, [(action, reason), ...])."""
    feat = feat or {}
    acts = []

    if source == "node" and feat.get("flame_any"):
        return "FIRE", [("MARK_AVOID", "flame sensor"), ("ALERT_GROUND", "fire hazard")]
    if source == "node" and feat.get("water_any"):
        return "FLOOD", [("MARK_AVOID", "water sensor")]
    if source == "node" and anomalous and feat.get("vib_hits", 0) >= VIB_STRUCTURAL:
        return "STRUCTURAL", [("MARK_AVOID", "vibration burst"), ("ALERT_GROUND", "possible collapse")]

    survivor = source == "person" or person_near or (
        source == "node" and anomalous and (feat.get("pir_frac", 0) > 0 or feat.get("loud_max", 0) > LOUD_SPIKE))
    if survivor:
        kind = "SURVIVOR"
        if node:
            acts.append(("BEACON", f"let survivor at {node} know help is coming"))
        zone = in_zone(x, y, avoid_zones)
        blocked = path_blocked(x, y, avoid_zones)
        if zone:
            acts.append(("ALERT_GROUND", f"survivor inside {zone['kind']} zone, drone held"))
        elif blocked:
            acts.append(("ALERT_GROUND", f"drone path crosses {blocked['kind']} zone, send ground team"))
        else:
            acts.append(("DISPATCH_DRONE", "confirm survivor and relay position"))
        return kind, acts

    if source == "lidar":
        return "OBSTRUCTION", [("UPDATE_MAP", "new object or debris in scan")]

    if anomalous:
        return "UNKNOWN", [("LOG", "anomalous node reading, no clear class")]
    return "NORMAL", []
