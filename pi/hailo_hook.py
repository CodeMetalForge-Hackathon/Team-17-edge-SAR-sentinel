"""Publish person detections from the Hailo AI HAT as map bearings.

The brain fuses bearing + LiDAR range at that bearing = person position on the map (no GPS, no depth cam).

How to hook it in (hailo-apps detection pipeline, https://github.com/hailo-ai/hailo-apps):
open the detection example's app_callback and add, inside the loop over detections:

    from hailo_hook import PersonPublisher          # put this file on the PYTHONPATH
    pub = PersonPublisher()                         # create once, outside the callback
    ...
    for det in detections:
        if det.get_label() == "person":
            bb = det.get_bbox()                     # normalized 0..1
            pub.publish(bb.xmin() + bb.width() / 2, det.get_confidence())

Method names come from the Hailo examples at the time of writing; check the example you clone.
"""
import math
import time

import config
from bus import make_bus


def pixel_to_bearing(cx_norm):
    """cx_norm: horizontal center of the bbox, 0 = left edge, 1 = right edge.
    Returns map bearing in degrees (CCW positive, so left of center is positive)."""
    half = math.radians(config.CAMERA_HFOV_DEG / 2)
    off = (0.5 - cx_norm) * 2 * math.tan(half)  # pinhole model
    return (math.degrees(math.atan(off)) + config.CAMERA_ANGLE_OFFSET_DEG) % 360


class PersonPublisher:
    def __init__(self, bus=None, min_interval_s=0.3):
        self.bus = bus or make_bus(client_id="vision")
        self.min_interval_s = min_interval_s
        self.last = 0

    def publish(self, cx_norm, conf):
        now = time.time()
        if conf < config.PERSON_MIN_CONF or now - self.last < self.min_interval_s:
            return
        self.last = now
        self.bus.publish("forge/vision/person", {"ts": now, "bearing_deg": pixel_to_bearing(cx_norm), "conf": float(conf)})
