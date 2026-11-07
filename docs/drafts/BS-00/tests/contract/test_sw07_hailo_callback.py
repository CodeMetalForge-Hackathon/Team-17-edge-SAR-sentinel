"""SW-07 acceptance, with fake detections. It does not show the real Hailo pipeline runs: HW-B3 does that."""
import sys

import forge_test_support  # noqa: F401  (adds pi/ to the path)

import config
from bus import LocalBus
from hailo_hook import PersonPublisher, pixel_to_bearing


class Box:
    def __init__(self, xmin, width):
        self._x, self._w = xmin, width

    def xmin(self):
        return self._x

    def width(self):
        return self._w


class Det:
    def __init__(self, label, conf, xmin=0.4, width=0.2):
        self.label, self.conf, self.box = label, conf, Box(xmin, width)

    def get_label(self):
        return self.label

    def get_confidence(self):
        return self.conf

    def get_bbox(self):
        return self.box


def test_importing_app_needs_no_hailo_runtime():
    sys.modules.pop("hailo", None)
    import hailo_person_app  # noqa: F401
    assert "hailo" not in sys.modules


def test_only_people_are_kept_with_center_and_confidence():
    import hailo_person_app
    out = hailo_person_app.persons_from_detections([Det("person", 0.9, 0.1, 0.2), Det("chair", 0.99), Det("person", 0.7, 0.5, 0.4)])
    assert len(out) == 2
    assert abs(out[0][0] - 0.2) < 1e-9 and out[0][1] == 0.9
    assert abs(out[1][0] - 0.7) < 1e-9 and out[1][1] == 0.7


def test_each_person_is_published_with_the_existing_payload():
    bus = LocalBus()
    PersonPublisher(bus=bus, min_interval_s=0).publish_many([(0.2, 0.9), (0.7, 0.8)])
    msgs = [p for t, p in bus.log if t == "forge/vision/person"]
    assert len(msgs) == 2
    assert all(set(m) == {"ts", "bearing_deg", "conf"} for m in msgs)
    assert abs(msgs[0]["bearing_deg"] - pixel_to_bearing(0.2)) < 1e-9


def test_low_confidence_is_not_published():
    bus = LocalBus()
    PersonPublisher(bus=bus, min_interval_s=0).publish_many([(0.5, config.PERSON_MIN_CONF - 0.1)])
    assert bus.log == []


def test_batches_are_rate_limited_as_a_whole():
    bus = LocalBus()
    pub = PersonPublisher(bus=bus, min_interval_s=100)
    pub.publish_many([(0.2, 0.9), (0.7, 0.8)])
    pub.publish_many([(0.2, 0.9), (0.7, 0.8)])
    assert len([1 for t, _ in bus.log if t == "forge/vision/person"]) == 2


def test_single_publish_still_works():
    bus = LocalBus()
    PersonPublisher(bus=bus, min_interval_s=0).publish(0.5, 0.9)
    assert len(bus.log) == 1
