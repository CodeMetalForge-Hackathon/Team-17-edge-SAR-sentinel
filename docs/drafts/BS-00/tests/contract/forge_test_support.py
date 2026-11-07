"""Shared helpers for contract acceptance tests. No fixtures or conftest, on purpose."""
import contextlib
import functools
import math
import os
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PI = os.path.join(REPO, "pi")
if PI not in sys.path:
    sys.path.insert(0, PI)

import config  # noqa: E402
from bus import LocalBus  # noqa: E402
from features import window_features  # noqa: E402

_MISSING = object()


@contextlib.contextmanager
def cfg(**values):
    """Temporarily change config values, then restore them. Never leaves a threshold changed."""
    old = {k: getattr(config, k, _MISSING) for k in values}
    for k, v in values.items():
        setattr(config, k, v)
    try:
        yield
    finally:
        for k, v in old.items():
            if v is _MISSING:
                delattr(config, k)
            else:
                setattr(config, k, v)


@functools.lru_cache(maxsize=1)
def trained_model():
    """The real model from anomaly.py, fit on the simulator's normal data."""
    from anomaly import NodeAnomalyModel
    from simulate import synthetic_normal_windows
    return NodeAnomalyModel.train(synthetic_normal_windows(800))


def brain_with(model=None, **kw):
    from brain import Brain
    bus = LocalBus()
    return bus, Brain(bus, model, **kw)


def kinds(bus):
    return [p["kind"] for t, p in bus.log if t == "forge/event"]


def actions(bus):
    return [p["action"] for t, p in bus.log if t == "forge/action"]


def statuses(bus):
    return [p for t, p in bus.log if t == "forge/status"]


def node_msg(feat, nid="n1", x=1.5, y=0.8, **extra):
    return {"node": nid, "x": x, "y": y, "ts": time.time(), "features": feat, **extra}


def feat(kind="normal", prev=600.0):
    from simulate import node_window
    return window_features(node_window(kind), prev)


def fresh_scan(objects=(), ts=None):
    from simulate import room_scan
    return {"ts": time.time() if ts is None else ts, "ranges_m": [round(float(r), 3) for r in room_scan(objects)]}


def bearing_to(x, y):
    return math.degrees(math.atan2(y, x)) % 360


def person_msg(x, y, ts=None):
    return {"ts": time.time() if ts is None else ts, "bearing_deg": bearing_to(x, y), "conf": 0.9}
