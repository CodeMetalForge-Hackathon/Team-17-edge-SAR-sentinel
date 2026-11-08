"""Tiny message bus. MqttBus for real runs, LocalBus for single-process tests/sims.

Topics (all payloads are JSON dicts):
  forge/node/<id>/features   node feature window       {node, x, y, ts, features{...}}
  forge/node/<id>/cmd        command to a node          {cmd: "beacon" | "ack"}
  forge/lidar/scan           latest scan, 1 deg bins    {ts, ranges_m[360]}   (map-frame degrees, CCW)
  forge/lidar/change         persistent LiDAR change    {ts, x, y, bearing_deg, range_m, width_deg, size_m}
  forge/vision/person        person seen by camera      {ts, bearing_deg, conf}
  forge/event                fused + scored event       {id, ts, kind, x, y, score, anomalous, sources, detail}
  forge/action               chosen response            {event_id, ts, action, x, y, reason}
"""
import json
import fnmatch
from collections import defaultdict


def _mqtt_to_glob(pattern):
    return pattern.replace("+", "*").replace("#", "*")


class LocalBus:
    def __init__(self):
        self.subs = defaultdict(list)
        self.log = []

    def publish(self, topic, payload):
        self.log.append((topic, payload))
        for pat, cbs in list(self.subs.items()):
            if fnmatch.fnmatch(topic, _mqtt_to_glob(pat)):
                for cb in cbs:
                    cb(topic, payload)

    def subscribe(self, pattern, cb):
        self.subs[pattern].append(cb)

    def loop_forever(self):
        pass


class MqttBus:
    def __init__(self, host, port=1883, client_id=None):
        import paho.mqtt.client as mqtt
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id or "")
        self.subs = []
        self.c.on_message = self._on_message
        self.c.on_connect = self._on_connect
        self.c.connect(host, port, keepalive=30)
        self.c.loop_start()

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        for pat, _ in self.subs:
            client.subscribe(pat)

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload)
        except ValueError:
            return
        for pat, cb in self.subs:
            if fnmatch.fnmatch(msg.topic, _mqtt_to_glob(pat)):
                try:
                    cb(msg.topic, payload)
                except Exception as e:  # one bad handler shouldn't kill the bus
                    print(f"[bus] handler error on {msg.topic}: {e}")

    def publish(self, topic, payload):
        self.c.publish(topic, json.dumps(payload))

    def subscribe(self, pattern, cb):
        self.subs.append((pattern, cb))
        self.c.subscribe(pattern)

    def loop_forever(self):
        import time
        while True:
            time.sleep(1)


def make_bus(local=False, client_id=None):
    if local:
        return LocalBus()
    import config
    return MqttBus(config.MQTT_HOST, config.MQTT_PORT, client_id)
