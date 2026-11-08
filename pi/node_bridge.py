"""Arduino serial -> feature windows -> MQTT. Also relays beacon commands back to the node.

Run one per node on the Pi:
    python node_bridge.py n1
    python node_bridge.py n1 --log      # also append windows to normal_features.csv (for training)
"""
import sys
import time
import json
import csv
import os

import config
from bus import make_bus
from features import window_features, FEATURE_NAMES


def main():
    node_id = sys.argv[1]
    log_normal = "--log" in sys.argv
    node = config.NODES[node_id]

    import serial
    ser = serial.Serial(node["port"], config.SERIAL_BAUD, timeout=1)
    time.sleep(2)  # Arduino resets when the port opens
    bus = make_bus(client_id=f"bridge-{node_id}")

    def on_cmd(topic, payload):
        cmd = payload.get("cmd")
        if cmd == "beacon":
            ser.write(b"B\n")
        elif cmd == "ack":
            ser.write(b"A\n")
    bus.subscribe(f"forge/node/{node_id}/cmd", on_cmd)

    writer = None
    if log_normal:
        new = not os.path.exists(config.NORMAL_LOG)
        f = open(config.NORMAL_LOG, "a", newline="")
        writer = csv.writer(f)
        if new:
            writer.writerow(["node", "ts"] + FEATURE_NAMES)

    buf, t0, prev_light = [], time.time(), None
    print(f"[bridge {node_id}] reading {node['port']}")
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if line.startswith("{"):
            try:
                buf.append(json.loads(line))
            except ValueError:
                pass
        if time.time() - t0 >= config.FEATURE_WINDOW_S and buf:
            feat = window_features(buf, prev_light)
            prev_light = feat["light_mean"]
            msg = {"node": node_id, "x": node["x"], "y": node["y"], "ts": time.time(), "features": feat}
            bus.publish(f"forge/node/{node_id}/features", msg)
            if writer:
                writer.writerow([node_id, msg["ts"]] + [feat[k] for k in FEATURE_NAMES])
            buf, t0 = [], time.time()


if __name__ == "__main__":
    main()
