# Forge 2026: Operation Lost and Found (edge SAR sentinel)

A Raspberry Pi 5 with an AI HAT watches an area with a LiDAR, a camera, and cheap Arduino sensor nodes. It detects anomalies on-device, places them on a GPS-free map, decides on a response, and dispatches a Crazyflie to the spot (or holds it and alerts the ground team if the path crosses a hazard).

**One-liner:** the Pi maps the area with LiDAR, detects anomalies on-device from LiDAR, camera, and sensor nodes, and dispatches a drone to the anomaly's location, with no GPS.

## Architecture

```
Arduino nodes (Grove sensors) --USB serial--> node_bridge.py --\
RPLIDAR C1/S2 --ultra_simple--> lidar_node.py ------------------+--> MQTT (mosquitto on Pi) --> brain.py --> drone_dispatch.py --> Crazyflie
Pi camera + Hailo --hailo-apps callback--> hailo_hook.py -------/                                 |   \--> node cmd: beacon
                                                                                                  \--> dashboard.py (live map)
```

Map frame = the LiDAR's own frame (it doesn't move, so no SLAM). Everything is in meters from the LiDAR center. Measure node and drone pad positions with a tape and put them in `pi/config.py`.

## Quick start

```bash
pip install -r requirements.txt
cd pi
python simulate.py --local            # whole pipeline, no hardware, no broker
python dashboard.py --sim-snapshot    # renders dashboard_sim.png
python ../tests/test_core.py          # unit tests
```

On the Pi with hardware:

```bash
sudo apt install -y mosquitto && sudo systemctl enable --now mosquitto
python anomaly.py train --synthetic                      # placeholder model until real normal data exists
python node_bridge.py n1 &                               # one per Arduino
./ultra_simple --channel --serial /dev/ttyUSB0 460800 | python lidar_node.py --baseline   # empty scene, once
./ultra_simple --channel --serial /dev/ttyUSB0 460800 | python lidar_node.py &
python brain.py &
python drone_dispatch.py &                               # dry run by default
python dashboard.py                                      # or on a laptop with MQTT_HOST = Pi's IP
```

Then retrain on real data: `python node_bridge.py n1 --log` for ~5 min of normal activity, then `python anomaly.py train`.

## WAVE ROVER keyboard control

A separate Wi-Fi controller provides browser-based hold-to-move WASD, release-to-stop, Space stop/disarm, and a Pi-side heartbeat timeout at power 0.30. It uses the existing rover HTTP protocol and starts in dry-run mode. The browser connects through an SSH tunnel; ordinary SSH terminal input does not reliably report key release.

See [the rover setup and validation guide](rover/README.md) for installation, controls, live testing, and failure limits. It does not change the existing `rover_wifi.py` or Wi-Fi/Tailscale settings. No rover-side watchdog is assumed.

## Owners

| Module | Files | Owner |
|---|---|---|
| A. LiDAR | `lidar_node.py`, `lidar_change.py` | |
| B. Vision + nodes | `hailo_hook.py`, `arduino/`, `node_bridge.py` | |
| C. Drone | `drone_dispatch.py` | |
| D. Brain + integration | `brain.py`, `anomaly.py`, `policy.py`, `dashboard.py`, `simulate.py` | |

Message topics and payloads are documented at the top of `pi/bus.py`. Don't change a payload without telling the integration owner.

## Anomaly model

Fit only on normal data, no labels:
- **Robust z-score per feature** (median/MAD, tail-calibrated so 99.9% of normal windows pass). It catches one sensor going off-scale and reports which one (`trigger`).
- **IsolationForest** over all features, threshold calibrated on training scores. It catches odd combinations.

On simulated data: about 0.2% false alarms per 1 s window, and tap, shake, flame, and water are all flagged.

## Demo script (table judging, ~15 min)

1. Show the live map: LiDAR outline, nodes, drone pad.
2. Tap on the table next to n1. SURVIVOR fires, the node beacons, and the drone dispatches (live flight if organizers allow, otherwise dry-run arrow plus recorded video).
3. Point an IR remote at n2's flame sensor. FIRE fires and a hazard zone appears.
4. Walk into the camera's view inside that zone. The camera bearing plus LiDAR range places you on the map. SURVIVOR fires, but the drone is held and the ground team is alerted.
5. Drop a box in the scanned area. OBSTRUCTION fires and the map updates.
6. Fallback: `python simulate.py` replays the same scenario through the real brain if any sensor dies.

## Answers for the judges

- **What existed before:** see `THIRD_PARTY.md`. Everything in `pi/` and `arduino/` was written this weekend (see git history).
- **Off the laptop:** everything runs on a Pi 5 (~10 W) with no network beyond a local MQTT broker. The nodes are $10-class microcontrollers. The model is a few hundred KB. Swapping USB for LoRa on the nodes is the next step for range.
