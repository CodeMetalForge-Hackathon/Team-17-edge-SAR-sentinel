# Project context: Forge 2026, Operation Lost and Found

## Event
- Code Metal Forge Hackathon 2026, BASE-X Boston. Team of 4.
- Mission: search and rescue in dangerous environments with no GPS, cell networks, or fixed comms. Four objectives: locate isolated people, detect life and hazards, map changing conditions, deliver critical info over degraded channels. Brief says pick one, or one hard piece of one.
- Build window: Sat Nov 7 10:00 to Sun Nov 8 12:00 (hard cutoff). Only work done in this window counts. Git history is the proof, so commit often.
- Judging, 25% each: real-world relevance, working live demo, deployability (hardware, power, latency, bandwidth, reliability), ideation. Same criteria in both rounds.
- Not scored: code cleanliness, pitch polish, production readiness, a finished product. Partial progress on a hard problem still scores.
- Round 1: table judging, Sun 12:00-14:00, about 15 min with one judge. Round 2: top 5 present for 10 min on the main stage at 14:45. Awards around 16:15.
- Have answers to: Can you show it working? What existed before, and what did you build here? What worked, what didn't, what's unfinished? What would it take to run outside your laptop?
- AI coding tools are allowed. Someone on the team must understand what they wrote.
- Drones fly only with Code Metal staff present, in daytime. Mentors (including Special Forces operators) are on the floor.

## Hardware available
- Raspberry Pi 5 with AI HAT (Hailo) and Pi camera
- RPLIDAR C1 and S2 (2D). Livox MID-360 is shared, so we don't depend on it
- Arduino boards
- Seeed Grove Creator Kit: Mini PIR, sound and loudness, flame, vibration (SW-420), 3-axis accelerometer, water, moisture, light, DHT11 temp/humidity, ultrasonic ranger, buzzer, LEDs, LED bar, RGB stick, LCD, IR emitter/receiver, and more
- Crazyflie 2.1+ (needs a Flow deck v2 and a Crazyradio; confirm at the kit table). Whoop FPV drones exist but aren't easily programmable, so we don't use them
- Limited Jetson Nanos and one shared Jetson Thor (not used)

## The project
**One-liner:** a Pi maps the area with LiDAR, detects anomalies on-device from LiDAR, camera, and sensor nodes, and dispatches a drone to the anomaly's location, with no GPS.

**Plain version:** a watch station with a laser scanner that knows what the empty space looks like, a camera with an AI chip that spots people, and small sensor boxes that listen for tapping, feel shaking, and detect motion, fire, and water. The Pi learns what normal looks like, flags unusual events, works out what kind of situation it is, and responds:
- Someone needs help: send the drone to them and buzz a beacon near them so they know help is coming.
- Fire, flooding, or possible collapse: mark the area as no-go and alert the human team.
- Someone stuck inside a dangerous area, or the drone's path crosses one: hold the drone and alert humans instead.
- New debris: update the map.

Everything runs on the device. No internet, GPS, or cell network needed.

## Key design decisions and why
- **No synthetic labeled dataset.** The original idea was a generated event CSV with yes/no anomaly labels. Dropped because the model would just learn our generator, and judges would ask who labeled the anomalies. Anomalies now come from real sensors, with models fit only on normal data.
- **No SLAM.** The LiDAR stays in one spot as a sentinel, so its own frame is the map. This removes a whole risky component.
- **GPS-free localization.** Camera gives the direction to a person, and LiDAR gives the distance in that direction. Together they place the person on the map with no GPS and no depth camera.
- **Crazyflie, not the whoops.** It has a clean Python API (cflib) and can fly to relative positions with a Flow deck.
- **Two-layer anomaly model.** Plain IsolationForest missed taps and can't see sensors that never vary in normal data (flame, water, PIR). So a per-sensor robust z-score sits alongside it and also reports which sensor triggered.
- **Rule-based response policy.** Easy to explain to a judge and easy to tune after talking to operators.
- **Sensors on Arduinos, not the Pi.** The Pi has no ADC for the analog Grove sensors. Separate nodes also give a better deployability story: cheap sensor nodes in the field, Pi as the edge brain.

## Coordinate frame
- Map frame = LiDAR frame. Origin at the LiDAR center, +x = LiDAR 0 degrees (forward), +y = 90 degrees counter-clockwise (left), meters.
- RPLIDAR angles increase clockwise, so `LIDAR_ANGLE_SIGN = -1`. Flip it if left and right look mirrored.
- Camera sits next to the LiDAR, aimed along its 0 degrees. Use `CAMERA_ANGLE_OFFSET_DEG` if not.
- Node and drone pad positions are tape-measured into `pi/config.py`.

## Repo layout (forge-sar)
- `arduino/sensor_node/sensor_node.ino`: reads Grove sensors and prints one JSON line every 100 ms: `{node, ms, pir, vib, flame, water, loud_max, loud_mean, light, temp?, hum?}`. Accepts `B` (beacon: buzzer + LED for 2 s) and `A` (ack blink). Pins: D2 PIR, D3 vibration, D4 flame, D5 water, D6 buzzer, D7 LED, A0 loudness, A1 light, D8 DHT11 (optional). Check the `*_ACTIVE` polarity settings on real boards.
- `pi/config.py`: positions, thresholds, ports, drone settings.
- `pi/bus.py`: MQTT bus (paho-mqtt v2) plus an in-process bus for sims and tests. Topic docs at the top.
- `pi/features.py`: 1 s of node readings becomes 11 features: loud_max, loud_mean, loud_std, light_mean, light_delta, pir_frac, vib_hits, flame_any, water_any, temp, hum.
- `pi/node_bridge.py`: Arduino serial to feature windows to MQTT. Relays beacon commands. `--log` writes `normal_features.csv` for training.
- `pi/lidar_node.py`, `pi/lidar_change.py`: reads the Slamtec SDK `ultra_simple` output, builds a baseline from about 10 s of empty-scene scans, and detects changes. A bin counts as changed if something is 0.25 m or more closer than baseline. Blobs must be at least 3 degrees wide and persist for 3 scans, and each is emitted once. Publishes the full scan at 2 Hz. C1 baud 460800, S2 1000000.
- `pi/hailo_hook.py`: converts a person's bounding-box center to a bearing using the camera's field of view, and publishes it. Must be called from the hailo-apps detection callback (check method names against the example you clone).
- `pi/anomaly.py`: robust z-score per feature (Z_THRESH = 6, tail-calibrated so 99.9% of normal windows pass) plus IsolationForest (threshold = 99.9th percentile of training scores + 0.02). Anomalous if either fires. `assess(x)` returns `(score, anomalous, top_feature)`. Simulated results: about 0.2% false alarms per window; tap, shake, flame, and water all flagged.
- `pi/policy.py`: rules in this order:
  1. Flame: FIRE (avoid zone + alert)
  2. Water: FLOOD (avoid zone)
  3. Anomaly with 5+ vibration hits: STRUCTURAL (avoid zone + alert)
  4. Person seen, person near a node, or anomaly with PIR or loudness spike: SURVIVOR. Beacon the node. If inside an avoid zone or the straight pad-to-target path crosses one, alert ground and hold the drone; otherwise dispatch the drone.
  5. LiDAR change with no person: OBSTRUCTION (update map)
  6. Anything else anomalous: LOG
  Avoid zones are 0.7 m circles.
- `pi/brain.py`: subscribes to everything, fuses person bearing with LiDAR range, scores node windows, applies the policy. 15 s cooldown per event type per 0.5 m grid cell. Publishes events and actions, and sends beacon commands.
- `pi/drone_dispatch.py`: converts map coordinates to the drone's takeoff frame (subtract pad position, rotate by pad heading), since the Flow deck only knows motion relative to takeoff. Refuses targets over 3 m away. Takes off to 0.5 m, flies there at 0.3 m/s, hovers 3 s, returns, lands. Dry run by default.
- `pi/simulate.py`: simulated room, nodes, LiDAR, and person, with a 7-step scenario. `--local` runs everything in one process with no broker. Without it, it publishes to MQTT as a hardware fallback.
- `pi/dashboard.py`: live map (LiDAR outline, nodes, pad, events, hazard zones, dispatch arrows, response log). `--sim-snapshot` saves a PNG.
- `pi/reset.py`: clears hazard zones, events, and cooldowns between judges (or press R in the dashboard).
- `run_pi.sh`: starts everything on the Pi. Modes: default (hardware, drone dry run), `sim`, `fly`, `baseline`. Logs go to `logs/`.
- `mac/sync_to_pi.sh`: copies code from the laptop to the Pi without touching the Pi's venv, model, or baseline.
- `mac/dashboard.sh`: runs the dashboard on the laptop against the Pi's broker (sets `FORGE_MQTT_HOST`).
- `tests/test_core.py`: 6 tests, all passing.
- `README.md` (setup, owners, demo script) and `THIRD_PARTY.md` (what existed before: Slamtec SDK, hailo-apps, cflib, PyPI libraries).

## MQTT contract (don't change payloads without telling the integration owner)
- `forge/node/<id>/features`: `{node, x, y, ts, features{...}}`
- `forge/node/<id>/cmd`: `{cmd: "beacon" | "ack"}`
- `forge/lidar/scan`: `{ts, ranges_m[360]}` (index = map degree counter-clockwise, null = no return)
- `forge/lidar/change`: `{ts, x, y, bearing_deg, range_m, width_deg, size_m}`
- `forge/vision/person`: `{ts, bearing_deg, conf}`
- `forge/event`: `{id, ts, kind, x, y, score, anomalous, source, detail}`
- `forge/action`: `{event_id, ts, action, x, y, reason, avoid_zones[]}`
- `forge/drone/status`: `{event_id, ok, ts}`

## Roles
- **A, LiDAR:** build the Slamtec SDK, get `ultra_simple` scanning, record the baseline, produce live change events.
- **B, vision and nodes:** flash the Arduino sketch, check JSON and sensor polarity, get Hailo person detection hooked into `hailo_hook.py`.
- **C, drone:** Flow deck and Crazyradio, hover with Bitcraze's `motion_commander_demo.py`, then `drone_dispatch.py`. Book a supervised flight slot and record a flight clip Saturday.
- **D (Pratik), brain and integration:** broker, brain, policy, anomaly model, dashboard. Swap real modules in for the simulator one at a time.

## Status as of Sat ~13:00
- Full pipeline written and passing in simulation. Code pushed to the team GitHub repo.
- Pi set up: broker running, environment installed, tests and simulation pass on the Pi. Launcher, reset, and sync scripts added.
- Nothing tested on real hardware yet.

## Timeline
- Sat 12:00-18:00: get each risky piece working on its own, in parallel
- Sat 18:00-02:00: thinnest end-to-end path on real hardware
- Sun 02:00-08:00: harden; demo must run 3 times in a row
- Sun 08:00-11:00: feature freeze, retrain the anomaly model on real normal data at our table (venue noise), record backup videos, rehearse
- Sun 11:00-12:00: final push, submit before 12:00

## Open TODOs
- Ask an operator mentor what they'd do with an automatic survivor alert, and fold the answer into the policy
- Record a supervised Crazyflie flight clip
- Retrain per node if the nodes sit in very different noise environments

## Known gaps (be upfront with judges)
- The drone flies straight lines; no path planning around hazard zones
- Flow deck drift: expect about 10-30 cm error at the target
- Person placement assumes the camera points along LiDAR 0 degrees
- One anomaly model is shared across all nodes
- Event "score" means different things by source (z-score ratio, YOLO confidence, blob size), so scores aren't comparable across sources

## Demo plan
- Setup: a cardboard-walled area about 1 m x 1.5 m. Pi, LiDAR and camera at one end, two nodes inside, a taped drone pad, dashboard on a screen facing the judge.
- Scenes:
  1. Tap near node 1: survivor, beacon, drone arrow, plus the recorded flight clip
  2. IR remote at node 2's flame sensor: fire zone appears
  3. Teammate walks into camera view inside the fire zone: drone held, ground team alerted. This is the key "judgment call" moment.
  4. Drop a box: obstruction
  5. Let the judge tap or shake a node themselves
- Fallbacks: `simulate.py` through the real brain and map, the recorded flight clip, and a full recorded run.
- Finals (10 min): 1 min problem, 1 min how it works, 5 min demo, 1 min deployability, 1 min what's next, 1 min buffer.
