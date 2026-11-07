# Project context: Forge 2026, Operation Lost and Found

Last updated: Sat Nov 7, ~15:00. Supersedes the earlier "LiDAR sentinel" context. Read this first, top to bottom.

## Event
- Code Metal Forge Hackathon 2026, BASE-X Boston. Team of 4.
- Mission: search and rescue in dangerous environments with no GPS, cell networks, or fixed comms. Four objectives: locate isolated people, detect life and hazards, map changing conditions, deliver critical info over degraded channels. Brief says pick one, or one hard piece of one.
- Build window: Sat Nov 7 10:00 to Sun Nov 8 12:00 (hard cutoff). Only work done in this window counts. Git history is the proof, so commit small and often.
- Judging, 25% each: real-world relevance, working live demo, deployability (hardware, power, latency, bandwidth, reliability), ideation. Same criteria in both rounds.
- Not scored: code cleanliness, pitch polish, production readiness, a finished product. Partial progress on a hard problem still scores.
- Round 1: table judging, Sun 12:00-14:00, about 15 min with one judge. Round 2: top 5 present for 10 min on the main stage at 14:45. Awards around 16:15.
- Have answers to: Can you show it working? What existed before, and what did you build here? What worked, what didn't, what's unfinished? What would it take to run outside your laptop?
- AI coding tools are allowed. Someone on the team must understand what they wrote.
- Drones fly only with Code Metal staff present, in daytime. Mentors (including Special Forces operators) are on the floor.

## The project (after the Sat 14:45 pivot)
**One-liner:** HQ sees the area from above before and after a disaster, plans a safe route, sends a ground robot to inspect, and shows responders one live map of debris, hazards and people. No GPS, no internet.

**Core loop:**
1. **Aerial image -> road map.** An overhead image of the sandbox is lined up to our grid using 4 corner markers. HQ compares it with the "before" image square by square; changed squares are blocked.
2. **Route.** HQ plans the robot's route around blocked and hazard squares (A* on the grid).
3. **Inspect.** The robot drives the route square by square and reads its sensors (survivor signs, flame, gas stand-in, etc.), sending `{square, readings}` back to HQ.
4. **Dashboard.** HQ shows everything live: before/after, blocked squares, hazards, robot position and path, findings ("person D2", "flame E3").

**Aerial source, in order:**
- **Now: a stand-in aerial camera** on a pole (~1.2 m above the sandbox), taking a snapshot every 30 s.
- **Later, if time: the Crazyflie** (we have one) as the real aerial asset. It only gets a camera with an AI deck (not confirmed). The mapping code is the same for any image source, since every image is lined up by the corner markers.

**What we say to judges:** "In the field the aerial image comes from a drone or tethered drone. We didn't get the drone working this weekend, so this camera stands in for it. HQ treats it exactly like drone imagery: a photo every 30 s, lined up by the corner markers. Swap in a drone and nothing downstream changes." Keep this true: run the stand-in on a 30 s snapshot cycle, and line up every frame with the markers.

## Demo flow (~5 min of the 15)
1. Before: HQ captures the empty sandbox from above and calibrates from the corner markers.
2. Event: boxes dropped as debris, IR remote at the flame sensor as fire, a "victim" in a square.
3. Survey: next aerial snapshot; HQ marks changed squares as blocked.
4. Route: HQ plans start -> target around blocked/hazard squares, sends it to the robot.
5. Inspect: robot drives square by square, reports readings; findings appear on the map.
6. Judge moves a box or triggers a sensor; map and route update.

Fallbacks: hand-carried sensor box along the planned route (if the robot isn't ready), `simulate.py` through the real brain, recorded videos of each step.

## Coordinate grid (we define the coordinates)
- Sandbox 1.5 m x 1.0 m, taped grid of 25 cm squares: columns A-F, rows 1-4 (24 squares). Confirm square size against the robot's footprint.
- Grid frame: origin at the outer corner of A1, +x along columns A->F, +y along rows 1->4, meters. Square centre A1 = (0.125, 0.125), F4 = (1.375, 0.875).
- 4 ArUco markers (DICT_4X4_50) at the sandbox corners map image pixels to meters. Print large enough to be detected from the camera height (~10 cm for the Pi camera; ~20 cm if the AI-deck drone camera is used).
- 1 ArUco marker on top of the robot (its position and heading).
- Drone pad square and robot start square are marked on the tape.

## Key design decisions and why
- **Fixed coordinates, no GPS, no SLAM.** Removes localization as a risk; everything is a named square.
- **Overhead image as the mapper** (not the LiDAR). A 2D LiDAR sees one horizontal slice from one spot (shadows behind objects); from above it would see one line, not an area. The overhead image sees the whole sandbox.
- **Corner markers on every image.** Any aerial source (pole camera, drone) gets lined up the same way, so drone drift doesn't matter.
- **Robot carries the close-up sensors** (what a camera can't see: flame, gas, a survivor's sound/motion).
- **Rule-based response policy** in the brain: easy to explain and tune.
- **Everything on the Pi** with a local MQTT broker and the Pi's own Wi-Fi hotspot for the robot.
- **Earlier decisions still valid:** no synthetic labelled dataset (anomalies from real sensors, models fit on normal data only); two-layer anomaly model (robust z-score per feature + IsolationForest).

## Hardware
| Item | Use | Status |
|---|---|---|
| Raspberry Pi 5 + AI HAT (Hailo) | HQ: broker, brain, mapping, planner, dashboard; Hailo person detection optional | Set up (broker, venv, tests pass) |
| Pi Camera (Module 3 Wide preferred, ~102 deg) or USB webcam | Stand-in aerial camera on a pole | Connecting now |
| Robot with Arduino UNO Q | Drives the route, carries Grove sensors | Chassis/motors/encoders not confirmed |
| Crazyflie 2.1+ (+ Flow deck v2, Crazyradio) | Aerial asset later; camera only with an AI deck | Have the Crazyflie; decks to confirm |
| RPLIDAR C1/S2 (2D) | Optional cross-check of blocked squares at floor level | Rohan |
| Seeed Grove kit | PIR, loudness, flame, vibration, water, light, DHT11, buzzer, LED, ... | No gas sensor in the kit list; gas needs a stand-in unless an MQ-type sensor turns up |
| BetaFPV Aquila16 | Not used (hand-flown, analog video, not programmable) | Dropped |

**Arduino UNO Q notes:** two processors (Linux MPU + STM32 MCU). `Serial` goes through the Router Bridge to its own Linux side, not to USB. Header UART = `Serial1` on D0/D1, 3.3 V. On a moving robot, use its Linux side + Wi-Fi to reach the Pi's broker (no cable).

**Pi camera notes:** Pi 5 needs the Pi 5 camera cable (22-pin). Check with `rpicam-hello --list-cameras`, test with `rpicam-still -o test.jpg --nopreview`. `picamera2` is a system package: the venv needs `--system-site-packages`, or run camera code with system `python3`.

## System and messages
Pi HQ runs mosquitto. Existing topics stay; new topics are proposed (owner: Pratik).

Existing:
- `forge/node/<id>/features`: `{node, x, y, ts, features{...}}`
- `forge/node/<id>/cmd`: `{cmd: "beacon" | "ack"}`
- `forge/lidar/scan`: `{ts, ranges_m[360]}`, `forge/lidar/change`: `{ts, x, y, bearing_deg, range_m, width_deg, size_m}`
- `forge/vision/person`: `{ts, bearing_deg, conf}`
- `forge/event`: `{id, ts, kind, x, y, score, anomalous, source, detail}`
- `forge/action`: `{event_id, ts, action, x, y, reason, avoid_zones[]}`
- `forge/drone/status`: `{event_id, ok, ts}`
- `forge/control/reset`: `{ts}` (dashboard R key / `reset.py`)

New (proposed):
- `forge/map/grid`: `{ts, cols, rows, cell_m, blocked[], hazards[]}` (squares as "C3")
- `forge/map/snapshot`: `{ts, kind: "before" | "after", path}`
- `forge/robot/plan`: `{ts, start, goal, waypoints[]}`
- `forge/robot/status`: `{ts, x, y, heading_deg, cell, state}`
- `forge/robot/cmd`: `{cmd: "go" | "stop", cell}`
- `forge/robot/reading`: `{ts, cell, readings{pir, flame, gas?, loud_max, ...}}`

## Repo (github.com/CodeMetalForge-Hackathon/Team-17)
Branches: `main` (initial bulk commit only), `pratik` (PR #1 open: run_pi.sh, reset, mac scripts), `Dhrubo` (drone + node fixes + UNO Q + Pratik's branch merged + owners), `Rohan`. Nothing merged to `main` yet: merge PR #1, then `Dhrubo`, so the Pi runs one codebase.

Existing files, status in the pivot:
- `pi/bus.py` MQTT + in-process bus. Reused.
- `pi/brain.py`, `pi/policy.py`, `pi/anomaly.py`, `pi/features.py`: events, hazards, anomaly model. Reused; robot readings feed them.
- `pi/dashboard.py`: live map, R to reset. Reused; needs grid layers and before/after.
- `pi/drone_dispatch.py` (+ preflight: Flow deck, battery >= 3.8 V, Kalman reset; safe landing on errors; `--leg FWD LEFT`) and `pi/drone_check.py` (`--scan`, check, `--flow`, `--hover`). For the Crazyflie stage later.
- `arduino/sensor_node/sensor_node.ino`: sensor sketch. vib reported 0/1 per 100 ms (fixes tap-reads-as-STRUCTURAL). Output on `NODE_SERIAL` (`Serial1` for UNO Q). Basis for the robot's sensor code.
- `pi/node_bridge.py`: serial -> features -> MQTT; `--raw`, `--port`, NODE_ID check, log flush. Pratik may use his own code instead.
- `pi/lidar_node.py`, `pi/lidar_change.py`: LiDAR change detection (optional cross-check).
- `pi/hailo_hook.py`: person bearing from Hailo (optional now).
- `pi/simulate.py`, `tests/test_core.py` (7 tests pass), `run_pi.sh` (modes: default, sim, fly, baseline), `pi/reset.py`, `mac/*.sh`.
- `.gitattributes` keeps `*.sh` LF on Windows checkouts.

Planned new code (not written yet):
- `pi/grid.py`: square names <-> meters, A* route around blocked/hazard squares.
- `pi/overhead.py`: capture (Pi camera / webcam / image file / drone frame) every 30 s, corner-marker calibration, flattened top-down map, before/after per square -> blocked, robot marker -> position, publish grid/status/plan.
- Robot: driving code (go to next square) and Wi-Fi bridge from the UNO Q's Linux side to the Pi broker.
- Dashboard: grid layers, before/after view, robot path, findings.

## Pi access
- User `khoj`, LAN `10.1.200.210`, Tailscale `100.119.240.31`. Code in `~/forge-sar` (synced from Pratik's Mac with `mac/sync_to_pi.sh`, not a git checkout). Venv at `~/forge-sar/.venv`.
- Tailscale is for development only: the demo must not depend on internet.

## Roles
| Who | Owns |
|---|---|
| Dhrubo | Aerial mapping (`overhead.py`, calibration, before/after), A* planner, Crazyflie stage later, vision (Hailo, optional) |
| Pratik | Brain + integration, dashboard, `run_pi.sh`, sensor nodes / robot sensors and its link to HQ |
| Rohan | LiDAR (optional cross-check); spare hands for robot driving |
| Chaitanya | Hardware: robot chassis/motors, camera mount, sandbox + tape grid + markers, flight spotting |
| Not owned yet | Robot driving code (go to a square -> motor commands) |

## Known issues to fix
- **Survivor false trigger (Pratik, policy/anomaly):** any anomalous window + any PIR blip = SURVIVOR, even if the anomaly came from slow drift (light level, temp). Fix: drop `temp`, `hum`, `light_mean` from model inputs; SURVIVOR only when the trigger itself is PIR or loudness. Applies to robot readings too.
- **Hazard zones pile up** while a flame persists (reset between judges exists; merge overlapping zones).
- **Lighting:** shadows look like change in the before/after compare; steady light, per-square tolerance, retake "before" right before each run.
- **Git on Windows:** commit with the GitHub noreply email (`50300187+dhrubo404@users.noreply.github.com`), or pushes are rejected.

## Timeline
- Sat 14:45-18:00: each piece alone (camera on Pi + calibration + before/after; robot drives one square; A* planner; Crazyflie hover if time).
- Sat 18:00-02:00: one full run: aerial map -> route -> robot -> readings on the dashboard.
- Sun 02:00-08:00: harden; demo 3x in a row; backup videos.
- Sun 08:00-11:00: feature freeze; retake "before" at our table; Crazyflie stage only if everything else is solid (daylight + staff); rehearse.
- Sun 11:00-12:00: final push, submit before 12:00.

## Open questions
- Robot hardware: wheels, motor driver, encoders? Who writes the driving code?
- Gas: real MQ-type sensor, or a stand-in?
- Camera: Pi Camera Module 3 Wide on a long ribbon, or USB webcam? Mount?
- Grid: 25 cm squares OK for the robot?
- Can the UNO Q's Linux side join the Pi's hotspot and reach the broker?
- Does the Crazyflie have an AI deck? (needed for drone images)
- Operator mentor feedback on the new flow.

## Known gaps (be upfront with judges)
- Aerial camera is a stand-in for a drone (see wording above).
- Robot position between snapshots comes from driving square by square; drift corrected at each snapshot.
- Straight 4-connected routes on a 25 cm grid; no fine motion planning.
- Gas sensing is a stand-in unless a real gas sensor is found.
