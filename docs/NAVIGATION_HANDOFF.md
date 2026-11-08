# Navigation handoff (camera -> rover), written 2026-11-08 ~00:35

Everything marked **FRESH** was read from the live system while writing this file. Everything marked **HISTORICAL**
happened earlier in the session and may no longer be true. Nothing here was committed, pushed or merged by the author.

## 1. Objective and the next task

Demo: the operator clicks an exact destination on the live overhead view; the system locates the rover with the overhead
depth camera, plans a trajectory that respects the rover footprint, obstacles and the tabletop boundary, drives there on
live camera feedback, then STOPs and disarms. Target scenario used all session: rover starts right-centre of the table,
destination is left-centre, with small obstacles in between.

**Next task (not started): optimise and complete a real run with the current controller.**
1. On target selection take ONE fresh RGB+depth snapshot, build the obstacle map from it and plan the whole trajectory to
   the exact point. Reuse that map while the scene is unchanged. Refresh the displayed overview about once a minute or on
   demand. Do not re-run depth mapping / full planning / rendering on every frame.
2. Keep only lightweight work per frame: rover position+heading, boundary/footprint check, stale-frame and STOP/watchdog
   monitoring (rate chosen from measured speed and latency). Trigger an expensive rebuild only on a meaningful change,
   tracking uncertainty, obstruction or route deviation.
3. Smoother motion: fewer stop/start bursts, continuous bounded control (streamed wheel commands with a watchdog already
   exist as "cruise"), prefer a reverse leg to a 180 degree turn (already implemented), slow near obstacles, edges, target.
4. **No blind driving**: never more than about one observation latency without a fresh pose; stale tracking, inadequate
   clearance, comm loss or unexpected movement => STOP and diagnose while stationary.
5. Profile before optimising (nothing was formally profiled yet, see section 9), remove redundant fetches/blocking calls/
   duplicate executors/excess rendering, keep the useful safety checks, do not delete tests to hide failures.
6. Then do a real supervised run (authorised, see section 11), verify arrival within the existing 50 mm tolerance from
   fresh camera observations, STOP, disarm, report time and position error.

## 2. Hardware (verified)

| Part | Detail |
|---|---|
| Rover | **Waveshare UGV01**, skid-steer, ESP32 access point SSID `UGV`, web/JSON API at `http://192.168.4.1` (`GET /js?json={...}`). Wheel command `{"T":1,"L":l,"R":r}` (UGV units, about +-0.5), feedback `{"T":130}` returns `L,R` and IMU `y` (yaw deg, noisy/drifting, occasional glitches). The firmware is NOT the old M5 RoverC UDP sketch (`rover/M5_RPi.ino` is unrelated to this robot). Deadband: forward below ~0.2 command barely moves; in-place turns need >= ~0.5. Teammate docs say the controller self-stops after ~3 s of silence (not independently verified). |
| Camera | **Aurora930** on a **Pi Zero 2 W** `zero@100.96.176.12` (Debian 13, 415 MB RAM). Vendor aarch64 SDK works unmodified (`~/aurora/sdk`). Capture `aurora_cap --mode 2 --fps 5`: aligned RGB (NV12) + 16-bit depth, 640x400, depth in millimetres (0 = invalid), measured 5.00 fps, 0 USB errors. Mounted about 1.9 m above the table, slightly tilted (fitted ground-plane normal ~ (-0.03,-0.07,1.0)). |
| Controller | **Raspberry Pi 5** `khoj@100.119.240.31` (Debian 13, 16 GB, cv2 4.10, numpy). `wlan0` = guest Wi-Fi (default route, Tailscale), `wlan1` = SSID `UGV` at 192.168.4.2 (rover link, never default). Runs the navigator. |
| Network | Pi and Zero cannot reach each other on the guest LAN (client isolation); they talk over Tailscale through a DERP relay. This is the main latency source. |
| Rover marker | Operator attached an **ArUco ID 5** marker at the rover front. It is only ~8 px wide in the image, **ID 5 is NOT decoded reliably (effectively never)**. |

**Heading fallback actually used (verified in the live runs):** position comes from the rover's raised outline in the
depth map ("depth-blob", 35-175 mm above the floor plane); heading comes from the *bright pattern of the marker at the
rover front relative to the dark body centre* (`vision.pattern_heading`, 3-frame circular mean, single-frame noise about
+-7 deg), plus a learned offset `hbias` anchored to the camera-measured direction of travel after forward moves.
If the pattern is unreadable the code can fall back to the controller IMU yaw (`calibrate_heading`, sim-tested only; not
used on hardware). A decoded marker would be used first (`source == "marker"`), sim-tested only.

## 3. Repository, git, files

- Repo `C:\Users\rohan\Documents\GitHub\Team-17`, branch **Rohan**, HEAD `8e87d27 Working rover` (earlier `f949038`,
  `5550447`). Someone already committed earlier versions of the navigator; **my later work is uncommitted**.
- `git status` (FRESH): modified `tests/test_navigator.py`, `tools/aurora/frame_server.py`, `tools/navigator/{control,geom,
  nav.html,perception,pi_start.sh,server,sim,sources,ugv,vision}`; untracked `tools/navigator/measure_latency.py`.
  `tools/overhead_viewer/` is the older manual-setup viewer (still deployed on the Pi, motion disabled); `rover/` holds the
  teammate's old RoverC code and is unrelated to the UGV.
- **Local vs deployed (FRESH diff, line endings ignored): identical** for all navigator `.py` files, `nav.html`, and the
  Zero's `frame_server.py`. The Pi has the latest vision fix (real-depth-pixels-only blob statistics). Caveat: the Pi
  copy of the tests was last run (20 PASS) **before** that final vision change; local run after it: 20 PASS.
- Deployed paths: Pi `~/forge-overhead/tools/navigator/*` (+ `~/forge-overhead/tests/test_navigator.py`), state in
  `~/forge-nav/`, log `~/forge-overhead/navigator.log`. Zero `~/aurora/{aurora_cap,frame_server.py,zero_start.sh,sdk/}`.
- Windows scratch (not in git): `C:\Users\rohan\aurora_dl\` holds downloaded SDK archives, test frames, `runleg.sh`
  (selects a target by API, arms, starts, polls to completion) and **backups**: `backup_20261107_2343\{local,pi_navigator,
  pi_state}` = code and learned state from before the last big rewrite.
- Files edited with Python on Windows get CRLF line endings; scripts that run on the Pi (`pi_start.sh`) must be LF
  (`sed -i 's/\r$//'`) or they fail with "required file not found".

## 4. Launch / recovery (verified working this session)

```bash
# Viewer / control UI (Pi navigator on 127.0.0.1:8790) through an SSH tunnel:
ssh -i C:\Users\rohan\.ssh\forge_lidar -N -L 8790:127.0.0.1:8790 khoj@100.119.240.31      # then open http://127.0.0.1:8790
# (a tunnel on local port 8791 may still be running from this session; the same UI is then at http://127.0.0.1:8791)

# (Re)start the navigator on the Pi. live = real UGV link, still requires "Arm" in the UI. dry = no rover packets.
ssh -f -i C:\Users\rohan\.ssh\forge_lidar khoj@100.119.240.31 "~/forge-overhead/tools/navigator/pi_start.sh live > /tmp/pistart.out 2>&1 < /dev/null"

# (Re)start capture + the CURRENT frame server on the Zero (zero_start.sh launches ~/aurora/frame_server.py, which is the
# current JPEG+quantised-depth version; verified). Not persistent across reboot (enabling systemd units was blocked).
ssh -i C:\Users\rohan\.ssh\forge_lidar zero@100.96.176.12 "~/aurora/zero_start.sh"

# Tests (needs opencv-python-headless + numpy; ~1 min). Same file runs on the Pi: python3 ~/forge-overhead/tests/test_navigator.py
python tests/test_navigator.py
```
Gotchas: `pkill -f` whose pattern also appears in the same remote command line kills your own SSH session (use the
`[x]` bracket trick and separate ssh calls). The navigator needs ~20-30 s after restart before the pose is reliable.
Server flags (see `server.py`): `--enable-motion --rover-mm 170x170 --margin-mm 15 --arrive-tol-mm 50 --no-cruise --sim`.
HTTP API (POST needs header `X-Requested-With: nav`): `/api/target {x,y}|{px,py}`, `/api/arm`, `/api/start`, `/api/stop`,
`/api/state`, images `/api/view.jpg|live.jpg|camera.jpg`, `/api/clear_map`, `/api/recal_ground`, `/api/record`.
No credentials are stored anywhere; SSH uses the key file only.

## 5. Current state (FRESH, read ~00:35; the rover was not moved after this)

- Navigator process running on the Pi with `--enable-motion` but **idle and disarmed**, target = none.
- Rover tracked: normalized pose about (0.72, 0.44), source depth-blob, raw pattern heading ~3.1 rad (facing left).
  Last wheel-command readback: L=0, R=0. The rover is on the table, right of the centre seam (operator reset it there).
- Calibration: boundary live (4 markers; **IDs 1 and 2 are hand-placed static markers** in `~/forge-nav/manual_markers.json`
  because the detector no longer decodes the two left markers; IDs 0 and 3 decode), ground plane fitted, aspect 3.18,
  sandbox height 722 mm. Corner rule: inner-x / outer-y black corner of each marker; roles assigned from image position
  (image left/right/top/bottom), so "left-centre" means image left. If the camera moves, drift detection discards the
  manual markers and they must be re-placed (look at crops of the left markers and write corners to that file).
- Learned motion (`~/forge-nav/model.json`, FRESH): forward model ~0.53 square-frame units/s at command 0.35,
  `hbias` 0.18 rad, `turn_dir` Q = CCW (+1), E = CW. The depth map file was deleted (rebuilds from frames in seconds).
- Environment: no big obstacle now; two small objects on the left table (a spinning/metal cylinder at the top right of the
  left table and one near the bottom centre), a small black item at lower left, a "DO NOT USE THIS SPACE" paper strip
  to the left of the left seam (outside the validated area).

## 6. Previous out-of-bounds incident

HISTORICAL. After a successful arrival at left-centre (observed pose x=0.179) the rover later sat at x=0.063, half over the
left boundary onto the paper strip, where tracking failed. **Confirmed cause: not a navigator-run failure.** Between the
arrival and the discovery I ran direct hardware scripts that bypass the navigator's checks: a turn-response
characterisation (8 in-place pulses at command 0.5 and 0.8, rover slides while turning), IMU-closed turn tests,
`measure_latency.py` (a 0.5 s streamed drive) and a heading-calibration probe launched while the rover was already near the
edge. Skid-steer in-place turns translate the robot; nothing checked the footprint against the boundary. The exact
distance split between those actions was **not measured**, so this is the most likely cause, not a proven one.
Fixes added afterwards (sim-tested; **not yet hardware-validated**): the run refuses to start, before any calibration
probe, if the rover centre is outside or the whole rotated footprint crosses the boundary ("physical reset needed");
the same check runs on every pose; footprint-aware clearance for speed selection; hard per-move footprint check
(+-8 deg heading allowance, latency lead + coasting distance); turns refused where the enclosing circle would sweep an
edge/obstacle; target validation requires the whole rover to fit (offers a suggested point, never alters it).
Unresolved risks: direct scripts still bypass all of this (do not run them near an edge); the boundary quad is the
marker rectangle, not a measured table edge; pose bias or stale pose near the edge; the rover can still coast/slide
after STOP (observed ~50 mm, position settled ~0.8 s after the STOP command).

## 7. Tests and hardware validation

- `tests/test_navigator.py`: **20 tests, all PASS locally** after the last change (planner geometry; vision/pose/ground/
  obstacles; target click mapping and rejection+suggestion; IMU-closed turns; arrival within tolerance; flipped rotation +
  rotated camera; marker mounted 90 deg off; narrow corridor with measured footprint; cruise used and bounded; cruise
  watchdog; stale frames during cruise; re-target single executor; IMU heading without marker (both IMU signs);
  reverse instead of 180 turn; refuses to start outside; unresponsive rover; no route; lost tracking; STOP/disarm).
  The earlier 3 failures were: arrival test (my widened tolerance; fixed by explicit 50 mm), RoverC strafing test
  (obsolete, removed with the strafing code), IMU-heading test (sim lacked IMU-closed turns; fixed in `sim.py`).
- Hardware validation actually completed (HISTORICAL, older controller versions): live RGB+depth capture; route around
  obstacles to left-centre in 91 s reaching 0.12 sandbox-heights (~85 mm, previous looser tolerance) from the target and
  stopping/disarming (wheels 0,0); IMU-closed turns (36 deg target -> 37 deg turned); transport diagnosis (UGV HTTP).
  **Not validated on hardware with the current controller:** cruise streaming, reverse legs, footprint/boundary guard,
  retarget, snapshot behaviour. The last attempted run with the new controller failed safely (tracking lost) because of a
  vision bug since fixed (offline: pose found 16/16 frames; before 6/14). No completed run exists since.

## 8. Control conventions

- Coordinates: normalized sandbox BL=(0,0), BR=(1,0), TR=(1,1), TL=(0,1). Planning/angles use the square frame
  u = x*aspect, v = y (units of sandbox height; mm via `calib.height_mm`). Heading 0 = +x, CCW positive. No metres are
  assumed except via the depth-measured height.
- Rover footprint: measured from depth (about 170-190 mm square incl. wheels) or `--rover-mm`, plus `--margin-mm` 15.
  Planner (`geom.plan_oriented`): 4-connected A* on 120x40 cells over (cell, direction) states, rectangle clearance for
  straight legs, enclosing circle for turns, +-8 deg heading allowance, initial-heading cost (reverse allowed), unknown
  depth = blocked, rover's own cells/swept path excluded.
- Actions (`control.Navigator._plan_action`): turn in place via `UgvLink.turn_by` (closed on IMU yaw magnitude, sign-agnostic,
  stops at ~80 % of the error); arc (forward or reverse) when error <= 40 deg and close; bursts (F/B with trim);
  cruise = streamed wheel commands (`UgvLink.drive`, re-sent every 80 ms, **expires after 0.9 s unless refreshed** by a new
  pose, then the link stops the rover). Cruise only if the segment end is farther than latency lead + 0.45 and the
  footprint gap to obstacles/edges > 0.30 heights. Reverse is chosen per leg when it needs >0.2 rad less turning than
  forward; the leg's direction is kept unless the other needs 0.5 rad less. Trim > 0 yaws CCW in both directions.
  Speeds: cruise 0.40 (reverse 0.30), bursts 0.5/0.4/0.3 depending on distance and clearance.
- Arrival tolerance: **50 mm** centre-to-target (`arrive_tol_mm`), justified as ~1/3 of the rover half-width, ~20x the
  measured stationary position noise (sigma 2.5 mm), ~1.5x the shortest reliable step. Do not widen it to declare success.
  The final approach fails honestly ("could not settle") after 7 non-improving approach steps.
- Safety: STOP button/Space and `/api/stop` always work and disarm; arming requires a tracked rover; start requires arm;
  any comm error, stale frame, lost pose, no route, step/time limit, footprint outside boundary => stop + disarm.
  Single executor (`Navigator.start/retarget` join the old thread first). Calibration, map and learned model persist
  across STOP/restart (`~/forge-nav`).
- Tracking requirements: depth blob must be found (size 80-340 mm, not touching tall objects), pattern heading readable
  (else frames are skipped, then fail), frame fresher than the cruise TTL.

## 9. Measured performance (HISTORICAL, partly while two clients fetched frames)

Frame size 520 KB raw -> **112 KB** (JPEG RGB + 4 mm-quantised zlib depth). Transfer 0.3-0.6 s alone (1.8 s when two
fetchers competed), `/meta` 50-75 ms alone (300 ms under contention); Zero `tailscaled` uses 33-41 % CPU (relay
encryption). Delivered ~0.7-2.5 fps vs 5 fps captured. Stationary position noise 2.5 mm, pattern heading noise 7 deg per
frame. STOP request ~0.22 s round trip; first visible motion ~1.4 s after a command (inflated); position settled ~0.8 s
after STOP. Command 0.35 -> ~0.3-0.4 m/s once moving. **Not profiled yet:** per-frame CPU on the Pi (`Perception.process`:
ArUco detect, plane/blob, depth-map update, pattern heading), `App.state()` (re-plans and builds the sweep every poll),
JPEG encodes for `/api/view.jpg|live.jpg|camera.jpg`, the background loop plus the navigator both processing frames.
These are the obvious suspects for the "stop-start" feel and should be measured first (cProfile on the Pi).

## 10. Known issues / watch list

- Left markers (IDs 1,2) must be hand-placed after any camera move; ID 1 also touches the dark table edge.
- Pattern heading is noisy; a larger marker or a lower camera would remove the heading fallback.
- Frame latency is dominated by the Tailscale relay; no direct LAN path.
- The Pi navigator was restarted several times; `~/forge-nav/depthmap.json` is absent on purpose (stale swept-path obstacles
  were once persisted; fixed by excluding recent rover poses from mapping).
- Zero services are not boot-persistent. systemd enabling and installing an SSH key were refused by the tool's safety
  classifier as unauthorised persistence; do not work around that.

## 11. Authorization and restrictions

- The operator authorised real supervised rover movement inside the validated tabletop area, bounded calibration probes,
  application edits, dependency installs, deployment and restarting these services, without per-step confirmation.
- Do NOT push or merge. Do not flash firmware (current firmware is sufficient). Ask only for physical resets (e.g. rover off
  the table), credentials, flashing, or a real tool-approval block; never bypass a rejection. If the rover is outside the
  usable area ask for a physical reset instead of attempting edge recovery.
- Repository contracts live in `docs/contracts/` (`INDEX.md`, `SW-00..05`, `HW-*`, `INT-*`, `DEMO-1`, ...). **The author did
  not read them**; read `INDEX.md` and the contracts that cover `tools/` and tests before large edits or committing.
  Hackathon context: `context.md` (build window ends Sun 2026-11-08 12:00; judged on a working live demo).

## 12. Read these first (in order)

1. `tools/navigator/control.py` (executor, actions, safety checks) 2. `tools/navigator/server.py` (API, per-poll work, rendering)
3. `tools/navigator/perception.py` + `vision.py` (per-frame cost, blob/pattern/plane/map) 4. `tools/navigator/ugv.py`
(HTTP link, cruise watchdog, IMU turns) 5. `tools/navigator/geom.py` (planner) 6. `tools/navigator/sources.py` (frame fetch)
7. `tests/test_navigator.py` + `tools/navigator/sim.py` 8. `tools/aurora/frame_server.py`, `aurora_cap.cc`, `zero_start.sh`
9. `tools/navigator/nav.html`.
