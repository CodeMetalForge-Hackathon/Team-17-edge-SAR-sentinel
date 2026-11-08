# HW-A1: LiDAR bring-up

Status: PROPOSED. **Conditional.** Human bench work; can start now.

| Field | Value |
|---|---|
| Objective | Real scans in the map frame, correct direction, with a saved baseline |
| Owner | A |
| Branch | None needed for bench work. Evidence through a human commit on `codex/HW-A1`. **Any code change gets its own contract** |
| Depends on | Nothing |
| Budget | Bench time until Sat 18:00. SDK clone and build on the Pi only |

## Scope
1. Build the [Slamtec SDK](https://github.com/Slamtec/rplidar_sdk). Run `ultra_simple` with C1 at 460800. S2 at 1000000 if C1 fails. **C1 is preferred**: lower data rate and the 6 m clamp makes S2's range unnecessary.
2. Confirm the output matches `LINE_RE`. Upstream's `printf` currently does.
3. Run `lidar_node.py --baseline`, then live.
4. **Direction test:** place an object to the physical left and front. Fix `LIDAR_ANGLE_SIGN` and `LIDAR_ANGLE_OFFSET_DEG` from the measurement. These values are not pinned.
5. Box test in the scan plane.

## Explicit exclusions
No SLAM. No `lidar_node` rewrite. Do not replace the driver without a new contract.

## Allowed edits
`pi/config.py` measured LiDAR values, by a human. `docs/evidence/HW-A1/*`.

## Pre-approved commands and runs
The README's LiDAR commands.

## Acceptance (evidence file `docs/evidence/HW-A1/report.md`)
Model used. Serial device. Scans per second. CPU use during the run. Points in a scan. Measured sign and offset with
photo or table. Box position against tape, with error. Scan-plane height and whether it hits a standing person's torso or legs.

## Risks
Scan plane misses low boxes or a person. Pipe buffering adds lag. Baseline captured with people in view.

## Stop conditions
Hardware dead for 45 minutes: tell D, and swap to the other LiDAR.

## Handoff and close
Report to D. Human commits evidence by explicit path.
