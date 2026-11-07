# HW-B3: Camera bearing calibration

Status: PROPOSED. **Conditional** on HW-B2, SW-07 and HW-A1.

| Field | Value |
|---|---|
| Objective | The real pipeline publishes bearings that agree with LiDAR and a tape measure |
| Owner | B, with A |
| Branch | Evidence through a human commit on `codex/HW-B3` |
| Depends on | HW-B2, SW-07 merged, HW-A1 |
| Budget | 2 hours on the bench |

## Scope
1. Run `pi/hailo_person_app.py` in the Hailo venv with `PYTHONPATH=pi`. Watch `forge/vision/person`.
2. Stand at known positions: at least 5 bearings across the view, 2 ranges each. Record detected bearing, LiDAR bearing and the tape value.
3. Set `CAMERA_HFOV_DEG` and `CAMERA_ANGLE_OFFSET_DEG` from those measurements. The pipeline may crop, so the effective view can differ from the datasheet.
4. Confirm a person actually registers in LiDAR range at each spot.

## Explicit exclusions
No depth estimation. No tracking.

## Allowed edits
`pi/config.py` measured camera values, by a human. `docs/evidence/HW-B3/*`. API fixes need a contract change to SW-07 or a new contract.

## Pre-approved commands and runs
The commands above, on the Pi.

## Acceptance (evidence file `docs/evidence/HW-B3/report.md`)
Table of positions with errors in degrees and metres, plus worst and median. **Numbers are measured, not assumed.**

## Risks
Camera and LiDAR are not truly co-located. Nearest-return logic grabs a partition in front of the person.

## Stop conditions
Median error over 1 m with no clear fix: tell D, and shrink the demo region.

## Handoff and close
Report to D. Human commits evidence by explicit path.
