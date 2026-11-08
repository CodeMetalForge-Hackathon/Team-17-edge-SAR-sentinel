# Overhead boundary viewer (static-photo mode)

Manual boundary + perspective flatten + click-to-normalized-coordinate. No live feed, auto-detection, rover or routing.

Run (Windows, from the repo root):

    python tools\overhead_viewer\serve.py --image C:\Users\rohan\Pictures\sandbox_setup.jpg

Open http://127.0.0.1:8765. Stdlib Python only; the browser does the maths (`homography.js`).

- Drag the 4 handles (BL green, BR blue, TR orange, TL magenta, as seen in the image) onto the black squares' corners nearest the sandbox. "Save corners" writes `corners.json` (git-ignored by convention; delete to reset).
- Click inside the polygon, or in the flattened view, for x/y (0–1, y up) and a cell. Cells: column letter left→right, row number bottom→top (A1 = bottom-left).
- Clicks outside the boundary, and invalid boundaries (crossed, mirrored, collinear, tiny, outside image), are rejected rather than guessed.
- Coordinates are dimensionless fractions, not metres. Lens distortion is NOT corrected. The flattened view is stretched to 3:2, not to physical scale.
- Marker IDs are not detected; the handle order defines BL/BR/TR/TL.

## Route preview (static, manual obstacles)

Mode menu: Inspect coordinate / Set start cell / Set target cell / Toggle blocked cell (works in either view). 4-connected A* avoids blocked cells, rejects a blocked start/target, shows "NO ROUTE" when unreachable, and lists ordered cell names plus normalized cell-centre waypoints. The route is cleared when calibration is invalid. "Save scenario" stores boundary, grid size, start, target and blocked cells in `corners.json`; changing the grid size or moving a boundary handle clears start/target/blocked. This is a preview over hand-marked obstacles: not obstacle detection, not a physically safe route, rover size is ignored, and it is not a motor command.

## Rover planning + supervised control (runs on the controller Pi)

Deployed copy: `~/forge-overhead/` on the controller Pi (same layout: `tools/overhead_viewer`, `rover`). The viewer must run on the Pi because only the Pi has the rover link (192.168.4.2 on the rover AP).

View only (no rover commands possible):

    ssh -i C:\Users\rohan\.ssh\forge_lidar -L 8765:127.0.0.1:8765 khoj@100.119.240.31
    # on the Pi:
    cd ~/forge-overhead/tools/overhead_viewer && python3 serve.py --image /home/khoj/overhead_latest.jpg

then open http://127.0.0.1:8765 on Windows. Supervised motion test: add `--enable-motion` (then tick "Enable motion" in the page). `--dry-run` logs instead of sending.

Per-photo workflow: Reload image -> check handles -> Confirm boundary alignment -> Set rover centre -> Set rover forward direction -> set target / toggle blocked cells -> read route -> one short burst -> STOP -> take a new photo -> repeat. The server refuses any move unless the image file is newer than the previous command and the pose was set on that exact image. STOP (button, Space, Esc) always works and disarms. Server binds to 127.0.0.1 only.

## Footprint planning (18x12 default)

Draw obstacle/rover rectangles in the flattened view (modes "Draw obstacle footprint", "Draw rover footprint"); click a rectangle to select, drag a corner to resize, Delete key or button removes it. Rectangles are stored in normalized coordinates (scenario version 2), so changing the grid only recomputes occupied cells. Planning inflates obstacles by the rover footprint's enclosing-circle radius plus the extra margin and keeps the rover centre that far from the sandbox edge (whole-cell test, so conservative; finer grid = narrower gaps allowed). Start/target footprints that overlap an obstacle are rejected; the margin is never lowered automatically. All sizes are normalized, not metres. Replacing the photo clears the rover pose and requires "Confirm boundary alignment" and "Confirm footprints" again. Old 6x4 scenarios are not reused (backups in `backups/`).
