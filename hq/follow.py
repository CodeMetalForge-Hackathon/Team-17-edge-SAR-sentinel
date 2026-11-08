#!/usr/bin/env python3
"""Drive the planned route WITHOUT new photos: plan once on one photo, then send timed bursts.

The rover cannot be corrected along the way (nothing watches it), so accuracy depends on the speed numbers.
Measure them once with --test (see below) and pass them in.

  # see the moves, send nothing
  python3 hq/follow.py --photo hq/sample_sandbox.jpg --rover-at B3 --target F1 --facing up

  # measure speed: drive forward 1 s, measure how far it went in metres, then sideways (L) 1 s
  python3 hq/follow.py --test F --secs 1.0
  python3 hq/follow.py --test L --secs 1.0

  # drive it
  python3 hq/follow.py --photo hq/sample_sandbox.jpg --rover-at B3 --target F1 --facing up \\
      --speed-fwd 0.20 --speed-side 0.16 --go

--facing: which way the rover's FRONT points in the photo (up = toward the top edge of the photo).
The rover never turns: mecanum wheels strafe sideways, so each leg is F, B, L or R.
If it strafes the wrong way, add --flip-side; if forward/back are swapped, add --flip-fwd.
Ctrl+C = STOP.
"""
import argparse
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from blob_pilot import BlobPilot
from devices import FileCamera, Rover

# map direction of travel (in the photo) -> rover command, for each way the rover can face
DIRS = {"up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0)}


def command_for(move, facing, flip_fwd=False, flip_side=False):
    """move/facing: (dx, dy) unit vectors on the map (y up). -> F/B/L/R."""
    fx, fy = DIRS[facing]
    lx, ly = -fy, fx                      # the rover's left = its front turned 90 deg counter-clockwise
    fwd = move[0] * fx + move[1] * fy
    left = move[0] * lx + move[1] * ly
    if abs(fwd) >= abs(left):
        c = "F" if fwd > 0 else "B"
        return {"F": "B", "B": "F"}[c] if flip_fwd else c
    c = "L" if left > 0 else "R"
    return {"L": "R", "R": "L"}[c] if flip_side else c


def legs(grid, path):
    """Path of squares -> list of (dx, dy, distance_m), joining straight runs."""
    out = []
    for a, b in zip(path, path[1:]):
        d = (b[0] - a[0], b[1] - a[1])
        dist = abs(d[0]) * grid.cw + abs(d[1]) * grid.ch
        if out and (out[-1][0], out[-1][1]) == d:
            out[-1] = (d[0], d[1], out[-1][2] + dist)
        else:
            out.append((d[0], d[1], dist))
    return out


def main():
    ap = argparse.ArgumentParser(description="Follow the planned route with timed bursts (no photos while driving)")
    ap.add_argument("--photo")
    ap.add_argument("--rover-at")
    ap.add_argument("--target")
    ap.add_argument("--facing", choices=list(DIRS), default="up")
    ap.add_argument("--size", default="1.18x0.62", help="board WxH in metres between the markers' outer corners")
    ap.add_argument("--grid", default="6x4")
    ap.add_argument("--speed-fwd", type=float, default=0.20, help="m/s for F/B (measure with --test F)")
    ap.add_argument("--speed-side", type=float, default=0.16, help="m/s for L/R (measure with --test L)")
    ap.add_argument("--flip-fwd", action="store_true")
    ap.add_argument("--flip-side", action="store_true")
    ap.add_argument("--pause", type=float, default=0.4, help="stop between legs, seconds")
    ap.add_argument("--go", action="store_true", help="actually send commands")
    ap.add_argument("--test", choices=list("FBLRQE"), help="send ONE burst to measure speed / check direction")
    ap.add_argument("--secs", type=float, default=1.0)
    a = ap.parse_args()

    rover = Rover(dry_run=False if (a.go or a.test) else True)
    signal.signal(signal.SIGINT, lambda *_: (rover.stop(), print("\nSTOP sent"), os._exit(1)))

    if a.test:
        print("sending %s for %.2f s ..." % (a.test, a.secs), flush=True)
        rover.move(a.test, min(a.secs, rover.max_burst))
        rover.stop()
        print("done. Measure how far it moved (metres) / %.2f s = speed" % a.secs)
        return 0

    if not (a.photo and a.rover_at and a.target):
        ap.error("--photo, --rover-at and --target are needed (or --test)")
    w, h = (float(v) for v in a.size.lower().split("x"))
    cols, rows = (int(v) for v in a.grid.lower().split("x"))
    pilot = BlobPilot(FileCamera(a.photo), rover, size=(w, h), grid=(cols, rows), log=lambda m: None,
                      view_path=os.path.expanduser("~/hq_view.png"))
    pilot.look()
    pilot.set_rover_hint(a.rover_at)
    pilot.set_target(a.target)
    s = pilot.summary()
    print("route:", " -> ".join(s["path"] or ["NONE"]), "| blocked:", " ".join(s["blocked"]))
    if not pilot.path:
        print("no safe route")
        return 2

    plan = []
    for dx, dy, dist in legs(pilot.grid, pilot.path):
        cmd = command_for((dx, dy), a.facing, a.flip_fwd, a.flip_side)
        speed = a.speed_fwd if cmd in "FB" else a.speed_side
        plan.append((cmd, dist / speed, dist, (dx, dy)))
    names = {(1, 0): "right", (-1, 0): "left", (0, 1): "up", (0, -1): "down"}
    for cmd, secs, dist, d in plan:
        print("  %-5s %.2f m  ->  %s for %.2f s" % (names[d], dist, cmd, secs))
    print("total %.1f s of driving" % sum(p[1] for p in plan))
    if not a.go:
        print("(plan only: add --go to drive)")
        return 0

    for cmd, secs, _, d in plan:
        print("driving %s %.2f s (%s)" % (cmd, secs, names[d]), flush=True)
        left = secs
        while left > 0.02:  # bursts of <= 1 s back to back (firmware watchdog needs a packet every 0.4 s)
            chunk = min(left, rover.max_burst)
            rover.move(cmd, chunk)
            left -= chunk
        rover.stop()
        time.sleep(a.pause)
    rover.stop()
    print("route finished: rover should be at", a.target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
