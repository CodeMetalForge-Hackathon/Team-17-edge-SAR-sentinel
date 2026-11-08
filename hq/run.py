#!/usr/bin/env python3
"""HQ: plan a route on the overhead photo and drive the RoverC-Pro there by itself. Runs on the HQ Pi.

Photos come from the aerial side (another process / person) at --photo; each new photo must be written to a
temp name and renamed over that file. --photo-cmd makes this program ask for each photo itself.

  # 1. just plan (uses the current photo, never moves anything)
  python3 hq/run.py --photo /home/khoj/overhead_latest.jpg --target F1

  # 2. drive, taking photos itself through the Zero (fully autonomous)
  python3 hq/run.py --photo /home/khoj/overhead_latest.jpg --target F1 --go \\
      --photo-cmd "ssh zero@100.96.176.12 'rpicam-still -n -t 1200 --width 1600 --height 1200 -o /tmp/s.jpg' && scp -q zero@100.96.176.12:/tmp/s.jpg ~/ov_new.jpg && mv ~/ov_new.jpg /home/khoj/overhead_latest.jpg"

  # 3. drive, waiting for each new photo someone else takes
  python3 hq/run.py --photo /home/khoj/overhead_latest.jpg --target F1 --go

Ctrl+C = STOP (the rover also stops by itself 0.4 s after its last command).
The live picture (route, blocked squares, rover track) is written to --view after every photo.
"""
import argparse
import json
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from blob_pilot import BlobPilot, Stop
from devices import Rover, WatchedFileCamera


def pair(s, cast=float):
    a, b = s.lower().split("x")
    return cast(a), cast(b)


def main():
    ap = argparse.ArgumentParser(description="HQ overhead route + autonomous rover drive")
    ap.add_argument("--photo", required=True, help="overhead photo file kept up to date by the aerial side")
    ap.add_argument("--photo-cmd", help="shell command that produces a NEW photo at --photo (makes it autonomous)")
    ap.add_argument("--target", required=True, help="target square, e.g. F1 (A1 = bottom-left as seen in the photo)")
    ap.add_argument("--rover-at", help="square the rover starts in (optional; otherwise found by its first move)")
    ap.add_argument("--size", default="1.18x0.62", help="board size in metres between the markers' outer corners, WxH")
    ap.add_argument("--grid", default="6x4", help="squares, COLSxROWS")
    ap.add_argument("--rover-radius", type=float, default=0.07, help="half the rover's width, metres")
    ap.add_argument("--margin", type=float, default=0.03, help="extra clearance from rubble, metres")
    ap.add_argument("--view", default=os.path.expanduser("~/hq_view.png"), help="live picture written here")
    ap.add_argument("--go", action="store_true", help="actually drive (otherwise: plan only)")
    ap.add_argument("--dry-run", action="store_true", help="with --go: log rover commands instead of sending")
    a = ap.parse_args()

    stop = {"flag": False}
    rover = Rover(dry_run=a.dry_run or not a.go)
    def on_sigint(*_):
        stop["flag"] = True
        rover.stop()
        print("\nSTOP sent", flush=True)
    signal.signal(signal.SIGINT, on_sigint)

    cam = WatchedFileCamera(a.photo, capture_cmd=a.photo_cmd if a.go else None)
    pilot = BlobPilot(cam, rover, size=pair(a.size), grid=pair(a.grid, int), rover_radius=a.rover_radius,
                      margin=a.margin, view_path=a.view, log=lambda m: print("[hq]", m, flush=True),
                      should_stop=lambda: stop["flag"])
    try:
        pilot.look()
        if a.rover_at:
            pilot.set_rover_hint(a.rover_at)
        pilot.set_target(a.target)
        print("[hq] plan:", json.dumps(pilot.summary()), flush=True)
        print("[hq] picture:", a.view, flush=True)
        if not a.go:
            if not pilot.rover_xy:
                print("[hq] rover not identified yet: pass --rover-at <square> to see the route, or --go to find it by its first move")
            return 0
        if not a.photo_cmd:
            print("[hq] no --photo-cmd: after every rover move, take a new photo (waiting up to 60 s each time)", flush=True)
        where = pilot.drive()
        print("[hq] DONE: rover at", where, flush=True)
        return 0
    except Stop:
        print("[hq] stopped", flush=True)
        return 1
    except Exception as e:
        rover.stop()
        print("[hq] STOPPED:", e, flush=True)
        return 2
    finally:
        rover.stop()


if __name__ == "__main__":
    sys.exit(main())
