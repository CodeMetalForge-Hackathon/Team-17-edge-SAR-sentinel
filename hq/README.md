# HQ: overhead route + autonomous rover

Runs on the HQ Pi. Takes the overhead photo of the white board (4 ArUco corner markers), finds every object on it, works out which one is the rover (the one that moves), plans a route to the target square around the rest, and drives the RoverC-Pro there in short bursts, checking a new photo after each burst.

No marker is needed on the rover. A corner marker whose white margin is covered (cardboard touching it) is still found.

## Files
- `run.py`: the command to use (plan only, or `--go` to drive).
- `blob_pilot.py`: marker-free rover tracking, self-calibration, and the drive loop.
- `vision.py`: markers, flattening, objects on the white board, before/after rubble.
- `planner.py`: grid squares (A1 = bottom-left as seen in the photo) and A* with clearance for the rover's size.
- `devices.py`: photo sources (watched file / Zero over SSH) and the rover link (`rover/rover_control.py`).
- `autopilot.py`: variant for a rover WITH an ArUco marker (ID 10) and before/after rubble; used by `tests/test_hq.py`.
- `sim.py`: simulated camera + rover for tests.

## Use
Needs `python3-opencv` (`sudo apt install -y python3-opencv`), the HQ Pi on the rover's Wi-Fi (as in `rover/`), and all 4 corner markers in the photo.

```
# plan only, never moves anything
python3 hq/run.py --photo /home/khoj/overhead_latest.jpg --target F1 --rover-at B3

# drive; HQ takes each photo itself through the Zero (fully autonomous)
python3 hq/run.py --photo /home/khoj/overhead_latest.jpg --target F1 --go \
  --photo-cmd "ssh zero@100.96.176.12 'rpicam-still -n -t 1200 --autofocus-on-capture --width 2304 --height 1296 -o /tmp/s.jpg' && scp -q zero@100.96.176.12:/tmp/s.jpg ~/ov_new.jpg && mv ~/ov_new.jpg /home/khoj/overhead_latest.jpg"
```
Without `--photo-cmd` it waits (up to 60 s) for someone else to put a new photo at `--photo` after every burst.

Ctrl+C sends STOP. The rover's firmware also stops 0.4 s after its last command.

The live picture (blocked squares, route, rover track) is rewritten to `~/hq_view.png` after every photo.

## What happens on --go
1. Two test bursts, F then L (0.35 s each). The object that moved is the rover. How far and which way each burst moved it gives the rover's speed and directions on the map (B and R are the opposites).
2. Loop: route from the rover's square (A*, 4-connected, blocked squares = rubble plus rover radius plus margin), then pick the burst that best points at the next square, shortened if its predicted track would enter a blocked square, then burst, then new photo. The estimates are updated from what each burst really did.
3. Stops on arrival, Ctrl+C, rover lost, no route, no safe burst, 3 bursts without motion, 60 steps, or 5 minutes.

## Notes
- `--size` is the board between the markers' outer corners in metres. The default is an estimate; speeds are learned in the same units, so a wrong size does not break driving, it only mislabels distances.
- Leave a gap of at least one square plus the rover's width between pieces of rubble, or there is no safe route.
- Run `python3 tests/test_hq.py` to check everything on the simulator.
