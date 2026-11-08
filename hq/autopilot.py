"""HQ brain for the overhead loop: photo -> map -> rubble -> route -> drive the rover there by itself.

Driving: the rover's motion directions are learned, not assumed. On the first GO it makes three short test
bursts (F, L, Q) and measures from the photos how far and which way each one moved it relative to the marker on
its back. After that every step is: photo -> where is the rover -> next square on the route -> pick the burst
direction that best points there -> burst -> photo -> learn from what actually happened -> repeat.
"""
import collections
import math
import threading
import time

import cv2
import numpy as np

from vision import PX_PER_M, Sandbox, corners_from_markers, detect_markers, rover_pose, rubble
from planner import Grid

CAL_BURST_S = 0.4         # length of each calibration burst
SETTLE_S = 0.4            # wait after a burst before the photo
MAX_STEPS = 80
MAX_RUN_S = 360
MIN_SPEED = 0.015         # m/s: slower than this on a calibration burst = "the rover did not move"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_rover(dx, dy, th):
    """world displacement -> (forward, left) in the rover marker's frame."""
    return dx * math.cos(th) + dy * math.sin(th), -dx * math.sin(th) + dy * math.cos(th)


def to_world(a, b, th):
    return a * math.cos(th) - b * math.sin(th), a * math.sin(th) + b * math.cos(th)


class HQ:
    def __init__(self, camera, rover, size=(1.5, 1.0), grid=(6, 4), rover_id=10, rover_radius=0.09,
                 margin=0.03, motion_allowed=False, on_arrive=None):
        self.camera, self.rover = camera, rover
        self.size, self.rover_id = size, rover_id
        self.rover_radius, self.margin = rover_radius, margin
        self.grid = Grid(size[0], size[1], *grid)
        self.motion_allowed = motion_allowed
        self.on_arrive = on_arrive
        self.sandbox = None
        self.photo = self.map = self.before_map = None
        self.before_pose = self.pose = None
        self.markers_seen = []
        self.obstacles, self.blocked = [], set()
        self.target, self.path = None, None
        self.cal = {}                 # cmd -> [forward, left] m/s in rover frame; "Q" -> rad/s
        self.trail = collections.deque(maxlen=200)
        self.log = collections.deque(maxlen=60)
        self.running, self.status = False, "idle"
        self.stop_evt = threading.Event()
        self.lock = threading.RLock()
        self.version = 0
        self.overlay = None

    # ------------------------------------------------------------------ helpers
    def say(self, msg):
        self.log.appendleft(time.strftime("%H:%M:%S ") + msg)
        print("[hq]", msg, flush=True)

    def cell_name(self, cell):
        return self.grid.name(*cell) if cell else "-"

    # ------------------------------------------------------------------ seeing
    def snapshot(self):
        """Take a photo, find the corners and the rover, flatten. Raises with a readable message on failure."""
        photo = self.camera.capture()
        markers = detect_markers(photo)
        corners = corners_from_markers(markers, self.rover_id)
        with self.lock:
            self.photo, self.markers_seen = photo, sorted(markers)
            if corners is not None:
                self.sandbox = Sandbox(self.size[0], self.size[1], corners)
            elif self.sandbox is None:
                n = len([i for i in markers if i != self.rover_id])
                raise RuntimeError("found %d corner marker(s), need all 4 in view (markers seen: %s)"
                                   % (n, self.markers_seen or "none"))
            self.map = self.sandbox.flatten(photo)
            self.pose = rover_pose(markers, self.sandbox, self.rover_id)
            if self.pose:
                self.trail.append(self.pose[:2])
            self.render()
        return self.pose

    def save_before(self):
        self.snapshot()
        with self.lock:
            self.before_map, self.before_pose = self.map.copy(), self.pose
            self.obstacles, self.blocked, self.path = [], set(), None
            self.render()
        self.say("BEFORE saved (empty sandbox%s)" % (", rover at " + self.cell_name(self.grid.cell_of(*self.pose[:2])) if self.pose else ""))

    def scan(self, take_photo=True):
        """Find rubble = what changed since BEFORE (the rover is ignored where it is now and where it was)."""
        if take_photo:
            self.snapshot()
        with self.lock:
            if self.before_map is None:
                raise RuntimeError("no BEFORE photo yet: clear the sandbox and press Save empty as BEFORE")
            if self.before_map.shape != self.map.shape:
                raise RuntimeError("sandbox size changed since BEFORE: save BEFORE again")
            ignore = [(p[0], p[1], self.rover_radius * 1.7) for p in (self.pose, self.before_pose) if p]
            self.obstacles, _ = rubble(self.before_map, self.map, ignore)
            self.blocked = self.grid.blocked(self.obstacles, self.rover_radius + self.margin)
            self.replan()
        cells = sorted(self.cell_name(c) for c in self.blocked)
        self.say("scan: %d rubble object(s), blocked squares: %s" % (len(self.obstacles), " ".join(cells) or "none"))

    def set_target(self, cell):
        with self.lock:
            c = self.grid.parse(cell) if isinstance(cell, str) else tuple(cell)
            if not self.grid.inside(*c):
                raise RuntimeError("target %s is outside the grid" % cell)
            self.target = c
            self.replan()
        self.say("target set: " + self.cell_name(c) + ("" if self.path else "  (no route yet)"))

    def replan(self):
        with self.lock:
            self.path = None
            if self.pose and self.target:
                start = self.grid.cell_of(*self.pose[:2])
                if start:
                    self.path = self.grid.route(start, self.target, self.blocked)
            self.render()
        return self.path

    # ------------------------------------------------------------------ driving
    def go(self):
        if self.running:
            raise RuntimeError("already driving")
        if not self.motion_allowed:
            raise RuntimeError("motion is disabled (start the server with --enable-motion, or use --sim)")
        if self.target is None:
            raise RuntimeError("set a target square first (click the map)")
        if self.before_map is None:
            raise RuntimeError("no BEFORE photo yet")
        self.stop_evt.clear()
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self, why="STOP pressed"):
        self.stop_evt.set()
        ok = self.rover.stop()
        self.say(why + ("" if ok is not False else " (STOP send FAILED: use the rover button)"))

    def _burst(self, cmd, seconds):
        if self.stop_evt.is_set():
            raise InterruptedError()
        self.rover.move(cmd, seconds)
        time.sleep(SETTLE_S)

    def _need_pose(self):
        for _ in range(3):
            if self.stop_evt.is_set():
                raise InterruptedError()
            if self.snapshot():
                return self.pose
        raise RuntimeError("rover marker not visible in 3 photos")

    def _calibrate(self):
        self.status = "calibrating"
        self.say("calibrating: 3 short test bursts (F, L, Q)")
        for cmd in ("F", "L", "Q"):
            x0, y0, t0 = self._need_pose()
            self._burst(cmd, CAL_BURST_S)
            x1, y1, t1 = self._need_pose()
            if cmd == "Q":
                rate = wrap(t1 - t0) / CAL_BURST_S
                if abs(rate) < math.radians(5):
                    raise RuntimeError("rover did not turn on Q: check its power / Wi-Fi link")
                self.cal["Q"] = rate
                self.say("Q turns %+.0f deg/s" % math.degrees(rate))
                continue
            a, b = to_rover(x1 - x0, y1 - y0, t0)
            v = (a / CAL_BURST_S, b / CAL_BURST_S)
            if math.hypot(*v) < MIN_SPEED:
                raise RuntimeError("rover did not move on %s: check its power / Wi-Fi link" % cmd)
            self.cal[cmd] = list(v)
            self.say("%s moves %.2f m/s at %+.0f deg from the marker's up" % (cmd, math.hypot(*v), math.degrees(math.atan2(v[1], v[0]))))
        self.cal["B"] = [-self.cal["F"][0], -self.cal["F"][1]]
        self.cal["R"] = [-self.cal["L"][0], -self.cal["L"][1]]

    def _segment_ok(self, p0, p1, allowed):
        for k in range(1, 11):
            x, y = p0[0] + (p1[0] - p0[0]) * k / 10, p0[1] + (p1[1] - p0[1]) * k / 10
            c = self.grid.cell_of(x, y)
            if c is None or (c in self.blocked and c not in allowed):
                return False
        return True

    def _plan_move(self, pose, aim, allowed):
        """Best burst toward aim: -> (cmd, seconds, predicted end) or None."""
        x, y, th = pose
        a, b = to_rover(aim[0] - x, aim[1] - y, th)
        dist = math.hypot(a, b)
        best = None
        for cmd in ("F", "B", "L", "R"):
            v = self.cal[cmd]
            sp = math.hypot(*v)
            along = (a * v[0] + b * v[1]) / sp
            if along <= 0:
                continue
            score = along / dist
            if not best or score > best[0]:
                best = (score, cmd, along, sp)
        if not best:
            return None
        _, cmd, along, sp = best
        secs = min(max(0.85 * along / sp, 0.1), self.rover.max_burst)
        for _ in range(3):
            vx, vy = to_world(*[c * secs for c in self.cal[cmd]], th)
            end = (x + vx, y + vy)
            if self._segment_ok((x, y), end, allowed):
                return cmd, secs, end, best[0]
            secs /= 2
        return None

    def _line_up(self, pose, aim):
        """Turn in place (Q/E) so the closest burst direction points at aim. -> True if it turned."""
        if "Q" not in self.cal:
            return False
        x, y, th = pose
        a, b = to_rover(aim[0] - x, aim[1] - y, th)
        if math.hypot(a, b) < 0.03:
            return False
        want = math.atan2(b, a)
        err = min((wrap(want - math.atan2(v[1], v[0])) for k, v in self.cal.items() if k in "FBLR"), key=abs)
        if abs(err) < math.radians(15):
            return False
        secs = min(abs(err / self.cal["Q"]), self.rover.max_burst)
        self._burst("Q" if err * self.cal["Q"] > 0 else "E", secs)
        self.say("turned %+.0f deg to line up" % math.degrees(err))
        return True

    def _learn(self, cmd, secs, before, after):
        a, b = to_rover(after[0] - before[0], after[1] - before[1], before[2])
        v = self.cal[cmd]
        meas = (a / secs, b / secs)
        self.cal[cmd] = [0.7 * v[0] + 0.3 * meas[0], 0.7 * v[1] + 0.3 * meas[1]]
        opp = {"F": "B", "B": "F", "L": "R", "R": "L"}[cmd]
        self.cal[opp] = [-self.cal[cmd][0], -self.cal[cmd][1]]
        return math.hypot(after[0] - before[0], after[1] - before[1])

    def _run(self):
        t_start, still = time.time(), 0
        try:
            self.say("GO: scanning for rubble")
            self.status = "scanning"
            self._need_pose()
            self.scan(take_photo=False)
            if not self.path:
                raise RuntimeError("no safe route from %s to %s" % (self.cell_name(self.grid.cell_of(*self.pose[:2])), self.cell_name(self.target)))
            if not all(k in self.cal for k in ("F", "L", "Q")):
                self._calibrate()
            self.status = "driving"
            tol = 0.35 * min(self.grid.cw, self.grid.ch)
            for step in range(MAX_STEPS):
                if time.time() - t_start > MAX_RUN_S:
                    raise RuntimeError("took too long, stopped")
                pose = self.pose if step == 0 else self._need_pose()
                here = self.grid.cell_of(pose[0], pose[1])
                if here is None:
                    raise RuntimeError("rover left the sandbox")
                path = self.replan()
                if not path:
                    raise RuntimeError("no safe route from %s" % self.cell_name(here))
                goal_xy = self.grid.center(*self.target)
                if here == self.target and math.hypot(pose[0] - goal_xy[0], pose[1] - goal_xy[1]) < tol:
                    self.say("ARRIVED at %s in %d moves" % (self.cell_name(here), step))
                    self.status = "arrived " + self.cell_name(here)
                    if self.on_arrive:
                        self.on_arrive(self.cell_name(here))
                    return
                nxt = path[1] if len(path) > 1 else path[0]
                aim = self.grid.center(*nxt)
                mv = self._plan_move(pose, aim, {here, nxt})
                if (mv is None or mv[3] < 0.8) and self._line_up(pose, aim):
                    continue  # turned so a burst points at the next square; look again
                if mv is None or mv[3] < 0.5:  # still cannot head there cleanly: centre in this square first
                    mv = self._plan_move(pose, self.grid.center(*here), {here})
                    if mv is None:
                        raise RuntimeError("no safe burst from %s" % self.cell_name(here))
                cmd, secs, _, score = mv
                self.say("%s -> %s: %s %.2fs" % (self.cell_name(here), self.cell_name(nxt), cmd, secs))
                self._burst(cmd, secs)
                after = self._need_pose()
                moved = self._learn(cmd, secs, pose, after)
                still = still + 1 if moved < 0.01 else 0
                if still >= 3:
                    raise RuntimeError("rover is not moving (3 bursts with no motion)")
            raise RuntimeError("step limit reached")
        except InterruptedError:
            self.status = "stopped"
        except Exception as e:
            self.rover.stop()
            self.status = "stopped: " + str(e)
            self.say("STOPPED: " + str(e))
        finally:
            self.rover.stop()
            self.running = False
            self.render()

    # ------------------------------------------------------------------ drawing + state
    def render(self):
        with self.lock:
            if self.map is None:
                return
            img = self.map.copy()
            g, sb = self.grid, self.sandbox
            tint = img.copy()
            for c, r in self.blocked:
                p0 = sb.m_to_map_px(c * g.cw, (r + 1) * g.ch)
                p1 = sb.m_to_map_px((c + 1) * g.cw, r * g.ch)
                cv2.rectangle(tint, p0, p1, (40, 40, 220), -1)
            img = cv2.addWeighted(tint, 0.35, img, 0.65, 0)
            for c in range(1, g.cols):
                x = sb.m_to_map_px(c * g.cw, 0)[0]
                cv2.line(img, (x, 0), (x, img.shape[0]), (235, 235, 235), 1)
            for r in range(1, g.rows):
                y = sb.m_to_map_px(0, r * g.ch)[1]
                cv2.line(img, (0, y), (img.shape[1], y), (235, 235, 235), 1)
            for c in range(g.cols):
                for r in range(g.rows):
                    x, y = sb.m_to_map_px(c * g.cw, (r + 1) * g.ch)
                    cv2.putText(img, g.name(c, r), (x + 4, y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            for o in self.obstacles:
                cv2.rectangle(img, sb.m_to_map_px(o["x0"], o["y1"]), sb.m_to_map_px(o["x1"], o["y0"]), (0, 0, 255), 2)
            if self.path and self.pose:
                pts = [sb.m_to_map_px(*self.pose[:2])] + [sb.m_to_map_px(*g.center(*c)) for c in self.path[1:]]
                cv2.polylines(img, [np.int32(pts)], False, (255, 220, 0), 3, cv2.LINE_AA)
            if len(self.trail) > 1:
                cv2.polylines(img, [np.int32([sb.m_to_map_px(*p) for p in self.trail])], False, (200, 120, 255), 1, cv2.LINE_AA)
            if self.target:
                cx, cy = sb.m_to_map_px(*g.center(*self.target))
                cv2.drawMarker(img, (cx, cy), (0, 230, 255), cv2.MARKER_STAR, 28, 2)
            if self.pose:
                x, y, th = self.pose
                p = sb.m_to_map_px(x, y)
                q = sb.m_to_map_px(x + 0.12 * math.cos(th), y + 0.12 * math.sin(th))
                cv2.circle(img, p, int(self.rover_radius * PX_PER_M), (80, 255, 80), 2)
                cv2.arrowedLine(img, p, q, (80, 255, 80), 2, tipLength=0.3)
            self.overlay = img
            self.version += 1

    def state(self):
        with self.lock:
            here = self.grid.cell_of(*self.pose[:2]) if self.pose else None
            return {
                "version": self.version, "status": self.status, "running": self.running,
                "motion_allowed": self.motion_allowed, "calibrated": all(k in self.cal for k in ("F", "L", "Q")),
                "have_before": self.before_map is not None, "have_map": self.map is not None,
                "markers_seen": self.markers_seen, "rover_cell": self.cell_name(here),
                "rover_heading_deg": round(math.degrees(self.pose[2])) if self.pose else None,
                "target": self.cell_name(self.target), "path": [self.cell_name(c) for c in self.path] if self.path else [],
                "blocked": sorted(self.cell_name(c) for c in self.blocked), "obstacles": len(self.obstacles),
                "grid": [self.grid.cols, self.grid.rows], "size_m": list(self.size),
                "map_px": [self.overlay.shape[1], self.overlay.shape[0]] if self.overlay is not None else None,
                "log": list(self.log)[:25],
            }
