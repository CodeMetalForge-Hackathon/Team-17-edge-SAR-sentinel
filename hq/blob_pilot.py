"""Drive the rover to a target square using only overhead photos of a plain white board. No marker on the rover.

How it knows where the rover is: every non-white thing on the board is an object. The rover is the object that
moves when we send it a burst (found on the very first test burst), and after that it is tracked from photo to
photo as the object nearest to where it was. Everything else is rubble.

How it knows how to move: the first two test bursts (F, then L) are measured on the photos, which gives the
direction and speed of each burst on the map. B and R are their opposites. Every later burst is measured too,
and the estimates are updated, so slow drift in the rover's heading is absorbed.

Each step: photo -> rover position -> route (A*) from its square to the target -> burst toward the next square
-> new photo. It stops on: arrival, STOP/Ctrl+C, rover not found, no route, no safe burst, not moving, time limit.
"""
import math
import time

import cv2
import numpy as np

from planner import Grid
from vision import PX_PER_M, Sandbox, find_markers, objects_on_white, outer_corners

CAL_BURST_S = 0.35
MAX_STEPS = 60
MAX_RUN_S = 300
TRACK_RADIUS_M = 0.15     # the rover moves less than this between two photos
MIN_MOVE_M = 0.01


class Stop(Exception):
    pass


class BlobPilot:
    def __init__(self, camera, rover, size=(1.18, 0.62), grid=(6, 4), rover_radius=0.07, margin=0.03,
                 view_path=None, log=print, should_stop=lambda: False):
        self.camera, self.rover = camera, rover
        self.size = size
        self.grid = Grid(size[0], size[1], *grid)
        self.clear = rover_radius + margin
        self.rover_radius = rover_radius
        self.view_path, self.log, self.should_stop = view_path, log, should_stop
        self.sandbox = None
        self.top = None
        self.objects = []
        self.rover_xy = None          # last known rover centre (m)
        self.vel = {}                 # cmd -> (vx, vy) m/s on the map
        self.target = None
        self.path, self.blocked, self.obstacles = None, set(), []
        self.trail = []

    # ------------------------------------------------------------------ seeing
    def look(self, expect=None):
        """Photo -> sandbox -> objects; updates rover position if it is already known (searched for near
        `expect`, where the last burst should have taken it, else near where it was). -> objects."""
        if self.should_stop():
            raise Stop()
        photo = self.camera.capture()
        markers = find_markers(photo)
        corners = outer_corners(markers)
        if corners is not None:
            self.sandbox = Sandbox(self.size[0], self.size[1], corners)
        elif self.sandbox is None:
            raise RuntimeError("need all 4 corner markers in the photo (found %s)" % (sorted(markers) or "none"))
        self.top = self.sandbox.flatten(photo)
        self.objects = objects_on_white(self.top, self.sandbox, markers)
        if self.rover_xy is not None:
            r = self._nearest(expect or self.rover_xy)
            if r is None and expect:
                r = self._nearest(self.rover_xy)
            if r is None:
                raise RuntimeError("lost the rover: no object within %d cm of where it was" % (TRACK_RADIUS_M * 100))
            self.rover_xy = (r["cx"], r["cy"])
            self.trail.append(self.rover_xy)
        self._update_plan()
        return self.objects

    def _nearest(self, xy, radius=TRACK_RADIUS_M):
        best = min(self.objects, key=lambda o: math.hypot(o["cx"] - xy[0], o["cy"] - xy[1]), default=None)
        if best and math.hypot(best["cx"] - xy[0], best["cy"] - xy[1]) <= radius:
            return best
        return None

    def set_rover_hint(self, cell):
        """Optional: say which square the rover starts in (otherwise it is found by the first test burst)."""
        cx, cy = self.grid.center(*self.grid.parse(cell))
        self.look() if self.top is None else None
        r = self._nearest((cx, cy), radius=0.6 * math.hypot(self.grid.cw, self.grid.ch))
        if r is None:
            raise RuntimeError("no object near %s to call the rover" % cell)
        self.rover_xy = (r["cx"], r["cy"])
        self._update_plan()

    def set_target(self, cell):
        c = self.grid.parse(cell)
        if not self.grid.inside(*c):
            raise RuntimeError("target %s is outside the %dx%d grid" % (cell, self.grid.cols, self.grid.rows))
        self.target = c
        self._update_plan()

    def _update_plan(self):
        rov = self._nearest(self.rover_xy, 0.03) if self.rover_xy else None
        self.obstacles = [o for o in self.objects if o is not rov]
        self.blocked = self.grid.blocked(self.obstacles, self.clear)
        self.path = None
        if self.rover_xy and self.target:
            start = self.grid.cell_of(*self.rover_xy)
            if start:
                self.path = self.grid.route(start, self.target, self.blocked)
        self.render()

    # ------------------------------------------------------------------ moving
    def _burst(self, cmd, secs):
        if self.should_stop():
            raise Stop()
        self.rover.move(cmd, secs)
        time.sleep(0.3)

    def calibrate(self):
        """Two test bursts. The first also finds the rover if we don't know it yet (the object that moved)."""
        self.log("calibrating: test bursts F then L")
        if self.top is None:
            self.look()
        for cmd in ("F", "L"):
            before = [dict(o) for o in self.objects]
            p0 = self.rover_xy
            self._burst(cmd, CAL_BURST_S)
            if p0 is None:  # find the rover = the object that moved
                self.rover_xy = None
                self.look()
                moved = []
                for o in self.objects:
                    d = min((math.hypot(o["cx"] - b["cx"], o["cy"] - b["cy"]) for b in before), default=9)
                    if d > MIN_MOVE_M:
                        moved.append(o)
                gone = [b for b in before if min((math.hypot(o["cx"] - b["cx"], o["cy"] - b["cy"]) for o in self.objects), default=9) > MIN_MOVE_M]
                if len(moved) != 1 or len(gone) != 1:
                    raise RuntimeError("could not tell which object is the rover (%d moved): is it powered and on the link?" % len(moved))
                p0 = (gone[0]["cx"], gone[0]["cy"])
                self.rover_xy = (moved[0]["cx"], moved[0]["cy"])
                self.log("rover found at %s" % self.grid.name(*self.grid.cell_of(*self.rover_xy)))
            else:
                self.look()
            dx, dy = self.rover_xy[0] - p0[0], self.rover_xy[1] - p0[1]
            if math.hypot(dx, dy) < MIN_MOVE_M:
                raise RuntimeError("rover did not move on %s: check power / Wi-Fi link to the rover" % cmd)
            self.vel[cmd] = (dx / CAL_BURST_S, dy / CAL_BURST_S)
            self.log("%s: %.2f m/s heading %+.0f deg on the map" % (cmd, math.hypot(dx, dy) / CAL_BURST_S, math.degrees(math.atan2(dy, dx))))
        self.vel["B"] = tuple(-v for v in self.vel["F"])
        self.vel["R"] = tuple(-v for v in self.vel["L"])

    def _safe(self, p0, p1, allowed):
        for k in range(1, 11):
            c = self.grid.cell_of(p0[0] + (p1[0] - p0[0]) * k / 10, p0[1] + (p1[1] - p0[1]) * k / 10)
            if c is None or (c in self.blocked and c not in allowed):
                return False
        return True

    def _choose(self, aim, allowed):
        """Burst that best heads to aim, shortened until its predicted track is safe. -> (cmd, secs, score)."""
        x, y = self.rover_xy
        dx, dy = aim[0] - x, aim[1] - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return None
        cands = []
        for cmd, (vx, vy) in self.vel.items():
            sp = math.hypot(vx, vy)
            along = (dx * vx + dy * vy) / sp
            if along > 0:
                cands.append((along / dist, cmd, along / sp))
        for score, cmd, secs in sorted(cands, reverse=True)[:2]:
            sp = math.hypot(*self.vel[cmd])
            secs = min(max(0.85 * secs, 0.1), self.rover.max_burst, 0.6 * min(self.grid.cw, self.grid.ch) / sp)
            for _ in range(3):
                vx, vy = self.vel[cmd]
                if self._safe((x, y), (x + vx * secs, y + vy * secs), allowed):
                    return cmd, secs, score
                secs /= 2
        return None

    def _learn(self, cmd, secs, p0):
        dx, dy = self.rover_xy[0] - p0[0], self.rover_xy[1] - p0[1]
        vx, vy = self.vel[cmd]
        nv = (0.7 * vx + 0.3 * dx / secs, 0.7 * vy + 0.3 * dy / secs)
        opp = {"F": "B", "B": "F", "L": "R", "R": "L"}[cmd]
        self.vel[cmd], self.vel[opp] = nv, (-nv[0], -nv[1])
        return math.hypot(dx, dy)

    def drive(self):
        """Go to self.target. Returns the square it arrived in; raises with the reason otherwise."""
        if self.target is None:
            raise RuntimeError("no target square")
        t0, still = time.time(), 0
        try:
            if len(self.vel) < 4:
                self.calibrate()
            elif self.top is None:
                self.look()
            tol = 0.3 * min(self.grid.cw, self.grid.ch)
            for step in range(MAX_STEPS):
                if time.time() - t0 > MAX_RUN_S:
                    raise RuntimeError("time limit reached")
                here = self.grid.cell_of(*self.rover_xy)
                if here is None:
                    raise RuntimeError("rover is outside the sandbox")
                if not self.path:
                    raise RuntimeError("no safe route from %s to %s" % (self.grid.name(*here), self.grid.name(*self.target)))
                gx, gy = self.grid.center(*self.target)
                if here == self.target and math.hypot(self.rover_xy[0] - gx, self.rover_xy[1] - gy) < tol:
                    self.log("ARRIVED at %s after %d bursts" % (self.grid.name(*here), step))
                    return self.grid.name(*here)
                nxt = self.path[1] if len(self.path) > 1 else self.path[0]
                mv = self._choose(self.grid.center(*nxt), {here, nxt})
                if mv is None or mv[2] < 0.5:
                    mv = self._choose(self.grid.center(*here), {here}) or mv  # centre up first
                if mv is None:
                    raise RuntimeError("no safe burst from %s" % self.grid.name(*here))
                cmd, secs, _ = mv
                self.log("%s -> %s : %s %.2fs" % (self.grid.name(*here), self.grid.name(*nxt), cmd, secs))
                p0 = self.rover_xy
                self._burst(cmd, secs)
                vx, vy = self.vel[cmd]
                self.look(expect=(p0[0] + vx * secs, p0[1] + vy * secs))
                still = still + 1 if self._learn(cmd, secs, p0) < MIN_MOVE_M else 0
                if still >= 3:
                    raise RuntimeError("rover is not moving (3 bursts, no motion)")
            raise RuntimeError("step limit reached")
        finally:
            self.rover.stop()

    # ------------------------------------------------------------------ picture
    def render(self):
        if self.top is None:
            return None
        sb, g = self.sandbox, self.grid
        img = self.top.copy()
        tint = img.copy()
        for c, r in self.blocked:
            cv2.rectangle(tint, sb.m_to_map_px(c * g.cw, (r + 1) * g.ch), sb.m_to_map_px((c + 1) * g.cw, r * g.ch), (40, 40, 220), -1)
        img = cv2.addWeighted(tint, 0.35, img, 0.65, 0)
        for c in range(1, g.cols):
            x = sb.m_to_map_px(c * g.cw, 0)[0]; cv2.line(img, (x, 0), (x, img.shape[0]), (130, 130, 130), 1)
        for r in range(1, g.rows):
            y = sb.m_to_map_px(0, r * g.ch)[1]; cv2.line(img, (0, y), (img.shape[1], y), (130, 130, 130), 1)
        for c in range(g.cols):
            for r in range(g.rows):
                x, y = sb.m_to_map_px(c * g.cw, (r + 1) * g.ch)
                cv2.putText(img, g.name(c, r), (x + 4, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 90, 90), 1, cv2.LINE_AA)
        for o in self.obstacles:
            cv2.rectangle(img, sb.m_to_map_px(o["x0"], o["y1"]), sb.m_to_map_px(o["x1"], o["y0"]), (0, 0, 255), 2)
        if len(self.trail) > 1:
            cv2.polylines(img, [np.int32([sb.m_to_map_px(*p) for p in self.trail])], False, (200, 100, 255), 2, cv2.LINE_AA)
        if self.path and self.rover_xy:
            pts = [sb.m_to_map_px(*self.rover_xy)] + [sb.m_to_map_px(*g.center(*c)) for c in self.path[1:]]
            cv2.polylines(img, [np.int32(pts)], False, (255, 140, 0), 3, cv2.LINE_AA)
        if self.target:
            cv2.drawMarker(img, sb.m_to_map_px(*g.center(*self.target)), (0, 200, 255), cv2.MARKER_STAR, 26, 3)
        if self.rover_xy:
            cv2.circle(img, sb.m_to_map_px(*self.rover_xy), int(self.rover_radius * PX_PER_M), (0, 170, 0), 2)
        if self.view_path:
            cv2.imwrite(self.view_path + ".part.png", img)
            import os
            os.replace(self.view_path + ".part.png", self.view_path)
        return img

    def summary(self):
        here = self.grid.cell_of(*self.rover_xy) if self.rover_xy else None
        return {"rover": self.grid.name(*here) if here else None,
                "target": self.grid.name(*self.target) if self.target else None,
                "objects": len(self.objects),
                "blocked": sorted(self.grid.name(*c) for c in self.blocked),
                "path": [self.grid.name(*c) for c in self.path] if self.path else None}
