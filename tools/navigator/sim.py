"""Synthetic overhead world for dry-run validation (no hardware): renders an Aurora-like aligned RGB+depth frame of a
sandbox with boundary markers, obstacles and a marked rover, and simulates the rover's response to F/B/L/R/Q/E bursts.
The simulated rover's real motion conventions are hidden from the controller (they can be flipped/offset) so that the
online motion-model learning is exercised."""
import math
import time

import cv2
import numpy as np

from vision import ID_BL, ID_BR, ID_TL, ID_TR, ID_ROVER, DICT

W_MM, H_MM = 1200.0, 800.0          # sandbox size (hidden from the controller; it only sees normalized coords)
Z0 = 1100.0                          # camera height over the floor, mm
F_PX, COLS, ROWS = 424.0, 640, 400
MARK = 90.0


class SimFrame:
    pass


class SimWorld:
    def __init__(self, obstacles_mm=None, rover=(300.0, 250.0, 0.4), cam_rot=0.0, rot_sign=-1, fwd_offset=0.0,
                 fwd_speed=140.0, rot_rate=math.radians(55), noise=True, seed=3, marker=True, responsive=True, diff=False, imu_sign=-1, imu_bias=0.9):
        self.obstacles = obstacles_mm or []      # (u0,v0,u1,v1,height_mm)
        self.u, self.v, self.th = rover
        self.cam_rot, self.rot_sign, self.fwd_offset = cam_rot, rot_sign, fwd_offset
        self.fwd_speed, self.rot_rate = fwd_speed, rot_rate
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.seq = 0
        self.log = []                            # (cmd, dur, true pose after)
        self.collisions = 0
        self.marker, self.responsive = marker, responsive
        self.diff, self.imu_sign, self.imu_bias = diff, imu_sign, imu_bias

    # ----- projection: sandbox mm (u right, v up) + height -> pixel
    def px(self, u, v, h=0.0):
        X, Y = u - W_MM / 2, H_MM / 2 - v
        c, s = math.cos(self.cam_rot), math.sin(self.cam_rot)
        X, Y = c * X - s * Y, s * X + c * Y
        Z = Z0 - h
        return COLS / 2 + F_PX * X / Z, ROWS / 2 + F_PX * Y / Z

    def _poly(self, pts, h):
        return np.array([self.px(u, v, h) for u, v in pts], np.float32)

    def _marker(self, img, mid, corners_uv, h, rot=0.0):
        side = 64
        m = cv2.aruco.generateImageMarker(DICT, mid, side)
        m = cv2.copyMakeBorder(m, 12, 12, 12, 12, cv2.BORDER_CONSTANT, value=255)
        n = m.shape[0]
        src = np.array([[0, 0], [n, 0], [n, n], [0, n]], np.float32)       # marker image TL,TR,BR,BL
        dst = self._poly(corners_uv, h)
        M = cv2.getPerspectiveTransform(src, dst)
        warp = cv2.warpPerspective(m, M, (COLS, ROWS), flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), M, (COLS, ROWS))
        img[mask > 128] = warp[mask > 128, None]

    def render(self):
        g = np.full((ROWS, COLS), 150, np.uint8)
        bgr = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        floor = self._poly([(0, 0), (W_MM, 0), (W_MM, H_MM), (0, H_MM)], 0)
        cv2.fillConvexPoly(bgr, floor.astype(np.int32), (190, 190, 190))
        depth = np.full((ROWS, COLS), Z0, np.float32)
        s = MARK
        # marker corners in marker-image order TL,TR,BR,BL for an upright (un-rotated) marker, u right v up
        for mid, (u0, v0) in ((ID_BL, (-s, 0)), (ID_BR, (W_MM, 0)), (ID_TL, (-s, H_MM - s)), (ID_TR, (W_MM, H_MM - s))):
            self._marker(bgr, mid, [(u0, v0 + s), (u0 + s, v0 + s), (u0 + s, v0), (u0, v0)], 0)
        for (u0, v0, u1, v1, hh) in sorted(self.obstacles, key=lambda o: o[4]):
            poly = self._poly([(u0, v0), (u1, v0), (u1, v1), (u0, v1)], hh)
            cv2.fillConvexPoly(bgr, poly.astype(np.int32), (60, 60, 200))
            cv2.fillConvexPoly(depth, poly.astype(np.int32), float(Z0 - hh))
        # rover body 110 x 150 (front = +heading), height 80, with a 60 mm marker on top whose up edge faces front
        c, sn = math.cos(self.th), math.sin(self.th)
        body = []
        for lu, lv in ((-75, -55), (75, -55), (75, 55), (-75, 55)):        # body frame (fwd, left)
            body.append((self.u + c * lu - sn * lv, self.v + sn * lu + c * lv))
        poly = self._poly(body, 80)
        cv2.fillConvexPoly(bgr, poly.astype(np.int32), (40, 160, 220))
        cv2.fillConvexPoly(depth, poly.astype(np.int32), float(Z0 - 80))
        half = 40
        mk = []
        for lu, lv in ((half, half), (half, -half), (-half, -half), (-half, half)):   # TL,TR,BR,BL: 'up' edge = front
            mk.append((self.u + c * lu - sn * lv, self.v + sn * lu + c * lv))
        if self.marker:
            self._marker(bgr, ID_ROVER, mk, 82)
        if self.noise:
            depth += self.rng.normal(0, 3, depth.shape)
            depth[self.rng.random(depth.shape) < 0.04] = 0
            bgr = np.clip(bgr.astype(np.int16) + self.rng.normal(0, 3, bgr.shape).astype(np.int16), 0, 255).astype(np.uint8)
        # outside the field of view / invalid
        depth = np.clip(depth, 0, 65535).astype(np.uint16)
        f = SimFrame()
        f.bgr, f.gray, f.depth = bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), depth
        f.K = (F_PX, F_PX, COLS / 2, ROWS / 2)
        f.cols, f.rows = COLS, ROWS
        self.seq += 1
        f.seq, f.wall = self.seq, time.time()
        return f

    # ----- FrameSource interface
    def current_seq(self):
        return self.seq

    def wait_fresh(self, min_seq, timeout=5.0):
        self.seq = max(self.seq, min_seq - 1)
        return self.render()

    def latest(self):
        return self.render()

    # ----- rover dynamics (what the controller must discover)
    def apply(self, cmd, dur):
        if not self.responsive:
            self.seq += 3
            return
        rate = self.rot_rate * (1 + self.rng.normal(0, 0.12))
        sp = self.fwd_speed * (1 + self.rng.normal(0, 0.12))
        ang = self.th + self.fwd_offset
        fwd = (math.cos(ang), math.sin(ang))
        left = (-math.sin(ang), math.cos(ang))
        if cmd == "Q":
            self.th += self.rot_sign * rate * dur
        elif cmd == "E":
            self.th -= self.rot_sign * rate * dur
        elif self.diff and cmd in ("L", "R"):
            self.seq += 3
            return
        else:
            vx, vy = {"F": fwd, "B": (-fwd[0], -fwd[1]), "L": left, "R": (-left[0], -left[1])}[cmd]
            nu, nv = self.u + vx * sp * dur, self.v + vy * sp * dur
            if any(o[0] - 70 < nu < o[2] + 70 and o[1] - 70 < nv < o[3] + 70 for o in self.obstacles):
                self.collisions += 1
            self.u, self.v = nu, nv
        self.th = (self.th + math.pi) % (2 * math.pi) - math.pi
        self.log.append((cmd, dur, (self.u, self.v, self.th)))
        self.seq += 3   # the camera keeps producing frames while the rover moves and settles


class SimRover:
    """Drop-in for RoverLink.move/stop that drives a SimWorld."""
    dry_run = True

    def __init__(self, world):
        self.world, self.sent = world, []
        if world.diff:
            self.yaw_deg = self._yaw

    def move(self, cmd, dur):
        self.sent.append((cmd, dur))
        self.world.apply(cmd, dur)

    def stop(self, repeats=3):
        self.sent.append(("S", 0))
        return True

    def _yaw(self):
        w = self.world
        return math.degrees(w.imu_sign * w.th + w.imu_bias)

    def close(self):
        pass
