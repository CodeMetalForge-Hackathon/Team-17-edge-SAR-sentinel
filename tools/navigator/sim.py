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
                 fwd_speed=140.0, rot_rate=math.radians(55), noise=True, seed=3, marker=True, responsive=True, diff=False, imu_sign=-1, imu_bias=0.9, rover_holes=False):
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
        self.driving = None            # (left, right) wheel command being streamed
        self.stale_stop_log = []
        self.diff, self.imu_sign, self.imu_bias = diff, imu_sign, imu_bias
        self.rover_holes = rover_holes           # the real rover's dark body returns NO depth (holes), as on the live camera

    def _hits(self, u, v, th):
        """True if the rover's actual footprint (150 x 110 mm rectangle along its heading) overlaps an obstacle."""
        c, sn = math.cos(th), math.sin(th)
        pts = [(a, b) for a in (-75, -37, 0, 37, 75) for b in (-55, 0, 55)]
        for o in self.obstacles:
            for a, b in pts:
                pu, pv = u + c * a - sn * b, v + sn * a + c * b
                if o[0] < pu < o[2] and o[1] < pv < o[3]:
                    return True
        return False

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
        c, sn = math.cos(self.th + self.fwd_offset), math.sin(self.th + self.fwd_offset)
        body = []
        for lu, lv in ((-75, -55), (75, -55), (75, 55), (-75, 55)):        # body frame (fwd, left)
            body.append((self.u + c * lu - sn * lv, self.v + sn * lu + c * lv))
        poly = self._poly(body, 80)
        cv2.fillConvexPoly(bgr, poly.astype(np.int32), (40, 160, 220))
        cv2.fillConvexPoly(depth, poly.astype(np.int32), float(Z0 - 80))
        c, sn = math.cos(self.th), math.sin(self.th)
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
        if self.rover_holes:
            c0, s0 = math.cos(self.th + self.fwd_offset), math.sin(self.th + self.fwd_offset)
            inner = [(self.u + c0 * lu * 0.8 - s0 * lv * 0.8, self.v + s0 * lu * 0.8 + c0 * lv * 0.8) for lu, lv in ((-75, -55), (75, -55), (75, 55), (-75, 55))]
            m = np.zeros(depth.shape, np.uint8)
            cv2.fillConvexPoly(m, self._poly(inner, 80).astype(np.int32), 1)
            depth[m.astype(bool)] = 0
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
        new = max(self.seq + 1, min_seq)
        if self.driving is not None:            # streamed driving: the rover keeps moving between frames (5 fps)
            self.integrate(self.driving, (new - self.seq) * 0.2)
        self.seq = new - 1
        return self.render()

    def integrate(self, lr, dt):
        l, r = lr
        v = self.fwd_speed * ((l + r) / 2) / 0.35
        w = self.rot_sign * self.rot_rate * (r - l)
        steps = max(1, int(dt / 0.05))
        for _ in range(steps):
            h = dt / steps
            self.th += w * h
            self.u += math.cos(self.th + self.fwd_offset) * v * h
            self.v += math.sin(self.th + self.fwd_offset) * v * h
            if self._hits(self.u, self.v, self.th + self.fwd_offset):
                self.collisions += 1
        self.th = (self.th + math.pi) % (2 * math.pi) - math.pi

    def latest(self):
        return self.render()

    def latest_full(self):
        f = self.render()
        f.rx = time.time()
        return f

    # ----- rover dynamics (what the controller must discover)
    def apply(self, cmd, dur, speed=0.35, trim=0.0):
        if not self.responsive:
            self.seq += 3
            return
        rate = self.rot_rate * (1 + self.rng.normal(0, 0.12))
        sp = self.fwd_speed * (1 + self.rng.normal(0, 0.12)) * (speed / 0.35)
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
            self.th += self.rot_sign * self.rot_rate * (2 * trim * speed / 0.35) * dur * (1 if cmd == "F" else -1)
            vx, vy = {"F": fwd, "B": (-fwd[0], -fwd[1]), "L": left, "R": (-left[0], -left[1])}[cmd]
            nu, nv = self.u + vx * sp * dur, self.v + vy * sp * dur
            if self._hits(nu, nv, self.th + self.fwd_offset):
                self.collisions += 1
            self.u, self.v = nu, nv
        self.th = (self.th + math.pi) % (2 * math.pi) - math.pi
        self.log.append((cmd, dur, (self.u, self.v, self.th)))
        self.seq += 3   # the camera keeps producing frames while the rover moves and settles


class SimRover:
    """Drop-in for UgvLink: move / turn_by / drive / halt / stop / yaw_deg driving a SimWorld."""
    dry_run = True

    def __init__(self, world):
        self.world, self.sent = world, []
        self.yaw_deg = self._yaw if world.diff else None
        self.speed = 0.35

    def move(self, cmd, dur, speed=None, trim=0.0):
        self.halt()
        self.sent.append((cmd, dur))
        self.world.apply(cmd, dur, speed or self.speed, trim)

    def turn_by(self, cmd, target_rad):
        """IMU-closed turn: rotates by about the target (+-10 %), sign by the rover's real (hidden) convention."""
        self.halt()
        w = self.world
        got = target_rad * (1 + w.rng.normal(0, 0.10))
        w.th += (w.rot_sign if cmd == "Q" else -w.rot_sign) * got
        w.th = (w.th + math.pi) % (2 * math.pi) - math.pi
        self.sent.append((cmd, got))
        w.seq += 3
        return got

    def drive(self, left, right, ttl=0.9):
        self.sent.append(("DRIVE", (round(left, 3), round(right, 3))))
        self.world.driving = (left, right)

    def halt(self):
        if self.world.driving is not None:
            self.world.driving = None
            self.sent.append(("S", 0))
        return True

    def stop(self, repeats=3):
        self.world.driving = None
        self.sent.append(("S", 0))
        return True

    def _yaw(self):
        w = self.world
        return math.degrees(w.imu_sign * w.th + w.imu_bias)

    def close(self):
        pass
