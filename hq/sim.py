"""Simulated overhead camera + rover, for testing the whole loop without hardware (and as a demo fallback).

The fake camera draws the sandbox, 4 ArUco corner markers, cardboard boxes and the rover (with its ArUco marker)
from above, then warps it into a tilted 2304x1296 "photo" with blur and noise. The fake rover's real motion
directions are deliberately NOT aligned with its marker, so the autopilot has to calibrate like on hardware.
"""
import math
import random

import cv2
import numpy as np

CANVAS_PX_PER_M = 500
MARGIN_M = 0.16
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


class SimWorld:
    def __init__(self, size=(1.5, 1.0), rover=(0.2, 0.15, math.radians(90)), corner_ids=(4, 7, 9, 11),
                 rover_id=10, corner_marker_m=0.10, rover_marker_m=0.08, seed=0, white_board=False):
        self.w, self.h = size
        self.x, self.y, self.th = rover
        self.corner_ids, self.rover_id = corner_ids, rover_id
        self.cm, self.rm = corner_marker_m, rover_marker_m
        self.boxes = []          # (x0, y0, x1, y1) metres
        self.rng = np.random.default_rng(seed)
        self.white = white_board
        self.floor = self._floor()

    # canvas: sandbox plus a margin for the corner markers, y down
    def px(self, x, y):
        return ((x + MARGIN_M) * CANVAS_PX_PER_M, (self.h + MARGIN_M - y) * CANVAS_PX_PER_M)

    def _floor(self):
        W = int((self.w + 2 * MARGIN_M) * CANVAS_PX_PER_M)
        H = int((self.h + 2 * MARGIN_M) * CANVAS_PX_PER_M)
        img = np.full((H, W, 3), (205, 212, 218), np.uint8)
        img = (img + self.rng.normal(0, 5, img.shape)).clip(0, 255).astype(np.uint8)
        p0, p1 = self.px(0, self.h), self.px(self.w, 0)
        if self.white:  # plain white board, markers sit on it (like the real table)
            img[:] = (232, 236, 238)
            img = (img + self.rng.normal(0, 3, img.shape)).clip(0, 255).astype(np.uint8)
        else:
            cv2.rectangle(img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), (150, 170, 185), -1)  # sandbox floor
        for k in range(1, 6) if not self.white else ():  # tape grid
            x = int(self.px(k * self.w / 6, 0)[0]); cv2.line(img, (x, int(p0[1])), (x, int(p1[1])), (90, 90, 90), 2)
        for k in range(1, 4) if not self.white else ():
            y = int(self.px(0, k * self.h / 4)[1]); cv2.line(img, (int(p0[0]), y), (int(p1[0]), y), (90, 90, 90), 2)
        # corner markers, outside the sandbox with their inner corner on the sandbox corner (BL, BR, TR, TL)
        s = self.cm
        for mid, (x0, y0) in zip(self.corner_ids, [(-s, -s), (self.w, -s), (self.w, self.h), (-s, self.h)]):
            self._marker(img, mid, x0 + s / 2, y0 + s / 2, math.pi / 2, s)
        return img

    def _marker(self, img, mid, cx, cy, th, side):
        """Paste ArUco marker mid centred at (cx, cy) m, its 'up' along heading th, with a white quiet zone."""
        ux, uy, rx, ry = math.cos(th), math.sin(th), math.sin(th), -math.cos(th)
        def quad(k):
            h = side / 2 * k
            return np.float32([self.px(cx + (-rx + ux) * h, cy + (-ry + uy) * h), self.px(cx + (rx + ux) * h, cy + (ry + uy) * h),
                               self.px(cx + (rx - ux) * h, cy + (ry - uy) * h), self.px(cx + (-rx - ux) * h, cy + (-ry - uy) * h)])
        cv2.fillConvexPoly(img, np.int32(quad(1.35)), (255, 255, 255))
        m = cv2.cvtColor(cv2.aruco.generateImageMarker(DICT, mid, 120), cv2.COLOR_GRAY2BGR)
        M = cv2.getPerspectiveTransform(np.float32([[0, 0], [120, 0], [120, 120], [0, 120]]), quad(1.0))
        warped = cv2.warpPerspective(m, M, (img.shape[1], img.shape[0]), flags=cv2.INTER_NEAREST)
        mask = cv2.warpPerspective(np.full((120, 120), 255, np.uint8), M, (img.shape[1], img.shape[0]), flags=cv2.INTER_NEAREST)
        img[mask > 0] = warped[mask > 0]

    def top_view(self):
        img = self.floor.copy()
        for x0, y0, x1, y1 in self.boxes:
            p0, p1 = self.px(x0, y1), self.px(x1, y0)
            cv2.rectangle(img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), (70, 125, 175), -1)
            cv2.rectangle(img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), (40, 75, 110), 4)
        c = self.px(self.x, self.y)
        if self.rover_id is None:  # marker-less rover: dark square body with yellow wheels
            r = int(0.05 * CANVAS_PX_PER_M)
            box = cv2.boxPoints(((c[0], c[1]), (2 * r, 2 * r), -math.degrees(self.th)))
            cv2.fillConvexPoly(img, np.int32(box), (50, 50, 55))
            for q in box:
                cv2.circle(img, (int(q[0]), int(q[1])), r // 3, (30, 200, 240), -1)
        else:
            cv2.circle(img, (int(c[0]), int(c[1])), int(0.075 * CANVAS_PX_PER_M), (60, 60, 60), -1)  # rover body
            self._marker(img, self.rover_id, self.x, self.y, self.th, self.rm)
        return img

    def photo(self, gain=0):
        top = self.top_view()
        H, W = top.shape[:2]
        src = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
        dst = np.float32([[790, 300], [1560, 290], [1610, 1010], [740, 1020]])  # slightly tilted camera
        M = cv2.getPerspectiveTransform(src, dst)
        out = np.full((1296, 2304, 3), (95, 95, 95), np.uint8)
        warped = cv2.warpPerspective(top, M, (2304, 1296), flags=cv2.INTER_AREA)
        mask = cv2.warpPerspective(np.full((H, W), 255, np.uint8), M, (2304, 1296))
        out[mask > 0] = warped[mask > 0]
        out = cv2.GaussianBlur(out, (3, 3), 0)
        return (out.astype(np.int16) + gain + self.rng.normal(0, 3, out.shape)).clip(0, 255).astype(np.uint8)

    def inside_box(self, x, y, pad=0.0):
        return any(x0 - pad < x < x1 + pad and y0 - pad < y < y1 + pad for x0, y0, x1, y1 in self.boxes)


class SimCamera:
    def __init__(self, world):
        self.world = world

    def capture(self):
        return self.world.photo(gain=random.randint(-4, 4))


class SimRover:
    """True motion per command, in the rover marker's frame (forward, left) m/s, deliberately skewed."""
    TRUE = {"F": (0.16, math.radians(14)), "L": (0.13, math.radians(102))}
    TURN = math.radians(85)  # Q, deg/s (CCW)

    def __init__(self, world, noise=0.08):
        self.world, self.noise, self.max_burst = world, noise, 1.0
        self.log = []

    def move(self, cmd, secs):
        w = self.world
        self.log.append((cmd, round(secs, 2)))
        k = 1 + random.uniform(-self.noise, self.noise)
        if cmd in ("Q", "E"):
            w.th += (1 if cmd == "Q" else -1) * self.TURN * secs * k
            return
        base = {"F": ("F", 1), "B": ("F", -1), "L": ("L", 1), "R": ("L", -1)}[cmd]
        sp, ang = self.TRUE[base[0]]
        sp *= base[1] * k
        for _ in range(10):  # integrate in small steps, slight heading drift while moving
            a, b = sp * math.cos(ang) * secs / 10, sp * math.sin(ang) * secs / 10
            w.x += a * math.cos(w.th) - b * math.sin(w.th)
            w.y += a * math.sin(w.th) + b * math.cos(w.th)
            w.th += random.uniform(-0.004, 0.004)

    def stop(self):
        return True
