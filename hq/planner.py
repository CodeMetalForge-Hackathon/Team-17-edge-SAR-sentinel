"""Grid over the sandbox + A* route that keeps the rover clear of rubble and hazards.

Squares are named like the overhead viewer: column letter left -> right, row number bottom -> top (A1 = bottom-left).
"""
import heapq
import math


class Grid:
    def __init__(self, width_m, height_m, cols, rows):
        self.w, self.h, self.cols, self.rows = float(width_m), float(height_m), int(cols), int(rows)
        self.cw, self.ch = self.w / self.cols, self.h / self.rows

    def name(self, c, r):
        return chr(65 + c) + str(r + 1)

    def parse(self, cell):
        cell = cell.strip().upper()
        return ord(cell[0]) - 65, int(cell[1:]) - 1

    def inside(self, c, r):
        return 0 <= c < self.cols and 0 <= r < self.rows

    def center(self, c, r):
        return (c + 0.5) * self.cw, (r + 0.5) * self.ch

    def cell_of(self, x, y):
        c, r = int(math.floor(x / self.cw)), int(math.floor(y / self.ch))
        return (c, r) if self.inside(c, r) else None

    def blocked(self, obstacles, clearance_m, hazards=()):
        """Squares the rover centre must not enter: any square whose centre is closer than clearance_m to an
        obstacle rectangle (or that an obstacle covers), plus hazard squares. -> set of (c, r)."""
        out = set(hazards)
        for c in range(self.cols):
            for r in range(self.rows):
                cx, cy = self.center(c, r)
                x0, x1, y0, y1 = c * self.cw, (c + 1) * self.cw, r * self.ch, (r + 1) * self.ch
                for o in obstacles:
                    covers = o["x0"] < x1 and x0 < o["x1"] and o["y0"] < y1 and y0 < o["y1"]
                    dx = max(o["x0"] - cx, 0, cx - o["x1"])
                    dy = max(o["y0"] - cy, 0, cy - o["y1"])
                    area = (min(o["x1"], x1) - max(o["x0"], x0)) * (min(o["y1"], y1) - max(o["y0"], y0))
                    if math.hypot(dx, dy) < clearance_m or (covers and area > 0.15 * self.cw * self.ch):
                        out.add((c, r))
                        break
        return out

    def route(self, start, goal, blocked):
        """4-connected A* from start to goal (cells as (c, r)). The start square is always allowed (the rover is
        there). -> list of cells or None."""
        if not (self.inside(*start) and self.inside(*goal)) or goal in blocked:
            return None
        avoid = set(blocked) - {start}
        h = lambda p: abs(p[0] - goal[0]) + abs(p[1] - goal[1])
        frontier, came, cost = [(h(start), 0, start)], {start: None}, {start: 0}
        while frontier:
            _, d, cur = heapq.heappop(frontier)
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came[cur]
                return path[::-1]
            for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (cur[0] + dc, cur[1] + dr)
                if not self.inside(*nxt) or nxt in avoid:
                    continue
                if d + 1 < cost.get(nxt, 1e9):
                    cost[nxt], came[nxt] = d + 1, cur
                    heapq.heappush(frontier, (d + 1 + h(nxt), d + 1, nxt))
        return None
