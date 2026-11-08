"""Planning geometry in the sandbox 'square frame'.

Poses and obstacles are stored in normalized sandbox coordinates (BL=(0,0), BR=(1,0), TR=(1,1), TL=(0,1)).
Distances and angles are computed in the square frame u = x * aspect, v = y (aspect = sandbox width / height,
measured from the depth ground plane when available) so that headings and clearances are not distorted.
All lengths here are dimensionless (units of sandbox height); there are no metres.
"""
import heapq
import math

EPS = 1e-9


def to_sq(x, y, aspect):
    return x * aspect, y


def from_sq(u, v, aspect):
    return u / aspect, v


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def cell_rect_sq(c, r, cols, rows, aspect):
    return (c / cols * aspect, r / rows, (c + 1) / cols * aspect, (r + 1) / rows)  # u0,v0,u1,v1


def rect_to_sq(rect, aspect):
    return (rect[0] * aspect, rect[1], rect[2] * aspect, rect[3])


def rect_dist(a, b):
    dx = max(0.0, a[0] - b[2], b[0] - a[2])
    dy = max(0.0, a[1] - b[3], b[1] - a[3])
    return math.hypot(dx, dy)


def rects_overlap(a, b):
    return a[0] < b[2] - EPS and b[0] < a[2] - EPS and a[1] < b[3] - EPS and b[1] < a[3] - EPS


def enclosing_radius(w, h):
    return math.hypot(w, h) / 2


def cell_of(x, y, cols, rows):
    c = min(cols - 1, max(0, int(x * cols)))
    r = min(rows - 1, max(0, int(y * rows)))
    return c, r


def cell_centre(c, r, cols, rows):
    return (c + 0.5) / cols, (r + 0.5) / rows


def planning_blocked(obstacles_norm, cols, rows, d, aspect):
    """Cells where the rover CENTRE may not be: within d of any obstacle rect (normalized x0,y0,x1,y1),
    or closer than d to the sandbox edge. Whole-cell test, conservative. Vectorised (obstacles can be thousands)."""
    import numpy as np
    cc, rr = np.meshgrid(np.arange(cols), np.arange(rows), indexing="ij")
    cu0, cv0 = cc.ravel() / cols * aspect, rr.ravel() / rows
    cu1, cv1 = (cc.ravel() + 1) / cols * aspect, (rr.ravel() + 1) / rows
    bad = (cu0 < d - EPS) | (cv0 < d - EPS) | (cu1 > aspect - d + EPS) | (cv1 > 1 - d + EPS)
    if len(obstacles_norm):
        O = np.asarray(obstacles_norm, np.float64)
        ou0, ov0, ou1, ov1 = O[:, 0] * aspect, O[:, 1], O[:, 2] * aspect, O[:, 3]
        for i in range(0, len(cu0), 128):                      # chunk to bound memory
            sl = slice(i, i + 128)
            dx = np.maximum(0, np.maximum(cu0[sl, None] - ou1[None], ou0[None] - cu1[sl, None]))
            dy = np.maximum(0, np.maximum(cv0[sl, None] - ov1[None], ov0[None] - cv1[sl, None]))
            bad[sl] |= (np.hypot(dx, dy) < d - EPS).any(1)
    return {(int(c), int(r)) for c, r, b in zip(cc.ravel(), rr.ravel(), bad) if b}


def astar(cols, rows, blocked, start, target):
    """Four-connected A* (Manhattan heuristic). The start cell is allowed even when inflated-blocked
    (the rover may be sitting inside the clearance band and must be able to leave it)."""
    inb = lambda p: 0 <= p[0] < cols and 0 <= p[1] < rows
    if not inb(start):
        return None, "start outside grid"
    if not inb(target):
        return None, "target outside grid"
    if target in blocked:
        return None, "target cell is blocked"
    h = lambda p: abs(p[0] - target[0]) + abs(p[1] - target[1])
    g = {start: 0}
    prev = {}
    closed = set()
    heap = [(h(start), 0, start)]
    while heap:
        _, gc, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == target:
            route = [cur]
            while cur in prev:
                cur = prev[cur]
                route.append(cur)
            return route[::-1], ""
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (cur[0] + dc, cur[1] + dr)
            if not inb(n) or n in blocked or n in closed:
                continue
            ng = gc + 1
            if ng < g.get(n, 1e9):
                g[n] = ng
                prev[n] = cur
                heapq.heappush(heap, (ng + h(n), -ng, n))
    return None, "no route: target unreachable with the current obstacles and clearance"


def _segment_free(p, q, cols, rows, blocked, start_cell):
    n = int(max(abs(q[0] - p[0]) * cols, abs(q[1] - p[1]) * rows) * 4) + 1
    for i in range(n + 1):
        t = i / n
        c = cell_of(p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, cols, rows)
        if c in blocked and c != start_cell:
            return False
    return True


def plan(pos, target, obstacles_norm, cols, rows, radius, margin, aspect):
    """Route from pos to target (both normalized x,y). Returns dict with waypoints (normalized, string-pulled
    straight segments), cells, blocked set, and an error string when there is no route."""
    d = radius + margin
    blocked = planning_blocked(obstacles_norm, cols, rows, d, aspect)
    sc = cell_of(pos[0], pos[1], cols, rows)
    tc = cell_of(target[0], target[1], cols, rows)
    route, err = astar(cols, rows, blocked, sc, tc)
    res = {"blocked": blocked, "cells": route or [], "waypoints": [], "error": err, "clearance": d}
    if not route:
        return res
    pts = [tuple(pos)] + [cell_centre(c, r, cols, rows) for c, r in route[1:-1]] + [tuple(target)]
    # string-pull: skip intermediate points while the straight segment stays out of blocked cells
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _segment_free(pts[i], pts[j], cols, rows, blocked, sc):
            j -= 1
        out.append(pts[j])
        i = j
    res["waypoints"] = out
    return res


def path_length_sq(wps, aspect):
    return sum(math.hypot((b[0] - a[0]) * aspect, b[1] - a[1]) for a, b in zip(wps, wps[1:]))


# ---------------------------------------------------------------------------------------------------------------
# Oriented planning. The rover is a rectangle (L along its heading, W across). It travels along grid axes with its
# heading aligned to the segment, so a straight run needs only the rectangle's half-extents as clearance, while
# turning in place needs the enclosing circle. Planning therefore searches (cell, direction) states: a move along x is
# checked against the footprint with heading +-x, a move along y against heading +-y, and a change of direction is only
# allowed in cells that are free for the enclosing circle. Everything is dimensionless (sandbox height units).
def rect_blocked(obstacles_norm, cols, rows, ex, ey, aspect):
    """Cells where an axis-aligned footprint with half extents (ex, ey) (square-frame units) centred in the cell
    could touch an obstacle or leave the sandbox. Whole-cell test (conservative)."""
    import numpy as np
    cc, rr = np.meshgrid(np.arange(cols), np.arange(rows), indexing="ij")
    cu0, cv0 = cc.ravel() / cols * aspect, rr.ravel() / rows
    cu1, cv1 = (cc.ravel() + 1) / cols * aspect, (rr.ravel() + 1) / rows
    bad = (cu0 < ex - EPS) | (cv0 < ey - EPS) | (cu1 > aspect - ex + EPS) | (cv1 > 1 - ey + EPS)
    if len(obstacles_norm):
        O = np.asarray(obstacles_norm, np.float64)
        ou0, ov0, ou1, ov1 = O[:, 0] * aspect - ex, O[:, 1] - ey, O[:, 2] * aspect + ex, O[:, 3] + ey
        for i in range(0, len(cu0), 256):
            sl = slice(i, i + 256)
            ov = (cu0[sl, None] < ou1[None] - EPS) & (ou0[None] < cu1[sl, None] - EPS) & \
                 (cv0[sl, None] < ov1[None] - EPS) & (ov0[None] < cv1[sl, None] - EPS)
            bad[sl] |= ov.any(1)
    return {(int(c), int(r)) for c, r, b in zip(cc.ravel(), rr.ravel(), bad) if b}


DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))   # E, W, N, S


def clearance_sets(obstacles_norm, cols, rows, L, W, margin, aspect):
    """(blocked_for_x_moves, blocked_for_y_moves, blocked_for_turns) for footprint L x W plus margin."""
    m = margin
    bx = rect_blocked(obstacles_norm, cols, rows, L / 2 + m, W / 2 + m, aspect)
    by = rect_blocked(obstacles_norm, cols, rows, W / 2 + m, L / 2 + m, aspect)
    bt = planning_blocked(obstacles_norm, cols, rows, math.hypot(L, W) / 2 + m, aspect)
    return bx, by, bt


def plan_oriented(pos, target, obstacles_norm, cols, rows, L, W, margin, aspect, turn_cost=10, fixed=False, heading=None):
    """fixed=True: the rover keeps a fixed world orientation (mecanum strafing, no turns): one AABB footprint
    (L along x, W along y) for every move and a small direction-change cost."""
    sc = cell_of(pos[0], pos[1], cols, rows)
    tc = cell_of(target[0], target[1], cols, rows)
    if fixed:
        bx = rect_blocked(obstacles_norm, cols, rows, L / 2 + margin, W / 2 + margin, aspect)
        by, bt, turn_cost = bx, bx, 2
    else:
        bx, by, bt = clearance_sets(obstacles_norm, cols, rows, L, W, margin, aspect)
    res = {"blocked": bx | by, "blocked_x": bx, "blocked_y": by, "blocked_turn": bt, "cells": [], "waypoints": [],
           "error": "", "clearance": (L, W, margin)}
    if tc in bx and tc in by:
        res["error"] = "target is too close to an obstacle or the table edge for the rover"
        return res
    inb = lambda p: 0 <= p[0] < cols and 0 <= p[1] < rows
    h = lambda p: abs(p[0] - tc[0]) + abs(p[1] - tc[1])
    import heapq
    start = (sc, -1)
    g = {start: 0}
    prev = {}
    heap = [(h(sc), 0, 0, start)]
    tie = 0
    goal = None
    closed = set()
    while heap:
        _, gc, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        (cell, d) = cur
        if cell == tc:
            goal = cur
            break
        for nd, (dc, dr) in enumerate(DIRS):
            n = (cell[0] + dc, cell[1] + dr)
            if not inb(n):
                continue
            blk = bx if nd < 2 else by
            if n in blk:
                continue
            cost = 1
            if d == -1 and heading is not None:
                # the rover starts with a known heading and can drive forward OR reverse: charge the rotation needed to
                # line up with the first leg (a 180 degree turn is never needed: reversing covers it)
                ang = math.atan2(dr, dc * aspect) if False else math.atan2(dr, dc)
                dth = min(abs(wrap(ang - heading)), abs(wrap(ang - heading - math.pi)))
                cost += turn_cost * dth / (math.pi / 2)
            if d != -1 and nd != d:
                # direction change (a rotation in place) happens at `cell`: it must be free for the enclosing circle,
                # unless it is the start cell (the rover starts wherever it is and may already be aligned)
                if cell != sc and cell in bt:
                    continue
                cost += turn_cost
            if d == -1 and nd != d:
                cost += 0
            ns = (n, nd)
            ng = gc + cost
            if ng < g.get(ns, 1e9):
                g[ns] = ng
                prev[ns] = cur
                tie += 1
                heapq.heappush(heap, (ng + h(n), ng, tie, ns))
    if goal is None:
        res["error"] = "no route: target unreachable with the current obstacles, rover size and clearance"
        return res
    seq = [goal]
    while seq[-1] in prev:
        seq.append(prev[seq[-1]])
    seq = seq[::-1]
    cells = [s[0] for s in seq]
    res["cells"] = cells
    # vertices at direction changes; actual start and target positions at the ends
    pts = [tuple(pos)]
    for i in range(1, len(seq) - 1):
        if seq[i + 1][1] != seq[i][1]:
            pts.append(cell_centre(cells[i][0], cells[i][1], cols, rows))
    pts.append(tuple(target))
    res["waypoints"] = pts
    return res
