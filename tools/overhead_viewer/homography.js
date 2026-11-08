// Pure geometry for the overhead viewer. Works in browser (window.Homography) and Node (module.exports).
// Normalized frame: BL=(0,0) BR=(1,0) TR=(1,1) TL=(0,1). Dimensionless; NOT metres.
(function (root) {
  "use strict";

  // Solve A x = b (n x n) by Gaussian elimination with partial pivoting. Returns null if singular.
  function solve(A, b) {
    const n = b.length;
    const M = A.map((r, i) => r.concat([b[i]]));
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-12) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = c + 1; r < n; r++) {
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    const x = new Array(n).fill(0);
    for (let r = n - 1; r >= 0; r--) {
      let s = M[r][n];
      for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
      x[r] = s / M[r][r];
    }
    return x;
  }

  // 3x3 homography (row-major, length 9) mapping src[i] -> dst[i] for 4 point pairs.
  function homography(src, dst) {
    const A = [], b = [];
    for (let i = 0; i < 4; i++) {
      const [x, y] = src[i], [u, v] = dst[i];
      A.push([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.push(u);
      A.push([0, 0, 0, x, y, 1, -v * x, -v * y]); b.push(v);
    }
    const h = solve(A, b);
    return h ? h.concat([1]) : null;
  }

  function apply(H, x, y) {
    const w = H[6] * x + H[7] * y + H[8];
    if (Math.abs(w) < 1e-12) return null;
    return [(H[0] * x + H[1] * y + H[2]) / w, (H[3] * x + H[4] * y + H[5]) / w];
  }

  function invert(H) {
    const [a, b, c, d, e, f, g, h, i] = H;
    const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    const det = a * A + b * B + c * C;
    if (Math.abs(det) < 1e-14) return null;
    return [A, -(b * i - c * h), b * f - c * e,
            B, a * i - c * g, -(a * f - c * d),
            C, -(a * h - b * g), a * e - b * d].map(v => v / det);
  }

  function cross(o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); }

  // corners: [BL, BR, TR, TL] in image pixels. Returns {ok, error, area}.
  function validateQuad(corners, imgW, imgH, minAreaFrac) {
    minAreaFrac = minAreaFrac == null ? 0.02 : minAreaFrac;
    if (!corners || corners.length !== 4) return { ok: false, error: "need 4 corners" };
    for (const [x, y] of corners) {
      if (!isFinite(x) || !isFinite(y)) return { ok: false, error: "non-finite corner" };
      if (x < 0 || y < 0 || x > imgW || y > imgH) return { ok: false, error: "corner outside image" };
    }
    // Image y points down, so BL->BR->TR->TL (counter-clockwise on screen with y up) has NEGATIVE cross in image coords.
    const signs = [];
    for (let i = 0; i < 4; i++) signs.push(Math.sign(cross(corners[i], corners[(i + 1) % 4], corners[(i + 2) % 4])));
    if (signs.some(s => s === 0)) return { ok: false, error: "degenerate boundary (collinear corners)" };
    if (!signs.every(s => s === signs[0])) return { ok: false, error: "boundary is not convex / corners are crossed" };
    if (signs[0] > 0) return { ok: false, error: "corner order is mirrored: expected BL, BR, TR, TL as seen in the image" };
    let area = 0;
    for (let i = 0; i < 4; i++) { const p = corners[i], q = corners[(i + 1) % 4]; area += p[0] * q[1] - q[0] * p[1]; }
    area = Math.abs(area) / 2;
    if (area < minAreaFrac * imgW * imgH) return { ok: false, error: "boundary too small (area " + area.toFixed(0) + " px²)" };
    return { ok: true, area };
  }

  // Build {toNorm, toImg} from corners, or {error}.
  function calibrate(corners, imgW, imgH) {
    const v = validateQuad(corners, imgW, imgH);
    if (!v.ok) return { error: v.error };
    const H = homography(corners, [[0, 0], [1, 0], [1, 1], [0, 1]]);
    const Hi = H && invert(H);
    if (!H || !Hi) return { error: "singular homography" };
    return { H, Hi, area: v.area };
  }

  const EPS = 1e-9;
  // Image pixel -> {x,y,col,row,cell} or {error}. Rejects outside boundary.
  function pixelToCell(cal, px, py, cols, rows) {
    const p = apply(cal.H, px, py);
    if (!p) return { error: "point at infinity" };
    const [x, y] = p;
    if (x < -EPS || x > 1 + EPS || y < -EPS || y > 1 + EPS) return { error: "outside sandbox boundary", x, y };
    return normToCell(x, y, cols, rows);
  }
  function normToCell(x, y, cols, rows) {
    const cx = Math.min(Math.max(x, 0), 1), cy = Math.min(Math.max(y, 0), 1);
    const col = Math.min(cols - 1, Math.floor(cx * cols));
    const row = Math.min(rows - 1, Math.floor(cy * rows)); // row 0 = bottom
    return { x: cx, y: cy, col, row, cell: colName(col) + (row + 1) };
  }
  function colName(i) { let s = ""; i += 1; while (i > 0) { const m = (i - 1) % 26; s = String.fromCharCode(65 + m) + s; i = Math.floor((i - 1) / 26); } return s; }


  // A* on a cols x rows grid, 4-connected, unit cost, Manhattan heuristic. Cells are [col,row] (row 0 = bottom).
  // blocked: array/Set of "c,r" strings. Returns {route:[[c,r],...]} or {error}.
  function astar(cols, rows, blocked, start, target) {
    const B = new Set(blocked), key = (c, r) => c + "," + r;
    const inb = p => p && Number.isInteger(p[0]) && Number.isInteger(p[1]) && p[0] >= 0 && p[1] >= 0 && p[0] < cols && p[1] < rows;
    if (!inb(start)) return { error: "start cell not set or outside grid" };
    if (!inb(target)) return { error: "target cell not set or outside grid" };
    if (B.has(key(...start))) return { error: "start cell is blocked" };
    if (B.has(key(...target))) return { error: "target cell is blocked" };
    const h = (c, r) => Math.abs(c - target[0]) + Math.abs(r - target[1]);
    const g = new Map([[key(...start), 0]]), prev = new Map(), closed = new Set();
    const open = [[h(...start), 0, start[0], start[1]]]; // [f, g, c, r]; small grids, linear scan is fine
    while (open.length) {
      let bi = 0;
      for (let i = 1; i < open.length; i++) if (open[i][0] < open[bi][0] || (open[i][0] === open[bi][0] && open[i][1] > open[bi][1])) bi = i;
      const [, gc, c, r] = open.splice(bi, 1)[0], k = key(c, r);
      if (closed.has(k)) continue;
      closed.add(k);
      if (c === target[0] && r === target[1]) {
        const route = [[c, r]];
        for (let q = prev.get(k); q; q = prev.get(key(...q))) route.unshift(q);
        return { route };
      }
      for (const [dc, dr] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const nc = c + dc, nr = r + dr, nk = key(nc, nr);
        if (nc < 0 || nr < 0 || nc >= cols || nr >= rows || B.has(nk) || closed.has(nk)) continue;
        if (!g.has(nk) || gc + 1 < g.get(nk)) { g.set(nk, gc + 1); prev.set(nk, [c, r]); open.push([gc + 1 + h(nc, nr), gc + 1, nc, nr]); }
      }
    }
    return { error: "no route: target unreachable with the current blocked cells" };
  }
  function cellCentre(c, r, cols, rows) { return [(c + 0.5) / cols, (r + 0.5) / rows]; }

  // ---- footprint geometry (all in normalized sandbox coords; rect = {x0,y0,x1,y1}, x0<x1, y0<y1) ----
  const GEPS = 1e-9;
  function normRect(ax, ay, bx, by) {
    const cl = v => Math.min(1, Math.max(0, v));
    return { x0: cl(Math.min(ax, bx)), y0: cl(Math.min(ay, by)), x1: cl(Math.max(ax, bx)), y1: cl(Math.max(ay, by)) };
  }
  function rectCentre(r) { return [(r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2]; }
  function rectSize(r) { return [r.x1 - r.x0, r.y1 - r.y0]; }
  // radius of the circle enclosing the rectangle (conservative for any heading)
  function enclosingRadius(w, h) { return Math.hypot(w, h) / 2; }
  // positive-area overlap (touching edges do not count)
  function rectsOverlap(a, b) { return a.x0 < b.x1 - GEPS && b.x0 < a.x1 - GEPS && a.y0 < b.y1 - GEPS && b.y0 < a.y1 - GEPS; }
  function rectDist(a, b) {
    const dx = Math.max(0, a.x0 - b.x1, b.x0 - a.x1), dy = Math.max(0, a.y0 - b.y1, b.y0 - a.y1);
    return Math.hypot(dx, dy);
  }
  function cellRect(c, r, cols, rows) { return { x0: c / cols, x1: (c + 1) / cols, y0: r / rows, y1: (r + 1) / rows }; }
  // every cell the rectangle overlaps (positive area)
  function overlapCells(rect, cols, rows) {
    const out = [];
    for (let c = 0; c < cols; c++) for (let r = 0; r < rows; r++) if (rectsOverlap(rect, cellRect(c, r, cols, rows))) out.push([c, r]);
    return out;
  }
  // Cells where the rover CENTRE may not be, for radius d (rover enclosing radius + margin):
  // any cell within d of an obstacle, or not fully inside [d,1-d]^2 (centre must keep d from the sandbox edge).
  // Whole-cell test, so it is conservative wherever the centre sits inside the cell.
  function planningBlocked(obstacles, cols, rows, d) {
    const out = new Set();
    for (let c = 0; c < cols; c++) for (let r = 0; r < rows; r++) {
      const cr = cellRect(c, r, cols, rows);
      let bad = cr.x0 < d - GEPS || cr.y0 < d - GEPS || cr.x1 > 1 - d + GEPS || cr.y1 > 1 - d + GEPS;
      for (let i = 0; !bad && i < obstacles.length; i++) bad = rectDist(cr, obstacles[i]) < d - GEPS;
      if (bad) out.add(c + "," + r);
    }
    return out;
  }
  // Rover footprint (w x h, axis-aligned) centred at cell (c,r): why it is not allowed there, or "".
  function footprintProblem(c, r, cols, rows, w, h, obstacles) {
    const [cx, cy] = cellCentre(c, r, cols, rows);
    const f = { x0: cx - w / 2, x1: cx + w / 2, y0: cy - h / 2, y1: cy + h / 2 };
    if (f.x0 < -GEPS || f.y0 < -GEPS || f.x1 > 1 + GEPS || f.y1 > 1 + GEPS) return "footprint extends outside the sandbox";
    for (let i = 0; i < obstacles.length; i++) if (rectsOverlap(f, obstacles[i])) return "footprint overlaps obstacle " + (i + 1);
    return "";
  }

  const api = { normRect, rectCentre, rectSize, enclosingRadius, rectsOverlap, rectDist, cellRect, overlapCells, planningBlocked, footprintProblem, astar, cellCentre, solve, homography, apply, invert, validateQuad, calibrate, pixelToCell, normToCell, colName };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.Homography = api;
})(typeof window !== "undefined" ? window : globalThis);
