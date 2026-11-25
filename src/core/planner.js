// Constrained route planner. Theta* over a terrain grid with slope, forbidden
// zone and clearance penalties. Produces a smoothed control polyline that the
// spline sampler turns into a drivable centreline.

import { clamp, dist, lerp, simplify } from './math.js';

const NEIGHBOURS = [
  [1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1],
  [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2],
];

class MinHeap {
  constructor() {
    this.items = [];
    this.keys = [];
  }
  get size() {
    return this.items.length;
  }
  push(item, key) {
    this.items.push(item);
    this.keys.push(key);
    let i = this.items.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.keys[p] <= this.keys[i]) break;
      [this.items[p], this.items[i]] = [this.items[i], this.items[p]];
      [this.keys[p], this.keys[i]] = [this.keys[i], this.keys[p]];
      i = p;
    }
  }
  pop() {
    const top = this.items[0];
    const lastItem = this.items.pop();
    const lastKey = this.keys.pop();
    if (this.items.length) {
      this.items[0] = lastItem;
      this.keys[0] = lastKey;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < this.items.length && this.keys[l] < this.keys[m]) m = l;
        if (r < this.items.length && this.keys[r] < this.keys[m]) m = r;
        if (m === i) break;
        [this.items[m], this.items[i]] = [this.items[i], this.items[m]];
        [this.keys[m], this.keys[i]] = [this.keys[i], this.keys[m]];
        i = m;
      }
    }
    return top;
  }
}

export function planRoute(terrain, start, end, cls, opts = {}) {
  const b = terrain.bounds;
  const span = Math.max(b.maxX - b.minX, b.maxZ - b.minZ);
  const cell = opts.cell ?? clamp(span / 190, 2.5, 5.5);
  const margin = (cls.width / 2) + (opts.clearancePad ?? 1.0);
  const nx = Math.floor((b.maxX - b.minX) / cell) + 1;
  const nz = Math.floor((b.maxZ - b.minZ) / cell) + 1;
  const slopeWeight = opts.slopeWeight ?? 26;
  const clearanceWeight = opts.clearanceWeight ?? 1.0;
  const turnWeight = opts.turnWeight ?? 0.55;
  const maxGrade = cls.maxGrade;
  const hCache = new Float32Array(nx * nz);
  const hValid = new Uint8Array(nx * nz);

  const toIndex = (ix, iz) => iz * nx + ix;
  const toGrid = (p) => ({
    ix: clamp(Math.round((p.x - b.minX) / cell), 0, nx - 1),
    iz: clamp(Math.round((p.y - b.minZ) / cell), 0, nz - 1),
  });
  const toPlan = (ix, iz) => ({ x: b.minX + ix * cell, y: b.minZ + iz * cell });
  const height = (ix, iz) => {
    const k = toIndex(ix, iz);
    if (!hValid[k]) {
      hValid[k] = 1;
      hCache[k] = terrain.heightAt(b.minX + ix * cell, b.minZ + iz * cell);
    }
    return hCache[k];
  };

  const forbiddenAt = (ix, iz, pad = margin) => {
    const p = toPlan(ix, iz);
    return terrain.forbidden({ x: p.x, y: p.y }, pad);
  };
  // Cache obstacle clearance per cell for speed.
  const clearCache = new Map();
  const clearance = (ix, iz) => {
    const k = toIndex(ix, iz);
    let v = clearCache.get(k);
    if (v === undefined) {
      const p = toPlan(ix, iz);
      v = terrain.obstacleClearance({ x: p.x, y: p.y });
      clearCache.set(k, v);
    }
    return v;
  };

  const passable = (ix, iz) => {
    if (ix < 0 || iz < 0 || ix >= nx || iz >= nz) return false;
    return forbiddenAt(ix, iz).length === 0;
  };

  // Terrain slope penalty for a step.
  const stepCost = (a, b2, diag) => {
    const len = cell * diag;
    const dh = Math.abs(height(b2[0], b2[1]) - height(a[0], a[1]));
    const grade = dh / len;
    let cost = len;
    if (grade > maxGrade * 1.6) return Infinity;
    if (grade > maxGrade * 0.75) {
      const t = (grade - maxGrade * 0.75) / (maxGrade * 0.85);
      cost += len * t * t * slopeWeight * 0.12;
    }
    const cl = clearance(b2[0], b2[1]);
    const need = cls.width / 2 + 0.6;
    if (cl < need) {
      const t = (need - cl) / need;
      cost += len * t * t * clearanceWeight * 10;
    } else if (cl < need + 6) {
      cost += len * (1 - (cl - need) / 6) * clearanceWeight * 0.8;
    }
    return cost;
  };

  const lineOfSight = (a, b2) => {
    const pa = toPlan(a[0], a[1]);
    const pb = toPlan(b2[0], b2[1]);
    const L = dist(pa, pb);
    const steps = Math.max(2, Math.ceil(L / (cell * 0.55)));
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      const x = lerp(pa.x, pb.x, t);
      const z = lerp(pa.y, pb.y, t);
      if (!terrain.inBounds(x, z, 1.5)) return false;
      if (terrain.forbidden({ x, y: z }, margin).length) return false;
      const ix = clamp(Math.round((x - b.minX) / cell), 0, nx - 1);
      const iz = clamp(Math.round((z - b.minZ) / cell), 0, nz - 1);
      if (forbiddenAt(ix, iz).length) return false;
    }
    return true;
  };

  const startG = toGrid(start);
  const endG = toGrid(end);
  let sIx = startG.ix;
  let sIz = startG.iz;
  let eIx = endG.ix;
  let eIz = endG.iz;
  const startFree = passable(sIx, sIz);
  const endFree = passable(eIx, eIz);
  if (!startFree || !endFree) {
    // Find the nearest free cell to each endpoint.
    const nearestFree = (ix, iz) => {
      for (let r = 1; r < 14; r++) {
        for (let di = -r; di <= r; di++) {
          for (let dj = -r; dj <= r; dj++) {
            if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;
            if (passable(ix + di, iz + dj)) return { ix: ix + di, iz: iz + dj };
          }
        }
      }
      return { ix, iz };
    };
    if (!startFree) Object.assign(startG, nearestFree(sIx, sIz));
    if (!endFree) Object.assign(endG, nearestFree(eIx, eIz));
    sIx = startG.ix;
    sIz = startG.iz;
    eIx = endG.ix;
    eIz = endG.iz;
  }

  const N = nx * nz;
  const gScore = new Float64Array(N).fill(Infinity);
  const parent = new Int32Array(N).fill(-1);
  const closed = new Uint8Array(N);
  const startIdx = toIndex(sIx, sIz);
  const endIdx = toIndex(eIx, eIz);
  gScore[startIdx] = 0;
  const heap = new MinHeap();
  const heur = (ix, iz) => Math.hypot(ix - eIx, iz - eIz) * cell;
  heap.push(startIdx, heur(sIx, sIz));
  let found = false;

  while (heap.size) {
    const cur = heap.pop();
    if (closed[cur]) continue;
    closed[cur] = 1;
    if (cur === endIdx) {
      found = true;
      break;
    }
    const cx = cur % nx;
    const cz = Math.floor(cur / nx);
    if (closed[cur] && cur !== startIdx) continue;
    for (const [dx, dz, diag] of NEIGHBOURS) {
      const nxp = cx + dx;
      const nzp = cz + dz;
      if (nxp < 0 || nzp < 0 || nxp >= nx || nzp >= nz) continue;
      if (diag > 1 && (!passable(cx + dx, cz) || !passable(cx, cz + dz))) continue;
      if (!passable(nxp, nzp)) continue;
      const nIdx = toIndex(nxp, nzp);
      if (closed[nIdx]) continue;
      let bestG = Infinity;
      let bestParent = -1;
      // Theta* path 1: direct to parent's parent if visible.
      const pIdx = parent[cur];
      if (pIdx >= 0) {
        const px = pIdx % nx;
        const pz = Math.floor(pIdx / nx);
        if (lineOfSight([px, pz], [nxp, nzp])) {
          const c = stepCost([px, pz], [nxp, nzp], Math.hypot(nxp - px, nzp - pz) / 1);
          if (c < Infinity) {
            const g = gScore[pIdx] + c;
            if (g < bestG) {
              bestG = g;
              bestParent = pIdx;
            }
          }
        }
      }
      const c2 = stepCost([cx, cz], [nxp, nzp], diag);
      if (c2 < Infinity) {
        // Straightness: penalise changing direction relative to incoming.
        let turn = 0;
        if (pIdx >= 0) {
          const px = pIdx % nx;
          const pz = Math.floor(pIdx / nx);
          const h1 = Math.atan2(cz - pz, cx - px);
          const h2 = Math.atan2(nzp - cz, nxp - cx);
          let dh = Math.abs(h2 - h1);
          while (dh > Math.PI) dh = Math.abs(dh - Math.PI * 2);
          turn = dh * turnWeight * cell * 0.25;
        }
        const g = gScore[cur] + c2 + turn;
        if (g < bestG) {
          bestG = g;
          bestParent = cur;
        }
      }
      if (bestParent >= 0 && bestG < gScore[nIdx]) {
        gScore[nIdx] = bestG;
        parent[nIdx] = bestParent;
        heap.push(nIdx, bestG + heur(nxp, nzp));
      }
    }
  }

  if (!found) {
    return {
      ok: false,
      points: [start, end],
      reason: 'no-path',
      length: dist(start, end),
    };
  }

  const gridPath = [];
  let cur = endIdx;
  let guard = 0;
  while (cur !== -1 && guard++ < N) {
    gridPath.push([cur % nx, Math.floor(cur / nx)]);
    if (cur === startIdx) break;
    cur = parent[cur];
  }
  gridPath.reverse();
  let pts = gridPath.map(([ix, iz]) => toPlan(ix, iz));
  // Pin exact endpoints.
  pts[0] = { x: start.x, y: start.y };
  pts[pts.length - 1] = { x: end.x, y: end.y };

  pts = smoothPolyline(pts, opts.smoothPasses ?? 2);
  const simplified = simplify(pts, 1e-3);
  const rdp = rdpSimplify(simplified, opts.simplifyTolerance ?? Math.max(1.5, cell * 0.9));

  // Keep interior control points but bound their count.
  let control = rdp.slice(1, -1);
  if (control.length > 14) {
    control = pickEvenly(control, 14);
  }
  const all = [start, ...control, end];
  const stats = {
    length: polylineLength(all),
    maxGrade: 0,
    minClearance: Infinity,
  };
  // Measure along the smoothed control polyline.
  const dense = catmullRomLite(all, 12);
  for (let i = 1; i < dense.length; i++) {
    const ds = dist(dense[i - 1], dense[i]) || 1e-6;
    const grade = Math.abs(terrain.heightAt(dense[i].x, dense[i].y) - terrain.heightAt(dense[i - 1].x, dense[i - 1].y)) / ds;
    stats.maxGrade = Math.max(stats.maxGrade, grade);
    stats.minClearance = Math.min(stats.minClearance, terrain.obstacleClearance(dense[i]));
  }
  return { ok: true, points: all, control, stats };
}

function smoothPolyline(pts, passes) {
  let src = pts;
  for (let p = 0; p < passes; p++) {
    const out = src.map((q) => ({ ...q }));
    for (let i = 1; i < src.length - 1; i++) {
      out[i] = {
        x: src[i - 1].x * 0.25 + src[i].x * 0.5 + src[i + 1].x * 0.25,
        y: src[i - 1].y * 0.25 + src[i].y * 0.5 + src[i + 1].y * 0.25,
      };
    }
    src = out;
  }
  return src;
}

function rdpSimplify(points, tolerance) {
  if (points.length <= 2) return points;
  let maxD = 0;
  let idx = 0;
  const a = points[0];
  const b = points[points.length - 1];
  for (let i = 1; i < points.length - 1; i++) {
    const d = perpDistance(points[i], a, b);
    if (d > maxD) {
      maxD = d;
      idx = i;
    }
  }
  if (maxD > tolerance) {
    const left = rdpSimplify(points.slice(0, idx + 1), tolerance);
    const right = rdpSimplify(points.slice(idx), tolerance);
    return left.slice(0, -1).concat(right);
  }
  return [a, b];
}

function perpDistance(p, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const l = Math.hypot(dx, dy) || 1e-9;
  return Math.abs((p.x - a.x) * dy - (p.y - a.y) * dx) / l;
}

function pickEvenly(arr, count) {
  if (arr.length <= count) return arr;
  const out = [];
  for (let i = 0; i < count; i++) {
    const t = (i * (arr.length - 1)) / (count - 1);
    out.push(arr[Math.round(t)]);
  }
  return out;
}

function polylineLength(points) {
  let L = 0;
  for (let i = 1; i < points.length; i++) L += dist(points[i - 1], points[i]);
  return L;
}

function catmullRomLite(points, perSpan) {
  if (points.length < 2) return points.slice();
  const out = [];
  const wrap = (i) => {
    if (i < 0) return { x: 2 * points[0].x - points[1].x, y: 2 * points[0].y - points[1].y };
    if (i >= points.length) {
      const n = points.length;
      return { x: 2 * points[n - 1].x - points[n - 2].x, y: 2 * points[n - 1].y - points[n - 2].y };
    }
    return points[i];
  };
  for (let i = 0; i < points.length - 1; i++) {
    const p0 = wrap(i - 1);
    const p1 = points[i];
    const p2 = points[i + 1];
    const p3 = wrap(i + 2);
    for (let s = 0; s < perSpan; s++) {
      const t = s / perSpan;
      const t2 = t * t;
      const t3 = t2 * t;
      out.push({
        x: 0.5 * ((2 * p1.x) + (-p0.x + p2.x) * t + (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 + (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3),
        y: 0.5 * ((2 * p1.y) + (-p0.y + p2.y) * t + (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 + (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3),
      });
    }
  }
  out.push({ ...points[points.length - 1] });
  return out;
}
