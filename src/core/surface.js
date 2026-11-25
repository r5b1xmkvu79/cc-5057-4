// Pavement surface construction: triangulate plan polygons, sample heights from
// a known profile, and provide a collidable triangle index used by both the
// visible mesh and the drive mode.

import earcut from 'earcut';
import { clamp, dist, lerp, polygonArea, polygonPerimeter } from './math.js';

export function resampleRing(ring, maxEdge = 2.5) {
  const out = [];
  const n = ring.length;
  for (let i = 0; i < n; i++) {
    const a = ring[i];
    const b = ring[(i + 1) % n];
    const l = Math.hypot(b.x - a.x, b.y - a.y);
    const steps = Math.max(1, Math.ceil(l / maxEdge));
    for (let s = 0; s < steps; s++) out.push({ x: lerp(a.x, b.x, s / steps), y: lerp(a.y, b.y, s / steps) });
  }
  return out;
}

export function ensureCCW(poly) {
  return polygonArea(poly) < 0 ? poly.slice().reverse() : poly.slice();
}

// Build a non-indexed triangle soup (positions & per-vertex z) from a polygon,
// optionally with holes.
export function triangulatePolygon(ring, zFn, maxEdge = 2.5, holes = []) {
  let outer = ensureCCW(ring);
  if (outer.length < 3) return { positions: [], triangles: [] };
  outer = resampleRing(outer, maxEdge);
  const flat = [];
  const holeIndices = [];
  for (const p of outer) flat.push(p.x, p.y);
  const holeRings = [];
  for (const hole of holes) {
    if (!hole || hole.length < 3) continue;
    const h = ensureCCW(hole).slice().reverse(); // holes are clockwise
    let hres = resampleRing(h, maxEdge);
    // Drop a duplicate closing vertex.
    if (hres.length > 1 && Math.hypot(hres[0].x - hres[hres.length - 1].x, hres[0].y - hres[hres.length - 1].y) < 1e-6) {
      hres = hres.slice(0, -1);
    }
    holeIndices.push(flat.length / 2);
    holeRings.push(hres);
    for (const p of hres) flat.push(p.x, p.y);
  }
  const tris = holeIndices.length ? earcut(flat, holeIndices, 2) : earcut(flat);
  if (!tris.length) return { positions: [], triangles: [] };
  const allPts = outer.concat(...holeRings);
  const verts = [];
  const triangles = [];
  for (let i = 0; i < tris.length; i += 3) {
    const a = allPts[tris[i]];
    const b = allPts[tris[i + 1]];
    const c = allPts[tris[i + 2]];
    if (!a || !b || !c) continue;
    const area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    const order = area >= 0 ? [a, b, c] : [a, c, b];
    const tri = [];
    for (const p of order) tri.push({ x: p.x, y: p.y, z: zFn(p.x, p.y) });
    triangles.push(tri);
    for (const v of tri) verts.push(v.x, v.z, v.y);
  }
  return { positions: new Float32Array(verts), triangles };
}

// Build a fan around `center` for star-shaped junction rings. Each boundary
// vertex supplies its own height; the centre supplies zJ.
export function fanTriangulate(ring, zBoundary, center, zCenter) {
  let outer = ring;
  // Fan requires matching winding; try both directions and keep the valid one.
  const tryFan = (pts) => {
    const tris = [];
    let score = 0;
    for (let i = 0; i < pts.length; i++) {
      const a = pts[i];
      const b = pts[(i + 1) % pts.length];
      const area = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
      if (area <= 0) score++;
      const za = zBoundary(a);
      const zb = zBoundary(b);
      const tri = [
        { x: center.x, y: center.y, z: zCenter },
        { x: a.x, y: a.y, z: za },
        { x: b.x, y: b.y, z: zb },
      ];
      tris.push(tri);
    }
    return { tris, bad: score };
  };
  let best = tryFan(outer);
  if (best.bad > 0) {
    const alt = tryFan(outer.slice().reverse());
    if (alt.bad < best.bad) best = alt;
  }
  if (best.bad > Math.max(1, Math.floor(outer.length * 0.08))) {
    // Not star shaped: fall back to earcut.
    const poly = ensureCCW(outer);
    const res = triangulatePolygon(poly, zBoundary, 2.5);
    return res.triangles;
  }
  return best.tris;
}

export function trianglesToPositions(triangles) {
  const arr = new Float32Array(triangles.length * 9);
  let o = 0;
  for (const tri of triangles) {
    for (const v of tri) {
      arr[o++] = v.x;
      arr[o++] = v.z;
      arr[o++] = v.y;
    }
  }
  return arr;
}

export function triangleCentroid(tri) {
  return {
    x: (tri[0].x + tri[1].x + tri[2].x) / 3,
    y: (tri[0].y + tri[1].y + tri[2].y) / 3,
    z: (tri[0].z + tri[1].z + tri[2].z) / 3,
  };
}

// Signed plan area of a triangle (positive => CCW => up-facing).
export function triangleUpward(tri) {
  return (tri[1].x - tri[0].x) * (tri[2].y - tri[0].y) - (tri[2].x - tri[0].x) * (tri[1].y - tri[0].y);
}

function pointInTriangle2D(px, py, a, b, c) {
  const d1 = (px - b.x) * (a.y - b.y) - (a.x - b.x) * (py - b.y);
  const d2 = (px - c.x) * (b.y - c.y) - (b.x - c.x) * (py - c.y);
  const d3 = (px - a.x) * (c.y - a.y) - (c.x - a.x) * (py - a.y);
  const neg = d1 < 0 || d2 < 0 || d3 < 0;
  const pos = d1 > 0 || d2 > 0 || d3 > 0;
  return !(neg && pos);
}

// Uniform-grid triangle index for height queries and collision.
export class SurfaceIndex {
  constructor(triangles, cell = 8) {
    this.triangles = triangles;
    this.cell = cell;
    this.grid = new Map();
    this.minX = Infinity;
    this.maxX = -Infinity;
    this.minY = Infinity;
    this.maxY = -Infinity;
    for (let ti = 0; ti < triangles.length; ti++) {
      const t = triangles[ti];
      let x0 = Infinity;
      let x1 = -Infinity;
      let y0 = Infinity;
      let y1 = -Infinity;
      for (const v of t) {
        x0 = Math.min(x0, v.x);
        x1 = Math.max(x1, v.x);
        y0 = Math.min(y0, v.y);
        y1 = Math.max(y1, v.y);
      }
      this.minX = Math.min(this.minX, x0);
      this.maxX = Math.max(this.maxX, x1);
      this.minY = Math.min(this.minY, y0);
      this.maxY = Math.max(this.maxY, y1);
      const i0 = Math.floor(x0 / cell);
      const i1 = Math.floor(x1 / cell);
      const j0 = Math.floor(y0 / cell);
      const j1 = Math.floor(y1 / cell);
      for (let i = i0; i <= i1; i++) {
        for (let j = j0; j <= j1; j++) {
          const k = `${i},${j}`;
          let arr = this.grid.get(k);
          if (!arr) this.grid.set(k, (arr = []));
          arr.push(ti);
        }
      }
    }
  }

  candidates(x, y) {
    const arr = this.grid.get(`${Math.floor(x / this.cell)},${Math.floor(y / this.cell)}`);
    return arr || [];
  }

  // Highest pavement height at (x,y) or null if no pavement.
  heightAt(x, y) {
    let best = null;
    const cand = this.candidates(x, y);
    for (const ti of cand) {
      const t = this.triangles[ti];
      if (!pointInTriangle2D(x, y, t[0], t[1], t[2])) continue;
      const z = baryHeight(x, y, t[0], t[1], t[2]);
      if (best === null || z > best) best = z;
    }
    return best;
  }

  // Highest pavement at or nearest within radius (for wheels at edges).
  heightNear(x, y, radius = 1.2) {
    const direct = this.heightAt(x, y);
    if (direct !== null) return direct;
    let best = null;
    let bestD = Infinity;
    const r = Math.ceil(radius / this.cell);
    const ci = Math.floor(x / this.cell);
    const cj = Math.floor(y / this.cell);
    for (let i = ci - r; i <= ci + r; i++) {
      for (let j = cj - r; j <= cj + r; j++) {
        const arr = this.grid.get(`${i},${j}`);
        if (!arr) continue;
        for (const ti of arr) {
          const t = this.triangles[ti];
          const cx = (t[0].x + t[1].x + t[2].x) / 3;
          const cy = (t[0].y + t[1].y + t[2].y) / 3;
          const d = Math.hypot(cx - x, cy - y);
          if (d < bestD && d <= radius) {
            bestD = d;
            best = (t[0].z + t[1].z + t[2].z) / 3;
          }
        }
      }
    }
    return best;
  }

  hasPavement(x, y) {
    return this.heightAt(x, y) !== null;
  }

  // Segment crossings against pavement edges (for junction seam diagnostics).
  containsPoint(x, y) {
    return this.heightAt(x, y) !== null;
  }
}

export function baryHeight(px, py, a, b, c) {
  const det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
  if (Math.abs(det) < 1e-12) return (a.z + b.z + c.z) / 3;
  const l1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / det;
  const l2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / det;
  const l3 = 1 - l1 - l2;
  return l1 * a.z + l2 * b.z + l3 * c.z;
}

// Project a point onto a sampled centerline; returns station and interpolated z.
export function projectToCenterline(samples, x, y) {
  let best = null;
  let bestD = Infinity;
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1];
    const b = samples[i];
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const l2 = dx * dx + dy * dy;
    if (l2 < 1e-12) continue;
    let t = ((x - a.x) * dx + (y - a.y) * dy) / l2;
    t = clamp(t, 0, 1);
    const px = a.x + dx * t;
    const py = a.y + dy * t;
    const d = (px - x) * (px - x) + (py - y) * (py - y);
    if (d < bestD) {
      bestD = d;
      best = { i, t, s: lerp(a.s, b.s, t), z: lerp(a.z, b.z, t), lateral: Math.sqrt(d) };
    }
  }
  return best;
}
