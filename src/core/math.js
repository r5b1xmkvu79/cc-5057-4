// Small vector / interpolation / rng helpers shared by the kernel.

export const EPS = 1e-9;
export const TAU = Math.PI * 2;

export function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function smoothstep(t) {
  t = clamp(t, 0, 1);
  return t * t * (3 - 2 * t);
}

export function smootherstep(t) {
  t = clamp(t, 0, 1);
  return t * t * t * (t * (t * 6 - 15) + 10);
}

export function v2(x, y) {
  return { x, y };
}

export function add(a, b) {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub(a, b) {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function mul(a, s) {
  return { x: a.x * s, y: a.y * s };
}

export function dot(a, b) {
  return a.x * b.x + a.y * b.y;
}

export function cross(a, b) {
  return a.x * b.y - a.y * b.x;
}

export function len(a) {
  return Math.hypot(a.x, a.y);
}

export function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function dist2(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return dx * dx + dy * dy;
}

export function norm(a) {
  const l = Math.hypot(a.x, a.y);
  return l < EPS ? { x: 0, y: 0 } : { x: a.x / l, y: a.y / l };
}

export function perp(a) {
  return { x: -a.y, y: a.x };
}

export function lerpV2(a, b, t) {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
}

// Remove duplicate points and co-linear interior points.
export function simplify(points, tol = 1e-7) {
  const out = [];
  for (const p of points) {
    if (!out.length || dist(out[out.length - 1], p) > tol) out.push(p);
  }
  if (out.length < 3) return out.slice();
  let changed = true;
  while (changed && out.length > 2) {
    changed = false;
    for (let i = 0; i < out.length; i++) {
      const a = out[(i - 1 + out.length) % out.length];
      const b = out[i];
      const c = out[(i + 1) % out.length];
      const ab = sub(b, a);
      const bc = sub(c, b);
      const area = Math.abs(cross(ab, bc));
      const base = Math.max(len(ab) * len(bc), EPS);
      if (area / base < tol && dot(ab, bc) > 0) {
        out.splice(i, 1);
        changed = true;
        break;
      }
    }
  }
  return out;
}

// Signed area. Positive for counter-clockwise in a Y-up 2D frame.
export function polygonArea(points) {
  let a = 0;
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const q = points[(i + 1) % points.length];
    a += p.x * q.y - q.x * p.y;
  }
  return a * 0.5;
}

export function polygonPerimeter(points) {
  let s = 0;
  for (let i = 0; i < points.length; i++) {
    s += dist(points[i], points[(i + 1) % points.length]);
  }
  return s;
}

export function pointInPolygon(pt, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i];
    const b = poly[j];
    if (a.y > pt.y !== b.y > pt.y && pt.x < ((b.x - a.x) * (pt.y - a.y)) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}

export function closestPointOnSegment(p, a, b) {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  if (l2 < EPS) return { point: { ...a }, t: 0 };
  const t = clamp(dot(sub(p, a), ab) / l2, 0, 1);
  return { point: { x: a.x + ab.x * t, y: a.y + ab.y * t }, t };
}

export function distancePointSegment(p, a, b) {
  return dist(p, closestPointOnSegment(p, a, b).point);
}

export function segmentIntersection(p1, p2, p3, p4, includeEnds = true) {
  const d1 = sub(p2, p1);
  const d2 = sub(p4, p3);
  const den = cross(d1, d2);
  if (Math.abs(den) < 1e-12) return null;
  const t = cross(sub(p3, p1), d2) / den;
  const u = cross(sub(p3, p1), d1) / den;
  const m = includeEnds ? 0 : 1e-7;
  if (t < -m || t > 1 + m || u < -m || u > 1 + m) return null;
  return { x: p1.x + d1.x * t, y: p1.y + d1.y * t, t, u };
}

// open=false closes the ring (polyline vs polyline helper).
export function polylineSelfIntersections(points, closed = false) {
  const hits = [];
  const n = points.length;
  const last = closed ? n : n - 1;
  for (let i = 0; i < last; i++) {
    const a = points[i];
    const b = points[(i + 1) % n];
    for (let j = i + 2; j < last; j++) {
      if (closed && i === 0 && j === last - 1) continue;
      const c = points[j];
      const d = points[(j + 1) % n];
      const hit = segmentIntersection(a, b, c, d, false);
      if (hit) hits.push({ i, j, point: hit });
    }
  }
  return hits;
}

// Deterministic PRNG (mulberry32).
export function makeRng(seed) {
  let a = seed >>> 0;
  return function rng() {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Catmull-Rom through control points, sampled at `perSpan` per segment.
export function catmullRom(points, perSpan = 16, alpha = 0.5) {
  const pts = points.map((p) => ({ x: p.x, y: p.y }));
  if (pts.length < 2) return pts;
  if (pts.length === 2) {
    const out = [];
    for (let i = 0; i <= perSpan; i++) out.push(lerpV2(pts[0], pts[1], i / perSpan));
    return out;
  }
  const out = [];
  const wrap = (i) => {
    if (i < 0) return { x: 2 * pts[0].x - pts[1].x, y: 2 * pts[0].y - pts[1].y };
    if (i >= pts.length) {
      const n = pts.length;
      return { x: 2 * pts[n - 1].x - pts[n - 2].x, y: 2 * pts[n - 1].y - pts[n - 2].y };
    }
    return pts[i];
  };
  for (let i = 0; i < pts.length - 1; i++) {
    const p0 = wrap(i - 1);
    const p1 = pts[i];
    const p2 = pts[i + 1];
    const p3 = wrap(i + 2);
    for (let s = 0; s < perSpan; s++) {
      const t = s / perSpan;
      out.push(catmullRomPoint(p0, p1, p2, p3, t, alpha));
    }
  }
  out.push({ ...pts[pts.length - 1] });
  return out;
}

function catmullRomPoint(p0, p1, p2, p3, t, alpha) {
  const t0 = 0;
  const t1 = t0 + Math.pow(dist(p0, p1), alpha);
  const t2 = t1 + Math.pow(dist(p1, p2), alpha);
  const t3 = t2 + Math.pow(dist(p2, p3), alpha);
  const tt = lerp(t1, t2, t);
  const e1 = Math.max(t1 - t0, 1e-6);
  const e2 = Math.max(t2 - t1, 1e-6);
  const e3 = Math.max(t3 - t2, 1e-6);
  const a1 = lerpV2(p0, p1, (tt - t0) / e1);
  const a2 = lerpV2(p1, p2, (tt - t1) / e2);
  const a3 = lerpV2(p2, p3, (tt - t2) / e3);
  const b1 = lerpV2(a1, a2, (tt - t0) / Math.max(t2 - t0, 1e-6));
  const b2 = lerpV2(a2, a3, (tt - t1) / Math.max(t3 - t1, 1e-6));
  return lerpV2(b1, b2, (tt - t1) / Math.max(t2 - t1, 1e-6));
}

export function resamplePolyline(points, spacing) {
  if (points.length < 2) return points.map((p) => ({ ...p }));
  const out = [{ ...points[0] }];
  let carry = 0;
  for (let i = 0; i < points.length - 1; i++) {
    const a = points[i];
    const b = points[i + 1];
    const segLen = dist(a, b);
    if (segLen < EPS) continue;
    let travelled = spacing - carry;
    while (travelled <= segLen + EPS) {
      const t = travelled / segLen;
      out.push(lerpV2(a, b, t));
      travelled += spacing;
    }
    carry = segLen - (travelled - spacing);
  }
  const last = points[points.length - 1];
  if (dist(out[out.length - 1], last) > spacing * 0.01) out.push({ ...last });
  return out;
}

export function headingOf(a, b) {
  return Math.atan2(b.y - a.y, b.x - a.x);
}

export function angleDelta(a, b) {
  let d = b - a;
  while (d > Math.PI) d -= TAU;
  while (d < -Math.PI) d += TAU;
  return d;
}

export function hashString(str) {
  let h = 2166136261 >>> 0;
  for (let i = 0; i < str.length; i++) {
    h ^= str.charCodeAt(i);
    h = Math.imul(h, 16777619);
  }
  return h >>> 0;
}

export function roundTo(v, step) {
  return Math.round(v / step) * step;
}

export function fmt(v, digits = 2) {
  return Number(v.toFixed(digits));
}
