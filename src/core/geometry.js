// Pavement geometry: ribbons, junctions, markings and turning trajectories.
// All plan coordinates are {x, y}; world mapping is (x, z, y).

import {
  angleDelta,
  clamp,
  dist,
  lerp,
  norm,
  perp,
  polygonArea,
  polylineSelfIntersections,
  smoothstep,
  smootherstep,
  sub,
} from './math.js';
import { sampleAt } from './road.js';
import { ensureCCW, triangleUpward } from './surface.js';

const MARK_COLOR = 0.95;

// ---------------------------------------------------------------------------
// Road ribbons
// ---------------------------------------------------------------------------
export function buildRoadRibbon(samples, startStation, endStation, cls, connId) {
  const stations = clipStations(samples, startStation, endStation);
  if (stations.length < 2) {
    return emptyRibbon(connId);
  }
  const left = [];
  const right = [];
  const centers = [];
  for (const q of stations) {
    const l = { x: q.x + q.nx * q.halfLeft, y: q.y + q.ny * q.halfLeft, z: q.z };
    const r = { x: q.x - q.nx * q.halfRight, y: q.y - q.ny * q.halfRight, z: q.z };
    left.push(l);
    right.push(r);
    centers.push({ x: q.x, y: q.y, z: q.z, s: q.s, heading: q.heading, halfLeft: q.halfLeft, halfRight: q.halfRight });
  }

  const pavementTris = [];
  const shoulderTris = [];
  const skirtTris = [];
  const markingTris = [];

  for (let i = 0; i < stations.length - 1; i++) {
    const L0 = left[i];
    const L1 = left[i + 1];
    const R0 = right[i];
    const R1 = right[i + 1];
    pushQuad(pavementTris, L0, R0, L1, R1);

    // Shoulder: a narrow strip outside the paved edge, dropped slightly.
    const sw = cls.shoulder;
    if (sw > 0.02) {
      const drop = 0.08;
      const la = { x: L0.x + (L0.x - R0.x) * 0, y: L0.y, z: L0.z };
      // Use station normals for the outward direction.
      const q0 = stations[i];
      const q1 = stations[i + 1];
      const Ls0 = { x: L0.x + q0.nx * sw, y: L0.y + q0.ny * sw, z: L0.z - drop };
      const Ls1 = { x: L1.x + q1.nx * sw, y: L1.y + q1.ny * sw, z: L1.z - drop };
      const Rs0 = { x: R0.x - q0.nx * sw, y: R0.y - q0.ny * sw, z: R0.z - drop };
      const Rs1 = { x: R1.x - q1.nx * sw, y: R1.y - q1.ny * sw, z: R1.z - drop };
      pushQuad(shoulderTris, Ls0, L0, Ls1, L1);
      pushQuad(shoulderTris, R0, Rs0, R1, Rs1);
      // Vertical skirts conceal the graded-terrain seam.
      pushWall(skirtTris, Ls0, Ls1, -0.5);
      pushWall(skirtTris, Rs0, Rs1, -0.5);
    }
  }

  // Dashed lane marking (single centre line) for vehicular classes.
  if (cls.width >= 5.5) {
    const dash = 4.5;
    const gap = 5.5;
    let phase = 0;
    for (let i = 0; i < stations.length - 1; i++) {
      const a = stations[i];
      const b = stations[i + 1];
      const segLen = b.s - a.s;
      if (segLen <= 1e-6) continue;
      const midHalf = 0.075;
      const start = phase;
      phase = (phase + segLen) % (dash + gap);
      const from = Math.max(0, start);
      const t0 = Math.min(1, from / segLen);
      const t1 = Math.min(1, (from + Math.min(dash, segLen)) / segLen);
      if (t1 <= t0) continue;
      const q0 = sampleAt(stations, a.s + segLen * t0);
      const q1 = sampleAt(stations, a.s + segLen * t1);
      if (!q0 || !q1) continue;
      const la = { x: q0.x + q0.nx * midHalf + 0, y: q0.y + q0.ny * midHalf, z: q0.z + 0.025 };
      const ra = { x: q0.x - q0.nx * midHalf, y: q0.y - q0.ny * midHalf, z: q0.z + 0.025 };
      const lb = { x: q1.x + q1.nx * midHalf, y: q1.y + q1.ny * midHalf, z: q1.z + 0.025 };
      const rb = { x: q1.x - q1.nx * midHalf, y: q1.y - q1.ny * midHalf, z: q1.z + 0.025 };
      pushQuad(markingTris, la, ra, lb, rb);
    }
  }

  const ring = makeRibbonRing(left, right);
  return {
    id: connId,
    part: 'road',
    startStation,
    endStation,
    stations,
    left,
    right,
    centers,
    pavementTriangles: pavementTris,
    shoulderTriangles: shoulderTris,
    skirtTriangles: skirtTris,
    markingTriangles: markingTris,
    colliderTriangles: pavementTris.concat(shoulderTris),
    ring,
  };
}

export function clipStations(samples, startStation, endStation) {
  const out = [];
  const first = sampleAt(samples, startStation);
  const last = sampleAt(samples, endStation);
  if (!first || !last) return out;
  out.push(first);
  for (const q of samples) {
    if (q.s > startStation + 1e-6 && q.s < endStation - 1e-6) out.push({ ...q });
  }
  out.push(last);
  return out;
}

function makeRibbonRing(left, right) {
  const ring = [];
  for (const p of left) ring.push({ x: p.x, y: p.y });
  for (let i = right.length - 1; i >= 0; i--) ring.push({ x: right[i].x, y: right[i].y });
  return ring;
}

function pushQuad(tris, a, b, c, d) {
  // a,b are one cross-section (left,right); c,d the next. Upward winding.
  if (triangleUpward([a, b, c]) > 0) tris.push([a, b, c]);
  else tris.push([a, c, b]);
  if (triangleUpward([c, b, d]) > 0) tris.push([c, b, d]);
  else tris.push([c, d, b]);
}

function pushWall(tris, a, b, drop) {
  const a2 = { x: a.x, y: a.y, z: a.z + drop };
  const b2 = { x: b.x, y: b.y, z: b.z + drop };
  tris.push([a, b, a2], [b, b2, a2]);
}

function emptyRibbon(id) {
  return {
    id,
    part: 'road',
    startStation: 0,
    endStation: 0,
    stations: [],
    left: [],
    right: [],
    centers: [],
    pavementTriangles: [],
    shoulderTriangles: [],
    skirtTriangles: [],
    markingTriangles: [],
    colliderTriangles: [],
    ring: [],
  };
}

// ---------------------------------------------------------------------------
// Junction topology
// ---------------------------------------------------------------------------
// A port describes where an arm's pavement meets the junction.
export function makePort(conn, end, station, sample) {
  // `outward` points from the junction out along the arm (away from the node).
  const outward = end === 'a'
    ? { x: sample.tx, y: sample.ty }
    : { x: -sample.tx, y: -sample.ty };
  const n = perp(outward);
  return {
    connectionId: conn.id,
    end,
    station,
    x: sample.x,
    y: sample.y,
    z: sample.z,
    tx: sample.tx,
    ty: sample.ty,
    heading: sample.heading,
    ux: outward.x,
    uy: outward.y,
    nx: n.x,
    ny: n.y,
    halfLeft: sample.halfLeft,
    halfRight: sample.halfRight,
    width: sample.halfLeft + sample.halfRight,
    cls: conn.classId,
    leftEdge: { x: sample.x + n.x * sample.halfLeft, y: sample.y + n.y * sample.halfLeft },
    rightEdge: { x: sample.x - n.x * sample.halfRight, y: sample.y - n.y * sample.halfRight },
  };
}

// The junction pavement is the union of a central pad with the untrimmed arm
// ribbons. Heights blend smoothly from each port cross-section to the pad.
export function junctionPadPolygon(center, radius, sides = 40) {
  const pts = [];
  for (let i = 0; i < sides; i++) {
    const a = (i / sides) * Math.PI * 2;
    pts.push({ x: center.x + Math.cos(a) * radius, y: center.y + Math.sin(a) * radius });
  }
  return pts;
}

// Junction height field: a least-squares plane through the port cross-section
// heights, clamped to the port height range. A plane keeps cross-slope low and
// meets every arm smoothly; inverse-distance blending produced local bowls.
export function makeJunctionHeightField(ports, center) {
  const pts = [];
  for (const p of ports) {
    // Weight the arm's own cross-section heavily: both edges at the arm height.
    pts.push({ x: p.leftEdge.x, y: p.leftEdge.y, z: p.z, w: 2 });
    pts.push({ x: p.rightEdge.x, y: p.rightEdge.y, z: p.z, w: 2 });
    pts.push({ x: p.x, y: p.y, z: p.z, w: 1 });
  }
  // The pad surface is defined by how its arms arrive. Seeding the terrain
  // height at the node would bow the plane away from all three ports when the
  // ground there is not the arms' average; keep it as a weak tie-breaker only.
  if (ports.length < 3) pts.push({ x: center.x, y: center.y, z: center.z, w: 1 });
  const zs = pts.map((p) => p.z);
  const zMin = Math.min(...zs);
  const zMax = Math.max(...zs);

  // Weighted least squares z = a + b*(x-cx) + c*(y-cy).
  let Sw = 0, Sx = 0, Sy = 0, Sxx = 0, Sxy = 0, Syy = 0, Sz = 0, Sxz = 0, Syz = 0;
  for (const p of pts) {
    const x = p.x - center.x;
    const y = p.y - center.y;
    Sw += p.w;
    Sx += p.w * x;
    Sy += p.w * y;
    Sxx += p.w * x * x;
    Sxy += p.w * x * y;
    Syy += p.w * y * y;
    Sz += p.w * p.z;
    Sxz += p.w * x * p.z;
    Syz += p.w * y * p.z;
  }
  // Solve 3x3 normal equations.
  const M = [
    [Sw, Sx, Sy],
    [Sx, Sxx, Sxy],
    [Sy, Sxy, Syy],
  ];
  const rhs = [Sz, Sxz, Syz];
  const det = det3(M);
  let a = center.z;
  let b = 0;
  let c = 0;
  if (Math.abs(det) > 1e-6) {
    const sol = solve3(M, rhs);
    a = sol[0];
    b = sol[1];
    c = sol[2];
  }
  // Limit the plane's grade so the junction is not a ramp.
  const grade = Math.hypot(b, c);
  const maxGrade = 0.1;
  if (grade > maxGrade) {
    b *= maxGrade / grade;
    c *= maxGrade / grade;
  }
  const planeZ = (x, y) => clamp(a + b * (x - center.x) + c * (y - center.y), zMin - 0.05, zMax + 0.05);

  // Near each port mouth, pull toward that arm's elevation so the pad and
  // ribbon meet without a step. The correction is the port's residual against
  // the plane, blended with normalized radial weights. Using a weighted sum
  // (not a max) keeps the field continuous where two port windows overlap;
  // max-based selection produced cliffs.
  const blends = ports.map((p) => ({
    p,
    cx: p.x,
    cy: p.y,
    ix: -p.ux,
    iy: -p.uy,
    nx: p.nx,
    ny: p.ny,
    half: Math.max(p.halfLeft, p.halfRight) + 1.6,
    reach: 8.0,
    residual: p.z - (a + b * (p.x - center.x) + c * (p.y - center.y)),
  }));
  return (x, y) => {
    const zPlane = planeZ(x, y);
    let sumW = 0;
    let sumR = 0;
    for (const bl of blends) {
      const dx = x - bl.cx;
      const dy = y - bl.cy;
      const inward = dx * bl.ix + dy * bl.iy; // positive inside the pad
      const lateral = Math.abs(dx * bl.nx + dy * bl.ny);
      if (lateral > bl.half || inward < -0.4) continue;
      const wIn = inward <= 0 ? 1 : smoothstep(1 - inward / bl.reach);
      // Lateral falloff spans the whole mouth width, so the residual is applied
      // gently rather than as a steep wedge at the pad edge.
      const wLat = 1 - smoothstep(lateral / bl.half);
      const w = wIn * wIn * wLat;
      if (w <= 0) continue;
      sumW += w;
      sumR += w * bl.residual;
    }
    if (sumW <= 1e-9) return zPlane;
    // Normalized weighted average of the port residuals, faded in smoothly as
    // the total window weight grows. Dividing by max(sumW,1) instead would make
    // the correction jump where the summed weight crosses 1.
    const correction = (sumR / sumW) * clamp(sumW, 0, 1);
    return clamp(zPlane + correction, zMin - 0.05, zMax + 0.05);
  };
}

function det3(m) {
  return (
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
  );
}

function solve3(M, rhs) {
  const d = det3(M);
  const rep = (col) =>
    M.map((row, i) => row.map((v, j) => (j === col ? rhs[i] : v)));
  return [det3(rep(0)) / d, det3(rep(1)) / d, det3(rep(2)) / d];
}

export function lineIntersection(p1, p2, p3, p4) {
  const d1 = sub(p2, p1);
  const d2 = sub(p4, p3);
  const den = d1.x * d2.y - d1.y * d2.x;
  if (Math.abs(den) < 1e-9) return null;
  const t = ((p3.x - p1.x) * d2.y - (p3.y - p1.y) * d2.x) / den;
  return { x: p1.x + d1.x * t, y: p1.y + d1.y * t };
}

export function junctionMaxGrade(tris) {
  let max = 0;
  for (const t of tris) {
    // Skip sliver facets. Three nearly collinear vertices on a gently curved
    // field define a wildly tilted plane, but the facet is only centimetres
    // wide; no vehicle sees that grade and it would swamp all real slope.
    const area2 = Math.abs(
      (t[1].x - t[0].x) * (t[2].y - t[0].y) - (t[2].x - t[0].x) * (t[1].y - t[0].y),
    );
    if (area2 < 0.5) continue;
    const longest = Math.max(
      Math.hypot(t[0].x - t[1].x, t[0].y - t[1].y),
      Math.hypot(t[1].x - t[2].x, t[1].y - t[2].y),
      Math.hypot(t[2].x - t[0].x, t[2].y - t[0].y),
    );
    if (area2 / longest < 0.42) continue;
    const a = t[0];
    const b = t[1];
    const c = t[2];
    const ux = b.x - a.x;
    const uy = b.y - a.y;
    const uz = b.z - a.z;
    const vx = c.x - a.x;
    const vy = c.y - a.y;
    const vz = c.z - a.z;
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    const h = Math.hypot(nx, ny);
    if (h < 1e-9 || nz <= 1e-9) continue;
    // Compare the slope across a typical footprint, not an infinitesimal facet.
    max = Math.max(max, h / nz);
  }
  return max;
}

// ---------------------------------------------------------------------------
// Junction turning trajectories
// ---------------------------------------------------------------------------
// Corners of the vehicle body placed with its rear axle on each point of a
// polyline. Used to size junction pads against the real swept envelope.
export function sweepBodyEnvelope(path, vehicle, rearToAxle = vehicle.wheelbase / 2, step = 1.0) {
  const out = [];
  if (path.length < 2) return out;
  const halfLen = vehicle.length / 2;
  const halfW = vehicle.width / 2;
  // Resample so long straight segments are still tested.
  const pts = [path[0]];
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1];
    const b = path[i];
    const d = Math.hypot(b.x - a.x, b.y - a.y);
    const n = Math.max(1, Math.ceil(d / step));
    for (let k = 1; k <= n; k++) {
      pts.push({ x: a.x + (b.x - a.x) * (k / n), y: a.y + (b.y - a.y) * (k / n) });
    }
  }
  for (let i = 0; i < pts.length; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(pts.length - 1, i + 1)];
    const heading = Math.atan2(b.y - a.y, b.x - a.x);
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);
    const cx = pts[i].x + cos * rearToAxle;
    const cy = pts[i].y + sin * rearToAxle;
    out.push(
      { x: cx + cos * halfLen - sin * halfW, y: cy + sin * halfLen + cos * halfW },
      { x: cx + cos * halfLen + sin * halfW, y: cy + sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen + sin * halfW, y: cy - sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen - sin * halfW, y: cy - sin * halfLen + cos * halfW },
    );
  }
  return out;
}

// Build a drivable centreline for a turn from arm A to arm B through the
// junction. The vehicle travels inward along arm A, follows a circular fillet
// tangent to both lane directions through the pad, then exits along arm B.
export function buildTurnPath(portA, samplesA, portB, samplesB, opts = {}) {
  const approach = opts.approach ?? 26;
  const vehicleHalf = opts.vehicleHalfWidth ?? 1.0;
  // Lane centre must leave room for the body inside the paved edge.
  const laneOffset = (p) => {
    const half = p.width / 2;
    const ideal = Math.max(1.2, Math.min(p.width * 0.18, 3.0));
    return Math.min(ideal, Math.max(0.8, half - vehicleHalf - 0.25));
  };
  const offA = laneOffset(portA);
  const offB = laneOffset(portB);
  const step = 1.4;

  const rightOf = (d) => ({ x: d.y, y: -d.x });
  const dIn = { x: -portA.ux, y: -portA.uy };
  const dOut = { x: portB.ux, y: portB.uy };
  const pA = { x: portA.x + rightOf(dIn).x * offA, y: portA.y + rightOf(dIn).y * offA };
  const pB = { x: portB.x + rightOf(dOut).x * offB, y: portB.y + rightOf(dOut).y * offB };

  // Corner where the entry and exit rays meet.
  const corner = lineIntersection(pA, { x: pA.x + dIn.x, y: pA.y + dIn.y }, pB, { x: pB.x - dOut.x, y: pB.y - dOut.y });

  const mid = [];
  let filletRadius = Infinity;
  if (corner) {
    const v1 = { x: pA.x - corner.x, y: pA.y - corner.y };
    const v2 = { x: pB.x - corner.x, y: pB.y - corner.y };
    const l1 = Math.hypot(v1.x, v1.y);
    const l2 = Math.hypot(v2.x, v2.y);
    const u1 = { x: v1.x / (l1 || 1), y: v1.y / (l1 || 1) };
    const u2 = { x: v2.x / (l2 || 1), y: v2.y / (l2 || 1) };
    const beta = Math.acos(clamp(u1.x * u2.x + u1.y * u2.y, -1, 1));
    // Near-collinear arms must be a straight bridge: a fillet of infinite
    // radius degenerates into a tiny loop at the corner.
    if (l1 > 1e-3 && l2 > 1e-3 && beta > 1e-3 && beta < Math.PI - 0.12) {
      let t = Math.min(l1, l2) * 0.92;
      t = Math.min(t, l1 - 0.15, l2 - 0.15);
      if (t > 0.6) {
        const T1 = { x: corner.x + u1.x * t, y: corner.y + u1.y * t };
        const T2 = { x: corner.x + u2.x * t, y: corner.y + u2.y * t };
        const r = t * Math.tan(beta / 2);
        filletRadius = r;
        const bx = u1.x + u2.x;
        const by = u1.y + u2.y;
        const bl = Math.hypot(bx, by) || 1;
        const O = { x: corner.x + (bx / bl) * (t / Math.cos(beta / 2)), y: corner.y + (by / bl) * (t / Math.cos(beta / 2)) };
        const a1 = Math.atan2(T1.y - O.y, T1.x - O.x);
        const a2 = Math.atan2(T2.y - O.y, T2.x - O.x);
        let sweep = a2 - a1;
        while (sweep > Math.PI) sweep -= Math.PI * 2;
        while (sweep < -Math.PI) sweep += Math.PI * 2;
        const arcLen = Math.abs(sweep) * r;
        const n = Math.max(6, Math.ceil(arcLen / (step * 0.7)));
        // Lead-in to T1 and out of T2 (short straights along the rays).
        mid.push({ x: pA.x, y: pA.y });
        const lead = Math.min(dist(pA, T1), 6);
        if (lead > 0.8) mid.push({ x: pA.x + dIn.x * lead, y: pA.y + dIn.y * lead });
        for (let i = 0; i <= n; i++) {
          const a = a1 + sweep * (i / n);
          mid.push({ x: O.x + Math.cos(a) * r, y: O.y + Math.sin(a) * r });
        }
        const leadOut = Math.min(dist(T2, pB), 6);
        if (leadOut > 0.8) mid.push({ x: pB.x - dOut.x * leadOut, y: pB.y - dOut.y * leadOut });
        mid.push({ x: pB.x, y: pB.y });
      }
    }
  }
  if (!mid.length) {
    // Nearly collinear arms: a straight bridge is the correct turn.
    mid.push({ x: pA.x, y: pA.y }, { x: pB.x, y: pB.y });
  }

  // Approach and exit lane polylines.
  const armPoint = (samples, s, travelSign, off) => {
    const q = sampleAt(samples, s);
    if (!q) return null;
    const r = rightOf({ x: q.tx * travelSign, y: q.ty * travelSign });
    return { x: q.x + r.x * off, y: q.y + r.y * off };
  };
  const approachPts = [];
  if (portA.end === 'a') {
    const to = Math.min(samplesA[samplesA.length - 1].s, portA.station + approach);
    for (let s = portA.station + step; s <= to + 1e-6; s += step) {
      const q = armPoint(samplesA, s, -1, offA);
      if (q) approachPts.push(q);
    }
  } else {
    const to = Math.max(0, portA.station - approach);
    for (let s = portA.station - step; s >= to - 1e-6; s -= step) {
      const q = armPoint(samplesA, s, +1, offA);
      if (q) approachPts.push(q);
    }
  }
  approachPts.reverse();

  const exitPts = [];
  if (portB.end === 'a') {
    const to = Math.min(samplesB[samplesB.length - 1].s, portB.station + approach);
    for (let s = portB.station + step; s <= to + 1e-6; s += step) {
      const q = armPoint(samplesB, s, +1, offB);
      if (q) exitPts.push(q);
    }
  } else {
    const to = Math.max(0, portB.station - approach);
    for (let s = portB.station - step; s >= to - 1e-6; s -= step) {
      const q = armPoint(samplesB, s, -1, offB);
      if (q) exitPts.push(q);
    }
  }

  const full = [...approachPts, ...mid, ...exitPts];
  const clean = [full[0]];
  for (let i = 1; i < full.length; i++) {
    if (dist(full[i], clean[clean.length - 1]) > 0.08) clean.push(full[i]);
  }
  for (let i = 0; i < clean.length; i++) {
    const a = clean[Math.max(0, i - 1)];
    const b = clean[Math.min(clean.length - 1, i + 1)];
    clean[i].heading = Math.atan2(b.y - a.y, b.x - a.x);
  }
  clean.filletRadius = filletRadius;
  return clean;
}

// Sweep the vehicle along an arbitrary path, checking pavement coverage.
// The path is treated as the rear-axle trace; the required steering comes from
// the path curvature (bicycle model), and the body footprint is tested against
// the actual pavement triangles.
export function sweepPathOnSurface(path, vehicle, surfaceIndex, opts = {}) {
  if (path.length < 2) {
    return { pass: false, footprints: [], maxSteer: 0, steerRatio: 0, offPavement: 4, maxHeightSpread: 0, reason: 'no-path' };
  }
  // Arc-length resample.
  const cum = [0];
  for (let i = 1; i < path.length; i++) cum.push(cum[i - 1] + dist(path[i - 1], path[i]));
  const total = cum[cum.length - 1];
  const step = opts.step ?? 0.4;
  const n = Math.max(2, Math.ceil(total / step) + 1);
  const stations = [];
  for (let i = 0; i < n; i++) {
    const s = (total * i) / (n - 1);
    let lo = 0;
    let hi = cum.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (cum[mid] <= s) lo = mid;
      else hi = mid;
    }
    const t = (s - cum[lo]) / Math.max(cum[hi] - cum[lo], 1e-9);
    stations.push({
      x: lerp(path[lo].x, path[hi].x, t),
      y: lerp(path[lo].y, path[hi].y, t),
      s,
    });
  }
  // Headings and curvature from the resampled trace.
  for (let i = 0; i < n; i++) {
    const a = stations[Math.max(0, i - 1)];
    const b = stations[Math.min(n - 1, i + 1)];
    stations[i].heading = Math.atan2(b.y - a.y, b.x - a.x);
  }
  const curv = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    const ds = stations[i + 1].s - stations[i - 1].s;
    if (ds > 1e-6) curv[i] = angleDelta(stations[i - 1].heading, stations[i + 1].heading) / ds;
  }
  curv[0] = curv[1] || 0;
  curv[n - 1] = curv[n - 2] || 0;

  const halfLen = vehicle.length / 2;
  const halfW = vehicle.width / 2;
  const rearToCenter = vehicle.wheelbase / 2;
  const maxSteerAngle = vehicle.maxSteer;
  const footprints = [];
  let maxSteer = 0;
  let maxCurvature = 0;
  let offPavement = 0;
  let maxHeightSpread = 0; // total corner spread (informational)
  let maxHeightStep = 0; // residual off the footprint's best-fit plane (real step)
  let worst = null;
  let badGroups = 0;
  let inBad = false;

  for (let i = 0; i < n; i++) {
    const st = stations[i];
    const cos = Math.cos(st.heading);
    const sin = Math.sin(st.heading);
    const cx = st.x + cos * rearToCenter;
    const cy = st.y + sin * rearToCenter;
    const corners = [
      { x: cx + cos * halfLen - sin * halfW, y: cy + sin * halfLen + cos * halfW },
      { x: cx + cos * halfLen + sin * halfW, y: cy + sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen + sin * halfW, y: cy - sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen - sin * halfW, y: cy - sin * halfLen + cos * halfW },
    ];
    footprints.push(corners);
    const need = Math.atan(Math.abs(curv[i]) * vehicle.wheelbase);
    if (need > maxSteer) maxSteer = need;
    maxCurvature = Math.max(maxCurvature, Math.abs(curv[i]));
    let bad = 0;
    const hs = [];
    for (const c of corners) {
      const h = surfaceIndex.heightAt(c.x, c.y);
      if (h === null) bad++;
      else hs.push(h);
    }
    if (bad) {
      offPavement++;
      if (!inBad) {
        badGroups++;
        inBad = true;
      }
      if (!worst) worst = { x: cx, y: cy, off: bad, station: st.s };
    } else {
      inBad = false;
    }
    if (hs.length > 1) {
      const spread = Math.max(...hs) - Math.min(...hs);
      maxHeightSpread = Math.max(maxHeightSpread, spread);
      // A vehicle sitting on a smooth grade legitimately has a spread of
      // grade * length; only deviation from a plane is a surface defect.
      const mean = hs.reduce((a, b) => a + b, 0) / hs.length;
      const step = Math.max(...hs.map((h) => Math.abs(h - mean)));
      maxHeightStep = Math.max(maxHeightStep, step);
    }
  }

  const steerRatio = maxSteer / maxSteerAngle;
  // Step is compared against 0.30 m: a 12 m bus on a 9% ramp still sits flat.
  const pass = offPavement === 0 && maxSteer <= maxSteerAngle + 1e-3 && maxHeightStep <= 0.3;
  return {
    pass,
    footprints,
    maxSteer,
    steerRatio,
    maxCurvature,
    turnRadius: maxCurvature > 1e-6 ? 1 / maxCurvature : Infinity,
    offPavement,
    badGroups,
    maxHeightSpread,
    maxHeightStep,
    worst,
  };
}
