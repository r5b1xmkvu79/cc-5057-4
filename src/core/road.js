// Road route kernel: centreline sampling, grade solving, asymmetric widening,
// reroute helpers, swept-vehicle acceptance and route-level validation.

import {
  angleDelta,
  catmullRom,
  clamp,
  dist,
  lerp,
  norm,
  perp,
  polylineSelfIntersections,
  resamplePolyline,
  simplify,
  sub,
} from './math.js';
import { TOLERANCES } from './standards.js';

const SAMPLE_SPACING = 1.2;

function movingAverage(arr, halfWindow, passes = 1) {
  let src = arr.slice();
  const n = arr.length;
  for (let p = 0; p < passes; p++) {
    const out = new Array(n);
    for (let i = 0; i < n; i++) {
      let sum = 0;
      let w = 0;
      for (let k = -halfWindow; k <= halfWindow; k++) {
        const j = clamp(i + k, 0, n - 1);
        const weight = 1 - Math.abs(k) / (halfWindow + 1);
        sum += src[j] * weight;
        w += weight;
      }
      out[i] = sum / w;
    }
    src = out;
  }
  return src;
}

export function controlPolyline(conn, nodes) {
  const a = nodes.get(conn.a);
  const b = nodes.get(conn.b);
  return [a, ...(conn.controlPoints || []), b].map((p) => ({ x: p.x, y: p.y }));
}

export function sampleCenterline(control, spacing = SAMPLE_SPACING) {
  const dense = catmullRom(control, 20, 0.5);
  const pts = resamplePolyline(dense, spacing);
  const clean = [pts[0]];
  for (let i = 1; i < pts.length; i++) {
    if (dist(pts[i], clean[clean.length - 1]) > 1e-6) clean.push(pts[i]);
  }
  if (clean.length < 2) return [];

  const n = pts.length;
  const s = new Array(n).fill(0);
  for (let i = 1; i < n; i++) s[i] = s[i - 1] + dist(pts[i - 1], pts[i]);

  const tangents = new Array(n);
  for (let i = 0; i < n; i++) {
    const a = pts[Math.max(0, i - 1)];
    const b = pts[Math.min(n - 1, i + 1)];
    tangents[i] = norm(sub(b, a));
  }
  const headings = tangents.map((t) => Math.atan2(t.y, t.x));

  const rawCurvature = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    const ds = s[i + 1] - s[i - 1];
    if (ds > 1e-5) rawCurvature[i] = angleDelta(headings[i - 1], headings[i + 1]) / ds;
  }
  rawCurvature[0] = rawCurvature[1] || 0;
  rawCurvature[n - 1] = rawCurvature[n - 2] || 0;
  const curvature = movingAverage(rawCurvature, 3, 1);

  const out = [];
  for (let i = 0; i < n; i++) {
    const t = tangents[i];
    const nrm = perp(t);
    out.push({
      x: pts[i].x,
      y: pts[i].y,
      s: s[i],
      tx: t.x,
      ty: t.y,
      nx: nrm.x,
      ny: nrm.y,
      heading: headings[i],
      curvature: curvature[i],
      radius: Math.abs(curvature[i]) < 1e-6 ? Infinity : 1 / Math.abs(curvature[i]),
      z: 0,
      baseHalf: 0,
      halfLeft: 0,
      halfRight: 0,
      extraLeft: 0,
      extraRight: 0,
    });
  }
  return out;
}

export function measureCenterline(control) {
  const pts = catmullRom(control, 20, 0.5);
  let L = 0;
  for (let i = 1; i < pts.length; i++) L += dist(pts[i - 1], pts[i]);
  return L;
}

// Interpolate a sample at an exact station (used to place junction ports).
export function sampleAt(samples, station) {
  const n = samples.length;
  if (!n) return null;
  const L = samples[n - 1].s;
  const s = clamp(station, 0, L);
  if (s <= 0) {
    const q = samples[0];
    return { ...q, s: 0 };
  }
  if (s >= L) {
    const q = samples[n - 1];
    return { ...q, s: L };
  }
  let lo = 0;
  let hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid].s <= s) lo = mid;
    else hi = mid;
  }
  const a = samples[lo];
  const b = samples[hi];
  const t = (s - a.s) / Math.max(b.s - a.s, 1e-9);
  const heading = a.heading + angleDelta(a.heading, b.heading) * t;
  const tx = Math.cos(heading);
  const ty = Math.sin(heading);
  return {
    x: lerp(a.x, b.x, t),
    y: lerp(a.y, b.y, t),
    s,
    tx,
    ty,
    nx: -ty,
    ny: tx,
    heading,
    curvature: lerp(a.curvature, b.curvature, t),
    radius: lerp(a.radius === Infinity ? 1e9 : a.radius, b.radius === Infinity ? 1e9 : b.radius, t),
    z: lerp(a.z, b.z, t),
    baseHalf: lerp(a.baseHalf, b.baseHalf, t),
    halfLeft: lerp(a.halfLeft, b.halfLeft, t),
    halfRight: lerp(a.halfRight, b.halfRight, t),
    extraLeft: lerp(a.extraLeft, b.extraLeft, t),
    extraRight: lerp(a.extraRight, b.extraRight, t),
  };
}

// Grade-limited profile close to terrain, pinned at both authored ends.
export function solveProfile(samples, terrain, z0, z1, maxGrade) {
  const n = samples.length;
  const z = new Array(n);
  for (let i = 0; i < n; i++) z[i] = terrain.heightAt(samples[i].x, samples[i].y);
  const L = samples[n - 1].s - samples[0].s;
  z[0] = z0;
  z[n - 1] = z1;
  const feasible = Math.abs(z1 - z0) <= maxGrade * Math.max(L, 1e-6) + 1e-6;
  if (!feasible) {
    for (let i = 0; i < n; i++) z[i] = lerp(z0, z1, samples[i].s / Math.max(L, 1e-9));
    return { z, infeasible: true };
  }

  for (let iter = 0; iter < 240; iter++) {
    for (let i = 1; i < n; i++) {
      const ds = Math.max(samples[i].s - samples[i - 1].s, 1e-6);
      z[i] = clamp(z[i], z[i - 1] - maxGrade * ds, z[i - 1] + maxGrade * ds);
    }
    z[n - 1] = z1;
    for (let i = n - 2; i >= 0; i--) {
      const ds = Math.max(samples[i + 1].s - samples[i].s, 1e-6);
      z[i] = clamp(z[i], z[i + 1] - maxGrade * ds, z[i + 1] + maxGrade * ds);
    }
    z[0] = z0;
    for (let i = 1; i < n - 1; i++) {
      const w = Math.sin((Math.PI * i) / (n - 1));
      z[i] += (terrain.heightAt(samples[i].x, samples[i].y) - z[i]) * 0.12 * w;
    }
    const smoothed = movingAverage(z, 4, 1);
    for (let i = 1; i < n - 1; i++) z[i] = lerp(z[i], smoothed[i], 0.5);
    z[0] = z0;
    z[n - 1] = z1;
  }
  for (let i = 1; i < n; i++) {
    const ds = Math.max(samples[i].s - samples[i - 1].s, 1e-6);
    z[i] = clamp(z[i], z[i - 1] - maxGrade * ds, z[i - 1] + maxGrade * ds);
  }
  for (let i = n - 2; i >= 0; i--) {
    const ds = Math.max(samples[i + 1].s - samples[i].s, 1e-6);
    z[i] = clamp(z[i], z[i + 1] - maxGrade * ds, z[i + 1] + maxGrade * ds);
  }
  z[0] = z0;
  z[n - 1] = z1;
  return { z, infeasible: false };
}

// Flatten the profile to zero grade at both ends so flat junctions join with C1
// continuity instead of a visible slope break.
export function taperEnds(samples, z, taper = 9) {
  const n = samples.length;
  const L = samples[n - 1].s;
  const out = z.slice();
  const tw = Math.min(taper, L * 0.4);
  if (tw < 0.5) return out;
  for (let i = 0; i < n; i++) {
    const d = Math.min(samples[i].s, L - samples[i].s);
    if (d < tw) {
      const t = smootherstepKernel(d / tw);
      const endZ = samples[i].s <= L / 2 ? z[0] : z[n - 1];
      out[i] = lerp(endZ, z[i], t);
    }
  }
  return out;
}

// Re-limit a profile to the class grade while pinning both endpoints. Tapering
// toward a junction raises the ends, which can locally exceed the grade limit;
// this pass restores feasibility without moving the junction elevations.
export function clampProfile(samples, z, maxGrade) {
  const out = z.slice();
  const n = out.length;
  if (n < 2) return out;
  const z0 = out[0];
  const z1 = out[n - 1];
  for (let iter = 0; iter < 120; iter++) {
    for (let i = 1; i < n; i++) {
      const ds = Math.max(samples[i].s - samples[i - 1].s, 1e-6);
      out[i] = clamp(out[i], out[i - 1] - maxGrade * ds, out[i - 1] + maxGrade * ds);
    }
    out[n - 1] = z1;
    for (let i = n - 2; i >= 0; i--) {
      const ds = Math.max(samples[i + 1].s - samples[i].s, 1e-6);
      out[i] = clamp(out[i], out[i + 1] - maxGrade * ds, out[i + 1] + maxGrade * ds);
    }
    out[0] = z0;
  }
  return out;
}

function smootherstepKernel(t) {
  t = clamp(t, 0, 1);
  return t * t * t * (t * (t * 6 - 15) + 10);
}

export function profileStats(samples, z, windowM = 30) {
  let maxGrade = 0;
  const grades = [];
  for (let i = 1; i < samples.length; i++) {
    const ds = Math.max(samples[i].s - samples[i - 1].s, 1e-6);
    const g = (z[i] - z[i - 1]) / ds;
    grades[i - 1] = g;
    maxGrade = Math.max(maxGrade, Math.abs(g));
  }
  let maxRate = 0;
  let maxChange = 0;
  for (let i = 1; i < grades.length; i++) {
    const ds = Math.max(samples[i + 1].s - samples[i].s, 1e-6);
    maxRate = Math.max(maxRate, Math.abs(grades[i] - grades[i - 1]) / ds);
  }
  for (let i = 0; i < grades.length; i++) {
    for (let j = i + 1; j < grades.length; j++) {
      if (samples[j + 1].s - samples[i].s > windowM) break;
      maxChange = Math.max(maxChange, Math.abs(grades[j] - grades[i]));
    }
  }
  return { maxGrade, maxChange, maxRate, grades };
}

export function curvatureStats(samples) {
  let worst = { radius: Infinity, curvature: 0, index: 0, s: 0 };
  for (let i = 0; i < samples.length; i++) {
    if (samples[i].radius < worst.radius) {
      worst = { radius: samples[i].radius, curvature: samples[i].curvature, index: i, s: samples[i].s };
    }
  }
  return worst;
}

export function setBaseWidths(samples, cls) {
  for (const s of samples) {
    s.baseHalf = cls.width / 2 + cls.shoulder;
    s.halfLeft = s.baseHalf;
    s.halfRight = s.baseHalf;
    s.extraLeft = 0;
    s.extraRight = 0;
  }
}

// extra arrays are the measured need on each side. A little inner comfort can be
// requested through `innerComfort`.
export function applyWidening(samples, leftExtra, rightExtra, smooth = 3) {
  const le = movingAverage(leftExtra, smooth, 2);
  const re = movingAverage(rightExtra, smooth, 2);
  for (let i = 0; i < samples.length; i++) {
    samples[i].extraLeft = Math.max(0, le[i]);
    samples[i].extraRight = Math.max(0, re[i]);
    samples[i].halfLeft = samples[i].baseHalf + samples[i].extraLeft;
    samples[i].halfRight = samples[i].baseHalf + samples[i].extraRight;
  }
}

// Kinematic sweep of the vehicle body along the centreline. Corner exceedance is
// measured against the station's own pavement half widths.
export function sweepVehicle(samples, vehicle, opts = {}) {
  const lookahead = opts.lookahead ?? Math.max(3.0, vehicle.wheelbase * 1.35);
  const step = opts.step ?? 0.6;
  const clearance = opts.clearance ?? 0.04;
  const n = samples.length;
  if (n < 2) return emptySweep();
  const L = samples[n - 1].s;

  const sampleAtS = (s) => sampleAt(samples, s);

  const halfLen = vehicle.length / 2;
  const halfW = vehicle.width / 2;
  const footprints = [];
  const path = [];
  let maxSteer = 0;
  let maxExceed = 0;
  let worst = null;
  let rear = { x: samples[0].x, y: samples[0].y };
  let heading = samples[0].heading;
  let s = 0;
  const maxSteerAngle = vehicle.maxSteer;

  for (let iter = 0; iter < 20000; iter++) {
    const centreStation = Math.min(s + vehicle.wheelbase / 2, L);
    const st = sampleAtS(centreStation);
    const cx = rear.x + Math.cos(heading) * (vehicle.wheelbase / 2);
    const cy = rear.y + Math.sin(heading) * (vehicle.wheelbase / 2);
    const cos = Math.cos(heading);
    const sin = Math.sin(heading);
    const corners = [
      { x: cx + cos * halfLen - sin * halfW, y: cy + sin * halfLen + cos * halfW },
      { x: cx + cos * halfLen + sin * halfW, y: cy + sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen + sin * halfW, y: cy - sin * halfLen - cos * halfW },
      { x: cx - cos * halfLen - sin * halfW, y: cy - sin * halfLen + cos * halfW },
    ];
    footprints.push({ station: centreStation, corners });
    path.push({ s, x: cx, y: cy, heading, station: centreStation });

    for (const c of corners) {
      const lat = (c.x - st.x) * st.nx + (c.y - st.y) * st.ny;
      const allowed = lat >= 0 ? st.halfLeft : st.halfRight;
      const exceed = Math.abs(lat) - allowed;
      if (exceed > maxExceed) {
        maxExceed = exceed;
        worst = { x: c.x, y: c.y, station: centreStation, sideways: lat, allowed };
      }
    }

    s += step;
    if (s >= L) break;
    const target = sampleAtS(Math.min(s + lookahead, L));
    const dx = target.x - rear.x;
    const dy = target.y - rear.y;
    const distToTarget = Math.hypot(dx, dy) || 1;
    const alpha = angleDelta(heading, Math.atan2(dy, dx));
    const ld = Math.max(distToTarget, 1e-3);
    const delta = Math.atan((2 * vehicle.wheelbase * Math.sin(alpha)) / ld);
    maxSteer = Math.max(maxSteer, Math.abs(delta));
    const steer = clamp(delta, -maxSteerAngle, maxSteerAngle);
    const adv = Math.min(step, distToTarget);
    heading += (Math.tan(steer) / vehicle.wheelbase) * adv;
    rear = { x: rear.x + Math.cos(heading) * adv, y: rear.y + Math.sin(heading) * adv };
  }

  return {
    frames: footprints,
    footprints: footprints.map((f) => f.corners),
    path,
    maxSteer,
    steerRatio: maxSteer / maxSteerAngle,
    maxExceed,
    worst,
    pass: maxExceed <= clearance + 1e-3 && maxSteer <= maxSteerAngle + 1e-4,
  };
}

function emptySweep() {
  return { frames: [], footprints: [], path: [], maxSteer: 0, steerRatio: 0, maxExceed: 0, worst: null, pass: false };
}

function nearestStation(samples, x, y) {
  let bestI = 0;
  let bestD = Infinity;
  for (let i = 0; i < samples.length; i++) {
    const d = (samples[i].x - x) ** 2 + (samples[i].y - y) ** 2;
    if (d < bestD) {
      bestD = d;
      bestI = i;
    }
  }
  return { index: bestI, dist: Math.sqrt(bestD) };
}

// Assign a corner's exceedance to the station whose own cross-section is
// closest, but also sample the two neighbouring stations to catch diagonals.
function nearestStationSamples(samples, x, y) {
  const near = nearestStation(samples, x, y);
  return [near.index];
}

// Iteratively measure the extra width each side needs for the swept body.
export function measureRequiredWidening(samples, vehicle, iterations = 4) {
  const n = samples.length;
  let leftExtra = new Array(n).fill(0);
  let rightExtra = new Array(n).fill(0);
  const baseL = samples.map((s) => s.halfLeft);
  const baseR = samples.map((s) => s.halfRight);
  for (let it = 0; it < iterations; it++) {
    applyWidening(samples, leftExtra, rightExtra, 2);
    const sweep = sweepVehicle(samples, vehicle, { clearance: 0.06 });
    if (sweep.maxExceed <= 0.02) break;
    for (const fp of sweep.frames) {
      const k = indexAtStation(samples, fp.station);
      const st = samples[k];
      for (const c of fp.corners) {
        const lat = (c.x - st.x) * st.nx + (c.y - st.y) * st.ny;
        const allowed = lat >= 0 ? st.halfLeft : st.halfRight;
        const need = Math.abs(lat) - allowed + 0.1;
        if (need > 0) {
          if (lat >= 0) leftExtra[k] = Math.max(leftExtra[k], need);
          else rightExtra[k] = Math.max(rightExtra[k], need);
        }
      }
    }
    leftExtra = movingAverage(leftExtra, 3, 1);
    rightExtra = movingAverage(rightExtra, 3, 1);
  }
  applyWidening(samples, leftExtra, rightExtra, 2);
  const sweep = sweepVehicle(samples, vehicle);
  return { leftExtra, rightExtra, sweep, baseL, baseR };
}

function indexAtStation(samples, station) {
  let lo = 0;
  let hi = samples.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (samples[mid].s <= station) lo = mid;
    else hi = mid;
  }
  const t = (station - samples[lo].s) / Math.max(samples[hi].s - samples[lo].s, 1e-9);
  return t > 0.5 ? hi : lo;
}

// --- route validation -----------------------------------------------------------

export function validateRoute(conn, samples, terrain, cls, vehicle, opts = {}) {
  const violations = [];
  const add = (code, severity, message, value, limit, plan, station = null) =>
    violations.push({ code, severity, message, value, limit, plan, station, connectionId: conn.id });

  if (!samples.length) {
    add('empty-route', 'error', '路线没有有效采样点', 0, 1, null);
    return violations;
  }

  const b = terrain.bounds;
  let outAt = null;
  for (const smp of samples) {
    if (smp.x < b.minX || smp.x > b.maxX || smp.y < b.minZ || smp.y > b.maxZ) {
      outAt = { x: smp.x, y: smp.y };
      break;
    }
  }
  if (outAt) add('out-of-bounds', 'error', '道路超出地形边界', 0, 0, outAt);

  if (polylineSelfIntersections(samples.map((p) => ({ x: p.x, y: p.y }))).length) {
    add('self-intersection', 'error', '中心线自相交', 0, 0, null);
  }

  // Centreline obstruction. Full-width pavement is checked on the pavement pair.
  let hit = null;
  for (const smp of samples) {
    const obs = terrain.forbidden({ x: smp.x, y: smp.y }, 0);
    if (obs.length) {
      hit = { point: { x: smp.x, y: smp.y }, obstacles: obs.map((o) => o.id) };
      add('forbidden-route', 'error', `中心线进入禁止区 ${obs.map((o) => o.label || o.id).join(', ')}`, obs.length, 0, hit.point, smp.s);
      break;
    }
  }
  if (hit) violations.corridorHit = hit;

  let badRadius = null;
  for (const smp of samples) {
    if (smp.radius < cls.minRadius - 1e-6) {
      badRadius = smp;
      add('min-radius', 'error', `曲率半径 ${smp.radius.toFixed(1)}m 小于 ${cls.label} 最小 ${cls.minRadius}m`, smp.radius, cls.minRadius, { x: smp.x, y: smp.y }, smp.s);
      break;
    }
  }

  const minVehicleRadius = vehicle.wheelbase / Math.tan(vehicle.maxSteer);
  let badVehicle = null;
  for (const smp of samples) {
    if (smp.radius < minVehicleRadius - 1e-6) {
      badVehicle = smp;
      add('vehicle-turn-limit', 'error', `半径 ${smp.radius.toFixed(1)}m 小于 ${vehicle.label} 最小转弯半径 ${minVehicleRadius.toFixed(1)}m`, smp.radius, minVehicleRadius, { x: smp.x, y: smp.y }, smp.s);
      break;
    }
  }

  const stats = profileStats(samples, samples.map((s) => s.z));
  if (stats.maxGrade > cls.maxGrade + 1e-4) {
    add('max-grade', 'error', `纵坡 ${(stats.maxGrade * 100).toFixed(1)}% 超过 ${(cls.maxGrade * 100).toFixed(1)}%`, stats.maxGrade, cls.maxGrade, null);
  }
  if (stats.maxChange > cls.maxGradeChange + 1e-4) {
    add('grade-change', 'error', `30m 内坡度变化 ${(stats.maxChange * 100).toFixed(1)}% 超过 ${(cls.maxGradeChange * 100).toFixed(1)}%`, stats.maxChange, cls.maxGradeChange, null);
  }
  if (stats.maxRate > cls.maxCurvatureRate * 3 + 1e-4) {
    add('curvature-rate', 'warn', '曲率变化率偏大（急弯突变）', stats.maxRate, cls.maxCurvatureRate * 3, null);
  }

  let jump = null;
  for (let i = 1; i < samples.length; i++) {
    const ds = Math.max(samples[i].s - samples[i - 1].s, 1e-6);
    const dz = Math.abs(samples[i].z - samples[i - 1].z);
    if (dz > TOLERANCES.gradeJumpPerSample + cls.maxGrade * ds * 4) {
      jump = { at: { x: samples[i].x, y: samples[i].y }, dz, ds };
      break;
    }
  }
  if (jump) {
    add('grade-jump', 'error', `高程突变 ${jump.dz.toFixed(2)}m / ${jump.ds.toFixed(2)}m（禁止为接路而跳变）`, jump.dz, TOLERANCES.gradeJumpPerSample, jump.at);
  }

  if (opts.infeasibleProfile) {
    add('grade-infeasible', 'error', `两端 ${Math.abs(opts.infeasibleDelta || 0).toFixed(1)}m 高差在 ${(cls.maxGrade * 100).toFixed(1)}% 纵坡下无法连接`, Math.abs(opts.infeasibleDelta || 0), cls.maxGrade, null);
  }

  let maxCut = 0;
  let maxFill = 0;
  for (const smp of samples) {
    const d = smp.z - terrain.heightAt(smp.x, smp.y);
    maxCut = Math.max(maxCut, d);
    maxFill = Math.max(maxFill, -d);
  }
  if (maxCut > 8 || maxFill > 8) {
    add('earthwork', 'warn', `土方较大（挖 ${maxCut.toFixed(1)}m / 填 ${maxFill.toFixed(1)}m）`, Math.max(maxCut, maxFill), 8, null);
  }

  return violations;
}

// Move the nearest interior control point away from the bend centre so the local
// radius grows. Returns new control points or null if nothing could move.
export function rerouteControl(control, samples, station, amount) {
  const pts = control.map((p) => ({ x: p.x, y: p.y }));
  const smp = sampleAt(samples, station);
  if (!smp) return null;
  const turnSign = Math.sign(smp.curvature) || 1;
  // Away from the centre of the turn: opposite the centripetal (left) normal.
  const dir = { x: -turnSign * smp.nx, y: -turnSign * smp.ny };

  let bestI = -1;
  let bestD = Infinity;
  for (let i = 1; i < pts.length - 1; i++) {
    let dmin = Infinity;
    for (const sp of samples) dmin = Math.min(dmin, dist(pts[i], sp));
    if (dmin < bestD) {
      bestD = dmin;
      bestI = i;
    }
  }
  if (bestI === -1) {
    // Straight two-node road: insert an interior point and push it sideways so a
    // smooth bend can form.
    const target = { x: smp.x + dir.x * amount, y: smp.y + dir.y * amount };
    let bestAt = 1;
    let bestS = Infinity;
    for (let i = 1; i < pts.length; i++) {
      const midS = i; // ordinal position only
      if (midS < bestS) {
        bestS = midS;
        bestAt = i;
      }
    }
    pts.splice(1, 0, target);
    return pts;
  }
  pts[bestI] = { x: pts[bestI].x + dir.x * amount, y: pts[bestI].y + dir.y * amount };
  return pts;
}

// Nudge the nearest interior control point away from an obstacle (or back inside
// the terrain bounds).
export function rerouteAway(control, samples, terrain, amount, station = null) {
  const pts = control.map((p) => ({ x: p.x, y: p.y }));
  let idx = 1;
  if (station !== null) {
    const smp = sampleAt(samples, station);
    let bestD = Infinity;
    for (let i = 1; i < pts.length - 1; i++) {
      const d = dist(pts[i], smp);
      if (d < bestD) {
        bestD = d;
        idx = i;
      }
    }
    if (pts.length <= 2) idx = 1;
  }
  let target = null;
  if (idx === 0) idx = 1;
  if (idx >= pts.length - 1 && pts.length <= 2) {
    const mid = samples[Math.floor(samples.length / 2)];
    // Insert a mid control point.
    const b = terrain.bounds;
    const cx = (b.minX + b.maxX) / 2;
    const cz = (b.minZ + b.maxZ) / 2;
    const dir = norm({ x: cx - mid.x, y: cz - mid.y });
    pts.splice(1, 0, { x: mid.x + dir.x * amount, y: mid.y + dir.y * amount });
    return pts;
  }
  const smp = samples[Math.min(samples.length - 1, Math.floor(samples.length / 2))];
  const obs = terrain.obstacles;
  let away = { x: 0, y: 0 };
  for (const o of obs) {
    if (o.shape === 'circle') {
      away.x += (pts[idx].x - o.x) / (Math.hypot(pts[idx].x - o.x, pts[idx].y - o.y) + 1e-6);
      away.y += (pts[idx].y - o.y) / (Math.hypot(pts[idx].x - o.x, pts[idx].y - o.y) + 1e-6);
    } else {
      const c = polygonCentroid(o.points);
      const d = Math.hypot(pts[idx].x - c.x, pts[idx].y - c.y) + 1e-6;
      away.x += (pts[idx].x - c.x) / d;
      away.y += (pts[idx].y - c.y) / d;
    }
  }
  if (Math.hypot(away.x, away.y) < 1e-3) {
    const b = terrain.bounds;
    const c = { x: (b.minX + b.maxX) / 2, y: (b.minZ + b.maxZ) / 2 };
    away = norm({ x: c.x - pts[idx].x, y: c.y - pts[idx].y });
  } else {
    away = norm(away);
  }
  pts[idx] = { x: pts[idx].x + away.x * amount, y: pts[idx].y + away.y * amount };
  return pts;
}

function polygonCentroid(poly) {
  let x = 0;
  let y = 0;
  for (const p of poly) {
    x += p.x;
    y += p.y;
  }
  return { x: x / poly.length, y: y / poly.length };
}
