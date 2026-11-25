// Equivalence metrics for before/after road construction diffs.
//
// Used by the net-directory test suite and the CLI to prove a refactor (or a
// constrained edit) preserves geometry. Two independent measurements are used:
//
//   - centreline error: max and RMS planar deviation between matched route
//     centrelines (metres), plus elevation deviation,
//   - seam correspondence: per-connection pavement-edge seam gaps measured on
//     both builds, paired by connection id.
//
// Everything is computed from fresh kernel output — there are no hand-edited
// fixtures — so a pass cannot be reported without a matching measurement.

import { dist } from './math.js';

// Compare two built results for the same project. Routes are paired by
// connection id; junctions by junction id.
export function diffBuilds(before, after, opts = {}) {
  const planarTol = opts.planarTolerance ?? 1e-6; // metres
  const elevationTol = opts.elevationTolerance ?? 1e-6; // metres
  const seamTol = opts.seamTolerance ?? 1e-6; // metres

  const routeDiffs = [];
  const beforeIds = new Set(before.routes.map((r) => r.conn.id));
  const afterIds = new Set(after.routes.map((r) => r.conn.id));
  const missing = [...beforeIds].filter((id) => !afterIds.has(id));
  const added = [...afterIds].filter((id) => !beforeIds.has(id));

  for (const id of beforeIds) {
    if (!afterIds.has(id)) continue;
    const ra = before.routes.find((r) => r.conn.id === id);
    const rb = after.routes.find((r) => r.conn.id === id);
    routeDiffs.push(compareCenterlines(id, ra.samples, rb.samples));
  }

  const seamDiffs = compareSeams(before, after);
  const junctionDiffs = compareJunctions(before, after);

  const maxPlanar = Math.max(0, ...routeDiffs.map((d) => d.maxPlanar));
  const maxElevation = Math.max(0, ...routeDiffs.map((d) => d.maxElevation));
  const maxSeamDelta = Math.max(0, ...seamDiffs.map((d) => Math.abs(d.delta)));

  const equivalent =
    missing.length === 0 &&
    added.length === 0 &&
    maxPlanar <= planarTol &&
    maxElevation <= elevationTol &&
    maxSeamDelta <= seamTol &&
    junctionDiffs.every((d) => d.trianglesEqual && d.areaDelta <= planarTol);

  return {
    equivalent,
    tolerances: { planarTol, elevationTol, seamTol },
    maxPlanar,
    maxElevation,
    maxSeamDelta,
    routeDiffs,
    seamDiffs,
    junctionDiffs,
    missingRoutes: missing,
    addedRoutes: added,
    units: { distance: 'm', elevation: 'm', seam: 'm' },
  };
}

// Sample-count-independent centreline comparison: project each after-sample onto
// the before polyline and measure the deviation.
export function compareCenterlines(id, a, b) {
  let maxPlanar = 0;
  let sumSqPlanar = 0;
  let maxElevation = 0;
  let sumSqElev = 0;
  let n = 0;
  for (const q of b) {
    const nearest = nearestOnPolyline(a, q.x, q.y);
    const planar = Math.hypot(q.x - nearest.x, q.y - nearest.y);
    const elev = Math.abs(q.z - nearest.z);
    maxPlanar = Math.max(maxPlanar, planar);
    maxElevation = Math.max(maxElevation, elev);
    sumSqPlanar += planar * planar;
    sumSqElev += elev * elev;
    n += 1;
  }
  return {
    connectionId: id,
    samplesBefore: a.length,
    samplesAfter: b.length,
    maxPlanar,
    rmsPlanar: Math.sqrt(sumSqPlanar / Math.max(1, n)),
    maxElevation,
    rmsElevation: Math.sqrt(sumSqElev / Math.max(1, n)),
    lengthBefore: a.length ? a[a.length - 1].s : 0,
    lengthAfter: b.length ? b[b.length - 1].s : 0,
  };
}

function nearestOnPolyline(samples, x, y) {
  let best = samples[0];
  let bestD = Infinity;
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1];
    const c = samples[i];
    const dx = c.x - a.x;
    const dy = c.y - a.y;
    const l2 = dx * dx + dy * dy;
    const t = l2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((x - a.x) * dx + (y - a.y) * dy) / l2));
    const px = a.x + dx * t;
    const py = a.y + dy * t;
    const d = (px - x) ** 2 + (py - y) ** 2;
    if (d < bestD) {
      bestD = d;
      best = { x: px, y: py, z: a.z + (c.z - a.z) * t };
    }
  }
  return best;
}

// Pair seam measurements by connection id. Each build's worst seam gap per
// connection is compared so the correspondence is explicit in the report.
export function compareSeams(before, after) {
  const worstByConn = (built) => {
    const m = new Map();
    for (const s of built.seams) {
      const v = Math.abs(s.value || 0);
      if (!m.has(s.connectionId) || v > m.get(s.connectionId).gap) {
        m.set(s.connectionId, { gap: v, code: s.code, severity: s.severity });
      }
    }
    return m;
  };
  const ba = worstByConn(before);
  const bb = worstByConn(after);
  const ids = new Set([...ba.keys(), ...bb.keys()]);
  return [...ids].map((id) => {
    const a = ba.get(id);
    const b = bb.get(id);
    return {
      connectionId: id,
      beforeGap: a ? a.gap : 0,
      afterGap: b ? b.gap : 0,
      delta: (b ? b.gap : 0) - (a ? a.gap : 0),
      beforeCode: a?.code ?? null,
      afterCode: b?.code ?? null,
    };
  });
}

export function compareJunctions(before, after) {
  const out = [];
  const map = new Map(after.junctionReports.map((j) => [j.id, j]));
  for (const ja of before.junctionReports) {
    const jb = map.get(ja.id);
    if (!jb) {
      out.push({ id: ja.id, present: false, trianglesEqual: false, areaDelta: Infinity });
      continue;
    }
    out.push({
      id: ja.id,
      present: true,
      trianglesEqual: ja.triangles === jb.triangles,
      areaDelta: Math.abs(ja.area - jb.area),
      gradeDelta: Math.abs(ja.maxGrade - jb.maxGrade),
      spreadDelta: Math.abs(ja.heightSpread - jb.heightSpread),
    });
  }
  return out;
}

// Convenience: assert-equivalent with a human-readable failure message.
export function assertEquivalent(diff) {
  if (diff.equivalent) return;
  const lines = [];
  if (diff.missingRoutes.length) lines.push(`missing routes: ${diff.missingRoutes.join(',')}`);
  if (diff.addedRoutes.length) lines.push(`added routes: ${diff.addedRoutes.join(',')}`);
  for (const d of diff.routeDiffs) {
    if (d.maxPlanar > diff.tolerances.planarTol || d.maxElevation > diff.tolerances.elevationTol) {
      lines.push(`${d.connectionId}: planar ${d.maxPlanar.toExponential(2)}m elev ${d.maxElevation.toExponential(2)}m`);
    }
  }
  for (const d of diff.seamDiffs) {
    if (Math.abs(d.delta) > diff.tolerances.seamTol) lines.push(`seam ${d.connectionId}: delta ${d.delta.toExponential(2)}m`);
  }
  const err = new Error(`geometry not equivalent: ${lines.join('; ') || 'junction mismatch'}`);
  err.diff = diff;
  throw err;
}

export { dist };
