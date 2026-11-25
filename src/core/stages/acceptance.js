// Stage 5 — acceptance: route re-validation on final widths and junction turn
// tests against the published collision mesh. All feasibility numbers come
// from real computation on the actual SurfaceIndex; nothing is reported without
// a corresponding measurement, so the geometry interface cannot claim a pass it
// did not compute.

import { classById, vehicleById } from '../standards.js';
import { validateRoute } from '../road.js';
import { buildTurnPath, sweepPathOnSurface } from '../geometry.js';

function angleOf(ax, ay, bx, by) {
  let d = Math.atan2(by, bx) - Math.atan2(ay, ax);
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

// Re-run route-level acceptance on the final (widened, junction-trimmed) widths.
// The terrain is an explicit argument: acceptance never reaches into hidden
// solver state.
export function validateRoutes(project, routes, terrain) {
  for (const r of routes) {
    r.sweep = r.sweepFinal;
    r.finalViolations = validateRoute(
      r.conn,
      r.samples,
      terrain,
      classById(r.conn.classId),
      vehicleById(r.conn.vehicleId || project.vehicleId || 'sedan'),
      { infeasibleProfile: r.profileInfeasible, infeasibleDelta: r.profileDelta },
    );
    if (r.sweep && !r.sweep.pass) {
      r.finalViolations.push({
        code: 'swept-corridor',
        severity: 'error',
        message: `车辆扫掠超出路面 ${(r.sweep.maxExceed).toFixed(2)}m`,
        value: r.sweep.maxExceed,
        limit: 0,
        plan: r.sweep.worst ? { x: r.sweep.worst.x, y: r.sweep.worst.y } : null,
        station: r.sweep.worst ? r.sweep.worst.station : null,
        connectionId: r.conn.id,
      });
    }
  }
}

// Junction turn tests on the real collision mesh.
export function runTurnTests(project, junctions, routes, surfaceIndex) {
  const turnTests = [];
  for (const j of junctions) {
    if (j.incident.length < 2) continue;
    const arms = j.incident;
    for (let i = 0; i < arms.length; i++) {
      const armA = arms[i];
      const armB = arms[(i + 1) % arms.length];
      if (arms.length === 2 && i === 1) continue;
      const ra = routes.find((r) => r.conn.id === armA.conn.id);
      const rb = routes.find((r) => r.conn.id === armB.conn.id);
      const fromPort = armA.port;
      const toPort = armB.port;
      const vehicle = vehicleById(project.vehicleId || 'sedan');
      const path = buildTurnPath(fromPort, ra.samples, toPort, rb.samples, {
        vehicleHalfWidth: vehicle.width / 2,
      });
      const test = sweepPathOnSurface(path, vehicle, surfaceIndex);
      const turnAngle = Math.abs(angleOf(-fromPort.ux, -fromPort.uy, toPort.ux, toPort.uy));
      turnTests.push({
        junctionId: j.id,
        from: { connectionId: armA.conn.id, end: armA.end },
        to: { connectionId: armB.conn.id, end: armB.end },
        turningAngleDeg: (turnAngle * 180) / Math.PI,
        pass: test.pass,
        offPavement: test.offPavement,
        steerRatio: test.steerRatio,
        maxSteerDeg: (test.maxSteer * 180) / Math.PI,
        maxHeightSpread: test.maxHeightSpread,
        maxHeightStep: test.maxHeightStep,
        turnRadius: test.turnRadius,
        footprints: test.footprints,
        path,
        worst: test.worst,
      });
    }
  }
  return turnTests;
}

export function junctionReports(junctions) {
  return junctions.map((j) => ({
    id: j.id,
    nodeId: j.node.id,
    arms: j.incident.length,
    area: j.area,
    maxGrade: j.surface ? j.surface.maxGrade : 0,
    heightSpread: j.heightSpread,
    issues: j.issues || [],
    triangles: j.surface ? j.surface.triangles.length : 0,
    portElevations: j.incident.map((i) => ({ connectionId: i.conn.id, end: i.end, z: i.port?.z ?? 0 })),
  }));
}
