// Stage 7 — build record and aggregate result.
// Produces the deterministic build record and the route reports / aggregate
// violation list. Provenance is embedded per resolved route so the relationship
// between a route and its pavement stays traceable after a local edit.

import { terrainToRecord } from '../terrain.js';
import { provenanceRecord } from '../provenance.js';

function round(v, d = 3) {
  return Number(v.toFixed(d));
}

export function makeBuildRecord({
  project,
  seed,
  controls,
  terrain,
  routes,
  junctions,
  turnTests,
  junctionReports,
  diagnostics,
  overlapIssues,
  seams,
  revision,
  graphId,
}) {
  const fixed = {};
  for (const c of project.connections) {
    if (c.controlPoints) fixed[c.id] = c.controlPoints.map((p) => ({ x: round(p.x), y: round(p.y) }));
  }
  return {
    version: 1,
    graphId: graphId ?? null,
    revision: revision ?? null,
    seed,
    controls,
    vehicleId: project.vehicleId || 'sedan',
    terrain: terrainToRecord(terrain),
    terrainSeed: terrain.seed,
    nodes: project.nodes.map((n) => ({ id: n.id, x: round(n.x), y: round(n.y), elevation: n.elevation ?? null, kind: n.kind || 'junction' })),
    connections: project.connections.map((c) => ({
      id: c.id,
      a: c.a,
      b: c.b,
      classId: c.classId,
      name: c.name,
      vehicleId: c.vehicleId || null,
      locked: !!c.locked,
      controlPoints: (c.controlPoints || []).map((p) => ({ x: round(p.x), y: round(p.y) })),
    })),
    fixedControls: fixed,
    resolved: routes.map((r) => ({
      id: r.conn.id,
      control: r.control.map((p) => ({ x: round(p.x), y: round(p.y) })),
      startStation: round(r.startStation),
      endStation: round(r.endStation),
      length: round(r.samples.length ? r.samples[r.samples.length - 1].s : 0),
      minRadius: round(r.curvatureStats.radius),
      maxGrade: round(r.profileStats.maxGrade, 4),
      maxWidening: round(Math.max(0, ...r.samples.map((s) => Math.max(s.extraLeft, s.extraRight)))),
      pass: r.finalViolations.filter((v) => v.severity === 'error').length === 0,
      decisions: r.decisions,
      provenance: provenanceRecord(r),
      ribbonProvenance: provenanceRecord(r.ribbon),
    })),
    junctions: junctionReports,
    turns: turnTests.map((t) => ({
      junctionId: t.junctionId,
      from: t.from,
      to: t.to,
      pass: t.pass,
      steerDeg: round(t.maxSteerDeg, 1),
      turnRadius: round(t.turnRadius, 1),
      maxHeightStep: round(t.maxHeightStep ?? 0, 2),
    })),
    diagnostics,
    overlapIssues,
    seams: seams.filter((s) => s.severity === 'error'),
    createdAt: new Date().toISOString(),
  };
}

export function buildRouteReports(project, routes) {
  return routes.map((r) => ({
    id: r.conn.id,
    name: r.conn.name || r.conn.id,
    classId: r.conn.classId,
    vehicleId: r.conn.vehicleId || project.vehicleId || 'sedan',
    length: r.samples.length ? r.samples[r.samples.length - 1].s : 0,
    startStation: r.startStation,
    endStation: r.endStation,
    minRadius: r.curvatureStats.radius,
    maxGrade: r.profileStats.maxGrade,
    gradeChange: r.profileStats.maxChange,
    maxWidening: Math.max(0, ...r.samples.map((s) => Math.max(s.extraLeft, s.extraRight))),
    widenedFraction: r.samples.filter((s) => Math.max(s.extraLeft, s.extraRight) > 0.05).length / Math.max(1, r.samples.length),
    sweep: {
      pass: r.sweep ? r.sweep.pass : false,
      maxExceed: r.sweep ? r.sweep.maxExceed : 0,
      maxSteerDeg: r.sweep ? (r.sweep.maxSteer * 180) / Math.PI : 0,
      worst: r.sweep ? r.sweep.worst : null,
    },
    violations: r.finalViolations,
    decisions: r.decisions,
    locked: !!r.conn.locked,
    errorCount: r.finalViolations.filter((v) => v.severity === 'error').length,
    warnCount: r.finalViolations.filter((v) => v.severity === 'warn').length,
  }));
}

export function aggregateViolations({ routeReports, overlapIssues, seams, junctionReports, turnTests, nodes }) {
  const allViolations = [
    ...routeReports.flatMap((r) => r.violations),
    ...overlapIssues,
    ...seams.filter((s) => s.severity === 'error'),
    ...junctionReports.flatMap((j) =>
      j.issues.map((code) => ({
        code,
        severity: 'error',
        message: `路口 ${j.id} 拓扑问题: ${code}`,
        plan: { x: nodes.get(j.nodeId).x, y: nodes.get(j.nodeId).y },
        junctionId: j.id,
      })),
    ),
    ...turnTests
      .filter((t) => !t.pass)
      .map((t) => ({
        code: 'turn-fail',
        severity: 'error',
        message: `路口 ${t.junctionId} 转向不通过（离开路面 ${t.offPavement} 点，转向 ${t.maxSteerDeg.toFixed(0)}°）`,
        plan: t.worst ? { x: t.worst.x, y: t.worst.y } : null,
        junctionId: t.junctionId,
        turn: { from: t.from, to: t.to },
      })),
  ];
  return {
    violations: allViolations,
    errorCount: allViolations.filter((v) => v.severity === 'error').length,
    warnCount: allViolations.filter((v) => v.severity === 'warn').length,
  };
}
