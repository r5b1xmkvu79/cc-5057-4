#!/usr/bin/env node
// Road Forge command-line helper.
//
// The graphical editor (npm run dev) remains the supported way to construct
// roads; this CLI only assists the same kernel from a terminal: build a sample,
// print its geometry/validation summary, run an equivalence diff between two
// builds, and inspect which pavement a recorded drive actually used. It imports
// the same staged kernel the editor uses, so results cannot diverge.

import process from 'node:process';
import { buildProject } from '../src/core/build.js';
import { RoadGeometryInterface } from '../src/core/geometry-interface.js';
import { diffBuilds } from '../src/core/equivalence.js';
import { sampleProject, SAMPLE_IDS } from '../src/project.js';

function parseArgs(argv) {
  const args = { _: [] };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a.startsWith('--')) args[a.slice(2)] = argv[++i] ?? true;
    else args._.push(a);
  }
  return args;
}

function summarize(b) {
  return {
    graphId: b.graphId,
    revision: b.revision,
    ok: b.ok,
    errorCount: b.errorCount,
    warnCount: b.warnCount,
    routes: b.routeReports.map((r) => ({
      id: r.id,
      length: Number(r.length.toFixed(2)),
      minRadius: isFinite(r.minRadius) ? Number(r.minRadius.toFixed(2)) : null,
      maxGrade: Number((r.maxGrade * 100).toFixed(2)),
      maxWidening: Number(r.maxWidening.toFixed(2)),
      sweepPass: r.sweep.pass,
      locked: r.locked,
    })),
    junctions: b.junctionReports.map((j) => ({
      id: j.id,
      arms: j.arms,
      triangles: j.triangles,
      maxGradePct: Number((j.maxGrade * 100).toFixed(2)),
      issues: j.issues,
    })),
    turns: b.turnTests.map((t) => ({
      junction: t.junctionId,
      from: t.from.connectionId,
      to: t.to.connectionId,
      feasible: t.pass,
      offPavement: t.offPavement,
    })),
    pavementTriangles: b.pavement.pavementTriangles.length,
    pavementArea: Number(b.pavementArea.toFixed(2)),
    seamErrors: b.seams.filter((s) => s.severity === 'error').length,
  };
}

function cmdBuild(args) {
  const id = args.scene || 'arterial';
  if (!SAMPLE_IDS.includes(id)) throw new Error(`unknown scene ${id}; choose ${SAMPLE_IDS.join(', ')}`);
  const b = buildProject(sampleProject(id));
  process.stdout.write(JSON.stringify(summarize(b), null, 2) + '\n');
  if (!b.ok) process.exitCode = 2;
}

function cmdDiff(args) {
  // Two builds of the same scene (optionally after a programmatic control-point
  // edit expressed as --conn/--dx/--dy) compared with measured centreline and
  // seam metrics.
  const id = args.scene || 'arterial';
  const before = buildProject(sampleProject(id));
  const edited = sampleProject(id);
  if (args.conn) {
    const c = edited.connections.find((x) => x.id === args.conn);
    if (!c) throw new Error(`connection ${args.conn} not found`);
    const dx = Number(args.dx || 0);
    const dy = Number(args.dy || 0);
    c.controlPoints = (c.controlPoints || []).map((p, i) => (i === 0 ? { x: p.x + dx, y: p.y + dy } : p));
    if (!c.controlPoints.length) c.controlPoints = [{ x: edited.nodes[0].x + dx, y: edited.nodes[0].y + dy }];
  }
  const after = buildProject(edited);
  const diff = diffBuilds(before, after);
  process.stdout.write(JSON.stringify({
    scene: id,
    equivalent: diff.equivalent,
    maxPlanar: diff.maxPlanar,
    maxElevation: diff.maxElevation,
    maxSeamDelta: diff.maxSeamDelta,
    routes: diff.routeDiffs.map((d) => ({
      connectionId: d.connectionId,
      maxPlanar: Number(d.maxPlanar.toFixed(5)),
      rmsPlanar: Number(d.rmsPlanar.toFixed(5)),
      maxElevation: Number(d.maxElevation.toFixed(5)),
    })),
    seams: diff.seamDiffs,
    units: diff.units,
  }, null, 2) + '\n');
  if (!diff.equivalent) process.exitCode = 2;
}

// Simulate a drive across a junction on an open (vehicle-free) edit, reporting
// when traffic can pass and the exact pavement source the wheels adopted.
function cmdOpenJunction(args) {
  const id = args.scene || 'arterial';
  const iface = new RoadGeometryInterface(sampleProject(id));
  const b = iface.built;
  const j = b.junctions[0];
  if (!j) throw new Error('scene has no junction');
  const connIds = b.routes.map((r) => r.conn.id);
  const plan = iface.planEdit(connIds);
  const turns = iface.junctionTurns(j.id);
  const passable = turns.every((t) => t.feasible);

  // Replay a drive from the first route, through the junction node, onto an
  // arm beyond it, so the log records both ribbon and junction-pad sources.
  iface.startDriveSession(b.routes[0].conn.id, null);
  const node = b.nodes.find((n) => n.id === j.node.id);
  const r0 = b.routes[0].samples;
  const start = r0[Math.floor(r0.length * 0.2)];
  const waypoints = [
    start,
    { x: node.x, y: node.y },
    { x: node.x + 18, y: node.y },
  ];
  const steps = 18;
  let time = 0;
  for (let w = 0; w < waypoints.length - 1; w++) {
    const a = waypoints[w];
    const c = waypoints[w + 1];
    const segSteps = 8;
    for (let i = 0; i <= segSteps; i++) {
      const t = i / segSteps;
      iface.recordDriveFrame({
        x: a.x + (c.x - a.x) * t,
        y: a.y + (c.y - a.y) * t,
        time: time++,
        heading: Math.atan2(c.y - a.y, c.x - a.x),
        speed: 6,
      });
    }
  }
  iface.endDriveSession();

  process.stdout.write(JSON.stringify({
    scene: id,
    junction: j.id,
    openToEdit: plan.retained.length === 0,
    removable: plan.removable,
    passable,
    turnCount: turns.length,
    adoptedSurfaces: iface.adoptedSurfaces(),
    driveRecord: iface.driveLog.record(),
  }, null, 2) + '\n');
  if (!passable) process.exitCode = 2;
}

const command = process.argv[2] || 'build';
const args = parseArgs(process.argv.slice(3));
try {
  if (command === 'build') cmdBuild(args);
  else if (command === 'diff') cmdDiff(args);
  else if (command === 'open-junction') cmdOpenJunction(args);
  else {
    process.stderr.write(`unknown command: ${command}\nusage: roadforge [build|diff|open-junction] [--scene id] [--conn id --dx n --dy n]\n`);
    process.exit(1);
  }
} catch (err) {
  process.stderr.write(`roadforge: ${err.message}\n`);
  process.exit(1);
}
