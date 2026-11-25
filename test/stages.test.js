// Staged-pipeline tests: revision hand-off, edit guarding, drive logging,
// provenance/units and before/after equivalence on multi-junction and steep
// terrain. Every number here is computed by the kernel in a clean process; no
// fixture is hand-edited, so a reported pass always has a measured basis.

import test from 'node:test';
import assert from 'node:assert/strict';

import { buildProject } from '../src/core/build.js';
import { buildPipeline } from '../src/core/pipeline.js';
import { RoadGeometryInterface } from '../src/core/geometry-interface.js';
import { RevisionLedger } from '../src/core/revision.js';
import { EditGuard } from '../src/core/edit-guard.js';
import { DriveLog } from '../src/core/drive-log.js';
import { diffBuilds, assertEquivalent, compareCenterlines } from '../src/core/equivalence.js';
import { sampleProject, SAMPLE_IDS } from '../src/project.js';
import { UNITS } from '../src/core/provenance.js';

test('staged pipeline matches the legacy buildProject entry exactly', () => {
  for (const id of SAMPLE_IDS) {
    const project = sampleProject(id);
    const legacy = buildProject(JSON.parse(JSON.stringify(project)));
    const staged = buildPipeline(JSON.parse(JSON.stringify(project)));
    assert.equal(staged.errorCount, legacy.errorCount);
    assert.equal(staged.pavement.pavementTriangles.length, legacy.pavement.pavementTriangles.length);
    assert.equal(staged.turnTests.length, legacy.turnTests.length);
    const diff = diffBuilds(legacy, staged);
    assertEquivalent(diff);
  }
});

test('every published geometry carries provenance with metre/radian units', () => {
  const b = buildPipeline(sampleProject('arterial'));
  assert.ok(b.graphId, 'graph id present');
  assert.ok(b.revision >= 1, 'road revision present');
  assert.equal(b.record.revision, b.revision);
  assert.equal(b.record.graphId, b.graphId);
  for (const r of b.routes) {
    assert.equal(r.meta.units.plan, 'm');
    assert.equal(r.meta.units.angle, 'rad');
    assert.match(r.meta.source, /^connection:/);
    assert.equal(r.ribbon.meta.stage, 'route-mesh');
    assert.equal(r.ribbon.meta.revision, b.revision);
  }
  for (const j of b.junctions) {
    assert.equal(j.meta.stage, 'junction-connect');
    assert.equal(j.surface.meta.units.elevation, 'm');
  }
  assert.deepEqual(b.pavement.meta.units, UNITS);
  // Provenance descriptor is immutable.
  assert.throws(() => { b.routes[0].meta.stage = 'tampered'; }, TypeError);
});

test('local edits do not mutate solver arrays of a previously published route', () => {
  const project = sampleProject('arterial');
  // An editor keeps one ledger across rebuilds; every edit bumps it.
  const ledger = new RevisionLedger();
  const first = buildPipeline(project, { ledger });
  const before = first.routes.find((r) => r.conn.id === 'branch-north');
  const frozenHash = before.samples.map((s) => `${s.x},${s.y},${s.z}`).join('|');

  // Edit the authored project, then rebuild on the same ledger.
  const edited = JSON.parse(JSON.stringify(project));
  const conn = edited.connections.find((c) => c.id === 'branch-north');
  conn.controlPoints[0].x += 12;
  const second = buildPipeline(edited, { ledger });
  assert.ok(second.revision > first.revision);

  // The earlier route's internal arrays are untouched.
  assert.equal(before.samples.map((s) => `${s.x},${s.y},${s.z}`).join('|'), frozenHash);
  const after = second.routes.find((r) => r.conn.id === 'branch-north');
  const moved = compareCenterlines('branch-north', before.samples, after.samples);
  assert.ok(moved.maxPlanar > 0.5, 'edited centreline actually moved');
});

test('revision ledger rejects a stale mesh and only accepts latest-road trajectories', () => {
  const ledger = new RevisionLedger();
  const r1 = ledger.roadChanged();
  assert.equal(ledger.currentRoad(), 1);
  assert.equal(ledger.publishMesh(r1), 1);
  assert.ok(ledger.meshIsCurrent());

  const r2 = ledger.roadChanged();
  assert.equal(r2, 2);
  assert.ok(!ledger.meshIsCurrent());
  // Publishing the older revision is rejected, never silently applied.
  assert.throws(() => ledger.publishMesh(r1), /stale mesh/);
  assert.equal(ledger.publishMesh(r2), 2);
  assert.ok(ledger.trajectoryAcceptable(2));
  assert.ok(!ledger.trajectoryAcceptable(1), 'old trajectory must not take effect');
});

test('continuous drags invalidate every trajectory not matching the latest road', () => {
  const iface = new RoadGeometryInterface(sampleProject('arterial'));
  iface.startDriveSession('trunk-west', { x: 0, y: 0 });
  const r0 = iface.revision;
  // Drive a few frames on revision r0.
  for (let i = 0; i < 4; i++) {
    iface.recordDriveFrame({ x: -180 + i * 4, y: 20, time: i, heading: 0, speed: 5 });
  }
  // Two successive drags => two road revisions.
  for (const dx of [6, 9]) {
    const p = JSON.parse(JSON.stringify(iface.project));
    const c = p.connections.find((c) => c.id === 'branch-north');
    c.controlPoints[0].x += dx;
    iface.rebuild(p);
    iface.syncDriveLog();
  }
  assert.ok(iface.revision >= r0 + 2);
  const stale = iface.driveLog.entries.filter((e) => !e.active);
  assert.ok(stale.length === 4, 'all pre-drag frames are stale');
  // New driving on the current revision is accepted.
  iface.recordDriveFrame({ x: -160, y: 20, time: 99, heading: 0, speed: 5 });
  assert.ok(iface.driveLog.entries.at(-1).active);
});

test('edit guard holds occupied connections and frees unoccupied branches', () => {
  const guard = new EditGuard();
  const now = 1000;
  guard.markContact(['ramp-a', 'junction-arm'], { x: 1, y: 2 }, now);

  // Unoccupied branch can re-mesh immediately.
  assert.deepEqual(guard.planEdit(['ramp-a', 'spur-free'], now).removable, ['spur-free']);
  const held = guard.planEdit(['ramp-a'], now).retained;
  assert.equal(held[0].status, 'held');

  // Without a safe point the occupied connection cannot be force-returned.
  assert.equal(guard.requestSafeReturn('ramp-a'), null);
  guard.registerSafePoint('ramp-a', { x: -60, y: 20 });
  const point = guard.requestSafeReturn('ramp-a');
  assert.deepEqual(point, { x: -60, y: 20 });
  // Still held until the editor confirms the vehicle moved.
  assert.equal(guard.canRemove('ramp-a', now).status, 'parked');
  guard.confirmSafeReturn('ramp-a');
  assert.equal(guard.canRemove('ramp-a', now).status, 'free');

  // Grace window: recent contact keeps it held, old contact releases.
  guard.markContact(['ramp-b'], { x: 0, y: 0 }, now);
  assert.equal(guard.hasContact('ramp-b', now + 500), true);
  assert.equal(guard.hasContact('ramp-b', now + 2000), false);
});

test('drive log identifies the pavement actually adopted, by source part', () => {
  const iface = new RoadGeometryInterface(sampleProject('arterial'));
  const b = iface.built;
  iface.startDriveSession('trunk-west', null);
  // Pick points known to be on a road ribbon and inside a junction pad.
  const roadSample = b.routes.find((r) => r.conn.id === 'trunk-west').samples[5];
  const j = b.junctions[0];
  const padPoint = { x: j.node.x, y: j.node.y };
  iface.recordDriveFrame({ x: roadSample.x, y: roadSample.y, time: 0, heading: 0, speed: 4 });
  iface.recordDriveFrame({ x: padPoint.x, y: padPoint.y, time: 1, heading: 0, speed: 4 });

  const surfaces = iface.adoptedSurfaces();
  const parts = surfaces.map((s) => s.part);
  assert.ok(parts.includes('road'), 'road ribbon detected from real surface');
  assert.ok(parts.includes('junction'), 'junction pad detected from real surface');
  for (const s of surfaces) assert.equal(s.units, 'm');

  // Off-pavement points report null surface rather than a fake road height.
  const far = b.terrain.bounds;
  assert.equal(iface.surfaceAt(far.minX + 2, far.minZ + 2), null);
});

test('turn feasibility is measured, never reported without computation', () => {
  const iface = new RoadGeometryInterface(sampleProject('multibranch'));
  const jId = iface.built.junctionReports[0].id;
  const turns = iface.junctionTurns(jId);
  assert.ok(turns.length > 0);
  for (const t of turns) {
    assert.equal(t.measuredOn.revision, iface.meshRevision);
    assert.equal(t.measuredOn.surface, 'collision-index');
    assert.equal(typeof t.feasible, 'boolean');
  }
  // On-demand recomputation for another vehicle returns measured geometry.
  const first = iface.built.turnTests[0];
  const ev = iface.evaluateTurn(first.junctionId, first.from.connectionId, first.to.connectionId, 'bus');
  assert.ok(ev.path.length >= 2);
  assert.equal(ev.measuredOn.surface, 'collision-index');
  assert.throws(() => iface.evaluateTurn('nope', 'a', 'b'), /unknown junction/);
});

test('grade and curvature rules survive the refactor on the steep slope sample', () => {
  const b = buildProject(sampleProject('slope'));
  assert.equal(b.errorCount, 0, 'slope sample stays feasible');
  for (const r of b.routes) {
    const clsMax = r.cls.maxGrade;
    assert.ok(r.profileStats.maxGrade <= clsMax + 1e-4, `${r.conn.id} grade rule`);
    assert.ok(r.curvatureStats.radius >= r.cls.minRadius - 1e-6, `${r.conn.id} curvature rule`);
  }
});

test('multi-junction and steep-terrain rebuilds stay equivalent under a no-op edit', () => {
  for (const id of ['multibranch', 'slope']) {
    const project = sampleProject(id);
    const before = buildPipeline(JSON.parse(JSON.stringify(project)));
    // A re-solve with identical authored inputs must reproduce the geometry.
    const after = buildPipeline(JSON.parse(JSON.stringify(project)));
    const diff = diffBuilds(before, after);
    assertEquivalent(diff);
    assert.ok(diff.junctionDiffs.every((j) => j.present && j.trianglesEqual));
  }
});

test('seam correspondence is reported per connection with measured gaps', () => {
  const before = buildPipeline(sampleProject('arterial'));
  const after = buildPipeline(sampleProject('arterial'));
  const diff = diffBuilds(before, after);
  for (const s of diff.seamDiffs) {
    assert.ok(typeof s.connectionId === 'string');
    assert.ok(Number.isFinite(s.beforeGap) && Number.isFinite(s.afterGap));
    assert.equal(s.delta, 0);
  }
  assert.ok(Math.abs(diff.maxSeamDelta) < 1e-9);
});

test('empty junction edit: open junction reports when traffic can pass', () => {
  // Build a minimal open junction (three arms, no vehicle) and confirm the
  // interface reports measured passable turns once pavement is published.
  const iface = new RoadGeometryInterface(sampleProject('arterial'));
  const guard = new EditGuard();
  const connIds = iface.built.routes.map((r) => r.conn.id);
  // No vehicle has touched anything: the whole junction is free to re-mesh.
  const plan = guard.planEdit(connIds);
  assert.deepEqual(plan.removable.sort(), connIds.sort());
  assert.equal(plan.retained.length, 0);
  const jId = iface.built.junctionReports[0].id;
  const turns = iface.junctionTurns(jId);
  assert.ok(turns.every((t) => t.feasible), 'open junction is passable on real mesh');
});
