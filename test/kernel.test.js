import test from 'node:test';
import assert from 'node:assert/strict';
import { buildProject, rebuildWithLocked, interiorControl } from '../src/core/build.js';
import { sampleProject, SAMPLE_IDS } from '../src/project.js';
import { parkingTurnRadius } from '../src/core/standards.js';

const cache = new Map();
function build(id) {
  if (!cache.has(id)) cache.set(id, buildProject(sampleProject(id)));
  return cache.get(id);
}

test('samples build with no errors and every turn passes', () => {
  for (const id of SAMPLE_IDS) {
    const r = build(id);
    const errs = r.violations.filter((v) => v.severity === 'error');
    assert.deepEqual(
      errs.map((v) => v.code),
      [],
      `${id}: ${errs.map((v) => v.message).join('; ')}`,
    );
    assert.ok(r.turnTests.length > 0, `${id}: no turn tests`);
    for (const t of r.turnTests) {
      assert.ok(t.pass, `${id} ${t.junctionId} ${t.from.connectionId}->${t.to.connectionId} off=${t.offPavement} steer=${t.maxSteerDeg}`);
      assert.equal(t.offPavement, 0);
      // Step, not total spread: a truck on a grade is allowed a tall spread.
      assert.ok(t.maxHeightStep <= 0.3, `${id} step ${t.maxHeightStep}`);
    }
  }
});

test('every connection produces a ribbon and pavement covers it', () => {
  for (const id of SAMPLE_IDS) {
    const r = build(id);
    for (const route of r.routes) {
      assert.ok(route.ribbon.ring.length >= 3, `${id}/${route.conn.id} ring`);
      assert.ok(route.ribbon.pavementTriangles.length > 0);
    }
    assert.ok(r.pavement.collisionTriangles.length > 0);
  }
});

test('junctions are real surfaces, not overlapping ribbons', () => {
  for (const id of SAMPLE_IDS) {
    const r = build(id);
    assert.ok(r.junctionReports.length > 0, `${id}: no junctions`);
    for (const j of r.junctionReports) {
      assert.ok(j.triangles > 0, `${id}/${j.id}: no pad triangles`);
      assert.ok(j.area > 0, `${id}/${j.id}: no area`);
      assert.deepEqual(j.issues, [], `${id}/${j.id}: ${j.issues}`);
      assert.ok(j.maxGrade <= 0.1, `${id}/${j.id}: pad grade ${j.maxGrade}`);
    }
  }
});

test('pavement is single-valued and continuous for collision', () => {
  const r = build('arterial');
  const route = r.routes.find((x) => x.conn.id === 'trunk-west');
  for (let i = 0; i < route.samples.length; i += 3) {
    const s = route.samples[i];
    const h = r.surfaceIndex.heightAt(s.x, s.y);
    assert.ok(h !== null, `no pavement at ${s.x},${s.y}`);
    assert.ok(Math.abs(h - s.z) < 0.5, `collision height ${h} vs road ${s.z} at s=${s.s}`);
  }
});

test('rebuild is deterministic for the same seed and controls', () => {
  const a = buildProject(sampleProject('arterial'));
  const b = buildProject(sampleProject('arterial'));
  assert.equal(a.pavementArea, b.pavementArea);
  assert.equal(a.pavement.pavementTriangles.length, b.pavement.pavementTriangles.length);
  assert.equal(a.record.seed, b.record.seed);
  assert.deepEqual(a.record.controls, b.record.controls);
});

test('build record captures seed, controls and resolved control points', () => {
  const r = build('arterial');
  assert.equal(r.record.seed, 20260401);
  assert.equal(r.record.vehicleId, 'sedan');
  assert.ok(r.record.controls.resolution > 0);
  assert.ok(r.record.fixedControls['branch-north'].length >= 1);
  assert.ok(r.record.junctions.length > 0);
  assert.ok(r.record.turns.length > 0);
});

test('locking one connection pins its control points while others rebuild', () => {
  const project = sampleProject('arterial');
  const first = buildProject(project);
  const branch = first.routes.find((x) => x.conn.id === 'branch-north');
  // Locked controls are interior points; the node endpoints stay implicit.
  const lockedControl = interiorControl(branch);
  assert.ok(lockedControl.length >= 1);
  const next = rebuildWithLocked(project, 'branch-north', lockedControl);
  const branch2 = next.routes.find((x) => x.conn.id === 'branch-north');
  assert.ok(branch2.conn.locked);
  assert.deepEqual(
    branch2.control.slice(1, -1).map((p) => ({ x: p.x, y: p.y })),
    lockedControl,
  );
  // Other routes are still resolved.
  assert.ok(next.routes.every((x) => x.ribbon.ring.length >= 3));
  assert.equal(next.errorCount, 0);
});

test('parking turn radius contains the tightest turn for every vehicle', () => {
  // The pad must contain the outside front corner, not just the rear axle.
  for (const [id, expected] of [['sedan', 6], ['boxTruck', 12], ['bus', 12]]) {
    assert.ok(parkingTurnRadius(id) >= expected, `${id} radius ${parkingTurnRadius(id)}`);
  }
});
