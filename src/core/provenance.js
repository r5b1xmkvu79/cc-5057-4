// Provenance and measurement metadata for geometry produced by the kernel.
//
// Every geometric object that crosses a stage boundary carries a `meta`
// descriptor: which stage and source record produced it, the build revision it
// belongs to, and the units of each coordinate. Local edits never mutate solver
// arrays directly; they create a new revision, so every geometry stays
// traceable to the inputs that made it.
//
// Units are fixed across the kernel:
//   plan coordinates x, y : metre (m)
//   elevation z            : metre (m)
//   station s              : metre (m) along the centreline
//   heading / angles       : radian (rad)
//   grade / curvature      : ratio (1) and 1/metre (1/m)

export const UNITS = Object.freeze({
  plan: 'm',
  elevation: 'm',
  station: 'm',
  angle: 'rad',
  grade: 'ratio',
  curvature: '1/m',
  area: 'm^2',
});

export const STAGES = Object.freeze({
  INPUT: 'input-normalize',
  ROUTING: 'centreline-routing',
  JUNCTION: 'junction-connect',
  MESH: 'route-mesh',
  ACCEPTANCE: 'acceptance',
  TERRAIN: 'terrain-grade',
});

let nextRevision = 1;

// Monotonic revision for a build graph. A fresh graph starts at 1; each centre
// recompute bumps the revision and the collision mesh can only be published for
// the revision it was built from.
export function createRevision(initial = 0) {
  let revision = initial;
  return {
    current() {
      return revision;
    },
    bump() {
      revision += 1;
      return revision;
    },
  };
}

// A process-wide unique id keeps revisions from separate graphs distinct even
// when their numeric revision matches.
export function uniqueGraphId() {
  const v = nextRevision;
  nextRevision += 1;
  return `graph-${Date.now().toString(36)}-${v}`;
}

// Attach provenance to a geometry object without copying its payload. Existing
// callers keep reading the same fields; the descriptor is additive. Re-tagging
// the same target (e.g. stamping the graph id once a route joins a build graph)
// replaces the descriptor but never touches the geometry payload.
export function tagProvenance(target, { stage, source, revision, graphId, note } = {}) {
  if (!target || typeof target !== 'object') return target;
  const descriptor = Object.freeze({
    stage: stage || null,
    source: source || null,
    revision: revision ?? null,
    graphId: graphId || null,
    units: UNITS,
    note: note || null,
    createdAt: new Date().toISOString(),
  });
  Object.defineProperty(target, 'meta', {
    value: descriptor,
    enumerable: true,
    configurable: true,
    writable: false,
  });
  return target;
}

// Plain (JSON-safe) copy of the descriptor, used by the build record.
export function provenanceRecord(target) {
  const m = target?.meta;
  if (!m) return null;
  return {
    stage: m.stage,
    source: m.source,
    revision: m.revision,
    graphId: m.graphId,
    units: { ...m.units },
    note: m.note,
    createdAt: m.createdAt,
  };
}

// Snapshot a centreline sample array. Routing works on the copy so an edit to
// the authored control points can never reach into a previously published
// route's internal arrays.
export function snapshotSamples(samples) {
  return samples.map((s) => ({ ...s }));
}

export function snapshotPoints(points) {
  return points.map((p) => ({ x: p.x, y: p.y }));
}
