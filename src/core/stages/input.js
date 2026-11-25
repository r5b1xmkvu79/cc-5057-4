// Stage 1 — input normalisation.
//
// Pure projection of the authored project record into the kernel's working
// shape: copied nodes, normalized connections and the node->incidence map. No
// solving happens here and no input array is mutated; the routing stage receives
// fresh objects so local edits cannot reach into solver internals.

import { Terrain } from '../terrain.js';
import { snapshotPoints } from '../provenance.js';

export const DEFAULT_CONTROLS = Object.freeze({
  resolution: 240,
  junctionTrimScale: 1.0,
  junctionRadius: 7.5,
  allowReroute: true,
  allowWiden: true,
  widenPreference: 0.8,
  rerouteAttempts: 8,
  widenAttempts: 4,
  taper: 9,
  maxWiden: 5.5,
});

export function normalizeConnection(c, controls) {
  return {
    id: c.id,
    a: c.a,
    b: c.b,
    name: c.name || c.id,
    classId: c.classId || 'local',
    vehicleId: c.vehicleId || null,
    locked: !!c.locked,
    controlPoints: (c.controlPoints || []).map((p) => ({ x: p.x, y: p.y })),
    trimScale: c.trimScale ?? controls.junctionTrimScale,
  };
}

// Returns an immutable-at-boundary input bundle. Nodes and connections are
// shallow-copied; control points are deep-copied via snapshotPoints.
export function normalizeProjectInput(project, opts = {}) {
  const controls = {
    ...DEFAULT_CONTROLS,
    ...(project.controls || {}),
    ...(opts.controls || {}),
  };
  const seed = project.seed ?? 1;
  const terrain = new Terrain(project.terrain || {});

  const nodes = new Map(project.nodes.map((n) => [n.id, { ...n }]));
  const connections = project.connections.map((c) => normalizeConnection(c, controls));
  const connById = new Map(connections.map((c) => [c.id, c]));

  const nodeConnections = new Map();
  for (const n of nodes.values()) nodeConnections.set(n.id, []);
  for (const c of connections) {
    nodeConnections.get(c.a)?.push({ conn: c, end: 'a' });
    nodeConnections.get(c.b)?.push({ conn: c, end: 'b' });
  }

  const junctionZ = new Map();
  for (const n of nodes.values()) {
    if (n.elevation !== undefined && n.elevation !== null) {
      junctionZ.set(n.id, n.elevation);
    } else {
      junctionZ.set(n.id, terrain.heightAt(n.x, n.y));
    }
  }

  return {
    project,
    controls,
    seed,
    terrain,
    nodes,
    connections,
    connById,
    nodeConnections,
    junctionZ,
    // Authored control points, isolated from the caller's arrays.
    authoredControls: new Map(
      project.connections.map((c) => [c.id, snapshotPoints(c.controlPoints || [])]),
    ),
  };
}
