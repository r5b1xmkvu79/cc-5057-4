// Compatibility entry point for the staged road construction kernel.
//
// The implementation now lives in pipeline.js and the stages/ modules. This file
// preserves the historical public surface (buildProject, rebuildWithLocked,
// interiorControl) so existing callers and samples run unchanged. New code
// should prefer the RoadGeometryInterface (geometry-interface.js), which adds
// revision hand-off, edit guarding and drive logging.

import { buildPipeline } from './pipeline.js';

export function buildProject(project, opts = {}) {
  return buildPipeline(project, opts);
}

export function rebuildWithLocked(project, lockedConnectionId, newControls) {
  // Lock one route's resolved interior control points, then resolve the rest
  // again. Endpoints are node positions, not authored controls; storing them
  // would duplicate the node and change the geometry on the next build.
  const next = JSON.parse(JSON.stringify(project));
  const conn = next.connections.find((c) => c.id === lockedConnectionId);
  if (conn && newControls) {
    conn.controlPoints = newControls.map((p) => ({ x: p.x, y: p.y }));
    conn.locked = true;
  }
  return buildProject(next);
}

// Strip the node endpoints from a resolved control polyline so it can be
// re-authored as interior control points.
export function interiorControl(route) {
  const pts = route.control.map((p) => ({ x: p.x, y: p.y }));
  if (pts.length <= 2) return [];
  return pts.slice(1, -1);
}
