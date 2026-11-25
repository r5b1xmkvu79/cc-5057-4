// Stage 4 — route/mesh boundary: pavement union and collision index.
//
// This is the decoupling seam between "the road" (centrelines, ports, ribbons)
// and "the mesh" (triangles, SurfaceIndex). Upstream stages describe geometry;
// this stage is the only place that publishes collidable triangles and stamps
// them with the road revision. Consumers (drive mode, turn tests, seams) ask
// the published SurfaceIndex rather than reaching into ribbons, so a centreline
// recompute cannot silently change what a vehicle is standing on.
//
// Triangle assembly is moved verbatim from build.js's assemblePavement.

import {
  differenceShapes,
  overlapAreaShapes,
  pointInRing,
  shapesArea,
  unionShapes,
} from '../clip.js';
import { triangulatePolygon, trianglesToPositions, SurfaceIndex, resampleRing } from '../surface.js';
import { STAGES, tagProvenance } from '../provenance.js';

function buildCoverageIndex(triangles) {
  return new SurfaceIndex(triangles, 4);
}

function planeZ(tri) {
  const [a, b, c] = tri;
  const det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
  if (Math.abs(det) < 1e-9) {
    const z = (a.z + b.z + c.z) / 3;
    return () => z;
  }
  return (x, y) => {
    const l1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / det;
    const l2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / det;
    return l1 * a.z + l2 * b.z + (1 - l1 - l2) * c.z;
  };
}

function findUncoveredPatches(ring, holes, coverage, step) {
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (const p of ring) {
    minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
  }
  const cell = Math.max(step, (maxX - minX) / 60);
  const cells = [];
  for (let x = minX; x <= maxX; x += cell) {
    for (let y = minY; y <= maxY; y += cell) {
      if (!pointInRing({ x, y }, ring)) continue;
      if (holes.some((h) => pointInRing({ x, y }, h))) continue;
      if (coverage.heightAt(x, y) !== null) continue;
      cells.push({ x, y });
    }
  }
  if (!cells.length) return [];
  const patches = [];
  for (const c of cells) {
    patches.push({
      outer: [
        { x: c.x - cell / 2, y: c.y - cell / 2 },
        { x: c.x + cell / 2, y: c.y - cell / 2 },
        { x: c.x + cell / 2, y: c.y + cell / 2 },
        { x: c.x - cell / 2, y: c.y + cell / 2 },
      ],
      holes: [],
    });
  }
  return patches;
}

function surfaceHeightAt(x, y, routes, junctions) {
  for (const j of junctions) {
    if (!j.surface) continue;
    if (pointInRing({ x, y }, j.surface.ring)) return j.surface.heightField(x, y);
  }
  for (const r of routes) {
    const h = ribbonHeightAt(r, x, y);
    if (h !== null) return h;
  }
  let best = null;
  let bestD = Infinity;
  for (const j of junctions) {
    if (!j.surface) continue;
    const d = Math.hypot(x - j.node.x, y - j.node.y);
    if (d < bestD) {
      bestD = d;
      best = j.surface.heightField(x, y);
    }
  }
  for (const r of routes) {
    const stations = r.ribbon.stations;
    for (const st of stations) {
      const d = Math.hypot(x - st.x, y - st.y);
      if (d < bestD) {
        bestD = d;
        best = st.z;
      }
    }
  }
  return best ?? 0;
}

function ribbonHeightAt(r, x, y) {
  const stations = r.ribbon.stations;
  if (!stations.length) return null;
  let best = null;
  let bestD = Infinity;
  for (let i = 0; i < stations.length; i++) {
    const st = stations[i];
    const dx = x - st.x;
    const dy = y - st.y;
    const lat = dx * st.nx + dy * st.ny;
    const along = Math.abs(dx * st.tx + dy * st.ty);
    if (along < 2.5 && Math.abs(lat) <= (lat >= 0 ? st.halfLeft : st.halfRight) + 0.05) {
      const d = along * along + (Math.abs(lat) - st.baseHalf) ** 2;
      if (d < bestD) {
        bestD = d;
        best = st.z;
      }
    }
  }
  return best;
}

export function assemblePavement(routes, junctions) {
  const shapes = [];
  for (const r of routes) {
    if (r.ribbon && r.ribbon.ring.length >= 3) shapes.push({ outer: r.ribbon.ring, holes: [] });
  }
  for (const j of junctions) {
    if (j.surface && j.surface.ring.length >= 3) shapes.push({ outer: j.surface.ring, holes: [] });
  }
  const allShapes = unionShapes(shapes);
  const unionRings = allShapes.map((s) => s.outer);

  const markingTriangles = [];
  const shoulderTriangles = [];
  for (const r of routes) {
    markingTriangles.push(...r.ribbon.markingTriangles);
    shoulderTriangles.push(...r.ribbon.shoulderTriangles);
  }

  const colliderTriangles = [];
  const pavementTriangles = [];

  const padShapes = [];
  for (const j of junctions) {
    if (!j.surface) continue;
    padShapes.push({ j, shape: { outer: j.surface.ring, holes: [] } });
  }
  const padPavement = [];
  for (let pi = 0; pi < padShapes.length; pi++) {
    let shape = padShapes[pi].shape;
    for (let qi = 0; qi < padShapes.length; qi++) {
      if (qi === pi) continue;
      const other = padShapes[qi];
      if (other.j.z <= padShapes[pi].j.z) continue;
      const cut = differenceShapes([shape], [other.shape]);
      if (cut.length) shape = cut.length === 1 ? cut[0] : unionShapes(cut)[0];
    }
    const ring = resampleRing(shape.outer, 2.2);
    const holes = shape.holes.map((h) => resampleRing(h, 2.2));
    const res = triangulatePolygon(ring, padShapes[pi].j.surface.heightField, 2.2, holes);
    for (const tri of res.triangles) {
      pavementTriangles.push(tri);
      colliderTriangles.push(tri);
      padPavement.push(tri);
    }
  }
  const padCoverage = buildCoverageIndex(padPavement);

  const padRings = padShapes.map((p) => p.shape);
  for (const r of routes) {
    for (const tri of r.ribbon.pavementTriangles) {
      let pieces = [{ outer: tri.map((v) => ({ x: v.x, y: v.y })), holes: [] }];
      const cx = (tri[0].x + tri[1].x + tri[2].x) / 3;
      const cy = (tri[0].y + tri[1].y + tri[2].y) / 3;
      const onPad = padCoverage.heightAt(cx, cy) !== null;
      if (onPad && padRings.length) {
        const cut = differenceShapes(pieces, padRings);
        if (!cut.length) continue;
        pieces = cut;
      }
      const plane = planeZ(tri);
      for (const piece of pieces) {
        const ring = resampleRing(piece.outer, 2.2);
        if (ring.length < 3) continue;
        const holes = piece.holes.map((h) => resampleRing(h, 2.2));
        const res = triangulatePolygon(ring, plane, 2.2, holes);
        for (const t of res.triangles) {
          pavementTriangles.push(t);
          colliderTriangles.push(t);
        }
      }
    }
  }

  const coverage = buildCoverageIndex(pavementTriangles);
  for (const shape of allShapes) {
    const ring = resampleRing(shape.outer, 3.0);
    const holes = shape.holes.map((h) => resampleRing(h, 3.0));
    const gaps = findUncoveredPatches(ring, holes, coverage, 3.0);
    for (const patch of gaps) {
      const res = triangulatePolygon(patch.outer, (x, y) => surfaceHeightAt(x, y, routes, junctions), 3.0, patch.holes.map((h) => resampleRing(h, 3.0)));
      for (const tri of res.triangles) {
        pavementTriangles.push(tri);
        colliderTriangles.push(tri);
      }
    }
  }

  const pavement = {
    shapes: allShapes,
    pavementTriangles,
    markingTriangles,
    shoulderTriangles,
    collisionTriangles: colliderTriangles,
    outline: unionRings,
    positions: trianglesToPositions(pavementTriangles),
  };
  tagProvenance(pavement, { stage: STAGES.MESH, source: 'pavement-union' });
  return pavement;
}

// Build the collision index for a specific road revision. Throws via the ledger
// if a caller tries to publish a mesh older than the current centreline.
export function publishCollisionMesh(pavement, ledger, roadRevision) {
  ledger.publishMesh(roadRevision);
  const surfaceIndex = new SurfaceIndex(pavement.collisionTriangles);
  tagProvenance(pavement, {
    stage: STAGES.MESH,
    source: 'collision-index',
    revision: roadRevision,
  });
  return surfaceIndex;
}

export function findOverlaps(routes, junctions) {
  const issues = [];
  for (let i = 0; i < routes.length; i++) {
    for (let j = i + 1; j < routes.length; j++) {
      const a = routes[i];
      const b = routes[j];
      if (!a.ribbon.ring.length || !b.ribbon.ring.length) continue;
      const sharedJunction = junctions.find(
        (jj) =>
          (jj.node.id === a.conn.a || jj.node.id === a.conn.b) &&
          (jj.node.id === b.conn.a || jj.node.id === b.conn.b),
      );
      const shapesA = [{ outer: a.ribbon.ring, holes: [] }];
      const shapesB = [{ outer: b.ribbon.ring, holes: [] }];
      const overlap = overlapAreaShapes(shapesA, shapesB);
      if (overlap > 0.5) {
        if (!sharedJunction) {
          issues.push({
            code: 'pavement-overlap',
            severity: 'error',
            message: `路面重叠 ${overlap.toFixed(1)}m²（${a.conn.id} × ${b.conn.id}）且无共享路口`,
            value: overlap,
            limit: 0.5,
            plan: null,
            connectionId: a.conn.id,
          });
        }
      }
    }
  }
  return issues;
}

export function checkSeams(routes, junctions, terrainMesh, surfaceIndex) {
  const seams = [];
  for (const r of routes) {
    const st = r.ribbon.stations;
    for (let i = 0; i < st.length; i += Math.max(1, Math.floor(st.length / 20))) {
      const q = st[i];
      for (const side of ['left', 'right']) {
        const half = side === 'left' ? q.halfLeft : q.halfRight;
        const nx = q.nx * (side === 'left' ? 1 : -1);
        const ny = q.ny * (side === 'left' ? 1 : -1);
        const edge = { x: q.x + nx * half, y: q.y + ny * half };
        const ground = terrainMesh.sampleHeight(edge.x, edge.y);
        const paved = surfaceIndex ? surfaceIndex.heightAt(edge.x, edge.y) : null;
        const gap = (paved != null ? paved : q.z) - ground;
        if (gap > 0.4) {
          seams.push({
            code: 'seam-gap',
            severity: gap > 0.8 ? 'error' : 'warn',
            message: `路面边缘悬空 ${gap.toFixed(2)}m（填方未到达路缘）`,
            value: gap,
            plan: edge,
            connectionId: r.conn.id,
            station: q.s,
          });
          break;
        }
        if (gap < -0.6) {
          seams.push({
            code: 'seam-cut',
            severity: 'warn',
            message: `地形切入路缘 ${(-gap).toFixed(2)}m（挖方坡脚）`,
            value: gap,
            plan: edge,
            connectionId: r.conn.id,
            station: q.s,
          });
          break;
        }
      }
    }
  }
  return seams;
}
