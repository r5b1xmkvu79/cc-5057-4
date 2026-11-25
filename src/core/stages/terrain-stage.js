// Stage 6 — terrain grading and seam verification.
// Builds the graded terrain mesh from the routed corridors and junction pads,
// then measures pavement-edge seams against the actual built surface. Moved
// verbatim from build.js.

import { checkSeams } from './mesh.js';

export function gradeTerrain(input, routes, junctions, controls, surfaceIndex) {
  const { terrain } = input;
  const pads = junctions.map((j) => ({
    x: j.node.x,
    y: j.node.y,
    z: j.z,
    heightAt: j.surface ? (x, y) => j.surface.heightField(x, y) : null,
    radius: Math.max(j.radius, 3) + 3,
    influence: 16,
  }));
  const corridors = routes.map((r) => ({
    influence: r.cls.width * 1.6 + 16,
    cutSlope: r.cls.cutSlope,
    fillSlope: r.cls.fillSlope,
    samples: r.samples
      .filter((s) => s.s >= r.startStation - 0.01 && s.s <= r.endStation + 0.01)
      // Terrain.buildMesh reads x/z as plan coordinates and y as elevation.
      .map((s) => ({ x: s.x, y: s.z, z: s.y, halfPaved: Math.max(s.halfLeft, s.halfRight) + r.cls.shoulder })),
  }));
  const terrainMesh = terrain.buildMesh(controls.resolution, corridors, pads);
  const seams = checkSeams(routes, junctions, terrainMesh, surfaceIndex);
  return { terrainMesh, seams };
}
