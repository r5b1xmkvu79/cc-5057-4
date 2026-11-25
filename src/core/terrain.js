// Procedural terrain: height sampling, features, forbidden zones and graded mesh.
// Plan coordinates are {x, y}; the world transform maps plan y -> world Z.

import {
  clamp,
  lerp,
  smoothstep,
  smootherstep,
  dist,
  closestPointOnSegment,
  pointInPolygon,
  simplify,
} from './math.js';

function hash2(ix, iy, seed) {
  let h = seed ^ Math.imul(ix, 374761393) ^ Math.imul(iy, 668265263);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

export function valueNoise(x, y, seed) {
  const ix = Math.floor(x);
  const iy = Math.floor(y);
  const ux = smootherstep(x - ix);
  const uy = smootherstep(y - iy);
  const a = hash2(ix, iy, seed);
  const b = hash2(ix + 1, iy, seed);
  const c = hash2(ix, iy + 1, seed);
  const d = hash2(ix + 1, iy + 1, seed);
  return lerp(lerp(a, b, ux), lerp(c, d, ux), uy) * 2 - 1;
}

export function fbm(x, y, seed, octaves = 4, lacunarity = 2.03, gain = 0.5) {
  let sum = 0;
  let amp = 1;
  let norm = 0;
  let fx = x;
  let fy = y;
  for (let i = 0; i < octaves; i++) {
    sum += valueNoise(fx, fy, seed + i * 1013) * amp;
    norm += amp;
    amp *= gain;
    fx *= lacunarity;
    fy *= lacunarity;
  }
  return sum / norm;
}

export const DEFAULT_TERRAIN = {
  id: 'terrain',
  seed: 1337,
  baseHeight: 0,
  roughness: 2.2,
  noiseScale: 120,
  features: [],
  obstacles: [],
  bounds: { minX: -320, maxX: 320, minZ: -320, maxZ: 320 },
};

export class Terrain {
  constructor(spec = {}) {
    const s = { ...DEFAULT_TERRAIN, ...spec };
    this.id = s.id;
    this.seed = s.seed;
    this.baseHeight = s.baseHeight;
    this.roughness = s.roughness;
    this.noiseScale = s.noiseScale;
    this.features = s.features || [];
    this.obstacles = (s.obstacles || []).map((o, i) => normalizeObstacle(o, i));
    this.bounds = s.bounds;
  }

  heightAt(x, z) {
    let h = this.baseHeight;
    h += fbm(x / this.noiseScale, z / this.noiseScale, this.seed, 4) * this.roughness;
    h += fbm(x / (this.noiseScale * 0.31) + 11.7, z / (this.noiseScale * 0.31) - 4.2, this.seed, 3) * this.roughness * 0.45;
    for (const f of this.features) h += featureHeight(f, x, z);
    return h;
  }

  gradientAt(x, z, h = 1.0) {
    const hx = (this.heightAt(x + h, z) - this.heightAt(x - h, z)) / (2 * h);
    const hz = (this.heightAt(x, z + h) - this.heightAt(x, z - h)) / (2 * h);
    return { x: hx, z: hz };
  }

  slopeAt(x, z) {
    const g = this.gradientAt(x, z);
    return Math.hypot(g.x, g.z);
  }

  inBounds(x, z, pad = 0) {
    const b = this.bounds;
    return x >= b.minX + pad && x <= b.maxX - pad && z >= b.minZ + pad && z <= b.maxZ - pad;
  }

  // Obstacles whose interior or buffer contains `p`.
  forbidden(p, extra = 0) {
    const hits = [];
    for (const o of this.obstacles) {
      if (o.shape === 'circle') {
        if (Math.hypot(p.x - o.x, p.y - o.y) <= o.radius + extra) hits.push(o);
      } else if (o.shape === 'polygon') {
        if (pointInPolygon(p, o.points)) {
          hits.push(o);
          continue;
        }
        for (let i = 0; i < o.points.length; i++) {
          const a = o.points[i];
          const b = o.points[(i + 1) % o.points.length];
          if (dist(p, closestPointOnSegment(p, a, b).point) <= extra) {
            hits.push(o);
            break;
          }
        }
      }
    }
    return hits;
  }

  // Positive distance outside an obstacle, negative inside. Infinity if none.
  obstacleClearance(p) {
    let best = Infinity;
    for (const o of this.obstacles) {
      if (o.shape === 'circle') {
        best = Math.min(best, Math.hypot(p.x - o.x, p.y - o.y) - o.radius);
      } else if (o.shape === 'polygon') {
        let d = Infinity;
        for (let i = 0; i < o.points.length; i++) {
          const a = o.points[i];
          const b = o.points[(i + 1) % o.points.length];
          d = Math.min(d, dist(p, closestPointOnSegment(p, a, b).point));
        }
        if (pointInPolygon(p, o.points)) d = -d;
        best = Math.min(best, d);
      }
    }
    return best;
  }

  // Approximate maximum terrain grade along a straight span, in rise/run.
  spanGrade(a, b, step = 1.5) {
    const L = dist(a, b);
    const n = Math.max(2, Math.ceil(L / step));
    let worst = 0;
    let prev = this.heightAt(a.x, a.y);
    for (let i = 1; i <= n; i++) {
      const t = i / n;
      const x = lerp(a.x, b.x, t);
      const z = lerp(a.y, b.y, t);
      const h = this.heightAt(x, z);
      worst = Math.max(worst, Math.abs(h - prev) / (L / n));
      prev = h;
    }
    return worst;
  }

  buildMesh(resolution = 200, corridors = [], pads = []) {
    const b = this.bounds;
    const nx = resolution + 1;
    const nz = resolution + 1;
    const stepX = (b.maxX - b.minX) / resolution;
    const stepZ = (b.maxZ - b.minZ) / resolution;
    const heights = new Float32Array(nx * nz);
    const index = new Map();

    for (const c of corridors) {
      for (const s of c.samples) {
        const ci = Math.floor((s.x - b.minX) / 12);
        const cj = Math.floor((s.z - b.minZ) / 12);
        for (let i = ci - 1; i <= ci + 1; i++) {
          for (let j = cj - 1; j <= cj + 1; j++) {
            const k = `${i},${j}`;
            let arr = index.get(k);
            if (!arr) index.set(k, (arr = []));
            arr.push({ s, c });
          }
        }
      }
    }
    for (const p of pads) {
      // Index every cell the pad's shelter can reach, otherwise a large pad is
      // missed by the 3x3 neighbourhood lookup and terrain pokes through.
      const reach = (p.radius ?? 0) + (p.influence ?? 14) + 4;
      const ci0 = Math.floor((p.x - reach - b.minX) / 12);
      const ci1 = Math.floor((p.x + reach - b.minX) / 12);
      const cj0 = Math.floor((p.y - reach - b.minZ) / 12);
      const cj1 = Math.floor((p.y + reach - b.minZ) / 12);
      for (let i = ci0; i <= ci1; i++) {
        for (let j = cj0; j <= cj1; j++) {
          const k = `${i},${j}`;
          let arr = index.get(k);
          if (!arr) index.set(k, (arr = []));
          arr.push({ pad: p });
        }
      }
    }

    const nearby = (x, z, radius) => {
      const out = [];
      const ci = Math.floor((x - b.minX) / 12);
      const cj = Math.floor((z - b.minZ) / 12);
      for (let i = ci - 1; i <= ci + 1; i++) {
        for (let j = cj - 1; j <= cj + 1; j++) {
          const arr = index.get(`${i},${j}`);
          if (!arr) continue;
          for (const e of arr) {
            if (e.s && Math.abs(e.s.x - x) < radius && Math.abs(e.s.z - z) < radius) out.push(e);
            else if (e.pad) out.push(e);
          }
        }
      }
      return out;
    };

    for (let gy = 0; gy < nz; gy++) {
      const z = lerp(b.minZ, b.maxZ, gy / resolution);
      for (let gx = 0; gx < nx; gx++) {
        const x = lerp(b.minX, b.maxX, gx / resolution);
        const base = this.heightAt(x, z);
        let h = base;
        const near = nearby(x, z, 40);
        // Pick the closest road sample (grade from a single nearest source to avoid
        // two corridors fighting over the same vertex).
        let best = null;
        let bestD = Infinity;
        for (const e of near) {
          if (!e.s) continue;
          const d = Math.hypot(x - e.s.x, z - e.s.z);
          if (d < bestD) {
            bestD = d;
            best = e;
          }
        }
        let inRoadShield = false;
        if (best) {
          const roadY = best.s.y;
          const hp = best.s.halfPaved;
          const influence = best.c.influence ?? 26;
          // Shield the shoulder plus at least one full grid cell beyond the paved
          // edge: a cell straddling the shield boundary otherwise interpolates
          // from a high outside vertex and pokes through the pavement.
          const shield = hp + Math.max(1.6, stepX * 1.5);
          if (bestD <= shield) {
            // Build the roadbed exactly: raise fill and cut down to it.
            h = roadY - 0.24;
            inRoadShield = true;
          } else if (bestD < shield + influence) {
            const cut = roadY + best.c.cutSlope * (bestD - shield);
            const fill = roadY - best.c.fillSlope * (bestD - shield);
            const target = base >= roadY ? Math.min(base, cut) : Math.max(base, fill);
            const t = smoothstep(1 - (bestD - shield) / influence);
            h = lerp(base, target, t);
          }
        }
        // Junction pads are authoritative inside the pad. Their outer influence
        // band must not lower the roadbed (which would leave the pavement
        // floating), so leave road-shield vertices to corridor grading.
        for (const e of near) {
          if (!e.pad) continue;
          const p = e.pad;
          const d = Math.hypot(x - p.x, z - p.y);
          const padZ = p.heightAt ? p.heightAt(x, z) : p.z;
          if (d <= p.radius) {
            h = padZ - 0.2;
          } else if (!inRoadShield && d < p.radius + (p.influence ?? 14)) {
            const target = padZ - 0.2;
            const t = smoothstep(1 - (d - p.radius) / (p.influence ?? 14));
            h = lerp(h, target, t);
          }
        }
        heights[gy * nx + gx] = h;
      }
    }

    const positions = new Float32Array(nx * nz * 3);
    const colors = new Float32Array(nx * nz * 3);
    const uvs = new Float32Array(nx * nz * 2);
    for (let gy = 0; gy < nz; gy++) {
      for (let gx = 0; gx < nx; gx++) {
        const k = gy * nx + gx;
        const x = lerp(b.minX, b.maxX, gx / resolution);
        const z = lerp(b.minZ, b.maxZ, gy / resolution);
        const h = heights[k];
        positions[k * 3] = x;
        positions[k * 3 + 1] = h;
        positions[k * 3 + 2] = z;
        uvs[k * 2] = gx / resolution;
        uvs[k * 2 + 1] = gy / resolution;
        const hl = heights[gy * nx + Math.max(0, gx - 1)];
        const hr = heights[gy * nx + Math.min(nx - 1, gx + 1)];
        const hd = heights[Math.max(0, gy - 1) * nx + gx];
        const hu = heights[Math.min(nz - 1, gy + 1) * nx + gx];
        const gX = (hr - hl) / (2 * stepX);
        const gZ = (hu - hd) / (2 * stepZ);
        const slope = Math.hypot(gX, gZ);
        const rock = smoothstep((slope - 0.55) / 0.9);
        const snow = smoothstep((h - 55) / 22);
        const grain = 0.9 + 0.1 * valueNoise(x * 0.08, z * 0.08, this.seed + 7);
        colors[k * 3] = lerp(lerp(0.26, 0.5, rock), 0.93, snow) * grain;
        colors[k * 3 + 1] = lerp(lerp(0.42, 0.47, rock), 0.95, snow) * grain;
        colors[k * 3 + 2] = lerp(lerp(0.2, 0.4, rock), 0.97, snow) * grain;
      }
    }

    const indices = [];
    for (let gy = 0; gy < resolution; gy++) {
      for (let gx = 0; gx < resolution; gx++) {
        const a = gy * nx + gx;
        const b0 = gy * nx + gx + 1;
        const c = (gy + 1) * nx + gx + 1;
        const d = (gy + 1) * nx + gx;
        indices.push(a, d, b0, b0, d, c);
      }
    }

    return {
      positions,
      colors,
      uvs,
      indices: new Uint32Array(indices),
      resolution,
      grid: { nx, nz, stepX, stepZ, heights, minX: b.minX, minZ: b.minZ },
      heightAt(i, j) {
        const x = b.minX + i * stepX;
        const z = b.minZ + j * stepZ;
        return this.sampleHeight(x, z);
      },
      // Bilinear sample of the graded surface, matching what is rendered.
      sampleHeight(x, z) {
        const fx = clamp((x - b.minX) / stepX, 0, nx - 1.000001);
        const fz = clamp((z - b.minZ) / stepZ, 0, nz - 1.000001);
        const i = Math.floor(fx);
        const j = Math.floor(fz);
        const tx = fx - i;
        const tz = fz - j;
        const h00 = heights[j * nx + i];
        const h10 = heights[j * nx + i + 1];
        const h01 = heights[(j + 1) * nx + i];
        const h11 = heights[(j + 1) * nx + i + 1];
        return lerp(lerp(h00, h10, tx), lerp(h01, h11, tx), tz);
      },
    };
  }
}

export function featureHeight(f, x, z) {
  // Scene records use plan `y`; accept `z` too for convenience.
  const fy = f.y ?? f.z ?? 0;
  switch (f.type) {
    case 'hill': {
      const d = Math.hypot(x - f.x, z - fy);
      if (d >= f.radius) return 0;
      return f.height * Math.pow(1 - d / f.radius, 1.9);
    }
    case 'ridge': {
      const seg = closestPointOnSegment({ x, y: z }, f.a, f.b);
      const d = Math.hypot(x - seg.point.x, z - seg.point.y);
      if (d >= f.width) return 0;
      const taper = f.taper ? smoothstep(Math.min(seg.t, 1 - seg.t) / Math.max(f.taper, 1e-6)) : 1;
      return f.height * Math.pow(1 - d / f.width, 1.6) * taper;
    }
    case 'slopePlane':
      return (x * f.dir.x + z * f.dir.y) * f.grade;
    case 'plateau': {
      const d = Math.hypot(x - f.x, z - fy);
      if (d >= f.radius + f.falloff) return 0;
      const t = clamp((f.radius + f.falloff - d) / f.falloff, 0, 1);
      return f.height * smootherstep(t);
    }
    case 'valley': {
      const d = Math.hypot(x - f.x, z - fy);
      if (d >= f.radius) return 0;
      return -f.depth * Math.pow(1 - d / f.radius, 1.4);
    }
    case 'berm': {
      const seg = closestPointOnSegment({ x, y: z }, f.a, f.b);
      const d = Math.hypot(x - seg.point.x, z - seg.point.y);
      return f.height * Math.exp(-(d * d) / (2 * f.width * f.width));
    }
    default:
      return 0;
  }
}

function normalizeObstacle(o, i) {
  const id = o.id || `obstacle-${i}`;
  if (o.shape === 'circle' || (o.x !== undefined && o.radius !== undefined && !o.points)) {
    return { id, shape: 'circle', x: o.x, y: o.y, radius: o.radius, kind: o.kind || 'forbidden', label: o.label || 'forbidden' };
  }
  return {
    id,
    shape: 'polygon',
    points: simplify((o.points || []).map((p) => ({ x: p.x, y: p.y }))),
    kind: o.kind || 'forbidden',
    label: o.label || 'forbidden',
  };
}

export function terrainToRecord(t) {
  return JSON.parse(JSON.stringify({
    id: t.id,
    seed: t.seed,
    baseHeight: t.baseHeight,
    roughness: t.roughness,
    noiseScale: t.noiseScale,
    features: t.features,
    obstacles: t.obstacles,
    bounds: t.bounds,
  }));
}
