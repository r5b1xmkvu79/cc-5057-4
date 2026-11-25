// Stage 3 — junction connection and arm trimming.
//
// Resolves where each arm's pavement meets a junction: sizes the pad so every
// swept turn fits, places ports at the trim stations, re-flattens approach
// profiles, builds the junction height field and finally emits each road's
// ribbon clipped to the junction rim. Moved verbatim from build.js so geometry
// is unchanged; the only addition is provenance tagging and an explicit
// route->mesh hand-off object.

import { clamp, lerp, polygonArea } from '../math.js';
import { classById, vehicleById, parkingTurnRadius } from '../standards.js';
import {
  sampleAt,
  solveProfile,
  taperEnds,
  clampProfile,
} from '../road.js';
import {
  buildRoadRibbon,
  makePort,
  junctionPadPolygon,
  makeJunctionHeightField,
  junctionMaxGrade,
  buildTurnPath,
  sweepBodyEnvelope,
} from '../geometry.js';
import { triangulatePolygon } from '../surface.js';
import { STAGES, tagProvenance } from '../provenance.js';

function endPort(route, end) {
  const st = end === 'a' ? 0 : route.samples[route.samples.length - 1].s;
  const sample = end === 'a' ? route.samples[0] : route.samples[route.samples.length - 1];
  return makePort(route.conn, end, st, sample);
}

function assembleJunction({ node, incident, routes, terrain, controls, junctionZ }) {
  const z = junctionZ.get(node.id);
  const ports = [];
  for (const inc of incident) {
    const r = routes.find((x) => x.conn.id === inc.conn.id);
    const isA = r.conn.a === node.id;
    const st = isA ? 0 : r.samples[r.samples.length - 1].s;
    const sample = isA ? r.samples[0] : r.samples[r.samples.length - 1];
    ports.push({ inc, route: r, sample, end: isA ? 'a' : 'b', station: st });
  }

  const maxWidth = Math.max(...ports.map((p) => p.sample.halfLeft + p.sample.halfRight), 4);
  let minTurnR = 0;
  for (const p of ports) {
    const vid = p.route.conn.vehicleId || controls.vehicleId || 'sedan';
    minTurnR = Math.max(minTurnR, parkingTurnRadius(vid));
  }
  let radius = Math.max(
    clamp(maxWidth * 0.9, 5, 15) * controls.junctionTrimScale,
    controls.junctionRadius * 0.7,
    minTurnR * 1.12,
  );

  const placePorts = (rad) => {
    const trimStations = new Map();
    const portByConn = new Map();
    for (const p of ports) {
      const r = p.route;
      const L = r.samples[r.samples.length - 1].s;
      const cut = Math.min(rad, L * 0.38);
      let station = p.end === 'b' ? p.station - cut : p.station + cut;
      station = clamp(station, 0, L);
      trimStations.set(r.conn.id, station);
      const s = sampleAt(r.samples, station);
      portByConn.set(r.conn.id, makePort(r.conn, p.end, station, s));
    }
    return { trimStations, portByConn };
  };
  let placed = placePorts(radius);
  if (ports.length >= 2) {
    const vehicle = vehicleById(controls.vehicleId || 'sedan');
    const coverDist = (route) => {
      const arr = route.samples;
      return (q) => {
        for (let i = 1; i < arr.length; i++) {
          const a = arr[i - 1];
          const b = arr[i];
          const dx = b.x - a.x;
          const dy = b.y - a.y;
          const l2 = dx * dx + dy * dy;
          if (l2 < 1e-9) continue;
          const t = clamp(((q.x - a.x) * dx + (q.y - a.y) * dy) / l2, 0, 1);
          const px = a.x + dx * t;
          const py = a.y + dy * t;
          const lat = Math.hypot(q.x - px, q.y - py);
          const half = lerp(Math.max(a.halfLeft, a.halfRight), Math.max(b.halfLeft, b.halfRight), t);
          if (lat <= half + 0.2) return true;
        }
        return false;
      };
    };
    const armCover = ports.map((p) => coverDist(p.route));
    for (let pass = 0; pass < 3; pass++) {
      let reach = 0;
      for (let i = 0; i < ports.length; i++) {
        const a = ports[i];
        const b = ports[(i + 1) % ports.length];
        if (ports.length === 2 && i === 1) break;
        const pa = placed.portByConn.get(a.route.conn.id);
        const pb = placed.portByConn.get(b.route.conn.id);
        if (!pa || !pb) continue;
        const path = buildTurnPath(pa, a.route.samples, pb, b.route.samples, {
          vehicleHalfWidth: vehicle.width / 2,
        });
        const line = [];
        for (const q of path) line.push({ x: q.x, y: q.y });
        const swept = sweepBodyEnvelope(line, vehicle, vehicle.wheelbase / 2, 1.0);
        for (const c of swept) {
          if (armCover[i](c) || armCover[(i + 1) % ports.length](c)) continue;
          reach = Math.max(reach, Math.hypot(c.x - node.x, c.y - node.y));
        }
      }
      if (reach + 0.35 <= radius + 1e-6) break;
      radius = Math.min(reach + 0.35, 24);
      placed = placePorts(radius);
    }
  }
  const { trimStations, portByConn } = placed;

  const junction = {
    id: `j-${node.id}`,
    node: { ...node },
    z,
    radius,
    incident: ports.map((p) => ({ conn: p.inc.conn, end: p.end, route: p.route, port: null, sample: p.sample })),
    trimStations,
    portByConn,
  };
  tagProvenance(junction, { stage: STAGES.JUNCTION, source: `node:${node.id}` });
  return junction;
}

function finalizeJunctionSurface(j, terrain, nodeZ) {
  const ports = j.incident.map((inc) => inc.port).filter(Boolean);
  if (!ports.length) {
    j.surface = null;
    j.issues = ['no-ports'];
    return;
  }
  const zs = ports.map((p) => p.z);
  const spread = Math.max(...zs) - Math.min(...zs);
  j.heightSpread = spread;
  j.z = nodeZ;
  const center = { x: j.node.x, y: j.node.y, z: nodeZ };
  const heightField = makeJunctionHeightField(ports, center);
  const ring = junctionPadPolygon(center, j.radius, 44);
  const res = triangulatePolygon(ring, heightField, 2.4);
  const surface = {
    id: j.id,
    part: 'junction',
    ports,
    ring,
    heightField,
    triangles: res.triangles,
    footprintArea: Math.abs(polygonArea(ring)),
    maxGrade: junctionMaxGrade(res.triangles),
  };
  tagProvenance(surface, { stage: STAGES.MESH, source: `junction:${j.id}` });
  j.surface = surface;
  j.area = surface.footprintArea;
  j.issues = [];
  if (surface.maxGrade > 0.2) j.issues.push('junction-excess-grade');
  if (spread > 1.6) j.issues.push('junction-port-elevation-gap');
  let worstStep = 0;
  for (const p of ports) {
    for (const e of [p.leftEdge, p.rightEdge, p]) {
      const h = heightField(e.x, e.y);
      worstStep = Math.max(worstStep, Math.abs(h - p.z));
    }
  }
  j.seamStep = worstStep;
  if (worstStep > 0.22) j.issues.push('junction-seam-step');
}

// Run the full junction stage over the routed network. Mutates route stations/
// ports (these are the routing stage's own arrays, never the authored input)
// and attaches ribbons; returns the junction list.
export function connectJunctions(input, routes, { graphId, revision } = {}) {
  const { nodes, nodeConnections, terrain, controls, junctionZ, project } = input;
  const junctions = [];
  const junctionByNode = new Map();
  for (const n of nodes.values()) {
    const incident = nodeConnections.get(n.id) || [];
    if (!incident.length) continue;
    if (incident.length === 1) continue;
    const junction = assembleJunction({
      node: n,
      incident,
      routes,
      nodes,
      terrain,
      controls: { ...controls, vehicleId: project.vehicleId || 'sedan' },
      junctionZ,
    });
    junctions.push(junction);
    junctionByNode.set(n.id, junction);
  }

  for (const r of routes) {
    const inA = junctionByNode.get(r.conn.a);
    const inB = junctionByNode.get(r.conn.b);
    r.startStation = inA ? inA.trimStations.get(r.conn.id) : 0;
    r.endStation = inB ? inB.trimStations.get(r.conn.id) : r.samples[r.samples.length - 1].s;
    r.portA = inA ? inA.portByConn.get(r.conn.id) : endPort(r, 'a');
    r.portB = inB ? inB.portByConn.get(r.conn.id) : endPort(r, 'b');
  }

  for (const r of routes) {
    if (r.startStation > r.endStation) {
      const t = r.startStation;
      r.startStation = r.endStation;
      r.endStation = t;
    }
    let taper = controls.taper;
    for (const end of ['a', 'b']) {
      const j = junctionByNode.get(r.conn[end]);
      if (j) taper = Math.max(taper, j.radius * 1.7);
    }
    if (taper > controls.taper + 1e-6) {
      const z0 = r.samples[0].z;
      const z1 = r.samples[r.samples.length - 1].z;
      const cls = classById(r.conn.classId);
      const baseZ = solveProfile(r.samples, terrain, z0, z1, cls.maxGrade);
      if (!baseZ.infeasible) {
        const tapered = taperEnds(r.samples, baseZ.z, taper);
        const limited = clampProfile(r.samples, tapered, cls.maxGrade);
        for (let i = 0; i < r.samples.length; i++) r.samples[i].z = limited[i];
      }
    }
    const inA = junctionByNode.get(r.conn.a);
    const inB = junctionByNode.get(r.conn.b);
    if (inA) {
      const s = sampleAt(r.samples, r.startStation);
      r.portA = makePort(r.conn, 'a', r.startStation, s);
      inA.portByConn.set(r.conn.id, r.portA);
    }
    if (inB) {
      const s = sampleAt(r.samples, r.endStation);
      r.portB = makePort(r.conn, 'b', r.endStation, s);
      inB.portByConn.set(r.conn.id, r.portB);
    }
  }

  for (const j of junctions) {
    for (const inc of j.incident) {
      const r = inc.route;
      inc.port = r.conn.a === j.node.id ? r.portA : r.portB;
    }
    finalizeJunctionSurface(j, terrain, junctionZ.get(j.node.id));
  }

  for (const r of routes) {
    const L = r.samples[r.samples.length - 1].s;
    const cls = classById(r.conn.classId);
    const overlap = cls.width * 0.7 + 1.5;
    const ribbonStart = junctionByNode.get(r.conn.a) ? Math.max(0, r.startStation - overlap) : r.startStation;
    const ribbonEnd = junctionByNode.get(r.conn.b) ? Math.min(L, r.endStation + overlap) : r.endStation;
    r.ribbon = buildRoadRibbon(r.samples, ribbonStart, ribbonEnd, cls, r.conn.id);
    tagProvenance(r.ribbon, {
      stage: STAGES.MESH,
      source: `connection:${r.conn.id}#ribbon`,
      graphId,
      revision,
    });
  }

  return { junctions, junctionByNode };
}
