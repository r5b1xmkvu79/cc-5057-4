// Road construction kernel entry point.
// Takes a project record and returns routed roads, pavement, junctions,
// graded terrain, validation and a deterministic build record.

import {
  clamp,
  dist,
  hashString,
  lerp,
  norm,
  perp,
  polygonArea,
  smoothstep,
} from './math.js';
import { Terrain, terrainToRecord } from './terrain.js';
import { classById, vehicleById, parkingTurnRadius } from './standards.js';
import {
  applyWidening,
  curvatureStats,
  measureCenterline,
  measureRequiredWidening,
  profileStats,
  rerouteAway,
  rerouteControl,
  sampleAt,
  sampleCenterline,
  setBaseWidths,
  solveProfile,
  taperEnds,
  clampProfile,
  validateRoute,
  controlPolyline,
  sweepVehicle,
} from './road.js';
import { planRoute } from './planner.js';
import {
  buildRoadRibbon,
  makePort,
  junctionPadPolygon,
  makeJunctionHeightField,
  junctionMaxGrade,
  buildTurnPath,
  sweepBodyEnvelope,
  sweepPathOnSurface,
} from './geometry.js';
import {
  differenceShapes,
  offsetRings,
  overlapAreaShapes,
  pointInRing,
  shapesArea,
  unionShapes,
} from './clip.js';
import { triangulatePolygon, trianglesToPositions, SurfaceIndex, resampleRing } from './surface.js';

const DEFAULT_CONTROLS = {
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
};

export function buildProject(project, opts = {}) {
  const t0 = performanceNow();
  const controls = { ...DEFAULT_CONTROLS, ...(project.controls || {}), ...(opts.controls || {}) };
  const seed = project.seed ?? 1;
  const terrain = new Terrain(project.terrain || {});

  // ---- 1. Nodes and connections ------------------------------------------
  const nodes = new Map(project.nodes.map((n) => [n.id, { ...n }]));
  const connections = project.connections.map((c) => normalizeConnection(c, controls));
  const connById = new Map(connections.map((c) => [c.id, c]));

  // Junction centres and ports ------------------------------------------------
  const junctionNodes = [...nodes.values()].filter((n) => n.kind !== 'corner' || true);
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

  // ---- 2. Route each connection -------------------------------------------
  const routes = [];
  const diagnostics = { decisions: [], iterations: [] };

  for (const conn of connections) {
    const nodeA = nodes.get(conn.a);
    const nodeB = nodes.get(conn.b);
    const cls = classById(conn.classId);
    const vehicle = vehicleById(conn.vehicleId || project.vehicleId || 'sedan');
    const route = buildConnection({
      conn,
      project,
      terrain,
      cls,
      vehicle,
      nodeA,
      nodeB,
      nodeConnections,
      nodes,
      junctionZ,
      controls,
      seed,
      diagnostics,
    });
    routes.push(route);
  }

  // ---- 3. Junction topology -------------------------------------------------
  const junctions = [];
  const junctionByNode = new Map();
  for (const n of nodes.values()) {
    const incident = nodeConnections.get(n.id) || [];
    if (!incident.length) continue;
    if (incident.length === 1) continue; // road end, not a junction
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

  // ---- 4. Trim roads at junctions and build ribbons -------------------------
  for (const r of routes) {
    const inA = junctionByNode.get(r.conn.a);
    const inB = junctionByNode.get(r.conn.b);
    r.startStation = inA ? inA.trimStations.get(r.conn.id) : 0;
    r.endStation = inB ? inB.trimStations.get(r.conn.id) : r.samples[r.samples.length - 1].s;
    r.portA = inA ? inA.portByConn.get(r.conn.id) : endPort(r, 'a');
    r.portB = inB ? inB.portByConn.get(r.conn.id) : endPort(r, 'b');
  }

  // Junction aprons must be nearly level where the pad meets the arm. The pad
  // radius is known now, so flatten each approach over at least the pad
  // footprint before deriving the pad's own height field.
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
    // Ports sit on the reconstructed profile; refresh them in place.
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
    // Extend the ribbon slightly into each junction so the union has no sliver
    // seam between the circular pad and the trimmed road. Heights inside the pad
    // come from the junction field, so the overlap is invisible.
    const L = r.samples[r.samples.length - 1].s;
    const cls = classById(r.conn.classId);
    const overlap = cls.width * 0.7 + 1.5;
    const ribbonStart = junctionByNode.get(r.conn.a) ? Math.max(0, r.startStation - overlap) : r.startStation;
    const ribbonEnd = junctionByNode.get(r.conn.b) ? Math.min(L, r.endStation + overlap) : r.endStation;
    r.ribbon = buildRoadRibbon(r.samples, ribbonStart, ribbonEnd, cls, r.conn.id);
  }

  // ---- 5. Pavement union, collisions, validation ----------------------------
  const pavement = assemblePavement(routes, junctions);
  const surfaceIndex = new SurfaceIndex(pavement.collisionTriangles);
  const pavementArea = shapesArea(pavement.shapes);

  // Route-level acceptance is re-run on the final pavement (widened) widths.
  for (const r of routes) {
    r.sweep = r.sweepFinal;
    r.finalViolations = validateRoute(
      r.conn,
      r.samples,
      terrain,
      classById(r.conn.classId),
      vehicleById(r.conn.vehicleId || project.vehicleId || 'sedan'),
      { infeasibleProfile: r.profileInfeasible, infeasibleDelta: r.profileDelta },
    );
    // Swept pavement corridor test (not just centreline).
    if (r.sweep && !r.sweep.pass) {
      r.finalViolations.push({
        code: 'swept-corridor',
        severity: 'error',
        message: `车辆扫掠超出路面 ${(r.sweep.maxExceed).toFixed(2)}m`,
        value: r.sweep.maxExceed,
        limit: 0,
        plan: r.sweep.worst ? { x: r.sweep.worst.x, y: r.sweep.worst.y } : null,
        station: r.sweep.worst ? r.sweep.worst.station : null,
        connectionId: r.conn.id,
      });
    }
  }

  // Junction turn tests.
  const turnTests = [];
  for (const j of junctions) {
    if (j.incident.length < 2) continue;
    const arms = j.incident;
    for (let i = 0; i < arms.length; i++) {
      const armA = arms[i];
      const armB = arms[(i + 1) % arms.length];
      if (arms.length === 2 && i === 1) continue; // only one turn for a through node
      const ra = routes.find((r) => r.conn.id === armA.conn.id);
      const rb = routes.find((r) => r.conn.id === armB.conn.id);
      const directionA = armA.port.ux;
      const fromPort = armA.port;
      const toPort = armB.port;
      const vehicle = vehicleById(project.vehicleId || 'sedan');
      const path = buildTurnPath(fromPort, ra.samples, toPort, rb.samples, {
        vehicleHalfWidth: vehicle.width / 2,
      });
      const test = sweepPathOnSurface(path, vehicle, surfaceIndex);
      // Turn angle is between the travel directions: into the junction along A
      // and out along B. Outward port vectors are not travel vectors.
      const turnAngle = Math.abs(angleOf(-fromPort.ux, -fromPort.uy, toPort.ux, toPort.uy));
      turnTests.push({
        junctionId: j.id,
        from: { connectionId: armA.conn.id, end: armA.end },
        to: { connectionId: armB.conn.id, end: armB.end },
        turningAngleDeg: (turnAngle * 180) / Math.PI,
        pass: test.pass,
        offPavement: test.offPavement,
        steerRatio: test.steerRatio,
        maxSteerDeg: (test.maxSteer * 180) / Math.PI,
        maxHeightSpread: test.maxHeightSpread,
        maxHeightStep: test.maxHeightStep,
        turnRadius: test.turnRadius,
        footprints: test.footprints,
        path,
        worst: test.worst,
      });
    }
  }

  // Junction geometry checks: seams, direction, overlaps.
  const junctionReports = junctions.map((j) => ({
    id: j.id,
    nodeId: j.node.id,
    arms: j.incident.length,
    area: j.area,
    maxGrade: j.surface ? j.surface.maxGrade : 0,
    heightSpread: j.heightSpread,
    issues: j.issues || [],
    triangles: j.surface ? j.surface.triangles.length : 0,
    portElevations: j.incident.map((i) => ({ connectionId: i.conn.id, end: i.end, z: i.port?.z ?? 0 })),
  }));

  // Overlap between separate pavements (roads should be unioned at junctions).
  const overlapIssues = findOverlaps(routes, junctions);

  // ---- 6. Terrain grading -------------------------------------------------
  const pads = junctions.map((j) => ({
    x: j.node.x,
    y: j.node.y,
    z: j.z,
    // Grade to the actual junction surface, not the raw node elevation, and
    // extend the authoritative disc past the ring so grid cells that straddle
    // the pavement edge stay entirely under the shelter.
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

  // Seam check: compare pavement edge heights with graded terrain.
  const seams = checkSeams(routes, junctions, terrainMesh, surfaceIndex);

  // ---- 7. Build record -----------------------------------------------------
  const record = makeBuildRecord({
    project,
    seed,
    controls,
    terrain,
    routes,
    junctions,
    turnTests,
    junctionReports,
    diagnostics,
    overlapIssues,
    seams,
  });

  // ---- 8. Aggregate result -------------------------------------------------
  const routeReports = routes.map((r) => ({
    id: r.conn.id,
    name: r.conn.name || r.conn.id,
    classId: r.conn.classId,
    vehicleId: r.conn.vehicleId || project.vehicleId || 'sedan',
    length: r.samples.length ? r.samples[r.samples.length - 1].s : 0,
    startStation: r.startStation,
    endStation: r.endStation,
    minRadius: r.curvatureStats.radius,
    maxGrade: r.profileStats.maxGrade,
    gradeChange: r.profileStats.maxChange,
    maxWidening: Math.max(0, ...r.samples.map((s) => Math.max(s.extraLeft, s.extraRight))),
    widenedFraction: r.samples.filter((s) => Math.max(s.extraLeft, s.extraRight) > 0.05).length / Math.max(1, r.samples.length),
    sweep: {
      pass: r.sweep ? r.sweep.pass : false,
      maxExceed: r.sweep ? r.sweep.maxExceed : 0,
      maxSteerDeg: r.sweep ? (r.sweep.maxSteer * 180) / Math.PI : 0,
      worst: r.sweep ? r.sweep.worst : null,
    },
    violations: r.finalViolations,
    decisions: r.decisions,
    locked: !!r.conn.locked,
    errorCount: r.finalViolations.filter((v) => v.severity === 'error').length,
    warnCount: r.finalViolations.filter((v) => v.severity === 'warn').length,
  }));

  const allViolations = [
    ...routeReports.flatMap((r) => r.violations),
    ...overlapIssues,
    ...seams.filter((s) => s.severity === 'error'),
    ...junctionReports.flatMap((j) =>
      j.issues.map((code) => ({
        code,
        severity: 'error',
        message: `路口 ${j.id} 拓扑问题: ${code}`,
        plan: { x: nodes.get(j.nodeId).x, y: nodes.get(j.nodeId).y },
        junctionId: j.id,
      })),
    ),
    ...turnTests
      .filter((t) => !t.pass)
      .map((t) => ({
        code: 'turn-fail',
        severity: 'error',
        message: `路口 ${t.junctionId} 转向不通过（离开路面 ${t.offPavement} 点，转向 ${t.maxSteerDeg.toFixed(0)}°）`,
        plan: t.worst ? { x: t.worst.x, y: t.worst.y } : null,
        junctionId: t.junctionId,
        turn: { from: t.from, to: t.to },
      })),
  ];

  const errorCount = allViolations.filter((v) => v.severity === 'error').length;
  const warnCount = allViolations.filter((v) => v.severity === 'warn').length;

  return {
    ok: errorCount === 0,
    seed,
    controls,
    terrain,
    terrainMesh,
    terrainRecord: terrainToRecord(terrain),
    nodes: [...nodes.values()],
    connections,
    routes,
    routeReports,
    junctions,
    junctionReports,
    turnTests,
    pavement,
    surfaceIndex,
    pavementArea,
    seams,
    overlapIssues,
    violations: allViolations,
    errorCount,
    warnCount,
    diagnostics,
    record,
    computeMs: performanceNow() - t0,
  };
}

// ---------------------------------------------------------------------------
// Connection routing with widen-vs-reroute decision
// ---------------------------------------------------------------------------
function normalizeConnection(c, controls) {
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

function buildConnection(ctx) {
  const { conn, project, terrain, cls, vehicle, nodeA, nodeB, nodeConnections, nodes, junctionZ, controls, seed, diagnostics } = ctx;
  const decisions = [];
  const originalControl = controlPolyline(conn, nodes);
  let control = conn.controlPoints ? originalControl : null;
  let usedPlan = false;

  const zA = junctionZ.get(conn.a);
  const zB = junctionZ.get(conn.b);

  if (!control || control.length < 2) {
    // No authored control points: plan a constrained route.
    const plan = planRoute(terrain, nodeA, nodeB, cls, {
      cell: project.controls?.plannerCell,
    });
    control = plan.points;
    usedPlan = true;
    decisions.push({
      kind: 'plan',
      message: plan.ok ? '自动规划平面路线' : '自动规划失败，回退为直线',
      length: plan.stats?.length,
    });
  }

  let attempt = 0;
  let best = null;
  let workingControl = control.map((p) => ({ x: p.x, y: p.y }));
  const maxAttempts = controls.rerouteAttempts + controls.widenAttempts + 1;

  while (attempt < maxAttempts) {
    attempt++;
    const samples = sampleCenterline(workingControl);
    if (!samples.length) {
      decisions.push({ kind: 'error', message: '采样失败' });
      break;
    }
    setBaseWidths(samples, cls);

    // Heading continuity at junctions: align the first/last tangent with the
    // connecting arm so the arm exits radially. We only nudge authored controls.
    const profile = solveProfile(samples, terrain, zA, zB, cls.maxGrade);
    const tapered = taperEnds(samples, profile.z, controls.taper);
    const limited = profile.infeasible ? tapered : clampProfile(samples, tapered, cls.maxGrade);
    for (let i = 0; i < samples.length; i++) samples[i].z = limited[i];

    // Apply intrinsic curve widening based on radius so vehicles track the bend.
    const leftExtra = [];
    const rightExtra = [];
    const innerComfort = 0.35;
    for (const s of samples) {
      const wid = curveWidenForRadius(s.radius, cls, vehicle);
      leftExtra.push(wid + (wid > 0 ? innerComfort * 0.5 : 0));
      rightExtra.push(wid);
    }
    applyWidening(samples, leftExtra, rightExtra, 4);

    // Swept-corridor acceptance against the current pavement widths.
    let sweep = null;
    if (controls.allowWiden) {
      const settled = measureRequiredWidening(samples, vehicle, 4);
      sweep = settled.sweep;
      const maxWiden = Math.max(0, ...samples.map((s) => Math.max(s.extraLeft, s.extraRight)));
      for (const s of samples) {
        s.extraLeft = Math.min(s.extraLeft, controls.maxWiden);
        s.extraRight = Math.min(s.extraRight, controls.maxWiden);
        s.halfLeft = s.baseHalf + s.extraLeft;
        s.halfRight = s.baseHalf + s.extraRight;
      }
      sweep = sweepVehicleFinal(samples, vehicle);
      decisions.push({
        kind: 'widen',
        message: `弯道加宽最大 ${maxWiden.toFixed(2)}m`,
        value: maxWiden,
      });
    } else {
      sweep = sweepVehicleFinal(samples, vehicle);
    }

    const violations = validateRoute(conn, samples, terrain, cls, vehicle, {
      infeasibleProfile: profile.infeasible,
      infeasibleDelta: zB - zA,
    });

    // Spatial conflicts that can be resolved either way.
    const radiusViolation = violations.find((v) => v.code === 'min-radius' || v.code === 'vehicle-turn-limit');
    const obstacleViolation = violations.find((v) => v.code === 'forbidden-route');
    const sweepViolation = !sweep.pass;

    const score =
      violations.filter((v) => v.severity === 'error').length * 1000 +
      violations.filter((v) => v.severity === 'warn').length * 25 +
      (sweepViolation ? 2000 : 0) +
      (sweep ? sweep.maxExceed * 100 : 0) +
      Math.max(0, ...samples.map((s) => Math.max(s.extraLeft, s.extraRight)));

    if (!best || score < best.score) {
      best = {
        score,
        control: workingControl.map((p) => ({ ...p })),
        samples,
        sweep,
        violations,
        profile,
        attempt,
      };
    }

    if (!violations.some((v) => v.severity === 'error') && sweep.pass) break;

    // Decide: widen or reroute?
    const action = chooseAction({
      controls,
      obstacleViolation,
      radiusViolation,
      sweep,
      samples,
      cls,
      vehicle,
    });

    if (action === 'reroute-obstacle' && obstacleViolation) {
      if (!controls.allowReroute) break;
      const next = rerouteAway(workingControl, samples, terrain, 7.5, obstacleViolation.station);
      decisions.push({
        kind: 'reroute',
        message: '禁止区冲突：改线绕行',
        reason: obstacleViolation.message,
        attempt,
      });
      workingControl = next;
      continue;
    }
    if (action === 'reroute-bend' && radiusViolation) {
      if (!controls.allowReroute) break;
      const amount = Math.max(4, 18 - cls.minRadius * 0.1);
      const next = rerouteControl(workingControl, samples, radiusViolation.station ?? samples[Math.floor(samples.length / 2)].s, amount);
      if (!next) break;
      decisions.push({
        kind: 'reroute',
        message: '急弯半径不足：改变平面曲率（改线）',
        reason: radiusViolation.message,
        attempt,
      });
      workingControl = next;
      continue;
    }
    if (action === 'widen' && sweepViolation) {
      // Already widened in this loop; if the vehicle still exceeds, reroute when
      // widening is exhausted, otherwise accept and report.
      decisions.push({
        kind: 'widen-fail',
        message: '加宽已达上限仍无法通过扫掠，保留违规',
        attempt,
      });
      break;
    }
    break;
  }

  const chosen = best || {
    control: workingControl,
    samples: sampleCenterline(workingControl),
    sweep: null,
    violations: [],
    profile: { infeasible: false },
  };
  const samples = chosen.samples;
  setBaseWidths(samples, cls);
  const profile = chosen.profile.infeasible === undefined ? solveProfile(samples, terrain, zA, zB, cls.maxGrade) : chosen.profile;
  const tapered = taperEnds(samples, profile.z, controls.taper);
  const limited = profile.infeasible ? tapered : clampProfile(samples, tapered, cls.maxGrade);
  for (let i = 0; i < samples.length; i++) samples[i].z = limited[i];
  // Re-apply the widening that was chosen.
  const leftExtra = [];
  const rightExtra = [];
  for (const s of samples) {
    const wid = curveWidenForRadius(s.radius, cls, vehicle);
    leftExtra.push(Math.min(wid + 0.175, controls.maxWiden));
    rightExtra.push(Math.min(wid, controls.maxWiden));
  }
  applyWidening(samples, leftExtra, rightExtra, 4);
  const settled = measureRequiredWidening(samples, vehicle, 4);
  for (const s of samples) {
    s.extraLeft = Math.min(s.extraLeft, controls.maxWiden);
    s.extraRight = Math.min(s.extraRight, controls.maxWiden);
    s.halfLeft = s.baseHalf + s.extraLeft;
    s.halfRight = s.baseHalf + s.extraRight;
  }
  const sweepFinal = sweepVehicleFinal(samples, vehicle);

  return {
    conn,
    cls,
    vehicle,
    control: chosen.control,
    samples,
    decisions,
    sweepFinal,
    curvatureStats: curvatureStats(samples),
    profileStats: profileStats(samples, samples.map((s) => s.z)),
    profileInfeasible: !!profile.infeasible,
    profileDelta: zB - zA,
    usedPlan,
    startStation: 0,
    endStation: samples.length ? samples[samples.length - 1].s : 0,
  };
}

function sweepVehicleFinal(samples, vehicle) {
  return sweepVehicle(samples, vehicle);
}

function curveWidenForRadius(radius, cls, vehicle) {
  if (!isFinite(radius) || radius <= 0) return 0;
  if (radius > 120) return 0;
  const wb = vehicle.wheelbase;
  const r = Math.max(radius, 4);
  const off = (wb * wb) / (2 * r);
  const outer = Math.sqrt(r * r + wb * wb) - r;
  const need = off * 0.85 + outer * 0.35;
  return clamp(need, 0, 4.5);
}

function chooseAction({ controls, obstacleViolation, radiusViolation, sweep, samples, cls, vehicle }) {
  if (obstacleViolation) return controls.allowReroute ? 'reroute-obstacle' : 'widen';
  if (radiusViolation) {
    // Prefer reroute when widening cannot buy enough radius, otherwise widen.
    const needExtra = clamp(radiusViolation.limit - radiusViolation.value, 0, 999);
    const comfort = curveWidenForRadius(radiusViolation.value, cls, vehicle);
    if (controls.widenPreference > 0.65 && comfort > needExtra * 0.45) return 'widen';
    return controls.allowReroute ? 'reroute-bend' : 'widen';
  }
  if (sweep && !sweep.pass) return 'widen';
  return 'none';
}

// ---------------------------------------------------------------------------
// Junction assembly
// ---------------------------------------------------------------------------
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
  // Pad radius must contain the widest turn between any two arms. Minimum inner
  // radius for a U-turn at the widest arm plus the body's corner radius, capped
  // to avoid swallowing short roads.
  // Pad must contain the tightest 90-degree turn for every vehicle using it.
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

  // Place each port where the junction surface meets that arm, i.e. roughly at
  // the junction radius along the road. The ribbon is trimmed there.
  const placePorts = (rad) => {
    const trimStations = new Map();
    const portByConn = new Map();
    for (const p of ports) {
      const r = p.route;
      const L = r.samples[r.samples.length - 1].s;
      // Do not trim more than 38% of a short road.
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
  // The pad must physically contain every swept body envelope, not just the
  // parking circle: lane offsets push the arc outward and the vehicle's corners
  // swing wider than its rear-axle trace. Measure the real footprints and grow
  // the pad until they fit (each pass re-places the ports, which moves the arms
  // too). Points already covered by an arm ribbon are ignored.
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
        // Sample the actual body footprint along the turn.
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

  return {
    id: `j-${node.id}`,
    node: { ...node },
    z,
    radius,
    incident: ports.map((p) => ({ conn: p.inc.conn, end: p.end, route: p.route, port: null, sample: p.sample })),
    trimStations,
    portByConn,
  };
}

function endPort(route, end) {
  const st = end === 'a' ? 0 : route.samples[route.samples.length - 1].s;
  const sample = end === 'a' ? route.samples[0] : route.samples[route.samples.length - 1];
  return makePort(route.conn, end, st, sample);
}

function refreshJunctionPorts(j, defaultZ) {
  for (const inc of j.incident) {
    if (inc.port) continue;
    const r = inc.route;
    const station = j.trimStations.get(r.conn.id);
    const s = sampleAt(r.samples, station);
    inc.port = makePort(r.conn, inc.end, station, s);
  }
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
  j.surface = surface;
  j.area = surface.footprintArea;
  j.issues = [];
  if (surface.maxGrade > 0.2) j.issues.push('junction-excess-grade');
  if (spread > 1.6) j.issues.push('junction-port-elevation-gap');
  // Each arm's port cross-section must meet the pad height within tolerance.
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

// ---------------------------------------------------------------------------
// Pavement union and collision
// ---------------------------------------------------------------------------
function assemblePavement(routes, junctions) {
  const shapes = [];
  for (const r of routes) {
    if (r.ribbon && r.ribbon.ring.length >= 3) shapes.push({ outer: r.ribbon.ring, holes: [] });
  }
  for (const j of junctions) {
    if (j.surface && j.surface.ring.length >= 3) shapes.push({ outer: j.surface.ring, holes: [] });
  }
  const allShapes = unionShapes(shapes);
  const unionRings = allShapes.map((s) => s.outer);

  // Triangulate locally so each triangle samples the right height source and is
  // not a long sliver spanning the whole scene:
  //  1. junction pads (planar field, blended to ports),
  //  2. each road ribbon clipped to the union,
  //  3. the union fallback for any remaining area (e.g. overlaps with no source).
  // Steps 1 and 2 are then clipped to the union to prevent double pavements.

  const markingTriangles = [];
  const shoulderTriangles = [];
  for (const r of routes) {
    markingTriangles.push(...r.ribbon.markingTriangles);
    shoulderTriangles.push(...r.ribbon.shoulderTriangles);
  }

  const colliderTriangles = [];
  const pavementTriangles = [];

  // 1. Junction pads. Pads are authoritative and must not overlap each other;
  // where they do, the higher one wins (the pad is a single level surface).
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

  // 2. Road pavement. Instead of re-triangulating the whole (possibly
  // non-convex, pad-cut) ribbon — which makes earcut fan long triangles across
  // it and linearly interpolate a curved profile badly — emit each ribbon's own
  // small strip triangles, clipped against the pad rings so the pad surface is
  // the only pavement inside a junction while the arms still meet it exactly.
  const padRings = padShapes.map((p) => p.shape);
  for (const r of routes) {
    for (const tri of r.ribbon.pavementTriangles) {
      let pieces = [{ outer: tri.map((v) => ({ x: v.x, y: v.y })), holes: [] }];
      const cx = (tri[0].x + tri[1].x + tri[2].x) / 3;
      const cy = (tri[0].y + tri[1].y + tri[2].y) / 3;
      const onPad = padCoverage.heightAt(cx, cy) !== null;
      if (onPad && padRings.length) {
        const cut = differenceShapes(pieces, padRings);
        if (!cut.length) continue; // fully inside a pad
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

  // 3. Any union area not covered by a pad or ribbon (measured by sampling) gets
  // a blended height. This is a safety net; normally empty.
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

  return {
    shapes: allShapes,
    pavementTriangles,
    markingTriangles,
    shoulderTriangles,
    collisionTriangles: colliderTriangles,
    outline: unionRings,
    positions: trianglesToPositions(pavementTriangles),
  };
}

function buildCoverageIndex(triangles) {
  return new SurfaceIndex(triangles, 4);
}

// Height function for the plane through a ribbon strip triangle.
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

// Find regions of a polygon whose interior has no pavement triangles. For a
// well-formed build this returns [], keeping the pipeline cheap. When the pad
// and ribbon sets leave a sliver (e.g. rasterisation mismatch), the patch is
// filled with the blended height field.
function findUncoveredPatches(ring, holes, coverage, step) {
  // Sample a coarse grid over the ring; collect cells that are inside the ring
  // (and outside holes) but not covered.
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
  // Convert uncovered cell centres into small quads so the exact geometry stays
  // watertight against the sampled surface; this only runs on rare gaps.
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
  // Fallback: nearest junction field, else the nearest ribbon station height.
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

// ---------------------------------------------------------------------------
// Overlap and seam diagnostics
// ---------------------------------------------------------------------------
function findOverlaps(routes, junctions) {
  const issues = [];
  for (let i = 0; i < routes.length; i++) {
    for (let j = i + 1; j < routes.length; j++) {
      const a = routes[i];
      const b = routes[j];
      if (!a.ribbon.ring.length || !b.ribbon.ring.length) continue;
      // Overlap is allowed only inside a shared junction.
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
        } else {
          // Overlap within a junction footprint is expected and unioned.
        }
      }
    }
  }
  return issues;
}

function checkSeams(routes, junctions, terrainMesh, surfaceIndex) {
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
        // Inside a junction the pad is the real pavement; the untrimmed ribbon
        // edge there may sit at a different height and is not a seam. Measure
        // against the actual built surface when one exists.
        const paved = surfaceIndex ? surfaceIndex.heightAt(edge.x, edge.y) : null;
        const gap = (paved != null ? paved : q.z) - ground;
        // A road floating above ground means fill was not built. Ground above
        // the road edge is a legitimate cut slope, so only large cut pokes warn.
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

// ---------------------------------------------------------------------------
// Build record
// ---------------------------------------------------------------------------
function makeBuildRecord({ project, seed, controls, terrain, routes, junctions, turnTests, junctionReports, diagnostics, overlapIssues, seams }) {
  const fixed = {};
  for (const c of project.connections) {
    if (c.controlPoints) fixed[c.id] = c.controlPoints.map((p) => ({ x: round(p.x), y: round(p.y) }));
  }
  return {
    version: 1,
    seed,
    controls,
    vehicleId: project.vehicleId || 'sedan',
    terrain: terrainToRecord(terrain),
    terrainSeed: terrain.seed,
    nodes: project.nodes.map((n) => ({ id: n.id, x: round(n.x), y: round(n.y), elevation: n.elevation ?? null, kind: n.kind || 'junction' })),
    connections: project.connections.map((c) => ({
      id: c.id,
      a: c.a,
      b: c.b,
      classId: c.classId,
      name: c.name,
      vehicleId: c.vehicleId || null,
      locked: !!c.locked,
      controlPoints: (c.controlPoints || []).map((p) => ({ x: round(p.x), y: round(p.y) })),
    })),
    fixedControls: fixed,
    resolved: routes.map((r) => ({
      id: r.conn.id,
      control: r.control.map((p) => ({ x: round(p.x), y: round(p.y) })),
      startStation: round(r.startStation),
      endStation: round(r.endStation),
      length: round(r.samples.length ? r.samples[r.samples.length - 1].s : 0),
      minRadius: round(r.curvatureStats.radius),
      maxGrade: round(r.profileStats.maxGrade, 4),
      maxWidening: round(Math.max(0, ...r.samples.map((s) => Math.max(s.extraLeft, s.extraRight)))),
      pass: r.finalViolations.filter((v) => v.severity === 'error').length === 0,
      decisions: r.decisions,
    })),
    junctions: junctionReports,
    turns: turnTests.map((t) => ({
      junctionId: t.junctionId,
      from: t.from,
      to: t.to,
      pass: t.pass,
      steerDeg: round(t.maxSteerDeg, 1),
      turnRadius: round(t.turnRadius, 1),
      maxHeightStep: round(t.maxHeightStep ?? 0, 2),
    })),
    diagnostics,
    overlapIssues,
    seams: seams.filter((s) => s.severity === 'error'),
    createdAt: new Date().toISOString(),
  };
}

function round(v, d = 3) {
  return Number(v.toFixed(d));
}

function performanceNow() {
  return typeof performance !== 'undefined' ? performance.now() : Date.now();
}

function angleOf(ax, ay, bx, by) {
  let d = Math.atan2(by, bx) - Math.atan2(ay, ax);
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

export function rebuildWithLocked(project, lockedConnectionId, newControls, baseProject) {
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
