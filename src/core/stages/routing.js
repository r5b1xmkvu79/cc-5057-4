// Stage 2 — centreline routing.
//
// For each connection: plan or adopt the control polyline, sample the spline
// centreline, solve the grade-limited profile, apply curvature widening and run
// the widen-vs-reroute acceptance loop. This is the exact algorithm that lived
// in build.js's buildConnection; it is moved verbatim (same thresholds and
// order of operations) so centreline results are numerically identical.
//
// The stage never mutates the authored control arrays: every iteration works on
// a copy, and the returned samples are the solver's own arrays, tagged with
// provenance and revision.

import { clamp } from '../math.js';
import { classById, vehicleById } from '../standards.js';
import { planRoute } from '../planner.js';
import {
  applyWidening,
  curvatureStats,
  measureRequiredWidening,
  profileStats,
  rerouteAway,
  rerouteControl,
  sampleCenterline,
  setBaseWidths,
  solveProfile,
  taperEnds,
  clampProfile,
  validateRoute,
  controlPolyline,
  sweepVehicle,
} from '../road.js';
import { STAGES, tagProvenance } from '../provenance.js';

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
    const needExtra = clamp(radiusViolation.limit - radiusViolation.value, 0, 999);
    const comfort = curveWidenForRadius(radiusViolation.value, cls, vehicle);
    if (controls.widenPreference > 0.65 && comfort > needExtra * 0.45) return 'widen';
    return controls.allowReroute ? 'reroute-bend' : 'widen';
  }
  if (sweep && !sweep.pass) return 'widen';
  return 'none';
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

    const profile = solveProfile(samples, terrain, zA, zB, cls.maxGrade);
    const tapered = taperEnds(samples, profile.z, controls.taper);
    const limited = profile.infeasible ? tapered : clampProfile(samples, tapered, cls.maxGrade);
    for (let i = 0; i < samples.length; i++) samples[i].z = limited[i];

    const leftExtra = [];
    const rightExtra = [];
    const innerComfort = 0.35;
    for (const s of samples) {
      const wid = curveWidenForRadius(s.radius, cls, vehicle);
      leftExtra.push(wid + (wid > 0 ? innerComfort * 0.5 : 0));
      rightExtra.push(wid);
    }
    applyWidening(samples, leftExtra, rightExtra, 4);

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

  const route = {
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
  tagProvenance(route, {
    stage: STAGES.ROUTING,
    source: `connection:${conn.id}`,
    note: usedPlan ? 'planned-control' : 'authored-control',
  });
  tagProvenance(samples, {
    stage: STAGES.ROUTING,
    source: `connection:${conn.id}#centreline`,
  });
  return route;
}

// Route every connection. Returns the route array and the shared diagnostics
// accumulator. The graphId/revision are stamped on each route so downstream
// mesh consumers can verify they build against the right road.
export function routeConnections(input, { graphId, revision } = {}) {
  const { project, terrain, connections, nodes, nodeConnections, junctionZ, controls, seed } = input;
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
    if (graphId || revision) {
      tagProvenance(route, {
        stage: STAGES.ROUTING,
        source: `connection:${conn.id}`,
        graphId,
        revision,
        note: route.usedPlan ? 'planned-control' : 'authored-control',
      });
    }
    routes.push(route);
  }
  return { routes, diagnostics };
}
