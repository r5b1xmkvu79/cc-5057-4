// Road geometry interface — the stable boundary of the road construction kernel.
//
// Editors, the drive model and the CLI talk to this facade instead of reaching
// into ribbons, junction surfaces or the SurfaceIndex. Every query answer is a
// measured result (never a hard-coded pass) carrying its source part and units,
// so "which pavement is the car actually on" and "is this turn feasible" are
// answered by real computation and stay traceable to the build revision.

import { buildPipeline } from './pipeline.js';
import { RevisionLedger } from './revision.js';
import { EditGuard } from './edit-guard.js';
import { DriveLog } from './drive-log.js';
import { vehicleById } from './standards.js';
import { buildTurnPath, sweepPathOnSurface } from './geometry.js';
import { pointInRing } from './clip.js';
import { UNITS } from './provenance.js';

export class RoadGeometryInterface {
  constructor(project, opts = {}) {
    this.project = project;
    this.opts = opts;
    this.ledger = opts.ledger || new RevisionLedger();
    this.guard = new EditGuard();
    this.driveLog = new DriveLog(project.vehicleId || 'sedan');
    this.built = null;
    this.rebuild(project);
  }

  // Full rebuild through the staged pipeline. A new road revision is issued and
  // the collision mesh is published for that same revision before this returns.
  rebuild(nextProject = this.project) {
    this.project = nextProject;
    this.built = buildPipeline(nextProject, { ...this.opts, ledger: this.ledger });
    return this.built;
  }

  get revision() {
    return this.built ? this.built.revision : 0;
  }

  get meshRevision() {
    return this.ledger.currentMesh();
  }

  // --- geometry queries (all measured, units in the descriptor) -------------

  // Surface height and the exact source part under a plan point. Returns null
  // off-pavement rather than guessing, so callers cannot mistake terrain for road.
  surfaceAt(x, y) {
    const b = this.built;
    const h = b.surfaceIndex.heightAt(x, y);
    if (h === null) return null;
    const source = this.locateSource(x, y);
    return {
      x,
      y,
      height: h,
      source,
      revision: this.meshRevision,
      units: { plan: UNITS.plan, elevation: UNITS.elevation },
    };
  }

  // Identify whether a paved point belongs to a junction pad or a road ribbon.
  locateSource(x, y) {
    for (const j of this.built.junctions) {
      if (j.surface && pointInRing({ x, y }, j.surface.ring)) {
        return { part: 'junction', junctionId: j.id, nodeId: j.node.id };
      }
    }
    for (const r of this.built.routes) {
      if (this.pointOnRibbon(r, x, y)) {
        return { part: 'road', connectionId: r.conn.id };
      }
    }
    return { part: 'unknown' };
  }

  pointOnRibbon(route, x, y) {
    for (const st of route.ribbon.stations) {
      const dx = x - st.x;
      const dy = y - st.y;
      const lat = dx * st.nx + dy * st.ny;
      const along = Math.abs(dx * st.tx + dy * st.ty);
      const half = lat >= 0 ? st.halfLeft : st.halfRight;
      if (along < 2.5 && Math.abs(lat) <= half + 0.05) return true;
    }
    return false;
  }

  // Feasible turns at a junction, computed on the actual collision mesh.
  junctionTurns(junctionId) {
    const b = this.built;
    const j = b.junctions.find((jj) => jj.id === junctionId);
    if (!j) return [];
    return b.turnTests
      .filter((t) => t.junctionId === junctionId)
      .map((t) => ({
        from: t.from,
        to: t.to,
        feasible: t.pass,
        turningAngleDeg: t.turningAngleDeg,
        offPavement: t.offPavement,
        maxSteerDeg: t.maxSteerDeg,
        maxHeightStep: t.maxHeightStep,
        turnRadius: t.turnRadius,
        path: t.path,
        measuredOn: { revision: this.meshRevision, surface: 'collision-index' },
      }));
  }

  // Recompute a single turn on demand (e.g. after selecting a different vehicle)
  // so the UI always shows a measured result, never a cached assumption.
  evaluateTurn(junctionId, fromConnId, toConnId, vehicleId = this.project.vehicleId || 'sedan') {
    const b = this.built;
    const j = b.junctions.find((jj) => jj.id === junctionId);
    if (!j) throw new Error(`unknown junction ${junctionId}`);
    const armA = j.incident.find((i) => i.conn.id === fromConnId);
    const armB = j.incident.find((i) => i.conn.id === toConnId);
    if (!armA || !armB) throw new Error('turn arms not found at junction');
    const ra = b.routes.find((r) => r.conn.id === armA.conn.id);
    const rb = b.routes.find((r) => r.conn.id === armB.conn.id);
    const vehicle = vehicleById(vehicleId);
    const path = buildTurnPath(armA.port, ra.samples, armB.port, rb.samples, {
      vehicleHalfWidth: vehicle.width / 2,
    });
    const result = sweepPathOnSurface(path, vehicle, b.surfaceIndex);
    return {
      feasible: result.pass,
      path,
      result,
      measuredOn: { revision: this.meshRevision, surface: 'collision-index' },
      units: { plan: UNITS.plan, angle: UNITS.angle },
    };
  }

  // --- live edit safety ------------------------------------------------------

  // Plan an edit that removes/re-meshes the given connections. Unoccupied
  // branches are immediately removable; occupied ones are retained unless a safe
  // parking point has been registered.
  planEdit(connectionIds) {
    return this.guard.planEdit(connectionIds);
  }

  registerSafePoint(connectionId, point) {
    this.guard.registerSafePoint(connectionId, point);
  }

  requestSafeReturn(connectionId) {
    return this.guard.requestSafeReturn(connectionId);
  }

  confirmSafeReturn(connectionId) {
    this.guard.confirmSafeReturn(connectionId);
  }

  // Record one drive frame, resolving the real surface source under the wheels.
  recordDriveFrame(frame) {
    const probe = this.surfaceAt(frame.x, frame.y);
    const surface = probe
      ? { ...probe.source, height: probe.height }
      : null;
    return this.driveLog.sample({
      ...frame,
      revision: this.meshRevision,
      latestRevision: this.revision,
      surface,
    });
  }

  startDriveSession(routeId, start) {
    return this.driveLog.startSession({ routeId, revision: this.meshRevision, start });
  }

  endDriveSession() {
    return this.driveLog.endSession(this.revision);
  }

  // After a rebuild, invalidate driving recorded on older road revisions.
  syncDriveLog() {
    return this.driveLog.markStale(this.revision);
  }

  adoptedSurfaces(opts) {
    return this.driveLog.adoptedSurfaces(opts);
  }
}
