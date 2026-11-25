// Drive log: an append-only record of where the test car actually drove and
// which pavement revision carried each wheel.
//
// Every sample is stamped with the road revision in effect at that instant.
// Because the RevisionLedger rejects trajectories older than the latest road, a
// log entry also records whether it is still in effect (`active`) or has been
// superseded by a later edit (`stale`). The surface source is the exact part of
// pavement under the wheel: a road ribbon (with connection id) or a junction
// pad (with junction id), so the actual adopted surface is recoverable from the
// log rather than inferred.

export class DriveLog {
  constructor(vehicleId = 'sedan') {
    this.vehicleId = vehicleId;
    this.entries = [];
    this.sessions = [];
    this.sessionIndex = -1;
  }

  startSession({ routeId, revision, start }) {
    this.sessionIndex += 1;
    const session = {
      index: this.sessionIndex,
      routeId,
      startRevision: revision,
      startedAt: Date.now(),
      endedAt: null,
      endRevision: null,
      samples: 0,
    };
    this.sessions.push(session);
    return session;
  }

  endSession(revision) {
    const s = this.sessions[this.sessionIndex];
    if (!s) return null;
    s.endedAt = Date.now();
    s.endRevision = revision;
    return s;
  }

  // Append one telemetry sample.
  //   surface: { part: 'road'|'junction', connectionId?, junctionId?, height }
  //   revision: road revision the collision mesh was published for
  //   latestRevision: current road revision (may be newer mid-edit)
  sample(frame) {
    const entry = {
      t: this.entries.length,
      session: this.sessionIndex,
      time: frame.time ?? this.entries.length,
      x: frame.x,
      y: frame.y,
      z: frame.z ?? null,
      heading: frame.heading ?? null,
      speed: frame.speed ?? 0,
      revision: frame.revision,
      latestRevision: frame.latestRevision ?? frame.revision,
      surface: frame.surface
        ? {
            part: frame.surface.part,
            connectionId: frame.surface.connectionId ?? null,
            junctionId: frame.surface.junctionId ?? null,
            height: frame.surface.height ?? null,
            units: 'm',
          }
        : null,
      active: frame.revision === frame.latestRevision,
    };
    this.entries.push(entry);
    const s = this.sessions[this.sessionIndex];
    if (s) s.samples += 1;
    return entry;
  }

  // Mark every entry that does not match the latest road revision as stale.
  // Used after a rebuild so the editor can show which driving was invalidated.
  markStale(latestRevision) {
    let stale = 0;
    for (const e of this.entries) {
      const wasActive = e.active;
      e.active = e.revision === latestRevision;
      if (wasActive && !e.active) stale += 1;
    }
    return stale;
  }

  // The distinct pavement parts the vehicle actually rode on, in first-contact
  // order, restricted to entries that are still in effect unless includeStale.
  adoptedSurfaces({ includeStale = false, session = null } = {}) {
    const order = [];
    const seen = new Set();
    for (const e of this.entries) {
      if (session !== null && e.session !== session) continue;
      if (!includeStale && !e.active) continue;
      const su = e.surface;
      if (!su) continue;
      const key = su.part === 'junction' ? `junction:${su.junctionId}` : `road:${su.connectionId}`;
      if (seen.has(key)) continue;
      seen.add(key);
      order.push({ ...su, firstContact: e.time, revision: e.revision });
    }
    return order;
  }

  // Connection ids with at least one active wheel contact — feeds the EditGuard.
  activeContactConnectionIds() {
    const ids = new Set();
    for (const e of this.entries) {
      if (!e.active || !e.surface) continue;
      if (e.surface.part === 'road' && e.surface.connectionId) ids.add(e.surface.connectionId);
      if (e.surface.part === 'junction' && e.surface.connectionId) ids.add(e.surface.connectionId);
    }
    return [...ids];
  }

  // True when the last recorded position still sits on the named connection.
  lastContactOn(connectionId) {
    for (let i = this.entries.length - 1; i >= 0; i--) {
      const e = this.entries[i];
      if (!e.surface) continue;
      return e.surface.connectionId === connectionId && e.active;
    }
    return false;
  }

  record() {
    return {
      vehicleId: this.vehicleId,
      sessions: this.sessions,
      samples: this.entries.length,
      activeSamples: this.entries.filter((e) => e.active).length,
      staleSamples: this.entries.filter((e) => !e.active).length,
      adopted: this.adoptedSurfaces({ includeStale: true }),
    };
  }
}
