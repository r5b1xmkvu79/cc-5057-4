// Edit guard: which pavement may be replaced while a vehicle is driving.
//
// The editor allows junctions and ramps to be edited while a test car is on the
// network. Two rules govern what may be torn out:
//
//   1. A branch with no tyre contact is free to re-mesh immediately.
//   2. A connection (or junction arm) that currently has tyre contact must be
//      retained until the vehicle has driven clear, or the vehicle is
//      explicitly returned to a safe parking point first.
//
// The guard is pure kernel state: it knows nothing about the DOM. The editor
// asks `canRemove` before tearing out geometry and either defers the swap or
// calls `requestSafeReturn` to move the vehicle off the affected pavement.

// Contact is retained for a short grace window after the last wheel leaves so a
// vehicle oscillating on a seam does not flicker the pavement under itself.
const CONTACT_GRACE_MS = 600;

export class EditGuard {
  constructor() {
    // connectionId -> { since, lastContact, wheels, point }
    this.contact = new Map();
    // connectionId -> parked safe point supplied by the editor.
    this.safePoints = new Map();
    // Pending removals blocked by contact: connectionId -> { reason, at }.
    this.deferred = new Map();
  }

  // Record a wheel contact sample. `point` is the plan position of the vehicle
  // centre; `connections` lists every connection whose pavement any wheel
  // touches at this instant (junctions report their incident connections).
  markContact(connections, point, now = Date.now()) {
    for (const id of connections) {
      const prev = this.contact.get(id);
      this.contact.set(id, {
        since: prev?.since ?? now,
        lastContact: now,
        wheels: (prev?.wheels ?? 0) + 1,
        point: point ? { x: point.x, y: point.y } : prev?.point ?? null,
      });
    }
  }

  // The editor tells the guard where a vehicle can safely wait for a given
  // connection (typically a station well back from the junction, on pavement
  // that is not part of the edit).
  registerSafePoint(connectionId, point) {
    this.safePoints.set(connectionId, { x: point.x, y: point.y });
  }

  hasContact(connectionId, now = Date.now()) {
    const c = this.contact.get(connectionId);
    if (!c) return false;
    return now - c.lastContact <= CONTACT_GRACE_MS;
  }

  occupiedConnections(now = Date.now()) {
    return [...this.contact.keys()].filter((id) => this.hasContact(id, now));
  }

  // Decide whether a connection may be removed/re-meshed right now.
  //   status: 'free'    no recent tyre contact, safe to re-mesh
  //           'held'    tyres on (or recently on) the pavement, retain
  //           'parked'  a safe point exists; caller may move the vehicle there
  canRemove(connectionId, now = Date.now()) {
    if (!this.hasContact(connectionId, now)) {
      return { status: 'free', connectionId };
    }
    if (this.safePoints.has(connectionId)) {
      return {
        status: 'parked',
        connectionId,
        safePoint: this.safePoints.get(connectionId),
        contact: this.contact.get(connectionId)?.point ?? null,
      };
    }
    return {
      status: 'held',
      connectionId,
      contact: this.contact.get(connectionId)?.point ?? null,
      lastContact: this.contact.get(connectionId)?.lastContact ?? now,
    };
  }

  // Batch check for a whole edit set. Unoccupied branches are returned as
  // removable; occupied ones as retained, with the safe-return option where one
  // is available.
  planEdit(connectionIds, now = Date.now()) {
    const removable = [];
    const retained = [];
    for (const id of connectionIds) {
      const decision = this.canRemove(id, now);
      if (decision.status === 'free') removable.push(id);
      else retained.push(decision);
    }
    return { removable, retained };
  }

  // Explicitly send the vehicle back to a registered safe point so the edit can
  // proceed. Returns the point to move the vehicle to, or null if none exists.
  // Contact for the connection is cleared only after the editor confirms the
  // move with `confirmSafeReturn`.
  requestSafeReturn(connectionId) {
    const point = this.safePoints.get(connectionId);
    if (!point) return null;
    this.deferred.set(connectionId, { reason: 'safe-return', at: Date.now() });
    return { x: point.x, y: point.y };
  }

  confirmSafeReturn(connectionId) {
    this.contact.delete(connectionId);
    this.deferred.delete(connectionId);
  }

  // Called after a vehicle drives clear and the editor wants the slate clean.
  release(connectionId) {
    this.contact.delete(connectionId);
  }

  reset() {
    this.contact.clear();
    this.deferred.clear();
  }
}
