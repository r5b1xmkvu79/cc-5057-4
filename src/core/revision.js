// Revision hand-off contract between centreline recomputation and the collision
// mesh. The centreline solver and the collision index run on different cadence
// during an interactive drag: the solver can be bumped several times while a
// vehicle is still driving on a previously published surface.
//
// Rule: a vehicle trajectory (and any collision query) is only valid against
// the mesh revision it was evaluated on. After a sequence of drags, trajectories
// stamped with an older road revision are stale and must not take effect; only
// a trajectory matching the latest road is accepted.

import { createRevision } from './provenance.js';

export class RevisionLedger {
  constructor() {
    this.road = createRevision(0);
    this.roadRevision = 0;
    // The mesh always lags until publishMesh is called for a road revision.
    this.meshRevision = 0;
    this.publishedAt = new Map(); // revision -> timestamp
  }

  // Called when the centreline of one or more connections is recomputed.
  roadChanged() {
    this.roadRevision = this.road.bump();
    return this.roadRevision;
  }

  currentRoad() {
    return this.roadRevision;
  }

  // The collision mesh is rebuilt for a specific road revision. Publishing an
  // older revision is rejected: the caller must rebuild against the latest road.
  publishMesh(roadRevision) {
    if (roadRevision < this.roadRevision) {
      const err = new Error(
        `stale mesh: built for revision ${roadRevision}, road is at ${this.roadRevision}`,
      );
      err.code = 'stale-mesh';
      err.roadRevision = this.roadRevision;
      err.meshRevision = roadRevision;
      throw err;
    }
    this.meshRevision = roadRevision;
    this.publishedAt.set(roadRevision, Date.now());
    return this.meshRevision;
  }

  currentMesh() {
    return this.meshRevision;
  }

  meshIsCurrent() {
    return this.meshRevision === this.roadRevision;
  }

  // A trajectory is accepted only when the road it was driven on is still the
  // latest and its mesh was published for that revision.
  trajectoryAcceptable(trajectoryRevision) {
    return (
      trajectoryRevision === this.roadRevision &&
      trajectoryRevision === this.meshRevision
    );
  }
}
