// Pipeline orchestrator: runs the clearly-bounded stages in order and hands the
// road revision from centreline routing to the collision mesh. This is the only
// module that knows every stage; each stage alone knows only its inputs.
//
// External behaviour is frozen relative to the monolithic buildProject: the
// returned object carries the same fields the editor and tests consume. The
// additions (revision, ledger, graphId) are additive and optional for old
// callers.

import { terrainToRecord } from './terrain.js';
import { shapesArea } from './clip.js';
import { uniqueGraphId } from './provenance.js';
import { RevisionLedger } from './revision.js';
import { normalizeProjectInput } from './stages/input.js';
import { routeConnections } from './stages/routing.js';
import { connectJunctions } from './stages/junction.js';
import { assemblePavement, publishCollisionMesh, findOverlaps } from './stages/mesh.js';
import { validateRoutes, runTurnTests, junctionReports } from './stages/acceptance.js';
import { gradeTerrain } from './stages/terrain-stage.js';
import { makeBuildRecord, buildRouteReports, aggregateViolations } from './stages/record.js';

function performanceNow() {
  return typeof performance !== 'undefined' ? performance.now() : Date.now();
}

export function buildPipeline(project, opts = {}) {
  const t0 = performanceNow();
  const graphId = opts.graphId || uniqueGraphId();
  const ledger = opts.ledger || new RevisionLedger();

  // Stage 1 — input.
  const input = normalizeProjectInput(project, opts);
  const { controls, terrain, nodes, connections, seed } = input;

  // Stage 2 — centreline routing. Every recompute is a new road revision.
  const roadRevision = ledger.roadChanged();
  const { routes, diagnostics } = routeConnections(input, { graphId, revision: roadRevision });

  // Stage 3 — junction connection and ribbons.
  const { junctions } = connectJunctions(input, routes, { graphId, revision: roadRevision });

  // Stage 4 — pavement union and the revision-stamped collision mesh.
  const pavement = assemblePavement(routes, junctions);
  const surfaceIndex = publishCollisionMesh(pavement, ledger, roadRevision);
  const pavementArea = shapesArea(pavement.shapes);

  // Stage 5 — acceptance on the published mesh.
  validateRoutes(project, routes, terrain);
  const turnTests = runTurnTests(project, junctions, routes, surfaceIndex);
  const jReports = junctionReports(junctions);
  const overlapIssues = findOverlaps(routes, junctions);

  // Stage 6 — terrain grading and seams.
  const { terrainMesh, seams } = gradeTerrain(input, routes, junctions, controls, surfaceIndex);

  // Stage 7 — record and aggregate.
  const routeReports = buildRouteReports(project, routes);
  const { violations, errorCount, warnCount } = aggregateViolations({
    routeReports,
    overlapIssues,
    seams,
    junctionReports: jReports,
    turnTests,
    nodes,
  });
  const record = makeBuildRecord({
    project,
    seed,
    controls,
    terrain,
    routes,
    junctions,
    turnTests,
    junctionReports: jReports,
    diagnostics,
    overlapIssues,
    seams,
    revision: roadRevision,
    graphId,
  });

  return {
    ok: errorCount === 0,
    graphId,
    revision: roadRevision,
    ledger,
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
    junctionReports: jReports,
    turnTests,
    pavement,
    surfaceIndex,
    pavementArea,
    seams,
    overlapIssues,
    violations,
    errorCount,
    warnCount,
    diagnostics,
    record,
    computeMs: performanceNow() - t0,
  };
}
