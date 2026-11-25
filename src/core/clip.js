// Boolean polygon operations via clipper-lib, in plan coordinates (metres).
// Results preserve ring nesting so pavement surfaces can be triangulated with holes.
import ClipperLib from 'clipper-lib';
import { polygonArea } from './math.js';

const SCALE = 1000;

function toPath(points) {
  const path = points.map((p) => ({ X: Math.round(p.x * SCALE), Y: Math.round(p.y * SCALE) }));
  if (ClipperLib.Clipper.Orientation(path)) path.reverse();
  return path;
}

function fromPath(path) {
  return path.map((p) => ({ x: p.X / SCALE, y: p.Y / SCALE }));
}

function flattenTree(node, out, depth = 0) {
  for (const child of node.Childs()) {
    const ring = fromPath(child.Contour());
    if (Math.abs(ClipperLib.Clipper.Area(child.Contour())) < 1) {
      flattenTree(child, out, depth);
      continue;
    }
    if (!child.IsHole()) {
      const shape = { outer: ring, holes: [] };
      out.push(shape);
      collectHoles(child, shape.holes);
    }
    // Holes are attached to their enclosing outer; orphans are dropped.
  }
  return out;
}

function collectHoles(node, holes) {
  for (const child of node.Childs()) {
    if (child.IsHole()) {
      const ring = fromPath(child.Contour());
      if (Math.abs(ClipperLib.Clipper.Area(child.Contour())) >= 1) holes.push(ring);
      collectHoles(child, holes);
    }
  }
}

function run(subject, clip, clipType) {
  const c = new ClipperLib.Clipper();
  if (subject.length) c.AddPaths(subject, ClipperLib.PolyType.ptSubject, true);
  if (clip.length) c.AddPaths(clip, ClipperLib.PolyType.ptClip, true);
  const tree = new ClipperLib.PolyTree();
  c.Execute(clipType, tree, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero);
  return flattenTree(tree, []);
}

function shapesToPaths(shapes) {
  const paths = [];
  for (const s of shapes) {
    paths.push(toPath(s.outer));
    for (const h of s.holes) paths.push(toPath(h));
  }
  return paths;
}

export function unionShapes(shapes) {
  if (!shapes.length) return [];
  return run(shapesToPaths(shapes), [], ClipperLib.ClipType.ctUnion);
}

export function intersectShapes(a, b) {
  if (!a.length || !b.length) return [];
  return run(shapesToPaths(a), shapesToPaths(b), ClipperLib.ClipType.ctIntersection);
}

export function differenceShapes(a, b) {
  if (!a.length) return [];
  return run(shapesToPaths(a), shapesToPaths(b), ClipperLib.ClipType.ctDifference);
}

// Convenience wrappers for plain rings.
export function ringsAsShapes(rings) {
  return rings.filter((r) => r.length >= 3).map((r) => ({ outer: r, holes: [] }));
}

export function shapesToRings(shapes) {
  const rings = [];
  for (const s of shapes) {
    rings.push(s.outer);
    for (const h of s.holes) rings.push(h);
  }
  return rings;
}

export function offsetRings(rings, delta) {
  const co = new ClipperLib.ClipperOffset(2, 0.25);
  co.AddPaths(rings.map(toPath), ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);
  const tree = new ClipperLib.PolyTree();
  co.Execute(tree, delta * SCALE);
  return flattenTree(tree, []);
}

export function polygonsArea(rings) {
  return rings.reduce((s, p) => s + Math.abs(polygonArea(p)), 0);
}

export function shapesArea(shapes) {
  let a = 0;
  for (const s of shapes) {
    a += Math.abs(polygonArea(s.outer));
    for (const h of s.holes) a -= Math.abs(polygonArea(h));
  }
  return a;
}

export function overlapAreaShapes(a, b) {
  return shapesArea(intersectShapes(a, b));
}

export function ringContainsPolygon(outer, inner) {
  // Cheap check: all inner vertices inside outer, via clipper intersection area.
  const inter = intersectShapes([{ outer, holes: [] }], [{ outer: inner, holes: [] }]);
  return shapesArea(inter) > Math.abs(polygonArea(inner)) - 1e-3;
}

export function pointInRing(pt, ring) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const a = ring[i];
    const b = ring[j];
    if (a.y > pt.y !== b.y > pt.y && pt.x < ((b.x - a.x) * (pt.y - a.y)) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
  }
  return inside;
}
