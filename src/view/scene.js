// Three.js scene construction for the road editor: terrain, pavement, markings,
// junctions, violation markers and the drivable vehicle.
// The kernel works in plan coordinates {x, y} with z as elevation; the viewer
// maps plan y to world Z and kernel z to world Y.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { vehicleById } from '../core/standards.js';

const PLAN = (x, y, z) => new THREE.Vector3(x, z, y);

function makeGradientMaterial(top, bottom) {
  return new THREE.ShaderMaterial({
    uniforms: {
      top: { value: new THREE.Color(top) },
      bottom: { value: new THREE.Color(bottom) },
    },
    vertexShader: /* glsl */ `
      varying float vY;
      void main() {
        vY = position.y;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    fragmentShader: /* glsl */ `
      uniform vec3 top; uniform vec3 bottom; varying float vY;
      void main() {
        float t = smoothstep(0.0, 1.0, clamp((vY + 30.0) / 80.0, 0.0, 1.0));
        gl_FragColor = vec4(mix(bottom, top, t), 1.0);
      }
    `,
  });
}

export class RoadView {
  constructor() {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.canvas = this.renderer.domElement;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x8fb8d8);
    this.scene.fog = new THREE.Fog(0x8fb8d8, 420, 900);

    this.camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.4, 3000);
    this.camera.position.set(0, 210, 250);

    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI * 0.495;
    this.controls.target.set(0, 0, 0);
    this.controls.maxDistance = 1100;

    const hemi = new THREE.HemisphereLight(0xd8ecff, 0x4b4535, 1.15);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xfff3dc, 2.3);
    sun.position.set(-220, 340, 180);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    const sc = sun.shadow.camera;
    sc.left = -420; sc.right = 420; sc.top = 420; sc.bottom = -420;
    sc.near = 1; sc.far = 1200;
    sun.shadow.bias = -0.0004;
    this.scene.add(sun);
    this.sun = sun;

    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.markerRoot = new THREE.Group();
    this.scene.add(this.markerRoot);
    this.overlayRoot = new THREE.Group();
    this.scene.add(this.overlayRoot);

    this.vehicle = null;
    this.pavementMesh = null;
    this.demoTurn = null;

    this.north = new THREE.Group();
    this.scene.add(this.north);
    this.buildCompass();

    window.addEventListener('resize', () => this.resize());
  }

  resize() {
    const w = window.innerWidth;
    const h = window.innerHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  buildCompass() {
    const dir = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, 0), 26, 0xffffff, 8, 5);
    this.north.add(dir);
  }

  clear() {
    for (const group of [this.root, this.markerRoot, this.overlayRoot]) {
      for (let i = group.children.length - 1; i >= 0; i--) {
        const c = group.children[i];
        group.remove(c);
        disposeTree(c);
      }
    }
    this.vehicle = null;
    this.pavementMesh = null;
    this.demoTurn = null;
  }

  // -------------------------------------------------------------------------
  // Terrain
  // -------------------------------------------------------------------------
  addTerrain(built) {
    const mesh = built.terrainMesh;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(mesh.colors, 3));
    geo.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
    geo.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.95,
      metalness: 0.0,
      flatShading: false,
    });
    const terrain = new THREE.Mesh(geo, mat);
    terrain.receiveShadow = true;
    terrain.name = 'terrain';
    this.root.add(terrain);
    this.terrainMesh = terrain;

    // Obstacles as translucent red volumes; forbidden zones must read clearly.
    for (const o of built.terrain.obstacles) {
      if (o.shape === 'circle') {
        const g = new THREE.CylinderGeometry(o.radius, o.radius, 6, 40, 1, true);
        const m = new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.17, side: THREE.DoubleSide, depthWrite: false });
        const cyl = new THREE.Mesh(g, m);
        cyl.position.copy(PLAN(o.x, o.y, (built.terrain.heightAt(o.x, o.y) || 0) + 3));
        this.root.add(cyl);
        const edge = new THREE.Mesh(
          new THREE.RingGeometry(o.radius - 0.4, o.radius, 64),
          new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.6, side: THREE.DoubleSide }),
        );
        edge.rotation.x = -Math.PI / 2;
        edge.position.copy(PLAN(o.x, o.y, (built.terrain.heightAt(o.x, o.y) || 0) + 0.2));
        this.root.add(edge);
      } else if (o.shape === 'polygon' && o.points?.length >= 3) {
        const shape = new THREE.Shape(o.points.map((p) => new THREE.Vector2(p.x, -p.y)));
        const g = new THREE.ShapeGeometry(shape);
        g.rotateX(-Math.PI / 2);
        const m = new THREE.MeshBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false });
        const poly = new THREE.Mesh(g, m);
        poly.position.y = (built.terrain.heightAt(o.points[0].x, o.points[0].y) || 0) + 0.25;
        this.root.add(poly);
        const pts = o.points.map((p) => {
          const h = (built.terrain.heightAt(p.x, p.y) || 0) + 0.35;
          return PLAN(p.x, p.y, h);
        });
        pts.push(pts[0].clone());
        const line = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(pts),
          new THREE.LineBasicMaterial({ color: 0xff3b30, transparent: true, opacity: 0.9 }),
        );
        this.root.add(line);
      }
    }
  }

  addWater(built) {
    // A simple plane at the lowest terrain level for visual grounding, clipped
    // to the terrain bounds. Purely decorative.
    const b = built.terrain.bounds || { minX: -260, maxX: 260, minZ: -260, maxZ: 260 };
    const g = new THREE.PlaneGeometry(b.maxX - b.minX, b.maxZ - b.minZ, 1, 1);
    g.rotateX(-Math.PI / 2);
    const m = new THREE.MeshStandardMaterial({
      color: 0x2f6f8f,
      transparent: true,
      opacity: 0.55,
      roughness: 0.15,
      metalness: 0.4,
    });
    const water = new THREE.Mesh(g, m);
    water.position.set((b.minX + b.maxX) / 2, -4.5, (b.minZ + b.maxZ) / 2);
    water.receiveShadow = true;
    this.root.add(water);
  }

  // -------------------------------------------------------------------------
  // Pavement, shoulders and markings
  // -------------------------------------------------------------------------
  addPavement(built) {
    const p = built.pavement;
    // Kernel triangles are wound counter-clockwise with +z (elevation) up in
    // plan space. Mapping plan y -> world Z is a mirror, so the world winding
    // flips; use DoubleSide so the drivable surface is visible from any angle.
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(p.positions, 3));
    geo.computeVertexNormals();
    // The plan->world mirror flips triangle winding, so the computed normals
    // point down; on a DoubleSide material the shader negates them for the
    // back faces we see from above, which yields correct upward lighting.
    // Do not flip them here or the drivable surface renders black.
    const mat = new THREE.MeshStandardMaterial({
      color: 0x3d4148,
      roughness: 0.88,
      metalness: 0.02,
      polygonOffset: true,
      polygonOffsetFactor: -1,
      polygonOffsetUnits: -1,
      side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.receiveShadow = true;
    mesh.name = 'pavement';
    this.root.add(mesh);
    this.pavementMesh = mesh;

    // Shoulders: a wider gravel band under the asphalt.
    const shoulderTris = [];
    for (const r of built.routes) shoulderTris.push(...(r.ribbon.shoulderTriangles || []));
    if (shoulderTris.length) {
      const sgeo = trianglesGeometry(shoulderTris);
      const smat = new THREE.MeshStandardMaterial({
        color: 0x8a7d62,
        roughness: 1.0,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: 1,
        polygonOffsetUnits: 1,
      });
      const shoulder = new THREE.Mesh(sgeo, smat);
      shoulder.receiveShadow = true;
      this.root.add(shoulder);
    }

    // Centre markings.
    const markTris = [];
    for (const r of built.routes) markTris.push(...(r.ribbon.markingTriangles || []));
    if (markTris.length) {
      const mgeo = trianglesGeometry(markTris, 0.035);
      const mmat = new THREE.MeshBasicMaterial({
        color: 0xf2e7c9,
        side: THREE.DoubleSide,
        polygonOffset: true,
        polygonOffsetFactor: -3,
        polygonOffsetUnits: -3,
      });
      this.root.add(new THREE.Mesh(mgeo, mmat));
    }

    // Kerb skirts hide the ribbon-to-terrain gap on fill.
    const skirtTris = [];
    for (const r of built.routes) skirtTris.push(...(r.ribbon.skirtTriangles || []));
    if (skirtTris.length) {
      const kgeo = trianglesGeometry(skirtTris);
      const kmat = new THREE.MeshStandardMaterial({ color: 0x6b6d70, roughness: 0.9, side: THREE.DoubleSide });
      this.root.add(new THREE.Mesh(kgeo, kmat));
    }
  }

  addJunctions(built) {
    for (const j of built.junctions) {
      if (!j.surface) continue;
      const tris = j.surface.triangles;
      if (!tris || !tris.length) continue;
      const geo = trianglesGeometry(tris);
      const mat = new THREE.MeshStandardMaterial({
        color: 0x36393f,
        roughness: 0.86,
        metalness: 0.03,
        polygonOffset: true,
        polygonOffsetFactor: -2,
        polygonOffsetUnits: -2,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.receiveShadow = true;
      mesh.name = `junction-${j.id}`;
      this.root.add(mesh);
    }
  }

  // -------------------------------------------------------------------------
  // Control points and route overlays
  // -------------------------------------------------------------------------
  addRoutes(built, handlers) {
    for (const r of built.routes) {
      // Centreline as a thin ribbon so the authored route is legible.
      const pts = r.samples.map((s) => PLAN(s.x, s.y, s.z + 0.12));
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x5fd0ff, transparent: true, opacity: 0.55 }),
      );
      this.root.add(line);

      // Control points: draggable spheres. The first/last are node endpoints and
      // are not draggable; interior points are.
      const control = r.control;
      for (let i = 0; i < control.length; i++) {
        const p = control[i];
        const isEnd = i === 0 || i === control.length - 1;
        const sample = nearestSample(r.samples, p.x, p.y);
        const z = sample ? sample.z : built.terrain.heightAt(p.x, p.y);
        const g = new THREE.SphereGeometry(isEnd ? 2.4 : 3.0, 18, 14);
        const m = new THREE.MeshStandardMaterial({
          color: isEnd ? 0xffffff : r.conn.locked ? 0xffb020 : 0x35e08a,
          emissive: isEnd ? 0x333333 : r.conn.locked ? 0x4a3000 : 0x0d3a22,
          roughness: 0.4,
          metalness: 0.1,
        });
        const sphere = new THREE.Mesh(g, m);
        sphere.position.copy(PLAN(p.x, p.y, z + 2.2));
        sphere.userData = {
          kind: 'control',
          connectionId: r.conn.id,
          index: i,
          isEnd,
        };
        sphere.castShadow = true;
        this.root.add(sphere);
        if (!isEnd) this.controlHandles = [...(this.controlHandles || []), sphere];

        if (!isEnd) {
          const stem = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
              PLAN(p.x, p.y, z + 0.2),
              PLAN(p.x, p.y, z + 1.2),
            ]),
            new THREE.LineBasicMaterial({ color: 0x35e08a, transparent: true, opacity: 0.7 }),
          );
          this.root.add(stem);
        }
      }
    }
  }

  // -------------------------------------------------------------------------
  // Violation markers
  // -------------------------------------------------------------------------
  addViolations(built) {
    const seen = new Set();
    const add = (plan, code, message, severity) => {
      if (!plan || !isFinite(plan.x) || !isFinite(plan.y)) return;
      const key = `${code}:${Math.round(plan.x)}:${Math.round(plan.y)}`;
      if (seen.has(key)) return;
      seen.add(key);
      const z = built.terrain.sampleHeight
        ? built.terrain.sampleHeight(plan.x, plan.y)
        : built.terrain.heightAt(plan.x, plan.y);
      const grounded = built.surfaceIndex.heightAt(plan.x, plan.y);
      const baseY = grounded != null ? grounded : z || 0;
      const color = severity === 'error' ? 0xff2d55 : 0xffb020;
      const h = severity === 'error' ? 14 : 9;
      const cone = new THREE.Mesh(
        new THREE.ConeGeometry(2.2, h, 12),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }),
      );
      cone.position.copy(PLAN(plan.x, plan.y, baseY + h / 2 + 0.5));
      cone.userData = { kind: 'violation', code, message, severity, plan };
      this.markerRoot.add(cone);

      const halo = new THREE.Mesh(
        new THREE.RingGeometry(2.6, 4.2, 24),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.75, side: THREE.DoubleSide }),
      );
      halo.rotation.x = -Math.PI / 2;
      halo.position.copy(PLAN(plan.x, plan.y, baseY + 0.3));
      this.markerRoot.add(halo);
    };
    for (const v of built.violations) {
      const plan = v.plan || (v.station != null && v.connectionId ? stationPoint(built, v) : null);
      add(plan, v.code, v.message, v.severity);
    }
    for (const s of built.seams) add(s.plan, s.code, s.message, s.severity);
  }

  // -------------------------------------------------------------------------
  // Turn trajectories
  // -------------------------------------------------------------------------
  addTurnPaths(built) {
    for (const t of built.turnTests) {
      if (!t.path || t.path.length < 2) continue;
      const pts = t.path.map((q) => {
        const h = built.surfaceIndex.heightAt(q.x, q.y);
        return PLAN(q.x, q.y, (h != null ? h : 0) + 0.18);
      });
      const pass = t.pass;
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({
          color: pass ? 0x54e6a0 : 0xff3b30,
          transparent: true,
          opacity: pass ? 0.9 : 1.0,
        }),
      );
      line.userData = { kind: 'turn', junctionId: t.junctionId };
      this.root.add(line);
    }
  }

  // Ghost swept envelope for the selected turn; toggled from the UI.
  showTurn(select) {
    this.clearDemo();
    if (!select) return;
    const { built, test } = select;
    const group = new THREE.Group();
    const path = test.path.map((q) => {
      const h = built.surfaceIndex.heightAt(q.x, q.y);
      return PLAN(q.x, q.y, (h != null ? h : 0) + 0.2);
    });
    group.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(path),
      new THREE.LineBasicMaterial({ color: 0xffd166 }),
    ));
    const vehicle = vehicleById(test.vehicleId || 'sedan');
    const halfLen = vehicle.length / 2;
    const halfW = vehicle.width / 2;
    const mat = new THREE.MeshBasicMaterial({
      color: test.pass ? 0x54e6a0 : 0xff3b30,
      transparent: true,
      opacity: 0.28,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    for (let i = 0; i < test.path.length; i += 2) {
      const a = test.path[Math.max(0, i - 1)];
      const b = test.path[Math.min(test.path.length - 1, i + 1)];
      const heading = Math.atan2(b.y - a.y, b.x - a.x);
      const q = test.path[i];
      const h = built.surfaceIndex.heightAt(q.x, q.y);
      if (h == null) continue;
      const g = new THREE.PlaneGeometry(vehicle.length, vehicle.width);
      g.rotateX(-Math.PI / 2);
      const mesh = new THREE.Mesh(g, mat);
      const cos = Math.cos(heading);
      const sin = Math.sin(heading);
      const cx = q.x + cos * (vehicle.wheelbase / 2);
      const cy = q.y + sin * (vehicle.wheelbase / 2);
      mesh.position.copy(PLAN(cx, cy, h + 0.35));
      mesh.rotation.y = -heading;
      group.add(mesh);
    }
    this.overlayRoot.add(group);
    this.demoTurn = group;
  }

  clearDemo() {
    if (this.demoTurn) {
      this.overlayRoot.remove(this.demoTurn);
      disposeTree(this.demoTurn);
      this.demoTurn = null;
    }
  }

  // -------------------------------------------------------------------------
  // Drivable vehicle
  // -------------------------------------------------------------------------
  addVehicle(built, vehicleId) {
    this.removeVehicle();
    const vehicle = vehicleById(vehicleId);
    const group = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.BoxGeometry(vehicle.width, 1.15, vehicle.length),
      new THREE.MeshStandardMaterial({ color: 0xf25f4b, roughness: 0.35, metalness: 0.35 }),
    );
    body.position.y = 0.75;
    body.castShadow = true;
    group.add(body);
    const cabin = new THREE.Mesh(
      new THREE.BoxGeometry(vehicle.width * 0.82, 0.62, vehicle.length * 0.36),
      new THREE.MeshStandardMaterial({ color: 0x2b3a45, roughness: 0.25, metalness: 0.5 }),
    );
    cabin.position.set(0, 1.5, -vehicle.length * 0.06);
    cabin.castShadow = true;
    group.add(cabin);
    const wheelGeo = new THREE.CylinderGeometry(0.42, 0.42, 0.32, 16);
    wheelGeo.rotateZ(Math.PI / 2);
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x151515, roughness: 0.9 });
    const wb = vehicle.wheelbase / 2;
    const track = vehicle.track / 2;
    for (const [sx, sz] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat);
      wheel.position.set(sx * track, 0.42, sz * wb);
      wheel.castShadow = true;
      group.add(wheel);
    }
    this.root.add(group);
    this.vehicle = {
      group,
      spec: vehicle,
      plan: { x: 0, y: 0 },
      heading: 0,
      speed: 0,
      steering: 0,
    };
    return this.vehicle;
  }

  removeVehicle() {
    if (this.vehicle) {
      this.root.remove(this.vehicle.group);
      disposeTree(this.vehicle.group);
      this.vehicle = null;
    }
  }

  placeVehicle(route, built) {
    if (!this.vehicle) return;
    const samples = route.samples;
    const mid = samples[Math.floor(samples.length * 0.18)];
    const idx = Math.floor(samples.length * 0.18);
    const next = samples[Math.min(samples.length - 1, idx + 3)];
    const heading = Math.atan2(next.y - mid.y, next.x - mid.x);
    this.vehicle.plan = { x: mid.x, y: mid.y };
    this.vehicle.heading = heading;
    this.vehicle.speed = 0;
    this.syncVehicle(built);
    return { route, station: mid.s };
  }

  // Advance the vehicle along the pavement using a bicycle model, sampling the
  // *pavement surface* for height so wheels sit on what is rendered.
  stepVehicle(dt, input, built) {
    const v = this.vehicle;
    if (!v || !built) return;
    const spec = v.spec;
    const accel = input.throttle * 9.0 - input.brake * 14.0;
    const drag = 0.55 * v.speed * Math.abs(v.speed) * 0.12 + (input.throttle === 0 ? v.speed * 0.6 : 0);
    v.speed = Math.max(-6, Math.min(30, v.speed + (accel - drag) * dt));
    if (Math.abs(v.speed) < 0.02 && input.throttle === 0) v.speed = 0;
    const steerTarget = input.steer * spec.maxSteer;
    v.steering += (steerTarget - v.steering) * Math.min(1, dt * 6.5);
    // Bicycle model about the rear axle.
    const rear = {
      x: v.plan.x - Math.cos(v.heading) * (spec.wheelbase / 2),
      y: v.plan.y - Math.sin(v.heading) * (spec.wheelbase / 2),
    };
    const dist = v.speed * dt;
    const angular = (dist * Math.tan(v.steering)) / spec.wheelbase;
    v.heading += angular;
    rear.x += Math.cos(v.heading) * dist;
    rear.y += Math.sin(v.heading) * dist;
    const nx = rear.x + Math.cos(v.heading) * (spec.wheelbase / 2);
    const ny = rear.y + Math.sin(v.heading) * (spec.wheelbase / 2);
    // Block driving off the pavement: if the centre leaves pavement, stop and
    // keep the previous position rather than teleporting.
    const onPave = built.surfaceIndex.heightAt(nx, ny) != null;
    if (onPave) {
      v.plan = { x: nx, y: ny };
    } else {
      v.speed = Math.min(v.speed, 0);
    }
    this.syncVehicle(built);
  }

  syncVehicle(built) {
    const v = this.vehicle;
    if (!v) return;
    const spec = v.spec;
    const h = built.surfaceIndex.heightAt(v.plan.x, v.plan.y);
    const ground = built.terrain.sampleHeight
      ? built.terrain.sampleHeight(v.plan.x, v.plan.y)
      : built.terrain.heightAt(v.plan.x, v.plan.y);
    const y = h != null ? h : ground;
    // Body group is authored with its wheels on y=0 and centred on the middle
    // of the wheelbase; offset to plan position.
    v.group.position.copy(PLAN(v.plan.x, v.plan.y, y + 0.02));
    v.group.rotation.y = -v.heading;
    // Pitch and roll follow the pavement plane so the body does not clip.
    const probe = 1.4;
    const fx = Math.cos(v.heading);
    const fy = Math.sin(v.heading);
    const rx = Math.cos(v.heading + Math.PI / 2);
    const ry = Math.sin(v.heading + Math.PI / 2);
    const hF = this.samplePavement(built, v.plan.x + fx * probe, v.plan.y + fy * probe);
    const hB = this.samplePavement(built, v.plan.x - fx * probe, v.plan.y - fy * probe);
    const hR = this.samplePavement(built, v.plan.x + rx * probe, v.plan.y + ry * probe);
    const hL = this.samplePavement(built, v.plan.x - rx * probe, v.plan.y - ry * probe);
    const pitch = Math.atan2(hF - hB, probe * 2);
    const roll = Math.atan2(hR - hL, probe * 2);
    v.group.rotation.x = pitch;
    v.group.rotation.z = -roll;
  }

  samplePavement(built, x, y) {
    const h = built.surfaceIndex.heightAt(x, y);
    if (h != null) return h;
    return built.terrain.sampleHeight
      ? built.terrain.sampleHeight(x, y)
      : built.terrain.heightAt(x, y);
  }

  // -------------------------------------------------------------------------
  // Picking
  // -------------------------------------------------------------------------
  pickControl(event) {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, this.camera);
    const handles = (this.controlHandles || []).filter((h) => h.parent);
    const hits = ray.intersectObjects(handles, false);
    return hits.length ? hits[0].object : null;
  }

  // Project a screen point onto the terrain plane for dragging.
  planPoint(event, fallbackY = 0) {
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, this.camera);
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -fallbackY);
    const point = new THREE.Vector3();
    if (!ray.ray.intersectPlane(plane, point)) return null;
    return { x: point.x, y: point.z };
  }

  focusOn(plan, height = 90) {
    this.controls.target.copy(PLAN(plan.x, plan.y, 0));
    this.camera.position.set(plan.x - height * 0.9, height * 1.35, plan.y + height * 1.1);
    this.controls.update();
  }

  // Frame the whole project, not just its centre.
  frame(built) {
    const xs = [];
    const ys = [];
    for (const n of built.nodes) {
      xs.push(n.x);
      ys.push(n.y);
    }
    for (const r of built.routes) {
      for (const s of r.samples) {
        xs.push(s.x);
        ys.push(s.y);
      }
    }
    if (!xs.length) return;
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    const span = Math.max(maxX - minX, maxY - minY, 80);
    const dist = span * 1.25;
    this.controls.target.set(cx, 0, cy);
    this.camera.position.set(cx - dist * 0.55, dist * 0.72, cy + dist * 0.85);
    this.camera.near = Math.max(0.4, dist / 900);
    this.camera.far = dist * 8;
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  render() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  animate(cb) {
    const loop = (t) => {
      cb(t);
      this.render();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
function trianglesGeometry(tris, lift = 0) {
  const arr = new Float32Array(tris.length * 9);
  let i = 0;
  for (const t of tris) {
    for (const v of t) {
      arr[i++] = v.x;
      arr[i++] = v.z + lift;
      arr[i++] = v.y;
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
  geo.computeVertexNormals();
  // Same mirror caveat as the pavement: keep the downward computed normals and
  // let the DoubleSide shader negate them for back faces.
  return geo;
}

function nearestSample(samples, x, y) {
  let best = null;
  let bestD = Infinity;
  for (const s of samples) {
    const d = (s.x - x) ** 2 + (s.y - y) ** 2;
    if (d < bestD) {
      bestD = d;
      best = s;
    }
  }
  return best;
}

function stationPoint(built, v) {
  const route = built.routes.find((r) => r.conn.id === v.connectionId);
  if (!route) return null;
  let best = null;
  let bestD = Infinity;
  for (const s of route.samples) {
    const d = Math.abs(s.s - v.station);
    if (d < bestD) {
      bestD = d;
      best = s;
    }
  }
  return best ? { x: best.x, y: best.y } : null;
}

function disposeTree(obj) {
  obj.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    if (o.material) {
      const mats = Array.isArray(o.material) ? o.material : [o.material];
      for (const m of mats) {
        if (m.map) m.map.dispose();
        m.dispose();
      }
    }
  });
}
