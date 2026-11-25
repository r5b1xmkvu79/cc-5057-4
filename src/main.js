// Level editor entry point.
// Owns the project state, rebuilds it through the staged road geometry
// interface, and wires the Three.js viewer to the control panel. All geometry
// and validation come from the kernel (src/core via geometry-interface.js); this
// file only orchestrates and renders.

import { rebuildWithLocked, interiorControl } from './core/build.js';
import { RoadGeometryInterface } from './core/geometry-interface.js';
import { sampleProject, SAMPLE_IDS } from './project.js';
import { VEHICLES, vehicleById, ROAD_CLASSES } from './core/standards.js';
import { RoadView } from './view/scene.js';

const SCENE_LABELS = {
  arterial: '主支路 T 形',
  slope: '跨坡地',
  multibranch: '近距离多岔',
};

const state = {
  sceneId: 'arterial',
  project: null,
  iface: null,
  built: null,
  selectedConnection: null,
  drive: {
    active: false,
    routeId: null,
    keys: new Set(),
  },
  dragging: null,
  selectedTurn: null,
};

// Geometry exceptions raised by the kernel are surfaced through the editor UI,
// never swallowed or reported as a pass.
function geometryError(err) {
  console.error(err);
  toast(`道路几何异常: ${err.message}`, 'error');
}

const view = new RoadView();
const canvas = document.getElementById('view');
canvas.replaceWith(view.canvas);
view.canvas.id = 'view';

const el = (id) => document.getElementById(id);
const ui = {
  sceneSwitch: el('scene-switch'),
  sceneDesc: el('scene-desc'),
  connSelect: el('conn-select'),
  lockBtn: el('btn-lock'),
  vehicleSelect: el('vehicle-select'),
  solveBtn: el('btn-solve'),
  resetBtn: el('btn-reset'),
  driveBtn: el('btn-drive'),
  exitDriveBtn: el('btn-exit-drive'),
  driveKeys: el('drive-keys'),
  driveRoute: el('drive-route'),
  telemetry: el('telemetry'),
  routesList: el('routes-list'),
  junctionsList: el('junctions-list'),
  violationsList: el('violations-list'),
  recordBox: el('record-box'),
  copyRecord: el('btn-copy-record'),
  statusBuild: el('status-build'),
  statusTurns: el('status-turns'),
  statusViol: el('status-viol'),
  toast: el('toast'),
  loading: el('loading'),
  pickHint: el('pick-hint'),
};

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------
function boot() {
  buildSceneSwitch();
  ui.vehicleSelect.innerHTML = Object.values(VEHICLES)
    .map((v) => `<option value="${v.id}">${v.label}</option>`)
    .join('');
  loadScene(state.sceneId);

  ui.solveBtn.addEventListener('click', resolveRest);
  ui.resetBtn.addEventListener('click', () => loadScene(state.sceneId));
  ui.lockBtn.addEventListener('click', toggleLock);
  ui.connSelect.addEventListener('change', () => {
    state.selectedConnection = ui.connSelect.value;
    state.selectedTurn = null;
    view.showTurn(null);
    renderLists();
    renderLockButton();
  });
  ui.vehicleSelect.addEventListener('change', () => {
    state.project.vehicleId = ui.vehicleSelect.value;
    rebuild();
  });
  ui.driveBtn.addEventListener('click', enterDrive);
  ui.exitDriveBtn.addEventListener('click', exitDrive);
  ui.driveRoute.addEventListener('change', () => {
    state.drive.routeId = ui.driveRoute.value;
    placeDriveVehicle();
  });
  ui.copyRecord.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(JSON.stringify(state.built.record, null, 2));
      toast('构建记录已复制到剪贴板', 'ok');
    } catch {
      toast('复制失败，请手动选择文本', 'error');
    }
  });

  bindPointer();
  bindKeys();

  view.animate((t) => {
    if (state.drive.active) driveTick(t);
  });
}

function buildSceneSwitch() {
  ui.sceneSwitch.innerHTML = '';
  for (const id of SAMPLE_IDS) {
    const b = document.createElement('button');
    b.textContent = SCENE_LABELS[id] || id;
    b.dataset.scene = id;
    b.addEventListener('click', () => loadScene(id));
    ui.sceneSwitch.appendChild(b);
  }
}

// ---------------------------------------------------------------------------
// Build pipeline
// ---------------------------------------------------------------------------
function loadScene(id) {
  state.sceneId = id;
  state.project = sampleProject(id);
  state.selectedConnection = state.project.connections[0]?.id || null;
  state.selectedTurn = null;
  state.drive.active = false;
  state.drive.routeId = null;
  exitDrive();
  ui.vehicleSelect.value = state.project.vehicleId || 'sedan';
  // A fresh scene gets a fresh geometry interface (and therefore a fresh
  // revision ledger and drive log).
  state.iface = new RoadGeometryInterface(state.project);
  state.built = state.iface.built;
  rebuild();
}

function rebuild() {
  showLoading(true);
  // Yield a frame so the loading indicator paints before the synchronous kernel.
  requestAnimationFrame(() => {
    try {
      state.built = state.iface.rebuild(state.project);
      state.built.vehicleId = state.project.vehicleId;
      // Driving recorded on an older road revision is now stale; the log keeps
      // it for inspection but it can no longer take effect.
      state.iface.syncDriveLog();
      renderSceneGeometry();
      renderLists();
      renderStatus();
      renderRecord();
      populateConnectionSelect();
      populateDriveRoutes();
      renderLockButton();
      if (state.drive.active) placeDriveVehicle();
      if (!state.hasFramed?.[state.sceneId]) {
        view.frame(state.built);
        state.hasFramed = { ...(state.hasFramed || {}), [state.sceneId]: true };
      }
    } catch (err) {
      geometryError(err);
    } finally {
      showLoading(false);
    }
  });
}

function resolveRest() {
  const id = state.selectedConnection;
  if (!id) return;
  const route = state.built.routes.find((r) => r.conn.id === id);
  if (!route) return;
  const interior = interiorControl(route);
  showLoading(true);
  requestAnimationFrame(() => {
    try {
      const next = rebuildWithLocked(state.project, id, interior);
      // Persist lock state into the working project so subsequent edits keep it.
      const conn = state.project.connections.find((c) => c.id === id);
      conn.controlPoints = interior;
      conn.locked = true;
      // Re-enter through the staged interface so the revision ledger advances.
      state.built = state.iface.rebuild(state.project);
      state.built.vehicleId = state.project.vehicleId;
      state.iface.syncDriveLog();
      renderSceneGeometry();
      renderLists();
      renderStatus();
      renderRecord();
      renderLockButton();
      toast(`已锁定 ${route.conn.name || id}，其余连接重新求解`, 'ok');
    } catch (err) {
      geometryError(err);
    } finally {
      showLoading(false);
    }
  });
}

function toggleLock() {
  const id = state.selectedConnection;
  const conn = state.project.connections.find((c) => c.id === id);
  if (!conn) return;
  if (conn.locked) {
    conn.locked = false;
    // Drop authored controls back to the sample's originals by reloading only
    // this connection's controls from the pristine sample.
    const fresh = sampleProject(state.sceneId).connections.find((c) => c.id === id);
    conn.controlPoints = fresh?.controlPoints ? fresh.controlPoints.map((p) => ({ ...p })) : undefined;
    rebuild();
    toast('已解锁，恢复场景默认控制点', 'ok');
  } else {
    const route = state.built.routes.find((r) => r.conn.id === id);
    if (route) {
      conn.controlPoints = interiorControl(route);
      conn.locked = true;
    }
    rebuild();
    toast('已锁定本段路线', 'ok');
  }
  renderLockButton();
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------
function renderSceneGeometry() {
  view.clear();
  view.controlHandles = [];
  view.addTerrain(state.built);
  view.addWater(state.built);
  view.addPavement(state.built);
  view.addJunctions(state.built);
  view.addRoutes(state.built);
  view.addViolations(state.built);
  view.addTurnPaths(state.built);
}

function renderStatus() {
  const b = state.built;
  const errs = b.violations.filter((v) => v.severity === 'error').length;
  const turnFails = b.turnTests.filter((t) => !t.pass).length;
  ui.statusBuild.textContent = `构建 ${b.errorCount === 0 ? '通过' : '有误'} · ${b.computeMs.toFixed(0)}ms`;
  ui.statusBuild.className = `pill ${b.errorCount === 0 ? 'ok' : 'bad'}`;
  ui.statusTurns.textContent = `转弯 ${b.turnTests.length - turnFails}/${b.turnTests.length} 可行`;
  ui.statusTurns.className = `pill ${turnFails === 0 ? 'ok' : 'bad'}`;
  ui.statusViol.textContent = errs === 0 ? '无违规' : `${errs} 处违规`;
  ui.statusViol.className = `pill ${errs === 0 ? 'ok' : 'bad'}`;
}

function renderLists() {
  renderRoutes();
  renderJunctions();
  renderViolations();
}

function renderRoutes() {
  const b = state.built;
  ui.routesList.innerHTML = '';
  for (const rr of b.routeReports) {
    const div = document.createElement('div');
    const isSel = rr.id === state.selectedConnection;
    div.className = `item clickable${isSel ? ' selected' : ''}`;
    const conn = b.connections.find((c) => c.id === rr.id);
    const cls = ROAD_CLASSES[rr.classId] || ROAD_CLASSES.local;
    const errTag = rr.errorCount > 0
      ? `<span class="tag bad">${rr.errorCount} 违规</span>`
      : `<span class="tag ok">可行</span>`;
    const lockTag = conn?.locked ? '<span class="tag lock">已锁定</span>' : '';
    const decisions = (rr.decisions || [])
      .filter((d) => d.kind !== 'widen' || (d.value || 0) > 0.05)
      .map((d) => d.message)
      .join(' · ');
    div.innerHTML = `
      <div class="head">
        <span class="name">${rr.name || rr.id}</span>
        <span class="tag info">${cls.label.split('/')[1]?.trim() || rr.classId}</span>
        ${lockTag}${errTag}
      </div>
      <div class="metrics">
        长 <b>${rr.length.toFixed(1)}m</b> · 最小半径 ${rr.minRadius && isFinite(rr.minRadius) ? rr.minRadius.toFixed(1) + 'm' : '—'} ·
        最大纵坡 ${(rr.maxGrade * 100).toFixed(1)}% · 加宽 ${rr.maxWidening.toFixed(2)}m
        <br/>扫掠: ${rr.sweep?.pass ? '<span class="ok">通过</span>' : '<em>未通过</em>'}
        ${decisions ? ` · ${decisions}` : ''}
      </div>`;
    div.addEventListener('click', () => {
      state.selectedConnection = rr.id;
      ui.connSelect.value = rr.id;
      state.selectedTurn = null;
      view.showTurn(null);
      renderRoutes();
      renderLockButton();
    });
    ui.routesList.appendChild(div);
  }
}

function renderJunctions() {
  const b = state.built;
  ui.junctionsList.innerHTML = '';
  if (!b.junctionReports.length) {
    ui.junctionsList.innerHTML = '<div class="muted">本场景没有路口</div>';
    return;
  }
  for (const j of b.junctionReports) {
    const card = document.createElement('div');
    card.className = 'item';
    const issue = j.issues.length && j.issues[0] === 'no-ports';
    const gradeBad = j.maxGrade > 0.1;
    card.innerHTML = `
      <div class="head">
        <span class="name">路口 ${j.id.replace('j-', '')}</span>
        <span class="tag info">${j.arms} 臂</span>
        <span class="tag ${gradeBad ? 'warn' : 'ok'}">纵坡 ${(j.maxGrade * 100).toFixed(1)}%</span>
      </div>
      <div class="metrics">
        面积 ${j.area.toFixed(0)}m² · 端口高差 ${j.heightSpread.toFixed(2)}m · 三角面 ${j.triangles}
        ${issue ? ' · <em>无端口</em>' : ''}
      </div>`;
    ui.junctionsList.appendChild(card);
    const turns = b.turnTests.filter((t) => t.junctionId === j.id);
    for (const t of turns) {
      const row = document.createElement('div');
      row.className = 'item clickable';
      const selected = state.selectedTurn === t;
      if (selected) row.classList.add('selected');
      const steerPct = Math.round((t.maxSteerDeg / (vehicleById(state.project.vehicleId).maxSteer * 180 / Math.PI)) * 100);
      row.innerHTML = `
        <div class="head">
          <span class="name" style="font-weight:500">${armName(b, t.from.connectionId)} → ${armName(b, t.to.connectionId)}</span>
          <span class="tag ${t.pass ? 'ok' : 'bad'}">${t.pass ? '可行' : '不可行'}</span>
        </div>
        <div class="metrics">
          转角 ${t.turningAngleDeg.toFixed(0)}° · 转弯半径 ${isFinite(t.turnRadius) ? t.turnRadius.toFixed(1) + 'm' : '—'} ·
          最大转向 ${t.maxSteerDeg.toFixed(0)}° (${steerPct}%) · 离面 ${t.offPavement} 点 · 高差 ${t.maxHeightStep?.toFixed(2) ?? '—'}m
        </div>`;
      row.addEventListener('click', (e) => {
        e.stopPropagation();
        selectTurn(t, row);
      });
      ui.junctionsList.appendChild(row);
    }
  }
}

function selectTurn(t, rowEl) {
  if (state.selectedTurn === t) {
    state.selectedTurn = null;
    view.showTurn(null);
  } else {
    state.selectedTurn = t;
    view.showTurn({ built: state.built, test: { ...t, vehicleId: state.project.vehicleId } });
  }
  renderJunctions();
  if (state.selectedTurn) {
    const turn = state.built.turnTests.find(
      (x) => x.junctionId === t.junctionId && x.from.connectionId === t.from.connectionId && x.to.connectionId === t.to.connectionId,
    );
    if (turn) {
      const mid = turn.path[Math.floor(turn.path.length / 2)];
      view.focusOn({ x: mid.x, y: mid.y }, 55);
    }
  }
}

function renderViolations() {
  const b = state.built;
  const all = [...b.violations, ...b.seams];
  ui.violationsList.innerHTML = '';
  if (!all.length) {
    ui.violationsList.innerHTML = '<div class="muted">没有违规。所有路线、路口与转弯通过验收。</div>';
    return;
  }
  for (const v of all) {
    const div = document.createElement('div');
    div.className = 'item clickable';
    const plan = v.plan || stationPlan(v);
    div.innerHTML = `
      <div class="head">
        <span class="tag ${v.severity === 'error' ? 'bad' : 'warn'}">${v.severity === 'error' ? '错误' : '警告'}</span>
        <span class="name" style="font-size:11.5px;font-weight:500">${v.message}</span>
      </div>
      <div class="metrics">${plan ? `位置 (${plan.x.toFixed(0)}, ${plan.y.toFixed(0)})` : ''} ${v.connectionId ? '· ' + v.connectionId : ''} ${v.junctionId ? '· ' + v.junctionId : ''}</div>`;
    if (plan) {
      div.addEventListener('click', () => view.focusOn(plan, 42));
    }
    ui.violationsList.appendChild(div);
  }
}

function stationPlan(v) {
  if (!v.connectionId || v.station == null) return null;
  const route = state.built.routes.find((r) => r.conn.id === v.connectionId);
  if (!route) return null;
  let best = null;
  let bd = Infinity;
  for (const s of route.samples) {
    const d = Math.abs(s.s - v.station);
    if (d < bd) { bd = d; best = s; }
  }
  return best ? { x: best.x, y: best.y } : null;
}

function armName(built, id) {
  const c = built.connections.find((x) => x.id === id);
  return c?.name || id;
}

function renderRecord() {
  const rec = state.built.record;
  const adopted = state.iface ? state.iface.adoptedSurfaces({ includeStale: true }) : [];
  const summary = {
    graphId: rec.graphId,
    revision: rec.revision,
    meshRevision: state.iface ? state.iface.meshRevision : null,
    seed: rec.seed,
    vehicle: rec.vehicleId,
    controls: rec.controls,
    fixedControls: rec.fixedControls,
    junctions: rec.junctions.length,
    turns: rec.turns.map((t) => ({ j: t.junctionId, from: t.from, to: t.to, pass: t.pass })),
    locked: rec.connections.filter((c) => c.locked).map((c) => c.id),
    adoptedSurfaces: adopted.map((s) =>
      s.part === 'junction' ? `路口:${s.junctionId}@r${s.revision}` : `路面:${s.connectionId}@r${s.revision}`,
    ),
  };
  ui.recordBox.textContent = JSON.stringify(summary, null, 1);
}

function renderLockButton() {
  const conn = state.project.connections.find((c) => c.id === state.selectedConnection);
  const locked = !!conn?.locked;
  ui.lockBtn.textContent = locked ? '已锁定 · 点击解锁' : '锁定本段';
  ui.lockBtn.setAttribute('aria-pressed', locked ? 'true' : 'false');
  ui.solveBtn.disabled = !locked;
}

function populateConnectionSelect() {
  ui.connSelect.innerHTML = state.project.connections
    .map((c) => `<option value="${c.id}">${c.name || c.id}</option>`)
    .join('');
  if (!state.project.connections.some((c) => c.id === state.selectedConnection)) {
    state.selectedConnection = state.project.connections[0]?.id || null;
  }
  ui.connSelect.value = state.selectedConnection;
  ui.sceneDesc.textContent = state.project.name || '';
  for (const b of ui.sceneSwitch.children) {
    b.classList.toggle('active', b.dataset.scene === state.sceneId);
  }
}

function populateDriveRoutes() {
  ui.driveRoute.innerHTML = state.project.connections
    .map((c) => `<option value="${c.id}">${c.name || c.id}</option>`)
    .join('');
  if (!state.drive.routeId) state.drive.routeId = state.project.connections[0]?.id;
  ui.driveRoute.value = state.drive.routeId;
}

// ---------------------------------------------------------------------------
// Driving
// ---------------------------------------------------------------------------
function enterDrive() {
  const route = state.built.routes.find((r) => r.conn.id === state.drive.routeId) || state.built.routes[0];
  if (!route) return;
  state.drive.routeId = route.conn.id;
  state.drive.active = true;
  ui.driveRoute.value = route.conn.id;
  ui.driveBtn.hidden = true;
  ui.exitDriveBtn.hidden = false;
  ui.driveKeys.hidden = false;
  ui.pickHint.hidden = true;
  view.controls.enabled = false;
  view.addVehicle(state.built, state.project.vehicleId);
  const placed = view.placeVehicle(route, state.built);
  state.drive.startRoute = route;
  // Register a safe parking point well back from any junction, on this route's
  // own pavement, so an edit can offer an explicit safe-return instead of
  // tearing out pavement under the wheels.
  const safeSample = route.samples[Math.max(1, Math.floor(route.samples.length * 0.12))];
  state.iface.registerSafePoint(route.conn.id, { x: safeSample.x, y: safeSample.y });
  state.iface.startDriveSession(route.conn.id, { x: placed.station, y: 0 });
  // Chase camera.
  const v = view.vehicle;
  chaseCamera(v);
  toast('已进入驾驶：WASD 或方向键控制车辆，Esc 退出', 'ok');
}

function exitDrive() {
  if (state.iface && state.drive.active) state.iface.endDriveSession();
  state.drive.active = false;
  view.controls.enabled = true;
  view.removeVehicle();
  ui.driveBtn.hidden = false;
  ui.exitDriveBtn.hidden = true;
  ui.driveKeys.hidden = true;
  ui.telemetry.innerHTML = '';
}

function placeDriveVehicle() {
  const route = state.built.routes.find((r) => r.conn.id === state.drive.routeId);
  if (!route || !view.vehicle) return;
  view.addVehicle(state.built, state.project.vehicleId);
  view.placeVehicle(route, state.built);
}

function chaseCamera(v) {
  if (!v) return;
  const back = 16;
  const up = 7;
  view.camera.position.set(
    v.plan.x - Math.cos(v.heading) * back,
    (view.samplePavement(state.built, v.plan.x, v.plan.y) || 0) + up,
    v.plan.y - Math.sin(v.heading) * back,
  );
  view.controls.target.set(v.plan.x, (view.samplePavement(state.built, v.plan.x, v.plan.y) || 0) + 1.5, v.plan.y);
}

const keys = new Set();
function bindKeys() {
  window.addEventListener('keydown', (e) => {
    if (!state.drive.active) return;
    const k = e.key.toLowerCase();
    if (['w', 's', 'a', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(k)) {
      e.preventDefault();
      keys.add(k);
    }
    if (k === 'escape') exitDrive();
    if (k === 'r') {
      exitDrive();
      enterDrive();
    }
  });
  window.addEventListener('keyup', (e) => {
    keys.delete(e.key.toLowerCase());
  });
  window.addEventListener('blur', () => keys.clear());
}

function readInput() {
  const throttle = keys.has('w') || keys.has('arrowup') ? 1 : 0;
  const brake = keys.has('s') || keys.has('arrowdown') ? 1 : 0;
  let steer = 0;
  if (keys.has('a') || keys.has('arrowleft')) steer += 1;
  if (keys.has('d') || keys.has('arrowright')) steer -= 1;
  return { throttle, brake, steer };
}

let lastT = 0;
function driveTick(t) {
  const dt = Math.min(0.05, (t - lastT) / 1000 || 0.016);
  lastT = t;
  if (!view.vehicle) return;
  const input = readInput();
  view.stepVehicle(dt, input, state.built);
  const v = view.vehicle;
  // Record the actual pavement under the wheels (resolved by the geometry
  // interface) and feed contact into the edit guard. Stale-revision frames are
  // kept but flagged; only current-revision driving stays active.
  const frame = state.iface.recordDriveFrame({
    x: v.plan.x,
    y: v.plan.y,
    heading: v.heading,
    speed: v.speed,
    time: t,
  });
  if (frame.surface) {
    const touched = [];
    if (frame.surface.connectionId) touched.push(frame.surface.connectionId);
    state.iface.guard.markContact(touched, v.plan);
  }
  // Camera follows smoothly.
  const cam = view.camera.position;
  const targetX = v.plan.x - Math.cos(v.heading) * 15;
  const targetY = view.samplePavement(state.built, v.plan.x, v.plan.y) + 6.5;
  const targetZ = v.plan.y - Math.sin(v.heading) * 15;
  cam.x += (targetX - cam.x) * Math.min(1, dt * 5);
  cam.y += (targetY - cam.y) * Math.min(1, dt * 5);
  cam.z += (targetZ - cam.z) * Math.min(1, dt * 5);
  view.controls.target.set(v.plan.x, view.samplePavement(state.built, v.plan.x, v.plan.y) + 1.4, v.plan.y);
  renderTelemetry(v);
}

function renderTelemetry(v) {
  const probe = state.iface.surfaceAt(v.plan.x, v.plan.y);
  const h = probe ? probe.height : view.samplePavement(state.built, v.plan.x, v.plan.y);
  const sourceLabel = probe
    ? probe.source.part === 'junction'
      ? `路口 ${probe.source.junctionId}`
      : `路面 ${probe.source.connectionId}`
    : '路外';
  const stale = probe && probe.revision !== state.iface.revision;
  ui.telemetry.innerHTML = `
    <span>车速 <b>${(v.speed * 3.6).toFixed(0)} km/h</b></span>
    <span>路面高 <b>${h != null ? h.toFixed(2) : '—'} m</b></span>
    <span>实际路面 <b>${sourceLabel}</b></span>
    <span>坐标 <b>${v.plan.x.toFixed(0)}, ${v.plan.y.toFixed(0)}</b></span>
    <span>转向 <b>${(v.steering * 180 / Math.PI).toFixed(0)}°</b></span>
    ${stale ? '<span class="warn">该轨迹已被新道路修订取代</span>' : ''}`;
}

// ---------------------------------------------------------------------------
// Control point dragging
// ---------------------------------------------------------------------------
function bindPointer() {
  canvas.addEventListener('pointerdown', (e) => {
    if (state.drive.active) return;
    const handle = view.pickControl(e);
    if (!handle) {
      // Clicking empty space deselects the turn overlay.
      if (state.selectedTurn) {
        state.selectedTurn = null;
        view.showTurn(null);
        renderJunctions();
      }
      return;
    }
    e.preventDefault();
    const connectionId = handle.userData.connectionId;
    // Live-edit safety: while driving, a connection with tyre contact is
    // retained. The editor blocks the drag and offers an explicit safe return
    // instead of removing pavement under the vehicle.
    if (state.drive.active) {
      const decision = state.iface.guard.canRemove(connectionId);
      if (decision.status !== 'free') {
        if (decision.status === 'parked') {
          const point = state.iface.requestSafeReturn(connectionId);
          if (point) {
            view.vehicle.plan = { x: point.x, y: point.y };
            view.syncVehicle(state.built);
            state.iface.confirmSafeReturn(connectionId);
            toast('车辆已送回安全停车点，现在可以编辑该连接', 'ok');
          }
        } else {
          toast('该连接仍有轮胎接触，请待车辆通过或先退出驾驶', 'error');
        }
        return;
      }
    }
    view.controls.enabled = false;
    const p = handle.position;
    state.dragging = {
      connectionId,
      index: handle.userData.index,
      handle,
      planeY: p.y,
    };
    ui.pickHint.hidden = false;
    canvas.style.cursor = 'grabbing';
  });

  canvas.addEventListener('pointermove', (e) => {
    if (state.drive.active) return;
    if (!state.dragging) {
      const hover = view.pickControl(e);
      canvas.style.cursor = hover ? 'grab' : 'default';
      return;
    }
    const pt = view.planPoint(e, state.dragging.planeY);
    if (!pt) return;
    const conn = state.project.connections.find((c) => c.id === state.dragging.connectionId);
    if (!conn) return;
    const interiorIndex = state.dragging.index - 1;
    if (!conn.controlPoints) {
      const route = state.built.routes.find((r) => r.conn.id === conn.id);
      conn.controlPoints = interiorControl(route);
    }
    if (interiorIndex >= 0 && interiorIndex < conn.controlPoints.length) {
      conn.controlPoints[interiorIndex] = { x: pt.x, y: pt.y };
      conn.locked = false;
      dragPreview(pt);
    }
  });

  const end = () => {
    if (!state.dragging) return;
    const conn = state.project.connections.find((c) => c.id === state.dragging.connectionId);
    state.dragging = null;
    view.controls.enabled = true;
    canvas.style.cursor = 'default';
    ui.pickHint.hidden = true;
    if (conn) {
      // Keep the other routes free to re-solve.
      conn.locked = false;
      state.selectedConnection = conn.id;
      rebuild();
    }
  };
  canvas.addEventListener('pointerup', end);
  canvas.addEventListener('pointercancel', end);
}

// Move the handle during a drag without rebuilding every frame.
function dragPreview(pt) {
  const h = view.terrainSample ? view.terrainSample(pt.x, pt.y) : null;
  if (state.dragging?.handle) {
    const y = state.built.terrain.sampleHeight ? state.built.terrain.sampleHeight(pt.x, pt.y) : state.built.terrain.heightAt(pt.x, pt.y);
    state.dragging.handle.position.set(pt.x, (y || 0) + 2.2, pt.y);
  }
  // Rebuild thin on pointerup; previewing the full route is expensive.
}

// ---------------------------------------------------------------------------
// UI helpers
// ---------------------------------------------------------------------------
let toastTimer = null;
function toast(message, kind = '') {
  ui.toast.textContent = message;
  ui.toast.className = `toast ${kind}`;
  ui.toast.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { ui.toast.hidden = true; }, 3200);
}

function showLoading(on) {
  ui.loading.hidden = !on;
}

// Minimal debug handle for the Playwright e2e runner. Editing a control point
// through the same project state the pointer handler uses exercises the real
// rebuild path without depending on screen-space picking.
window.__roadforge = {
  get state() { return state; },
  get built() { return state.built; },
  get view() { return view; },
  get iface() { return state.iface; },
  status() {
    const b = state.built;
    return {
      sceneId: state.sceneId,
      revision: b.revision,
      meshRevision: state.iface.meshRevision,
      meshCurrent: state.iface.ledger.meshIsCurrent(),
      errorCount: b.errorCount,
      turns: b.turnTests.length,
      turnFails: b.turnTests.filter((t) => !t.pass).length,
      violations: b.violations.length,
      seams: b.seams.length,
      occupied: state.iface.guard.occupiedConnections(),
      adoptedSurfaces: state.iface.adoptedSurfaces({ includeStale: true }),
      controls: state.project.connections.map((c) => ({ id: c.id, locked: !!c.locked, controlPoints: c.controlPoints })),
    };
  },
  // Plan an edit against the guard without mutating geometry.
  planEdit(connectionIds) {
    return state.iface.planEdit(connectionIds);
  },
  // Move the first interior control point of the selected connection.
  debugDragControl(index = 0, dx = 0, dy = 0) {
    const id = state.selectedConnection;
    const route = state.built.routes.find((r) => r.conn.id === id);
    const conn = state.project.connections.find((c) => c.id === id);
    if (!route || !conn) return 'no-route';
    const interior = interiorControl(route);
    if (index < 0 || index >= interior.length) return 'no-interior';
    if (!conn.controlPoints) conn.controlPoints = interior.map((p) => ({ ...p }));
    const before = { ...conn.controlPoints[index] };
    conn.controlPoints[index] = { x: before.x + dx, y: before.y + dy };
    conn.locked = false;
    rebuild();
    return { id, index, before, after: conn.controlPoints[index], revision: state.iface.revision };
  },
};

boot();
