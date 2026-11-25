import { chromium } from 'playwright';
import process from 'node:process';

const BASE = process.env.BASE_URL || 'http://localhost:5173/';

// This runner may be on a machine without system copies of Chromium's shared
// libraries. Point ROADFORGE_SYSLIBS at a directory containing them (colon
// separated); otherwise a couple of common extracted-package locations are
// tried. See README "Headless browser setup".
const EXTRA_LIBS = process.env.ROADFORGE_SYSLIBS || [
  '/tmp/syslibs/usr/lib/aarch64-linux-gnu',
  '/tmp/syslibs/lib/aarch64-linux-gnu',
].join(':');
process.env.LD_LIBRARY_PATH = [EXTRA_LIBS, process.env.LD_LIBRARY_PATH].filter(Boolean).join(':');

export async function launch() {
  const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const page = await browser.newPage({ viewport: { width: 1440, height: 900 } });
  const errors = [];
  page.on('console', (msg) => {
    if (msg.type() === 'error') errors.push(msg.text());
  });
  page.on('pageerror', (err) => errors.push(String(err)));
  await page.goto(BASE, { waitUntil: 'networkidle' });
  return { browser, page, errors };
}

export async function waitForBuild(page) {
  await page.waitForFunction(() => {
    const loading = document.getElementById('loading');
    const status = document.getElementById('status-build');
    return loading && loading.hidden && status && status.textContent.includes('构建');
  }, { timeout: 30000 });
}

const run = async () => {
  const { browser, page, errors } = await launch();
  try {
    await waitForBuild(page);
    const status = await page.textContent('#status-build');
    const turns = await page.textContent('#status-turns');
    const viol = await page.textContent('#status-viol');
    console.log('status:', status, '|', turns, '|', viol);
    const turnFail = await page.evaluate(() => {
      const items = [...document.querySelectorAll('#junctions-list .tag')];
      return items.filter((t) => t.textContent.includes('不可行')).length;
    });
    const routeRows = await page.$$eval('#routes-list .item', (els) => els.length);
    console.log('route rows:', routeRows, 'turn fail tags:', turnFail);

    // Screenshot each scene.
    for (const scene of ['arterial', 'slope', 'multibranch']) {
      await page.click(`.scene-switch button[data-scene="${scene}"]`);
      await waitForBuild(page);
      await page.screenshot({ path: `/tmp/scene-${scene}.png` });
      console.log(`screenshot ${scene}`);
    }
    // Drive mode smoke test.
    await page.click('.scene-switch button[data-scene="arterial"]');
    await waitForBuild(page);
    await page.click('#btn-drive');
    await page.waitForTimeout(400);
    await page.keyboard.down('w');
    await page.waitForTimeout(1400);
    await page.keyboard.up('w');
    await page.waitForTimeout(400);
    const tele = await page.textContent('#telemetry');
    console.log('telemetry:', (tele || '').replace(/\s+/g, ' ').trim());
    await page.screenshot({ path: '/tmp/drive.png' });

    // While the car is on pavement the drive log names the real surface source
    // and the edit guard reports the occupied connection.
    const driveInfo = await page.evaluate(() => {
      const s = window.__roadforge.status();
      return { occupied: s.occupied, adopted: s.adoptedSurfaces };
    });
    console.log('drive occupied:', JSON.stringify(driveInfo.occupied), 'adopted:', JSON.stringify(driveInfo.adopted));
    if (!driveInfo.adopted.length) errors.push('drive log recorded no pavement source');
    if (!driveInfo.occupied.length) errors.push('edit guard saw no occupied connection while driving');

    // A drag on the occupied connection must be refused while driving.
    const blocked = await page.evaluate((occupied) => {
      const app = window.__roadforge;
      const connId = occupied[0] || app.built.routes[0].conn.id;
      return app.planEdit([connId]);
    }, driveInfo.occupied);
    console.log('occupied edit plan:', JSON.stringify(blocked));
    if (blocked.removable.length) errors.push('occupied connection was offered for removal');

    await page.keyboard.press('Escape');
    await page.waitForTimeout(300);

    // Control point drag: move an interior control and confirm a rebuild.
    const beforeLen = await page.evaluate(() => {
      const r = window.__roadforge.built.routes.find((x) => x.conn.id === 'branch-north');
      return r && r.samples.length ? r.samples[r.samples.length - 1].s : null;
    });
    const revBefore = await page.evaluate(() => window.__roadforge.status().revision);
    const dragged = await page.evaluate(() => {
      const app = window.__roadforge;
      if (!app) return 'no-api';
      app.state.selectedConnection = 'branch-north';
      return app.debugDragControl(0, 9, 6);
    });
    await page.waitForTimeout(1200);
    const afterLen = await page.evaluate(() => {
      const r = window.__roadforge.built.routes.find((x) => x.conn.id === 'branch-north');
      return r && r.samples.length ? r.samples[r.samples.length - 1].s : null;
    });
    console.log('drag result:', JSON.stringify(dragged), 'branch length', beforeLen?.toFixed(1), '->', afterLen?.toFixed(1));
    const afterViol = await page.textContent('#status-viol');
    console.log('after drag status:', afterViol);

    // Revision hand-off: every drag bumps the road revision and the collision
    // mesh is published for the same revision (never lagging behind the road).
    const statusAfterDrag = await page.evaluate(() => window.__roadforge.status());
    console.log('revision', revBefore, '->', statusAfterDrag.revision, 'meshCurrent', statusAfterDrag.meshCurrent);
    if (!(statusAfterDrag.revision > revBefore)) {
      errors.push(`revision did not advance: ${revBefore} -> ${statusAfterDrag.revision}`);
    }
    if (!statusAfterDrag.meshCurrent) errors.push('collision mesh lags the latest road revision');

    // Edit guard on an open (vehicle-free) scene: every connection is removable.
    const plan = await page.evaluate(() =>
      window.__roadforge.planEdit(window.__roadforge.built.routes.map((r) => r.conn.id)),
    );
    console.log('open edit plan: removable', plan.removable.length, 'retained', plan.retained.length);
    if (plan.retained.length !== 0) errors.push('vehicle-free junction should be fully editable');
    await page.screenshot({ path: '/tmp/after-drag.png' });

    if (errors.length) {
      console.log('CONSOLE ERRORS:', errors.slice(0, 10));
      process.exitCode = 1;
    } else {
      console.log('no console errors');
    }
  } finally {
    await browser.close();
  }
};

if (process.argv[1] && process.argv[1].endsWith('run.js')) {
  run().catch((e) => {
    console.error(e);
    process.exit(1);
  });
}
