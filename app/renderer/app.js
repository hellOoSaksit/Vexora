// UI logic. All simulation state lives in vexora_sim; this page only sends commands and shows
// what comes back. Settings are remembered per viewer (localStorage) as a convenience.
import { createForm } from './form.js';
import { Viewport } from './viewport.js';

const $ = (id) => document.getElementById(id);
const fmt = new Intl.NumberFormat('th-TH', { maximumFractionDigits: 2 });
const STORAGE_KEY = 'vexora.settings.v1';

const WARNING_TEXT = {
  ExcessiveVelocity: {
    title: 'วัตถุเคลื่อนที่เร็วผิดปกติ',
    hint: 'การคำนวณอาจไม่เสถียร ลองเปลี่ยนคุณภาพเป็น "ดูละเอียด" หรือลดความเร็ว/น้ำหนักของวัตถุ',
  },
  NonFiniteState: {
    title: 'การคำนวณผิดพลาดกับวัตถุบางชิ้น',
    hint: 'ระบบหยุดวัตถุนั้นไว้ที่ตำแหน่งล่าสุดที่ถูกต้องแล้ว ลองกด "เริ่มใหม่"',
    bad: true,
  },
  EngineCapacity: {
    title: 'จำนวนการชนเกินขีดจำกัด',
    hint: 'การชนบางจุดถูกข้ามไป ลองลดจำนวนวัตถุ',
    bad: true,
  },
};

const DEVICE_TYPE_TEXT = {
  CPU: 'ตัวประมวลผลหลัก (CPU)',
  'Discrete GPU': 'การ์ดจอแยก',
  'Integrated GPU': 'การ์ดจอในตัว',
  'Virtual GPU': 'การ์ดจอเสมือน',
  'Software GPU (runs on CPU)': 'GPU จำลองด้วยซอฟต์แวร์ (ช้า ไม่ใช้โดยอัตโนมัติ)',
};

const state = {
  hello: null,
  scene: null,
  values: {},      // sceneId -> params
  global: {},
  playing: false,
  loaded: null,
  resumeAfterLoad: false,
  recentWarnings: new Map(),
  brokenTotal: 0,
  lastStats: null,
  forms: [],
};

// The 3D view is optional: if WebGL is unavailable the settings and simulation still work.
let viewport;
try {
  viewport = new Viewport($('viewport'));
} catch (err) {
  console.error(err);
  viewport = { noWebGL: true, load() {}, addBodies() {}, applyFrame() {} };
  queueMicrotask(() => {
    $('overlay').textContent = 'แสดงภาพ 3 มิติไม่ได้ (เครื่องนี้ไม่รองรับ WebGL) — ลองอัปเดตไดรเวอร์การ์ดจอ การคำนวณยังทำงานตามปกติ';
  });
}

// ---------------------------------------------------------------------------------- storage
function restoreSettings() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) || '{}');
    state.values = saved.values ?? {};
    state.global = saved.global ?? {};
    state.scene = saved.scene ?? null;
    $('show-advanced').checked = Boolean(saved.advanced);
  } catch {
    /* Storage unavailable: start from defaults. */
  }
}

function saveSettings() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      values: state.values, global: state.global, scene: state.scene, advanced: $('show-advanced').checked,
    }));
  } catch {
    /* Not critical. */
  }
}

// -------------------------------------------------------------------------------- commands
const send = (cmd) => window.vexora.send(cmd);

function loadScene() {
  if (!state.scene) return;
  state.resumeAfterLoad = state.playing;
  state.recentWarnings.clear();
  state.brokenTotal = 0;
  send({ cmd: 'load', scene: state.scene, params: state.values[state.scene] ?? {}, global: state.global });
}

function sceneDef(id = state.scene) {
  return state.hello?.schema.scenes.find((s) => s.id === id);
}

function selectScene(id) {
  state.scene = id;
  saveSettings();
  for (const b of $('scene-tabs').children) b.setAttribute('aria-selected', String(b.dataset.id === id));
  renderSceneInfo();
  renderForms();
  state.playing = false;
  loadScene();
}

// ------------------------------------------------------------------------------- rendering
function renderTabs() {
  const tabs = $('scene-tabs');
  tabs.replaceChildren();
  for (const s of state.hello.schema.scenes) {
    const b = document.createElement('button');
    b.textContent = s.label;
    b.dataset.id = s.id;
    b.setAttribute('role', 'tab');
    b.addEventListener('click', () => selectScene(s.id));
    tabs.append(b);
  }
}

function renderSceneInfo() {
  const s = sceneDef();
  $('scene-title').textContent = s.label;
  $('scene-desc').textContent = s.description;
  const actions = $('scene-actions');
  actions.replaceChildren();
  for (const a of s.actions) {
    const b = document.createElement('button');
    b.textContent = a.label;
    b.addEventListener('click', () => {
      send({ cmd: 'action', id: a.id });
      if (!state.playing) send({ cmd: 'play' });
    });
    actions.append(b);
  }
}

function renderForms() {
  const s = sceneDef();
  state.values[s.id] = { ...(state.values[s.id] ?? {}) };
  const sceneForm = createForm($('scene-form'), s.params, state.values[s.id], (key, value) => {
    state.values[s.id][key] = value;
    saveSettings();
    loadScene();
  });
  const globalForm = createForm($('global-form'), state.hello.schema.global, state.global, (key, value) => {
    state.global[key] = value;
    saveSettings();
    loadScene();
  });
  state.forms = [sceneForm, globalForm];
  applyAdvanced();
}

function applyAdvanced() {
  for (const f of state.forms) f.setAdvanced($('show-advanced').checked);
}

function el(tag, cls, text) {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
}

function renderDevices() {
  const box = $('devices');
  box.replaceChildren();
  for (const d of state.hello.devices) {
    const card = el('div', 'device');
    card.append(el('div', 'name', d.backend === 'CPU' ? `CPU (${d.threads} threads)` : d.name));
    const meta = [DEVICE_TYPE_TEXT[d.type] ?? d.type];
    if (d.memoryMB) meta.push(`${fmt.format(d.memoryMB / 1024)} GB`);
    if (d.api) meta.push(d.api);
    card.append(el('div', 'meta', meta.join(' · ')));
    box.append(card);
  }
  if (!state.hello.gpuAvailable) {
    const n = el('div', 'notice info', 'ไม่พบการ์ดจอที่ใช้คำนวณได้ — ทุกอย่างจะคำนวณด้วย CPU');
    const details = el('details');
    details.append(el('summary', '', 'รายละเอียด'), el('div', '', state.hello.gpuReason));
    n.append(details);
    box.append(n);
  }

  const systems = el('div', 'device');
  systems.append(el('div', 'name', 'ระบบไหนใช้อะไรคำนวณ'));
  const rigid = el('div', 'system-row');
  rigid.append(el('span', '', 'วัตถุแข็ง / ข้อต่อ'), el('span', 'pill cpu', 'CPU เท่านั้น'));
  systems.append(rigid);
  const note = el('div', 'meta', 'Physics engine (Jolt) ไม่รองรับการคำนวณบน GPU');
  systems.append(note);

  const particles = el('div', 'system-row');
  particles.append(el('span', '', 'อนุภาค'));
  const stats = state.lastStats;
  if (state.loaded?.particles && stats?.backend) {
    particles.append(el('span', stats.backend === 'GPU' ? 'pill gpu' : 'pill cpu',
      stats.backend === 'GPU' ? `GPU · ${stats.device}` : 'CPU'));
    systems.append(particles);
    const details = el('details');
    details.append(el('summary', '', 'ทำไมถึงเลือกแบบนี้'), el('div', '', stats.reason));
    systems.append(details);
  } else {
    particles.append(el('span', 'pill off', 'CPU / GPU'));
    systems.append(particles);
  }
  box.append(systems);
}

function renderStats(stats, t) {
  const rows = [
    ['เวลาในฉาก', `${fmt.format(t)} s`],
    ['วัตถุทั้งหมด', fmt.format(stats.bodies)],
    ['วัตถุที่กำลังเคลื่อนที่', fmt.format(stats.activeBodies)],
    ['เวลาคำนวณต่อขั้น', `${fmt.format(stats.physicsMs + (stats.particleMs ?? 0))} ms`],
  ];
  if (stats.particlesAlive !== undefined) rows.push(['อนุภาคที่มีอยู่', fmt.format(stats.particlesAlive)]);
  if (state.brokenTotal) rows.push(['ข้อต่อที่ขาด', fmt.format(state.brokenTotal)]);
  if (state.playing && stats.speed !== undefined) rows.push(['ความเร็วเทียบเวลาจริง', `${Math.round(stats.speed * 100)}%`]);
  const dl = $('stats');
  dl.replaceChildren();
  for (const [k, v] of rows) dl.append(el('dt', '', k), el('dd', '', v));

  const slow = state.playing && stats.speed !== undefined && stats.speed < 0.9;
  const warn = $('speed-warning');
  warn.hidden = !slow;
  if (slow)
    warn.textContent = `เครื่องคำนวณไม่ทันเวลาจริง (${Math.round(stats.speed * 100)}%) ภาพจึงเล่นช้ากว่าความจริง ` +
      'ลองลดจำนวนวัตถุ หรือเลือกคุณภาพ "ดูตัวอย่างเร็ว"';
}

function renderWarnings() {
  const box = $('warnings');
  const now = performance.now();
  for (const [type, w] of state.recentWarnings) if (now - w.seen > 5000) state.recentWarnings.delete(type);
  box.replaceChildren();
  if (!state.recentWarnings.size) {
    box.append(el('p', 'muted', 'ไม่มีคำเตือน'));
    return;
  }
  for (const [type, w] of state.recentWarnings) {
    const text = WARNING_TEXT[type] ?? { title: type, hint: w.message };
    const card = el('div', text.bad ? 'warning bad' : 'warning');
    card.append(el('strong', '', `${text.title} (${fmt.format(w.count)} ครั้ง)`), el('span', '', text.hint));
    box.append(card);
  }
}

let toastTimer = 0;
function toast(message, kind = 'error', ms = 6000) {
  const t = $('toast');
  t.textContent = message;
  t.className = kind === 'info' ? 'toast info' : 'toast';
  t.hidden = false;
  clearTimeout(toastTimer);
  if (ms) toastTimer = setTimeout(() => (t.hidden = true), ms);
}

function setPlaying(playing) {
  state.playing = playing;
  $('btn-play').textContent = playing ? '⏸ หยุด' : '▶ เล่น';
  $('btn-step').disabled = playing || !state.loaded;
}

// -------------------------------------------------------------------------------- messages
let lastStatsPaint = 0;
window.vexora.onMessage((msg) => {
  switch (msg.type) {
    case 'hello': {
      const reconnect = state.hello !== null;
      state.hello = msg;
      renderTabs();
      if (!sceneDef(state.scene)) state.scene = msg.schema.scenes[0].id;
      renderDevices();
      if (reconnect) {
        toast('โปรแกรมคำนวณกลับมาทำงานแล้ว', 'info', 3000);
        loadScene();
      } else {
        selectScene(state.scene);
      }
      break;
    }
    case 'loaded':
      state.loaded = msg;
      viewport.load(msg);
      for (const id of ['btn-play', 'btn-reset']) $(id).disabled = false;
      setPlaying(false);
      if (!viewport.noWebGL) $('overlay').textContent = `${sceneDef(msg.scene)?.label ?? ''} · ${msg.bodies.length} วัตถุ`;
      if (state.resumeAfterLoad) send({ cmd: 'play' });
      renderDevices();
      break;
    case 'frame':
      viewport.applyFrame(msg);
      if (msg.stats) {
        state.brokenTotal += msg.stats.broken ?? 0;
        const backendChanged = state.lastStats?.backend !== msg.stats.backend;
        state.lastStats = msg.stats;
        for (const w of msg.warnings ?? []) {
          const prev = state.recentWarnings.get(w.type);
          state.recentWarnings.set(w.type, { count: (prev?.count ?? 0) + w.count, message: w.message, seen: performance.now() });
        }
        const now = performance.now();
        if (now - lastStatsPaint > 250) {
          lastStatsPaint = now;
          renderStats(msg.stats, msg.t);
          renderWarnings();
        }
        if (backendChanged) renderDevices();
      }
      break;
    case 'bodiesAdded':
      viewport.addBodies(msg.bodies);
      break;
    case 'state':
      setPlaying(msg.playing);
      break;
    case 'error':
      toast(msg.message);
      break;
    case 'crashed':
      setPlaying(false);
      toast(msg.willRestart ? 'โปรแกรมคำนวณหยุดทำงานกะทันหัน กำลังเริ่มใหม่…'
                            : 'โปรแกรมคำนวณหยุดทำงานซ้ำหลายครั้ง กรุณาปิดแล้วเปิดโปรแกรมใหม่', 'error', 0);
      break;
    case 'fatal':
      toast(msg.message, 'error', 0);
      $('scene-title').textContent = 'เปิดโปรแกรมคำนวณไม่ได้';
      break;
  }
});

// ---------------------------------------------------------------------------------- inputs
$('btn-play').addEventListener('click', () => send({ cmd: state.playing ? 'pause' : 'play' }));
$('btn-step').addEventListener('click', () => send({ cmd: 'step', count: 1 }));
$('btn-reset').addEventListener('click', () => {
  state.playing = false;
  loadScene();
});
$('btn-defaults').addEventListener('click', () => {
  state.values[state.scene] = {};
  saveSettings();
  renderForms();
  loadScene();
});
$('show-advanced').addEventListener('change', () => {
  applyAdvanced();
  saveSettings();
});
document.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
  if (e.code === 'Space' && state.loaded) {
    e.preventDefault();
    send({ cmd: state.playing ? 'pause' : 'play' });
  } else if (e.key === 'r' || e.key === 'R') {
    state.playing = false;
    loadScene();
  }
});

restoreSettings();
