// Electron main process: owns the window and the background simulation process.
// The simulation (vexora_sim) runs as a separate native process so heavy computation never
// freezes the UI, and a crash there does not take the window down.
const { app, BrowserWindow, ipcMain, Menu } = require('electron');
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const readline = require('readline');

const SIM_EXE = process.platform === 'win32' ? 'vexora_sim.exe' : 'vexora_sim';
const MAX_RESTARTS = 3;

let win = null;
let sim = null;
let restarts = 0;
let quitting = false;

function simPath() {
  const candidates = app.isPackaged
    ? [path.join(process.resourcesPath, 'bin', SIM_EXE)]
    : [
        path.join(__dirname, 'bin', SIM_EXE),
        path.join(__dirname, '..', 'build', 'engine', 'simserver', 'Release', SIM_EXE),
        path.join(__dirname, '..', 'build', 'engine', 'simserver', SIM_EXE),
      ];
  return candidates.find((p) => fs.existsSync(p));
}

// Frames carry Float32 data as base64; decode here (Node Buffer) and hand typed arrays to the
// renderer, which structured-clone transfers efficiently.
function decodeFloats(b64) {
  const buf = Buffer.from(b64, 'base64');
  const out = new Float32Array(buf.length / 4);
  new Uint8Array(out.buffer).set(buf);
  return out;
}

function toRenderer(msg) {
  if (win && !win.isDestroyed()) win.webContents.send('sim:message', msg);
}

function startSim() {
  const exe = simPath();
  if (!exe) {
    toRenderer({ type: 'fatal', message: `ไม่พบโปรแกรมคำนวณ (${SIM_EXE}) — ต้อง build ก่อน ดู README` });
    return;
  }
  sim = spawn(exe, [], { stdio: ['pipe', 'pipe', 'pipe'], windowsHide: true });
  const lines = readline.createInterface({ input: sim.stdout });
  lines.on('line', (line) => {
    let msg;
    try {
      msg = JSON.parse(line);
    } catch {
      return;
    }
    if (msg.type === 'frame') {
      if (msg.bodies) msg.bodies = decodeFloats(msg.bodies);
      if (msg.sleep) msg.sleep = new Uint8Array(Buffer.from(msg.sleep, 'base64'));
      if (msg.particles) msg.particles = decodeFloats(msg.particles);
    }
    if (msg.type === 'hello') restarts = 0;
    toRenderer(msg);
  });
  sim.stderr.on('data', (d) => console.error('[vexora_sim]', d.toString()));
  sim.on('exit', (code, signal) => {
    sim = null;
    if (quitting) return;
    toRenderer({ type: 'crashed', code, signal, willRestart: restarts < MAX_RESTARTS });
    if (restarts++ < MAX_RESTARTS) setTimeout(startSim, 1000);
  });
  sim.on('error', (err) => toRenderer({ type: 'fatal', message: `เปิดโปรแกรมคำนวณไม่ได้: ${err.message}` }));
}

ipcMain.on('sim:send', (_event, cmd) => {
  if (sim && sim.stdin.writable) sim.stdin.write(JSON.stringify(cmd) + '\n');
});

function createWindow() {
  win = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 960,
    minHeight: 600,
    backgroundColor: '#15181d',
    title: 'Vexora',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      sandbox: true,
      nodeIntegration: false,
    },
  });
  win.loadFile(path.join(__dirname, 'renderer', 'index.html'));
  win.webContents.on('did-finish-load', () => {
    if (!sim) startSim();
    else sim.stdin.write('{"cmd":"hello"}\n');  // Window reloaded: resend capabilities.
  });
}

// Machines without a usable GPU driver would otherwise get no WebGL at all (blank viewport).
// SwiftShader renders on the CPU instead; it is only used when hardware WebGL is unavailable.
app.commandLine.appendSwitch('enable-unsafe-swiftshader');

Menu.setApplicationMenu(null);
app.whenReady().then(createWindow);
app.on('window-all-closed', () => app.quit());
app.on('before-quit', () => {
  quitting = true;
  if (sim) sim.stdin.end('{"cmd":"quit"}\n');
});
