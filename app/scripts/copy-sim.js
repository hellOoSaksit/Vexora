// Copies the built vexora_sim executable into app/bin so `npm start` and the installer use it.
const fs = require('fs');
const path = require('path');

const exe = process.platform === 'win32' ? 'vexora_sim.exe' : 'vexora_sim';
const root = path.resolve(__dirname, '..', '..');
const candidates = [
  path.join(root, 'build', 'engine', 'simserver', 'Release', exe), // Visual Studio (multi-config)
  path.join(root, 'build', 'engine', 'simserver', exe),            // Ninja / Makefiles
];
const found = candidates.find((p) => fs.existsSync(p));
if (!found) {
  console.error(`[vexora] ${exe} not found. Build it first:\n` +
    '  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release\n' +
    '  cmake --build build --config Release --target vexora_sim');
  process.exit(1);
}
const binDir = path.join(__dirname, '..', 'bin');
fs.mkdirSync(binDir, { recursive: true });
fs.copyFileSync(found, path.join(binDir, exe));
if (process.platform !== 'win32') fs.chmodSync(path.join(binDir, exe), 0o755);
console.log(`[vexora] copied ${path.relative(root, found)} -> app/bin/${exe}`);
