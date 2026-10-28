// Drives vexora_sim through the same JSON protocol the app uses (no Electron needed).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const here = path.dirname(fileURLToPath(import.meta.url));
const exe = path.join(here, '..', 'bin', process.platform === 'win32' ? 'vexora_sim.exe' : 'vexora_sim');

function startSim() {
  assert.ok(existsSync(exe), `missing ${exe}; run "npm run copy-sim" after building`);
  const proc = spawn(exe, [], { stdio: ['pipe', 'pipe', 'inherit'] });
  const queue = [];
  const waiters = [];
  createInterface({ input: proc.stdout }).on('line', (line) => {
    const msg = JSON.parse(line);
    const i = waiters.findIndex((w) => w.match(msg));
    if (i >= 0) waiters.splice(i, 1)[0].resolve(msg);
    else queue.push(msg);
  });
  return {
    send: (cmd) => proc.stdin.write(JSON.stringify(cmd) + '\n'),
    next(match, timeoutMs = 15000) {
      const i = queue.findIndex(match);
      if (i >= 0) return Promise.resolve(queue.splice(i, 1)[0]);
      return new Promise((resolve, reject) => {
        const timer = setTimeout(() => reject(new Error('timeout waiting for message')), timeoutMs);
        waiters.push({ match, resolve: (m) => { clearTimeout(timer); resolve(m); } });
      });
    },
    drain() { queue.length = 0; },
    close() { proc.stdin.end('{"cmd":"quit"}\n'); return new Promise((r) => proc.on('exit', r)); },
  };
}

const type = (t) => (m) => m.type === t;
const decode = (b64) => { const b = Buffer.from(b64, 'base64'); return new Float32Array(b.buffer, b.byteOffset, b.length / 4); };

test('hello lists the schema, devices and which systems can use the GPU', async () => {
  const sim = startSim();
  const hello = await sim.next(type('hello'));
  assert.deepEqual(hello.schema.scenes.map((s) => s.id), ['drop', 'tower', 'chain', 'fountain']);
  assert.equal(hello.devices[0].backend, 'CPU');
  for (const s of hello.schema.scenes) for (const p of s.params) {
    assert.ok(p.label && p.type, `param ${p.key} needs label and type`);
    if (p.type === 'number' || p.type === 'int') assert.ok(p.min <= p.default && p.default <= p.max, `${p.key} default in range`);
  }
  assert.equal(hello.systems.find((s) => s.name === 'rigid').gpu, false);
  await sim.close();
});

test('every scene loads with defaults and simulates', async () => {
  const sim = startSim();
  await sim.next(type('hello'));
  for (const scene of ['drop', 'tower', 'chain', 'fountain']) {
    sim.send({ cmd: 'load', scene, params: scene === 'fountain' ? { backend: 'cpu' } : {} });
    const loaded = await sim.next(type('loaded'));
    assert.equal(loaded.scene, scene);
    await sim.next(type('frame'));
    sim.send({ cmd: 'step', count: 60 });
    const frame = await sim.next(type('frame'));
    assert.ok(frame.t > 0.9, `${scene} advanced one second`);
    assert.ok(frame.stats.steps === 60);
    if (scene === 'fountain') assert.ok(frame.stats.particlesAlive > 0 && decode(frame.particles).length > 0);
  }
  await sim.close();
});

test('dropped bodies fall and come to rest on the floor', async () => {
  const sim = startSim();
  await sim.next(type('hello'));
  sim.send({ cmd: 'load', scene: 'drop', params: { count: 20, shape: 'box', size: 0.25 } });
  await sim.next(type('loaded'));
  await sim.next(type('frame'));
  sim.send({ cmd: 'step', count: 600 });
  const frame = await sim.next(type('frame'));
  const xf = decode(frame.bodies);
  for (let i = 0; i < 20; i++) assert.ok(xf[i * 7 + 1] > 0.2 && xf[i * 7 + 1] < 2.0, `body ${i} y=${xf[i * 7 + 1]}`);
  await sim.close();
});

test('invalid settings are rejected with a message naming the setting', async () => {
  const sim = startSim();
  await sim.next(type('hello'));
  sim.send({ cmd: 'load', scene: 'drop', params: { count: 999999 } });
  const err = await sim.next(type('error'));
  assert.match(err.message, /จำนวนวัตถุ/);
  sim.send({ cmd: 'load', scene: 'nope' });
  assert.ok((await sim.next(type('error'))).message.length > 0);
  sim.send({ cmd: 'bogus' });
  assert.ok((await sim.next(type('error'))).message.length > 0);
  await sim.close();
});

test('actions add bodies; play streams frames; pause stops them', async () => {
  const sim = startSim();
  await sim.next(type('hello'));
  sim.send({ cmd: 'load', scene: 'tower' });
  await sim.next(type('loaded'));
  sim.send({ cmd: 'action', id: 'shoot' });
  const added = await sim.next(type('bodiesAdded'));
  assert.equal(added.bodies.length, 1);
  sim.send({ cmd: 'play' });
  await sim.next((m) => m.type === 'state' && m.playing);
  sim.drain();
  await new Promise((r) => setTimeout(r, 500));
  sim.send({ cmd: 'pause' });
  await sim.next((m) => m.type === 'state' && !m.playing);
  await sim.close();
});
