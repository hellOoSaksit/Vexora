// 3D viewport (three.js). Draws what the simulation reports; it never simulates anything.
import * as THREE from '../node_modules/three/build/three.module.js';

const SLEEP_TINT = 0.55;

export class Viewport {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f1115);
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 500);

    this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x2a2420, 1.1));
    this.sun = new THREE.DirectionalLight(0xffffff, 2.2);
    this.sun.position.set(8, 16, 6);
    this.sun.castShadow = true;
    this.sun.shadow.mapSize.set(2048, 2048);
    const s = this.sun.shadow.camera;
    s.left = s.bottom = -20;
    s.right = s.top = 20;
    s.far = 60;
    this.scene.add(this.sun);
    const grid = new THREE.GridHelper(40, 40, 0x3c4450, 0x2a3038);
    grid.position.y = 0.002;
    this.scene.add(grid);

    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.dynamic = [];          // Index = render index sent by the simulation.
    this.geometries = new Map();
    this.materials = new Map();
    this.points = null;

    this.orbit = { target: new THREE.Vector3(0, 2, 0), radius: 16, yaw: 0.7, pitch: 0.42 };
    this.installControls();
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.resize();
    const loop = () => {
      this.render();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  installControls() {
    let dragging = null;
    let last = { x: 0, y: 0 };
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.canvas.addEventListener('pointerdown', (e) => {
      dragging = e.button === 2 || e.shiftKey ? 'pan' : 'orbit';
      last = { x: e.clientX, y: e.clientY };
      this.canvas.setPointerCapture(e.pointerId);
    });
    this.canvas.addEventListener('pointerup', () => (dragging = null));
    this.canvas.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      const dx = e.clientX - last.x, dy = e.clientY - last.y;
      last = { x: e.clientX, y: e.clientY };
      if (dragging === 'orbit') {
        this.orbit.yaw -= dx * 0.006;
        this.orbit.pitch = Math.min(1.5, Math.max(-0.2, this.orbit.pitch + dy * 0.006));
      } else {
        const scale = this.orbit.radius * 0.0015;
        const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 0);
        const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 1);
        this.orbit.target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
      }
    });
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.orbit.radius = Math.min(200, Math.max(1, this.orbit.radius * Math.exp(e.deltaY * 0.001)));
    }, { passive: false });
  }

  resize() {
    const { clientWidth: w, clientHeight: h } = this.canvas.parentElement;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  geometryFor(shape, size) {
    const key = `${shape}:${size.join(',')}`;
    if (!this.geometries.has(key)) {
      let g;
      if (shape === 'box') g = new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2);
      else if (shape === 'sphere') g = new THREE.SphereGeometry(size[0], 24, 16);
      else g = new THREE.CapsuleGeometry(size[0], size[1] * 2, 6, 16);
      this.geometries.set(key, g);
    }
    return this.geometries.get(key);
  }

  materialFor(color, sleeping) {
    const key = `${color}:${sleeping}`;
    if (!this.materials.has(key)) {
      const c = new THREE.Color(color);
      if (sleeping) c.multiplyScalar(SLEEP_TINT);
      this.materials.set(key, new THREE.MeshStandardMaterial({ color: c, roughness: 0.7, metalness: 0.1 }));
    }
    return this.materials.get(key);
  }

  clear() {
    this.root.clear();
    this.dynamic = [];
    this.character = null;
    if (this.points) {
      this.points.geometry.dispose();
      this.scene.remove(this.points);
      this.points = null;
    }
  }

  // Scene description from {"type":"loaded"}.
  load(desc) {
    this.clear();
    this.addBodies(desc.bodies);
    const castShadows = this.dynamic.length <= 600;  // Shadows get expensive with many bodies.
    this.sun.castShadow = castShadows;
    if (desc.particles) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
      const material = new THREE.PointsMaterial({ color: 0x7cc4ff, size: Math.max(0.03, desc.particleRadius * 2.5), sizeAttenuation: true });
      this.points = new THREE.Points(geometry, material);
      this.points.frustumCulled = false;
      this.scene.add(this.points);
    }
    if (desc.character) this.addCharacter(desc.character);
    this.frameCamera(desc.scene);
  }

  // Skinned character: flat list of bones posed in world space each frame (pose comes from
  // vexora_sim, which already resolved animation / ragdoll / blending).
  addCharacter(c) {
    const bones = c.parents.map(() => new THREE.Bone());
    const inverses = c.parents.map((_, i) => {
      const o = i * 7, ib = c.inverseBind;
      return new THREE.Matrix4().compose(new THREE.Vector3(ib[o], ib[o + 1], ib[o + 2]),
        new THREE.Quaternion(ib[o + 3], ib[o + 4], ib[o + 5], ib[o + 6]), new THREE.Vector3(1, 1, 1));
    });
    for (const b of bones) this.root.add(b);
    const skeleton = new THREE.Skeleton(bones, inverses);
    for (const m of c.meshes) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(m.positions, 3));
      g.setAttribute('skinIndex', new THREE.Uint16BufferAttribute(m.joints, 4));
      g.setAttribute('skinWeight', new THREE.BufferAttribute(m.weights, 4));
      g.setIndex(new THREE.BufferAttribute(m.indices, 1));
      if (m.normals) g.setAttribute('normal', new THREE.BufferAttribute(m.normals, 3));
      else g.computeVertexNormals();
      const mesh = new THREE.SkinnedMesh(g, new THREE.MeshStandardMaterial({ color: m.color, roughness: 0.75, metalness: 0.05 }));
      mesh.bind(skeleton, new THREE.Matrix4());
      mesh.castShadow = mesh.receiveShadow = true;
      mesh.frustumCulled = false;  // Bounds are from the rest pose; poses move far from it.
      this.root.add(mesh);
    }
    let lines = null;
    if (c.showSkeleton) {
      const pairs = c.parents.filter((p) => p >= 0).length;
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pairs * 6), 3));
      lines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0xffd060, depthTest: false }));
      lines.renderOrder = 10;
      lines.frustumCulled = false;
      this.root.add(lines);
    }
    this.character = { bones, parents: c.parents, lines };
  }

  applyCharacterPose(pose) {
    const { bones, parents, lines } = this.character;
    for (let i = 0; i < bones.length && (i + 1) * 7 <= pose.length; i++) {
      bones[i].position.set(pose[i * 7], pose[i * 7 + 1], pose[i * 7 + 2]);
      bones[i].quaternion.set(pose[i * 7 + 3], pose[i * 7 + 4], pose[i * 7 + 5], pose[i * 7 + 6]);
    }
    if (lines) {
      const a = lines.geometry.attributes.position.array;
      let k = 0;
      parents.forEach((p, i) => {
        if (p < 0) return;
        a.set([pose[p * 7], pose[p * 7 + 1], pose[p * 7 + 2], pose[i * 7], pose[i * 7 + 1], pose[i * 7 + 2]], k);
        k += 6;
      });
      lines.geometry.attributes.position.needsUpdate = true;
    }
  }

  addBodies(bodies) {
    for (const b of bodies) {
      const mesh = new THREE.Mesh(this.geometryFor(b.shape, b.size), this.materialFor(b.color, false));
      mesh.position.fromArray(b.position);
      mesh.quaternion.fromArray(b.rotation);
      mesh.castShadow = b.dynamic;
      mesh.receiveShadow = true;
      mesh.userData.color = b.color;
      this.root.add(mesh);
      if (b.dynamic) this.dynamic[b.index] = mesh;
    }
  }

  // Fit the camera to where the scene's objects are (the floor is ignored).
  frameCamera(sceneId) {
    const fixed = {
      fountain: { target: [0, 2.5, 0], radius: 15, pitch: 0.3 },
      tail: { target: [0, 1, -0.8], radius: 9, pitch: 0.3, yaw: 1.35 },  // Side view: the tail is behind the body.
      character: { target: [0, 0.9, 0], radius: 5, pitch: 0.2 },
    }[sceneId];
    if (fixed) {
      this.orbit.target.fromArray(fixed.target);
      this.orbit.radius = fixed.radius;
      this.orbit.pitch = fixed.pitch;
      if (fixed.yaw !== undefined) this.orbit.yaw = fixed.yaw;
      return;
    }
    const box = new THREE.Box3();
    for (const mesh of this.dynamic) if (mesh) box.expandByPoint(mesh.position);
    if (box.isEmpty()) box.set(new THREE.Vector3(-2, 0, -2), new THREE.Vector3(2, 4, 2));
    if (sceneId === 'chain') box.min.x -= box.max.x - box.min.x;  // It swings down to both sides of the pivot.
    box.min.y = 0;  // Objects fall towards the floor; keep it in view.
    const size = box.getSize(new THREE.Vector3());
    box.getCenter(this.orbit.target);
    this.orbit.radius = Math.max(sceneId === 'ragdoll' ? 6 : 8, Math.max(size.x, size.y, size.z) * 1.9);
    this.orbit.pitch = sceneId === 'drop' ? 0.45 : 0.25;
  }

  // Frame from the simulation: Float32 transforms (7 per dynamic body), sleep flags, particles.
  applyFrame(frame) {
    const xf = frame.bodies;
    if (xf) {
      for (let i = 0; i < this.dynamic.length; i++) {
        const mesh = this.dynamic[i];
        if (!mesh || (i + 1) * 7 > xf.length) continue;
        mesh.position.set(xf[i * 7], xf[i * 7 + 1], xf[i * 7 + 2]);
        mesh.quaternion.set(xf[i * 7 + 3], xf[i * 7 + 4], xf[i * 7 + 5], xf[i * 7 + 6]);
        if (frame.sleep) mesh.material = this.materialFor(mesh.userData.color, frame.sleep[i] === 1);
      }
    }
    if (this.character && frame.pose) this.applyCharacterPose(frame.pose);
    if (this.points && frame.particles) {
      const geometry = this.points.geometry;
      geometry.setAttribute('position', new THREE.BufferAttribute(frame.particles, 3));
      geometry.computeBoundingSphere();
    }
  }

  render() {
    const o = this.orbit;
    this.camera.position.set(
      o.target.x + o.radius * Math.cos(o.pitch) * Math.sin(o.yaw),
      o.target.y + o.radius * Math.sin(o.pitch),
      o.target.z + o.radius * Math.cos(o.pitch) * Math.cos(o.yaw));
    this.camera.lookAt(o.target);
    this.renderer.render(this.scene, this.camera);
  }
}
