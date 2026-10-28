# Vexora

3D content-creation software for character animation, physical simulation, VFX and game-asset production.

## Status

Only the **Physics & Simulation core (Phase A)** exists so far: `engine/physics`, a C++17 library built on
[Jolt Physics](https://github.com/jrouwe/JoltPhysics) v5.3.0 (MIT), with headless test scenes and a stress benchmark, plus a particle system with CPU and Vulkan GPU backends
(rigid bodies are CPU-only).
Character rig import (glTF), skinning and animation playback live in `engine/rig` ([docs/rig](docs/rig/README.md)).
See [docs/physics](docs/physics/README.md) for the engine evaluation, architecture, test plan with measured results,
and roadmap.

## วิธีใช้งานแบบง่าย (Windows)

**ไม่อยาก build เอง:** เปิดแท็บ **Actions** ของรีโปบน GitHub → เลือกงาน `windows-app` ล่าสุดที่ขึ้นเครื่องหมายถูก →
ดาวน์โหลด **Vexora-Windows** ในส่วน Artifacts → แตกไฟล์ zip แล้วเลือก
`Vexora-Portable-*.exe` (เปิดได้ทันที) หรือ `Vexora-Setup-*.exe` (ติดตั้งลงเครื่อง)

**Build เอง:** ติดตั้ง CMake, Visual Studio 2022 (C++) และ Node.js LTS แล้วดับเบิลคลิก

- `build-windows.bat` → สร้างไฟล์ `.exe` ไว้ที่ `app\dist\`
- `start-windows.bat` → เปิดโปรแกรมทันทีโดยไม่ต้องติดตั้ง

ไฟล์ .exe ยังไม่ได้ลงลายเซ็นดิจิทัล (code signing) Windows อาจขึ้นหน้าจอ "Windows protected your PC"
ให้กด **More info → Run anyway**

## App structure

```
app/                     Electron UI (plain HTML/JS + three.js, no bundler)
  main.js                window + starts the background simulation process (vexora_sim)
  renderer/              settings forms are generated from the schema vexora_sim sends
engine/simserver/        vexora_sim: JSON-lines protocol on stdin/stdout, scenes, settings schema
engine/core/             shared math and Result types
engine/rig/              skeleton, poses, skinning, animation clips, glTF import (cgltf)
assets/samples/          CC-BY / CC0 glTF sample characters (see its README for credits)
```

## Build and test

Requirements: CMake ≥ 3.24, a C++17 compiler, network access on first configure (fetches pinned Jolt, doctest,
Vulkan-Headers and volk). No Vulkan SDK is needed: the loader is opened at runtime, and machines without a
Vulkan driver use the CPU. `-DVEXORA_PHYSICS_GPU=OFF` builds without GPU support. Optional: `glslangValidator`
to regenerate shader SPIR-V (`--target vexora_physics_shaders`).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/engine/physics/bench/vexora_physics_bench
```

## Layout

```
engine/physics/include/vexora/physics/   public API (no Jolt types)
engine/physics/src/                      implementation; JoltPhysicsWorld.cpp is the only file including Jolt
engine/physics/tests/                    test scenes (doctest)
engine/physics/shaders/                  GPU compute shaders (GLSL); SPIR-V committed in src/generated
engine/physics/bench/                    rigid-body stress benchmark (scene 13) and CPU/GPU particle benchmark
docs/physics/                            design and evaluation documents
```

## Third-party licenses

- Jolt Physics — MIT, Copyright 2021 Jorrit Rouwe
- doctest (tests only) — MIT
- volk — MIT
- Vulkan-Headers — Apache-2.0 / MIT
- nlohmann/json — MIT
- cgltf — MIT
- Electron, three.js — MIT
