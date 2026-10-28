# Vexora

3D content-creation software for character animation, physical simulation, VFX and game-asset production.

## Status

Only the **Physics & Simulation core (Phase A)** exists so far: `engine/physics`, a C++17 library built on
[Jolt Physics](https://github.com/jrouwe/JoltPhysics) v5.3.0 (MIT), with headless test scenes and a stress benchmark, plus a particle system with CPU and Vulkan GPU backends
(rigid bodies are CPU-only).
See [docs/physics](docs/physics/README.md) for the engine evaluation, architecture, test plan with measured results,
and roadmap.

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
