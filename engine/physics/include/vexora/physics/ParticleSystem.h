// Particle simulation (VFX physics) with interchangeable CPU and GPU backends.
//
// Both backends run the same per-particle update (see ParticleKernel.h and particles.comp)
// on the same buffer layout. Emission is always done on the CPU from a seeded RNG, so the
// set of particles is identical on both backends; positions agree within float tolerance
// but are NOT bit-identical between CPU and GPU (different compilers/FMA use).
//
// Simulation is separate from rendering: the system only produces particle state.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vexora/physics/ComputeDevice.h"
#include "vexora/physics/Types.h"

namespace vexora::physics {

// GPU-friendly layout (std430): 3 x vec4 = 48 bytes per particle.
struct Particle {
    float px, py, pz, age;       // position, seconds alive
    float vx, vy, vz, lifetime;  // velocity, total lifetime (<= 0: slot is free)
    float radius, pad0, pad1, pad2;
};
static_assert(sizeof(Particle) == 48, "Particle must match the GPU layout");

struct PlaneCollider {
    Vec3 normal{0.0f, 1.0f, 0.0f};  // Unit normal; the solid side is behind the plane.
    float offset = 0.0f;            // dot(normal, p) = offset on the plane.
};

struct SphereCollider {
    Vec3 center;
    float radius = 0.5f;
};

constexpr int kMaxParticleColliders = 8;  // Per collider type; fixed so the GPU kernel needs no extra buffers.

struct ParticleSettings {
    uint32_t maxParticles = 10000;
    float emissionRate = 1000.0f;  // particles per second
    Vec3 emitterPosition{0.0f, 2.0f, 0.0f};
    float emitterRadius = 0.1f;
    Vec3 initialVelocity{0.0f, 5.0f, 0.0f};
    float velocitySpread = 2.0f;   // Random +-spread added per axis.
    float lifetimeMin = 2.0f;
    float lifetimeMax = 3.0f;
    float particleRadius = 0.02f;
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    Vec3 wind{0.0f, 0.0f, 0.0f};
    float drag = 0.1f;             // 1/s, pulls velocity towards the wind velocity.
    float restitution = 0.5f;
    float friction = 0.1f;         // Fraction of tangential velocity removed per bounce.
    std::vector<PlaneCollider> planes;
    std::vector<SphereCollider> spheres;
    uint32_t seed = 1;
    double fixedDt = 1.0 / 60.0;
};

struct BackendRequest {
    ComputeBackend backend = ComputeBackend::Auto;
    int gpuIndex = -1;              // -1: best GPU. Otherwise ComputeDeviceInfo::index.
    bool allowSoftwareGpu = false;  // Software Vulkan is slower than our CPU path; tests only.
    // Auto uses a hardware GPU only at or above this many particles. Must be calibrated with
    // vexora_particle_bench on real hardware (see docs/physics/05-cpu-gpu.md).
    uint32_t autoGpuMinParticles = 50000;
    int cpuThreads = 0;             // 0 = hardware concurrency.
};

class ParticleSystem {
public:
    // Validates settings and creates the requested backend.
    static Result<std::unique_ptr<ParticleSystem>> create(const ParticleSettings& settings,
                                                         const BackendRequest& request = {});
    ~ParticleSystem();
    ParticleSystem(const ParticleSystem&) = delete;
    ParticleSystem& operator=(const ParticleSystem&) = delete;

    // One fixed step: emit (CPU, seeded), then update every particle on the active backend.
    Status step();
    // Re-emits from the initial state; same seed -> same emission sequence.
    void reset();

    // Copies particle state to the CPU (a GPU readback on the GPU backend).
    const std::vector<Particle>& readback();
    uint32_t aliveCount();

    const ComputeDeviceInfo& device() const;
    // Human-readable explanation of why this backend was chosen (shown in the UI).
    const std::string& selectionReason() const;
    uint64_t stepIndex() const;
    double lastStepMilliseconds() const;

private:
    ParticleSystem();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

Status validateParticleSettings(const ParticleSettings& settings);

}  // namespace vexora::physics
