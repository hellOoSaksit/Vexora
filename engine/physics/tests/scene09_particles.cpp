// Test Scene 9 — Particle Collision, plus CPU/GPU backend selection and parity.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "vexora/physics/ParticleSystem.h"

using namespace vexora::physics;

namespace {

#define VX_REPORT(...) std::printf("  [measure] " __VA_ARGS__), std::printf("\n")

ParticleSettings fountain(uint32_t maxParticles) {
    ParticleSettings s;
    s.maxParticles = maxParticles;
    s.emissionRate = float(maxParticles) / 2.0f;  // Fills about 2/2.5 of the pool at steady state.
    s.emitterPosition = {0, 1, 0};
    s.initialVelocity = {1, 6, 0};
    s.velocitySpread = 2.0f;
    s.wind = {2, 0, 0};
    s.drag = 0.2f;
    s.planes.push_back({{0, 1, 0}, 0.0f});
    s.spheres.push_back({{1.5f, 0.5f, 0}, 0.5f});
    s.seed = 42;
    return s;
}

std::unique_ptr<ParticleSystem> make(const ParticleSettings& s, BackendRequest r = {}) {
    auto res = ParticleSystem::create(s, r);
    INFO(res.message);
    REQUIRE(res.ok());
    return std::move(res.value);
}

bool hasAnyGpu() {
    for (const ComputeDeviceInfo& d : enumerateComputeDevices())
        if (d.backend == ComputeBackend::GPU) return true;
    return false;
}

}  // namespace

TEST_CASE("Scene09: compute devices — CPU is always listed first") {
    std::vector<ComputeDeviceInfo> devices = enumerateComputeDevices();
    REQUIRE_FALSE(devices.empty());
    CHECK((devices[0].backend == ComputeBackend::CPU));
    CHECK(devices[0].cpuThreads >= 1);
    for (const ComputeDeviceInfo& d : devices) {
        VX_REPORT("device: %s | %s | %s | %.0f MB", toString(d.backend), toString(d.type), d.name.c_str(),
                  double(d.memoryBytes) / (1024.0 * 1024.0));
        CHECK_FALSE(d.name.empty());
    }
    if (devices.size() == 1) VX_REPORT("no GPU: %s", gpuUnavailableReason().c_str());
}

TEST_CASE("Scene09: invalid particle settings are rejected") {
    ParticleSettings s = fountain(100);
    s.restitution = 1.5f;
    CHECK_FALSE(ParticleSystem::create(s).ok());
    s = fountain(100);
    s.planes.push_back({{0, 2, 0}, 0});  // Not unit length.
    CHECK_FALSE(ParticleSystem::create(s).ok());
    s = fountain(100);
    s.drag = 100.0f;  // drag * dt > 1 would overshoot the wind velocity.
    CHECK_FALSE(ParticleSystem::create(s).ok());
    s = fountain(0);
    CHECK_FALSE(ParticleSystem::create(s).ok());
    s = fountain(100);
    s.spheres.assign(kMaxParticleColliders + 1, {{0, 0, 0}, 1});
    CHECK_FALSE(ParticleSystem::create(s).ok());
}

TEST_CASE("Scene09: CPU particles never end a step below the floor or inside the sphere") {
    ParticleSettings s = fountain(20000);
    auto sys = make(s, {ComputeBackend::CPU});
    float worstFloor = 0.0f, worstSphere = 0.0f;
    for (int i = 0; i < 300; ++i) {
        REQUIRE(sys->step().ok());
        if (i % 10) continue;
        for (const Particle& p : sys->readback()) {
            if (p.lifetime <= 0.0f) continue;
            worstFloor = std::min(worstFloor, p.py - p.radius);
            const float dx = p.px - 1.5f, dy = p.py - 0.5f, dz = p.pz;
            worstSphere = std::min(worstSphere, std::sqrt(dx * dx + dy * dy + dz * dz) - 0.5f - p.radius);
        }
    }
    VX_REPORT("CPU worst floor penetration=%.6f m, sphere=%.6f m", worstFloor, worstSphere);
    // Planes are resolved last, so the floor is exact (float rounding only). Particles wedged
    // between sphere and floor may stay slightly inside the sphere (measured, see test plan).
    CHECK(worstFloor > -1e-5f);
    CHECK(worstSphere > -5e-3f);
}

TEST_CASE("Scene09: single particle bounce follows restitution") {
    ParticleSettings s;
    s.maxParticles = 1;
    s.emissionRate = 60.0f;  // One particle on the first step; the pool is then full.
    s.emitterRadius = 0.0f;
    s.velocitySpread = 0.0f;
    s.initialVelocity = {0, 0, 0};
    s.emitterPosition = {0, 2.0f, 0};
    s.particleRadius = 0.0f;
    s.drag = 0.0f;
    s.lifetimeMin = s.lifetimeMax = 10.0f;
    s.restitution = 0.5f;
    s.planes.push_back({{0, 1, 0}, 0.0f});
    auto sys = make(s, {ComputeBackend::CPU});
    float peak = 0.0f, prevVy = 0.0f;
    bool bounced = false;
    for (int i = 0; i < 300; ++i) {
        sys->step();
        const Particle& p = sys->readback()[0];
        if (!bounced && prevVy < 0.0f && p.vy > 0.0f) bounced = true;
        if (bounced) peak = std::max(peak, p.py);
        if (bounced && p.vy < 0.0f) break;
        prevVy = p.vy;
    }
    VX_REPORT("particle bounce e=0.5 from 2 m: peak=%.4f expected=%.4f", peak, 0.25f * 2.0f);
    CHECK(peak == doctest::Approx(0.5f).epsilon(0.1));
}

TEST_CASE("Scene09: lifetime and pool limits are exact and deterministic") {
    ParticleSettings s = fountain(1000);
    s.emissionRate = 6000.0f;  // 100 per step: the pool fills in 10 steps.
    s.lifetimeMin = s.lifetimeMax = 0.5f;  // 30 steps.
    auto sys = make(s, {ComputeBackend::CPU});
    for (int i = 0; i < 10; ++i) sys->step();
    CHECK(sys->aliveCount() == 1000);
    for (int i = 0; i < 20; ++i) sys->step();  // Still full: no overwrite of live particles.
    CHECK(sys->aliveCount() == 1000);
    sys->step();  // Step 31: the first 100 particles (born on step 1, 30 steps) expired, 100 new.
    CHECK(sys->aliveCount() == 1000);
    uint32_t counted = 0;
    for (const Particle& p : sys->readback()) counted += p.lifetime > 0.0f ? 1u : 0u;
    CHECK(counted == sys->aliveCount());

    s.emissionRate = 0.0f;
    auto none = make(s, {ComputeBackend::CPU});
    for (int i = 0; i < 10; ++i) none->step();
    CHECK(none->aliveCount() == 0);
}

TEST_CASE("Scene09: CPU result is identical across runs, thread counts and reset") {
    ParticleSettings s = fountain(70000);  // Large enough to engage worker threads.
    auto run = [&](int threads) {
        auto sys = make(s, {ComputeBackend::CPU, -1, false, 50000, threads});
        for (int i = 0; i < 120; ++i) sys->step();
        return sys->readback();
    };
    const std::vector<Particle> a = run(1), b = run(4);
    REQUIRE(a.size() == b.size());
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(Particle)) == 0);

    auto sys = make(s, {ComputeBackend::CPU, -1, false, 50000, 1});
    for (int i = 0; i < 120; ++i) sys->step();
    sys->reset();
    for (int i = 0; i < 120; ++i) sys->step();
    CHECK(std::memcmp(sys->readback().data(), a.data(), a.size() * sizeof(Particle)) == 0);
}

TEST_CASE("Scene09: backend selection rules") {
    ParticleSettings s = fountain(1000);
    // An index that cannot exist: explicit GPU request fails loudly, never silently uses CPU.
    BackendRequest bad{ComputeBackend::GPU, 999, true};
    auto r = ParticleSystem::create(s, bad);
    CHECK_FALSE(r.ok());
    CHECK(r.error == ErrorCode::DeviceUnavailable);

    // Auto below the threshold always picks CPU and says why.
    auto small = make(s, {ComputeBackend::Auto, -1, true, 50000});
    CHECK((small->device().backend == ComputeBackend::CPU));
    VX_REPORT("auto(1000): %s", small->selectionReason().c_str());
    CHECK_FALSE(small->selectionReason().empty());

    // Auto never picks a software GPU unless allowed.
    auto big = make(fountain(60000), {ComputeBackend::Auto, -1, false, 50000});
    if (big->device().backend == ComputeBackend::GPU) CHECK((big->device().type != DeviceType::SoftwareGpu));
    VX_REPORT("auto(60000): %s -> %s", big->selectionReason().c_str(), big->device().name.c_str());
}

TEST_CASE("Scene09: GPU backend matches the CPU backend") {
    if (!hasAnyGpu()) {
        VX_REPORT("No Vulkan compute device: GPU parity NOT tested on this machine (%s)", gpuUnavailableReason().c_str());
        return;
    }
    ParticleSettings s = fountain(50000);
    auto cpu = make(s, {ComputeBackend::CPU});
    auto gpu = make(s, {ComputeBackend::GPU, -1, /*allowSoftwareGpu=*/true});
    VX_REPORT("parity on GPU device: %s (%s)", gpu->device().name.c_str(), toString(gpu->device().type));
    float maxPosDiff = 0.0f, maxVelDiff = 0.0f;
    size_t aliveMismatch = 0;
    for (int i = 0; i < 180; ++i) {
        REQUIRE(cpu->step().ok());
        REQUIRE(gpu->step().ok());
        if (i % 30 != 29) continue;
        const std::vector<Particle>& a = cpu->readback();
        const std::vector<Particle>& b = gpu->readback();
        REQUIRE(a.size() == b.size());
        for (size_t k = 0; k < a.size(); ++k) {
            if ((a[k].lifetime > 0.0f) != (b[k].lifetime > 0.0f)) ++aliveMismatch;
            if (a[k].lifetime <= 0.0f) continue;
            maxPosDiff = std::max({maxPosDiff, std::abs(a[k].px - b[k].px), std::abs(a[k].py - b[k].py), std::abs(a[k].pz - b[k].pz)});
            maxVelDiff = std::max({maxVelDiff, std::abs(a[k].vx - b[k].vx), std::abs(a[k].vy - b[k].vy), std::abs(a[k].vz - b[k].vz)});
        }
    }
    VX_REPORT("CPU vs GPU after 3 s: alive-slot mismatches=%zu, max |dp|=%.2e m, max |dv|=%.2e m/s", aliveMismatch, maxPosDiff,
              maxVelDiff);
    CHECK(aliveMismatch == 0);  // Lifecycle is CPU-side: must match exactly.
    CHECK(cpu->aliveCount() == gpu->aliveCount());
    // Float results differ slightly (FMA, compiler). Collisions can amplify tiny differences, so
    // the bound is loose but still far below any visible difference for VFX.
    CHECK(maxPosDiff < 1e-2f);
}
