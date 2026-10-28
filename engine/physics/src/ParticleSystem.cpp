#include "vexora/physics/ParticleSystem.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <random>

#include "ParticleBackend.h"

namespace vexora::physics {

namespace {

Status invalid(std::string msg) { return {ErrorCode::InvalidArgument, std::move(msg)}; }
bool finite(float v) { return std::isfinite(v); }

// Deterministic uniform float in [0,1): std::uniform_real_distribution is implementation-
// defined, mt19937 output is not, so this gives the same sequence on every platform.
float unit(std::mt19937& rng) { return float(rng() >> 8) * (1.0f / 16777216.0f); }

}  // namespace

Status validateParticleSettings(const ParticleSettings& s) {
    if (s.maxParticles == 0 || s.maxParticles > 64u * 1024u * 1024u) return invalid("maxParticles must be in [1, 67108864]");
    if (!finite(s.emissionRate) || s.emissionRate < 0.0f) return invalid("Emission rate must be >= 0");
    if (!(s.fixedDt > 0.0) || !std::isfinite(s.fixedDt)) return invalid("Time step must be > 0");
    if (!finite(s.lifetimeMin) || s.lifetimeMin <= 0.0f || !finite(s.lifetimeMax) || s.lifetimeMax < s.lifetimeMin)
        return invalid("Lifetime needs 0 < min <= max");
    if (!finite(s.particleRadius) || s.particleRadius < 0.0f) return invalid("Particle radius must be >= 0");
    if (!finite(s.emitterRadius) || s.emitterRadius < 0.0f || !finite(s.velocitySpread) || s.velocitySpread < 0.0f)
        return invalid("Emitter radius and velocity spread must be >= 0");
    if (!finite(s.drag) || s.drag < 0.0f || s.drag * float(s.fixedDt) > 1.0f)
        return invalid("Drag must be >= 0 and drag * timestep <= 1 (higher values make the integration unstable)");
    if (!(s.restitution >= 0.0f && s.restitution <= 1.0f)) return invalid("Restitution must be in [0, 1]");
    if (!(s.friction >= 0.0f && s.friction <= 1.0f)) return invalid("Friction must be in [0, 1]");
    if (!s.emitterPosition.isFinite() || !s.initialVelocity.isFinite() || !s.gravity.isFinite() || !s.wind.isFinite())
        return invalid("Emitter, velocity, gravity and wind must be finite");
    if (s.planes.size() > kMaxParticleColliders || s.spheres.size() > kMaxParticleColliders)
        return invalid("At most " + std::to_string(kMaxParticleColliders) + " planes and " +
                       std::to_string(kMaxParticleColliders) + " spheres are supported");
    for (const PlaneCollider& p : s.planes)
        if (!p.normal.isFinite() || std::abs(p.normal.length() - 1.0f) > 1e-3f || !finite(p.offset))
            return invalid("Plane collider normals must be unit length");
    for (const SphereCollider& sp : s.spheres)
        if (!sp.center.isFinite() || !finite(sp.radius) || sp.radius <= 0.0f) return invalid("Sphere colliders need radius > 0");
    return {};
}

struct ParticleSystem::Impl {
    ParticleSettings settings;
    std::unique_ptr<detail::ParticleBackend> backend;
    ComputeDeviceInfo device;
    std::string reason;
    detail::KernelParams params{};

    std::mt19937 rng;
    double emitAccumulator = 0.0;
    uint64_t step = 0;
    uint32_t alive = 0;
    std::vector<uint32_t> freeSlots;              // Stack; lowest slot index on top.
    std::map<uint64_t, std::vector<uint32_t>> deaths;  // step -> slots that expire then
    std::vector<uint32_t> dirty;
    std::vector<Particle> readbackCache;
    bool readbackValid = false;
    double lastMs = 0.0;

    void resetState() {
        rng.seed(settings.seed);
        emitAccumulator = 0.0;
        step = 0;
        alive = 0;
        freeSlots.resize(settings.maxParticles);
        for (uint32_t i = 0; i < settings.maxParticles; ++i) freeSlots[i] = settings.maxParticles - 1 - i;
        deaths.clear();
        dirty.clear();
        Particle* host = backend->hostSlots();
        std::memset(host, 0, sizeof(Particle) * settings.maxParticles);
        for (uint32_t i = 0; i < settings.maxParticles; ++i) dirty.push_back(i);
        readbackValid = false;
    }

    void emitOne(uint64_t atStep) {
        const uint32_t slot = freeSlots.back();
        freeSlots.pop_back();
        const ParticleSettings& s = settings;
        // Uniform point inside the emitter sphere by rejection sampling.
        float ox, oy, oz;
        do {
            ox = unit(rng) * 2.0f - 1.0f;
            oy = unit(rng) * 2.0f - 1.0f;
            oz = unit(rng) * 2.0f - 1.0f;
        } while (ox * ox + oy * oy + oz * oz > 1.0f);
        Particle p{};
        p.px = s.emitterPosition.x + ox * s.emitterRadius;
        p.py = s.emitterPosition.y + oy * s.emitterRadius;
        p.pz = s.emitterPosition.z + oz * s.emitterRadius;
        p.vx = s.initialVelocity.x + (unit(rng) * 2.0f - 1.0f) * s.velocitySpread;
        p.vy = s.initialVelocity.y + (unit(rng) * 2.0f - 1.0f) * s.velocitySpread;
        p.vz = s.initialVelocity.z + (unit(rng) * 2.0f - 1.0f) * s.velocitySpread;
        p.lifetime = s.lifetimeMin + unit(rng) * (s.lifetimeMax - s.lifetimeMin);
        p.radius = s.particleRadius;
        backend->hostSlots()[slot] = p;
        dirty.push_back(slot);
        // Lifetime in whole steps, decided here (integer) so both backends kill on the same step.
        const uint64_t lifeSteps = std::max<uint64_t>(1, uint64_t(std::ceil(double(p.lifetime) / s.fixedDt)));
        deaths[atStep + lifeSteps].push_back(slot);
        ++alive;
    }

    detail::SlotRanges takeDirtyRanges() {
        std::sort(dirty.begin(), dirty.end());
        dirty.erase(std::unique(dirty.begin(), dirty.end()), dirty.end());
        detail::SlotRanges ranges;
        for (uint32_t slot : dirty) {
            if (!ranges.empty() && ranges.back().second == slot) ranges.back().second = slot + 1;
            else ranges.push_back({slot, slot + 1});
        }
        dirty.clear();
        return ranges;
    }
};

ParticleSystem::ParticleSystem() : impl_(std::make_unique<Impl>()) {}
ParticleSystem::~ParticleSystem() = default;

Result<std::unique_ptr<ParticleSystem>> ParticleSystem::create(const ParticleSettings& settings, const BackendRequest& req) {
    using R = Result<std::unique_ptr<ParticleSystem>>;
    Status valid = validateParticleSettings(settings);
    if (!valid.ok()) return R::failure(valid.error, valid.message);

    std::unique_ptr<ParticleSystem> sys(new ParticleSystem());
    Impl& m = *sys->impl_;
    m.settings = settings;

    // ---- Backend selection ------------------------------------------------------------
    std::vector<ComputeDeviceInfo> devices = enumerateComputeDevices();
    const ComputeDeviceInfo* gpu = nullptr;
    auto usable = [&](const ComputeDeviceInfo& d) {
        return d.backend == ComputeBackend::GPU && (req.allowSoftwareGpu || d.type != DeviceType::SoftwareGpu);
    };
    auto rank = [](DeviceType t) { return t == DeviceType::DiscreteGpu ? 0 : t == DeviceType::IntegratedGpu ? 1 : t == DeviceType::VirtualGpu ? 2 : 3; };
    for (const ComputeDeviceInfo& d : devices) {
        if (!usable(d)) continue;
        if (req.gpuIndex >= 0) {
            if (d.index == uint32_t(req.gpuIndex)) gpu = &d;
        } else if (!gpu || rank(d.type) < rank(gpu->type)) {
            gpu = &d;
        }
    }
    std::string noGpu = !gpuBackendCompiledIn() ? gpuUnavailableReason()
                        : devices.size() == 1    ? gpuUnavailableReason()
                        : req.gpuIndex >= 0      ? "GPU " + std::to_string(req.gpuIndex) + " not found or not allowed"
                                                 : "Only software Vulkan devices found (slower than the CPU path)";

    bool useGpu = false;
    switch (req.backend) {
        case ComputeBackend::CPU:
            m.reason = "CPU requested";
            break;
        case ComputeBackend::GPU:
            if (!gpu) return R::failure(ErrorCode::DeviceUnavailable, "GPU requested but unavailable: " + noGpu);
            useGpu = true;
            m.reason = "GPU requested";
            break;
        case ComputeBackend::Auto:
            if (!gpu) m.reason = "Auto: CPU (" + noGpu + ")";
            else if (gpu->type == DeviceType::SoftwareGpu) m.reason = "Auto: CPU (only a software GPU is available)";
            else if (settings.maxParticles < req.autoGpuMinParticles)
                m.reason = "Auto: CPU (" + std::to_string(settings.maxParticles) + " particles is below the GPU threshold of " +
                           std::to_string(req.autoGpuMinParticles) + ")";
            else {
                useGpu = true;
                m.reason = "Auto: GPU (" + std::to_string(settings.maxParticles) + " particles)";
            }
            break;
    }

    if (useGpu) {
#ifdef VEXORA_PHYSICS_HAS_VULKAN
        auto backend = detail::createVulkanParticleBackend(gpu->index, settings.maxParticles);
        if (!backend.ok()) {
            if (req.backend == ComputeBackend::GPU) return R::failure(backend.error, backend.message);
            m.reason += "; GPU init failed, using CPU: " + backend.message;
            useGpu = false;
        } else {
            m.backend = std::move(backend.value);
            m.device = *gpu;
        }
#endif
    }
    if (!useGpu) {
        m.backend = detail::createCpuParticleBackend(settings.maxParticles, req.cpuThreads);
        m.device = devices.front();
    }

    // ---- Kernel parameters ------------------------------------------------------------
    detail::KernelParams& k = m.params;
    k.gravityDt[0] = settings.gravity.x; k.gravityDt[1] = settings.gravity.y; k.gravityDt[2] = settings.gravity.z;
    k.gravityDt[3] = float(settings.fixedDt);
    k.windDrag[0] = settings.wind.x; k.windDrag[1] = settings.wind.y; k.windDrag[2] = settings.wind.z;
    k.windDrag[3] = settings.drag;
    k.response[0] = settings.restitution;
    k.response[1] = settings.friction;
    k.counts[0] = settings.maxParticles;
    k.counts[1] = uint32_t(settings.planes.size());
    k.counts[2] = uint32_t(settings.spheres.size());
    for (size_t i = 0; i < settings.planes.size(); ++i) {
        const PlaneCollider& p = settings.planes[i];
        k.planes[i][0] = p.normal.x; k.planes[i][1] = p.normal.y; k.planes[i][2] = p.normal.z; k.planes[i][3] = p.offset;
    }
    for (size_t i = 0; i < settings.spheres.size(); ++i) {
        const SphereCollider& s = settings.spheres[i];
        k.spheres[i][0] = s.center.x; k.spheres[i][1] = s.center.y; k.spheres[i][2] = s.center.z; k.spheres[i][3] = s.radius;
    }
    m.resetState();
    return R::success(std::move(sys));
}

Status ParticleSystem::step() {
    Impl& m = *impl_;
    const auto start = std::chrono::steady_clock::now();
    ++m.step;
    // 1. Expire particles whose lifetime ends this step (CPU-side, integer bookkeeping).
    auto it = m.deaths.find(m.step);
    if (it != m.deaths.end()) {
        for (uint32_t slot : it->second) {
            m.backend->hostSlots()[slot].lifetime = 0.0f;
            m.dirty.push_back(slot);
            m.freeSlots.push_back(slot);
            --m.alive;
        }
        m.deaths.erase(it);
        // Keep allocation order deterministic: lowest free slot is reused first.
        std::sort(m.freeSlots.begin(), m.freeSlots.end(), std::greater<uint32_t>());
    }
    // 2. Emit (seeded RNG). If the pool is full, particles are not emitted (no overwrite).
    m.emitAccumulator += double(m.settings.emissionRate) * m.settings.fixedDt;
    while (m.emitAccumulator >= 1.0) {
        m.emitAccumulator -= 1.0;
        if (m.freeSlots.empty()) continue;
        m.emitOne(m.step);
    }
    // 3. Update every slot on the active backend.
    Status s = m.backend->update(m.params, m.takeDirtyRanges());
    m.readbackValid = false;
    m.lastMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return s;
}

void ParticleSystem::reset() { impl_->resetState(); }

const std::vector<Particle>& ParticleSystem::readback() {
    Impl& m = *impl_;
    if (!m.readbackValid) {
        if (m.backend->readback(m.readbackCache).ok()) m.readbackValid = true;
        else m.readbackCache.clear();
    }
    return m.readbackCache;
}

uint32_t ParticleSystem::aliveCount() { return impl_->alive; }
const ComputeDeviceInfo& ParticleSystem::device() const { return impl_->device; }
const std::string& ParticleSystem::selectionReason() const { return impl_->reason; }
uint64_t ParticleSystem::stepIndex() const { return impl_->step; }
double ParticleSystem::lastStepMilliseconds() const { return impl_->lastMs; }

}  // namespace vexora::physics
