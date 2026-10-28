// Internal interface between ParticleSystem (lifecycle, emission) and its compute backends.
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ParticleKernel.h"
#include "vexora/physics/ComputeDevice.h"

namespace vexora::physics::detail {

// Inclusive-exclusive slot ranges [first, last) that the CPU changed since the last update.
using SlotRanges = std::vector<std::pair<uint32_t, uint32_t>>;

class ParticleBackend {
public:
    virtual ~ParticleBackend() = default;
    // CPU-writable copy of every slot. The CPU writes spawned/killed particles here and lists
    // them in the next update(); other slots in this array may be stale on the GPU backend.
    virtual Particle* hostSlots() = 0;
    virtual Status update(const KernelParams& params, const SlotRanges& dirty) = 0;
    virtual Status readback(std::vector<Particle>& out) = 0;
};

std::unique_ptr<ParticleBackend> createCpuParticleBackend(uint32_t slots, int threads);

#ifdef VEXORA_PHYSICS_HAS_VULKAN
std::vector<ComputeDeviceInfo> enumerateVulkanDevices(std::string& unavailableReason);
Result<std::unique_ptr<ParticleBackend>> createVulkanParticleBackend(uint32_t gpuIndex, uint32_t slots);
#endif

}  // namespace vexora::physics::detail
