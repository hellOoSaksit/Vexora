#include "vexora/physics/ComputeDevice.h"

#include <algorithm>
#include <thread>

#include "ParticleBackend.h"

namespace vexora::physics {

bool gpuBackendCompiledIn() {
#ifdef VEXORA_PHYSICS_HAS_VULKAN
    return true;
#else
    return false;
#endif
}

namespace {
ComputeDeviceInfo cpuDevice() {
    ComputeDeviceInfo cpu;
    cpu.backend = ComputeBackend::CPU;
    cpu.type = DeviceType::Cpu;
    cpu.name = "CPU";
    cpu.cpuThreads = std::max(1u, std::thread::hardware_concurrency());
    return cpu;
}
}  // namespace

std::vector<ComputeDeviceInfo> enumerateComputeDevices() {
    std::vector<ComputeDeviceInfo> out{cpuDevice()};
#ifdef VEXORA_PHYSICS_HAS_VULKAN
    std::string reason;
    for (ComputeDeviceInfo& d : detail::enumerateVulkanDevices(reason)) out.push_back(std::move(d));
#endif
    return out;
}

std::string gpuUnavailableReason() {
#ifdef VEXORA_PHYSICS_HAS_VULKAN
    std::string reason;
    return detail::enumerateVulkanDevices(reason).empty() ? reason : std::string();
#else
    return "This build of Vexora was compiled without GPU support (VEXORA_PHYSICS_GPU=OFF)";
#endif
}

const char* toString(ComputeBackend backend) {
    switch (backend) {
        case ComputeBackend::Auto: return "Auto";
        case ComputeBackend::CPU: return "CPU";
        case ComputeBackend::GPU: return "GPU";
    }
    return "Unknown";
}

const char* toString(DeviceType type) {
    switch (type) {
        case DeviceType::Cpu: return "CPU";
        case DeviceType::DiscreteGpu: return "Discrete GPU";
        case DeviceType::IntegratedGpu: return "Integrated GPU";
        case DeviceType::VirtualGpu: return "Virtual GPU";
        case DeviceType::SoftwareGpu: return "Software GPU (runs on CPU)";
    }
    return "Unknown";
}

}  // namespace vexora::physics
