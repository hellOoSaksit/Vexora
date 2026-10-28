// Compute device discovery and CPU/GPU backend selection.
//
// Which systems can use the GPU (keep in sync with docs/physics/05-cpu-gpu.md):
//  - Rigid bodies, collision, constraints: CPU only (Jolt Physics has no GPU path).
//  - Particles: CPU or GPU (Vulkan compute).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vexora::physics {

enum class ComputeBackend {
    Auto,  // Pick per system from device type and workload size; never fails if a CPU exists.
    CPU,
    GPU,   // Fails with DeviceUnavailable instead of silently falling back to CPU.
};

enum class DeviceType {
    Cpu,            // Vexora's own multi-threaded CPU implementation.
    DiscreteGpu,
    IntegratedGpu,
    VirtualGpu,
    SoftwareGpu,    // A Vulkan driver that runs on the CPU (e.g. Mesa llvmpipe/lavapipe).
};

struct ComputeDeviceInfo {
    ComputeBackend backend = ComputeBackend::CPU;
    DeviceType type = DeviceType::Cpu;
    std::string name;
    uint32_t index = 0;           // Index among GPU devices (for GPU entries).
    uint64_t memoryBytes = 0;     // Device-local memory (GPU) or 0 when unknown.
    uint32_t cpuThreads = 0;      // Hardware threads (CPU entry).
    std::string apiVersion;       // e.g. "Vulkan 1.3.0".
};

// True if this build includes the Vulkan GPU backend (CMake option VEXORA_PHYSICS_GPU).
bool gpuBackendCompiledIn();

// The CPU entry is always first. GPU entries follow when a Vulkan loader and a device with
// compute support are present. Never throws; a missing driver just yields no GPU entries.
std::vector<ComputeDeviceInfo> enumerateComputeDevices();

// Why GPU devices are missing, for display in the UI ("" when at least one GPU was found).
std::string gpuUnavailableReason();

const char* toString(ComputeBackend backend);
const char* toString(DeviceType type);

}  // namespace vexora::physics
