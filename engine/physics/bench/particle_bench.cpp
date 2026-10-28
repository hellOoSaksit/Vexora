// CPU vs GPU particle benchmark: the data for choosing BackendRequest::autoGpuMinParticles.
// Usage: vexora_particle_bench [steps=300] [count...]
// Times include GPU submit + wait; one readback per run is timed separately.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "vexora/physics/ParticleSystem.h"

using namespace vexora::physics;

int main(int argc, char** argv) {
    const int steps = argc > 1 ? std::atoi(argv[1]) : 300;
    std::vector<uint32_t> counts;
    for (int i = 2; i < argc; ++i) counts.push_back(uint32_t(std::atoi(argv[i])));
    if (counts.empty()) counts = {10000, 100000, 1000000};

    std::printf("devices:\n");
    for (const ComputeDeviceInfo& d : enumerateComputeDevices())
        std::printf("  %-4s %-28s %s\n", toString(d.backend), toString(d.type), d.name.c_str());
    if (enumerateComputeDevices().size() == 1) std::printf("  (no GPU: %s)\n", gpuUnavailableReason().c_str());

    struct Config { const char* label; BackendRequest req; };
    const int hw = int(std::max(1u, std::thread::hardware_concurrency()));
    std::vector<Config> configs = {
        {"CPU 1 thread", {ComputeBackend::CPU, -1, false, 0, 1}},
        {"CPU all threads", {ComputeBackend::CPU, -1, false, 0, hw}},
        {"GPU", {ComputeBackend::GPU, -1, true, 0, 0}},
    };

    std::printf("\n%-16s %-36s %9s %10s %10s %12s\n", "backend", "device", "particles", "avg ms", "p95 ms", "readback ms");
    for (uint32_t n : counts) {
        for (const Config& c : configs) {
            ParticleSettings s;
            s.maxParticles = n;
            s.emissionRate = float(n);  // Pool saturates after ~1 s; later steps update n slots.
            s.lifetimeMin = 1.5f;
            s.lifetimeMax = 2.5f;
            s.planes.push_back({{0, 1, 0}, 0});
            s.spheres.push_back({{1, 0.5f, 0}, 0.5f});
            auto made = ParticleSystem::create(s, c.req);
            if (!made.ok()) {
                std::printf("%-16s %-36s %9u  unavailable: %s\n", c.label, "-", n, made.message.c_str());
                continue;
            }
            ParticleSystem& sys = *made.value;
            std::vector<double> ms;
            for (int i = 0; i < steps; ++i) {
                if (!sys.step().ok()) {
                    std::printf("step failed\n");
                    return 1;
                }
                ms.push_back(sys.lastStepMilliseconds());
            }
            const auto t0 = std::chrono::steady_clock::now();
            sys.readback();
            const double readMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            std::sort(ms.begin(), ms.end());
            double total = 0;
            for (double v : ms) total += v;
            std::printf("%-16s %-36.36s %9u %10.3f %10.3f %12.3f\n", c.label, sys.device().name.c_str(), n,
                        total / double(ms.size()), ms[size_t(0.95 * double(ms.size() - 1))], readMs);
            std::fflush(stdout);
        }
    }
    return 0;
}
