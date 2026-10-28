#include <algorithm>
#include <thread>

#include "ParticleBackend.h"

namespace vexora::physics::detail {
namespace {

// Below this many slots per thread, spawning threads costs more than it saves.
constexpr uint32_t kMinSlotsPerThread = 16384;

class CpuParticleBackend final : public ParticleBackend {
public:
    CpuParticleBackend(uint32_t slots, int threads)
        : slots_(slots), threads_(threads > 0 ? threads : int(std::max(1u, std::thread::hardware_concurrency()))) {}

    Particle* hostSlots() override { return slots_.data(); }

    Status update(const KernelParams& params, const SlotRanges&) override {
        const uint32_t n = uint32_t(slots_.size());
        const uint32_t workers = std::min<uint32_t>(uint32_t(threads_), std::max(1u, n / kMinSlotsPerThread));
        auto run = [&](uint32_t begin, uint32_t end) {
            for (uint32_t i = begin; i < end; ++i) updateParticle(slots_[i], params);
        };
        if (workers <= 1) {
            run(0, n);
            return {};
        }
        // Particles are independent, so splitting the range never changes the result.
        std::vector<std::thread> pool;
        const uint32_t chunk = (n + workers - 1) / workers;
        for (uint32_t w = 1; w < workers; ++w) pool.emplace_back(run, std::min(n, w * chunk), std::min(n, (w + 1) * chunk));
        run(0, std::min(n, chunk));
        for (std::thread& t : pool) t.join();
        return {};
    }

    Status readback(std::vector<Particle>& out) override {
        out = slots_;
        return {};
    }

private:
    std::vector<Particle> slots_;
    int threads_;
};

}  // namespace

std::unique_ptr<ParticleBackend> createCpuParticleBackend(uint32_t slots, int threads) {
    return std::make_unique<CpuParticleBackend>(slots, threads);
}

}  // namespace vexora::physics::detail
