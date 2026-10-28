// Test Scene 13 — Performance stress test.
// Drops N mixed bodies into a walled bin and measures per-step cost for each quality profile
// and thread count. Usage: vexora_physics_bench [seconds=4] [N...]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <thread>
#include <vector>

#include "vexora/physics/PhysicsWorld.h"

using namespace vexora::physics;

namespace {

void buildBin(PhysicsWorld& world, int count) {
    auto wall = [&](Vec3 pos, Vec3 half) {
        BodyDesc d;
        d.motion = MotionType::Static;
        d.shape = ShapeDesc::box(half);
        d.transform.position = pos;
        world.createBody(d);
    };
    wall({0, -0.5f, 0}, {20, 0.5f, 20});
    wall({-10, 5, 0}, {0.5f, 5, 10});
    wall({10, 5, 0}, {0.5f, 5, 10});
    wall({0, 5, -10}, {10, 5, 0.5f});
    wall({0, 5, 10}, {10, 5, 0.5f});

    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    const int perLayer = 15 * 15;
    for (int i = 0; i < count; ++i) {
        const int layer = i / perLayer, k = i % perLayer;
        BodyDesc d;
        d.shape = (i % 3 == 0) ? ShapeDesc::sphere(0.25f)
                : (i % 3 == 1) ? ShapeDesc::box({0.25f, 0.25f, 0.25f})
                               : ShapeDesc::capsule(0.2f, 0.15f);
        d.transform.position = {-8.4f + (k % 15) * 1.2f + 0.1f * u(rng), 1.0f + layer * 1.0f, -8.4f + (k / 15) * 1.2f + 0.1f * u(rng)};
        d.transform.rotation = Quat::fromAxisAngle({u(rng), u(rng), 1.0f}, u(rng) * 3.0f);
        world.createBody(d);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const double seconds = argc > 1 ? std::atof(argv[1]) : 4.0;
    std::vector<int> counts;
    for (int i = 2; i < argc; ++i) counts.push_back(std::atoi(argv[i]));
    if (counts.empty()) counts = {500, 2000, 5000};
    const int hw = int(std::max(1u, std::thread::hardware_concurrency()));

    std::printf("hardware threads: %d, simulated time per run: %.1f s\n", hw, seconds);
    std::printf("%-20s %6s %8s %10s %10s %10s %8s %9s\n", "profile", "bodies", "workers", "avg ms", "p95 ms", "max ms",
                "x-real", "warnings");
    for (QualityProfile profile : {QualityProfile::InteractivePreview, QualityProfile::HighQualityPreview, QualityProfile::FinalBake}) {
        for (int n : counts) {
            for (int workers : {0, hw - 1}) {
                if (workers == 0 && hw - 1 == 0 && n != counts.front()) continue;
                WorldSettings ws;
                ws.profile = profile;
                ws.workerThreads = workers;
                PhysicsWorld world(ws);
                buildBin(world, n);
                const int steps = int(seconds / world.fixedDt() + 0.5);
                std::vector<double> ms;
                ms.reserve(size_t(steps));
                size_t warnings = 0;
                for (int s = 0; s < steps; ++s) {
                    world.step();
                    ms.push_back(world.lastStepStats().stepMilliseconds);
                    warnings += world.warnings().size();
                }
                std::vector<double> sorted = ms;
                std::sort(sorted.begin(), sorted.end());
                double total = 0.0;
                for (double v : ms) total += v;
                const double avg = total / double(ms.size());
                const double p95 = sorted[size_t(0.95 * double(sorted.size() - 1))];
                // x-real > 1 means the simulation runs faster than real time.
                const double realtime = (world.fixedDt() * 1000.0) / avg;
                std::printf("%-20s %6d %8d %10.3f %10.3f %10.3f %8.2f %9zu\n", toString(profile), n, workers, avg, p95,
                            sorted.back(), realtime, warnings);
                std::fflush(stdout);
            }
        }
    }
    return 0;
}
