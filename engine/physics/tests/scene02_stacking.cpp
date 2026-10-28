// Test Scene 2 — Stacked Objects and Stability.
#include "TestHelpers.h"

#include <cmath>

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {

struct StackResult {
    float maxDrift = 0.0f;   // Horizontal drift of any box from its start column.
    float maxSink = 0.0f;    // Lowest box below its ideal height.
    int sleeping = 0;
    int total = 0;
    size_t warnings = 0;
};

StackResult runTower(QualityProfile profile, int height, double seconds) {
    PhysicsWorld world({{0, -9.81f, 0}, profile});
    addBoxFloor(world);
    std::vector<BodyHandle> boxes;
    const float h = 0.25f;
    for (int i = 0; i < height; ++i)
        boxes.push_back(mustCreate(world, dynamicBody(ShapeDesc::box({h, h, h}), {0, h + i * 2.0f * h + 0.001f * i, 0})));
    StackResult r;
    for (int i = 0, n = int(seconds / world.fixedDt() + 0.5); i < n; ++i) {
        world.step();
        r.warnings += world.warnings().size();
    }
    for (int i = 0; i < height; ++i) {
        const Vec3 p = world.getTransform(boxes[i]).position;
        r.maxDrift = std::max(r.maxDrift, std::sqrt(p.x * p.x + p.z * p.z));
        r.maxSink = std::max(r.maxSink, (h + i * 2.0f * h) - p.y);
        r.sleeping += world.isSleeping(boxes[i]) ? 1 : 0;
    }
    r.total = height;
    return r;
}

}  // namespace

TEST_CASE("Scene02: 10-box tower stays upright and sleeps (every profile)") {
    for (QualityProfile p : {QualityProfile::InteractivePreview, QualityProfile::HighQualityPreview, QualityProfile::FinalBake}) {
        StackResult r = runTower(p, 10, 10.0);
        VX_REPORT("tower10 %s drift=%.4f sink=%.4f sleeping=%d/%d", toString(p), r.maxDrift, r.maxSink, r.sleeping, r.total);
        CAPTURE(toString(p));
        CHECK(r.maxDrift < 0.02f);
        CHECK(r.maxSink < 0.05f);  // Accumulated slop over 10 contacts.
        CHECK(r.sleeping == r.total);
        CHECK(r.warnings == 0);
    }
}

TEST_CASE("Scene02: 55-box pyramid is stable") {
    PhysicsWorld world;
    addBoxFloor(world);
    const float h = 0.25f;
    std::vector<std::pair<BodyHandle, Vec3>> boxes;
    for (int row = 0; row < 10; ++row)
        for (int i = 0; i < 10 - row; ++i) {
            const Vec3 p{(i - (9 - row) * 0.5f) * 2.05f * h, h + row * 2.0f * h, 0};
            boxes.push_back({mustCreate(world, dynamicBody(ShapeDesc::box({h, h, h}), p)), p});
        }
    run(world, 10.0);
    float maxMove = 0.0f;
    int sleeping = 0;
    for (auto& [b, start] : boxes) {
        maxMove = std::max(maxMove, (world.getTransform(b).position - start).length());
        sleeping += world.isSleeping(b) ? 1 : 0;
    }
    VX_REPORT("pyramid55 max displacement=%.4f sleeping=%d/%zu", maxMove, sleeping, boxes.size());
    CHECK(boxes.size() == 55);
    CHECK(maxMove < 0.05f);
    CHECK(sleeping == int(boxes.size()));
}

TEST_CASE("Scene02: sleeping stack wakes when hit and settles again") {
    PhysicsWorld world;
    addBoxFloor(world);
    std::vector<BodyHandle> boxes;
    for (int i = 0; i < 5; ++i) boxes.push_back(mustCreate(world, dynamicBody(ShapeDesc::box({0.25f, 0.25f, 0.25f}), {0, 0.25f + i * 0.5f, 0})));
    run(world, 5.0);
    for (BodyHandle b : boxes) REQUIRE(world.isSleeping(b));
    BodyDesc ball = dynamicBody(ShapeDesc::sphere(0.2f), {-3, 2.25f, 0});
    ball.linearVelocity = {15, 0, 0};
    ball.massOverride = 20.0f;
    mustCreate(world, ball);
    run(world, 0.5);
    CHECK_FALSE(world.isSleeping(boxes[4]));
    run(world, 15.0);
    int sleeping = 0;
    for (BodyHandle b : boxes) sleeping += world.isSleeping(b) ? 1 : 0;
    CHECK(sleeping == 5);
}
