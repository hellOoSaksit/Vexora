// Test Scene 10 (Phase A part) — reproducibility: same-binary determinism, thread-count
// independence, and snapshot save/restore replay. Bake caches (Phase F) build on these.
#include "TestHelpers.h"

#include <random>

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {

// A chaotic scene: 200 mixed bodies spawned from a fixed seed into a pile, plus a joint chain.
void buildChaos(PhysicsWorld& world, uint32_t seed) {
    addBoxFloor(world);
    std::mt19937 rng(seed);  // mt19937 output is specified by the standard: identical everywhere.
    auto uni = [&](float lo, float hi) { return lo + (hi - lo) * float(rng() % 100000u) / 100000.0f; };
    for (int i = 0; i < 200; ++i) {
        ShapeDesc s = (i % 3 == 0) ? ShapeDesc::sphere(uni(0.1f, 0.3f))
                     : (i % 3 == 1) ? ShapeDesc::box({uni(0.1f, 0.3f), uni(0.1f, 0.3f), uni(0.1f, 0.3f)})
                                    : ShapeDesc::capsule(uni(0.1f, 0.3f), uni(0.1f, 0.2f));
        BodyDesc d = dynamicBody(s, {uni(-2, 2), uni(1, 15), uni(-2, 2)});
        d.transform.rotation = Quat::fromAxisAngle({uni(-1, 1), uni(-1, 1), uni(0.1f, 1)}, uni(0, 3));
        d.angularVelocity = {uni(-5, 5), uni(-5, 5), uni(-5, 5)};
        mustCreate(world, d);
    }
    BodyHandle prev{};
    for (int i = 0; i < 8; ++i) {
        BodyHandle link = mustCreate(world, dynamicBody(ShapeDesc::box({0.2f, 0.05f, 0.05f}), {4 + 0.4f * i + 0.2f, 6, 0}));
        ConstraintDesc c;
        c.type = ConstraintType::Point;
        c.bodyA = link;
        c.bodyB = prev;
        c.anchor = {4 + 0.4f * i, 6, 0};
        REQUIRE(world.createConstraint(c).ok());
        prev = link;
    }
}

uint64_t simulate(int threads, int steps) {
    WorldSettings ws;
    ws.workerThreads = threads;
    PhysicsWorld world(ws);
    buildChaos(world, 1234);
    for (int i = 0; i < steps; ++i) world.step();
    return world.stateHash();
}

}  // namespace

TEST_CASE("Scene10: same inputs, same binary -> bit-identical results") {
    const uint64_t a = simulate(0, 300);
    const uint64_t b = simulate(0, 300);
    VX_REPORT("determinism run1=%016llx run2=%016llx", (unsigned long long)a, (unsigned long long)b);
    CHECK(a == b);
}

TEST_CASE("Scene10: result does not depend on worker thread count") {
    const uint64_t single = simulate(0, 300);
    const uint64_t four = simulate(4, 300);
    const uint64_t four2 = simulate(4, 300);
    VX_REPORT("threads 0=%016llx 4=%016llx 4(again)=%016llx", (unsigned long long)single, (unsigned long long)four,
              (unsigned long long)four2);
    CHECK(single == four);
    CHECK(four == four2);
}

TEST_CASE("Scene10: snapshot restore replays bit-identically") {
    PhysicsWorld world;
    buildChaos(world, 99);
    for (int i = 0; i < 60; ++i) world.step();
    const SimulationSnapshot snap = world.saveSnapshot();
    for (int i = 0; i < 120; ++i) world.step();
    const uint64_t firstRun = world.stateHash();

    REQUIRE(world.restoreSnapshot(snap).ok());
    CHECK(world.stepIndex() == 60);
    for (int i = 0; i < 120; ++i) world.step();
    VX_REPORT("snapshot size=%zu bytes; replay hash match=%d", snap.data.size(), int(world.stateHash() == firstRun));
    CHECK(world.stateHash() == firstRun);
}

TEST_CASE("Scene10: snapshot from a different world structure is rejected") {
    PhysicsWorld world;
    buildChaos(world, 5);
    const SimulationSnapshot snap = world.saveSnapshot();
    mustCreate(world, dynamicBody(ShapeDesc::sphere(0.5f), {0, 30, 0}));
    const uint64_t before = world.stateHash();
    Status s = world.restoreSnapshot(snap);
    CHECK(s.error == ErrorCode::StateMismatch);
    CHECK(world.stateHash() == before);  // Untouched on failure.
}

TEST_CASE("Scene10: broken constraints stay broken after restore of a post-break snapshot") {
    PhysicsWorld world;
    BodyDesc d = dynamicBody(ShapeDesc::box({0.2f, 0.2f, 0.2f}), {0, 5, 0});
    d.massOverride = 500.0f;
    BodyHandle b = mustCreate(world, d);
    ConstraintDesc c;
    c.type = ConstraintType::Fixed;
    c.bodyA = b;
    c.anchor = {0, 5, 0};
    c.breakForce = 100.0f;
    ConstraintHandle h = world.createConstraint(c).value;
    const SimulationSnapshot before = world.saveSnapshot();
    run(world, 0.5);
    REQUIRE_FALSE(world.isConstraintEnabled(h));
    const SimulationSnapshot after = world.saveSnapshot();
    REQUIRE(world.restoreSnapshot(before).ok());
    CHECK(world.isConstraintEnabled(h));
    REQUIRE(world.restoreSnapshot(after).ok());
    CHECK_FALSE(world.isConstraintEnabled(h));
}
