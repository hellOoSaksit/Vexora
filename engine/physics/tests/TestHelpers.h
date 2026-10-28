// Shared builders for physics test scenes.
#pragma once

#include <doctest/doctest.h>

#include <cstdio>

#include "vexora/physics/PhysicsWorld.h"

namespace vexora::physics::test {

inline BodyHandle mustCreate(PhysicsWorld& world, const BodyDesc& desc) {
    Result<BodyHandle> r = world.createBody(desc);
    INFO(r.message);
    REQUIRE(r.ok());
    return r.value;
}

// Static floor whose top surface is at y = 0.
inline BodyHandle addBoxFloor(PhysicsWorld& world, float friction = 0.5f, float restitution = 0.0f) {
    BodyDesc d;
    d.name = "floor";
    d.motion = MotionType::Static;
    d.shape = ShapeDesc::box({50.0f, 0.5f, 50.0f});
    d.transform.position = {0.0f, -0.5f, 0.0f};
    d.material.friction = friction;
    d.material.restitution = restitution;
    return mustCreate(world, d);
}

inline BodyDesc dynamicBody(const ShapeDesc& shape, Vec3 position) {
    BodyDesc d;
    d.motion = MotionType::Dynamic;
    d.shape = shape;
    d.transform.position = position;
    return d;
}

inline void run(PhysicsWorld& world, double seconds) {
    const int steps = static_cast<int>(seconds / world.fixedDt() + 0.5);
    for (int i = 0; i < steps; ++i) world.step();
}

// Measured values are printed so test logs double as the evidence for docs/physics/03-test-plan.md.
#define VX_REPORT(...) std::printf("  [measure] " __VA_ARGS__), std::printf("\n")

}  // namespace vexora::physics::test
