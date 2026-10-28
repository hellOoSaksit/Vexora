// Transform synchronisation (single writer per body), interpolation, events, debug draw.
#include "TestHelpers.h"

#include "vexora/physics/SimulationClock.h"

using namespace vexora::physics;
using namespace vexora::physics::test;

TEST_CASE("Kinematic body follows animation targets exactly and pushes dynamic bodies") {
    PhysicsWorld world;
    addBoxFloor(world);
    BodyDesc k;
    k.motion = MotionType::Kinematic;
    k.shape = ShapeDesc::box({0.25f, 0.5f, 1.0f});
    k.transform.position = {-2, 0.5f, 0};
    BodyHandle paddle = mustCreate(world, k);
    BodyHandle box = mustCreate(world, dynamicBody(ShapeDesc::box({0.25f, 0.25f, 0.25f}), {0, 0.25f, 0}));

    // "Animation" moves the paddle 1 m/s along +X for 3 seconds.
    float maxTrackingError = 0.0f;
    for (int i = 1; i <= 180; ++i) {
        const Transform target{{-2.0f + i / 60.0f, 0.5f, 0}, {}};
        REQUIRE(world.setKinematicTarget(paddle, target).ok());
        world.step();
        maxTrackingError = std::max(maxTrackingError, (world.getTransform(paddle).position - target.position).length());
    }
    VX_REPORT("kinematic tracking error=%.6f m, pushed box x=%.3f", maxTrackingError, world.getTransform(box).position.x);
    CHECK(maxTrackingError < 1e-4f);                      // Physics never overrides the animated body.
    CHECK(world.getTransform(box).position.x > 0.4f);     // ...but its motion acts on dynamic bodies.
    CHECK(world.setKinematicTarget(box, {}).error == ErrorCode::InvalidArgument);  // Dynamic: physics owns it.
    CHECK(world.addForce(paddle, {1, 0, 0}).error == ErrorCode::InvalidArgument);  // Kinematic: animation owns it.
}

TEST_CASE("Render interpolation blends previous and current physics transforms") {
    PhysicsWorld world({{0, 0, 0}});
    BodyDesc d = dynamicBody(ShapeDesc::sphere(0.5f), {0, 0, 0});
    d.linearVelocity = {6, 0, 0};
    d.linearDamping = 0.0f;
    BodyHandle b = mustCreate(world, d);
    FixedStepClock clock(world.fixedDt());
    int steps = clock.advance(1.5 / 60.0);  // One and a half steps of frame time.
    for (int i = 0; i < steps; ++i) world.step();
    REQUIRE(steps == 1);
    const float alpha = clock.interpolationAlpha();
    CHECK(alpha == doctest::Approx(0.5f));
    const float x = world.getInterpolatedTransform(b, alpha).position.x;
    CHECK(x == doctest::Approx(0.05f).epsilon(1e-3));  // Half of 6 m/s * 1/60 s.
    // Teleport resets interpolation history so the body does not smear across the jump.
    REQUIRE(world.teleport(b, {{100, 0, 0}, {}}).ok());
    CHECK(world.getInterpolatedTransform(b, 0.0f).position.x == doctest::Approx(100.0f));
}

TEST_CASE("Contact events: begin / persist / end, and trigger volumes") {
    PhysicsWorld world;
    BodyHandle floor = addBoxFloor(world);
    BodyDesc trig;
    trig.motion = MotionType::Static;
    trig.isTrigger = true;
    trig.shape = ShapeDesc::box({1, 1, 1});
    trig.transform.position = {0, 3, 0};
    BodyHandle trigger = mustCreate(world, trig);
    BodyDesc bd = dynamicBody(ShapeDesc::sphere(0.25f), {0, 6, 0});
    bd.material.restitution = 0.0f;
    BodyHandle ball = mustCreate(world, bd);

    bool enteredTrigger = false, leftTrigger = false, hitFloor = false, persisted = false;
    for (int i = 0; i < 180; ++i) {
        world.step();
        for (const ContactEvent& e : world.contactEvents()) {
            const bool withTrigger = (e.bodyA == trigger || e.bodyB == trigger) && (e.bodyA == ball || e.bodyB == ball);
            const bool withFloor = (e.bodyA == floor || e.bodyB == floor) && (e.bodyA == ball || e.bodyB == ball);
            if (withTrigger && e.type == ContactEventType::Begin) enteredTrigger = e.isTrigger;
            if (withTrigger && e.type == ContactEventType::End) leftTrigger = true;
            if (withFloor && e.type == ContactEventType::Begin) {
                hitFloor = true;
                CHECK(e.point.y == doctest::Approx(0.0f).epsilon(0.05));
            }
            if (withFloor && e.type == ContactEventType::Persist) persisted = true;
        }
    }
    CHECK(enteredTrigger);
    CHECK(leftTrigger);  // Passed through: triggers have no collision response.
    CHECK(hitFloor);
    CHECK(persisted);
}

TEST_CASE("Debug draw exposes bounds, centre of mass, velocity, contacts and constraints") {
    PhysicsWorld world;
    addBoxFloor(world);
    BodyHandle a = mustCreate(world, dynamicBody(ShapeDesc::box({0.5f, 0.5f, 0.5f}), {0, 0.5f, 0}));
    ConstraintDesc c;
    c.type = ConstraintType::Point;
    c.bodyA = a;
    c.anchor = {0, 2, 0};
    REQUIRE(world.createConstraint(c).ok());
    world.step();
    DebugDrawList list;
    world.collectDebugDraw(list, DebugDrawBounds);
    CHECK(list.lines.size() == 24);  // 12 edges x 2 bodies.
    DebugDrawList all;
    world.collectDebugDraw(all);
    CHECK(all.lines.size() > 24);
    CHECK_FALSE(all.points.empty());
}

TEST_CASE("NaN injected through the API is rejected; world keeps running") {
    PhysicsWorld world;
    BodyHandle b = mustCreate(world, dynamicBody(ShapeDesc::sphere(0.5f), {0, 5, 0}));
    CHECK_FALSE(world.addForce(b, {std::nanf(""), 0, 0}).ok());
    CHECK_FALSE(world.teleport(b, {{std::nanf(""), 0, 0}, {}}).ok());
    run(world, 0.5);
    CHECK(world.getTransform(b).position.isFinite());
    CHECK(world.warnings().empty());
}

TEST_CASE("Excessive velocity is reported as a solver warning") {
    PhysicsWorld world({{0, 0, 0}});
    BodyDesc d = dynamicBody(ShapeDesc::sphere(0.5f), {0, 0, 0});
    d.linearVelocity = {400, 0, 0};
    d.linearDamping = 0.0f;
    mustCreate(world, d);
    world.step();
    REQUIRE(world.warnings().size() == 1);
    CHECK(world.warnings()[0].type == WarningType::ExcessiveVelocity);
}
