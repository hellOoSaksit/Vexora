// Test Scene 3 — Jointed Mechanical Objects (constraints computed by Jolt).
#include "TestHelpers.h"

#include <cmath>

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {

// Released horizontally, the chain whips; joint stretch peaks under that tension.
// Max gap thresholds per profile = measured baseline + headroom (see test plan).
void hingeChain(QualityProfile profile, float maxAllowedGap) {
    PhysicsWorld world({{0, -9.81f, 0}, profile});
    const float half = 0.25f;  // Link half length along X.
    std::vector<BodyHandle> links;
    std::vector<Vec3> pivots;
    for (int i = 0; i < 10; ++i) {
        BodyDesc d = dynamicBody(ShapeDesc::box({half, 0.05f, 0.05f}), {half + i * 2 * half, 10, 0});
        d.linearDamping = 0.0f;
        d.angularDamping = 0.0f;
        d.allowSleeping = false;
        d.massOverride = 1.0f;
        d.layer = 1;
        links.push_back(mustCreate(world, d));
    }
    world.setLayerCollision(1, 1, false);  // Neighbouring links overlap at the pivots.
    for (int i = 0; i < 10; ++i) {
        ConstraintDesc c;
        c.type = ConstraintType::Hinge;
        c.bodyA = links[i];
        c.bodyB = i == 0 ? BodyHandle{} : links[i - 1];
        c.anchor = {i * 2 * half, 10, 0};
        c.axis = {0, 0, 1};
        c.normal = {1, 0, 0};
        REQUIRE(world.createConstraint(c).ok());
    }
    auto energy = [&] {
        float e = 0.0f;
        for (BodyHandle b : links) {
            const float v = world.getLinearVelocity(b).length();
            const float w = world.getAngularVelocity(b).length();
            // Box inertia about z: m(a^2+b^2)/12 with full lengths a=0.5, b=0.1; potential relative to y = 0.
            const float I = (half * half * 4 + 0.01f) / 12.0f;
            e += 0.5f * v * v + 0.5f * I * w * w + 9.81f * world.getTransform(b).position.y;
        }
        return e;
    };
    const float e0 = energy();
    float maxGap = 0.0f, maxEnergy = e0;
    for (int i = 0, n = int(10.0 / world.fixedDt() + 0.5); i < n; ++i) {  // 10 s
        world.step();
        maxEnergy = std::max(maxEnergy, energy());
        // Gap: end of link i-1 vs start of link i.
        for (int k = 1; k < 10; ++k) {
            const Transform a = world.getTransform(links[k - 1]);
            const Transform b = world.getTransform(links[k]);
            const Vec3 endA = a.position + a.rotation.rotate({half, 0, 0});
            const Vec3 startB = b.position + b.rotation.rotate({-half, 0, 0});
            maxGap = std::max(maxGap, (endA - startB).length());
        }
        REQUIRE(world.warnings().empty());
    }
    VX_REPORT("hinge chain %s: max joint gap=%.5f m, energy start=%.3f max=%.3f end=%.3f J", toString(profile), maxGap, e0,
              maxEnergy, energy());
    CAPTURE(toString(profile));
    CHECK(maxGap < maxAllowedGap);
    CHECK(maxEnergy <= e0 + 0.01f * std::abs(e0));  // Solver may dissipate energy but must not add it.
}

}  // namespace

TEST_CASE("Scene03: 10-link hinge chain — bounded joint stretch, no energy gain") {
    hingeChain(QualityProfile::InteractivePreview, 0.08f);  // measured 0.064
    hingeChain(QualityProfile::HighQualityPreview, 0.015f); // measured 0.0088
    hingeChain(QualityProfile::FinalBake, 0.005f);          // measured 0.0026
}

TEST_CASE("Scene03: hinge limits and motor") {
    PhysicsWorld world;
    BodyDesc d = dynamicBody(ShapeDesc::box({0.5f, 0.05f, 0.05f}), {0.5f, 5, 0});
    d.allowSleeping = false;
    BodyHandle door = mustCreate(world, d);
    ConstraintDesc c;
    c.type = ConstraintType::Hinge;
    c.bodyA = door;
    c.anchor = {0, 5, 0};
    c.axis = {0, 0, 1};
    c.normal = {1, 0, 0};
    c.hasLimits = true;
    c.limitMin = -0.5f;
    c.limitMax = 0.5f;
    ConstraintHandle h = world.createConstraint(c).value;
    float minAngle = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step();
        minAngle = std::min(minAngle, world.getConstraintPosition(h));
    }
    VX_REPORT("hinge limit -0.5 rad: min angle reached=%.4f", minAngle);
    // Hard limits overshoot briefly on impact (measured 0.03 rad at 60 Hz), then settle on the limit.
    CHECK(minAngle > -0.5f - 0.04f);
    CHECK(std::abs(world.getConstraintPosition(h) + 0.5f) < 0.005f);

    // Velocity motor spinning a wheel in zero gravity reaches its target speed.
    PhysicsWorld zeroG({{0, 0, 0}});
    BodyDesc wd = dynamicBody(ShapeDesc::box({0.5f, 0.5f, 0.1f}), {0, 0, 0});
    wd.allowSleeping = false;
    BodyHandle wheel = mustCreate(zeroG, wd);
    ConstraintDesc m;
    m.type = ConstraintType::Hinge;
    m.bodyA = wheel;
    m.axis = {0, 0, 1};
    m.normal = {1, 0, 0};
    m.motorEnabled = true;
    m.motorTargetVelocity = 3.0f;
    REQUIRE(zeroG.createConstraint(m).ok());
    run(zeroG, 2.0);
    VX_REPORT("hinge motor target 3 rad/s: reached %.4f", zeroG.getAngularVelocity(wheel).z);
    CHECK(zeroG.getAngularVelocity(wheel).z == doctest::Approx(3.0f).epsilon(0.01));
}

TEST_CASE("Scene03: slider limits, fixed joint and soft distance spring") {
    PhysicsWorld world;
    BodyDesc sd = dynamicBody(ShapeDesc::box({0.1f, 0.1f, 0.1f}), {0, 5, 0});
    BodyHandle slider = mustCreate(world, sd);
    ConstraintDesc s;
    s.type = ConstraintType::Slider;
    s.bodyA = slider;
    s.anchor = {0, 5, 0};
    s.axis = {0, 1, 0};
    s.normal = {1, 0, 0};
    s.hasLimits = true;
    s.limitMin = -1.0f;
    s.limitMax = 1.0f;
    ConstraintHandle sh = world.createConstraint(s).value;

    BodyHandle welded = mustCreate(world, dynamicBody(ShapeDesc::box({0.1f, 0.1f, 0.1f}), {3, 5, 0}));
    ConstraintDesc f;
    f.type = ConstraintType::Fixed;
    f.bodyA = welded;
    f.anchor = {3, 5, 0};
    REQUIRE(world.createConstraint(f).ok());

    BodyDesc bd = dynamicBody(ShapeDesc::sphere(0.1f), {6, 4, 0});
    bd.massOverride = 1.0f;
    BodyHandle bob = mustCreate(world, bd);
    ConstraintDesc spring;
    spring.type = ConstraintType::Distance;
    spring.bodyA = bob;
    spring.anchor = {6, 4, 0};
    spring.anchorB = {6, 5, 0};
    spring.hasLimits = true;
    spring.limitMin = spring.limitMax = 1.0f;
    spring.springFrequency = 2.0f;
    spring.springDamping = 1.0f;
    REQUIRE(world.createConstraint(spring).ok());

    run(world, 3.0);
    const Vec3 sp = world.getTransform(slider).position;
    VX_REPORT("slider pos=%.4f (limit -1), fixed drift=%.5f, spring bob y=%.4f",
              world.getConstraintPosition(sh), (world.getTransform(welded).position - Vec3{3, 5, 0}).length(),
              world.getTransform(bob).position.y);
    CHECK(sp.y == doctest::Approx(4.0f).epsilon(0.01));
    CHECK(std::abs(sp.x) < 1e-3f);
    CHECK((world.getTransform(welded).position - Vec3{3, 5, 0}).length() < 0.01f);
    // A 2 Hz spring with 1 kg sags by g/(2*pi*f)^2 = 0.062 m below rest length.
    const float sag = 9.81f / std::pow(2.0f * 3.14159265f * 2.0f, 2.0f);
    CHECK(world.getTransform(bob).position.y == doctest::Approx(4.0f - sag).epsilon(0.01));
}

TEST_CASE("Scene03: breakable fixed joint breaks only above its threshold") {
    auto hang = [](float massKg) {
        PhysicsWorld world;
        BodyDesc d = dynamicBody(ShapeDesc::box({0.2f, 0.2f, 0.2f}), {0, 5, 0});
        d.massOverride = massKg;
        BodyHandle b = mustCreate(world, d);
        ConstraintDesc c;
        c.type = ConstraintType::Fixed;
        c.bodyA = b;
        c.anchor = {0, 5.2f, 0};
        c.breakForce = 1000.0f;  // Holds ~100 kg under gravity.
        ConstraintHandle h = world.createConstraint(c).value;
        bool brokeEvent = false;
        for (int i = 0; i < 120; ++i) {
            world.step();
            for (const ConstraintBrokenEvent& e : world.brokenConstraints()) brokeEvent |= e.constraint == h;
        }
        return std::make_tuple(world.isConstraintEnabled(h), brokeEvent, world.getTransform(b).position.y);
    };
    auto [heldLight, eventLight, yLight] = hang(50.0f);
    auto [heldHeavy, eventHeavy, yHeavy] = hang(200.0f);
    VX_REPORT("breakable: 50kg held=%d y=%.3f, 200kg held=%d y=%.3f", int(heldLight), yLight, int(heldHeavy), yHeavy);
    CHECK(heldLight);
    CHECK_FALSE(eventLight);
    CHECK(yLight == doctest::Approx(5.0f).epsilon(0.01));
    CHECK_FALSE(heldHeavy);
    CHECK(eventHeavy);
    CHECK(yHeavy < 0.0f);  // Fell after release.
}
