// Test Scene 1 — Rigid Body Falling and Collision.
// See docs/physics/03-test-plan.md for expected results and acceptance criteria.
#include "TestHelpers.h"

#include <cmath>

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {

// Rest-height tolerance: Jolt keeps up to PhysicsSettings::mPenetrationSlop (0.02 m) of
// allowed penetration, plus solver tolerance. 3 cm is the acceptance bound for preview.
constexpr float kRestTolerance = 0.03f;

struct ShapeCase {
    const char* name;
    ShapeDesc shape;
    float restHeight;  // Expected centre height when resting on y = 0.
};

std::vector<ShapeCase> shapeCases() {
    std::vector<Vec3> tetra = {{-0.5f, 0, -0.5f}, {0.5f, 0, -0.5f}, {0, 0, 0.5f}, {0, 0.8f, 0}};
    auto cubeA = std::make_shared<ShapeDesc>(ShapeDesc::box({0.25f, 0.25f, 0.25f}));
    std::vector<CompoundChild> dumbbell = {{cubeA, {{-0.5f, 0, 0}, {}}}, {cubeA, {{0.5f, 0, 0}, {}}}};
    return {
        {"box", ShapeDesc::box({0.5f, 0.5f, 0.5f}), 0.5f},
        {"sphere", ShapeDesc::sphere(0.5f), 0.5f},
        // Capsule along Y would roll unpredictably; it falls upright and stays up only if
        // perfectly balanced, so we drop it lying down (rotated 90 degrees about Z).
        {"capsule", ShapeDesc::capsule(0.5f, 0.25f), 0.25f},
        // Hull origin is at the base; centre of mass is computed by the engine.
        {"convex_hull", ShapeDesc::convexHull(tetra), 0.0f},
        {"compound", ShapeDesc::compound(dumbbell), 0.25f},
    };
}

void dropOnto(bool meshFloor) {
    for (const ShapeCase& sc : shapeCases()) {
        PhysicsWorld world;
        if (meshFloor) {
            // Static triangle mesh floor (allowed for static bodies).
            BodyDesc f;
            f.motion = MotionType::Static;
            f.shape = ShapeDesc::triangleMesh({{-20, 0, -20}, {20, 0, -20}, {20, 0, 20}, {-20, 0, 20}}, {0, 2, 1, 0, 3, 2});
            mustCreate(world, f);
        } else {
            addBoxFloor(world);
        }
        BodyDesc d = dynamicBody(sc.shape, {0, 3, 0});
        if (std::string(sc.name) == "capsule") d.transform.rotation = Quat::fromAxisAngle({0, 0, 1}, 1.5707963f);
        BodyHandle body = mustCreate(world, d);

        float minY = 1e9f;
        bool sawBegin = false;
        for (int i = 0; i < 300; ++i) {  // 5 s at 60 Hz
            world.step();
            minY = std::min(minY, world.getTransform(body).position.y);
            for (const ContactEvent& e : world.contactEvents())
                if (e.type == ContactEventType::Begin) sawBegin = true;
        }
        const Transform t = world.getTransform(body);
        const float speed = world.getLinearVelocity(body).length();
        VX_REPORT("%s floor=%s restY=%.4f minY=%.4f speed=%.5f sleeping=%d", sc.name, meshFloor ? "mesh" : "box",
                  t.position.y, minY, speed, int(world.isSleeping(body)));
        CAPTURE(sc.name);
        CHECK(sawBegin);
        CHECK(world.isSleeping(body));
        CHECK(speed < 0.05f);
        CHECK(t.position.y > sc.restHeight - kRestTolerance);  // Did not sink / pass through.
        if (std::string(sc.name) != "convex_hull")             // Hull may settle on a face or edge.
            CHECK(t.position.y < sc.restHeight + kRestTolerance);
        CHECK(minY > sc.restHeight - 0.1f);  // No deep transient penetration on impact.
    }
}

}  // namespace

TEST_CASE("Scene01: each collision shape falls, collides, comes to rest and sleeps (box floor)") { dropOnto(false); }

TEST_CASE("Scene01: each collision shape rests on a static triangle-mesh floor") { dropOnto(true); }

TEST_CASE("Scene01: free fall matches analytic gravity before impact") {
    PhysicsWorld world;
    BodyDesc d = dynamicBody(ShapeDesc::sphere(0.5f), {0, 100, 0});
    d.linearDamping = 0.0f;
    BodyHandle b = mustCreate(world, d);
    run(world, 1.0);
    const float y = world.getTransform(b).position.y;
    // Semi-implicit Euler at 60 Hz: y = y0 - g*t^2/2 - g*t*dt/2 (one-step bias).
    const float expected = 100.0f - 0.5f * 9.81f * 1.0f - 0.5f * 9.81f * 1.0f / 60.0f;
    VX_REPORT("free fall after 1s: y=%.4f expected=%.4f", y, expected);
    CHECK(y == doctest::Approx(expected).epsilon(1e-3));
    CHECK(world.getLinearVelocity(b).y == doctest::Approx(-9.81f).epsilon(1e-3));
}

TEST_CASE("Scene01: restitution — bounce height follows e^2 * h") {
    for (float e : {0.0f, 0.5f, 0.8f}) {
        PhysicsWorld world({{0, -9.81f, 0}, QualityProfile::HighQualityPreview});
        addBoxFloor(world, 0.5f, e);
        BodyDesc d = dynamicBody(ShapeDesc::sphere(0.25f), {0, 2.25f, 0});  // Drop height 2 m.
        d.material.restitution = e;
        d.linearDamping = 0.0f;
        BodyHandle b = mustCreate(world, d);
        float peakAfterBounce = 0.0f;
        bool bounced = false;
        for (int i = 0; i < 360; ++i) {
            world.step();
            const float vy = world.getLinearVelocity(b).y;
            const float y = world.getTransform(b).position.y - 0.25f;
            if (!bounced && vy > 0.0f) bounced = true;
            if (bounced) peakAfterBounce = std::max(peakAfterBounce, y);
            if (bounced && vy < 0.0f) break;
        }
        const float expected = e * e * 2.0f;
        VX_REPORT("restitution e=%.2f first bounce peak=%.4f expected=%.4f", e, peakAfterBounce, expected);
        CAPTURE(e);
        // Discrete stepping loses/gains a little energy at impact: 10% of drop height or 15% relative.
        CHECK(std::abs(peakAfterBounce - expected) <= std::max(0.15f * expected, 0.05f));
    }
}

TEST_CASE("Scene01: friction — box holds on an incline when mu > tan(theta) and slides when below") {
    const float theta = 0.35f;  // ~20 degrees, tan = 0.365
    for (float mu : {0.6f, 0.2f}) {
        PhysicsWorld world;
        BodyDesc ramp;
        ramp.motion = MotionType::Static;
        ramp.shape = ShapeDesc::box({10, 0.5f, 10});
        ramp.transform.rotation = Quat::fromAxisAngle({0, 0, 1}, theta);
        ramp.material.friction = mu;
        mustCreate(world, ramp);
        // Place the box resting on the ramp surface.
        const Quat rot = Quat::fromAxisAngle({0, 0, 1}, theta);
        BodyDesc box = dynamicBody(ShapeDesc::box({0.25f, 0.25f, 0.25f}), rot.rotate({0, 0.75f, 0}));
        box.transform.rotation = rot;
        box.material.friction = mu;
        BodyHandle b = mustCreate(world, box);
        const Vec3 start = world.getTransform(b).position;
        run(world, 2.0);
        const float moved = (world.getTransform(b).position - start).length();
        VX_REPORT("incline tan=%.3f mu=%.2f displacement after 2s=%.4f m", std::tan(theta), mu, moved);
        CAPTURE(mu);
        if (mu > std::tan(theta)) CHECK(moved < 0.02f);
        else CHECK(moved > 1.0f);
    }
}

TEST_CASE("Scene01: tunnelling — fast thin-wall hit, discrete vs continuous collision") {
    auto shoot = [](bool ccd) {
        PhysicsWorld world({{0, 0, 0}});
        BodyDesc wall;
        wall.motion = MotionType::Static;
        wall.shape = ShapeDesc::box({0.01f, 2.0f, 2.0f});  // 2 cm thick
        wall.transform.position = {5, 0, 0};
        mustCreate(world, wall);
        BodyDesc bullet = dynamicBody(ShapeDesc::sphere(0.05f), {0, 0, 0});
        bullet.linearVelocity = {300, 0, 0};  // 5 m per 60 Hz step: far more than its size.
        bullet.continuousCollision = ccd;
        bullet.linearDamping = 0.0f;
        BodyHandle b = mustCreate(world, bullet);
        run(world, 0.5);
        return world.getTransform(b).position.x;
    };
    const float discreteX = shoot(false);
    const float ccdX = shoot(true);
    VX_REPORT("tunnelling: discrete final x=%.3f, linear-cast CCD final x=%.3f (wall at x=5)", discreteX, ccdX);
    CHECK(discreteX > 5.0f);  // Documents the limitation: discrete stepping tunnels here.
    CHECK(ccdX < 5.0f);       // CCD prevents it.
}

TEST_CASE("Scene01: collision layers filter contacts") {
    PhysicsWorld world;
    world.setLayerCollision(1, 0, false);  // Layer 1 ignores layer 0 (floor).
    addBoxFloor(world);
    BodyDesc ghost = dynamicBody(ShapeDesc::sphere(0.5f), {0, 2, 0});
    ghost.layer = 1;
    BodyHandle g = mustCreate(world, ghost);
    BodyHandle solid = mustCreate(world, dynamicBody(ShapeDesc::sphere(0.5f), {3, 2, 0}));
    run(world, 2.0);
    CHECK(world.getTransform(g).position.y < -5.0f);
    CHECK(world.getTransform(solid).position.y == doctest::Approx(0.5f).epsilon(0.05));
    CHECK_FALSE(world.layersCollide(0, 1));
    CHECK(world.layersCollide(0, 2));
}

TEST_CASE("Scene01: mass from density and explicit mass override") {
    PhysicsWorld world;
    BodyDesc d = dynamicBody(ShapeDesc::box({0.5f, 0.5f, 0.5f}), {0, 5, 0});
    d.material.density = 500.0f;
    BodyHandle byDensity = mustCreate(world, d);
    d.massOverride = 42.0f;
    BodyHandle byOverride = mustCreate(world, d);
    CHECK(world.getMass(byDensity) == doctest::Approx(500.0f).epsilon(1e-3));  // 1 m^3
    CHECK(world.getMass(byOverride) == doctest::Approx(42.0f).epsilon(1e-3));
    // Off-centre compound: centre of mass shifts towards the heavier side.
    auto big = std::make_shared<ShapeDesc>(ShapeDesc::box({0.5f, 0.5f, 0.5f}));
    auto small = std::make_shared<ShapeDesc>(ShapeDesc::box({0.1f, 0.1f, 0.1f}));
    BodyDesc c = dynamicBody(ShapeDesc::compound({{big, {{1, 0, 0}, {}}}, {small, {{-1, 0, 0}, {}}}}), {0, 10, 0});
    BodyHandle comp = mustCreate(world, c);
    const Vec3 com = world.getCenterOfMass(comp);
    VX_REPORT("compound COM x=%.4f (expected ~ +0.984)", com.x);
    CHECK(com.x == doctest::Approx((1.0f * 1.0f - 0.008f * 1.0f) / 1.008f).epsilon(1e-3));
}

TEST_CASE("Scene01: forces, torque and impulses") {
    PhysicsWorld world({{0, 0, 0}});
    BodyDesc d = dynamicBody(ShapeDesc::sphere(0.5f), {});
    d.massOverride = 2.0f;
    d.linearDamping = 0.0f;
    d.angularDamping = 0.0f;
    BodyHandle b = mustCreate(world, d);
    CHECK(world.addImpulse(b, {4, 0, 0}).ok());  // dv = J/m = 2 m/s
    world.step();
    CHECK(world.getLinearVelocity(b).x == doctest::Approx(2.0f).epsilon(1e-4));
    for (int i = 0; i < 60; ++i) {
        CHECK(world.addForce(b, {0, 2, 0}).ok());  // a = 1 m/s^2 for 1 s
        world.step();
    }
    CHECK(world.getLinearVelocity(b).y == doctest::Approx(1.0f).epsilon(1e-3));
    CHECK(world.addTorque(b, {0, 0, 10}).ok());
    world.step();
    CHECK(world.getAngularVelocity(b).z > 0.0f);
}
