// Input validation: invalid data is rejected with a reason instead of reaching the solver.
#include "TestHelpers.h"

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {
ShapeDesc unitCubeMesh() {
    std::vector<Vec3> v = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                           {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
    std::vector<uint32_t> i = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4,
                               2, 3, 7, 2, 7, 6, 1, 2, 6, 1, 6, 5, 0, 4, 7, 0, 7, 3};
    return ShapeDesc::triangleMesh(v, i);
}
}  // namespace

TEST_CASE("Dynamic triangle mesh is rejected with guidance; static is accepted") {
    PhysicsWorld world;
    BodyDesc d = dynamicBody(unitCubeMesh(), {0, 2, 0});
    Result<BodyHandle> r = world.createBody(d);
    CHECK_FALSE(r.ok());
    CHECK(r.error == ErrorCode::UnsupportedShapeForMotionType);
    CHECK(r.message.find("Compound") != std::string::npos);

    d.motion = MotionType::Static;
    CHECK(world.createBody(d).ok());
    d.motion = MotionType::Kinematic;
    CHECK(world.createBody(d).ok());
}

TEST_CASE("Invalid shapes and bodies are rejected") {
    PhysicsWorld world;
    CHECK_FALSE(world.createBody(dynamicBody(ShapeDesc::box({0, 1, 1}), {})).ok());
    CHECK_FALSE(world.createBody(dynamicBody(ShapeDesc::sphere(-1.0f), {})).ok());
    CHECK_FALSE(world.createBody(dynamicBody(ShapeDesc::convexHull({{0, 0, 0}, {1, 0, 0}}), {})).ok());
    CHECK_FALSE(world.createBody(dynamicBody(ShapeDesc::compound({}), {})).ok());
    // Coplanar points would make a zero-volume (zero-mass) hull; the engine itself accepts them.
    Result<BodyHandle> flat = world.createBody(dynamicBody(ShapeDesc::convexHull({{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}}), {}));
    CHECK_FALSE(flat.ok());
    CHECK(flat.message.find("coplanar") != std::string::npos);

    BodyDesc nan = dynamicBody(ShapeDesc::sphere(0.5f), {std::nanf(""), 0, 0});
    CHECK_FALSE(world.createBody(nan).ok());
    BodyDesc badMat = dynamicBody(ShapeDesc::sphere(0.5f), {});
    badMat.material.restitution = 2.0f;
    CHECK_FALSE(world.createBody(badMat).ok());
    BodyDesc badLayer = dynamicBody(ShapeDesc::sphere(0.5f), {});
    badLayer.layer = 16;
    CHECK_FALSE(world.createBody(badLayer).ok());
    CHECK(world.bodyCount() == 0);
}

TEST_CASE("Destroyed handles are detected (generational handles)") {
    PhysicsWorld world;
    BodyHandle a = mustCreate(world, dynamicBody(ShapeDesc::sphere(0.5f), {0, 1, 0}));
    CHECK(world.destroyBody(a).ok());
    CHECK_FALSE(world.isAlive(a));
    CHECK(world.destroyBody(a).error == ErrorCode::InvalidHandle);
    BodyHandle b = mustCreate(world, dynamicBody(ShapeDesc::sphere(0.5f), {0, 1, 0}));
    CHECK(b.index == a.index);  // Slot reused...
    CHECK(b != a);              // ...but the old handle does not alias the new body.
    CHECK(world.addImpulse(a, {1, 0, 0}).error == ErrorCode::InvalidHandle);
    CHECK(world.addImpulse(b, {std::nanf(""), 0, 0}).error == ErrorCode::InvalidArgument);
}

TEST_CASE("Destroying a body removes its constraints") {
    PhysicsWorld world;
    BodyHandle a = mustCreate(world, dynamicBody(ShapeDesc::box({0.5f, 0.5f, 0.5f}), {0, 5, 0}));
    ConstraintDesc c;
    c.type = ConstraintType::Point;
    c.bodyA = a;
    c.anchor = {0, 6, 0};
    Result<ConstraintHandle> ch = world.createConstraint(c);
    REQUIRE(ch.ok());
    CHECK(world.destroyBody(a).ok());
    CHECK_FALSE(world.isConstraintEnabled(ch.value));
    CHECK(world.destroyConstraint(ch.value).error == ErrorCode::InvalidHandle);
    world.step();  // Must not touch freed memory.
}

TEST_CASE("Invalid constraints are rejected") {
    PhysicsWorld world;
    BodyHandle s = mustCreate(world, [] { BodyDesc d; d.motion = MotionType::Static; d.shape = ShapeDesc::box({1, 1, 1}); return d; }());
    BodyHandle a = mustCreate(world, dynamicBody(ShapeDesc::box({0.5f, 0.5f, 0.5f}), {0, 5, 0}));
    ConstraintDesc c;
    c.type = ConstraintType::Hinge;
    c.bodyA = s;
    CHECK_FALSE(world.createConstraint(c).ok());  // Static-to-world: nothing can move.
    c.bodyA = a;
    c.axis = {1, 0, 0};
    c.normal = {1, 0, 0};
    CHECK_FALSE(world.createConstraint(c).ok());  // Axis parallel to normal.
    c.normal = {0, 1, 0};
    c.hasLimits = true;
    c.limitMin = 0.5f;
    c.limitMax = 1.0f;
    CHECK_FALSE(world.createConstraint(c).ok());  // Hinge min must be <= 0.
    c.limitMin = -0.5f;
    CHECK(world.createConstraint(c).ok());
}

TEST_CASE("Material presets are distinct, valid starting points") {
    for (MaterialPreset p : {MaterialPreset::Metal, MaterialPreset::Wood, MaterialPreset::Stone, MaterialPreset::Rubber,
                             MaterialPreset::Ice, MaterialPreset::Fabric, MaterialPreset::FleshLike}) {
        PhysicsMaterial m = makeMaterial(p);
        CAPTURE(m.name);
        CHECK(m.density > 0.0f);
        CHECK(m.friction >= 0.0f);
        CHECK(m.restitution >= 0.0f);
        CHECK(m.restitution <= 1.0f);
    }
    CHECK(makeMaterial(MaterialPreset::Ice).friction < makeMaterial(MaterialPreset::Rubber).friction);
}
