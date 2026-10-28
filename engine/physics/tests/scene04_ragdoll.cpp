// Test Scenes 4 and 5 — Character ragdoll, and animation <-> physics blending.
#include "TestHelpers.h"

#include <cmath>
#include <random>

#include "vexora/physics/Ragdoll.h"

using namespace vexora::physics;
using namespace vexora::physics::test;

namespace {

struct Character {
    DemoHumanoid rig = makeDemoHumanoid();
    LocalPose animLocal = restPose(rig.skeleton);
    ModelPose anim() const { return toModel(rig.skeleton, animLocal); }
};

std::unique_ptr<Ragdoll> makeRagdoll(PhysicsWorld& world, const Character& c, const Transform& at) {
    auto r = Ragdoll::create(world, c.rig.skeleton, c.rig.ragdoll, c.anim(), at);
    INFO(r.message);
    REQUIRE(r.ok());
    return std::move(r.value);
}

float lowestPoint(const PhysicsWorld& world, const Ragdoll& rd) {
    float lowest = 1e9f;
    for (size_t i = 0; i < rd.partCount(); ++i) {
        Vec3 lo, hi;
        REQUIRE(world.getBounds(rd.body(i), lo, hi));
        lowest = std::min(lowest, lo.y);
    }
    return lowest;
}

}  // namespace

TEST_CASE("Scene04: ragdoll falls, respects joint limits, does not sink into the floor, and comes to rest") {
    Character c;
    int seedsWithIssues = 0;
    float worstExcess = 0.0f, worstLowest = 0.0f, worstSettleSpeed = 0.0f;
    for (uint32_t seed = 1; seed <= 25; ++seed) {
        PhysicsWorld world({{0, -9.81f, 0}, QualityProfile::HighQualityPreview});
        addBoxFloor(world);
        const Transform at{{0, 0.6f, 0}, Quat::fromAxisAngle({1, 0, 0}, 0.3f)};
        auto rd = makeRagdoll(world, c, at);
        rd->setDrive(BoneDrive::Simulated);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> u(-1, 1);
        for (size_t p = 0; p < rd->partCount(); ++p)
            world.addImpulse(rd->body(p), {u(rng) * 15.0f, u(rng) * 5.0f, u(rng) * 15.0f});
        float excess = 0.0f, lowest = 1e9f;
        bool warned = false;
        for (int i = 0, n = int(6.0 / world.fixedDt()); i < n; ++i) {
            world.step();
            warned |= !world.warnings().empty();
            for (size_t p = 0; p < rd->partCount(); ++p) excess = std::max(excess, rd->jointSwing(p) - rd->swingLimit(p));
            lowest = std::min(lowest, lowestPoint(world, *rd));
        }
        float settleSpeed = 0.0f;
        for (size_t p = 0; p < rd->partCount(); ++p) {
            settleSpeed = std::max(settleSpeed, world.getLinearVelocity(rd->body(p)).length());
            CHECK(world.getTransform(rd->body(p)).position.isFinite());
        }
        worstExcess = std::max(worstExcess, excess);
        worstLowest = std::min(worstLowest, lowest);
        worstSettleSpeed = std::max(worstSettleSpeed, settleSpeed);
        if (warned || excess > 0.15f || lowest < -0.03f || settleSpeed > 0.05f) ++seedsWithIssues;
    }
    VX_REPORT("ragdoll x25 seeds: worst joint over-limit=%.3f rad, lowest point=%.4f m, max speed after 6 s=%.4f m/s, "
              "seeds with issues=%d", worstExcess, worstLowest, worstSettleSpeed, seedsWithIssues);
    CHECK(seedsWithIssues == 0);
}

TEST_CASE("Scene04: Animated mode follows the animation exactly (kinematic bones)") {
    Character c;
    PhysicsWorld world;
    addBoxFloor(world);
    auto rd = makeRagdoll(world, c, {});
    rd->setDrive(BoneDrive::Animated);
    float worst = 0.0f;
    for (int i = 1; i <= 120; ++i) {
        // Procedural "animation": walk forward while swinging the arms.
        const Transform root{{0, 0, i * 0.02f}, {}};
        c.animLocal[c.rig.skeleton.find("upperArm.L")].rotation = Quat::fromAxisAngle({0, 0, 1}, -1.0f + 0.5f * std::sin(i * 0.1f));
        const ModelPose anim = c.anim();
        rd->drive(anim, root);
        world.step();
        const ModelPose phys = rd->physicsPose(anim, root);
        for (size_t b = 0; b < anim.size(); ++b) {
            worst = std::max(worst, (phys[b].position - anim[b].position).length());
            worst = std::max(worst, angleBetween(phys[b].rotation, anim[b].rotation));
        }
    }
    VX_REPORT("animated mode: max pose error=%.2e (m or rad)", worst);
    CHECK(worst < 1e-3f);
    CHECK(world.motionType(rd->body(0)) == MotionType::Kinematic);
}

TEST_CASE("Scene05: animation -> ragdoll keeps momentum; blend weight 0/1 select the right pose") {
    Character c;
    PhysicsWorld world({{0, -9.81f, 0}});
    addBoxFloor(world);
    auto rd = makeRagdoll(world, c, {});
    rd->setDrive(BoneDrive::Animated);
    // Run forward at 3 m/s under animation control.
    Transform root{};
    for (int i = 1; i <= 60; ++i) {
        root.position.z = 3.0f * float(i) / 60.0f;
        rd->drive(c.anim(), root);
        world.step();
    }
    const Vec3 before = world.getLinearVelocity(rd->body(0));
    rd->setDrive(BoneDrive::Simulated);  // Hit: becomes a ragdoll.
    world.step();
    const Vec3 after = world.getLinearVelocity(rd->body(0));
    VX_REPORT("switch to ragdoll: pelvis velocity before=%.3f after=%.3f m/s (z)", before.z, after.z);
    CHECK(before.z == doctest::Approx(3.0f).epsilon(0.02));
    CHECK(after.z == doctest::Approx(before.z).epsilon(0.1));  // No sudden stop at the switch.

    run(world, 1.0);
    const ModelPose anim = c.anim();
    const ModelPose phys = rd->physicsPose(anim, root);
    const LocalPose animL = toLocal(c.rig.skeleton, anim), physL = toLocal(c.rig.skeleton, phys);
    const std::vector<float> zero(anim.size(), 0.0f), one(anim.size(), 1.0f), half(anim.size(), 0.5f);
    const LocalPose b0 = blendPoses(c.rig.skeleton, animL, physL, zero);
    const LocalPose b1 = blendPoses(c.rig.skeleton, animL, physL, one);
    const LocalPose bh = blendPoses(c.rig.skeleton, animL, physL, half);
    for (size_t b = 0; b < anim.size(); ++b) {
        CHECK(angleBetween(b0[b].rotation, animL[b].rotation) < 1e-6f);
        CHECK(angleBetween(b1[b].rotation, physL[b].rotation) < 1e-5f);
        const float total = angleBetween(animL[b].rotation, physL[b].rotation);
        CHECK(angleBetween(bh[b].rotation, animL[b].rotation) == doctest::Approx(total * 0.5f).epsilon(0.02).scale(1e-3));
    }
}

TEST_CASE("Scene05: Powered mode pulls the ragdoll towards the animated pose") {
    Character c;
    auto finalError = [&](BoneDrive mode) {
        PhysicsWorld world({{0, 0, 0}});  // No gravity: isolate the motor effect.
        auto rd = makeRagdoll(world, c, {});
        rd->setDrive(mode);
        // Target pose: elbows bent 90 degrees. Start from the straight rest pose.
        Character target = c;
        target.animLocal[target.rig.skeleton.find("lowerArm.L")].rotation = Quat::fromAxisAngle({0, 0, 1}, 1.2f);
        target.animLocal[target.rig.skeleton.find("lowerArm.R")].rotation = Quat::fromAxisAngle({0, 0, 1}, -1.2f);
        const ModelPose anim = target.anim();
        for (int i = 0; i < 120; ++i) {
            rd->drive(anim, {});
            world.step();
        }
        const ModelPose phys = rd->physicsPose(anim, {});
        const int elbow = c.rig.skeleton.find("lowerArm.L");
        const int shoulder = c.rig.skeleton.find("upperArm.L");
        const Quat relPhys = phys[size_t(shoulder)].rotation.conjugate() * phys[size_t(elbow)].rotation;
        const Quat relAnim = anim[size_t(shoulder)].rotation.conjugate() * anim[size_t(elbow)].rotation;
        return angleBetween(relPhys, relAnim);
    };
    const float limp = finalError(BoneDrive::Simulated), powered = finalError(BoneDrive::Powered);
    VX_REPORT("elbow error after 2 s: limp=%.3f rad, powered=%.3f rad", limp, powered);
    CHECK(powered < 0.1f);
    CHECK(powered < limp * 0.25f);
}

TEST_CASE("Scene04: invalid ragdoll definitions are rejected; destroying the ragdoll cleans up") {
    Character c;
    PhysicsWorld world;
    RagdollSettings bad = c.rig.ragdoll;
    bad.parts[1].bone = bad.parts[0].bone;  // Same bone twice.
    CHECK_FALSE(Ragdoll::create(world, c.rig.skeleton, bad, c.anim(), {}).ok());
    bad = c.rig.ragdoll;
    std::swap(bad.parts[0], bad.parts[1]);  // Child before parent.
    CHECK_FALSE(Ragdoll::create(world, c.rig.skeleton, bad, c.anim(), {}).ok());
    const uint32_t before = world.bodyCount();
    {
        auto rd = makeRagdoll(world, c, {});
        CHECK(world.bodyCount() == before + rd->partCount());
        world.step();
    }
    CHECK(world.bodyCount() == before);
    world.step();
}

#include "vexora/rig/GltfImport.h"

TEST_CASE("Scene04: auto ragdoll from imported glTF characters falls and settles") {
    for (const char* name : {"RiggedFigure.glb", "Fox.glb"}) {
        auto imported = vexora::rig::importGltf(std::string(VEXORA_SAMPLES_DIR) + "/" + name);
        REQUIRE(imported.ok());
        vexora::rig::ImportedCharacter ch = std::move(imported.value);
        const float height = ch.boundsMax.y - ch.boundsMin.y;
        vexora::rig::scaleCharacter(ch, 1.0f / height);  // 1 m tall regardless of file units.
        const RagdollSettings settings = makeAutoRagdoll(ch.skeleton, ch.restModel, 0.05f);
        PhysicsWorld world({{0, -9.81f, 0}, QualityProfile::HighQualityPreview});
        addBoxFloor(world);
        const Transform at{{0, 0.3f - ch.boundsMin.y, 0}, {}};
        auto rd = Ragdoll::create(world, ch.skeleton, settings, ch.restModel, at);
        INFO(rd.message);
        REQUIRE(rd.ok());
        rd.value->setDrive(BoneDrive::Simulated);
        float lowest = 1e9f;
        bool warned = false;
        for (int i = 0, n = int(6.0 / world.fixedDt()); i < n; ++i) {
            world.step();
            warned |= !world.warnings().empty();
            for (size_t p = 0; p < rd.value->partCount(); ++p) {
                Vec3 lo, hi;
                world.getBounds(rd.value->body(p), lo, hi);
                lowest = std::min(lowest, lo.y);
            }
        }
        float speed = 0.0f;
        for (size_t p = 0; p < rd.value->partCount(); ++p) speed = std::max(speed, world.getLinearVelocity(rd.value->body(p)).length());
        VX_REPORT("auto ragdoll %s: %zu parts from %zu bones, lowest=%.4f m, speed after 6 s=%.4f m/s", name, rd.value->partCount(),
                  ch.skeleton.bones.size(), lowest, speed);
        CAPTURE(name);
        CHECK(rd.value->partCount() >= 5);
        CHECK_FALSE(warned);
        CHECK(lowest > -0.03f);
        CHECK(speed < 0.05f);
    }
}
