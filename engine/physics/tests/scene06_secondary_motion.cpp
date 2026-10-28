// Test Scenes 6 and 8 — Spring-based secondary motion and bone-chain simulation.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "vexora/physics/SecondaryMotion.h"

using namespace vexora::physics;

#define VX_REPORT(...) std::printf("  [measure] " __VA_ARGS__), std::printf("\n")

namespace {

// A body bone with an 8-segment tail pointing backwards (-Z), 0.15 m per segment.
Skeleton tailSkeleton(int segments = 8) {
    Skeleton s;
    s.bones.push_back({"body", -1, {{0, 1, 0}, {}}});
    for (int i = 0; i < segments; ++i)
        s.bones.push_back({"tail" + std::to_string(i), i, {{0, 0, i == 0 ? -0.2f : -0.15f}, {}}});
    return s;
}

SpringChainSettings tailChain(const Skeleton& s) {
    SpringChainSettings c;
    for (size_t i = 1; i < s.bones.size(); ++i) c.bones.push_back(int(i));
    c.tipLength = 0.15f;
    c.stiffness = 0.05f;
    c.damping = 0.08f;
    return c;
}

struct Rig {
    Skeleton skeleton = tailSkeleton();
    LocalPose animLocal = restPose(skeleton);
    SecondaryMotion motion{skeleton};
    int chain = -1;
    explicit Rig(SpringChainSettings c) {
        auto r = motion.addChain(c);
        REQUIRE(r.ok());
        chain = r.value;
    }
    explicit Rig() : Rig(tailChain(tailSkeleton())) {}
    ModelPose model() const { return toModel(skeleton, animLocal); }
    void step(const Transform& world, double dt = 1.0 / 60.0) { motion.step(dt, model(), world); }
    const std::vector<Vec3>& p() const { return motion.particles(chain); }
};

float maxSegmentLengthError(const Rig& rig, const Transform& world) {
    const ModelPose m = rig.model();
    float err = 0.0f;
    const auto& p = rig.p();
    for (size_t i = 1; i + 1 < p.size(); ++i) {
        const float animated = ((world * m[i + 1]).position - (world * m[i]).position).length();
        err = std::max(err, std::abs((p[i] - p[i - 1]).length() - animated));
    }
    return err;
}

}  // namespace

TEST_CASE("Scene06: weight 0 leaves the animation untouched; stiffness 1 follows it exactly") {
    Rig rig;
    rig.motion.chainSettings(rig.chain).weight = 0.0f;
    const Transform world{};
    rig.motion.reset(rig.model(), world);
    for (int i = 0; i < 120; ++i) rig.step(world);
    LocalPose out = rig.animLocal;
    rig.motion.apply(rig.model(), world, out);
    for (size_t i = 0; i < out.size(); ++i) CHECK(angleBetween(out[i].rotation, rig.animLocal[i].rotation) < 1e-6f);

    SpringChainSettings rigid = tailChain(rig.skeleton);
    rigid.stiffness = 1.0f;
    rigid.inertia = 0.0f;
    Rig stiff(rigid);
    stiff.motion.reset(stiff.model(), world);
    float worst = 0.0f;
    for (int i = 0; i < 120; ++i) {
        const Transform moving{{std::sin(i * 0.1f), 0, 0}, Quat::fromAxisAngle({0, 1, 0}, i * 0.02f)};
        stiff.step(moving);
        LocalPose res = stiff.animLocal;
        stiff.motion.apply(stiff.model(), moving, res);
        for (size_t b = 0; b < res.size(); ++b) worst = std::max(worst, angleBetween(res[b].rotation, stiff.animLocal[b].rotation));
    }
    VX_REPORT("stiffness=1 max rotation deviation from animation: %.2e rad", worst);
    CHECK(worst < 2e-3f);
}

TEST_CASE("Scene06: tail droops under gravity, keeps exact segment lengths, respects the angle limit") {
    auto droop = [](float stiffness, float& worstLength, float& worstAngle) {
        SpringChainSettings c = tailChain(tailSkeleton());
        c.stiffness = stiffness;
        c.maxAngle = 0.6f;
        Rig rig(c);
        const Transform world{};
        rig.motion.reset(rig.model(), world);
        for (int i = 0; i < 600; ++i) {
            rig.step(world);
            worstLength = std::max(worstLength, maxSegmentLengthError(rig, world));
            const ModelPose m = rig.model();
            const auto& p = rig.p();
            for (size_t s = 1; s + 1 < p.size(); ++s) {
                const Vec3 a = normalized((world * m[s + 1]).position - (world * m[s]).position);
                const Vec3 d = normalized(p[s] - p[s - 1]);
                worstAngle = std::max(worstAngle, std::acos(std::clamp(dot(a, d), -1.0f, 1.0f)));
            }
        }
        CHECK(rig.motion.recoveries() == 0);
        return 1.0f - rig.p().back().y;  // Root height is 1 m; the rest pose is horizontal.
    };
    float worstLength = 0.0f, worstAngle = 0.0f;
    const float stiff = droop(0.05f, worstLength, worstAngle);
    const float soft = droop(0.005f, worstLength, worstAngle);
    const float limp = droop(0.0f, worstLength, worstAngle);
    VX_REPORT("tip droop: stiffness 0.05 -> %.3f m, 0.005 -> %.3f m, 0 -> %.3f m; max length error=%.2e m, "
              "max segment angle=%.3f rad (limit 0.6)", stiff, soft, limp, worstLength, worstAngle);
    CHECK(stiff > 0.02f);
    CHECK(soft > stiff);  // Softer chains sag more.
    CHECK(limp > soft);
    CHECK(worstLength < 1e-4f);
    CHECK(worstAngle < 0.6f + 1e-3f);
}

TEST_CASE("Scene06: swinging decays after the root stops (no energy build-up) and settles") {
    Rig rig;
    rig.motion.reset(rig.model(), {});
    // Root sways left-right for 2 s, then stops.
    for (int i = 0; i < 120; ++i) rig.step({{0.3f * std::sin(i / 60.0f * 6.0f), 0, 0}, {}});
    const Transform still{{0.3f * std::sin(2.0f * 6.0f), 0, 0}, {}};
    std::vector<float> peaks;
    float prevX = rig.p().back().x, prevVel = 0.0f, maxSpeed = 0.0f;
    Vec3 settled{};
    for (int i = 0; i < 600; ++i) {
        rig.step(still);
        const float x = rig.p().back().x, vel = x - prevX;
        if (prevVel > 0.0f && vel <= 0.0f) peaks.push_back(std::abs(x - still.position.x));
        maxSpeed = std::max(maxSpeed, std::abs(vel) * 60.0f);
        prevX = x;
        prevVel = vel;
        if (i == 599) settled = rig.p().back();
    }
    bool decaying = true;
    for (size_t i = 1; i < peaks.size(); ++i) decaying &= peaks[i] <= peaks[i - 1] + 1e-4f;
    // Settled: after 10 s the tip barely moves.
    rig.step(still);
    const float drift = (rig.p().back() - settled).length();
    VX_REPORT("after stop: %zu swing peaks, first=%.3f m last=%.4f m, decaying=%d, final drift/step=%.2e m",
              peaks.size(), peaks.empty() ? 0.f : peaks.front(), peaks.empty() ? 0.f : peaks.back(), int(decaying), drift);
    CHECK(decaying);
    CHECK(drift < 1e-4f);
}

TEST_CASE("Scene06: collision capsule is never penetrated") {
    SpringChainSettings c = tailChain(tailSkeleton());
    c.radius = 0.03f;
    c.stiffness = 0.02f;
    Rig rig(c);
    const ChainCapsule leg{{-1, 0.7f, -0.6f}, {1, 0.7f, -0.6f}, 0.12f};  // Bar under the tail.
    rig.motion.setColliders({leg});
    rig.motion.reset(rig.model(), {});
    float closest = 1e9f;
    for (int i = 0; i < 300; ++i) {
        rig.step({});
        for (size_t k = 1; k < rig.p().size(); ++k) {
            const Vec3 p = rig.p()[k];
            closest = std::min(closest, (Vec3{p.x, p.y - 0.7f, p.z + 0.6f} - Vec3{p.x, 0, 0}).length());
        }
    }
    VX_REPORT("capsule: closest particle distance=%.4f m (min allowed %.3f)", closest, 0.15f);
    CHECK(closest >= 0.15f - 1e-3f);
}

TEST_CASE("Scene06: reset is exact and results do not depend much on the time step") {
    auto runFor = [](double dt, double seconds) {
        Rig rig;
        rig.motion.reset(rig.model(), {});
        for (int i = 0, n = int(seconds / dt + 0.5); i < n; ++i)
            rig.step({{0.3f * std::sin(float(i * dt) * 4.0f), 0, 0}, {}}, dt);
        return rig.p();
    };
    const auto a = runFor(1.0 / 60.0, 3.0), b = runFor(1.0 / 60.0, 3.0);
    CHECK(std::memcmp(a.data(), b.data(), a.size() * sizeof(Vec3)) == 0);

    // Steady state (root still) at 60 Hz vs 120 Hz: same settled shape.
    auto settle = [](double dt) {
        Rig rig;
        rig.motion.reset(rig.model(), {});
        for (int i = 0, n = int(8.0 / dt); i < n; ++i) rig.step({}, dt);
        return rig.p().back();
    };
    const float diff = (settle(1.0 / 60.0) - settle(1.0 / 120.0)).length();
    VX_REPORT("settled tip 60 Hz vs 120 Hz differs by %.4f m", diff);
    CHECK(diff < 0.005f);
}

TEST_CASE("Scene06: a 100 m teleport without reset stays stable") {
    Rig rig;
    rig.motion.reset(rig.model(), {});
    for (int i = 0; i < 30; ++i) rig.step({});
    float worstLength = 0.0f;
    const Transform far{{100, 0, 0}, {}};
    for (int i = 0; i < 300; ++i) {
        rig.step(far);
        worstLength = std::max(worstLength, maxSegmentLengthError(rig, far));
    }
    for (const Vec3& p : rig.p()) CHECK(p.isFinite());
    CHECK(rig.motion.recoveries() == 0);
    CHECK(worstLength < 1e-3f);
    CHECK(std::abs(rig.p().front().x - 100.0f) < 1e-3f);  // Still attached to the root.
}

TEST_CASE("Scene08: invalid chains are rejected") {
    Skeleton s = tailSkeleton();
    SecondaryMotion m(s);
    SpringChainSettings c;
    CHECK_FALSE(m.addChain(c).ok());
    c.bones = {1, 3};  // Not parent-child.
    CHECK_FALSE(m.addChain(c).ok());
    c.bones = {1, 2};
    c.stiffness = 2.0f;
    CHECK_FALSE(m.addChain(c).ok());
    c.stiffness = 0.5f;
    CHECK(m.addChain(c).ok());
}
