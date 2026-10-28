#include "vexora/physics/SecondaryMotion.h"

#include <algorithm>
#include <cmath>

namespace vexora::physics {

namespace {

constexpr float kMaxParticleSpeed = 60.0f;  // m/s; caps energy injected by a teleporting root.

Vec3 closestOnSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const float len2 = dot(ab, ab);
    const float t = len2 > 0.0f ? std::clamp(dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
    return a + ab * t;
}

}  // namespace

struct SecondaryMotion::Impl {
    struct Chain {
        SpringChainSettings settings;
        Vec3 tipDirLocal{0.0f, 1.0f, 0.0f};  // Tip direction in the last bone's frame.
        std::vector<Vec3> pos, prev, target;
        bool initialised = false;
    };

    const Skeleton& skeleton;
    Vec3 gravity;
    std::vector<Chain> chains;
    std::vector<ChainCapsule> capsules;
    std::vector<ChainPlane> planes;
    int recoveries = 0;

    Impl(const Skeleton& s, Vec3 g) : skeleton(s), gravity(g) {}

    // Animated joint + tip positions in world space.
    void targets(Chain& c, const ModelPose& animated, const Transform& toWorld) const {
        const size_t n = c.settings.bones.size();
        c.target.resize(n + 1);
        for (size_t i = 0; i < n; ++i) c.target[i] = (toWorld * animated[size_t(c.settings.bones[i])]).position;
        const Transform last = toWorld * animated[size_t(c.settings.bones.back())];
        c.target[n] = last.apply(c.tipDirLocal * c.settings.tipLength);
    }

    void snap(Chain& c) {
        c.pos = c.target;
        c.prev = c.target;
        c.initialised = true;
    }
};

SecondaryMotion::SecondaryMotion(const Skeleton& skeleton, Vec3 gravity) : impl_(std::make_unique<Impl>(skeleton, gravity)) {}
SecondaryMotion::~SecondaryMotion() = default;

Result<int> SecondaryMotion::addChain(const SpringChainSettings& s) {
    using R = Result<int>;
    const Skeleton& sk = impl_->skeleton;
    if (s.bones.empty()) return R::failure(ErrorCode::InvalidArgument, "Spring chain needs at least one bone");
    for (size_t i = 0; i < s.bones.size(); ++i) {
        const int b = s.bones[i];
        if (b < 0 || b >= int(sk.bones.size())) return R::failure(ErrorCode::InvalidArgument, "Spring chain bone index out of range");
        if (i > 0 && sk.bones[size_t(b)].parent != s.bones[i - 1])
            return R::failure(ErrorCode::InvalidArgument, "Spring chain bones must form a parent-to-child chain");
    }
    auto in01 = [](float v) { return v >= 0.0f && v <= 1.0f; };
    if (!in01(s.stiffness) || !in01(s.damping) || !in01(s.inertia) || !in01(s.weight))
        return R::failure(ErrorCode::InvalidArgument, "Stiffness, damping, inertia and weight must be in [0, 1]");
    if (!(s.tipLength >= 0.0f) || !(s.radius >= 0.0f) || !(s.maxAngle >= 0.0f) || !std::isfinite(s.gravityScale))
        return R::failure(ErrorCode::InvalidArgument, "Tip length, radius and max angle must be >= 0");

    Impl::Chain c;
    c.settings = s;
    const Bone& last = sk.bones[size_t(s.bones.back())];
    if (last.restLocal.position.length() > 1e-6f)
        c.tipDirLocal = normalized(last.restLocal.rotation.conjugate().rotate(last.restLocal.position));
    impl_->chains.push_back(std::move(c));
    return R::success(int(impl_->chains.size() - 1));
}

SpringChainSettings& SecondaryMotion::chainSettings(int chain) { return impl_->chains[size_t(chain)].settings; }

void SecondaryMotion::setColliders(std::vector<ChainCapsule> capsules, std::vector<ChainPlane> planes) {
    impl_->capsules = std::move(capsules);
    impl_->planes = std::move(planes);
}

void SecondaryMotion::reset(const ModelPose& animated, const Transform& toWorld) {
    for (Impl::Chain& c : impl_->chains) {
        impl_->targets(c, animated, toWorld);
        impl_->snap(c);
    }
}

void SecondaryMotion::step(double dtD, const ModelPose& animated, const Transform& toWorld) {
    const float dt = float(dtD);
    if (!(dt > 0.0f)) return;
    for (Impl::Chain& c : impl_->chains) {
        const SpringChainSettings& s = c.settings;
        impl_->targets(c, animated, toWorld);
        if (!c.initialised) {
            impl_->snap(c);
            continue;
        }
        // Stiffness is a spring acceleration (like gravity it scales with dt^2), defined so that
        // at 60 Hz a value of 1 snaps fully to the animation. Clamped to 1 for stability at large
        // steps. Damping removes a fraction of velocity per 1/60 s. Both make the settled shape
        // and decay independent of the step size.
        const float frames = dt * 60.0f;
        const float k = std::min(1.0f, s.stiffness * frames * frames);
        const float keep = std::pow(1.0f - s.damping, frames);
        const Vec3 rootDelta = c.target[0] - c.pos[0];
        const Vec3 g = impl_->gravity * (s.gravityScale * dt * dt);
        const float maxStep = kMaxParticleSpeed * dt;
        const size_t n = c.pos.size();

        c.prev[0] = c.pos[0];
        c.pos[0] = c.target[0];  // Root particle is attached to the animated parent.
        for (size_t i = 1; i < n; ++i) {
            const Vec3 carry = rootDelta * (1.0f - s.inertia);  // Non-inertial part follows the root.
            Vec3 velocity = (c.pos[i] - c.prev[i]) * keep;
            c.prev[i] = c.pos[i] + carry;
            Vec3 p = c.pos[i] + carry + velocity + g;
            p = p + (c.target[i] - p) * k;
            const Vec3 moved = p - c.prev[i];
            if (moved.length() > maxStep) p = c.prev[i] + normalized(moved) * maxStep;
            c.pos[i] = p;
        }
        // Constraints parent-first: angle limit, collision, then exact length.
        for (size_t i = 1; i < n; ++i) {
            const Vec3 animatedSeg = c.target[i] - c.target[i - 1];
            const float length = animatedSeg.length();
            Vec3 dir = c.pos[i] - c.pos[i - 1];
            if (length < 1e-6f || dir.length() < 1e-6f) {
                c.pos[i] = c.pos[i - 1] + animatedSeg;
                continue;
            }
            const Vec3 a = normalized(animatedSeg);
            dir = normalized(dir);
            const float angle = std::acos(std::clamp(dot(a, dir), -1.0f, 1.0f));
            if (angle > s.maxAngle) {
                Vec3 axis = cross(a, dir);
                if (axis.length() < 1e-6f) axis = cross(a, std::fabs(a.x) < 0.9f ? Vec3{1, 0, 0} : Vec3{0, 1, 0});
                dir = Quat::fromAxisAngle(normalized(axis), s.maxAngle).rotate(a);
            }
            Vec3 p = c.pos[i - 1] + dir * length;
            for (const ChainCapsule& cap : impl_->capsules) {
                const Vec3 q = closestOnSegment(p, cap.a, cap.b);
                const Vec3 d = p - q;
                const float minDist = cap.radius + s.radius;
                if (d.length() < minDist) p = q + (d.length() > 1e-6f ? normalized(d) : Vec3{0, 1, 0}) * minDist;
            }
            for (const ChainPlane& pl : impl_->planes) {
                const float dist = dot(pl.normal, p) - pl.offset - s.radius;
                if (dist < 0.0f) p = p - pl.normal * dist;
            }
            c.pos[i] = c.pos[i - 1] + normalized(p - c.pos[i - 1]) * length;
        }
        bool finite = true;
        for (const Vec3& p : c.pos) finite &= p.isFinite();
        if (!finite) {
            impl_->snap(c);
            ++impl_->recoveries;
        }
    }
}

void SecondaryMotion::apply(const ModelPose& animated, const Transform& toWorld, LocalPose& pose) const {
    const Skeleton& sk = impl_->skeleton;
    for (const Impl::Chain& c : impl_->chains) {
        if (!c.initialised) continue;
        const std::vector<int>& bones = c.settings.bones;
        const int parentOfRoot = sk.bones[size_t(bones[0])].parent;
        Transform parentWorld = parentOfRoot < 0 ? toWorld : toWorld * animated[size_t(parentOfRoot)];
        for (size_t j = 0; j < bones.size(); ++j) {
            const Transform animWorld = toWorld * animated[size_t(bones[j])];
            const Vec3 a = c.target[j + 1] - c.target[j];
            const Vec3 s = c.pos[j + 1] - c.pos[j];
            Transform simWorld{c.pos[j], animWorld.rotation};
            if (a.length() > 1e-6f && s.length() > 1e-6f)
                simWorld.rotation = (Quat::fromTo(normalized(a), normalized(s)) * animWorld.rotation).normalized();
            const Quat simLocal = (parentWorld.rotation.conjugate() * simWorld.rotation).normalized();
            Transform& out = pose[size_t(bones[j])];
            const float w = std::clamp(c.settings.weight, 0.0f, 1.0f);
            out.rotation = w >= 1.0f ? simLocal : slerp(out.rotation, simLocal, w);
            // The next bone's parent is this bone as it will actually be posed (after blending).
            parentWorld = parentWorld * Transform{out.position, out.rotation};
        }
    }
}

const std::vector<Vec3>& SecondaryMotion::particles(int chain) const { return impl_->chains[size_t(chain)].pos; }
int SecondaryMotion::recoveries() const { return impl_->recoveries; }

}  // namespace vexora::physics
