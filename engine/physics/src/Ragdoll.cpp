#include "vexora/physics/Ragdoll.h"

#include <algorithm>
#include <cmath>

namespace vexora::physics {

struct Ragdoll::Impl {
    struct Part {
        RagdollPart settings;
        BodyHandle body;
        ConstraintHandle joint;        // Invalid for the root part.
        int parent = -1;               // Parent part index.
        Transform boneToBody;          // body = bone * boneToBody (constant).
        Vec3 restAxisInParent;         // Capsule axis expressed in the parent body frame at creation.
        float halfHeight = 0.0f;
    };
    PhysicsWorld* world = nullptr;
    Skeleton skeleton;  // Own copy: callers need not keep theirs alive.
    RagdollSettings settings;
    std::vector<Part> parts;
    std::vector<int> partOfBone;
    BoneDrive mode = BoneDrive::Simulated;

    Transform bodyTarget(const Part& p, const ModelPose& pose, const Transform& toWorld) const {
        return toWorld * pose[size_t(p.settings.bone)] * p.boneToBody;
    }
};

namespace {

constexpr Vec3 kCapsuleAxis{0.0f, 1.0f, 0.0f};

// Unit vector perpendicular to `v`.
Vec3 perpendicular(const Vec3& v) {
    const Vec3 c = cross(v, std::fabs(v.y) < 0.9f ? Vec3{0, 1, 0} : Vec3{1, 0, 0});
    return normalized(c);
}

}  // namespace

Ragdoll::Ragdoll() : impl_(std::make_unique<Impl>()) {}

Ragdoll::~Ragdoll() {
    if (!impl_->world) return;
    for (Impl::Part& p : impl_->parts) {
        if (p.joint.isValid()) impl_->world->destroyConstraint(p.joint);
        if (p.body.isValid()) impl_->world->destroyBody(p.body);
    }
}

Result<std::unique_ptr<Ragdoll>> Ragdoll::create(PhysicsWorld& world, const Skeleton& sk, const RagdollSettings& settings,
                                                 const ModelPose& pose, const Transform& toWorld) {
    using R = Result<std::unique_ptr<Ragdoll>>;
    Status valid = sk.validate();
    if (!valid.ok()) return R::failure(valid.error, valid.message);
    if (pose.size() != sk.bones.size()) return R::failure(ErrorCode::InvalidArgument, "Pose and skeleton bone counts differ");
    if (settings.parts.empty()) return R::failure(ErrorCode::InvalidArgument, "Ragdoll needs at least one part");

    std::unique_ptr<Ragdoll> rd(new Ragdoll());
    Impl& m = *rd->impl_;
    m.world = &world;
    m.skeleton = sk;
    m.settings = settings;
    m.partOfBone.assign(sk.bones.size(), -1);
    for (size_t i = 0; i < settings.parts.size(); ++i) {
        const RagdollPart& p = settings.parts[i];
        if (p.bone < 0 || p.bone >= int(sk.bones.size()) || m.partOfBone[size_t(p.bone)] >= 0)
            return R::failure(ErrorCode::InvalidArgument, "Ragdoll part bone is out of range or used twice");
        if (p.childBone >= int(sk.bones.size())) return R::failure(ErrorCode::InvalidArgument, "Ragdoll part child bone out of range");
        if (!(p.radius > 0.0f) || !(p.mass > 0.0f)) return R::failure(ErrorCode::InvalidArgument, "Ragdoll parts need radius > 0 and mass > 0");
        m.partOfBone[size_t(p.bone)] = int(i);
    }

    const uint32_t group = world.createCollisionGroup(uint32_t(settings.parts.size()));
    for (size_t i = 0; i < settings.parts.size(); ++i) {
        const RagdollPart& ps = settings.parts[i];
        Impl::Part part;
        part.settings = ps;
        for (int b = sk.bones[size_t(ps.bone)].parent; b >= 0 && part.parent < 0; b = sk.bones[size_t(b)].parent)
            part.parent = m.partOfBone[size_t(b)];
        if (part.parent >= int(i)) return R::failure(ErrorCode::InvalidArgument, "Ragdoll parts must be listed parents first");

        const Transform boneWorld = toWorld * pose[size_t(ps.bone)];
        Vec3 tail;
        if (ps.childBone >= 0) {
            tail = (toWorld * pose[size_t(ps.childBone)]).position;
        } else {
            const Bone& b = sk.bones[size_t(ps.bone)];
            const Vec3 dirLocal = b.restLocal.position.length() > 1e-6f
                                      ? normalized(b.restLocal.rotation.conjugate().rotate(b.restLocal.position))
                                      : kCapsuleAxis;
            tail = boneWorld.apply(dirLocal * ps.length);
        }
        const Vec3 head = boneWorld.position;
        const float segment = (tail - head).length();
        if (segment < 1e-4f) return R::failure(ErrorCode::InvalidArgument, "Ragdoll part '" + sk.bones[size_t(ps.bone)].name + "' has zero length");
        const Vec3 dir = normalized(tail - head);
        const Transform bodyWorld{(head + tail) * 0.5f, Quat::fromTo(kCapsuleAxis, dir)};
        part.boneToBody = boneWorld.inverse() * bodyWorld;

        BodyDesc d;
        d.name = sk.bones[size_t(ps.bone)].name;
        part.halfHeight = std::max(0.01f, segment * 0.5f - ps.radius * 0.5f);
        d.shape = ShapeDesc::capsule(part.halfHeight, ps.radius);
        d.transform = bodyWorld;
        d.material = settings.material;
        d.massOverride = ps.mass;
        d.layer = settings.layer;
        d.collisionGroup = group;
        d.subGroup = uint32_t(i);
        d.angularDamping = 0.3f;  // Limbs settle instead of spinning; reduces jitter.
        Result<BodyHandle> body = world.createBody(d);
        if (!body.ok()) return R::failure(body.error, body.message);
        part.body = body.value;

        if (part.parent >= 0) {
            const Impl::Part& parent = m.parts[size_t(part.parent)];
            world.setSubGroupCollision(group, uint32_t(i), uint32_t(part.parent), false);  // Neighbours overlap at joints.
            ConstraintDesc c;
            c.type = ConstraintType::SwingTwist;
            c.bodyA = part.body;
            c.bodyB = parent.body;
            c.anchor = head;
            c.axis = dir;
            c.normal = perpendicular(dir);
            c.swingLimit = ps.swingLimit;
            c.twistMin = ps.twistMin;
            c.twistMax = ps.twistMax;
            c.motorFrequency = settings.motorFrequency;
            c.motorDamping = settings.motorDamping;
            c.motorMaxForce = settings.motorMaxTorque;
            c.breakForce = settings.jointBreakForce;
            Result<ConstraintHandle> joint = world.createConstraint(c);
            if (!joint.ok()) return R::failure(joint.error, joint.message);
            part.joint = joint.value;
            const Transform parentBody = world.getTransform(parent.body);
            part.restAxisInParent = parentBody.rotation.conjugate().rotate(dir);
        }
        m.parts.push_back(part);
    }
    return R::success(std::move(rd));
}

void Ragdoll::setDrive(BoneDrive mode) {
    Impl& m = *impl_;
    for (Impl::Part& p : m.parts) {
        m.world->setMotionType(p.body, mode == BoneDrive::Animated ? MotionType::Kinematic : MotionType::Dynamic);
        if (p.joint.isValid() && mode != BoneDrive::Powered) m.world->setSwingTwistTarget(p.joint, false, {});
    }
    m.mode = mode;
}

BoneDrive Ragdoll::drive() const { return impl_->mode; }

void Ragdoll::drive(const ModelPose& animated, const Transform& toWorld) {
    Impl& m = *impl_;
    if (m.mode == BoneDrive::Simulated) return;
    for (Impl::Part& p : m.parts) {
        const Transform target = m.bodyTarget(p, animated, toWorld);
        if (m.mode == BoneDrive::Animated) {
            m.world->setKinematicTarget(p.body, target);
        } else if (p.joint.isValid()) {
            const Transform parentTarget = m.bodyTarget(m.parts[size_t(p.parent)], animated, toWorld);
            m.world->setSwingTwistTarget(p.joint, true, (parentTarget.rotation.conjugate() * target.rotation).normalized());
        }
    }
}

ModelPose Ragdoll::physicsPose(const ModelPose& animated, const Transform& toWorld) const {
    const Impl& m = *impl_;
    const Transform toModel = toWorld.inverse();
    const LocalPose animLocal = toLocal(m.skeleton, animated);
    ModelPose out(animated.size());
    for (size_t b = 0; b < animated.size(); ++b) {
        const int part = m.partOfBone[b];
        if (part >= 0) {
            const Impl::Part& p = m.parts[size_t(part)];
            out[b] = toModel * m.world->getTransform(p.body) * p.boneToBody.inverse();
        } else {
            const int parent = m.skeleton.bones[b].parent;
            out[b] = parent < 0 ? animated[b] : out[size_t(parent)] * animLocal[b];
        }
    }
    return out;
}

void Ragdoll::teleport(const ModelPose& pose, const Transform& toWorld) {
    for (Impl::Part& p : impl_->parts) impl_->world->teleport(p.body, impl_->bodyTarget(p, pose, toWorld));
}

size_t Ragdoll::partCount() const { return impl_->parts.size(); }
BodyHandle Ragdoll::body(size_t part) const { return impl_->parts[part].body; }
int Ragdoll::partForBone(int bone) const { return bone >= 0 && size_t(bone) < impl_->partOfBone.size() ? impl_->partOfBone[size_t(bone)] : -1; }
int Ragdoll::parentPart(size_t part) const { return impl_->parts[part].parent; }
float Ragdoll::swingLimit(size_t part) const { return impl_->parts[part].settings.swingLimit; }
float Ragdoll::partRadius(size_t part) const { return impl_->parts[part].settings.radius; }
float Ragdoll::partHalfHeight(size_t part) const { return impl_->parts[part].halfHeight; }

float Ragdoll::jointSwing(size_t part) const {
    const Impl::Part& p = impl_->parts[part];
    if (p.parent < 0) return 0.0f;
    const Transform child = impl_->world->getTransform(p.body);
    const Transform parent = impl_->world->getTransform(impl_->parts[size_t(p.parent)].body);
    const Vec3 axisNow = parent.rotation.conjugate().rotate(child.rotation.rotate(kCapsuleAxis));
    return std::acos(std::clamp(dot(normalized(axisNow), normalized(p.restAxisInParent)), -1.0f, 1.0f));
}

DemoHumanoid makeDemoHumanoid() {
    DemoHumanoid h;
    auto add = [&](const char* name, int parent, Vec3 offset) {
        h.skeleton.bones.push_back({name, parent, {offset, {}}});
        return int(h.skeleton.bones.size() - 1);
    };
    const int pelvis = add("pelvis", -1, {0, 0.95f, 0});
    const int spine = add("spine", pelvis, {0, 0.15f, 0});
    const int chest = add("chest", spine, {0, 0.2f, 0});
    const int head = add("head", chest, {0, 0.25f, 0});
    const int upperArmL = add("upperArm.L", chest, {0.18f, 0.15f, 0});
    const int lowerArmL = add("lowerArm.L", upperArmL, {0.28f, 0, 0});
    const int handL = add("hand.L", lowerArmL, {0.25f, 0, 0});
    const int upperArmR = add("upperArm.R", chest, {-0.18f, 0.15f, 0});
    const int lowerArmR = add("lowerArm.R", upperArmR, {-0.28f, 0, 0});
    const int handR = add("hand.R", lowerArmR, {-0.25f, 0, 0});
    const int thighL = add("thigh.L", pelvis, {0.1f, -0.05f, 0});
    const int shinL = add("shin.L", thighL, {0, -0.42f, 0});
    const int footL = add("foot.L", shinL, {0, -0.42f, 0});
    const int thighR = add("thigh.R", pelvis, {-0.1f, -0.05f, 0});
    const int shinR = add("shin.R", thighR, {0, -0.42f, 0});
    const int footR = add("foot.R", shinR, {0, -0.42f, 0});
    // Limits are rough human ranges. Elbows/knees use a narrow cone, an approximation of a hinge.
    auto part = [](int bone, int child, float radius, float mass, float swing, float twist, float length = 0.2f) {
        RagdollPart p;
        p.bone = bone;
        p.childBone = child;
        p.radius = radius;
        p.mass = mass;
        p.swingLimit = swing;
        p.twistMin = -twist;
        p.twistMax = twist;
        p.length = length;
        return p;
    };
    h.ragdoll.parts = {
        part(pelvis, spine, 0.12f, 10.0f, 0, 0),
        part(spine, chest, 0.11f, 8.0f, 0.4f, 0.3f),
        part(chest, head, 0.13f, 12.0f, 0.3f, 0.3f),
        part(head, -1, 0.1f, 5.0f, 0.6f, 0.6f, 0.22f),
        part(upperArmL, lowerArmL, 0.05f, 2.5f, 1.4f, 0.8f),
        part(lowerArmL, handL, 0.045f, 1.5f, 1.2f, 0.3f),
        part(upperArmR, lowerArmR, 0.05f, 2.5f, 1.4f, 0.8f),
        part(lowerArmR, handR, 0.045f, 1.5f, 1.2f, 0.3f),
        part(thighL, shinL, 0.07f, 7.0f, 1.0f, 0.4f),
        part(shinL, footL, 0.055f, 4.0f, 1.2f, 0.1f),
        part(thighR, shinR, 0.07f, 7.0f, 1.0f, 0.4f),
        part(shinR, footR, 0.055f, 4.0f, 1.2f, 0.1f),
    };
    return h;
}

}  // namespace vexora::physics
