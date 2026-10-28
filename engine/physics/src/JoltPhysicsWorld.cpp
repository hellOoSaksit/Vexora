// Jolt Physics backend for vexora::physics::PhysicsWorld.
// This is the only translation unit that includes Jolt headers.

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <mutex>
#include <tuple>
#include <utility>

#include "vexora/physics/PhysicsWorld.h"

namespace vexora::physics {
namespace {

// ---------------------------------------------------------------------------------------
// Process-wide Jolt registration, reference counted so several worlds can coexist.
std::mutex gJoltInitMutex;
int gJoltRefCount = 0;

void acquireJolt() {
    std::lock_guard<std::mutex> lock(gJoltInitMutex);
    if (gJoltRefCount++ == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
}

void releaseJolt() {
    std::lock_guard<std::mutex> lock(gJoltInitMutex);
    if (--gJoltRefCount == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

JPH::Vec3 toJ(const Vec3& v) { return {v.x, v.y, v.z}; }
JPH::RVec3 toJR(const Vec3& v) { return {v.x, v.y, v.z}; }
JPH::Quat toJ(const Quat& q) { return JPH::Quat(q.x, q.y, q.z, q.w).Normalized(); }
Vec3 fromJ(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
#ifdef JPH_DOUBLE_PRECISION
Vec3 fromJ(JPH::RVec3Arg v) { return {float(v.GetX()), float(v.GetY()), float(v.GetZ())}; }
#endif
Quat fromJ(JPH::QuatArg q) { return {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}; }

// ---------------------------------------------------------------------------------------
// Collision filtering.
// ObjectLayer = user layer (low 4 bits) | kMovingBit. Static bodies never collide with each
// other; any other pair collides when both layers' masks accept each other.
constexpr JPH::ObjectLayer kMovingBit = 0x10;
constexpr JPH::BroadPhaseLayer kBPStatic(0);
constexpr JPH::BroadPhaseLayer kBPMoving(1);

JPH::ObjectLayer makeObjectLayer(uint8_t userLayer, MotionType motion) {
    return JPH::ObjectLayer(userLayer & 0x0F) | (motion == MotionType::Static ? 0 : kMovingBit);
}

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return (layer & kMovingBit) ? kBPMoving : kBPStatic;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == kBPMoving ? "Moving" : "Static";
    }
#endif
};

class ObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
        return (layer & kMovingBit) ? true : bp == kBPMoving;
    }
};

class LayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    LayerPairFilter() { masks.fill(0xFFFF); }
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        if (!(a & kMovingBit) && !(b & kMovingBit)) return false;
        const int la = a & 0x0F, lb = b & 0x0F;
        return ((masks[la] >> lb) & 1u) && ((masks[lb] >> la) & 1u);
    }
    std::array<uint16_t, kMaxCollisionLayers> masks{};
};

// ---------------------------------------------------------------------------------------
// Contact collection. Jolt calls these from worker threads, so writes are locked and the
// list is sorted after the step to make event order independent of thread scheduling.
struct RawContact {
    ContactEventType type;
    JPH::BodyID a, b;
    Vec3 point, normal;
    float penetration;
    bool trigger;
};

class ContactCollector final : public JPH::ContactListener {
public:
    void OnContactAdded(const JPH::Body& b1, const JPH::Body& b2, const JPH::ContactManifold& m,
                        JPH::ContactSettings&) override {
        record(ContactEventType::Begin, b1, b2, m);
    }
    void OnContactPersisted(const JPH::Body& b1, const JPH::Body& b2, const JPH::ContactManifold& m,
                            JPH::ContactSettings&) override {
        record(ContactEventType::Persist, b1, b2, m);
    }
    void OnContactRemoved(const JPH::SubShapeIDPair& pair) override {
        std::lock_guard<std::mutex> lock(mutex);
        contacts.push_back({ContactEventType::End, pair.GetBody1ID(), pair.GetBody2ID(), {}, {}, 0.0f, false});
    }

    std::mutex mutex;
    std::vector<RawContact> contacts;

private:
    void record(ContactEventType type, const JPH::Body& b1, const JPH::Body& b2, const JPH::ContactManifold& m) {
        Vec3 p = m.mRelativeContactPointsOn1.empty() ? Vec3{} : fromJ(m.GetWorldSpaceContactPointOn1(0));
        std::lock_guard<std::mutex> lock(mutex);
        contacts.push_back({type, b1.GetID(), b2.GetID(), p, fromJ(m.mWorldSpaceNormal), m.mPenetrationDepth,
                            b1.IsSensor() || b2.IsSensor()});
    }
};

struct BodyRecord {
    JPH::BodyID id;
    uint32_t generation = 0;
    bool alive = false;
    MotionType motion = MotionType::Static;
    Transform previous;  // Transform before the last step, for render interpolation.
    uint64_t userData = 0;
};

struct ConstraintRecord {
    JPH::Ref<JPH::TwoBodyConstraint> constraint;
    ConstraintType type = ConstraintType::Fixed;
    uint32_t generation = 0;
    bool alive = false;
    float breakForce = 0.0f;
    float breakTorque = 0.0f;
};

void fnv(uint64_t& h, const void* data, size_t n) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 1099511628211ull;
    }
}

}  // namespace

// =========================================================================================
struct PhysicsWorld::Impl {
    WorldSettings settings;
    QualitySettings quality;
    BroadPhaseLayers bpLayers;
    ObjectVsBroadPhase objVsBp;
    LayerPairFilter pairFilter;
    ContactCollector contactCollector;
    std::unique_ptr<JPH::TempAllocator> tempAllocator;
    std::unique_ptr<JPH::JobSystem> jobSystem;
    JPH::PhysicsSystem system;

    std::vector<BodyRecord> bodies;
    std::vector<uint32_t> freeBodySlots;
    std::vector<ConstraintRecord> constraints;
    std::vector<uint32_t> freeConstraintSlots;

    uint64_t stepIndex = 0;
    uint64_t structureVersion = 0;
    std::vector<ContactEvent> contactEvents;
    std::vector<ConstraintBrokenEvent> brokenEvents;
    std::vector<SolverWarning> warnings;
    StepStats stats;

    JPH::BodyInterface& bi() { return system.GetBodyInterfaceNoLock(); }
    const JPH::BodyInterface& bi() const { return system.GetBodyInterfaceNoLock(); }

    const BodyRecord* find(BodyHandle h) const {
        if (h.index >= bodies.size()) return nullptr;
        const BodyRecord& r = bodies[h.index];
        return (r.alive && r.generation == h.generation) ? &r : nullptr;
    }
    BodyRecord* find(BodyHandle h) { return const_cast<BodyRecord*>(std::as_const(*this).find(h)); }

    const ConstraintRecord* find(ConstraintHandle h) const {
        if (h.index >= constraints.size()) return nullptr;
        const ConstraintRecord& r = constraints[h.index];
        return (r.alive && r.generation == h.generation) ? &r : nullptr;
    }

    BodyHandle handleFor(const JPH::BodyID& id) const {
        if (id.IsInvalid()) return {};
        const auto slot = static_cast<uint32_t>(bi().GetUserData(id));
        if (slot >= bodies.size() || bodies[slot].id != id) return {};
        return {slot, bodies[slot].generation};
    }

    Transform currentTransform(const BodyRecord& r) const {
        JPH::RVec3 p;
        JPH::Quat q;
        bi().GetPositionAndRotation(r.id, p, q);
        return {fromJ(p), fromJ(q)};
    }

    Result<JPH::ShapeRefC> buildShape(const ShapeDesc& d, float density) const;
    void applyQuality();
    void collectEvents();
    void checkBreakables();
    void checkHealth();
};

Result<JPH::ShapeRefC> PhysicsWorld::Impl::buildShape(const ShapeDesc& d, float density) const {
    using R = Result<JPH::ShapeRefC>;
    JPH::ShapeSettings::ShapeResult result;
    switch (d.type) {
        case ShapeType::Box: {
            const float minHalf = std::min({d.halfExtents.x, d.halfExtents.y, d.halfExtents.z});
            JPH::BoxShapeSettings s(toJ(d.halfExtents), std::min(JPH::cDefaultConvexRadius, minHalf * 0.5f));
            s.mDensity = density;
            result = s.Create();
            break;
        }
        case ShapeType::Sphere: {
            JPH::SphereShapeSettings s(d.radius);
            s.mDensity = density;
            result = s.Create();
            break;
        }
        case ShapeType::Capsule: {
            JPH::CapsuleShapeSettings s(d.halfHeight, d.radius);
            s.mDensity = density;
            result = s.Create();
            break;
        }
        case ShapeType::ConvexHull: {
            JPH::Array<JPH::Vec3> pts;
            pts.reserve(d.points.size());
            for (const Vec3& p : d.points) pts.push_back(toJ(p));
            JPH::ConvexHullShapeSettings s(pts);
            s.mDensity = density;
            result = s.Create();
            break;
        }
        case ShapeType::TriangleMesh: {
            JPH::VertexList verts;
            verts.reserve(d.points.size());
            for (const Vec3& p : d.points) verts.push_back(JPH::Float3(p.x, p.y, p.z));
            JPH::IndexedTriangleList tris;
            tris.reserve(d.indices.size() / 3);
            for (size_t i = 0; i + 2 < d.indices.size(); i += 3)
                tris.push_back(JPH::IndexedTriangle(d.indices[i], d.indices[i + 1], d.indices[i + 2]));
            JPH::MeshShapeSettings s(std::move(verts), std::move(tris));
            result = s.Create();
            break;
        }
        case ShapeType::Compound: {
            JPH::StaticCompoundShapeSettings s;
            for (const CompoundChild& c : d.children) {
                R child = buildShape(*c.shape, density);
                if (!child.ok()) return child;
                s.AddShape(toJ(c.localTransform.position), toJ(c.localTransform.rotation), child.value);
            }
            result = s.Create();
            break;
        }
    }
    if (result.HasError()) return R::failure(ErrorCode::ShapeCreationFailed, result.GetError().c_str());
    return R::success(result.Get());
}

void PhysicsWorld::Impl::applyQuality() {
    JPH::PhysicsSettings ps = system.GetPhysicsSettings();
    ps.mNumVelocitySteps = static_cast<JPH::uint>(quality.velocityIterations);
    ps.mNumPositionSteps = static_cast<JPH::uint>(quality.positionIterations);
    system.SetPhysicsSettings(ps);
}

void PhysicsWorld::Impl::collectEvents() {
    std::vector<RawContact> raw;
    {
        std::lock_guard<std::mutex> lock(contactCollector.mutex);
        raw.swap(contactCollector.contacts);
    }
    contactEvents.clear();
    contactEvents.reserve(raw.size());
    for (const RawContact& c : raw) {
        ContactEvent e;
        e.type = c.type;
        e.bodyA = handleFor(c.a);
        e.bodyB = handleFor(c.b);
        e.point = c.point;
        e.normal = c.normal;
        e.penetration = c.penetration;
        e.isTrigger = c.trigger;
        contactEvents.push_back(e);
    }
    // End events can arrive once per sub-shape pair; deterministic order, then de-duplicate.
    auto key = [](const ContactEvent& e) {
        return std::make_tuple(e.bodyA.index, e.bodyB.index, int(e.type), e.point.x, e.point.y, e.point.z);
    };
    std::sort(contactEvents.begin(), contactEvents.end(),
              [&](const ContactEvent& l, const ContactEvent& r) { return key(l) < key(r); });
    contactEvents.erase(std::unique(contactEvents.begin(), contactEvents.end(),
                                    [](const ContactEvent& l, const ContactEvent& r) {
                                        return l.type == ContactEventType::End && r.type == ContactEventType::End &&
                                               l.bodyA == r.bodyA && l.bodyB == r.bodyB;
                                    }),
                        contactEvents.end());
}

void PhysicsWorld::Impl::checkBreakables() {
    // Jolt reports accumulated impulses (lambda) of the last sub-step; divide by its dt for force.
    const float subDt = static_cast<float>(quality.fixedDt) / static_cast<float>(quality.collisionSteps);
    for (uint32_t i = 0; i < constraints.size(); ++i) {
        ConstraintRecord& r = constraints[i];
        if (!r.alive || !r.constraint->GetEnabled() || (r.breakForce <= 0.0f && r.breakTorque <= 0.0f)) continue;
        float linear = 0.0f, angular = 0.0f;
        switch (r.type) {
            case ConstraintType::Fixed: {
                auto* c = static_cast<JPH::FixedConstraint*>(r.constraint.GetPtr());
                linear = c->GetTotalLambdaPosition().Length();
                angular = c->GetTotalLambdaRotation().Length();
                break;
            }
            case ConstraintType::Point:
                linear = static_cast<JPH::PointConstraint*>(r.constraint.GetPtr())->GetTotalLambdaPosition().Length();
                break;
            case ConstraintType::Hinge: {
                auto* c = static_cast<JPH::HingeConstraint*>(r.constraint.GetPtr());
                linear = c->GetTotalLambdaPosition().Length();
                const JPH::Vector<2> rot = c->GetTotalLambdaRotation();
                angular = std::sqrt(rot[0] * rot[0] + rot[1] * rot[1]);
                break;
            }
            case ConstraintType::Slider: {
                auto* c = static_cast<JPH::SliderConstraint*>(r.constraint.GetPtr());
                const JPH::Vector<2> pos = c->GetTotalLambdaPosition();
                linear = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
                angular = c->GetTotalLambdaRotation().Length();
                break;
            }
            case ConstraintType::Distance:
                linear = std::abs(static_cast<JPH::DistanceConstraint*>(r.constraint.GetPtr())->GetTotalLambdaPosition());
                break;
        }
        const float force = linear / subDt, torque = angular / subDt;
        if ((r.breakForce > 0.0f && force > r.breakForce) || (r.breakTorque > 0.0f && torque > r.breakTorque)) {
            r.constraint->SetEnabled(false);
            brokenEvents.push_back({ConstraintHandle{i, r.generation}, force, torque});
            // Wake both bodies so they react to the release immediately.
            for (const JPH::Body* body : {r.constraint->GetBody1(), r.constraint->GetBody2()})
                if (!body->GetID().IsInvalid()) bi().ActivateBody(body->GetID());
        }
    }
}

void PhysicsWorld::Impl::checkHealth() {
    for (uint32_t i = 0; i < bodies.size(); ++i) {
        const BodyRecord& r = bodies[i];
        if (!r.alive || r.motion != MotionType::Dynamic || !bi().IsActive(r.id)) continue;
        const Transform t = currentTransform(r);
        const Vec3 v = fromJ(bi().GetLinearVelocity(r.id));
        const BodyHandle h{i, r.generation};
        if (!t.position.isFinite() || !t.rotation.isFinite() || !v.isFinite()) {
            // Freeze the body instead of letting NaNs spread through contacts and constraints.
            bi().SetPositionAndRotation(r.id, toJR(r.previous.position), toJ(r.previous.rotation),
                                        JPH::EActivation::DontActivate);
            bi().SetLinearAndAngularVelocity(r.id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
            bi().DeactivateBody(r.id);
            warnings.push_back({WarningType::NonFiniteState, h, "Non-finite state; body restored to last valid pose and frozen"});
        } else if (v.length() > settings.explosionSpeedWarning) {
            warnings.push_back({WarningType::ExcessiveVelocity, h,
                                "Speed " + std::to_string(v.length()) + " m/s exceeds warning threshold; possible solver explosion"});
        }
    }
}

// =========================================================================================
PhysicsWorld::PhysicsWorld(const WorldSettings& settings) : impl_(std::make_unique<Impl>()) {
    acquireJolt();
    Impl& m = *impl_;
    m.settings = settings;
    m.quality = settings.qualityOverride ? *settings.qualityOverride : settingsFor(settings.profile);
    // Fast arena for typical scenes; falls back to malloc instead of aborting on large ones.
    m.tempAllocator = std::make_unique<JPH::TempAllocatorImplWithMallocFallback>(32 * 1024 * 1024);
    if (settings.workerThreads > 0)
        m.jobSystem = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
                                                                 settings.workerThreads);
    else
        m.jobSystem = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    m.system.Init(settings.maxBodies, 0, settings.maxBodyPairs, settings.maxContactConstraints, m.bpLayers,
                  m.objVsBp, m.pairFilter);
    m.system.SetGravity(toJ(settings.gravity));
    m.system.SetContactListener(&m.contactCollector);
    m.applyQuality();
}

PhysicsWorld::~PhysicsWorld() {
    Impl& m = *impl_;
    for (ConstraintRecord& c : m.constraints)
        if (c.alive) m.system.RemoveConstraint(c.constraint);
    for (BodyRecord& b : m.bodies)
        if (b.alive) {
            m.bi().RemoveBody(b.id);
            m.bi().DestroyBody(b.id);
        }
    m.constraints.clear();
    impl_.reset();
    releaseJolt();
}

void PhysicsWorld::setLayerCollision(uint8_t a, uint8_t b, bool collide) {
    if (a >= kMaxCollisionLayers || b >= kMaxCollisionLayers) return;
    auto& masks = impl_->pairFilter.masks;
    if (collide) {
        masks[a] |= uint16_t(1u << b);
        masks[b] |= uint16_t(1u << a);
    } else {
        masks[a] &= uint16_t(~(1u << b));
        masks[b] &= uint16_t(~(1u << a));
    }
}

bool PhysicsWorld::layersCollide(uint8_t a, uint8_t b) const {
    if (a >= kMaxCollisionLayers || b >= kMaxCollisionLayers) return false;
    return impl_->pairFilter.ShouldCollide(JPH::ObjectLayer(a | kMovingBit), JPH::ObjectLayer(b | kMovingBit));
}

Result<BodyHandle> PhysicsWorld::createBody(const BodyDesc& desc) {
    using R = Result<BodyHandle>;
    Impl& m = *impl_;
    if (desc.layer >= kMaxCollisionLayers) return R::failure(ErrorCode::InvalidArgument, "Collision layer out of range");
    if (!desc.transform.position.isFinite() || !desc.transform.rotation.isFinite() || !desc.linearVelocity.isFinite() ||
        !desc.angularVelocity.isFinite())
        return R::failure(ErrorCode::InvalidArgument, "Body transform or velocity is not finite");
    if (!(desc.material.density > 0.0f) || !std::isfinite(desc.material.friction) || desc.material.friction < 0.0f ||
        !(desc.material.restitution >= 0.0f && desc.material.restitution <= 1.0f))
        return R::failure(ErrorCode::InvalidArgument, "Material needs density > 0, friction >= 0, restitution in [0,1]");
    if (desc.massOverride < 0.0f || !std::isfinite(desc.massOverride))
        return R::failure(ErrorCode::InvalidArgument, "Mass override must be >= 0");
    Status valid = validateShape(desc.shape, desc.motion);
    if (!valid.ok()) return R::failure(valid.error, valid.message);

    Result<JPH::ShapeRefC> shape = m.buildShape(desc.shape, desc.material.density);
    if (!shape.ok()) return R::failure(shape.error, shape.message);

    const JPH::EMotionType motion = desc.motion == MotionType::Static    ? JPH::EMotionType::Static
                                    : desc.motion == MotionType::Kinematic ? JPH::EMotionType::Kinematic
                                                                           : JPH::EMotionType::Dynamic;
    JPH::BodyCreationSettings bcs(shape.value, toJR(desc.transform.position), toJ(desc.transform.rotation), motion,
                                  makeObjectLayer(desc.layer, desc.motion));
    bcs.mFriction = desc.material.friction;
    bcs.mRestitution = desc.material.restitution;
    bcs.mLinearDamping = desc.linearDamping;
    bcs.mAngularDamping = desc.angularDamping;
    bcs.mGravityFactor = desc.gravityFactor;
    bcs.mAllowSleeping = desc.allowSleeping;
    bcs.mIsSensor = desc.isTrigger;
    bcs.mLinearVelocity = toJ(desc.linearVelocity);
    bcs.mAngularVelocity = toJ(desc.angularVelocity);
    bcs.mMotionQuality = (desc.continuousCollision || m.quality.forceContinuousCollision)
                             ? JPH::EMotionQuality::LinearCast
                             : JPH::EMotionQuality::Discrete;
    if (desc.massOverride > 0.0f && desc.motion == MotionType::Dynamic) {
        bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        bcs.mMassPropertiesOverride.mMass = desc.massOverride;
    }

    uint32_t slot;
    if (!m.freeBodySlots.empty()) {
        slot = m.freeBodySlots.back();
        m.freeBodySlots.pop_back();
    } else {
        slot = static_cast<uint32_t>(m.bodies.size());
        m.bodies.emplace_back();
    }
    bcs.mUserData = slot;

    JPH::Body* body = m.bi().CreateBody(bcs);
    if (body == nullptr) {
        m.freeBodySlots.push_back(slot);
        return R::failure(ErrorCode::CapacityExceeded, "Physics world body capacity exceeded");
    }
    const bool activate = desc.motion != MotionType::Static;
    m.bi().AddBody(body->GetID(), activate ? JPH::EActivation::Activate : JPH::EActivation::DontActivate);

    BodyRecord& r = m.bodies[slot];
    r.id = body->GetID();
    r.generation += 1;
    r.alive = true;
    r.motion = desc.motion;
    r.userData = desc.userData;
    r.previous = desc.transform;
    ++m.structureVersion;
    return R::success({slot, r.generation});
}

Status PhysicsWorld::destroyBody(BodyHandle h) {
    Impl& m = *impl_;
    BodyRecord* r = m.find(h);
    if (!r) return {ErrorCode::InvalidHandle, "Unknown or destroyed body"};
    // Constraints referencing this body must go first; Jolt would otherwise keep dangling refs.
    for (uint32_t i = 0; i < m.constraints.size(); ++i) {
        ConstraintRecord& c = m.constraints[i];
        if (c.alive && (c.constraint->GetBody1()->GetID() == r->id || c.constraint->GetBody2()->GetID() == r->id))
            destroyConstraint({i, c.generation});
    }
    m.bi().RemoveBody(r->id);
    m.bi().DestroyBody(r->id);
    r->alive = false;
    r->id = JPH::BodyID();
    m.freeBodySlots.push_back(h.index);
    ++m.structureVersion;
    return {};
}

bool PhysicsWorld::isAlive(BodyHandle h) const { return impl_->find(h) != nullptr; }

uint32_t PhysicsWorld::bodyCount() const { return impl_->system.GetNumBodies(); }

Transform PhysicsWorld::getTransform(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    return r ? impl_->currentTransform(*r) : Transform{};
}

Transform PhysicsWorld::getInterpolatedTransform(BodyHandle h, float alpha) const {
    const BodyRecord* r = impl_->find(h);
    if (!r) return {};
    return interpolate(r->previous, impl_->currentTransform(*r), std::clamp(alpha, 0.0f, 1.0f));
}

Vec3 PhysicsWorld::getLinearVelocity(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    return r ? fromJ(impl_->bi().GetLinearVelocity(r->id)) : Vec3{};
}

Vec3 PhysicsWorld::getAngularVelocity(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    return r ? fromJ(impl_->bi().GetAngularVelocity(r->id)) : Vec3{};
}

Vec3 PhysicsWorld::getCenterOfMass(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    return r ? fromJ(impl_->bi().GetCenterOfMassPosition(r->id)) : Vec3{};
}

float PhysicsWorld::getMass(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    if (!r || r->motion != MotionType::Dynamic) return 0.0f;
    JPH::BodyLockRead lock(impl_->system.GetBodyLockInterfaceNoLock(), r->id);
    if (!lock.Succeeded()) return 0.0f;
    const float invMass = lock.GetBody().GetMotionProperties()->GetInverseMass();
    return invMass > 0.0f ? 1.0f / invMass : 0.0f;
}

bool PhysicsWorld::isSleeping(BodyHandle h) const {
    const BodyRecord* r = impl_->find(h);
    return r && !impl_->bi().IsActive(r->id);
}

namespace {
Status badHandle() { return {ErrorCode::InvalidHandle, "Unknown or destroyed body"}; }
Status badValue() { return {ErrorCode::InvalidArgument, "Value is not finite"}; }
}  // namespace

Status PhysicsWorld::setKinematicTarget(BodyHandle h, const Transform& target) {
    BodyRecord* r = impl_->find(h);
    if (!r) return badHandle();
    if (r->motion != MotionType::Kinematic) return {ErrorCode::InvalidArgument, "Body is not kinematic"};
    if (!target.position.isFinite() || !target.rotation.isFinite()) return badValue();
    impl_->bi().MoveKinematic(r->id, toJR(target.position), toJ(target.rotation), float(impl_->quality.fixedDt));
    return {};
}

Status PhysicsWorld::teleport(BodyHandle h, const Transform& t) {
    BodyRecord* r = impl_->find(h);
    if (!r) return badHandle();
    if (!t.position.isFinite() || !t.rotation.isFinite()) return badValue();
    impl_->bi().SetPositionAndRotation(r->id, toJR(t.position), toJ(t.rotation),
                                       r->motion == MotionType::Static ? JPH::EActivation::DontActivate
                                                                       : JPH::EActivation::Activate);
    if (r->motion != MotionType::Static)
        impl_->bi().SetLinearAndAngularVelocity(r->id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
    r->previous = t;  // No interpolation smear across a teleport.
    return {};
}

#define VX_DYNAMIC_BODY_OR_RETURN(h, ...)                                                           \
    BodyRecord* r = impl_->find(h);                                                                 \
    if (!r) return badHandle();                                                                     \
    if (r->motion != MotionType::Dynamic) return {ErrorCode::InvalidArgument, "Body is not dynamic"}; \
    for (const Vec3* v : {__VA_ARGS__})                                                             \
        if (!v->isFinite()) return badValue();

Status PhysicsWorld::setLinearVelocity(BodyHandle h, const Vec3& v) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &v);
    impl_->bi().SetLinearVelocity(r->id, toJ(v));
    impl_->bi().ActivateBody(r->id);
    return {};
}
Status PhysicsWorld::addForce(BodyHandle h, const Vec3& f) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &f);
    impl_->bi().AddForce(r->id, toJ(f));
    return {};
}
Status PhysicsWorld::addForceAtPoint(BodyHandle h, const Vec3& f, const Vec3& p) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &f, &p);
    impl_->bi().AddForce(r->id, toJ(f), toJR(p));
    return {};
}
Status PhysicsWorld::addTorque(BodyHandle h, const Vec3& t) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &t);
    impl_->bi().AddTorque(r->id, toJ(t));
    return {};
}
Status PhysicsWorld::addImpulse(BodyHandle h, const Vec3& i) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &i);
    impl_->bi().AddImpulse(r->id, toJ(i));
    impl_->bi().ActivateBody(r->id);
    return {};
}
Status PhysicsWorld::addImpulseAtPoint(BodyHandle h, const Vec3& i, const Vec3& p) {
    VX_DYNAMIC_BODY_OR_RETURN(h, &i, &p);
    impl_->bi().AddImpulse(r->id, toJ(i), toJR(p));
    impl_->bi().ActivateBody(r->id);
    return {};
}
#undef VX_DYNAMIC_BODY_OR_RETURN

Status PhysicsWorld::wake(BodyHandle h) {
    BodyRecord* r = impl_->find(h);
    if (!r) return badHandle();
    if (r->motion != MotionType::Static) impl_->bi().ActivateBody(r->id);
    return {};
}

// ---------------------------------------------------------------------------------------
Result<ConstraintHandle> PhysicsWorld::createConstraint(const ConstraintDesc& d) {
    using R = Result<ConstraintHandle>;
    Impl& m = *impl_;
    const BodyRecord* a = m.find(d.bodyA);
    if (!a) return R::failure(ErrorCode::InvalidHandle, "Constraint bodyA is invalid");
    const BodyRecord* b = d.bodyB.isValid() ? m.find(d.bodyB) : nullptr;
    if (d.bodyB.isValid() && !b) return R::failure(ErrorCode::InvalidHandle, "Constraint bodyB is invalid");
    if (b == a) return R::failure(ErrorCode::InvalidArgument, "Constraint needs two different bodies");
    if (a->motion != MotionType::Dynamic && (!b || b->motion != MotionType::Dynamic))
        return R::failure(ErrorCode::InvalidArgument, "At least one constrained body must be dynamic");
    if (!d.anchor.isFinite() || !d.anchorB.isFinite() || !d.axis.isFinite() || !d.normal.isFinite())
        return R::failure(ErrorCode::InvalidArgument, "Constraint anchor/axis is not finite");
    if (d.hasLimits && d.limitMin > d.limitMax)
        return R::failure(ErrorCode::InvalidArgument, "Constraint limitMin must be <= limitMax");

    JPH::Vec3 axis = toJ(d.axis), normal = toJ(d.normal);
    if (d.type == ConstraintType::Hinge || d.type == ConstraintType::Slider) {
        if (axis.Length() < 1e-6f || normal.Length() < 1e-6f || std::abs(axis.Normalized().Dot(normal.Normalized())) > 1e-3f)
            return R::failure(ErrorCode::InvalidArgument, "Constraint axis and normal must be non-zero and perpendicular");
        axis = axis.Normalized();
        normal = normal.Normalized();
    }
    if (d.type == ConstraintType::Hinge && d.hasLimits && (d.limitMin < -JPH::JPH_PI || d.limitMin > 0.0f ||
                                                           d.limitMax < 0.0f || d.limitMax > JPH::JPH_PI))
        return R::failure(ErrorCode::InvalidArgument, "Hinge limits must satisfy -pi <= min <= 0 <= max <= pi");

    JPH::Ref<JPH::TwoBodyConstraintSettings> settings;
    const JPH::RVec3 p = toJR(d.anchor);
    switch (d.type) {
        case ConstraintType::Fixed: {
            auto* s = new JPH::FixedConstraintSettings();
            s->mPoint1 = s->mPoint2 = p;
            settings = s;
            break;
        }
        case ConstraintType::Point: {
            auto* s = new JPH::PointConstraintSettings();
            s->mPoint1 = s->mPoint2 = p;
            settings = s;
            break;
        }
        case ConstraintType::Hinge: {
            auto* s = new JPH::HingeConstraintSettings();
            s->mPoint1 = s->mPoint2 = p;
            s->mHingeAxis1 = s->mHingeAxis2 = axis;
            s->mNormalAxis1 = s->mNormalAxis2 = normal;
            if (d.hasLimits) {
                s->mLimitsMin = d.limitMin;
                s->mLimitsMax = d.limitMax;
            }
            s->mMotorSettings.SetTorqueLimit(d.motorMaxForce);
            settings = s;
            break;
        }
        case ConstraintType::Slider: {
            auto* s = new JPH::SliderConstraintSettings();
            s->mPoint1 = s->mPoint2 = p;
            s->mSliderAxis1 = s->mSliderAxis2 = axis;
            s->mNormalAxis1 = s->mNormalAxis2 = normal;
            if (d.hasLimits) {
                s->mLimitsMin = d.limitMin;
                s->mLimitsMax = d.limitMax;
            }
            s->mMotorSettings.SetForceLimit(d.motorMaxForce);
            settings = s;
            break;
        }
        case ConstraintType::Distance: {
            auto* s = new JPH::DistanceConstraintSettings();
            s->mPoint1 = toJR(d.anchorB);  // Jolt body 1 = our bodyB (see below).
            s->mPoint2 = p;
            if (d.hasLimits) {
                s->mMinDistance = d.limitMin;
                s->mMaxDistance = d.limitMax;
            }
            s->mLimitsSpringSettings.mFrequency = d.springFrequency;
            s->mLimitsSpringSettings.mDamping = d.springDamping;
            settings = s;
            break;
        }
    }

    // Jolt measures angles/positions of body 2 relative to body 1, so the parent (bodyB, or the
    // world via an invalid ID -> Body::sFixedToWorld) is body 1 and the attached bodyA is body 2.
    // That makes limits, motors and getConstraintPosition() describe bodyA relative to bodyB.
    JPH::TwoBodyConstraint* c = m.bi().CreateConstraint(settings, b ? b->id : JPH::BodyID(), a->id);
    if (d.motorEnabled) {
        if (d.type == ConstraintType::Hinge) {
            auto* h = static_cast<JPH::HingeConstraint*>(c);
            h->SetMotorState(JPH::EMotorState::Velocity);
            h->SetTargetAngularVelocity(d.motorTargetVelocity);
        } else if (d.type == ConstraintType::Slider) {
            auto* s = static_cast<JPH::SliderConstraint*>(c);
            s->SetMotorState(JPH::EMotorState::Velocity);
            s->SetTargetVelocity(d.motorTargetVelocity);
        }
    }
    m.system.AddConstraint(c);
    m.bi().ActivateBody(a->id);
    if (b) m.bi().ActivateBody(b->id);

    uint32_t slot;
    if (!m.freeConstraintSlots.empty()) {
        slot = m.freeConstraintSlots.back();
        m.freeConstraintSlots.pop_back();
    } else {
        slot = static_cast<uint32_t>(m.constraints.size());
        m.constraints.emplace_back();
    }
    ConstraintRecord& r = m.constraints[slot];
    r.constraint = c;
    r.type = d.type;
    r.generation += 1;
    r.alive = true;
    r.breakForce = d.breakForce;
    r.breakTorque = d.breakTorque;
    ++m.structureVersion;
    return R::success({slot, r.generation});
}

Status PhysicsWorld::destroyConstraint(ConstraintHandle h) {
    Impl& m = *impl_;
    if (!m.find(h)) return {ErrorCode::InvalidHandle, "Unknown or destroyed constraint"};
    ConstraintRecord& r = m.constraints[h.index];
    m.system.RemoveConstraint(r.constraint);
    r.constraint = nullptr;
    r.alive = false;
    m.freeConstraintSlots.push_back(h.index);
    ++m.structureVersion;
    return {};
}

bool PhysicsWorld::isConstraintEnabled(ConstraintHandle h) const {
    const ConstraintRecord* r = impl_->find(h);
    return r && r->constraint->GetEnabled();
}

float PhysicsWorld::getConstraintPosition(ConstraintHandle h) const {
    const ConstraintRecord* r = impl_->find(h);
    if (!r) return 0.0f;
    if (r->type == ConstraintType::Hinge) return static_cast<const JPH::HingeConstraint*>(r->constraint.GetPtr())->GetCurrentAngle();
    if (r->type == ConstraintType::Slider)
        return static_cast<const JPH::SliderConstraint*>(r->constraint.GetPtr())->GetCurrentPosition();
    return 0.0f;
}

// ---------------------------------------------------------------------------------------
void PhysicsWorld::step() {
    Impl& m = *impl_;
    const auto start = std::chrono::steady_clock::now();
    m.brokenEvents.clear();
    m.warnings.clear();

    for (BodyRecord& r : m.bodies)
        if (r.alive && r.motion != MotionType::Static) r.previous = m.currentTransform(r);

    const JPH::EPhysicsUpdateError err =
        m.system.Update(float(m.quality.fixedDt), m.quality.collisionSteps, m.tempAllocator.get(), m.jobSystem.get());
    if (err != JPH::EPhysicsUpdateError::None) {
        std::string what;
        if ((err & JPH::EPhysicsUpdateError::ManifoldCacheFull) != JPH::EPhysicsUpdateError::None) what += " manifold-cache";
        if ((err & JPH::EPhysicsUpdateError::BodyPairCacheFull) != JPH::EPhysicsUpdateError::None) what += " body-pair-cache";
        if ((err & JPH::EPhysicsUpdateError::ContactConstraintsFull) != JPH::EPhysicsUpdateError::None) what += " contact-constraints";
        m.warnings.push_back({WarningType::EngineCapacity, {}, "Engine capacity exceeded:" + what + "; contacts were dropped"});
    }
    ++m.stepIndex;

    m.collectEvents();
    m.checkBreakables();
    m.checkHealth();

    m.stats.stepIndex = m.stepIndex;
    m.stats.stepMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    m.stats.totalBodies = m.system.GetNumBodies();
    m.stats.activeBodies = m.system.GetNumActiveBodies(JPH::EBodyType::RigidBody);
    m.stats.contactEvents = static_cast<uint32_t>(m.contactEvents.size());
}

double PhysicsWorld::fixedDt() const { return impl_->quality.fixedDt; }
uint64_t PhysicsWorld::stepIndex() const { return impl_->stepIndex; }
const QualitySettings& PhysicsWorld::quality() const { return impl_->quality; }
void PhysicsWorld::setGravity(const Vec3& g) {
    if (g.isFinite()) impl_->system.SetGravity(toJ(g));
}
const std::vector<ContactEvent>& PhysicsWorld::contactEvents() const { return impl_->contactEvents; }
const std::vector<ConstraintBrokenEvent>& PhysicsWorld::brokenConstraints() const { return impl_->brokenEvents; }
const std::vector<SolverWarning>& PhysicsWorld::warnings() const { return impl_->warnings; }
const StepStats& PhysicsWorld::lastStepStats() const { return impl_->stats; }

// ---------------------------------------------------------------------------------------
SimulationSnapshot PhysicsWorld::saveSnapshot() const {
    const Impl& m = *impl_;
    JPH::StateRecorderImpl recorder;
    m.system.SaveState(recorder);
    // Our own per-body interpolation state and constraint enable flags (breakage) travel too.
    for (const BodyRecord& r : m.bodies)
        if (r.alive) {
            recorder.Write(r.previous.position.x); recorder.Write(r.previous.position.y); recorder.Write(r.previous.position.z);
            recorder.Write(r.previous.rotation.x); recorder.Write(r.previous.rotation.y); recorder.Write(r.previous.rotation.z);
            recorder.Write(r.previous.rotation.w);
        }
    for (const ConstraintRecord& c : m.constraints)
        if (c.alive) recorder.Write(c.constraint->GetEnabled());
    return {recorder.GetData(), m.stepIndex, m.structureVersion};
}

Status PhysicsWorld::restoreSnapshot(const SimulationSnapshot& s) {
    Impl& m = *impl_;
    if (s.structureVersion != m.structureVersion)
        return {ErrorCode::StateMismatch,
                "Snapshot was taken with a different set of bodies/constraints; re-simulate from the initial state"};
    JPH::StateRecorderImpl recorder;
    recorder.WriteBytes(s.data.data(), s.data.size());
    recorder.Rewind();
    if (!m.system.RestoreState(recorder)) return {ErrorCode::StateMismatch, "Physics engine rejected the snapshot"};
    for (BodyRecord& r : m.bodies)
        if (r.alive) {
            recorder.Read(r.previous.position.x); recorder.Read(r.previous.position.y); recorder.Read(r.previous.position.z);
            recorder.Read(r.previous.rotation.x); recorder.Read(r.previous.rotation.y); recorder.Read(r.previous.rotation.z);
            recorder.Read(r.previous.rotation.w);
        }
    for (ConstraintRecord& c : m.constraints)
        if (c.alive) {
            bool enabled = true;
            recorder.Read(enabled);
            c.constraint->SetEnabled(enabled);
        }
    if (recorder.IsFailed()) return {ErrorCode::StateMismatch, "Snapshot data is truncated or corrupt"};
    m.stepIndex = s.stepIndex;
    m.contactEvents.clear();
    m.brokenEvents.clear();
    m.warnings.clear();
    return {};
}

uint64_t PhysicsWorld::stateHash() const {
    const Impl& m = *impl_;
    uint64_t h = 1469598103934665603ull;
    for (uint32_t i = 0; i < m.bodies.size(); ++i) {
        const BodyRecord& r = m.bodies[i];
        if (!r.alive) continue;
        const Transform t = m.currentTransform(r);
        const Vec3 v = fromJ(m.bi().GetLinearVelocity(r.id));
        const Vec3 w = fromJ(m.bi().GetAngularVelocity(r.id));
        fnv(h, &i, sizeof(i));
        fnv(h, &t.position, sizeof(t.position));
        fnv(h, &t.rotation, sizeof(t.rotation));
        fnv(h, &v, sizeof(v));
        fnv(h, &w, sizeof(w));
    }
    return h;
}

// ---------------------------------------------------------------------------------------
void PhysicsWorld::collectDebugDraw(DebugDrawList& out, uint32_t flags) const {
    const Impl& m = *impl_;
    constexpr uint32_t kSleeping = 0x808080FF, kActive = 0x40C040FF, kStatic = 0x4060C0FF, kTrigger = 0xC0C040FF;
    for (const BodyRecord& r : m.bodies) {
        if (!r.alive) continue;
        JPH::BodyLockRead lock(m.system.GetBodyLockInterfaceNoLock(), r.id);
        if (!lock.Succeeded()) continue;
        const JPH::Body& body = lock.GetBody();
        if (flags & DebugDrawBounds) {
            const JPH::AABox box = body.GetWorldSpaceBounds();
            const Vec3 lo = fromJ(box.mMin), hi = fromJ(box.mMax);
            const uint32_t c = body.IsSensor() ? kTrigger : body.IsStatic() ? kStatic : body.IsActive() ? kActive : kSleeping;
            const Vec3 v[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                               {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
            static constexpr int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                                             {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
            for (const auto& edge : e) out.lines.push_back({v[edge[0]], v[edge[1]], c});
        }
        if (body.IsStatic()) continue;
        const Vec3 com = fromJ(body.GetCenterOfMassPosition());
        if (flags & DebugDrawCenterOfMass) out.points.push_back({com, 0xFF00FFFF});
        if ((flags & DebugDrawVelocity) && body.IsActive())
            out.lines.push_back({com, com + fromJ(body.GetLinearVelocity()) * float(m.quality.fixedDt * 10.0), 0xFFFF00FF});
    }
    if (flags & DebugDrawContacts)
        for (const ContactEvent& c : m.contactEvents)
            if (c.type != ContactEventType::End) {
                out.points.push_back({c.point, 0xFF2020FF});
                out.lines.push_back({c.point, c.point + c.normal * 0.2f, 0xFF8080FF});
            }
    if (flags & DebugDrawConstraints)
        for (const ConstraintRecord& c : m.constraints)
            if (c.alive) {
                const uint32_t color = c.constraint->GetEnabled() ? 0x20E0E0FFu : 0xE02020FFu;
                const Vec3 child = fromJ(c.constraint->GetBody2()->GetCenterOfMassPosition());
                if (c.constraint->GetBody1()->GetID().IsInvalid())  // Attached to the world.
                    out.points.push_back({child, color});
                else
                    out.lines.push_back({fromJ(c.constraint->GetBody1()->GetCenterOfMassPosition()), child, color});
            }
}

}  // namespace vexora::physics
