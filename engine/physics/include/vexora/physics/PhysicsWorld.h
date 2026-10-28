// PhysicsWorld: the simulation state for one scene. It is owned by the scene/simulation
// layer, never by UI. UI reads state through this API and edits authored data elsewhere.
//
// Backend: Jolt Physics (see docs/physics/01-engine-evaluation.md). The backend is hidden
// behind this class (pimpl) so engine types never leak into the rest of Vexora, but only
// ONE backend exists and is tested. This is not a multi-engine abstraction.
//
// Threading: all methods must be called from the thread that owns the world. step() may
// use worker threads internally.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "vexora/physics/PhysicsMaterial.h"
#include "vexora/physics/QualityProfile.h"
#include "vexora/physics/Shapes.h"
#include "vexora/physics/Types.h"

namespace vexora::physics {

constexpr int kMaxCollisionLayers = 16;

// Which system writes a body's transform. Exactly one writer per body per step prevents
// animation and physics from overwriting each other without control.
//  - Static:    nobody after creation (moving it is an explicit teleport).
//  - Kinematic: the scene/animation writes targets via setKinematicTarget(); physics
//               derives velocity from them so contacts respond correctly.
//  - Dynamic:   physics writes; the scene reads via getTransform()/getInterpolatedTransform().
struct BodyDesc {
    std::string name;
    MotionType motion = MotionType::Dynamic;
    ShapeDesc shape;
    PhysicsMaterial material;
    Transform transform;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    float massOverride = 0.0f;   // > 0: use this mass (inertia scaled from shape); else density.
    float linearDamping = 0.05f;
    float angularDamping = 0.05f;
    float gravityFactor = 1.0f;
    bool continuousCollision = false;  // Linear-cast CCD against tunnelling for fast bodies.
    bool allowSleeping = true;
    bool isTrigger = false;            // Sensor: reports contacts, no collision response.
    uint8_t layer = 0;                 // Collision layer [0, kMaxCollisionLayers).
    uint64_t userData = 0;
};

struct WorldSettings {
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    QualityProfile profile = QualityProfile::InteractivePreview;
    std::optional<QualitySettings> qualityOverride;  // Custom settings instead of the profile's.
    int workerThreads = 0;  // 0 = single threaded (simplest to reason about); >0 = worker count.
    uint32_t maxBodies = 65536;
    uint32_t maxBodyPairs = 65536;
    uint32_t maxContactConstraints = 65536;
    // A dynamic body faster than this is reported as a likely solver explosion.
    float explosionSpeedWarning = 200.0f;
};

enum class ConstraintType { Fixed, Point, Hinge, Slider, Distance };

// Anchors and axes are given in world space at creation time.
// bodyA is the attached (child) body, bodyB its parent; bodyB may be invalid to attach bodyA to
// the static world. Limits, motors and getConstraintPosition() describe bodyA relative to bodyB.
struct ConstraintDesc {
    ConstraintType type = ConstraintType::Fixed;
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec3 anchor;                  // Fixed/Point/Hinge/Slider: shared pivot. Distance: point on A.
    Vec3 anchorB;                 // Distance: point on B.
    Vec3 axis{0.0f, 1.0f, 0.0f};  // Hinge rotation axis / Slider translation axis.
    Vec3 normal{1.0f, 0.0f, 0.0f};// Hinge/Slider: reference direction perpendicular to axis.
    bool hasLimits = false;
    float limitMin = 0.0f;        // Hinge: radians. Slider/Distance: metres.
    float limitMax = 0.0f;
    float springFrequency = 0.0f; // Distance: > 0 makes it a soft spring (Hz).
    float springDamping = 0.0f;   // Distance: damping ratio.
    bool motorEnabled = false;    // Hinge/Slider velocity motor.
    float motorTargetVelocity = 0.0f;
    float motorMaxForce = 1.0e4f; // N (slider) or N*m (hinge).
    float breakForce = 0.0f;      // > 0: break when the solver's constraint force exceeds it (N).
    float breakTorque = 0.0f;     // > 0: break on constraint torque (N*m). Fixed/Hinge/Slider.
};

enum class ContactEventType { Begin, Persist, End };

struct ContactEvent {
    ContactEventType type = ContactEventType::Begin;
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec3 point;          // World-space contact point (Begin/Persist only).
    Vec3 normal;         // From A to B.
    float penetration = 0.0f;
    bool isTrigger = false;
};

struct ConstraintBrokenEvent {
    ConstraintHandle constraint;
    float force = 0.0f;
    float torque = 0.0f;
};

enum class WarningType { NonFiniteState, ExcessiveVelocity, EngineCapacity };

struct SolverWarning {
    WarningType type;
    BodyHandle body;      // Invalid for world-level warnings.
    std::string message;
};

struct StepStats {
    uint64_t stepIndex = 0;
    double stepMilliseconds = 0.0;
    uint32_t activeBodies = 0;
    uint32_t totalBodies = 0;
    uint32_t contactEvents = 0;
};

enum DebugDrawFlags : uint32_t {
    DebugDrawBounds = 1u << 0,
    DebugDrawCenterOfMass = 1u << 1,
    DebugDrawVelocity = 1u << 2,
    DebugDrawContacts = 1u << 3,
    DebugDrawConstraints = 1u << 4,
    DebugDrawAll = 0xFFFFFFFFu,
};

// Renderer-agnostic debug geometry. A viewport draws this; physics never calls a renderer.
struct DebugLine {
    Vec3 from, to;
    uint32_t rgba;
};
struct DebugPoint {
    Vec3 position;
    uint32_t rgba;
};
struct DebugDrawList {
    std::vector<DebugLine> lines;
    std::vector<DebugPoint> points;
};

// Opaque copy of the dynamic simulation state (positions, velocities, sleep, contacts,
// constraint warm start). It is only valid for a world with the same set of bodies and
// constraints; restoreSnapshot() rejects it otherwise instead of corrupting state.
struct SimulationSnapshot {
    std::string data;
    uint64_t stepIndex = 0;
    uint64_t structureVersion = 0;
};

class PhysicsWorld {
public:
    explicit PhysicsWorld(const WorldSettings& settings = {});
    ~PhysicsWorld();
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // --- Collision filtering ------------------------------------------------------------
    // Layers a and b collide only if each one's mask contains the other. Default: all collide.
    void setLayerCollision(uint8_t a, uint8_t b, bool collide);
    bool layersCollide(uint8_t a, uint8_t b) const;

    // --- Body lifecycle ---------------------------------------------------------------
    Result<BodyHandle> createBody(const BodyDesc& desc);
    Status destroyBody(BodyHandle body);
    bool isAlive(BodyHandle body) const;
    uint32_t bodyCount() const;

    // --- State and control ------------------------------------------------------------
    Transform getTransform(BodyHandle body) const;
    // Blend between the transforms before and after the last step (alpha from FixedStepClock).
    Transform getInterpolatedTransform(BodyHandle body, float alpha) const;
    Vec3 getLinearVelocity(BodyHandle body) const;
    Vec3 getAngularVelocity(BodyHandle body) const;
    Vec3 getCenterOfMass(BodyHandle body) const;
    float getMass(BodyHandle body) const;
    bool isSleeping(BodyHandle body) const;

    Status setKinematicTarget(BodyHandle body, const Transform& target);  // Kinematic only.
    Status teleport(BodyHandle body, const Transform& transform);          // Resets velocity.
    Status setLinearVelocity(BodyHandle body, const Vec3& v);
    Status addForce(BodyHandle body, const Vec3& force);                 // Applied over next step.
    Status addForceAtPoint(BodyHandle body, const Vec3& force, const Vec3& worldPoint);
    Status addTorque(BodyHandle body, const Vec3& torque);
    Status addImpulse(BodyHandle body, const Vec3& impulse);             // Instant velocity change.
    Status addImpulseAtPoint(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint);
    Status wake(BodyHandle body);

    // --- Constraints ------------------------------------------------------------------
    Result<ConstraintHandle> createConstraint(const ConstraintDesc& desc);
    Status destroyConstraint(ConstraintHandle constraint);
    bool isConstraintEnabled(ConstraintHandle constraint) const;
    // Hinge: current angle (radians). Slider: current position (m). Others: 0.
    float getConstraintPosition(ConstraintHandle constraint) const;

    // --- Simulation -------------------------------------------------------------------
    // Advances exactly one fixed step of the active quality profile's fixedDt.
    void step();
    double fixedDt() const;
    uint64_t stepIndex() const;
    const QualitySettings& quality() const;
    void setGravity(const Vec3& g);

    // Events from the most recent step(), sorted deterministically. Cleared by the next step.
    const std::vector<ContactEvent>& contactEvents() const;
    const std::vector<ConstraintBrokenEvent>& brokenConstraints() const;
    const std::vector<SolverWarning>& warnings() const;
    const StepStats& lastStepStats() const;

    // --- Reproducibility --------------------------------------------------------------
    SimulationSnapshot saveSnapshot() const;
    Status restoreSnapshot(const SimulationSnapshot& snapshot);
    // FNV-1a over every live body's position, rotation and velocities, in handle order.
    // Equal hashes mean bit-identical state; used by determinism tests.
    uint64_t stateHash() const;

    // --- Debugging --------------------------------------------------------------------
    void collectDebugDraw(DebugDrawList& out, uint32_t flags = DebugDrawAll) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vexora::physics
