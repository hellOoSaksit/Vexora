// Ragdoll: one capsule body per selected bone, joined by SwingTwist constraints with limits.
// Built only on the PhysicsWorld API (bodies and constraints are simulated by Jolt; the pose
// mapping, drive modes and blending are Vexora's).
//
// Animation Pose vs Physics Pose (docs/physics/02-architecture.md §3):
//  - The animation pose is input only; the ragdoll never writes into it.
//  - physicsPose() returns a separate pose built from the bodies.
//  - Each bone has exactly one writer per step, chosen by the drive mode:
//      Animated  -> bodies are Kinematic, moved to the animation (they still push things).
//      Simulated -> bodies are Dynamic and limp (classic ragdoll).
//      Powered   -> bodies are Dynamic, joint motors pull them towards the animation.
//  - Mixing happens only in blendPoses(), never by two systems writing the same transform.
// Order per fixed step:  drive(animated) -> world.step() -> physicsPose() -> blendPoses().
#pragma once

#include <memory>
#include <vector>

#include "vexora/physics/PhysicsWorld.h"
#include "vexora/physics/Skeleton.h"

namespace vexora::physics {

struct RagdollPart {
    int bone = 0;
    int childBone = -1;      // Capsule spans bone head -> childBone head; -1: `length` along the bone.
    float length = 0.2f;     // Used when childBone is -1 (e.g. the head).
    float radius = 0.06f;
    float mass = 3.0f;       // kg
    float swingLimit = 0.6f; // Joint to the parent part (radians).
    float twistMin = -0.3f, twistMax = 0.3f;
};

struct RagdollSettings {
    std::vector<RagdollPart> parts;   // A part's parent is the nearest ancestor bone that has a part.
    PhysicsMaterial material = makeMaterial(MaterialPreset::FleshLike);
    uint8_t layer = 0;
    float motorFrequency = 6.0f;      // Powered mode joint motor stiffness (Hz).
    float motorDamping = 1.0f;
    float motorMaxTorque = 400.0f;    // N*m per joint.
    float jointBreakForce = 0.0f;     // > 0: joints tear apart above this force (N).
};

enum class BoneDrive { Animated, Simulated, Powered };

class Ragdoll {
public:
    static Result<std::unique_ptr<Ragdoll>> create(PhysicsWorld& world, const Skeleton& skeleton, const RagdollSettings& settings,
                                                   const ModelPose& pose, const Transform& modelToWorld);
    ~Ragdoll();  // Removes its bodies and constraints from the world.
    Ragdoll(const Ragdoll&) = delete;
    Ragdoll& operator=(const Ragdoll&) = delete;

    void setDrive(BoneDrive drive);
    BoneDrive drive() const;

    // Before world.step(): feeds the animation (kinematic targets or motor targets).
    void drive(const ModelPose& animated, const Transform& modelToWorld);
    // After world.step(): model-space pose from the bodies. Bones without a part follow their
    // animated local transform relative to their (physics-driven) parent.
    ModelPose physicsPose(const ModelPose& animated, const Transform& modelToWorld) const;
    // Places all bodies at a pose with zero velocity (e.g. before switching to Animated).
    void teleport(const ModelPose& pose, const Transform& modelToWorld);

    size_t partCount() const;
    BodyHandle body(size_t part) const;
    int partForBone(int bone) const;
    int parentPart(size_t part) const;  // -1 for the root part.
    // Current swing of a part's joint away from its rest direction (radians), for checks/UI.
    float jointSwing(size_t part) const;
    float swingLimit(size_t part) const;
    float partRadius(size_t part) const;
    float partHalfHeight(size_t part) const;  // Capsule cylinder half length (axis = body local +Y).

private:
    Ragdoll();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// A 16-bone humanoid (1.8 m, standing in T-pose at the origin) with a 12-part ragdoll, for
// demos and test scenes until Vexora has real character assets.
struct DemoHumanoid {
    Skeleton skeleton;
    RagdollSettings ragdoll;
};
DemoHumanoid makeDemoHumanoid();

// Builds ragdoll parts for any skeleton (imported characters): one capsule per bone whose
// longest child segment is at least `minBoneLength` (m), radius = 22% of that length (capped),
// mass from capsule volume at `density`, generic joint limits. Bones without a part follow
// their parent. This is a starting point; hand-tuned settings give better results.
RagdollSettings makeAutoRagdoll(const Skeleton& skeleton, const ModelPose& restModel, float minBoneLength,
                                float density = 1000.0f);

}  // namespace vexora::physics
