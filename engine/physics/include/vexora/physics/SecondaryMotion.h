// Spring-based secondary motion for bone chains (hair, tails, ears, straps, cloth flaps).
//
// This is an APPROXIMATION, not a full physical simulation: each chain is a string of
// particles at the bone joints, integrated with Verlet, pulled towards the animated pose
// (stiffness), with exact segment lengths, angular limits and sphere/capsule collision.
// It does not use the rigid-body engine. Chains are independent: no chain-vs-chain collision.
//
// Data flow per fixed step (see docs/physics/02-architecture.md §3–4):
//   animation -> animated ModelPose + model-to-world -> step() -> apply() -> final LocalPose
// The animated pose is never modified; apply() writes a separate result.
#pragma once

#include <memory>
#include <vector>

#include "vexora/physics/Skeleton.h"

namespace vexora::physics {

struct ChainCapsule {
    Vec3 a, b;           // Segment end points in world space.
    float radius = 0.1f;
};

struct ChainPlane {
    Vec3 normal{0.0f, 1.0f, 0.0f};  // Unit normal; particles stay on the positive side.
    float offset = 0.0f;            // dot(normal, p) = offset on the plane.
};

struct SpringChainSettings {
    std::vector<int> bones;   // Root-to-tip; each bone must be a child of the previous one.
    // The first bone stays attached to its animated parent; the chain hangs from its head.
    float tipLength = 0.1f;   // Extra virtual segment after the last bone (m), so it can swing too.
    float stiffness = 0.15f;  // 0..1: spring pulling bones back to the animated pose (1 = rigid at 60 Hz).
    float damping = 0.1f;     // 0..1: fraction of velocity removed (per 1/60 s).
    float inertia = 1.0f;     // 0..1: 1 = world-space inertia (lags behind the character), 0 = follows rigidly.
    float gravityScale = 1.0f;
    float maxAngle = 1.2f;    // Max deviation from the animated direction per segment (radians).
    float radius = 0.02f;     // Particle radius for collision (m).
    float weight = 1.0f;      // 0..1 blend between animated (0) and simulated (1) result.
};

class SecondaryMotion {
public:
    explicit SecondaryMotion(const Skeleton& skeleton, Vec3 gravity = {0.0f, -9.81f, 0.0f});
    ~SecondaryMotion();
    SecondaryMotion(const SecondaryMotion&) = delete;
    SecondaryMotion& operator=(const SecondaryMotion&) = delete;

    Result<int> addChain(const SpringChainSettings& settings);
    SpringChainSettings& chainSettings(int chain);

    // World colliders for all chains (e.g. the character's body, the floor), replaced every call.
    void setColliders(std::vector<ChainCapsule> capsules, std::vector<ChainPlane> planes = {});

    // Snaps every chain to the animated pose with zero velocity (teleports, cuts, loops).
    void reset(const ModelPose& animated, const Transform& modelToWorld);

    // One fixed step. `animated` is the animation's model pose this step.
    void step(double dt, const ModelPose& animated, const Transform& modelToWorld);

    // Writes chain bone rotations into `pose` (a copy of the animated local pose), blended
    // by each chain's weight. Bones outside chains are left untouched.
    void apply(const ModelPose& animated, const Transform& modelToWorld, LocalPose& pose) const;

    // World-space particle positions of a chain (joints + tip), for debug drawing.
    const std::vector<Vec3>& particles(int chain) const;
    // Number of times a chain was reset because of non-finite state (should stay 0).
    int recoveries() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vexora::physics
