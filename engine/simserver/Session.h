// One loaded scene: owns the PhysicsWorld and ParticleSystem, builds them from resolved
// parameters, and serialises frames for the UI. All calls come from the server's main thread.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Schema.h"
#include "vexora/physics/ParticleSystem.h"
#include "vexora/physics/PhysicsWorld.h"
#include "vexora/physics/Ragdoll.h"
#include "vexora/physics/SecondaryMotion.h"

namespace vexora::sim {

class Session {
public:
    // Builds the scene. On failure returns false with a user-facing message; the session is
    // then empty (no half-built world).
    bool load(const std::string& sceneId, const json& params, const json& global, std::string& error);
    bool loaded() const { return world_ != nullptr; }

    // Advances one fixed step. Returns false (with message) on a device error.
    bool step(std::string& error);
    double fixedDt() const;
    double simTime() const { return simTime_; }

    // Runs a scene action (e.g. "shoot"). New bodies are reported by takeAddedBodies().
    bool action(const std::string& id, std::string& error);

    json describeScene() const;           // Static description: every body's shape, size, colour.
    json takeAddedBodies();               // Bodies created since the last call (actions).
    json frame(bool includeParticles);    // Dynamic state since the last step.

private:
    struct RenderBody {
        physics::BodyHandle handle;
        std::string shape;  // "box" | "sphere" | "capsule"
        physics::Vec3 size; // box: half extents; sphere: x=radius; capsule: x=radius, y=half height
        uint32_t color;
        bool dynamic;
        int virtualIndex = -1;  // >= 0: not a physics body; transform comes from virtualXf_.
    };

    // A character in the ragdoll scene and its drive state.
    struct Actor {
        std::unique_ptr<physics::Ragdoll> ragdoll;
        physics::Transform root;
        std::string state;          // "animated" | "ragdoll" | "powered" | "gettingup"
        double blendTime = 0.0;
        physics::LocalPose blendFrom;
    };

    physics::BodyHandle addBody(const physics::BodyDesc& desc, const std::string& shape, physics::Vec3 size, uint32_t color);
    void buildDrop(const json& p, const json& g);
    void buildTower(const json& p);
    void buildChain(const json& p);
    bool buildFountain(const json& p, const json& g, std::string& error);
    void addFloor();
    int addVirtual(const std::string& shape, physics::Vec3 size, uint32_t color);
    physics::Transform transformOf(const RenderBody& b) const;
    bool buildRagdoll(const json& p, std::string& error);
    void buildTail(const json& p);
    physics::LocalPose idlePose(double t) const;
    void preStep(double dt);
    void postStep(double dt);
    const char* characterState() const;

    std::unique_ptr<physics::PhysicsWorld> world_;
    std::unique_ptr<physics::ParticleSystem> particles_;
    std::vector<RenderBody> bodies_;
    size_t reportedBodies_ = 0;
    std::string sceneId_;
    json params_;
    double simTime_ = 0.0;
    uint32_t displayLimit_ = 20000;
    std::vector<physics::BodyHandle> chainLinks_;
    std::vector<physics::Transform> virtualXf_;

    // Ragdoll scene.
    physics::DemoHumanoid human_;
    std::vector<Actor> actors_;
    // Spring tail scene.
    physics::Skeleton tailSkeleton_;
    std::unique_ptr<physics::SecondaryMotion> spring_;
    physics::Transform tailRoot_;
    int bodyVirtual_ = -1, firstSegmentVirtual_ = -1;

    // Per-frame accumulators (reset by frame()).
    double physicsMs_ = 0.0, particleMs_ = 0.0;
    int stepsSinceFrame_ = 0;
    std::vector<physics::SolverWarning> warnings_;
    size_t brokenSinceFrame_ = 0;
};

}  // namespace vexora::sim
