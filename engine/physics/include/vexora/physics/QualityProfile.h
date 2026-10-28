// Simulation quality profiles. Each profile changes solver behaviour, not just a label.
// A profile is part of a simulation's identity: results from different profiles differ,
// so any future bake cache must key on the profile's settings.
#pragma once

namespace vexora::physics {

enum class QualityProfile {
    InteractivePreview,  // Viewport scrubbing / live editing. Fast, tolerates small penetration.
    HighQualityPreview,  // Review playback. Finer step and more iterations.
    FinalBake,           // Offline bake. Small step, many iterations, CCD on every dynamic body.
};
// Note: "Game Runtime Export" is intentionally not a simulation profile. Exported game
// assets are re-simulated by the target game engine; see docs/physics/02-architecture.md.

struct QualitySettings {
    double fixedDt = 1.0 / 60.0;
    int collisionSteps = 1;        // Sub-steps per fixed step (Jolt "collision steps").
    int velocityIterations = 10;
    int positionIterations = 2;
    bool forceContinuousCollision = false;  // Use linear-cast CCD on all dynamic bodies.
};

QualitySettings settingsFor(QualityProfile profile);
const char* toString(QualityProfile profile);

}  // namespace vexora::physics
