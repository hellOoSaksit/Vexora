// Physics material: surface/mass properties for simulation. Independent of render materials.
#pragma once

#include <string>

namespace vexora::physics {

struct PhysicsMaterial {
    std::string name = "Default";
    float friction = 0.5f;     // Combined per pair as sqrt(a*b) (Jolt default).
    float restitution = 0.0f;  // Combined per pair as max(a, b) (Jolt default).
    float density = 1000.0f;   // kg/m^3, used when a body has no explicit mass.
};

enum class MaterialPreset { Metal, Wood, Stone, Rubber, Ice, Fabric, FleshLike };

// Presets are starting values for artists to tune. They are NOT measured physical
// constants and are not correct for every situation (real friction depends on both
// surfaces, wetness, finish, etc.).
PhysicsMaterial makeMaterial(MaterialPreset preset);

}  // namespace vexora::physics
