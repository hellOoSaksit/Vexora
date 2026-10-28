#include "vexora/physics/PhysicsMaterial.h"

namespace vexora::physics {

// Densities are typical handbook values; friction/restitution are game-style starting points.
PhysicsMaterial makeMaterial(MaterialPreset preset) {
    switch (preset) {
        case MaterialPreset::Metal: return {"Metal", 0.40f, 0.20f, 7850.0f};       // steel
        case MaterialPreset::Wood: return {"Wood", 0.50f, 0.25f, 600.0f};
        case MaterialPreset::Stone: return {"Stone", 0.70f, 0.10f, 2600.0f};       // granite-like
        case MaterialPreset::Rubber: return {"Rubber", 0.90f, 0.80f, 1100.0f};
        case MaterialPreset::Ice: return {"Ice", 0.03f, 0.05f, 917.0f};
        // Rigid-body approximation of cloth props (e.g. a folded towel); real cloth needs a cloth solver.
        case MaterialPreset::Fabric: return {"Fabric", 0.60f, 0.00f, 300.0f};
        // For ragdoll limbs; soft tissue deformation is out of scope for rigid bodies.
        case MaterialPreset::FleshLike: return {"FleshLike", 0.60f, 0.05f, 1050.0f};
    }
    return {};
}

}  // namespace vexora::physics
