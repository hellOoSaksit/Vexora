// Skinned mesh in model space, skinned with linear blend skinning (LBS).
// Vertices are stored at the rest pose; skinning matrices are bone * inverseBind[bone].
#pragma once

#include <array>
#include <string>
#include <vector>

#include "vexora/rig/Skeleton.h"

namespace vexora::rig {

struct SkinnedMesh {
    std::string name;
    std::vector<Vec3> positions;                 // Rest pose, model space (metres).
    std::vector<Vec3> normals;                   // Empty if the source had none.
    std::vector<std::array<uint16_t, 4>> joints; // Bone indices into the Skeleton.
    std::vector<std::array<float, 4>> weights;   // Normalised to sum 1.
    std::vector<uint32_t> indices;               // Triangles.
    std::vector<Transform> inverseBind;          // Per bone: inverse of its rest model transform.
};

// CPU skinning of positions (and normals if present) for a model-space pose.
void skin(const SkinnedMesh& mesh, const ModelPose& pose, std::vector<Vec3>& outPositions, std::vector<Vec3>* outNormals = nullptr);

}  // namespace vexora::rig
