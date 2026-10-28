// Skeleton and pose types: the rig data shared by animation, physics and export. Bone order is
// parents-before-children, so model-space poses can be computed in one pass.
#pragma once

#include <string>
#include <vector>

#include "vexora/core/Types.h"

namespace vexora::rig {

using core::Quat;
using core::Status;
using core::Transform;
using core::Vec3;

struct Bone {
    std::string name;
    int parent = -1;       // Index of an earlier bone, or -1 for a root (several roots are allowed).
    Transform restLocal;   // Rest (bind) transform relative to the parent.
};

struct Skeleton {
    std::vector<Bone> bones;

    int find(const std::string& name) const;
    // Checks that parents precede children and transforms are finite.
    Status validate() const;
};

// Local (parent-relative) transforms, one per bone.
using LocalPose = std::vector<Transform>;
// Model-space transforms, one per bone (relative to the skeleton root's space).
using ModelPose = std::vector<Transform>;

LocalPose restPose(const Skeleton& skeleton);
ModelPose toModel(const Skeleton& skeleton, const LocalPose& local);
LocalPose toLocal(const Skeleton& skeleton, const ModelPose& model);

// Per-bone blend of local rotations (and the root's translation): weight 0 = a, 1 = b.
// Non-root translations come from `a` so bone lengths never change.
LocalPose blendPoses(const Skeleton& skeleton, const LocalPose& a, const LocalPose& b, const std::vector<float>& weights);

}  // namespace vexora::rig
