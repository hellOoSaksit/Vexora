// Minimal skeleton and pose types used by character physics. Vexora's animation system does
// not exist yet; when it does, it should produce these (or convert into them). Bone order is
// parents-before-children, so model-space poses can be computed in one pass.
#pragma once

#include <string>
#include <vector>

#include "vexora/physics/Types.h"

namespace vexora::physics {

struct Bone {
    std::string name;
    int parent = -1;       // Index of an earlier bone, or -1 for the root.
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

}  // namespace vexora::physics
