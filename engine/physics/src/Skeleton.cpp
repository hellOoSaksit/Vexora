#include "vexora/physics/Skeleton.h"

#include <algorithm>

namespace vexora::physics {

int Skeleton::find(const std::string& name) const {
    for (size_t i = 0; i < bones.size(); ++i)
        if (bones[i].name == name) return int(i);
    return -1;
}

Status Skeleton::validate() const {
    if (bones.empty()) return {ErrorCode::InvalidArgument, "Skeleton has no bones"};
    for (size_t i = 0; i < bones.size(); ++i) {
        const Bone& b = bones[i];
        if (b.parent >= int(i)) return {ErrorCode::InvalidArgument, "Bone '" + b.name + "' must come after its parent"};
        if (i > 0 && b.parent < 0) return {ErrorCode::InvalidArgument, "Only the first bone may be a root"};
        if (!b.restLocal.position.isFinite() || !b.restLocal.rotation.isFinite())
            return {ErrorCode::InvalidArgument, "Bone '" + b.name + "' has a non-finite rest transform"};
    }
    return {};
}

LocalPose restPose(const Skeleton& skeleton) {
    LocalPose pose;
    pose.reserve(skeleton.bones.size());
    for (const Bone& b : skeleton.bones) pose.push_back(b.restLocal);
    return pose;
}

ModelPose toModel(const Skeleton& skeleton, const LocalPose& local) {
    ModelPose model(local.size());
    for (size_t i = 0; i < local.size(); ++i) {
        const int p = skeleton.bones[i].parent;
        model[i] = p < 0 ? local[i] : model[size_t(p)] * local[i];
    }
    return model;
}

LocalPose toLocal(const Skeleton& skeleton, const ModelPose& model) {
    LocalPose local(model.size());
    for (size_t i = 0; i < model.size(); ++i) {
        const int p = skeleton.bones[i].parent;
        local[i] = p < 0 ? model[i] : model[size_t(p)].inverse() * model[i];
    }
    return local;
}

LocalPose blendPoses(const Skeleton& skeleton, const LocalPose& a, const LocalPose& b, const std::vector<float>& weights) {
    LocalPose out = a;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        const float w = i < weights.size() ? weights[i] : 0.0f;
        if (w <= 0.0f) continue;
        out[i].rotation = w >= 1.0f ? b[i].rotation : slerp(a[i].rotation, b[i].rotation, w);
        if (skeleton.bones[i].parent < 0)
            out[i].position = a[i].position + (b[i].position - a[i].position) * std::min(w, 1.0f);
    }
    return out;
}

}  // namespace vexora::physics
