#include "vexora/rig/SkinnedMesh.h"

namespace vexora::rig {

void skin(const SkinnedMesh& mesh, const ModelPose& pose, std::vector<Vec3>& outPositions, std::vector<Vec3>* outNormals) {
    std::vector<Transform> m(mesh.inverseBind.size());
    for (size_t b = 0; b < m.size() && b < pose.size(); ++b) m[b] = pose[b] * mesh.inverseBind[b];
    outPositions.resize(mesh.positions.size());
    const bool normals = outNormals && mesh.normals.size() == mesh.positions.size();
    if (normals) outNormals->resize(mesh.normals.size());
    for (size_t v = 0; v < mesh.positions.size(); ++v) {
        Vec3 p{}, n{};
        for (int k = 0; k < 4; ++k) {
            const float w = mesh.weights[v][size_t(k)];
            if (w <= 0.0f) continue;
            const Transform& t = m[mesh.joints[v][size_t(k)]];
            p = p + t.apply(mesh.positions[v]) * w;
            if (normals) n = n + t.rotation.rotate(mesh.normals[v]) * w;
        }
        outPositions[v] = p;
        if (normals) (*outNormals)[v] = core::normalized(n);
    }
}

}  // namespace vexora::rig
