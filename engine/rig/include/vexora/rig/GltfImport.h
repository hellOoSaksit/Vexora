// glTF 2.0 (.glb / .gltf) character import: first skin, its meshes, and all animations.
//
// Conversion rules (documented so results are predictable):
//  - Output is Vexora space: metres, +Y up, right-handed (same as glTF).
//  - Node transforms above the skeleton (armature nodes, unit-scale roots) are folded into
//    the root bones, so model space = the glTF scene root.
//  - Vexora transforms have no scale. Uniform node scale is baked into positions; non-uniform
//    scale on joints is not supported and is reported as a warning.
//  - Vertices are stored skinned at the file's rest pose. If the rest pose is not the bind
//    pose of every joint, a warning is reported (skinning is then approximated).
//  - Animations are baked at every keyframe time of the source (exact for LINEAR curves);
//    STEP and CUBICSPLINE curves get extra samples (`curveSampleRate`). Morph-target
//    (weights) channels are ignored.
#pragma once

#include <string>
#include <vector>

#include "vexora/rig/Animation.h"
#include "vexora/rig/SkinnedMesh.h"

namespace vexora::rig {

struct ImportedCharacter {
    std::string sourcePath;
    Skeleton skeleton;
    ModelPose restModel;
    std::vector<SkinnedMesh> meshes;
    std::vector<AnimationClip> clips;
    std::vector<std::string> warnings;  // Everything lost or approximated during import.
    Vec3 boundsMin, boundsMax;          // Rest-pose mesh bounds (model space).
};

struct GltfImportOptions {
    double curveSampleRate = 60.0;  // Extra samples per second inside non-linear curves.
    int maxBones = 256;
};

// Uniformly rescales a character (rest pose, meshes, clips, bounds) about the model origin,
// e.g. to fix files authored in centimetres. Rotations are unchanged.
void scaleCharacter(ImportedCharacter& character, float factor);

core::Result<ImportedCharacter> importGltf(const std::string& path, const GltfImportOptions& options = {});

}  // namespace vexora::rig
