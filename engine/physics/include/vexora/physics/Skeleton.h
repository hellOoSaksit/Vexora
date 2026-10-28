// Character physics uses the rig module's skeleton and pose types.
#pragma once

#include "vexora/physics/Types.h"
#include "vexora/rig/Skeleton.h"

namespace vexora::physics {

using rig::blendPoses;
using rig::Bone;
using rig::LocalPose;
using rig::ModelPose;
using rig::restPose;
using rig::Skeleton;
using rig::toLocal;
using rig::toModel;

}  // namespace vexora::physics
