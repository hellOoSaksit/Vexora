// Animation clips, baked on import to local poses at the source's own keyframe times (plus
// extra samples where the source curve is not linear). Sampling interpolates between baked
// frames (slerp for rotations, lerp for positions).
#pragma once

#include <string>
#include <vector>

#include "vexora/rig/Skeleton.h"

namespace vexora::rig {

struct AnimationClip {
    std::string name;
    double duration = 0.0;       // Seconds.
    std::vector<double> times;   // Ascending time of each frame (seconds).
    std::vector<LocalPose> frames;
};

// Pose at time t. With loop, t wraps to [0, duration); otherwise it is clamped.
LocalPose sample(const AnimationClip& clip, double t, bool loop);

}  // namespace vexora::rig
