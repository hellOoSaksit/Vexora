// Animation clips, baked to local poses at a fixed sample rate on import. Sampling
// interpolates between baked frames (slerp for rotations, lerp for positions).
#pragma once

#include <string>
#include <vector>

#include "vexora/rig/Skeleton.h"

namespace vexora::rig {

struct AnimationClip {
    std::string name;
    double duration = 0.0;     // Seconds.
    double sampleRate = 60.0;  // Frames per second of `frames`.
    std::vector<LocalPose> frames;
};

// Pose at time t. With loop, t wraps to [0, duration); otherwise it is clamped.
LocalPose sample(const AnimationClip& clip, double t, bool loop);

}  // namespace vexora::rig
