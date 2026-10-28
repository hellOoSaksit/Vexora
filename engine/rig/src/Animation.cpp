#include "vexora/rig/Animation.h"

#include <algorithm>
#include <cmath>

namespace vexora::rig {

LocalPose sample(const AnimationClip& clip, double t, bool loop) {
    if (clip.frames.empty()) return {};
    if (clip.frames.size() == 1 || clip.duration <= 0.0) return clip.frames.front();
    if (loop) {
        t = std::fmod(t, clip.duration);
        if (t < 0.0) t += clip.duration;
    } else {
        t = std::clamp(t, 0.0, clip.duration);
    }
    // Frame interval containing t.
    const auto it = std::upper_bound(clip.times.begin(), clip.times.end(), t);
    const size_t last = clip.frames.size() - 1;
    const size_t j = std::min(last, size_t(it - clip.times.begin()));
    const size_t i = j == 0 ? 0 : j - 1;
    const double span = clip.times[j] - clip.times[i];
    const float u = span > 0.0 ? float(std::clamp((t - clip.times[i]) / span, 0.0, 1.0)) : 0.0f;
    const LocalPose& a = clip.frames[i];
    const LocalPose& b = clip.frames[j];
    LocalPose out(a.size());
    for (size_t k = 0; k < a.size(); ++k) {
        out[k].position = a[k].position + (b[k].position - a[k].position) * u;
        out[k].rotation = core::slerp(a[k].rotation, b[k].rotation, u);
    }
    return out;
}

}  // namespace vexora::rig
