#include "vexora/physics/SimulationClock.h"

#include <algorithm>

namespace vexora::physics {

FixedStepClock::FixedStepClock(double fixedDt, int maxStepsPerFrame)
    : fixedDt_(fixedDt > 0.0 ? fixedDt : 1.0 / 60.0), maxStepsPerFrame_(std::max(1, maxStepsPerFrame)) {}

int FixedStepClock::advance(double frameDt) {
    if (!(frameDt > 0.0)) return 0;  // Also rejects NaN.
    accumulator_ += frameDt;
    int steps = 0;
    // Small epsilon so 1/60 accumulated twice still yields 2 steps despite rounding.
    const double eps = fixedDt_ * 1e-9;
    while (accumulator_ + eps >= fixedDt_ && steps < maxStepsPerFrame_) {
        accumulator_ -= fixedDt_;
        ++steps;
    }
    if (accumulator_ + eps >= fixedDt_) {
        // Over budget: drop whole steps but keep the fractional part for smooth interpolation.
        const double whole = static_cast<double>(static_cast<uint64_t>((accumulator_ + eps) / fixedDt_)) * fixedDt_;
        droppedTime_ += whole;
        accumulator_ -= whole;
    }
    accumulator_ = std::max(0.0, accumulator_);
    totalSteps_ += static_cast<uint64_t>(steps);
    return steps;
}

float FixedStepClock::interpolationAlpha() const {
    return static_cast<float>(std::clamp(accumulator_ / fixedDt_, 0.0, 1.0));
}

void FixedStepClock::reset() {
    accumulator_ = 0.0;
    droppedTime_ = 0.0;
    totalSteps_ = 0;
}

}  // namespace vexora::physics
