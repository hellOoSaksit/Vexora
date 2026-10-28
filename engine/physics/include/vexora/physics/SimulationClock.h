// Fixed-timestep clock that decouples simulation from the render/UI frame rate.
#pragma once

#include <cstdint>

namespace vexora::physics {

class FixedStepClock {
public:
    // fixedDt: simulation step in seconds. maxStepsPerFrame: cap to avoid the "spiral of
    // death" when a frame takes longer than the steps it schedules; excess time is dropped
    // and reported through droppedTime() so the UI can show that playback fell behind.
    explicit FixedStepClock(double fixedDt = 1.0 / 60.0, int maxStepsPerFrame = 8);

    // Adds wall/frame time and returns the number of fixed steps to run now.
    int advance(double frameDt);

    // Fraction [0,1) of a step left in the accumulator: blend factor between the previous
    // and current physics transforms for rendering.
    float interpolationAlpha() const;

    double fixedDt() const { return fixedDt_; }
    double droppedTime() const { return droppedTime_; }
    uint64_t totalSteps() const { return totalSteps_; }
    void reset();

private:
    double fixedDt_;
    int maxStepsPerFrame_;
    double accumulator_ = 0.0;
    double droppedTime_ = 0.0;
    uint64_t totalSteps_ = 0;
};

}  // namespace vexora::physics
