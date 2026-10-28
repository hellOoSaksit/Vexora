#include <doctest/doctest.h>

#include "vexora/physics/QualityProfile.h"
#include "vexora/physics/SimulationClock.h"
#include "vexora/physics/Types.h"

using namespace vexora::physics;

TEST_CASE("FixedStepClock: steps are independent of the frame rate") {
    // 10 seconds of wall time delivered at 30, 60 and 144 FPS must schedule the same steps.
    for (double fps : {30.0, 60.0, 144.0}) {
        FixedStepClock clock(1.0 / 60.0, 8);
        const int frames = static_cast<int>(10.0 * fps + 0.5);
        for (int i = 0; i < frames; ++i) clock.advance(1.0 / fps);
        CAPTURE(fps);
        CHECK(clock.totalSteps() == 600);
        CHECK(clock.droppedTime() == doctest::Approx(0.0));
    }
}

TEST_CASE("FixedStepClock: caps steps per frame and reports dropped time") {
    FixedStepClock clock(1.0 / 60.0, 4);
    const int steps = clock.advance(1.0);  // A one second hitch.
    CHECK(steps == 4);
    CHECK(clock.droppedTime() == doctest::Approx(1.0 - 4.0 / 60.0).epsilon(1e-6));
    CHECK(clock.interpolationAlpha() < 1.0f);
}

TEST_CASE("FixedStepClock: interpolation alpha and invalid input") {
    FixedStepClock clock(0.1, 8);
    CHECK(clock.advance(0.25) == 2);
    CHECK(clock.interpolationAlpha() == doctest::Approx(0.5f));
    CHECK(clock.advance(-1.0) == 0);
    CHECK(clock.advance(std::nan("")) == 0);
    CHECK(clock.interpolationAlpha() == doctest::Approx(0.5f));
}

TEST_CASE("Transform interpolation") {
    Transform a{{0, 0, 0}, {}};
    Transform b{{2, 4, 6}, Quat::fromAxisAngle({0, 1, 0}, 1.0f)};
    Transform mid = interpolate(a, b, 0.5f);
    CHECK(mid.position.x == doctest::Approx(1.0f));
    CHECK(mid.position.y == doctest::Approx(2.0f));
    // nlerp half way between identity and 1 rad about Y is 0.5 rad about Y.
    Quat expected = Quat::fromAxisAngle({0, 1, 0}, 0.5f);
    CHECK(mid.rotation.y == doctest::Approx(expected.y).epsilon(1e-3));
    CHECK(mid.rotation.w == doctest::Approx(expected.w).epsilon(1e-3));
}

TEST_CASE("Quality profiles are behaviourally distinct") {
    QualitySettings i = settingsFor(QualityProfile::InteractivePreview);
    QualitySettings h = settingsFor(QualityProfile::HighQualityPreview);
    QualitySettings f = settingsFor(QualityProfile::FinalBake);
    CHECK(i.fixedDt > h.fixedDt);
    CHECK(h.fixedDt > f.fixedDt);
    CHECK(i.collisionSteps < h.collisionSteps);
    CHECK(h.velocityIterations < f.velocityIterations);
    CHECK(h.positionIterations < f.positionIterations);
    CHECK_FALSE(i.forceContinuousCollision);
    CHECK(f.forceContinuousCollision);
}
