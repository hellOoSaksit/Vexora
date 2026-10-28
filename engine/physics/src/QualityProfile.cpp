#include "vexora/physics/QualityProfile.h"

namespace vexora::physics {

// Values chosen from the parameter sweep recorded in docs/physics/03-test-plan.md
// (10-box tower drift and 10-link hinge chain stretch). Notably, fewer than 8 velocity
// iterations at 60 Hz let the tower collapse, so Interactive uses Jolt's defaults (10/2).
QualitySettings settingsFor(QualityProfile profile) {
    switch (profile) {
        case QualityProfile::InteractivePreview: return {1.0 / 60.0, 1, 10, 2, false};
        case QualityProfile::HighQualityPreview: return {1.0 / 120.0, 2, 10, 2, false};
        case QualityProfile::FinalBake: return {1.0 / 240.0, 2, 16, 4, true};
    }
    return {};
}

const char* toString(QualityProfile profile) {
    switch (profile) {
        case QualityProfile::InteractivePreview: return "InteractivePreview";
        case QualityProfile::HighQualityPreview: return "HighQualityPreview";
        case QualityProfile::FinalBake: return "FinalBake";
    }
    return "Unknown";
}

}  // namespace vexora::physics
