// Physics-facing types. Math and Result types live in vexora/core/Types.h (shared with the
// rig module); they are re-exported here so physics code and users keep one namespace.
#pragma once

#include "vexora/core/Types.h"

namespace vexora::physics {

using core::Vec3;
using core::Quat;
using core::Transform;
using core::ErrorCode;
using core::Result;
using core::Status;
using core::dot;
using core::cross;
using core::normalized;
using core::angleBetween;
using core::slerp;
using core::interpolate;

// Generational handle: a destroyed body's handle never aliases a new body.
struct BodyHandle {
    uint32_t index = UINT32_MAX;
    uint32_t generation = 0;
    bool isValid() const { return index != UINT32_MAX; }
    bool operator==(const BodyHandle& o) const { return index == o.index && generation == o.generation; }
    bool operator!=(const BodyHandle& o) const { return !(*this == o); }
};

struct ConstraintHandle {
    uint32_t index = UINT32_MAX;
    uint32_t generation = 0;
    bool isValid() const { return index != UINT32_MAX; }
    bool operator==(const ConstraintHandle& o) const { return index == o.index && generation == o.generation; }
};

}  // namespace vexora::physics
