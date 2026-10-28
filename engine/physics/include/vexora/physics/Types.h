// Engine-agnostic math and handle types for the Vexora physics module.
// Nothing in the public headers includes a third-party physics engine.
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace vexora::physics {

// Units: metres, kilograms, seconds, radians. Coordinate system: right-handed, +Y up.
// Conversions to other engines' conventions belong to the export layer, not here.
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    static Quat fromAxisAngle(const Vec3& axis, float radians);
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w); }
    Vec3 rotate(const Vec3& v) const;
};

struct Transform {
    Vec3 position;
    Quat rotation;
};

// Linear for position, normalised lerp (shortest path) for rotation. Used for
// render-side interpolation between two fixed physics steps.
Transform interpolate(const Transform& a, const Transform& b, float alpha);

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

enum class ErrorCode {
    None,
    InvalidArgument,
    UnsupportedShapeForMotionType,
    ShapeCreationFailed,
    InvalidHandle,
    CapacityExceeded,
    StateMismatch,
};

// Result of an API call that can fail. Failures are always reported, never silently ignored.
template <typename T>
struct Result {
    T value{};
    ErrorCode error = ErrorCode::None;
    std::string message;

    bool ok() const { return error == ErrorCode::None; }
    static Result success(T v) { return Result{v, ErrorCode::None, {}}; }
    static Result failure(ErrorCode e, std::string msg) { return Result{T{}, e, std::move(msg)}; }
};

struct Status {
    ErrorCode error = ErrorCode::None;
    std::string message;
    bool ok() const { return error == ErrorCode::None; }
};

}  // namespace vexora::physics
