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

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline Vec3 normalized(const Vec3& v) {
    const float l = v.length();
    return l > 0.0f ? v * (1.0f / l) : Vec3{};
}

struct Quat {
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;

    static Quat fromAxisAngle(const Vec3& axis, float radians);
    // Shortest rotation taking unit direction `from` onto unit direction `to`.
    static Quat fromTo(const Vec3& from, const Vec3& to);
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w); }
    Vec3 rotate(const Vec3& v) const;
    Quat conjugate() const { return {-x, -y, -z, w}; }
    Quat normalized() const;
    Quat operator*(const Quat& o) const {
        return {w * o.x + x * o.w + y * o.z - z * o.y, w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w, w * o.w - x * o.x - y * o.y - z * o.z};
    }
};

// Angle (radians, 0..pi) of the rotation between two orientations.
float angleBetween(const Quat& a, const Quat& b);
// Spherical blend along the shortest path; t = 0 gives a, 1 gives b.
Quat slerp(const Quat& a, const Quat& b, float t);

struct Transform {
    Vec3 position;
    Quat rotation;

    // Composition: (a * b) applies b first, then a (b is expressed in a's space).
    Transform operator*(const Transform& b) const { return {position + rotation.rotate(b.position), (rotation * b.rotation).normalized()}; }
    Transform inverse() const {
        const Quat inv = rotation.conjugate();
        return {inv.rotate(position * -1.0f), inv};
    }
    Vec3 apply(const Vec3& p) const { return position + rotation.rotate(p); }
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
    DeviceUnavailable,  // Requested compute device (e.g. GPU) does not exist or lacks support.
    DeviceError,        // The device failed at runtime (driver error, device lost, out of memory).
};

// Result of an API call that can fail. Failures are always reported, never silently ignored.
template <typename T>
struct Result {
    T value{};
    ErrorCode error = ErrorCode::None;
    std::string message;

    bool ok() const { return error == ErrorCode::None; }
    static Result success(T v) { return Result{std::move(v), ErrorCode::None, {}}; }
    static Result failure(ErrorCode e, std::string msg) { return Result{T{}, e, std::move(msg)}; }
};

struct Status {
    ErrorCode error = ErrorCode::None;
    std::string message;
    bool ok() const { return error == ErrorCode::None; }
};

}  // namespace vexora::physics
