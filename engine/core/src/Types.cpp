#include "vexora/core/Types.h"

#include <algorithm>

namespace vexora::core {

Quat Quat::fromAxisAngle(const Vec3& axis, float radians) {
    const float len = axis.length();
    if (len <= 0.0f) return {};
    const float s = std::sin(radians * 0.5f) / len;
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(radians * 0.5f)};
}

Vec3 Quat::rotate(const Vec3& v) const {
    // v' = v + 2w(q x v) + 2(q x (q x v))
    const Vec3 q{x, y, z};
    const Vec3 t{2.0f * (q.y * v.z - q.z * v.y), 2.0f * (q.z * v.x - q.x * v.z), 2.0f * (q.x * v.y - q.y * v.x)};
    const Vec3 c{q.y * t.z - q.z * t.y, q.z * t.x - q.x * t.z, q.x * t.y - q.y * t.x};
    return v + t * w + c;
}

Quat Quat::fromTo(const Vec3& from, const Vec3& to) {
    const float d = dot(from, to);
    if (d >= 1.0f - 1e-7f) return {};
    if (d <= -1.0f + 1e-7f) {
        // Opposite directions: rotate 180 degrees about any axis perpendicular to `from`.
        Vec3 axis = cross({1, 0, 0}, from);
        if (axis.length() < 1e-4f) axis = cross({0, 1, 0}, from);
        return fromAxisAngle(core::normalized(axis), 3.14159265f);
    }
    const Vec3 c = cross(from, to);
    return Quat{c.x, c.y, c.z, 1.0f + d}.normalized();
}

Quat Quat::normalized() const {
    const float n = std::sqrt(x * x + y * y + z * z + w * w);
    return n > 0.0f ? Quat{x / n, y / n, z / n, w / n} : Quat{};
}

float angleBetween(const Quat& a, const Quat& b) {
    // atan2 of the relative rotation stays accurate for tiny angles, unlike acos near 1.
    const Quat r = a.conjugate() * b;
    return 2.0f * std::atan2(std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z), std::fabs(r.w));
}

Quat slerp(const Quat& a, const Quat& b, float t) {
    Quat q = b;
    float d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0f) {
        q = {-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    if (d > 0.9995f) return interpolate({{}, a}, {{}, q}, t).rotation;  // Nearly equal: nlerp.
    const float theta = std::acos(d), s = std::sin(theta);
    const float wa = std::sin((1.0f - t) * theta) / s, wb = std::sin(t * theta) / s;
    return Quat{a.x * wa + q.x * wb, a.y * wa + q.y * wb, a.z * wa + q.z * wb, a.w * wa + q.w * wb}.normalized();
}

Transform interpolate(const Transform& a, const Transform& b, float alpha) {
    Transform out;
    out.position = a.position + (b.position - a.position) * alpha;
    const Quat& qa = a.rotation;
    Quat qb = b.rotation;
    const float dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w;
    if (dot < 0.0f) qb = {-qb.x, -qb.y, -qb.z, -qb.w};
    Quat q{qa.x + (qb.x - qa.x) * alpha, qa.y + (qb.y - qa.y) * alpha, qa.z + (qb.z - qa.z) * alpha,
           qa.w + (qb.w - qa.w) * alpha};
    const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    out.rotation = n > 0.0f ? Quat{q.x / n, q.y / n, q.z / n, q.w / n} : Quat{};
    return out;
}

}  // namespace vexora::core
