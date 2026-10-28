#include "vexora/physics/Types.h"

namespace vexora::physics {

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

}  // namespace vexora::physics
