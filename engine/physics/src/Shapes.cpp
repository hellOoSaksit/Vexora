#include "vexora/physics/Shapes.h"

#include <algorithm>

namespace vexora::physics {

ShapeDesc ShapeDesc::box(Vec3 halfExtents) {
    ShapeDesc s;
    s.type = ShapeType::Box;
    s.halfExtents = halfExtents;
    return s;
}

ShapeDesc ShapeDesc::sphere(float radius) {
    ShapeDesc s;
    s.type = ShapeType::Sphere;
    s.radius = radius;
    return s;
}

ShapeDesc ShapeDesc::capsule(float halfHeight, float radius) {
    ShapeDesc s;
    s.type = ShapeType::Capsule;
    s.halfHeight = halfHeight;
    s.radius = radius;
    return s;
}

ShapeDesc ShapeDesc::convexHull(std::vector<Vec3> points) {
    ShapeDesc s;
    s.type = ShapeType::ConvexHull;
    s.points = std::move(points);
    return s;
}

ShapeDesc ShapeDesc::triangleMesh(std::vector<Vec3> vertices, std::vector<uint32_t> indices) {
    ShapeDesc s;
    s.type = ShapeType::TriangleMesh;
    s.points = std::move(vertices);
    s.indices = std::move(indices);
    return s;
}

ShapeDesc ShapeDesc::compound(std::vector<CompoundChild> children) {
    ShapeDesc s;
    s.type = ShapeType::Compound;
    s.children = std::move(children);
    return s;
}

namespace {

Status fail(std::string msg) { return {ErrorCode::InvalidArgument, std::move(msg)}; }

bool positiveFinite(float v) { return std::isfinite(v) && v > 0.0f; }

// True if the points span a volume. Flat or collinear point sets would give a zero-mass hull.
bool spansVolume(const std::vector<Vec3>& pts) {
    float scale = 0.0f;
    for (const Vec3& p : pts) scale = std::max(scale, (p - pts[0]).length());
    const float eps = std::max(scale * 1e-4f, 1e-6f);
    size_t i1 = 0;
    for (size_t i = 1; i < pts.size() && i1 == 0; ++i)
        if ((pts[i] - pts[0]).length() > eps) i1 = i;
    if (i1 == 0) return false;
    const Vec3 e1 = pts[i1] - pts[0];
    Vec3 n;
    for (size_t i = 1; i < pts.size(); ++i) {
        n = cross(e1, pts[i] - pts[0]);
        if (n.length() > eps * e1.length()) break;
    }
    const float nl = n.length();
    if (nl <= eps * e1.length()) return false;
    for (const Vec3& p : pts)
        if (std::abs(dot(n, p - pts[0])) / nl > eps) return true;
    return false;
}

}  // namespace

Status validateShape(const ShapeDesc& shape, MotionType motion) {
    switch (shape.type) {
        case ShapeType::Box:
            if (!positiveFinite(shape.halfExtents.x) || !positiveFinite(shape.halfExtents.y) ||
                !positiveFinite(shape.halfExtents.z))
                return fail("Box half extents must be positive and finite");
            return {};
        case ShapeType::Sphere:
            if (!positiveFinite(shape.radius)) return fail("Sphere radius must be positive and finite");
            return {};
        case ShapeType::Capsule:
            if (!positiveFinite(shape.radius) || !positiveFinite(shape.halfHeight))
                return fail("Capsule radius and half height must be positive and finite");
            return {};
        case ShapeType::ConvexHull:
            if (shape.points.size() < 4) return fail("Convex hull needs at least 4 non-coplanar points");
            for (const Vec3& p : shape.points)
                if (!p.isFinite()) return fail("Convex hull contains a non-finite point");
            if (!spansVolume(shape.points)) return fail("Convex hull points are coplanar or collinear (zero volume)");
            return {};
        case ShapeType::TriangleMesh:
            if (motion == MotionType::Dynamic)
                return {ErrorCode::UnsupportedShapeForMotionType,
                        "Triangle meshes cannot be Dynamic: concave mesh contacts have no volume and are "
                        "unstable. Use a ConvexHull, or a Compound of convex pieces (convex decomposition)."};
            if (shape.indices.empty() || shape.indices.size() % 3 != 0)
                return fail("Triangle mesh index count must be a non-zero multiple of 3");
            for (uint32_t i : shape.indices)
                if (i >= shape.points.size()) return fail("Triangle mesh index out of range");
            for (const Vec3& p : shape.points)
                if (!p.isFinite()) return fail("Triangle mesh contains a non-finite vertex");
            return {};
        case ShapeType::Compound:
            if (shape.children.empty()) return fail("Compound shape needs at least one child");
            for (const CompoundChild& c : shape.children) {
                if (!c.shape) return fail("Compound child shape is null");
                if (c.shape->type == ShapeType::Compound) return fail("Nested compounds are not supported");
                if (!c.localTransform.position.isFinite() || !c.localTransform.rotation.isFinite())
                    return fail("Compound child transform is not finite");
                Status s = validateShape(*c.shape, motion);
                if (!s.ok()) return s;
            }
            return {};
    }
    return fail("Unknown shape type");
}

}  // namespace vexora::physics
