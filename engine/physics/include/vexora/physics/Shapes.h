// Collision shape descriptions. These are authored data, separate from the render mesh:
// a render mesh is never used as a collision shape implicitly.
#pragma once

#include <memory>
#include <vector>

#include "vexora/physics/Types.h"

namespace vexora::physics {

enum class ShapeType { Box, Sphere, Capsule, ConvexHull, TriangleMesh, Compound };

struct ShapeDesc;

struct CompoundChild {
    std::shared_ptr<const ShapeDesc> shape;
    Transform localTransform;
};

// Constraints enforced by validateShape() / PhysicsWorld::createBody():
//  - TriangleMesh is only allowed on Static or Kinematic bodies. Dynamic concave objects
//    must use a Compound of convex pieces (e.g. from convex decomposition) instead,
//    because mesh-vs-mesh contact has no volume and generates unstable contacts.
//  - Compound children must be convex (Box/Sphere/Capsule/ConvexHull) when used dynamically.
struct ShapeDesc {
    ShapeType type = ShapeType::Box;

    Vec3 halfExtents{0.5f, 0.5f, 0.5f};  // Box
    float radius = 0.5f;                  // Sphere, Capsule
    float halfHeight = 0.5f;              // Capsule: half length of the cylinder part along Y

    std::vector<Vec3> points;             // ConvexHull input points; TriangleMesh vertices
    std::vector<uint32_t> indices;        // TriangleMesh: 3 per triangle
    std::vector<CompoundChild> children;  // Compound

    static ShapeDesc box(Vec3 halfExtents);
    static ShapeDesc sphere(float radius);
    static ShapeDesc capsule(float halfHeight, float radius);
    static ShapeDesc convexHull(std::vector<Vec3> points);
    static ShapeDesc triangleMesh(std::vector<Vec3> vertices, std::vector<uint32_t> indices);
    static ShapeDesc compound(std::vector<CompoundChild> children);
};

enum class MotionType { Static, Kinematic, Dynamic };

// Checks a shape against a motion type without touching any engine.
Status validateShape(const ShapeDesc& shape, MotionType motion);

}  // namespace vexora::physics
