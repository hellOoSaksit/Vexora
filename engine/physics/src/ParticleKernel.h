// Per-particle update shared by the CPU backend. shaders/particles.comp is the GPU twin of
// updateParticle(); any change here must be mirrored there (ParticleBackendParity test).
#pragma once

#include <cmath>
#include <cstdint>

#include "vexora/physics/ParticleSystem.h"

namespace vexora::physics::detail {

// std140 uniform block; every member is a vec4/uvec4 so C++ and GLSL layouts match exactly.
struct KernelParams {
    float gravityDt[4];  // xyz gravity, w dt
    float windDrag[4];   // xyz wind, w drag
    float response[4];   // x restitution, y friction
    uint32_t counts[4];  // x particle slots, y planes, z spheres
    float planes[kMaxParticleColliders][4];   // xyz normal, w offset
    float spheres[kMaxParticleColliders][4];  // xyz centre, w radius
};
static_assert(sizeof(KernelParams) == 64 + 2 * 16 * kMaxParticleColliders, "std140 layout");

inline void collide(float& px, float& py, float& pz, float& vx, float& vy, float& vz, float nx, float ny, float nz,
                    float penetration, float restitution, float friction) {
    px += nx * penetration;
    py += ny * penetration;
    pz += nz * penetration;
    const float vn = vx * nx + vy * ny + vz * nz;
    if (vn < 0.0f) {
        const float tx = vx - nx * vn, ty = vy - ny * vn, tz = vz - nz * vn;
        const float keep = 1.0f - friction;
        vx = tx * keep - nx * vn * restitution;
        vy = ty * keep - ny * vn * restitution;
        vz = tz * keep - nz * vn * restitution;
    }
}

inline void collidePlanes(Particle& p, const KernelParams& k) {
    for (uint32_t i = 0; i < k.counts[1]; ++i) {
        const float* pl = k.planes[i];
        const float d = pl[0] * p.px + pl[1] * p.py + pl[2] * p.pz - pl[3] - p.radius;
        if (d < 0.0f) collide(p.px, p.py, p.pz, p.vx, p.vy, p.vz, pl[0], pl[1], pl[2], -d, k.response[0], k.response[1]);
    }
}

inline void updateParticle(Particle& p, const KernelParams& k) {
    if (p.lifetime <= 0.0f) return;  // Free slot. Lifecycle is decided on the CPU.
    const float dt = k.gravityDt[3], drag = k.windDrag[3];
    p.age += dt;
    p.vx += (k.gravityDt[0] + drag * (k.windDrag[0] - p.vx)) * dt;
    p.vy += (k.gravityDt[1] + drag * (k.windDrag[1] - p.vy)) * dt;
    p.vz += (k.gravityDt[2] + drag * (k.windDrag[2] - p.vz)) * dt;
    p.px += p.vx * dt;
    p.py += p.vy * dt;
    p.pz += p.vz * dt;
    collidePlanes(p, k);
    for (uint32_t i = 0; i < k.counts[2]; ++i) {
        const float* s = k.spheres[i];
        const float dx = p.px - s[0], dy = p.py - s[1], dz = p.pz - s[2];
        const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        const float pen = s[3] + p.radius - dist;
        if (pen > 0.0f && dist > 1e-6f)
            collide(p.px, p.py, p.pz, p.vx, p.vy, p.vz, dx / dist, dy / dist, dz / dist, pen, k.response[0], k.response[1]);
    }
    // Planes are the hard boundary (floor, walls): resolve them again last so a sphere's
    // push-out cannot leave a particle inside the floor. A particle wedged between both may
    // then stay slightly inside the sphere instead — acceptable for VFX.
    if (k.counts[2] > 0) collidePlanes(p, k);
}

}  // namespace vexora::physics::detail
