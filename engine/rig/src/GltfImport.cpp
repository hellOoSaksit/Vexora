#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>

#include "vexora/rig/GltfImport.h"

namespace vexora::rig {

namespace {

using core::ErrorCode;
using R = core::Result<ImportedCharacter>;

// Column-major 4x4 in double precision (import-time only).
struct Mat4 {
    std::array<double, 16> m{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    double& operator()(int r, int c) { return m[size_t(c * 4 + r)]; }
    double operator()(int r, int c) const { return m[size_t(c * 4 + r)]; }
    Mat4 operator*(const Mat4& b) const {
        Mat4 o;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                double s = 0;
                for (int k = 0; k < 4; ++k) s += (*this)(r, k) * b(k, c);
                o(r, c) = s;
            }
        return o;
    }
    Vec3 point(const Vec3& p) const {
        return {float((*this)(0, 0) * p.x + (*this)(0, 1) * p.y + (*this)(0, 2) * p.z + (*this)(0, 3)),
                float((*this)(1, 0) * p.x + (*this)(1, 1) * p.y + (*this)(1, 2) * p.z + (*this)(1, 3)),
                float((*this)(2, 0) * p.x + (*this)(2, 1) * p.y + (*this)(2, 2) * p.z + (*this)(2, 3))};
    }
    Vec3 vector(const Vec3& v) const {
        return {float((*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z),
                float((*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z),
                float((*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z)};
    }
};

struct TRS {
    double t[3] = {0, 0, 0};
    double r[4] = {0, 0, 0, 1};  // x y z w
    double s[3] = {1, 1, 1};
    bool hasMatrix = false;
    Mat4 matrix;

    Mat4 toMatrix() const {
        if (hasMatrix) return matrix;
        const double x = r[0], y = r[1], z = r[2], w = r[3];
        Mat4 o;
        o(0, 0) = (1 - 2 * (y * y + z * z)) * s[0];
        o(1, 0) = (2 * (x * y + z * w)) * s[0];
        o(2, 0) = (2 * (x * z - y * w)) * s[0];
        o(0, 1) = (2 * (x * y - z * w)) * s[1];
        o(1, 1) = (1 - 2 * (x * x + z * z)) * s[1];
        o(2, 1) = (2 * (y * z + x * w)) * s[1];
        o(0, 2) = (2 * (x * z + y * w)) * s[2];
        o(1, 2) = (2 * (y * z - x * w)) * s[2];
        o(2, 2) = (1 - 2 * (x * x + y * y)) * s[2];
        o(0, 3) = t[0];
        o(1, 3) = t[1];
        o(2, 3) = t[2];
        return o;
    }
};

TRS restTRS(const cgltf_node& n) {
    TRS t;
    if (n.has_matrix) {
        t.hasMatrix = true;
        for (int i = 0; i < 16; ++i) t.matrix.m[size_t(i)] = n.matrix[i];
        return t;
    }
    if (n.has_translation) for (int i = 0; i < 3; ++i) t.t[i] = n.translation[i];
    if (n.has_rotation) for (int i = 0; i < 4; ++i) t.r[i] = n.rotation[i];
    if (n.has_scale) for (int i = 0; i < 3; ++i) t.s[i] = n.scale[i];
    return t;
}

// Splits a matrix into a rigid transform; reports the scale it removed.
Transform rigid(const Mat4& m, double& scaleOut, bool& uniform, bool& mirrored) {
    double s[3];
    for (int c = 0; c < 3; ++c) s[c] = std::sqrt(m(0, c) * m(0, c) + m(1, c) * m(1, c) + m(2, c) * m(2, c));
    const double mean = (s[0] + s[1] + s[2]) / 3.0;
    uniform = mean > 0 && std::max({std::abs(s[0] - mean), std::abs(s[1] - mean), std::abs(s[2] - mean)}) <= 1e-3 * mean;
    scaleOut = mean;
    double r[3][3];
    for (int rr = 0; rr < 3; ++rr)
        for (int c = 0; c < 3; ++c) r[rr][c] = s[c] > 0 ? m(rr, c) / s[c] : (rr == c ? 1 : 0);
    const double det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                       r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    mirrored = det < 0;
    // Rotation matrix -> quaternion (Shepperd).
    double qw, qx, qy, qz;
    const double tr = r[0][0] + r[1][1] + r[2][2];
    if (tr > 0) {
        const double k = std::sqrt(tr + 1.0) * 2;
        qw = 0.25 * k, qx = (r[2][1] - r[1][2]) / k, qy = (r[0][2] - r[2][0]) / k, qz = (r[1][0] - r[0][1]) / k;
    } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        const double k = std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2;
        qw = (r[2][1] - r[1][2]) / k, qx = 0.25 * k, qy = (r[0][1] + r[1][0]) / k, qz = (r[0][2] + r[2][0]) / k;
    } else if (r[1][1] > r[2][2]) {
        const double k = std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2;
        qw = (r[0][2] - r[2][0]) / k, qx = (r[0][1] + r[1][0]) / k, qy = 0.25 * k, qz = (r[1][2] + r[2][1]) / k;
    } else {
        const double k = std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2;
        qw = (r[1][0] - r[0][1]) / k, qx = (r[0][2] + r[2][0]) / k, qy = (r[1][2] + r[2][1]) / k, qz = 0.25 * k;
    }
    return {{float(m(0, 3)), float(m(1, 3)), float(m(2, 3))}, Quat{float(qx), float(qy), float(qz), float(qw)}.normalized()};
}

// Keyframe sampling for one animation channel.
void sampleChannel(const cgltf_animation_channel& ch, double t, TRS& out) {
    const cgltf_animation_sampler& smp = *ch.sampler;
    const cgltf_accessor& in = *smp.input;
    const size_t n = in.count;
    if (n == 0) return;
    const int comps = ch.target_path == cgltf_animation_path_type_rotation ? 4 : 3;
    const bool cubic = smp.interpolation == cgltf_interpolation_type_cubic_spline;
    auto key = [&](size_t i, float* v) {  // Value (not tangent) of key i.
        cgltf_accessor_read_float(smp.output, cubic ? i * 3 + 1 : i, v, size_t(comps));
    };
    float t0 = 0, tN = 0;
    cgltf_accessor_read_float(&in, 0, &t0, 1);
    cgltf_accessor_read_float(&in, n - 1, &tN, 1);
    float v[4] = {0, 0, 0, 1};
    if (n == 1 || t <= t0) {
        key(0, v);
    } else if (t >= tN) {
        key(n - 1, v);
    } else {
        size_t lo = 0, hi = n - 1;  // Binary search for t in [tk, tk+1).
        while (hi - lo > 1) {
            const size_t mid = (lo + hi) / 2;
            float tm;
            cgltf_accessor_read_float(&in, mid, &tm, 1);
            (tm <= t ? lo : hi) = mid;
        }
        float ta, tb;
        cgltf_accessor_read_float(&in, lo, &ta, 1);
        cgltf_accessor_read_float(&in, hi, &tb, 1);
        const double dt = double(tb) - double(ta);
        const double u = dt > 0 ? (t - ta) / dt : 0.0;
        float a[4] = {0, 0, 0, 1}, b[4] = {0, 0, 0, 1};
        key(lo, a);
        key(hi, b);
        if (smp.interpolation == cgltf_interpolation_type_step) {
            std::copy(a, a + 4, v);
        } else if (cubic) {
            float outTan[4] = {0}, inTan[4] = {0};
            cgltf_accessor_read_float(smp.output, lo * 3 + 2, outTan, size_t(comps));
            cgltf_accessor_read_float(smp.output, hi * 3 + 0, inTan, size_t(comps));
            const double u2 = u * u, u3 = u2 * u;
            for (int c = 0; c < comps; ++c)
                v[c] = float((2 * u3 - 3 * u2 + 1) * a[c] + (u3 - 2 * u2 + u) * dt * outTan[c] + (-2 * u3 + 3 * u2) * b[c] +
                             (u3 - u2) * dt * inTan[c]);
        } else if (comps == 4) {
            const Quat q = core::slerp({a[0], a[1], a[2], a[3]}, {b[0], b[1], b[2], b[3]}, float(u));
            v[0] = q.x, v[1] = q.y, v[2] = q.z, v[3] = q.w;
        } else {
            for (int c = 0; c < 3; ++c) v[c] = float(a[c] + (b[c] - a[c]) * u);
        }
    }
    switch (ch.target_path) {
        case cgltf_animation_path_type_translation: for (int i = 0; i < 3; ++i) out.t[i] = v[i]; break;
        case cgltf_animation_path_type_scale: for (int i = 0; i < 3; ++i) out.s[i] = v[i]; break;
        case cgltf_animation_path_type_rotation: {
            const double len = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2] + double(v[3]) * v[3]);
            for (int i = 0; i < 4; ++i) out.r[i] = len > 0 ? v[i] / len : (i == 3 ? 1 : 0);
            break;
        }
        default: break;
    }
}

struct Graph {
    const cgltf_data* data;
    std::vector<TRS> local;  // Per node.

    size_t index(const cgltf_node* n) const { return size_t(n - data->nodes); }
    Mat4 global(const cgltf_node* n, std::vector<int>& done, std::vector<Mat4>& cache) const {
        const size_t i = index(n);
        if (done[i]) return cache[i];
        Mat4 m = local[i].toMatrix();
        if (n->parent) m = global(n->parent, done, cache) * m;
        done[i] = 1;
        cache[i] = m;
        return m;
    }
};

}  // namespace

void scaleCharacter(ImportedCharacter& c, float f) {
    for (Bone& b : c.skeleton.bones) b.restLocal.position = b.restLocal.position * f;
    for (Transform& t : c.restModel) t.position = t.position * f;
    for (SkinnedMesh& m : c.meshes) {
        for (Vec3& p : m.positions) p = p * f;
        for (Transform& t : m.inverseBind) t.position = t.position * f;
    }
    for (AnimationClip& clip : c.clips)
        for (LocalPose& pose : clip.frames)
            for (Transform& t : pose) t.position = t.position * f;
    c.boundsMin = c.boundsMin * f;
    c.boundsMax = c.boundsMax * f;
}

core::Result<ImportedCharacter> importGltf(const std::string& path, const GltfImportOptions& opt) {
    cgltf_options options{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&options, path.c_str(), &data) != cgltf_result_success)
        return R::failure(ErrorCode::ParseError, "Cannot read glTF file: " + path);
    struct Free {
        cgltf_data* d;
        ~Free() { cgltf_free(d); }
    } guard{data};
    if (cgltf_load_buffers(&options, data, path.c_str()) != cgltf_result_success)
        return R::failure(ErrorCode::ParseError, "Cannot load glTF buffers (missing .bin file?)");
    if (cgltf_validate(data) != cgltf_result_success) return R::failure(ErrorCode::ParseError, "glTF file failed validation");
    if (data->skins_count == 0) return R::failure(ErrorCode::Unsupported, "The file has no skin (no skeleton to import)");

    ImportedCharacter out;
    out.sourcePath = path;
    if (data->skins_count > 1) out.warnings.push_back("File has " + std::to_string(data->skins_count) + " skins; only the first is imported");
    const cgltf_skin& skin = data->skins[0];
    if (skin.joints_count == 0) return R::failure(ErrorCode::ParseError, "Skin has no joints");
    if (int(skin.joints_count) > opt.maxBones)
        return R::failure(ErrorCode::Unsupported, "Skin has " + std::to_string(skin.joints_count) + " joints (limit " + std::to_string(opt.maxBones) + ")");

    Graph g{data, {}};
    for (size_t i = 0; i < data->nodes_count; ++i) g.local.push_back(restTRS(data->nodes[i]));

    // Bones: skin joints sorted by node depth (parents first). Parent = nearest joint ancestor.
    std::map<const cgltf_node*, int> jointIndex;  // glTF skin joint index
    for (size_t j = 0; j < skin.joints_count; ++j) jointIndex[skin.joints[j]] = int(j);
    auto depth = [](const cgltf_node* n) {
        int d = 0;
        for (; n->parent; n = n->parent) ++d;
        return d;
    };
    std::vector<int> order(skin.joints_count);
    for (size_t j = 0; j < order.size(); ++j) order[j] = int(j);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return depth(skin.joints[a]) < depth(skin.joints[b]); });
    std::vector<int> boneOfJoint(skin.joints_count);
    for (size_t b = 0; b < order.size(); ++b) boneOfJoint[size_t(order[b])] = int(b);

    std::vector<int> doneRest(data->nodes_count, 0);
    std::vector<Mat4> cacheRest(data->nodes_count);
    std::vector<Mat4> globalRest(skin.joints_count);
    bool nonUniform = false, mirror = false;
    for (size_t b = 0; b < order.size(); ++b) {
        const cgltf_node* n = skin.joints[order[b]];
        int parent = -1;
        for (const cgltf_node* a = n->parent; a && parent < 0; a = a->parent) {
            auto it = jointIndex.find(a);
            if (it != jointIndex.end()) parent = boneOfJoint[size_t(it->second)];
        }
        globalRest[size_t(order[b])] = g.global(n, doneRest, cacheRest);
        double s;
        bool uniform, mirrored;
        const Transform t = rigid(globalRest[size_t(order[b])], s, uniform, mirrored);
        nonUniform |= !uniform;
        mirror |= mirrored;
        out.restModel.push_back(t);
        out.skeleton.bones.push_back({n->name ? n->name : "joint" + std::to_string(order[b]), parent, {}});
    }
    if (nonUniform) out.warnings.push_back("Non-uniform scale on joints is not supported; it was ignored");
    if (mirror) out.warnings.push_back("Mirrored (negative-scale) joints are not supported; results may be wrong");
    const LocalPose restLocal = toLocal(out.skeleton, out.restModel);
    for (size_t b = 0; b < restLocal.size(); ++b) out.skeleton.bones[b].restLocal = restLocal[b];

    // Inverse bind matrices from the file (identity if absent).
    std::vector<Mat4> invBind(skin.joints_count);
    if (skin.inverse_bind_matrices)
        for (size_t j = 0; j < skin.joints_count; ++j) {
            float m[16];
            cgltf_accessor_read_float(skin.inverse_bind_matrices, j, m, 16);
            for (int k = 0; k < 16; ++k) invBind[j].m[size_t(k)] = m[k];
        }
    std::vector<Mat4> jointRest(skin.joints_count);  // Skinning matrix at rest = G_rest * IBM.
    for (size_t j = 0; j < skin.joints_count; ++j) jointRest[j] = globalRest[j] * invBind[j];

    // Meshes on nodes that use this skin, skinned at rest into model space.
    bool restIsNotBind = false;
    for (size_t ni = 0; ni < data->nodes_count; ++ni) {
        const cgltf_node& node = data->nodes[ni];
        if (node.skin != &skin || !node.mesh) continue;
        for (size_t pi = 0; pi < node.mesh->primitives_count; ++pi) {
            const cgltf_primitive& prim = node.mesh->primitives[pi];
            if (prim.type != cgltf_primitive_type_triangles) {
                out.warnings.push_back("Skipped a non-triangle primitive");
                continue;
            }
            const cgltf_accessor *pos = nullptr, *nrm = nullptr, *jnt = nullptr, *wgt = nullptr;
            for (size_t a = 0; a < prim.attributes_count; ++a) {
                const cgltf_attribute& at = prim.attributes[a];
                if (at.type == cgltf_attribute_type_position) pos = at.data;
                if (at.type == cgltf_attribute_type_normal) nrm = at.data;
                if (at.type == cgltf_attribute_type_joints && at.index == 0) jnt = at.data;
                if (at.type == cgltf_attribute_type_weights && at.index == 0) wgt = at.data;
                if ((at.type == cgltf_attribute_type_joints || at.type == cgltf_attribute_type_weights) && at.index > 0 && pi == 0)
                    out.warnings.push_back("More than 4 bone influences per vertex: extra influences ignored");
            }
            if (!pos || !jnt || !wgt) {
                out.warnings.push_back("Skipped a primitive without POSITION/JOINTS_0/WEIGHTS_0");
                continue;
            }
            SkinnedMesh mesh;
            mesh.name = node.mesh->name ? node.mesh->name : "mesh";
            const size_t count = pos->count;
            mesh.positions.resize(count);
            mesh.joints.resize(count);
            mesh.weights.resize(count);
            if (nrm) mesh.normals.resize(count);
            for (size_t v = 0; v < count; ++v) {
                float p[3], n[3] = {0, 0, 1}, w[4];
                cgltf_uint j[4];
                cgltf_accessor_read_float(pos, v, p, 3);
                if (nrm) cgltf_accessor_read_float(nrm, v, n, 3);
                cgltf_accessor_read_uint(jnt, v, j, 4);
                cgltf_accessor_read_float(wgt, v, w, 4);
                const float sum = w[0] + w[1] + w[2] + w[3];
                Vec3 bp{}, bn{};
                for (int k = 0; k < 4; ++k) {
                    const float wk = sum > 0 ? w[k] / sum : (k == 0 ? 1.0f : 0.0f);
                    const size_t gj = std::min<size_t>(j[k], skin.joints_count - 1);
                    mesh.joints[v][size_t(k)] = uint16_t(boneOfJoint[gj]);
                    mesh.weights[v][size_t(k)] = wk;
                    if (wk <= 0) continue;
                    bp = bp + jointRest[gj].point({p[0], p[1], p[2]}) * wk;
                    bn = bn + jointRest[gj].vector({n[0], n[1], n[2]}) * wk;
                    // Joints influencing one vertex must agree at rest, or LBS here is approximate.
                    const Mat4& j0 = jointRest[std::min<size_t>(j[0], skin.joints_count - 1)];
                    for (int e = 0; e < 16 && !restIsNotBind; ++e)
                        if (std::abs(jointRest[gj].m[size_t(e)] - j0.m[size_t(e)]) > 1e-3 * (1.0 + std::abs(j0.m[size_t(e)])))
                            restIsNotBind = true;
                }
                mesh.positions[v] = bp;
                if (nrm) mesh.normals[v] = core::normalized(bn);
            }
            if (prim.indices) {
                mesh.indices.resize(prim.indices->count);
                for (size_t i = 0; i < prim.indices->count; ++i) mesh.indices[i] = uint32_t(cgltf_accessor_read_index(prim.indices, i));
            } else {
                for (uint32_t i = 0; i < count; ++i) mesh.indices.push_back(i);
            }
            mesh.inverseBind.resize(out.restModel.size());
            for (size_t b = 0; b < out.restModel.size(); ++b) mesh.inverseBind[b] = out.restModel[b].inverse();
            out.meshes.push_back(std::move(mesh));
        }
    }
    if (out.meshes.empty()) return R::failure(ErrorCode::Unsupported, "No skinned triangle mesh uses the skeleton");
    if (restIsNotBind)
        out.warnings.push_back("The rest pose differs from the bind pose for some joints; skinning is approximated");

    out.boundsMin = {1e30f, 1e30f, 1e30f};
    out.boundsMax = {-1e30f, -1e30f, -1e30f};
    for (const SkinnedMesh& m : out.meshes)
        for (const Vec3& p : m.positions) {
            out.boundsMin = {std::min(out.boundsMin.x, p.x), std::min(out.boundsMin.y, p.y), std::min(out.boundsMin.z, p.z)};
            out.boundsMax = {std::max(out.boundsMax.x, p.x), std::max(out.boundsMax.y, p.y), std::max(out.boundsMax.z, p.z)};
        }

    // Animations: evaluate the whole node graph per frame, then convert joints to bone poses.
    bool morphIgnored = false;
    for (size_t ai = 0; ai < data->animations_count; ++ai) {
        const cgltf_animation& anim = data->animations[ai];
        AnimationClip clip;
        clip.name = anim.name ? anim.name : "Animation " + std::to_string(ai + 1);
        std::set<double> times{0.0};
        for (size_t c = 0; c < anim.channels_count; ++c) {
            const cgltf_animation_channel& ch = anim.channels[c];
            if (ch.target_path == cgltf_animation_path_type_weights) morphIgnored = true;
            if (!ch.sampler || !ch.sampler->input || ch.sampler->input->count == 0) continue;
            const cgltf_accessor& in = *ch.sampler->input;
            const bool step = ch.sampler->interpolation == cgltf_interpolation_type_step;
            const bool cubic = ch.sampler->interpolation == cgltf_interpolation_type_cubic_spline;
            float prev = 0.0f;
            for (size_t k = 0; k < in.count; ++k) {
                float tk;
                cgltf_accessor_read_float(&in, k, &tk, 1);
                times.insert(double(tk));
                if (k > 0 && step) times.insert(std::max(double(prev), double(tk) - 1e-4));  // Hold until the next key.
                if (k > 0 && cubic)
                    for (double x = double(prev) + 1.0 / opt.curveSampleRate; x < double(tk); x += 1.0 / opt.curveSampleRate) times.insert(x);
                prev = tk;
            }
            clip.duration = std::max(clip.duration, double(prev));
        }
        for (const double t : times) {
            Graph frame = g;
            for (size_t c = 0; c < anim.channels_count; ++c) {
                const cgltf_animation_channel& ch = anim.channels[c];
                if (!ch.target_node || !ch.sampler || ch.target_path == cgltf_animation_path_type_weights) continue;
                TRS& trs = frame.local[frame.index(ch.target_node)];
                trs.hasMatrix = false;  // Animated nodes use TRS (glTF forbids matrix + animation).
                sampleChannel(ch, t, trs);
            }
            std::vector<int> done(data->nodes_count, 0);
            std::vector<Mat4> cache(data->nodes_count);
            ModelPose model(order.size());
            for (size_t b = 0; b < order.size(); ++b) {
                double s;
                bool uniform, mirrored;
                model[b] = rigid(frame.global(skin.joints[order[b]], done, cache), s, uniform, mirrored);
            }
            clip.times.push_back(t);
            clip.frames.push_back(toLocal(out.skeleton, model));
        }
        out.clips.push_back(std::move(clip));
    }
    if (morphIgnored) out.warnings.push_back("Morph-target (blend shape) animation is not supported and was ignored");
    return R::success(std::move(out));
}

}  // namespace vexora::rig
