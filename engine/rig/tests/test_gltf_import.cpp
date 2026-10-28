// Rig test scenes: glTF import, skinning and animation against an independent reference.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <cgltf.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>

#include "vexora/rig/GltfImport.h"

using namespace vexora;
using namespace vexora::rig;

#define VX_REPORT(...) std::printf("  [measure] " __VA_ARGS__), std::printf("\n")

namespace {

std::string sample(const char* name) { return std::string(VEXORA_SAMPLES_DIR) + "/" + name; }

ImportedCharacter load(const char* name) {
    auto r = importGltf(sample(name));
    INFO(r.message);
    REQUIRE(r.ok());
    for (const std::string& w : r.value.warnings) VX_REPORT("%s warning: %s", name, w.c_str());
    return std::move(r.value);
}

// Reference: skin the file's original vertices with glTF's own math at a keyframe of the first
// animation (node TRS set to the key values, joint matrix = world * inverseBind), using only
// cgltf functions. Returns the max distance to Vexora's result (our pose -> our skinning).
float compareWithReference(const char* name, size_t keyIndex, double& keyTime) {
    const ImportedCharacter ch = load(name);
    cgltf_options opt{};
    cgltf_data* data = nullptr;
    REQUIRE(cgltf_parse_file(&opt, sample(name).c_str(), &data) == cgltf_result_success);
    REQUIRE(cgltf_load_buffers(&opt, data, sample(name).c_str()) == cgltf_result_success);
    const cgltf_animation& anim = data->animations[0];
    // Use the key times of the first channel; all channels of these samples share their times.
    const cgltf_accessor* times = anim.channels[0].sampler->input;
    keyIndex = std::min(keyIndex, times->count - 1);
    float t;
    cgltf_accessor_read_float(times, keyIndex, &t, 1);
    keyTime = t;
    for (size_t c = 0; c < anim.channels_count; ++c) {
        const cgltf_animation_channel& chn = anim.channels[c];
        float in;
        cgltf_accessor_read_float(chn.sampler->input, std::min(keyIndex, chn.sampler->input->count - 1), &in, 1);
        REQUIRE(std::abs(in - t) < 1e-5f);  // Precondition of this reference.
        const size_t k = std::min(keyIndex, chn.sampler->output->count - 1);
        cgltf_node* n = chn.target_node;
        if (chn.target_path == cgltf_animation_path_type_translation) cgltf_accessor_read_float(chn.sampler->output, k, n->translation, 3), n->has_translation = 1;
        if (chn.target_path == cgltf_animation_path_type_rotation) cgltf_accessor_read_float(chn.sampler->output, k, n->rotation, 4), n->has_rotation = 1;
        if (chn.target_path == cgltf_animation_path_type_scale) cgltf_accessor_read_float(chn.sampler->output, k, n->scale, 3), n->has_scale = 1;
    }
    const cgltf_skin& skin = data->skins[0];
    std::vector<std::array<float, 16>> joint(skin.joints_count);
    for (size_t j = 0; j < skin.joints_count; ++j) {
        float w[16], ib[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        cgltf_node_transform_world(skin.joints[j], w);
        if (skin.inverse_bind_matrices) cgltf_accessor_read_float(skin.inverse_bind_matrices, j, ib, 16);
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += w[k * 4 + r] * ib[c * 4 + k];
                joint[j][size_t(c * 4 + r)] = s;
            }
    }
    // Our side: pose at the key time, then skin.
    const LocalPose local = rig::sample(ch.clips[0], keyTime, false);
    std::vector<Vec3> ours;
    rig::skin(ch.meshes[0], toModel(ch.skeleton, local), ours);

    float worst = 0.0f;
    size_t meshIdx = 0;
    for (size_t ni = 0; ni < data->nodes_count && meshIdx == 0; ++ni) {
        const cgltf_node& node = data->nodes[ni];
        if (node.skin != &skin || !node.mesh) continue;
        const cgltf_primitive& prim = node.mesh->primitives[0];
        const cgltf_accessor *pos = nullptr, *jnt = nullptr, *wgt = nullptr;
        for (size_t a = 0; a < prim.attributes_count; ++a) {
            if (prim.attributes[a].type == cgltf_attribute_type_position) pos = prim.attributes[a].data;
            if (prim.attributes[a].type == cgltf_attribute_type_joints) jnt = prim.attributes[a].data;
            if (prim.attributes[a].type == cgltf_attribute_type_weights) wgt = prim.attributes[a].data;
        }
        REQUIRE(pos->count == ours.size());
        for (size_t v = 0; v < pos->count; ++v) {
            float p[3], w[4];
            cgltf_uint j[4];
            cgltf_accessor_read_float(pos, v, p, 3);
            cgltf_accessor_read_uint(jnt, v, j, 4);
            cgltf_accessor_read_float(wgt, v, w, 4);
            const float sum = w[0] + w[1] + w[2] + w[3];
            float ref[3] = {0, 0, 0};
            for (int k = 0; k < 4; ++k) {
                const auto& m = joint[j[k]];
                for (int r = 0; r < 3; ++r) ref[r] += w[k] / sum * (m[size_t(r)] * p[0] + m[size_t(4 + r)] * p[1] + m[size_t(8 + r)] * p[2] + m[size_t(12 + r)]);
            }
            const Vec3 d = ours[v] - Vec3{ref[0], ref[1], ref[2]};
            worst = std::max(worst, d.length());
        }
        ++meshIdx;
    }
    cgltf_free(data);
    return worst;
}

}  // namespace

TEST_CASE("Rig: RiggedSimple imports with a parents-first skeleton and one clip") {
    const ImportedCharacter c = load("RiggedSimple.glb");
    CHECK(c.skeleton.bones.size() == 2);
    CHECK(c.skeleton.validate().ok());
    CHECK(c.clips.size() == 1);
    CHECK(c.meshes.size() == 1);
    for (size_t b = 0; b < c.skeleton.bones.size(); ++b) CHECK(c.skeleton.bones[b].parent < int(b));
    const ModelPose fk = toModel(c.skeleton, restPose(c.skeleton));
    for (size_t b = 0; b < fk.size(); ++b) CHECK((fk[b].position - c.restModel[b].position).length() < 1e-4f);
}

TEST_CASE("Rig: skinning at the rest pose reproduces the rest mesh exactly") {
    for (const char* name : {"RiggedSimple.glb", "RiggedFigure.glb", "Fox.glb"}) {
        const ImportedCharacter c = load(name);
        std::vector<Vec3> out;
        float worst = 0.0f;
        for (const SkinnedMesh& m : c.meshes) {
            skin(m, c.restModel, out);
            for (size_t v = 0; v < out.size(); ++v) worst = std::max(worst, (out[v] - m.positions[v]).length());
        }
        const float height = c.boundsMax.y - c.boundsMin.y;
        VX_REPORT("%s: %zu bones, %zu clips, height %.3f, rest skinning error %.2e (relative %.2e)", name,
                  c.skeleton.bones.size(), c.clips.size(), height, worst, worst / height);
        CAPTURE(name);
        CHECK(worst / height < 1e-5f);
    }
}

TEST_CASE("Rig: animated skinning matches an independent glTF reference at keyframes") {
    for (const char* name : {"RiggedSimple.glb", "RiggedFigure.glb", "Fox.glb"}) {
        const ImportedCharacter c = load(name);
        const float height = c.boundsMax.y - c.boundsMin.y;
        for (size_t key : {size_t(0), size_t(3), size_t(10)}) {
            double t = 0.0;
            const float err = compareWithReference(name, key, t);
            VX_REPORT("%s key %zu (t=%.3f s): max vertex error %.2e (relative to height %.2e)", name, key, t, err, err / height);
            CAPTURE(name);
            CAPTURE(key);
            CHECK(err / height < 1e-3f);
        }
    }
}

TEST_CASE("Rig: Fox keeps its named clips; looping sampling wraps") {
    const ImportedCharacter c = load("Fox.glb");
    REQUIRE(c.clips.size() == 3);
    CHECK(c.clips[0].name == "Survey");
    CHECK(c.clips[1].name == "Walk");
    CHECK(c.clips[2].name == "Run");
    const AnimationClip& walk = c.clips[1];
    const LocalPose a = rig::sample(walk, 0.3, true), b = rig::sample(walk, 0.3 + walk.duration, true);
    float worst = 0.0f;
    for (size_t i = 0; i < a.size(); ++i) worst = std::max(worst, core::angleBetween(a[i].rotation, b[i].rotation));
    CHECK(worst < 1e-4f);
    // Clamped sampling past the end returns the last frame.
    const LocalPose end = rig::sample(walk, walk.duration + 5.0, false);
    CHECK(core::angleBetween(end[1].rotation, walk.frames.back()[1].rotation) < 1e-5f);
}

TEST_CASE("Rig: missing or unsuitable files fail with a reason") {
    auto missing = importGltf("does/not/exist.glb");
    CHECK(missing.error == core::ErrorCode::ParseError);
    const std::string path = std::string(VEXORA_TEST_TMP) + "/no_skin.gltf";
    std::ofstream(path) << R"({"asset":{"version":"2.0"},"nodes":[{"name":"empty"}],"scenes":[{"nodes":[0]}]})";
    auto noSkin = importGltf(path);
    CHECK(noSkin.error == core::ErrorCode::Unsupported);
    CHECK(noSkin.message.find("skin") != std::string::npos);
}

TEST_CASE("Rig: truncated and corrupted files are rejected without crashing") {
    std::ifstream in(sample("Fox.glb"), std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    int rejected = 0, accepted = 0;
    for (size_t cut : {size_t(8), size_t(20), size_t(500), size_t(5000), bytes.size() / 2, bytes.size() - 10}) {
        const std::string path = std::string(VEXORA_TEST_TMP) + "/truncated.glb";
        std::ofstream(path, std::ios::binary).write(bytes.data(), std::streamsize(cut));
        (importGltf(path).ok() ? accepted : rejected)++;
    }
    // Flip bytes inside the binary chunk: import may succeed (data is just numbers) but must not crash.
    std::string corrupt = bytes;
    for (size_t i = corrupt.size() / 3; i < corrupt.size(); i += 97) corrupt[i] = char(~corrupt[i]);
    const std::string path = std::string(VEXORA_TEST_TMP) + "/corrupt.glb";
    std::ofstream(path, std::ios::binary).write(corrupt.data(), std::streamsize(corrupt.size()));
    auto r = importGltf(path);
    VX_REPORT("truncated files: %d rejected, %d accepted; byte-flipped file: %s", rejected, accepted, r.ok() ? "imported" : r.message.c_str());
    CHECK(accepted == 0);
}
