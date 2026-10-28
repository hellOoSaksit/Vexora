// Imported glTF character scene: animation playback -> ragdoll -> blend back to animation.
#include <algorithm>
#include <cmath>
#include <filesystem>

#include "Base64.h"
#include "Session.h"

namespace vexora::sim {

using namespace vexora::physics;

namespace {

constexpr uint32_t kCharacterColor = 0xd9a77a;

std::string fmtNumber(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    return buf;
}

}  // namespace

// Binary payload inside JSON: {"$f32"|"$u16"|"$u32": base64}; decoded by the app's main process.
json tagged(const char* type, const void* data, size_t bytes) { return {{type, base64(data, bytes)}}; }

bool Session::buildCharacter(const json& p, std::string& error) {
    const std::string model = p["model"].get<std::string>();
    std::string path = model == "custom" ? p["customFile"].get<std::string>() : samplesDir() + "/" + model + ".glb";
    if (path.empty()) {
        error = "ยังไม่ได้เลือกไฟล์ตัวละคร — กด 'เลือกไฟล์…' ในช่อง 'ไฟล์ตัวละคร'";
        return false;
    }
    auto imported = rig::importGltf(path);
    if (!imported.ok()) {
        error = "โหลดไฟล์ตัวละครไม่ได้: " + imported.message;
        return false;
    }
    character_ = std::move(imported.value);
    importNotes_ = character_.warnings;

    // Size: physics needs real-world units.
    const float fileHeight = character_.boundsMax.y - character_.boundsMin.y;
    const std::string sizeMode = p["size"].get<std::string>();
    float target = fileHeight;
    if (sizeMode == "fit") target = float(p["height"].get<double>());
    else if (sizeMode == "auto" && (fileHeight < 0.3f || fileHeight > 3.0f)) target = 1.7f;
    characterScale_ = fileHeight > 0.0f ? target / fileHeight : 1.0f;
    if (std::abs(characterScale_ - 1.0f) > 1e-4f) {
        rig::scaleCharacter(character_, characterScale_);
        importNotes_.push_back("ปรับขนาดจาก " + fmtNumber(fileHeight) + " เป็น " + fmtNumber(target) + " เมตร");
    }
    if (character_.clips.empty()) importNotes_.push_back("ไฟล์นี้ไม่มี Animation — ตัวละครจะยืนท่าเริ่มต้น");
    clipIndex_ = character_.clips.empty() ? 0 : std::min(size_t(p["clip"].get<double>()) - 1, character_.clips.size() - 1);
    clipTime_ = 0.0;

    // Ragdoll from the skeleton; bones shorter than 3% of the height are folded into their parent.
    const RagdollSettings settings = makeAutoRagdoll(character_.skeleton, character_.restModel, 0.03f * target);
    if (settings.parts.empty()) {
        error = "โครงกระดูกในไฟล์สั้นเกินไปจนสร้าง ragdoll ไม่ได้";
        return false;
    }
    Actor& a = characterActor_;
    a = Actor{};
    a.root = {{0.0f, -character_.boundsMin.y + 0.01f, 0.0f}, {}};  // Feet on the floor.
    const ModelPose start = toModel(character_.skeleton, characterAnimPose());
    auto rd = Ragdoll::create(*world_, character_.skeleton, settings, start, a.root);
    if (!rd.ok()) {
        error = "สร้าง ragdoll จากโครงกระดูกไม่ได้: " + rd.message;
        return false;
    }
    a.ragdoll = std::move(rd.value);
    a.ragdoll->setDrive(BoneDrive::Animated);
    a.state = "animated";
    if (p["showRagdoll"].get<bool>())
        for (size_t k = 0; k < a.ragdoll->partCount(); ++k)
            bodies_.push_back({a.ragdoll->body(k), "capsule", {a.ragdoll->partRadius(k), a.ragdoll->partHalfHeight(k), 0}, 0x7cc4ff, true});
    characterPose_ = start;
    hasCharacter_ = true;
    return true;
}

LocalPose Session::characterAnimPose() const {
    if (character_.clips.empty()) return restPose(character_.skeleton);
    return rig::sample(character_.clips[clipIndex_], clipTime_, true);
}

void Session::stepCharacter(double dt) {
    Actor& a = characterActor_;
    clipTime_ += dt * params_["playbackSpeed"].get<double>();
    const LocalPose anim = characterAnimPose();
    if (a.state == "animated") {
        a.ragdoll->drive(toModel(character_.skeleton, anim), a.root);
    } else if (a.state == "gettingup") {
        a.blendTime += dt;
        const float w = float(std::clamp(1.0 - a.blendTime / params_["getUpTime"].get<double>(), 0.0, 1.0));
        const LocalPose blended = blendPoses(character_.skeleton, anim, a.blendFrom, std::vector<float>(anim.size(), w));
        a.ragdoll->drive(toModel(character_.skeleton, blended), a.root);
        if (w <= 0.0f) a.state = "animated";
    }
}

}  // namespace vexora::sim
