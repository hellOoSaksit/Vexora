// Character scenes: ragdoll (animation <-> physics) and spring tail (secondary motion).
#include <algorithm>
#include <cmath>

#include "Session.h"

namespace vexora::sim {

using namespace vexora::physics;

namespace {

constexpr uint32_t kShirt = 0x4f7fd0, kSkin = 0xe0a080, kTrousers = 0x3b4250, kTailBody = 0xc08a4a;

uint32_t partColor(const std::string& bone) {
    if (bone == "pelvis" || bone == "spine" || bone == "chest") return kShirt;
    if (bone.rfind("thigh", 0) == 0 || bone.rfind("shin", 0) == 0) return kTrousers;
    return kSkin;
}

Quat yaw(float angle) { return Quat::fromAxisAngle({0, 1, 0}, angle); }

}  // namespace

int Session::addVirtual(const std::string& shape, Vec3 size, uint32_t color) {
    virtualXf_.push_back({});
    bodies_.push_back({{}, shape, size, color, true, int(virtualXf_.size() - 1)});
    return int(virtualXf_.size() - 1);
}

Transform Session::transformOf(const RenderBody& b) const {
    return b.virtualIndex >= 0 ? virtualXf_[size_t(b.virtualIndex)] : world_->getTransform(b.handle);
}

// Procedural idle "animation" until Vexora has an animation system: arms down, gentle sway.
LocalPose Session::idlePose(double t) const {
    const Skeleton& sk = human_.skeleton;
    LocalPose pose = restPose(sk);
    const float sway = 0.12f * float(std::sin(t * 1.6));
    pose[size_t(sk.find("upperArm.L"))].rotation = Quat::fromAxisAngle({0, 0, 1}, -1.25f + sway);
    pose[size_t(sk.find("upperArm.R"))].rotation = Quat::fromAxisAngle({0, 0, 1}, 1.25f - sway);
    pose[size_t(sk.find("lowerArm.L"))].rotation = Quat::fromAxisAngle({0, 1, 0}, -0.3f);
    pose[size_t(sk.find("lowerArm.R"))].rotation = Quat::fromAxisAngle({0, 1, 0}, 0.3f);
    pose[size_t(sk.find("spine"))].rotation = Quat::fromAxisAngle({1, 0, 0}, 0.03f * float(std::sin(t * 2.0)));
    return pose;
}

bool Session::buildRagdoll(const json& p, std::string& error) {
    human_ = makeDemoHumanoid();
    human_.ragdoll.motorFrequency = float(p["motorStrength"].get<double>());
    const int count = int(p["count"].get<double>());
    const bool drop = p["start"].get<std::string>() == "drop";
    const ModelPose pose = toModel(human_.skeleton, idlePose(0.0));
    for (int i = 0; i < count; ++i) {
        Actor a;
        a.root = {{(float(i) - float(count - 1) * 0.5f) * 1.4f, drop ? float(p["dropHeight"].get<double>()) : 0.0f, 0.0f},
                  drop ? Quat::fromAxisAngle({1, 0, 0}, 0.4f) * yaw(0.3f * float(i)) : Quat{}};
        auto made = Ragdoll::create(*world_, human_.skeleton, human_.ragdoll, pose, a.root);
        if (!made.ok()) {
            error = "สร้าง Ragdoll ไม่ได้: " + made.message;
            return false;
        }
        a.ragdoll = std::move(made.value);
        for (size_t k = 0; k < a.ragdoll->partCount(); ++k) {
            const std::string& bone = human_.skeleton.bones[size_t(human_.ragdoll.parts[k].bone)].name;
            bodies_.push_back({a.ragdoll->body(k), "capsule", {a.ragdoll->partRadius(k), a.ragdoll->partHalfHeight(k), 0},
                               partColor(bone), true});
        }
        a.state = drop ? "ragdoll" : "animated";
        a.ragdoll->setDrive(drop ? BoneDrive::Simulated : BoneDrive::Animated);
        actors_.push_back(std::move(a));
    }
    return true;
}

void Session::buildTail(const json& p) {
    const int segments = int(p["segments"].get<double>());
    const float len = float(p["segmentLength"].get<double>());
    tailSkeleton_ = {};
    tailSkeleton_.bones.push_back({"body", -1, {}});
    for (int i = 0; i < segments; ++i)
        tailSkeleton_.bones.push_back({"tail" + std::to_string(i), i, {{0, 0, i == 0 ? -0.55f : -len}, {}}});
    spring_ = std::make_unique<SecondaryMotion>(tailSkeleton_, Vec3{0, -9.81f, 0});
    SpringChainSettings c;
    for (int i = 1; i <= segments; ++i) c.bones.push_back(i);
    c.tipLength = len;
    c.stiffness = float(p["stiffness"].get<double>());
    c.damping = float(p["damping"].get<double>());
    c.inertia = float(p["inertia"].get<double>());
    c.gravityScale = float(p["gravityScale"].get<double>());
    c.maxAngle = float(p["maxAngle"].get<double>()) * 3.14159265f / 180.0f;
    c.weight = float(p["weight"].get<double>());
    c.radius = 0.05f;
    spring_->addChain(c);

    bodyVirtual_ = addVirtual("box", {0.3f, 0.25f, 0.55f}, kTailBody);
    firstSegmentVirtual_ = -1;
    for (int i = 0; i < segments; ++i) {
        const float radius = 0.07f * (1.0f - 0.6f * float(i) / float(std::max(1, segments)));
        const int v = addVirtual("capsule", {radius, std::max(0.005f, len * 0.5f - radius * 0.5f), 0}, kTailBody);
        if (i == 0) firstSegmentVirtual_ = v;
    }
    tailRoot_ = {{0, 1.0f, 0}, {}};
    const ModelPose anim = toModel(tailSkeleton_, restPose(tailSkeleton_));
    spring_->reset(anim, tailRoot_);
    postStep(0.0);  // Fill the virtual transforms for the first frame.
}

void Session::preStep(double dt) {
    const double t = simTime_;
    for (Actor& a : actors_) {
        const LocalPose idle = idlePose(t);
        if (a.state == "animated" || a.state == "powered") {
            a.ragdoll->drive(toModel(human_.skeleton, idle), a.root);
        } else if (a.state == "gettingup") {
            a.blendTime += dt;
            const double duration = params_["getUpTime"].get<double>();
            const float w = float(std::clamp(1.0 - a.blendTime / duration, 0.0, 1.0));  // Weight of the fallen pose.
            const std::vector<float> weights(idle.size(), w);
            const LocalPose blended = blendPoses(human_.skeleton, idle, a.blendFrom, weights);
            a.ragdoll->drive(toModel(human_.skeleton, blended), a.root);
            if (w <= 0.0f) a.state = "animated";
        }
    }
    if (spring_) {
        // Root motion pattern for the tail's body ("animation" of the character).
        const std::string motion = params_["motion"].get<std::string>();
        const float w = 2.5f * float(params_["speed"].get<double>());
        const float ft = float(t + dt);
        if (motion == "sway") tailRoot_ = {{1.2f * std::sin(ft * w), 1.0f, 0}, yaw(0.5f * std::cos(ft * w))};
        else if (motion == "circle") tailRoot_ = {{3.0f * std::sin(ft * w * 0.4f), 1.0f, 3.0f * std::cos(ft * w * 0.4f)},
                                                 yaw(ft * w * 0.4f + 1.5707963f)};
        else if (motion == "jump") tailRoot_ = {{0, 1.0f + 1.2f * std::abs(std::sin(ft * w * 0.6f)), 0}, {}};
        else tailRoot_ = {{0, 1.0f, 0}, {}};
        // The tail must not pass through its own body or the floor.
        const Vec3 front = tailRoot_.apply({0, 0, 0.45f}), back = tailRoot_.apply({0, 0, -0.45f});
        spring_->setColliders({{front, back, 0.28f}}, {{{0, 1, 0}, 0.0f}});
        spring_->step(dt, toModel(tailSkeleton_, restPose(tailSkeleton_)), tailRoot_);
    }
}

void Session::postStep(double) {
    if (!spring_) return;
    const LocalPose animLocal = restPose(tailSkeleton_);
    const ModelPose anim = toModel(tailSkeleton_, animLocal);
    LocalPose finalLocal = animLocal;
    spring_->apply(anim, tailRoot_, finalLocal);
    const ModelPose finalModel = toModel(tailSkeleton_, finalLocal);
    virtualXf_[size_t(bodyVirtual_)] = tailRoot_;
    const size_t segments = tailSkeleton_.bones.size() - 1;
    const float len = float(params_["segmentLength"].get<double>());
    for (size_t i = 0; i < segments; ++i) {
        const Transform bone = tailRoot_ * finalModel[i + 1];
        const Vec3 head = bone.position;
        const Vec3 tail = i + 1 < segments ? (tailRoot_ * finalModel[i + 2]).position : bone.apply({0, 0, -len});
        virtualXf_[size_t(firstSegmentVirtual_) + i] = {(head + tail) * 0.5f, Quat::fromTo({0, 1, 0}, normalized(tail - head))};
    }
}

const char* Session::characterState() const {
    if (!actors_.empty() || hasCharacter_) {
        const std::string& s = hasCharacter_ ? characterActor_.state : actors_.front().state;
        if (s == "animated") return "ยืน (Animation ควบคุม)";
        if (s == "ragdoll") return "ล้ม (Ragdoll — ฟิสิกส์ควบคุม)";
        if (s == "powered") return "เกร็งตัว (ฟิสิกส์ + มอเตอร์ข้อต่อ)";
        if (s == "gettingup") return "กำลังลุก (ผสมกลับเป็น Animation)";
    }
    if (spring_) return "Secondary motion (ประมาณผลด้วยสปริง)";
    return nullptr;
}

}  // namespace vexora::sim
