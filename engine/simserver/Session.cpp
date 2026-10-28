#include "Session.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <thread>

#include "Base64.h"

namespace vexora::sim {

using namespace vexora::physics;

namespace {

constexpr uint32_t kFloorColor = 0x3a3f47;
constexpr uint32_t kBallColor = 0xf0c040;

MaterialPreset presetFor(const std::string& m) {
    static const std::map<std::string, MaterialPreset> table = {
        {"metal", MaterialPreset::Metal},   {"wood", MaterialPreset::Wood},     {"stone", MaterialPreset::Stone},
        {"rubber", MaterialPreset::Rubber}, {"ice", MaterialPreset::Ice},       {"fabric", MaterialPreset::Fabric},
        {"flesh", MaterialPreset::FleshLike}};
    auto it = table.find(m);
    return it == table.end() ? MaterialPreset::Wood : it->second;
}

uint32_t colorFor(const std::string& m) {
    static const std::map<std::string, uint32_t> table = {{"metal", 0x9aa4b1}, {"wood", 0xb07a45}, {"stone", 0x8c8c86},
                                                          {"rubber", 0xd0453a}, {"ice", 0xa8dcf0},  {"fabric", 0x6a7fd0},
                                                          {"flesh", 0xe0a080}};
    auto it = table.find(m);
    return it == table.end() ? 0xcccccc : it->second;
}

QualityProfile profileFor(const std::string& q) {
    if (q == "high") return QualityProfile::HighQualityPreview;
    if (q == "final") return QualityProfile::FinalBake;
    return QualityProfile::InteractivePreview;
}

int workerThreadsFor(const json& g) {
    const int requested = int(g["threads"].get<double>());
    if (requested > 0) return requested;
    return int(std::max(1u, std::thread::hardware_concurrency())) - 1;  // Leave one core for the UI.
}

const char* warningType(WarningType t) {
    switch (t) {
        case WarningType::NonFiniteState: return "NonFiniteState";
        case WarningType::ExcessiveVelocity: return "ExcessiveVelocity";
        case WarningType::EngineCapacity: return "EngineCapacity";
    }
    return "Unknown";
}

}  // namespace

BodyHandle Session::addBody(const BodyDesc& desc, const std::string& shape, Vec3 size, uint32_t color) {
    Result<BodyHandle> r = world_->createBody(desc);
    if (!r.ok()) return {};
    bodies_.push_back({r.value, shape, size, color, desc.motion != MotionType::Static});
    return r.value;
}

void Session::addFloor() {
    BodyDesc floor;
    floor.motion = MotionType::Static;
    floor.shape = ShapeDesc::box({40.0f, 0.5f, 40.0f});
    floor.transform.position = {0.0f, -0.5f, 0.0f};
    floor.material = makeMaterial(MaterialPreset::Stone);
    addBody(floor, "box", {40.0f, 0.5f, 40.0f}, kFloorColor);
}

bool Session::load(const std::string& sceneId, const json& params, const json& global, std::string& error) {
    world_.reset();
    particles_.reset();
    bodies_.clear();
    chainLinks_.clear();
    reportedBodies_ = 0;
    simTime_ = 0.0;
    warnings_.clear();
    physicsMs_ = particleMs_ = 0.0;
    stepsSinceFrame_ = 0;
    brokenSinceFrame_ = 0;

    WorldSettings ws;
    ws.profile = profileFor(global["quality"].get<std::string>());
    ws.gravity = {0.0f, float(global["gravity"].get<double>()), 0.0f};
    ws.workerThreads = workerThreadsFor(global);
    world_ = std::make_unique<PhysicsWorld>(ws);
    sceneId_ = sceneId;
    params_ = params;

    addFloor();
    if (sceneId == "drop") buildDrop(params, global);
    else if (sceneId == "tower") buildTower(params);
    else if (sceneId == "chain") buildChain(params);
    else if (sceneId == "fountain") {
        if (!buildFountain(params, global, error)) {
            world_.reset();
            bodies_.clear();
            return false;
        }
    } else {
        error = "ไม่รู้จักฉาก '" + sceneId + "'";
        world_.reset();
        return false;
    }
    reportedBodies_ = bodies_.size();  // describeScene() reports these.
    return true;
}

void Session::buildDrop(const json& p, const json&) {
    const int count = int(p["count"].get<double>());
    const float size = float(p["size"].get<double>());
    const float height = float(p["height"].get<double>());
    const float area = float(p["area"].get<double>());
    const std::string shape = p["shape"].get<std::string>();
    const std::string material = p["material"].get<std::string>();
    const PhysicsMaterial mat = makeMaterial(presetFor(material));
    const uint32_t color = colorFor(material);

    // Grid placement so bodies never start overlapping; extra layers go upwards.
    const float spacing = size * 2.6f;
    const int perRow = std::max(1, int(area / spacing));
    std::mt19937 rng(12345);  // Fixed seed: the same settings give the same scene.
    std::uniform_real_distribution<float> jitter(-0.2f, 0.2f), angle(0.0f, 6.2831853f);
    for (int i = 0; i < count; ++i) {
        const int layer = i / (perRow * perRow), cell = i % (perRow * perRow);
        const float x = (float(cell % perRow) - float(perRow - 1) * 0.5f) * spacing + jitter(rng) * size;
        const float z = (float(cell / perRow) - float(perRow - 1) * 0.5f) * spacing + jitter(rng) * size;
        const float y = height + float(layer) * spacing;
        const int kind = shape == "box" ? 0 : shape == "sphere" ? 1 : shape == "capsule" ? 2 : i % 3;
        BodyDesc d;
        d.material = mat;
        d.continuousCollision = p["ccd"].get<bool>();
        d.transform.position = {x, y, z};
        d.transform.rotation = Quat::fromAxisAngle({jitter(rng), 1.0f, jitter(rng)}, angle(rng));
        if (kind == 0) {
            d.shape = ShapeDesc::box({size, size, size});
            addBody(d, "box", {size, size, size}, color);
        } else if (kind == 1) {
            d.shape = ShapeDesc::sphere(size);
            addBody(d, "sphere", {size, 0, 0}, color);
        } else {
            d.shape = ShapeDesc::capsule(size, size * 0.6f);
            addBody(d, "capsule", {size * 0.6f, size, 0}, color);
        }
    }
}

void Session::buildTower(const json& p) {
    const int height = int(p["height"].get<double>());
    const float h = float(p["boxSize"].get<double>()) * 0.5f;
    const std::string material = p["material"].get<std::string>();
    for (int i = 0; i < height; ++i) {
        BodyDesc d;
        d.shape = ShapeDesc::box({h, h, h});
        d.material = makeMaterial(presetFor(material));
        d.transform.position = {0.0f, h + float(i) * 2.0f * h, 0.0f};
        addBody(d, "box", {h, h, h}, colorFor(material));
    }
}

void Session::buildChain(const json& p) {
    const int links = int(p["links"].get<double>());
    const float half = float(p["linkLength"].get<double>()) * 0.5f;
    const float thick = std::max(0.05f, half * 0.3f);
    const bool hinge = p["joint"].get<std::string>() == "hinge";
    const float breakForce = float(p["breakForce"].get<double>());
    const std::string material = p["material"].get<std::string>();
    const float anchorY = float(links) * 2.0f * half + 1.0f;  // Hanging chain clears the floor.
    world_->setLayerCollision(1, 1, false);  // Neighbouring links overlap at the joints.

    BodyHandle prev{};
    for (int i = 0; i < links; ++i) {
        BodyDesc d;
        d.shape = ShapeDesc::box({half, thick, thick});
        d.material = makeMaterial(presetFor(material));
        d.transform.position = {half + float(i) * 2.0f * half, anchorY, 0.0f};
        d.layer = 1;
        d.allowSleeping = false;
        BodyHandle link = addBody(d, "box", {half, thick, thick}, colorFor(material));
        ConstraintDesc c;
        c.type = hinge ? ConstraintType::Hinge : ConstraintType::Point;
        c.bodyA = link;
        c.bodyB = prev;
        c.anchor = {float(i) * 2.0f * half, anchorY, 0.0f};
        c.axis = {0, 0, 1};
        c.normal = {1, 0, 0};
        c.breakForce = breakForce;
        world_->createConstraint(c);
        chainLinks_.push_back(link);
        prev = link;
    }
}

bool Session::buildFountain(const json& p, const json& g, std::string& error) {
    ParticleSettings s;
    s.maxParticles = uint32_t(p["maxParticles"].get<double>());
    s.emissionRate = float(p["rate"].get<double>());
    s.lifetimeMin = float(p["lifetime"].get<double>()) * 0.8f;
    s.lifetimeMax = float(p["lifetime"].get<double>()) * 1.2f;
    s.emitterPosition = {0.0f, 0.3f, 0.0f};
    s.initialVelocity = {0.0f, float(p["speed"].get<double>()), 0.0f};
    s.velocitySpread = float(p["spread"].get<double>());
    s.wind = {float(p["windX"].get<double>()), 0.0f, 0.0f};
    s.gravity = {0.0f, float(g["gravity"].get<double>()), 0.0f};
    s.restitution = float(p["restitution"].get<double>());
    s.friction = float(p["friction"].get<double>());
    s.drag = float(p["drag"].get<double>());
    s.particleRadius = 0.02f;
    s.fixedDt = world_->fixedDt();
    s.planes.push_back({{0, 1, 0}, 0.0f});
    if (p["obstacle"].get<bool>()) {
        const Vec3 c{1.2f, 0.8f, 0.0f};
        s.spheres.push_back({c, 0.8f});
        BodyDesc ball;
        ball.motion = MotionType::Static;
        ball.shape = ShapeDesc::sphere(0.8f);
        ball.transform.position = c;
        addBody(ball, "sphere", {0.8f, 0, 0}, kBallColor);
    }
    displayLimit_ = uint32_t(p["displayLimit"].get<double>());

    BackendRequest req;
    const std::string backend = p["backend"].get<std::string>();
    req.backend = backend == "cpu" ? ComputeBackend::CPU : backend == "gpu" ? ComputeBackend::GPU : ComputeBackend::Auto;
    req.cpuThreads = int(g["threads"].get<double>());
    auto made = ParticleSystem::create(s, req);
    if (!made.ok()) {
        error = made.error == ErrorCode::DeviceUnavailable ? "ใช้ GPU ไม่ได้: " + made.message : "ตั้งค่าอนุภาคไม่ถูกต้อง: " + made.message;
        return false;
    }
    particles_ = std::move(made.value);
    return true;
}

double Session::fixedDt() const { return world_ ? world_->fixedDt() : 1.0 / 60.0; }

bool Session::step(std::string& error) {
    if (!world_) return true;
    world_->step();
    physicsMs_ += world_->lastStepStats().stepMilliseconds;
    for (const SolverWarning& w : world_->warnings()) warnings_.push_back(w);
    brokenSinceFrame_ += world_->brokenConstraints().size();
    if (particles_) {
        Status s = particles_->step();
        particleMs_ += particles_->lastStepMilliseconds();
        if (!s.ok()) {
            error = "การ์ดจอทำงานผิดพลาด: " + s.message;
            return false;
        }
    }
    simTime_ += world_->fixedDt();
    ++stepsSinceFrame_;
    return true;
}

bool Session::action(const std::string& id, std::string& error) {
    if (!world_) {
        error = "ยังไม่ได้โหลดฉาก";
        return false;
    }
    if (sceneId_ == "tower" && id == "shoot") {
        const float h = float(params_["boxSize"].get<double>()) * 0.5f;
        const float towerTop = float(params_["height"].get<double>()) * 2.0f * h;
        BodyDesc ball;
        ball.shape = ShapeDesc::sphere(0.3f);
        ball.material = makeMaterial(MaterialPreset::Rubber);
        ball.massOverride = float(params_["ballMass"].get<double>());
        ball.transform.position = {-8.0f, std::max(0.4f, towerTop * 0.4f), 0.0f};
        ball.linearVelocity = {float(params_["ballSpeed"].get<double>()), 1.0f, 0.0f};
        ball.continuousCollision = true;
        if (!addBody(ball, "sphere", {0.3f, 0, 0}, kBallColor).isValid()) {
            error = "สร้างลูกบอลไม่ได้ (จำนวนวัตถุเต็ม)";
            return false;
        }
        return true;
    }
    if (sceneId_ == "chain" && id == "kick" && !chainLinks_.empty()) {
        const BodyHandle tip = chainLinks_.back();
        const float m = world_->getMass(tip);
        world_->addImpulse(tip, {0.0f, 4.0f * m, 6.0f * m});
        return true;
    }
    error = "ฉากนี้ไม่มีคำสั่ง '" + id + "'";
    return false;
}

namespace {
json bodyJson(json renderIndex, const std::string& shape, const Vec3& size, uint32_t color, bool dynamic, const Transform& t) {
    return {{"index", renderIndex}, {"shape", shape},   {"size", {size.x, size.y, size.z}},
            {"color", color},       {"dynamic", dynamic},
            {"position", {t.position.x, t.position.y, t.position.z}},
            {"rotation", {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w}}};
}
}  // namespace

json Session::describeScene() const {
    json list = json::array();
    size_t dynamicIndex = 0;
    for (const RenderBody& b : bodies_)
        list.push_back(bodyJson(b.dynamic ? json(dynamicIndex++) : json(-1), b.shape, b.size, b.color, b.dynamic,
                                world_->getTransform(b.handle)));
    return {{"scene", sceneId_},
            {"bodies", list},
            {"fixedDt", fixedDt()},
            {"particles", particles_ != nullptr},
            {"particleRadius", 0.02}};
}

json Session::takeAddedBodies() {
    json list = json::array();
    size_t dynamicIndex = 0;
    for (size_t i = 0; i < bodies_.size(); ++i) {
        const RenderBody& b = bodies_[i];
        if (i >= reportedBodies_)
            list.push_back(bodyJson(b.dynamic ? json(dynamicIndex) : json(-1), b.shape, b.size, b.color, b.dynamic,
                                    world_->getTransform(b.handle)));
        if (b.dynamic) ++dynamicIndex;
    }
    reportedBodies_ = bodies_.size();
    return list;
}

json Session::frame(bool includeParticles) {
    json f = {{"type", "frame"}, {"t", simTime_}};
    if (!world_) return f;

    // Dynamic body transforms: 7 floats each (px py pz qx qy qz qw) + 1 sleep byte each.
    std::vector<float> xf;
    std::vector<uint8_t> sleep;
    for (const RenderBody& b : bodies_) {
        if (!b.dynamic) continue;
        const Transform t = world_->getTransform(b.handle);
        xf.insert(xf.end(), {t.position.x, t.position.y, t.position.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w});
        sleep.push_back(world_->isSleeping(b.handle) ? 1 : 0);
    }
    f["bodies"] = base64(xf.data(), xf.size() * sizeof(float));
    f["sleep"] = base64(sleep.data(), sleep.size());

    json stats = {{"steps", stepsSinceFrame_},
                  {"physicsMs", stepsSinceFrame_ ? physicsMs_ / stepsSinceFrame_ : 0.0},
                  {"activeBodies", world_->lastStepStats().activeBodies},
                  {"bodies", world_->bodyCount()},
                  {"fixedDt", world_->fixedDt()},
                  {"broken", brokenSinceFrame_}};

    if (particles_) {
        stats["particleMs"] = stepsSinceFrame_ ? particleMs_ / stepsSinceFrame_ : 0.0;
        stats["particlesAlive"] = particles_->aliveCount();
        stats["backend"] = toString(particles_->device().backend);
        stats["device"] = particles_->device().name;
        stats["deviceType"] = toString(particles_->device().type);
        stats["reason"] = particles_->selectionReason();
        if (includeParticles) {
            const std::vector<Particle>& all = particles_->readback();
            const uint32_t alive = particles_->aliveCount();
            // Evenly thin out the display when more particles are alive than may be drawn.
            const double stride = alive > displayLimit_ ? double(alive) / double(displayLimit_) : 1.0;
            std::vector<float> pos;
            pos.reserve(size_t(std::min(alive, displayLimit_)) * 3);
            double next = 0.0;
            uint32_t seen = 0;
            for (const Particle& q : all) {
                if (q.lifetime <= 0.0f) continue;
                if (double(seen++) >= next) {
                    pos.insert(pos.end(), {q.px, q.py, q.pz});
                    next += stride;
                }
            }
            f["particles"] = base64(pos.data(), pos.size() * sizeof(float));
        }
    }
    f["stats"] = stats;

    // Group warnings by type so a pile of bodies does not flood the UI.
    std::map<std::string, std::pair<int, std::string>> grouped;
    for (const SolverWarning& w : warnings_) {
        auto& g = grouped[warningType(w.type)];
        ++g.first;
        g.second = w.message;
    }
    json warnings = json::array();
    for (auto& [type, g] : grouped) warnings.push_back({{"type", type}, {"count", g.first}, {"message", g.second}});
    f["warnings"] = warnings;

    physicsMs_ = particleMs_ = 0.0;
    stepsSinceFrame_ = 0;
    warnings_.clear();
    brokenSinceFrame_ = 0;
    return f;
}

}  // namespace vexora::sim
