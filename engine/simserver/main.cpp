// vexora_sim: background simulation process for the Vexora app.
//
// Protocol: one JSON object per line. Commands on stdin, messages on stdout.
//   -> {"cmd":"load","scene":"drop","params":{...},"global":{...}}
//   -> {"cmd":"play"} | {"cmd":"pause"} | {"cmd":"reset"} | {"cmd":"step","count":1}
//   -> {"cmd":"action","id":"shoot"} | {"cmd":"quit"}
//   <- {"type":"hello",...schema, devices}   (sent once at start)
//   <- {"type":"loaded",...} | {"type":"frame",...} | {"type":"bodiesAdded",...}
//   <- {"type":"state","playing":bool} | {"type":"error","message":...}
// Simulation runs in fixed steps; while playing it follows wall-clock time and reports when
// it falls behind (speed < 1). The UI never owns simulation state.

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#include "Schema.h"
#include "Session.h"
#include "vexora/physics/ComputeDevice.h"
#include "vexora/physics/SimulationClock.h"

using namespace vexora::sim;
using vexora::physics::FixedStepClock;

namespace {

constexpr const char* kVersion = "0.1.0";

void send(const json& msg) {
    const std::string line = msg.dump(-1, ' ', false, json::error_handler_t::replace);
    std::fwrite(line.data(), 1, line.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
}

void sendError(const std::string& message) { send({{"type", "error"}, {"message", message}}); }

json helloMessage() {
    json devices = json::array();
    bool gpu = false;
    for (const auto& d : vexora::physics::enumerateComputeDevices()) {
        devices.push_back({{"backend", toString(d.backend)},
                           {"type", toString(d.type)},
                           {"name", d.name},
                           {"memoryMB", d.memoryBytes / (1024 * 1024)},
                           {"threads", d.cpuThreads},
                           {"api", d.apiVersion}});
        gpu |= d.backend == vexora::physics::ComputeBackend::GPU;
    }
    return {{"type", "hello"},
            {"version", kVersion},
            {"schema", schemaJson()},
            {"devices", devices},
            {"gpuAvailable", gpu},
            {"gpuReason", gpu ? "" : vexora::physics::gpuUnavailableReason()},
            {"systems", {{{"name", "rigid"}, {"label", "วัตถุแข็ง/ข้อต่อ"}, {"gpu", false}},
                         {{"name", "particles"}, {"label", "อนุภาค"}, {"gpu", true}}}}};
}

// Reads stdin on its own thread; the main thread owns all simulation state.
class CommandQueue {
public:
    void push(std::string line) {
        std::lock_guard<std::mutex> lock(mutex_);
        lines_.push_back(std::move(line));
        cv_.notify_one();
    }
    // Waits until a command arrives or the deadline passes.
    bool pop(std::string& out, std::chrono::steady_clock::time_point deadline) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_until(lock, deadline, [&] { return !lines_.empty(); })) return false;
        out = std::move(lines_.front());
        lines_.pop_front();
        return true;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> lines_;
};

}  // namespace

int main() {
#ifdef _WIN32
    _setmode(_fileno(stdout), _O_BINARY);  // No CRLF translation inside the protocol.
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    std::ios::sync_with_stdio(false);
    send(helloMessage());

    // Heap-allocated and never freed: the detached reader may still touch it after quit.
    CommandQueue& queue = *new CommandQueue();
    std::thread reader([&queue] {
        std::string line;
        while (std::getline(std::cin, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) queue.push(line);
        }
        queue.push(R"({"cmd":"quit"})");  // Parent closed the pipe: exit cleanly.
    });
    reader.detach();

    auto sessionPtr = std::make_unique<Session>();
    Session& session = *sessionPtr;
    json lastLoad;
    bool playing = false;
    FixedStepClock clock;
    auto lastTick = std::chrono::steady_clock::now();
    auto nextFrame = lastTick;
    double speedSim = 0.0, speedWall = 0.0, speed = 1.0;

    auto doLoad = [&](const json& cmd) {
        const std::string sceneId = cmd.value("scene", "");
        const SceneDef* def = findScene(sceneId);
        if (!def) return sendError("ไม่รู้จักฉาก '" + sceneId + "'");
        json params, global;
        std::string error;
        if (!resolveParams(def->params, cmd.value("params", json::object()), params, error) ||
            !resolveParams(globalParams(), cmd.value("global", json::object()), global, error))
            return sendError(error);
        if (!session.load(sceneId, params, global, error)) return sendError(error);
        lastLoad = cmd;
        clock = FixedStepClock(session.fixedDt(), 4);
        json loaded = session.describeScene();
        loaded["type"] = "loaded";
        loaded["params"] = params;
        loaded["global"] = global;
        send(loaded);
        send(session.frame(true));
    };

    for (;;) {
        // Process commands until the next frame is due.
        std::string line;
        const auto deadline = playing ? nextFrame : std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
        while (queue.pop(line, deadline)) {
            json cmd = json::parse(line, nullptr, false);
            if (cmd.is_discarded() || !cmd.is_object()) {
                sendError("คำสั่งไม่ใช่ JSON ที่ถูกต้อง");
                continue;
            }
            const std::string name = cmd.value("cmd", "");
            if (name == "quit") {
                sessionPtr.reset();  // Release GPU/physics resources in order.
                std::fflush(stdout);
                std::_Exit(0);       // Do not run static destructors under the detached reader thread.
            }
            if (name == "load") {
                playing = false;
                doLoad(cmd);
            } else if (name == "reset") {
                playing = false;
                if (!lastLoad.is_null()) doLoad(lastLoad);
            } else if (name == "play" || name == "pause") {
                playing = name == "play" && session.loaded();
                lastTick = nextFrame = std::chrono::steady_clock::now();
                speedSim = speedWall = 0.0;
                send({{"type", "state"}, {"playing", playing}});
            } else if (name == "step") {
                std::string error;
                const int count = std::max(1, std::min(1000, cmd.value("count", 1)));
                for (int i = 0; i < count && session.loaded(); ++i)
                    if (!session.step(error)) {
                        sendError(error);
                        break;
                    }
                send(session.frame(true));
            } else if (name == "action") {
                std::string error;
                if (!session.action(cmd.value("id", ""), error)) sendError(error);
                json added = session.takeAddedBodies();
                if (!added.empty()) send({{"type", "bodiesAdded"}, {"bodies", added}});
            } else if (name == "hello") {
                send(helloMessage());
            } else {
                sendError("ไม่รู้จักคำสั่ง '" + name + "'");
            }
            if (playing && std::chrono::steady_clock::now() >= nextFrame) break;
        }
        if (!playing) continue;

        // Advance simulation to wall-clock time in fixed steps.
        const auto now = std::chrono::steady_clock::now();
        const double wall = std::min(0.25, std::chrono::duration<double>(now - lastTick).count());
        lastTick = now;
        const int steps = clock.advance(wall);
        std::string error;
        for (int i = 0; i < steps; ++i)
            if (!session.step(error)) {
                sendError(error);
                playing = false;
                send({{"type", "state"}, {"playing", false}});
                break;
            }
        speedSim += steps * session.fixedDt();
        speedWall += wall;
        if (speedWall >= 0.5) {
            speed = speedSim / speedWall;
            speedSim = speedWall = 0.0;
        }

        json f = session.frame(true);
        f["stats"]["speed"] = speed;  // < 1: the computer cannot keep up with real time.
        send(f);
        // Particle frames are larger, so they stream at 30 Hz; rigid scenes at 60 Hz.
        nextFrame = now + std::chrono::milliseconds(f.contains("particles") ? 33 : 16);
    }
}
