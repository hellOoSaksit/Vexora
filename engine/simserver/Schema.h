// Settings schema: the single definition of every user-facing parameter. The UI builds its
// forms from this (labels, units, ranges, help), so adding a parameter needs no UI code.
#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace vexora::sim {

using nlohmann::json;

enum class ParamType { Number, Int, Bool, Enum };

struct ParamOption {
    std::string value;
    std::string label;
};

struct ParamDef {
    std::string key;
    std::string label;
    std::string help;
    ParamType type = ParamType::Number;
    double min = 0.0, max = 1.0, step = 0.1;
    json defaultValue;
    std::string unit;
    std::vector<ParamOption> options;  // Enum only
    bool advanced = false;             // Hidden in the UI's simple mode.
};

struct ActionDef {
    std::string id;
    std::string label;
};

struct SceneDef {
    std::string id;
    std::string label;
    std::string description;
    std::vector<ParamDef> params;
    std::vector<ActionDef> actions;
};

const std::vector<ParamDef>& globalParams();
const std::vector<SceneDef>& scenes();
const SceneDef* findScene(const std::string& id);

json schemaJson();

// Fills defaults for missing keys and validates the rest. On failure returns false and sets
// a message that names the parameter by its user-facing label.
bool resolveParams(const std::vector<ParamDef>& defs, const json& input, json& out, std::string& error);

}  // namespace vexora::sim
