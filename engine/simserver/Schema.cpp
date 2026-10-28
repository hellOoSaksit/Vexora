#include "Schema.h"

#include <cmath>
#include <sstream>

namespace vexora::sim {
namespace {

ParamDef number(std::string key, std::string label, double min, double max, double step, double def, std::string unit,
                std::string help, bool advanced = false) {
    ParamDef p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.type = ParamType::Number;
    p.min = min;
    p.max = max;
    p.step = step;
    p.defaultValue = def;
    p.unit = std::move(unit);
    p.help = std::move(help);
    p.advanced = advanced;
    return p;
}

ParamDef integer(std::string key, std::string label, int min, int max, int def, std::string unit, std::string help,
                 bool advanced = false) {
    ParamDef p = number(std::move(key), std::move(label), min, max, 1, def, std::move(unit), std::move(help), advanced);
    p.type = ParamType::Int;
    p.defaultValue = def;
    return p;
}

ParamDef boolean(std::string key, std::string label, bool def, std::string help, bool advanced = false) {
    ParamDef p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.type = ParamType::Bool;
    p.defaultValue = def;
    p.help = std::move(help);
    p.advanced = advanced;
    return p;
}

ParamDef choice(std::string key, std::string label, std::vector<ParamOption> options, std::string def, std::string help,
                bool advanced = false) {
    ParamDef p;
    p.key = std::move(key);
    p.label = std::move(label);
    p.type = ParamType::Enum;
    p.options = std::move(options);
    p.defaultValue = std::move(def);
    p.help = std::move(help);
    p.advanced = advanced;
    return p;
}

std::vector<ParamOption> materialOptions() {
    return {{"metal", "โลหะ"},  {"wood", "ไม้"},   {"stone", "หิน"},           {"rubber", "ยาง (เด้งมาก)"},
            {"ice", "น้ำแข็ง (ลื่น)"}, {"fabric", "ผ้า"}, {"flesh", "เนื้อ/ร่างกาย"}};
}

const char* kMaterialHelp = "ค่าตั้งต้นของความลื่น การเด้ง และน้ำหนัก เป็นค่าประมาณสำหรับเริ่มต้น ไม่ใช่ค่าทางฟิสิกส์ที่แม่นยำ";

std::string fmt(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

}  // namespace

const std::vector<ParamDef>& globalParams() {
    static const std::vector<ParamDef> params = {
        choice("quality", "คุณภาพการจำลอง",
               {{"interactive", "ดูตัวอย่างเร็ว"}, {"high", "ดูละเอียด"}, {"final", "เรนเดอร์จริง (ช้าที่สุด)"}}, "interactive",
               "ยิ่งละเอียด วัตถุยิ่งนิ่งและข้อต่อยิ่งแม่น แต่คำนวณช้าลง"),
        number("gravity", "แรงโน้มถ่วง", -30, 0, 0.01, -9.81, "m/s²", "ค่าบนโลกคือ -9.81", true),
        integer("threads", "จำนวน CPU thread", 0, 64, 0, "", "0 = ให้โปรแกรมเลือกเอง", true),
    };
    return params;
}

const std::vector<SceneDef>& scenes() {
    static const std::vector<SceneDef> list = {
        {"drop",
         "วัตถุร่วงกระแทก",
         "ปล่อยวัตถุจำนวนมากจากที่สูงให้ตกกองบนพื้น",
         {
             integer("count", "จำนวนวัตถุ", 1, 3000, 150, "ชิ้น", "มากกว่า 2,000 ชิ้นอาจช้ากว่าเวลาจริงในโหมดดูตัวอย่าง"),
             choice("shape", "รูปทรง",
                    {{"mixed", "คละแบบ"}, {"box", "กล่อง"}, {"sphere", "ลูกบอล"}, {"capsule", "แคปซูล"}}, "mixed", ""),
             number("size", "ขนาด", 0.05, 2, 0.05, 0.25, "m", "ครึ่งหนึ่งของความกว้างวัตถุ"),
             choice("material", "วัสดุ", materialOptions(), "wood", kMaterialHelp),
             number("height", "ความสูงที่ปล่อย", 1, 50, 0.5, 6, "m", ""),
             number("area", "ความกว้างพื้นที่ปล่อย", 1, 30, 0.5, 4, "m", ""),
             boolean("ccd", "กันทะลุสำหรับวัตถุเร็ว (CCD)", false, "เปิดเมื่อวัตถุเล็กและเร็วมากจนทะลุพื้น ใช้การคำนวณเพิ่มขึ้น", true),
         },
         {}},
        {"tower",
         "หอคอยกล่อง",
         "เรียงกล่องเป็นหอคอยแล้วยิงลูกบอลใส่",
         {
             integer("height", "จำนวนชั้น", 1, 40, 12, "ชั้น", ""),
             number("boxSize", "ขนาดกล่อง", 0.1, 2, 0.05, 0.4, "m", ""),
             choice("material", "วัสดุกล่อง", materialOptions(), "stone", kMaterialHelp),
             number("ballSpeed", "ความเร็วลูกบอล", 1, 100, 1, 20, "m/s", ""),
             number("ballMass", "น้ำหนักลูกบอล", 1, 500, 1, 30, "kg", "", true),
         },
         {{"shoot", "ยิงลูกบอล"}}},
        {"chain",
         "โซ่แกว่ง",
         "โซ่ที่ข้อต่อเชื่อมกัน ปล่อยจากแนวนอนให้แกว่งลง",
         {
             integer("links", "จำนวนข้อ", 2, 60, 15, "ข้อ", ""),
             number("linkLength", "ความยาวแต่ละข้อ", 0.1, 2, 0.05, 0.4, "m", ""),
             choice("joint", "ชนิดข้อต่อ", {{"ball", "ลูกหมาก (หมุนได้ทุกทิศ)"}, {"hinge", "บานพับ (หมุนแกนเดียว)"}}, "ball", ""),
             choice("material", "วัสดุ", materialOptions(), "metal", kMaterialHelp),
             number("breakForce", "แรงที่ทำให้ข้อต่อขาด", 0, 100000, 100, 0, "N", "0 = ไม่ขาด", true),
         },
         {{"kick", "ผลักปลายโซ่"}}},
        {"ragdoll",
         "ตุ๊กตาล้ม (Ragdoll)",
         "ตัวละครยืนด้วย Animation กด 'ผลัก' ให้ล้มเป็น Ragdoll แล้วกด 'ลุกขึ้น' เพื่อผสมกลับเป็น Animation",
         {
             number("pushStrength", "แรงผลัก", 0, 400, 5, 120, "N·s", ""),
             choice("start", "เริ่มต้นแบบ", {{"stand", "ยืน (Animation)"}, {"drop", "ร่วงจากที่สูง (Ragdoll)"}}, "stand", ""),
             number("dropHeight", "ความสูงตอนร่วง", 0, 10, 0.1, 2, "m", "ใช้เมื่อเริ่มต้นแบบร่วงจากที่สูง"),
             number("getUpTime", "เวลาผสมกลับเป็น Animation", 0.1, 3, 0.1, 0.8, "s", "ยิ่งนาน การเปลี่ยนท่ายิ่งนุ่ม", true),
             number("motorStrength", "ความแข็งของกล้ามเนื้อ (เกร็งตัว)", 1, 20, 0.5, 6, "Hz", "ใช้ในโหมดเกร็งตัว (Powered)", true),
             integer("count", "จำนวนตัวละคร", 1, 20, 1, "ตัว", "", true),
         },
         {{"push", "ผลักให้ล้ม"}, {"powered", "เกร็งตัว (Powered)"}, {"getup", "ลุกขึ้น"}}},
        {"tail",
         "หางสปริง",
         "การเคลื่อนไหวรอง (Secondary Motion) ของหาง เป็นการประมาณผลด้วยสปริง ไม่ใช่การจำลองเต็มรูปแบบ",
         {
             choice("motion", "การเคลื่อนไหวของตัว",
                    {{"sway", "ส่ายซ้าย-ขวา"}, {"circle", "วิ่งเป็นวงกลม"}, {"jump", "กระโดด"}, {"still", "อยู่นิ่ง"}}, "sway", ""),
             number("speed", "ความเร็วการเคลื่อนไหว", 0.1, 3, 0.1, 1, "เท่า", ""),
             integer("segments", "จำนวนข้อหาง", 2, 30, 10, "ข้อ", ""),
             number("segmentLength", "ความยาวแต่ละข้อ", 0.05, 0.5, 0.01, 0.15, "m", ""),
             number("stiffness", "ความแข็ง (คืนรูป)", 0, 1, 0.01, 0.05, "", "0 = ห้อยตามแรงโน้มถ่วง, 1 = แข็งตาม Animation"),
             number("damping", "การหน่วง", 0, 1, 0.01, 0.08, "", "ยิ่งมาก ยิ่งหยุดแกว่งเร็ว"),
             number("inertia", "แรงเฉื่อย", 0, 1, 0.05, 1, "", "1 = หางสะบัดตามแรงเฉื่อยเต็มที่, 0 = ขยับไปพร้อมตัว"),
             number("gravityScale", "สัดส่วนแรงโน้มถ่วง", 0, 3, 0.1, 1, "เท่า", "", true),
             number("maxAngle", "มุมงอสูงสุดต่อข้อ", 0, 180, 1, 70, "องศา", "", true),
             number("weight", "น้ำหนักการผสม", 0, 1, 0.05, 1, "", "0 = ตาม Animation ล้วน, 1 = สปริงเต็มที่", true),
         },
         {{"reset", "รีเซ็ตสปริง"}}},
        {"fountain",
         "น้ำพุอนุภาค",
         "อนุภาคพุ่งขึ้นแล้วตกกระทบพื้นและลูกบอล ใช้ GPU ได้",
         {
             integer("maxParticles", "จำนวนอนุภาคสูงสุด", 100, 2000000, 30000, "จุด", ""),
             number("rate", "อัตราการปล่อย", 10, 1000000, 100, 10000, "จุด/วินาที", ""),
             number("lifetime", "อายุอนุภาค", 0.1, 20, 0.1, 3, "s", ""),
             number("speed", "ความเร็วพุ่ง", 0, 50, 0.5, 8, "m/s", ""),
             number("spread", "การกระจาย", 0, 20, 0.1, 2, "m/s", "ยิ่งมาก อนุภาคยิ่งกระจายออกกว้าง"),
             number("windX", "ลม (แนวนอน)", -30, 30, 0.5, 0, "m/s", ""),
             number("restitution", "การเด้ง", 0, 1, 0.05, 0.4, "", "0 = ไม่เด้ง, 1 = เด้งเต็มที่"),
             number("friction", "แรงเสียดทานตอนชน", 0, 1, 0.05, 0.1, "", "", true),
             number("drag", "แรงต้านอากาศ", 0, 5, 0.05, 0.1, "1/s", "", true),
             boolean("obstacle", "มีลูกบอลกีดขวาง", true, ""),
             choice("backend", "ประมวลผลด้วย",
                    {{"auto", "อัตโนมัติ"}, {"cpu", "CPU"}, {"gpu", "GPU (การ์ดจอ)"}}, "auto",
                    "อัตโนมัติจะใช้ GPU เมื่อมีการ์ดจอและอนุภาคมากพอ"),
             integer("displayLimit", "จำนวนจุดที่แสดงผลสูงสุด", 1000, 200000, 20000, "จุด",
                     "จำกัดเฉพาะการแสดงผล การคำนวณยังทำครบทุกจุด", true),
         },
         {}},
    };
    return list;
}

const SceneDef* findScene(const std::string& id) {
    for (const SceneDef& s : scenes())
        if (s.id == id) return &s;
    return nullptr;
}

namespace {

const char* typeName(ParamType t) {
    switch (t) {
        case ParamType::Number: return "number";
        case ParamType::Int: return "int";
        case ParamType::Bool: return "bool";
        case ParamType::Enum: return "enum";
    }
    return "number";
}

json paramJson(const ParamDef& p) {
    json j = {{"key", p.key}, {"label", p.label}, {"help", p.help}, {"type", typeName(p.type)},
              {"default", p.defaultValue}, {"advanced", p.advanced}};
    if (p.type == ParamType::Number || p.type == ParamType::Int) {
        j["min"] = p.min;
        j["max"] = p.max;
        j["step"] = p.step;
        j["unit"] = p.unit;
    }
    if (p.type == ParamType::Enum) {
        j["options"] = json::array();
        for (const ParamOption& o : p.options) j["options"].push_back({{"value", o.value}, {"label", o.label}});
    }
    return j;
}

}  // namespace

json schemaJson() {
    json s = {{"version", 1}, {"global", json::array()}, {"scenes", json::array()}};
    for (const ParamDef& p : globalParams()) s["global"].push_back(paramJson(p));
    for (const SceneDef& sc : scenes()) {
        json j = {{"id", sc.id}, {"label", sc.label}, {"description", sc.description}, {"params", json::array()},
                  {"actions", json::array()}};
        for (const ParamDef& p : sc.params) j["params"].push_back(paramJson(p));
        for (const ActionDef& a : sc.actions) j["actions"].push_back({{"id", a.id}, {"label", a.label}});
        s["scenes"].push_back(j);
    }
    return s;
}

bool resolveParams(const std::vector<ParamDef>& defs, const json& input, json& out, std::string& error) {
    out = json::object();
    for (const ParamDef& p : defs) {
        const bool given = input.is_object() && input.contains(p.key) && !input[p.key].is_null();
        const json& v = given ? input[p.key] : p.defaultValue;
        switch (p.type) {
            case ParamType::Number:
            case ParamType::Int: {
                if (!v.is_number()) {
                    error = "ค่า '" + p.label + "' ต้องเป็นตัวเลข";
                    return false;
                }
                const double d = v.get<double>();
                if (!std::isfinite(d) || d < p.min || d > p.max) {
                    error = "ค่า '" + p.label + "' ต้องอยู่ระหว่าง " + fmt(p.min) + " ถึง " + fmt(p.max);
                    return false;
                }
                if (p.type == ParamType::Int && std::floor(d) != d) {
                    error = "ค่า '" + p.label + "' ต้องเป็นจำนวนเต็ม";
                    return false;
                }
                out[p.key] = d;
                break;
            }
            case ParamType::Bool:
                if (!v.is_boolean()) {
                    error = "ค่า '" + p.label + "' ต้องเป็น เปิด/ปิด";
                    return false;
                }
                out[p.key] = v;
                break;
            case ParamType::Enum: {
                bool ok = v.is_string();
                if (ok) {
                    ok = false;
                    for (const ParamOption& o : p.options) ok |= o.value == v.get<std::string>();
                }
                if (!ok) {
                    error = "ค่า '" + p.label + "' ไม่อยู่ในตัวเลือกที่มี";
                    return false;
                }
                out[p.key] = v;
                break;
            }
        }
    }
    return true;
}

}  // namespace vexora::sim
