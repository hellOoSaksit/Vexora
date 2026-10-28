// Base64 for binary frame payloads (Float32 arrays) inside JSON lines.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace vexora::sim {

inline std::string base64(const void* data, size_t size) {
    static const char* table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const auto* p = static_cast<const uint8_t*>(data);
    std::string out;
    out.reserve((size + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < size; i += 3) {
        const uint32_t v = (uint32_t(p[i]) << 16) | (uint32_t(p[i + 1]) << 8) | p[i + 2];
        out += table[(v >> 18) & 63];
        out += table[(v >> 12) & 63];
        out += table[(v >> 6) & 63];
        out += table[v & 63];
    }
    if (i < size) {
        uint32_t v = uint32_t(p[i]) << 16;
        if (i + 1 < size) v |= uint32_t(p[i + 1]) << 8;
        out += table[(v >> 18) & 63];
        out += table[(v >> 12) & 63];
        out += i + 1 < size ? table[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

}  // namespace vexora::sim
