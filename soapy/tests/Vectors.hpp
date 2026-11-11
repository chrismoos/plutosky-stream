// Loads tools/vectors/wire_formats.json, the vectors shared with the gateware
// and the daemon.

#pragma once

#include "Json.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace PlutoSkyTest {

inline PlutoSky::Json loadVectors()
{
    const std::string path = std::string(PLUTOSKY_VECTORS_DIR) + "/wire_formats.json";
    std::FILE *handle = std::fopen(path.c_str(), "rb");
    if (handle == nullptr) throw std::runtime_error("cannot open " + path);
    std::string text;
    char buffer[8192];
    size_t got = 0;
    while ((got = std::fread(buffer, 1, sizeof(buffer), handle)) > 0) text.append(buffer, got);
    std::fclose(handle);
    return PlutoSky::Json::parse(text);
}

inline std::vector<uint8_t> fromHex(const std::string &hex)
{
    std::vector<uint8_t> out;
    out.reserve(hex.size() / 2);
    for (size_t i = 0; i + 1 < hex.size(); i += 2)
    {
        out.push_back(static_cast<uint8_t>(std::strtol(hex.substr(i, 2).c_str(), nullptr, 16)));
    }
    return out;
}

inline std::string toHex(const uint8_t *data, size_t length)
{
    static const char *digits = "0123456789abcdef";
    std::string out;
    out.reserve(length * 2);
    for (size_t i = 0; i < length; i++)
    {
        out += digits[data[i] >> 4];
        out += digits[data[i] & 0xF];
    }
    return out;
}

}  // namespace PlutoSkyTest
