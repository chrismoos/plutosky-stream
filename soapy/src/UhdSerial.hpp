#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

namespace PlutoSky {

inline std::string uhdSerialAlias(const std::string &serial)
{
    static const uint32_t kFnvOffset = 2166136261u;
    static const uint32_t kFnvPrime = 16777619u;
    // UHD parses serial hints as signed 32-bit hexadecimal integers.
    static const uint32_t kSignedIntMax = 0x7fffffffu;

    uint32_t hash = kFnvOffset;
    for (const unsigned char byte : serial)
    {
        hash ^= byte;
        hash *= kFnvPrime;
    }
    char text[9];
    std::snprintf(text, sizeof(text), "%08x", hash & kSignedIntMax);
    return text;
}

}  // namespace PlutoSky
