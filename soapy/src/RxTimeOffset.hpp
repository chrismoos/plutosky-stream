#pragma once

#include "HardwareKey.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace PlutoSky {

// Shifts receive timestamps against transmit so a client tuned for another
// radio's receive delay lines up without changes of its own.
static const char *const kRxTimeOffsetArg = "rx_time_offset_ns";

inline long long parseRxTimeOffsetNs(const DeviceArgs &args)
{
    const auto arg = args.find(kRxTimeOffsetArg);
    if (arg == args.end()) return 0;
    const char *text = arg->second.c_str();
    char *end = nullptr;
    errno = 0;
    const long long ns = std::strtoll(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE)
        throw std::runtime_error("plutosky: rx_time_offset_ns must be an integer number of nanoseconds");
    return ns;
}

// The offset in samples at rate, rounded so the shift stays sample-exact at any
// rate.
inline long long rxTimeOffsetSamples(long long offsetNs, double rate)
{
    static const double kNsPerSecond = 1e9;
    return std::llround(static_cast<double>(offsetNs) * rate / kNsPerSecond);
}

// A sample the board stamps at boardTicks reached the antenna offsetTicks later
// on the transmit timeline.
inline long long rxClientTicks(uint64_t boardTicks, long long offsetTicks)
{
    return static_cast<long long>(boardTicks) - offsetTicks;
}

// The timebase starts at zero, so a start before it is simply in the past.
inline uint64_t rxBoardTicks(long long clientTicks, long long offsetTicks)
{
    const long long ticks = clientTicks + offsetTicks;
    return ticks < 0 ? 0 : static_cast<uint64_t>(ticks);
}

// UHD opens a device with the arguments find returned, not the ones it was given.
inline void keepRxTimeOffsetArg(DeviceArgs &found, const DeviceArgs &args)
{
    const auto arg = args.find(kRxTimeOffsetArg);
    if (arg != args.end()) found[kRxTimeOffsetArg] = arg->second;
}

}  // namespace PlutoSky
