#pragma once

#include <map>
#include <string>

namespace PlutoSky {

// Same type as SoapySDR::Kwargs, kept free of SoapySDR so tests need no runtime.
using DeviceArgs = std::map<std::string, std::string>;

// UHD reports the hardware key as the motherboard name, which osmo-trx matches
// against its device table.
static const char *const kHardwareArg = "hardware";
static const char *const kDefaultHardwareKey = "PlutoSky R1";

inline std::string hardwareKey(const DeviceArgs &args, const std::string &serial)
{
    const auto hardware = args.find(kHardwareArg);
    if (hardware != args.end() && !hardware->second.empty()) return hardware->second;
    return serial.empty() ? std::string(kDefaultHardwareKey) : serial;
}

// UHD opens a device with the arguments find returned, not the ones it was given.
inline void keepHardwareArg(DeviceArgs &found, const DeviceArgs &args)
{
    const auto hardware = args.find(kHardwareArg);
    if (hardware != args.end()) found[kHardwareArg] = hardware->second;
}

}  // namespace PlutoSky
