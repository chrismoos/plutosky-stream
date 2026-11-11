#include "Device.hpp"
#include "HardwareKey.hpp"
#include "RxTimeOffset.hpp"
#include "UhdSerial.hpp"

#include <SoapySDR/Registry.hpp>

#include <cstdlib>

namespace PlutoSky {

static bool isUhdLookup(const SoapySDR::Kwargs &args)
{
    const auto type = args.find("type");
    // SoapyUHD uses the marker for broad discovery and type=soapy for opening.
    return (type != args.end() && type->second == "soapy")
        || args.count("soapy_uhd_no_deeper") != 0;
}

static void addSerial(SoapySDR::Kwargs &found, const SoapySDR::Kwargs &args,
                      const std::string &serial)
{
    const bool uhd = isUhdLookup(args);
    found["serial"] = uhd ? uhdSerialAlias(serial) : serial;
    if (uhd) found["plutosky_serial"] = serial;
}

static SoapySDR::KwargsList findPlutoSky(const SoapySDR::Kwargs &args)
{
    SoapySDR::KwargsList results;

    const auto driver = args.find("driver");
    if (driver != args.end() && driver->second != kDriverKey) return results;

    const auto addrArg = args.find("addr");
    if (addrArg != args.end())
    {
        int port = kDefaultControlPort;
        const auto portArg = args.find("port");
        if (portArg != args.end()) port = std::atoi(portArg->second.c_str());
        try
        {
            ControlClient probe(addrArg->second, port);
            SoapySDR::Kwargs found;
            found["driver"] = kDriverKey;
            found["addr"] = addrArg->second;
            found["port"] = std::to_string(port);
            const std::string serial = probe.info().serial;
            addSerial(found, args, serial);
            found["label"] = std::string("PlutoSky ") + serial + " " + addrArg->second;
            keepHardwareArg(found, args);
            keepRxTimeOffsetArg(found, args);
            results.push_back(found);
        }
        catch (const std::exception &)
        {
            // An unreachable explicit address is simply not a match.
        }
        return results;
    }

    int discoveryPort = kDiscoveryPort;
    const auto discoveryArg = args.find("discovery_port");
    if (discoveryArg != args.end()) discoveryPort = std::atoi(discoveryArg->second.c_str());

    for (const auto &reply : discover(kDiscoveryTimeoutUs, discoveryPort))
    {
        SoapySDR::Kwargs found;
        found["driver"] = kDriverKey;
        found["addr"] = reply.addr;
        found["port"] = std::to_string(reply.reply.control_port() != 0
                                           ? static_cast<int>(reply.reply.control_port())
                                           : kDefaultControlPort);
        const std::string &serial = reply.reply.serial();
        addSerial(found, args, serial);
        found["version"] = reply.reply.version();
        found["mac"] = reply.reply.mac();
        const std::string name = reply.reply.name().empty() ? "PlutoSky" : reply.reply.name();
        found["label"] = name + " " + serial + " " + reply.addr;
        if (discoveryArg != args.end()) found["discovery_port"] = discoveryArg->second;
        keepHardwareArg(found, args);
        keepRxTimeOffsetArg(found, args);
        results.push_back(found);
    }

    return results;
}

static SoapySDR::Device *makePlutoSky(const SoapySDR::Kwargs &args)
{
    return new SoapyPlutoSky(args);
}

static SoapySDR::Registry registerPlutoSky(kDriverKey, &findPlutoSky, &makePlutoSky,
                                           SOAPY_SDR_ABI_VERSION);

}  // namespace PlutoSky
