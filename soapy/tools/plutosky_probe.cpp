// Command line exercise of the plutosky module, standing in for SoapySDRUtil
// where that tool is not installed.
//
//   plutosky_probe --find    [--args=...]
//   plutosky_probe --probe   [--args=...]
//   plutosky_probe --rate    [--args=...] [--wire=cs12] [--mtu=1500] [--format=CF32]
//                            [--channels=0,1] [--secs=2] [--rate=Hz] [--freq=Hz]
//                            [--read=N elements per readStream call]
//   plutosky_probe --tx      [...] [--burst=4096] [--bursts=4] [--at=0.05]
//   plutosky_probe --duplex  [...] [--secs=10] [--rate=30720000] [--tx_rate=Hz] [--at=0.1]
//
// Every mode ends with one RESULT line of key=value pairs for scripted checks.

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include <SoapySDR/Types.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

const double kDefaultSeconds = 2.0;
const size_t kDefaultBurstElems = 4096;
const int kDefaultBursts = 4;
const double kDefaultScheduleAhead = 0.05;
const long kReadTimeoutUs = 500000;
const double kToneCycles = 8.0;
const double kMaxDuplexSeconds = 600.0;
const double kMaxDuplexDeadlineGraceSeconds = 120.0;
const double kDuplexDrainSeconds = 0.05;

std::map<std::string, std::string> parseOptions(int argc, char **argv, std::string &mode)
{
    std::map<std::string, std::string> options;
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.rfind("--", 0) != 0) continue;
        arg = arg.substr(2);
        const size_t eq = arg.find('=');
        if (eq == std::string::npos)
        {
            mode = arg;
            continue;
        }
        options[arg.substr(0, eq)] = arg.substr(eq + 1);
    }
    return options;
}

std::string option(const std::map<std::string, std::string> &options, const std::string &key,
                   const std::string &fallback)
{
    const auto it = options.find(key);
    return it == options.end() ? fallback : it->second;
}

std::vector<size_t> parseChannels(const std::string &text)
{
    std::vector<size_t> out;
    size_t pos = 0;
    while (pos < text.size())
    {
        const size_t comma = text.find(',', pos);
        const std::string token = text.substr(pos, comma == std::string::npos ? comma : comma - pos);
        if (!token.empty()) out.push_back(static_cast<size_t>(std::atoi(token.c_str())));
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    if (out.empty()) out.push_back(0);
    return out;
}

SoapySDR::Kwargs streamArgs(const std::map<std::string, std::string> &options,
                           const std::string &direction = "")
{
    SoapySDR::Kwargs args;
    const std::string wire = option(options, "wire", "cs12");
    args["wire"] = direction.empty() ? wire : option(options, direction + "-wire", wire);
    args["mtu"] = option(options, "mtu", "1500");
    const auto queue = options.find("rx_queue");
    if (queue != options.end()) args["rx_queue"] = queue->second;
    const auto rateLimit = options.find("duplex_tx_rate_limit_sps");
    if (rateLimit != options.end()) args["duplex_tx_rate_limit_sps"] = rateLimit->second;
    const auto txOnlyLimit = options.find("tx_only_rate_limit_sps");
    if (txOnlyLimit != options.end()) args["tx_only_rate_limit_sps"] = txOnlyLimit->second;
    for (const auto *oldName : {"tx_pace_headroom", "tx_send_rate_multiplier"})
    {
        const auto oldSetting = options.find(oldName);
        if (oldSetting != options.end()) args[oldName] = oldSetting->second;
    }
    return args;
}

void printKwargs(const char *label, const SoapySDR::Kwargs &args)
{
    std::printf("%s:\n", label);
    for (const auto &entry : args) std::printf("  %s = %s\n", entry.first.c_str(),
                                               entry.second.c_str());
}

int doFind(const std::map<std::string, std::string> &options)
{
    const auto results = SoapySDR::Device::enumerate(option(options, "args", "driver=plutosky"));
    for (size_t i = 0; i < results.size(); i++)
    {
        std::printf("Found device %zu\n", i);
        for (const auto &entry : results[i])
        {
            std::printf("  %s = %s\n", entry.first.c_str(), entry.second.c_str());
        }
    }
    std::printf("RESULT devices=%zu\n", results.size());
    return results.empty() ? 1 : 0;
}

void probeDirection(SoapySDR::Device *device, const int direction, const char *label)
{
    const size_t channels = device->getNumChannels(direction);
    std::printf("%s channels: %zu\n", label, channels);
    for (size_t ch = 0; ch < channels; ch++)
    {
        std::printf("  channel %zu\n", ch);
        std::printf("    full duplex: %s\n", device->getFullDuplex(direction, ch) ? "yes" : "no");

        std::string antennas;
        for (const auto &name : device->listAntennas(direction, ch))
        {
            if (!antennas.empty()) antennas += ", ";
            antennas += name;
        }
        std::printf("    antennas: %s\n", antennas.c_str());
        std::printf("    antenna: %s\n", device->getAntenna(direction, ch).c_str());

        std::printf("    frequency: %.0f Hz\n", device->getFrequency(direction, ch));
        std::printf("    gain: %.2f dB\n", device->getGain(direction, ch));
        const SoapySDR::Range gain = device->getGainRange(direction, ch);
        std::printf("    gain range: %.2f to %.2f dB\n", gain.minimum(), gain.maximum());
        std::printf("    bandwidth: %.0f Hz\n", device->getBandwidth(direction, ch));
        if (device->hasGainMode(direction, ch))
        {
            std::printf("    gain mode: %s\n",
                        device->readSetting(direction, ch, "gain_mode").c_str());
        }

        std::string formats;
        for (const auto &name : device->getStreamFormats(direction, ch))
        {
            if (!formats.empty()) formats += ", ";
            formats += name;
        }
        std::printf("    stream formats: %s\n", formats.c_str());
        double fullScale = 0.0;
        std::printf("    native format: %s (full scale %.0f)\n",
                    device->getNativeStreamFormat(direction, ch, fullScale).c_str(), fullScale);

        for (const auto &arg : device->getStreamArgsInfo(direction, ch))
        {
            std::string opts;
            for (const auto &choice : arg.options)
            {
                if (!opts.empty()) opts += "|";
                opts += choice;
            }
            std::printf("    stream arg %s = %s [%s]\n", arg.key.c_str(), arg.value.c_str(),
                        opts.c_str());
        }

        for (const auto &sensor : device->listSensors(direction, ch))
        {
            std::printf("    sensor %s = %s\n", sensor.c_str(),
                        device->readSensor(direction, ch, sensor).c_str());
        }
    }
}

int doProbe(const std::map<std::string, std::string> &options)
{
    SoapySDR::Device *device = SoapySDR::Device::make(option(options, "args", "driver=plutosky"));
    std::printf("Driver: %s\n", device->getDriverKey().c_str());
    std::printf("Hardware: %s\n", device->getHardwareKey().c_str());
    printKwargs("Hardware info", device->getHardwareInfo());

    const SoapySDR::RangeList rates = device->getSampleRateRange(SOAPY_SDR_RX, 0);
    if (!rates.empty())
    {
        std::printf("Sample rate range: %.0f to %.0f Hz\n", rates.front().minimum(),
                    rates.front().maximum());
    }
    std::printf("Sample rate: %.0f Hz\n", device->getSampleRate(SOAPY_SDR_RX, 0));
    std::printf("Hardware time: %lld ns\n", device->getHardwareTime());

    for (const auto &sensor : device->listSensors())
    {
        std::printf("Sensor %s = %s\n", sensor.c_str(), device->readSensor(sensor).c_str());
    }

    probeDirection(device, SOAPY_SDR_RX, "RX");
    probeDirection(device, SOAPY_SDR_TX, "TX");

    const std::string hardware = device->getHardwareKey();
    const double rate = device->getSampleRate(SOAPY_SDR_RX, 0);
    SoapySDR::Device::unmake(device);
    std::printf("RESULT driver=plutosky hardware=%s rate=%.0f\n", hardware.c_str(), rate);
    return 0;
}

int doReceive(const std::map<std::string, std::string> &options)
{
    SoapySDR::Device *device = SoapySDR::Device::make(option(options, "args", "driver=plutosky"));
    const std::vector<size_t> channels = parseChannels(option(options, "channels", "0"));
    const std::string format = option(options, "format", SOAPY_SDR_CF32);
    const double seconds = std::atof(option(options, "secs", std::to_string(kDefaultSeconds)).c_str());

    const auto rateArg = options.find("rate");
    if (rateArg != options.end())
    {
        device->setSampleRate(SOAPY_SDR_RX, 0, std::atof(rateArg->second.c_str()));
    }
    const auto freqArg = options.find("freq");
    if (freqArg != options.end())
    {
        for (const auto ch : channels)
        {
            device->setFrequency(SOAPY_SDR_RX, ch, std::atof(freqArg->second.c_str()));
        }
    }
    const double rate = device->getSampleRate(SOAPY_SDR_RX, channels.front());

    SoapySDR::Stream *stream =
        device->setupStream(SOAPY_SDR_RX, format, channels, streamArgs(options));
    const size_t mtu = device->getStreamMTU(stream);
    std::printf("Stream MTU: %zu elements per channel\n", mtu);

    // Reading fewer elements than a packet holds leaves the rest for the next
    // call, which is what MORE_FRAGMENTS reports.
    size_t readElems = mtu;
    const auto readArg = options.find("read");
    if (readArg != options.end())
    {
        const int requested = std::atoi(readArg->second.c_str());
        if (requested > 0 && static_cast<size_t>(requested) < mtu)
        {
            readElems = static_cast<size_t>(requested);
        }
    }

    const size_t elementBytes = SoapySDR::formatToSize(format);
    std::vector<std::vector<uint8_t>> storage(channels.size());
    std::vector<void *> buffs(channels.size());
    for (size_t c = 0; c < channels.size(); c++)
    {
        storage[c].resize(mtu * elementBytes);
        buffs[c] = storage[c].data();
    }

    device->activateStream(stream);

    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::microseconds(
        static_cast<long long>(seconds * 1e6));

    size_t totalElems = 0;
    size_t reads = 0;
    size_t overflows = 0;
    size_t timeouts = 0;
    size_t fragments = 0;
    size_t timeJumps = 0;
    bool haveTick = false;
    long long expectedTick = 0;

    while (std::chrono::steady_clock::now() < deadline)
    {
        int flags = 0;
        long long timeNs = 0;
        const int got = device->readStream(stream, buffs.data(), readElems, flags, timeNs,
                                           kReadTimeoutUs);
        if (got == SOAPY_SDR_TIMEOUT) { timeouts++; continue; }
        if (got == SOAPY_SDR_OVERFLOW) { overflows++; haveTick = false; continue; }
        if (got < 0)
        {
            std::printf("readStream error %d (%s)\n", got, SoapySDR::errToStr(got));
            break;
        }

        reads++;
        totalElems += static_cast<size_t>(got);
        if ((flags & SOAPY_SDR_MORE_FRAGMENTS) != 0) fragments++;
        if ((flags & SOAPY_SDR_HAS_TIME) != 0)
        {
            const long long tick = SoapySDR::timeNsToTicks(timeNs, rate);
            if (haveTick && std::llabs(tick - expectedTick) > 1) timeJumps++;
            expectedTick = tick + got;
            haveTick = true;
        }
    }

    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();

    size_t chanMask = 0;
    int statusFlags = 0;
    long long statusTime = 0;
    size_t statusEvents = 0;
    while (device->readStreamStatus(stream, chanMask, statusFlags, statusTime, 0) != SOAPY_SDR_TIMEOUT)
    {
        statusEvents++;
        if (statusEvents > 64) break;
    }

    device->deactivateStream(stream);
    device->closeStream(stream);
    SoapySDR::Device::unmake(device);

    std::printf("RESULT elems=%zu reads=%zu secs=%.3f sps=%.0f overflows=%zu timeouts=%zu "
                "fragments=%zu time_jumps=%zu status=%zu mtu=%zu read=%zu rate=%.0f\n",
                totalElems, reads, elapsed, elapsed > 0 ? totalElems / elapsed : 0.0, overflows,
                timeouts, fragments, timeJumps, statusEvents, mtu, readElems, rate);
    return totalElems > 0 ? 0 : 1;
}

int doTransmit(const std::map<std::string, std::string> &options)
{
    SoapySDR::Device *device = SoapySDR::Device::make(option(options, "args", "driver=plutosky"));
    const std::vector<size_t> channels = parseChannels(option(options, "channels", "0"));
    const std::string format = option(options, "format", SOAPY_SDR_CF32);
    const size_t burst = static_cast<size_t>(
        std::atoi(option(options, "burst", std::to_string(kDefaultBurstElems)).c_str()));
    const int bursts = std::atoi(option(options, "bursts", std::to_string(kDefaultBursts)).c_str());
    const double ahead = std::atof(
        option(options, "at", std::to_string(kDefaultScheduleAhead)).c_str());

    const auto rateArg = options.find("rate");
    if (rateArg != options.end())
    {
        device->setSampleRate(SOAPY_SDR_TX, 0, std::atof(rateArg->second.c_str()));
    }
    const double rate = device->getSampleRate(SOAPY_SDR_TX, channels.front());

    SoapySDR::Stream *stream =
        device->setupStream(SOAPY_SDR_TX, format, channels, streamArgs(options));

    const size_t elementBytes = SoapySDR::formatToSize(format);
    std::vector<std::vector<uint8_t>> storage(channels.size());
    std::vector<const void *> buffs(channels.size());
    for (size_t c = 0; c < channels.size(); c++)
    {
        storage[c].resize(burst * elementBytes);
        buffs[c] = storage[c].data();
        if (format == SOAPY_SDR_CF32)
        {
            std::complex<float> *samples =
                reinterpret_cast<std::complex<float> *>(storage[c].data());
            for (size_t n = 0; n < burst; n++)
            {
                const double phase = 2.0 * M_PI * kToneCycles * n / burst;
                samples[n] = std::complex<float>(static_cast<float>(std::cos(phase) * 0.5),
                                                 static_cast<float>(std::sin(phase) * 0.5));
            }
        }
        else if (format == SOAPY_SDR_CS16)
        {
            int16_t *samples = reinterpret_cast<int16_t *>(storage[c].data());
            for (size_t n = 0; n < burst; n++)
            {
                const double phase = 2.0 * M_PI * kToneCycles * n / burst;
                samples[n * 2 + 0] = static_cast<int16_t>(std::cos(phase) * 1000.0);
                samples[n * 2 + 1] = static_cast<int16_t>(std::sin(phase) * 1000.0);
            }
        }
    }

    device->activateStream(stream);

    size_t written = 0;
    int errors = 0;
    for (int b = 0; b < bursts; b++)
    {
        const long long timeNs = device->getHardwareTime()
                               + static_cast<long long>(ahead * 1e9)
                               + static_cast<long long>(b * (burst / rate) * 2e9);
        size_t offset = 0;
        while (offset < burst)
        {
            std::vector<const void *> chunk(channels.size());
            for (size_t c = 0; c < channels.size(); c++)
            {
                chunk[c] = storage[c].data() + offset * elementBytes;
            }
            int flags = SOAPY_SDR_HAS_TIME;
            if (offset == 0) flags = SOAPY_SDR_HAS_TIME;
            const bool finalChunk = true;
            if (finalChunk) flags |= SOAPY_SDR_END_BURST;
            const long long stamp = timeNs + static_cast<long long>(offset / rate * 1e9);
            const int got = device->writeStream(stream, chunk.data(), burst - offset, flags, stamp,
                                                1000000);
            if (got < 0)
            {
                std::printf("writeStream error %d (%s)\n", got, SoapySDR::errToStr(got));
                errors++;
                break;
            }
            offset += static_cast<size_t>(got);
            written += static_cast<size_t>(got);
        }
    }

    size_t chanMask = 0;
    int statusFlags = 0;
    long long statusTime = 0;
    size_t lateEvents = 0;
    size_t underruns = 0;
    int code = 0;
    while ((code = device->readStreamStatus(stream, chanMask, statusFlags, statusTime, 200000))
           != SOAPY_SDR_TIMEOUT)
    {
        if (code == SOAPY_SDR_TIME_ERROR) lateEvents++;
        if (code == SOAPY_SDR_UNDERFLOW) underruns++;
        if (lateEvents + underruns > 64) break;
    }

    device->deactivateStream(stream);
    device->closeStream(stream);
    SoapySDR::Device::unmake(device);

    std::printf("RESULT written=%zu bursts=%d errors=%d late=%zu underrun=%zu rate=%.0f\n",
                written, bursts, errors, lateEvents, underruns, rate);
    return errors == 0 && written > 0 ? 0 : 1;
}

int doDuplex(const std::map<std::string, std::string> &options)
{
    const double seconds = std::atof(option(options, "secs", "10").c_str());
    const double ahead = std::atof(option(options, "at", "0.1").c_str());
    const double deadlineGrace =
        std::atof(option(options, "deadline_grace", "2").c_str());
    const bool txOnly = option(options, "tx-only", "0") == "1";
    const bool diagnostic = option(options, "tx-diagnostic", "0") == "1";
    if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > kMaxDuplexSeconds ||
        !std::isfinite(ahead) || ahead < 0.0 || ahead > 1.0 ||
        !std::isfinite(deadlineGrace) || deadlineGrace < 0.0 ||
        deadlineGrace > kMaxDuplexDeadlineGraceSeconds)
    {
        std::fprintf(stderr,
                     "duplex requires 0 < secs <= 600, 0 <= at <= 1, and "
                     "0 <= deadline_grace <= 120\n");
        return 2;
    }

    const auto destroy = [](SoapySDR::Device *device) { SoapySDR::Device::unmake(device); };
    std::unique_ptr<SoapySDR::Device, decltype(destroy)> device(
        SoapySDR::Device::make(option(options, "args", "driver=plutosky")), destroy);
    const auto rateArg = options.find("rate");
    if (rateArg != options.end())
    {
        device->setSampleRate(SOAPY_SDR_RX, 0, std::atof(rateArg->second.c_str()));
    }
    const auto txRateArg = options.find("tx_rate");
    if (txRateArg != options.end())
    {
        device->setSampleRate(SOAPY_SDR_TX, 0, std::atof(txRateArg->second.c_str()));
    }
    const double rate = device->getSampleRate(SOAPY_SDR_RX, 0);
    const double txRate = device->getSampleRate(SOAPY_SDR_TX, 0);
    const auto freqArg = options.find("freq");
    if (freqArg != options.end())
    {
        const double freq = std::atof(freqArg->second.c_str());
        device->setFrequency(SOAPY_SDR_RX, 0, freq);
        device->setFrequency(SOAPY_SDR_TX, 0, freq);
    }
    device->setGain(SOAPY_SDR_TX, 0, device->getGainRange(SOAPY_SDR_TX, 0).minimum());

    SoapySDR::Stream *rx = nullptr;
    SoapySDR::Stream *tx = nullptr;
    bool rxActive = false;
    bool txActive = false;
    std::atomic<bool> stop(false);
    std::thread reader;
    size_t rxElems = 0;
    size_t rxOverflows = 0;
    size_t rxTimeJumps = 0;
    size_t rxErrors = 0;
    size_t txElems = 0;
    size_t txTimeouts = 0;
    size_t txErrors = 0;
    size_t txLate = 0;
    size_t txUnderrun = 0;
    size_t txGaps = 0;
    size_t targetElems = 0;
    double rxElapsed = 0.0;
    double txElapsed = 0.0;
    size_t rxMtu = 0;
    std::vector<int16_t> rxSamples;
    void *rxBuff[] = {nullptr};

    try
    {
        if (!txOnly)
            rx = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0}, streamArgs(options, "rx"));
        tx = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0}, streamArgs(options, "tx"));
        if (!txOnly) rxMtu = device->getStreamMTU(rx);
        const size_t txMtu = device->getStreamMTU(tx);
        rxSamples.resize(rxMtu * 2);
        rxBuff[0] = rxSamples.data();
        std::vector<int16_t> txSamples(txMtu * 2, 0);
        const void *txBuff[] = {txSamples.data()};

        if (!txOnly)
        {
            device->activateStream(rx);
            rxActive = true;
            reader = std::thread([&]() {
                bool haveTick = false;
                long long expectedTick = 0;
                bool haveStart = false;
                std::chrono::steady_clock::time_point firstRead;
                while (!stop.load())
                {
                    int flags = 0;
                    long long timeNs = 0;
                    const int got = device->readStream(rx, rxBuff, rxMtu, flags, timeNs,
                                                       kReadTimeoutUs);
                    if (got == SOAPY_SDR_TIMEOUT) continue;
                    if (got == SOAPY_SDR_OVERFLOW)
                    {
                        rxOverflows++;
                        haveTick = false;
                        continue;
                    }
                    if (got <= 0)
                    {
                        rxErrors++;
                        break;
                    }
                    const auto now = std::chrono::steady_clock::now();
                    if (!haveStart)
                    {
                        firstRead = now;
                        haveStart = true;
                    }
                    rxElapsed = std::chrono::duration<double>(now - firstRead).count();
                    rxElems += static_cast<size_t>(got);
                    if ((flags & SOAPY_SDR_HAS_TIME) == 0)
                    {
                        rxErrors++;
                        continue;
                    }
                    const long long tick = SoapySDR::timeNsToTicks(timeNs, rate);
                    if (haveTick && std::llabs(tick - expectedTick) > 1) rxTimeJumps++;
                    expectedTick = tick + got;
                    haveTick = true;
                }
            });
        }

        device->activateStream(tx);
        txActive = true;
        const long long firstTick = SoapySDR::timeNsToTicks(device->getHardwareTime(), txRate)
                                  + static_cast<long long>(ahead * txRate);
        targetElems = static_cast<size_t>(txRate * seconds);
        const auto sendStart = std::chrono::steady_clock::now();
        auto nextDiagnostic = sendStart + std::chrono::seconds(1);
        const auto deadline = sendStart +
                              std::chrono::duration<double>(seconds + ahead + deadlineGrace);
        while (txElems < targetElems && std::chrono::steady_clock::now() < deadline)
        {
            const size_t count = std::min(txMtu, targetElems - txElems);
            const long long stamp = SoapySDR::ticksToTimeNs(firstTick + txElems, txRate);
            int flags = SOAPY_SDR_HAS_TIME;
            if (txElems + count == targetElems) flags |= SOAPY_SDR_END_BURST;
            const int got = device->writeStream(tx, txBuff, count, flags, stamp, 250000);
            if (got == SOAPY_SDR_TIMEOUT)
            {
                txTimeouts++;
                continue;
            }
            if (got <= 0)
            {
                txErrors++;
                break;
            }
            txElems += static_cast<size_t>(got);
            if (diagnostic && std::chrono::steady_clock::now() >= nextDiagnostic)
            {
                const long long boardTick = SoapySDR::timeNsToTicks(device->getHardwareTime(), txRate);
                const double leadMs = 1000.0 * (firstTick + static_cast<long long>(txElems)
                                                - boardTick) / txRate;
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - sendStart).count();
                std::fprintf(stderr, "TX_DIAG elapsed=%.3f elems=%zu lead_ms=%.3f\n",
                             elapsed, txElems, leadMs);
                nextDiagnostic += std::chrono::seconds(1);
            }
        }
        txElapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - sendStart).count();
        const long long finalTick = firstTick + txElems
                                  + static_cast<long long>(kDuplexDrainSeconds * txRate);
        while (txElems && SoapySDR::timeNsToTicks(device->getHardwareTime(), txRate) < finalTick
               && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "duplex error: %s\n", error.what());
        txErrors++;
    }

    if (txActive)
    {
        try { device->deactivateStream(tx); }
        catch (const std::exception &error)
        {
            std::fprintf(stderr, "TX stop error: %s\n", error.what());
            txErrors++;
        }
    }
    stop.store(true);
    if (reader.joinable()) reader.join();
    if (rxActive) device->deactivateStream(rx);
    if (tx != nullptr)
    {
        size_t chanMask = 0;
        int statusFlags = 0;
        long long statusTime = 0;
        for (size_t n = 0; n < 64; n++)
        {
            const int code = device->readStreamStatus(tx, chanMask, statusFlags, statusTime, 0);
            if (code == SOAPY_SDR_TIMEOUT) break;
            if (code == SOAPY_SDR_TIME_ERROR) txLate++;
            if (code == SOAPY_SDR_UNDERFLOW) txUnderrun++;
            if (code == SOAPY_SDR_OVERFLOW) txGaps++;
        }
        device->closeStream(tx);
    }
    if (rx != nullptr) device->closeStream(rx);

    const double rxSps = rxElapsed > 0.0 ? rxElems / rxElapsed : 0.0;
    const double txSps = txElapsed > 0.0 ? txElems / txElapsed : 0.0;
    std::printf("RESULT rx_elems=%zu rx_sps=%.0f rx_overflows=%zu rx_time_jumps=%zu "
                "rx_errors=%zu tx_elems=%zu tx_sps=%.0f tx_timeouts=%zu tx_errors=%zu "
                "tx_late=%zu tx_underrun=%zu tx_gaps=%zu target_elems=%zu rate=%.0f tx_rate=%.0f "
                "secs=%.3f\n",
                rxElems, rxSps, rxOverflows, rxTimeJumps, rxErrors, txElems, txSps,
                txTimeouts, txErrors, txLate, txUnderrun, txGaps, targetElems, rate, txRate,
                seconds);
    return (txOnly || rxElems) && txElems == targetElems && !rxOverflows && !rxTimeJumps && !rxErrors
                   && !txErrors && !txLate && !txUnderrun && !txGaps ? 0 : 1;
}

}  // namespace

int main(int argc, char **argv)
{
    std::string mode;
    const auto options = parseOptions(argc, argv, mode);

    const auto verbosity = options.find("log");
    if (verbosity != options.end())
    {
        SoapySDR::setLogLevel(static_cast<SoapySDR::LogLevel>(std::atoi(verbosity->second.c_str())));
    }

    try
    {
        if (mode == "find") return doFind(options);
        if (mode == "probe") return doProbe(options);
        if (mode == "rate") return doReceive(options);
        if (mode == "tx") return doTransmit(options);
        if (mode == "duplex") return doDuplex(options);
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "plutosky_probe: %s\n", error.what());
        return 2;
    }

    std::fprintf(stderr, "usage: plutosky_probe --find|--probe|--rate|--tx|--duplex [--key=value ...]\n");
    return 2;
}
