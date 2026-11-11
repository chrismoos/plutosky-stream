#include "Device.hpp"
#include "HardwareKey.hpp"
#include "RxTimeOffset.hpp"
#include "IntegerHz.hpp"
#include "UhdSerial.hpp"

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <cstdlib>
#include <cmath>
#include <stdexcept>

namespace PlutoSky {

static const double kRxGainMinDb = -3.0;
static const double kRxGainMaxDb = 73.0;
static const double kRxGainStepDb = 1.0;
static const double kTxGainMinDb = 0.0;
static const double kTxGainMaxDb = 89.75;
static const double kTxGainStepDb = 0.25;
static const long long kNsPerSecond = 1000000000LL;
// SoapyUHD adds this to the arguments of every device it opens.
static const char *const kUhdTypeArg = "type";
static const char *const kUhdTypeSoapy = "soapy";
static const double kFrequencyMinHz = 70e6;
static const double kFrequencyMaxHz = 6e9;
static const double kBandwidthMinHz = 200e3;
static const double kBandwidthMaxHz = 20e6;
// The AD9363's maximum interface rate, the largest clock a USRP application would set.
static const double kMasterClockMaxHz = 61.44e6;

Direction SoapyPlutoSky::protoDirection(int direction)
{
    return direction == SOAPY_SDR_RX ? plutosky::v1::DIRECTION_RX : plutosky::v1::DIRECTION_TX;
}

SoapyPlutoSky::SoapyPlutoSky(const SoapySDR::Kwargs &args)
{
    std::string addr;
    int port = kDefaultControlPort;

    const auto addrArg = args.find("addr");
    if (addrArg != args.end()) addr = addrArg->second;
    const auto portArg = args.find("port");
    if (portArg != args.end()) port = std::atoi(portArg->second.c_str());

    if (addr.empty())
    {
        int discoveryPort = kDiscoveryPort;
        const auto discoveryArg = args.find("discovery_port");
        if (discoveryArg != args.end()) discoveryPort = std::atoi(discoveryArg->second.c_str());
        const auto found = discover(kDiscoveryTimeoutUs, discoveryPort);
        if (found.empty()) throw std::runtime_error("plutosky: no daemon answered the discovery probe");
        addr = found.front().addr;
        if (found.front().reply.control_port() != 0)
        {
            port = static_cast<int>(found.front().reply.control_port());
        }
    }

    // UHD applications expect sc16 at full 16-bit scale, as every USRP gives it.
    const auto typeArg = args.find(kUhdTypeArg);
    if (typeArg != args.end() && typeArg->second == kUhdTypeSoapy)
    {
        cs16Format_ = HostFormat::Cs16Msb;
        uhdCompatibility_ = true;
    }
    const auto scaleArg = args.find(kCs16ScaleArg);
    if (scaleArg != args.end() && !cs16FormatFromScale(scaleArg->second, cs16Format_))
    {
        throw std::runtime_error("plutosky: cs16_scale must be 12 or 16");
    }
    if (args.count("tx_send_rate_multiplier") || args.count("tx_pace_headroom"))
        throw std::runtime_error("plutosky: TX send-rate multipliers are removed; use duplex_tx_rate_limit_sps");
    const auto rateLimitArg = args.find("duplex_tx_rate_limit_sps");
    if (rateLimitArg != args.end())
        duplexTxRateLimitSps_ = TxPacer::parseRateLimit(rateLimitArg->second,
                                                        "duplex_tx_rate_limit_sps");
    const auto txOnlyLimitArg = args.find("tx_only_rate_limit_sps");
    if (txOnlyLimitArg != args.end())
        txOnlyRateLimitSps_ = TxPacer::parseRateLimit(txOnlyLimitArg->second,
                                                      "tx_only_rate_limit_sps");

    control_.reset(new ControlClient(addr, port));
    control_->setStatusHandler(
        [this](const plutosky::v1::StatusTick &tick) { this->onStatus(tick); });

    const BoardInfo &info = control_->info();
    serial_ = info.serial;
    hardwareKey_ = hardwareKey(args, serial_);
    rxTimeOffsetNs_ = parseRxTimeOffsetNs(args);
    numChannels_ = info.channels;
    txGainDb_.assign(numChannels_, kTxGainMinDb);
    txEnabled_.assign(numChannels_, false);
    minRate_ = info.minRate;
    maxRate_ = info.maxRate;
    wireFormats_ = info.formats;
    mtus_ = info.mtus;
    if (wireFormats_.empty()) wireFormats_.push_back("cs12");
    if (mtus_.empty()) mtus_.push_back(kDefaultMtu);

    refreshSampleRate();
}

SoapyPlutoSky::~SoapyPlutoSky()
{
    rxSocket_.reset();
    control_.reset();
}

std::string SoapyPlutoSky::getDriverKey(void) const
{
    return kDriverKey;
}

std::string SoapyPlutoSky::getHardwareKey(void) const
{
    return hardwareKey_;
}

SoapySDR::Kwargs SoapyPlutoSky::getHardwareInfo(void) const
{
    SoapySDR::Kwargs info;
    info["serial"] = uhdCompatibility_ ? uhdSerialAlias(serial_) : serial_;
    if (uhdCompatibility_) info["plutosky_serial"] = serial_;
    info["addr"] = control_->addr();
    info["control_port"] = std::to_string(control_->port());
    info["daemon_version"] = control_->info().version;
    std::string formats;
    for (const auto &name : wireFormats_)
    {
        if (!formats.empty()) formats += ",";
        formats += name;
    }
    info["wire_formats"] = formats;
    std::string mtus;
    for (const auto mtu : mtus_)
    {
        if (!mtus.empty()) mtus += ",";
        mtus += std::to_string(mtu);
    }
    info["mtus"] = mtus;
    return info;
}

size_t SoapyPlutoSky::getNumChannels(const int) const
{
    return numChannels_;
}

bool SoapyPlutoSky::getFullDuplex(const int, const size_t) const
{
    return true;
}

long long SoapyPlutoSky::rxOffsetTicks() const
{
    return rxTimeOffsetSamples(rxTimeOffsetNs_, rates_.rxHz) * rates_.rxTicksPerSample
         + rates_.rxDelayTicks;
}

uint64_t SoapyPlutoSky::txBoardTicks(long long timeNs) const
{
    // The timebase starts at zero, so a negative time is simply in the past.
    // The board must see it as late rather than as a count near 2^64.
    const long long ticks = SoapySDR::timeNsToTicks(timeNs, rates_.tickRateHz)
                          - static_cast<long long>(rates_.txDelayTicks);
    return ticks < 0 ? 0 : static_cast<uint64_t>(ticks);
}

void SoapyPlutoSky::refreshSampleRate() const
{
    rates_ = control_->sampleRates();
}

std::vector<std::string> SoapyPlutoSky::listAntennas(const int direction, const size_t) const
{
    return control_->antennas(protoDirection(direction));
}

void SoapyPlutoSky::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    antennas_[direction][channel] = control_->setAntenna(protoDirection(direction), channel, name);
}

std::string SoapyPlutoSky::getAntenna(const int direction, const size_t channel) const
{
    const auto byDir = antennas_.find(direction);
    if (byDir != antennas_.end())
    {
        const auto byChan = byDir->second.find(channel);
        if (byChan != byDir->second.end()) return byChan->second;
    }
    const auto names = listAntennas(direction, channel);
    return names.empty() ? std::string() : names.front();
}

std::vector<std::string> SoapyPlutoSky::listGains(const int, const size_t) const
{
    return std::vector<std::string>(1, kGainElement);
}

bool SoapyPlutoSky::hasGainMode(const int direction, const size_t) const
{
    return direction == SOAPY_SDR_RX;
}

void SoapyPlutoSky::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    writeSetting(direction, channel, "gain_mode", automatic ? "slow_attack" : "manual");
}

bool SoapyPlutoSky::getGainMode(const int direction, const size_t channel) const
{
    const auto byDir = gainModes_.find(direction);
    if (byDir == gainModes_.end()) return false;
    const auto byChan = byDir->second.find(channel);
    if (byChan == byDir->second.end()) return false;
    return byChan->second != "manual";
}

void SoapyPlutoSky::setGain(const int direction, const size_t channel, const double value)
{
    if (direction == SOAPY_SDR_TX)
    {
        if (!std::isfinite(value) || value < kTxGainMinDb || value > kTxGainMaxDb)
            throw std::runtime_error("plutosky: TX gain must be between 0 and 89.75 dB");
        std::lock_guard<std::mutex> lock(txGainMutex_);
        const double gain = std::round(value / kTxGainStepDb) * kTxGainStepDb;
        control_->setGain(plutosky::v1::DIRECTION_TX, channel,
                          txEnabled_.at(channel) ? gain : kTxGainMinDb);
        txGainDb_.at(channel) = gain;
        return;
    }
    control_->setGain(protoDirection(direction), channel, value);
}

double SoapyPlutoSky::getGain(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_TX)
    {
        std::lock_guard<std::mutex> lock(txGainMutex_);
        return txGainDb_.at(channel);
    }
    return control_->gain(protoDirection(direction), channel);
}

void SoapyPlutoSky::setTxEnabled(size_t channel, bool enabled)
{
    std::lock_guard<std::mutex> lock(txGainMutex_);
    control_->setGain(plutosky::v1::DIRECTION_TX, channel,
                      enabled ? txGainDb_.at(channel) : kTxGainMinDb);
    txEnabled_.at(channel) = enabled;
}

SoapySDR::Range SoapyPlutoSky::getGainRange(const int direction, const size_t) const
{
    if (direction == SOAPY_SDR_TX) return SoapySDR::Range(kTxGainMinDb, kTxGainMaxDb, kTxGainStepDb);
    return SoapySDR::Range(kRxGainMinDb, kRxGainMaxDb, kRxGainStepDb);
}

static void checkGainElement(const std::string &name)
{
    if (name != kGainElement) throw std::runtime_error("plutosky: no gain element " + name);
}

void SoapyPlutoSky::setGain(const int direction, const size_t channel, const std::string &name,
                            const double value)
{
    checkGainElement(name);
    setGain(direction, channel, value);
}

double SoapyPlutoSky::getGain(const int direction, const size_t channel,
                              const std::string &name) const
{
    checkGainElement(name);
    return getGain(direction, channel);
}

SoapySDR::Range SoapyPlutoSky::getGainRange(const int direction, const size_t channel,
                                            const std::string &name) const
{
    checkGainElement(name);
    return getGainRange(direction, channel);
}

void SoapyPlutoSky::setFrequency(const int direction, const size_t channel, const double frequency,
                                 const SoapySDR::Kwargs &)
{
    control_->setFrequency(protoDirection(direction), channel,
                           static_cast<uint64_t>(integerHz(frequency, "frequency")));
}

double SoapyPlutoSky::getFrequency(const int direction, const size_t channel) const
{
    return static_cast<double>(control_->frequency(protoDirection(direction), channel));
}

std::vector<std::string> SoapyPlutoSky::listFrequencies(const int, const size_t) const
{
    return std::vector<std::string>(1, kFrequencyComponent);
}

SoapySDR::RangeList SoapyPlutoSky::getFrequencyRange(const int, const size_t) const
{
    return SoapySDR::RangeList(1, SoapySDR::Range(kFrequencyMinHz, kFrequencyMaxHz));
}

static void checkFrequencyComponent(const std::string &name)
{
    if (name != kFrequencyComponent)
    {
        throw std::runtime_error("plutosky: no tunable component " + name);
    }
}

void SoapyPlutoSky::setFrequency(const int direction, const size_t channel, const std::string &name,
                                 const double frequency, const SoapySDR::Kwargs &args)
{
    checkFrequencyComponent(name);
    setFrequency(direction, channel, frequency, args);
}

double SoapyPlutoSky::getFrequency(const int direction, const size_t channel,
                                   const std::string &name) const
{
    checkFrequencyComponent(name);
    return getFrequency(direction, channel);
}

SoapySDR::RangeList SoapyPlutoSky::getFrequencyRange(const int direction, const size_t channel,
                                                     const std::string &name) const
{
    checkFrequencyComponent(name);
    return getFrequencyRange(direction, channel);
}

void SoapyPlutoSky::setSampleRate(const int direction, const size_t, const double rate)
{
    // The daemon refuses a rate change while any stream runs, parked ones included.
    stopParkedTxStreams();
    forgetBoardTimeEstimate();
    rates_ = control_->setSampleRate(protoDirection(direction),
                                     static_cast<uint32_t>(integerHz(rate, "sample rate")));
}

double SoapyPlutoSky::getSampleRate(const int direction, const size_t) const
{
    refreshSampleRate();
    return sampleRate(direction);
}

SoapySDR::RangeList SoapyPlutoSky::getSampleRateRange(const int, const size_t) const
{
    return SoapySDR::RangeList(1, SoapySDR::Range(minRate_, maxRate_));
}

void SoapyPlutoSky::setBandwidth(const int direction, const size_t channel, const double bw)
{
    control_->setBandwidth(protoDirection(direction), channel,
                           static_cast<uint32_t>(integerHz(bw, "bandwidth")));
}

double SoapyPlutoSky::getBandwidth(const int direction, const size_t channel) const
{
    return static_cast<double>(control_->bandwidth(protoDirection(direction), channel));
}

SoapySDR::RangeList SoapyPlutoSky::getBandwidthRange(const int, const size_t) const
{
    return SoapySDR::RangeList(1, SoapySDR::Range(kBandwidthMinHz, kBandwidthMaxHz));
}

// The board has no master clock apart from the rate its timebase counts at. A requested rate
// is stored and reported back so applications written for USRPs can set one,
// and it changes nothing on the board.
void SoapyPlutoSky::setMasterClockRate(const double rate)
{
    if (!(rate > 0.0))
    {
        throw std::runtime_error("plutosky: master clock rate must be positive");
    }
    masterClockRate_ = rate;
}

double SoapyPlutoSky::getMasterClockRate(void) const
{
    if (masterClockRate_ > 0.0) return masterClockRate_;
    refreshSampleRate();
    return rates_.tickRateHz;
}

SoapySDR::RangeList SoapyPlutoSky::getMasterClockRates(void) const
{
    return SoapySDR::RangeList(1, SoapySDR::Range(minRate_, kMasterClockMaxHz));
}

std::vector<std::string> SoapyPlutoSky::listClockSources(void) const
{
    return std::vector<std::string>(1, kInternalSource);
}

void SoapyPlutoSky::setClockSource(const std::string &source)
{
    if (source != kInternalSource)
    {
        throw std::runtime_error("plutosky: clock source " + source + " is not available");
    }
}

std::string SoapyPlutoSky::getClockSource(void) const
{
    return kInternalSource;
}

std::vector<std::string> SoapyPlutoSky::listTimeSources(void) const
{
    return std::vector<std::string>(1, kInternalSource);
}

void SoapyPlutoSky::setTimeSource(const std::string &source)
{
    if (source != kInternalSource)
    {
        throw std::runtime_error("plutosky: time source " + source + " is not available");
    }
}

std::string SoapyPlutoSky::getTimeSource(void) const
{
    return kInternalSource;
}

bool SoapyPlutoSky::hasHardwareTime(const std::string &what) const
{
    return what.empty();
}

// There is no PPS input. For "PPS" the whole seconds of the timebase stand in
// for one, so code that waits for a PPS edge sees one every second.
long long SoapyPlutoSky::getHardwareTime(const std::string &what) const
{
    const auto ticks = static_cast<long long>(control_->hardwareTime());
    const long long timeNs = SoapySDR::ticksToTimeNs(ticks, rates_.tickRateHz);
    if (what == kTimePps) return timeNs - timeNs % kNsPerSecond;
    return timeNs;
}

// "PPS" sets the time immediately. "CMD" is UHD's command time, which never
// moves the timebase because timed commands are not supported.
void SoapyPlutoSky::setHardwareTime(const long long timeNs, const std::string &what)
{
    if (what == kTimeCommand)
    {
        if (timeNs != 0 && !warnedCommandTime_)
        {
            SoapySDR::log(SOAPY_SDR_WARNING,
                          "plutosky: timed commands are not supported, they apply immediately");
            warnedCommandTime_ = true;
        }
        return;
    }
    forgetBoardTimeEstimate();
    control_->setHardwareTime(
        static_cast<uint64_t>(SoapySDR::timeNsToTicks(timeNs, rates_.tickRateHz)));
}

std::vector<std::string> SoapyPlutoSky::listSensors(void) const
{
    std::vector<std::string> keys;
    keys.push_back("temperature_c");
    keys.push_back("rx_lo_locked");
    keys.push_back("tx_lo_locked");
    return keys;
}

SoapySDR::ArgInfo SoapyPlutoSky::getSensorInfo(const std::string &key) const
{
    SoapySDR::ArgInfo info;
    info.key = key;
    info.name = key;
    if (key == "temperature_c")
    {
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.units = "C";
        info.description = "AD9363 die temperature";
    }
    else
    {
        info.type = SoapySDR::ArgInfo::BOOL;
        info.description = "Synthesizer lock";
    }
    return info;
}

std::string SoapyPlutoSky::readSensor(const std::string &key) const
{
    const Sensors sensors = control_->sensors();
    if (key == "temperature_c") return std::to_string(sensors.temperatureC);
    if (key == "rx_lo_locked") return sensors.rxLoLocked ? "true" : "false";
    if (key == "tx_lo_locked") return sensors.txLoLocked ? "true" : "false";
    if (key == "tx_rf_enabled") return sensors.txRfEnabled ? "true" : "false";
    return std::string();
}

std::vector<std::string> SoapyPlutoSky::listSensors(const int direction, const size_t) const
{
    std::vector<std::string> keys;
    if (direction == SOAPY_SDR_RX) keys.push_back("rssi");
    keys.push_back("lo_locked");
    return keys;
}

SoapySDR::ArgInfo SoapyPlutoSky::getSensorInfo(const int, const size_t,
                                               const std::string &key) const
{
    SoapySDR::ArgInfo info;
    info.key = key;
    info.name = key;
    if (key == "rssi")
    {
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.units = "dB";
        info.description = "Received signal strength";
    }
    else
    {
        info.type = SoapySDR::ArgInfo::BOOL;
        info.description = "Synthesizer lock for this direction";
    }
    return info;
}

std::string SoapyPlutoSky::readSensor(const int direction, const size_t channel,
                                      const std::string &key) const
{
    if (key == "lo_locked")
    {
        return readSensor(direction == SOAPY_SDR_RX ? "rx_lo_locked" : "tx_lo_locked");
    }
    if (direction != SOAPY_SDR_RX || key != "rssi") return std::string();
    const Sensors sensors = control_->sensors();
    if (channel >= sensors.rssiDbfs.size()) return std::string();
    return std::to_string(sensors.rssiDbfs[channel]);
}

SoapySDR::ArgInfoList SoapyPlutoSky::getSettingInfo(void) const
{
    return SoapySDR::ArgInfoList();
}

void SoapyPlutoSky::writeSetting(const std::string &, const std::string &)
{
}

std::string SoapyPlutoSky::readSetting(const std::string &) const
{
    return std::string();
}

SoapySDR::ArgInfoList SoapyPlutoSky::getSettingInfo(const int direction, const size_t) const
{
    SoapySDR::ArgInfoList settings;
    if (direction != SOAPY_SDR_RX) return settings;
    SoapySDR::ArgInfo mode;
    mode.key = "gain_mode";
    mode.name = "Gain mode";
    mode.description = "AD9363 gain control mode";
    mode.type = SoapySDR::ArgInfo::STRING;
    mode.value = "manual";
    mode.options.push_back("manual");
    mode.options.push_back("slow_attack");
    mode.options.push_back("fast_attack");
    settings.push_back(mode);
    return settings;
}

void SoapyPlutoSky::writeSetting(const int direction, const size_t channel, const std::string &key,
                                 const std::string &value)
{
    if (key != "gain_mode") return;
    gainModes_[direction][channel] = control_->setGainMode(protoDirection(direction), channel, value);
}

std::string SoapyPlutoSky::readSetting(const int direction, const size_t channel,
                                       const std::string &key) const
{
    if (key != "gain_mode") return std::string();
    const auto byDir = gainModes_.find(direction);
    if (byDir == gainModes_.end()) return "manual";
    const auto byChan = byDir->second.find(channel);
    return byChan == byDir->second.end() ? std::string("manual") : byChan->second;
}

}  // namespace PlutoSky
