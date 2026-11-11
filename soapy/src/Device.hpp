// SoapySDR device for the PlutoSky VITA 49.2 streamer.

#pragma once

#include "Control.hpp"
#include "StatusLog.hpp"
#include "Convert.hpp"
#include "Vita.hpp"
#include "TxPacer.hpp"

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>

#include <atomic>
#include <limits>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace PlutoSky {

static const char *const kDriverKey = "plutosky";
static const size_t kDefaultMtu = 1500;
static const WireFormat kDefaultWireFormat = WireFormat::Cs12;
static const size_t kDefaultRxQueuePackets = 1024;
static const size_t kMaxDatagramBytes = 9216;
static const int kRxSocketBufferBytes = 16 * 1024 * 1024;
static const size_t kMaxStatusItems = 64;
static const char *const kGainElement = "PGA";
static const char *const kFrequencyComponent = "RF";
static const char *const kInternalSource = "internal";
static const char *const kTimePps = "PPS";
static const char *const kTimeCommand = "CMD";
static const char *const kCs16ScaleArg = "cs16_scale";
static const int64_t kStatusLogIntervalMs = 1000;

struct RxPacket {
    uint64_t ticks = 0;
    uint32_t count = 0;
    size_t samples = 0;
    size_t consumedElems = 0;
    bool endOfBurst = false;
    std::vector<uint8_t> payload;
};

struct StatusItem {
    int code = 0;
    int flags = 0;
    long long timeNs = 0;
};

struct TxBurstEnd
{
    // What the daemon's consumed counter reads once the packet has played.
    uint64_t consumed = 0;
    // The tick after the burst's last sample.
    uint64_t endTicks = 0;
};

struct PlutoSkyStream {
    int direction = 0;
    std::vector<size_t> channels;
    WireFormat wire = kDefaultWireFormat;
    HostFormat host = HostFormat::Cs16;
    size_t mtu = kDefaultMtu;
    bool running = false;  // the daemon side, which can outlive activation on transmit
    size_t samplesPerPacket = 0;  // interleaved, as the daemon reports it
    size_t elemsPerPacket = 0;    // per channel
    size_t window = 0;
    size_t queueLimit = kDefaultRxQueuePackets;
    int64_t streamId = -1;
    int dataPort = 0;
    bool active = false;

    // Receive side.
    std::mutex mutex;
    std::condition_variable cv;
    std::deque<RxPacket> queue;
    bool haveCount = false;
    uint32_t lastCount = 0;
    bool overflowPending = false;
    bool accepting = false;
    // Elements left in a finite receive burst, when one was asked for.
    bool burstLimited = false;
    size_t burstRemaining = 0;
    bool burstStopPending = false;

    // Transmit side.
    int txSocket = -1;
    uint32_t txCount = 0;
    uint64_t txTicks = 0;
    bool txTicksValid = false;
    bool txBurstOpen = false;
    TxPacer txPacer;
    double duplexTxRateLimitSps = TxPacer::kRateLimitUnset;
    double txOnlyRateLimitSps = TxPacer::kDefaultTxOnlyRateLimitSps;
    size_t txPackets = 0;
    size_t txDuplexPackets = 0;
    size_t txWindowWaits = 0;
    long long txWindowWaitTotalUs = 0;
    long long txWindowWaitMaxUs = 0;
    size_t txPaceWaits = 0;
    long long txMaxSendGapUs = 0;
    long long txMaxSteadySendGapUs = 0;
    long long txMaxSteadyWindowWaitUs = 0;
    std::atomic<uint64_t> txFirstTick{0};
    // Smallest time, in microseconds, by which a sent packet's first sample
    // led the estimated board time, since the last margin report.
    std::atomic<int64_t> txMinMarginUs{std::numeric_limits<int64_t>::max()};
    // Busy wait after each pacing sleep, in ns, adapted to how late sleeps
    // wake on this host. Written by the writer thread, logged by the status
    // thread.
    std::atomic<int64_t> txSpinTailNs{75000};
    int64_t txMarginLogMs = 0;
    bool txPastStartup = false;
    bool txSendQueueWarned = false;
    TxPacer::Time txLastSent{};
    std::atomic<uint64_t> consumedTicks{0};
    // What the daemon's consumed counter reads once each packet not yet
    // reported as consumed has played. Its depth is what the window limits.
    std::deque<uint64_t> txPending;
    // The last element of a one-channel cs8 write of odd length cannot go out
    // exactly on its own. It is held in host format, per channel, and sent at
    // the front of the next write.
    std::vector<std::vector<uint8_t>> txCarry;
    size_t txCarryElems = 0;
    std::vector<std::vector<uint8_t>> txJoined;
    std::vector<const void *> txJoinedPtrs;
    // Deactivating after an end of burst parks the stream: the daemon stream
    // keeps running so the next burst starts without a stop and start, and the
    // parking thread stops it once it has stayed parked for the idle delay.
    std::mutex parkMutex;
    std::condition_variable parkCv;
    bool parked = false;
    bool closing = false;
    uint64_t parkGeneration = 0;
    std::thread parkThread;
    // Steady-clock milliseconds of the last write, and whether the module
    // stopped the stream because writes stopped while it ran.
    std::atomic<int64_t> txLastWriteMs{0};
    std::atomic<bool> txStarvedStop{false};

    // Status events.
    std::mutex statusMutex;
    std::condition_variable statusCv;
    std::deque<StatusItem> status;
    // Every end-of-burst packet not yet consumed, for burst acks.
    std::deque<TxBurstEnd> burstEnds;
    uint64_t lastLate = 0;
    uint64_t lastUnderrun = 0;
    uint64_t lastOverflow = 0;
    uint64_t lastGaps = 0;
    StatusLogLimiter timingLog{kStatusLogIntervalMs};
    StatusLogLimiter lossLog{kStatusLogIntervalMs};

    std::vector<Sample> staging;
    std::vector<uint8_t> packet;
};

// One UDP socket shared by every receive stream on a device, demultiplexing
// incoming VITA packets by stream id.
class RxSocket {
public:
    RxSocket();
    ~RxSocket();

    int port() const { return port_; }
    void registerStream(int64_t streamId, PlutoSkyStream *stream);
    void unregisterStream(int64_t streamId);
    void sendHello(const std::string &addr, int dataPort, int64_t streamId, uint16_t packetClass);

private:
    void loop();

    int sock_ = -1;
    int port_ = 0;
    std::atomic<bool> running_{false};
    std::mutex mutex_;
    std::map<int64_t, PlutoSkyStream *> streams_;
    std::thread thread_;
};

class SoapyPlutoSky : public SoapySDR::Device {
public:
    explicit SoapyPlutoSky(const SoapySDR::Kwargs &args);
    ~SoapyPlutoSky() override;

    std::string getDriverKey(void) const override;
    std::string getHardwareKey(void) const override;
    SoapySDR::Kwargs getHardwareInfo(void) const override;

    size_t getNumChannels(const int direction) const override;
    bool getFullDuplex(const int direction, const size_t channel) const override;

    std::vector<std::string> getStreamFormats(const int direction,
                                              const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel,
                                      double &fullScale) const override;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction,
                                            const size_t channel) const override;

    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<size_t> &channels,
                                  const SoapySDR::Kwargs &args) override;
    void closeStream(SoapySDR::Stream *stream) override;
    size_t getStreamMTU(SoapySDR::Stream *stream) const override;
    int activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs,
                       const size_t numElems) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags,
                         const long long timeNs) override;
    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags,
                   long long &timeNs, const long timeoutUs) override;
    int writeStream(SoapySDR::Stream *stream, const void *const *buffs, const size_t numElems,
                    int &flags, const long long timeNs, const long timeoutUs) override;
    int readStreamStatus(SoapySDR::Stream *stream, size_t &chanMask, int &flags, long long &timeNs,
                         const long timeoutUs) override;

    std::vector<std::string> listAntennas(const int direction,
                                          const size_t channel) const override;
    void setAntenna(const int direction, const size_t channel, const std::string &name) override;
    std::string getAntenna(const int direction, const size_t channel) const override;

    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    bool hasGainMode(const int direction, const size_t channel) const override;
    void setGainMode(const int direction, const size_t channel, const bool automatic) override;
    bool getGainMode(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const double value) override;
    double getGain(const int direction, const size_t channel) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const std::string &name,
                 const double value) override;
    double getGain(const int direction, const size_t channel,
                   const std::string &name) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel,
                                 const std::string &name) const override;

    void setFrequency(const int direction, const size_t channel, const double frequency,
                      const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel) const override;
    void setFrequency(const int direction, const size_t channel, const std::string &name,
                      const double frequency, const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel,
                        const std::string &name) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel,
                                          const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int direction,
                                             const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction,
                                          const size_t channel) const override;

    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getSampleRateRange(const int direction,
                                           const size_t channel) const override;

    void setBandwidth(const int direction, const size_t channel, const double bw) override;
    double getBandwidth(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getBandwidthRange(const int direction,
                                          const size_t channel) const override;

    void setMasterClockRate(const double rate) override;
    double getMasterClockRate(void) const override;
    SoapySDR::RangeList getMasterClockRates(void) const override;

    std::vector<std::string> listClockSources(void) const override;
    void setClockSource(const std::string &source) override;
    std::string getClockSource(void) const override;
    std::vector<std::string> listTimeSources(void) const override;
    void setTimeSource(const std::string &source) override;
    std::string getTimeSource(void) const override;

    bool hasHardwareTime(const std::string &what) const override;
    long long getHardwareTime(const std::string &what) const override;
    void setHardwareTime(const long long timeNs, const std::string &what) override;

    std::vector<std::string> listSensors(void) const override;
    SoapySDR::ArgInfo getSensorInfo(const std::string &key) const override;
    std::string readSensor(const std::string &key) const override;
    std::vector<std::string> listSensors(const int direction,
                                         const size_t channel) const override;
    SoapySDR::ArgInfo getSensorInfo(const int direction, const size_t channel,
                                    const std::string &key) const override;
    std::string readSensor(const int direction, const size_t channel,
                           const std::string &key) const override;

    SoapySDR::ArgInfoList getSettingInfo(void) const override;
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;
    SoapySDR::ArgInfoList getSettingInfo(const int direction, const size_t channel) const override;
    void writeSetting(const int direction, const size_t channel, const std::string &key,
                      const std::string &value) override;
    std::string readSetting(const int direction, const size_t channel,
                            const std::string &key) const override;

    double sampleRate(int direction) const
    {
        return direction == SOAPY_SDR_TX ? rates_.txHz : rates_.rxHz;
    }
    // Timebase counts between a receive sample's board timestamp and the time
    // the client sees: the receive time offset, a whole number of receive
    // samples, and the board's receive filter delay.
    long long rxOffsetTicks() const;
    // The timestamp to send so a sample reaches the radio at timeNs, ahead of
    // it by the board's transmit filter delay.
    uint64_t txBoardTicks(long long timeNs) const;

private:
    static Direction protoDirection(int direction);
    void onStatus(const plutosky::v1::StatusTick &tick);
    void onStreamStatus(const plutosky::v1::StreamStatus &status, uint64_t ticks);
    void recordSendMargin(PlutoSkyStream *stream);
    bool estimateBoardTicks(double &ticks) const;
    void forgetBoardTimeEstimate();
    void pushStatus(PlutoSkyStream *stream, int code, int flags, long long timeNs);
    void startDaemonStream(PlutoSkyStream *stream, bool timed, uint64_t atTicks);
    void stopDaemonStream(PlutoSkyStream *stream);
    void parkTxStream(PlutoSkyStream *stream);
    void parkTxStreamLocked(PlutoSkyStream *stream);
    void runTxParking(PlutoSkyStream *stream);
    void stopParkedTxStream(PlutoSkyStream *stream);
    void stopParkedTxStreams();
    void setTxEnabled(size_t channel, bool enabled);
    PlutoSkyStream *lookup(int64_t streamId);
    void refreshSampleRate() const;
    RxSocket &rxSocket();

    // Board timebase from the latest status tick, and the host steady clock
    // when it arrived, for estimating board time between ticks.
    std::atomic<uint64_t> statusTicks_{0};
    std::atomic<int64_t> statusHostNs_{0};
    // Board clock minus host clock, in ticks, from the least delayed recent
    // status tick, and the host time it was last updated.
    std::atomic<double> boardOffsetTicks_{0.0};
    std::atomic<int64_t> boardOffsetHostNs_{0};
    void updateBoardClockOffset(uint64_t boardTicks, int64_t hostNs);
    // Status-thread only.
    int64_t libraryThreadCheckNs_ = 0;
    std::unique_ptr<ControlClient> control_;
    std::string serial_;
    std::string hardwareKey_;
    long long rxTimeOffsetNs_ = 0;
    bool uhdCompatibility_ = false;
    size_t numChannels_ = 1;
    double minRate_ = 0.0;
    double masterClockRate_ = 0.0;
    double maxRate_ = 0.0;
    std::vector<std::string> wireFormats_;
    std::vector<size_t> mtus_;
    HostFormat cs16Format_ = HostFormat::Cs16;
    double duplexTxRateLimitSps_ = TxPacer::kRateLimitUnset;
    double txOnlyRateLimitSps_ = TxPacer::kDefaultTxOnlyRateLimitSps;
    std::atomic<size_t> activeRxStreams_{0};
    bool warnedCommandTime_ = false;
    mutable std::mutex txGainMutex_;
    std::vector<double> txGainDb_;
    std::vector<bool> txEnabled_;

    // Timestamps on the wire count at tickRateHz, not at either sample rate.
    mutable SampleRates rates_;
    mutable std::map<int, std::map<size_t, std::string>> antennas_;
    mutable std::map<int, std::map<size_t, std::string>> gainModes_;

    std::unique_ptr<RxSocket> rxSocket_;
    std::mutex streamsMutex_;
    std::map<int64_t, PlutoSkyStream *> streamsById_;
};

}  // namespace PlutoSky
