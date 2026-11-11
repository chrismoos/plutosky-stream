#include "Device.hpp"

#ifdef __APPLE__
#include <mach/mach_time.h>
#endif
#ifdef __linux__
#include <sys/prctl.h>
#endif
#include "RxTimeOffset.hpp"

#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace PlutoSky {

// The busy wait after each pacing sleep, bounded. TX packets must leave on
// their slots because the board drops frames that arrive back to back while
// it also sends, and a plain sleep wakes too late to hold the spacing.
static const auto kTxSpinTailMin = std::chrono::microseconds(10);
// How long a send waits out a full interface queue, and the wait between tries.
static const auto kTxSendRetryLimit = std::chrono::milliseconds(20);
static const auto kTxSendRetryWait = std::chrono::microseconds(50);
static const auto kTxSpinTailMax = std::chrono::microseconds(500);
static const auto kTxSpinTailMargin = std::chrono::microseconds(5);
// A measured sleep that overshot by more than the tail sets it to that
// overshoot plus the margin. A shorter one moves it 1/kTxSpinTailGain of the
// way down. Each wait too short to sleep shortens it by 1/kTxSpinTailDecay,
// since a tail longer than the gap never measures again.
static const long long kTxSpinTailGain = 64;
static const long long kTxSpinTailDecay = 64;
#ifdef __linux__
static const unsigned long kTxTimerSlackNs = 1;
#endif
static const long kRxSocketTimeoutUs = 100000;
static const auto kTxWindowPollInterval = std::chrono::microseconds(200);
// A window wait this long is logged.
static const long long kTxWindowWaitLogUs = 10000;
static const int64_t kTxMarginLogIntervalMs = 10000;
// Largest drift assumed between the board and host clocks.
static const double kBoardClockDriftPpm = 100.0;
static const int64_t kLibraryThreadCheckNs = 5000000000;
static const double kNsPerSecondF = 1e9;
static const double kUsPerSecondF = 1e6;
static const double kTxStartupDiagnosticSeconds = 0.2;
// How far ahead of the current hardware time an untimed transmit burst starts,
// so its first packet is not late when it reaches the board.
static const long long kTxUntimedLeadNs = 10000000;
static const auto kTxEndBurstDrainTimeout = std::chrono::seconds(1);
// How long a transmit stream stays parked after an end of burst before the
// module mutes TX and stops it. osmo-trx resynchronizes by ending a burst and
// starting the next about 30 chunks later, well inside this.
static const auto kTxParkedStopDelay = std::chrono::milliseconds(500);
// How long a running transmit stream may go without a write while the board
// reports underruns before the module parks it. Ends the underrun reports of a
// client that is shutting down without closing its stream.
static const int64_t kTxStarvedStopDelayMs = 1000;
static const char *const kUhdWireArg = "WIRE";

static int64_t steadyMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// SoapyUHD passes UHD's otw_format as WIRE. sc16 is UHD's default and selects
// none of the wire packings, so it keeps the default wire format.
static bool wireFormatFromUhd(const std::string &otw, std::string &wireName)
{
    if (otw == "sc16") return true;
    if (otw == "sc12") { wireName = "cs12"; return true; }
    if (otw == "sc8") { wireName = "cs8"; return true; }
    WireFormat known;
    if (wireFormatFromName(otw, known)) { wireName = otw; return true; }
    return false;
}

RxSocket::RxSocket()
{
    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) throw std::runtime_error("plutosky: cannot create a data socket");

    int bufferBytes = kRxSocketBufferBytes;
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVBUF, &bufferBytes, sizeof(bufferBytes));

    // shutdown() need not wake recv() on an unconnected UDP socket.
    timeval receiveTimeout{};
    receiveTimeout.tv_usec = kRxSocketTimeoutUs;
    if (::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO,
                     &receiveTimeout, sizeof(receiveTimeout)) != 0)
    {
        ::close(sock_);
        throw std::runtime_error("plutosky: cannot set receive socket timeout");
    }

    sockaddr_in local;
    std::memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = 0;
    if (::bind(sock_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0)
    {
        ::close(sock_);
        throw std::runtime_error("plutosky: cannot bind the data socket");
    }
    socklen_t length = sizeof(local);
    ::getsockname(sock_, reinterpret_cast<sockaddr *>(&local), &length);
    port_ = ntohs(local.sin_port);

    running_ = true;
    thread_ = std::thread(&RxSocket::loop, this);
}

RxSocket::~RxSocket()
{
    running_ = false;
    if (sock_ >= 0) ::shutdown(sock_, SHUT_RDWR);
    if (thread_.joinable()) thread_.join();
    if (sock_ >= 0) ::close(sock_);
}

void RxSocket::registerStream(int64_t streamId, PlutoSkyStream *stream)
{
    std::lock_guard<std::mutex> lock(mutex_);
    streams_[streamId] = stream;
}

void RxSocket::unregisterStream(int64_t streamId)
{
    std::lock_guard<std::mutex> lock(mutex_);
    streams_.erase(streamId);
}

void RxSocket::sendHello(const std::string &addr, int dataPort, int64_t streamId,
                         uint16_t packetClass)
{
    uint8_t datagram[kVitaHeaderBytes];
    writePacketHeader(datagram, static_cast<uint32_t>(streamId), packetClass, 0, 0, 0, false);

    sockaddr_in dest;
    std::memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<uint16_t>(dataPort));
    if (::inet_pton(AF_INET, addr.c_str(), &dest.sin_addr) != 1) return;
    ::sendto(sock_, datagram, sizeof(datagram), 0, reinterpret_cast<sockaddr *>(&dest),
             sizeof(dest));
}

void RxSocket::loop()
{
    std::vector<uint8_t> buffer(kMaxDatagramBytes);
    while (running_)
    {
        const ssize_t got = ::recv(sock_, buffer.data(), buffer.size(), 0);
        if (got <= 0)
        {
            if (!running_) break;
            continue;
        }

        PacketHeader header;
        if (!parsePacket(buffer.data(), static_cast<size_t>(got), header)) continue;
        if (header.packetType != kPacketTypeData) continue;

        PlutoSkyStream *stream = nullptr;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = streams_.find(static_cast<int64_t>(header.streamId));
            if (it == streams_.end()) continue;
            stream = it->second;
        }

        WireFormat wire = WireFormat::Cs12;
        if (!wireFormatFromPacketClass(header.packetClass, wire)) continue;
        if (wire != stream->wire) continue;

        std::lock_guard<std::mutex> lock(stream->mutex);
        if (!stream->accepting) continue;
        if (stream->haveCount && countGap(stream->lastCount, header.count) != 0)
        {
            stream->overflowPending = true;
        }
        stream->haveCount = true;
        stream->lastCount = header.count;

        if (header.payloadBytes == 0) continue;
        if (stream->queue.size() >= stream->queueLimit)
        {
            stream->overflowPending = true;
            continue;
        }

        // The payload is padded to a whole word, and in cs12 and cs8 those pad
        // bytes are indistinguishable from a sample, so a full packet is capped
        // at the count the daemon negotiated.
        RxPacket packet;
        packet.ticks = header.ticks;
        packet.count = header.count;
        packet.samples = samplesInPayload(wire, header.payloadBytes);
        if (packet.samples > stream->samplesPerPacket) packet.samples = stream->samplesPerPacket;
        packet.endOfBurst = header.endOfBurst;
        packet.payload.assign(header.payload, header.payload + header.payloadBytes);
        stream->queue.push_back(std::move(packet));
        stream->cv.notify_one();
    }
}

RxSocket &SoapyPlutoSky::rxSocket()
{
    if (!rxSocket_) rxSocket_.reset(new RxSocket());
    return *rxSocket_;
}

PlutoSkyStream *SoapyPlutoSky::lookup(int64_t streamId)
{
    std::lock_guard<std::mutex> lock(streamsMutex_);
    const auto it = streamsById_.find(streamId);
    return it == streamsById_.end() ? nullptr : it->second;
}

void SoapyPlutoSky::pushStatus(PlutoSkyStream *stream, int code, int flags, long long timeNs)
{
    std::lock_guard<std::mutex> lock(stream->statusMutex);
    if (stream->status.size() >= kMaxStatusItems) stream->status.pop_front();
    StatusItem item;
    item.code = code;
    item.flags = flags;
    item.timeNs = timeNs;
    stream->status.push_back(item);
    stream->statusCv.notify_all();
}

static int64_t steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SoapyPlutoSky::onStatus(const plutosky::v1::StatusTick &tick)
{
    const int64_t nowNs = steadyNs();
    updateBoardClockOffset(tick.ticks(), nowNs);
    statusHostNs_.store(nowNs);
    statusTicks_.store(tick.ticks());
    // The host may promote the module's threads after the device opens, so
    // the check repeats rather than running once.
    if (nowNs - libraryThreadCheckNs_ >= kLibraryThreadCheckNs)
    {
        libraryThreadCheckNs_ = nowNs;
        const size_t restored = control_->restoreLibraryThreadPolicy();
        if (restored != 0)
        {
            SoapySDR::logf(SOAPY_SDR_INFO,
                           "plutosky: returned %zu gRPC threads to the standard scheduling policy",
                           restored);
        }
    }
    for (const auto &status : tick.streams()) onStreamStatus(status, tick.ticks());
}

// The board time estimate runs behind by the status message's transit time,
// so the margin reads high by that much.
// Board time from the latest status tick plus the host time since it
// arrived. It runs behind by the status message's transit time.
// Each status tick gives the board time minus the host time it arrived at.
// Ticks held up on the way read low, so the largest recent offset is the
// least delayed. It decays at kBoardClockDriftPpm so the estimate follows the
// drift between the two clocks.
void SoapyPlutoSky::updateBoardClockOffset(uint64_t boardTicks, int64_t hostNs)
{
    const double tickRate = rates_.tickRateHz;
    if (tickRate <= 0.0) return;
    const double sample = static_cast<double>(boardTicks) - hostNs * tickRate / kNsPerSecondF;
    const int64_t lastNs = boardOffsetHostNs_.load();
    double best = sample;
    if (lastNs != 0)
    {
        const double decay = (hostNs - lastNs) * kBoardClockDriftPpm * 1e-6 * tickRate
                           / kNsPerSecondF;
        best = std::max(sample, boardOffsetTicks_.load() - decay);
    }
    boardOffsetTicks_.store(best);
    boardOffsetHostNs_.store(hostNs);
}

bool SoapyPlutoSky::estimateBoardTicks(double &ticks) const
{
    const int64_t hostNs = statusHostNs_.load();
    const double tickRate = rates_.tickRateHz;
    if (hostNs == 0 || boardOffsetHostNs_.load() == 0 || tickRate <= 0.0) return false;
    ticks = steadyNs() * tickRate / kNsPerSecondF + boardOffsetTicks_.load();
    return true;
}

void SoapyPlutoSky::forgetBoardTimeEstimate()
{
    statusHostNs_.store(0);
    boardOffsetHostNs_.store(0);
}

void SoapyPlutoSky::recordSendMargin(PlutoSkyStream *stream)
{
    double boardTicks = 0.0;
    if (!estimateBoardTicks(boardTicks)) return;
    const auto marginUs = static_cast<int64_t>(
        (static_cast<double>(stream->txTicks) - boardTicks) * kUsPerSecondF / rates_.tickRateHz);
    if (marginUs < stream->txMinMarginUs.load()) stream->txMinMarginUs.store(marginUs);
}

void SoapyPlutoSky::onStreamStatus(const plutosky::v1::StreamStatus &status, const uint64_t ticks)
{
    PlutoSkyStream *stream = lookup(static_cast<int64_t>(status.stream_id()));
    if (stream == nullptr) return;

    if (!status.error().empty())
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "plutosky: stream %u stopped: %s", status.stream_id(),
                       status.error().c_str());
        pushStatus(stream, SOAPY_SDR_STREAM_ERROR, 0, 0);
        return;
    }

    const long long timeNs =
        SoapySDR::ticksToTimeNs(static_cast<long long>(ticks), rates_.tickRateHz);
    const uint64_t consumed = status.consumed();
    stream->consumedTicks.store(consumed);

    const uint64_t late = status.late();
    const uint64_t underrun = status.underrun();
    const uint64_t overflow = status.overflow();
    const uint64_t gaps = status.gaps();

    std::vector<StatusItem> items;
    {
        std::lock_guard<std::mutex> lock(stream->statusMutex);

        uint64_t folded = 0;
        const int64_t nowMs = steadyMs();
        const int64_t minMarginUs = stream->txMinMarginUs.load();
        const std::string marginText = minMarginUs == std::numeric_limits<int64_t>::max()
            ? std::string("n/a") : std::to_string(minMarginUs);
        if (stream->direction == SOAPY_SDR_TX &&
            (late > stream->lastLate || underrun > stream->lastUnderrun) &&
            stream->timingLog.due(nowMs, folded))
        {
            const uint64_t firstTick = stream->txFirstTick.load();
            const double elapsedMs = firstTick && ticks >= firstTick
                ? 1000.0 * (ticks - firstTick) / rates_.tickRateHz : -1.0;
            SoapySDR::logf(SOAPY_SDR_WARNING,
                           "plutosky: TX timing status late=%llu underrun=%llu elapsed_ms=%.3f "
                           "min_send_margin_us=%s (%llu reports folded)",
                           static_cast<unsigned long long>(late),
                           static_cast<unsigned long long>(underrun), elapsedMs,
                           marginText.c_str(), static_cast<unsigned long long>(folded));
        }
        if (stream->direction == SOAPY_SDR_TX && stream->running &&
            nowMs - stream->txMarginLogMs >= kTxMarginLogIntervalMs)
        {
            if (stream->txMarginLogMs != 0)
            {
                SoapySDR::logf(SOAPY_SDR_INFO,
                               "plutosky: TX stream %lld min_send_margin_us=%s spin_us=%lld over "
                               "%lld ms",
                               static_cast<long long>(stream->streamId), marginText.c_str(),
                               static_cast<long long>(stream->txSpinTailNs.load() / 1000),
                               static_cast<long long>(nowMs - stream->txMarginLogMs));
            }
            stream->txMarginLogMs = nowMs;
            stream->txMinMarginUs.store(std::numeric_limits<int64_t>::max());
        }

        if (stream->direction == SOAPY_SDR_TX && stream->running &&
            !stream->txStarvedStop.load() &&
            txStarved(nowMs, stream->txLastWriteMs.load(), underrun > stream->lastUnderrun,
                      kTxStarvedStopDelayMs))
        {
            // This runs on the control session thread, which must never wait
            // on a lock held across a control request. The next report retries.
            std::unique_lock<std::mutex> park(stream->parkMutex, std::try_to_lock);
            if (park.owns_lock())
            {
                SoapySDR::logf(SOAPY_SDR_WARNING,
                               "plutosky: TX stream %lld has had no write for %lld ms while "
                               "underrunning, stopping it until the next write",
                               static_cast<long long>(stream->streamId),
                               static_cast<long long>(nowMs - stream->txLastWriteMs.load()));
                stream->txStarvedStop.store(true);
                parkTxStreamLocked(stream);
            }
        }

        // A burst is acknowledged once the board has consumed its last sample.
        while (!stream->burstEnds.empty() && stream->burstEnds.front().consumed <= consumed)
        {
            StatusItem ack;
            ack.flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
            ack.timeNs = SoapySDR::ticksToTimeNs(
                static_cast<long long>(stream->burstEnds.front().endTicks + rates_.txDelayTicks),
                rates_.tickRateHz);
            items.push_back(ack);
            stream->burstEnds.pop_front();
        }

        // The counters are cumulative from the last start_stream, and
        // startDaemonStream resets the baseline to zero, so the first event of
        // a run is compared against zero like any other.
        StatusItem item;
        item.flags = SOAPY_SDR_HAS_TIME;
        item.timeNs = timeNs;
        if (late > stream->lastLate)
        {
            item.code = SOAPY_SDR_TIME_ERROR;
            items.push_back(item);
        }
        if (underrun > stream->lastUnderrun)
        {
            item.code = SOAPY_SDR_UNDERFLOW;
            items.push_back(item);
        }
        if (overflow > stream->lastOverflow || gaps > stream->lastGaps)
        {
            if (stream->lossLog.due(nowMs, folded))
            {
                SoapySDR::logf(SOAPY_SDR_INFO,
                               "plutosky: stream %lld loss status: overflow=%llu gaps=%llu "
                               "(%llu reports folded)",
                               static_cast<long long>(stream->streamId),
                               static_cast<unsigned long long>(overflow),
                               static_cast<unsigned long long>(gaps),
                               static_cast<unsigned long long>(folded));
            }
            item.code = stream->direction == SOAPY_SDR_TX ? SOAPY_SDR_UNDERFLOW
                                                           : SOAPY_SDR_OVERFLOW;
            items.push_back(item);
        }
        stream->lastLate = late;
        stream->lastUnderrun = underrun;
        stream->lastOverflow = overflow;
        stream->lastGaps = gaps;
    }
    for (const auto &item : items) pushStatus(stream, item.code, item.flags, item.timeNs);
}

std::vector<std::string> SoapyPlutoSky::getStreamFormats(const int, const size_t) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CF32);
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CS12);
    formats.push_back(SOAPY_SDR_CS8);
    return formats;
}

std::string SoapyPlutoSky::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    fullScale = kSampleMax;
    return SOAPY_SDR_CS12;
}

SoapySDR::ArgInfoList SoapyPlutoSky::getStreamArgsInfo(const int, const size_t) const
{
    SoapySDR::ArgInfoList args;

    SoapySDR::ArgInfo wire;
    wire.key = "wire";
    wire.name = "Wire format";
    wire.description = "Sample packing on the network";
    wire.type = SoapySDR::ArgInfo::STRING;
    wire.value = wireFormatName(kDefaultWireFormat);
    for (const auto &name : wireFormats_) wire.options.push_back(name);
    args.push_back(wire);

    SoapySDR::ArgInfo mtu;
    mtu.key = "mtu";
    mtu.name = "MTU";
    mtu.description = "Ethernet frame size the packetizer builds for";
    mtu.type = SoapySDR::ArgInfo::INT;
    mtu.value = std::to_string(kDefaultMtu);
    for (const auto value : mtus_) mtu.options.push_back(std::to_string(value));
    args.push_back(mtu);

    SoapySDR::ArgInfo depth;
    depth.key = "rx_queue";
    depth.name = "Receive queue";
    depth.description = "Packets buffered per stream before the oldest data is lost";
    depth.type = SoapySDR::ArgInfo::INT;
    depth.value = std::to_string(kDefaultRxQueuePackets);
    args.push_back(depth);

    SoapySDR::ArgInfo pace;
    pace.key = "duplex_tx_rate_limit_sps";
    pace.name = "Duplex TX send-rate limit";
    pace.description = "Maximum TX packet send rate in samples per second while RX is active. "
                       "Unset selects 10 MS/s, or the sample rate when that is higher";
    pace.type = SoapySDR::ArgInfo::FLOAT;
    if (duplexTxRateLimitSps_ != TxPacer::kRateLimitUnset)
        pace.value = std::to_string(duplexTxRateLimitSps_);
    args.push_back(pace);

    SoapySDR::ArgInfo txOnlyLimit;
    txOnlyLimit.key = "tx_only_rate_limit_sps";
    txOnlyLimit.name = "TX-only send-rate limit";
    txOnlyLimit.description = "Maximum TX packet send rate in samples per second without active RX";
    txOnlyLimit.type = SoapySDR::ArgInfo::FLOAT;
    txOnlyLimit.value = std::to_string(txOnlyRateLimitSps_);
    args.push_back(txOnlyLimit);

    SoapySDR::ArgInfo scale;
    scale.key = kCs16ScaleArg;
    scale.name = "CS16 scale";
    scale.description = "Bits of a CS16 sample the 12-bit value fills, 16 when opened through UHD";
    scale.type = SoapySDR::ArgInfo::STRING;
    scale.value = cs16ScaleName(cs16Format_);
    scale.options.push_back(cs16ScaleName(HostFormat::Cs16));
    scale.options.push_back(cs16ScaleName(HostFormat::Cs16Msb));
    args.push_back(scale);

    return args;
}

SoapySDR::Stream *SoapyPlutoSky::setupStream(const int direction, const std::string &format,
                                             const std::vector<size_t> &channels,
                                             const SoapySDR::Kwargs &args)
{
    std::unique_ptr<PlutoSkyStream> stream(new PlutoSkyStream());
    stream->direction = direction;
    stream->channels = channels.empty() ? std::vector<size_t>(1, 0) : channels;

    if (stream->channels.size() > numChannels_)
    {
        throw std::runtime_error("plutosky: too many channels for this device");
    }

    std::string wireName = wireFormatName(kDefaultWireFormat);
    const auto uhdWireArg = args.find(kUhdWireArg);
    if (uhdWireArg != args.end() && !wireFormatFromUhd(uhdWireArg->second, wireName))
    {
        throw std::runtime_error("plutosky: no wire format for " + uhdWireArg->second);
    }
    const auto wireArg = args.find("wire");
    if (wireArg != args.end()) wireName = wireArg->second;
    if (!wireFormatFromName(wireName, stream->wire))
    {
        throw std::runtime_error("plutosky: unknown wire format " + wireName);
    }

    stream->mtu = kDefaultMtu;
    const auto mtuArg = args.find("mtu");
    if (mtuArg != args.end()) stream->mtu = static_cast<size_t>(std::atoi(mtuArg->second.c_str()));
    bool mtuKnown = false;
    for (const auto value : mtus_) mtuKnown = mtuKnown || value == stream->mtu;
    if (!mtuKnown) throw std::runtime_error("plutosky: unsupported MTU " + std::to_string(stream->mtu));

    const auto queueArg = args.find("rx_queue");
    if (queueArg != args.end())
    {
        const int depth = std::atoi(queueArg->second.c_str());
        if (depth > 0) stream->queueLimit = static_cast<size_t>(depth);
    }

    stream->duplexTxRateLimitSps = duplexTxRateLimitSps_;
    stream->txOnlyRateLimitSps = txOnlyRateLimitSps_;
    if (args.count("tx_send_rate_multiplier") || args.count("tx_pace_headroom"))
        throw std::runtime_error("plutosky: TX send-rate multipliers are removed; use duplex_tx_rate_limit_sps");
    const auto rateLimitArg = args.find("duplex_tx_rate_limit_sps");
    if (rateLimitArg != args.end())
        stream->duplexTxRateLimitSps = TxPacer::parseRateLimit(rateLimitArg->second,
                                                               "duplex_tx_rate_limit_sps");
    const auto txOnlyLimitArg = args.find("tx_only_rate_limit_sps");
    if (txOnlyLimitArg != args.end())
        stream->txOnlyRateLimitSps = TxPacer::parseRateLimit(txOnlyLimitArg->second,
                                                              "tx_only_rate_limit_sps");

    if (!hostFormatFromName(format, stream->host))
    {
        throw std::runtime_error("plutosky: unsupported stream format " + format);
    }
    if (stream->host == HostFormat::Cs16)
    {
        stream->host = cs16Format_;
        const auto scaleArg = args.find(kCs16ScaleArg);
        if (scaleArg != args.end() && !cs16FormatFromScale(scaleArg->second, stream->host))
        {
            throw std::runtime_error("plutosky: cs16_scale must be 12 or 16");
        }
    }
    if (!hostFormatAllowed(stream->wire, stream->host))
    {
        throw std::runtime_error("plutosky: format " + format + " is not available on wire format "
                                 + wireName);
    }

    std::string hostIp = control_->localAddress();
    int hostPort = 0;
    if (direction == SOAPY_SDR_RX)
    {
        hostPort = rxSocket().port();
    }
    else
    {
        stream->txSocket = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (stream->txSocket < 0) throw std::runtime_error("plutosky: cannot create a data socket");
        sockaddr_in local;
        std::memset(&local, 0, sizeof(local));
        local.sin_family = AF_INET;
        local.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(stream->txSocket, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0)
        {
            ::close(stream->txSocket);
            throw std::runtime_error("plutosky: cannot bind the data socket");
        }
        socklen_t length = sizeof(local);
        ::getsockname(stream->txSocket, reinterpret_cast<sockaddr *>(&local), &length);
        hostPort = ntohs(local.sin_port);
    }

    OpenStreamParams params;
    params.direction = protoDirection(direction);
    params.channels = stream->channels;
    params.format = wireName;
    params.mtu = stream->mtu;
    params.hostIp = hostIp;
    params.hostPort = hostPort;

    const OpenedStream opened = control_->openStream(params);
    stream->streamId = opened.streamId;
    stream->dataPort = opened.dataPort;
    stream->samplesPerPacket = opened.samplesPerPacket;
    stream->window = opened.window;
    if (stream->samplesPerPacket == 0)
    {
        stream->samplesPerPacket = samplesPerPacket(stream->wire, stream->mtu);
    }
    stream->elemsPerPacket = stream->samplesPerPacket / stream->channels.size();
    if (stream->elemsPerPacket == 0) throw std::runtime_error("plutosky: empty packet size");

    stream->staging.resize(stream->samplesPerPacket + kBfp8BlockSamples);
    stream->packet.resize(packetBytes(payloadBytesFor(stream->wire, stream->samplesPerPacket), true));

    {
        std::lock_guard<std::mutex> lock(streamsMutex_);
        streamsById_[stream->streamId] = stream.get();
    }

    // The daemon learns the host MAC from this datagram before it builds the header
    // template, so it must go out before the first start_stream.
    if (direction == SOAPY_SDR_RX)
    {
        rxSocket().registerStream(stream->streamId, stream.get());
        rxSocket().sendHello(control_->addr(), stream->dataPort, stream->streamId,
                             packetClassOf(stream->wire));
    }
    else
    {
        uint8_t hello[kVitaHeaderBytes];
        writePacketHeader(hello, static_cast<uint32_t>(stream->streamId),
                          packetClassOf(stream->wire), 0, 0, 0, false);
        sockaddr_in dest;
        std::memset(&dest, 0, sizeof(dest));
        dest.sin_family = AF_INET;
        dest.sin_port = htons(static_cast<uint16_t>(stream->dataPort));
        ::inet_pton(AF_INET, control_->addr().c_str(), &dest.sin_addr);
        ::connect(stream->txSocket, reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
        ::send(stream->txSocket, hello, sizeof(hello), 0);
    }

    return reinterpret_cast<SoapySDR::Stream *>(stream.release());
}

void SoapyPlutoSky::closeStream(SoapySDR::Stream *handle)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    if (stream == nullptr) return;
    if (stream->active) deactivateStream(handle, 0, 0);
    {
        std::lock_guard<std::mutex> park(stream->parkMutex);
        stream->closing = true;
        stream->parked = false;
    }
    stream->parkCv.notify_all();
    if (stream->parkThread.joinable()) stream->parkThread.join();
    stopDaemonStream(stream);

    if (stream->direction == SOAPY_SDR_RX && rxSocket_) rxSocket_->unregisterStream(stream->streamId);
    {
        std::lock_guard<std::mutex> lock(streamsMutex_);
        streamsById_.erase(stream->streamId);
    }

    try
    {
        control_->closeStream(stream->streamId);
    }
    catch (const std::exception &error)
    {
        SoapySDR::logf(SOAPY_SDR_WARNING, "plutosky: close_stream failed: %s", error.what());
    }

    if (stream->txSocket >= 0) ::close(stream->txSocket);
    delete stream;
}

size_t SoapyPlutoSky::getStreamMTU(SoapySDR::Stream *handle) const
{
    const PlutoSkyStream *stream = reinterpret_cast<const PlutoSkyStream *>(handle);
    return stream->elemsPerPacket;
}

void SoapyPlutoSky::startDaemonStream(PlutoSkyStream *stream, const bool timed,
                                      const uint64_t atTicks)
{
    control_->startStream(stream->streamId, timed, atTicks);
    stream->running = true;
    std::lock_guard<std::mutex> lock(stream->statusMutex);
    stream->burstEnds.clear();
    stream->lastLate = 0;
    stream->lastUnderrun = 0;
    stream->lastOverflow = 0;
    stream->lastGaps = 0;
}

void SoapyPlutoSky::stopDaemonStream(PlutoSkyStream *stream)
{
    if (!stream->running) return;
    std::exception_ptr muteError;
    if (stream->direction == SOAPY_SDR_TX)
    {
        for (const auto channel : stream->channels)
        {
            try { setTxEnabled(channel, false); }
            catch (...) { muteError = std::current_exception(); }
        }
    }
    control_->stopStream(stream->streamId);
    stream->running = false;
    if (muteError) std::rethrow_exception(muteError);
}

void SoapyPlutoSky::parkTxStream(PlutoSkyStream *stream)
{
    std::lock_guard<std::mutex> park(stream->parkMutex);
    parkTxStreamLocked(stream);
}

void SoapyPlutoSky::parkTxStreamLocked(PlutoSkyStream *stream)
{
    stream->active = false;
    stream->parked = true;
    ++stream->parkGeneration;
    if (!stream->parkThread.joinable())
        stream->parkThread = std::thread(&SoapyPlutoSky::runTxParking, this, stream);
    stream->parkCv.notify_all();
}

void SoapyPlutoSky::runTxParking(PlutoSkyStream *stream)
{
    std::unique_lock<std::mutex> park(stream->parkMutex);
    while (!stream->closing)
    {
        stream->parkCv.wait(park, [stream] { return stream->parked || stream->closing; });
        if (stream->closing) break;
        const uint64_t generation = stream->parkGeneration;
        const bool resumed = stream->parkCv.wait_for(park, kTxParkedStopDelay, [stream, generation] {
            return !stream->parked || stream->closing || stream->parkGeneration != generation;
        });
        if (resumed) continue;
        try
        {
            stopParkedTxStream(stream);
        }
        catch (const std::exception &error)
        {
            SoapySDR::logf(SOAPY_SDR_WARNING, "plutosky: stopping parked TX stream failed: %s",
                           error.what());
        }
    }
}

// Called with parkMutex held. Lets the last burst play out, then mutes TX and
// stops the daemon stream.
void SoapyPlutoSky::stopParkedTxStream(PlutoSkyStream *stream)
{
    if (!stream->parked) return;
    {
        std::unique_lock<std::mutex> lock(stream->statusMutex);
        stream->statusCv.wait_for(lock, kTxEndBurstDrainTimeout,
                                  [stream] { return stream->burstEnds.empty(); });
    }
    stream->parked = false;
    stopDaemonStream(stream);
    std::lock_guard<std::mutex> lock(stream->mutex);
    stream->accepting = false;
    stream->queue.clear();
    stream->txPending.clear();
}

void SoapyPlutoSky::stopParkedTxStreams()
{
    std::vector<PlutoSkyStream *> streams;
    {
        std::lock_guard<std::mutex> lock(streamsMutex_);
        for (const auto &entry : streamsById_) streams.push_back(entry.second);
    }
    for (PlutoSkyStream *stream : streams)
    {
        if (stream->direction != SOAPY_SDR_TX) continue;
        std::lock_guard<std::mutex> park(stream->parkMutex);
        stopParkedTxStream(stream);
    }
}

int SoapyPlutoSky::activateStream(SoapySDR::Stream *handle, const int flags,
                                  const long long timeNs, const size_t numElems)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    refreshSampleRate();

    const bool timed = (flags & SOAPY_SDR_HAS_TIME) != 0;
    bool startTimed = false;
    uint64_t ticks = 0;
    if (timed)
    {
        ticks = stream->direction == SOAPY_SDR_RX
            ? rxBoardTicks(SoapySDR::timeNsToTicks(timeNs, rates_.tickRateHz), rxOffsetTicks())
            : txBoardTicks(timeNs);
        // TX packet timestamps schedule RF playback. Starting its daemon pump
        // immediately lets the bounded slot window fill before that time.
        startTimed = stream->direction == SOAPY_SDR_RX;
    }

    if (stream->direction == SOAPY_SDR_RX)
    {
        // A new stream command replaces whatever the last one started.
        stopDaemonStream(stream);
        {
            std::lock_guard<std::mutex> lock(stream->mutex);
            stream->queue.clear();
            stream->haveCount = false;
            stream->overflowPending = false;
            stream->accepting = true;
            stream->burstLimited = numElems > 0;
            stream->burstRemaining = numElems;
        }
        startDaemonStream(stream, startTimed, ticks);
        if (!stream->active) activeRxStreams_.fetch_add(1, std::memory_order_release);
    }
    else
    {
        std::lock_guard<std::mutex> park(stream->parkMutex);
        if (stream->parked)
        {
            stream->parked = false;
            stream->parkCv.notify_all();
        }
        stream->txTicks = ticks;
        stream->txTicksValid = timed;
        stream->txStarvedStop.store(false);
        stream->txLastWriteMs.store(0);
        if (!stream->running)
        {
            stream->txPacer.reset();
            stream->txPackets = 0;
            stream->txDuplexPackets = 0;
            stream->txWindowWaits = 0;
            stream->txWindowWaitTotalUs = 0;
            stream->txWindowWaitMaxUs = 0;
            stream->txPaceWaits = 0;
            stream->txMaxSendGapUs = 0;
            stream->txMaxSteadySendGapUs = 0;
            stream->txMaxSteadyWindowWaitUs = 0;
            stream->txFirstTick.store(0);
            stream->txPastStartup = false;
            stream->txLastSent = TxPacer::Time{};
            stream->consumedTicks.store(0);
            startDaemonStream(stream, startTimed, ticks);
            try
            {
                for (const auto channel : stream->channels) setTxEnabled(channel, true);
            }
            catch (...)
            {
                stopDaemonStream(stream);
                throw;
            }
        }
    }

    stream->active = true;
    return 0;
}

int SoapyPlutoSky::deactivateStream(SoapySDR::Stream *handle, const int, const long long)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    stream->txStarvedStop.store(false);
    if (stream->direction == SOAPY_SDR_TX && stream->running && !stream->txBurstOpen)
    {
        parkTxStream(stream);
        return 0;
    }
    if (stream->direction == SOAPY_SDR_TX)
    {
        std::unique_lock<std::mutex> lock(stream->statusMutex);
        if (!stream->burstEnds.empty())
        {
            const uint64_t endTick = stream->burstEnds.front().endTicks;
            stream->statusCv.wait_for(lock, kTxEndBurstDrainTimeout, [stream, endTick] {
                return stream->burstEnds.empty() || stream->burstEnds.front().endTicks != endTick;
            });
        }
    }
    const bool rxWasActive = stream->active && stream->direction == SOAPY_SDR_RX;
    stopDaemonStream(stream);
    stream->active = false;
    if (rxWasActive) activeRxStreams_.fetch_sub(1, std::memory_order_release);
    std::lock_guard<std::mutex> lock(stream->mutex);
    stream->accepting = false;
    stream->queue.clear();
    stream->haveCount = false;
    stream->txPending.clear();
    stream->txBurstOpen = false;
    stream->txCarryElems = 0;
    return 0;
}

int SoapyPlutoSky::readStream(SoapySDR::Stream *handle, void *const *buffs, const size_t numElems,
                              int &flags, long long &timeNs, const long timeoutUs)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    if (stream->direction != SOAPY_SDR_RX) return SOAPY_SDR_NOT_SUPPORTED;

    std::unique_lock<std::mutex> lock(stream->mutex);
    if (stream->overflowPending)
    {
        stream->overflowPending = false;
        return SOAPY_SDR_OVERFLOW;
    }

    if (stream->queue.empty())
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
        while (stream->queue.empty() && !stream->overflowPending)
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline) return SOAPY_SDR_TIMEOUT;
            const auto waitUntil = std::min(deadline, now + std::chrono::seconds(1));
            stream->cv.wait_until(lock, waitUntil);
            lock.unlock();
            lock.lock();
        }
        if (stream->overflowPending)
        {
            stream->overflowPending = false;
            return SOAPY_SDR_OVERFLOW;
        }
    }

    RxPacket &packet = stream->queue.front();
    const size_t channels = stream->channels.size();
    const size_t totalElems = packet.samples / channels;
    const size_t remaining = totalElems - packet.consumedElems;
    size_t take = remaining < numElems ? remaining : numElems;
    if (stream->burstLimited && take > stream->burstRemaining) take = stream->burstRemaining;

    payloadToHost(stream->wire, stream->host, packet.payload.data(),
                  packet.consumedElems * channels, take, channels, buffs, 0,
                  stream->staging.data());

    flags = SOAPY_SDR_HAS_TIME;
    timeNs = SoapySDR::ticksToTimeNs(
        rxClientTicks(packet.ticks + packet.consumedElems * rates_.rxTicksPerSample,
                      rxOffsetTicks()),
        rates_.tickRateHz);
    packet.consumedElems += take;

    bool burstDone = false;
    if (stream->burstLimited)
    {
        stream->burstRemaining -= take;
        burstDone = stream->burstRemaining == 0;
    }

    if (burstDone)
    {
        flags |= SOAPY_SDR_END_BURST;
        stream->burstLimited = false;
        stream->accepting = false;
        stream->queue.clear();
    }
    else if (packet.consumedElems < totalElems)
    {
        flags |= SOAPY_SDR_MORE_FRAGMENTS;
    }
    else
    {
        if (packet.endOfBurst) flags |= SOAPY_SDR_END_BURST;
        stream->queue.pop_front();
    }
    lock.unlock();

    if (burstDone)
    {
        try
        {
            stopDaemonStream(stream);
        }
        catch (const std::exception &error)
        {
            SoapySDR::logf(SOAPY_SDR_WARNING, "plutosky: stop_stream failed: %s", error.what());
        }
    }

    return static_cast<int>(take);
}

// macOS coalesces timers for ordinary sleeps. mach_wait_until waits on an
// absolute deadline without that slack.
static void sleepUntil(TxPacer::Time deadline)
{
#ifdef __APPLE__
    static const mach_timebase_info_data_t timebase = [] {
        mach_timebase_info_data_t info;
        mach_timebase_info(&info);
        return info;
    }();
    const auto now = TxPacer::Clock::now();
    if (deadline <= now) return;
    const auto ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count());
    mach_wait_until(mach_absolute_time() + ns * timebase.denom / timebase.numer);
#else
    std::this_thread::sleep_until(deadline);
#endif
}

int SoapyPlutoSky::writeStream(SoapySDR::Stream *handle, const void *const *buffs,
                               const size_t numElems, int &flags, const long long timeNs,
                               const long timeoutUs)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    if (stream->direction != SOAPY_SDR_TX) return SOAPY_SDR_NOT_SUPPORTED;
#ifdef __linux__
    // Linux pads every sleep with 50 us of timer slack unless the thread asks
    // for less, and the spin would have to cover it.
    static thread_local bool timerSlackSet = false;
    if (!timerSlackSet)
    {
        ::prctl(PR_SET_TIMERSLACK, kTxTimerSlackNs, 0, 0, 0);
        timerSlackSet = true;
    }
#endif
    if (stream->txStarvedStop.load()) activateStream(handle, 0, 0, 0);
    stream->txLastWriteMs.store(steadyMs());

    const size_t channels = stream->channels.size();
    const double txRate = rates_.txHz;
    size_t carried = stream->txCarryElems;
    if ((flags & SOAPY_SDR_HAS_TIME) != 0)
    {
        const uint64_t ticks = txBoardTicks(timeNs);
        const uint64_t carriedTicks = carried * rates_.txTicksPerSample;
        // A held element only belongs in front of this write when the write
        // starts on the sample after it. Across a time jump it is dropped.
        if (carried != 0 && ticks != stream->txTicks + carriedTicks) carried = 0;
        stream->txTicks = ticks - (carried != 0 ? carriedTicks : 0);
        stream->txTicksValid = true;
    }
    stream->txCarryElems = 0;

    const void *const *src = buffs;
    const size_t total = numElems + carried;
    if (carried != 0)
    {
        const size_t elemBytes = hostElementBytes(stream->host);
        stream->txJoined.resize(channels);
        stream->txJoinedPtrs.resize(channels);
        for (size_t ch = 0; ch < channels; ch++)
        {
            std::vector<uint8_t> &joined = stream->txJoined[ch];
            joined.resize(total * elemBytes);
            std::memcpy(joined.data(), stream->txCarry[ch].data(), carried * elemBytes);
            std::memcpy(joined.data() + carried * elemBytes, buffs[ch], numElems * elemBytes);
            stream->txJoinedPtrs[ch] = joined.data();
        }
        src = stream->txJoinedPtrs.data();
    }
    // Elements of this call written so far, given the joined count sent.
    const auto writeResult = [stream, carried](size_t done, int error) {
        if (done == 0)
        {
            stream->txCarryElems = carried;
            return error;
        }
        return static_cast<int>(done - carried);
    };
    if (!stream->txTicksValid)
    {
        const auto now = static_cast<long long>(control_->hardwareTime());
        stream->txTicks = static_cast<uint64_t>(
            now + SoapySDR::timeNsToTicks(kTxUntimedLeadNs, rates_.tickRateHz));
        stream->txTicksValid = true;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
    size_t sent = 0;
    const auto writeStarted = TxPacer::Clock::now();
    auto previousSend = writeStarted;
    long long maxSendGapUs = 0;
    while (sent < total)
    {
        const size_t remaining = total - sent;
        size_t take = exactPacketElems(stream->wire, channels, remaining, stream->elemsPerPacket);
        size_t pad = 0;
        if (take == 0)
        {
            // Only a one-channel cs8 element of an odd-length write gets here.
            take = remaining;
            if ((flags & SOAPY_SDR_END_BURST) == 0)
            {
                const size_t elemBytes = hostElementBytes(stream->host);
                stream->txCarry.resize(channels);
                for (size_t ch = 0; ch < channels; ch++)
                {
                    const uint8_t *from = static_cast<const uint8_t *>(src[ch]) + sent * elemBytes;
                    stream->txCarry[ch].assign(from, from + take * elemBytes);
                }
                stream->txCarryElems = take;
                sent += take;
                break;
            }
            while (!samplesExact(stream->wire, take * channels + pad)) pad++;
        }
        const bool last = sent + take == total;
        const bool endOfBurst = last && (flags & SOAPY_SDR_END_BURST) != 0;

        if (stream->wire == WireFormat::Bfp8)
        {
            const size_t partial = (take * channels) % kBfp8BlockSamples;
            if (partial != 0) pad = kBfp8BlockSamples - partial;
        }

        // The daemon's consumed counter is the only backpressure the module has.
        bool waitedForWindow = false;
        const auto windowWaitStarted = TxPacer::Clock::now();
        while (true)
        {
            // A packet whose end the board's clock has passed was played or
            // dropped as late. Only played samples advance consumed, and status
            // can arrive late over TCP, so the estimated board time frees the
            // window too.
            double boardTicks = 0.0;
            uint64_t consumed = stream->consumedTicks.load();
            if (estimateBoardTicks(boardTicks))
                consumed = std::max(consumed, static_cast<uint64_t>(boardTicks));
            while (!stream->txPending.empty() && stream->txPending.front() <= consumed)
            {
                stream->txPending.pop_front();
            }
            if (stream->txPending.size() < stream->window) break;
            if (!waitedForWindow)
            {
                stream->txWindowWaits++;
                waitedForWindow = true;
            }
            if (std::chrono::steady_clock::now() >= deadline)
            {
                return writeResult(sent, SOAPY_SDR_TIMEOUT);
            }
            std::this_thread::sleep_for(kTxWindowPollInterval);
        }
        if (waitedForWindow)
        {
            const auto waitedUs = static_cast<long long>(std::chrono::duration_cast<
                std::chrono::microseconds>(TxPacer::Clock::now() - windowWaitStarted).count());
            stream->txWindowWaitTotalUs += waitedUs;
            stream->txWindowWaitMaxUs = std::max(stream->txWindowWaitMaxUs, waitedUs);
            if (stream->txPastStartup)
                stream->txMaxSteadyWindowWaitUs = std::max(stream->txMaxSteadyWindowWaitUs, waitedUs);
            if (waitedUs >= kTxWindowWaitLogUs)
                SoapySDR::logf(SOAPY_SDR_INFO,
                               "plutosky: TX window blocked %lld us consumed=%llu pending=%zu",
                               waitedUs, static_cast<unsigned long long>(stream->consumedTicks.load()),
                               stream->txPending.size());
        }

        const size_t payloadBytes = payloadBytesFor(stream->wire, take * channels + pad);
        uint8_t *datagram = stream->packet.data();
        writePacketHeader(datagram, static_cast<uint32_t>(stream->streamId),
                          packetClassOf(stream->wire), stream->txCount, stream->txTicks,
                          payloadBytes, endOfBurst);
        hostToPayload(stream->wire, stream->host, src, sent, take, channels,
                      datagram + kVitaHeaderBytes, pad, stream->staging.data());
        const size_t total = packetBytes(payloadBytes, endOfBurst);
        if (endOfBurst) writeTrailer(datagram + total - kWordBytes, true);

        const bool duplex = activeRxStreams_.load(std::memory_order_acquire) != 0;
        const double rateLimitSps =
            duplex ? TxPacer::duplexRateLimit(stream->duplexTxRateLimitSps, txRate)
                   : std::max(stream->txOnlyRateLimitSps, txRate);
        const auto pacingNow = TxPacer::Clock::now();
        const auto sendAt = stream->txPacer.readyAt(pacingNow);
        if (sendAt > pacingNow)
        {
            stream->txPaceWaits++;
            if (sendAt >= deadline)
                return writeResult(sent, SOAPY_SDR_TIMEOUT);
            auto tail = std::chrono::nanoseconds(stream->txSpinTailNs.load());
            const auto sleepEnd = sendAt - tail;
            if (sleepEnd > TxPacer::Clock::now())
            {
                sleepUntil(sleepEnd);
                const auto wanted = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    TxPacer::Clock::now() - sleepEnd) + kTxSpinTailMargin;
                // A late wake sends its packet late, so a longer overshoot
                // takes effect at once and the tail eases down afterwards.
                if (wanted > tail) tail = wanted;
                else tail += (wanted - tail) / kTxSpinTailGain;
            }
            else
            {
                tail -= tail / kTxSpinTailDecay;
            }
            tail = std::max<std::chrono::nanoseconds>(
                kTxSpinTailMin, std::min<std::chrono::nanoseconds>(tail, kTxSpinTailMax));
            stream->txSpinTailNs.store(tail.count());
            while (TxPacer::Clock::now() < sendAt)
                std::atomic_signal_fence(std::memory_order_seq_cst);
        }

        recordSendMargin(stream);
        const auto sendStarted = TxPacer::Clock::now();
        // A full interface send queue is transient. macOS reports it as
        // ENOBUFS on a UDP socket instead of blocking.
        const auto sendDeadline = sendStarted + kTxSendRetryLimit;
        while (::send(stream->txSocket, datagram, total, 0) < 0)
        {
            const int error = errno;
            const bool transient = error == ENOBUFS || error == EAGAIN || error == EWOULDBLOCK
                                || error == EINTR;
            if (!transient || TxPacer::Clock::now() >= sendDeadline)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "plutosky: TX stream %lld send failed: %s",
                               static_cast<long long>(stream->streamId), std::strerror(error));
                return writeResult(sent, SOAPY_SDR_STREAM_ERROR);
            }
            if (!stream->txSendQueueWarned)
            {
                stream->txSendQueueWarned = true;
                SoapySDR::logf(SOAPY_SDR_WARNING,
                               "plutosky: TX stream %lld send queue full (%s), retrying",
                               static_cast<long long>(stream->streamId), std::strerror(error));
            }
            std::this_thread::sleep_for(kTxSendRetryWait);
        }

        const auto sentAt = TxPacer::Clock::now();
        maxSendGapUs = std::max(maxSendGapUs, static_cast<long long>(
            std::chrono::duration_cast<std::chrono::microseconds>(sentAt - previousSend).count()));
        previousSend = sentAt;
        if (stream->txPackets != 0)
        {
            const auto gapUs = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::microseconds>(sentAt - stream->txLastSent)
                    .count());
            stream->txMaxSendGapUs = std::max(stream->txMaxSendGapUs, gapUs);
            if (stream->txPastStartup)
                stream->txMaxSteadySendGapUs = std::max(stream->txMaxSteadySendGapUs, gapUs);
        }
        else stream->txFirstTick.store(stream->txTicks);
        stream->txLastSent = sentAt;
        stream->txPackets++;
        if (duplex) stream->txDuplexPackets++;

        stream->txCount = (stream->txCount + 1) % kPacketCountModulus;
        const uint64_t packetElems = (take * channels + pad) / channels;
        stream->txPacer.sent(sendStarted, packetElems, rateLimitSps);
        // The daemon reports a played packet as its timestamp plus its sample
        // count, which is its end tick only when a sample spans one tick.
        const uint64_t consumedAfter = stream->txTicks + packetElems;
        stream->txTicks += packetElems * rates_.txTicksPerSample;
        if (stream->txTicks - stream->txFirstTick.load() >=
            static_cast<uint64_t>(rates_.tickRateHz * kTxStartupDiagnosticSeconds))
            stream->txPastStartup = true;
        stream->txPending.push_back(consumedAfter);
        stream->txBurstOpen = !endOfBurst;
        if (endOfBurst)
        {
            std::lock_guard<std::mutex> lock(stream->statusMutex);
            stream->burstEnds.push_back(TxBurstEnd{consumedAfter, stream->txTicks});
        }
        sent += take;
    }

    // An untimed write after an end of burst starts a new burst, not one
    // butted against the last.
    if (!stream->txBurstOpen) stream->txTicksValid = false;

    if ((flags & SOAPY_SDR_END_BURST) != 0)
        SoapySDR::logf(SOAPY_SDR_INFO,
                      "plutosky: TX burst samples=%zu write_us=%lld max_send_gap_us=%lld "
                      "packets=%zu window_waits=%zu window_wait_total_us=%lld "
                      "window_wait_max_us=%lld steady_window_wait_max_us=%lld "
                      "pace_waits=%zu max_packet_gap_us=%lld steady_packet_gap_max_us=%lld "
                      "duplex_packets=%zu duplex_rate_limit_sps=%.0f tx_only_rate_limit_sps=%.0f",
                      sent, static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(
                          TxPacer::Clock::now() - writeStarted).count()), maxSendGapUs,
                      stream->txPackets, stream->txWindowWaits,
                      stream->txWindowWaitTotalUs, stream->txWindowWaitMaxUs,
                      stream->txMaxSteadyWindowWaitUs, stream->txPaceWaits,
                      stream->txMaxSendGapUs, stream->txMaxSteadySendGapUs,
                      stream->txDuplexPackets,
                      TxPacer::duplexRateLimit(stream->duplexTxRateLimitSps, txRate),
                      std::max(stream->txOnlyRateLimitSps, txRate));

    return writeResult(sent, 0);
}

int SoapyPlutoSky::readStreamStatus(SoapySDR::Stream *handle, size_t &chanMask, int &flags,
                                    long long &timeNs, const long timeoutUs)
{
    PlutoSkyStream *stream = reinterpret_cast<PlutoSkyStream *>(handle);
    std::unique_lock<std::mutex> lock(stream->statusMutex);
    if (stream->status.empty())
    {
        if (timeoutUs <= 0) return SOAPY_SDR_TIMEOUT;
        const bool arrived = stream->statusCv.wait_for(lock, std::chrono::microseconds(timeoutUs),
                                                       [stream] { return !stream->status.empty(); });
        if (!arrived) return SOAPY_SDR_TIMEOUT;
    }

    const StatusItem item = stream->status.front();
    stream->status.pop_front();

    chanMask = 0;
    for (size_t i = 0; i < stream->channels.size(); i++) chanMask |= size_t(1) << i;
    flags = item.flags;
    timeNs = item.timeNs;
    return item.code;
}

}  // namespace PlutoSky
