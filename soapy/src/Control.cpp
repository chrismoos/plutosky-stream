#include "Control.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef __APPLE__
#include <mach/mach.h>
#include <pthread.h>
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace PlutoSky {

static const long kUsPerSecond = 1000000;
static const char *const kSessionMetadataKey = "plutosky-session";
static const char *const kClientName = "soapy";
static const size_t kDiscoveryReplyBytes = 2048;

static void setTimeout(int sock, int option, long timeoutUs)
{
    timeval tv;
    tv.tv_sec = timeoutUs / kUsPerSecond;
    tv.tv_usec = timeoutUs % kUsPerSecond;
    ::setsockopt(sock, SOL_SOCKET, option, &tv, sizeof(tv));
}

std::vector<DiscoveryResult> discover(long timeoutUs, int port)
{
    std::vector<DiscoveryResult> results;
    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return results;

    int yes = 1;
    ::setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    setTimeout(sock, SO_RCVTIMEO, timeoutUs);

    std::vector<std::string> targets;
    targets.push_back("255.255.255.255");
    targets.push_back("127.0.0.1");

    ifaddrs *addrs = nullptr;
    if (::getifaddrs(&addrs) == 0)
    {
        for (ifaddrs *it = addrs; it != nullptr; it = it->ifa_next)
        {
            if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) continue;
            if ((it->ifa_flags & IFF_BROADCAST) == 0 || it->ifa_broadaddr == nullptr) continue;
            char text[INET_ADDRSTRLEN] = {0};
            const sockaddr_in *bcast = reinterpret_cast<sockaddr_in *>(it->ifa_broadaddr);
            if (::inet_ntop(AF_INET, &bcast->sin_addr, text, sizeof(text)) != nullptr)
            {
                targets.push_back(text);
            }
        }
        ::freeifaddrs(addrs);
    }

    for (const auto &target : targets)
    {
        sockaddr_in dest;
        std::memset(&dest, 0, sizeof(dest));
        dest.sin_family = AF_INET;
        dest.sin_port = htons(static_cast<uint16_t>(port));
        if (::inet_pton(AF_INET, target.c_str(), &dest.sin_addr) != 1) continue;
        ::sendto(sock, kDiscoveryProbe, std::strlen(kDiscoveryProbe), 0,
                 reinterpret_cast<sockaddr *>(&dest), sizeof(dest));
    }

    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::microseconds(timeoutUs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        char buffer[kDiscoveryReplyBytes];
        sockaddr_in from;
        socklen_t fromLen = sizeof(from);
        const ssize_t got = ::recvfrom(sock, buffer, sizeof(buffer), 0,
                                       reinterpret_cast<sockaddr *>(&from), &fromLen);
        if (got <= 0) break;
        DiscoveryResult result;
        // A stray datagram on the discovery port is not an error.
        if (!result.reply.ParseFromArray(buffer, static_cast<int>(got))) continue;
        char text[INET_ADDRSTRLEN] = {0};
        ::inet_ntop(AF_INET, &from.sin_addr, text, sizeof(text));
        result.addr = text;
        // One board answers the probe once per interface it can be reached
        // on, so the serial is what identifies it, not the source address.
        const std::string &serial = result.reply.serial();
        bool duplicate = false;
        for (const auto &seen : results)
        {
            if (seen.addr == result.addr) duplicate = true;
            if (!serial.empty() && seen.reply.serial() == serial) duplicate = true;
        }
        if (!duplicate) results.push_back(result);
    }

    ::close(sock);
    return results;
}


// The process's threads as Mach port names, each holding a send right the
// caller must release.
static std::vector<unsigned> taskThreads()
{
    std::vector<unsigned> threads;
#ifdef __APPLE__
    thread_act_array_t list = nullptr;
    mach_msg_type_number_t count = 0;
    if (task_threads(mach_task_self(), &list, &count) != KERN_SUCCESS) return threads;
    threads.assign(list, list + count);
    vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(list),
                  count * sizeof(thread_act_t));
#endif
    return threads;
}

static void releaseThreads(const std::vector<unsigned> &threads)
{
#ifdef __APPLE__
    for (const auto thread : threads) mach_port_deallocate(mach_task_self(), thread);
#else
    (void)threads;
#endif
}

static unsigned machThreadOf(std::thread &thread)
{
#ifdef __APPLE__
    return pthread_mach_thread_np(thread.native_handle());
#else
    (void)thread;
    return 0;
#endif
}

static std::string formatName(int format)
{
    switch (format)
    {
    case plutosky::v1::SAMPLE_FORMAT_CS12: return "cs12";
    case plutosky::v1::SAMPLE_FORMAT_CS8: return "cs8";
    case plutosky::v1::SAMPLE_FORMAT_BFP8: return "bfp8";
    default: return std::string();
    }
}

static plutosky::v1::SampleFormat formatValue(const std::string &name)
{
    if (name == "cs12") return plutosky::v1::SAMPLE_FORMAT_CS12;
    if (name == "cs8") return plutosky::v1::SAMPLE_FORMAT_CS8;
    if (name == "bfp8") return plutosky::v1::SAMPLE_FORMAT_BFP8;
    throw std::runtime_error("plutosky: unknown wire format " + name);
}

static std::string gainModeName(int mode)
{
    switch (mode)
    {
    case plutosky::v1::GAIN_MODE_MANUAL: return "manual";
    case plutosky::v1::GAIN_MODE_SLOW_ATTACK: return "slow_attack";
    case plutosky::v1::GAIN_MODE_FAST_ATTACK: return "fast_attack";
    default: return std::string();
    }
}

static plutosky::v1::GainMode gainModeValue(const std::string &name)
{
    if (name == "manual") return plutosky::v1::GAIN_MODE_MANUAL;
    if (name == "slow_attack") return plutosky::v1::GAIN_MODE_SLOW_ATTACK;
    if (name == "fast_attack") return plutosky::v1::GAIN_MODE_FAST_ATTACK;
    throw std::runtime_error("plutosky: unknown gain mode " + name);
}

static plutosky::v1::ChannelRef channelRef(Direction direction, size_t channel)
{
    plutosky::v1::ChannelRef ref;
    ref.set_dir(direction);
    ref.set_chan(static_cast<uint32_t>(channel));
    return ref;
}

ControlClient::ControlClient(const std::string &addr, int port, long timeoutUs)
    : addr_(addr), port_(port), timeoutUs_(timeoutUs)
{
    const std::vector<unsigned> before = taskThreads();
    channel_ = grpc::CreateChannel(addr + ":" + std::to_string(port),
                                   grpc::InsecureChannelCredentials());
    stub_ = plutosky::v1::Control::NewStub(channel_);

    plutosky::v1::GetInfoRequest request;
    request.set_client(kClientName);
    plutosky::v1::BoardInfo reply;
    auto context = callContext(false);
    const grpc::Status status = stub_->GetInfo(context.get(), request, &reply);
    recordLibraryThreads(before);
    releaseThreads(before);
    if (!status.ok())
    {
        throw std::runtime_error("plutosky: cannot reach " + addr + ": " + status.error_message());
    }
    info_.serial = reply.serial();
    info_.version = reply.version();
    info_.channels = reply.channels();
    info_.minRate = reply.min_rate_hz();
    info_.maxRate = reply.max_rate_hz();
    for (const auto format : reply.formats())
    {
        const std::string name = formatName(format);
        if (!name.empty()) info_.formats.push_back(name);
    }
    for (const auto mtu : reply.mtus()) info_.mtus.push_back(mtu);
}

ControlClient::~ControlClient()
{
    {
        std::lock_guard<std::mutex> lock(threadsMutex_);
        releaseThreads(libraryThreads_);
    }
    {
        std::lock_guard<std::mutex> lock(sessionMutex_);
        if (!session_) return;
        session_->WritesDone();
        sessionContext_->TryCancel();
    }
    if (sessionThread_.joinable()) sessionThread_.join();
    session_->Finish();
}

std::unique_ptr<grpc::ClientContext> ControlClient::callContext(bool session)
{
    std::unique_ptr<grpc::ClientContext> context(new grpc::ClientContext);
    context->set_deadline(std::chrono::system_clock::now() + std::chrono::microseconds(timeoutUs_));
    if (session)
    {
        ensureSession();
        context->AddMetadata(kSessionMetadataKey, sessionId_);
    }
    return context;
}

// The first call that changes the radio or a stream opens the session, so a
// client that only reads never takes radio control.
void ControlClient::ensureSession()
{
    std::lock_guard<std::mutex> lock(sessionMutex_);
    if (session_) return;
    const std::vector<unsigned> before = taskThreads();
    sessionContext_.reset(new grpc::ClientContext);
    session_ = stub_->Session(sessionContext_.get());
    plutosky::v1::SessionCommand start;
    start.mutable_start()->set_client(kClientName);
    plutosky::v1::SessionEvent opened;
    if (!session_->Write(start) || !session_->Read(&opened) || !opened.has_opened())
    {
        const grpc::Status status = session_->Finish();
        session_.reset();
        sessionContext_.reset();
        releaseThreads(before);
        throw std::runtime_error("plutosky: cannot open a control session: "
                                 + status.error_message());
    }
    sessionId_ = opened.opened().session_id();
    sessionThread_ = std::thread(&ControlClient::sessionLoop, this);
    std::vector<unsigned> exclude = before;
    exclude.push_back(machThreadOf(sessionThread_));
    recordLibraryThreads(exclude);
    releaseThreads(before);
}

// Keeps the threads that appeared since `before`, other than those listed in
// it. Only gRPC creates threads between the snapshots, apart from the
// session thread, which the caller adds to `before`.
void ControlClient::recordLibraryThreads(const std::vector<unsigned> &before)
{
    std::vector<unsigned> after = taskThreads();
    std::lock_guard<std::mutex> lock(threadsMutex_);
    for (const auto thread : after)
    {
        const bool known = std::find(before.begin(), before.end(), thread) != before.end() ||
                           std::find(libraryThreads_.begin(), libraryThreads_.end(), thread) !=
                               libraryThreads_.end();
        if (known)
        {
            releaseThreads({thread});
            continue;
        }
        libraryThreads_.push_back(thread);
    }
}

size_t ControlClient::restoreLibraryThreadPolicy()
{
    size_t restored = 0;
#ifdef __APPLE__
    std::lock_guard<std::mutex> lock(threadsMutex_);
    for (const auto thread : libraryThreads_)
    {
        thread_time_constraint_policy_data_t current{};
        mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
        // In: FALSE asks for the current policy. Out: TRUE when none is set.
        boolean_t isDefault = FALSE;
        if (thread_policy_get(thread, THREAD_TIME_CONSTRAINT_POLICY,
                              reinterpret_cast<thread_policy_t>(&current), &count,
                              &isDefault) != KERN_SUCCESS || isDefault)
        {
            continue;
        }
        thread_standard_policy_data_t standard{};
        if (thread_policy_set(thread, THREAD_STANDARD_POLICY,
                              reinterpret_cast<thread_policy_t>(&standard),
                              THREAD_STANDARD_POLICY_COUNT) == KERN_SUCCESS)
        {
            restored++;
        }
    }
#endif
    return restored;
}

void ControlClient::sessionLoop()
{
    plutosky::v1::SessionEvent event;
    while (session_->Read(&event))
    {
        if (!event.has_status()) continue;
        std::function<void(const plutosky::v1::StatusTick &)> handler;
        {
            std::lock_guard<std::mutex> lock(handlerMutex_);
            handler = statusHandler_;
        }
        if (handler) handler(event.status());
    }
}

void ControlClient::setStatusHandler(std::function<void(const plutosky::v1::StatusTick &)> handler)
{
    std::lock_guard<std::mutex> lock(handlerMutex_);
    statusHandler_ = std::move(handler);
}

void ControlClient::check(const grpc::Status &status, const char *op)
{
    if (!status.ok()) throw std::runtime_error(std::string("plutosky: ") + op + ": "
                                               + status.error_message());
}

std::string ControlClient::localAddress() const
{
    const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return "0.0.0.0";
    sockaddr_in dest;
    std::memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<uint16_t>(port_));
    std::string result = "0.0.0.0";
    if (::inet_pton(AF_INET, addr_.c_str(), &dest.sin_addr) == 1 &&
        ::connect(sock, reinterpret_cast<sockaddr *>(&dest), sizeof(dest)) == 0)
    {
        sockaddr_in local;
        socklen_t len = sizeof(local);
        char text[INET_ADDRSTRLEN] = {0};
        if (::getsockname(sock, reinterpret_cast<sockaddr *>(&local), &len) == 0 &&
            ::inet_ntop(AF_INET, &local.sin_addr, text, sizeof(text)) != nullptr)
        {
            result = text;
        }
    }
    ::close(sock);
    return result;
}

uint64_t ControlClient::setFrequency(Direction direction, size_t channel, uint64_t hz)
{
    plutosky::v1::SetFrequencyRequest request;
    request.set_dir(direction);
    request.set_chan(static_cast<uint32_t>(channel));
    request.set_hz(hz);
    plutosky::v1::FrequencyReply reply;
    auto context = callContext(true);
    check(stub_->SetFrequency(context.get(), request, &reply), "set_frequency");
    return reply.hz();
}

uint64_t ControlClient::frequency(Direction direction, size_t channel)
{
    plutosky::v1::FrequencyReply reply;
    auto context = callContext(false);
    check(stub_->GetFrequency(context.get(), channelRef(direction, channel), &reply),
          "get_frequency");
    return reply.hz();
}

static SampleRates sampleRatesOf(const plutosky::v1::SampleRateReply &reply)
{
    SampleRates rates;
    rates.rxHz = reply.rx_hz();
    rates.txHz = reply.tx_hz();
    rates.tickRateHz = reply.tick_rate_hz();
    rates.rxTicksPerSample = reply.rx_ticks_per_sample();
    rates.txTicksPerSample = reply.tx_ticks_per_sample();
    rates.rxDelayTicks = reply.rx_delay_ticks();
    rates.txDelayTicks = reply.tx_delay_ticks();
    if (rates.tickRateHz == 0 || rates.rxTicksPerSample == 0 || rates.txTicksPerSample == 0)
        throw std::runtime_error(
            "plutosky: the daemon reports no tick rate, it predates per-direction sample rates");
    return rates;
}

SampleRates ControlClient::setSampleRate(Direction direction, uint32_t hz)
{
    plutosky::v1::SetSampleRateRequest request;
    request.set_hz(hz);
    request.set_dir(direction);
    plutosky::v1::SampleRateReply reply;
    auto context = callContext(true);
    check(stub_->SetSampleRate(context.get(), request, &reply), "set_sample_rate");
    return sampleRatesOf(reply);
}

SampleRates ControlClient::sampleRates()
{
    plutosky::v1::SampleRateReply reply;
    auto context = callContext(false);
    check(stub_->GetSampleRate(context.get(), plutosky::v1::Empty(), &reply), "get_sample_rate");
    return sampleRatesOf(reply);
}

uint32_t ControlClient::setBandwidth(Direction direction, size_t channel, uint32_t hz)
{
    plutosky::v1::SetBandwidthRequest request;
    request.set_dir(direction);
    request.set_chan(static_cast<uint32_t>(channel));
    request.set_hz(hz);
    plutosky::v1::BandwidthReply reply;
    auto context = callContext(true);
    check(stub_->SetBandwidth(context.get(), request, &reply), "set_bandwidth");
    return reply.hz();
}

uint32_t ControlClient::bandwidth(Direction direction, size_t channel)
{
    plutosky::v1::BandwidthReply reply;
    auto context = callContext(false);
    check(stub_->GetBandwidth(context.get(), channelRef(direction, channel), &reply),
          "get_bandwidth");
    return reply.hz();
}

double ControlClient::setGain(Direction direction, size_t channel, double db)
{
    plutosky::v1::SetGainRequest request;
    request.set_dir(direction);
    request.set_chan(static_cast<uint32_t>(channel));
    request.set_db(db);
    plutosky::v1::GainReply reply;
    auto context = callContext(true);
    check(stub_->SetGain(context.get(), request, &reply), "set_gain");
    return reply.db();
}

double ControlClient::gain(Direction direction, size_t channel)
{
    plutosky::v1::GainReply reply;
    auto context = callContext(false);
    check(stub_->GetGain(context.get(), channelRef(direction, channel), &reply), "get_gain");
    return reply.db();
}

std::string ControlClient::setGainMode(Direction direction, size_t channel, const std::string &mode)
{
    plutosky::v1::SetGainModeRequest request;
    request.set_dir(direction);
    request.set_chan(static_cast<uint32_t>(channel));
    request.set_mode(gainModeValue(mode));
    plutosky::v1::GainModeReply reply;
    auto context = callContext(true);
    check(stub_->SetGainMode(context.get(), request, &reply), "set_gain_mode");
    return gainModeName(reply.mode());
}

std::vector<std::string> ControlClient::antennas(Direction direction)
{
    plutosky::v1::ListAntennasRequest request;
    request.set_dir(direction);
    plutosky::v1::AntennaList reply;
    auto context = callContext(false);
    check(stub_->ListAntennas(context.get(), request, &reply), "list_antennas");
    return std::vector<std::string>(reply.names().begin(), reply.names().end());
}

std::string ControlClient::setAntenna(Direction direction, size_t channel, const std::string &name)
{
    plutosky::v1::SetAntennaRequest request;
    request.set_dir(direction);
    request.set_chan(static_cast<uint32_t>(channel));
    request.set_name(name);
    plutosky::v1::AntennaReply reply;
    auto context = callContext(true);
    check(stub_->SetAntenna(context.get(), request, &reply), "set_antenna");
    return reply.name();
}

uint64_t ControlClient::hardwareTime()
{
    plutosky::v1::HardwareTime reply;
    auto context = callContext(false);
    check(stub_->GetHardwareTime(context.get(), plutosky::v1::Empty(), &reply),
          "get_hardware_time");
    return reply.ticks();
}

uint64_t ControlClient::setHardwareTime(uint64_t ticks)
{
    plutosky::v1::HardwareTime request;
    request.set_ticks(ticks);
    plutosky::v1::HardwareTime reply;
    auto context = callContext(true);
    check(stub_->SetHardwareTime(context.get(), request, &reply), "set_hardware_time");
    return reply.ticks();
}

Sensors ControlClient::sensors()
{
    plutosky::v1::Sensors reply;
    auto context = callContext(false);
    check(stub_->GetSensors(context.get(), plutosky::v1::Empty(), &reply), "get_sensors");
    Sensors sensors;
    sensors.temperatureC = reply.temperature_c();
    sensors.rssiDbfs.assign(reply.rssi_dbfs().begin(), reply.rssi_dbfs().end());
    sensors.rxLoLocked = reply.rx_lo_locked();
    sensors.txLoLocked = reply.tx_lo_locked();
    sensors.txRfEnabled = reply.tx_rf_enabled();
    return sensors;
}

OpenedStream ControlClient::openStream(const OpenStreamParams &params)
{
    plutosky::v1::OpenStreamRequest request;
    request.set_dir(params.direction);
    for (const auto channel : params.channels) request.add_chans(static_cast<uint32_t>(channel));
    request.set_format(formatValue(params.format));
    request.set_mtu(static_cast<uint32_t>(params.mtu));
    request.set_host_ip(params.hostIp);
    request.set_host_port(static_cast<uint32_t>(params.hostPort));
    plutosky::v1::OpenStreamReply reply;
    auto context = callContext(true);
    check(stub_->OpenStream(context.get(), request, &reply), "open_stream");
    OpenedStream opened;
    opened.streamId = reply.stream_id();
    opened.dataPort = static_cast<int>(reply.data_port());
    opened.samplesPerPacket = reply.samples_per_packet();
    opened.window = reply.window();
    return opened;
}

void ControlClient::startStream(int64_t streamId, bool timed, uint64_t atTicks)
{
    plutosky::v1::StartStreamRequest request;
    request.set_stream_id(static_cast<uint32_t>(streamId));
    if (timed) request.set_at_ticks(atTicks);
    plutosky::v1::Empty reply;
    auto context = callContext(true);
    check(stub_->StartStream(context.get(), request, &reply), "start_stream");
}

void ControlClient::stopStream(int64_t streamId)
{
    plutosky::v1::StreamRef request;
    request.set_stream_id(static_cast<uint32_t>(streamId));
    plutosky::v1::Empty reply;
    auto context = callContext(true);
    check(stub_->StopStream(context.get(), request, &reply), "stop_stream");
}

void ControlClient::closeStream(int64_t streamId)
{
    plutosky::v1::StreamRef request;
    request.set_stream_id(static_cast<uint32_t>(streamId));
    plutosky::v1::Empty reply;
    auto context = callContext(true);
    check(stub_->CloseStream(context.get(), request, &reply), "close_stream");
}

}  // namespace PlutoSky
