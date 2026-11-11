// Client for the daemon's gRPC control service in proto/plutosky/v1/control.proto,
// plus the UDP broadcast discovery probe.

#pragma once

#include "plutosky/v1/control.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace PlutoSky {

static const int kDefaultControlPort = 30000;
static const int kDiscoveryPort = 30001;
static const char *const kDiscoveryProbe = "plutosky-stream?";
// set_sample_rate reruns the LVDS calibration on the board, which takes a few seconds.
static const long kDefaultRequestTimeoutUs = 10000000;
static const long kDiscoveryTimeoutUs = 400000;

using Direction = plutosky::v1::Direction;

struct DiscoveryResult {
    std::string addr;
    plutosky::v1::DiscoveryReply reply;
};

std::vector<DiscoveryResult> discover(long timeoutUs = kDiscoveryTimeoutUs,
                                      int port = kDiscoveryPort);

struct BoardInfo {
    std::string serial;
    std::string version;
    size_t channels = 1;
    double minRate = 0.0;
    double maxRate = 0.0;
    std::vector<std::string> formats;
    std::vector<size_t> mtus;
};

struct Sensors {
    double temperatureC = 0.0;
    std::vector<double> rssiDbfs;
    bool rxLoLocked = false;
    bool txLoLocked = false;
    bool txRfEnabled = false;
};

struct OpenedStream {
    int64_t streamId = -1;
    int dataPort = 0;
    size_t samplesPerPacket = 0;
    size_t window = 1;
};

struct OpenStreamParams {
    Direction direction = plutosky::v1::DIRECTION_RX;
    std::vector<size_t> channels;
    std::string format;
    size_t mtu = 0;
    std::string hostIp;
    int hostPort = 0;
};

// The rate of each direction, the rate the timebase counts at, and the
// timebase counts one sample of each direction spans.
struct SampleRates
{
    uint32_t rxHz = 0;
    uint32_t txHz = 0;
    uint32_t tickRateHz = 0;
    uint32_t rxTicksPerSample = 1;
    uint32_t txTicksPerSample = 1;
    // Timebase counts the board's rate filter delays each direction by.
    uint32_t rxDelayTicks = 0;
    uint32_t txDelayTicks = 0;
};

class ControlClient {
public:
    ControlClient(const std::string &addr, int port, long timeoutUs = kDefaultRequestTimeoutUs);
    ~ControlClient();

    ControlClient(const ControlClient &) = delete;
    ControlClient &operator=(const ControlClient &) = delete;

    // Each call throws std::runtime_error with the daemon's message on failure.
    uint64_t setFrequency(Direction direction, size_t channel, uint64_t hz);
    uint64_t frequency(Direction direction, size_t channel);
    // DIRECTION_UNSPECIFIED sets both directions.
    SampleRates setSampleRate(Direction direction, uint32_t hz);
    SampleRates sampleRates();
    uint32_t setBandwidth(Direction direction, size_t channel, uint32_t hz);
    uint32_t bandwidth(Direction direction, size_t channel);
    double setGain(Direction direction, size_t channel, double db);
    double gain(Direction direction, size_t channel);
    // Mode names are manual, slow_attack and fast_attack.
    std::string setGainMode(Direction direction, size_t channel, const std::string &mode);
    std::vector<std::string> antennas(Direction direction);
    std::string setAntenna(Direction direction, size_t channel, const std::string &name);
    uint64_t hardwareTime();
    uint64_t setHardwareTime(uint64_t ticks);
    Sensors sensors();
    OpenedStream openStream(const OpenStreamParams &params);
    // atTicks is honored for receive streams. Zero with timed false starts at once.
    void startStream(int64_t streamId, bool timed, uint64_t atTicks);
    void stopStream(int64_t streamId);
    void closeStream(int64_t streamId);

    // Called from the session thread with every status tick.
    void setStatusHandler(std::function<void(const plutosky::v1::StatusTick &)> handler);

    const BoardInfo &info() const { return info_; }
    const std::string &addr() const { return addr_; }
    int port() const { return port_; }

    // Local address the daemon reaches this host on, for open_stream.
    std::string localAddress() const;

    // Returns the threads gRPC started inside this client that hold a
    // time-constraint policy to the standard policy, and reports how many it
    // changed. A host that promotes every thread created while it opens the
    // device would otherwise run gRPC's idle pool as real-time threads.
    // Effective on macOS only.
    size_t restoreLibraryThreadPolicy();

private:
    std::unique_ptr<grpc::ClientContext> callContext(bool session);
    void ensureSession();
    void sessionLoop();
    void check(const grpc::Status &status, const char *op);
    void recordLibraryThreads(const std::vector<unsigned> &before);

    std::string addr_;
    int port_;
    long timeoutUs_;
    BoardInfo info_;
    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<plutosky::v1::Control::Stub> stub_;

    std::mutex sessionMutex_;
    std::string sessionId_;
    std::unique_ptr<grpc::ClientContext> sessionContext_;
    std::unique_ptr<grpc::ClientReaderWriter<plutosky::v1::SessionCommand,
                                             plutosky::v1::SessionEvent>> session_;
    std::thread sessionThread_;

    std::mutex threadsMutex_;
    std::vector<unsigned> libraryThreads_;

    std::mutex handlerMutex_;
    std::function<void(const plutosky::v1::StatusTick &)> statusHandler_;
};

}  // namespace PlutoSky
