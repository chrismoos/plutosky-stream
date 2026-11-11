#include "Check.hpp"
#include "TxPacer.hpp"

using namespace PlutoSky;

int main()
{
    CHECK_NEAR(TxPacer::parseRateLimit("10000000", "duplex_tx_rate_limit_sps"),
               10000000.0, 1e-12);
    CHECK_NEAR(TxPacer::parseRateLimit("36000000", "tx_only_rate_limit_sps"),
               36000000.0, 1e-12);
    for (const auto *value : {"0", "0.5", "-1", "nan", "12e6x", ""})
    {
        bool rejected = false;
        try { TxPacer::parseRateLimit(value, "duplex_tx_rate_limit_sps"); }
        catch (const std::runtime_error &) { rejected = true; }
        CHECK(rejected);
    }

    CHECK_NEAR(TxPacer::duplexRateLimit(TxPacer::kRateLimitUnset, 5e6), 10e6, 1e-6);
    CHECK_NEAR(TxPacer::duplexRateLimit(TxPacer::kRateLimitUnset, 15.36e6), 15.36e6, 1e-6);
    CHECK_NEAR(TxPacer::duplexRateLimit(8e6, 5e6), 8e6, 1e-6);
    CHECK_NEAR(TxPacer::duplexRateLimit(2e6, 5e6), 5e6, 1e-6);

    TxPacer pacer;
    const auto start = TxPacer::Time{} + std::chrono::seconds(1);
    const size_t samples = 500;
    const double rateLimit = 10e6;
    for (size_t i = 0; i < TxPacer::kBurstPackets; ++i)
    {
        CHECK(pacer.readyAt(start) == start);
        pacer.sent(start, samples, rateLimit);
    }
    const auto period = std::chrono::nanoseconds(50000 * TxPacer::kBurstPackets);
    const auto next = start + period;
    CHECK(pacer.readyAt(start) == next);

    const auto delayed = next + std::chrono::seconds(1);
    for (size_t i = 0; i < TxPacer::kBurstPackets; ++i)
    {
        CHECK(pacer.readyAt(delayed) == delayed);
        pacer.sent(delayed, samples, rateLimit);
    }
    CHECK(pacer.readyAt(delayed) == delayed + period);
    pacer.reset();
    CHECK(pacer.readyAt(delayed) == delayed);

    pacer.sent(start, samples, 5e6);
    CHECK(pacer.readyAt(start) == start + std::chrono::microseconds(100));

    const auto first = start + std::chrono::microseconds(103);
    pacer.sent(first, samples, 5e6);
    CHECK(pacer.readyAt(first) == start + std::chrono::microseconds(200));

    const auto stalled = start + std::chrono::microseconds(260);
    pacer.sent(stalled, samples, 5e6);
    CHECK(pacer.readyAt(stalled) == stalled + std::chrono::microseconds(100));
    return PlutoSkyTest::summarize("test_tx_pacer");
}
