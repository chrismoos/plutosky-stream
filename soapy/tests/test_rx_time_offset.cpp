#include "Check.hpp"
#include "RxTimeOffset.hpp"

#include <stdexcept>

using namespace PlutoSky;

static bool rejects(const char *value)
{
    try { (void)parseRxTimeOffsetNs({{"rx_time_offset_ns", value}}); }
    catch (const std::runtime_error &) { return true; }
    return false;
}

int main()
{
    static const double kGsmRate = 1625e3 / 6 * 4;

    CHECK(parseRxTimeOffsetNs({}) == 0);
    CHECK(parseRxTimeOffsetNs({{"rx_time_offset_ns", "-12000"}}) == -12000);
    CHECK(parseRxTimeOffsetNs({{"rx_time_offset_ns", "61846"}}) == 61846);
    CHECK(rejects(""));
    CHECK(rejects("12us"));
    CHECK(rejects("1.5"));
    CHECK(rejects("99999999999999999999"));

    CHECK(rxTimeOffsetSamples(0, kGsmRate) == 0);
    CHECK(rxTimeOffsetSamples(1000, 1e6) == 1);
    CHECK(rxTimeOffsetSamples(-1000, 1e6) == -1);
    CHECK(rxTimeOffsetSamples(1499, 1e6) == 1);
    CHECK(rxTimeOffsetSamples(1500, 1e6) == 2);
    // One GSM symbol at 4 samples per symbol is 3692.3 ns.
    CHECK(rxTimeOffsetSamples(3692, kGsmRate) == 4);
    CHECK(rxTimeOffsetSamples(-3692, kGsmRate) == -4);

    CHECK(rxClientTicks(1000, 0) == 1000);
    CHECK(rxClientTicks(1000, 40) == 960);
    CHECK(rxClientTicks(1000, -40) == 1040);
    CHECK(rxClientTicks(10, 40) == -30);

    for (const long long offset : {0LL, 40LL, -40LL})
        CHECK(rxClientTicks(rxBoardTicks(5000, offset), offset) == 5000);
    CHECK(rxBoardTicks(10, -40) == 0);

    DeviceArgs found{{"driver", "plutosky"}, {"addr", "192.168.200.10"}};
    keepRxTimeOffsetArg(found, {{"driver", "plutosky"}});
    CHECK(found.count("rx_time_offset_ns") == 0);
    keepRxTimeOffsetArg(found, {{"driver", "plutosky"}, {"rx_time_offset_ns", "-12000"}});
    CHECK(parseRxTimeOffsetNs(found) == -12000);

    return PlutoSkyTest::summarize("test_rx_time_offset");
}
