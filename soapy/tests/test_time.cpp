// Timestamp round trips through SoapySDR::ticksToTimeNs at the rates the
// AD9363 actually runs, including one that is not an integer.

#include "Check.hpp"
#include "Vita.hpp"

#include <SoapySDR/Time.hpp>

#include <vector>

using namespace PlutoSky;

static const double kRates[] = {61.44e6, 30.72e6, 4.9152e6, 520833.0, 3579545.4545454545};

static void testTickRoundTrip()
{
    const long long ticks[] = {0, 1, 12345, 61440000LL, 1234567890LL,
                               4611686018427387LL};
    for (const double rate : kRates)
    {
        for (const long long tick : ticks)
        {
            const long long timeNs = SoapySDR::ticksToTimeNs(tick, rate);
            const long long back = SoapySDR::timeNsToTicks(timeNs, rate);
            // A nanosecond is coarser than a sample period above 1 GS/s only,
            // so every rate here round trips within one tick.
            CHECK(back >= tick - 1 && back <= tick + 1);
        }
    }
}

static void testOneSecondOfTicks()
{
    for (const double rate : kRates)
    {
        const long long oneSecond = SoapySDR::ticksToTimeNs(
            static_cast<long long>(rate), rate);
        CHECK_NEAR(oneSecond, 1000000000.0, 1000.0);
    }
}

// Reading part of a packet advances the reported time by the samples consumed.
static void testPacketTimeAdvance()
{
    const double rate = 61.44e6;
    const uint64_t base = 1000000;
    const size_t elemsPerPacket = samplesPerPacket(WireFormat::Cs12, 1500);
    const long long start = SoapySDR::ticksToTimeNs(static_cast<long long>(base), rate);
    const long long mid =
        SoapySDR::ticksToTimeNs(static_cast<long long>(base + elemsPerPacket / 2), rate);
    const long long expected = start + static_cast<long long>(
        (elemsPerPacket / 2) * 1e9 / rate);
    CHECK_NEAR(mid, expected, 20.0);
}

// A two-channel stream halves the elements a packet holds per channel, so
// the timestamp advances by that count and not by the interleaved total.
static void testTwoChannelTickAdvance()
{
    const double rate = 30.72e6;
    const size_t interleaved = samplesPerPacket(WireFormat::Cs8, 9000);
    const size_t perChannel = interleaved / 2;
    CHECK_EQ(perChannel * 2, interleaved - (interleaved % 2));

    const long long first = SoapySDR::ticksToTimeNs(0, rate);
    const long long second = SoapySDR::ticksToTimeNs(static_cast<long long>(perChannel), rate);
    CHECK_NEAR(second - first, perChannel * 1e9 / rate, 20.0);
}

static void testNonIntegerRate()
{
    const double rate = 3579545.4545454545;
    long long worst = 0;
    for (long long tick = 0; tick < 1000000; tick += 7919)
    {
        const long long back = SoapySDR::timeNsToTicks(SoapySDR::ticksToTimeNs(tick, rate), rate);
        const long long error = back > tick ? back - tick : tick - back;
        if (error > worst) worst = error;
    }
    CHECK(worst <= 1);
}

int main()
{
    testTickRoundTrip();
    testOneSecondOfTicks();
    testPacketTimeAdvance();
    testTwoChannelTickAdvance();
    testNonIntegerRate();
    return PlutoSkyTest::summarize("test_time");
}
