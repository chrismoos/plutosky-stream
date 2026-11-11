// Host buffer conversion: every wire format against CS16 and CF32, the native
// raw paths, and the two-channel interleave.

#include "Check.hpp"
#include "Convert.hpp"
#include "Vita.hpp"

#include <vector>

using namespace PlutoSky;

static std::vector<Sample> ramp(size_t count)
{
    std::vector<Sample> out(count);
    for (size_t n = 0; n < count; n++)
    {
        out[n].i = static_cast<int16_t>(kSampleMin + static_cast<int>(n * 37 % 4096));
        out[n].q = static_cast<int16_t>(kSampleMax - static_cast<int>(n * 53 % 4096));
    }
    return out;
}

static void testFormatPolicy()
{
    CHECK(hostFormatAllowed(WireFormat::Cs12, HostFormat::Cs12));
    CHECK(hostFormatAllowed(WireFormat::Cs12, HostFormat::Cs16));
    CHECK(hostFormatAllowed(WireFormat::Cs12, HostFormat::Cf32));
    CHECK(!hostFormatAllowed(WireFormat::Cs12, HostFormat::Cs8));
    CHECK(hostFormatAllowed(WireFormat::Cs8, HostFormat::Cs8));
    CHECK(!hostFormatAllowed(WireFormat::Cs8, HostFormat::Cs12));
    CHECK(hostFormatAllowed(WireFormat::Bfp8, HostFormat::Cs16));
    CHECK(hostFormatAllowed(WireFormat::Bfp8, HostFormat::Cf32));
    CHECK(!hostFormatAllowed(WireFormat::Bfp8, HostFormat::Cs12));

    CHECK(nativeHostFormat(WireFormat::Cs12) == HostFormat::Cs12);
    CHECK(nativeHostFormat(WireFormat::Cs8) == HostFormat::Cs8);
    CHECK(nativeHostFormat(WireFormat::Bfp8) == HostFormat::Cs16);
}

// A CS12 round trip is exact, so the host buffer must come back unchanged.
static void testCs12RoundTrip()
{
    const std::vector<Sample> samples = ramp(64);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs12, samples.size()));
    packSamples(WireFormat::Cs12, samples.data(), samples.size(), payload.data());

    std::vector<int16_t> host(samples.size() * 2);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cs16, payload.data(), 0, samples.size(), 1, buffs,
                  0, staging.data());
    for (size_t n = 0; n < samples.size(); n++)
    {
        CHECK_EQ(host[n * 2 + 0], samples[n].i);
        CHECK_EQ(host[n * 2 + 1], samples[n].q);
    }

    std::vector<uint8_t> again(payload.size());
    const void *inputs[1] = {host.data()};
    hostToPayload(WireFormat::Cs12, HostFormat::Cs16, inputs, 0, samples.size(), 1, again.data(), 0,
                  staging.data());
    CHECK(again == payload);
}

// The full-scale CS16 that UHD applications use holds the 12-bit value in
// the top bits and rounds the low bits away on transmit.
static void testCs16MsbScale()
{
    const std::vector<Sample> samples = ramp(64);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs12, samples.size()));
    packSamples(WireFormat::Cs12, samples.data(), samples.size(), payload.data());

    std::vector<int16_t> host(samples.size() * 2);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cs16Msb, payload.data(), 0, samples.size(), 1,
                  buffs, 0, staging.data());
    for (size_t n = 0; n < samples.size(); n++)
    {
        CHECK_EQ(host[n * 2 + 0], samples[n].i * 16);
        CHECK_EQ(host[n * 2 + 1], samples[n].q * 16);
    }

    std::vector<uint8_t> again(payload.size());
    const void *inputs[1] = {host.data()};
    hostToPayload(WireFormat::Cs12, HostFormat::Cs16Msb, inputs, 0, samples.size(), 1,
                  again.data(), 0, staging.data());
    CHECK(again == payload);

    const int16_t extremes[4] = {32767, -32768, 7, 8};
    const void *extremeInputs[1] = {extremes};
    std::vector<Sample> decoded(2);
    std::vector<uint8_t> packed(payloadBytesFor(WireFormat::Cs12, decoded.size()));
    hostToPayload(WireFormat::Cs12, HostFormat::Cs16Msb, extremeInputs, 0, decoded.size(), 1,
                  packed.data(), 0, staging.data());
    unpackSamples(WireFormat::Cs12, packed.data(), 0, decoded.size(), decoded.data());
    CHECK_EQ(decoded[0].i, kSampleMax);
    CHECK_EQ(decoded[0].q, kSampleMin);
    CHECK_EQ(decoded[1].i, 0);
    CHECK_EQ(decoded[1].q, 1);

    HostFormat format = HostFormat::Cs16;
    CHECK(cs16FormatFromScale("16", format));
    CHECK(format == HostFormat::Cs16Msb);
    CHECK(cs16FormatFromScale("12", format));
    CHECK(format == HostFormat::Cs16);
    CHECK(!cs16FormatFromScale("14", format));
    CHECK(hostFormatAllowed(WireFormat::Bfp8, HostFormat::Cs16Msb));
}

static void testCf32RoundTrip()
{
    const std::vector<Sample> samples = ramp(32);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs12, samples.size()));
    packSamples(WireFormat::Cs12, samples.data(), samples.size(), payload.data());

    std::vector<float> host(samples.size() * 2);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cf32, payload.data(), 0, samples.size(), 1, buffs,
                  0, staging.data());
    for (size_t n = 0; n < samples.size(); n++)
    {
        CHECK_NEAR(host[n * 2 + 0], samples[n].i / kSampleFullScale, 1e-6);
        CHECK_NEAR(host[n * 2 + 1], samples[n].q / kSampleFullScale, 1e-6);
    }

    std::vector<uint8_t> again(payload.size());
    const void *inputs[1] = {host.data()};
    hostToPayload(WireFormat::Cs12, HostFormat::Cf32, inputs, 0, samples.size(), 1, again.data(), 0,
                  staging.data());
    CHECK(again == payload);
}

static void testRawNativePath()
{
    const std::vector<Sample> samples = ramp(48);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs12, samples.size()));
    packSamples(WireFormat::Cs12, samples.data(), samples.size(), payload.data());

    std::vector<uint8_t> host(samples.size() * kCs12SampleBytes);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cs12, payload.data(), 0, samples.size(), 1, buffs,
                  0, staging.data());
    CHECK(host == payload);

    // The generic path must agree with the raw copy byte for byte.
    std::vector<int16_t> viaCs16(samples.size() * 2);
    void *wide[1] = {viaCs16.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cs16, payload.data(), 0, samples.size(), 1, wide, 0,
                  staging.data());
    std::vector<uint8_t> repacked(payload.size());
    const void *inputs[1] = {viaCs16.data()};
    hostToPayload(WireFormat::Cs12, HostFormat::Cs16, inputs, 0, samples.size(), 1, repacked.data(),
                  0, staging.data());
    CHECK(repacked == host);
}

static void testCs8Scaling()
{
    std::vector<Sample> samples(4);
    samples[0].i = 2047; samples[0].q = -2048;
    samples[1].i = 16; samples[1].q = -16;
    samples[2].i = 15; samples[2].q = -15;
    samples[3].i = 0; samples[3].q = 0;

    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs8, samples.size()));
    packSamples(WireFormat::Cs8, samples.data(), samples.size(), payload.data());
    CHECK_EQ(static_cast<int>(static_cast<int8_t>(payload[0])), 127);
    CHECK_EQ(static_cast<int>(static_cast<int8_t>(payload[1])), -128);

    // CS16 keeps the 12-bit scale, so a cs8 sample reads back shifted up by four.
    std::vector<int16_t> host(samples.size() * 2);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    payloadToHost(WireFormat::Cs8, HostFormat::Cs16, payload.data(), 0, samples.size(), 1, buffs, 0,
                  staging.data());
    CHECK_EQ(host[0], int16_t(2032));
    CHECK_EQ(host[1], int16_t(-2048));
    CHECK_EQ(host[2], int16_t(16));
    CHECK_EQ(host[3], int16_t(-16));
    CHECK_EQ(host[4], int16_t(0));
}

static void testTwoChannelInterleave()
{
    const size_t elems = 16;
    std::vector<int16_t> ch0(elems * 2), ch1(elems * 2);
    for (size_t n = 0; n < elems; n++)
    {
        ch0[n * 2 + 0] = static_cast<int16_t>(100 + n);
        ch0[n * 2 + 1] = static_cast<int16_t>(-(100 + static_cast<int>(n)));
        ch1[n * 2 + 0] = static_cast<int16_t>(-(900 + static_cast<int>(n)));
        ch1[n * 2 + 1] = static_cast<int16_t>(900 + n);
    }

    std::vector<Sample> staging(elems * 2 + kBfp8BlockSamples);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Cs12, elems * 2));
    const void *inputs[2] = {ch0.data(), ch1.data()};
    hostToPayload(WireFormat::Cs12, HostFormat::Cs16, inputs, 0, elems, 2, payload.data(), 0,
                  staging.data());

    // Channel 0 sample n sits immediately before channel 1 sample n.
    std::vector<Sample> onWire(elems * 2);
    unpackSamples(WireFormat::Cs12, payload.data(), 0, elems * 2, onWire.data());
    for (size_t n = 0; n < elems; n++)
    {
        CHECK_EQ(onWire[n * 2 + 0].i, ch0[n * 2 + 0]);
        CHECK_EQ(onWire[n * 2 + 1].i, ch1[n * 2 + 0]);
    }

    std::vector<int16_t> back0(elems * 2, 0), back1(elems * 2, 0);
    void *outputs[2] = {back0.data(), back1.data()};
    payloadToHost(WireFormat::Cs12, HostFormat::Cs16, payload.data(), 0, elems, 2, outputs, 0,
                  staging.data());
    CHECK(back0 == ch0);
    CHECK(back1 == ch1);
}

// A partial read leaves the rest of the packet for the next call.
static void testPartialConsumption()
{
    const std::vector<Sample> samples = ramp(32);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Bfp8, samples.size()));
    packSamples(WireFormat::Bfp8, samples.data(), samples.size(), payload.data());

    std::vector<Sample> reference(samples.size());
    unpackSamples(WireFormat::Bfp8, payload.data(), 0, samples.size(), reference.data());

    std::vector<int16_t> host(samples.size() * 2, 0);
    std::vector<Sample> staging(samples.size());
    void *buffs[1] = {host.data()};
    const size_t firstChunk = 21;
    payloadToHost(WireFormat::Bfp8, HostFormat::Cs16, payload.data(), 0, firstChunk, 1, buffs, 0,
                  staging.data());
    payloadToHost(WireFormat::Bfp8, HostFormat::Cs16, payload.data(), firstChunk,
                  samples.size() - firstChunk, 1, buffs, firstChunk, staging.data());
    for (size_t n = 0; n < samples.size(); n++)
    {
        CHECK_EQ(host[n * 2 + 0], reference[n].i);
        CHECK_EQ(host[n * 2 + 1], reference[n].q);
    }
}

// A transmit tail shorter than a block is padded with zeros at the block exponent.
static void testBfpTailPadding()
{
    const size_t elems = 20;
    std::vector<int16_t> host(elems * 2);
    for (size_t n = 0; n < elems * 2; n++) host[n] = static_cast<int16_t>(n + 1);

    const size_t pad = kBfp8BlockSamples - (elems % kBfp8BlockSamples);
    std::vector<Sample> staging(elems + pad);
    std::vector<uint8_t> payload(payloadBytesFor(WireFormat::Bfp8, elems + pad));
    const void *inputs[1] = {host.data()};
    hostToPayload(WireFormat::Bfp8, HostFormat::Cs16, inputs, 0, elems, 1, payload.data(), pad,
                  staging.data());

    std::vector<Sample> decoded(elems + pad);
    unpackSamples(WireFormat::Bfp8, payload.data(), 0, elems + pad, decoded.data());
    for (size_t n = elems; n < elems + pad; n++)
    {
        CHECK_EQ(decoded[n].i, int16_t(0));
        CHECK_EQ(decoded[n].q, int16_t(0));
    }
    CHECK_EQ(payload.size(), size_t(2 * kBfp8BlockBytes));
}

int main()
{
    testFormatPolicy();
    testCs12RoundTrip();
    testCs16MsbScale();
    testCf32RoundTrip();
    testRawNativePath();
    testCs8Scaling();
    testTwoChannelInterleave();
    testPartialConsumption();
    testBfpTailPadding();
    return PlutoSkyTest::summarize("test_convert");
}
