// The three packers and unpackers against tools/vectors/wire_formats.json,
// plus the samples-per-packet arithmetic for both MTUs.

#include "Check.hpp"
#include "Vectors.hpp"
#include "Vita.hpp"

#include <vector>

using namespace PlutoSky;
using PlutoSkyTest::fromHex;
using PlutoSkyTest::toHex;

static std::vector<Sample> samplesOf(const Json &array)
{
    std::vector<Sample> out;
    for (size_t i = 0; i < array.size(); i++)
    {
        Sample sample;
        sample.i = static_cast<int16_t>(array.at(i).at(0).asInt());
        sample.q = static_cast<int16_t>(array.at(i).at(1).asInt());
        out.push_back(sample);
    }
    return out;
}

static void testCodecVectors(const Json &doc)
{
    const Json &cases = doc["codec"];
    CHECK(cases.size() >= 5);
    for (size_t c = 0; c < cases.size(); c++)
    {
        const Json &item = cases.at(c);
        WireFormat wire;
        CHECK(wireFormatFromName(item["format"].asString(), wire));

        const std::vector<Sample> samples = samplesOf(item["samples"]);
        std::vector<uint8_t> payload(payloadBytesFor(wire, samples.size()));
        packSamples(wire, samples.data(), samples.size(), payload.data());
        CHECK_STR(toHex(payload.data(), payload.size()), item["payload_hex"].asString());

        const std::vector<Sample> expected = samplesOf(item["decoded"]);
        std::vector<Sample> decoded(samples.size());
        unpackSamples(wire, payload.data(), 0, samples.size(), decoded.data());
        for (size_t n = 0; n < samples.size(); n++)
        {
            CHECK_EQ(decoded[n].i, expected[n].i);
            CHECK_EQ(decoded[n].q, expected[n].q);
        }

        // Unpacking from an offset must match unpacking the whole payload.
        if (samples.size() > 3)
        {
            const size_t first = wire == WireFormat::Bfp8 ? 17 : 3;
            const size_t count = samples.size() - first;
            std::vector<Sample> tail(count);
            unpackSamples(wire, payload.data(), first, count, tail.data());
            for (size_t n = 0; n < count; n++)
            {
                CHECK_EQ(tail[n].i, expected[first + n].i);
                CHECK_EQ(tail[n].q, expected[first + n].q);
            }
        }
    }
}

static void testBfpBlocks(const Json &doc)
{
    const Json &blocks = doc["bfp8_blocks"];
    CHECK(blocks.size() >= 8);
    for (size_t b = 0; b < blocks.size(); b++)
    {
        const Json &item = blocks.at(b);
        const std::vector<Sample> samples = samplesOf(item["samples"]);
        CHECK_EQ(samples.size(), kBfp8BlockSamples);

        CHECK_EQ(bfp8Exponent(static_cast<int>(item["peak_magnitude"].asInt())),
                 static_cast<int>(item["exponent"].asInt()));

        std::vector<uint8_t> payload(kBfp8BlockBytes);
        packSamples(WireFormat::Bfp8, samples.data(), samples.size(), payload.data());
        CHECK_STR(toHex(payload.data(), payload.size()), item["payload_hex"].asString());
        CHECK_EQ(payload[kBfp8BlockBytes - 1], static_cast<uint8_t>(item["exponent"].asInt()));

        for (size_t n = 0; n < 32; n++)
        {
            CHECK_EQ(static_cast<int>(static_cast<int8_t>(payload[n])),
                     static_cast<int>(item["mantissas"].at(n).asInt()));
        }

        const std::vector<Sample> expected = samplesOf(item["decoded"]);
        std::vector<Sample> decoded(samples.size());
        unpackSamples(WireFormat::Bfp8, payload.data(), 0, samples.size(), decoded.data());
        for (size_t n = 0; n < samples.size(); n++)
        {
            CHECK_EQ(decoded[n].i, expected[n].i);
            CHECK_EQ(decoded[n].q, expected[n].q);
        }
    }
}

static void testBfpRounding()
{
    // Halves round away from zero, and the exponent is the smallest that fits.
    CHECK_EQ(bfp8Round(3, 1), 2);
    CHECK_EQ(bfp8Round(-3, 1), -2);
    CHECK_EQ(bfp8Round(5, 1), 3);
    CHECK_EQ(bfp8Round(-5, 1), -3);
    CHECK_EQ(bfp8Round(4, 1), 2);
    CHECK_EQ(bfp8Round(7, 2), 2);
    CHECK_EQ(bfp8Round(-7, 2), -2);
    CHECK_EQ(bfp8Round(6, 2), 2);
    CHECK_EQ(bfp8Round(123, 0), 123);

    CHECK_EQ(bfp8Exponent(0), 0);
    CHECK_EQ(bfp8Exponent(1), 0);
    CHECK_EQ(bfp8Exponent(127), 0);
    CHECK_EQ(bfp8Exponent(128), 1);
    CHECK_EQ(bfp8Exponent(255), 1);
    CHECK_EQ(bfp8Exponent(256), 2);
    CHECK_EQ(bfp8Exponent(511), 2);
    CHECK_EQ(bfp8Exponent(1024), 4);
    CHECK_EQ(bfp8Exponent(2047), 4);
    CHECK_EQ(bfp8Exponent(2048), 5);
}

static void testSizing(const Json &doc)
{
    const Json &sizes = doc["samples_per_packet"];
    CHECK_EQ(sizes.size(), size_t(6));
    for (size_t i = 0; i < sizes.size(); i++)
    {
        const Json &item = sizes.at(i);
        WireFormat wire;
        CHECK(wireFormatFromName(item["format"].asString(), wire));
        const size_t mtu = static_cast<size_t>(item["mtu"].asInt());
        const size_t count = samplesPerPacket(wire, mtu);
        CHECK_EQ(count, static_cast<size_t>(item["samples"].asInt()));
        CHECK_EQ(payloadBytesFor(wire, count), static_cast<size_t>(item["payload_bytes"].asInt()));
        CHECK_EQ(packetWords(wire, count), static_cast<size_t>(item["packet_words"].asInt()));

        // A full packet must fit the MTU even when a burst ends on it and it
        // grows the one-word trailer.
        const size_t onWire = packetWords(wire, count) * kWordBytes + kIpv4HeaderBytes
                            + kUdpHeaderBytes;
        CHECK(onWire <= mtu);
        CHECK(onWire + kVitaTrailerBytes <= mtu);

        // A payload that already fills whole words takes no padding.
        CHECK_EQ(paddedPayloadBytes(payloadBytesFor(wire, count)) % kWordBytes, size_t(0));
        CHECK(paddedPayloadBytes(payloadBytesFor(wire, count))
              - payloadBytesFor(wire, count) < kWordBytes);

        // One more sample, or one more block, must not fit.
        const size_t step = wire == WireFormat::Bfp8 ? kBfp8BlockSamples : 1;
        CHECK(paddedPayloadBytes(payloadBytesFor(wire, count + step)) > payloadCapacity(mtu));
    }
}

static void testSamplesInPayload()
{
    // The payload lengths a full 1500 MTU packet holds in each format.
    CHECK_EQ(samplesInPayload(WireFormat::Cs12, 1443), size_t(481));
    CHECK_EQ(samplesInPayload(WireFormat::Cs8, 1444), size_t(722));
    CHECK_EQ(samplesInPayload(WireFormat::Bfp8, 1419), size_t(688));
    // Pad bytes at the end of a short payload never become samples.
    CHECK_EQ(samplesInPayload(WireFormat::Cs12, 1444), size_t(481));
    CHECK_EQ(samplesInPayload(WireFormat::Bfp8, 1420), size_t(688));
}

int main()
{
    const Json doc = PlutoSkyTest::loadVectors();
    testCodecVectors(doc);
    testBfpBlocks(doc);
    testBfpRounding();
    testSizing(doc);
    testSamplesInPayload();
    return PlutoSkyTest::summarize("test_formats");
}
