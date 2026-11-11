// Packet build and parse against the shared vectors, plus the header bit
// layout and the packet count gap arithmetic.

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

static void testPacketVectors(const Json &doc)
{
    const Json &packets = doc["packets"];
    CHECK(packets.size() >= 4);
    for (size_t p = 0; p < packets.size(); p++)
    {
        const Json &item = packets.at(p);
        WireFormat wire;
        CHECK(wireFormatFromName(item["format"].asString(), wire));

        const std::vector<Sample> samples = samplesOf(item["samples"]);
        const bool endOfBurst = item["end_of_burst"].asBool();
        const uint64_t ticks = static_cast<uint64_t>(item["ticks"].asInt());
        const uint32_t streamId = static_cast<uint32_t>(item["stream_id"].asInt());
        const uint32_t count = static_cast<uint32_t>(item["packet_count"].asInt());

        const size_t payloadBytes = payloadBytesFor(wire, samples.size());
        const size_t total = packetBytes(payloadBytes, endOfBurst);
        std::vector<uint8_t> datagram(total, 0);
        writePacketHeader(datagram.data(), streamId, packetClassOf(wire), count, ticks,
                          payloadBytes, endOfBurst);
        packSamples(wire, samples.data(), samples.size(), datagram.data() + kVitaHeaderBytes);
        if (endOfBurst) writeTrailer(datagram.data() + total - kWordBytes, true);

        CHECK_STR(toHex(datagram.data(), datagram.size()), item["packet_hex"].asString());

        PacketHeader header;
        CHECK(parsePacket(datagram.data(), datagram.size(), header));
        CHECK_EQ(header.packetType, kPacketTypeData);
        CHECK(header.classPresent);
        CHECK_EQ(header.tsi, kTsiNone);
        CHECK_EQ(header.tsf, kTsfSampleCount);
        CHECK_EQ(header.streamId, streamId);
        CHECK_EQ(header.count, count % kPacketCountModulus);
        CHECK_EQ(header.ticks, ticks);
        CHECK_EQ(header.informationClass, kInformationClass);
        CHECK_EQ(header.packetClass, packetClassOf(wire));
        CHECK_EQ(header.endOfBurst, endOfBurst);
        CHECK_EQ(header.words * kWordBytes, total);
        CHECK_EQ(samplesInPayload(wire, header.payloadBytes), samples.size());

        std::vector<Sample> decoded(samples.size());
        unpackSamples(wire, header.payload, 0, samples.size(), decoded.data());
        if (wire == WireFormat::Cs12)
        {
            for (size_t n = 0; n < samples.size(); n++)
            {
                CHECK_EQ(decoded[n].i, samples[n].i);
                CHECK_EQ(decoded[n].q, samples[n].q);
            }
        }
    }
}

static void testHeaderWordLayout()
{
    const uint32_t word = buildHeaderWord(kPacketTypeData, true, false, kTsiNone,
                                          kTsfSampleCount, 5, 368);
    CHECK_EQ((word >> 28) & 0xF, kPacketTypeData);
    CHECK_EQ((word >> 27) & 0x1, 1u);
    CHECK_EQ((word >> 26) & 0x1, 0u);
    CHECK_EQ((word >> 22) & 0x3, 0u);
    CHECK_EQ((word >> 20) & 0x3, 1u);
    CHECK_EQ((word >> 16) & 0xF, 5u);
    CHECK_EQ(word & 0xFFFF, 368u);

    // The packet count wraps at the width of its field.
    CHECK_EQ((buildHeaderWord(kPacketTypeData, true, false, 0, 1, 17, 7) >> 16) & 0xF, 1u);
    // The trailer bit is the end of burst marker.
    CHECK_EQ((buildHeaderWord(kPacketTypeData, true, true, 0, 1, 0, 8) >> 26) & 0x1, 1u);
}

static void testTrailer()
{
    uint8_t trailer[kWordBytes];
    writeTrailer(trailer, true);
    CHECK_EQ(trailer[0], 0u);
    CHECK_EQ(trailer[3], 1u);
    writeTrailer(trailer, false);
    CHECK_EQ(trailer[3], 0u);
}

static void testEmptyHelloPacket()
{
    // The datagram that gives the daemon the host MAC is a header with no payload.
    uint8_t datagram[kVitaHeaderBytes];
    writePacketHeader(datagram, 0xA001, kPacketClassCs12, 0, 0, 0, false);
    PacketHeader header;
    CHECK(parsePacket(datagram, sizeof(datagram), header));
    CHECK_EQ(header.payloadBytes, size_t(0));
    CHECK_EQ(header.words, kVitaHeaderWords);
    CHECK_EQ(header.streamId, 0xA001u);
}

static void testRejectsBadPackets()
{
    uint8_t datagram[kVitaHeaderBytes];
    writePacketHeader(datagram, 1, kPacketClassCs12, 0, 0, 0, false);
    PacketHeader header;
    CHECK(!parsePacket(datagram, kVitaHeaderBytes - 1, header));

    // A packet size word longer than the datagram must be refused.
    std::vector<uint8_t> truncated(datagram, datagram + kVitaHeaderBytes);
    truncated[2] = 0x01;
    truncated[3] = 0x00;
    CHECK(!parsePacket(truncated.data(), truncated.size(), header));

    // The profile always includes a class id.
    std::vector<uint8_t> noClass(datagram, datagram + kVitaHeaderBytes);
    noClass[0] = static_cast<uint8_t>(noClass[0] & ~0x08);
    CHECK(!parsePacket(noClass.data(), noClass.size(), header));
}

static void testCountGap()
{
    CHECK_EQ(countGap(0, 1), 0u);
    CHECK_EQ(countGap(15, 0), 0u);
    CHECK_EQ(countGap(0, 3), 2u);
    CHECK_EQ(countGap(14, 1), 2u);
    CHECK_EQ(countGap(5, 5), 15u);
}

static void testExactPacketSizes()
{
    CHECK(samplesExact(WireFormat::Cs12, 96));
    CHECK(!samplesExact(WireFormat::Cs12, 95));
    CHECK(samplesExact(WireFormat::Cs12, 94));
    CHECK(samplesExact(WireFormat::Cs12, 1));
    CHECK(!samplesExact(WireFormat::Cs8, 95));
    CHECK(samplesExact(WireFormat::Cs8, 96));
    CHECK(samplesExact(WireFormat::Bfp8, 95));

    // osmo-trx writes 2500 samples, which leaves 95 after five full packets.
    CHECK(exactPacketElems(WireFormat::Cs12, 1, 2500, 481) == 481);
    CHECK(exactPacketElems(WireFormat::Cs12, 1, 95, 481) == 94);
    CHECK(exactPacketElems(WireFormat::Cs12, 1, 1, 481) == 1);
    CHECK(exactPacketElems(WireFormat::Cs12, 2, 95, 240) == 95);
    CHECK(exactPacketElems(WireFormat::Cs8, 1, 95, 722) == 94);
    CHECK(exactPacketElems(WireFormat::Cs8, 1, 1, 722) == 0);
    CHECK(exactPacketElems(WireFormat::Cs8, 2, 1, 361) == 1);
}

int main()
{
    const Json doc = PlutoSkyTest::loadVectors();
    testPacketVectors(doc);
    testHeaderWordLayout();
    testTrailer();
    testEmptyHelloPacket();
    testRejectsBadPackets();
    testCountGap();
    testExactPacketSizes();
    return PlutoSkyTest::summarize("test_vita");
}
