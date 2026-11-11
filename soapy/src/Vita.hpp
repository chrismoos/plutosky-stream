// VITA 49.2 signal data packets and the three sample wire formats.
//
// Normative description: docs/wire-formats.md. tools/vita.py is the Python
// twin of this file and tools/vectors/wire_formats.json pins both.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace PlutoSky {

enum class WireFormat { Cs12, Cs8, Bfp8 };

static const size_t kIpv4HeaderBytes = 20;
static const size_t kUdpHeaderBytes = 8;
static const size_t kVitaHeaderWords = 6;
static const size_t kWordBytes = 4;
static const size_t kVitaHeaderBytes = kVitaHeaderWords * kWordBytes;
static const size_t kVitaTrailerBytes = kWordBytes;

// The MTU budget always reserves the optional end-of-burst trailer, so a full
// size packet still fits when a burst ends on it.
static const size_t kVitaOverheadBytes = kVitaHeaderBytes + kVitaTrailerBytes;

static const uint32_t kPacketTypeData = 0x1;
static const uint32_t kPacketTypeContext = 0x4;
static const uint32_t kTsiNone = 0x0;
static const uint32_t kTsfSampleCount = 0x1;

static const uint32_t kInformationClass = 0x0001;
static const uint32_t kClassIdOuiWord = 0x00000000;

static const uint16_t kPacketClassCs12 = 0x0001;
static const uint16_t kPacketClassCs8 = 0x0002;
static const uint16_t kPacketClassBfp8 = 0x0003;

static const uint32_t kPacketCountModulus = 16;
static const uint32_t kMaxPacketWords = 0xFFFF;

static const size_t kCs12SampleBytes = 3;
static const size_t kCs8SampleBytes = 2;
static const size_t kBfp8BlockSamples = 16;
static const size_t kBfp8BlockBytes = 33;

static const uint32_t kTrailerEndOfBurstMask = 0x00000001;

static const int kSampleBits = 12;
static const int kSampleMin = -(1 << (kSampleBits - 1));
static const int kSampleMax = (1 << (kSampleBits - 1)) - 1;
static const int kMantissaBits = 8;
static const int kMantissaMin = -128;
static const int kMantissaMax = 127;

// CF32 uses the 12-bit full scale, so a sample of kSampleFullScale reads 1.0.
static const float kSampleFullScale = 2048.0f;

const char *wireFormatName(WireFormat format);
bool wireFormatFromName(const std::string &name, WireFormat &out);
uint16_t packetClassOf(WireFormat format);
bool wireFormatFromPacketClass(uint16_t packetClass, WireFormat &out);

size_t payloadCapacity(size_t mtu);

// Payload length rounded up to a whole word, unpadded when it already is one.
size_t paddedPayloadBytes(size_t payloadBytes);
size_t payloadBytesFor(WireFormat format, size_t samples);
size_t samplesPerPacket(WireFormat format, size_t mtu);
size_t samplesInPayload(WireFormat format, size_t payloadBytes);
// True when the board decodes a packet of `samples` interleaved samples back to
// the same count. A packet has no count field, only a length padded to a whole
// word, and the board takes the largest count that fits. A cs12 count of 3
// modulo 4 or an odd cs8 count therefore reads one sample long. bfp8 is padded
// to whole blocks by the sender and is always exact.
bool samplesExact(WireFormat format, size_t samples);
// Per-channel elements for the next packet of a write: at most `limit`, cut so
// the board decodes the count exactly. 0 means not even one element goes out
// exactly on its own.
size_t exactPacketElems(WireFormat format, size_t channels, size_t remaining, size_t limit);
size_t packetWords(WireFormat format, size_t samples);

struct Sample {
    int16_t i;
    int16_t q;
};

int bfp8Exponent(int peakMagnitude);
int bfp8Round(int value, int exponent);

void packSamples(WireFormat format, const Sample *samples, size_t count, uint8_t *payload);
void unpackSamples(WireFormat format, const uint8_t *payload, size_t first, size_t count,
                   Sample *samples);

struct PacketHeader {
    uint32_t packetType = 0;
    bool classPresent = false;
    bool trailerPresent = false;
    uint32_t tsi = 0;
    uint32_t tsf = 0;
    uint32_t count = 0;
    uint32_t words = 0;
    uint32_t streamId = 0;
    uint32_t ouiWord = 0;
    uint16_t informationClass = 0;
    uint16_t packetClass = 0;
    uint64_t ticks = 0;
    bool endOfBurst = false;
    const uint8_t *payload = nullptr;
    size_t payloadBytes = 0;
};

uint32_t buildHeaderWord(uint32_t packetType, bool classPresent, bool trailerPresent,
                         uint32_t tsi, uint32_t tsf, uint32_t count, uint32_t words);

// Writes the 24-byte header and returns the byte offset of the payload.
size_t writePacketHeader(uint8_t *out, uint32_t streamId, uint16_t packetClass, uint32_t count,
                         uint64_t ticks, size_t payloadBytes, bool endOfBurst);

size_t packetBytes(size_t payloadBytes, bool endOfBurst);
void writeTrailer(uint8_t *out, bool endOfBurst);

bool parsePacket(const uint8_t *datagram, size_t length, PacketHeader &out);

// Packets missing between two VITA packet counts, modulo the field width.
uint32_t countGap(uint32_t previous, uint32_t current);

}  // namespace PlutoSky
