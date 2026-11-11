#include "Vita.hpp"

#include <cstring>

namespace PlutoSky {

// VITA 49.2 header word 0 fields.
static const int kPacketTypeShift = 28;
static const int kClassPresentShift = 27;
static const int kTrailerPresentShift = 26;
static const int kTsiShift = 22;
static const int kTsfShift = 20;
static const int kPacketCountShift = 16;
static const uint32_t kNibbleMask = 0xF;
static const uint32_t kTwoBitMask = 0x3;
static const uint32_t kHalfWordMask = 0xFFFF;
static const int kHalfWordBits = 16;
static const int kWordBits = 32;

// Byte offsets of the words after word 0.
static const size_t kStreamIdAt = 4;
static const size_t kClassOuiAt = 8;
static const size_t kClassCodesAt = 12;
static const size_t kTimestampHighAt = 16;
static const size_t kTimestampLowAt = 20;

const char *wireFormatName(WireFormat format)
{
    switch (format)
    {
    case WireFormat::Cs12: return "cs12";
    case WireFormat::Cs8: return "cs8";
    case WireFormat::Bfp8: return "bfp8";
    }
    return "";
}

bool wireFormatFromName(const std::string &name, WireFormat &out)
{
    if (name == "cs12") { out = WireFormat::Cs12; return true; }
    if (name == "cs8") { out = WireFormat::Cs8; return true; }
    if (name == "bfp8") { out = WireFormat::Bfp8; return true; }
    return false;
}

uint16_t packetClassOf(WireFormat format)
{
    switch (format)
    {
    case WireFormat::Cs12: return kPacketClassCs12;
    case WireFormat::Cs8: return kPacketClassCs8;
    case WireFormat::Bfp8: return kPacketClassBfp8;
    }
    return 0;
}

bool wireFormatFromPacketClass(uint16_t packetClass, WireFormat &out)
{
    switch (packetClass)
    {
    case kPacketClassCs12: out = WireFormat::Cs12; return true;
    case kPacketClassCs8: out = WireFormat::Cs8; return true;
    case kPacketClassBfp8: out = WireFormat::Bfp8; return true;
    default: return false;
    }
}

size_t payloadCapacity(size_t mtu)
{
    return mtu - kIpv4HeaderBytes - kUdpHeaderBytes - kVitaOverheadBytes;
}

size_t paddedPayloadBytes(size_t payloadBytes)
{
    return (payloadBytes + kWordBytes - 1) / kWordBytes * kWordBytes;
}

size_t payloadBytesFor(WireFormat format, size_t samples)
{
    switch (format)
    {
    case WireFormat::Cs12: return samples * kCs12SampleBytes;
    case WireFormat::Cs8: return samples * kCs8SampleBytes;
    case WireFormat::Bfp8: return samples / kBfp8BlockSamples * kBfp8BlockBytes;
    }
    return 0;
}

size_t samplesPerPacket(WireFormat format, size_t mtu)
{
    const size_t capacity = payloadCapacity(mtu);
    const size_t step = format == WireFormat::Bfp8 ? kBfp8BlockSamples : 1;
    size_t count = 0;
    while (paddedPayloadBytes(payloadBytesFor(format, count + step)) <= capacity) count += step;
    return count;
}

size_t samplesInPayload(WireFormat format, size_t payloadBytes)
{
    switch (format)
    {
    case WireFormat::Cs12: return payloadBytes / kCs12SampleBytes;
    case WireFormat::Cs8: return payloadBytes / kCs8SampleBytes;
    case WireFormat::Bfp8: return payloadBytes / kBfp8BlockBytes * kBfp8BlockSamples;
    }
    return 0;
}

bool samplesExact(WireFormat format, size_t samples)
{
    if (format == WireFormat::Bfp8) return true;
    return samplesInPayload(format, paddedPayloadBytes(payloadBytesFor(format, samples))) == samples;
}

size_t exactPacketElems(WireFormat format, size_t channels, size_t remaining, size_t limit)
{
    for (size_t take = remaining < limit ? remaining : limit; take > 0; take--)
    {
        if (samplesExact(format, take * channels)) return take;
    }
    return 0;
}

size_t packetWords(WireFormat format, size_t samples)
{
    const size_t body = payloadBytesFor(format, samples);
    return kVitaHeaderWords + (body + kWordBytes - 1) / kWordBytes;
}

int bfp8Exponent(int peakMagnitude)
{
    if (peakMagnitude <= 0) return 0;
    int msb = 0;
    for (int value = peakMagnitude; value > 1; value >>= 1) msb++;
    const int exponent = msb - (kMantissaBits - 2);
    return exponent > 0 ? exponent : 0;
}

int bfp8Round(int value, int exponent)
{
    if (exponent == 0) return value;
    const int half = 1 << (exponent - 1);
    if (value >= 0) return (value + half) >> exponent;
    return -((-value + half) >> exponent);
}

static inline int clampMantissa(int value)
{
    if (value < kMantissaMin) return kMantissaMin;
    if (value > kMantissaMax) return kMantissaMax;
    return value;
}

static inline int16_t signExtend(uint32_t value, int bits)
{
    const uint32_t sign = 1u << (bits - 1);
    return static_cast<int16_t>(static_cast<int32_t>(value & (sign - 1))
                                - static_cast<int32_t>(value & sign));
}

static void packCs12(const Sample *samples, size_t count, uint8_t *out)
{
    for (size_t n = 0; n < count; n++)
    {
        const uint32_t iu = static_cast<uint32_t>(samples[n].i) & 0xFFF;
        const uint32_t qu = static_cast<uint32_t>(samples[n].q) & 0xFFF;
        out[n * 3 + 0] = static_cast<uint8_t>((iu >> 4) & 0xFF);
        out[n * 3 + 1] = static_cast<uint8_t>(((iu & 0xF) << 4) | ((qu >> 8) & 0xF));
        out[n * 3 + 2] = static_cast<uint8_t>(qu & 0xFF);
    }
}

static void unpackCs12(const uint8_t *payload, size_t first, size_t count, Sample *out)
{
    for (size_t n = 0; n < count; n++)
    {
        const uint8_t *p = payload + (first + n) * kCs12SampleBytes;
        const uint32_t i = (static_cast<uint32_t>(p[0]) << 4) | (p[1] >> 4);
        const uint32_t q = (static_cast<uint32_t>(p[1] & 0xF) << 8) | p[2];
        out[n].i = signExtend(i, kSampleBits);
        out[n].q = signExtend(q, kSampleBits);
    }
}

static void packCs8(const Sample *samples, size_t count, uint8_t *out)
{
    for (size_t n = 0; n < count; n++)
    {
        out[n * 2 + 0] = static_cast<uint8_t>((samples[n].i >> 4) & 0xFF);
        out[n * 2 + 1] = static_cast<uint8_t>((samples[n].q >> 4) & 0xFF);
    }
}

static void unpackCs8(const uint8_t *payload, size_t first, size_t count, Sample *out)
{
    for (size_t n = 0; n < count; n++)
    {
        const uint8_t *p = payload + (first + n) * kCs8SampleBytes;
        out[n].i = static_cast<int16_t>(signExtend(p[0], kMantissaBits) << 4);
        out[n].q = static_cast<int16_t>(signExtend(p[1], kMantissaBits) << 4);
    }
}

static void packBfp8(const Sample *samples, size_t count, uint8_t *out)
{
    for (size_t base = 0; base < count; base += kBfp8BlockSamples)
    {
        int peak = 0;
        for (size_t n = 0; n < kBfp8BlockSamples; n++)
        {
            const int i = samples[base + n].i;
            const int q = samples[base + n].q;
            const int ai = i < 0 ? -i : i;
            const int aq = q < 0 ? -q : q;
            if (ai > peak) peak = ai;
            if (aq > peak) peak = aq;
        }
        const int exponent = bfp8Exponent(peak);
        uint8_t *block = out + base / kBfp8BlockSamples * kBfp8BlockBytes;
        for (size_t n = 0; n < kBfp8BlockSamples; n++)
        {
            block[n * 2 + 0] =
                static_cast<uint8_t>(clampMantissa(bfp8Round(samples[base + n].i, exponent)));
            block[n * 2 + 1] =
                static_cast<uint8_t>(clampMantissa(bfp8Round(samples[base + n].q, exponent)));
        }
        block[kBfp8BlockBytes - 1] = static_cast<uint8_t>(exponent & 0x0F);
    }
}

static void unpackBfp8(const uint8_t *payload, size_t first, size_t count, Sample *out)
{
    for (size_t n = 0; n < count; n++)
    {
        const size_t index = first + n;
        const uint8_t *block = payload + index / kBfp8BlockSamples * kBfp8BlockBytes;
        const int exponent = block[kBfp8BlockBytes - 1] & 0x0F;
        const size_t slot = index % kBfp8BlockSamples;
        out[n].i = static_cast<int16_t>(signExtend(block[slot * 2 + 0], kMantissaBits) << exponent);
        out[n].q = static_cast<int16_t>(signExtend(block[slot * 2 + 1], kMantissaBits) << exponent);
    }
}

void packSamples(WireFormat format, const Sample *samples, size_t count, uint8_t *payload)
{
    switch (format)
    {
    case WireFormat::Cs12: packCs12(samples, count, payload); break;
    case WireFormat::Cs8: packCs8(samples, count, payload); break;
    case WireFormat::Bfp8: packBfp8(samples, count, payload); break;
    }
}

void unpackSamples(WireFormat format, const uint8_t *payload, size_t first, size_t count,
                   Sample *samples)
{
    switch (format)
    {
    case WireFormat::Cs12: unpackCs12(payload, first, count, samples); break;
    case WireFormat::Cs8: unpackCs8(payload, first, count, samples); break;
    case WireFormat::Bfp8: unpackBfp8(payload, first, count, samples); break;
    }
}

uint32_t buildHeaderWord(uint32_t packetType, bool classPresent, bool trailerPresent,
                         uint32_t tsi, uint32_t tsf, uint32_t count, uint32_t words)
{
    return ((packetType & kNibbleMask) << kPacketTypeShift)
         | (static_cast<uint32_t>(classPresent) << kClassPresentShift)
         | (static_cast<uint32_t>(trailerPresent) << kTrailerPresentShift)
         | ((tsi & kTwoBitMask) << kTsiShift)
         | ((tsf & kTwoBitMask) << kTsfShift)
         | ((count % kPacketCountModulus) << kPacketCountShift)
         | (words & kHalfWordMask);
}

static inline void storeBe32(uint8_t *out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value >> 24);
    out[1] = static_cast<uint8_t>(value >> 16);
    out[2] = static_cast<uint8_t>(value >> 8);
    out[3] = static_cast<uint8_t>(value);
}

static inline uint32_t loadBe32(const uint8_t *in)
{
    return (static_cast<uint32_t>(in[0]) << 24) | (static_cast<uint32_t>(in[1]) << 16)
         | (static_cast<uint32_t>(in[2]) << 8) | static_cast<uint32_t>(in[3]);
}

size_t packetBytes(size_t payloadBytes, bool endOfBurst)
{
    return kVitaHeaderBytes + paddedPayloadBytes(payloadBytes)
         + (endOfBurst ? kVitaTrailerBytes : 0);
}

size_t writePacketHeader(uint8_t *out, uint32_t streamId, uint16_t packetClass, uint32_t count,
                         uint64_t ticks, size_t payloadBytes, bool endOfBurst)
{
    const size_t total = packetBytes(payloadBytes, endOfBurst);
    const uint32_t words = static_cast<uint32_t>(total / kWordBytes);
    storeBe32(out + 0, buildHeaderWord(kPacketTypeData, true, endOfBurst, kTsiNone,
                                       kTsfSampleCount, count, words));
    storeBe32(out + kStreamIdAt, streamId);
    storeBe32(out + kClassOuiAt, kClassIdOuiWord);
    storeBe32(out + kClassCodesAt, (kInformationClass << kHalfWordBits) | packetClass);
    storeBe32(out + kTimestampHighAt, static_cast<uint32_t>(ticks >> kWordBits));
    storeBe32(out + kTimestampLowAt, static_cast<uint32_t>(ticks));
    // The pad bytes between the payload and the trailer must be deterministic.
    const size_t padded = paddedPayloadBytes(payloadBytes);
    if (padded > payloadBytes) std::memset(out + kVitaHeaderBytes + payloadBytes, 0,
                                           padded - payloadBytes);
    return kVitaHeaderBytes;
}

void writeTrailer(uint8_t *out, bool endOfBurst)
{
    storeBe32(out, endOfBurst ? kTrailerEndOfBurstMask : 0);
}

bool parsePacket(const uint8_t *datagram, size_t length, PacketHeader &out)
{
    if (length < kVitaHeaderBytes) return false;
    const uint32_t word0 = loadBe32(datagram);
    out.packetType = (word0 >> kPacketTypeShift) & kNibbleMask;
    out.classPresent = ((word0 >> kClassPresentShift) & 1) != 0;
    out.trailerPresent = ((word0 >> kTrailerPresentShift) & 1) != 0;
    out.tsi = (word0 >> kTsiShift) & kTwoBitMask;
    out.tsf = (word0 >> kTsfShift) & kTwoBitMask;
    out.count = (word0 >> kPacketCountShift) & kNibbleMask;
    out.words = word0 & kHalfWordMask;
    if (!out.classPresent) return false;
    const size_t declared = static_cast<size_t>(out.words) * kWordBytes;
    if (declared < kVitaHeaderBytes || declared > length) return false;
    out.streamId = loadBe32(datagram + kStreamIdAt);
    out.ouiWord = loadBe32(datagram + kClassOuiAt);
    const uint32_t classWord = loadBe32(datagram + kClassCodesAt);
    out.informationClass = static_cast<uint16_t>(classWord >> kHalfWordBits);
    out.packetClass = static_cast<uint16_t>(classWord & kHalfWordMask);
    out.ticks = (static_cast<uint64_t>(loadBe32(datagram + kTimestampHighAt)) << kWordBits)
              | static_cast<uint64_t>(loadBe32(datagram + kTimestampLowAt));
    size_t bodyEnd = declared;
    out.endOfBurst = false;
    if (out.trailerPresent)
    {
        if (declared < kVitaHeaderBytes + kWordBytes) return false;
        bodyEnd -= kWordBytes;
        out.endOfBurst = (loadBe32(datagram + bodyEnd) & kTrailerEndOfBurstMask) != 0;
    }
    out.payload = datagram + kVitaHeaderBytes;
    out.payloadBytes = bodyEnd - kVitaHeaderBytes;
    return true;
}

uint32_t countGap(uint32_t previous, uint32_t current)
{
    return (current - previous - 1 + kPacketCountModulus * 2) % kPacketCountModulus;
}

}  // namespace PlutoSky
