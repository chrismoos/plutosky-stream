#include "Convert.hpp"

#include <cstring>

namespace PlutoSky {

static const char *const kCs16Scale12 = "12";
static const char *const kCs16Scale16 = "16";
// cs8 holds the top eight bits of a 12-bit sample.
static const int kCs8Shift = kSampleBits - kMantissaBits;

bool cs16FormatFromScale(const std::string &scale, HostFormat &out)
{
    if (scale == kCs16Scale12) { out = HostFormat::Cs16; return true; }
    if (scale == kCs16Scale16) { out = HostFormat::Cs16Msb; return true; }
    return false;
}

const char *cs16ScaleName(HostFormat format)
{
    return format == HostFormat::Cs16Msb ? kCs16Scale16 : kCs16Scale12;
}

bool hostFormatFromName(const std::string &name, HostFormat &out)
{
    if (name == "CS12") { out = HostFormat::Cs12; return true; }
    if (name == "CS8") { out = HostFormat::Cs8; return true; }
    if (name == "CS16") { out = HostFormat::Cs16; return true; }
    if (name == "CF32") { out = HostFormat::Cf32; return true; }
    return false;
}

const char *hostFormatName(HostFormat format)
{
    switch (format)
    {
    case HostFormat::Cs12: return "CS12";
    case HostFormat::Cs8: return "CS8";
    case HostFormat::Cs16: return "CS16";
    case HostFormat::Cs16Msb: return "CS16";
    case HostFormat::Cf32: return "CF32";
    }
    return "";
}

size_t hostElementBytes(HostFormat format)
{
    switch (format)
    {
    case HostFormat::Cs12: return kCs12SampleBytes;
    case HostFormat::Cs8: return kCs8SampleBytes;
    case HostFormat::Cs16:
    case HostFormat::Cs16Msb: return 2 * sizeof(int16_t);
    case HostFormat::Cf32: return 2 * sizeof(float);
    }
    return 0;
}

HostFormat nativeHostFormat(WireFormat wire)
{
    switch (wire)
    {
    case WireFormat::Cs12: return HostFormat::Cs12;
    case WireFormat::Cs8: return HostFormat::Cs8;
    case WireFormat::Bfp8: return HostFormat::Cs16;
    }
    return HostFormat::Cs16;
}

bool hostFormatAllowed(WireFormat wire, HostFormat host)
{
    if (host == HostFormat::Cs16 || host == HostFormat::Cs16Msb || host == HostFormat::Cf32)
    {
        return true;
    }
    return host == nativeHostFormat(wire);
}

bool isRawHostFormat(WireFormat wire, HostFormat host)
{
    return (wire == WireFormat::Cs12 && host == HostFormat::Cs12)
        || (wire == WireFormat::Cs8 && host == HostFormat::Cs8);
}

static size_t wireSampleBytes(WireFormat wire)
{
    return wire == WireFormat::Cs12 ? kCs12SampleBytes : kCs8SampleBytes;
}

static void rawToHost(WireFormat wire, const uint8_t *payload, size_t firstSample, size_t elems,
                      size_t channels, void *const *buffs, size_t elemOffset)
{
    const size_t stride = wireSampleBytes(wire);
    for (size_t c = 0; c < channels; c++)
    {
        uint8_t *dst = static_cast<uint8_t *>(buffs[c]) + elemOffset * stride;
        for (size_t n = 0; n < elems; n++)
        {
            std::memcpy(dst + n * stride, payload + (firstSample + n * channels + c) * stride,
                        stride);
        }
    }
}

static void rawFromHost(WireFormat wire, const void *const *buffs, size_t elemOffset, size_t elems,
                        size_t channels, uint8_t *payload)
{
    const size_t stride = wireSampleBytes(wire);
    for (size_t c = 0; c < channels; c++)
    {
        const uint8_t *src = static_cast<const uint8_t *>(buffs[c]) + elemOffset * stride;
        for (size_t n = 0; n < elems; n++)
        {
            std::memcpy(payload + (n * channels + c) * stride, src + n * stride, stride);
        }
    }
}

void payloadToHost(WireFormat wire, HostFormat host, const uint8_t *payload, size_t firstSample,
                   size_t elems, size_t channels, void *const *buffs, size_t elemOffset,
                   Sample *staging)
{
    if (elems == 0) return;
    if (isRawHostFormat(wire, host))
    {
        rawToHost(wire, payload, firstSample, elems, channels, buffs, elemOffset);
        return;
    }

    unpackSamples(wire, payload, firstSample, elems * channels, staging);

    for (size_t c = 0; c < channels; c++)
    {
        if (host == HostFormat::Cs16)
        {
            int16_t *dst = static_cast<int16_t *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                dst[n * 2 + 0] = staging[n * channels + c].i;
                dst[n * 2 + 1] = staging[n * channels + c].q;
            }
        }
        else if (host == HostFormat::Cs16Msb)
        {
            int16_t *dst = static_cast<int16_t *>(buffs[c]) + elemOffset * 2;
            const int scale = 1 << kCs16MsbShift;
            for (size_t n = 0; n < elems; n++)
            {
                dst[n * 2 + 0] = static_cast<int16_t>(staging[n * channels + c].i * scale);
                dst[n * 2 + 1] = static_cast<int16_t>(staging[n * channels + c].q * scale);
            }
        }
        else if (host == HostFormat::Cf32)
        {
            float *dst = static_cast<float *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                dst[n * 2 + 0] = staging[n * channels + c].i / kSampleFullScale;
                dst[n * 2 + 1] = staging[n * channels + c].q / kSampleFullScale;
            }
        }
        else if (host == HostFormat::Cs8)
        {
            int8_t *dst = static_cast<int8_t *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                dst[n * 2 + 0] = static_cast<int8_t>(staging[n * channels + c].i >> kCs8Shift);
                dst[n * 2 + 1] = static_cast<int8_t>(staging[n * channels + c].q >> kCs8Shift);
            }
        }
        else
        {
            uint8_t *dst = static_cast<uint8_t *>(buffs[c]) + elemOffset * kCs12SampleBytes;
            for (size_t n = 0; n < elems; n++)
            {
                packSamples(WireFormat::Cs12, &staging[n * channels + c], 1,
                            dst + n * kCs12SampleBytes);
            }
        }
    }
}

static inline int16_t clampSample(int value)
{
    if (value < kSampleMin) return static_cast<int16_t>(kSampleMin);
    if (value > kSampleMax) return static_cast<int16_t>(kSampleMax);
    return static_cast<int16_t>(value);
}

// Drops the low bits of a full-scale 16-bit value, rounding half up.
static inline int roundFromMsb(int16_t value)
{
    return (value + (1 << (kCs16MsbShift - 1))) >> kCs16MsbShift;
}

void hostToPayload(WireFormat wire, HostFormat host, const void *const *buffs, size_t elemOffset,
                   size_t elems, size_t channels, uint8_t *payload, size_t padSamples,
                   Sample *staging)
{
    if (elems == 0 && padSamples == 0) return;
    if (padSamples == 0 && isRawHostFormat(wire, host))
    {
        rawFromHost(wire, buffs, elemOffset, elems, channels, payload);
        return;
    }

    for (size_t c = 0; c < channels; c++)
    {
        if (host == HostFormat::Cs16)
        {
            const int16_t *src = static_cast<const int16_t *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                staging[n * channels + c].i = clampSample(src[n * 2 + 0]);
                staging[n * channels + c].q = clampSample(src[n * 2 + 1]);
            }
        }
        else if (host == HostFormat::Cs16Msb)
        {
            const int16_t *src = static_cast<const int16_t *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                staging[n * channels + c].i = clampSample(roundFromMsb(src[n * 2 + 0]));
                staging[n * channels + c].q = clampSample(roundFromMsb(src[n * 2 + 1]));
            }
        }
        else if (host == HostFormat::Cf32)
        {
            const float *src = static_cast<const float *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                const float i = src[n * 2 + 0] * kSampleFullScale;
                const float q = src[n * 2 + 1] * kSampleFullScale;
                staging[n * channels + c].i =
                    clampSample(static_cast<int>(i >= 0 ? i + 0.5f : i - 0.5f));
                staging[n * channels + c].q =
                    clampSample(static_cast<int>(q >= 0 ? q + 0.5f : q - 0.5f));
            }
        }
        else if (host == HostFormat::Cs8)
        {
            const int8_t *src = static_cast<const int8_t *>(buffs[c]) + elemOffset * 2;
            for (size_t n = 0; n < elems; n++)
            {
                staging[n * channels + c].i = static_cast<int16_t>(src[n * 2 + 0] << kCs8Shift);
                staging[n * channels + c].q = static_cast<int16_t>(src[n * 2 + 1] << kCs8Shift);
            }
        }
        else
        {
            const uint8_t *src =
                static_cast<const uint8_t *>(buffs[c]) + elemOffset * kCs12SampleBytes;
            for (size_t n = 0; n < elems; n++)
            {
                unpackSamples(WireFormat::Cs12, src + n * kCs12SampleBytes, 0, 1,
                              &staging[n * channels + c]);
            }
        }
    }

    const size_t total = elems * channels;
    for (size_t n = 0; n < padSamples; n++)
    {
        staging[total + n].i = 0;
        staging[total + n].q = 0;
    }
    packSamples(wire, staging, total + padSamples, payload);
}

}  // namespace PlutoSky
