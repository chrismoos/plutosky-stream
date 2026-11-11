// Conversion between the wire payload and the host buffers of a SoapySDR stream.
//
// Everything meets in the 12-bit sample scale of Vita.hpp: CS16 holds the
// raw sign-extended 12-bit value and CF32 that value over kSampleFullScale, so
// a round trip through CS12 is exact. Cs16Msb is CS16 with the 12-bit value in
// the top bits, the full 16-bit scale UHD applications expect of sc16.

#pragma once

#include "Vita.hpp"

#include <cstddef>
#include <string>

namespace PlutoSky {

enum class HostFormat { Cs12, Cs8, Cs16, Cs16Msb, Cf32 };

static const int kCs16MsbShift = 4;

// The cs16_scale argument: "12" selects Cs16 and "16" selects Cs16Msb.
bool cs16FormatFromScale(const std::string &scale, HostFormat &out);
const char *cs16ScaleName(HostFormat format);

bool hostFormatFromName(const std::string &name, HostFormat &out);
const char *hostFormatName(HostFormat format);
size_t hostElementBytes(HostFormat format);
HostFormat nativeHostFormat(WireFormat wire);
bool hostFormatAllowed(WireFormat wire, HostFormat host);

// True when the host buffer holds the wire bytes unchanged, so the payload can
// be copied element by element with no decode.
bool isRawHostFormat(WireFormat wire, HostFormat host);

// Scatters elems per-channel elements, read from interleaved sample firstSample
// of the payload, into the per-channel host buffers at elemOffset.
void payloadToHost(WireFormat wire, HostFormat host, const uint8_t *payload, size_t firstSample,
                   size_t elems, size_t channels, void *const *buffs, size_t elemOffset,
                   Sample *staging);

// Gathers elems per-channel elements from the host buffers into the payload,
// zero-filling padSamples interleaved samples past the end.
void hostToPayload(WireFormat wire, HostFormat host, const void *const *buffs, size_t elemOffset,
                   size_t elems, size_t channels, uint8_t *payload, size_t padSamples,
                   Sample *staging);

}  // namespace PlutoSky
