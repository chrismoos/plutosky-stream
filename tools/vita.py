"""VITA 49.2 packets and sample formats for plutosky-stream.

Reference implementation shared by the fake daemon and the packet tools. The
normative description is docs/wire-formats.md, and the C++ module in soapy/
mirrors this file byte for byte.
"""

import struct

IPV4_HEADER_BYTES = 20
UDP_HEADER_BYTES = 8
VITA_HEADER_WORDS = 6
WORD_BYTES = 4
VITA_HEADER_BYTES = VITA_HEADER_WORDS * WORD_BYTES
VITA_TRAILER_BYTES = WORD_BYTES

# The MTU budget always reserves the optional end-of-burst trailer, so a full
# size packet still fits when a burst ends on it.
VITA_OVERHEAD_BYTES = VITA_HEADER_BYTES + VITA_TRAILER_BYTES

PACKET_TYPE_DATA = 0x1
PACKET_TYPE_CONTEXT = 0x4

TSI_NONE = 0b00
TSF_SAMPLE_COUNT = 0b01

INFORMATION_CLASS = 0x0001
CLASS_ID_OUI_WORD = 0x00000000

PACKET_CLASS_CS12 = 0x0001
PACKET_CLASS_CS8 = 0x0002
PACKET_CLASS_BFP8 = 0x0003

PACKET_COUNT_MODULUS = 16
MAX_PACKET_WORDS = 0xFFFF

CS12_SAMPLE_BYTES = 3
CS8_SAMPLE_BYTES = 2
BFP8_BLOCK_SAMPLES = 16
BFP8_BLOCK_BYTES = 33

TRAILER_END_OF_BURST_MASK = 0x00000001

SAMPLE_BITS = 12
SAMPLE_MIN = -(1 << (SAMPLE_BITS - 1))
SAMPLE_MAX = (1 << (SAMPLE_BITS - 1)) - 1

MANTISSA_MIN = -128
MANTISSA_MAX = 127
MANTISSA_BITS = 8

FORMATS = ("cs12", "cs8", "bfp8")

PACKET_CLASS_BY_FORMAT = {
    "cs12": PACKET_CLASS_CS12,
    "cs8": PACKET_CLASS_CS8,
    "bfp8": PACKET_CLASS_BFP8,
}
FORMAT_BY_PACKET_CLASS = {v: k for k, v in PACKET_CLASS_BY_FORMAT.items()}


class VitaError(Exception):
    pass


def payload_capacity(mtu):
    """Largest VITA payload that fits an IPv4/UDP datagram at this MTU."""
    return mtu - IPV4_HEADER_BYTES - UDP_HEADER_BYTES - VITA_OVERHEAD_BYTES


def padded_payload_bytes(payload_bytes):
    """Payload length rounded up to a whole word, unpadded when it already is one."""
    return -(-payload_bytes // WORD_BYTES) * WORD_BYTES


def payload_bytes_for(fmt, samples):
    if fmt == "cs12":
        return samples * CS12_SAMPLE_BYTES
    if fmt == "cs8":
        return samples * CS8_SAMPLE_BYTES
    if fmt == "bfp8":
        if samples % BFP8_BLOCK_SAMPLES:
            raise VitaError("bfp8 sample count must be a multiple of the block size")
        return samples // BFP8_BLOCK_SAMPLES * BFP8_BLOCK_BYTES
    raise VitaError("unknown format " + str(fmt))


def samples_per_packet(fmt, mtu):
    """Largest sample count whose padded payload still fits the MTU."""
    capacity = payload_capacity(mtu)
    step = BFP8_BLOCK_SAMPLES if fmt == "bfp8" else 1
    count = 0
    while padded_payload_bytes(payload_bytes_for(fmt, count + step)) <= capacity:
        count += step
    return count


def packet_words(fmt, samples):
    """Total VITA packet length in words, header included, payload padded."""
    body = payload_bytes_for(fmt, samples)
    return VITA_HEADER_WORDS + (body + WORD_BYTES - 1) // WORD_BYTES


def _clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def _sign_extend(value, bits):
    sign = 1 << (bits - 1)
    return (value & (sign - 1)) - (value & sign)


def pack_cs12(samples):
    out = bytearray()
    for i, q in samples:
        iu = i & 0xFFF
        qu = q & 0xFFF
        out.append((iu >> 4) & 0xFF)
        out.append(((iu & 0xF) << 4) | ((qu >> 8) & 0xF))
        out.append(qu & 0xFF)
    return bytes(out)


def unpack_cs12(payload, samples):
    if len(payload) < samples * CS12_SAMPLE_BYTES:
        raise VitaError("cs12 payload short")
    out = []
    for n in range(samples):
        b0, b1, b2 = payload[n * 3], payload[n * 3 + 1], payload[n * 3 + 2]
        i = (b0 << 4) | (b1 >> 4)
        q = ((b1 & 0xF) << 8) | b2
        out.append((_sign_extend(i, SAMPLE_BITS), _sign_extend(q, SAMPLE_BITS)))
    return out


def pack_cs8(samples):
    out = bytearray()
    for i, q in samples:
        out.append((i >> 4) & 0xFF)
        out.append((q >> 4) & 0xFF)
    return bytes(out)


def unpack_cs8(payload, samples):
    if len(payload) < samples * CS8_SAMPLE_BYTES:
        raise VitaError("cs8 payload short")
    out = []
    for n in range(samples):
        i = _sign_extend(payload[n * 2], MANTISSA_BITS) << 4
        q = _sign_extend(payload[n * 2 + 1], MANTISSA_BITS) << 4
        out.append((i, q))
    return out


def bfp8_exponent(peak_magnitude):
    """Smallest exponent that brings the block peak into a signed byte."""
    if peak_magnitude <= 0:
        return 0
    msb = peak_magnitude.bit_length() - 1
    return max(0, msb - (MANTISSA_BITS - 2))


def bfp8_round(value, exponent):
    """Divide by 2^exponent rounding halves away from zero."""
    if exponent == 0:
        return value
    half = 1 << (exponent - 1)
    if value >= 0:
        return (value + half) >> exponent
    return -((-value + half) >> exponent)


def pack_bfp8(samples):
    if len(samples) % BFP8_BLOCK_SAMPLES:
        raise VitaError("bfp8 sample count must be a multiple of the block size")
    out = bytearray()
    for base in range(0, len(samples), BFP8_BLOCK_SAMPLES):
        block = samples[base:base + BFP8_BLOCK_SAMPLES]
        peak = 0
        for i, q in block:
            peak = max(peak, abs(i), abs(q))
        exponent = bfp8_exponent(peak)
        for i, q in block:
            for component in (i, q):
                mantissa = _clamp(bfp8_round(component, exponent), MANTISSA_MIN, MANTISSA_MAX)
                out.append(mantissa & 0xFF)
        out.append(exponent & 0x0F)
    return bytes(out)


def unpack_bfp8(payload, samples, first=0):
    """Decodes `samples` complex values from interleaved index `first`.

    Any offset and count work: a sample's block, and so its exponent, follows
    from its index.
    """
    last = first + samples
    if len(payload) < -(-last // BFP8_BLOCK_SAMPLES) * BFP8_BLOCK_BYTES:
        raise VitaError("bfp8 payload short")
    out = []
    for index in range(first, last):
        base = index // BFP8_BLOCK_SAMPLES * BFP8_BLOCK_BYTES
        slot = index % BFP8_BLOCK_SAMPLES
        exponent = payload[base + BFP8_BLOCK_BYTES - 1] & 0x0F
        i = _sign_extend(payload[base + slot * 2], MANTISSA_BITS) << exponent
        q = _sign_extend(payload[base + slot * 2 + 1], MANTISSA_BITS) << exponent
        out.append((i, q))
    return out


def pack(fmt, samples):
    if fmt == "cs12":
        return pack_cs12(samples)
    if fmt == "cs8":
        return pack_cs8(samples)
    if fmt == "bfp8":
        return pack_bfp8(samples)
    raise VitaError("unknown format " + str(fmt))


def unpack(fmt, payload, samples):
    if fmt == "cs12":
        return unpack_cs12(payload, samples)
    if fmt == "cs8":
        return unpack_cs8(payload, samples)
    if fmt == "bfp8":
        return unpack_bfp8(payload, samples)
    raise VitaError("unknown format " + str(fmt))


def samples_in_payload(fmt, payload_len):
    """Sample count in a payload of this length, ignoring pad bytes."""
    if fmt == "cs12":
        return payload_len // CS12_SAMPLE_BYTES
    if fmt == "cs8":
        return payload_len // CS8_SAMPLE_BYTES
    if fmt == "bfp8":
        return payload_len // BFP8_BLOCK_BYTES * BFP8_BLOCK_SAMPLES
    raise VitaError("unknown format " + str(fmt))


def build_header_word(packet_type, class_present, trailer_present, tsi, tsf, count, words):
    if words > MAX_PACKET_WORDS:
        raise VitaError("packet too long")
    return (
        (packet_type & 0xF) << 28
        | (1 if class_present else 0) << 27
        | (1 if trailer_present else 0) << 26
        | (tsi & 0x3) << 22
        | (tsf & 0x3) << 20
        | (count % PACKET_COUNT_MODULUS) << 16
        | (words & 0xFFFF)
    )


def parse_header_word(word):
    return {
        "packet_type": (word >> 28) & 0xF,
        "class_present": bool((word >> 27) & 1),
        "trailer_present": bool((word >> 26) & 1),
        "tsi": (word >> 22) & 0x3,
        "tsf": (word >> 20) & 0x3,
        "count": (word >> 16) & 0xF,
        "words": word & 0xFFFF,
    }


def build_packet(stream_id, packet_class, count, ticks, payload, end_of_burst=False,
                 packet_type=PACKET_TYPE_DATA):
    """Build one VITA signal data packet, padding the payload to a whole word."""
    pad = (-len(payload)) % WORD_BYTES
    body = bytes(payload) + b"\x00" * pad
    words = VITA_HEADER_WORDS + len(body) // WORD_BYTES + (1 if end_of_burst else 0)
    header = build_header_word(
        packet_type, True, end_of_burst, TSI_NONE, TSF_SAMPLE_COUNT, count, words)
    out = struct.pack(
        ">IIIIII",
        header,
        stream_id & 0xFFFFFFFF,
        CLASS_ID_OUI_WORD,
        (INFORMATION_CLASS << 16) | (packet_class & 0xFFFF),
        (ticks >> 32) & 0xFFFFFFFF,
        ticks & 0xFFFFFFFF,
    ) + body
    if end_of_burst:
        out += struct.pack(">I", TRAILER_END_OF_BURST_MASK)
    return out


def parse_packet(datagram):
    if len(datagram) < VITA_HEADER_BYTES:
        raise VitaError("datagram shorter than a VITA header")
    words = struct.unpack(">6I", datagram[:24])
    info = parse_header_word(words[0])
    if not info["class_present"]:
        raise VitaError("class id is required by this profile")
    declared = info["words"] * WORD_BYTES
    if declared > len(datagram):
        raise VitaError("packet size word exceeds the datagram")
    info["stream_id"] = words[1]
    info["oui_word"] = words[2]
    info["information_class"] = (words[3] >> 16) & 0xFFFF
    info["packet_class"] = words[3] & 0xFFFF
    info["format"] = FORMAT_BY_PACKET_CLASS.get(info["packet_class"])
    info["ticks"] = (words[4] << 32) | words[5]
    body_end = declared - (WORD_BYTES if info["trailer_present"] else 0)
    info["payload"] = datagram[VITA_HEADER_BYTES:body_end]
    if info["trailer_present"]:
        trailer = struct.unpack(">I", datagram[body_end:declared])[0]
        info["trailer"] = trailer
        info["end_of_burst"] = bool(trailer & TRAILER_END_OF_BURST_MASK)
    else:
        info["trailer"] = None
        info["end_of_burst"] = False
    return info


def count_gap(previous, current):
    """Packets missing between two VITA packet counts, modulo the field width."""
    return (current - previous - 1) % PACKET_COUNT_MODULUS
