"""Python reference implementation of the wire formats and the VITA framing.

Everything the gateware emits on the network is defined here first and the RTL
is checked against it byte for byte. ``docs/wire-formats.md`` is the normative
text, this file is its executable form.

The VITA header is six words. The "minus 28" in the samples-per-packet rule is
those 24 bytes plus 4 bytes held back for the trailer, which only end-of-burst
packets hold, so a full end-of-burst packet still fits the MTU.

The padding brings a payload up to a whole number of 32-bit words and is empty
when it already is one, which is what lets cs8 fill a 1500 MTU packet exactly
at 722 samples.
"""

ETH_HEADER_BYTES = 14
IP_HEADER_BYTES = 20
UDP_HEADER_BYTES = 8
VITA_HEADER_BYTES = 24
# Reserved in the sizing rule and present only on end-of-burst packets.
VITA_TRAILER_BYTES = 4
FRAME_HEADER_BYTES = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES + VITA_HEADER_BYTES

# Byte offset of VITA word 0 within the Ethernet frame.
VITA_OFFSET = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES
# Byte offset of the 64-bit fractional timestamp (VITA words 4 and 5).
VITA_TSF_OFFSET = VITA_OFFSET + 16

# The gateware's header template, in bytes and in 32-bit words.
HEADER_TEMPLATE_BYTES = 72
HEADER_TEMPLATE_WORDS = HEADER_TEMPLATE_BYTES // 4

FORMAT_CS12 = 0
FORMAT_CS8 = 1
FORMAT_BFP8 = 2

PACKET_CLASS = {FORMAT_CS12: 0x0001, FORMAT_CS8: 0x0002, FORMAT_BFP8: 0x0003}
INFORMATION_CLASS = 0x0001

BFP_BLOCK_SAMPLES = 16
BFP_BLOCK_BYTES = 33

SAMPLE_BITS = 12
SAMPLE_MIN = -(1 << (SAMPLE_BITS - 1))
SAMPLE_MAX = (1 << (SAMPLE_BITS - 1)) - 1


def clamp_sample(value):
    """Saturate to the 12-bit signed range the AD9361 delivers and accepts."""
    return max(SAMPLE_MIN, min(SAMPLE_MAX, value))


def to_twos(value, bits=SAMPLE_BITS):
    return value & ((1 << bits) - 1)


def from_twos(value, bits=SAMPLE_BITS):
    value &= (1 << bits) - 1
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value


def payload_bytes(fmt, samples):
    """Payload length for ``samples`` complex samples, before the tail padding."""
    if fmt == FORMAT_CS12:
        return samples * 3
    if fmt == FORMAT_CS8:
        return samples * 2
    if fmt == FORMAT_BFP8:
        blocks, rem = divmod(samples, BFP_BLOCK_SAMPLES)
        assert rem == 0, "bfp8 packs whole blocks of 16 samples"
        return blocks * BFP_BLOCK_BYTES
    raise ValueError(fmt)


def samples_per_packet(fmt, mtu):
    """Largest sample count whose payload and tail padding fit the UDP payload."""
    avail = mtu - IP_HEADER_BYTES - UDP_HEADER_BYTES - VITA_HEADER_BYTES - VITA_TRAILER_BYTES
    step = BFP_BLOCK_SAMPLES if fmt == FORMAT_BFP8 else 1
    best = 0
    n = step
    while True:
        body = payload_bytes(fmt, n)
        if body + pad_bytes(body) > avail:
            return best
        best = n
        n += step


def pad_bytes(body):
    """Tail padding: up to the next 32-bit word, and nothing on an exact fit."""
    return (4 - body % 4) % 4


# cs12

def pack_cs12(samples):
    """Two 12-bit values per sample, I then Q, most significant bit first."""
    bits = []
    for i, q in samples:
        for value in (i, q):
            v = to_twos(clamp_sample(value))
            bits.extend((v >> b) & 1 for b in reversed(range(SAMPLE_BITS)))
    assert len(bits) % 8 == 0
    out = bytearray()
    for base in range(0, len(bits), 8):
        byte = 0
        for bit in bits[base:base + 8]:
            byte = (byte << 1) | bit
        out.append(byte)
    return bytes(out)


def unpack_cs12(data, samples):
    bits = []
    for byte in data:
        bits.extend((byte >> b) & 1 for b in reversed(range(8)))
    out = []
    pos = 0
    for _ in range(samples):
        values = []
        for _ in range(2):
            v = 0
            for bit in bits[pos:pos + SAMPLE_BITS]:
                v = (v << 1) | bit
            pos += SAMPLE_BITS
            values.append(from_twos(v))
        out.append(tuple(values))
    return out


# cs8

def pack_cs8(samples):
    """The top 8 bits of each 12-bit value, as a signed byte."""
    out = bytearray()
    for i, q in samples:
        for value in (i, q):
            out.append((clamp_sample(value) >> 4) & 0xFF)
    return bytes(out)


def unpack_cs8(data, samples):
    out = []
    for n in range(samples):
        i = from_twos(data[2 * n], 8) << 4
        q = from_twos(data[2 * n + 1], 8) << 4
        out.append((i, q))
    return out


# bfp8

def bfp_exponent(block):
    """Smallest e that brings every component of the block into 8 bits.

    ``e = max(0, msb(max|x|) - 6)``, where ``msb`` is the index of the highest
    set bit of the magnitude. -2048 is the one 12-bit input that reaches e = 5,
    because its magnitude needs twelve bits where +2047 needs eleven.
    """
    acc = 0
    for i, q in block:
        acc |= abs(clamp_sample(i)) | abs(clamp_sample(q))
    msb = acc.bit_length() - 1
    return max(0, msb - 6)


def bfp_mantissa(value, exponent):
    """round_half_away_from_zero(value / 2**e), clamped to a signed byte."""
    value = clamp_sample(value)
    magnitude = abs(value)
    if exponent:
        magnitude = (magnitude + (1 << (exponent - 1))) >> exponent
    if value < 0:
        return max(-128, -magnitude)
    return min(127, magnitude)


def pack_bfp8(samples):
    """Blocks of 16 samples: 32 mantissa bytes then one exponent byte."""
    assert len(samples) % BFP_BLOCK_SAMPLES == 0
    out = bytearray()
    for base in range(0, len(samples), BFP_BLOCK_SAMPLES):
        block = samples[base:base + BFP_BLOCK_SAMPLES]
        e = bfp_exponent(block)
        for i, q in block:
            out.append(bfp_mantissa(i, e) & 0xFF)
            out.append(bfp_mantissa(q, e) & 0xFF)
        out.append(e & 0x0F)
    return bytes(out)


def unpack_bfp8(data, samples):
    """Reconstruct mantissa * 2**e, saturated to the 12-bit sample range.

    The clamp matters for one case the encoder itself produces: +2047 with a
    block peak of -2048 rounds to a mantissa of 64 at e = 5, and 64 * 32 is
    2048, one past the largest positive sample. Saturating keeps the value at
    full scale where truncation would wrap it to negative full scale.
    """
    out = []
    for base in range(0, samples, BFP_BLOCK_SAMPLES):
        off = base // BFP_BLOCK_SAMPLES * BFP_BLOCK_BYTES
        e = data[off + 32] & 0x0F
        for n in range(BFP_BLOCK_SAMPLES):
            i = clamp_sample(from_twos(data[off + 2 * n], 8) << e)
            q = clamp_sample(from_twos(data[off + 2 * n + 1], 8) << e)
            out.append((i, q))
    return out[:samples]


PACKERS = {FORMAT_CS12: pack_cs12, FORMAT_CS8: pack_cs8, FORMAT_BFP8: pack_bfp8}
UNPACKERS = {FORMAT_CS12: unpack_cs12, FORMAT_CS8: unpack_cs8, FORMAT_BFP8: unpack_bfp8}


def pack(fmt, samples):
    return PACKERS[fmt](samples)


def unpack(fmt, data, samples):
    return UNPACKERS[fmt](data, samples)


def interleave(channels):
    """Interleave per-channel sample lists: ch0 n, ch1 n, ch0 n+1, ..."""
    if len(channels) == 1:
        return list(channels[0])
    out = []
    for n in range(len(channels[0])):
        for chan in channels:
            out.append(chan[n])
    return out


def deinterleave(samples, count):
    if count == 1:
        return [list(samples)]
    return [list(samples[c::count]) for c in range(count)]


# VITA framing

def vita_header_word(packet_count, size_words, end_of_burst=False):
    """VITA word 0: data-with-stream-id, class id present, TSF = sample count."""
    word = 0b0001 << 28          # packet type: signal data with stream id
    word |= 1 << 27              # C: class id present
    if end_of_burst:
        word |= 1 << 26          # T: trailer present
    word |= 0b00 << 22           # TSI: no integer seconds
    word |= 0b01 << 20           # TSF: sample count
    word |= (packet_count & 0xF) << 16
    word |= size_words & 0xFFFF
    return word


def build_frame(template, fmt, samples, packet_count, tick, frame_len,
                end_of_burst=False):
    """The exact Ethernet frame the packetizer writes into a slot.

    ``template`` is the 72-byte CSR template, of which the frame uses the first
    ``FRAME_HEADER_BYTES``. The gateware copies those, ORs the packet count and
    packet size into VITA word 0, writes the 64-bit sample count over VITA words
    4 and 5, appends the packed payload and zero-fills to ``frame_len``. An
    end-of-burst packet ends in a trailer word with bit 0 set, which the caller
    accounts for in ``frame_len``.
    """
    frame = bytearray(template[:FRAME_HEADER_BYTES])
    body = pack(fmt, samples)
    size_words = frame_len_to_size_words(frame_len)
    base = int.from_bytes(frame[VITA_OFFSET:VITA_OFFSET + 4], "big")
    base |= (packet_count & 0xF) << 16
    base |= size_words & 0xFFFF
    if end_of_burst:
        base |= 1 << 26
    frame[VITA_OFFSET:VITA_OFFSET + 4] = base.to_bytes(4, "big")
    frame[VITA_TSF_OFFSET:VITA_TSF_OFFSET + 8] = (tick & ((1 << 64) - 1)).to_bytes(8, "big")
    frame += body
    if end_of_burst:
        frame += bytes(frame_len - VITA_TRAILER_BYTES - len(frame))
        frame += (1).to_bytes(VITA_TRAILER_BYTES, "big")
    assert len(frame) <= frame_len, (len(frame), frame_len)
    frame += bytes(frame_len - len(frame))
    return bytes(frame)


def frame_len_to_size_words(frame_len):
    """VITA packet size in 32-bit words: everything from VITA word 0 onwards."""
    return (frame_len - VITA_OFFSET) // 4


def frame_len_for(fmt, samples, end_of_burst=False):
    body = payload_bytes(fmt, samples)
    trailer = VITA_TRAILER_BYTES if end_of_burst else 0
    return FRAME_HEADER_BYTES + body + pad_bytes(body) + trailer


def make_template(dst_mac=b"\x02\x00\x00\x00\x00\x01", src_mac=b"\x02\x00\x00\x00\x00\x02",
                  src_ip="192.168.2.1", dst_ip="192.168.2.10", src_port=30002,
                  dst_port=30010, stream_id=0x11223344, fmt=FORMAT_CS12,
                  frame_len=None):
    """A plausible 72-byte template, used by the tests to stand in for the daemon."""
    assert frame_len is not None
    tpl = bytearray(HEADER_TEMPLATE_BYTES)
    tpl[0:6] = dst_mac
    tpl[6:12] = src_mac
    tpl[12:14] = (0x0800).to_bytes(2, "big")

    ip_len = frame_len - ETH_HEADER_BYTES
    ip = bytearray(IP_HEADER_BYTES)
    ip[0] = 0x45
    ip[2:4] = ip_len.to_bytes(2, "big")
    ip[8] = 64
    ip[9] = 17
    ip[12:16] = bytes(int(p) for p in src_ip.split("."))
    ip[16:20] = bytes(int(p) for p in dst_ip.split("."))
    ip[10:12] = ip_checksum(ip).to_bytes(2, "big")
    tpl[ETH_HEADER_BYTES:ETH_HEADER_BYTES + IP_HEADER_BYTES] = ip

    udp_off = ETH_HEADER_BYTES + IP_HEADER_BYTES
    tpl[udp_off:udp_off + 2] = src_port.to_bytes(2, "big")
    tpl[udp_off + 2:udp_off + 4] = dst_port.to_bytes(2, "big")
    tpl[udp_off + 4:udp_off + 6] = (ip_len - IP_HEADER_BYTES).to_bytes(2, "big")

    # VITA word 0 holds the constant type bits, the packetizer ORs in the
    # packet count and the packet size.
    tpl[VITA_OFFSET:VITA_OFFSET + 4] = vita_header_word(0, 0).to_bytes(4, "big")
    tpl[VITA_OFFSET + 4:VITA_OFFSET + 8] = stream_id.to_bytes(4, "big")
    tpl[VITA_OFFSET + 8:VITA_OFFSET + 12] = (0).to_bytes(4, "big")
    tpl[VITA_OFFSET + 12:VITA_OFFSET + 16] = (
        (INFORMATION_CLASS << 16) | PACKET_CLASS[fmt]).to_bytes(4, "big")
    return bytes(tpl)


def ip_checksum(header):
    total = 0
    for off in range(0, len(header), 2):
        total += int.from_bytes(header[off:off + 2], "big")
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def template_words(template):
    """The template as the 18 words the daemon writes, lowest address first."""
    return [int.from_bytes(template[4 * n:4 * n + 4], "big")
            for n in range(HEADER_TEMPLATE_WORDS)]
