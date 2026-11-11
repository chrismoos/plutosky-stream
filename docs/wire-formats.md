# Wire formats

The board and the host exchange samples as VITA 49.2 signal data packets, one
per UDP/IPv4 datagram. All fields are big-endian.

## Packet layout

| Word | Field | Value |
|---|---|---|
| 0 | Header | Packet type 0001 (data with stream id), C=1, T=1 on end of burst and 0 otherwise, TSI=00, TSF=01, packet count, packet size |
| 1 | Stream id | Assigned by the daemon in `OpenStream` |
| 2 | Class id, OUI | 0 |
| 3 | Class id, codes | Information class 0x0001 in the high half, packet class in the low half |
| 4 | Timestamp, high word | Sample count of the first sample in the payload |
| 5 | Timestamp, low word | |
| 6.. | Payload | Samples in the stream's format, zero-padded to a whole word |
| last | Trailer | End-of-burst packets only: bit 0 set |

Header word 0 fields:

| Bits | Field |
|---|---|
| 31:28 | Packet type, 0001 |
| 27 | C, class id present, always 1 |
| 26 | T, trailer present |
| 23:22 | TSI, 00 (no integer timestamp) |
| 21:20 | TSF, 01 (sample count) |
| 19:16 | Packet count, incremented per stream modulo 16 |
| 15:0 | Packet size in 32-bit words, header and trailer included |

The timestamp is the board timebase: a 64-bit count at the `tick_rate_hz` that
`GetSampleRate` reports. Consecutive samples of a stream are
`rx_ticks_per_sample` or `tx_ticks_per_sample` counts apart. `SetHardwareTime`
sets the timebase.

## Formats

| Packet class | Format | Bytes | Samples per packet |
|---|---|---|---|
| 0x0001 | cs12 | 3 per sample | 481 |
| 0x0002 | cs8 | 2 per sample | 722 |
| 0x0003 | bfp8 | 33 per 16 samples | 688 (43 blocks) |

### cs12

Two 12-bit two's-complement values per sample, I then Q, most significant bit
first, with no padding between samples. Each sample occupies three bytes:

| Byte | Bits |
|---|---|
| 0 | I[11:4] |
| 1 | I[3:0], Q[11:8] |
| 2 | Q[7:0] |

### cs8

One signed byte for I and one for Q: the top 8 bits of each 12-bit value. On
transmit each byte is shifted left by 4.

### bfp8

Blocks of 16 samples in 33 bytes:

| Bytes | Content |
|---|---|
| 0..31 | 16 pairs of signed 8-bit mantissas, I then Q |
| 32 | Exponent in bits 3:0, bits 7:4 zero |

The exponent is the smallest `e` for which every 12-bit component of the block
divided by 2^e fits a signed byte: `e = max(0, msb(max|x|) - 6)`, where `msb`
is the index of the highest set bit of the magnitude. Each mantissa is
`x / 2^e` rounded half away from zero and clamped to [-128, 127]. The decoded
value is `mantissa * 2^e`, saturated to the 12-bit range. `e` is 0 to 4, or 5
for a block that holds -2048.

## Packet size

A full packet holds the largest sample count whose padded payload fits the MTU
less 56 bytes: 20 for IPv4, 8 for UDP, 24 for the VITA header and 4 for the
trailer. The trailer allowance is reserved in every packet, so a full packet
that ends a burst still fits. For bfp8 the count is a whole number of blocks.

| Format | 1500 MTU payload |
|---|---|
| cs12 | 481 samples, 1443 bytes padded to 1444 |
| cs8 | 722 samples, 1444 bytes |
| bfp8 | 43 blocks, 1419 bytes padded to 1420 |

The receiver takes a packet's sample count as the largest count that fits its
padded payload. A short packet therefore must not hold a cs12 count of 3
modulo 4 or an odd cs8 count, which pad to one sample more.

## Two channels

A two-channel stream interleaves the channels by sample: channel 0 sample n,
channel 1 sample n, channel 0 sample n+1. The packet count is rounded down to
an even number so that a packet holds whole sample periods, 480 samples (240
per channel) for cs12. In bfp8 a block holds 8 samples of each
channel under one exponent. The timestamp is the sample count of the first
sample period, the same for both channels.

`OpenStream` returns the packet sample count, and both ends use it.

## Transmit

The host sends packets in the same layout.

- A packet plays when the timebase reaches its timestamp, and each later
  sample `tx_ticks_per_sample` counts after the one before. The packet that
  continues a burst has the timestamp of the last packet plus its sample count
  times `tx_ticks_per_sample`.
- A packet whose timestamp has passed is dropped and counted as late.
- An end-of-burst packet sets T and ends with the trailer. Without it, the next
  packet continues the burst, and a gap in timestamps plays as zeros.
- The host keeps at most `window` packets outstanding, a depth the daemon
  returns in `OpenStream`. `StatusTick` reports how far the board has
  consumed.
