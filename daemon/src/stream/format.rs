//! Wire formats and the frame arithmetic both ends of the link repeat.
//!
//! Every number here comes from `docs/wire-formats.md`. The daemon reports
//! `samples_per_packet` when a stream opens and the host recomputes it, so the
//! two must agree exactly.

use std::fmt;

use thiserror::Error;

pub const ETHERNET_HEADER_BYTES: usize = 14;
pub const WORD_BYTES: usize = 4;
const CS12_SAMPLE_BYTES: usize = 3;
const CS8_SAMPLE_BYTES: usize = 2;
pub const IPV4_HEADER_BYTES: usize = 20;
pub const UDP_HEADER_BYTES: usize = 8;
/// VITA header bytes: the packet header, the stream id, the two class id
/// words and the 64-bit sample-count timestamp.
pub const VITA_HEADER_BYTES: usize = 24;
/// Words of VITA header in the frame template.
pub const VITA_HEADER_WORDS: usize = VITA_HEADER_BYTES / WORD_BYTES;
/// The trailer an end-of-burst packet appends. Sizing reserves it in every
/// packet so a full one with its trailer still fits the MTU.
pub const VITA_TRAILER_BYTES: usize = 4;
/// VITA bytes the samples-per-packet arithmetic sets aside per packet.
pub const VITA_SIZING_OVERHEAD_BYTES: usize = VITA_HEADER_BYTES + VITA_TRAILER_BYTES;

/// Fixed bytes ahead of the first payload byte of a frame, counted from the
/// destination MAC. The packetizer never sets end of burst, so its frames
/// have no trailer and this is the whole header.
pub const FRAME_HEADER_BYTES: usize =
    ETHERNET_HEADER_BYTES + IPV4_HEADER_BYTES + UDP_HEADER_BYTES + VITA_HEADER_BYTES;

/// Complex samples in one block-floating-point block.
pub const BFP_BLOCK_SAMPLES: usize = 16;
/// Bytes one block-floating-point block occupies: 32 mantissa bytes and one
/// exponent byte.
pub const BFP_BLOCK_BYTES: usize = 33;

/// VITA information class the profile uses for every signal data packet.
pub const VITA_INFORMATION_CLASS: u16 = 0x0001;

#[derive(Debug, Error, PartialEq, Eq)]
pub enum FormatError {
    #[error("unknown wire format `{0}`, expected cs12, cs8 or bfp8")]
    UnknownFormat(String),
    #[error("unsupported MTU {0}, expected 1500 or 9000")]
    UnsupportedMtu(usize),
}

/// How complex samples are packed into a VITA payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireFormat {
    /// Two 12-bit values per sample, packed with no padding.
    Cs12,
    /// One signed byte per component.
    Cs8,
    /// Blocks of 16 samples with 8-bit mantissas and a shared 4-bit exponent.
    Bfp8,
}

/// Every format, in the order `docs/control-protocol.md` advertises them.
pub const WIRE_FORMATS: [WireFormat; 3] = [WireFormat::Cs12, WireFormat::Cs8, WireFormat::Bfp8];

impl WireFormat {
    pub fn as_str(self) -> &'static str {
        match self {
            WireFormat::Cs12 => "cs12",
            WireFormat::Cs8 => "cs8",
            WireFormat::Bfp8 => "bfp8",
        }
    }

    pub fn parse(name: &str) -> Result<Self, FormatError> {
        match name {
            "cs12" => Ok(WireFormat::Cs12),
            "cs8" => Ok(WireFormat::Cs8),
            "bfp8" => Ok(WireFormat::Bfp8),
            other => Err(FormatError::UnknownFormat(other.to_string())),
        }
    }

    /// VITA packet class, the second half of the class id word.
    pub fn packet_class(self) -> u16 {
        match self {
            WireFormat::Cs12 => 0x0001,
            WireFormat::Cs8 => 0x0002,
            WireFormat::Bfp8 => 0x0003,
        }
    }

    /// Value of the packetizer and depacketizer `ctrl` format field.
    pub fn ctrl_code(self) -> u32 {
        match self {
            WireFormat::Cs12 => 0,
            WireFormat::Cs8 => 1,
            WireFormat::Bfp8 => 2,
        }
    }

    /// Payload bytes `samples` complex samples occupy, before word padding.
    pub fn payload_bytes(self, samples: usize) -> usize {
        match self {
            WireFormat::Cs12 => samples * CS12_SAMPLE_BYTES,
            WireFormat::Cs8 => samples * CS8_SAMPLE_BYTES,
            WireFormat::Bfp8 => samples.div_ceil(BFP_BLOCK_SAMPLES) * BFP_BLOCK_BYTES,
        }
    }

    /// Complex samples in a payload of `bytes`, ignoring word padding.
    pub fn samples_in_payload(self, bytes: usize) -> usize {
        match self {
            WireFormat::Cs12 => bytes / CS12_SAMPLE_BYTES,
            WireFormat::Cs8 => bytes / CS8_SAMPLE_BYTES,
            WireFormat::Bfp8 => (bytes / BFP_BLOCK_BYTES) * BFP_BLOCK_SAMPLES,
        }
    }

    /// How many samples one more step of the format adds: a sample for the
    /// fixed-width formats, a whole block for block floating point.
    fn sample_step(self) -> usize {
        match self {
            WireFormat::Cs12 | WireFormat::Cs8 => 1,
            WireFormat::Bfp8 => BFP_BLOCK_SAMPLES,
        }
    }
}

impl fmt::Display for WireFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The two frame sizes the profile negotiates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mtu {
    Standard,
    Jumbo,
}

/// Every MTU, in the order `docs/control-protocol.md` advertises them.
pub const MTUS: [Mtu; 2] = [Mtu::Standard, Mtu::Jumbo];

impl Mtu {
    pub const STANDARD_BYTES: usize = 1500;
    pub const JUMBO_BYTES: usize = 9000;

    pub fn bytes(self) -> usize {
        match self {
            Mtu::Standard => Self::STANDARD_BYTES,
            Mtu::Jumbo => Self::JUMBO_BYTES,
        }
    }

    pub fn parse(bytes: usize) -> Result<Self, FormatError> {
        match bytes {
            Self::STANDARD_BYTES => Ok(Mtu::Standard),
            Self::JUMBO_BYTES => Ok(Mtu::Jumbo),
            other => Err(FormatError::UnsupportedMtu(other)),
        }
    }

    /// Bytes left for the VITA payload once the IPv4 and UDP headers, the VITA
    /// header and the reserved trailer are accounted for: MTU minus 20 minus 8
    /// minus 28.
    pub fn payload_budget_bytes(self) -> usize {
        self.bytes() - IPV4_HEADER_BYTES - UDP_HEADER_BYTES - VITA_SIZING_OVERHEAD_BYTES
    }
}

/// Everything about a packet that follows from the format and the MTU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketPlan {
    pub format: WireFormat,
    pub mtu: Mtu,
    /// Complex samples in a full packet, interleaved across the stream's
    /// channels.
    pub samples_per_packet: usize,
    /// Payload bytes those samples occupy, rounded up to a whole 32-bit word.
    pub payload_bytes: usize,
    /// Total Ethernet frame bytes excluding the FCS, the value the
    /// packetizer's `frame_len` register takes.
    pub frame_len: usize,
    /// VITA packet size in 32-bit words, header included.
    pub packet_words: usize,
}

impl PacketPlan {
    /// The largest packet that fits the MTU.
    ///
    /// The payload is padded up to a whole 32-bit word, and only when it is
    /// not one already, and what fits the budget fits. A cs8 payload that
    /// lands exactly on the budget therefore counts, which is where this
    /// differs from requiring a pad byte.
    pub fn new(format: WireFormat, mtu: Mtu) -> Self {
        Self::for_channels(format, mtu, 1)
    }

    /// The largest packet that fits the MTU and holds whole sample periods of
    /// `channels` interleaved channels.
    ///
    /// The packetizer fills the last sample period of a two-channel packet
    /// with channel 0 alone when the count is odd, which drops one sample per
    /// channel from every packet while the timestamps still advance.
    pub fn for_channels(format: WireFormat, mtu: Mtu, channels: usize) -> Self {
        let budget = mtu.payload_budget_bytes();
        let step = lcm(format.sample_step(), channels.max(1));
        let mut samples = 0;
        while format
            .payload_bytes(samples + step)
            .next_multiple_of(WORD_BYTES)
            <= budget
        {
            samples += step;
        }
        let payload_bytes = format.payload_bytes(samples).next_multiple_of(WORD_BYTES);
        Self {
            format,
            mtu,
            samples_per_packet: samples,
            payload_bytes,
            frame_len: FRAME_HEADER_BYTES + payload_bytes,
            packet_words: (VITA_HEADER_BYTES + payload_bytes) / WORD_BYTES,
        }
    }
}

fn lcm(a: usize, b: usize) -> usize {
    let (mut x, mut y) = (a, b);
    while y != 0 {
        (x, y) = (y, x % y);
    }
    a / x * b
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Samples per packet for every format and MTU.
    const SAMPLES_PER_PACKET: [(WireFormat, Mtu, usize); 6] = [
        (WireFormat::Cs12, Mtu::Standard, 481),
        (WireFormat::Cs8, Mtu::Standard, 722),
        (WireFormat::Bfp8, Mtu::Standard, 688),
        (WireFormat::Cs12, Mtu::Jumbo, 2981),
        (WireFormat::Cs8, Mtu::Jumbo, 4472),
        (WireFormat::Bfp8, Mtu::Jumbo, 4336),
    ];

    #[test]
    fn samples_per_packet_matches_the_documented_table() {
        for (format, mtu, expected) in SAMPLES_PER_PACKET {
            let plan = PacketPlan::new(format, mtu);
            assert_eq!(
                plan.samples_per_packet,
                expected,
                "{format} at MTU {}",
                mtu.bytes()
            );
        }
    }

    #[test]
    fn one_more_sample_would_not_fit_and_the_padded_payload_does() {
        for (format, mtu, _) in SAMPLES_PER_PACKET {
            let plan = PacketPlan::new(format, mtu);
            let budget = mtu.payload_budget_bytes();
            assert!(
                plan.payload_bytes <= budget,
                "{format} at MTU {}: padded payload {} exceeds {budget}",
                mtu.bytes(),
                plan.payload_bytes
            );
            let next = plan.samples_per_packet + format.sample_step();
            assert!(
                format.payload_bytes(next).next_multiple_of(WORD_BYTES) > budget,
                "{format} at MTU {}: {next} samples would also have fit",
                mtu.bytes()
            );
        }
    }

    #[test]
    fn the_ip_packet_never_exceeds_the_mtu() {
        for (format, mtu, _) in SAMPLES_PER_PACKET {
            let plan = PacketPlan::new(format, mtu);
            assert_eq!(
                plan.frame_len,
                ETHERNET_HEADER_BYTES + plan.packet_ip_bytes()
            );
            assert!(
                plan.packet_ip_bytes() <= mtu.bytes(),
                "{format} at MTU {}: IP packet is {} bytes",
                mtu.bytes(),
                plan.packet_ip_bytes()
            );
        }
    }

    #[test]
    fn two_channel_packets_hold_whole_sample_periods() {
        for (format, mtu, single) in SAMPLES_PER_PACKET {
            let plan = PacketPlan::for_channels(format, mtu, 2);
            assert_eq!(
                plan.samples_per_packet % 2,
                0,
                "{format} at MTU {}",
                mtu.bytes()
            );
            assert_eq!(plan.samples_per_packet, single - single % 2);
        }
        assert_eq!(
            PacketPlan::for_channels(WireFormat::Cs12, Mtu::Standard, 2).samples_per_packet,
            480
        );
    }

    #[test]
    fn frame_lengths_and_packet_sizes_match_the_documented_overheads() {
        assert_eq!(FRAME_HEADER_BYTES, 66);
        let standard = PacketPlan::new(WireFormat::Cs12, Mtu::Standard);
        assert_eq!(standard.payload_bytes, 1444);
        assert_eq!(standard.frame_len, 1510);
        assert_eq!(standard.packet_words, (24 + 1444) / 4);
        let jumbo = PacketPlan::new(WireFormat::Cs8, Mtu::Jumbo);
        assert_eq!(jumbo.payload_bytes, 8944);
        assert_eq!(jumbo.frame_len, 9010);
    }

    #[test]
    fn a_full_end_of_burst_packet_with_its_trailer_fills_the_udp_payload() {
        // The UDP payload of a full packet at each MTU.
        for (mtu, expected) in [(Mtu::Standard, 1472), (Mtu::Jumbo, 8972)] {
            let plan = PacketPlan::new(WireFormat::Cs12, mtu);
            assert_eq!(
                VITA_HEADER_BYTES + plan.payload_bytes + VITA_TRAILER_BYTES,
                expected
            );
        }
    }

    #[test]
    fn a_cs8_payload_that_lands_on_the_budget_still_counts() {
        // cs8 is the format whose payload can land exactly on the budget, so
        // it is where the padding rule decides the count.
        for mtu in MTUS {
            let plan = PacketPlan::new(WireFormat::Cs8, mtu);
            let budget = mtu.payload_budget_bytes();
            assert_eq!(
                WireFormat::Cs8.payload_bytes(plan.samples_per_packet),
                budget
            );
            assert_eq!(plan.payload_bytes, budget, "no padding is added");
        }
    }

    #[test]
    fn block_floating_point_packets_hold_whole_blocks() {
        for mtu in MTUS {
            let plan = PacketPlan::new(WireFormat::Bfp8, mtu);
            assert_eq!(plan.samples_per_packet % BFP_BLOCK_SAMPLES, 0);
            let blocks = plan.samples_per_packet / BFP_BLOCK_SAMPLES;
            assert_eq!(
                WireFormat::Bfp8.payload_bytes(plan.samples_per_packet),
                blocks * BFP_BLOCK_BYTES
            );
        }
        assert_eq!(
            PacketPlan::new(WireFormat::Bfp8, Mtu::Standard).samples_per_packet / BFP_BLOCK_SAMPLES,
            43
        );
        assert_eq!(
            PacketPlan::new(WireFormat::Bfp8, Mtu::Jumbo).samples_per_packet / BFP_BLOCK_SAMPLES,
            271
        );
    }

    #[test]
    fn formats_and_mtus_round_trip_through_their_protocol_names() {
        for format in WIRE_FORMATS {
            assert_eq!(WireFormat::parse(format.as_str()), Ok(format));
        }
        for mtu in MTUS {
            assert_eq!(Mtu::parse(mtu.bytes()), Ok(mtu));
        }
        assert!(WireFormat::parse("cf32").is_err());
        assert!(Mtu::parse(4000).is_err());
    }

    impl PacketPlan {
        fn packet_ip_bytes(&self) -> usize {
            self.frame_len - ETHERNET_HEADER_BYTES
        }
    }
}
