//! The frame template the packetizer prepends to every packet it builds.
//!
//! The template is everything that does not change packet to packet: the
//! Ethernet, IPv4 and UDP headers with the IP checksum already computed for
//! the fixed frame length, and the whole 24-byte VITA header. The packetizer
//! overwrites the VITA packet count nibble and the two timestamp words as it
//! emits each frame.
//!
//! The header is 66 bytes and `docs/csr-contract.md` sizes the template
//! register at 72 bytes, 18 words, so the last 6 bytes go out as zero. The IP
//! lengths cover the packet the packetizer actually sends: it never sets end
//! of burst, so its frames have no trailer even though the sizing arithmetic
//! reserves room for one.

use std::net::Ipv4Addr;

use crate::net::MacAddr;

use super::format::{
    ETHERNET_HEADER_BYTES, FRAME_HEADER_BYTES, IPV4_HEADER_BYTES, PacketPlan, UDP_HEADER_BYTES,
    VITA_HEADER_BYTES, VITA_INFORMATION_CLASS, VITA_TRAILER_BYTES, WireFormat,
};

/// Bytes of the template register, as `docs/csr-contract.md` sizes it.
pub const HEADER_TEMPLATE_BYTES: usize = 72;
/// Words of the template register.
pub const HEADER_TEMPLATE_WORDS: usize = HEADER_TEMPLATE_BYTES / 4;

const ETHERTYPE_IPV4: u16 = 0x0800;

/// IPv4 version 4, header length 5 words.
const IPV4_VERSION_IHL: u8 = 0x45;
/// The packetizer emits one fixed-length frame per packet and cannot
/// fragment, so the frames go out marked Don't Fragment.
const IPV4_FLAGS_DONT_FRAGMENT: u16 = 0x4000;
const IPV4_TTL: u8 = 64;
const IPV4_PROTOCOL_UDP: u8 = 17;
/// UDP checksums are optional over IPv4 and the PL does not compute one.
const UDP_CHECKSUM_NONE: u16 = 0;

/// Byte offsets inside the frame header.
const IPV4_AT: usize = ETHERNET_HEADER_BYTES;
const UDP_AT: usize = IPV4_AT + IPV4_HEADER_BYTES;
const VITA_AT: usize = UDP_AT + UDP_HEADER_BYTES;

/// VITA packet header word fields.
const VITA_PACKET_TYPE_DATA_WITH_STREAM_ID: u32 = 0b0001;
const VITA_PACKET_TYPE_SHIFT: u32 = 28;
/// C: a class id follows the stream id.
const VITA_CLASS_ID_PRESENT: u32 = 1 << 27;
/// TSF = 01: the fractional timestamp is a 64-bit sample count.
const VITA_TSF_SAMPLE_COUNT: u32 = 0b01 << 20;
/// T: a one-word trailer follows the payload.
const VITA_TRAILER_PRESENT: u32 = 1 << 26;
/// TSI and TSF field mask in the header word.
const VITA_TIMESTAMP_FIELDS_MASK: u32 = 0b1111 << 20;
/// Packet size field, in 32-bit words including the header.
const VITA_PACKET_SIZE_MASK: u32 = 0xffff;
/// Offset of the VITA header inside a frame, for readers of received frames.
pub const VITA_HEADER_AT: usize = VITA_AT;

/// Class id OUI word. The reserved top byte is zero and the OUI stays zero
/// until the project has one of its own.
const VITA_CLASS_ID_OUI: u32 = 0x0000_0000;

/// What the packetizer needs to address a host and label its packets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameEndpoints {
    pub board_mac: MacAddr,
    pub host_mac: MacAddr,
    pub board_ip: Ipv4Addr,
    pub host_ip: Ipv4Addr,
    pub board_port: u16,
    pub host_port: u16,
}

/// Builds the 66-byte frame header for a stream.
pub fn frame_header(
    endpoints: &FrameEndpoints,
    plan: &PacketPlan,
    stream_id: u32,
) -> [u8; FRAME_HEADER_BYTES] {
    let mut header = [0u8; FRAME_HEADER_BYTES];
    header[..6].copy_from_slice(&endpoints.host_mac.octets());
    header[6..12].copy_from_slice(&endpoints.board_mac.octets());
    header[12..14].copy_from_slice(&ETHERTYPE_IPV4.to_be_bytes());

    let total_length = (plan.frame_len - ETHERNET_HEADER_BYTES) as u16;
    let ip = &mut header[IPV4_AT..IPV4_AT + IPV4_HEADER_BYTES];
    ip[0] = IPV4_VERSION_IHL;
    ip[2..4].copy_from_slice(&total_length.to_be_bytes());
    ip[6..8].copy_from_slice(&IPV4_FLAGS_DONT_FRAGMENT.to_be_bytes());
    ip[8] = IPV4_TTL;
    ip[9] = IPV4_PROTOCOL_UDP;
    ip[12..16].copy_from_slice(&endpoints.board_ip.octets());
    ip[16..20].copy_from_slice(&endpoints.host_ip.octets());
    let checksum = ones_complement_checksum(ip);
    ip[10..12].copy_from_slice(&checksum.to_be_bytes());

    let udp_length = (total_length as usize - IPV4_HEADER_BYTES) as u16;
    let udp = &mut header[UDP_AT..UDP_AT + UDP_HEADER_BYTES];
    udp[0..2].copy_from_slice(&endpoints.board_port.to_be_bytes());
    udp[2..4].copy_from_slice(&endpoints.host_port.to_be_bytes());
    udp[4..6].copy_from_slice(&udp_length.to_be_bytes());
    udp[6..8].copy_from_slice(&UDP_CHECKSUM_NONE.to_be_bytes());

    let vita = &mut header[VITA_AT..VITA_AT + VITA_HEADER_BYTES];
    vita[0..4].copy_from_slice(&vita_header_word(plan).to_be_bytes());
    vita[4..8].copy_from_slice(&stream_id.to_be_bytes());
    vita[8..12].copy_from_slice(&VITA_CLASS_ID_OUI.to_be_bytes());
    vita[12..16].copy_from_slice(&class_id_word(plan.format).to_be_bytes());
    header
}

/// Packs the frame header into the words of the `header` template register,
/// zero-filling the bytes past the 66-byte header.
pub fn header_template_words(header: &[u8; FRAME_HEADER_BYTES]) -> [u32; HEADER_TEMPLATE_WORDS] {
    let mut bytes = [0u8; HEADER_TEMPLATE_BYTES];
    bytes[..FRAME_HEADER_BYTES].copy_from_slice(header);
    let mut words = [0u32; HEADER_TEMPLATE_WORDS];
    let (chunks, _) = bytes.as_chunks::<4>();
    for (word, chunk) in words.iter_mut().zip(chunks) {
        *word = u32::from_be_bytes(*chunk);
    }
    words
}

/// VITA word 0: a signal data packet with a stream id, a class id, no integer
/// seconds and a 64-bit sample-count fractional timestamp. The packet count
/// nibble stays zero for the packetizer to fill.
fn vita_header_word(plan: &PacketPlan) -> u32 {
    (VITA_PACKET_TYPE_DATA_WITH_STREAM_ID << VITA_PACKET_TYPE_SHIFT)
        | VITA_CLASS_ID_PRESENT
        | VITA_TSF_SAMPLE_COUNT
        | plan.packet_words as u32
}

/// VITA word 3: the information class and the format's packet class.
fn class_id_word(format: WireFormat) -> u32 {
    (u32::from(VITA_INFORMATION_CLASS) << 16) | u32::from(format.packet_class())
}

/// The 16-bit one's complement checksum of a header whose checksum field is
/// already zero.
fn ones_complement_checksum(header: &[u8]) -> u16 {
    let mut sum = 0u32;
    for pair in header.chunks(2) {
        let word = match pair {
            [hi, lo] => u16::from_be_bytes([*hi, *lo]),
            [hi] => u16::from_be_bytes([*hi, 0]),
            _ => unreachable!("chunks(2) yields one or two bytes"),
        };
        sum += u32::from(word);
    }
    while sum >> 16 != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::stream::format::{MTUS, Mtu, WIRE_FORMATS};

    fn endpoints() -> FrameEndpoints {
        FrameEndpoints {
            board_mac: MacAddr([0x00, 0x0a, 0x35, 0x00, 0x11, 0x22]),
            host_mac: MacAddr([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01]),
            board_ip: Ipv4Addr::new(192, 168, 2, 1),
            host_ip: Ipv4Addr::new(192, 168, 2, 10),
            board_port: 30100,
            host_port: 40000,
        }
    }

    #[test]
    fn the_template_holds_the_header_and_zero_fills_the_rest_of_the_register() {
        assert_eq!(FRAME_HEADER_BYTES, 66);
        assert_eq!(HEADER_TEMPLATE_WORDS, 18);
        let plan = PacketPlan::new(WireFormat::Cs12, Mtu::Standard);
        let header = frame_header(&endpoints(), &plan, 7);
        let words = header_template_words(&header);
        assert_eq!(words[0], 0xaabb_ccdd, "destination MAC first");
        assert_eq!(words[1], 0xee01_000a);
        assert_eq!(words[16] & 0x0000_ffff, 0, "past the header");
        assert_eq!(words[17], 0);
    }

    #[test]
    fn the_ip_checksum_makes_the_header_sum_to_zero() {
        let plan = PacketPlan::new(WireFormat::Bfp8, Mtu::Jumbo);
        let header = frame_header(&endpoints(), &plan, 1);
        let ip = &header[IPV4_AT..IPV4_AT + IPV4_HEADER_BYTES];
        assert_eq!(ones_complement_checksum(ip), 0);
        assert_ne!(u16::from_be_bytes([ip[10], ip[11]]), 0);
    }

    #[test]
    fn the_ip_and_udp_lengths_cover_the_packet_without_a_trailer() {
        for format in WIRE_FORMATS {
            for mtu in MTUS {
                let plan = PacketPlan::new(format, mtu);
                let header = frame_header(&endpoints(), &plan, 0);
                let total = u16::from_be_bytes([header[16], header[17]]) as usize;
                let udp_len = u16::from_be_bytes([header[38], header[39]]) as usize;
                assert_eq!(total, plan.frame_len - ETHERNET_HEADER_BYTES);
                assert_eq!(
                    udp_len,
                    UDP_HEADER_BYTES + VITA_HEADER_BYTES + plan.payload_bytes
                );
                assert!(total <= mtu.bytes());
            }
        }
    }

    #[test]
    fn the_vita_header_carries_the_type_class_and_packet_size() {
        let plan = PacketPlan::new(WireFormat::Cs8, Mtu::Standard);
        let header = frame_header(&endpoints(), &plan, 0x1234_5678);
        let word = |index: usize| {
            let at = VITA_AT + index * 4;
            u32::from_be_bytes([header[at], header[at + 1], header[at + 2], header[at + 3]])
        };
        assert_eq!(word(0) >> 28, 0b0001, "signal data with stream id");
        assert_eq!(word(0) >> 27 & 1, 1, "class id present");
        assert_eq!(word(0) >> 26 & 1, 0, "no trailer");
        assert_eq!(word(0) >> 22 & 0b11, 0b00, "no integer timestamp");
        assert_eq!(word(0) >> 20 & 0b11, 0b01, "sample count timestamp");
        assert_eq!(
            word(0) >> 16 & 0xf,
            0,
            "packet count left to the packetizer"
        );
        assert_eq!(word(0) & 0xffff, plan.packet_words as u32);
        assert_eq!(word(1), 0x1234_5678);
        assert_eq!(word(2), 0);
        assert_eq!(word(3), 0x0001_0002);
        assert_eq!(
            (word(4), word(5)),
            (0, 0),
            "timestamp left to the packetizer"
        );
    }
}

/// What a received data packet's header says about its samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VitaDataHeader {
    /// Sample count of the first sample.
    pub ticks: u64,
    /// Complex samples in the payload.
    pub samples: usize,
    pub end_of_burst: bool,
}

impl VitaDataHeader {
    /// Sample count just past the packet's last sample.
    pub fn end_ticks(&self) -> u64 {
        self.ticks + self.samples as u64
    }
}

/// Reads the 24-byte VITA header of a data packet in this profile: type 1 with
/// a stream id, a class id, no integer timestamp and a sample-count TSF.
/// Anything else, or a size that does not cover the header, is `None`.
pub fn parse_vita_data_header(vita: &[u8], format: WireFormat) -> Option<VitaDataHeader> {
    if vita.len() < VITA_HEADER_BYTES {
        return None;
    }
    let word = |i: usize| u32::from_be_bytes(vita[i * 4..i * 4 + 4].try_into().unwrap());
    let header = word(0);
    if header >> VITA_PACKET_TYPE_SHIFT != VITA_PACKET_TYPE_DATA_WITH_STREAM_ID
        || header & VITA_CLASS_ID_PRESENT == 0
        || header & VITA_TIMESTAMP_FIELDS_MASK != VITA_TSF_SAMPLE_COUNT
    {
        return None;
    }
    let end_of_burst = header & VITA_TRAILER_PRESENT != 0;
    let total = (header & VITA_PACKET_SIZE_MASK) as usize * 4;
    let overhead = VITA_HEADER_BYTES + if end_of_burst { VITA_TRAILER_BYTES } else { 0 };
    if total < overhead {
        return None;
    }
    let ticks = (u64::from(word(4)) << 32) | u64::from(word(5));
    Some(VitaDataHeader {
        ticks,
        samples: format.samples_in_payload(total - overhead),
        end_of_burst,
    })
}
