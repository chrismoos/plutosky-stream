#!/usr/bin/env python3
"""Parse and print plutosky-stream VITA 49.2 packets.

Reads from a pcap file or a live UDP socket, prints one line per packet, and
can write the samples out as a 16-bit stereo wav or a numpy array.

    vrt_dump.py --pcap capture.pcap
    vrt_dump.py --listen 0.0.0.0:31000 --count 200 --npy out.npy
    vrt_dump.py --pcap capture.pcap --stream-id 0xa001 --samples 4 --wav out.wav

Both MTUs and all three wire formats are recognized from the packet class, so
no format argument is needed.
"""

import argparse
import os
import socket
import struct
import sys
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vita

PCAP_MAGIC_LE = 0xA1B2C3D4
PCAP_MAGIC_BE = 0xD4C3B2A1
PCAP_MAGIC_LE_NS = 0xA1B23C4D
PCAP_MAGIC_BE_NS = 0x4DC3B2A1
PCAPNG_BLOCK_SECTION_HEADER = 0x0A0D0D0A
PCAPNG_BLOCK_ENHANCED_PACKET = 0x00000006
PCAPNG_BLOCK_INTERFACE_DESCRIPTION = 0x00000001
PCAPNG_BYTE_ORDER_MAGIC = 0x1A2B3C4D

LINKTYPE_ETHERNET = 1
LINKTYPE_RAW = 101
LINKTYPE_LINUX_SLL = 113
LINKTYPE_LINUX_SLL2 = 276

ETHERTYPE_IPV4 = 0x0800
ETHERTYPE_VLAN = 0x8100
IP_PROTOCOL_UDP = 17
ETHERNET_HEADER_BYTES = 14
VLAN_TAG_BYTES = 4
UDP_HEADER_BYTES = 8
LINUX_SLL_HEADER_BYTES = 16
LINUX_SLL2_HEADER_BYTES = 20

MAX_DATAGRAM_BYTES = 9216
WAV_SAMPLE_RATE_HZ = 48000
DEFAULT_LISTEN = "0.0.0.0:31000"


def udp_payload(link_type, frame):
    """Strips the link, IPv4 and UDP headers, returning the datagram payload."""
    data = frame
    if link_type == LINKTYPE_ETHERNET:
        if len(data) < ETHERNET_HEADER_BYTES:
            return None
        ethertype = struct.unpack(">H", data[12:14])[0]
        offset = ETHERNET_HEADER_BYTES
        while ethertype == ETHERTYPE_VLAN:
            if len(data) < offset + VLAN_TAG_BYTES:
                return None
            ethertype = struct.unpack(">H", data[offset + 2:offset + 4])[0]
            offset += VLAN_TAG_BYTES
        if ethertype != ETHERTYPE_IPV4:
            return None
        data = data[offset:]
    elif link_type == LINKTYPE_LINUX_SLL:
        if len(data) < LINUX_SLL_HEADER_BYTES:
            return None
        if struct.unpack(">H", data[14:16])[0] != ETHERTYPE_IPV4:
            return None
        data = data[LINUX_SLL_HEADER_BYTES:]
    elif link_type == LINKTYPE_LINUX_SLL2:
        if len(data) < LINUX_SLL2_HEADER_BYTES:
            return None
        if struct.unpack(">H", data[0:2])[0] != ETHERTYPE_IPV4:
            return None
        data = data[LINUX_SLL2_HEADER_BYTES:]
    elif link_type != LINKTYPE_RAW:
        return None

    if len(data) < 20:
        return None
    version_ihl = data[0]
    if version_ihl >> 4 != 4:
        return None
    header_bytes = (version_ihl & 0x0F) * 4
    if data[9] != IP_PROTOCOL_UDP or len(data) < header_bytes + UDP_HEADER_BYTES:
        return None
    total_length = struct.unpack(">H", data[2:4])[0]
    if total_length and total_length <= len(data):
        data = data[:total_length]
    udp = data[header_bytes:]
    length = struct.unpack(">H", udp[4:6])[0]
    payload = udp[UDP_HEADER_BYTES:]
    if length >= UDP_HEADER_BYTES:
        payload = payload[:length - UDP_HEADER_BYTES]
    return payload


def read_pcap(path):
    """Yields (seconds, udp payload) from a classic pcap or a pcapng file."""
    with open(path, "rb") as handle:
        blob = handle.read()
    if len(blob) < 4:
        return
    magic = struct.unpack("<I", blob[:4])[0]
    if magic == PCAPNG_BLOCK_SECTION_HEADER:
        yield from _read_pcapng(blob)
        return

    if magic in (PCAP_MAGIC_LE, PCAP_MAGIC_LE_NS):
        endian = "<"
    elif magic in (PCAP_MAGIC_BE, PCAP_MAGIC_BE_NS):
        endian = ">"
    else:
        raise ValueError("not a pcap or pcapng file")
    nanoseconds = magic in (PCAP_MAGIC_LE_NS, PCAP_MAGIC_BE_NS)
    link_type = struct.unpack(endian + "I", blob[20:24])[0]

    offset = 24
    while offset + 16 <= len(blob):
        ts_sec, ts_frac, captured, _original = struct.unpack(
            endian + "IIII", blob[offset:offset + 16])
        offset += 16
        frame = blob[offset:offset + captured]
        offset += captured
        payload = udp_payload(link_type, frame)
        if payload:
            yield ts_sec + ts_frac / (1e9 if nanoseconds else 1e6), payload


def _read_pcapng(blob):
    offset = 0
    endian = "<"
    link_types = []
    while offset + 12 <= len(blob):
        block_type = struct.unpack(endian + "I", blob[offset:offset + 4])[0]
        if block_type == PCAPNG_BLOCK_SECTION_HEADER:
            if struct.unpack("<I", blob[offset + 8:offset + 12])[0] != PCAPNG_BYTE_ORDER_MAGIC:
                endian = ">"
            link_types = []
        block_length = struct.unpack(endian + "I", blob[offset + 4:offset + 8])[0]
        if block_length < 12 or offset + block_length > len(blob):
            return
        body = blob[offset + 8:offset + block_length - 4]

        if block_type == PCAPNG_BLOCK_INTERFACE_DESCRIPTION:
            link_types.append(struct.unpack(endian + "H", body[0:2])[0])
        elif block_type == PCAPNG_BLOCK_ENHANCED_PACKET:
            interface, ts_high, ts_low, captured, _original = struct.unpack(
                endian + "IIIII", body[:20])
            frame = body[20:20 + captured]
            link_type = link_types[interface] if interface < len(link_types) else LINKTYPE_ETHERNET
            payload = udp_payload(link_type, frame)
            if payload:
                yield ((ts_high << 32) | ts_low) / 1e6, payload
        offset += block_length


def read_socket(endpoint, count, timeout):
    host, _, port = endpoint.rpartition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)
    sock.bind((host or "0.0.0.0", int(port)))
    seen = 0
    import time as _time
    start = _time.monotonic()
    while count == 0 or seen < count:
        try:
            datagram, _peer = sock.recvfrom(MAX_DATAGRAM_BYTES)
        except socket.timeout:
            break
        seen += 1
        yield _time.monotonic() - start, datagram
    sock.close()


def describe(packet, index, timestamp, show_samples):
    kind = "data" if packet["packet_type"] == vita.PACKET_TYPE_DATA else (
        "context" if packet["packet_type"] == vita.PACKET_TYPE_CONTEXT else "type%d"
        % packet["packet_type"])
    fmt = packet["format"] or "class 0x%04x" % packet["packet_class"]
    samples = 0
    if packet["format"]:
        samples = vita.samples_in_payload(packet["format"], len(packet["payload"]))
    line = ("#%-6d t=%.6f %-7s stream=0x%08x count=%2d words=%4d %-5s ticks=%d "
            "payload=%dB samples=%d%s"
            % (index, timestamp, kind, packet["stream_id"], packet["count"], packet["words"],
               fmt, packet["ticks"], len(packet["payload"]), samples,
               " END_BURST" if packet["end_of_burst"] else ""))
    print(line)
    if show_samples and packet["format"] and samples:
        take = min(show_samples, samples)
        decoded = vita.unpack(packet["format"], packet["payload"], take)
        print("        " + " ".join("(%d,%d)" % (i, q) for i, q in decoded))
    return samples


def write_wav(path, samples):
    with wave.open(path, "wb") as handle:
        handle.setnchannels(2)
        handle.setsampwidth(2)
        handle.setframerate(WAV_SAMPLE_RATE_HZ)
        # The wire is 12-bit, the file is full-scale 16-bit.
        frames = bytearray()
        for i, q in samples:
            frames += struct.pack("<hh", max(-32768, min(32767, i << 4)),
                                  max(-32768, min(32767, q << 4)))
        handle.writeframes(bytes(frames))


def write_npy(path, samples):
    import numpy

    data = numpy.array([complex(i, q) for i, q in samples], dtype=numpy.complex64)
    data /= vita.SAMPLE_MAX
    numpy.save(path, data)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--pcap", help="pcap or pcapng file to read")
    source.add_argument("--listen", nargs="?", const=DEFAULT_LISTEN,
                        help="bind address:port to receive from, default " + DEFAULT_LISTEN)
    parser.add_argument("--stream-id", default=None,
                        help="only show this stream id, decimal or 0x hex")
    parser.add_argument("--count", type=int, default=0, help="stop after this many packets")
    parser.add_argument("--timeout", type=float, default=2.0, help="socket idle timeout")
    parser.add_argument("--samples", type=int, default=0,
                        help="print this many decoded samples per packet")
    parser.add_argument("--quiet", action="store_true", help="only print the summary")
    parser.add_argument("--wav", help="write the samples to a 16-bit stereo wav")
    parser.add_argument("--npy", help="write the samples to a complex64 npy")
    args = parser.parse_args(argv)

    wanted = int(str(args.stream_id), 0) if args.stream_id is not None else None
    collect = args.wav or args.npy
    collected = []

    packets = 0
    total_samples = 0
    bursts = 0
    gaps = 0
    errors = 0
    last_count = {}

    stream = read_pcap(args.pcap) if args.pcap else read_socket(args.listen, args.count,
                                                                args.timeout)
    for timestamp, datagram in stream:
        try:
            packet = vita.parse_packet(datagram)
        except vita.VitaError:
            errors += 1
            continue
        if wanted is not None and packet["stream_id"] != wanted:
            continue

        packets += 1
        if packet["packet_type"] == vita.PACKET_TYPE_DATA:
            previous = last_count.get(packet["stream_id"])
            if previous is not None:
                gaps += vita.count_gap(previous, packet["count"])
            last_count[packet["stream_id"]] = packet["count"]

        if args.quiet:
            if packet["format"]:
                total_samples += vita.samples_in_payload(packet["format"], len(packet["payload"]))
        else:
            total_samples += describe(packet, packets, timestamp, args.samples)
        if packet["end_of_burst"]:
            bursts += 1

        if collect and packet["format"] and packet["payload"]:
            count = vita.samples_in_payload(packet["format"], len(packet["payload"]))
            collected.extend(vita.unpack(packet["format"], packet["payload"], count))

        if args.count and packets >= args.count:
            break

    if args.wav and collected:
        write_wav(args.wav, collected)
        print("wrote %d samples to %s" % (len(collected), args.wav))
    if args.npy and collected:
        write_npy(args.npy, collected)
        print("wrote %d samples to %s" % (len(collected), args.npy))

    print("SUMMARY packets=%d samples=%d bursts=%d sequence_gaps=%d malformed=%d"
          % (packets, total_samples, bursts, gaps, errors))
    return 0 if packets else 1


if __name__ == "__main__":
    sys.exit(main())
