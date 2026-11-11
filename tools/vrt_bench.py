#!/usr/bin/env python3
"""Measure a plutosky-stream receive flow.

Reports packets per second, samples per second, sequence gaps, timestamp
continuity and host CPU. It can drive the daemon itself or watch a flow
somebody else set up.

    vrt_bench.py --addr 127.0.0.1 --wire cs12 --mtu 1500 --secs 5
    vrt_bench.py --listen 0.0.0.0:31000 --secs 5

Timestamp continuity counts packets whose sample count is not exactly where the
previous packet ended, which is the same test the Soapy module applies.
"""

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import control
import udp_socket
import vita

MAX_DATAGRAM_BYTES = 9216
SOCKET_BUFFER_BYTES = 16 << 20
SOCKET_TIMEOUT_S = 0.25
PROGRESS_PERIOD_S = 1.0
CLOCK_TICKS_PER_SECOND = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
BITS_PER_BYTE = 8
FRAME_OVERHEAD_BYTES = 14 + 20 + 8 + 38  # ethernet, IPv4, UDP, preamble, FCS and gap


def process_cpu_seconds():
    with open("/proc/self/stat", "rb") as handle:
        fields = handle.read().split()
    # utime and stime, after the comm field which may itself contain spaces.
    return (int(fields[13]) + int(fields[14])) / CLOCK_TICKS_PER_SECOND


def system_cpu_times():
    with open("/proc/stat", "rb") as handle:
        fields = handle.readline().split()
    values = [int(v) for v in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


class Bench:
    def __init__(self, args):
        self.args = args
        self.packets = 0
        self.samples = 0
        self.payload_bytes = 0
        self.wire_bytes = 0
        self.sequence_gaps = 0
        self.lost_packets = 0
        self.time_jumps = 0
        self.missing_samples = 0
        self.largest_jump = 0
        self.malformed = 0
        self.socket_drops = None
        self.bursts = 0
        self.last_count = None
        self.last_end_tick = None
        self.first_tick = None
        self.last_tick = None

    def consume(self, datagram):
        try:
            packet = vita.parse_packet(datagram)
        except vita.VitaError:
            self.malformed += 1
            return
        if self.args.stream_id is not None and packet["stream_id"] != self.args.stream_id:
            return
        if packet["packet_type"] != vita.PACKET_TYPE_DATA:
            return
        if not packet["payload"]:
            return

        fmt = packet["format"]
        if fmt is None:
            self.malformed += 1
            return
        samples = vita.samples_in_payload(fmt, len(packet["payload"]))
        if self.args.samples_per_packet:
            samples = min(samples, self.args.samples_per_packet)

        if self.last_count is not None:
            missing = vita.count_gap(self.last_count, packet["count"])
            if missing:
                self.sequence_gaps += 1
                self.lost_packets += missing
        self.last_count = packet["count"]

        if self.last_end_tick is not None:
            jump = packet["ticks"] - self.last_end_tick
            if jump:
                self.time_jumps += 1
                self.missing_samples += max(jump, 0)
                self.largest_jump = max(self.largest_jump, abs(jump))
        if self.first_tick is None:
            self.first_tick = packet["ticks"]

        channels = max(1, self.args.channels)
        self.last_end_tick = packet["ticks"] + samples // channels
        self.last_tick = packet["ticks"]
        self.packets += 1
        self.samples += samples
        self.payload_bytes += len(packet["payload"])
        self.wire_bytes += len(datagram) + FRAME_OVERHEAD_BYTES
        if packet["end_of_burst"]:
            self.bursts += 1

    def report(self, elapsed, cpu_seconds, system_busy, rate):
        pps = self.packets / elapsed if elapsed else 0.0
        sps = self.samples / elapsed if elapsed else 0.0
        mbit = self.wire_bytes * BITS_PER_BYTE / elapsed / 1e6 if elapsed else 0.0
        cpu = cpu_seconds / elapsed * 100.0 if elapsed else 0.0

        print("packets      %d (%.0f/s)" % (self.packets, pps))
        print("samples      %d (%.3f MS/s)" % (self.samples, sps / 1e6))
        print("payload      %.1f MB (%.1f Mbit/s on the wire)"
              % (self.payload_bytes / 1e6, mbit))
        print("sequence     %d gaps, %d packets lost" % (self.sequence_gaps, self.lost_packets))
        print("timestamps   %d discontinuities, %d end of burst"
              % (self.time_jumps, self.bursts))
        print("sample gaps  %d missing, largest jump %d samples"
              % (self.missing_samples, self.largest_jump))
        print("malformed    %d" % self.malformed)
        print("cpu          %.1f%% of one core in this process, %.1f%% busy across the host"
              % (cpu, system_busy))
        if rate:
            print("rate         %.0f Hz configured, %.1f%% of it received"
                  % (rate, sps / rate * 100.0 if rate else 0.0))
        print("RESULT packets=%d samples=%d pps=%.0f sps=%.0f mbit=%.1f gaps=%d lost=%d "
              "time_jumps=%d missing_samples=%d largest_jump=%d malformed=%d cpu=%.1f secs=%.3f "
              "socket_drops=%s"
              % (self.packets, self.samples, pps, sps, mbit, self.sequence_gaps,
                 self.lost_packets, self.time_jumps, self.missing_samples,
                 self.largest_jump, self.malformed, cpu, elapsed,
                 "n/a" if self.socket_drops is None else self.socket_drops))


def run(args):
    bench = Bench(args)
    client = None
    stream = None
    rate = 0.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.enlarge_receive_buffer(sock, SOCKET_BUFFER_BYTES)
    sock.settimeout(SOCKET_TIMEOUT_S)

    if args.listen:
        host, _, port = args.listen.rpartition(":")
        sock.bind((host or "0.0.0.0", int(port)))
    else:
        sock.bind(("0.0.0.0", 0))
        client = control.ControlClient(args.addr, args.port, client="vrt_bench")
        if args.rate:
            client.request("set_sample_rate", hz=int(args.rate))
        rate = float(client.request("get_sample_rate")["hz"])
        chans = list(range(max(1, args.channels)))
        stream = client.request("open_stream", dir="rx", chans=chans, format=args.wire,
                                mtu=args.mtu, host_ip=sock.getsockname()[0],
                                host_port=sock.getsockname()[1])
        args.stream_id = stream["stream_id"]
        args.samples_per_packet = stream["samples_per_packet"]
        print("stream 0x%08x %s mtu=%d samples_per_packet=%d window=%d rate=%.0f"
              % (stream["stream_id"], args.wire, args.mtu, stream["samples_per_packet"],
                 stream["window"], rate))
        # The daemon needs a datagram from the host before it can build the header
        # template, exactly as the Soapy module does after open_stream.
        hello = vita.build_packet(stream["stream_id"],
                                  vita.PACKET_CLASS_BY_FORMAT[args.wire], 0, 0, b"")
        sock.sendto(hello, (args.addr, stream["data_port"]))
        client.request("start_stream", stream_id=stream["stream_id"])

    start = time.monotonic()
    cpu_start = process_cpu_seconds()
    system_start = system_cpu_times()
    deadline = start + args.secs
    next_progress = start + PROGRESS_PERIOD_S
    last_packets = 0

    try:
        while time.monotonic() < deadline:
            try:
                datagram, _peer = sock.recvfrom(MAX_DATAGRAM_BYTES)
            except socket.timeout:
                continue
            bench.consume(datagram)
            now = time.monotonic()
            if args.progress and now >= next_progress:
                print("  %.0fs %d packets (%d/s)"
                      % (now - start, bench.packets, bench.packets - last_packets))
                last_packets = bench.packets
                next_progress = now + PROGRESS_PERIOD_S
    except KeyboardInterrupt:
        pass

    elapsed = time.monotonic() - start
    cpu_seconds = process_cpu_seconds() - cpu_start
    system_end = system_cpu_times()
    busy_total = system_end[0] - system_start[0]
    busy_idle = system_end[1] - system_start[1]
    system_busy = (busy_total - busy_idle) / busy_total * 100.0 if busy_total else 0.0

    last_status = None
    if client is not None and stream is not None:
        try:
            client.request("stop_stream", stream_id=stream["stream_id"])
            client.request("close_stream", stream_id=stream["stream_id"])
        except control.ControlError:
            pass
        statuses = [event for event in client.drain_events()
                    if event.get("event") == "status"
                    and event.get("stream_id") == stream["stream_id"]]
        if statuses:
            last_status = statuses[-1]
        client.close()
    bench.socket_drops = udp_socket.receive_drops(sock)
    sock.close()

    bench.report(elapsed, cpu_seconds, system_busy, rate)
    if last_status is not None:
        print("BOARD_STATUS produced=%d consumed=%d overflow=%d gaps=%d late=%d underrun=%d"
              % tuple(last_status[key] for key in
                      ("produced", "consumed", "overflow", "gaps", "late", "underrun")))
    return 0 if bench.packets else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--addr", default="127.0.0.1", help="daemon address to drive")
    parser.add_argument("--port", type=int, default=control.DEFAULT_CONTROL_PORT)
    parser.add_argument("--listen", default=None,
                        help="watch an existing flow on this address:port instead")
    parser.add_argument("--wire", default="cs12", choices=list(vita.FORMATS))
    parser.add_argument("--mtu", type=int, default=1500, choices=(1500, 9000))
    parser.add_argument("--channels", type=int, default=1)
    parser.add_argument("--rate", type=float, default=0.0, help="sample rate to request")
    parser.add_argument("--secs", type=float, default=5.0)
    parser.add_argument("--stream-id", type=lambda v: int(v, 0), default=None)
    parser.add_argument("--samples-per-packet", type=int, default=0)
    parser.add_argument("--progress", action="store_true")
    return run(parser.parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
