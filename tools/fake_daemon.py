#!/usr/bin/env python3
"""A fake of the PlutoSky board daemon, for testing the host side.

Serves the gRPC control service in proto/plutosky/v1/control.proto and the
VITA profile in docs/wire-formats.md over loopback or any host interface.
Receive streams are fed from a generated tone or a sample file. Transmit
streams are checked for stream id, packet sequence, timestamp continuity and
end of burst, and the findings are available over the fake-only `TxReport`
call.

It backs the Soapy module and UHD tests. It never touches a board.
"""

import argparse
import collections
import json
import math
import queue
import os
import select
import socket
import struct
import sys
import threading
import time
import wave
from concurrent import futures

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import grpc

import control
import udp_socket
import vita

STATUS_PERIOD_S = 0.01
DEFAULT_STATUS_INTERVAL_MS = 10
MIN_STATUS_INTERVAL_MS = 1
MAX_STATUS_INTERVAL_MS = 1000
SESSION_POLL_S = 0.05
GRPC_WORKERS = 32
DEFAULT_CONTROL_PORT = 30000
DEFAULT_DISCOVERY_PORT = 30001
DEFAULT_RATE_HZ = 4915200.0
MIN_RATE_HZ = 520833.0
MAX_RATE_HZ = 61440000.0
# The rate filter's largest ratio and the fastest radio rate it runs at.
RATIO_MAX = 32
FILTER_MAX_RATE_HZ = 15360000
DEFAULT_WINDOW_PACKETS = 256
DEFAULT_MAX_PPS = 2000.0
DEFAULT_TONE_HZ = 100000.0
DEFAULT_TONE_AMPLITUDE = 0.5
# How long a data socket may stay empty before a closing stream stops reading it.
DATA_POLL_S = 0.05
DATA_SOCKET_BUFFER_BYTES = 8 << 20
CLOSE_DRAIN_TIMEOUT_S = 5.0
# SO_TIMESTAMPNS on Linux: the kernel stamps each datagram with the realtime
# clock when it arrives, as a struct timespec of two native longs.
LINUX_SO_TIMESTAMPNS = 35
TIMESPEC = struct.Struct("@ll")
DEFAULT_CHANNELS = 2
STREAM_ID_BASE = 0x0000A000
ANTENNAS = {"rx": ["A_BALANCED"], "tx": ["A"]}
DEFAULT_FREQUENCY_HZ = 900e6
DEFAULT_BANDWIDTH_HZ = 4e6
DEFAULT_GAIN_DB = 30.0
DEFAULT_TX_GAIN_DB = 0.0
MAX_TX_GAIN_DB = 89.75
TX_GAIN_STEP_DB = 0.25
DEFAULT_TEMPERATURE_C = 41.5
DEFAULT_RSSI_DB = -62.0
GENERATOR_IDLE_SLEEP_S = 0.002


def log(enabled, message):
    if enabled:
        sys.stderr.write("fake_daemon: %s\n" % message)
        sys.stderr.flush()


class SampleSource:
    """Supplies complex samples in the 12-bit scale the wire formats use."""

    def __init__(self, path=None, tone_hz=DEFAULT_TONE_HZ, amplitude=DEFAULT_TONE_AMPLITUDE):
        self.tone_hz = tone_hz
        self.amplitude = amplitude
        self.samples = None
        if path is not None:
            self.samples = self._load(path)

    @staticmethod
    def _load(path):
        if path.endswith(".npy"):
            import numpy

            data = numpy.load(path)
            if data.dtype.kind == "c":
                scale = vita.SAMPLE_MAX
                return [(int(round(v.real * scale)), int(round(v.imag * scale))) for v in data]
            flat = data.reshape(-1, 2)
            return [(int(a), int(b)) for a, b in flat]
        if path.endswith(".wav"):
            with wave.open(path, "rb") as handle:
                if handle.getnchannels() != 2 or handle.getsampwidth() != 2:
                    raise ValueError("a wav source must be 16-bit stereo, I in left and Q in right")
                frames = handle.readframes(handle.getnframes())
            values = struct.unpack("<%dh" % (len(frames) // 2), frames)
            # The file is full-scale 16-bit, the wire is 12-bit.
            return [(values[n] >> 4, values[n + 1] >> 4) for n in range(0, len(values), 2)]
        raise ValueError("unsupported sample source " + path)

    def block(self, first_tick, count, rate, channels):
        """Interleaved samples for `count` per-channel samples from first_tick."""
        out = []
        if self.samples is not None:
            total = len(self.samples)
            for n in range(count):
                value = self.samples[(first_tick + n) % total]
                for _ in range(channels):
                    out.append(value)
            return out

        step = 2.0 * math.pi * self.tone_hz / rate
        scale = self.amplitude * vita.SAMPLE_MAX
        for n in range(count):
            phase = step * (first_tick + n)
            i = int(round(math.cos(phase) * scale))
            q = int(round(math.sin(phase) * scale))
            for c in range(channels):
                # The second channel is inverted so a two-channel capture shows
                # which samples came from which channel.
                out.append((i, q) if c == 0 else (-i, -q))
        return out


class TxReport:
    def __init__(self):
        self.packets = 0
        self.samples = 0
        self.bursts = 0
        self.count_gaps = 0
        self.tick_gaps = 0
        self.late = 0
        self.bad_stream_id = 0
        self.bad_packet_class = 0
        self.decode_errors = 0
        self.first_ticks = None
        self.last_ticks = None
        self.end_of_burst_ticks = []
        self.sample_checksum = 0
        self.hello_packets = 0
        self.after_burst = False

    def as_dict(self):
        return {
            "packets": self.packets,
            "samples": self.samples,
            "bursts": self.bursts,
            "count_gaps": self.count_gaps,
            "tick_gaps": self.tick_gaps,
            "late": self.late,
            "bad_stream_id": self.bad_stream_id,
            "bad_packet_class": self.bad_packet_class,
            "decode_errors": self.decode_errors,
            "first_ticks": self.first_ticks,
            "last_ticks": self.last_ticks,
            "end_of_burst_ticks": list(self.end_of_burst_ticks),
            "sample_checksum": self.sample_checksum,
            "hello_packets": self.hello_packets,
        }


class Stream:
    def __init__(self, stream_id, direction, chans, fmt, mtu, host_ip, host_port, window):
        self.stream_id = stream_id
        self.direction = direction
        self.chans = list(chans)
        self.format = fmt
        self.mtu = mtu
        self.host_ip = host_ip
        self.host_port = host_port
        self.window = window
        self.samples_per_packet = vita.samples_per_packet(fmt, mtu)
        self.elems_per_packet = self.samples_per_packet // max(1, len(self.chans))
        self.packet_class = vita.PACKET_CLASS_BY_FORMAT[fmt]

        self.socket = None
        self.data_port = 0
        self.peer = None
        self.owner = None

        self.running = threading.Event()
        self.closing = threading.Event()
        self.thread = None
        self.start_tick = None
        self.run_id = 0

        self.produced = 0
        self.consumed = 0
        # TX: the tick after the last sample received, and for each packet not
        # yet played that tick with what consumed reads once it has played.
        self.frontier = 0
        self.playing = collections.deque()
        self.late = 0
        self.underrun = 0
        # The tick up to which underrun has been counted.
        self.underrun_mark = 0
        self.overflow = 0
        self.gaps = 0
        self.packet_count = 0
        self.report = TxReport()
        self.lock = threading.Lock()


class Session:
    def __init__(self, session_id):
        self.session_id = session_id
        self.alive = True
        self.events = queue.Queue()
        self.interval_s = DEFAULT_STATUS_INTERVAL_MS / 1000.0
        self.last_status = 0.0


class FakeDaemon:
    def __init__(self, args):
        self.args = args
        self.verbose = args.verbose
        self.bind = args.bind
        # Receive and transmit sample rates, and the rate the timebase counts at.
        self.rates = {"rx": int(args.rate), "tx": int(args.rate)}
        self.tick_rate = int(args.rate)
        # Directions whose rate a client set, and rates a later request for the
        # other direction replaced.
        self.rate_set = {"rx": False, "tx": False}
        self.rate_replaced = {"rx": None, "tx": None}
        self.source = SampleSource(args.source, args.tone_hz, args.amplitude)

        self.lock = threading.Lock()
        self.streams = {}
        self.closed_reports = []
        self.next_stream_id = STREAM_ID_BASE
        self.sessions = {}
        self.next_session_id = int(time.time() * 1000)
        self.stopping = threading.Event()
        self.ready = threading.Event()

        self.tick_base = 0
        self.time_base = time.monotonic()
        self.control_port = args.control_port
        self.discovery_port = args.discovery_port

        self.frequency = {"rx": {}, "tx": {}}
        self.bandwidth = {"rx": {}, "tx": {}}
        self.gain = {"rx": {}, "tx": {}}
        self.gain_mode = {"rx": {}, "tx": {}}
        self.antenna = {"rx": {}, "tx": {}}
        for direction in ("rx", "tx"):
            for chan in range(args.channels):
                self.frequency[direction][chan] = DEFAULT_FREQUENCY_HZ
                self.bandwidth[direction][chan] = DEFAULT_BANDWIDTH_HZ
                self.gain[direction][chan] = (DEFAULT_TX_GAIN_DB if direction == "tx"
                                              else DEFAULT_GAIN_DB)
                self.gain_mode[direction][chan] = "manual"
                self.antenna[direction][chan] = ANTENNAS[direction][0]

    # Hardware time

    def ticks(self):
        return self.tick_base + int((time.monotonic() - self.time_base) * self.tick_rate)

    def set_ticks(self, ticks):
        self.tick_base = int(ticks)
        self.time_base = time.monotonic()

    # Control service

    def op_get_info(self, client, request):
        return {
            "serial": self.args.serial,
            "version": "fake",
            "channels": self.args.channels,
            "min_rate_hz": int(MIN_RATE_HZ),
            "max_rate_hz": int(MAX_RATE_HZ),
            "formats": list(vita.FORMATS),
            "mtus": [1500, 9000],
        }

    def op_set_frequency(self, client, request):
        self.frequency[request["dir"]][int(request["chan"])] = float(request["hz"])
        return {"hz": float(request["hz"])}

    def op_get_frequency(self, client, request):
        return {"hz": self.frequency[request["dir"]][int(request["chan"])]}

    def ticks_per_sample(self, direction):
        return self.tick_rate // self.rates[direction]

    @staticmethod
    def radio_rate(rx_hz, tx_hz):
        """The slowest radio rate that is a whole multiple of both, or None."""
        common = math.lcm(rx_hz, tx_hz)
        radio = common * max(1, -(-int(MIN_RATE_HZ) // common))
        filtered = radio != rx_hz or radio != tx_hz
        if radio // rx_hz > RATIO_MAX or radio // tx_hz > RATIO_MAX:
            return None
        if filtered and radio > FILTER_MAX_RATE_HZ:
            return None
        return radio

    def sample_rate_reply(self, direction=None):
        return {
            "hz": self.rates["tx" if direction == "tx" else "rx"],
            "rx_hz": self.rates["rx"],
            "tx_hz": self.rates["tx"],
            "tick_rate_hz": self.tick_rate,
            "rx_ticks_per_sample": self.ticks_per_sample("rx"),
            "tx_ticks_per_sample": self.ticks_per_sample("tx"),
        }

    def op_set_sample_rate(self, client, request):
        with self.lock:
            if any(s.running.is_set() for s in self.streams.values()):
                raise ValueError("the sample rate cannot change while a stream is active")
        hz = int(max(MIN_RATE_HZ, min(MAX_RATE_HZ, float(request["hz"]))))
        direction = request.get("dir")
        if direction not in ("rx", "tx"):
            direction = None
        pair = {"rx": hz, "tx": hz}
        if direction is None:
            self.rate_set = {"rx": True, "tx": True}
            self.rate_replaced = {"rx": None, "tx": None}
        else:
            other = "tx" if direction == "rx" else "rx"
            if self.rate_set[other]:
                pair[other] = self.rates[other]
                if self.radio_rate(pair["rx"], pair["tx"]) is None:
                    self.rate_replaced[other] = pair[other]
                    pair[other] = hz
            self.rate_set[direction] = True
            self.rate_replaced[direction] = None
        radio = self.radio_rate(pair["rx"], pair["tx"])
        if radio is None:
            raise ValueError("sample rate %d Hz is out of range" % hz)
        seconds = self.ticks() / self.tick_rate
        self.rates = pair
        self.tick_rate = radio
        self.set_ticks(int(seconds * radio))
        return self.sample_rate_reply(direction)

    def op_get_sample_rate(self, client, request):
        return self.sample_rate_reply()

    def check_rate_request(self, direction):
        replaced = self.rate_replaced[direction]
        if replaced is not None:
            raise ValueError(
                "the %s rate %d Hz has no common radio rate with the other direction and "
                "was replaced by %d Hz: set the %s rate again"
                % (direction, replaced, self.rates[direction], direction))

    def op_set_bandwidth(self, client, request):
        self.bandwidth[request["dir"]][int(request["chan"])] = float(request["hz"])
        return {"hz": float(request["hz"])}

    def op_get_bandwidth(self, client, request):
        return {"hz": self.bandwidth[request["dir"]][int(request["chan"])]}

    def op_set_gain(self, client, request):
        gain = float(request["db"])
        if request["dir"] == "tx":
            if not math.isfinite(gain) or not 0 <= gain <= MAX_TX_GAIN_DB:
                raise ValueError("TX gain must be between 0 and 89.75 dB")
            gain = round(gain / TX_GAIN_STEP_DB) * TX_GAIN_STEP_DB
        self.gain[request["dir"]][int(request["chan"])] = gain
        return {"db": gain}

    def op_get_gain(self, client, request):
        return {"db": self.gain[request["dir"]][int(request["chan"])]}

    def op_set_gain_mode(self, client, request):
        mode = request["mode"]
        if mode not in ("manual", "slow_attack", "fast_attack"):
            raise ValueError("unknown gain mode " + str(mode))
        self.gain_mode[request["dir"]][int(request["chan"])] = mode
        return {"mode": mode}

    def op_list_antennas(self, client, request):
        return {"names": ANTENNAS[request["dir"]]}

    def op_set_antenna(self, client, request):
        name = request["name"]
        if name not in ANTENNAS[request["dir"]]:
            raise ValueError("unknown antenna " + str(name))
        self.antenna[request["dir"]][int(request["chan"])] = name
        return {"name": name}

    def op_get_hardware_time(self, client, request):
        return {"ticks": self.ticks()}

    def op_set_hardware_time(self, client, request):
        self.set_ticks(int(request["ticks"]))
        return {"ticks": self.ticks()}

    def op_get_sensors(self, client, request):
        with self.lock:
            tx_rf_enabled = any(stream.direction == "tx" and stream.running.is_set()
                                for stream in self.streams.values())
        return {
            "temperature_c": DEFAULT_TEMPERATURE_C,
            "rssi_dbfs": [DEFAULT_RSSI_DB] * self.args.channels,
            "rx_lo_locked": True,
            "tx_lo_locked": True,
            "tx_rf_enabled": tx_rf_enabled,
        }

    def op_open_stream(self, client, request):
        direction = request["dir"]
        fmt = request["format"]
        mtu = int(request["mtu"])
        if fmt not in vita.FORMATS:
            raise ValueError("unknown wire format " + str(fmt))
        if mtu not in (1500, 9000):
            raise ValueError("unsupported mtu " + str(mtu))
        self.check_rate_request(direction)

        with self.lock:
            stream_id = self.next_stream_id
            self.next_stream_id += 1

        stream = Stream(stream_id, direction, request.get("chans", [0]), fmt, mtu,
                        request.get("host_ip", ""), int(request.get("host_port", 0)),
                        self.args.window)
        stream.owner = client
        stream.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        granted = udp_socket.enlarge_receive_buffer(stream.socket, DATA_SOCKET_BUFFER_BYTES)
        stream.socket.bind((self.bind, 0))
        stream.data_port = stream.socket.getsockname()[1]
        log(self.verbose, "stream %d data port %d receive buffer %d bytes"
            % (stream_id, stream.data_port, granted))
        if stream.host_ip and stream.host_port:
            stream.peer = (stream.host_ip, stream.host_port)

        with self.lock:
            self.streams[stream_id] = stream

        stream.thread = threading.Thread(target=self.data_loop, args=(stream,), daemon=True)
        stream.thread.start()

        log(self.verbose, "open_stream %d %s %s mtu=%d spp=%d port=%d"
            % (stream_id, direction, fmt, mtu, stream.samples_per_packet, stream.data_port))

        return {
            "stream_id": stream_id,
            "data_port": stream.data_port,
            "samples_per_packet": stream.samples_per_packet,
            "window": stream.window,
        }

    def op_start_stream(self, client, request):
        stream = self.stream_of(request)
        self.check_rate_request(stream.direction)
        at_ticks = request.get("at_ticks")
        stream.start_tick = int(at_ticks) if at_ticks is not None else self.ticks()
        # The board's counters start from zero with every run.
        with stream.lock:
            stream.produced = 0
            stream.consumed = 0
            stream.frontier = 0
            stream.playing.clear()
            stream.late = 0
            stream.underrun = 0
            stream.underrun_mark = 0
            stream.overflow = 0
            stream.gaps = 0
        stream.run_id += 1
        stream.running.set()
        if stream.direction == "rx":
            generator = threading.Thread(target=self.generate, args=(stream, stream.run_id),
                                         daemon=True)
            generator.start()
        log(self.verbose, "start_stream %d at %d" % (stream.stream_id, stream.start_tick))
        return {}

    def op_stop_stream(self, client, request):
        stream = self.stream_of(request)
        stream.running.clear()
        log(self.verbose, "stop_stream %d" % stream.stream_id)
        return {}

    def op_close_stream(self, client, request):
        stream = self.stream_of(request)
        stream.running.clear()
        stream.closing.set()
        # The checker can trail the host, so the report waits for it to read
        # what the host sent before the close.
        if stream.thread is not None and stream.thread is not threading.current_thread():
            stream.thread.join(CLOSE_DRAIN_TIMEOUT_S)
        with stream.lock:
            report = stream.report.as_dict()
        report["stream_id"] = stream.stream_id
        report["dir"] = stream.direction
        report["socket_drops"] = udp_socket.receive_drops(stream.socket)
        with self.lock:
            self.streams.pop(stream.stream_id, None)
            self.closed_reports.append(report)
        stream.socket.close()
        log(self.verbose, "close_stream %d" % stream.stream_id)
        return {}

    def op_tx_report(self, client, request):
        """Fake-only: what the transmit checker saw, open or already closed."""
        stream_id = request.get("stream_id")
        with self.lock:
            streams = list(self.streams.values())
            closed = list(self.closed_reports)

        for stream in streams:
            if stream_id is not None and stream.stream_id != int(stream_id):
                continue
            if stream_id is None and stream.direction != "tx":
                continue
            with stream.lock:
                report = stream.report.as_dict()
            report["stream_id"] = stream.stream_id
            report["dir"] = stream.direction
            report["socket_drops"] = udp_socket.receive_drops(stream.socket)
            return report

        for report in reversed(closed):
            if stream_id is not None and report["stream_id"] != int(stream_id):
                continue
            if stream_id is None and report["dir"] != "tx":
                continue
            return report
        return {}

    def stream_of(self, request):
        with self.lock:
            stream = self.streams.get(int(request["stream_id"]))
        if stream is None:
            raise ValueError("unknown stream " + str(request.get("stream_id")))
        return stream

    # Data plane

    def arrival_ticks(self, ancdata):
        """The timebase when the datagram reached the socket, or now when the
        kernel did not stamp it."""
        now_ticks = self.ticks()
        for level, kind, data in ancdata:
            if level == socket.SOL_SOCKET and kind == LINUX_SO_TIMESTAMPNS \
                    and len(data) >= TIMESPEC.size:
                seconds, nanoseconds = TIMESPEC.unpack_from(data)
                age = max(0.0, time.time() - (seconds + nanoseconds / 1e9))
                return now_ticks - int(age * self.tick_rate)
        return now_ticks

    def data_loop(self, stream):
        """Reads the stream's data socket and hands each datagram to a checker
        thread. Reading does nothing else, so a checker slowed by a loaded host
        does not leave datagrams for the kernel to drop when the socket buffer
        fills, which a tail of lost packets would not show as a count gap."""
        # Lateness is judged when a packet arrives, as the board does, not when
        # the checker gets to it.
        ancillary = 0
        if sys.platform.startswith("linux"):
            try:
                stream.socket.setsockopt(socket.SOL_SOCKET, LINUX_SO_TIMESTAMPNS, 1)
                ancillary = socket.CMSG_SPACE(TIMESPEC.size)
            except OSError:
                pass
        received = queue.SimpleQueue()
        checker = threading.Thread(target=self.check_loop, args=(stream, received),
                                   daemon=True)
        checker.start()
        try:
            while True:
                try:
                    ready, _, _ = select.select([stream.socket], [], [], DATA_POLL_S)
                except (OSError, ValueError):
                    break
                if not ready:
                    if stream.closing.is_set():
                        break
                    continue
                try:
                    datagram, ancdata, _, peer = stream.socket.recvmsg(
                        vita.payload_capacity(9000) + 64, ancillary)
                except OSError:
                    break
                if datagram:
                    received.put((datagram, self.arrival_ticks(ancdata), peer))
        finally:
            received.put(None)
            checker.join()

    def check_loop(self, stream, received):
        while True:
            item = received.get()
            if item is None:
                return
            datagram, arrival, peer = item
            try:
                packet = vita.parse_packet(datagram)
            except vita.VitaError:
                with stream.lock:
                    stream.report.decode_errors += 1
                continue

            if packet["stream_id"] != stream.stream_id:
                with stream.lock:
                    stream.report.bad_stream_id += 1
                continue

            # The first datagram has no samples. It teaches the host address before
            # the packetizer header template is built.
            if not packet["payload"]:
                stream.peer = peer
                with stream.lock:
                    stream.report.hello_packets += 1
                log(self.verbose, "stream %d learned host %s:%d"
                    % (stream.stream_id, peer[0], peer[1]))
                continue

            if stream.direction == "rx":
                continue
            self.check_tx_packet(stream, packet, arrival)

    def check_tx_packet(self, stream, packet, arrival_ticks):
        channels = max(1, len(stream.chans))
        with stream.lock:
            report = stream.report
            if packet["packet_class"] != stream.packet_class:
                report.bad_packet_class += 1
                return

            samples = vita.samples_in_payload(stream.format, len(packet["payload"]))
            try:
                decoded = vita.unpack(stream.format, packet["payload"], samples)
            except vita.VitaError:
                report.decode_errors += 1
                return

            if report.packets:
                if vita.count_gap(stream.packet_count, packet["count"]):
                    report.count_gaps += 1
                    stream.gaps += 1
                # A jump in sample counts after an end of burst starts the next
                # burst. Anywhere else it is a hole the PL would fill with zeros.
                if packet["ticks"] != report.last_ticks and not report.after_burst:
                    report.tick_gaps += 1
                    stream.gaps += 1
            else:
                report.first_ticks = packet["ticks"]

            stream.packet_count = packet["count"]
            report.packets += 1
            report.samples += samples
            elems = samples // channels
            report.last_ticks = packet["ticks"] + elems * self.ticks_per_sample("tx")
            for i, q in decoded:
                report.sample_checksum = (report.sample_checksum * 31 + (i & 0xFFFF)) & 0xFFFFFFFF
                report.sample_checksum = (report.sample_checksum * 31 + (q & 0xFFFF)) & 0xFFFFFFFF

            report.after_burst = packet["end_of_burst"]
            if packet["end_of_burst"]:
                report.bursts += 1
                report.end_of_burst_ticks.append(report.last_ticks)

            stream.produced = report.samples // channels
            if packet["ticks"] < arrival_ticks:
                # The board drops a late packet, so it never counts as consumed.
                report.late += 1
                stream.late += 1
                return

            # The status loop holds this back until the timebase reaches the
            # packet's end, which is when the board would free the slot. The
            # board then reports the packet's timestamp plus its sample count.
            stream.frontier = max(stream.frontier, report.last_ticks)
            stream.playing.append((report.last_ticks, packet["ticks"] + elems))

    def generate(self, stream, run_id):
        channels = max(1, len(stream.chans))
        elems = stream.elems_per_packet
        rate = self.rates["rx"]
        ticks_per_sample = self.ticks_per_sample("rx")
        packets_per_second = rate / max(1, elems)
        if self.args.max_pps > 0:
            packets_per_second = min(packets_per_second, self.args.max_pps)
        period = 1.0 / packets_per_second

        tick = stream.start_tick
        count = 0
        sent = 0
        next_send = time.monotonic()

        # A stop and a quick restart must not leave the old run's generator
        # sending alongside the new one.
        while (stream.running.is_set() and not stream.closing.is_set()
               and stream.run_id == run_id):
            if stream.peer is None:
                time.sleep(GENERATOR_IDLE_SLEEP_S)
                next_send = time.monotonic()
                continue
            # A timed start holds the first packet until the timebase gets there,
            # as the packetizer's armed start does.
            if sent == 0 and self.ticks() < stream.start_tick:
                time.sleep(GENERATOR_IDLE_SLEEP_S)
                next_send = time.monotonic()
                continue

            now = time.monotonic()
            if now < next_send:
                time.sleep(min(GENERATOR_IDLE_SLEEP_S, next_send - now))
                continue
            next_send += period

            samples = self.source.block(tick // ticks_per_sample, elems, rate, channels)
            payload = vita.pack(stream.format, samples)
            datagram = vita.build_packet(stream.stream_id, stream.packet_class, count, tick,
                                         payload)
            try:
                stream.socket.sendto(datagram, stream.peer)
            except OSError:
                break

            with stream.lock:
                stream.produced += elems
                stream.consumed += elems
            count = (count + 1) % vita.PACKET_COUNT_MODULUS
            tick += elems * ticks_per_sample
            sent += 1
            if self.args.rx_packets and sent >= self.args.rx_packets:
                break

        log(self.verbose, "generator for stream %d sent %d packets" % (stream.stream_id, sent))

    # Events

    def status_loop(self):
        while not self.stopping.is_set():
            time.sleep(STATUS_PERIOD_S)
            now = time.monotonic()
            with self.lock:
                streams = list(self.streams.values())
                sessions = list(self.sessions.values())
            ticks = self.ticks()
            for session in sessions:
                if not session.alive or now - session.last_status < session.interval_s:
                    continue
                owned = [stream for stream in streams if stream.owner is session]
                if not owned:
                    continue
                session.last_status = now
                statuses = [self.stream_status(stream, ticks) for stream in owned]
                session.events.put(control.from_dict(
                    "SessionEvent", {"status": {"ticks": ticks, "streams": statuses}}))

    def stream_status(self, stream, ticks):
        with stream.lock:
            if stream.direction == "tx":
                # A transmit packet is consumed when the timebase reaches its end.
                while stream.playing and stream.playing[0][0] <= ticks:
                    stream.consumed = max(stream.consumed, stream.playing.popleft()[1])
                # Like the PL, count every sample period an open burst had
                # nothing to send.
                open_burst = stream.report.packets and not stream.report.after_burst
                frontier = max(stream.frontier, stream.underrun_mark)
                if stream.running.is_set() and open_burst and ticks > frontier:
                    stream.underrun += (ticks - frontier) // self.ticks_per_sample("tx")
                    stream.underrun_mark = ticks
            consumed = stream.consumed
            return {
                "stream_id": stream.stream_id,
                "produced": stream.produced,
                "consumed": consumed,
                "late": stream.late,
                "underrun": stream.underrun,
                "overflow": stream.overflow,
                "gaps": stream.gaps,
            }

    # Servers

    def discovery_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((self.bind, self.args.discovery_port))
        self.discovery_port = sock.getsockname()[1]
        self.ready.set()
        reply = control.from_dict("DiscoveryReply", {
            "name": self.args.name,
            "serial": self.args.serial,
            "version": "fake",
            "control_port": self.control_port,
            "mac": self.args.mac,
        }).SerializeToString()
        while not self.stopping.is_set():
            try:
                datagram, peer = sock.recvfrom(1024)
            except OSError:
                break
            if datagram.strip() == vita_probe():
                sock.sendto(reply, peer)
                log(self.verbose, "discovery probe from %s:%d" % peer)

    def unary(self, op, request, context):
        _, _, reply_name, needs_session = control.RPCS[op]
        session = None
        if needs_session:
            session_id = dict(context.invocation_metadata()).get(control.SESSION_METADATA_KEY)
            with self.lock:
                session = self.sessions.get(session_id)
            if session is None:
                context.abort(grpc.StatusCode.FAILED_PRECONDITION,
                              "this call needs `%s` metadata from an open Session"
                              % control.SESSION_METADATA_KEY)
        try:
            result = getattr(self, "op_" + op)(session, control.to_dict(request))
        except (ValueError, KeyError) as error:
            log(self.verbose, "error on %s: %s" % (op, error))
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(error))
        return control.from_dict(reply_name, result)

    def tx_report(self, request, context):
        return self.op_tx_report(None, request or {})

    def session(self, commands, context):
        try:
            first = next(commands)
        except StopIteration:
            return
        if first.WhichOneof("command") != "start":
            context.abort(grpc.StatusCode.INVALID_ARGUMENT,
                          "the first Session message must be SessionStart")
        with self.lock:
            self.next_session_id += 1
            session = Session(str(self.next_session_id))
            self.sessions[session.session_id] = session
        log(self.verbose, "session %s opened by %s" % (session.session_id, first.start.client))

        def read_commands():
            try:
                for command in commands:
                    if command.WhichOneof("command") == "set_status_interval":
                        interval_ms = command.set_status_interval.interval_ms
                        if not MIN_STATUS_INTERVAL_MS <= interval_ms <= MAX_STATUS_INTERVAL_MS:
                            session.events.put(grpc.StatusCode.OUT_OF_RANGE)
                            break
                        session.interval_s = interval_ms / 1000.0
            except grpc.RpcError:
                pass
            session.alive = False

        threading.Thread(target=read_commands, daemon=True).start()
        try:
            yield control.from_dict("SessionEvent", {"opened": {
                "session_id": session.session_id,
                "status_interval_ms": DEFAULT_STATUS_INTERVAL_MS}})
            while session.alive and context.is_active() and not self.stopping.is_set():
                try:
                    event = session.events.get(timeout=SESSION_POLL_S)
                except queue.Empty:
                    continue
                if isinstance(event, grpc.StatusCode):
                    context.abort(event, "status interval is outside %d..=%d ms"
                                  % (MIN_STATUS_INTERVAL_MS, MAX_STATUS_INTERVAL_MS))
                yield event
        finally:
            session.alive = False
            with self.lock:
                self.sessions.pop(session.session_id, None)
                owned = [s for s in self.streams.values() if s.owner is session]
            # The board daemon closes a session's streams when it ends.
            for stream in owned:
                self.op_close_stream(session, {"stream_id": stream.stream_id})
            log(self.verbose, "session %s closed" % session.session_id)

    def handler(self):
        daemon = self
        serialize = lambda message: message.SerializeToString()  # noqa: E731
        handlers = {
            control.method("Session"): grpc.stream_stream_rpc_method_handler(
                self.session,
                request_deserializer=control.MESSAGES["SessionCommand"].FromString,
                response_serializer=serialize),
            control.FAKE_TX_REPORT_METHOD: grpc.unary_unary_rpc_method_handler(
                self.tx_report,
                request_deserializer=lambda data: json.loads(data or b"{}"),
                response_serializer=lambda value: json.dumps(value).encode("utf-8")),
        }
        for op, (rpc, request_name, _, _) in control.RPCS.items():
            handlers[control.method(rpc)] = grpc.unary_unary_rpc_method_handler(
                lambda request, context, op=op: daemon.unary(op, request, context),
                request_deserializer=control.MESSAGES[request_name].FromString,
                response_serializer=serialize)

        class Handler(grpc.GenericRpcHandler):
            def service(self, details):
                return handlers.get(details.method)

        return Handler()

    def run(self):
        server = grpc.server(futures.ThreadPoolExecutor(max_workers=GRPC_WORKERS))
        server.add_generic_rpc_handlers((self.handler(),))
        self.control_port = server.add_insecure_port("%s:%d" % (self.bind, self.args.control_port))
        server.start()

        threading.Thread(target=self.discovery_loop, daemon=True).start()
        threading.Thread(target=self.status_loop, daemon=True).start()
        self.ready.wait(timeout=5.0)

        print("fake_daemon listening on %s:%d discovery=%d serial=%s rate=%.0f"
              % (self.bind, self.control_port, self.discovery_port, self.args.serial, self.rates["rx"]),
              flush=True)

        try:
            while not self.stopping.is_set():
                self.stopping.wait(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.stopping.set()
            server.stop(grace=None)


def vita_probe():
    return b"plutosky-stream?"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bind", default="127.0.0.1", help="address to serve on")
    parser.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT)
    parser.add_argument("--discovery-port", type=int, default=DEFAULT_DISCOVERY_PORT)
    parser.add_argument("--serial", default="FAKE0001")
    parser.add_argument("--name", default="plutosky-stream")
    parser.add_argument("--mac", default="02:00:00:00:00:01")
    parser.add_argument("--channels", type=int, default=DEFAULT_CHANNELS)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--window", type=int, default=DEFAULT_WINDOW_PACKETS,
                        help="packets the host may keep outstanding on transmit")
    parser.add_argument("--max-pps", type=float, default=DEFAULT_MAX_PPS,
                        help="cap on generated packets per second, 0 to send at the sample rate")
    parser.add_argument("--rx-packets", type=int, default=0,
                        help="stop a receive stream after this many packets, 0 for no limit")
    parser.add_argument("--tone-hz", type=float, default=DEFAULT_TONE_HZ)
    parser.add_argument("--amplitude", type=float, default=DEFAULT_TONE_AMPLITUDE)
    parser.add_argument("--source", default=None,
                        help="npy or 16-bit stereo wav file to stream instead of a tone")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    FakeDaemon(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
