"""The packet tools against the fake daemon and against a synthetic capture."""

import os
import re
import shutil
import struct
import subprocess
import sys

import pytest
from google.protobuf import descriptor_pb2

import control
import vita

BENCH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vrt_bench.py")
DUMP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vrt_dump.py")

PCAP_MAGIC = 0xA1B2C3D4
PCAP_VERSION_MAJOR = 2
PCAP_VERSION_MINOR = 4
PCAP_SNAPLEN = 65535
LINKTYPE_ETHERNET = 1
IP_PROTOCOL_UDP = 17
IPV4_VERSION_IHL = 0x45
IPV4_TTL = 64
BENCH_SECONDS = 1.0
BENCH_RATE_HZ = 1000000
BENCH_ATTEMPTS = 3
RESULT_LINE = re.compile(r"^RESULT (?P<fields>.*)$", re.MULTILINE)
CAPTURE_PACKETS = 6


def ethernet_frame(payload, sport=31000, dport=31001):
    udp = struct.pack(">HHHH", sport, dport, 8 + len(payload), 0) + payload
    ip = struct.pack(">BBHHHBBH4s4s", IPV4_VERSION_IHL, 0, 20 + len(udp), 0, 0, IPV4_TTL,
                     IP_PROTOCOL_UDP, 0, bytes([192, 168, 2, 1]),
                     bytes([192, 168, 2, 10])) + udp
    return b"\x02\x00\x00\x00\x00\x01\x02\x00\x00\x00\x00\x02\x08\x00" + ip


@pytest.fixture(scope="module")
def capture(tmp_path_factory):
    """A pcap holding one packet of each wire format, the last ending a burst."""
    path = tmp_path_factory.mktemp("capture") / "stream.pcap"
    blob = struct.pack("<IHHiIII", PCAP_MAGIC, PCAP_VERSION_MAJOR, PCAP_VERSION_MINOR, 0, 0,
                       PCAP_SNAPLEN, LINKTYPE_ETHERNET)
    tick = 0
    for n in range(CAPTURE_PACKETS):
        fmt = vita.FORMATS[n % len(vita.FORMATS)]
        count = vita.BFP8_BLOCK_SAMPLES if fmt == "bfp8" else 8
        samples = [(100 + n + k, -(100 + n + k)) for k in range(count)]
        packet = vita.build_packet(0xA001, vita.PACKET_CLASS_BY_FORMAT[fmt], n, tick,
                                   vita.pack(fmt, samples),
                                   end_of_burst=(n == CAPTURE_PACKETS - 1))
        tick += count
        frame = ethernet_frame(packet)
        blob += struct.pack("<IIII", 1700000000 + n, n * 1000, len(frame), len(frame)) + frame
    path.write_bytes(blob)
    return str(path)


def test_vrt_dump_reads_a_capture(capture):
    completed = subprocess.run([sys.executable, DUMP, "--pcap", capture, "--samples", "2"],
                               capture_output=True, text=True, check=True)
    text = completed.stdout
    assert "SUMMARY packets=%d" % CAPTURE_PACKETS in text
    assert "sequence_gaps=0" in text
    assert "malformed=0" in text
    assert "bursts=1" in text
    for fmt in vita.FORMATS:
        assert fmt in text, "no %s packet in the dump\n%s" % (fmt, text)
    assert "END_BURST" in text


def test_vrt_dump_writes_samples(capture, tmp_path):
    npy = tmp_path / "samples.npy"
    wav = tmp_path / "samples.wav"
    subprocess.run([sys.executable, DUMP, "--pcap", capture, "--quiet",
                    "--npy", str(npy), "--wav", str(wav)], capture_output=True, text=True,
                   check=True)

    import numpy
    import wave

    data = numpy.load(str(npy))
    assert data.dtype == numpy.complex64
    assert len(data) == 4 * 8 + 2 * vita.BFP8_BLOCK_SAMPLES
    with wave.open(str(wav)) as handle:
        assert handle.getnchannels() == 2
        assert handle.getsampwidth() == 2
        assert handle.getnframes() == len(data)


def test_vrt_dump_filters_by_stream_id(capture):
    completed = subprocess.run(
        [sys.executable, DUMP, "--pcap", capture, "--quiet", "--stream-id", "0xbeef"],
        capture_output=True, text=True)
    assert "SUMMARY packets=0" in completed.stdout


@pytest.mark.parametrize("wire,mtu", [("cs12", 1500), ("cs8", 1500), ("bfp8", 9000)])
def test_vrt_bench_measures_a_stream(daemon, wire, mtu):
    # The fake daemon is shared by the whole session, so the rate is set here
    # rather than left at whatever an earlier test chose. A run in which the
    # bench's own socket dropped datagrams, which the kernel counts, says
    # nothing about the stream and is repeated.
    for _ in range(BENCH_ATTEMPTS):
        completed = subprocess.run(
            [sys.executable, BENCH, "--addr", daemon.addr, "--port", str(daemon.port),
             "--wire", wire, "--mtu", str(mtu), "--secs", str(BENCH_SECONDS),
             "--rate", str(BENCH_RATE_HZ)],
            capture_output=True, text=True, check=True)
        result = RESULT_LINE.search(completed.stdout)
        assert result is not None, completed.stdout
        fields = {}
        for token in result.group("fields").split():
            key, _, value = token.partition("=")
            fields[key] = value
        if fields["socket_drops"] in ("0", "n/a"):
            break
    else:
        pytest.fail("the bench's socket dropped datagrams in each of %d runs" % BENCH_ATTEMPTS)

    assert int(fields["packets"]) > 0
    assert int(fields["samples"]) > 0
    assert int(fields["gaps"]) == 0
    assert int(fields["lost"]) == 0
    assert int(fields["time_jumps"]) == 0
    assert int(fields["malformed"]) == 0
    assert float(fields["pps"]) > 0
    assert "cpu" in fields


def test_control_rejects_an_unknown_operation(daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        with pytest.raises(control.ControlError):
            client.request("set_warp")


def test_control_rejects_a_bad_enum_value(daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        with pytest.raises(control.ControlError):
            client.request("get_gain", dir="sideways", chan=0)


def test_sample_rate_is_locked_while_a_stream_runs(daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        stream = client.request("open_stream", dir="rx", chans=[0], format="cs12", mtu=1500,
                                host_ip=daemon.addr, host_port=0)
        client.request("start_stream", stream_id=stream["stream_id"])
        try:
            with pytest.raises(control.ControlError):
                client.request("set_sample_rate", hz=30720000)
        finally:
            client.request("stop_stream", stream_id=stream["stream_id"])
            client.request("close_stream", stream_id=stream["stream_id"])


def test_status_events_arrive_for_an_open_stream(daemon):
    with control.ControlClient(daemon.addr, daemon.port, client="pytest") as client:
        stream = client.request("open_stream", dir="rx", chans=[0], format="cs12", mtu=1500,
                                host_ip=daemon.addr, host_port=0)
        client.request("start_stream", stream_id=stream["stream_id"])
        events = []
        for _ in range(20):
            events.extend(client.drain_events(timeout=0.2))
            if events:
                break
        client.request("stop_stream", stream_id=stream["stream_id"])
        client.request("close_stream", stream_id=stream["stream_id"])

    assert events, "no status events arrived"
    status = events[0]
    assert status["event"] == "status"
    assert status["stream_id"] == stream["stream_id"]
    for key in ("produced", "consumed", "late", "underrun", "overflow", "gaps", "ticks"):
        assert key in status


def test_the_compiled_descriptor_matches_the_proto(tmp_path):
    protoc = shutil.which("protoc")
    if protoc is None:
        pytest.skip("protoc is not installed")
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    out = tmp_path / "control.desc"
    subprocess.run([protoc, "-I", os.path.join(root, "proto"), "--descriptor_set_out", str(out),
                    "plutosky/v1/control.proto"], check=True)
    fresh = descriptor_pb2.FileDescriptorSet.FromString(out.read_bytes())
    with open(control.DESCRIPTOR_PATH, "rb") as handle:
        shipped = descriptor_pb2.FileDescriptorSet.FromString(handle.read())
    assert fresh == shipped, "tools/control.desc is stale: run tools/gen_proto.sh"
