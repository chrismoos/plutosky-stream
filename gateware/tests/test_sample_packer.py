"""The packer and unpacker against the wire-format reference, byte for byte."""

import random

import pytest
from migen import Module, run_simulation

from cores.sample_packer import SamplePacker, SampleUnpacker
from tests import golden

FORMATS = [golden.FORMAT_CS12, golden.FORMAT_CS8, golden.FORMAT_BFP8]
FORMAT_NAMES = {golden.FORMAT_CS12: "cs12", golden.FORMAT_CS8: "cs8",
                golden.FORMAT_BFP8: "bfp8"}


class PackerDut(Module):
    def __init__(self):
        self.submodules.dut = SamplePacker()


class UnpackerDut(Module):
    def __init__(self):
        self.submodules.dut = SampleUnpacker()


def random_samples(count, seed, full_scale=False):
    rng = random.Random(seed)
    if full_scale:
        return [(rng.randrange(-2048, 2048), rng.randrange(-2048, 2048))
                for _ in range(count)]
    return [(rng.randrange(-600, 600), rng.randrange(-600, 600)) for _ in range(count)]


def pack_through_rtl(fmt, samples, sink_gap=0, source_gap=0, limit=200000):
    """Run the samples through ``SamplePacker`` and collect the payload bytes."""
    dut = PackerDut()
    out = bytearray()

    def stim():
        yield dut.dut.fmt.eq(fmt)
        yield dut.dut.clear.eq(1)
        yield
        yield dut.dut.clear.eq(0)
        yield
        index = 0
        done = False
        cycle = 0
        while not done and cycle < limit:
            cycle += 1
            feeding = index < len(samples) and (sink_gap == 0 or cycle % (sink_gap + 1))
            if feeding:
                i, q = samples[index]
                yield dut.dut.sink.i.eq(i & 0xFFF)
                yield dut.dut.sink.q.eq(q & 0xFFF)
                yield dut.dut.sink.last.eq(1 if index == len(samples) - 1 else 0)
                yield dut.dut.sink.valid.eq(1)
            else:
                yield dut.dut.sink.valid.eq(0)
            yield dut.dut.source.ready.eq(
                1 if source_gap == 0 or cycle % (source_gap + 1) else 0)
            yield
            if feeding and (yield dut.dut.sink.ready):
                index += 1
            if (yield dut.dut.source.valid) and (yield dut.dut.source.ready):
                word = yield dut.dut.source.data
                n = yield dut.dut.source.bytes
                out.extend(word.to_bytes(4, "little")[:n])
                if (yield dut.dut.source.last):
                    done = True
        assert done, "packer never closed the packet"

    run_simulation(dut, [stim()])
    return bytes(out)


def unpack_through_rtl(fmt, payload, count, sink_gap=0, source_gap=0, limit=200000):
    """Run payload bytes through ``SampleUnpacker`` and collect the samples."""
    dut = UnpackerDut()
    out = []

    def stim():
        yield dut.dut.fmt.eq(fmt)
        yield dut.dut.clear.eq(1)
        yield
        yield dut.dut.clear.eq(0)
        yield
        pos = 0
        cycle = 0
        while len(out) < count and cycle < limit:
            cycle += 1
            feeding = pos < len(payload) and (sink_gap == 0 or cycle % (sink_gap + 1))
            n = min(4, len(payload) - pos)
            if feeding:
                yield dut.dut.sink.data.eq(
                    int.from_bytes(payload[pos:pos + n].ljust(4, b"\x00"), "little"))
                yield dut.dut.sink.bytes.eq(n)
                yield dut.dut.sink.valid.eq(1)
            else:
                yield dut.dut.sink.valid.eq(0)
            yield dut.dut.source.ready.eq(
                1 if source_gap == 0 or cycle % (source_gap + 1) else 0)
            yield
            if feeding and (yield dut.dut.sink.ready):
                pos += n
            if (yield dut.dut.source.valid) and (yield dut.dut.source.ready):
                i = yield dut.dut.source.i
                q = yield dut.dut.source.q
                out.append((golden.from_twos(i), golden.from_twos(q)))
        assert len(out) == count, f"unpacker produced {len(out)} of {count} samples"

    run_simulation(dut, [stim()])
    return out


@pytest.mark.parametrize("fmt", FORMATS, ids=FORMAT_NAMES.get)
def test_packer_matches_the_reference(fmt):
    samples = random_samples(64, seed=fmt, full_scale=True)
    assert pack_through_rtl(fmt, samples) == golden.pack(fmt, samples)


@pytest.mark.parametrize("fmt", FORMATS, ids=FORMAT_NAMES.get)
def test_packer_matches_the_reference_under_back_pressure(fmt):
    samples = random_samples(48, seed=fmt + 100, full_scale=True)
    assert pack_through_rtl(fmt, samples, sink_gap=2, source_gap=3) == golden.pack(fmt, samples)


@pytest.mark.parametrize("fmt", FORMATS, ids=FORMAT_NAMES.get)
def test_unpacker_recovers_the_samples(fmt):
    samples = random_samples(64, seed=fmt + 7, full_scale=True)
    payload = golden.pack(fmt, samples)
    recovered = unpack_through_rtl(fmt, payload, len(samples))
    assert recovered == golden.unpack(fmt, payload, len(samples))


@pytest.mark.parametrize("fmt", FORMATS, ids=FORMAT_NAMES.get)
def test_unpacker_recovers_the_samples_under_back_pressure(fmt):
    samples = random_samples(48, seed=fmt + 11, full_scale=True)
    payload = golden.pack(fmt, samples)
    recovered = unpack_through_rtl(fmt, payload, len(samples), sink_gap=1, source_gap=2)
    assert recovered == golden.unpack(fmt, payload, len(samples))


@pytest.mark.parametrize("fmt", [golden.FORMAT_CS12, golden.FORMAT_CS8],
                         ids=FORMAT_NAMES.get)
def test_lossless_formats_round_trip(fmt):
    samples = random_samples(32, seed=fmt + 21, full_scale=True)
    payload = pack_through_rtl(fmt, samples)
    recovered = unpack_through_rtl(fmt, payload, len(samples))
    if fmt == golden.FORMAT_CS12:
        assert recovered == samples
    else:
        assert recovered == [(i & ~0xF, q & ~0xF) for i, q in samples]


def test_packer_handles_a_short_odd_packet():
    # cs12 is three bytes a sample, so an odd count leaves a partial output beat.
    samples = random_samples(5, seed=99, full_scale=True)
    assert pack_through_rtl(golden.FORMAT_CS12, samples) == golden.pack(
        golden.FORMAT_CS12, samples)


BFP_EDGE_BLOCKS = {
    "all zero": [(0, 0)] * 16,
    "exponent zero": [(n - 8, 8 - n) for n in range(16)],
    "exponent one boundary": [(127, -127)] + [(0, 0)] * 15,
    "first value needing e=1": [(128, -128)] + [(1, -1)] * 15,
    "round half away up": [(129, -129)] + [(0, 0)] * 15,
    "positive full scale clamps": [(2047, 2047)] + [(0, 0)] * 15,
    "negative full scale reaches e=5": [(-2048, -2048)] + [(0, 0)] * 15,
    "negative full scale with a peer": [(-2048, 2047)] + [(1000, -1000)] * 15,
    "mixed magnitudes": [(2047, -2048), (1023, -1024), (511, -512), (255, -256),
                         (127, -128), (63, -64), (31, -32), (15, -16),
                         (7, -8), (3, -4), (1, -2), (0, -1),
                         (2046, 2046), (-2047, -2047), (1, 1), (-1, -1)],
}


@pytest.mark.parametrize("name", sorted(BFP_EDGE_BLOCKS))
def test_bfp8_exponent_and_rounding_edges(name):
    samples = BFP_EDGE_BLOCKS[name]
    expected = golden.pack(golden.FORMAT_BFP8, samples)
    assert pack_through_rtl(golden.FORMAT_BFP8, samples) == expected
    assert unpack_through_rtl(golden.FORMAT_BFP8, expected, len(samples)) == \
        golden.unpack(golden.FORMAT_BFP8, expected, len(samples))


def test_bfp8_covers_every_exponent_exhaustively_in_the_reference():
    # One block per magnitude decade pins the exponent boundaries in the model
    # the RTL is compared against.
    seen = set()
    for peak in (0, 1, 63, 64, 127, 128, 255, 256, 511, 512, 1023, 1024, 2047, -2048):
        block = [(peak, -peak)] + [(0, 0)] * 15
        seen.add(golden.bfp_exponent(block))
    assert seen == {0, 1, 2, 3, 4, 5}


@pytest.mark.parametrize("fmt", FORMATS, ids=FORMAT_NAMES.get)
def test_two_channels_interleave_before_packing(fmt):
    per_channel = 32
    ch0 = random_samples(per_channel, seed=fmt + 31, full_scale=True)
    ch1 = random_samples(per_channel, seed=fmt + 32, full_scale=True)
    stream_samples = golden.interleave([ch0, ch1])
    payload = pack_through_rtl(fmt, stream_samples)
    assert payload == golden.pack(fmt, stream_samples)
    recovered = golden.unpack(fmt, payload, len(stream_samples))
    back0, back1 = golden.deinterleave(recovered, 2)
    if fmt == golden.FORMAT_CS12:
        assert back0 == ch0 and back1 == ch1
