"""The AXI HP masters against a memory model."""

import random

import pytest
from migen import Module, passive, run_simulation
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import AxiHpReader, AxiHpWriter
from tests.axi_model import AxiMemory, read_generator, write_generator

MEM_BASE = 0x1000_0000
MEM_SIZE = 64 * 1024


def make_axi():
    return AXIInterface(data_width=64, address_width=32, id_width=6, version="axi3")


class WriterDut(Module):
    def __init__(self, **kwargs):
        self.axi = make_axi()
        self.submodules.dut = AxiHpWriter(self.axi, **kwargs)


class ReaderDut(Module):
    def __init__(self, **kwargs):
        self.axi = make_axi()
        self.submodules.dut = AxiHpReader(self.axi, **kwargs)


def feed_bytes(dut, addr, payload, gap=0):
    """Push ``payload`` through the write master as 1..4 byte beats."""
    yield dut.cmd.addr.eq(addr)
    yield dut.cmd.length.eq(len(payload))
    yield dut.cmd.valid.eq(1)
    while True:
        yield
        if (yield dut.cmd.ready):
            break
    yield dut.cmd.valid.eq(0)

    pos = 0
    while pos < len(payload):
        n = min(4, len(payload) - pos)
        word = int.from_bytes(payload[pos:pos + n].ljust(4, b"\x00"), "little")
        yield dut.sink.data.eq(word)
        yield dut.sink.bytes.eq(n)
        yield dut.sink.valid.eq(1)
        yield
        while not (yield dut.sink.ready):
            yield
        pos += n
        yield dut.sink.valid.eq(0)
        for _ in range(gap):
            yield


def wait_done(dut, limit=20000):
    for _ in range(limit):
        yield
        if (yield dut.done):
            return
    raise AssertionError("write master never reported the region complete")


@pytest.mark.parametrize("offset", [0, 1, 2, 5, 7])
@pytest.mark.parametrize("length", [1, 7, 8, 9, 70, 1500])
def test_write_master_places_every_byte(offset, length):
    dut = WriterDut()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(random.Random(length * 8 + offset).randrange(256) for _ in range(length))
    addr = MEM_BASE + 0x40 + offset

    def stim():
        yield from feed_bytes(dut.dut, addr, payload)
        yield from wait_done(dut.dut)

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory)])
    assert memory[addr:addr + length] == payload
    # Nothing outside the region may be touched.
    assert all(MEM_BASE <= a < MEM_BASE + MEM_SIZE for a in memory.writes)
    assert set(memory.writes) == set(range(addr, addr + length))


def test_write_master_uses_full_bursts_and_several_outstanding():
    dut = WriterDut(max_outstanding=4)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(range(256)) * 8
    addr = MEM_BASE + 2
    lens = []

    @passive
    def watch():
        while True:
            if (yield dut.axi.aw.valid) and (yield dut.axi.aw.ready):
                lens.append((yield dut.axi.aw.len) + 1)
            yield

    def stim():
        yield from feed_bytes(dut.dut, addr, payload)
        yield from wait_done(dut.dut)

    run_simulation(dut, [stim(), watch(),
                         write_generator(dut.axi, memory, b_delay=3),
                         read_generator(dut.axi, memory)])
    assert memory[addr:addr + len(payload)] == payload
    assert lens[0] == 16, "the first burst should be a full INCR16"
    assert sum(lens) * 8 >= len(payload)
    assert memory.max_write_outstanding > 1, "bursts should overlap"
    assert memory.max_write_outstanding <= 4


def test_write_master_survives_back_pressure():
    dut = WriterDut()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(random.Random(3).randrange(256) for _ in range(600))
    addr = MEM_BASE + 0x82

    def stim():
        yield from feed_bytes(dut.dut, addr, payload, gap=2)
        yield from wait_done(dut.dut)

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory,
                                         aw_ready=lambda c: c % 5 != 0,
                                         w_ready=lambda c: c % 3 != 0,
                                         b_delay=5),
                         read_generator(dut.axi, memory)])
    assert memory[addr:addr + len(payload)] == payload


def test_write_master_does_not_cross_a_four_kilobyte_boundary():
    # The model asserts the boundary rule, so reaching the end is the check.
    dut = WriterDut()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(random.Random(9).randrange(256) for _ in range(4000))
    addr = MEM_BASE + 0x1000 - 100

    def stim():
        yield from feed_bytes(dut.dut, addr, payload)
        yield from wait_done(dut.dut)

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory)])
    assert memory[addr:addr + len(payload)] == payload


def drain_bytes(dut, addr, length, ready=None):
    out = bytearray()
    yield dut.cmd.addr.eq(addr)
    yield dut.cmd.length.eq(length)
    yield dut.cmd.valid.eq(1)
    while True:
        yield
        if (yield dut.cmd.ready):
            break
    yield dut.cmd.valid.eq(0)

    cycle = 0
    last = False
    yield dut.source.ready.eq(1)
    while not last and cycle < 40000:
        take = (yield dut.source.ready) and (yield dut.source.valid)
        if take:
            word = yield dut.source.data
            n = yield dut.source.bytes
            last = bool((yield dut.source.last))
            out += word.to_bytes(4, "little")[:n]
        cycle += 1
        yield dut.source.ready.eq(1 if ready is None else (1 if ready(cycle) else 0))
        yield
    assert last, "read master never reported the final beat"
    return bytes(out)


@pytest.mark.parametrize("offset", [0, 2, 3, 7])
@pytest.mark.parametrize("length", [1, 4, 8, 13, 70, 1500])
def test_read_master_returns_every_byte(offset, length):
    dut = ReaderDut()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(random.Random(length * 3 + offset).randrange(256) for _ in range(length))
    addr = MEM_BASE + 0x100 + offset
    memory.load(addr, payload)

    result = []

    def stim():
        result.append((yield from drain_bytes(dut.dut, addr, length)))

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory)])
    assert result[0] == payload


def test_read_master_overlaps_transactions_and_takes_back_pressure():
    dut = ReaderDut(max_outstanding=4)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    payload = bytes(random.Random(11).randrange(256) for _ in range(2048))
    addr = MEM_BASE + 0x202
    memory.load(addr, payload)
    result = []

    def stim():
        result.append((yield from drain_bytes(dut.dut, addr, len(payload),
                                              ready=lambda c: c % 4 != 0)))

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory,
                                        ar_ready=lambda c: c % 7 != 0, latency=6)])
    assert result[0] == payload
    assert memory.max_read_outstanding > 1, "reads should overlap"
    assert memory.max_read_outstanding <= 4


def test_masters_handle_back_to_back_regions():
    dut = WriterDut()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    rng = random.Random(21)
    regions = [(MEM_BASE + 0x400 + n * 0x100 + 2,
                bytes(rng.randrange(256) for _ in range(rng.randrange(20, 200))))
               for n in range(6)]

    def stim():
        for addr, payload in regions:
            yield from feed_bytes(dut.dut, addr, payload)
            yield from wait_done(dut.dut)

    run_simulation(dut, [stim(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory)])
    for addr, payload in regions:
        assert memory[addr:addr + len(payload)] == payload


# What a Gigabit link asks of one direction, and the slowest sys clock line
# rate must still hold at. Together they set the bytes per cycle a master has
# to sustain.
LINE_RATE_BYTES_PER_SECOND = 118e6
FALLBACK_SYS_CLK_HZ = 33.333e6
REQUIRED_BYTES_PER_CYCLE = LINE_RATE_BYTES_PER_SECOND / FALLBACK_SYS_CLK_HZ


def measure_write(length, offset=2, b_delay=4):
    dut = WriterDut()
    memory = AxiMemory(MEM_SIZE * 4, MEM_BASE)
    payload = bytes((n * 7) & 0xFF for n in range(length))
    addr = MEM_BASE + 0x40 + offset
    stats = {"cycles": 0, "beats": 0, "bursts": 0}

    @passive
    def watch():
        counting = False
        while True:
            if (yield dut.dut.cmd.valid) and (yield dut.dut.cmd.ready):
                counting = True
            if counting:
                stats["cycles"] += 1
            if (yield dut.axi.w.valid) and (yield dut.axi.w.ready):
                stats["beats"] += 1
            if (yield dut.axi.aw.valid) and (yield dut.axi.aw.ready):
                stats["bursts"] += 1
            if (yield dut.dut.done):
                counting = False
            yield

    def stim():
        yield from feed_bytes(dut.dut, addr, payload)
        yield from wait_done(dut.dut)

    run_simulation(dut, [stim(), watch(),
                         write_generator(dut.axi, memory, b_delay=b_delay),
                         read_generator(dut.axi, memory)])
    assert memory[addr:addr + length] == payload
    return stats


def measure_read(length, offset=2, latency=6):
    dut = ReaderDut()
    memory = AxiMemory(MEM_SIZE * 4, MEM_BASE)
    payload = bytes((n * 11) & 0xFF for n in range(length))
    addr = MEM_BASE + 0x100 + offset
    memory.load(addr, payload)
    stats = {"cycles": 0, "beats": 0, "bursts": 0}
    out = []

    @passive
    def watch():
        counting = False
        while True:
            if (yield dut.dut.cmd.valid) and (yield dut.dut.cmd.ready):
                counting = True
            if counting:
                stats["cycles"] += 1
            if (yield dut.axi.r.valid) and (yield dut.axi.r.ready):
                stats["beats"] += 1
            if (yield dut.axi.ar.valid) and (yield dut.axi.ar.ready):
                stats["bursts"] += 1
            yield

    def stim():
        out.append((yield from drain_bytes(dut.dut, addr, length)))

    run_simulation(dut, [stim(), watch(),
                         write_generator(dut.axi, memory),
                         read_generator(dut.axi, memory, latency=latency)])
    assert out[0] == payload
    return stats


@pytest.mark.parametrize("length", [1510, 9010], ids=["mtu1500", "mtu9000"])
def test_the_write_master_sustains_the_line_rate_budget(length):
    stats = measure_write(length)
    rate = length / stats["cycles"]
    # Sixteen-beat bursts: a frame costs its beats plus a handful of cycles.
    assert stats["bursts"] == -(-stats["beats"] // 16)
    assert rate >= REQUIRED_BYTES_PER_CYCLE, (
        f"{rate:.2f} bytes a cycle over {stats['cycles']} cycles, "
        f"needs {REQUIRED_BYTES_PER_CYCLE:.2f}")


@pytest.mark.parametrize("length", [1510, 9010], ids=["mtu1500", "mtu9000"])
def test_the_read_master_sustains_the_line_rate_budget(length):
    stats = measure_read(length)
    rate = length / stats["cycles"]
    assert stats["bursts"] == -(-stats["beats"] // 16)
    assert rate >= REQUIRED_BYTES_PER_CYCLE, (
        f"{rate:.2f} bytes a cycle over {stats['cycles']} cycles, "
        f"needs {REQUIRED_BYTES_PER_CYCLE:.2f}")
