"""Transmit release timing: on time, late, gaps and end of burst."""

import pytest
from migen import ClockDomain, ClockDomainsRenamer, If, Module, Signal, passive, run_simulation
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import AxiHpReader, ReadRegionArbiter
from cores.tx_depacketizer import TxDepacketizer
from tests import golden
from tests.axi_model import AxiMemory, read_generator, write_generator

MEM_BASE = 0x2000_0000
MEM_SIZE = 64 * 1024
SLOT_SIZE = 2048
RX_OFFSET = 0
STREAM_ID = 0x0BAD_C0DE
FIRST_TICK = 0x4_0000_0010
TICK_PERIOD = 4

FORMAT_NAMES = {golden.FORMAT_CS12: "cs12", golden.FORMAT_CS8: "cs8",
                golden.FORMAT_BFP8: "bfp8"}


class Harness(Module):
    def __init__(self, sample_fifo_depth=512, stream_cd="sys"):
        self.clock_domains.cd_rfic = ClockDomain()
        if stream_cd != "sys":
            self.clock_domains.cd_stream = ClockDomain()
        self.axi = AXIInterface(data_width=64, address_width=32, id_width=6,
                                version="axi3")
        self.tick = Signal()
        self.count = Signal(64, reset=FIRST_TICK)
        self.pool_base = Signal(32, reset=MEM_BASE)
        self.slot_size = Signal(32, reset=SLOT_SIZE)
        self.ring_offset = Signal(32, reset=RX_OFFSET)

        self.submodules.reader = reader = ClockDomainsRenamer(stream_cd)(AxiHpReader(self.axi))
        self.submodules.dut = dut = TxDepacketizer(
            self.count, self.tick, self.pool_base, self.slot_size, self.ring_offset,
            sample_fifo_depth=sample_fifo_depth, stream_cd=stream_cd)
        self.submodules.arbiter = arbiter = ClockDomainsRenamer(stream_cd)(ReadRegionArbiter(reader, 1))
        self.comb += [
            dut.cmd.connect(arbiter.cmd[0]),
            arbiter.source[0].connect(dut.source),
        ]
        self.sync.rfic += If(self.tick, self.count.eq(self.count + 1))


def slot_address(slot):
    return MEM_BASE + RX_OFFSET + slot * SLOT_SIZE + 2


def make_packet(fmt, samples, packet_count, stamp, end_of_burst=False):
    frame_len = golden.frame_len_for(fmt, len(samples), end_of_burst)
    template = golden.make_template(fmt=fmt, stream_id=STREAM_ID, frame_len=frame_len)
    frame = golden.build_frame(template, fmt, samples, packet_count, stamp,
                               frame_len, end_of_burst)
    return frame, frame_len


def program(dut, fmt, mask=0b01, stream_id=STREAM_ID):
    yield dut.dut.stream_id.storage.eq(stream_id)
    yield dut.dut.ctrl.storage.eq(1 | (fmt << 2) | (mask << 4))
    yield
    yield


def submit(dut, slot, length):
    yield dut.dut.submit.storage.eq(slot | (length << 16))
    yield dut.dut.submit.re.eq(1)
    yield
    yield dut.dut.submit.re.eq(0)
    yield


@passive
def air_recorder(dut, out, channels=1):
    """Record what leaves for the radio, one entry per sample period."""
    while True:
        while not (yield dut.tick):
            yield
        yield
        i0 = yield dut.dut.ch0_i
        q0 = yield dut.dut.ch0_q
        if channels == 2:
            yield
            i1 = yield dut.dut.ch1_i
            q1 = yield dut.dut.ch1_q
            out.append(((golden.from_twos(i0), golden.from_twos(q0)),
                        (golden.from_twos(i1), golden.from_twos(q1))))
        else:
            out.append((golden.from_twos(i0), golden.from_twos(q0)))


def run_stream(fmt, packets, channels=1, cycles=4000, stream_id=STREAM_ID,
               sample_fifo_depth=512):
    """``packets`` is a list of (slot, samples, packet_count, stamp, eob)."""
    dut = Harness(sample_fifo_depth=sample_fifo_depth)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    lengths = []
    for slot, samples, count, stamp, eob in packets:
        frame, frame_len = make_packet(fmt, samples, count, stamp, eob)
        memory.load(slot_address(slot), frame)
        lengths.append(frame_len)

    air = []
    state = {}

    def stim():
        mask = 0b11 if channels == 2 else 0b01
        yield from program(dut, fmt, mask, stream_id)
        for (slot, _, _, _, _), length in zip(packets, lengths):
            yield from submit(dut, slot, length)
        for _ in range(cycles):
            yield
        state["late"] = yield dut.dut.late.status
        state["underrun"] = yield dut.dut.underrun.status
        state["gaps"] = yield dut.dut.gaps.status
        state["bad_id"] = yield dut.dut.bad_id.status
        state["consumed"] = yield dut.dut.consumed_count.status
        state["fifo_level"] = yield dut.dut.fifo_level.status

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [ticker(), air_recorder(dut, air, channels)]},
                   clocks={"sys": 10, "rfic": 10})
    return air, state


# A short packet's sample count is only recoverable from its padded size if the
# count is the largest one that size can hold. That rules out odd counts for
# cs8 and counts three modulo four for cs12.
RECOVERABLE_COUNT = {golden.FORMAT_CS12: 34, golden.FORMAT_CS8: 34,
                     golden.FORMAT_BFP8: 32}


def ramp(count, start=1, step=7):
    return [((start + n * step) % 1000, -((start + n * step) % 1000))
            for n in range(count)]


def find_start(air, first):
    """Index of the first recorded period that is not silence."""
    for n, value in enumerate(air):
        if value != (0, 0):
            return n
    raise AssertionError(f"nothing was transmitted, expected {first}")


@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_a_packet_is_released_at_its_sample_count(fmt):
    samples = ramp(RECOVERABLE_COUNT[fmt])
    stamp = FIRST_TICK + 40
    air, state = run_stream(fmt, [(0, samples, 0, stamp, True)])

    expected = golden.unpack(fmt, golden.pack(fmt, samples), len(samples))
    start = find_start(air, expected[0])
    assert air[start] == expected[0]
    assert air[start:start + len(expected)] == expected
    # The recording begins with the period whose count is FIRST_TICK, so the
    # packet's first sample lands exactly at its own stamp.
    assert start == stamp - FIRST_TICK
    assert state["late"] == 0
    assert state["consumed"] == 1


def test_a_packet_whose_time_has_passed_is_dropped_and_counted():
    samples = ramp(34)
    air, state = run_stream(golden.FORMAT_CS8,
                            [(0, samples, 0, FIRST_TICK - 5, True)])
    assert state["late"] == 1
    assert all(value == (0, 0) for value in air), "a late packet must stay off the air"
    assert state["consumed"] == 1


def test_consumed_interrupt_clears_after_slot_is_reclaimed():
    dut = Harness()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    frame, frame_len = make_packet(golden.FORMAT_CS8, ramp(34), 0,
                                   FIRST_TICK - 5, True)
    memory.load(slot_address(0), frame)
    memory.load(slot_address(1), frame)

    def stim():
        yield from program(dut, golden.FORMAT_CS8)
        yield dut.dut.irq_threshold.storage.eq(2)
        yield from submit(dut, 0, frame_len)
        for _ in range(4000):
            if (yield dut.dut.consumed_count.status):
                break
            yield
        else:
            pytest.fail("depacketizer did not reclaim the slot")
        assert (yield dut.dut.irq) == 0
        yield from submit(dut, 1, frame_len)
        for _ in range(4000):
            if (yield dut.dut.consumed_count.status) == 2:
                break
            yield
        else:
            pytest.fail("depacketizer did not reclaim both slots")
        assert (yield dut.dut.irq) == 1
        yield dut.dut.consumed.we.eq(1)
        yield
        yield dut.dut.consumed.we.eq(0)
        yield
        assert (yield dut.dut.consumed_count.status) == 1
        assert (yield dut.dut.irq) == 0

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [ticker()]},
                   clocks={"sys": 10, "rfic": 10})


def test_full_cs12_packets_leave_no_samples_after_late_drop():
    packet_count = 24
    samples = ramp(481)
    packets = [(slot, samples, slot % 16, FIRST_TICK - 5, True)
               for slot in range(packet_count)]
    air, state = run_stream(golden.FORMAT_CS12, packets, cycles=20000,
                            sample_fifo_depth=2048)
    assert state["consumed"] == packet_count
    assert state["late"] == packet_count
    assert state["fifo_level"] == 0
    assert all(value == (0, 0) for value in air)


def test_prefetch_holds_more_than_four_packets_before_playback():
    samples = ramp(34)
    packets = [(slot, samples, slot % 16, FIRST_TICK + 5000 + slot * len(samples), False)
               for slot in range(12)]
    _, state = run_stream(golden.FORMAT_CS8, packets, cycles=8000,
                          sample_fifo_depth=4096)
    assert state["consumed"] == len(packets)
    assert state["fifo_level"] == len(packets) * len(samples)
    assert state["late"] == 0
    assert state["underrun"] == 0


def test_a_gap_between_packets_is_filled_with_zeros():
    first = ramp(18, start=100)
    second = ramp(18, start=300)
    gap = 24
    air, state = run_stream(golden.FORMAT_CS8, [
        (0, first, 0, FIRST_TICK + 30, False),
        (1, second, 1, FIRST_TICK + 30 + len(first) + gap, True),
    ])
    expect_first = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, first), 18)
    expect_second = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, second), 18)
    start = find_start(air, expect_first[0])
    assert air[start:start + 18] == expect_first
    assert air[start + 18:start + 18 + gap] == [(0, 0)] * gap
    assert air[start + 18 + gap:start + 36 + gap] == expect_second
    assert state["late"] == 0
    assert state["gaps"] == 0


def test_end_of_burst_stops_the_stream_and_the_next_packet_starts_a_new_one():
    first = ramp(18, start=100)
    second = ramp(18, start=500)
    quiet = 40
    # The first stamp leaves the fetch the same room the other tests do. A
    # frame takes a few dozen cycles to read, parse and unpack before its
    # packet can be released.
    lead = 40
    air, state = run_stream(golden.FORMAT_CS8, [
        (0, first, 0, FIRST_TICK + lead, True),
        (1, second, 1, FIRST_TICK + lead + len(first) + quiet, True),
    ])
    expect_first = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, first), 18)
    expect_second = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, second), 18)
    start = find_start(air, expect_first[0])
    assert air[start:start + 18] == expect_first
    assert air[start + 18:start + 18 + quiet] == [(0, 0)] * quiet
    assert air[start + 18 + quiet:start + 36 + quiet] == expect_second
    # The burst ended, so the quiet stretch is not counted as starvation.
    assert state["underrun"] == 0


def test_a_frame_for_another_stream_is_dropped_and_counted():
    samples = ramp(18)
    air, state = run_stream(golden.FORMAT_CS8,
                            [(0, samples, 0, FIRST_TICK + 30, True)],
                            stream_id=STREAM_ID + 1)
    assert state["bad_id"] == 1
    assert all(value == (0, 0) for value in air)
    assert state["consumed"] == 1


def test_a_break_in_the_packet_count_is_counted():
    first = ramp(18, start=100)
    second = ramp(18, start=300)
    air, state = run_stream(golden.FORMAT_CS8, [
        (0, first, 0, FIRST_TICK + 30, False),
        (1, second, 5, FIRST_TICK + 60, True),
    ])
    assert state["gaps"] == 1


def test_two_channels_leave_interleaved():
    ch0 = ramp(18, start=11, step=5)
    ch1 = ramp(18, start=900, step=3)
    samples = golden.interleave([ch0, ch1])
    stamp = FIRST_TICK + 30
    air, state = run_stream(golden.FORMAT_CS8,
                            [(0, samples, 0, stamp, True)], channels=2)
    expected = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, samples),
                             len(samples))
    pairs = [(expected[2 * n], expected[2 * n + 1]) for n in range(18)]
    start = next(n for n, v in enumerate(air) if v != ((0, 0), (0, 0)))
    assert air[start:start + 18] == pairs
    assert state["late"] == 0


def test_a_stopped_stream_gives_the_slots_back():
    samples = ramp(18)
    frame, frame_len = make_packet(golden.FORMAT_CS8, samples, 0, FIRST_TICK + 30, True)
    dut = Harness()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    memory.load(slot_address(3), frame)
    state = {}

    def stim():
        yield dut.dut.stream_id.storage.eq(STREAM_ID)
        yield dut.dut.ctrl.storage.eq(0)
        yield
        yield from submit(dut, 3, frame_len)
        for _ in range(200):
            yield
        state["count"] = yield dut.dut.consumed_count.status
        state["slot"] = yield dut.dut.consumed.status

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    run_simulation(dut, {"sys": [stim(), write_generator(dut.axi, memory),
                                 read_generator(dut.axi, memory)],
                         "rfic": [ticker()]},
                   clocks={"sys": 10, "rfic": 10})
    assert state["count"] == 1
    assert state["slot"] == 3


def test_a_late_packet_does_not_take_the_next_one_with_it():
    """The dropped packet's samples must leave the ring before the next plays."""
    late_samples = ramp(34, start=700)
    good = ramp(18, start=100)
    air, state = run_stream(golden.FORMAT_CS8, [
        (0, late_samples, 0, FIRST_TICK - 5, False),
        (1, good, 1, FIRST_TICK + 60, True),
    ])
    expected = golden.unpack(golden.FORMAT_CS8, golden.pack(golden.FORMAT_CS8, good), 18)
    start = find_start(air, expected[0])
    assert state["late"] == 1
    assert air[start:start + 18] == expected, (
        f"the good packet did not survive the dropped one\n got {air[start:start + 18]}")
    assert start == 60, "the good packet must still start at its own count"


@pytest.mark.parametrize("stop_mode", ["playing", "future", "HEADER", "BODY_COMMAND", "PAYLOAD"])
@pytest.mark.parametrize("split", [False, True], ids=["one-clock", "split"])
def test_restart_discards_a_buffered_burst(stop_mode, split):
    stream_cd = "stream" if split else "sys"
    dut = Harness(stream_cd=stream_cd)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    in_flight = stop_mode not in ("playing", "future")
    old = ramp(482 if in_flight else 34, start=700)
    old_lead = 40 if stop_mode == "playing" else 10000
    stop_state = dut.dut.fsm.ongoing(stop_mode if in_flight else "IDLE")
    fresh = ramp(34, start=100)
    old_frame, old_length = make_packet(
        golden.FORMAT_CS8, old, 0, FIRST_TICK + old_lead, True)
    memory.load(slot_address(0), old_frame)
    air = []

    def stim():
        yield from program(dut, golden.FORMAT_CS8)
        yield from submit(dut, 0, old_length)
        if in_flight:
            for _ in range(400):
                if (yield stop_state):
                    break
                yield
            assert (yield stop_state)
        else:
            for _ in range(200):
                yield
            assert (yield dut.dut.consumed_count.status) == 1
        yield dut.dut.ctrl.storage.eq(0)
        for _ in range(1000 if in_flight else 100):
            yield
        assert (yield dut.dut.consumed_count.status) == 1
        air.clear()
        stamp = (yield dut.count) + 100
        frame, length = make_packet(golden.FORMAT_CS8, fresh, 1, stamp, True)
        memory.load(slot_address(1), frame)
        yield from program(dut, golden.FORMAT_CS8)
        yield from submit(dut, 1, length)
        for _ in range(1000):
            yield

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    generators = {"sys": [stim()], "rfic": [ticker(), air_recorder(dut, air)]}
    generators.setdefault(stream_cd, []).extend([
        write_generator(dut.axi, memory), read_generator(dut.axi, memory)])
    clocks = {"sys": 20 if split else 10, "rfic": 16 if split else 10}
    if split:
        clocks["stream"] = 10
    run_simulation(dut, generators, clocks=clocks)
    expected = golden.unpack(golden.FORMAT_CS8,
                             golden.pack(golden.FORMAT_CS8, fresh), len(fresh))
    assert [sample for sample in air if sample != (0, 0)] == expected


def test_a_frame_too_short_to_hold_a_header_is_returned_unread():
    dut = Harness()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    state = {}

    def stim():
        yield from program(dut, golden.FORMAT_CS8, 0b01)
        yield from submit(dut, 5, 60)
        for _ in range(400):
            yield
        state["count"] = yield dut.dut.consumed_count.status
        state["slot"] = yield dut.dut.consumed.status
        state["bad_id"] = yield dut.dut.bad_id.status

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    run_simulation(dut, {"sys": [stim(), write_generator(dut.axi, memory),
                                 read_generator(dut.axi, memory)],
                         "rfic": [ticker()]},
                   clocks={"sys": 10, "rfic": 10})
    assert state["count"] == 1
    assert state["slot"] == 5
    assert state["bad_id"] == 1
    assert memory.reads == [], "a runt frame must not be read from memory"
