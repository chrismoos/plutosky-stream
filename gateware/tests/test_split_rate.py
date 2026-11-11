"""Transmit release against a timebase that counts radio samples.

The release engine runs on the rate filter's transmit tick while the timebase
counts every radio tick, as in the SoC when the transmit rate is a fraction of
the radio rate.
"""

from migen import ClockDomain, If, Module, Signal, passive, run_simulation
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import AxiHpReader, ReadRegionArbiter
from cores.rate_filter import LANES, RateFilter
from cores.rfic_stream import MODE_2R2T
from cores.sample_packer import SAMPLE_BITS
from cores.tx_depacketizer import TxDepacketizer
from tests import golden
from tests.axi_model import AxiMemory, read_generator, write_generator
from tests.test_tx_depacketizer import (FIRST_TICK, MEM_BASE, MEM_SIZE, RX_OFFSET, SLOT_SIZE,
                                        TICK_PERIOD, make_packet, program, slot_address, submit)

FMT = golden.FORMAT_CS12
SETTLE_CYCLES = 40


class Harness(Module):
    def __init__(self):
        self.clock_domains.cd_rfic = ClockDomain()
        self.axi = AXIInterface(data_width=64, address_width=32, id_width=6, version="axi3")
        self.mode = Signal(reset=MODE_2R2T)
        self.tick = Signal()
        self.count = Signal(64, reset=FIRST_TICK)
        self.pool_base = Signal(32, reset=MEM_BASE)
        self.slot_size = Signal(32, reset=SLOT_SIZE)
        self.ring_offset = Signal(32, reset=RX_OFFSET)
        rx = [Signal(SAMPLE_BITS) for _ in range(LANES)]

        self.submodules.reader = reader = AxiHpReader(self.axi)
        self.tx_tick = Signal()
        self.submodules.dut = dut = TxDepacketizer(
            self.count, self.tx_tick, self.pool_base, self.slot_size, self.ring_offset,
            sample_fifo_depth=512)
        self.submodules.rate = rate = RateFilter(
            self.mode, self.tick, rx, [dut.ch0_i, dut.ch0_q, dut.ch1_i, dut.ch1_q])
        self.submodules.arbiter = arbiter = ReadRegionArbiter(reader, 1)
        self.comb += [
            self.tx_tick.eq(rate.tx_tick),
            rate.tx_start.eq(dut.start),
            rate.tx_busy.eq(dut.playing),
            dut.cmd.connect(arbiter.cmd[0]),
            arbiter.source[0].connect(dut.source),
        ]
        self.sync.rfic += If(self.tick, self.count.eq(self.count + 1))


def run_stream(ratio, packets, cycles=6000):
    """``packets`` is a list of (slot, samples, packet_count, stamp, eob).

    Returns the (count, sample) the engine released on each transmit tick it
    had a sample for, and the engine's counters.
    """
    dut = Harness()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    lengths = []
    for slot, samples, count, stamp, eob in packets:
        frame, frame_len = make_packet(FMT, samples, count, stamp, eob)
        memory.load(slot_address(slot), frame)
        lengths.append(frame_len)
    released = []
    state = {}
    ready = [False]

    def stim():
        yield dut.rate.tx_ratio.storage.eq(ratio)
        for _ in range(SETTLE_CYCLES):
            yield
        yield from program(dut, FMT)
        for (slot, _, _, _, _), length in zip(packets, lengths):
            yield from submit(dut, slot, length)
        ready[0] = True
        for _ in range(cycles):
            yield
        state["late"] = yield dut.dut.late.status
        state["underrun"] = yield dut.dut.underrun.status

    @passive
    def ticker():
        while True:
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            for _ in range(TICK_PERIOD - 1):
                yield

    @passive
    def recorder():
        while True:
            while not (yield dut.tx_tick):
                yield
            count = yield dut.count
            active = (yield dut.dut.playing) or (yield dut.dut.start)
            yield
            if active:
                released.append((count, (golden.from_twos((yield dut.dut.ch0_i)),
                                         golden.from_twos((yield dut.dut.ch0_q)))))

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [ticker(), recorder()]},
                   clocks={"sys": 10, "rfic": 10})
    return released, state


def ramp(count, first=1):
    return [(first + n, -(first + n)) for n in range(count)]


def test_a_burst_starts_on_its_count_and_plays_one_sample_per_ratio():
    ratio = 3
    stamp = FIRST_TICK + 601
    samples = ramp(20)
    released, state = run_stream(ratio, [(0, samples, 0, stamp, True)])
    assert state == {"late": 0, "underrun": 0}
    assert [sample for _, sample in released] == samples
    assert [count for count, _ in released] == [stamp + n * ratio for n in range(len(samples))]


def test_contiguous_packets_are_stamped_a_ratio_apart_per_sample():
    ratio = 4
    stamp = FIRST_TICK + 702
    first, second = ramp(16), ramp(16, first=100)
    released, state = run_stream(ratio, [
        (0, first, 0, stamp, False),
        (1, second, 1, stamp + len(first) * ratio, True),
    ])
    assert state == {"late": 0, "underrun": 0}
    assert [sample for _, sample in released] == first + second
    assert [count for count, _ in released] == [stamp + n * ratio for n in range(32)]


def test_a_later_burst_starts_on_its_own_count():
    ratio = 5
    stamp = FIRST_TICK + 600
    # Two counts off the first burst's sample grid.
    later = stamp + 16 * ratio + 42
    first, second = ramp(16), ramp(16, first=100)
    released, state = run_stream(ratio, [
        (0, first, 0, stamp, True),
        (1, second, 1, later, True),
    ])
    assert state == {"late": 0, "underrun": 0}
    assert [count for count, _ in released] == (
        [stamp + n * ratio for n in range(16)] + [later + n * ratio for n in range(16)])
    assert [sample for _, sample in released] == first + second
