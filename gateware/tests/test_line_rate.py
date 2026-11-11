"""Both directions at Gigabit line rate, at once, for a millisecond.

A packetizer filling slots and a depacketizer draining them run together
against a memory model that answers with the latency a PS7 high-performance
port does, 150 to 250 ns to the first beat. Nothing here is allowed to drop a
sample: the receive side must not overflow and the transmit side must not
underrun, for the whole run.

This is the test that says the design sustains the link, rather than the
arithmetic saying it should.
"""

import os
from collections import deque

import pytest
from migen import (ClockDomain, ClockDomainsRenamer, If, Module, Signal, passive,
                   run_simulation)
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import (AxiHpReader, AxiHpWriter, ReadRegionArbiter,
                                 RegionArbiter)
from cores.rx_packetizer import HEADER_TEMPLATE_WORDS, SLOT_FRAME_OFFSET, RxPacketizer
from cores.tx_depacketizer import TxDepacketizer
from tests import golden
from tests.axi_model import AxiMemory, read_generator, write_generator

# The fabric clock. SYS_PERIOD_NS=20 runs the same demand at 50 MHz, where a
# master has to move twice the bytes each cycle. At that clock cs8 and bfp8
# fail on receive overflow and late packets, and are meant to: the packer and
# unpacker move one sample a cycle, so 50 MHz moves 50 MS/s, under what those
# two formats need at line rate and above anything the radio delivers.
SYS_PERIOD_NS = int(os.environ.get("SYS_PERIOD_NS", 10))
# The radio domain is clocked as fast as the fabric here on purpose. A Gigabit
# link's worth of cs8 is 59 MS/s, which is twice what the AD9363 can deliver at
# its own 61.44 MHz ceiling, so pacing this domain at the real DATA_CLK would
# measure the radio rather than the datapath under test.
RFIC_PERIOD_NS = 10
MEM_BASE = 0x4000_0000
SLOT_SIZE = 2048
TX_OFFSET = 0
RX_OFFSET = 32 * SLOT_SIZE
SLOT_COUNT = 8
MEM_SIZE = 64 * SLOT_SIZE
STREAM_ID = 0x4C49_4E45
FIRST_TICK = 0x9_0000_0000

# A PS7 HP port answers a read in roughly 150 to 250 ns, so the model is told
# the top of that and the write side is given a matching response delay, in
# whatever the fabric cycle is.
HP_READ_LATENCY_CYCLES = 250 // SYS_PERIOD_NS
HP_WRITE_RESPONSE_CYCLES = 200 // SYS_PERIOD_NS

# One millisecond of simulated time. Shorten it with LINE_RATE_CYCLES while
# working on the stimulus, but the rate assertions only hold over the full run.
CYCLES = int(os.environ.get("LINE_RATE_CYCLES", 1_000_000 // SYS_PERIOD_NS))
SIMULATED_NS = CYCLES * SYS_PERIOD_NS

# 118 MB/s is the payload a Gigabit link delivers. In cs8 that is 59 MS/s, which
# is more than one sample per 100 MHz cycle, so the pair of channels is driven
# at the rate the format needs to reach line rate and no faster.
LINE_RATE_BYTES = 118e6
# The standard MTU is the harder case: more frames a second, so the per-frame
# command, response and arbitration overheads are paid more often.
MTU = 1500
# What the SoC configures. The transmit side will not start fetching a packet
# until the ring has room for the whole of it, so a ring that holds only one
# packet makes the fetch wait for the playback instead of overlapping it.
RX_FIFO_DEPTH = 8192
TX_FIFO_DEPTH = 16384
# Sample periods between the start of the run and the first packet's timestamp.
# The transmit side needs about five hundred cycles to fetch a frame and unpack
# it, and this is comfortably more than that in every format.
FIRST_PACKET_LEAD = 400


def rate_for(fmt, spp, frame_len):
    """Sample periods a second that put line rate on the wire in this format.

    Every payload byte costs its share of the frame's header and padding, so
    the sample rate that fills the link is the one whose frames do.
    """
    return LINE_RATE_BYTES / frame_len * spp


def slot_addr(slot):
    return MEM_BASE + RX_OFFSET + slot * SLOT_SIZE + SLOT_FRAME_OFFSET


class Loop(Module):
    """The two cores and the two masters, wired as the SoC wires them.

    With ``split`` the datapath runs on its own ``stream`` clock and the
    registers stay on ``sys``, which is how the SoC is built. Without it both
    are one clock, which is what every other test uses.
    """

    def __init__(self, rx_fifo_depth=RX_FIFO_DEPTH, tx_fifo_depth=TX_FIFO_DEPTH,
                 split=False):
        self.clock_domains.cd_rfic = ClockDomain()
        stream_cd = "sys"
        if split:
            self.clock_domains.cd_stream = ClockDomain()
            stream_cd = "stream"
        on_stream = ClockDomainsRenamer(stream_cd)
        self.write_axi = AXIInterface(data_width=64, address_width=32, id_width=6,
                                      version="axi3")
        self.read_axi = AXIInterface(data_width=64, address_width=32, id_width=6,
                                     version="axi3")
        self.tick = Signal()
        self.count = Signal(64, reset=FIRST_TICK)
        self.ch0_i = Signal(12)
        self.ch0_q = Signal(12)
        self.ch1_i = Signal(12)
        self.ch1_q = Signal(12)
        self.pool_base = Signal(32, reset=MEM_BASE)
        self.slot_size = Signal(32, reset=SLOT_SIZE)
        self.tx_offset = Signal(32, reset=TX_OFFSET)
        self.rx_offset = Signal(32, reset=RX_OFFSET)

        self.submodules.writer = writer = on_stream(AxiHpWriter(self.write_axi))
        self.submodules.reader = reader = on_stream(AxiHpReader(self.read_axi))
        self.submodules.pkt = pkt = RxPacketizer(
            self.count, self.tick, self.ch0_i, self.ch0_q, self.ch1_i, self.ch1_q,
            self.pool_base, self.slot_size, self.tx_offset,
            stream_cd=stream_cd, sample_fifo_depth=rx_fifo_depth)
        self.submodules.depkt = depkt = TxDepacketizer(
            self.count, self.tick, self.pool_base, self.slot_size, self.rx_offset,
            stream_cd=stream_cd, sample_fifo_depth=tx_fifo_depth)
        self.submodules.rx_arbiter = rx_arbiter = on_stream(RegionArbiter(writer, 1))
        self.submodules.tx_arbiter = tx_arbiter = on_stream(ReadRegionArbiter(reader, 1))
        self.comb += [
            pkt.cmd.connect(rx_arbiter.cmd[0]),
            pkt.sink.connect(rx_arbiter.sink[0]),
            pkt.done.eq(rx_arbiter.done[0]),
            depkt.cmd.connect(tx_arbiter.cmd[0]),
            tx_arbiter.source[0].connect(depkt.source),
        ]
        self.sync.rfic += If(self.tick, self.count.eq(self.count + 1))


def template_value(template):
    value = 0
    for n, word in enumerate(golden.template_words(template)):
        value |= word << (32 * (HEADER_TEMPLATE_WORDS - 1 - n))
    return value


SCALE = 1 << 20


@passive
def sample_driver(dut, step):
    """Sample periods at a fractional rate, so the average is exact.

    A whole number of cycles between samples would round the rate down and
    quietly test something slower than the link.
    """
    acc = 0
    value = 0
    while True:
        acc += step
        if acc >= SCALE:
            acc -= SCALE
            value = (value + 37) & 0x7FF
            yield dut.ch0_i.eq(value)
            yield dut.ch0_q.eq((~value) & 0xFFF)
            yield dut.ch1_i.eq(value)
            yield dut.ch1_q.eq((~value) & 0xFFF)
            yield dut.tick.eq(1)
        else:
            yield dut.tick.eq(0)
        yield


@pytest.mark.slow
@pytest.mark.parametrize("split", [False, True], ids=["one-clock", "split"])
@pytest.mark.parametrize("fmt", [golden.FORMAT_CS12, golden.FORMAT_CS8,
                                 golden.FORMAT_BFP8],
                         ids=["cs12", "cs8", "bfp8"])
def test_both_directions_hold_line_rate_for_a_millisecond(fmt, split):
    # Split, the datapath keeps the fabric period and the register side runs
    # at half that, as the SoC does. The daemon model then paces itself by the
    # slower clock and its counts arrive through synchronisers.
    stream_ns = SYS_PERIOD_NS
    sys_ns = 2 * SYS_PERIOD_NS if split else SYS_PERIOD_NS
    cycles = SIMULATED_NS // sys_ns
    # Two channels share a sample period, so a packet must hold an even number
    # of samples for the period boundaries to line up with the packet ones.
    spp = golden.samples_per_packet(fmt, MTU) & ~1
    frame_len = golden.frame_len_for(fmt, spp)
    ticks_per_packet = spp // 2
    template = golden.make_template(fmt=fmt, stream_id=STREAM_ID, frame_len=frame_len)

    # Both channels share a sample period, so the period rate is half the
    # sample rate the link needs. The driver runs in the radio domain, so it is
    # paced by that clock, whatever the fabric clock is.
    step = round(rate_for(fmt, spp, frame_len) / 2 * RFIC_PERIOD_NS / 1e9 * SCALE)

    dut = Loop(split=split)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    state = {}

    frame = golden.build_frame(template, fmt, [(1000, -1000)] * spp, 0,
                               FIRST_TICK, frame_len)
    for slot in range(SLOT_COUNT):
        memory.load(slot_addr(slot), frame)

    def prepare(slot, seq, stamp):
        """What the daemon does to a returned slot before handing it back.

        Only the packet count and the timestamp change, so the packets arrive
        in sequence and each one starts exactly where the last one ended.
        """
        addr = slot_addr(slot)
        word = int.from_bytes(memory[addr + golden.VITA_OFFSET:
                                     addr + golden.VITA_OFFSET + 4], "big")
        word = (word & ~(0xF << 16)) | ((seq & 0xF) << 16)
        memory.load(addr + golden.VITA_OFFSET, word.to_bytes(4, "big"))
        memory.load(addr + golden.VITA_TSF_OFFSET, stamp.to_bytes(8, "big"))

    def stim():
        yield dut.pkt.header.storage.eq(template_value(template))
        yield dut.pkt.samples_per_packet.storage.eq(spp)
        yield dut.pkt.frame_len.storage.eq(frame_len)
        yield dut.pkt.slot_first.storage.eq(0)
        yield dut.pkt.slot_count.storage.eq(SLOT_COUNT)
        yield dut.depkt.stream_id.storage.eq(STREAM_ID)
        yield
        yield dut.pkt.ctrl.storage.eq(1 | (fmt << 2) | (0b11 << 4))
        yield dut.depkt.ctrl.storage.eq(1 | (fmt << 2) | (0b11 << 4))
        yield

        seq = 0
        stamp = FIRST_TICK + FIRST_PACKET_LEAD
        submitted_slots = deque()
        for slot in range(SLOT_COUNT):
            prepare(slot, seq, stamp)
            seq += 1
            stamp += ticks_per_packet
            yield dut.depkt.submit.storage.eq(slot | (frame_len << 16))
            yield dut.depkt.submit.re.eq(1)
            yield
            submitted_slots.append(slot)
        yield dut.depkt.submit.re.eq(0)

        released = 0
        cooldown = 0
        for _ in range(cycles):
            yield
            # Stand in for the daemon on the receive side: return the slots the
            # packetizer has filled so it never runs out.
            produced = yield dut.pkt.produced.status
            if produced != released:
                released = produced
                yield dut.pkt.released.storage.eq(released)
            # And on the transmit side: take back a finished slot, refill its
            # header for the next packet in the stream and resubmit it.
            if cooldown:
                cooldown -= 1
                yield dut.depkt.consumed.we.eq(0)
                yield dut.depkt.submit.re.eq(0)
                continue
            if (yield dut.depkt.consumed_count.status):
                slot = submitted_slots.popleft()
                prepare(slot, seq, stamp)
                seq += 1
                stamp += ticks_per_packet
                yield dut.depkt.consumed.we.eq(1)
                yield dut.depkt.submit.storage.eq(slot | (frame_len << 16))
                yield dut.depkt.submit.re.eq(1)
                submitted_slots.append(slot)
                # The pop lands a cycle later, so hold off long enough not to
                # read the same slot twice.
                cooldown = 2
            else:
                yield dut.depkt.consumed.we.eq(0)
                yield dut.depkt.submit.re.eq(0)

        state["produced"] = yield dut.pkt.produced.status
        state["overflow"] = yield dut.pkt.overflow.status
        state["underrun"] = yield dut.depkt.underrun.status
        state["late"] = yield dut.depkt.late.status
        state["bad_id"] = yield dut.depkt.bad_id.status
        state["gaps"] = yield dut.depkt.gaps.status
        state["played"] = seq - SLOT_COUNT

    bus = [write_generator(dut.write_axi, memory, b_delay=HP_WRITE_RESPONSE_CYCLES),
           read_generator(dut.write_axi, memory),
           write_generator(dut.read_axi, memory),
           read_generator(dut.read_axi, memory, latency=HP_READ_LATENCY_CYCLES)]
    generators = {"sys": [stim()], "rfic": [sample_driver(dut, step)]}
    clocks = {"sys": sys_ns, "rfic": RFIC_PERIOD_NS}
    if split:
        generators["stream"] = bus
        clocks["stream"] = stream_ns
    else:
        generators["sys"] += bus
    run_simulation(dut, generators, clocks=clocks)

    seconds = SIMULATED_NS / 1e9
    rx_bytes = state["produced"] * frame_len
    rx_rate = rx_bytes / seconds
    # The transmit side plays to the timestamps it is given, so it holds the
    # link from its first packet rather than from the first cycle. Its window
    # is the run less the lead in front of that packet.
    # A sample period has one sample of each channel, so the lead in
    # periods is twice that many samples at the sample rate.
    tx_seconds = seconds - FIRST_PACKET_LEAD * 2 / rate_for(fmt, spp, frame_len)
    assert tx_seconds > 0.9 * seconds
    tx_bytes = state["played"] * frame_len
    tx_rate = tx_bytes / tx_seconds
    bus_rate = memory.r_beats * 8 / seconds
    # A beat is eight bytes and one is available every cycle, so beats over
    # cycles is how much of the port the master used.
    datapath_cycles = SIMULATED_NS // stream_ns
    w_util = memory.w_beats / datapath_cycles
    r_util = memory.r_beats / datapath_cycles
    report = (f"{['cs12', 'cs8', 'bfp8'][fmt]}: "
              f"receive {rx_rate/1e6:.1f} MB/s over {state['produced']} frames, "
              f"transmit {tx_rate/1e6:.1f} MB/s over {state['played']} frames "
              f"and {tx_seconds*1e6:.0f} us, {bus_rate/1e6:.1f} MB/s on the bus, "
              f"total {(rx_rate + tx_rate)/1e6:.1f} MB/s. "
              f"HP0 write {memory.w_beats} beats, {100*w_util:.1f} % of the port. "
              f"HP2 read {memory.r_beats} beats, {100*r_util:.1f} % of the port. "
              f"outstanding writes {memory.max_write_outstanding}, "
              f"reads {memory.max_read_outstanding}. "
              f"overflow {state['overflow']}, underrun {state['underrun']}, "
              f"late {state['late']}, gaps {state['gaps']}")
    print("\n" + ("split " if split else "") + report)

    assert state["overflow"] == 0, f"receive overflowed. {report}"
    assert state["underrun"] == 0, f"transmit underran. {report}"
    assert state["late"] == 0, f"transmit dropped a late packet. {report}"
    assert state["bad_id"] == 0, report
    assert state["gaps"] == 0, f"the transmit stream lost a packet. {report}"
    # Whole frames only land inside the window, so allow the one frame of
    # quantization that costs.
    assert rx_bytes >= LINE_RATE_BYTES * seconds - frame_len, (
        f"receive carried {rx_rate/1e6:.1f} MB/s, needs {LINE_RATE_BYTES/1e6:.0f}")
    assert tx_bytes >= LINE_RATE_BYTES * tx_seconds - frame_len, (
        f"transmit carried {tx_rate/1e6:.1f} MB/s, needs {LINE_RATE_BYTES/1e6:.0f}")
