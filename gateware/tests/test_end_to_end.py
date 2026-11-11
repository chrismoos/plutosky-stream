"""The whole loop, one format at a time.

Samples go into a receive packetizer, which writes a finished Ethernet frame
into a slot in the memory model. The frame is handed straight back to a
transmit depacketizer, which reads it out of the same memory, and the samples
it releases are compared against the ones that went in, at the sample count the
receive side stamped them with.

Nothing here stands in for anything: the same frame bytes cross both cores, and
the memory model is the only thing between them.
"""

import pytest
from migen import ClockDomain, If, Module, Signal, passive, run_simulation
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import (AxiHpReader, AxiHpWriter, ReadRegionArbiter,
                                 RegionArbiter)
from cores.rx_packetizer import HEADER_TEMPLATE_WORDS, RxPacketizer
from cores.tx_depacketizer import TxDepacketizer
from tests import golden
from tests.axi_model import AxiMemory, read_generator, write_generator

MEM_BASE = 0x3000_0000
MEM_SIZE = 128 * 1024
SLOT_SIZE = 4096
TX_OFFSET = 0
RX_OFFSET = 8 * SLOT_SIZE
SLOT_COUNT = 4
STREAM_ID = 0x5150_4E47
FIRST_TICK = 0x7_0000_0100
TICK_PERIOD = 4
# One packet a burst keeps the timing assertion simple, and the burst is small
# enough that a full loop fits in a reasonable simulation.
SAMPLES_PER_PACKET = {golden.FORMAT_CS12: 34, golden.FORMAT_CS8: 34,
                      golden.FORMAT_BFP8: 32}
FORMAT_NAMES = {golden.FORMAT_CS12: "cs12", golden.FORMAT_CS8: "cs8",
                golden.FORMAT_BFP8: "bfp8"}
# The transmit side plays the packet back this far after the timebase passed
# the count the receive side stamped it with.
REPLAY_DELAY = 200


class Loop(Module):
    def __init__(self, sample_fifo_depth=512):
        self.clock_domains.cd_rfic = ClockDomain()
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

        self.submodules.writer = writer = AxiHpWriter(self.write_axi)
        self.submodules.reader = reader = AxiHpReader(self.read_axi)
        self.submodules.pkt = pkt = RxPacketizer(
            self.count, self.tick, self.ch0_i, self.ch0_q, self.ch1_i, self.ch1_q,
            self.pool_base, self.slot_size, self.tx_offset,
            sample_fifo_depth=sample_fifo_depth)
        self.submodules.depkt = depkt = TxDepacketizer(
            self.count, self.tick, self.pool_base, self.slot_size, self.rx_offset,
            sample_fifo_depth=sample_fifo_depth)
        self.submodules.rx_arbiter = rx_arbiter = RegionArbiter(writer, 1)
        self.submodules.tx_arbiter = tx_arbiter = ReadRegionArbiter(reader, 1)
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


@passive
def ticker(dut):
    while True:
        yield dut.tick.eq(1)
        yield
        yield dut.tick.eq(0)
        for _ in range(TICK_PERIOD - 1):
            yield


@passive
def sample_driver(dut, samples):
    index = 0
    while True:
        i, q = samples[index] if index < len(samples) else (0, 0)
        yield dut.ch0_i.eq(i & 0xFFF)
        yield dut.ch0_q.eq(q & 0xFFF)
        yield dut.tick.eq(1)
        yield
        yield dut.tick.eq(0)
        for _ in range(TICK_PERIOD - 1):
            yield
        index += 1


@passive
def air_recorder(dut, out):
    while True:
        while not (yield dut.tick):
            yield
        yield
        i = yield dut.depkt.ch0_i
        q = yield dut.depkt.ch0_q
        out.append((golden.from_twos(i), golden.from_twos(q)))


def ramp(count, start=3, step=57):
    return [(((start + n * step) % 4000) - 2000, 1500 - ((start + n * step) % 3000))
            for n in range(count)]


@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_a_packet_survives_the_whole_loop(fmt):
    spp = SAMPLES_PER_PACKET[fmt]
    frame_len = golden.frame_len_for(fmt, spp, end_of_burst=True)
    template = golden.make_template(fmt=fmt, stream_id=STREAM_ID, frame_len=frame_len)
    samples = ramp(spp * 3)

    dut = Loop()
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    air = []
    state = {}

    def stim():
        # Receive side: one stream, one slot range, no arming.
        yield dut.pkt.header.storage.eq(template_value(template))
        yield dut.pkt.samples_per_packet.storage.eq(spp)
        # The receive side never appends a trailer, so it writes the shorter
        # frame and the transmit side is told the end-of-burst length.
        yield dut.pkt.frame_len.storage.eq(golden.frame_len_for(fmt, spp))
        yield dut.pkt.slot_first.storage.eq(0)
        yield dut.pkt.slot_count.storage.eq(SLOT_COUNT)
        yield dut.depkt.stream_id.storage.eq(STREAM_ID)
        yield
        yield dut.pkt.ctrl.storage.eq(1 | (fmt << 2) | (0b01 << 4))
        yield dut.depkt.ctrl.storage.eq(1 | (fmt << 2) | (0b01 << 4))
        yield

        # Wait for the first frame to land in a transmit slot.
        for _ in range(20000):
            yield
            if (yield dut.pkt.produced.status):
                break
        assert (yield dut.pkt.produced.status), "no frame was ever produced"
        for _ in range(40):
            yield

        # Copy it into a receive slot, exactly as GEM would have delivered it,
        # and move the timestamp forward so the transmit side has time to read
        # it back before its release moment.
        source = MEM_BASE + TX_OFFSET + 2
        written = memory[source:source + golden.frame_len_for(fmt, spp)]
        state["frame"] = written
        stamp = int.from_bytes(
            written[golden.VITA_TSF_OFFSET:golden.VITA_TSF_OFFSET + 8], "big")
        now = yield dut.count
        state["stamp"] = stamp
        replay = now + REPLAY_DELAY
        state["replay"] = replay
        frame = bytearray(written)
        frame[golden.VITA_TSF_OFFSET:golden.VITA_TSF_OFFSET + 8] = \
            replay.to_bytes(8, "big")
        # Mark it as the end of a burst, which lengthens the packet by its
        # trailer word.
        header = int.from_bytes(frame[golden.VITA_OFFSET:golden.VITA_OFFSET + 4], "big")
        header |= 1 << 26
        header &= ~0xFFFF
        header |= golden.frame_len_to_size_words(frame_len)
        frame[golden.VITA_OFFSET:golden.VITA_OFFSET + 4] = header.to_bytes(4, "big")
        frame += bytes(frame_len - golden.frame_len_for(fmt, spp) - 4)
        frame += (1).to_bytes(4, "big")
        memory.load(MEM_BASE + RX_OFFSET + 2, bytes(frame))

        yield dut.depkt.submit.storage.eq(0 | (frame_len << 16))
        yield dut.depkt.submit.re.eq(1)
        yield
        yield dut.depkt.submit.re.eq(0)
        for _ in range(8000):
            yield
        state["late"] = yield dut.depkt.late.status
        state["bad_id"] = yield dut.depkt.bad_id.status
        state["consumed"] = yield dut.depkt.consumed_count.status

    run_simulation(dut,
                   {"sys": [stim(),
                            write_generator(dut.write_axi, memory),
                            read_generator(dut.write_axi, memory),
                            write_generator(dut.read_axi, memory),
                            read_generator(dut.read_axi, memory)],
                    "rfic": [sample_driver(dut, samples), air_recorder(dut, air)]},
                   clocks={"sys": 10, "rfic": 10})

    assert state["late"] == 0, "the replayed packet was not on time"
    assert state["bad_id"] == 0
    assert state["consumed"] == 1, "the slot was not returned"

    # The frame the receive side built is the frame the reference builder makes
    # for the samples it took, starting at the count it stamped them with.
    offset = state["stamp"] - FIRST_TICK
    taken = samples[offset:offset + spp]
    expected_frame = golden.build_frame(template, fmt, taken, 0,
                                        state["stamp"], golden.frame_len_for(fmt, spp))
    assert state["frame"] == expected_frame, "the receive frame is not byte exact"

    # And the samples that came back out are those samples, put on the air at
    # the count the replayed packet asked for.
    expected_air = golden.unpack(fmt, golden.pack(fmt, taken), spp)
    index = state["replay"] - FIRST_TICK
    assert air[index:index + spp] == expected_air, (
        f"the loop did not return the samples\n got {air[index:index + spp]}"
        f"\nwant {expected_air}")
