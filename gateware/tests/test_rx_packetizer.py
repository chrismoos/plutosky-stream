"""Receive frames, byte for byte against the reference VITA builder."""

import random

import pytest
from migen import ClockDomain, If, Module, Signal, passive, run_simulation
from litex.soc.interconnect.axi import AXIInterface

from cores.axi_hp_master import AxiHpWriter, RegionArbiter
from cores.rx_packetizer import (HEADER_TEMPLATE_WORDS, SLOT_FRAME_OFFSET,
                                 RxPacketizer)
from tests import golden
from tests.axi_model import AxiMemory, read_generator, write_generator

MEM_BASE = 0x1000_0000
MEM_SIZE = 96 * 1024
SLOT_SIZE = 9728
TX_OFFSET = 0
SLOT_COUNT = 4
FIRST_TICK = 0x1_0000_0003

FORMAT_NAMES = {golden.FORMAT_CS12: "cs12", golden.FORMAT_CS8: "cs8",
                golden.FORMAT_BFP8: "bfp8"}


def fifo_depth_for(spp):
    """The packetizer reserves room for a whole packet before it starts one,
    so the ring has to be able to hold one. Keep it as small as that allows:
    the simulator evaluates every entry of it each cycle."""
    depth = 512
    while depth < spp + 16:
        depth *= 2
    return depth


class Harness(Module):
    def __init__(self, sample_fifo_depth=512, first_tick=FIRST_TICK):
        self.clock_domains.cd_rfic = ClockDomain()
        self.axi = AXIInterface(data_width=64, address_width=32, id_width=6,
                                version="axi3")
        self.tick = Signal()
        # The count advances the way the timebase advances it, in the cycle
        # after the tick, so a tick sees the count it is stamped with.
        self.count = Signal(64, reset=first_tick)
        self.sync.rfic += If(self.tick, self.count.eq(self.count + 1))
        self.ch0_i = Signal(12)
        self.ch0_q = Signal(12)
        self.ch1_i = Signal(12)
        self.ch1_q = Signal(12)
        self.pool_base = Signal(32, reset=MEM_BASE)
        self.slot_size = Signal(32, reset=SLOT_SIZE)
        self.ring_offset = Signal(32, reset=TX_OFFSET)

        self.submodules.writer = writer = AxiHpWriter(self.axi)
        self.submodules.pkt = pkt = RxPacketizer(
            self.count, self.tick, self.ch0_i, self.ch0_q, self.ch1_i, self.ch1_q,
            self.pool_base, self.slot_size, self.ring_offset,
            sample_fifo_depth=sample_fifo_depth)
        self.submodules.arbiter = arbiter = RegionArbiter(writer, 1)
        self.comb += [
            pkt.cmd.connect(arbiter.cmd[0]),
            pkt.sink.connect(arbiter.sink[0]),
            pkt.done.eq(arbiter.done[0]),
        ]


def template_value(template):
    """The 72-byte template as the CSR storage word, word 0 at the top."""
    value = 0
    for n, word in enumerate(golden.template_words(template)):
        value |= word << (32 * (HEADER_TEMPLATE_WORDS - 1 - n))
    return value


def program(dut, template, fmt, spp, frame_len, mask, arm_at=None):
    yield dut.pkt.header.storage.eq(template_value(template))
    yield dut.pkt.samples_per_packet.storage.eq(spp)
    yield dut.pkt.frame_len.storage.eq(frame_len)
    yield dut.pkt.slot_first.storage.eq(0)
    yield dut.pkt.slot_count.storage.eq(SLOT_COUNT)
    yield dut.pkt.released.storage.eq(0)
    yield dut.pkt.irq_threshold.storage.eq(2)
    if arm_at is not None:
        yield dut.pkt.start_lo.storage.eq(arm_at & 0xFFFF_FFFF)
        yield dut.pkt.start_hi.storage.eq(arm_at >> 32)
    yield
    yield
    ctrl = 1 | ((1 if arm_at is not None else 0) << 1) | (fmt << 2) | (mask << 4)
    yield dut.pkt.ctrl.storage.eq(ctrl)
    yield
    yield


@passive
def sample_driver(dut, per_channel, first_tick, period=4):
    """Present one sample period every ``period`` rfic cycles."""
    index = 0
    while True:
        if index < len(per_channel[0]):
            i0, q0 = per_channel[0][index]
            i1, q1 = per_channel[-1][index]
        else:
            i0 = q0 = i1 = q1 = 0
        yield dut.ch0_i.eq(i0 & 0xFFF)
        yield dut.ch0_q.eq(q0 & 0xFFF)
        yield dut.ch1_i.eq(i1 & 0xFFF)
        yield dut.ch1_q.eq(q1 & 0xFFF)
        yield dut.tick.eq(1)
        yield
        yield dut.tick.eq(0)
        for _ in range(period - 1):
            yield
        index += 1


def run_capture(fmt, mtu, channels, packets=3, seed=5, period=3, arm_at=None,
                sample_fifo_depth=None, limit=400000):
    """Drive the packetizer until ``packets`` frames land, then read them back."""
    # A two-channel packet is whole sample periods, so its count has to be a
    # multiple of the channel count.
    spp = golden.samples_per_packet(fmt, mtu) // channels * channels
    frame_len = golden.frame_len_for(fmt, spp)
    per_packet_periods = spp // channels
    total_periods = per_packet_periods * (packets + 1)

    rng = random.Random(seed)
    per_channel = [[(rng.randrange(-2048, 2048), rng.randrange(-2048, 2048))
                    for _ in range(total_periods)] for _ in range(channels)]
    template = golden.make_template(fmt=fmt, frame_len=frame_len)
    mask = 0b11 if channels == 2 else 0b01

    dut = Harness(sample_fifo_depth=sample_fifo_depth or fifo_depth_for(spp))
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    seen = []
    snapshot = []

    def stim():
        yield from program(dut, template, fmt, spp, frame_len, mask, arm_at)
        cycle = 0
        while len(seen) < packets and cycle < limit:
            cycle += 1
            yield
            produced = yield dut.pkt.produced.status
            while produced > len(seen):
                seen.append(len(seen))
            # Stand in for the daemon handing slots to the driver and getting
            # them back. Without it the ring fills at slot_count and the
            # packetizer stops, which is what the overflow test checks instead.
            if produced:
                yield dut.pkt.released.storage.eq(produced)
        assert len(seen) == packets, f"only {len(seen)} of {packets} frames landed"
        # produced only advances once the final write response has returned, so
        # the pool is complete the moment it does. Take the copy now: the stream
        # is still running and the next frame will overwrite a slot.
        for slot in range(SLOT_COUNT):
            base = MEM_BASE + TX_OFFSET + slot * SLOT_SIZE + SLOT_FRAME_OFFSET
            snapshot.append(memory[base:base + frame_len])
        yield dut.pkt.ctrl.storage.eq(0)

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [sample_driver(dut, per_channel, FIRST_TICK, period)]},
                   clocks={"sys": 10, "rfic": 10})

    frames = [snapshot[n % SLOT_COUNT] for n in range(min(packets, SLOT_COUNT))]
    return dict(frames=frames, slots=snapshot, template=template, spp=spp,
                frame_len=frame_len, per_channel=per_channel, memory=memory)


def stamp_of(frame):
    return int.from_bytes(frame[golden.VITA_TSF_OFFSET:golden.VITA_TSF_OFFSET + 8], "big")


def expected_frames(result, fmt, channels, packets, first_tick=FIRST_TICK):
    """Frames the reference builder produces for the samples the stream took.

    The stream starts at whichever sample period was current when the daemon
    enabled it, so the first frame's own timestamp says where to begin.
    """
    spp = result["spp"]
    per_packet_periods = spp // channels
    offset = stamp_of(result["frames"][0]) - first_tick
    assert 0 <= offset < per_packet_periods, f"stream began {offset} periods in"
    out = []
    for n in range(packets):
        lo = offset + n * per_packet_periods
        hi = lo + per_packet_periods
        chunk = golden.interleave([c[lo:hi] for c in result["per_channel"]])
        out.append(golden.build_frame(result["template"], fmt, chunk, n,
                                      first_tick + lo, result["frame_len"]))
    return out


@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_frames_are_byte_exact_at_1500(fmt):
    result = run_capture(fmt, 1500, channels=1, packets=3)
    assert result["frames"] == expected_frames(result, fmt, 1, 3)


@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_frames_are_byte_exact_at_1500_with_two_channels(fmt):
    result = run_capture(fmt, 1500, channels=2, packets=2, seed=17)
    assert result["frames"] == expected_frames(result, fmt, 2, 2)


@pytest.mark.slow
@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_frames_are_byte_exact_at_9000(fmt):
    result = run_capture(fmt, 9000, channels=1, packets=2, seed=23,
                         limit=4000000)
    assert result["frames"] == expected_frames(result, fmt, 1, 2)


@pytest.mark.slow
@pytest.mark.parametrize("fmt", list(FORMAT_NAMES), ids=FORMAT_NAMES.get)
def test_frames_are_byte_exact_at_9000_with_two_channels(fmt):
    result = run_capture(fmt, 9000, channels=2, packets=2, seed=29,
                         limit=4000000)
    assert result["frames"] == expected_frames(result, fmt, 2, 2)


def test_slots_are_assigned_in_order_and_wrap():
    """Frame n goes to slot n mod slot_count, so the ring comes round."""
    fmt = golden.FORMAT_CS8
    total = SLOT_COUNT + 2
    result = run_capture(fmt, 1500, channels=1, packets=total, seed=31,
                         limit=1000000)
    spp = result["spp"]
    # Slot 2 was written once and not written again, so its timestamp says
    # where in the sample stream the run began.
    offset = stamp_of(result["slots"][2]) - FIRST_TICK - 2 * spp
    assert 0 <= offset < spp, f"stream began {offset} periods in"

    # Only the last slot_count frames are still in the pool, and where they
    # are is the point of the test.
    for n in range(total - SLOT_COUNT, total):
        lo = offset + n * spp
        chunk = result["per_channel"][0][lo:lo + spp]
        expected = golden.build_frame(result["template"], fmt, chunk, n % 16,
                                      FIRST_TICK + lo, result["frame_len"])
        assert result["slots"][n % SLOT_COUNT] == expected, \
            f"frame {n} is not in slot {n % SLOT_COUNT}"


def test_irq_acknowledges_observed_frames_before_gem_releases_slots():
    fmt = golden.FORMAT_CS8
    spp = golden.samples_per_packet(fmt, 1500)
    frame_len = golden.frame_len_for(fmt, spp)
    template = golden.make_template(fmt=fmt, frame_len=frame_len)
    dut = Harness(sample_fifo_depth=fifo_depth_for(spp))
    memory = AxiMemory(MEM_SIZE, MEM_BASE)

    def stim():
        yield from program(dut, template, fmt, spp, frame_len, 0b01)
        for _ in range(30000):
            if (yield dut.pkt.produced.status) >= 2:
                break
            yield
        else:
            pytest.fail("packetizer did not produce two frames")
        assert (yield dut.pkt.irq) == 1
        produced = yield dut.pkt.produced.status
        yield dut.pkt.observed.storage.eq(produced)
        yield
        assert (yield dut.pkt.irq) == 0
        assert (yield dut.pkt.released.storage) == 0

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [sample_driver(dut, [[(0, 0)]], FIRST_TICK, period=2)]},
                   clocks={"sys": 10, "rfic": 10})


def test_armed_start_holds_until_the_timebase_arrives():
    fmt = golden.FORMAT_CS8
    spp = golden.samples_per_packet(fmt, 1500)
    start = FIRST_TICK + spp + 5
    result = run_capture(fmt, 1500, channels=1, packets=2, seed=37, arm_at=start,
                         limit=1000000)
    assert stamp_of(result["frames"][0]) == start, \
        "the first packet must begin exactly at the armed count"


def test_packet_count_increments_modulo_sixteen():
    fmt = golden.FORMAT_CS8
    result = run_capture(fmt, 1500, channels=1, packets=3, seed=41)
    counts = []
    for frame in result["frames"]:
        word = int.from_bytes(frame[golden.VITA_OFFSET:golden.VITA_OFFSET + 4], "big")
        counts.append((word >> 16) & 0xF)
    assert counts == [0, 1, 2]


def test_timestamps_advance_by_one_packet_of_samples():
    fmt = golden.FORMAT_CS12
    result = run_capture(fmt, 1500, channels=1, packets=3, seed=43)
    stamps = [stamp_of(f) for f in result["frames"]]
    assert stamps == [stamps[0] + n * result["spp"] for n in range(3)]


def test_overflow_counts_samples_dropped_when_no_slot_is_free():
    """With released left at zero the ring fills and the stream starts skipping."""
    fmt = golden.FORMAT_CS8
    spp = golden.samples_per_packet(fmt, 1500)
    frame_len = golden.frame_len_for(fmt, spp)
    template = golden.make_template(fmt=fmt, frame_len=frame_len)
    dut = Harness(sample_fifo_depth=fifo_depth_for(spp))
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    observed = {}

    @passive
    def driver():
        index = 0
        while True:
            yield dut.ch0_i.eq(index & 0xFFF)
            yield dut.ch0_q.eq(index & 0xFFF)
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            yield
            index += 1

    def stim():
        yield from program(dut, template, fmt, spp, frame_len, 0b01)
        for _ in range(14000):
            yield
        observed["produced"] = yield dut.pkt.produced.status
        observed["overflow"] = yield dut.pkt.overflow.status
        observed["irq"] = yield dut.pkt.irq

    run_simulation(dut, {"sys": [stim(), write_generator(dut.axi, memory),
                                 read_generator(dut.axi, memory)],
                         "rfic": [driver()]},
                   clocks={"sys": 10, "rfic": 10})

    # The ring is four slots deep and nothing is released, so it stops there and
    # every later sample period is counted as dropped.
    assert observed["produced"] == SLOT_COUNT
    assert observed["overflow"] > 0
    assert observed["irq"] == 1


def _restart_cycles(fmt, mtu, channels, cycles, packets_each, sample_fifo_depth):
    """Run the stream, stop it mid-flight, and restart it several times.

    A stop leaves the samples of the packet in progress in the queue. With a
    queue only a little larger than one packet, that leftover is enough to keep
    `room` short on the next start, so without a flush on disable the second or
    third run never produces again. Each restart must still deliver its packets.
    """
    spp = golden.samples_per_packet(fmt, mtu) // channels * channels
    frame_len = golden.frame_len_for(fmt, spp)
    total_periods = (spp // channels) * (packets_each + 2) * cycles
    rng = random.Random(7)
    per_channel = [[(rng.randrange(-2048, 2048), rng.randrange(-2048, 2048))
                    for _ in range(total_periods)] for _ in range(channels)]
    template = golden.make_template(fmt=fmt, frame_len=frame_len)
    mask = 0b11 if channels == 2 else 0b01
    dut = Harness(sample_fifo_depth=sample_fifo_depth)
    memory = AxiMemory(MEM_SIZE, MEM_BASE)
    per_cycle = []

    def stim():
        yield from program(dut, template, fmt, spp, frame_len, mask)
        for _ in range(cycles):
            base = (yield dut.pkt.produced.status)
            yield dut.pkt.released.storage.eq(base)
            yield dut.pkt.ctrl.storage.eq(1 | (fmt << 2) | (mask << 4))
            got = 0
            cyc = 0
            while got < packets_each and cyc < 200000:
                cyc += 1
                yield
                produced = (yield dut.pkt.produced.status) - base
                got = produced
                if produced:
                    yield dut.pkt.released.storage.eq(base + produced)
            per_cycle.append(got)
            # Stop mid-flight so the packet in progress is left in the queue.
            yield dut.pkt.ctrl.storage.eq(0)
            for _ in range(64):
                yield

    run_simulation(dut,
                   {"sys": [stim(), write_generator(dut.axi, memory),
                            read_generator(dut.axi, memory)],
                    "rfic": [sample_driver(dut, per_channel, FIRST_TICK, period=3)]},
                   clocks={"sys": 10, "rfic": 10})
    return per_cycle


def test_restart_after_stop_keeps_producing():
    # depth only 32 above one packet, so any leftover wedges the next start
    # unless the pipeline is flushed on disable.
    spp = golden.samples_per_packet(golden.FORMAT_CS12, 1500)
    got = _restart_cycles(golden.FORMAT_CS12, 1500, channels=1, cycles=4,
                          packets_each=2, sample_fifo_depth=fifo_depth_for(spp))
    assert got == [2, 2, 2, 2], f"per-cycle produced {got}"
