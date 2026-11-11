"""One receive stream: samples in, whole Ethernet frames in DDR slots.

The packetizer writes frames the GEM can transmit unaltered, so it owns the
whole frame, not just the payload. The daemon programs a 72-byte template
holding everything constant for the stream, from the destination MAC through
the VITA class id, with the IP header checksum already computed for
``frame_len``. The packetizer copies the first 66 bytes of that template, ORs
the packet count and the packet size into VITA word 0, writes the 64-bit sample
count over VITA words 4 and 5, appends the packed payload and zero-fills to
``frame_len``.

Frame offsets are all fixed, which is what makes that cheap. The Ethernet
header is fourteen bytes, so from the IP header on every field sits two bytes
past a 32-bit boundary. That is why a frame starts at byte 2 of its slot: the
IP header and the payload then land aligned in memory even though they are
misaligned in the frame.

Timing has two halves. The sample-rate half runs in ``rfic`` and decides which
sample starts a packet, capturing the timebase at that exact sample so the
timestamp is not an estimate. It only starts a packet when there is room for
all of it, so a packet is never torn in the middle: when there is no room the
samples are counted in ``overflow`` and the stream simply resumes at the next
packet, which the host sees as a jump in sample count. The bus half runs in
``sys``, claims a slot, streams the frame through the shared write master and
waits for the final write response before it advances ``produced``. Only then
is the frame really in DDR for the driver to send.
"""

from migen import *
from migen.genlib.cdc import BusSynchronizer, MultiReg
from migen.genlib.fsm import FSM, NextState, NextValue

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from cores.fifo import AsyncFifo
from cores.axi_hp_master import byte_stream_layout, command_layout
from cores.sample_packer import SAMPLE_BITS, SamplePacker

ETH_HEADER_BYTES = 14
IP_HEADER_BYTES = 20
UDP_HEADER_BYTES = 8
VITA_HEADER_BYTES = 24
# The sizing rule holds four more bytes back for the end-of-burst trailer,
# which receive frames never include.
VITA_TRAILER_BYTES = 4
VITA_OFFSET = ETH_HEADER_BYTES + IP_HEADER_BYTES + UDP_HEADER_BYTES
FRAME_HEADER_BYTES = VITA_OFFSET + VITA_HEADER_BYTES

HEADER_TEMPLATE_WORDS = 18
HEADER_TEMPLATE_BYTES = 4 * HEADER_TEMPLATE_WORDS
# Full 32-bit beats of header, then a two-byte beat closing the timestamp.
HEADER_FULL_BEATS = FRAME_HEADER_BYTES // 4
HEADER_TAIL_BYTES = FRAME_HEADER_BYTES % 4

# VITA word 0 straddles two template words because the Ethernet header is not
# a multiple of four bytes long.
VITA_WORD0_LOW_BEAT = VITA_OFFSET // 4
TSF_FIRST_BEAT = (VITA_OFFSET + 16) // 4

SLOT_FRAME_OFFSET = 2

LENGTH_BITS = 16
COUNTER_BITS = 32
TICK_BITS = 64
SLOT_BITS = 16

DEFAULT_SAMPLE_FIFO_DEPTH = 4096
# Stream-clock cycles the enable must hold before the stream side acts on it.
ENABLE_SETTLE_CYCLES = 4
PACKETS_IN_FLIGHT = 4


def _byte_swap(word):
    """Wire order to memory order for one 32-bit template word."""
    return Cat(word[24:32], word[16:24], word[8:16], word[0:8])


class RxPacketizer(LiteXModule):
    def __init__(self, timebase_count, tick, ch0_i, ch0_q, ch1_i, ch1_q,
                 pool_base, slot_size, ring_offset, rfic_cd="rfic", stream_cd="sys",
                 sample_fifo_depth=DEFAULT_SAMPLE_FIFO_DEPTH, count_tick=None):
        """``tick`` pulses once per sample. ``count_tick`` pulses when
        ``timebase_count`` advances, and is ``tick`` when omitted."""
        if count_tick is None:
            count_tick = tick
        self.ctrl = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, description="Run the stream."),
            CSRField("arm", size=1, offset=1,
                description="Hold the stream until the timebase reaches start_hi/start_lo."),
            CSRField("format", size=2, offset=2, values=[
                ("``0``", "cs12"), ("``1``", "cs8"), ("``2``", "bfp8")],
                description="Wire format."),
            CSRField("channel_mask", size=2, offset=4,
                description="Channels in the stream, bit 0 for channel 0."),
        ], description="Stream control.")
        self.samples_per_packet = CSRStorage(LENGTH_BITS,
            description="Payload samples per packet, counting both channels when the stream has both. Keep it even for a two-channel stream, or the last sample period of each packet holds channel 0 alone.")
        self.frame_len = CSRStorage(LENGTH_BITS,
            description="Total Ethernet frame bytes excluding the FCS.")
        self.start_hi = CSRStorage(32, description="High word of the armed start count.")
        self.start_lo = CSRStorage(32, description="Low word of the armed start count.")
        self.header = CSRStorage(32 * HEADER_TEMPLATE_WORDS,
            description="Frame template, 72 bytes as 18 words written lowest address first.")
        self.slot_first = CSRStorage(SLOT_BITS, description="First slot this stream may write.")
        self.slot_count = CSRStorage(SLOT_BITS, description="Slots in this stream's range.")
        self.produced = CSRStatus(COUNTER_BITS, description="Frames written, monotonic.")
        self.released = CSRStorage(COUNTER_BITS,
            description="Frames the producer may overwrite.")
        self.overflow = CSRStatus(COUNTER_BITS,
            description="Samples dropped because no slot was free.")
        self.irq_threshold = CSRStorage(COUNTER_BITS,
            description="Raise the interrupt once this many completed frames are unobserved.")
        self.observed = CSRStorage(COUNTER_BITS,
            description="Completed frames the kernel has observed.")

        # Shared write master, through the region arbiter.
        self.cmd = cmd = stream.Endpoint(command_layout())
        self.sink = sink = stream.Endpoint(byte_stream_layout())
        self.done = Signal()
        self.irq = Signal()

        # # #

        sync_rfic = getattr(self.sync, rfic_cd)
        sync_stream = getattr(self.sync, stream_cd)

        # Read the control bits out of the storage word rather than through the
        # CSR's own field logic, which only exists once a CSR bank has finalised
        # the register. That keeps the core whole in a simulation with no bus.
        enable_bit = self.ctrl.storage[0]
        arm_bit = self.ctrl.storage[1]
        mask_bits = self.ctrl.storage[4:6]

        # The registers live on the control clock and the frame is built on the
        # stream clock. Nothing on the stream side reads a register directly:
        # each quasi-static setting comes across through a synchroniser into a
        # copy that only follows it while the stream is disabled, so what the
        # daemon wrote before enabling is what the whole run uses, and the
        # register bank can sit wherever the control bus is.
        enable_stream = Signal()
        format_bits = Signal(2)
        cfg_spp = Signal(LENGTH_BITS, reset_less=True)
        cfg_frame_len = Signal(LENGTH_BITS, reset_less=True)
        cfg_slot_first = Signal(SLOT_BITS, reset_less=True)
        cfg_slot_count = Signal(SLOT_BITS, reset_less=True)
        cfg_header = Signal(32 * HEADER_TEMPLATE_WORDS, reset_less=True)
        crossed_format = Signal(2)
        crossed_spp = Signal(LENGTH_BITS)
        crossed_frame_len = Signal(LENGTH_BITS)
        crossed_slot_first = Signal(SLOT_BITS)
        crossed_slot_count = Signal(SLOT_BITS)
        crossed_header = Signal(32 * HEADER_TEMPLATE_WORDS)
        self.specials += [
            MultiReg(enable_bit, enable_stream, odomain=stream_cd),
            MultiReg(self.ctrl.storage[2:4], crossed_format, odomain=stream_cd),
            MultiReg(self.samples_per_packet.storage, crossed_spp, odomain=stream_cd),
            MultiReg(self.frame_len.storage, crossed_frame_len, odomain=stream_cd),
            MultiReg(self.slot_first.storage, crossed_slot_first, odomain=stream_cd),
            MultiReg(self.slot_count.storage, crossed_slot_count, odomain=stream_cd),
            MultiReg(self.header.storage, crossed_header, odomain=stream_cd),
        ]
        # Enable takes effect on the stream side only once it has been high
        # for a few cycles. The format shares a register with the enable bit,
        # so the two cross in the same cycle through parallel synchronisers,
        # and a copy that stopped following the moment enable arrived would
        # miss the format that arrived beside it. The copies keep following
        # until the window closes, which also covers settings written in the
        # cycle before the enable, however the synchronisers reorder them.
        enable_settled = Signal()
        settling = Signal(ENABLE_SETTLE_CYCLES)
        sync_stream += [
            settling.eq(Cat(enable_stream, settling[:-1])),
            enable_settled.eq(settling == (1 << ENABLE_SETTLE_CYCLES) - 1),
        ]
        sync_stream += If(~enable_settled,
            format_bits.eq(crossed_format),
            cfg_spp.eq(crossed_spp),
            cfg_frame_len.eq(crossed_frame_len),
            cfg_slot_first.eq(crossed_slot_first),
            cfg_slot_count.eq(crossed_slot_count),
            cfg_header.eq(crossed_header),
        )

        # The slot pool's registers arrive as inputs and belong to the control
        # clock too, so they cross the same way as this core's own settings.
        cfg_pool_base = Signal(32, reset_less=True)
        cfg_slot_size = Signal(32, reset_less=True)
        cfg_ring_offset = Signal(32, reset_less=True)
        crossed_pool_base = Signal(32)
        crossed_slot_size = Signal(32)
        crossed_ring_offset = Signal(32)
        self.specials += [
            MultiReg(pool_base, crossed_pool_base, odomain=stream_cd),
            MultiReg(slot_size, crossed_slot_size, odomain=stream_cd),
            MultiReg(ring_offset, crossed_ring_offset, odomain=stream_cd),
        ]
        sync_stream += If(~enable_settled,
            cfg_pool_base.eq(crossed_pool_base),
            cfg_slot_size.eq(crossed_slot_size),
            cfg_ring_offset.eq(crossed_ring_offset),
        )

        self.dbg = {"enable_stream": enable_stream, "enable_settled": enable_settled, "cfg_frame_len": cfg_frame_len,
                    "cfg_spp": cfg_spp, "cfg_slot_count": cfg_slot_count,
                    "format_bits": format_bits, "cfg_pool_base": cfg_pool_base,
                    "cfg_slot_size": cfg_slot_size}

        tpl = Array([cfg_header[32 * (HEADER_TEMPLATE_WORDS - 1 - n):
                                32 * (HEADER_TEMPLATE_WORDS - n)]
                     for n in range(HEADER_TEMPLATE_WORDS)])

        # Sample-rate side
        enable = Signal()
        armed = Signal()
        mask = Signal(2)
        spp = Signal(LENGTH_BITS)
        start_at = Signal(TICK_BITS)
        self.specials += [
            MultiReg(enable_bit, enable, odomain=rfic_cd),
            MultiReg(arm_bit, armed, odomain=rfic_cd),
            MultiReg(mask_bits, mask, odomain=rfic_cd),
            MultiReg(self.samples_per_packet.storage, spp, odomain=rfic_cd),
            MultiReg(Cat(self.start_lo.storage, self.start_hi.storage), start_at,
                     odomain=rfic_cd),
        ]

        self.samples = samples = ClockDomainsRenamer({"write": rfic_cd, "read": stream_cd})(
            AsyncFifo(2 * SAMPLE_BITS, sample_fifo_depth))
        self.stamps = stamps = ClockDomainsRenamer({"write": rfic_cd, "read": stream_cd})(
            AsyncFifo(TICK_BITS, PACKETS_IN_FLIGHT))
        # Empty both queues whenever the stream is disabled, in each queue's
        # own domain, so a restart begins from an empty, pointer-aligned
        # pipeline. Without this the samples left in flight when a stream stops
        # keep `room` short on the next run, and after enough stop/start cycles
        # `can_start` never asserts again.
        self.comb += [
            samples.flush_write.eq(~enable),
            stamps.flush_write.eq(~enable),
            samples.flush_read.eq(~enable_stream),
            stamps.flush_read.eq(~enable_stream),
        ]

        written = Signal(COUNTER_BITS)
        read_back = Signal(COUNTER_BITS)
        level = Signal(COUNTER_BITS)
        room = Signal(COUNTER_BITS)
        self.read_sync = read_sync = BusSynchronizer(COUNTER_BITS, stream_cd, rfic_cd)
        self.comb += [
            read_sync.i.eq(read_back),
            level.eq(written - read_sync.o),
            room.eq(sample_fifo_depth - level),
        ]

        both = Signal()
        left = Signal(LENGTH_BITS)
        budget = Signal(LENGTH_BITS)
        pushing = Signal(2)
        running = Signal()
        pend_ch1 = Signal()
        overflow = Signal(COUNTER_BITS)
        time_reached = Signal()
        room_ok = Signal()
        can_start = Signal()
        push_count = Signal(2)
        sel_i = Signal(SAMPLE_BITS)
        sel_q = Signal(SAMPLE_BITS)

        self.comb += [
            both.eq(mask == 0b11),
            push_count.eq(Mux(both, 2, 1)),
            can_start.eq(enable & time_reached & stamps.writable & room_ok
                         & (spp != 0)),
            # Channel 1 only, or channel 0 when the mask selects it, and the
            # second half of an interleaved pair on the cycle after the tick.
            If(pend_ch1,
                sel_i.eq(ch1_i), sel_q.eq(ch1_q),
            ).Elif(mask == 0b10,
                sel_i.eq(ch1_i), sel_q.eq(ch1_q),
            ).Else(
                sel_i.eq(ch0_i), sel_q.eq(ch0_q),
            ),
            # Samples still owed for the packet in progress, or for the one
            # about to start, and how many of them this tick can hold. A count
            # that is not a whole number of sample periods ends with a single
            # channel rather than running the packet past its length.
            budget.eq(Mux(running, left, spp)),
            pushing.eq(Mux(both & (budget >= 2), 2, 1)),
            samples.we.eq(((tick & (running | can_start)) | pend_ch1) & samples.writable),
            samples.din.eq(Cat(sel_i, sel_q)),
            stamps.we.eq(tick & ~running & can_start),
            stamps.din.eq(timebase_count),
        ]
        # Two compares that gate the sample and stamp queues' write strobes,
        # each a wide net at the sample rate. Both are registered a cycle ahead
        # and stay exact: the count only moves on its tick, so it is judged
        # against where it is about to be, and the ring only loses room to
        # this core's own writes, at most one a cycle, which the extra sample
        # of margin covers.
        sync_rfic += [
            time_reached.eq(~armed | ((timebase_count + count_tick) >= start_at)),
            room_ok.eq(room >= (spp + 1)),
        ]
        sync_rfic += [
            pend_ch1.eq(0),
            If(samples.we,
                written.eq(written + 1),
            ),
            If(tick,
                If(running | can_start,
                    If(both & (budget >= 2), pend_ch1.eq(1)),
                    left.eq(budget - pushing),
                    running.eq(budget > pushing),
                ).Elif(enable,
                    overflow.eq(overflow + push_count),
                ),
            ),
            If(~enable,
                running.eq(0),
                pend_ch1.eq(0),
                written.eq(0),
            ),
        ]
        self.overflow_sync = overflow_sync = BusSynchronizer(COUNTER_BITS, rfic_cd, "sys")
        self.comb += [
            overflow_sync.i.eq(overflow),
            self.overflow.status.eq(overflow_sync.o),
        ]

        # Bus side
        self.packer = packer = ClockDomainsRenamer(stream_cd)(SamplePacker())
        produced = Signal(COUNTER_BITS)
        cursor = Signal(SLOT_BITS)
        slot_base = Signal(32, reset_less=True)
        slot_addr = Signal(32, reset_less=True)
        # Where the next frame goes, added up while this one is still being
        # written. The completion it waits on arrives through the shared write
        # master and the arbiter, which is a long way for an add to follow.
        next_slot_addr = Signal(32, reset_less=True)
        slot_first_offset = Signal(32, reset_less=True)
        outstanding = Signal(COUNTER_BITS)
        frame_bytes = Signal(LENGTH_BITS)
        beat = Signal(max=HEADER_FULL_BEATS + 1)
        payload_left = Signal(LENGTH_BITS)
        packet_count = Signal(4)
        stamp = Signal(TICK_BITS, reset_less=True)
        pad_bytes = Signal(3)
        frame_done = Signal()
        beat_taken = Signal()
        starting = Signal()
        next_frame_bytes = Signal(LENGTH_BITS)
        size_words = Signal(16)
        vita0 = Signal(32, reset_less=True)
        tpl_vita0 = Signal(32)
        header_word = Signal(32, reset_less=True)
        enable_sys = Signal()
        slot_ready = Signal()
        in_idle = Signal()
        claimed = Signal()
        stamp_ready = Signal()

        # Producer acknowledgement controls the interrupt independently of slot
        # release, since GEM may hold a frame until it reaches the wire.
        self.produced_sync = produced_sync = BusSynchronizer(COUNTER_BITS, stream_cd, "sys")
        self.released_sync = released_sync = BusSynchronizer(COUNTER_BITS, "sys", stream_cd)
        unobserved_sys = Signal(COUNTER_BITS)
        self.comb += [
            produced_sync.i.eq(produced),
            self.produced.status.eq(produced_sync.o),
            released_sync.i.eq(self.released.storage),
            outstanding.eq(produced - released_sync.o),
            unobserved_sys.eq(produced_sync.o - self.observed.storage),
            self.irq.eq((self.irq_threshold.storage != 0)
                        & (unobserved_sys >= self.irq_threshold.storage)),
            enable_sys.eq(enable_settled),
            claimed.eq(~in_idle | (enable_sys & stamp_ready & slot_ready)),
            size_words.eq((cfg_frame_len - VITA_OFFSET) >> 2),
            tpl_vita0.eq(Cat(tpl[VITA_WORD0_LOW_BEAT + 1][16:32],
                             tpl[VITA_WORD0_LOW_BEAT][0:16])),
            vita0.eq(tpl_vita0 | (packet_count << 16) | size_words),
            packer.fmt.eq(format_bits),
            packer.sink.i.eq(samples.dout[:SAMPLE_BITS]),
            packer.sink.q.eq(samples.dout[SAMPLE_BITS:]),
            beat_taken.eq(sink.valid & sink.ready),
            next_frame_bytes.eq(Mux(starting, cfg_frame_len,
                                    frame_bytes - Mux(beat_taken, sink.bytes, 0))),
        ]
        # What is left of the frame decides whether the padding beat happens
        # and how wide it is, and from there it reaches the write master's
        # staging register. Both come off the count the frame is about to hold,
        # which is the same value a cycle earlier and a register by the time it
        # is used.
        sync_stream += [
            frame_done.eq(next_frame_bytes == 0),
            pad_bytes.eq(Mux(next_frame_bytes < 4, next_frame_bytes, 4)),
        ]

        # One header beat, with the two patched fields dropped into place.
        # Picking a word out of the seventy-two byte template is a mux as wide
        # as the template, and it fed the write master's own byte shifter, so
        # the beat about to be sent is selected a cycle early and registered.
        beat_tpl = Signal(32)
        next_word = Signal(32)
        next_beat = Signal(max=HEADER_TEMPLATE_WORDS)
        emit_header = Signal()
        in_header = Signal()
        self.comb += [
            emit_header.eq(in_header & sink.ready
                           & (beat != HEADER_FULL_BEATS)),
            next_beat.eq(beat + emit_header),
            beat_tpl.eq(_byte_swap(Array(tpl)[next_beat])),
            next_word.eq(beat_tpl),
            Case(next_beat, {
                VITA_WORD0_LOW_BEAT: next_word.eq(
                    Cat(beat_tpl[0:16], vita0[24:32], vita0[16:24])),
                VITA_WORD0_LOW_BEAT + 1: next_word.eq(
                    Cat(vita0[8:16], vita0[0:8], beat_tpl[16:32])),
                TSF_FIRST_BEAT: next_word.eq(
                    Cat(beat_tpl[0:16], stamp[56:64], stamp[48:56])),
                TSF_FIRST_BEAT + 1: next_word.eq(
                    Cat(stamp[40:48], stamp[32:40], stamp[24:32], stamp[16:24])),
                TSF_FIRST_BEAT + 2: next_word.eq(
                    Cat(stamp[8:16], stamp[0:8], Constant(0, 16))),
            }),
        ]
        sync_stream += header_word.eq(next_word)

        self.fsm = fsm = ClockDomainsRenamer(stream_cd)(FSM(reset_state="IDLE"))
        fsm.act("IDLE",
            packer.clear.eq(1),
            in_idle.eq(1),
            If(enable_sys & stamp_ready & slot_ready,
                starting.eq(1),
                NextValue(stamp, stamps.dout),
                stamps.re.eq(1),
                NextValue(frame_bytes, cfg_frame_len),
                NextValue(payload_left, cfg_spp),
                NextValue(beat, 0),
                NextState("COMMAND"),
            ),
        )
        fsm.act("COMMAND",
            cmd.valid.eq(1),
            cmd.addr.eq(slot_addr + SLOT_FRAME_OFFSET),
            cmd.length.eq(cfg_frame_len),
            If(cmd.ready, NextState("HEADER")),
        )
        fsm.act("HEADER",
            in_header.eq(1),
            sink.valid.eq(1),
            sink.data.eq(header_word),
            sink.bytes.eq(Mux(beat == HEADER_FULL_BEATS, HEADER_TAIL_BYTES, 4)),
            If(sink.ready,
                NextValue(frame_bytes, frame_bytes - sink.bytes),
                If(beat == HEADER_FULL_BEATS,
                    NextState("PAYLOAD"),
                ).Else(
                    NextValue(beat, beat + 1),
                ),
            ),
        )
        fsm.act("PAYLOAD",
            packer.sink.valid.eq(samples.readable & (payload_left != 0)),
            packer.sink.last.eq(payload_left == 1),
            samples.re.eq(packer.sink.valid & packer.sink.ready),
            packer.source.connect(sink, omit={"first", "last"}),
            If(packer.sink.valid & packer.sink.ready,
                NextValue(payload_left, payload_left - 1),
            ),
            If(sink.valid & sink.ready,
                NextValue(frame_bytes, frame_bytes - sink.bytes),
                If(packer.source.last, NextState("PAD")),
            ),
            # A stop drains the sample queue while a frame is mid-flight. The
            # radio side has stopped refilling it, so waiting for the rest of
            # the payload would stall here forever and leave the AXI write
            # burst unfinished, which hangs the PS HP slave. When disabled and
            # the queue has run dry, pad the frame to length instead so the
            # burst completes cleanly. The half-written last frame is discarded
            # by the host on stop.
            If(~enable_sys & (payload_left != 0) & ~samples.readable,
                NextState("PAD"),
            ),
        )
        fsm.act("PAD",
            If(frame_done,
                NextState("SETTLE"),
            ).Else(
                sink.valid.eq(1),
                sink.data.eq(0),
                sink.bytes.eq(pad_bytes),
                If(sink.ready, NextValue(frame_bytes, frame_bytes - pad_bytes)),
            ),
        )
        fsm.act("SETTLE",
            If(self.done,
                NextValue(produced, produced + 1),
                NextValue(packet_count, packet_count + 1),
                NextValue(cursor, Mux(cursor == cfg_slot_count - 1,
                                      0, cursor + 1)),
                NextValue(slot_addr, next_slot_addr),
                NextState("IDLE"),
            ),
        )

        sync_stream += [
            If(samples.re, read_back.eq(read_back + 1)),
            If(~enable_stream, read_back.eq(0)),
            # The multiply and the add that place the first slot are a cycle
            # apart: together they are far too long for one at 100 MHz, and a
            # stopped stream has all the cycles in the world.
            slot_first_offset.eq(cfg_slot_first * cfg_slot_size),
            next_slot_addr.eq(Mux(cursor == cfg_slot_count - 1,
                                  slot_base, slot_addr + cfg_slot_size)),
            # A thirty-two bit subtract and a compare, which reached the packer
            # through the state machine. Registering it keeps it off that path.
            # A frame in flight holds a slot that produced has not counted yet,
            # so it counts here, or the one cycle of staleness would hand out
            # the same slot twice.
            slot_ready.eq((cfg_slot_count != 0)
                          & ((outstanding + claimed) < cfg_slot_count)),
            # Also a gray pointer comparison across the crossing. Only this
            # core pops that queue, so a reading one cycle old can never claim
            # a timestamp that is not there.
            stamp_ready.eq(stamps.readable),
            # A stopped stream rewinds to the first slot and the first packet
            # count, so a restart is a clean sequence for the host to follow.
            If(~enable_sys,
                cursor.eq(0),
                packet_count.eq(0),
                slot_base.eq(cfg_pool_base + cfg_ring_offset + slot_first_offset),
                slot_addr.eq(cfg_pool_base + cfg_ring_offset + slot_first_offset),
            ),
        ]
