"""One transmit stream: host frames out of DDR slots, samples on the air.

The daemon hands over slots the driver has already steered to this stream, as
a (slot, length) pair in a submit queue. The depacketizer reads each frame back
through the shared read master, checks that it belongs to this stream, unpacks
the payload and hands the samples to a release engine, then returns the slot so
the daemon can give it back to the driver.

Release is the part that has to be exact, so it happens in ``rfic`` next to the
timebase rather than in ``sys`` next to the bus. Each packet arrives with its
own sample count and the engine holds it until the timebase reaches that count.
A packet already in the past is dropped and counted as late, because putting it
on the air at the wrong time is worse than not putting it on the air at all. A
gap between one packet's end and the next packet's start is filled with zeros,
so a stream that skips ahead in time stays aligned rather than sliding. An
end-of-burst packet, marked by the VITA T bit and a trailer word with bit 0
set, ends the burst instead: the engine goes quiet and the next packet starts a
new one at its own count.

How many samples a packet holds is not written down anywhere in it. The packet
has a size in words, which covers the payload and the padding that rounds
it up, and one padded size can stand for two different sample counts. Both ends
resolve that the way the sizing rule does, by taking the largest count that
fits, which is the payload length divided by the bytes a sample or a block
costs. A short sequential divide at the head of each frame recovers it.

Only bfp8 is exact on its own, because a block is 33 bytes and three bytes of
padding can never bridge that. A full-size cs12 or cs8 packet is exact too,
because the sizing rule already picks the largest count. A short final packet
is only exact if the host picks a count the rule can reach: even for cs8, since
an odd count pads to the same size as the next even one, and not three modulo
four for cs12, for the same reason three bytes further along.

Two counters say why a stream sounds wrong. ``late`` counts packets that
arrived too late to transmit. ``underrun`` counts sample periods a running
burst had nothing to send, which is the host failing to keep ahead.
"""

from migen import *
from migen.genlib.cdc import BusSynchronizer, MultiReg
from migen.genlib.fsm import FSM, NextState, NextValue

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

from cores.fifo import AsyncFifo, AsyncFifoBuffered, SyncFifo
from cores.axi_hp_master import byte_stream_layout, command_layout
from cores.rx_packetizer import (FRAME_HEADER_BYTES, SLOT_FRAME_OFFSET,
                                 VITA_OFFSET, VITA_TRAILER_BYTES)
from cores.sample_packer import (BFP_BLOCK_BYTES, BFP_BLOCK_SAMPLES, FORMAT_BFP8,
                                 FORMAT_CS8, FORMAT_CS12, SAMPLE_BITS,
                                 SampleUnpacker)

LENGTH_BITS = 16
CS12_SAMPLE_BYTES = 3
CS8_SAMPLE_BYTES = 2
# The payload length is built from shifts and adds, which hold only for these.
assert (CS12_SAMPLE_BYTES, CS8_SAMPLE_BYTES, BFP_BLOCK_BYTES, BFP_BLOCK_SAMPLES) == (3, 2, 33, 16)
COUNTER_BITS = 32
TICK_BITS = 64
SLOT_BITS = 16

SUBMIT_DEPTH = 64
# Room for a 9000-byte frame of the densest format and the one playing out.
DEFAULT_SAMPLE_FIFO_DEPTH = 16384
# Stream-clock cycles the enable must hold before the stream side acts on it.
ENABLE_SETTLE_CYCLES = 4
# Ready packet headers must cover DDR read stalls, not just the packet playing now.
PACKETS_IN_FLIGHT = 128

VITA_TRAILER_BIT = 26
VITA_COUNT_SHIFT = 16

# Beat indices within the frame header, all fixed because every frame has the
# same layout up to the payload.
VITA_BEAT = VITA_OFFSET // 4
TSF_BEAT = (VITA_OFFSET + 16) // 4
HEADER_BEATS = FRAME_HEADER_BYTES // 4


def _submit_layout():
    return [("slot", SLOT_BITS), ("length", LENGTH_BITS)]


class _Divider(LiteXModule):
    """Restoring divide, one bit a cycle. Small divisors, one use per frame."""

    def __init__(self, width, divisor_width):
        self.dividend = Signal(width)
        self.divisor = Signal(divisor_width)
        self.start = Signal()
        self.busy = Signal()
        self.quotient = Signal(width, reset_less=True)

        # # #

        rest = Signal(width, reset_less=True)
        remainder = Signal(width + divisor_width, reset_less=True)
        step = Signal(max=width + 1)
        shifted = Signal(width + divisor_width)

        self.comb += [
            self.busy.eq(step != 0),
            shifted.eq(Cat(rest[width - 1], remainder)),
        ]
        self.sync += [
            If(self.start,
                rest.eq(self.dividend),
                remainder.eq(0),
                self.quotient.eq(0),
                step.eq(width),
            ).Elif(self.busy,
                rest.eq(rest << 1),
                step.eq(step - 1),
                If(shifted >= self.divisor,
                    remainder.eq(shifted - self.divisor),
                    self.quotient.eq(Cat(1, self.quotient)),
                ).Else(
                    remainder.eq(shifted),
                    self.quotient.eq(Cat(0, self.quotient)),
                ),
            ),
        ]


class TxDepacketizer(LiteXModule):
    def __init__(self, timebase_count, tick, pool_base, slot_size, ring_offset,
                 rfic_cd="rfic", stream_cd="sys",
                 sample_fifo_depth=DEFAULT_SAMPLE_FIFO_DEPTH):
        self.ctrl = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, description="Run the stream."),
            CSRField("format", size=2, offset=2, values=[
                ("``0``", "cs12"), ("``1``", "cs8"), ("``2``", "bfp8")],
                description="Wire format."),
            CSRField("channel_mask", size=2, offset=4,
                description="Channels in the stream, bit 0 for channel 0."),
        ], description="Stream control.")
        self.stream_id = CSRStorage(32,
            description="VITA stream id. Frames with another are dropped and counted.")
        self.submit = CSRStorage(fields=[
            CSRField("slot", size=SLOT_BITS, offset=0, description="Slot holding the frame."),
            CSRField("length", size=LENGTH_BITS, offset=SLOT_BITS,
                description="Frame bytes excluding the FCS."),
        ], description="Queue one received frame for transmission.")
        self.submit_level = CSRStatus(8, description="Frames queued but not yet read.")
        self.consumed = CSRStatus(SLOT_BITS,
            description="Oldest finished slot. Reading this register removes it.")
        self.consumed_count = CSRStatus(8, description="Finished slots waiting to be read.")
        self.irq_threshold = CSRStorage(8,
            description="Raise the interrupt when this many finished slots wait to be read. Zero disables it.")
        self.late = CSRStatus(COUNTER_BITS,
            description="Packets dropped because their sample count had passed.")
        self.underrun = CSRStatus(COUNTER_BITS,
            description="Sample periods a running burst had nothing to send.")
        self.gaps = CSRStatus(COUNTER_BITS,
            description="Breaks in the VITA packet count sequence.")
        self.bad_id = CSRStatus(COUNTER_BITS,
            description="Frames this stream cannot use: another stream id, or too short to hold a header.")
        self.fifo_level = CSRStatus(COUNTER_BITS, description="Samples buffered on chip.")

        # Shared read master, through the region arbiter.
        self.cmd = cmd = stream.Endpoint(command_layout())
        self.source = source = stream.Endpoint(byte_stream_layout())

        # Sample-rate outputs, one per channel.
        self.ch0_i = Signal(SAMPLE_BITS)
        self.ch0_q = Signal(SAMPLE_BITS)
        self.ch1_i = Signal(SAMPLE_BITS)
        self.ch1_q = Signal(SAMPLE_BITS)
        self.active = Signal()
        # For the stage that makes ``tick``: a packet is due on this count, and
        # a packet is on the air.
        self.start = Signal()
        self.playing = Signal()
        self.irq = Signal()

        # # #

        sync_rfic = getattr(self.sync, rfic_cd)
        sync_stream = getattr(self.sync, stream_cd)

        # Read the control bits out of the storage word rather than through the
        # CSR's own field logic, which only exists once a CSR bank has finalised
        # the register. That keeps the core whole in a simulation with no bus.
        enable_bit = self.ctrl.storage[0]
        mask_bits = self.ctrl.storage[4:6]

        # The registers live on the control clock and the frame is parsed on
        # the stream clock. Nothing on the stream side reads a register
        # directly: the format and the stream id come across through
        # synchronisers into copies that only follow them while the stream is
        # disabled, so what the daemon wrote before enabling is what the run
        # uses, and the register bank can sit wherever the control bus is.
        enable_stream = Signal()
        fmt = Signal(2)
        cfg_stream_id = Signal(32, reset_less=True)
        config_idle = Signal()
        crossed_fmt = Signal(2)
        crossed_stream_id = Signal(32)
        self.specials += [
            MultiReg(enable_bit, enable_stream, odomain=stream_cd),
            MultiReg(self.ctrl.storage[2:4], crossed_fmt, odomain=stream_cd),
            MultiReg(self.stream_id.storage, crossed_stream_id, odomain=stream_cd),
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
        sync_stream += If(~enable_settled & config_idle,
            fmt.eq(crossed_fmt),
            cfg_stream_id.eq(crossed_stream_id),
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
        sync_stream += If(~enable_settled & config_idle,
            cfg_pool_base.eq(crossed_pool_base),
            cfg_slot_size.eq(crossed_slot_size),
            cfg_ring_offset.eq(crossed_ring_offset),
        )

        self.dbg = {"enable_stream": enable_stream, "enable_settled": enable_settled, "fmt": fmt,
                    "cfg_stream_id": cfg_stream_id, "cfg_pool_base": cfg_pool_base,
                    "cfg_slot_size": cfg_slot_size}

        # The submit queue is written by the daemon on the control clock and
        # read on the stream clock. Completions are strictly ordered, so only
        # their count crosses back and software retires submitted slots FIFO.
        self.submit_fifo = submit_fifo = ClockDomainsRenamer({"write": "sys", "read": stream_cd})(
            AsyncFifoBuffered(SLOT_BITS + LENGTH_BITS, SUBMIT_DEPTH))
        submitted = Signal(8)
        submits_taken = Signal(8)
        consumed_pushed = Signal(8)
        consumed_popped = Signal(8)
        last_consumed = Signal(SLOT_BITS)
        self.submits_taken_sync = submits_taken_sync = BusSynchronizer(8, stream_cd, "sys")
        self.consumed_pushed_sync = consumed_pushed_sync = BusSynchronizer(8, stream_cd, "sys")
        self.last_consumed_sync = last_consumed_sync = BusSynchronizer(
            SLOT_BITS, stream_cd, "sys")
        submit_valid = Signal()
        submit_take = Signal()
        submit_slot = Signal(SLOT_BITS)
        submit_length = Signal(LENGTH_BITS)
        return_valid = Signal()
        return_slot = Signal(SLOT_BITS)
        return_ready = Signal()
        self.comb += [
            submit_fifo.din.eq(self.submit.storage[:SLOT_BITS + LENGTH_BITS]),
            submit_fifo.we.eq(self.submit.re),
            submit_valid.eq(submit_fifo.readable),
            submit_slot.eq(submit_fifo.dout[:SLOT_BITS]),
            submit_length.eq(submit_fifo.dout[SLOT_BITS:]),
            submit_fifo.re.eq(submit_take),
            submits_taken_sync.i.eq(submits_taken),
            self.submit_level.status.eq(submitted - submits_taken_sync.o),
            return_ready.eq(1),
            consumed_pushed_sync.i.eq(consumed_pushed),
            last_consumed_sync.i.eq(last_consumed),
            self.consumed.status.eq(last_consumed_sync.o),
            self.consumed_count.status.eq(consumed_pushed_sync.o - consumed_popped),
            self.irq.eq((self.irq_threshold.storage != 0) &
                        (self.consumed_count.status >= self.irq_threshold.storage)),
        ]
        self.sync += [
            If(submit_fifo.we & submit_fifo.writable, submitted.eq(submitted + 1)),
            If(self.consumed.we & (consumed_pushed_sync.o != consumed_popped),
                consumed_popped.eq(consumed_popped + 1)),
        ]
        sync_stream += [
            If(submit_take, submits_taken.eq(submits_taken + 1)),
            If(return_valid & return_ready,
                last_consumed.eq(return_slot),
                consumed_pushed.eq(consumed_pushed + 1),
            ),
        ]

        # Parsing side
        self.unpacker = unpacker = ClockDomainsRenamer(stream_cd)(SampleUnpacker())
        self.samples = samples = ClockDomainsRenamer({"write": stream_cd, "read": rfic_cd})(
            AsyncFifo(2 * SAMPLE_BITS, sample_fifo_depth))
        self.packets = packets = ClockDomainsRenamer({"write": stream_cd, "read": rfic_cd})(
            AsyncFifo(TICK_BITS + LENGTH_BITS + 1, PACKETS_IN_FLIGHT))
        self.divider = divider = ClockDomainsRenamer(stream_cd)(_Divider(LENGTH_BITS, 6))

        self.comb += [
            samples.flush_write.eq(~enable_stream),
            packets.flush_write.eq(~enable_stream),
        ]

        slot_addr = Signal(32, reset_less=True)
        slot_offset = Signal(32, reset_less=True)
        beat = Signal(max=HEADER_BEATS + 2)
        id_match = Signal()
        room_ok = Signal()
        queue_ok = Signal()
        vita0 = Signal(32, reset_less=True)
        frame_id = Signal(32, reset_less=True)
        stamp = Signal(TICK_BITS, reset_less=True)
        body_bytes = Signal(LENGTH_BITS, reset_less=True)
        sample_count = Signal(LENGTH_BITS, reset_less=True)
        expect_count = Signal(4)
        have_count = Signal()
        current_slot = Signal(SLOT_BITS, reset_less=True)
        packet_aborted = Signal()
        written = Signal(COUNTER_BITS)
        read_out = Signal(COUNTER_BITS)
        both = Signal()
        take = Signal()
        trailer_bytes = Signal(3)
        room = Signal(COUNTER_BITS)
        level = Signal(COUNTER_BITS)
        sample_room = Signal()
        payload_len = Signal(LENGTH_BITS, reset_less=True)
        bad_id = Signal(COUNTER_BITS)
        gaps = Signal(COUNTER_BITS)

        self.read_sync = read_sync = BusSynchronizer(COUNTER_BITS, rfic_cd, stream_cd)
        self.level_sync = level_sync = BusSynchronizer(COUNTER_BITS, stream_cd, "sys")
        self.comb += [
            both.eq(mask_bits == 0b11),
            unpacker.fmt.eq(fmt),
            take.eq(source.valid & source.ready),
            trailer_bytes.eq(Mux(vita0[VITA_TRAILER_BIT], VITA_TRAILER_BYTES, 0)),
            read_sync.i.eq(read_out),
            level.eq(written - read_sync.o),
            room.eq(sample_fifo_depth - level),
            level_sync.i.eq(level),
            self.fifo_level.status.eq(level_sync.o),
        ]

        # Samples in the packet: the largest count whose payload fits what the
        # packet says it holds, which is that length divided by the cost of a
        # sample, or of a block for bfp8.
        divisor = Signal(6)
        quotient = Signal(LENGTH_BITS)
        self.comb += [
            Case(fmt, {
                FORMAT_CS12: divisor.eq(CS12_SAMPLE_BYTES),
                FORMAT_CS8: divisor.eq(CS8_SAMPLE_BYTES),
                FORMAT_BFP8: divisor.eq(BFP_BLOCK_BYTES),
                "default": divisor.eq(CS12_SAMPLE_BYTES),
            }),
            divider.dividend.eq(body_bytes),
            divider.divisor.eq(divisor),
            quotient.eq(divider.quotient),
            If(fmt == FORMAT_BFP8,
                sample_count.eq(quotient << 4),
            ).Else(
                sample_count.eq(quotient),
            ),
        ]
        # Bytes the unpacker must see, which is the payload without its padding:
        # the count times 3, 2 or 33 as shifts and adds.
        payload_bytes = Signal(LENGTH_BITS)
        self.comb += If(fmt == FORMAT_CS12,
            payload_bytes.eq((quotient << 1) + quotient),
        ).Elif(fmt == FORMAT_CS8,
            payload_bytes.eq(quotient << 1),
        ).Else(
            payload_bytes.eq((quotient << 5) + quotient),
        )

        # The head of the submit queue, held in registers a cycle behind the
        # queue's own block RAM output. Everything the state machine decides on
        # entry started at that block RAM otherwise: the length compare, the
        # body length, and the slot going into the multiply that places the
        # frame, which the tool puts inside a DSP where a block RAM output has
        # no business being. ``head_settled`` says the copies match the head,
        # which they do not for the one cycle after a pop.
        #
        # A frame too short to hold a header is not one of ours, whatever
        # steered it here, and is returned without being read.
        submit_body = Signal(LENGTH_BITS, reset_less=True)
        usable = Signal()
        head_slot = Signal(SLOT_BITS, reset_less=True)
        head_settled = Signal()
        head_slot.attr.add(("dont_touch", "true"))
        sync_stream += [
            submit_body.eq(submit_length - FRAME_HEADER_BYTES),
            usable.eq(submit_length > FRAME_HEADER_BYTES),
            head_slot.eq(submit_slot),
            head_settled.eq(submit_valid & ~submit_take),
        ]

        self.fsm = fsm = ClockDomainsRenamer(stream_cd)(FSM(reset_state="IDLE"))
        # An aborted read must finish using the format and pool it started with.
        self.comb += config_idle.eq(fsm.ongoing("IDLE"))
        fsm.act("IDLE",
            unpacker.clear.eq(1),
            If(enable_settled,
                If(head_settled & ~usable & return_ready,
                    submit_take.eq(1),
                    return_valid.eq(1),
                    return_slot.eq(head_slot),
                ).Elif(head_settled & queue_ok & room_ok,
                    NextValue(current_slot, head_slot),
                    NextValue(body_bytes, submit_body),
                    NextValue(beat, 0),
                    NextValue(vita0, 0),
                    submit_take.eq(1),
                    NextState("MULTIPLY"),
                ),
            ).Elif(~enable_stream & head_settled & return_ready,
                # A stopped stream gives the daemon its slots back rather than
                # holding them. Gate this on the enable being deasserted, not
                # merely unsettled: a coherent submit queue can present its head
                # during the brief enable-settling window, and a frame the stream
                # is about to send must wait for settling, not be returned.
                submit_take.eq(1),
                return_valid.eq(1),
                return_slot.eq(head_slot),
            ),
        )
        # The slot address is a multiply and an add, which together are far
        # too long for one cycle at 100 MHz. A frame is hundreds of cycles, so
        # each gets its own, with a third for the latched slot to reach the
        # multiply.
        fsm.act("MULTIPLY",
            NextState("ADDRESS"),
        )
        fsm.act("ADDRESS",
            NextState("HEADER_COMMAND"),
        )
        # The header and the payload are read as two regions. The header is 66
        # bytes, so a single region would put two header bytes and two payload
        # bytes in the same beat, and splitting there costs nothing: the payload
        # starts on a word boundary within the slot either way.
        fsm.act("HEADER_COMMAND",
            cmd.valid.eq(1),
            cmd.addr.eq(slot_addr + SLOT_FRAME_OFFSET),
            cmd.length.eq(FRAME_HEADER_BYTES),
            If(cmd.ready, NextState("HEADER")),
        )
        fsm.act("HEADER",
            source.ready.eq(1),
            If(take,
                NextValue(beat, beat + 1),
                # Every multi-byte field sits two bytes past a word boundary,
                # because the Ethernet header is fourteen bytes long, so each
                # one spans two beats.
                If(beat == VITA_BEAT,
                    NextValue(vita0[16:32], Cat(source.data[24:32], source.data[16:24])),
                ),
                If(beat == VITA_BEAT + 1,
                    NextValue(vita0[0:16], Cat(source.data[8:16], source.data[0:8])),
                    NextValue(frame_id[16:32], Cat(source.data[24:32], source.data[16:24])),
                ),
                If(beat == VITA_BEAT + 2,
                    NextValue(frame_id[0:16], Cat(source.data[8:16], source.data[0:8])),
                ),
                If(beat == TSF_BEAT,
                    NextValue(stamp[48:64], Cat(source.data[24:32], source.data[16:24])),
                ),
                If(beat == TSF_BEAT + 1,
                    NextValue(stamp[16:48], Cat(source.data[24:32], source.data[16:24],
                                                source.data[8:16], source.data[0:8])),
                ),
                If(source.last,
                    NextValue(stamp[0:16], Cat(source.data[8:16], source.data[0:8])),
                    NextState("TRIM"),
                ),
            ),
        )
        fsm.act("TRIM",
            # The trailer is not payload, so take it off before measuring.
            NextValue(body_bytes, body_bytes - trailer_bytes),
            NextState("MEASURE"),
        )
        fsm.act("MEASURE",
            divider.start.eq(1),
            NextState("DIVIDE"),
        )
        fsm.act("DIVIDE",
            If(~divider.busy,
                If(~id_match | (payload_bytes == 0),
                    NextState("RETURN"),
                ).Else(
                    NextState("LENGTH"),
                ),
            ),
        )
        # The payload length is a shift and an add on the recovered sample
        # count, and it goes out as a read command, where the master measures
        # what it owes against it. That is too far to reach in the cycle the
        # divide finishes, so it gets its own.
        fsm.act("LENGTH",
            NextState("BODY_COMMAND"),
        )
        # The read stops at the end of the payload, so the padding and any
        # trailer are simply never fetched.
        fsm.act("BODY_COMMAND",
            cmd.valid.eq(1),
            cmd.addr.eq(slot_addr + SLOT_FRAME_OFFSET + FRAME_HEADER_BYTES),
            cmd.length.eq(payload_len),
            If(cmd.ready, NextState("PAYLOAD")),
        )
        fsm.act("PAYLOAD",
            source.ready.eq(unpacker.sink.ready),
            unpacker.sink.valid.eq(source.valid),
            unpacker.sink.data.eq(source.data),
            unpacker.sink.bytes.eq(source.bytes),
            If(take & source.last, NextState("DRAIN")),
        )
        fsm.act("DRAIN",
            # Let the unpacker finish its last block, so the whole packet is in
            # the ring before it is announced.
            If(~unpacker.busy, NextState("PUBLISH")),
        )
        fsm.act("PUBLISH",
            packets.we.eq(enable_settled & ~packet_aborted),
            NextState("RETURN"),
        )
        fsm.act("RETURN",
            return_valid.eq(1),
            return_slot.eq(current_slot),
            If(return_ready, NextState("IDLE")),
        )

        self.comb += [
            packets.din.eq(Cat(stamp, sample_count, vita0[VITA_TRAILER_BIT])),
            # The ring's writable flag is a gray pointer comparison across the
            # clock crossing, and through the unpacker it reached the read
            # master's staging register. A registered flag with margin keeps it
            # off that path: nothing but this unpacker writes the ring, and it
            # writes at most one sample a cycle, so the eight entries it asks
            # for cover both the cycle it is old and the sample in the register
            # below.
            unpacker.source.ready.eq(sample_room | ~enable_stream | packet_aborted),
        ]

        sync_stream += [
            payload_len.eq(payload_bytes),
            # Deciding a sample's value runs through the whole unpack path, and
            # the ring's data input is a block RAM on the far side of the clock
            # crossing, so the sample and its write enable are registered on
            # the way in.
            samples.din.eq(Cat(unpacker.source.i, unpacker.source.q)),
            samples.we.eq(unpacker.source.valid & sample_room
                          & enable_stream & ~packet_aborted),
            sample_room.eq(room > 8),
            If(~enable_stream,
                written.eq(0),
                packet_aborted.eq(1),
            ).Else(
                If(samples.we, written.eq(written + 1)),
                If(submit_take, packet_aborted.eq(0)),
            ),
            If(fsm.ongoing("IDLE") & head_settled
               & submit_take & ~usable & enable_settled,
                bad_id.eq(bad_id + 1),
            ),
            # Registered so the stream id comparison and the room check are
            # not in front of the state machine, and the multiply and the add
            # that form the slot address are a cycle apart.
            id_match.eq(frame_id == cfg_stream_id),
            room_ok.eq(room > (submit_body >> 1)),
            # The packet queue's writable flag is a gray pointer comparison off
            # a clock crossing, and it reached the read master through the state
            # machine. Only this state machine ever adds to that queue, so a
            # reading one cycle old can never say there is room when there is
            # not.
            queue_ok.eq(packets.writable),
            # The multiply reads its operand from the register the machine
            # latched the slot into, not from the queue's block RAM output.
            # Straight out of a block RAM and into a DSP's cascade is a long
            # way for one cycle.
            slot_offset.eq(current_slot * cfg_slot_size),
            If(~enable_settled & config_idle,
                slot_addr.eq(cfg_pool_base + cfg_ring_offset),
                have_count.eq(0),
            ).Elif(fsm.ongoing("ADDRESS"),
                slot_addr.eq(cfg_pool_base + cfg_ring_offset + slot_offset),
            ),
            If(fsm.ongoing("DIVIDE") & ~divider.busy,
                If(~id_match,
                    bad_id.eq(bad_id + 1),
                ).Else(
                    have_count.eq(1),
                    expect_count.eq(vita0[VITA_COUNT_SHIFT:VITA_COUNT_SHIFT + 4] + 1),
                    If(have_count
                       & (vita0[VITA_COUNT_SHIFT:VITA_COUNT_SHIFT + 4] != expect_count),
                        gaps.eq(gaps + 1),
                    ),
                ),
            ),
        ]

        # Release side
        enable_rfic = Signal()
        both_rfic = Signal()
        self.specials += [
            MultiReg(enable_bit, enable_rfic, odomain=rfic_cd),
            MultiReg(both, both_rfic, odomain=rfic_cd),
        ]
        self.comb += [
            samples.flush_read.eq(~enable_rfic),
            packets.flush_read.eq(~enable_rfic),
        ]

        head_stamp = Signal(TICK_BITS)
        head_samples = Signal(LENGTH_BITS)
        head_eob = Signal()
        self.comb += [
            head_stamp.eq(packets.dout[:TICK_BITS]),
            head_samples.eq(packets.dout[TICK_BITS:TICK_BITS + LENGTH_BITS]),
            head_eob.eq(packets.dout[TICK_BITS + LENGTH_BITS]),
        ]

        bursting = Signal()
        remaining = Signal(LENGTH_BITS)
        late = Signal(COUNTER_BITS)
        underrun = Signal(COUNTER_BITS)
        second = Signal()
        playing = Signal()
        starts_now = Signal()
        too_late = Signal()
        drop = Signal()
        drop_left = Signal(LENGTH_BITS)
        step = Signal(2)

        self.comb += [
            playing.eq(remaining != 0),
            # A dropped packet's samples are still being flushed out of the
            # ring, so nothing else may start until that is finished.
            starts_now.eq(packets.readable & ~playing & (drop_left == 0)
                          & (head_stamp == timebase_count)),
            too_late.eq(packets.readable & ~playing & (drop_left == 0)
                        & (head_stamp < timebase_count)),
            step.eq(Mux(both_rfic, 2, 1)),
            self.active.eq(playing | bursting),
            self.playing.eq(playing),
            self.start.eq(starts_now),
            packets.re.eq(drop | (tick & starts_now)),
            # One read for channel 0 on the tick, a second for channel 1 on the
            # cycle after it, and a free-running drain while a late packet's
            # samples are being thrown away. The tick a packet starts on is the
            # tick its first sample goes out on, which is what makes the
            # timestamp mean what it says.
            samples.re.eq(samples.readable
                          & ((tick & (playing | starts_now)) | second
                             | (drop_left != 0))),
        ]

        sync_rfic += [
            second.eq(0),
            drop.eq(0),
            If(~enable_rfic,
                bursting.eq(0),
                remaining.eq(0),
                drop_left.eq(0),
                self.ch0_i.eq(0), self.ch0_q.eq(0),
                self.ch1_i.eq(0), self.ch1_q.eq(0),
            ).Elif(tick,
                self.ch0_i.eq(0), self.ch0_q.eq(0),
                self.ch1_i.eq(0), self.ch1_q.eq(0),
                If(playing | starts_now,
                    If(samples.readable,
                        self.ch0_i.eq(samples.dout[:SAMPLE_BITS]),
                        self.ch0_q.eq(samples.dout[SAMPLE_BITS:]),
                        If(both_rfic, second.eq(1)),
                        # A malformed packet claiming fewer samples than a
                        # sample period holds must not wrap the counter and
                        # play the ring out.
                        remaining.eq(Mux(playing, remaining - step,
                                         Mux(head_samples > step,
                                             head_samples - step, 0))),
                        If(starts_now, bursting.eq(~head_eob)),
                    ).Else(
                        underrun.eq(underrun + 1),
                    ),
                ).Elif(too_late,
                    drop.eq(1),
                    drop_left.eq(head_samples),
                    late.eq(late + 1),
                ).Elif(bursting & ~packets.readable,
                    underrun.eq(underrun + 1),
                ),
            ).Elif(second,
                self.ch1_i.eq(samples.dout[:SAMPLE_BITS]),
                self.ch1_q.eq(samples.dout[SAMPLE_BITS:]),
            ),
            If(drop_left != 0,
                If(samples.readable, drop_left.eq(drop_left - 1)),
            ),
            If(~enable_rfic,
                read_out.eq(0),
            ).Elif(samples.re, read_out.eq(read_out + 1)),
        ]

        self.bad_id_sync = bad_id_sync = BusSynchronizer(COUNTER_BITS, stream_cd, "sys")
        self.gaps_sync = gaps_sync = BusSynchronizer(COUNTER_BITS, stream_cd, "sys")
        self.comb += [
            bad_id_sync.i.eq(bad_id),
            self.bad_id.status.eq(bad_id_sync.o),
            gaps_sync.i.eq(gaps),
            self.gaps.status.eq(gaps_sync.o),
        ]
        self.late_sync = late_sync = BusSynchronizer(COUNTER_BITS, rfic_cd, "sys")
        self.underrun_sync = underrun_sync = BusSynchronizer(COUNTER_BITS, rfic_cd, "sys")
        self.comb += [
            late_sync.i.eq(late),
            self.late.status.eq(late_sync.o),
            underrun_sync.i.eq(underrun),
            self.underrun.status.eq(underrun_sync.o),
        ]
