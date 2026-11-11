"""AXI3 bursting masters for the PS7 high-performance slave ports.

One write master on HP0 moves finished Ethernet frames into the slot pool and
one read master on HP2 brings host frames back out. Both present the same pair
of interfaces to the rest of the design:

- ``cmd``: a byte address and a byte count, one region per frame
- a 32-bit data stream with a 1..4 byte count on each beat, bytes in memory
  order with the first byte in the low lane

Frames live at a slot's byte 2, so neither end of a region is 64-bit aligned.
The masters own that problem: each holds a twelve-byte staging register that
re-lanes the 32-bit stream onto the 64-bit bus, and the partial first and last
beats have a write strobe instead of a separate aligned copy.

Transfers go out as INCR bursts of up to 16 beats, never crossing a 4 KB
boundary, with several bursts outstanding so a burst's address phase overlaps
the previous burst's data. One region is in flight at a time: the write
master's ``done`` pulses only after the last write response of the region has
returned, which is what lets a packetizer publish a slot knowing the frame is
in DDR.

The budget is 118 MB/s each way, the Gigabit link's payload rate. The staging
register moves four bytes per cycle, so at the 100 MHz PS7 FCLK0 the masters
have roughly three times the headroom they need and DDR latency, not the
bridge, sets the real ceiling.
"""

from functools import reduce
from operator import or_

from migen import *
from migen.genlib.fsm import FSM, NextState

from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.interconnect.axi import BURST_INCR

HP_DATA_WIDTH = 64
HP_ADDR_WIDTH = 32
BEAT_BYTES = HP_DATA_WIDTH // 8
# AXI3 encodes one to sixteen beats in a four-bit AWLEN/ARLEN.
MAX_BURST_BEATS = 16
BOUNDARY_BYTES = 4096
BEATS_PER_BOUNDARY = BOUNDARY_BYTES // BEAT_BYTES
# Sixteen bytes. A beat's worth plus a beat's worth of headroom: the handshakes
# are registered, so room is judged on what is staged rather than on what a
# transfer in the same cycle is about to free, and without that headroom the
# stage would refuse a push on every cycle a beat leaves.
STAGE_BYTES = BEAT_BYTES + 8
STAGE_BITS = 8 * STAGE_BYTES
# Longest region either master handles, a 9000-byte frame with room to spare.
MAX_LENGTH_BITS = 16
BEATS_BITS = MAX_LENGTH_BITS - 2
DEFAULT_OUTSTANDING = 4


def command_layout():
    return [("addr", HP_ADDR_WIDTH), ("length", MAX_LENGTH_BITS)]


def byte_stream_layout():
    """32-bit beats with a 1..4 byte count, first byte in the low lane."""
    return [("data", 32), ("bytes", 3)]


def _burst_beats(beats_left, addr):
    """Beats in the next burst: at most 16, and never over a 4 KB boundary.

    Both caps are narrow by construction, which keeps this off the critical
    path. Fewer than sixteen beats fit below the boundary only in the last
    sixteen of the page, which is exactly the top five bits of the beat index
    being set, and sixteen beats is fewer than what is left to do only when
    everything above the low five bits of the count is clear.
    """
    near_end = Signal()
    capped = Signal(max=MAX_BURST_BEATS + 1)
    beats = Signal(max=MAX_BURST_BEATS + 1)
    comb = [
        near_end.eq(addr[7:12] == (1 << 5) - 1),
        If(near_end,
            capped.eq(MAX_BURST_BEATS - addr[3:7]),
        ).Else(
            capped.eq(MAX_BURST_BEATS),
        ),
        If((beats_left[5:] == 0) & (beats_left[:5] < capped),
            beats.eq(beats_left[:5]),
        ).Else(
            beats.eq(capped),
        ),
    ]
    return beats, comb


def _round_robin(choice, granted, valids):
    """Pick the lowest requester after ``granted``, wrapping to the lowest.

    The statements are built inside out, so the last one written is the
    outermost test and wins.
    """
    count = len(valids)
    stmts = [choice.eq(granted)]
    for n in reversed(range(count)):
        stmts = [If(valids[n], choice.eq(n)).Else(*stmts)]
    for n in reversed(range(count)):
        stmts = [If(valids[n] & (granted < n), choice.eq(n)).Else(*stmts)]
    return stmts


class AxiHpWriter(LiteXModule):
    """Byte stream in, AXI3 write bursts out."""

    def __init__(self, axi, max_outstanding=DEFAULT_OUTSTANDING):
        assert axi.data_width == HP_DATA_WIDTH
        self.cmd = cmd = stream.Endpoint(command_layout())
        self.sink = sink = stream.Endpoint(byte_stream_layout())
        self.done = Signal()
        self.busy = Signal()
        finished = Signal()
        self.resp_errors = Signal(16)

        # # #

        start_off = Signal(3, reset_less=True)
        bytes_left = Signal(MAX_LENGTH_BITS)
        beats_left = Signal(BEATS_BITS)
        burst_addr = Signal(HP_ADDR_WIDTH, reset_less=True)
        stage = Signal(STAGE_BITS, reset_less=True)
        stage_bytes = Signal(max=STAGE_BYTES + 1)
        first_beat = Signal()
        bursts_issued = Signal(BEATS_BITS)
        bursts_done = Signal(BEATS_BITS)
        outstanding = Signal(max=max_outstanding + 1)
        w_left = Signal(max=MAX_BURST_BEATS + 1)
        w_burst_addr = Signal(HP_ADDR_WIDTH, reset_less=True)
        w_beats_left = Signal(BEATS_BITS)
        w_bursts_started = Signal(BEATS_BITS)

        beats, beats_comb = _burst_beats(beats_left, burst_addr)
        self.comb += beats_comb
        # The data phase chops the region exactly as the address phase does, so
        # it recovers each burst's length without a queue between them.
        w_beats, w_beats_comb = _burst_beats(w_beats_left, w_burst_addr)
        self.comb += w_beats_comb

        # The command is taken into two registers and the region starts one
        # cycle later off a registered strobe. That strobe loads every counter
        # in the master at once, so it must be a register rather than the
        # arbiter's valid ANDed with the state machine. A cycle per region of
        # hundreds is free.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            cmd.ready.eq(1),
            If(cmd.valid, NextState("LOAD")),
        )
        fsm.act("LOAD",
            self.busy.eq(1),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self.busy.eq(1),
            If(beats_left == 0, NextState("DRAIN")),
        )
        fsm.act("DRAIN",
            self.busy.eq(1),
            If(bursts_done == bursts_issued,
                finished.eq(1),
                NextState("IDLE"),
            ),
        )
        # The completion is registered. It goes out through the arbiter to
        # whichever core owns the region, and there it lands on the counters
        # that place the next frame, so an unregistered burst-count compare would
        # head a path across two cores. The arbiter releases its grant on this same
        # signal, so a cycle later is still the right requester.
        self.sync += self.done.eq(finished)

        emit = Signal()
        take = Signal()
        accepted_aw = Signal()
        accepted_b = Signal()
        after_emit = Signal(max=STAGE_BYTES + 1)
        shifted = Signal(STAGE_BITS)
        strb = Signal(BEAT_BYTES)
        first_strb = Signal(BEAT_BYTES)
        full_strb = Signal(BEAT_BYTES)
        flush = Signal()
        cmd_accept = Signal()
        take_ready = Signal()
        more_wanted = Signal()
        next_room = Signal(max=STAGE_BYTES + 1)
        cmd_addr = Signal(HP_ADDR_WIDTH, reset_less=True)
        cmd_len = Signal(MAX_LENGTH_BITS, reset_less=True)
        load = Signal()

        self.comb += [
            cmd_accept.eq(cmd.valid & cmd.ready),
            after_emit.eq(stage_bytes - Mux(emit, BEAT_BYTES, 0)),
            first_strb.eq(Replicate(1, BEAT_BYTES) << start_off),
            full_strb.eq(Replicate(1, BEAT_BYTES)),
            Case(after_emit, {
                n: shifted.eq(sink.data << (8 * n)) for n in range(STAGE_BYTES - 3)
            }),
        ]

        # Address phase.
        self.comb += [
            axi.aw.valid.eq(fsm.ongoing("RUN") & (beats_left != 0)
                            & (outstanding != max_outstanding)),
            axi.aw.addr.eq(burst_addr),
            axi.aw.burst.eq(BURST_INCR),
            axi.aw.len.eq(beats - 1),
            axi.aw.size.eq(log2_int(BEAT_BYTES)),
            axi.aw.id.eq(0),
            accepted_aw.eq(axi.aw.valid & axi.aw.ready),
        ]

        # Data phase. ``flush`` covers the tail of a region, where fewer than
        # eight bytes are staged but the burst still owes a beat.
        #
        # The incoming handshake is registered, for the reason the read master
        # registers its own: the byte counter reached the staging register's
        # clock enable through it. It is computed from what the counters are
        # about to hold, so it stays exact, and measuring room on the staged
        # bytes rather than on what is left after a beat still takes four bytes
        # every cycle.
        self.comb += [
            flush.eq((bytes_left == 0) & (stage_bytes != 0)),
            emit.eq(axi.w.valid & axi.w.ready),
            take.eq(sink.valid & sink.ready),
            sink.ready.eq(take_ready),
            # Room is judged as if a full push were coming, so the producer's
            # valid stays out of it: the only thing left of this master in that
            # path is the beat leaving on the bus. That costs a beat's worth of
            # the stage and no cycles.
            next_room.eq(Mux(load, cmd_addr[:3] + 4,
                             after_emit + 4)),
            # Whether the region still wants bytes, as a compare against what
            # this cycle takes rather than a subtract feeding a test for zero.
            more_wanted.eq(Mux(load, cmd_len != 0,
                               (bytes_left[3:] != 0)
                               | (bytes_left[:3] != Mux(take, sink.bytes, 0)))),
            axi.w.valid.eq((w_left != 0) & ((stage_bytes >= BEAT_BYTES) | flush)),
            axi.w.data.eq(stage[:HP_DATA_WIDTH]),
            axi.w.strb.eq(strb),
            axi.w.last.eq(w_left == 1),
            axi.w.id.eq(0),
            If(flush,
                strb.eq(((Constant(1, BEAT_BYTES + 1) << stage_bytes) - 1)
                        & Mux(first_beat, first_strb, full_strb)),
            ).Else(
                strb.eq(Mux(first_beat, first_strb, full_strb)),
            ),
        ]

        # Response phase.
        self.comb += [
            axi.b.ready.eq(1),
            accepted_b.eq(axi.b.valid & axi.b.ready),
        ]

        self.sync += [
            take_ready.eq(more_wanted & (next_room <= STAGE_BYTES - 4)),
            If(cmd_accept,
                cmd_addr.eq(cmd.addr),
                cmd_len.eq(cmd.length),
            ),
            load.eq(cmd_accept),
            If(load,
                start_off.eq(cmd_addr[:3]),
                burst_addr.eq(cmd_addr & ~Constant(BEAT_BYTES - 1, HP_ADDR_WIDTH)),
                bytes_left.eq(cmd_len),
                beats_left.eq((cmd_addr[:3] + cmd_len + BEAT_BYTES - 1) >> 3),
                stage.eq(0),
                stage_bytes.eq(cmd_addr[:3]),
                first_beat.eq(1),
                bursts_issued.eq(0),
                bursts_done.eq(0),
                w_burst_addr.eq(cmd_addr & ~Constant(BEAT_BYTES - 1, HP_ADDR_WIDTH)),
                w_beats_left.eq((cmd_addr[:3] + cmd_len + BEAT_BYTES - 1) >> 3),
                w_bursts_started.eq(0),
                w_left.eq(0),
            ).Else(
                If(emit | take,
                    stage.eq(Mux(emit, stage >> HP_DATA_WIDTH, stage)
                             | Mux(take, shifted, 0)),
                    stage_bytes.eq(after_emit + Mux(take, sink.bytes, 0)),
                ),
                If(accepted_aw,
                    burst_addr.eq(burst_addr + (beats << 3)),
                    beats_left.eq(beats_left - beats),
                    bursts_issued.eq(bursts_issued + 1),
                ),
                If(take,
                    bytes_left.eq(bytes_left - sink.bytes),
                ),
                If(emit,
                    first_beat.eq(0),
                ),
                If(accepted_b,
                    bursts_done.eq(bursts_done + 1),
                ),
                If(w_left == 0,
                    If(w_bursts_started != bursts_issued,
                        w_left.eq(w_beats),
                        w_burst_addr.eq(w_burst_addr + (w_beats << 3)),
                        w_beats_left.eq(w_beats_left - w_beats),
                        w_bursts_started.eq(w_bursts_started + 1),
                    ),
                ).Elif(emit,
                    w_left.eq(w_left - 1),
                ),
            ),
            If(accepted_aw & ~accepted_b,
                outstanding.eq(outstanding + 1),
            ).Elif(accepted_b & ~accepted_aw,
                outstanding.eq(outstanding - 1),
            ),
            If(accepted_b & (axi.b.resp != 0),
                self.resp_errors.eq(self.resp_errors + 1),
            ),
        ]


class AxiHpReader(LiteXModule):
    """AXI3 read bursts in, byte stream out."""

    def __init__(self, axi, max_outstanding=DEFAULT_OUTSTANDING):
        assert axi.data_width == HP_DATA_WIDTH
        self.cmd = cmd = stream.Endpoint(command_layout())
        self.source = source = stream.Endpoint(byte_stream_layout())
        self.busy = Signal()
        self.resp_errors = Signal(16)

        # # #

        start_off = Signal(3, reset_less=True)
        bytes_left = Signal(MAX_LENGTH_BITS)
        beats_left = Signal(BEATS_BITS)
        burst_addr = Signal(HP_ADDR_WIDTH, reset_less=True)
        stage = Signal(STAGE_BITS, reset_less=True)
        stage_bytes = Signal(max=STAGE_BYTES + 1)
        first_beat = Signal()
        outstanding = Signal(max=max_outstanding + 1)

        beats, beats_comb = _burst_beats(beats_left, burst_addr)
        self.comb += beats_comb

        cmd_accept = Signal()
        accept = Signal()
        accepted_ar = Signal()
        emit = Signal()
        after_emit = Signal(max=STAGE_BYTES + 1)
        incoming = Signal(max=BEAT_BYTES + 1)
        out_bytes = Signal(max=5)
        out_valid = Signal()
        out_last = Signal()
        cmd_addr = Signal(HP_ADDR_WIDTH, reset_less=True)
        cmd_len = Signal(MAX_LENGTH_BITS, reset_less=True)
        load = Signal()
        r_ready = Signal()
        whole = Signal()
        next_whole = Signal()
        next_low = Signal(2)
        next_is_four = Signal()
        next_stage_bytes = Signal(max=STAGE_BYTES + 1)
        next_out_bytes = Signal(max=5)
        shifted = Signal(STAGE_BITS)
        landed = Signal(STAGE_BITS)
        landed_hold = Signal(STAGE_BITS)
        landed_take = Signal(STAGE_BITS)
        held = Signal(max=STAGE_BYTES + 1)

        # As in the write master: the command is taken into two registers and
        # the region starts one cycle later off a registered strobe, so the
        # burst count's add reads registers rather than the arbiter's mux.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            cmd.ready.eq(1),
            If(cmd.valid, NextState("LOAD")),
        )
        fsm.act("LOAD",
            self.busy.eq(1),
            NextState("RUN"),
        )
        fsm.act("RUN",
            self.busy.eq(1),
            If(bytes_left == 0, NextState("IDLE")),
        )

        self.comb += [
            cmd_accept.eq(cmd.valid & cmd.ready),
            axi.ar.valid.eq(fsm.ongoing("RUN") & (beats_left != 0)
                            & (outstanding != max_outstanding)),
            axi.ar.addr.eq(burst_addr),
            axi.ar.burst.eq(BURST_INCR),
            axi.ar.len.eq(beats - 1),
            axi.ar.size.eq(log2_int(BEAT_BYTES)),
            axi.ar.id.eq(0),
            accepted_ar.eq(axi.ar.valid & axi.ar.ready),
        ]

        # Both handshakes are registered, off the counters that feed them.
        # Arithmetic on ``bytes_left`` reaching the staging register's clock
        # enable would be the longest path in the design. Each is computed from
        # the value its counter is about to take, so the decision is exact
        # rather than a cycle stale, and the enable sees a register.
        self.comb += [
            incoming.eq(BEAT_BYTES - Mux(first_beat, start_off, 0)),
            emit.eq(source.valid & source.ready),
            accept.eq(axi.r.valid & axi.r.ready),
            axi.r.ready.eq(r_ready),
            source.valid.eq(out_valid),
            source.data.eq(stage[:32]),
            source.bytes.eq(out_bytes),
            source.last.eq(out_last),
            # How many bytes the region still owes, as slice tests rather than
            # a subtract. A beat takes four bytes whenever four are left, and
            # subtracting four leaves the low two bits alone, so the next
            # count's low bits are this one's and whether four remain is a test
            # on bits further up.
            whole.eq(bytes_left[2:] != 0),
            next_whole.eq(Mux(load, cmd_len[2:] != 0,
                              Mux(emit, whole & (bytes_left[3:] != 0), whole))),
            next_low.eq(Mux(load, cmd_len[:2],
                            Mux(emit & ~whole, 0, bytes_left[:2]))),
            next_stage_bytes.eq(Mux(load, 0,
                                    after_emit + Mux(accept, incoming, 0))),
            next_out_bytes.eq(Mux(next_whole, 4, next_low)),
            # The last beat of a region is the one after which nothing is
            # owed. Below four bytes that is always the next beat, and at four
            # or more it is the beat that takes the count down to four.
            next_is_four.eq(Mux(load, cmd_len == 4,
                                Mux(emit & whole, bytes_left == 8,
                                    bytes_left == 4))),
            shifted.eq(axi.r.data >> Mux(first_beat, Cat(Constant(0, 3), start_off), 0)),
            # Where an arriving beat lands depends on how full the stage is
            # after this cycle's transfer, and that is one of two known
            # amounts. Both shifts are built, and whether the consumer took a
            # word picks between them, so the consumer's ready arrives at a two
            # to one mux instead of in front of the whole barrel shifter.
            held.eq(stage_bytes - out_bytes),
            Case(stage_bytes, {
                n: landed_hold.eq(shifted << (8 * n))
                for n in range(STAGE_BYTES - BEAT_BYTES + 1)
            }),
            Case(held, {
                n: landed_take.eq(shifted << (8 * n))
                for n in range(STAGE_BYTES - BEAT_BYTES + 1)
            }),
            landed.eq(Mux(emit, landed_take, landed_hold)),
            after_emit.eq(Mux(emit, held, stage_bytes)),
        ]
        self.sync += [
            out_bytes.eq(next_out_bytes),
            out_valid.eq((next_whole | (next_low != 0))
                         & (next_stage_bytes >= next_out_bytes)),
            out_last.eq(~next_whole | next_is_four),
            # Room for a beat measured on the staged bytes rather than on what
            # is left after an emit. That is one beat's worth more cautious,
            # which still leaves a beat every other cycle and so the same four
            # bytes a cycle out. Nothing is in flight outside a region, so this
            # needs no state from the machine.
            r_ready.eq(next_stage_bytes <= STAGE_BYTES - BEAT_BYTES),
        ]

        self.sync += [
            If(cmd_accept,
                cmd_addr.eq(cmd.addr),
                cmd_len.eq(cmd.length),
            ),
            load.eq(cmd_accept),
            If(load,
                start_off.eq(cmd_addr[:3]),
                burst_addr.eq(cmd_addr & ~Constant(BEAT_BYTES - 1, HP_ADDR_WIDTH)),
                bytes_left.eq(cmd_len),
                beats_left.eq((cmd_addr[:3] + cmd_len + BEAT_BYTES - 1) >> 3),
                stage.eq(0),
                stage_bytes.eq(0),
                first_beat.eq(1),
            ).Else(
                If(emit | accept,
                    stage.eq(Mux(emit, stage >> 32, stage) | Mux(accept, landed, 0)),
                    stage_bytes.eq(after_emit + Mux(accept, incoming, 0)),
                ),
                If(accepted_ar,
                    burst_addr.eq(burst_addr + (beats << 3)),
                    beats_left.eq(beats_left - beats),
                ),
                If(accept,
                    first_beat.eq(0),
                    If(axi.r.resp != 0, self.resp_errors.eq(self.resp_errors + 1)),
                ),
                If(emit,
                    bytes_left.eq(bytes_left - out_bytes),
                ),
            ),
            If(accepted_ar & ~(accept & axi.r.last),
                outstanding.eq(outstanding + 1),
            ).Elif(accept & axi.r.last & ~accepted_ar,
                outstanding.eq(outstanding - 1),
            ),
        ]


class RegionArbiter(LiteXModule):
    """Share one master between several producers, a whole region at a time.

    A region is one Ethernet frame, so the grant is long-lived and the arbiter
    is a formality rather than a per-beat contest. Round robin from the last
    holder keeps one busy stream from starving another.
    """

    def __init__(self, master, count):
        self.cmd = [stream.Endpoint(command_layout()) for _ in range(count)]
        self.sink = [stream.Endpoint(byte_stream_layout()) for _ in range(count)]
        self.done = [Signal() for _ in range(count)]

        # # #

        master_done = master.done
        granted = Signal(max=max(count, 2))
        busy = Signal()
        valids = [c.valid for c in self.cmd]
        request = Signal()
        choice = Signal(max=max(count, 2))
        self.comb += [request.eq(reduce(or_, valids))] + _round_robin(
            choice, granted, valids)

        self.comb += [
            master.cmd.valid.eq(Array([c.valid for c in self.cmd])[granted] & busy),
            master.cmd.addr.eq(Array([c.addr for c in self.cmd])[granted]),
            master.cmd.length.eq(Array([c.length for c in self.cmd])[granted]),
            master.sink.valid.eq(Array([s.valid for s in self.sink])[granted] & busy),
            master.sink.data.eq(Array([s.data for s in self.sink])[granted]),
            master.sink.bytes.eq(Array([s.bytes for s in self.sink])[granted]),
            master.sink.last.eq(Array([s.last for s in self.sink])[granted]),
        ]
        for n in range(count):
            self.comb += [
                self.cmd[n].ready.eq(busy & (granted == n) & master.cmd.ready),
                self.sink[n].ready.eq(busy & (granted == n) & master.sink.ready),
                self.done[n].eq(busy & (granted == n) & master_done),
            ]

        self.sync += [
            If(~busy,
                If(request,
                    granted.eq(choice),
                    busy.eq(1),
                ),
            ).Elif(master_done,
                busy.eq(0),
            ),
        ]


class ReadRegionArbiter(LiteXModule):
    """The read master's counterpart: one region at a time, round robin.

    The grant ends on the last beat of the region rather than on a response,
    because a read is finished when its data has been delivered.
    """

    def __init__(self, master, count):
        self.cmd = [stream.Endpoint(command_layout()) for _ in range(count)]
        self.source = [stream.Endpoint(byte_stream_layout()) for _ in range(count)]

        # # #

        granted = Signal(max=max(count, 2))
        busy = Signal()
        valids = [c.valid for c in self.cmd]
        request = Signal()
        finished = Signal()
        choice = Signal(max=max(count, 2))
        self.comb += [
            request.eq(reduce(or_, valids)),
            finished.eq(master.source.valid & master.source.ready & master.source.last),
        ] + _round_robin(choice, granted, valids)

        self.comb += [
            master.cmd.valid.eq(Array([c.valid for c in self.cmd])[granted] & busy),
            master.cmd.addr.eq(Array([c.addr for c in self.cmd])[granted]),
            master.cmd.length.eq(Array([c.length for c in self.cmd])[granted]),
            master.source.ready.eq(Array([s.ready for s in self.source])[granted] & busy),
        ]
        for n in range(count):
            self.comb += [
                self.cmd[n].ready.eq(busy & (granted == n) & master.cmd.ready),
                self.source[n].valid.eq(busy & (granted == n) & master.source.valid),
                self.source[n].data.eq(master.source.data),
                self.source[n].bytes.eq(master.source.bytes),
                self.source[n].last.eq(master.source.last),
            ]

        self.sync += [
            If(~busy,
                If(request,
                    granted.eq(choice),
                    busy.eq(1),
                ),
            ).Elif(finished,
                busy.eq(0),
            ),
        ]
