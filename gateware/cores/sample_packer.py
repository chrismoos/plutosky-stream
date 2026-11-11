"""The three wire formats, in both directions.

``SamplePacker`` turns a stream of 12-bit I/Q samples into the payload bytes of
a VITA packet and ``SampleUnpacker`` turns those bytes back into samples. Both
are channel-agnostic: a two-channel stream is already interleaved when it
arrives, ch0 then ch1 for each sample period, which is exactly what the wire
format asks for, and a bfp8 block of sixteen samples is then eight sample
periods of both channels sharing one exponent.

cs12 and cs8 are fixed-width and go straight through. bfp8 cannot: the block's
exponent is only known once all sixteen samples have been seen, so each side
keeps two block buffers and works on one while the other fills. That keeps a
sample moving every cycle instead of stalling for the length of a block.

The exponent and rounding rule is the one in ``docs/wire-formats.md`` and
``tests/golden.py`` checks the RTL against it byte for byte, including the
clamp and the -2048 input that is the only 12-bit value needing e = 5.
"""

from migen import *
from migen.genlib.fsm import FSM, NextState, NextValue

from litex.gen import LiteXModule
from litex.soc.interconnect import stream

from cores.axi_hp_master import byte_stream_layout

FORMAT_CS12 = 0
FORMAT_CS8 = 1
FORMAT_BFP8 = 2

SAMPLE_BITS = 12
SAMPLE_MAX = (1 << (SAMPLE_BITS - 1)) - 1
SAMPLE_MIN = -(1 << (SAMPLE_BITS - 1))
MANTISSA_BITS = 8
BFP_BLOCK_SAMPLES = 16
BFP_BLOCK_BYTES = 33
BFP_PAIRS = BFP_BLOCK_SAMPLES // 2
BFP_EXP_BITS = 4
# The exponent is msb(max|x|) - 6, so a 12-bit magnitude reaches 5.
MAX_EXP = 5

# Byte staging between the format logic and the 32-bit output beat.
# Twelve bytes: a beat's worth in hand, a beat's worth of headroom and a beat
# on the way out. The handshakes either side are registered, so room is judged
# on what is staged rather than on what this cycle's transfer is about to free,
# and the headroom is what keeps that from costing a byte a cycle.
STAGE_BYTES = 12
STAGE_BITS = 8 * STAGE_BYTES
BEAT_BYTES = 4


def sample_layout():
    return [("i", SAMPLE_BITS), ("q", SAMPLE_BITS)]


def _magnitude(value):
    """Absolute value of a 12-bit two's-complement sample, in 12 bits.

    2048 is representable because it is the magnitude of -2048 and needs the
    same twelve bits the negative value occupies.
    """
    mag = Signal(SAMPLE_BITS)
    return mag, [If(value[SAMPLE_BITS - 1], mag.eq(-value)).Else(mag.eq(value))]


def _mantissa_round(value, exponent):
    """round_half_away_from_zero(|value| / 2**e), and the sign to apply to it.

    The rounding and the clamping are separated because together they are the
    longest chain in the packer, and a block has cycles to spare for a pipeline
    register between them.
    """
    mag, comb = _magnitude(value)
    rounded = Signal(SAMPLE_BITS + 1)
    comb += [
        Case(exponent, {
            0: rounded.eq(mag),
            **{e: rounded.eq((mag + (1 << (e - 1))) >> e) for e in range(1, MAX_EXP + 1)},
            "default": rounded.eq(mag >> MAX_EXP),
        }),
    ]
    return rounded, value[SAMPLE_BITS - 1], comb


def _mantissa_clamp(rounded, negative):
    """Saturate to a signed byte and apply the sign."""
    limit = Signal(MANTISSA_BITS + 1)
    clamped = Signal(MANTISSA_BITS + 1)
    out = Signal(MANTISSA_BITS)
    comb = [
        limit.eq(Mux(negative, 1 << (MANTISSA_BITS - 1), (1 << (MANTISSA_BITS - 1)) - 1)),
        If(rounded > limit, clamped.eq(limit)).Else(clamped.eq(rounded)),
        If(negative, out.eq(-clamped)).Else(out.eq(clamped)),
    ]
    return out, comb


def _exponent(accumulated):
    """max(0, msb(accumulated) - 6) over the block's or-ed magnitudes."""
    exp = Signal(BFP_EXP_BITS)
    comb = [exp.eq(0)]
    for bit in range(7, SAMPLE_BITS):
        comb = [If(accumulated[bit], exp.eq(bit - 6)).Else(*comb)]
    return exp, comb


class SamplePacker(LiteXModule):
    """Interleaved samples in, VITA payload bytes out.

    ``sink.last`` marks the final sample of a packet and the packer then drains
    everything it holds and raises ``source.last`` on the closing beat. For
    bfp8 the packet's sample count must be a whole number of blocks, which the
    samples-per-packet rule guarantees.
    """

    def __init__(self):
        self.fmt = Signal(2)
        self.clear = Signal()
        self.sink = sink = stream.Endpoint(sample_layout())
        self.source = source = stream.Endpoint(byte_stream_layout())

        # # #

        stage = Signal(STAGE_BITS, reset_less=True)
        stage_bytes = Signal(max=STAGE_BYTES + 1)
        flushing = Signal()

        push_valid = Signal()
        push_ready = Signal()
        push_data = Signal(32)
        push_bytes = Signal(3)
        push = Signal()

        emit = Signal()
        out_bytes = Signal(max=BEAT_BYTES + 1)
        after_emit = Signal(max=STAGE_BYTES + 1)
        landed = Signal(STAGE_BITS)
        drained = Signal()

        self.comb += [
            out_bytes.eq(Mux(stage_bytes < BEAT_BYTES, stage_bytes, BEAT_BYTES)),
            emit.eq(source.valid & source.ready),
            after_emit.eq(stage_bytes - Mux(emit, out_bytes, 0)),
            push_ready.eq(after_emit <= STAGE_BYTES - BEAT_BYTES),
            push.eq(push_valid & push_ready),
            Case(after_emit, {
                n: landed.eq(push_data << (8 * n))
                for n in range(STAGE_BYTES - BEAT_BYTES + 1)
            }),
            source.valid.eq((stage_bytes >= BEAT_BYTES)
                            | (flushing & drained & (stage_bytes != 0))),
            source.data.eq(stage[:32]),
            source.bytes.eq(out_bytes),
            source.last.eq(flushing & drained & (stage_bytes <= BEAT_BYTES)),
        ]
        self.sync += [
            If(self.clear,
                stage.eq(0),
                stage_bytes.eq(0),
                flushing.eq(0),
            ).Elif(emit | push,
                stage.eq(Mux(emit, stage >> (8 * BEAT_BYTES), stage)
                         | Mux(push, landed, 0)),
                stage_bytes.eq(after_emit + Mux(push, push_bytes, 0)),
            ),
        ]

        # cs12 and cs8 feed the stage straight from the sink.
        direct_bytes = Signal(3)
        direct_data = Signal(32)
        cs12 = Signal(24)
        cs8 = Signal(16)
        self.comb += [
            cs12.eq(Cat(sink.i[4:12],
                        Cat(sink.q[8:12], sink.i[0:4]),
                        sink.q[0:8])),
            cs8.eq(Cat(sink.i[4:12], sink.q[4:12])),
            If(self.fmt == FORMAT_CS8,
                direct_data.eq(cs8),
                direct_bytes.eq(2),
            ).Else(
                direct_data.eq(cs12),
                direct_bytes.eq(3),
            ),
        ]

        # bfp8 write side: fill one block buffer while the read side drains the
        # other, keeping the running or of every magnitude for the exponent.
        pairs = Memory(2 * 2 * SAMPLE_BITS, 2 * BFP_PAIRS)
        self.specials += pairs
        pairs_wr = pairs.get_port(write_capable=True)
        # The read enable is the pipeline's own advance: the address register
        # must stall exactly when the pipeline does, or the read runs ahead of
        # the stage consuming it the first time the output back-pressures.
        pairs_rd = pairs.get_port(has_re=True)
        self.specials += pairs_wr, pairs_rd
        exponents = Array([Signal(BFP_EXP_BITS) for _ in range(2)])
        full = Array([Signal() for _ in range(2)])
        wr_bank = Signal()
        wr_idx = Signal(max=BFP_BLOCK_SAMPLES)
        wr_acc = Signal(SAMPLE_BITS)
        even_i = Signal(SAMPLE_BITS, reset_less=True)
        even_q = Signal(SAMPLE_BITS, reset_less=True)

        mag_i, mag_i_comb = _magnitude(sink.i)
        mag_q, mag_q_comb = _magnitude(sink.q)
        self.comb += mag_i_comb + mag_q_comb
        block_acc = Signal(SAMPLE_BITS)
        # The exponent search is a priority encode over the whole block's
        # accumulated magnitude, which is too much to sit behind the two
        # magnitudes and the OR in one cycle. The accumulator is registered at
        # the end of a block and encoded the cycle after, which a block of
        # sixteen samples has ample room for.
        acc_r = Signal(SAMPLE_BITS, reset_less=True)
        exp_pending = Signal()
        exp_bank = Signal()
        block_exp, block_exp_comb = _exponent(acc_r)
        self.comb += [block_acc.eq(wr_acc | mag_i | mag_q)] + block_exp_comb

        bfp_take = Signal()
        block_done = Signal()
        self.comb += [
            bfp_take.eq((self.fmt == FORMAT_BFP8) & sink.valid & sink.ready),
            block_done.eq(bfp_take & (wr_idx == BFP_BLOCK_SAMPLES - 1)),
            pairs_wr.adr.eq(Cat(wr_idx[1:], wr_bank)),
            pairs_wr.dat_w.eq(Cat(even_i, even_q, sink.i, sink.q)),
            pairs_wr.we.eq(bfp_take & wr_idx[0]),
        ]

        # bfp8 read side: two samples, four mantissa bytes, per cycle, through
        # a three stage pipeline. Fetching a pair out of the block buffer,
        # rounding its four components to mantissas and landing those in the
        # byte stage are each far too much for one cycle together, and they are
        # independent, so they overlap instead.
        rd_bank = Signal()
        rd_idx = Signal(max=BFP_PAIRS)
        fetch_idx = Signal(max=BFP_PAIRS)
        pushes = Signal(max=BFP_PAIRS)
        exp_r = Signal(BFP_EXP_BITS, reset_less=True)
        round_r = [Signal(SAMPLE_BITS + 1, reset_less=True) for _ in range(4)]
        neg_r = Signal(4, reset_less=True)
        bytes_r = Signal(32, reset_less=True)
        rd_active = Signal()
        advance = Signal()
        rd_exp = Signal(BFP_EXP_BITS)
        block_clear = Signal()
        bfp_push_valid = Signal()
        bfp_push_data = Signal(32)
        bfp_push_bytes = Signal(3)
        pair = Signal(2 * 2 * SAMPLE_BITS)
        # The read port registers its own output, which is the fetch stage of
        # the read pipeline.
        self.comb += [
            pairs_rd.adr.eq(Cat(fetch_idx, rd_bank)),
            pairs_rd.re.eq(advance),
            pair.eq(pairs_rd.dat_r),
            rd_exp.eq(exponents[rd_bank]),
        ]
        rounds = []
        negatives = []
        for lane in range(4):
            value = pair[lane * SAMPLE_BITS:(lane + 1) * SAMPLE_BITS]
            rounded, negative, comb = _mantissa_round(value, exp_r)
            self.comb += comb
            rounds.append(rounded)
            negatives.append(negative)
        mantissas = []
        for lane in range(4):
            out, comb = _mantissa_clamp(round_r[lane], neg_r[lane])
            self.comb += comb
            mantissas.append(out)
        load_round = [round_r[lane].eq(rounds[lane]) for lane in range(4)]
        load_round.append(neg_r.eq(Cat(*negatives)))

        self.dbg = {"fetch_idx": fetch_idx, "pair": pair, "push": push,
                    "rd_bank": rd_bank, "bytes_r": bytes_r, "pushes": pushes}
        self.rd_fsm = rd_fsm = FSM(reset_state="WAIT")
        rd_fsm.act("WAIT",
            If(full[rd_bank],
                NextValue(fetch_idx, 0),
                NextState("FETCH"),
            ),
        )
        # The memory read port holds the fetched pair, so the address runs one
        # stage ahead of the rounding and there is no separate fetch register.
        rd_fsm.act("FETCH",
            rd_active.eq(1),
            advance.eq(1),
            NextValue(exp_r, rd_exp),
            NextValue(fetch_idx, fetch_idx + 1),
            If(self.clear, NextState("WAIT")).Else(NextState("ROUND")),
        )
        rd_fsm.act("ROUND",
            rd_active.eq(1),
            advance.eq(1),
            *[NextValue(t, v) for t, v in
              [(round_r[n], rounds[n]) for n in range(4)] + [(neg_r, Cat(*negatives))]],
            NextValue(fetch_idx, fetch_idx + 1),
            If(self.clear, NextState("WAIT")).Else(NextState("CLAMP")),
        )
        rd_fsm.act("CLAMP",
            rd_active.eq(1),
            advance.eq(1),
            NextValue(bytes_r, Cat(*mantissas)),
            *[NextValue(t, v) for t, v in
              [(round_r[n], rounds[n]) for n in range(4)] + [(neg_r, Cat(*negatives))]],
            NextValue(fetch_idx, fetch_idx + 1),
            NextValue(pushes, 0),
            If(self.clear, NextState("WAIT")).Else(NextState("PUSH")),
        )
        rd_fsm.act("PUSH",
            rd_active.eq(1),
            advance.eq(push),
            bfp_push_valid.eq(1),
            bfp_push_data.eq(bytes_r),
            bfp_push_bytes.eq(4),
            If(self.clear, NextState("WAIT")).Elif(push,
                NextValue(bytes_r, Cat(*mantissas)),
                *[NextValue(t, v) for t, v in
                  [(round_r[n], rounds[n]) for n in range(4)] + [(neg_r, Cat(*negatives))]],
                NextValue(fetch_idx, fetch_idx + 1),
                If(pushes == BFP_PAIRS - 1,
                    NextState("EXPONENT"),
                ).Else(
                    NextValue(pushes, pushes + 1),
                ),
            ),
        )
        rd_fsm.act("EXPONENT",
            rd_active.eq(1),
            bfp_push_valid.eq(1),
            bfp_push_data.eq(exp_r),
            bfp_push_bytes.eq(1),
            If(self.clear,
                NextState("WAIT"),
            ).Elif(push,
                block_clear.eq(1),
                NextValue(rd_bank, ~rd_bank),
                NextState("WAIT"),
            ),
        )

        self.comb += [
            If(self.fmt == FORMAT_BFP8,
                sink.ready.eq(~flushing & ~full[wr_bank]),
                push_valid.eq(bfp_push_valid),
                push_data.eq(bfp_push_data),
                push_bytes.eq(bfp_push_bytes),
            ).Else(
                sink.ready.eq(~flushing & push_ready),
                push_valid.eq(sink.valid),
                push_data.eq(direct_data),
                push_bytes.eq(direct_bytes),
            ),
            drained.eq((self.fmt != FORMAT_BFP8)
                       | (~full[0] & ~full[1] & ~rd_active & ~exp_pending)),
        ]

        self.sync += [
            If(self.clear,
                full[0].eq(0),
                full[1].eq(0),
                wr_bank.eq(0),
                rd_bank.eq(0),
                wr_idx.eq(0),
                wr_acc.eq(0),
                exp_pending.eq(0),
            ).Else(
                exp_pending.eq(block_done),
                If(bfp_take,
                    If(~wr_idx[0],
                        even_i.eq(sink.i),
                        even_q.eq(sink.q),
                    ),
                    wr_acc.eq(block_acc),
                    wr_idx.eq(wr_idx + 1),
                ),
                If(block_done,
                    acc_r.eq(block_acc),
                    exp_bank.eq(wr_bank),
                    wr_bank.eq(~wr_bank),
                    wr_acc.eq(0),
                    wr_idx.eq(0),
                ),
                If(exp_pending, exponents[exp_bank].eq(block_exp)),
                # Set wins over clear: the two never address the same bank while
                # the read side only ever drains a bank the write side has
                # finished with.
                *[If(exp_pending & (exp_bank == bank),
                     full[bank].eq(1),
                 ).Elif(block_clear & (rd_bank == bank),
                     full[bank].eq(0),
                 ) for bank in range(2)],
            ),
            If(self.clear,
                flushing.eq(0),
            ).Elif(sink.valid & sink.ready & sink.last,
                flushing.eq(1),
            ).Elif(emit & source.last,
                flushing.eq(0),
            ),
        ]


class SampleUnpacker(LiteXModule):
    """VITA payload bytes in, interleaved samples out.

    The byte stage hands one sample's worth of bytes to the format logic every
    cycle. bfp8 collects a block's thirty-three bytes into one buffer while the
    other emits, so a block costs seventeen cycles rather than thirty-three.
    """

    def __init__(self):
        self.fmt = Signal(2)
        self.clear = Signal()
        self.sink = sink = stream.Endpoint(byte_stream_layout())
        self.source = source = stream.Endpoint(sample_layout())
        # High while anything is still held anywhere inside. A bfp8 block
        # buffer goes quiet for a cycle between banks, so a consumer cannot
        # tell the unpacker is finished by watching ``source.valid`` alone.
        self.busy = Signal()

        # # #

        stage = Signal(STAGE_BITS, reset_less=True)
        stage_bytes = Signal(max=STAGE_BYTES + 1)

        pop_bytes = Signal(3)
        pop_ready = Signal()
        pop = Signal()
        push = Signal()
        landed = Signal(STAGE_BITS)

        self.comb += [
            push.eq(sink.valid & sink.ready),
            pop.eq(pop_ready & (stage_bytes >= pop_bytes) & (pop_bytes != 0)),
        ]

        after_pop = Signal(max=STAGE_BYTES + 1)
        next_stage_bytes = Signal(max=STAGE_BYTES + 1)
        take_ready = Signal()
        shifted = Signal(STAGE_BITS)
        self.comb += [
            after_pop.eq(stage_bytes - Mux(pop, pop_bytes, 0)),
            next_stage_bytes.eq(Mux(self.clear, 0,
                                    after_pop + Mux(push, sink.bytes, 0))),
            # Registered, because through the format logic this reached the
            # read master's staging register. It is judged on what the stage is
            # about to hold, which is a beat more cautious than judging it on
            # what is left after this cycle's sample, and the headroom in the
            # stage is what pays for that.
            sink.ready.eq(take_ready),
            Case(pop_bytes, {
                n: shifted.eq(stage >> (8 * n)) for n in range(1, BEAT_BYTES + 1)
            }),
            Case(after_pop, {
                n: landed.eq(sink.data << (8 * n))
                for n in range(STAGE_BYTES - BEAT_BYTES + 1)
            }),
        ]
        self.sync += [
            take_ready.eq(next_stage_bytes <= STAGE_BYTES - BEAT_BYTES),
            If(self.clear,
                stage.eq(0),
                stage_bytes.eq(0),
            ).Elif(pop | push,
                stage.eq(Mux(pop, shifted, stage) | Mux(push, landed, 0)),
                stage_bytes.eq(after_pop + Mux(push, sink.bytes, 0)),
            ),
        ]

        head = Signal(4 * 8)
        self.comb += head.eq(stage[:32])

        # cs12 and cs8 come straight off the head of the stage.
        cs12_i = Signal(SAMPLE_BITS)
        cs12_q = Signal(SAMPLE_BITS)
        cs8_i = Signal(SAMPLE_BITS)
        cs8_q = Signal(SAMPLE_BITS)
        self.comb += [
            cs12_i.eq(Cat(head[12:16], head[0:8])),
            cs12_q.eq(Cat(head[16:24], head[8:12])),
            cs8_i.eq(Cat(Constant(0, 4), head[0:8])),
            cs8_q.eq(Cat(Constant(0, 4), head[8:16])),
        ]

        # bfp8: collect thirty-three bytes into one bank, emit from the other.
        mantissa_pairs = Memory(2 * MANTISSA_BITS, 2 * BFP_BLOCK_SAMPLES)
        self.specials += mantissa_pairs
        mantissa_wr = mantissa_pairs.get_port(write_capable=True)
        # As on the packing side, the read enable is the consumer's own advance
        # so the address register never runs ahead of the stage reading it.
        mantissa_rd = mantissa_pairs.get_port(has_re=True)
        self.specials += mantissa_wr, mantissa_rd
        exponents = Array([Signal(BFP_EXP_BITS) for _ in range(2)])
        full = Array([Signal() for _ in range(2)])
        wr_bank = Signal()
        wr_idx = Signal(max=BFP_BLOCK_SAMPLES)
        rd_bank = Signal()
        rd_idx = Signal(max=BFP_BLOCK_SAMPLES)
        rd_active = Signal()
        block_full = Signal()
        block_clear = Signal()
        bfp_pop_ready = Signal()
        bfp_pop_bytes = Signal(3)
        bfp_pop = Signal()

        rd_exp = Signal(BFP_EXP_BITS)
        rd_pair = Signal(2 * MANTISSA_BITS)
        bfp_i = Signal(SAMPLE_BITS)
        bfp_q = Signal(SAMPLE_BITS)
        # The read port's own output register is the head, held by a gated
        # read enable, and a fetch is only issued once the entry sitting there
        # has been taken. The same rule as the queues in cores/fifo.py.
        fetch_idx = Signal(max=BFP_BLOCK_SAMPLES)
        fetched = Signal(max=BFP_BLOCK_SAMPLES + 1)
        head_valid = Signal()
        bfp_fetch = Signal()
        bfp_consume = Signal()
        self.comb += [
            rd_exp.eq(exponents[rd_bank]),
            rd_pair.eq(mantissa_rd.dat_r),
            mantissa_rd.adr.eq(Cat(fetch_idx, rd_bank)),
            mantissa_rd.re.eq(bfp_fetch),
            bfp_consume.eq(head_valid & source.ready),
            bfp_fetch.eq(full[rd_bank] & (fetched != BFP_BLOCK_SAMPLES)
                         & (~head_valid | bfp_consume)),
        ]
        for name, out, raw in (("i", bfp_i, rd_pair[:MANTISSA_BITS]),
                               ("q", bfp_q, rd_pair[MANTISSA_BITS:])):
            wide = Signal((SAMPLE_BITS + 1, True), name="bfp_wide_" + name)
            signed = Signal((MANTISSA_BITS, True), name="bfp_signed_" + name)
            self.comb += [
                signed.eq(raw),
                wide.eq(signed << rd_exp),
                # A conforming stream never overflows twelve bits, so this only
                # bounds a malformed exponent instead of wrapping it.
                If(wide > SAMPLE_MAX, out.eq(SAMPLE_MAX)).Elif(wide < SAMPLE_MIN, out.eq(SAMPLE_MIN))
                .Else(out.eq(wide)),
            ]

        self.wr_fsm = wr_fsm = FSM(reset_state="MANTISSAS")
        wr_fsm.act("MANTISSAS",
            If(~full[wr_bank],
                bfp_pop_ready.eq(1),
                bfp_pop_bytes.eq(2),
                If(bfp_pop,
                    If(wr_idx == BFP_BLOCK_SAMPLES - 1,
                        NextState("EXPONENT"),
                    ).Else(
                        NextValue(wr_idx, wr_idx + 1),
                    ),
                ),
            ),
            If(self.clear, NextValue(wr_idx, 0), NextState("MANTISSAS")),
        )
        wr_fsm.act("EXPONENT",
            bfp_pop_ready.eq(1),
            bfp_pop_bytes.eq(1),
            If(bfp_pop,
                block_full.eq(1),
                NextValue(wr_idx, 0),
                NextValue(wr_bank, ~wr_bank),
                NextState("MANTISSAS"),
            ),
            If(self.clear, NextValue(wr_idx, 0), NextState("MANTISSAS")),
        )

        self.comb += [
            bfp_pop.eq(pop & (self.fmt == FORMAT_BFP8)),
            mantissa_wr.adr.eq(Cat(wr_idx, wr_bank)),
            mantissa_wr.dat_w.eq(stage[:2 * MANTISSA_BITS]),
            mantissa_wr.we.eq(wr_fsm.ongoing("MANTISSAS") & bfp_pop),
            If(self.fmt == FORMAT_BFP8,
                pop_ready.eq(bfp_pop_ready),
                pop_bytes.eq(bfp_pop_bytes),
                rd_active.eq(head_valid),
                source.valid.eq(rd_active),
                source.i.eq(bfp_i),
                source.q.eq(bfp_q),
            ).Elif(self.fmt == FORMAT_CS8,
                pop_ready.eq(source.ready),
                pop_bytes.eq(2),
                source.valid.eq(stage_bytes >= 2),
                source.i.eq(cs8_i),
                source.q.eq(cs8_q),
            ).Else(
                pop_ready.eq(source.ready),
                pop_bytes.eq(3),
                source.valid.eq(stage_bytes >= 3),
                source.i.eq(cs12_i),
                source.q.eq(cs12_q),
            ),
            block_clear.eq(rd_active & source.ready & (rd_idx == BFP_BLOCK_SAMPLES - 1)),
            self.busy.eq(source.valid | (stage_bytes != 0) | full[0] | full[1]
                         | (wr_idx != 0) | wr_fsm.ongoing("EXPONENT") | head_valid | (fetched != 0)),
        ]

        self.sync += [
            If(self.clear,
                full[0].eq(0),
                full[1].eq(0),
                wr_bank.eq(0),
                rd_bank.eq(0),
                wr_idx.eq(0),
                rd_idx.eq(0),
                fetch_idx.eq(0),
                fetched.eq(0),
                head_valid.eq(0),
            ).Else(
                If(bfp_fetch,
                    fetch_idx.eq(fetch_idx + 1),
                    fetched.eq(fetched + 1),
                ),
                head_valid.eq(bfp_fetch | (head_valid & ~bfp_consume)),
                If(wr_fsm.ongoing("EXPONENT") & bfp_pop,
                    exponents[wr_bank].eq(stage[:BFP_EXP_BITS]),
                ),
                If(rd_active & source.ready,
                    If(rd_idx == BFP_BLOCK_SAMPLES - 1,
                        rd_idx.eq(0),
                        rd_bank.eq(~rd_bank),
                        fetch_idx.eq(0),
                        fetched.eq(0),
                    ).Else(
                        rd_idx.eq(rd_idx + 1),
                    ),
                ),
                *[If(block_full & (wr_bank == bank),
                     full[bank].eq(1),
                 ).Elif(block_clear & (rd_bank == bank),
                     full[bank].eq(0),
                 ) for bank in range(2)],
            ),
        ]
