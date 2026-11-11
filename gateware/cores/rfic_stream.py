"""The sample-rate boundary between the AD9361 PHY and the stream cores.

The PHY hands over one word every four DATA_CLK cycles and what that word holds
depends on the interface mode. In 1R1T it is two consecutive samples of the one
channel, so a word is two sample periods. In 2R2T it is one sample of each
channel, so a word is one sample period. Everything above this file works in
sample periods, so the frontend and backend translate.

``tick`` is that sample period, the unit the timebase counts and every VITA
timestamp is expressed in. The receive frontend produces it from the PHY's own
data valid, which makes it exact by construction rather than by a divider that
could drift.
"""

from migen import *

from litex.gen import LiteXModule

from cores.sample_packer import SAMPLE_BITS

# Matches AD9361PHY1R1T_MODE / AD9361PHY2R2T_MODE in the PHY's mode CSR.
MODE_2R2T = 0
MODE_1R1T = 1

TX_STAGE_DEPTH = 4


class RficRxFrontend(LiteXModule):
    """PHY receive words to per-channel samples and the sample-period tick.

    In 1R1T the second sample of a word is emitted on the cycle after the
    first, so the pair leaves at the same rate it arrived. Channel 1 repeats
    channel 0 there, since the radio only delivers one.
    """

    def __init__(self, phy_source, mode, rfic_cd="rfic"):
        self.tick = Signal()
        self.valid = Signal()
        self.ch0_i = Signal(SAMPLE_BITS)
        self.ch0_q = Signal(SAMPLE_BITS)
        self.ch1_i = Signal(SAMPLE_BITS)
        self.ch1_q = Signal(SAMPLE_BITS)

        # # #

        sync_rfic = getattr(self.sync, rfic_cd)
        second_pending = Signal()
        held_i = Signal(SAMPLE_BITS)
        held_q = Signal(SAMPLE_BITS)

        sync_rfic += [
            self.valid.eq(0),
            second_pending.eq(0),
            If(phy_source.valid,
                self.valid.eq(1),
                self.ch0_i.eq(phy_source.ia),
                self.ch0_q.eq(phy_source.qa),
                If(mode == MODE_1R1T,
                    # Channel 1 mirrors channel 0 so a two-channel stream still
                    # produces a well-defined second lane on a one-channel radio.
                    self.ch1_i.eq(phy_source.ia),
                    self.ch1_q.eq(phy_source.qa),
                    second_pending.eq(1),
                    held_i.eq(phy_source.ib),
                    held_q.eq(phy_source.qb),
                ).Else(
                    self.ch1_i.eq(phy_source.ib),
                    self.ch1_q.eq(phy_source.qb),
                ),
            ).Elif(second_pending,
                self.valid.eq(1),
                self.ch0_i.eq(held_i),
                self.ch0_q.eq(held_q),
                self.ch1_i.eq(held_i),
                self.ch1_q.eq(held_q),
            ),
        ]
        self.comb += self.tick.eq(self.valid)


class RficTxBackend(LiteXModule):
    """Per-channel samples to PHY transmit words.

    The release engines upstream produce one sample per channel per ``tick``.
    A short staging ring absorbs the phase difference between that cadence and
    the PHY's word cadence, which are the same rate but not the same counter.
    An empty ring transmits zeros, which is the same thing an idle stream sends.
    """

    def __init__(self, phy_sink, mode, tick, ch0_i, ch0_q, ch1_i, ch1_q,
                 rfic_cd="rfic"):
        self.starved = Signal(32)

        # # #

        sync_rfic = getattr(self.sync, rfic_cd)
        entries = Array([Signal(4 * SAMPLE_BITS) for _ in range(TX_STAGE_DEPTH)])
        wr = Signal(max=TX_STAGE_DEPTH)
        rd = Signal(max=TX_STAGE_DEPTH)
        level = Signal(max=TX_STAGE_DEPTH + 1)

        rd_next = Signal(max=TX_STAGE_DEPTH)
        first = Signal(4 * SAMPLE_BITS)
        second = Signal(4 * SAMPLE_BITS)
        take = Signal(max=3)
        pull = Signal()

        self.comb += [
            rd_next.eq(Mux(rd == TX_STAGE_DEPTH - 1, 0, rd + 1)),
            first.eq(entries[rd]),
            second.eq(entries[rd_next]),
            take.eq(Mux(mode == MODE_1R1T, 2, 1)),
            pull.eq(phy_sink.valid & phy_sink.ready),
            phy_sink.valid.eq(1),
            If(level == 0,
                phy_sink.ia.eq(0), phy_sink.qa.eq(0),
                phy_sink.ib.eq(0), phy_sink.qb.eq(0),
            ).Elif(mode == MODE_1R1T,
                phy_sink.ia.eq(first[0 * SAMPLE_BITS:1 * SAMPLE_BITS]),
                phy_sink.qa.eq(first[1 * SAMPLE_BITS:2 * SAMPLE_BITS]),
                # A 1R1T word is two sample periods, so a ring holding only one
                # sends the second as silence rather than as stale data.
                If(level >= 2,
                    phy_sink.ib.eq(second[0 * SAMPLE_BITS:1 * SAMPLE_BITS]),
                    phy_sink.qb.eq(second[1 * SAMPLE_BITS:2 * SAMPLE_BITS]),
                ),
            ).Else(
                phy_sink.ia.eq(first[0 * SAMPLE_BITS:1 * SAMPLE_BITS]),
                phy_sink.qa.eq(first[1 * SAMPLE_BITS:2 * SAMPLE_BITS]),
                phy_sink.ib.eq(first[2 * SAMPLE_BITS:3 * SAMPLE_BITS]),
                phy_sink.qb.eq(first[3 * SAMPLE_BITS:4 * SAMPLE_BITS]),
            ),
        ]

        pushed = Signal()
        popped = Signal(max=3)
        self.comb += [
            pushed.eq(tick & (level != TX_STAGE_DEPTH)),
            If(pull,
                If(level >= take, popped.eq(take)).Else(popped.eq(level)),
            ),
        ]
        sync_rfic += [
            If(pushed,
                entries[wr].eq(Cat(ch0_i, ch0_q, ch1_i, ch1_q)),
                wr.eq(Mux(wr == TX_STAGE_DEPTH - 1, 0, wr + 1)),
            ),
            If(popped != 0,
                rd.eq(Mux(rd + popped >= TX_STAGE_DEPTH,
                          rd + popped - TX_STAGE_DEPTH, rd + popped)),
            ),
            level.eq(level + pushed - popped),
            If(pull & (level < take),
                self.starved.eq(self.starved + 1),
            ),
        ]
