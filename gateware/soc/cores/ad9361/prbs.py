#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

# Constants / Helpers ------------------------------------------------------------------------------

def _prbs16_next(state):
    """One step of the AD9361 BIST PRBS LFSR (taps 1,2,4..15; LSB-in shift)."""
    return Cat((
        state[1]  ^ state[2]  ^ state[4]  ^ state[5]  ^
        state[6]  ^ state[7]  ^ state[8]  ^ state[9]  ^
        state[10] ^ state[11] ^ state[12] ^ state[13] ^
        state[14] ^ state[15]),
        state[:-1]
    )

# AD9361 PRBS Generator ----------------------------------------------------------------------------

class AD9361PRBSGenerator(LiteXModule):
    def __init__(self, seed=0x0a54):
        self.ce = Signal(reset=1)
        self.o  = Signal(16)

        # # #

        # PRBS Generation.
        data = Signal(16, reset=seed)
        self.sync += If(self.ce, data.eq(_prbs16_next(data)))
        self.comb += self.o.eq(data)

# AD9361 PRBS Word Generator -----------------------------------------------------------------------

class AD9361PRBSWordGenerator(LiteXModule):
    """PRBS generator for one PHY word.

    In 1R1T the a/b slots are two consecutive samples of one stream, so slot b
    carries the state after slot a and the reference advances by two states per
    accepted word. In 2R2T (``two_channels`` set) the slots are two channels that
    both carry the current state, and the reference advances by one.
    """
    def __init__(self, seed=0x0a54):
        self.ce           = Signal(reset=1)
        self.two_channels = Signal()
        self.o            = Signal(16)
        self.o_next       = Signal(16)
        self.ia           = Signal(12)
        self.qa           = Signal(12)
        self.ib           = Signal(12)
        self.qb           = Signal(12)

        # # #

        data   = Signal(16, reset=seed)
        data_n = Signal(16)
        data_b = Signal(16)

        self.comb += [
            data_n.eq(_prbs16_next(data)),
            data_b.eq(Mux(self.two_channels, data, data_n)),
        ]
        self.sync += If(self.ce,
            data.eq(Mux(self.two_channels, data_n, _prbs16_next(data_n)))
        )
        self.comb += [
            self.o.eq(data),
            self.o_next.eq(data_n),
            self.ia.eq(data[4:16]),
            self.qa.eq(Cat(*[data[11 - i] for i in range(12)])),
            self.ib.eq(data_b[4:16]),
            self.qb.eq(Cat(*[data_b[11 - i] for i in range(12)])),
        ]

# AD9361 PRBS Checker ----------------------------------------------------------------------------

class AD9361PRBSChecker(LiteXModule):
    def __init__(self, seed=0x0a54):
        self.ce     = Signal(reset=1)
        self.i      = Signal(12)
        self.synced = Signal()

        # # #

        error = Signal()

        # # #

        # PRBS reference.
        prbs = AD9361PRBSGenerator(seed=seed)
        prbs = ResetInserter()(prbs)
        self.submodules += prbs

        # PRBS re-synchronization.
        self.comb += prbs.ce.eq(self.ce)
        self.comb += prbs.reset.eq(error)

        # Error generation.
        self.comb += If(self.ce, error.eq(self.i != prbs.o[:12]))


        # Sync generation.
        self.sync_timer = WaitTimer(1024)
        self.comb += self.sync_timer.wait.eq(~error)
        self.comb += self.synced.eq(self.sync_timer.done)

# AD9361 PRBS Word Checker -------------------------------------------------------------------------

class AD9361PRBSWordChecker(LiteXModule):
    """Validate the I/Q samples and their PRBS sequence in each PHY word.

    In 1R1T slot b must hold the state after slot a and the next word continues
    from slot b. In 2R2T (``two_channels`` set) each slot is its own channel and
    continues from its own previous state.
    """
    def __init__(self):
        self.ce           = Signal(reset=1)
        self.two_channels = Signal()
        self.ia           = Signal(12)
        self.qa           = Signal(12)
        self.ib           = Signal(12)
        self.qb           = Signal(12)
        self.synced       = Signal()

        # # #

        qa_rev        = Signal(12)
        qb_rev        = Signal(12)
        state_a       = Signal(16)
        state_b       = Signal(16)
        state_a_next  = Signal(16)
        state_b_next  = Signal(16)
        expected_a    = Signal(16)
        expected_b    = Signal(16)
        have_expected = Signal()
        slots_valid   = Signal()
        error         = Signal()

        self.comb += [
            qa_rev.eq(Cat(*[self.qa[11 - i] for i in range(12)])),
            qb_rev.eq(Cat(*[self.qb[11 - i] for i in range(12)])),
            state_a.eq(Cat(qa_rev[:4], self.ia)),
            state_b.eq(Cat(qb_rev[:4], self.ib)),
            state_a_next.eq(_prbs16_next(state_a)),
            state_b_next.eq(_prbs16_next(state_b)),
            slots_valid.eq(
                (state_a != 0) & (state_b != 0)
                & (self.ia[:8] == qa_rev[4:12])
                & (self.ib[:8] == qb_rev[4:12])
                & (self.two_channels | (state_b == state_a_next))
            ),
            If(self.ce,
                error.eq(
                    ~slots_valid
                    | (have_expected & (state_a != expected_a))
                    | (have_expected & self.two_channels & (state_b != expected_b))
                )
            ),
        ]
        self.sync += If(self.ce,
            have_expected.eq(slots_valid),
            expected_a.eq(Mux(self.two_channels, state_a_next, state_b_next)),
            expected_b.eq(state_b_next),
        )

        # Sync generation.
        self.sync_timer = WaitTimer(1024)
        self.comb += self.sync_timer.wait.eq(~error)
        self.comb += self.synced.eq(self.sync_timer.done)
