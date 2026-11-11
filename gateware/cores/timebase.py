"""The board's sample clock: a 64-bit counter of sample periods.

Every timestamp on the wire is a value of this counter, and both the receive
packetizers and the transmit release engines read it directly in the ``rfic``
domain so a timestamp is exact to the sample. The CSR side is in ``sys``, which
makes latch, load and armed load cross-domain operations:

- ``latch`` samples the counter into a holding register the daemon then reads
  as two words, so a read never tears across a carry.
- ``load`` overwrites the counter with ``load_hi``/``load_lo``.
- ``arm_load`` does the same load the cycle the counter equals
  ``arm_hi``/``arm_lo``. The comparison is exact, so an arm value already in the
  past never fires and the daemon can tell by reading the counter back.
"""

from migen import *
from migen.genlib.cdc import BusSynchronizer, MultiReg, PulseSynchronizer

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRField, CSRStatus, CSRStorage

TICK_BITS = 64


class Timebase(LiteXModule):
    """Parameters: ``tick`` pulses once per sample period in ``rfic``."""

    def __init__(self, tick, rfic_cd="rfic"):
        self.ctrl = CSRStorage(fields=[
            CSRField("latch", size=1, offset=0, pulse=True,
                description="Capture the counter into now_hi/now_lo."),
            CSRField("load", size=1, offset=1, pulse=True,
                description="Overwrite the counter with load_hi/load_lo."),
            CSRField("arm_load", size=1, offset=2,
                description="Load load_hi/load_lo when the counter reaches arm_hi/arm_lo."),
        ], description="Timebase control.")
        self.now_hi = CSRStatus(32, description="High word of the latched counter.")
        self.now_lo = CSRStatus(32, description="Low word of the latched counter.")
        self.load_hi = CSRStorage(32, description="High word written by load or arm_load.")
        self.load_lo = CSRStorage(32, description="Low word written by load or arm_load.")
        self.arm_hi = CSRStorage(32, description="High word of the armed-load trigger count.")
        self.arm_lo = CSRStorage(32, description="Low word of the armed-load trigger count.")

        # The live counter, for anything that needs sample-exact timing.
        self.count = Signal(TICK_BITS)

        # The control bits are read straight out of the storage word rather
        # than through the CSR's own field logic, which only exists once a CSR
        # bank has finalised the register. That keeps the core whole in a
        # simulation that has no bus.
        latch = self.ctrl.re & self.ctrl.storage[0]
        load = self.ctrl.re & self.ctrl.storage[1]
        arm_load = self.ctrl.storage[2]

        # # #

        sync_rfic = getattr(self.sync, rfic_cd)

        load_value = Signal(TICK_BITS)
        arm_value = Signal(TICK_BITS)
        arm_enable = Signal()
        self.specials += [
            MultiReg(Cat(self.load_lo.storage, self.load_hi.storage), load_value, odomain=rfic_cd),
            MultiReg(Cat(self.arm_lo.storage, self.arm_hi.storage), arm_value, odomain=rfic_cd),
            MultiReg(arm_load, arm_enable, odomain=rfic_cd),
        ]

        self.load_ps = load_ps = PulseSynchronizer("sys", rfic_cd)
        self.latch_ps = latch_ps = PulseSynchronizer("sys", rfic_cd)
        self.comb += [
            load_ps.i.eq(load),
            latch_ps.i.eq(latch),
        ]

        latched = Signal(TICK_BITS)
        armed_hit = Signal()
        self.comb += armed_hit.eq(arm_enable & tick & (self.count == arm_value))
        sync_rfic += [
            If(load_ps.o | armed_hit,
                self.count.eq(load_value),
            ).Elif(tick,
                self.count.eq(self.count + 1),
            ),
            If(latch_ps.o,
                latched.eq(self.count),
            ),
        ]

        self.latched_sync = latched_sync = BusSynchronizer(TICK_BITS, rfic_cd, "sys")
        self.comb += [
            latched_sync.i.eq(latched),
            self.now_lo.status.eq(latched_sync.o[:32]),
            self.now_hi.status.eq(latched_sync.o[32:]),
        ]
