"""Wishbone to CSR with two register stages in each direction.

The CSR bus reaches every control and status register in the design. On a part
this size that is a net across most of the die, and post-route it is the
longest path with no logic on it at all, which is a distance only a register
can shorten. LiteX's own bridge already registers the outgoing side
once, at the bridge. A second stage lets the placer put a repeater halfway, so
each hop is half as far, and the read data comes back the same way.

The cost is four extra cycles on a read and two on a write, on a bus the daemon
touches a few thousand times a second.
"""

from migen import *
from migen.genlib.fsm import FSM, NextState, NextValue

from litex.gen import LiteXModule
from litex.soc.interconnect import csr_bus, wishbone

# Cycles between the bridge issuing an address and the twice-registered read
# data being valid: two out, one for the bank's own read register, two back.
READ_LATENCY = 5


class PipelinedCsrBridge(LiteXModule):
    def __init__(self, bus_wishbone=None, bus_csr=None):
        if bus_wishbone is None:
            bus_wishbone = wishbone.Interface()
        if bus_csr is None:
            bus_csr = csr_bus.Interface()
        if bus_wishbone.data_width != bus_csr.data_width:
            raise ValueError("this bridge handles one CSR word per bus word")

        self.wishbone = bus_wishbone
        self.csr = bus_csr

        # # #

        alignment_bytes = bus_csr.alignment // 8
        if bus_wishbone.addressing == "byte":
            csr_adr = bus_wishbone.adr[log2_int(alignment_bytes):]
        else:
            csr_adr = bus_wishbone.adr

        adr = Signal(len(bus_csr.adr))
        we = Signal()
        re = Signal()
        dat_w = Signal(bus_csr.data_width)
        dat_r = Signal(bus_csr.data_width)
        dat_r_back = Signal(bus_csr.data_width)
        waited = Signal(max=READ_LATENCY + 1)

        self.sync += [
            bus_csr.adr.eq(adr),
            bus_csr.we.eq(we),
            bus_csr.re.eq(re),
            bus_csr.dat_w.eq(dat_w),
            dat_r_back.eq(bus_csr.dat_r),
            dat_r.eq(dat_r_back),
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(bus_wishbone.cyc & bus_wishbone.stb,
                NextValue(adr, csr_adr),
                NextValue(dat_w, bus_wishbone.dat_w),
                NextValue(we, bus_wishbone.we & (bus_wishbone.sel != 0)),
                NextValue(re, ~bus_wishbone.we & (bus_wishbone.sel != 0)),
                NextValue(waited, 0),
                NextState("ACCESS"),
            ),
        )
        fsm.act("ACCESS",
            # The strobes are one cycle wide at the slaves. The address and the
            # write data are held, so a slave sees them settled either side.
            NextValue(we, 0),
            NextValue(re, 0),
            NextValue(waited, waited + 1),
            If(waited == READ_LATENCY - 1, NextState("ACK")),
        )
        fsm.act("ACK",
            bus_wishbone.ack.eq(1),
            bus_wishbone.dat_r.eq(dat_r),
            NextState("IDLE"),
        )
