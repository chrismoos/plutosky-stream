#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect     import stream

"""
AD9361 RFIC PHY (Dual Port Full Duplex LVDS Mode).

This module provides a physical layer interface for the AD9361 RFIC in Dual Port Full Duplex LVDS mode
on Xilinx 7-series FPGAs. It supports simultaneous RX and TX operations using separate Rx_D[5:0] and
Tx_D[5:0] buses. The module handles 1R1T (4-way interleaving) and 2R2T (8-way interleaving) modes.
For 2R1T or 1R2T configurations, use 2R2T mode and ignore unused slots.

RX data and frame use dedicated IDDR inputs when rx_iddr is selected. The fabric capture
path supports toolchains without working IDDR mapping. Both paths pair rising-edge I
and falling-edge Q nibbles with the same latency. AD9361 delay calibration centers the eye.
The TX path relies on the AD9361's internal clock/data alignment and needs no FPGA phase adjustment.

Key Features:
- Data format           : Two's complement, MSBs first, I/Q interleaved.
- 1R1T mode             : 'a' for first sample, 'b' for second; Interleave: I_MSB, Q_MSB, I_LSB, Q_LSB,...
- 2R2T mode             : 'a' for channel 1,    'b' for channel 2; Interleave: I1_MSB, Q1_MSB, I1_LSB, Q1_LSB, I2_MSB,...
- Configuration via CSR : `mode` (1R1T/2R2T) and `loopback` (enable/disable).
- RX capture            : rising (I) and falling (Q) edges of DATA_CLK.

"""

# Constants ----------------------------------------------------------------------------------------

AD9361PHY2R2T_MODE = 0
AD9361PHY1R1T_MODE = 1

# The RX capture uses no FPGA input delay. These describe the glue's rx_idelay
# CSR (kept for register-map compatibility) and the target's optional IDELAYCTRL
# reference, neither of which drives the capture path.
RX_IDELAY_GROUP = "rx_idelay"
IDELAY_REFCLK_FREQ_HZ = 200_000_000
IDELAY_TAP_BITS = 5

def phy_layout():
    """Defines 12-bit I/Q sample layout for AD9361 PHY channels 'a' and 'b'."""
    return stream.EndpointDescription([
        ("ia", 12),  # I samples for channel/sample 'a'.
        ("qa", 12),  # Q samples for channel/sample 'a'.
        ("ib", 12),  # I samples for channel/sample 'b'.
        ("qb", 12),  # Q samples for channel/sample 'b'.
    ])

# AD9361PHY ----------------------------------------------------------------------------------------

class AD9361PHY(LiteXModule):
    """
    AD9361 RFIC PHY Module.

    Manages the physical layer interface for the AD9361 RFIC, handling RX and TX data streams
    in Dual Port Full Duplex LVDS mode. Leverages AD9361's clock/data alignment and software
    calibration to avoid FPGA-based phase alignment.

    Parameters:
        pads: External FPGA pins connected to the AD9361.
    """
    def __init__(self, pads, rx_iddr=False):
        self.sink    = sink   = stream.Endpoint(phy_layout()) # TX input  stream.
        self.source  = source = stream.Endpoint(phy_layout()) # RX output stream.
        self.control = CSRStorage(fields=[
            CSRField("mode", size=1, offset=0, values=[
                ("``0b0``", "2R2T mode (8-way interleaving)"),
                ("``0b1``", "1R1T mode (4-way interleaving)"),
            ], description="Selects between 1R1T and 2R2T modes"),
            CSRField("loopback", size=1, offset=1, values=[
                ("``0b0``", "Normal operation (no loopback)"),
                ("``0b1``", "Loopback TX data to RX internally"),
            ], description="Enables/disables internal loopback mode"),
        ])

        # RX input-delay control, driven by the glue's rx_idelay CSR in the sys
        # domain. The tap loads on an LD pulse and is shared across every RX
        # data/frame IDELAYE2. The load clock C is sys, so no crossing is needed
        # between the CSR and these primitives.
        self.rx_idelay_value = Signal(IDELAY_TAP_BITS)
        self.rx_idelay_load  = Signal()

        # # #

        # Control Signals
        # ---------------
        mode     = Signal()  # Mode selection (1R1T or 2R2T).
        loopback = Signal()  # Loopback enable/disable.
        self.specials += [
            MultiReg(self.control.fields.mode,     mode,     odomain="rfic"),
            MultiReg(self.control.fields.loopback, loopback, odomain="rfic"),
        ]
        # Mode as seen in the rfic domain, for consumers that interpret the a/b slots.
        self.mode_rfic = mode

        # RX PHY -----------------------------------------------------------------------------------

        # RX Clocking.
        # ------------
        rx_clk_ibufds = Signal()
        self.specials += [
            Instance("IBUFDS", i_I=pads.rx_clk_p, i_IB=pads.rx_clk_n, o_O=rx_clk_ibufds),
            Instance("BUFG", i_I=rx_clk_ibufds, o_O=ClockSignal("rfic")),
            AsyncResetSynchronizer(ClockDomain("rfic"), ResetSignal("sys")),
        ]
        if not rx_iddr:
            self.cd_rfic_n = ClockDomain(reset_less=True)
            self.specials += Instance("BUFG", i_I=~rx_clk_ibufds, o_O=ClockSignal("rfic_n"))

        # RX Framing.
        # -----------
        # RX_FRAME is SDR: high for MSBs (or channel 1 in 2R2T), low for LSBs (or channel 2 in
        # 2R2T). It is captured on the rising edge through two registers so it lands with the
        # same latency as the data nibbles below.
        rx_frame_ibufds = Signal()
        rx_frame_r      = Signal()
        rx_frame        = Signal()
        self.specials += Instance("IBUFDS",
            i_I  = pads.rx_frame_p,
            i_IB = pads.rx_frame_n,
            o_O  = rx_frame_ibufds
        )
        if rx_iddr:
            self.specials += Instance("IDDR", name="rx_frame_iddr", p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                p_SRTYPE="SYNC", p_INIT_Q1=0, p_INIT_Q2=0,
                i_C=ClockSignal("rfic"), i_CE=1, i_D=rx_frame_ibufds,
                i_R=ResetSignal("rfic"), i_S=0, o_Q1=rx_frame)
        else:
            self.sync.rfic += [rx_frame_r.eq(rx_frame_ibufds), rx_frame.eq(rx_frame_r)]

        # RX Counter
        # ----------
        # Maintains a free-running counter, reset to 1 on RX_FRAME rising edge for synchronization.
        rx_count   = Signal(2)
        rx_frame_d = Signal()
        self.sync.rfic += [
            rx_frame_d.eq(rx_frame),
            rx_count.eq(rx_count + 1),
            If(rx_frame & ~rx_frame_d,
                Case(mode, {
                    AD9361PHY1R1T_MODE: rx_count[0].eq(1), # Align for 1R1T.
                    AD9361PHY2R2T_MODE: rx_count   .eq(1), # Align for 2R2T.
                })
            )
        ]

        # RX Data.
        # --------
        # Both capture paths align the data pair and RX_FRAME on the following rising edge.
        rx_data_ibufds  = Signal(6)
        rx_data_half_ir = Signal(6)
        rx_data_half_qn = Signal(6)
        rx_data_half_i  = Signal(6)
        rx_data_half_q  = Signal(6)
        for i in range(6):
            self.specials += Instance("IBUFDS",
                i_I  = pads.rx_data_p[i],
                i_IB = pads.rx_data_n[i],
                o_O  = rx_data_ibufds[i]
            )
        if rx_iddr:
            # Q1 delays the rising nibble by one cycle to pair it with the following falling nibble.
            for i in range(6):
                self.specials += Instance("IDDR", name=f"rx_data_iddr{i}", p_DDR_CLK_EDGE="SAME_EDGE_PIPELINED",
                    p_SRTYPE="SYNC", p_INIT_Q1=0, p_INIT_Q2=0,
                    i_C=ClockSignal("rfic"), i_CE=1, i_D=rx_data_ibufds[i],
                    i_R=ResetSignal("rfic"), i_S=0,
                    o_Q1=rx_data_half_i[i], o_Q2=rx_data_half_q[i])
        else:
            self.sync.rfic += [rx_data_half_ir.eq(rx_data_ibufds),
                               rx_data_half_i.eq(rx_data_half_ir),
                               rx_data_half_q.eq(rx_data_half_qn)]
            self.sync.rfic_n += rx_data_half_qn.eq(rx_data_ibufds)


        # Raw capture taps for the glue's RX snapshot: the two nibbles the
        # assembler consumes, the aligned frame and the beat strobe.
        self.rx_dbg_half_i = rx_data_half_i
        self.rx_dbg_half_q = rx_data_half_q
        self.rx_dbg_frame  = rx_frame

        # RX pin-activity probe.
        # Taps the raw IBUFDS outputs (the same nets the capture registers read),
        # sampled in the rfic domain, to show whether the AD9361 is toggling
        # RX_FRAME/RX_D at the pins. seen0/seen1 latch per line when that line
        # is ever read low/high, so a toggling line sets both bits and a dead
        # line sets one. rx_frame_edges counts RX_FRAME
        # rising edges. All are read-only diagnostics crossed to sys in the glue.
        self.rx_frame_edges = Signal(16)
        self.rx_data_seen0  = Signal(6)
        self.rx_data_seen1  = Signal(6)
        self.rx_frame_seen0 = Signal()
        self.rx_frame_seen1 = Signal()
        rx_frame_ibufds_d = Signal()
        self.sync.rfic += [
            rx_frame_ibufds_d.eq(rx_frame_ibufds),
            If(rx_frame_ibufds & ~rx_frame_ibufds_d,
                self.rx_frame_edges.eq(self.rx_frame_edges + 1),
            ),
            self.rx_frame_seen0.eq(self.rx_frame_seen0 | ~rx_frame_ibufds),
            self.rx_frame_seen1.eq(self.rx_frame_seen1 |  rx_frame_ibufds),
            self.rx_data_seen0.eq(self.rx_data_seen0 | ~rx_data_ibufds),
            self.rx_data_seen1.eq(self.rx_data_seen1 |  rx_data_ibufds),
        ]

        rx_data_ia  = Signal(12)
        rx_data_qa  = Signal(12)
        rx_data_ib  = Signal(12)
        rx_data_qb  = Signal(12)
        rx_valid    = Signal()
        rx_source_ia = Signal(12)
        rx_source_qa = Signal(12)
        rx_source_ib = Signal(12)
        rx_source_qb = Signal(12)
        self.sync.rfic += [
            Case(rx_count[1], {
                0b0: [  # Assemble IA/QA: MSBs in high bits, LSBs in low bits.
                    rx_data_ia[0: 6].eq(rx_data_half_i),
                    rx_data_ia[6:12].eq(rx_data_ia[0:6]),
                    rx_data_qa[0: 6].eq(rx_data_half_q),
                    rx_data_qa[6:12].eq(rx_data_qa[0:6]),
                ],
                0b1: [  # Assemble IB/QB: MSBs in high bits, LSBs in low bits.
                    rx_data_ib[0: 6].eq(rx_data_half_i),
                    rx_data_ib[6:12].eq(rx_data_ib[0:6]),
                    rx_data_qb[0: 6].eq(rx_data_half_q),
                    rx_data_qb[6:12].eq(rx_data_qb[0:6]),
                ]
            })
        ]

        # RX Source Interface.
        # --------------------
        # Outputs assembled RX samples when valid (rx_count == 0). The source
        # is free-running: valid pulses for a single cycle per sample set and
        # is not held under backpressure, so samples are dropped when the
        # downstream stalls (overflow accounting happens at the DMA level).
        self.sync.rfic += [
            rx_valid.eq(0),
            If(rx_count == 0,
                rx_valid.eq(1),
                rx_source_ia.eq(rx_data_ia),
                rx_source_qa.eq(rx_data_qa),
                rx_source_ib.eq(rx_data_ib),
                rx_source_qb.eq(rx_data_qb),
            )
        ]

        # TX -> RX Loopback ------------------------------------------------------------------------

        # Routes TX data to RX when loopback is enabled. Keep an RFIC-domain FIFO
        # here instead of a single registered word so TX serializer pacing is
        # decoupled from RX-side Ethernet and backpressure pauses.
        self.rx_dbg_valid = rx_valid
        self.loopback_fifo = loopback_fifo = ClockDomainsRenamer("rfic")(
            stream.SyncFIFO(phy_layout(), depth=16, buffered=True)
        )
        self.comb += [
            If(loopback,
                loopback_fifo.source.connect(source),
            ).Else(
                loopback_fifo.source.ready.eq(1),
                source.valid.eq(rx_valid),
                source.ia.eq(rx_source_ia),
                source.qa.eq(rx_source_qa),
                source.ib.eq(rx_source_ib),
                source.qb.eq(rx_source_qb),
            )
        ]

        # TX PHY -----------------------------------------------------------------------------------

        # TX Sink Interface.
        # ------------------
        # Accepts new TX samples just before the serializer wraps to count 0,
        # so the next word starts with the freshly latched IA/QA MSBs.
        tx_count = Signal(2)
        self.sync.rfic += tx_count.eq(tx_count + 1)
        self.comb += [
            sink.ready.eq((tx_count == 3) & (~loopback | loopback_fifo.sink.ready)),
            loopback_fifo.sink.valid.eq(loopback & sink.valid & (tx_count == 3)),
            loopback_fifo.sink.ia.eq(sink.ia),
            loopback_fifo.sink.qa.eq(sink.qa),
            loopback_fifo.sink.ib.eq(sink.ib),
            loopback_fifo.sink.qb.eq(sink.qb),
        ]

        # TX Data Latching.
        # -----------------
        # Latches TX samples when ready; defaults to 0 if invalid to avoid spurs.
        tx_data_ia = Signal(12)
        tx_data_qa = Signal(12)
        tx_data_ib = Signal(12)
        tx_data_qb = Signal(12)
        self.sync.rfic += [
            If(sink.ready,
                # Clear to avoid spurs.
                tx_data_ia.eq(0),
                tx_data_qa.eq(0),
                tx_data_ib.eq(0),
                tx_data_qb.eq(0),
                If(sink.valid,
                    # Latch valid samples.
                    tx_data_ia.eq(sink.ia),
                    tx_data_qa.eq(sink.qa),
                    tx_data_ib.eq(sink.ib),
                    tx_data_qb.eq(sink.qb),
                )
            )
        ]

        # TX Clocking.
        # ------------
        # Output: FB_CLK to AD9361, derived from RFIC clock (same frequency as DATA_CLK).
        tx_clk_obufds = Signal()
        self.specials += [
            Instance("ODDR",
                p_DDR_CLK_EDGE = "SAME_EDGE",
                i_C  = ClockSignal("rfic"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D1 = 1,
                i_D2 = 0,
                o_Q  = tx_clk_obufds,
            ),
            Instance("OBUFDS",
                i_I  = tx_clk_obufds,
                o_O  = pads.tx_clk_p,
                o_OB = pads.tx_clk_n
            )
        ]


        # TX Framing
        # ----------
        # Generates TX_FRAME: high for MSBs (or channel 1 in 2R2T), low for LSBs(or channel 2 in
        # 2R2T). Uses SDR with 50% duty cycle.
        tx_frame = Signal()
        tx_frame_obufds = Signal()
        self.comb += [
            If(mode == AD9361PHY1R1T_MODE,
                Case(tx_count, {  # 1R1T: 4-way interleaving over 2 clocks.
                    0b00 : tx_frame.eq(1),
                    0b01 : tx_frame.eq(0),
                    0b10 : tx_frame.eq(1),
                    0b11 : tx_frame.eq(0),
                })
            ).Elif(mode == AD9361PHY2R2T_MODE,
                Case(tx_count, {  # 2R2T: 8-way interleaving over 4 clocks.
                    0b00 : tx_frame.eq(1),
                    0b01 : tx_frame.eq(1),
                    0b10 : tx_frame.eq(0),
                    0b11 : tx_frame.eq(0),
                })
            )
        ]
        self.specials += [
            Instance("ODDR",
                p_DDR_CLK_EDGE = "SAME_EDGE",
                i_C  = ClockSignal("rfic"),
                i_CE = 1,
                i_S  = 0,
                i_R  = 0,
                i_D1 = tx_frame,
                i_D2 = tx_frame,
                o_Q  = tx_frame_obufds,
            ),
            Instance("OBUFDS",
                i_I  = tx_frame_obufds,
                o_O  = pads.tx_frame_p,
                o_OB = pads.tx_frame_n
            )
        ]

        # TX Data
        # -------
        # Outputs TX_D[5:0] in DDR mode: I samples on rising edge, Q samples on falling edge.
        # Sequences MSBs first, then LSBs; I and Q interleaved. Relies on AD9361's alignment for
        # correct data transmission.
        tx_data_half_i = Signal(6)
        tx_data_half_q = Signal(6)
        tx_data_obufds = Signal(6)
        self.comb += [
            Case(tx_count, {
                0b00: [  # IA/QA MSBs.
                    tx_data_half_i.eq(tx_data_ia[6:12]),
                    tx_data_half_q.eq(tx_data_qa[6:12]),
                ],
                0b01: [  # IA/QA LSBs.
                    tx_data_half_i.eq(tx_data_ia[0:6]),
                    tx_data_half_q.eq(tx_data_qa[0:6]),
                ],
                0b10: [  # IB/QB MSBs.
                    tx_data_half_i.eq(tx_data_ib[6:12]),
                    tx_data_half_q.eq(tx_data_qb[6:12]),
                ],
                0b11: [  # IB/QB LSBs.
                    tx_data_half_i.eq(tx_data_ib[0:6]),
                    tx_data_half_q.eq(tx_data_qb[0:6]),
                ]
            })
        ]
        for i in range(6):
            self.specials += [
                Instance("ODDR",
                    p_DDR_CLK_EDGE = "SAME_EDGE",
                    i_C  = ClockSignal("rfic"),
                    i_CE = 1,
                    i_S  = 0,
                    i_R  = 0,
                    i_D1 = tx_data_half_i[i],
                    i_D2 = tx_data_half_q[i],
                    o_Q  = tx_data_obufds[i],
                ),
                Instance("OBUFDS",
                    i_I  = tx_data_obufds[i],
                    o_O  = pads.tx_data_p[i],
                    o_OB = pads.tx_data_n[i]
                )
            ]
