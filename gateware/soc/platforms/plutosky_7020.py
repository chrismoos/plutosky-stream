"""LiteX platform for the OpenSourceSDRLab PlutoSky 7020 (XC7Z020-CLG400 + AD9363).

The AD9363 pinout and I/O standards are verified against the vendor's own
schematic and Vivado constraints for this board: every RX/TX data, clock and
frame pin is LVDS_25 with DIFF_TERM on the RX pairs and LVDS_25 on the TX
pairs, and the single-ended control lines are LVCMOS25. The vendor ships
LVDS_25/LVCMOS25 as the working configuration regardless of the 1.8 V net
label on the power sheet.

Resource attribute names match what ``soc.cores.ad9361.phy.AD9361PHY`` and
``soc.cores.ad9361.spi.AD9361SPIMaster`` read from their ``pads``.
"""

from litex.build.generic_platform import IOStandard, Misc, Pins, Subsignal
from litex.build.xilinx import Xilinx7SeriesPlatform

DEVICE = "xc7z020clg400-1"

# In dual-port LVDS mode each direction has 6 DDR pairs, 12 bits per DATA_CLK
# cycle, so a 12-bit I and Q sample takes two cycles. DATA_CLK runs at twice
# the sample rate in 1R1T and four times in 2R2T. The constraint is the
# fastest the daemon allows, 30.72 MS/s in 1R1T or 15.36 MS/s in 2R2T.
RFIC_DATA_CLK_MAX_HZ = 61_440_000

PL_CLK_HZ = 50_000_000

_rx_lvds = [IOStandard("LVDS_25"), Misc("DIFF_TERM=TRUE")]
_tx_lvds = [IOStandard("LVDS_25")]

_io = [
    # 50 MHz XO Y1 on PL_GCLK. Unused by the SoC (sys comes from FCLK0) but
    # listed so a design can pick it up.
    ("clk50", 0, Pins("N18"), IOStandard("LVCMOS25")),

    ("ad9361_rfic", 0,
        Subsignal("rx_clk_p",   Pins("U18"), *_rx_lvds),
        Subsignal("rx_clk_n",   Pins("U19"), *_rx_lvds),
        Subsignal("rx_frame_p", Pins("Y16"), *_rx_lvds),
        Subsignal("rx_frame_n", Pins("Y17"), *_rx_lvds),
        Subsignal("rx_data_p",  Pins("Y18 T16 V20 T17 T20 W18"), *_rx_lvds),
        Subsignal("rx_data_n",  Pins("Y19 U17 W20 R18 U20 W19"), *_rx_lvds),

        Subsignal("tx_clk_p",   Pins("U14"), *_tx_lvds),
        Subsignal("tx_clk_n",   Pins("U15"), *_tx_lvds),
        Subsignal("tx_frame_p", Pins("V16"), *_tx_lvds),
        Subsignal("tx_frame_n", Pins("W16"), *_tx_lvds),
        Subsignal("tx_data_p",  Pins("V15 V12 W14 T12 T11 U13"), *_tx_lvds),
        Subsignal("tx_data_n",  Pins("W15 W13 Y14 U12 T10 V13"), *_tx_lvds),

        Subsignal("rst_n",  Pins("R19"), IOStandard("LVCMOS25")),
        Subsignal("enable", Pins("T15"), IOStandard("LVCMOS25")),
        Subsignal("txnrx",  Pins("P18"), IOStandard("LVCMOS25")),
        Subsignal("en_agc", Pins("P20"), IOStandard("LVCMOS25")),
        # CTRL_IN2 is R14 on the schematic (the physical routing from AD9363 ball
        # D6). The vendor Vivado XDC lists L17, which is unused in SPI-control
        # mode. R14 follows the schematic.
        Subsignal("ctrl",   Pins("J19 K14 R14 J20"), IOStandard("LVCMOS25")),
        Subsignal("stat",   Pins("L20 L19 K19 T14 P15 M20 M19 N20"), IOStandard("LVCMOS25")),
    ),

    ("ad9361_spi", 0,
        Subsignal("clk",  Pins("V18")),
        Subsignal("mosi", Pins("P16")),
        Subsignal("miso", Pins("V17")),
        Subsignal("cs_n", Pins("R17"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS25"),
    ),

    ("ad9361_sync", 0, Pins("T19"), IOStandard("LVCMOS25")),
]

# PS7 pads are reserved, not routed through the PL. The sizes match what
# LiteX's Zynq7000 core expects.
_ps7_io = [
    ("ps7_clk",   0, Pins(1)),
    ("ps7_porb",  0, Pins(1)),
    ("ps7_srstb", 0, Pins(1)),
    ("ps7_mio",   0, Pins(54)),
    ("ps7_ddram", 0,
        Subsignal("addr",    Pins(15)),
        Subsignal("ba",      Pins(3)),
        Subsignal("cas_n",   Pins(1)),
        Subsignal("ck_n",    Pins(1)),
        Subsignal("ck_p",    Pins(1)),
        Subsignal("cke",     Pins(1)),
        Subsignal("cs_n",    Pins(1)),
        Subsignal("dm",      Pins(4)),
        Subsignal("dq",      Pins(32)),
        Subsignal("dqs_n",   Pins(4)),
        Subsignal("dqs_p",   Pins(4)),
        Subsignal("odt",     Pins(1)),
        Subsignal("ras_n",   Pins(1)),
        Subsignal("reset_n", Pins(1)),
        Subsignal("we_n",    Pins(1)),
        Subsignal("vrn",     Pins(1)),
        Subsignal("vrp",     Pins(1)),
    ),
]

_connectors = []


class Platform(Xilinx7SeriesPlatform):
    default_clk_name = "clk50"
    default_clk_period = 1e9 / PL_CLK_HZ

    def __init__(self, toolchain="vivado"):
        Xilinx7SeriesPlatform.__init__(self, DEVICE, _io, _connectors, toolchain=toolchain)
        self.add_extension(_ps7_io)

    def do_finalize(self, fragment):
        Xilinx7SeriesPlatform.do_finalize(self, fragment)
        clk50 = self.lookup_request("clk50", loose=True)
        if clk50 is not None:
            self.add_period_constraint(clk50, 1e9 / PL_CLK_HZ)
        rfic = self.lookup_request("ad9361_rfic", loose=True)
        if rfic is not None:
            self.add_period_constraint(rfic.rx_clk_p, 1e9 / RFIC_DATA_CLK_MAX_HZ)
