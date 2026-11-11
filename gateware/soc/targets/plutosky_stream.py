#!/usr/bin/env python3
"""The plutosky-stream SoC for the PlutoSky 7020.

A Zynq PS7 with no soft CPU. The daemon on the Cortex-A9 reaches every register
through M_AXI_GP0 at ``CSR_BASE``, the receive packetizers write finished
Ethernet frames into DDR over HP0, and the transmit depacketizers read host
frames back over HP2.

Generate the Vivado project without needing Vivado::

    .venv/bin/python -m soc.targets.plutosky_stream --build --no-compile

Outputs land in ``build/plutosky_stream/gateware`` and the register map the
daemon reads at start-up lands in ``build/plutosky_stream/csr.json``.
"""

import os
from functools import reduce
from operator import or_

from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.soc_core import SoCCore
from litex.soc.cores import cpu as litex_cpu
from litex.soc.cores.cpu.zynq7000.core import Zynq7000
from litex.soc.interconnect import csr_bus, wishbone

from cores.csr_bridge import PipelinedCsrBridge
from cores import rate_filter
from cores.axi_hp_master import (HP_DATA_WIDTH, AxiHpReader, AxiHpWriter,
                                 ReadRegionArbiter, RegionArbiter)
from cores.rfic_stream import RficRxFrontend, RficTxBackend
from cores.rx_packetizer import (FRAME_HEADER_BYTES, HEADER_TEMPLATE_BYTES,
                                 RxPacketizer, VITA_HEADER_BYTES,
                                 VITA_TRAILER_BYTES)
from cores.sample_packer import SAMPLE_BITS
from cores.slot_pool import SlotPool
from cores.timebase import Timebase
from cores.tx_depacketizer import TxDepacketizer
from soc.cores.ad9361.phy import AD9361PHY2R2T_MODE
from soc.cores.ad9361_glue import AD9361Glue
from soc.cores.soc_magic import SOC_MAGIC, SOC_VERSION, add_identity
from soc.platforms import plutosky_7020
from soc.ps7_config import PS7_CONFIG

# The Zynq GP0 aperture is 0x4000_0000-0x7FFF_FFFF. 0x43C0_0000 is where Vivado
# places the first AXI-Lite peripheral and what the daemon and the device tree
# UIO node use.
CSR_BASE = 0x43C0_0000
CSR_ADDRESS_WIDTH = 14  # 2**14 words = 64 KiB
CSR_SIZE = 1 << (CSR_ADDRESS_WIDTH + 2)

# Two fabric clocks from the PS7. The control plane, which is the register
# bus and everything the daemon touches, runs on FCLK0. The streaming datapath,
# which is the two AXI HP masters and the cores between them and the sample
# rings, runs on FCLK1. The register bus reaches every core and cannot be
# placed near all of them, so it gets the slow clock, and the datapath gets
# the fast one with nothing on it that has to span the die.
SYS_CLK_FREQ = int(50e6)
STREAM_CLK_FREQ = int(100e6)

# Nominal PS DDR region for LiteX's memory map, sized so its power-of-two
# rounding stays below the GP0 aperture. The board has 1 GiB.
PS_DDR_REGION_SIZE = 512 * 1024 * 1024
PS_BOOTROM_SIZE = 32 * 1024 * 1024

IDENT = "plutosky-stream VITA 49 SDR"
BUILD_NAME = "plutosky_stream"

DEFAULT_RX_STREAMS = 2
DEFAULT_TX_STREAMS = 2
MAX_STREAMS = 4
# The AD9363 has two RF channels. The datapath always carries both.
RF_CHANNELS = 2

RX_SAMPLE_FIFO_DEPTH = 8192
# TX holds more than two milliseconds of single-channel samples at 30 MS/s.
TX_SAMPLE_FIFO_DEPTH = 65536

# The longest paths in this design are routing, not logic, so placer effort is
# the lever that buys timing margin. These directives change no constraint,
# only how hard Vivado tries.
VIVADO_DIRECTIVES = {
    "vivado_opt_directive": "ExploreWithRemap",
    "vivado_place_directive": "ExtraTimingOpt",
    "vivado_post_place_phys_opt_directive": "AggressiveExplore",
    "vivado_route_directive": "AggressiveExplore",
    "vivado_post_route_phys_opt_directive": "AggressiveExplore",
}

OPENXC7_NEXTPNR_SEED = int(os.environ.get("OPENXC7_SEED", "7"))

# FCLK0 is the IO PLL at 1000 MHz divided by two divisors. Keep the first at
# five and pick the second, which is how the vendor configuration reaches
# 100 MHz.
_FCLK0_PLL_MHZ = 1000
_FCLK0_DIVISOR0 = 5


def _ps7_config_fclk(config, n, freq_hz):
    """A copy of the PS7 configuration with fabric clock ``n`` set to ``freq_hz``.

    Changing the timing constraint alone would only move the target, not the
    clock the PS actually programs, so the two move together.
    """
    divisor1 = max(1, round(_FCLK0_PLL_MHZ / (_FCLK0_DIVISOR0 * freq_hz / 1e6)))
    actual_mhz = _FCLK0_PLL_MHZ / (_FCLK0_DIVISOR0 * divisor1)
    updated = dict(config)
    updated[f"PCW_EN_CLK{n}_PORT"] = "1"
    updated[f"PCW_FPGA_FCLK{n}_ENABLE"] = "1"
    updated[f"PCW_FCLK{n}_PERIPHERAL_CLKSRC"] = "IO PLL"
    updated[f"PCW_FCLK{n}_PERIPHERAL_DIVISOR0"] = str(_FCLK0_DIVISOR0)
    updated[f"PCW_FCLK{n}_PERIPHERAL_DIVISOR1"] = str(divisor1)
    updated[f"PCW_ACT_FPGA{n}_PERIPHERAL_FREQMHZ"] = f"{actual_mhz:.6f}"
    updated[f"PCW_FPGA{n}_PERIPHERAL_FREQMHZ"] = f"{actual_mhz:.1f}"
    return updated, int(actual_mhz * 1e6)


class _CRG(LiteXModule):
    """sys is FCLK0 and its reset, straight from the PS7.

    The receive path reads each AD9361 line off its input buffer, using the
    RFIC's own delay calibration to center the eye, so there is no IDELAYE2 and
    no 200 MHz reference to generate.
    """

    def __init__(self):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_stream = ClockDomain()
        # The soft reset comes straight off a CSR write strobe. Registering it
        # twice in the PS7 domain, which this reset does not touch, keeps that
        # fan-out off a combinational path out of the CSR bus.
        rst_meta = Signal()
        rst_held = Signal()
        self.sync.ps7 += [
            rst_meta.eq(self.rst),
            rst_held.eq(rst_meta),
        ]
        self.comb += [
            ClockSignal("sys").eq(ClockSignal("ps7")),
            ResetSignal("sys").eq(ResetSignal("ps7") | rst_held),
        ]
        # The stream clock comes in from the PS7 on its own pin, wired up by
        # the SoC once the PS7 exists. Its reset is the same pair, released
        # in step with that clock.
        self.specials += AsyncResetSynchronizer(self.cd_stream,
                                                ResetSignal("ps7") | rst_held)


class Zynq7000Stream(Zynq7000):
    """Zynq7000 with the CSR window at CSR_BASE instead of LiteX's GP0 default."""

    @property
    def mem_map(self):
        return {**Zynq7000.mem_map.fget(self), "csr": CSR_BASE}


litex_cpu.CPUS["zynq7000"] = Zynq7000Stream


class StreamSoC(SoCCore):
    def add_csr_bridge(self, name="csr", origin=None, with_register=False):
        """The SoC's own bridge, with a register stage each way on the CSR bus.

        LiteX builds this out of ``AXILite2CSR``, whose CSR side is wired
        straight through. That bus reaches every register in the design, which
        is a net across most of the die and post-route the longest path with
        no logic on it to pipeline.
        """
        bridge = PipelinedCsrBridge(
            wishbone.Interface(address_width=self.bus.address_width,
                               data_width=self.bus.data_width,
                               addressing="word"),
            csr_bus.Interface(address_width=self.csr.address_width,
                              data_width=self.csr.data_width,
                              alignment=self.csr.alignment))
        self.add_module(name=f"{name}_bridge", module=bridge)
        region = SoCRegion(origin=origin, size=2**(self.csr.address_width + 2),
                           cached=False, decode=True)
        self.bus.add_slave(name=name, slave=bridge.wishbone, region=region)
        self.csr.add_master(name=name, master=bridge.csr)
        self.add_config("CSR_DATA_WIDTH", self.csr.data_width)
        self.add_config("CSR_ALIGNMENT", self.csr.alignment)
        self.add_config(f"CSR_ORDERING_{self.csr.ordering.upper()}")

    def __init__(self, sys_clk_freq=SYS_CLK_FREQ, stream_clk_freq=STREAM_CLK_FREQ,
                 toolchain="vivado",
                 rx_streams=DEFAULT_RX_STREAMS, tx_streams=DEFAULT_TX_STREAMS,
                 two_channels=True, rx_iddr=False, **kwargs):
        if not 1 <= rx_streams <= MAX_STREAMS or not 1 <= tx_streams <= MAX_STREAMS:
            raise ValueError(f"between one and {MAX_STREAMS} streams each way")
        platform = plutosky_7020.Platform(toolchain=toolchain)
        if toolchain == "vivado":
            platform.toolchain.pre_synthesis_commands.append(
                "set_msg_config -id {{Synth 8-4445}} -new_severity ERROR")

        self.crg = _CRG()

        kwargs.update(
            cpu_type="zynq7000",
            integrated_sram_size=0,
            integrated_rom_size=0,
            with_uart=False,
            with_timer=False,
            ident=IDENT,
            ident_version=True,
            csr_address_width=CSR_ADDRESS_WIDTH,
        )
        SoCCore.__init__(self, platform, sys_clk_freq, **kwargs)

        # Both fabric clocks are programmed into the PS7 to match what the
        # constraints hold the design to. The vendor handoff has FCLK0 at
        # 100 MHz and FCLK1 off.
        ps7_config, sys_clk_freq = _ps7_config_fclk(PS7_CONFIG, 0, sys_clk_freq)
        ps7_config, stream_clk_freq = _ps7_config_fclk(ps7_config, 1, stream_clk_freq)
        self.cpu.set_ps7(name="ps7_litex", config=ps7_config)
        self.cpu.cpu_params.update(o_FCLK_CLK1=ClockSignal("stream"))
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9 / sys_clk_freq)
        platform.add_period_constraint(self.crg.cd_stream.clk, 1e9 / stream_clk_freq)
        # The three fabric domains only meet at synchronisers and dual-clock
        # block RAMs, so they are timed as unrelated.
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.crg.cd_stream.clk)
        # PS-side regions LiteX expects for the hard CPU. Nothing in the PL decodes them.
        self.bus.add_region("sram", SoCRegion(
            origin=self.cpu.mem_map["sram"],
            size=PS_DDR_REGION_SIZE - self.cpu.mem_map["sram"]))
        self.bus.add_region("rom", SoCRegion(
            origin=self.cpu.mem_map["rom"], size=PS_BOOTROM_SIZE, linker=True))

        add_identity(self)
        self.add_constant("FRAME_HEADER_BYTES", FRAME_HEADER_BYTES)
        self.add_constant("VITA_HEADER_BYTES", VITA_HEADER_BYTES)
        self.add_constant("VITA_TRAILER_BYTES", VITA_TRAILER_BYTES)
        self.add_constant("HEADER_TEMPLATE_BYTES", HEADER_TEMPLATE_BYTES)
        self.add_constant("RX_STREAMS", rx_streams)
        self.add_constant("TX_STREAMS", tx_streams)
        self.add_constant("CHANNELS", RF_CHANNELS if two_channels else 1)
        self.add_constant("RATE_FILTER_PHASE_TAPS", rate_filter.PHASE_TAPS)
        self.add_constant("RATE_FILTER_RATIO_MAX", rate_filter.RATIO_MAX)
        self.add_constant("RATE_FILTER_COEF_BITS", rate_filter.COEF_BITS)
        self.add_constant("RATE_FILTER_RX_FRAC_BITS", rate_filter.RX_FRAC_BITS)
        self.add_constant("RATE_FILTER_TX_FRAC_BITS", rate_filter.TX_FRAC_BITS)
        self.add_constant("SOC_MAGIC", SOC_MAGIC)
        self.add_constant("SOC_VERSION", SOC_VERSION)

        # Radio
        self.ad9361 = AD9361Glue(platform.request("ad9361_rfic"),
                                 platform.request("ad9361_spi"), rx_iddr=rx_iddr)
        if two_channels:
            # 2R2T has a sample of each channel per PHY word.
            self.ad9361.phy.control.storage.reset = Constant(
                AD9361PHY2R2T_MODE, len(self.ad9361.phy.control.storage))
        platform.add_false_path_constraints(self.crg.cd_sys.clk, self.ad9361.cd_rfic.clk)
        platform.add_false_path_constraints(self.crg.cd_stream.clk, self.ad9361.cd_rfic.clk)

        phy_mode = Signal()
        self.specials += MultiReg(self.ad9361.phy.control.storage[0], phy_mode,
                                  odomain="rfic")
        self.rx_frontend = rx_frontend = RficRxFrontend(self.ad9361.phy.source, phy_mode)

        # The timebase and the transmit backend count the radio's tick. The
        # packetizers and the release engines run on their direction's tick.
        tx_i = [Signal(SAMPLE_BITS) for _ in range(RF_CHANNELS)]
        tx_q = [Signal(SAMPLE_BITS) for _ in range(RF_CHANNELS)]
        self.rate_filter = rate = rate_filter.RateFilter(
            phy_mode, rx_frontend.tick,
            [rx_frontend.ch0_i, rx_frontend.ch0_q, rx_frontend.ch1_i, rx_frontend.ch1_q],
            [tx_i[0], tx_q[0], tx_i[1], tx_q[1]])
        rx_ch0_i, rx_ch0_q, rx_ch1_i, rx_ch1_q = rate.rx_out

        # Time and slots
        self.timebase = timebase = Timebase(rx_frontend.tick)
        self.slots = slots = SlotPool()

        # Each core gets its own domain on the stream clock, with a reset
        # register fed by the synchroniser and placed beside the core. One
        # flip-flop driving every control register is over a thousand loads
        # across the die.
        def stream_domain(name):
            cd = ClockDomain(name)
            setattr(self, f"cd_{name}", cd)
            self.comb += cd.clk.eq(ClockSignal("stream"))
            self.sync.stream += cd.rst.eq(ResetSignal("stream"))
            return name

        # Receive
        hp_write = self.cpu.add_axi_hp_slave(clock_domain="stream",
                                             data_width=HP_DATA_WIDTH,
                                             clock_freq=stream_clk_freq)
        hpw_cd = stream_domain("stream_hpw")
        self.hp_writer = hp_writer = ClockDomainsRenamer(hpw_cd)(AxiHpWriter(hp_write))
        self.rx_arbiter = rx_arbiter = ClockDomainsRenamer(hpw_cd)(
            RegionArbiter(hp_writer, rx_streams))

        irqs = []
        for n in range(rx_streams):
            packetizer = RxPacketizer(
                timebase.count, rate.rx_tick,
                rx_ch0_i, rx_ch0_q, rx_ch1_i, rx_ch1_q,
                slots.pool_base.storage, slots.slot_size.storage,
                slots.tx_offset.storage,
                stream_cd=stream_domain(f"stream_rx{n}"),
                sample_fifo_depth=RX_SAMPLE_FIFO_DEPTH,
                count_tick=rx_frontend.tick)
            setattr(self, f"rxpkt{n}", packetizer)
            self.comb += [
                packetizer.cmd.connect(rx_arbiter.cmd[n]),
                packetizer.sink.connect(rx_arbiter.sink[n]),
                packetizer.done.eq(rx_arbiter.done[n]),
            ]
            irqs.append(packetizer.irq)

        # Transmit
        hp_read = self.cpu.add_axi_hp_slave(clock_domain="stream",
                                            data_width=HP_DATA_WIDTH,
                                            clock_freq=stream_clk_freq)
        # HP0/HP1 share a DDR interconnect port. Put the concurrent reader on
        # HP2 so each stream master reaches a different DDR port.
        hp1_keys = [key for key in self.cpu.cpu_params if "_S_AXI_HP1_" in key]
        assert hp1_keys and not any("_S_AXI_HP2_" in key for key in self.cpu.cpu_params)
        for key in hp1_keys:
            self.cpu.cpu_params[key.replace("_S_AXI_HP1_", "_S_AXI_HP2_")] = \
                self.cpu.cpu_params.pop(key)
        self.cpu.add_ps7_config({
            "PCW_USE_S_AXI_HP1": 0,
            "PCW_USE_S_AXI_HP2": 1,
            "PCW_S_AXI_HP2_DATA_WIDTH": HP_DATA_WIDTH,
            "PCW_S_AXI_HP2_ID_WIDTH": 6,
            "PCW_S_AXI_HP2_FREQMHZ": int(stream_clk_freq / 1e6),
        })
        hpr_cd = stream_domain("stream_hpr")
        self.hp_reader = hp_reader = ClockDomainsRenamer(hpr_cd)(AxiHpReader(hp_read))
        self.tx_arbiter = tx_arbiter = ClockDomainsRenamer(hpr_cd)(
            ReadRegionArbiter(hp_reader, tx_streams))

        depacketizers = []
        for n in range(tx_streams):
            depacketizer = TxDepacketizer(
                timebase.count, rate.tx_tick,
                slots.pool_base.storage, slots.slot_size.storage,
                slots.rx_offset.storage,
                stream_cd=stream_domain(f"stream_tx{n}"),
                sample_fifo_depth=TX_SAMPLE_FIFO_DEPTH)
            setattr(self, f"txpkt{n}", depacketizer)
            self.comb += [
                depacketizer.cmd.connect(tx_arbiter.cmd[n]),
                tx_arbiter.source[n].connect(depacketizer.source),
            ]
            depacketizers.append(depacketizer)
            irqs.append(depacketizer.irq)

        self.comb += [
            rate.tx_start.eq(reduce(or_, [d.start for d in depacketizers])),
            rate.tx_busy.eq(reduce(or_, [d.playing for d in depacketizers])),
        ]

        # One radio, so the lowest-numbered running stream drives the air.
        # Overlapping channel masks are a configuration error, not a hardware
        # one, and the daemon is the single owner of both.
        for channel in range(RF_CHANNELS):
            stmts = [tx_i[channel].eq(0), tx_q[channel].eq(0)]
            for depacketizer in reversed(depacketizers):
                source_i = depacketizer.ch0_i if channel == 0 else depacketizer.ch1_i
                source_q = depacketizer.ch0_q if channel == 0 else depacketizer.ch1_q
                stmts = [If(depacketizer.active,
                            tx_i[channel].eq(source_i),
                            tx_q[channel].eq(source_q),
                        ).Else(*stmts)]
            self.comb += stmts

        self.tx_backend = RficTxBackend(self.ad9361.sink, phy_mode, rx_frontend.tick,
                                        *rate.tx_out)

        # One PL-to-PS interrupt line serves receive and transmit completions.
        if irqs:
            self.comb += self.cpu.interrupt[0].eq(reduce(or_, irqs))


class NoSoftwareBuilder(Builder):
    """The PS boots its own Linux, so there is no LiteX BIOS to compile.

    Turning software off also skips the BIOS timer0/UART requirement check and
    the picolibc and compiler-rt lookups, while csr.json is still exported.
    """

    def __init__(self, *args, **kwargs):
        kwargs["compile_software"] = False
        super().__init__(*args, **kwargs)

    def _get_variables_contents(self):
        return "# no software for this SoC\n"


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=plutosky_7020.Platform, description=IDENT)
    parser.add_target_argument("--rx-streams", default=DEFAULT_RX_STREAMS, type=int,
        help=f"Receive packetizer instances, 1 to {MAX_STREAMS}.")
    parser.add_target_argument("--tx-streams", default=DEFAULT_TX_STREAMS, type=int,
        help=f"Transmit depacketizer instances, 1 to {MAX_STREAMS}.")
    parser.add_target_argument("--one-channel", action="store_true",
        help="Come up in 1R1T instead of 2R2T. The daemon can still change the mode.")
    parser.add_target_argument("--rx-iddr", action="store_true",
        help="Capture RX data and frame in dedicated IDDR inputs. Needs a toolchain with working IDDR mapping.")
    parser.add_target_argument("--sys-clk-freq", default=SYS_CLK_FREQ, type=int,
        help="FCLK0 frequency in Hz for the control plane, rounded to what the PS7 divisors can reach.")
    parser.add_target_argument("--stream-clk-freq", default=STREAM_CLK_FREQ, type=int,
        help="FCLK1 frequency in Hz for the streaming datapath, rounded the same way. The radio clock is unaffected.")
    parser.set_defaults(cpu_type="zynq7000")
    args = parser.parse_args()

    soc = StreamSoC(
        sys_clk_freq=args.sys_clk_freq,
        stream_clk_freq=args.stream_clk_freq,
        toolchain=args.toolchain,
        rx_streams=args.rx_streams,
        tx_streams=args.tx_streams,
        two_channels=not args.one_channel,
        rx_iddr=args.rx_iddr,
        **parser.soc_argdict,
    )

    builder_kwargs = dict(parser.builder_argdict)
    build_dir = builder_kwargs.get("output_dir") or os.path.join("build", BUILD_NAME)
    builder_kwargs["output_dir"] = build_dir
    if not builder_kwargs.get("csr_json"):
        builder_kwargs["csr_json"] = os.path.join(build_dir, "csr.json")
    if not builder_kwargs.get("csr_csv"):
        builder_kwargs["csr_csv"] = os.path.join(build_dir, "csr.csv")

    builder = NoSoftwareBuilder(soc, **builder_kwargs)
    if args.build:
        toolchain_kwargs = dict(parser.toolchain_argdict)
        if args.toolchain == "openxc7":
            toolchain_kwargs["timingstrict"] = True
            toolchain_kwargs["seed"] = OPENXC7_NEXTPNR_SEED
        elif args.toolchain == "vivado":
            for step, directive in VIVADO_DIRECTIVES.items():
                if toolchain_kwargs.get(step) in (None, "default"):
                    toolchain_kwargs[step] = directive
        builder.build(build_name=BUILD_NAME, **toolchain_kwargs)


if __name__ == "__main__":
    main()
