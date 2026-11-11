"""AD9361/AD9363 glue: LVDS PHY, SPI master and control CSRs.

CSR layout follows LiteX-M2SDR's ``AD9361RFIC`` so the daemon's SPI/GPIO
shim can drive it: ``config`` (rst_n bit 0, enable bit 1, txnrx bit 4,
en_agc bit 5), ``ctrl``/``stat`` for CTRL_IN/CTRL_OUT, ``spi_*`` from the
SPI master and ``phy_control`` from the PHY.

The PHY runs in the ``rfic`` clock domain it recovers from DATA_CLK. The
stream path feeds ``sink`` (one PHY word = two consecutive 12-bit I/Q samples in
1R1T mode) in that domain. For interface testing without a stream the TX
source can be switched to a PRBS or a constant pattern. The RX stream runs
through the M2SDR PRBS checker and is also exposed as ``rx_valid``/``rx_i``/
``rx_q`` (plus the second 1R1T sample ``rx_ib``/``rx_qb``) for a capture engine.
"""

from migen import *
from migen.genlib.cdc import BusSynchronizer, MultiReg, PulseSynchronizer

from litex.gen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from soc.cores.ad9361.phy import (
    AD9361PHY,
    AD9361PHY1R1T_MODE,
    AD9361PHY2R2T_MODE,
    IDELAY_TAP_BITS as RX_IDELAY_TAP_BITS,
    phy_layout,
)
from soc.cores.ad9361.prbs import AD9361PRBSWordChecker, AD9361PRBSWordGenerator
from soc.cores.ad9361.spi import AD9361SPIMaster

# The AD9361 SPI port accepts up to 50 MHz. The system clock over 8 keeps
# margin on the board traces without slowing the no-OS init noticeably.
SPI_CLK_DIVIDER = 8
SPI_DATA_WIDTH = 24

SAMPLE_BITS = 12

TX_SOURCE_STREAM = 0
TX_SOURCE_PRBS = 1
TX_SOURCE_PATTERN = 2


def _phy_clock_domain(phy):
    """Return the PHY's ``rfic`` ClockDomain so the SoC registers that object.

    The PHY drives the domain's clock by name (``ClockSignal("rfic")`` from a
    BUFG) but hands its reset synchronizer an unregistered
    ``ClockDomain("rfic")`` instance. Registering that same instance makes the
    name resolve to it, so the synchronized reset really reaches the domain.
    """
    from migen.genlib.resetsync import AsyncResetSynchronizer
    for special in phy._fragment.specials:
        if isinstance(special, AsyncResetSynchronizer) and special.cd.name == "rfic":
            return special.cd
    raise RuntimeError("AD9361PHY no longer resets the rfic domain the way this glue expects")


class RxSnapshot(LiteXModule):
    """Records `depth` consecutive rfic cycles of the PHY's receive side.

    Each 64-bit word holds the rising and falling nibbles the assembler
    consumes, the aligned RX_FRAME, the beat strobe and the assembled beat, so
    a host can decode the interface's bit and edge mapping against a known
    test pattern without any other instrumentation.
    """
    def __init__(self, phy, depth=256):
        self._ctrl = CSRStorage(fields=[
            CSRField("arm", size=1, offset=0, pulse=True, description="Start one capture."),
        ])
        self._status = CSRStatus(fields=[
            CSRField("done", size=1, offset=0, description="The capture holds `depth` words."),
        ])
        self._index = CSRStorage(log2_int(depth), description="Word to read back.")
        self._word_lo = CSRStatus(32, description="Bits 31..0 of the selected word.")
        self._word_hi = CSRStatus(32, description="Bits 63..32 of the selected word.")

        # # #

        mem = Memory(64, depth)
        wr = mem.get_port(write_capable=True, clock_domain="rfic")
        rd = mem.get_port(clock_domain="sys")
        self.specials += mem, wr, rd

        arm = PulseSynchronizer("sys", "rfic")
        self.submodules += arm
        self.comb += arm.i.eq(self._ctrl.fields.arm)

        count = Signal(max=depth)
        capturing = Signal()
        done = Signal()
        word = Cat(phy.rx_dbg_half_i, phy.rx_dbg_half_q, phy.rx_dbg_frame, phy.rx_dbg_valid,
                   phy.source.ia, phy.source.qa, phy.source.ib, phy.source.qb)
        self.sync.rfic += [
            wr.we.eq(0),
            If(arm.o,
                count.eq(0),
                capturing.eq(1),
                done.eq(0),
            ).Elif(capturing,
                wr.adr.eq(count),
                wr.dat_w.eq(word),
                wr.we.eq(1),
                If(count == depth - 1,
                    capturing.eq(0),
                    done.eq(1),
                ).Else(
                    count.eq(count + 1),
                ),
            ),
        ]
        self.specials += MultiReg(done, self._status.fields.done)
        self.comb += [
            rd.adr.eq(self._index.storage),
            self._word_lo.status.eq(rd.dat_r[:32]),
            self._word_hi.status.eq(rd.dat_r[32:]),
        ]


class AD9361Glue(LiteXModule):
    def __init__(self, rfic_pads, spi_pads, rx_iddr=False):
        self.sink = stream.Endpoint(phy_layout())

        self._config = CSRStorage(fields=[
            CSRField("rst_n",  size=1, offset=0, description="AD9361 RESETB (active low)."),
            CSRField("enable", size=1, offset=1, description="AD9361 ENABLE pin."),
            CSRField("txnrx",  size=1, offset=4, description="AD9361 TXNRX pin."),
            CSRField("en_agc", size=1, offset=5, description="AD9361 EN_AGC pin."),
        ], description="AD9361 control pins.")
        self._ctrl = CSRStorage(4, description="AD9361 CTRL_IN[3:0].")
        self._stat = CSRStatus(8, description="AD9361 CTRL_OUT[7:0].")

        self._test = CSRStorage(fields=[
            CSRField("tx_source", size=2, offset=0, values=[
                ("``0``", "Stream samples from ``sink``."),
                ("``1``", "PRBS (for the internal PHY loopback check)."),
                ("``2``", "Constant pattern from ``tx_pattern``."),
            ], description="Selects what the TX PHY transmits."),
        ])
        self._tx_pattern = CSRStorage(fields=[
            CSRField("i", size=SAMPLE_BITS, offset=0,  description="I value sent in both slots."),
            CSRField("q", size=SAMPLE_BITS, offset=16, description="Q value sent in both slots."),
        ], description="Sample sent when ``tx_source`` selects the pattern.")
        self._prbs_rx = CSRStatus(fields=[
            CSRField("synced", size=1, offset=0, description="RX PRBS checker is locked."),
        ])
        self._rx_idelay = CSRStorage(fields=[
            CSRField("tap", size=RX_IDELAY_TAP_BITS, offset=0,
                description="IDELAYE2 tap (0-31) for every RX data/frame line; loaded on write when `load` is set."),
            CSRField("load", size=1, offset=RX_IDELAY_TAP_BITS,
                description="Write 1 with `tap` to pulse LD and load the new tap into every RX IDELAYE2."),
        ], description="RX LVDS input-delay tap. Sweep it to center the DATA_CLK sampling eye.")
        # Raw RX pin activity at the IBUFDS outputs. Kept fieldless: a CSRStatus
        # with fields drives `.status` from the (undriven) field signals and
        # overwrites a direct `.status` assignment, so the whole word must be one
        # unfielded status. Bit layout: [15:0] RX_FRAME rising-edge count,
        # [21:16] per-RX_D-line "ever read low", [27:22] per-line "ever read high",
        # [28] RX_FRAME ever low, [29] RX_FRAME ever high. A toggling line sets both
        # its seen0 and seen1 bit.
        self._rx_activity = CSRStatus(30, description="Raw RX pin activity at the IBUFDS outputs (see glue for bit layout).")
        # Fieldless for the same reason as _rx_activity. Bit layout: [15:0] count
        # of emitted 'a' samples that were non-zero, [31:16] the same for 'b'.
        self._cap_activity = CSRStatus(32, description="Non-zero sample counts per capture slot: [15:0] slot a, [31:16] slot b.")
        self._rfic_cycles = CSRStatus(32, description="Free-running DATA_CLK cycle counter; read twice against wall time to measure the interface clock.")
        self._tx_underrun = CSRStatus(32, description="PHY words sent as zeros because no stream sample was ready. Counts continuously while no stream runs.")

        # # #

        self.spi = AD9361SPIMaster(spi_pads, data_width=SPI_DATA_WIDTH, clk_divider=SPI_CLK_DIVIDER)
        self.phy = phy = AD9361PHY(rfic_pads, rx_iddr=rx_iddr)
        self.cd_rfic = _phy_clock_domain(phy)
        # The PHY's mode CSR resets to 2R2T. Start in 1R1T and let the daemon
        # select the mode at init. The PRBS generator and checker follow it.
        phy.control.storage.reset = Constant(AD9361PHY1R1T_MODE, len(phy.control.storage))

        # RX input-delay tap. The CSR is sys-domain and so is the PHY's IDELAYE2
        # load clock, so the tap and load strobe reach the primitives with no
        # crossing. LD pulses for the one cycle the write lands with load set.
        self.comb += [
            phy.rx_idelay_value.eq(self._rx_idelay.fields.tap),
            phy.rx_idelay_load.eq(self._rx_idelay.re & self._rx_idelay.fields.load),
        ]

        # Control pins.
        self.comb += [
            rfic_pads.rst_n.eq(self._config.fields.rst_n),
            rfic_pads.enable.eq(self._config.fields.enable),
            rfic_pads.txnrx.eq(self._config.fields.txnrx),
            rfic_pads.en_agc.eq(self._config.fields.en_agc),
            rfic_pads.ctrl.eq(self._ctrl.storage),
        ]
        self.specials += MultiReg(rfic_pads.stat, self._stat.status)

        # TX source mux (rfic domain).
        tx_source = Signal(2)
        pattern_i = Signal(SAMPLE_BITS)
        pattern_q = Signal(SAMPLE_BITS)
        self.specials += [
            MultiReg(self._test.fields.tx_source, tx_source, odomain="rfic"),
            MultiReg(self._tx_pattern.fields.i, pattern_i, odomain="rfic"),
            MultiReg(self._tx_pattern.fields.q, pattern_q, odomain="rfic"),
        ]

        self.prbs_tx = prbs_tx = ClockDomainsRenamer("rfic")(AD9361PRBSWordGenerator())
        self.comb += [
            prbs_tx.ce.eq(phy.sink.valid & phy.sink.ready),
            prbs_tx.two_channels.eq(phy.mode_rfic == AD9361PHY2R2T_MODE),
        ]

        self.comb += Case(tx_source, {
            TX_SOURCE_PRBS: [
                phy.sink.valid.eq(1),
                phy.sink.ia.eq(prbs_tx.ia),
                phy.sink.qa.eq(prbs_tx.qa),
                phy.sink.ib.eq(prbs_tx.ib),
                phy.sink.qb.eq(prbs_tx.qb),
            ],
            TX_SOURCE_PATTERN: [
                phy.sink.valid.eq(1),
                phy.sink.ia.eq(pattern_i),
                phy.sink.qa.eq(pattern_q),
                phy.sink.ib.eq(pattern_i),
                phy.sink.qb.eq(pattern_q),
            ],
            "default": [
                self.sink.connect(phy.sink),
            ],
        })

        # RX taps (rfic domain). The source is free-running and lossy: valid
        # pulses one cycle per sample set and phy.source.ready stays high, so the
        # samples are a broadcast both consumers observe without a handshake. The
        # PRBS checker keeps running for interface checks, and the same fields are
        # exposed for a capture engine. In 1R1T a valid pulse has two
        # consecutive samples: 'a' first, then 'b'.
        self.rx_valid = Signal()
        self.rx_i = Signal(SAMPLE_BITS)
        self.rx_q = Signal(SAMPLE_BITS)
        self.rx_ib = Signal(SAMPLE_BITS)
        self.rx_qb = Signal(SAMPLE_BITS)
        self.comb += [
            self.rx_valid.eq(phy.source.valid),
            self.rx_i.eq(phy.source.ia),
            self.rx_q.eq(phy.source.qa),
            self.rx_ib.eq(phy.source.ib),
            self.rx_qb.eq(phy.source.qb),
        ]

        # Full-rate capture stream. Each rx_valid has two consecutive 1R1T
        # samples ('a' then 'b'). A consumer that takes only 'a' records half the
        # sample rate. valid pulses once every four rfic cycles, so 'b' is emitted
        # on the idle cycle right after 'a', giving one sample per pulse at the
        # full 2x-DATA_CLK/... reverse rate.
        # The stream is fully registered: 'a' is emitted the cycle after valid
        # and 'b' the cycle after that, so no combinational mux sits between the
        # PHY word and the capture FIFO's write strobe. cap_a_nonzero and
        # cap_b_nonzero count emitted samples that were non-zero, per slot, as a
        # read-only diagnostic of which slot has data.
        self.cap_valid = Signal()
        self.cap_i = Signal(SAMPLE_BITS)
        self.cap_q = Signal(SAMPLE_BITS)
        self.cap_a_nonzero = Signal(16)
        self.cap_b_nonzero = Signal(16)
        cap_pend_b = Signal()
        cap_b_i = Signal(SAMPLE_BITS)
        cap_b_q = Signal(SAMPLE_BITS)
        self.sync.rfic += [
            self.cap_valid.eq(0),
            If(phy.source.valid,
                self.cap_valid.eq(1),
                self.cap_i.eq(phy.source.ia),
                self.cap_q.eq(phy.source.qa),
                cap_pend_b.eq(1),
                cap_b_i.eq(phy.source.ib),
                cap_b_q.eq(phy.source.qb),
                If((phy.source.ia != 0) | (phy.source.qa != 0),
                    self.cap_a_nonzero.eq(self.cap_a_nonzero + 1),
                ),
            ).Elif(cap_pend_b,
                self.cap_valid.eq(1),
                self.cap_i.eq(cap_b_i),
                self.cap_q.eq(cap_b_q),
                cap_pend_b.eq(0),
                If((cap_b_i != 0) | (cap_b_q != 0),
                    self.cap_b_nonzero.eq(self.cap_b_nonzero + 1),
                ),
            ),
        ]

        self.prbs_rx = prbs_rx = ClockDomainsRenamer("rfic")(AD9361PRBSWordChecker())
        self.comb += [
            prbs_rx.ce.eq(phy.source.valid),
            prbs_rx.two_channels.eq(phy.mode_rfic == AD9361PHY2R2T_MODE),
            prbs_rx.ia.eq(phy.source.ia),
            prbs_rx.qa.eq(phy.source.qa),
            prbs_rx.ib.eq(phy.source.ib),
            prbs_rx.qb.eq(phy.source.qb),
            phy.source.ready.eq(1),
        ]
        self.specials += MultiReg(prbs_rx.synced, self._prbs_rx.fields.synced)
        self.rx_snapshot = RxSnapshot(phy)

        # Interface clock and underrun accounting, crossed as whole words.
        rfic_cycles = Signal(32)
        tx_underrun = Signal(32)
        self.sync.rfic += [
            rfic_cycles.eq(rfic_cycles + 1),
            If((tx_source == TX_SOURCE_STREAM) & phy.sink.ready & ~phy.sink.valid,
                tx_underrun.eq(tx_underrun + 1)
            ),
        ]
        self.cycles_sync = BusSynchronizer(32, "rfic", "sys")
        self.underrun_sync = BusSynchronizer(32, "rfic", "sys")
        self.activity_sync = BusSynchronizer(30, "rfic", "sys")
        self.cap_activity_sync = BusSynchronizer(32, "rfic", "sys")
        self.comb += [
            self.cap_activity_sync.i.eq(Cat(self.cap_a_nonzero, self.cap_b_nonzero)),
            self._cap_activity.status.eq(self.cap_activity_sync.o),
        ]
        self.comb += [
            self.cycles_sync.i.eq(rfic_cycles),
            self._rfic_cycles.status.eq(self.cycles_sync.o),
            self.underrun_sync.i.eq(tx_underrun),
            self._tx_underrun.status.eq(self.underrun_sync.o),
            self.activity_sync.i.eq(Cat(
                self.phy.rx_frame_edges,
                self.phy.rx_data_seen0,
                self.phy.rx_data_seen1,
                self.phy.rx_frame_seen0,
                self.phy.rx_frame_seen1,
            )),
            self._rx_activity.status.eq(self.activity_sync.o),
        ]
