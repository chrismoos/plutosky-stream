"""Integer sample-rate change between the RFIC and the stream cores.

The radio runs at one rate and each direction has its own whole ratio to it.
The decimator low-pass filters each receive lane and keeps one sample in
``rx_ratio``. The interpolator turns each transmit sample into ``tx_ratio``
filtered samples. ``rx_tick`` and ``tx_tick`` pulse once per stream sample of
their direction. The timebase counts the radio's own tick, so a receive sample
spans ``rx_ratio`` counts and a transmit sample ``tx_ratio``.

``tx_tick`` repeats every ``tx_ratio`` radio ticks. A burst may start on any
count: while no release engine is playing, ``tx_start`` fires the tick on the
current radio tick and the repeat continues from there.

Both filters are polyphase FIRs of ``PHASE_TAPS`` taps per phase, so a filter
is ``PHASE_TAPS * ratio`` taps long. Coefficient ``n = k * ratio + p`` is held
in bank ``k`` at address ``p``, so every bank's multiplier works in parallel.
The daemon loads both coefficient sets over CSR while no stream runs.

A ratio of 1 bypasses that direction's filter, and its tick, samples and
timing are exactly those of the radio.

The filters process the four lanes of a sample over the four radio clock
cycles 2R2T gives each sample. 1R1T delivers samples two cycles apart, so it
always runs at a ratio of 1.
"""

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRField, CSRStorage

from cores.rfic_stream import MODE_2R2T
from cores.sample_packer import SAMPLE_BITS

PHASE_TAPS = 24
RATIO_MAX = 32
COEF_BITS = 18
# Receive coefficients have a DC gain of one. Transmit coefficients have a DC
# gain of one per phase, so their largest tap reaches one and needs a bit more.
RX_FRAC_BITS = 17
TX_FRAC_BITS = 16
LANES = 4
RATIO_BITS = bits_for(RATIO_MAX)
PHASE_BITS = bits_for(RATIO_MAX - 1)
BANK_BITS = bits_for(PHASE_TAPS - 1)
PRODUCT_BITS = SAMPLE_BITS + COEF_BITS
# Wide enough for any coefficient set, not just a well-behaved one.
RX_ACC_BITS = PRODUCT_BITS + bits_for(PHASE_TAPS * RATIO_MAX)
TX_ACC_BITS = PRODUCT_BITS + bits_for(PHASE_TAPS)
SAMPLE_MIN = -(1 << (SAMPLE_BITS - 1))
SAMPLE_MAX = (1 << (SAMPLE_BITS - 1)) - 1


def _round_saturate(module, value, frac_bits):
    """A signed sample-width signal holding ``value / 2**frac_bits``, rounded
    half up and clamped to the sample range."""
    width = len(value) - frac_bits + 1
    biased = Signal((len(value) + 1, True))
    rounded = Signal((width, True))
    out = Signal((SAMPLE_BITS, True))
    # A slice, not a shift: Migen prints >> as a logical shift even on signed values.
    module.comb += [
        biased.eq(value + (1 << (frac_bits - 1))),
        rounded.eq(biased[frac_bits:]),
        If(rounded > SAMPLE_MAX,
            out.eq(SAMPLE_MAX),
        ).Elif(rounded < SAMPLE_MIN,
            out.eq(SAMPLE_MIN),
        ).Else(
            out.eq(rounded),
        ),
    ]
    return out


class _LaneSequencer(LiteXModule):
    """One lane per cycle for the four cycles after each ``tick``.

    ``valid`` is high for those cycles, with ``lane`` counting 0 to 3. A tick
    every four cycles keeps it busy without a gap.
    """

    def __init__(self, tick, restart, rfic_cd):
        self.valid = Signal()
        self.lane = Signal(2)

        # # #

        sync = getattr(self.sync, rfic_cd)
        sync += [
            If(restart,
                self.valid.eq(0),
            ).Elif(tick,
                self.valid.eq(1),
                self.lane.eq(0),
            ).Elif(self.valid,
                self.lane.eq(self.lane + 1),
                If(self.lane == LANES - 1, self.valid.eq(0)),
            ),
        ]


class RateDecimator(LiteXModule):
    """Receive lanes at the radio rate to lanes at the stream rate.

    ``banks`` are ``PHASE_TAPS`` asynchronous read ports of the receive
    coefficients. Each input sample is multiplied by one coefficient of every
    bank and added into that bank's accumulator, which holds a partial sum of
    an output ``k`` samples ahead. When a block of ``ratio`` inputs ends, bank
    0's sum is the next output and every accumulator moves down one bank.

    ``slow_tick`` and ``lanes`` change on the same edge, so a reader that
    samples the lanes on the tick or a few cycles after it sees one sample.
    """

    def __init__(self, tick, lanes_in, ratio, restart, banks, rfic_cd="rfic"):
        self.slow_tick = Signal()
        self.lanes = [Signal((SAMPLE_BITS, True)) for _ in range(LANES)]

        # # #

        sync = getattr(self.sync, rfic_cd)
        self.sequencer = seq = _LaneSequencer(tick, restart, rfic_cd)
        word = Array(Signal((SAMPLE_BITS, True)) for _ in range(LANES))

        # Phase counts down so the last input of a block is phase 0, the one
        # whose coefficients line up with the output it completes.
        phase = Signal(PHASE_BITS)
        op_phase = Signal(PHASE_BITS)
        sync += [
            If(restart,
                phase.eq(ratio - 1),
            ).Elif(tick,
                [word[n].eq(lanes_in[n]) for n in range(LANES)],
                op_phase.eq(phase),
                phase.eq(Mux(phase == 0, ratio - 1, phase - 1)),
            ),
        ]
        self.comb += [bank.adr.eq(op_phase) for bank in banks]

        s1_valid = Signal()
        s1_lane = Signal(2)
        s1_last = Signal()
        s1_x = Signal((SAMPLE_BITS, True))
        s1_coef = [Signal((COEF_BITS, True)) for _ in range(PHASE_TAPS)]
        sync += [
            s1_valid.eq(seq.valid & ~restart),
            s1_lane.eq(seq.lane),
            s1_last.eq(op_phase == 0),
            s1_x.eq(word[seq.lane]),
            [s1_coef[k].eq(banks[k].dat_r) for k in range(PHASE_TAPS)],
        ]

        s2_valid = Signal()
        s2_lane = Signal(2)
        s2_last = Signal()
        product = [Signal((PRODUCT_BITS, True)) for _ in range(PHASE_TAPS)]
        sync += [
            s2_valid.eq(s1_valid & ~restart),
            s2_lane.eq(s1_lane),
            s2_last.eq(s1_last),
            [product[k].eq(s1_x * s1_coef[k]) for k in range(PHASE_TAPS)],
        ]

        acc = [Array(Signal((RX_ACC_BITS, True)) for _ in range(LANES))
               for _ in range(PHASE_TAPS)]
        finished = Signal((RX_ACC_BITS, True))
        self.comb += finished.eq(acc[0][s2_lane] + product[0])
        sample = _round_saturate(self, finished, RX_FRAC_BITS)
        held = Array(Signal((SAMPLE_BITS, True)) for _ in range(LANES))

        shift = [acc[k][s2_lane].eq(acc[k + 1][s2_lane] + product[k + 1])
                 for k in range(PHASE_TAPS - 1)]
        shift.append(acc[PHASE_TAPS - 1][s2_lane].eq(0))
        sync += [
            self.slow_tick.eq(0),
            If(restart,
                [acc[k][n].eq(0) for k in range(PHASE_TAPS) for n in range(LANES)],
            ).Elif(s2_valid,
                If(s2_last,
                    shift,
                    held[s2_lane].eq(sample),
                    If(s2_lane == LANES - 1,
                        [self.lanes[n].eq(held[n]) for n in range(LANES - 1)],
                        self.lanes[LANES - 1].eq(sample),
                        self.slow_tick.eq(1),
                    ),
                ).Else(
                    [acc[k][s2_lane].eq(acc[k][s2_lane] + product[k])
                     for k in range(PHASE_TAPS)],
                ),
            ),
        ]


class RateInterpolator(LiteXModule):
    """Transmit lanes at the stream rate to lanes at the radio rate.

    ``slow_tick`` stages the stream sample in ``lanes_in``. The next radio tick
    moves it into a ``PHASE_TAPS`` deep history and starts phase 0. Each later
    radio tick computes the next phase, one lane per cycle, as the dot product
    of the history with bank ``k``'s coefficient for that phase. The four lanes
    of a radio sample land in ``lanes`` together, so a reader that samples them
    on a radio tick never sees two samples mixed.
    """

    def __init__(self, tick, slow_tick, lanes_in, ratio, restart, banks, rfic_cd="rfic"):
        self.lanes = [Signal((SAMPLE_BITS, True)) for _ in range(LANES)]

        # # #

        sync = getattr(self.sync, rfic_cd)
        staged = [Signal((SAMPLE_BITS, True)) for _ in range(LANES)]
        pending = Signal()
        history = [Array(Signal((SAMPLE_BITS, True)) for _ in range(LANES))
                   for _ in range(PHASE_TAPS)]
        phase = Signal(PHASE_BITS)
        sync += [
            If(restart,
                pending.eq(0),
                phase.eq(0),
                [history[k][n].eq(0) for k in range(PHASE_TAPS) for n in range(LANES)],
            ).Else(
                If(tick,
                    If(pending,
                        [history[0][n].eq(staged[n]) for n in range(LANES)],
                        [history[k][n].eq(history[k - 1][n])
                         for k in range(1, PHASE_TAPS) for n in range(LANES)],
                        phase.eq(0),
                        pending.eq(0),
                    ).Elif(phase != ratio - 1,
                        phase.eq(phase + 1),
                    ),
                ),
                # After the tick's block so a stage on the same cycle is kept.
                If(slow_tick,
                    [staged[n].eq(lanes_in[n]) for n in range(LANES)],
                    pending.eq(1),
                ),
            ),
        ]

        self.sequencer = seq = _LaneSequencer(tick, restart, rfic_cd)
        self.comb += [bank.adr.eq(phase) for bank in banks]

        s1_valid = Signal()
        s1_lane = Signal(2)
        s1_x = [Signal((SAMPLE_BITS, True)) for _ in range(PHASE_TAPS)]
        s1_coef = [Signal((COEF_BITS, True)) for _ in range(PHASE_TAPS)]
        sync += [
            s1_valid.eq(seq.valid & ~restart),
            s1_lane.eq(seq.lane),
            [s1_x[k].eq(history[k][seq.lane]) for k in range(PHASE_TAPS)],
            [s1_coef[k].eq(banks[k].dat_r) for k in range(PHASE_TAPS)],
        ]

        valid = Signal()
        lane = Signal(2)
        terms = [Signal((PRODUCT_BITS, True)) for _ in range(PHASE_TAPS)]
        sync += [
            valid.eq(s1_valid & ~restart),
            lane.eq(s1_lane),
            [terms[k].eq(s1_x[k] * s1_coef[k]) for k in range(PHASE_TAPS)],
        ]

        while len(terms) > 1:
            width = len(terms[0]) + 1
            summed = []
            for n in range(0, len(terms) - 1, 2):
                s = Signal((width, True))
                sync += s.eq(terms[n] + terms[n + 1])
                summed.append(s)
            if len(terms) % 2:
                s = Signal((width, True))
                sync += s.eq(terms[-1])
                summed.append(s)
            next_valid = Signal()
            next_lane = Signal(2)
            sync += [next_valid.eq(valid & ~restart), next_lane.eq(lane)]
            terms, valid, lane = summed, next_valid, next_lane

        sample = _round_saturate(self, terms[0], TX_FRAC_BITS)
        held = Array(Signal((SAMPLE_BITS, True)) for _ in range(LANES))
        sync += [
            If(restart,
                [self.lanes[n].eq(0) for n in range(LANES)],
            ).Elif(valid,
                held[lane].eq(sample),
                If(lane == LANES - 1,
                    [self.lanes[n].eq(held[n]) for n in range(LANES - 1)],
                    self.lanes[LANES - 1].eq(sample),
                ),
            ),
        ]


class RateFilter(LiteXModule):
    """The decimator and interpolator, their coefficients and the ratio.

    ``rx_lanes`` are the radio's receive samples and ``tx_lanes`` the release
    engines' transmit samples, both ordered channel 0 I, channel 0 Q, channel
    1 I, channel 1 Q. ``rx_out`` and ``tx_out`` replace the receive samples
    and the transmit samples downstream. The packetizers sample on ``rx_tick``
    and the release engines on ``tx_tick``. ``tx_start`` is high while a
    release engine's next packet is due on the current count, and ``tx_busy``
    while one is playing. The timebase and the transmit backend keep the radio
    tick.
    """

    def __init__(self, mode, tick, rx_lanes, tx_lanes, rfic_cd="rfic"):
        self.rx_ratio = CSRStorage(RATIO_BITS, reset=1,
            description=f"Radio samples per receive sample, 1 to {RATIO_MAX}. "
                        "1 bypasses the filter. Change it only while no stream runs.")
        self.tx_ratio = CSRStorage(RATIO_BITS, reset=1,
            description=f"Radio samples per transmit sample, 1 to {RATIO_MAX}. "
                        "1 bypasses the filter. Change it only while no stream runs.")
        self.coef_addr = CSRStorage(fields=[
            CSRField("phase", size=PHASE_BITS, offset=0,
                description="Coefficient index modulo the ratio."),
            CSRField("bank", size=BANK_BITS, offset=PHASE_BITS,
                description="Coefficient index divided by the ratio."),
            CSRField("tx", size=1, offset=PHASE_BITS + BANK_BITS,
                description="Set for the transmit filter, clear for receive."),
        ], description="Where the next coef_data write is stored.")
        self.coef_data = CSRStorage(COEF_BITS,
            description="Writing stores this signed coefficient at coef_addr.")

        self.rx_tick = Signal()
        self.tx_tick = Signal()
        self.tx_start = Signal()
        self.tx_busy = Signal()
        self.rx_out = [Signal(SAMPLE_BITS) for _ in range(LANES)]
        self.tx_out = [Signal(SAMPLE_BITS) for _ in range(LANES)]

        # # #

        sync = getattr(self.sync, rfic_cd)

        # Read straight out of the storage word, as the timebase does, so the
        # core simulates without a CSR bank.
        addr = self.coef_addr.storage
        addr_phase = addr[:PHASE_BITS]
        addr_bank = addr[PHASE_BITS:PHASE_BITS + BANK_BITS]
        addr_tx = addr[PHASE_BITS + BANK_BITS]
        write = self.coef_data.re

        banks = {}
        for direction in (0, 1):
            ports = []
            for k in range(PHASE_TAPS):
                mem = Memory(COEF_BITS, RATIO_MAX)
                wport = mem.get_port(write_capable=True, clock_domain="sys")
                rport = mem.get_port(async_read=True, clock_domain=rfic_cd)
                self.specials += mem, wport, rport
                self.comb += [
                    wport.adr.eq(addr_phase),
                    wport.dat_w.eq(self.coef_data.storage),
                    wport.we.eq(write & (addr_bank == k) & (addr_tx == direction)),
                ]
                ports.append(rport)
            banks[direction] = ports

        restart = Signal()
        ratios = []
        for storage in (self.rx_ratio.storage, self.tx_ratio.storage):
            requested = Signal(RATIO_BITS)
            self.specials += MultiReg(storage, requested, odomain=rfic_cd)
            ratio = Signal(RATIO_BITS, reset=1)
            last_ratio = Signal(RATIO_BITS, reset=1)
            changed = Signal()
            bypass = Signal(reset=1)
            sync += [
                If((mode == MODE_2R2T) & (requested >= 2) & (requested <= RATIO_MAX),
                    ratio.eq(requested),
                ).Else(
                    ratio.eq(1),
                ),
                last_ratio.eq(ratio),
                changed.eq(ratio != last_ratio),
                bypass.eq(ratio == 1),
            ]
            ratios.append((ratio, changed, bypass))
        (rx_ratio, rx_changed, rx_bypass), (tx_ratio, tx_changed, tx_bypass) = ratios
        self.comb += restart.eq(rx_changed | tx_changed)

        # Radio ticks left before the transmit tick repeats.
        tx_wait = Signal(PHASE_BITS)
        self.comb += self.tx_tick.eq(
            tick & ((tx_wait == 0) | (self.tx_start & ~self.tx_busy)))
        sync += [
            If(restart,
                tx_wait.eq(0),
            ).Elif(self.tx_tick,
                tx_wait.eq(tx_ratio - 1),
            ).Elif(tick,
                tx_wait.eq(tx_wait - 1),
            ),
        ]

        self.decimator = decimator = RateDecimator(
            tick, rx_lanes, rx_ratio, restart, banks[0], rfic_cd)
        self.interpolator = interpolator = RateInterpolator(
            tick, self.tx_tick, tx_lanes, tx_ratio, restart, banks[1], rfic_cd)

        self.comb += [
            self.rx_tick.eq(Mux(rx_bypass, tick, decimator.slow_tick)),
            [self.rx_out[n].eq(Mux(rx_bypass, rx_lanes[n], decimator.lanes[n]))
             for n in range(LANES)],
            [self.tx_out[n].eq(Mux(tx_bypass, tx_lanes[n], interpolator.lanes[n]))
             for n in range(LANES)],
        ]
