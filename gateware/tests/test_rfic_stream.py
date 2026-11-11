"""The PHY word boundary: one tick per sample period, in both modes."""

from migen import ClockDomain, Module, Signal, passive, run_simulation
from litex.soc.interconnect import stream

from cores.rfic_stream import MODE_1R1T, MODE_2R2T, RficRxFrontend, RficTxBackend
from soc.cores.ad9361.phy import phy_layout

CLOCKS = {"sys": 10, "rfic": 10}
# The PHY hands over a word every four DATA_CLK cycles in both modes.
WORD_PERIOD = 4


class RxHarness(Module):
    def __init__(self):
        self.clock_domains.cd_rfic = ClockDomain()
        self.source = stream.Endpoint(phy_layout())
        self.mode = Signal()
        self.submodules.dut = RficRxFrontend(self.source, self.mode)


class TxHarness(Module):
    def __init__(self):
        self.clock_domains.cd_rfic = ClockDomain()
        self.sink = stream.Endpoint(phy_layout())
        self.mode = Signal()
        self.tick = Signal()
        self.ch0_i = Signal(12)
        self.ch0_q = Signal(12)
        self.ch1_i = Signal(12)
        self.ch1_q = Signal(12)
        self.submodules.dut = RficTxBackend(
            self.sink, self.mode, self.tick,
            self.ch0_i, self.ch0_q, self.ch1_i, self.ch1_q)


def phy_words(count, base=0x100):
    return [(base + 4 * n, base + 4 * n + 1, base + 4 * n + 2, base + 4 * n + 3)
            for n in range(count)]


def run_rx(mode, words):
    dut = RxHarness()
    seen = []

    def stim():
        yield dut.mode.eq(mode)
        for _ in range(4):
            yield

    @passive
    def driver():
        for ia, qa, ib, qb in words:
            yield dut.source.ia.eq(ia)
            yield dut.source.qa.eq(qa)
            yield dut.source.ib.eq(ib)
            yield dut.source.qb.eq(qb)
            yield dut.source.valid.eq(1)
            yield
            yield dut.source.valid.eq(0)
            for _ in range(WORD_PERIOD - 1):
                yield
        while True:
            yield

    @passive
    def recorder():
        while True:
            if (yield dut.dut.valid):
                seen.append((
                    (yield dut.dut.ch0_i), (yield dut.dut.ch0_q),
                    (yield dut.dut.ch1_i), (yield dut.dut.ch1_q)))
            yield

    def runner():
        yield from stim()
        for _ in range(len(words) * WORD_PERIOD + 20):
            yield

    run_simulation(dut, {"sys": [], "rfic": [runner(), driver(), recorder()]},
                   clocks=CLOCKS)
    return seen


def test_2r2t_gives_one_tick_and_both_channels_per_word():
    words = phy_words(6)
    seen = run_rx(MODE_2R2T, words)
    assert seen == words, f"2R2T frontend\n got {seen}\nwant {words}"


def test_1r1t_gives_two_ticks_per_word_on_one_channel():
    words = phy_words(6)
    seen = run_rx(MODE_1R1T, words)
    # A 1R1T word is two consecutive samples of channel 0, and channel 1
    # mirrors it so a two-channel stream still has a defined second lane.
    expected = []
    for ia, qa, ib, qb in words:
        expected.append((ia, qa, ia, qa))
        expected.append((ib, qb, ib, qb))
    assert seen == expected, f"1R1T frontend\n got {seen}\nwant {expected}"


def run_tx(mode, samples):
    """Feed one sample per tick and collect the PHY words that come out."""
    dut = TxHarness()
    seen = []

    @passive
    def driver():
        index = 0
        while True:
            if index < len(samples):
                (i0, q0), (i1, q1) = samples[index]
                yield dut.ch0_i.eq(i0)
                yield dut.ch0_q.eq(q0)
                yield dut.ch1_i.eq(i1)
                yield dut.ch1_q.eq(q1)
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            # 1R1T has two sample periods per PHY word, 2R2T has one.
            period = WORD_PERIOD // 2 if mode == MODE_1R1T else WORD_PERIOD
            for _ in range(period - 1):
                yield
            index += 1

    @passive
    def phy():
        # The PHY takes a word every fourth cycle, as its serializer wraps.
        cycle = 0
        while True:
            take = cycle % WORD_PERIOD == WORD_PERIOD - 1
            yield dut.sink.ready.eq(1 if take else 0)
            yield
            if take and (yield dut.sink.valid):
                seen.append(((yield dut.sink.ia), (yield dut.sink.qa),
                             (yield dut.sink.ib), (yield dut.sink.qb)))
            cycle += 1

    def runner():
        yield dut.mode.eq(mode)
        for _ in range(len(samples) * WORD_PERIOD + 40):
            yield

    run_simulation(dut, {"sys": [], "rfic": [runner(), driver(), phy()]},
                   clocks=CLOCKS)
    return seen


def test_2r2t_puts_one_sample_of_each_channel_in_a_word():
    samples = [((0x101 + n, 0x202 + n), (0x303 + n, 0x404 + n)) for n in range(8)]
    seen = run_tx(MODE_2R2T, samples)
    wanted = [(i0, q0, i1, q1) for (i0, q0), (i1, q1) in samples]
    assert wanted[0] in seen, f"2R2T backend\n got {seen}\nwant {wanted}"
    first = seen.index(wanted[0])
    assert seen[first:first + len(wanted)] == wanted, (
        f"2R2T backend\n got {seen[first:]}\nwant {wanted}")


def test_1r1t_puts_two_consecutive_samples_in_a_word():
    samples = [((0x101 + n, 0x202 + n), (0, 0)) for n in range(8)]
    seen = run_tx(MODE_1R1T, samples)
    wanted = [(samples[2 * k][0][0], samples[2 * k][0][1],
               samples[2 * k + 1][0][0], samples[2 * k + 1][0][1])
              for k in range(len(samples) // 2)]
    assert wanted[0] in seen, f"1R1T backend\n got {seen}\nwant {wanted}"
    first = seen.index(wanted[0])
    assert seen[first:first + len(wanted)] == wanted, (
        f"1R1T backend\n got {seen[first:]}\nwant {wanted}")
