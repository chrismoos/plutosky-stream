"""The rate filters against an integer model of the same polyphase FIRs."""

import numpy as np
from migen import ClockDomain, Module, Signal, passive, run_simulation

from cores.rate_filter import (BANK_BITS, COEF_BITS, LANES, PHASE_BITS, PHASE_TAPS, RATIO_MAX,
                               RX_FRAC_BITS, SAMPLE_MAX, SAMPLE_MIN, TX_FRAC_BITS,
                               RateFilter)
from cores.rfic_stream import MODE_1R1T, MODE_2R2T

CLOCKS = {"sys": 10, "rfic": 10}
# 2R2T hands over one sample every four radio clock cycles.
TICK_PERIOD = 4
KAISER_BETA = 6.0
SAMPLE_BITS = 12


def prototype(ratio):
    """The windowed-sinc low-pass the daemon loads, cut off at half the stream rate."""
    taps = PHASE_TAPS * ratio
    n = np.arange(taps) - (taps - 1) / 2
    h = np.sinc(n / ratio) * np.kaiser(taps, KAISER_BETA)
    return h / h.sum()


def quantize(h, frac_bits):
    q = np.round(h * (1 << frac_bits)).astype(np.int64)
    limit = 1 << (COEF_BITS - 1)
    assert q.max() < limit and q.min() >= -limit
    return q


def coefficients(ratio):
    h = prototype(ratio)
    return quantize(h, RX_FRAC_BITS), quantize(h * ratio, TX_FRAC_BITS)


def round_saturate(acc, frac_bits):
    value = (int(acc) + (1 << (frac_bits - 1))) >> frac_bits
    return max(SAMPLE_MIN, min(SAMPLE_MAX, value))


def decimate(x, coefs, ratio):
    """y[j] = sum h[n] x[j*ratio + ratio - 1 - n], with x zero before the start."""
    out = []
    for j in range(len(x) // ratio):
        end = j * ratio + ratio - 1
        acc = sum(int(coefs[n]) * int(x[end - n]) for n in range(len(coefs)) if end - n >= 0)
        out.append(round_saturate(acc, RX_FRAC_BITS))
    return out


def interpolate(s, coefs, ratio):
    """y[j*ratio + p] = sum_k h[k*ratio + p] s[j - k], with s zero before the start."""
    out = []
    for j in range(len(s)):
        for p in range(ratio):
            acc = sum(int(coefs[k * ratio + p]) * int(s[j - k])
                      for k in range(PHASE_TAPS) if j - k >= 0)
            out.append(round_saturate(acc, TX_FRAC_BITS))
    return out


def signed(value):
    return value - (1 << SAMPLE_BITS) if value & (1 << (SAMPLE_BITS - 1)) else value


def read_lanes(signals):
    values = []
    for signal in signals:
        values.append(signed((yield signal)))
    return tuple(values)


class Harness(Module):
    def __init__(self):
        self.clock_domains.cd_rfic = ClockDomain()
        self.mode = Signal()
        self.tick = Signal()
        self.rx = [Signal(SAMPLE_BITS) for _ in range(LANES)]
        self.tx = [Signal(SAMPLE_BITS) for _ in range(LANES)]
        self.submodules.dut = RateFilter(self.mode, self.tick, self.rx, self.tx)


def load(dut, rx_coefs, tx_coefs, rx_ratio, tx_ratio):
    """Writes both coefficient sets in bank order and then the ratios, as the daemon does."""
    for direction, coefs, ratio in ((0, rx_coefs, rx_ratio), (1, tx_coefs, tx_ratio)):
        if ratio == 1:
            continue
        for n, value in enumerate(coefs):
            bank, phase = divmod(n, ratio)
            addr = phase | (bank << PHASE_BITS) | (direction << (PHASE_BITS + BANK_BITS))
            yield dut.dut.coef_addr.storage.eq(addr)
            yield dut.dut.coef_data.storage.eq(int(value) & ((1 << COEF_BITS) - 1))
            yield dut.dut.coef_data.re.eq(1)
            yield
            yield dut.dut.coef_data.re.eq(0)
    yield dut.dut.rx_ratio.storage.eq(rx_ratio)
    yield dut.dut.tx_ratio.storage.eq(tx_ratio)
    for _ in range(20):
        yield


def run(ratio, rx_inputs, tx_samples, mode=MODE_2R2T, extra_ticks=8, tx_ratio=None,
        start_at=(), ticks=None):
    """Drives one radio sample per tick and returns what downstream would see.

    Returns the stream-rate receive samples taken on each receive tick, the
    receive tick period in radio ticks, and the transmit samples the backend
    would take on each radio tick. ``tx_ratio`` is ``ratio`` when omitted.
    ``start_at`` are radio ticks on which a release engine asks to start, and
    ``ticks`` receives the radio ticks the transmit tick fired on.
    """
    dut = Harness()
    if tx_ratio is None:
        tx_ratio = ratio
    rx_coefs, _ = coefficients(max(ratio, 2))
    _, tx_coefs = coefficients(max(tx_ratio, 2))
    loaded = [False]
    rx_seen = []
    tx_seen = []
    out_ticks = []
    radio_ticks = [0]

    def sys_side():
        yield from load(dut, rx_coefs, tx_coefs, ratio, tx_ratio)
        loaded[0] = True

    def rfic_side():
        yield dut.mode.eq(mode)
        while not loaded[0]:
            yield
        for _ in range(20):
            yield
        total = len(rx_inputs) + extra_ticks * max(ratio, tx_ratio)
        for index in range(total):
            word = rx_inputs[index] if index < len(rx_inputs) else (0,) * LANES
            for lane in range(LANES):
                yield dut.rx[lane].eq(word[lane] & 0xFFF)
            yield dut.dut.tx_start.eq(index in start_at)
            yield dut.tick.eq(1)
            yield
            yield dut.tick.eq(0)
            radio_ticks[0] += 1
            for _ in range(TICK_PERIOD - 1):
                yield

    @passive
    def watcher():
        # Stands in for the packetizer, the release engine and the backend:
        # samples go out on the output tick, and the backend takes a word on
        # every radio tick.
        index = 0
        staged = None
        radio_index = 0
        while True:
            if staged is not None:
                for lane in range(LANES):
                    yield dut.tx[lane].eq(staged[lane] & 0xFFF)
                staged = None
            if (yield dut.dut.rx_tick):
                out_ticks.append(radio_ticks[0])
                rx_seen.append((yield from read_lanes(dut.dut.rx_out)))
            if (yield dut.dut.tx_tick):
                if ticks is not None:
                    ticks.append(radio_index)
                staged = tx_samples[index] if index < len(tx_samples) else (0,) * LANES
                index += 1
            if (yield dut.tick):
                tx_seen.append((yield from read_lanes(dut.dut.tx_out)))
                radio_index += 1
            yield

    run_simulation(dut, {"sys": [sys_side()], "rfic": [rfic_side(), watcher()]},
                   clocks=CLOCKS)
    periods = set(np.diff(out_ticks)) if len(out_ticks) > 1 else set()
    return rx_seen, periods, tx_seen, rx_coefs, tx_coefs


def random_words(count, seed, amplitude=SAMPLE_MAX):
    rng = np.random.default_rng(seed)
    return [tuple(int(v) for v in rng.integers(-amplitude, amplitude + 1, LANES))
            for _ in range(count)]


def check_decimation(ratio, outputs=PHASE_TAPS + 6, seed=1):
    inputs = random_words(outputs * ratio, seed)
    rx_seen, periods, _, rx_coefs, _ = run(ratio, inputs, [])
    assert periods == {ratio}, f"output tick every {periods} radio ticks, want {ratio}"
    for lane in range(LANES):
        want = decimate([w[lane] for w in inputs], rx_coefs, ratio)
        got = [s[lane] for s in rx_seen][:len(want)]
        assert got == want, f"ratio {ratio} lane {lane}\n got {got}\nwant {want}"


def check_interpolation(ratio, samples=PHASE_TAPS + 4, seed=2):
    stream = random_words(samples, seed, amplitude=SAMPLE_MAX // 2)
    _, _, tx_seen, _, tx_coefs = run(ratio, [], stream, extra_ticks=samples + 4)
    for lane in range(LANES):
        want = interpolate([s[lane] for s in stream], tx_coefs, ratio)
        got = [s[lane] for s in tx_seen]
        first = next((n for n in range(len(got)) if got[n:n + len(want) - ratio] ==
                      want[:len(want) - ratio]), None)
        assert first is not None, f"ratio {ratio} lane {lane}\n got {got}\nwant {want}"


def test_ratio_one_passes_samples_and_ticks_through():
    inputs = random_words(16, 3)
    rx_seen, periods, _, _, _ = run(1, inputs, [], extra_ticks=0)
    assert periods == {1}
    assert rx_seen == inputs


def test_1r1t_ignores_the_ratio():
    inputs = random_words(16, 4)
    rx_seen, periods, _, _, _ = run(4, inputs, [], mode=MODE_1R1T, extra_ticks=0)
    assert periods == {1}
    assert rx_seen == inputs


def test_decimates_by_two():
    check_decimation(2)


def test_decimates_by_an_odd_ratio():
    check_decimation(5)


def test_decimates_by_the_largest_ratio():
    check_decimation(RATIO_MAX, outputs=PHASE_TAPS + 2)


def test_full_scale_input_saturates_instead_of_wrapping():
    inputs = [(SAMPLE_MAX, SAMPLE_MIN, SAMPLE_MAX, SAMPLE_MIN)] * (PHASE_TAPS * 3 * 4)
    rx_seen, _, _, rx_coefs, _ = run(3, inputs, [])
    steady = rx_seen[PHASE_TAPS + 2:PHASE_TAPS * 3]
    for lane, level in enumerate((SAMPLE_MAX, SAMPLE_MIN, SAMPLE_MAX, SAMPLE_MIN)):
        for sample in steady:
            assert abs(sample[lane] - level) <= 2, f"lane {lane} {sample}"


def test_interpolates_by_two():
    check_interpolation(2)


def test_interpolates_by_an_odd_ratio():
    check_interpolation(3)


def test_interpolates_by_the_largest_ratio():
    check_interpolation(RATIO_MAX, samples=PHASE_TAPS + 1)


def test_each_direction_has_its_own_ratio():
    rx_ratio, tx_ratio = 3, 2
    inputs = random_words((PHASE_TAPS + 4) * rx_ratio, 5)
    stream = random_words(PHASE_TAPS + 4, 6, amplitude=SAMPLE_MAX // 2)
    ticks = []
    rx_seen, periods, tx_seen, rx_coefs, tx_coefs = run(
        rx_ratio, inputs, stream, tx_ratio=tx_ratio, extra_ticks=PHASE_TAPS + 8, ticks=ticks)
    assert periods == {rx_ratio}
    assert set(np.diff(ticks)) == {tx_ratio}
    for lane in range(LANES):
        want = decimate([w[lane] for w in inputs], rx_coefs, rx_ratio)
        assert [s[lane] for s in rx_seen][:len(want)] == want
        want = interpolate([s[lane] for s in stream], tx_coefs, tx_ratio)
        got = [s[lane] for s in tx_seen]
        assert any(got[n:n + len(want) - tx_ratio] == want[:len(want) - tx_ratio]
                   for n in range(len(got)))


def test_receive_bypass_with_transmit_interpolation():
    inputs = random_words(40, 7)
    ticks = []
    rx_seen, periods, _, _, _ = run(1, inputs, [], tx_ratio=4, extra_ticks=0, ticks=ticks)
    assert periods == {1}
    assert rx_seen == inputs
    assert set(np.diff(ticks)) == {4}


def test_a_start_moves_the_transmit_tick_to_that_count():
    ticks = []
    run(1, random_words(60, 8), [], tx_ratio=5, extra_ticks=0, ticks=ticks, start_at=(22,))
    assert 22 in ticks
    after = [t for t in ticks if t >= 22]
    assert set(np.diff(after)) == {5}
    before = [t for t in ticks if t <= 22]
    assert set(np.diff(before[:-1])) == {5}


def test_the_prototype_meets_its_band_edges():
    for ratio in (2, 3, 8, RATIO_MAX):
        rx, tx = coefficients(ratio)
        for coefs, frac in ((rx, RX_FRAC_BITS), (tx / ratio, TX_FRAC_BITS)):
            points = 1 << 18
            response = np.abs(np.fft.rfft(coefs / (1 << frac), points))
            freq = np.arange(len(response)) / points * ratio
            db = 20 * np.log10(response + 1e-15)
            passband = db[freq <= 0.42]
            assert passband.max() - passband.min() < 0.1, ratio
            assert db[freq >= 0.58].max() < -55, ratio
