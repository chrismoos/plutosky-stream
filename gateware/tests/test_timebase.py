"""Latch, load and armed load across the CSR boundary."""

from migen import ClockDomain, Module, Signal, passive, run_simulation

from cores.timebase import Timebase

CLOCKS = {"sys": 10, "rfic": 16}


class Harness(Module):
    def __init__(self):
        self.clock_domains.cd_rfic = ClockDomain()
        self.tick = Signal()
        self.submodules.dut = Timebase(self.tick)


@passive
def ticker(dut, period=4):
    while True:
        yield dut.tick.eq(1)
        yield
        yield dut.tick.eq(0)
        for _ in range(period - 1):
            yield


def settle(count=40):
    for _ in range(count):
        yield


def read_now(dut):
    yield dut.dut.ctrl.storage.eq(0b001)
    yield dut.dut.ctrl.re.eq(1)
    yield
    yield dut.dut.ctrl.re.eq(0)
    yield dut.dut.ctrl.storage.eq(0)
    yield from settle()
    hi = yield dut.dut.now_hi.status
    lo = yield dut.dut.now_lo.status
    return (hi << 32) | lo


def write_load(dut, value, armed_at=None):
    yield dut.dut.load_lo.storage.eq(value & 0xFFFF_FFFF)
    yield dut.dut.load_hi.storage.eq(value >> 32)
    if armed_at is not None:
        yield dut.dut.arm_lo.storage.eq(armed_at & 0xFFFF_FFFF)
        yield dut.dut.arm_hi.storage.eq(armed_at >> 32)
    yield from settle(10)


def test_the_counter_advances_once_per_tick():
    dut = Harness()
    seen = []

    def stim():
        yield from settle(20)
        first = yield from read_now(dut)
        yield from settle(80)
        second = yield from read_now(dut)
        seen.extend([first, second])

    run_simulation(dut, {"sys": [stim()], "rfic": [ticker(dut)]}, clocks=CLOCKS)
    # About 120 sys cycles pass between the two latches, which is 75 rfic cycles
    # at these periods and a tick every fourth one.
    assert 14 <= seen[1] - seen[0] <= 22


def test_load_sets_the_counter():
    dut = Harness()
    seen = []

    def stim():
        yield from settle(20)
        yield from write_load(dut, 0x0000_0002_1234_5678)
        yield dut.dut.ctrl.storage.eq(0b010)
        yield dut.dut.ctrl.re.eq(1)
        yield
        yield dut.dut.ctrl.re.eq(0)
        yield dut.dut.ctrl.storage.eq(0)
        yield from settle(20)
        seen.append((yield from read_now(dut)))

    run_simulation(dut, {"sys": [stim()], "rfic": [ticker(dut)]}, clocks=CLOCKS)
    assert 0x0000_0002_1234_5678 <= seen[0] <= 0x0000_0002_1234_5678 + 32


def test_a_latched_read_does_not_tear_across_a_carry():
    dut = Harness()
    seen = []

    def stim():
        yield from settle(20)
        yield from write_load(dut, 0xFFFF_FFFE)
        yield dut.dut.ctrl.storage.eq(0b010)
        yield dut.dut.ctrl.re.eq(1)
        yield
        yield dut.dut.ctrl.re.eq(0)
        yield dut.dut.ctrl.storage.eq(0)
        # A latch issued in the same breath as a load is a race across the
        # domain crossing, so let the load land first.
        yield from settle(20)
        for _ in range(12):
            seen.append((yield from read_now(dut)))

    run_simulation(dut, {"sys": [stim()], "rfic": [ticker(dut)]}, clocks=CLOCKS)
    # Every reading must be at or past the loaded value and never step
    # backwards, which a torn high and low word would show as a large jump.
    assert seen == sorted(seen)
    assert seen[0] >= 0xFFFF_FFFE
    assert seen[-1] > 0x1_0000_0000, "the counter must carry into the high word"


def test_armed_load_fires_when_the_counter_reaches_the_trigger():
    dut = Harness()
    seen = []

    def stim():
        yield from settle(20)
        yield from write_load(dut, 0x9_0000_0000, armed_at=100)
        yield dut.dut.ctrl.storage.eq(0b110)
        yield dut.dut.ctrl.re.eq(1)
        yield
        yield dut.dut.ctrl.re.eq(0)
        yield dut.dut.ctrl.storage.eq(0b100)
        before = yield from read_now(dut)
        yield from settle(600)
        seen.extend([before, (yield from read_now(dut))])

    run_simulation(dut, {"sys": [stim()], "rfic": [ticker(dut)]}, clocks=CLOCKS)
    assert seen[0] < 100, "the load must not fire before the trigger count"
    assert seen[1] >= 0x9_0000_0000, "the load must fire at the trigger count"


def test_an_armed_load_left_disarmed_never_fires():
    dut = Harness()
    seen = []

    def stim():
        yield from settle(20)
        yield from write_load(dut, 0x9_0000_0000, armed_at=100)
        yield from settle(600)
        seen.append((yield from read_now(dut)))

    run_simulation(dut, {"sys": [stim()], "rfic": [ticker(dut)]}, clocks=CLOCKS)
    assert seen[0] < 0x9_0000_0000
