"""The memory-backed queues, for order, throughput and back-pressure.

A memory read costs a cycle, and the obvious way to hide that loses an entry
the first time the consumer stalls. These check both that nothing is lost and
that the queue still moves an entry a cycle when nothing stalls.
"""

import pytest
from migen import Module, passive, run_simulation

from cores.fifo import AsyncFifoBuffered, SyncFifo

LAYOUT = [("value", 16)]


def drive(depth, count, gap_in, gap_out):
    class Dut(Module):
        def __init__(self):
            self.submodules.fifo = SyncFifo(LAYOUT, depth)

    dut = Dut()
    sent = list(range(1, count + 1))
    got = []
    cycles = [0]

    def stim():
        index = 0
        for cycle in range(1, 40000):
            if len(got) == count:
                cycles[0] = cycle
                break
            feeding = index < count and (not gap_in or cycle % (gap_in + 1))
            yield dut.fifo.sink.value.eq(sent[index] if index < count else 0)
            yield dut.fifo.sink.valid.eq(1 if feeding else 0)
            yield dut.fifo.source.ready.eq(
                1 if not gap_out or cycle % (gap_out + 1) else 0)
            yield
            if feeding and (yield dut.fifo.sink.ready):
                index += 1
            if (yield dut.fifo.source.valid) and (yield dut.fifo.source.ready):
                got.append((yield dut.fifo.source.value))
        assert got == sent, f"got {got[:16]} of {len(got)}, want {sent[:16]}"

    run_simulation(dut, [stim()])
    return cycles[0]


@pytest.mark.parametrize("depth", [4, 8, 64])
@pytest.mark.parametrize("gap_in,gap_out", [(0, 0), (2, 0), (0, 3), (1, 2), (3, 1)])
def test_nothing_is_lost_or_reordered(depth, gap_in, gap_out):
    drive(depth, 100, gap_in, gap_out)


def test_it_moves_an_entry_a_cycle():
    """Two cycles an entry would not keep up with bfp8 at line rate."""
    count = 200
    cycles = drive(64, count, 0, 0)
    # A few cycles of fill and drain either side, then one entry a cycle.
    assert cycles < count + 12, f"{cycles} cycles for {count} entries"


@pytest.mark.parametrize("clocks", [{"write": 10, "read": 14},
                                    {"write": 14, "read": 10}])
def test_async_buffered_fifo_preserves_order_across_clock_domains(clocks):
    class Dut(Module):
        def __init__(self):
            self.submodules.fifo = AsyncFifoBuffered(16, 8)

    dut = Dut()
    sent = list(range(1, 101))
    got = []

    @passive
    def writer():
        index = 0
        while True:
            yield dut.fifo.din.eq(sent[index] if index < len(sent) else 0)
            yield dut.fifo.we.eq(index < len(sent))
            yield
            if index < len(sent) and (yield dut.fifo.writable):
                index += 1

    def reader():
        for cycle in range(2000):
            yield dut.fifo.re.eq(cycle % 4 != 0)
            yield
            if (yield dut.fifo.readable) and (yield dut.fifo.re):
                got.append((yield dut.fifo.dout))
                if len(got) == len(sent):
                    break
        assert got == sent

    run_simulation(dut, {"write": [writer()], "read": [reader()]},
                   clocks=clocks)
