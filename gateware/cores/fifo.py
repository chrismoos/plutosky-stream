"""Queues built on memory, which simulate as well as they synthesize.

Migen ships both of these, but their block RAM write ports are created
write-only and the simulator's memory lowering dereferences every port's read
data, so a design holding one cannot be simulated. The whole receive and
transmit path is queues, so that would leave the cores that matter most
untestable. These are the same designs with a read-capable write port whose
read data is simply unused.

Both present their output through a register rather than a multiplexer over
storage. An array of registers indexed by a pointer is a multiplexer as wide as
the array, and whatever it feeds inherits that depth, so the queues that matter
are memories and their outputs are registered.
"""

from migen import *
from migen.genlib.cdc import GrayCounter, MultiReg

from litex.gen import LiteXModule
from litex.soc.interconnect import stream


class AsyncFifo(LiteXModule):
    """``depth`` must be a power of two, at least two."""

    def __init__(self, width, depth):
        self.width = width
        self.depth = depth
        self.din = Signal(width)
        self.we = Signal()
        self.writable = Signal()
        self.dout = Signal(width)
        self.re = Signal()
        self.readable = Signal()
        # Hold high in the write and read domains to empty the queue and align
        # both pointers at zero. Both are quasi-static: assert them together
        # for a stopped stream and release together at start, so the graycode
        # pointers reset cleanly rather than glitch across the crossing.
        self.flush_write = Signal()
        self.flush_read = Signal()

        # # #

        bits = log2_int(depth, True)

        self.produce = produce = ClockDomainsRenamer("write")(
            ResetInserter()(GrayCounter(bits + 1)))
        self.consume = consume = ClockDomainsRenamer("read")(
            ResetInserter()(GrayCounter(bits + 1)))
        self.comb += [
            produce.reset.eq(self.flush_write),
            consume.reset.eq(self.flush_read),
            produce.ce.eq(self.writable & self.we),
            consume.ce.eq(self.readable & self.re),
        ]

        produce_in_read = Signal(bits + 1)
        consume_in_write = Signal(bits + 1)
        produce.q.attr.add("no_retiming")
        consume.q.attr.add("no_retiming")
        self.specials += [
            MultiReg(produce.q, produce_in_read, "read"),
            MultiReg(consume.q, consume_in_write, "write"),
        ]

        # Full is the gray-code condition: the pointers differ only in their
        # top two bits, which is one lap apart.
        same_top = Signal()
        self.comb += [
            same_top.eq((produce.q[-1] == consume_in_write[-1])
                        | (produce.q[-2] == consume_in_write[-2])),
        ]
        if bits == 1:
            self.comb += self.writable.eq(same_top)
        else:
            self.comb += self.writable.eq(
                same_top | (produce.q[:-2] != consume_in_write[:-2]))
        # The readable flag is registered because it addresses the storage:
        # through the consume counter's enable it picks the word the read port
        # fetches, which would make the synchroniser the start of a path into a
        # block RAM address. Judging it on the pointer the counter is about to hold
        # keeps it exact, since the write side only ever moves further away.
        readable = Signal()
        self.comb += self.readable.eq(readable)
        self.sync.read += If(self.flush_read,
            readable.eq(0),
        ).Else(
            readable.eq(consume.q_next != produce_in_read),
        )

        storage = Memory(width, depth)
        self.specials += storage
        # read_capable on the write port only so the simulator's memory
        # lowering has a read-data signal to drive.
        write_port = storage.get_port(write_capable=True, clock_domain="write")
        read_port = storage.get_port(clock_domain="read")
        self.specials += write_port, read_port
        self.comb += [
            write_port.adr.eq(produce.q_binary[:-1]),
            write_port.dat_w.eq(self.din),
            write_port.we.eq(produce.ce),
            read_port.adr.eq(consume.q_next_binary[:-1]),
            self.dout.eq(read_port.dat_r),
        ]


class AsyncFifoBuffered(LiteXModule):
    """Cross-clock queue with a registered first-word output."""

    def __init__(self, width, depth):
        self.width = width
        self.depth = depth
        self.din = Signal(width)
        self.we = Signal()
        self.writable = Signal()
        self.dout = Signal(width)
        self.re = Signal()
        self.readable = Signal()
        self.flush_write = Signal()
        self.flush_read = Signal()

        self.fifo = fifo = AsyncFifo(width, depth)
        self.comb += [
            self.writable.eq(fifo.writable),
            fifo.din.eq(self.din),
            fifo.we.eq(self.we),
            fifo.flush_write.eq(self.flush_write),
            fifo.flush_read.eq(self.flush_read),
            fifo.re.eq(self.re | ~self.readable),
        ]
        self.sync.read += If(self.flush_read,
            self.readable.eq(0),
        ).Elif(self.re | ~self.readable,
            self.dout.eq(fifo.dout),
            self.readable.eq(fifo.readable),
        )


class SyncFifo(LiteXModule):
    """Single-clock queue in memory, first word fall through, one entry a cycle.

    The read port's own output register is the head. Its read enable is gated,
    so the entry it holds stays there until the next fetch is issued, and a
    fetch is only issued when that entry has been taken or was never there.
    Issuing one while the head is still occupied would overwrite it the
    following cycle, which is the trap in the obvious version of this.
    """

    def __init__(self, layout, depth):
        self.sink = sink = stream.Endpoint(layout)
        self.source = source = stream.Endpoint(layout)
        self.level = Signal(max=depth + 1)

        # # #

        width = len(sink.payload.raw_bits())
        storage = Memory(width, depth)
        self.specials += storage
        # read_capable on the write port only so the simulator's memory
        # lowering has a read-data signal to drive.
        write_port = storage.get_port(write_capable=True)
        read_port = storage.get_port(has_re=True)
        self.specials += write_port, read_port

        wr = Signal(max=depth)
        rd = Signal(max=depth)
        stored = Signal(max=depth + 1)
        have = Signal()
        write = Signal()
        fetch = Signal()
        consume = Signal()

        self.comb += [
            write.eq(sink.valid & sink.ready),
            consume.eq(have & source.ready),
            fetch.eq((stored != 0) & (~have | consume)),
            self.level.eq(stored + have),
            sink.ready.eq(self.level != depth),
            source.valid.eq(have),
            source.payload.raw_bits().eq(read_port.dat_r),
            write_port.adr.eq(wr),
            write_port.dat_w.eq(sink.payload.raw_bits()),
            write_port.we.eq(write),
            read_port.adr.eq(rd),
            read_port.re.eq(fetch),
        ]
        self.sync += [
            If(write, wr.eq(Mux(wr == depth - 1, 0, wr + 1))),
            If(fetch, rd.eq(Mux(rd == depth - 1, 0, rd + 1))),
            have.eq(fetch | (have & ~consume)),
            stored.eq(stored + write - fetch),
        ]
