"""An AXI3 slave memory for the simulations, standing in for the PS7 HP port.

Both sides take a ``ready`` callable of the cycle number so a test can squeeze
the master with arbitrary back-pressure, and the read side takes a latency so a
test can check that outstanding transactions really do overlap.
"""

from collections import deque

from migen import passive

WRITE_RESP_OKAY = 0


class AxiMemory:
    """Byte-addressed backing store shared by the write and read generators."""

    def __init__(self, size, base=0):
        self.data = bytearray(size)
        self.base = base
        self.writes = []
        self.reads = []
        self.max_write_outstanding = 0
        self.max_read_outstanding = 0
        self.w_beats = 0
        self.r_beats = 0

    def __getitem__(self, key):
        if isinstance(key, slice):
            return bytes(self.data[key.start - self.base:key.stop - self.base])
        return self.data[key - self.base]

    def load(self, addr, payload):
        off = addr - self.base
        self.data[off:off + len(payload)] = payload


def _always(_cycle):
    return True


@passive
def write_generator(axi, memory, aw_ready=_always, w_ready=_always, b_delay=0):
    """Drive AWREADY/WREADY/BVALID and commit accepted beats into ``memory``."""
    beat_bytes = axi.data_width // 8
    bursts = deque()
    current = None
    responses = deque()
    outstanding = 0
    cycle = 0
    yield axi.aw.ready.eq(1)
    yield axi.w.ready.eq(1)
    yield axi.b.resp.eq(WRITE_RESP_OKAY)
    while True:
        if (yield axi.aw.valid) and (yield axi.aw.ready):
            addr = yield axi.aw.addr
            length = yield axi.aw.len
            burst = yield axi.aw.burst
            size = yield axi.aw.size
            assert burst == 0b01, "only INCR bursts are expected"
            assert size == (beat_bytes - 1).bit_length(), "beats must fill the bus"
            assert addr % beat_bytes == 0, "bursts must start on a beat boundary"
            assert (addr & ~0xFFF) == ((addr + (length + 1) * beat_bytes - 1) & ~0xFFF), \
                "a burst must not cross a 4 KB boundary"
            bursts.append([addr, length + 1])
            outstanding += 1
            memory.max_write_outstanding = max(memory.max_write_outstanding, outstanding)

        if (yield axi.w.valid) and (yield axi.w.ready):
            if current is None:
                assert bursts, "write data arrived before its address"
                current = bursts.popleft()
            memory.w_beats += 1
            data = yield axi.w.data
            strb = yield axi.w.strb
            last = yield axi.w.last
            addr, beats = current
            for lane in range(beat_bytes):
                if strb & (1 << lane):
                    memory.data[addr + lane - memory.base] = (data >> (8 * lane)) & 0xFF
                    memory.writes.append(addr + lane)
            current[0] = addr + beat_bytes
            current[1] = beats - 1
            assert (current[1] == 0) == bool(last), "WLAST must land on the final beat"
            if current[1] == 0:
                current = None
                responses.append(b_delay)

        if (yield axi.b.valid) and (yield axi.b.ready):
            outstanding -= 1
            yield axi.b.valid.eq(0)

        if responses and responses[0] == 0 and not (yield axi.b.valid):
            responses.popleft()
            yield axi.b.valid.eq(1)
        elif responses:
            responses[0] = max(0, responses[0] - 1)

        cycle += 1
        yield axi.aw.ready.eq(1 if aw_ready(cycle) else 0)
        yield axi.w.ready.eq(1 if w_ready(cycle) else 0)
        yield


@passive
def read_generator(axi, memory, ar_ready=_always, latency=2):
    """Drive ARREADY and the R channel out of ``memory``."""
    beat_bytes = axi.data_width // 8
    pending = deque()
    active = None
    cycle = 0
    yield axi.ar.ready.eq(1)
    yield axi.r.resp.eq(0)
    while True:
        if (yield axi.ar.valid) and (yield axi.ar.ready):
            addr = yield axi.ar.addr
            length = yield axi.ar.len
            burst = yield axi.ar.burst
            assert burst == 0b01, "only INCR bursts are expected"
            assert addr % beat_bytes == 0, "bursts must start on a beat boundary"
            assert (addr & ~0xFFF) == ((addr + (length + 1) * beat_bytes - 1) & ~0xFFF), \
                "a burst must not cross a 4 KB boundary"
            pending.append([addr, length + 1, latency])
            memory.reads.append((addr, length + 1))
            memory.max_read_outstanding = max(memory.max_read_outstanding, len(pending))

        accepted = (yield axi.r.valid) and (yield axi.r.ready)
        if accepted:
            memory.r_beats += 1
            active[0] += beat_bytes
            active[1] -= 1
            if active[1] == 0:
                active = None
                pending.popleft()

        # Latency is a pipeline delay, not a per-burst stall: a burst becomes
        # ready that many cycles after its address was accepted, so bursts
        # issued early return back to back. A slave that stalled for the whole
        # latency between every burst would make an outstanding-transaction
        # measurement meaningless.
        for entry in pending:
            if entry[2] > 0:
                entry[2] -= 1
        if active is None and pending and pending[0][2] == 0:
            active = pending[0]

        if active is not None and (accepted or not (yield axi.r.valid)):
            addr, beats, _ = active
            off = addr - memory.base
            word = int.from_bytes(memory.data[off:off + beat_bytes], "little")
            yield axi.r.valid.eq(1)
            yield axi.r.data.eq(word)
            yield axi.r.last.eq(1 if beats == 1 else 0)
        elif active is None:
            yield axi.r.valid.eq(0)

        cycle += 1
        yield axi.ar.ready.eq(1 if ar_ready(cycle) else 0)
        yield
