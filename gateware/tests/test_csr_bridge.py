"""The pipelined CSR bridge, driven the way the SoC's main bus drives it."""

from migen import Module, run_simulation
from litex.gen import LiteXModule
from litex.soc.interconnect import csr_bus, wishbone
from litex.soc.interconnect.csr import AutoCSR, CSRStatus, CSRStorage

from cores.csr_bridge import PipelinedCsrBridge

DATA_WIDTH = 32
ADDRESS_WIDTH = 6


class _Bank(LiteXModule, AutoCSR):
    def __init__(self):
        self.control = CSRStorage(DATA_WIDTH)
        self.marker = CSRStorage(DATA_WIDTH)
        self.mirror = CSRStatus(DATA_WIDTH)
        self.comb += self.mirror.status.eq(self.control.storage + 1)


class _Dut(Module):
    def __init__(self):
        self.bus = wishbone.Interface(data_width=DATA_WIDTH, adr_width=30,
                                      addressing="word")
        csr_if = csr_bus.Interface(data_width=DATA_WIDTH,
                                   address_width=ADDRESS_WIDTH)
        self.submodules.bridge = PipelinedCsrBridge(self.bus, csr_if)
        self.submodules.bank_logic = bank = _Bank()
        self.submodules.bank = csr_bus.CSRBank(bank.get_csrs(), address=0,
                                               bus=csr_if)
        self.csrs = {csr.name: n for n, csr in enumerate(bank.get_csrs())}


def _cycle(bus, adr, we, value=0):
    yield bus.adr.eq(adr)
    yield bus.dat_w.eq(value)
    yield bus.sel.eq(0xF)
    yield bus.we.eq(1 if we else 0)
    yield bus.cyc.eq(1)
    yield bus.stb.eq(1)
    yield
    while not (yield bus.ack):
        yield
    read = yield bus.dat_r
    yield bus.cyc.eq(0)
    yield bus.stb.eq(0)
    yield
    return read


def test_a_write_lands_in_the_register_it_addresses():
    dut = _Dut()
    seen = {}

    def stim():
        yield from _cycle(dut.bus, dut.csrs["control"], True, 0xDEAD_BEEF)
        yield from _cycle(dut.bus, dut.csrs["marker"], True, 0x0BAD_F00D)
        seen["control"] = yield dut.bank_logic.control.storage
        seen["marker"] = yield dut.bank_logic.marker.storage

    run_simulation(dut, stim())
    assert seen["control"] == 0xDEAD_BEEF
    assert seen["marker"] == 0x0BAD_F00D


def test_a_read_returns_what_the_addressed_register_holds():
    dut = _Dut()
    seen = {}

    def stim():
        yield from _cycle(dut.bus, dut.csrs["control"], True, 0x1234_5677)
        yield from _cycle(dut.bus, dut.csrs["marker"], True, 0x0000_00AA)
        seen["mirror"] = yield from _cycle(dut.bus, dut.csrs["mirror"], False)
        seen["control"] = yield from _cycle(dut.bus, dut.csrs["control"], False)
        seen["marker"] = yield from _cycle(dut.bus, dut.csrs["marker"], False)

    run_simulation(dut, stim())
    assert seen["control"] == 0x1234_5677
    assert seen["marker"] == 0x0000_00AA
    assert seen["mirror"] == 0x1234_5678


def test_a_write_strobe_lasts_one_cycle():
    dut = _Dut()
    counts = {"we": 0, "re": 0}

    def watch():
        for _ in range(60):
            counts["we"] += yield dut.bridge.csr.we
            counts["re"] += yield dut.bridge.csr.re
            yield

    def stim():
        yield from _cycle(dut.bus, dut.csrs["control"], True, 1)
        yield from _cycle(dut.bus, dut.csrs["control"], False)

    run_simulation(dut, [stim(), watch()])
    assert counts["we"] == 1
    assert counts["re"] == 1
