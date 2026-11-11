"""PRBS word generator and checker agree in 1R1T and 2R2T slot semantics."""

from migen import *

from soc.cores.ad9361.prbs import AD9361PRBSWordChecker, AD9361PRBSWordGenerator


class _Loop(Module):
    def __init__(self, two_channels):
        self.submodules.gen = AD9361PRBSWordGenerator()
        self.submodules.chk = AD9361PRBSWordChecker()
        self.comb += [
            self.gen.two_channels.eq(two_channels),
            self.chk.two_channels.eq(two_channels),
            self.chk.ia.eq(self.gen.ia),
            self.chk.qa.eq(self.gen.qa),
            self.chk.ib.eq(self.gen.ib),
            self.chk.qb.eq(self.gen.qb),
        ]


def _run(two_channels, cross_mode=False):
    dut = _Loop(two_channels)
    if cross_mode:
        dut.comb += dut.chk.two_channels.eq(~two_channels)
    result = {}

    def tb():
        for _ in range(1100):
            yield
        result["synced"] = (yield dut.chk.synced)

    run_simulation(dut, tb())
    return result["synced"]


def test_1r1t_locks():
    assert _run(0) == 1


def test_2r2t_locks():
    assert _run(1) == 1


def test_mode_mismatch_does_not_lock():
    assert _run(0, cross_mode=True) == 0
    assert _run(1, cross_mode=True) == 0


def test_zero_samples_never_lock():
    for two_channels in (0, 1):
        dut = AD9361PRBSWordChecker()
        result = {}

        def tb():
            yield dut.two_channels.eq(two_channels)
            for _ in range(1100):
                yield
            result["synced"] = yield dut.synced

        run_simulation(dut, tb())
        assert result["synced"] == 0
