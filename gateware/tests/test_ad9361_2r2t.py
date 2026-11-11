"""The AD9361 PHY in 2R2T.

The PHY has a mode register and separate 'a' and 'b' lanes. These tests drive the RFIC side of the LVDS link in
an event simulator and check what comes back out.

Migen's own simulator cannot answer this, because the whole capture path runs
through IBUFDS, BUFG and ODDR instances it treats as black boxes. So the PHY is
converted to Verilog and run under Icarus against behavioral models of those
primitives, with the bus driven the way the RFIC drives it.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE / "phy_sim"
GATEWARE = HERE.parent

pytestmark = pytest.mark.skipif(
    shutil.which("iverilog") is None or shutil.which("vvp") is None,
    reason="needs iverilog and vvp")

MODE_2R2T = 0
MODE_1R1T = 1


def generate_phy_verilog(path):
    """Convert the PHY on its own, with everything a testbench needs exposed."""
    from migen import Module, Record, Signal
    from migen.fhdl.verilog import convert
    from litex.build.xilinx.common import xilinx_special_overrides

    from soc.cores.ad9361.phy import AD9361PHY

    class PhyTop(Module):
        def __init__(self):
            self.pads = pads = Record([
                ("rx_clk_p", 1), ("rx_clk_n", 1),
                ("rx_frame_p", 1), ("rx_frame_n", 1),
                ("rx_data_p", 6), ("rx_data_n", 6),
                ("tx_clk_p", 1), ("tx_clk_n", 1),
                ("tx_frame_p", 1), ("tx_frame_n", 1),
                ("tx_data_p", 6), ("tx_data_n", 6),
            ])
            self.submodules.phy = phy = AD9361PHY(pads)
            self.mode = Signal()
            self.tx_valid = Signal()
            self.tx_ready = Signal()
            self.tx_ia = Signal(12)
            self.tx_qa = Signal(12)
            self.tx_ib = Signal(12)
            self.tx_qb = Signal(12)
            self.rx_valid = Signal()
            self.rx_ia = Signal(12)
            self.rx_qa = Signal(12)
            self.rx_ib = Signal(12)
            self.rx_qb = Signal(12)
            self.comb += [
                phy.control.storage.eq(self.mode),
                # The register's field logic lives inside the CSR object, which
                # only joins the design once a CSR bank finalises it. A SoC does
                # that. Here the field is driven directly instead.
                phy.control.fields.mode.eq(self.mode),
                phy.sink.valid.eq(self.tx_valid),
                self.tx_ready.eq(phy.sink.ready),
                phy.sink.ia.eq(self.tx_ia), phy.sink.qa.eq(self.tx_qa),
                phy.sink.ib.eq(self.tx_ib), phy.sink.qb.eq(self.tx_qb),
                phy.source.ready.eq(1),
                self.rx_valid.eq(phy.source.valid),
                self.rx_ia.eq(phy.source.ia), self.rx_qa.eq(phy.source.qa),
                self.rx_ib.eq(phy.source.ib), self.rx_qb.eq(phy.source.qb),
            ]

    top = PhyTop()
    ios = {top.mode, top.tx_valid, top.tx_ready, top.tx_ia, top.tx_qa,
           top.tx_ib, top.tx_qb, top.rx_valid, top.rx_ia, top.rx_qa,
           top.rx_ib, top.rx_qb}
    for entry in top.pads.layout:
        ios.add(getattr(top.pads, entry[0]))
    path.write_text(str(convert(top, ios=ios, name="ad9361_phy_top",
                                special_overrides=xilinx_special_overrides)))


def run_phy_sim(tmp_path, mode, samples=8, period=8):
    phy = tmp_path / "ad9361_phy_top.v"
    generate_phy_verilog(phy)
    binary = tmp_path / "phy.vvp"
    subprocess.run(
        ["iverilog", "-g2005", "-o", str(binary),
         f"-Ptb.MODE={mode}", f"-Ptb.SAMPLES={samples}", f"-Ptb.PERIOD={period}",
         str(SIM_DIR / "tb_ad9361_phy.v"), str(SIM_DIR / "xilinx_primitives.v"),
         str(phy)],
        check=True, capture_output=True, text=True, cwd=str(GATEWARE))
    result = subprocess.run(["vvp", str(binary)], check=True, capture_output=True,
                            text=True, cwd=str(GATEWARE))
    return result.stdout


def driven_samples(count):
    """The same ramps the testbench drives, as (i1, q1, i2, q2)."""
    return [(0x100 + n * 0x11, (0x700 - n * 0x13) & 0xFFF,
             0xA00 + n * 0x17, 0x0C0 + n * 0x21) for n in range(count)]


def parse(prefix, output):
    rows = []
    for line in output.splitlines():
        parts = line.split()
        if parts and parts[0] == prefix:
            rows.append(tuple(int(value) for value in parts[1:]))
    return rows


def contiguous_index(haystack, needle):
    """Where ``needle`` appears whole inside ``haystack``, or None."""
    for start in range(len(haystack) - len(needle) + 1):
        if haystack[start:start + len(needle)] == needle:
            return start
    return None


def test_the_phy_recovers_both_channels_in_2r2t(tmp_path):
    count = 8
    output = run_phy_sim(tmp_path, MODE_2R2T, samples=count)
    received = parse("RX", output)
    expected = driven_samples(count)

    # The framing only locks on the first frame edge, so the first word out
    # holds a half-assembled channel 0. Every sample after it must be exact,
    # on both channels, in the right lanes.
    wanted = expected[1:]
    start = contiguous_index(received, wanted)
    assert start is not None, (
        f"2R2T receive never produced the driven samples\n got {received}\nwant {wanted}")


def test_the_frame_marks_the_sample_boundary_in_1r1t(tmp_path):
    """1R1T as a control for the framing itself.

    Only the transmit side is checked here. Driving the receive side in 1R1T
    needs the RFIC's own frame convention for that mode, which this testbench
    does not reproduce, so a receive comparison would be testing the testbench.
    """
    output = run_phy_sim(tmp_path, MODE_1R1T, samples=4)
    frame = [row[0] for row in parse("TXBUS", output)]
    start = next(n for n in range(1, len(frame) - 8) if frame[n] and not frame[n - 1])
    assert frame[start:start + 8] == [1, 0, 1, 0, 1, 0, 1, 0], \
        f"1R1T frame is not alternating: {frame[start:start + 8]}"


def decode_tx(rows, mode):
    """Turn the recorded wire back into words, aligned on the frame."""
    words = []
    start = None
    for index in range(1, len(rows)):
        frame_rise, _, _, _ = rows[index]
        previous = rows[index - 1][0]
        if frame_rise and not previous:
            start = index
            break
    if start is None:
        return words
    step = 4 if mode == MODE_2R2T else 2
    for base in range(start, len(rows) - step + 1, step):
        group = rows[base:base + step]
        i_high, q_high = group[0][1], group[0][3]
        i_low, q_low = group[1][1], group[1][3]
        first = ((i_high << 6) | i_low, (q_high << 6) | q_low)
        if step == 2:
            words.append(first)
            continue
        i_high, q_high = group[2][1], group[2][3]
        i_low, q_low = group[3][1], group[3][3]
        second = ((i_high << 6) | i_low, (q_high << 6) | q_low)
        words.append(first + second)
    return words


def test_the_phy_transmits_both_channels_in_2r2t(tmp_path):
    count = 8
    output = run_phy_sim(tmp_path, MODE_2R2T, samples=count)
    wire = decode_tx(parse("TXBUS", output), MODE_2R2T)
    handed = parse("TXWORD", output)
    assert wire, "nothing was decoded off the transmit wire"
    assert handed, "the PHY never asked for a transmit word"
    # Every word the PHY was handed must appear on the wire with both channels
    # in the right lanes.
    start = contiguous_index(wire, handed[:count - 2])
    assert start is not None, (
        f"the handed words did not reach the wire\n wire {wire[:8]}\n handed {handed[:8]}")


def test_the_frame_marks_the_channel_boundary_in_2r2t(tmp_path):
    output = run_phy_sim(tmp_path, MODE_2R2T, samples=4)
    rows = parse("TXBUS", output)
    frame = [row[0] for row in rows]
    # Two clocks high for channel 0 then two low for channel 1, repeating.
    start = next(n for n in range(1, len(frame) - 8) if frame[n] and not frame[n - 1])
    assert frame[start:start + 8] == [1, 1, 0, 0, 1, 1, 0, 0], \
        f"2R2T frame is not two high and two low: {frame[start:start + 8]}"
