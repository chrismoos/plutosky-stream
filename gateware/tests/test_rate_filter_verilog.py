"""The rate filters as Verilog, against the same integer model.

Migen's simulator evaluates signed arithmetic itself. Vivado reads the Verilog
Migen prints, where a signed operand that loses its sign through an array
lookup or a mixed expression changes the result. So the converted core is
also run under Icarus.
"""

import shutil
import subprocess

import pytest

from tests.test_rate_filter import (LANES, PHASE_TAPS, RATIO_MAX, coefficients, decimate,
                                    interpolate, random_words)
from cores.rate_filter import BANK_BITS, COEF_BITS, PHASE_BITS, RATIO_BITS

pytestmark = pytest.mark.skipif(
    shutil.which("iverilog") is None or shutil.which("vvp") is None,
    reason="needs iverilog and vvp")

SAMPLE_MASK = 0xFFF
ADDR_BITS = PHASE_BITS + BANK_BITS + 1

TESTBENCH = """
`timescale 1ns/1ps
module tb;
  parameter RATIO = 2;
  parameter NCOEF = 1;
  parameter NRX = 1;
  parameter NTX = 1;
  parameter TICKS = 1;
  reg clk = 0;
  always #5 clk = ~clk;
  reg rst = 1;
  reg tick = 0;
  reg [11:0] rx0 = 0, rx1 = 0, rx2 = 0, rx3 = 0;
  reg [11:0] tx0 = 0, tx1 = 0, tx2 = 0, tx3 = 0;
  reg [RATIO_W-1:0] ratio = 1;
  reg [ADDR_W-1:0] coef_addr = 0;
  reg [COEF_W-1:0] coef_data = 0;
  reg coef_we = 0;
  wire tick_out;
  wire tx_tick_out;
  wire [11:0] rxo0, rxo1, rxo2, rxo3, txo0, txo1, txo2, txo3;
  reg [ADDR_W+COEF_W-1:0] coefs [0:NCOEF-1];
  reg [47:0] rxmem [0:NRX-1];
  reg [47:0] txmem [0:NTX-1];
  integer i;
  integer txi = 0;
  reg tx_tick_out_d = 0;

  rate_filter_top dut(
    .sys_clk(clk), .sys_rst(rst), .rfic_clk(clk), .rfic_rst(rst),
    .mode(1'b0), .tick(tick),
    .rx0(rx0), .rx1(rx1), .rx2(rx2), .rx3(rx3),
    .tx0(tx0), .tx1(tx1), .tx2(tx2), .tx3(tx3),
    .ratio(ratio), .coef_addr(coef_addr), .coef_data(coef_data), .coef_we(coef_we),
    .tick_out(tick_out), .tx_tick_out(tx_tick_out),
    .rxo0(rxo0), .rxo1(rxo1), .rxo2(rxo2), .rxo3(rxo3),
    .txo0(txo0), .txo1(txo1), .txo2(txo2), .txo3(txo3));

  always @(posedge clk) begin
    if (tick_out)
      $display("RX %0d %0d %0d %0d", $signed(rxo0), $signed(rxo1), $signed(rxo2), $signed(rxo3));
    if (tick)
      $display("TX %0d %0d %0d %0d", $signed(txo0), $signed(txo1), $signed(txo2), $signed(txo3));
    tx_tick_out_d <= tx_tick_out;
    if (tx_tick_out_d) begin
      if (txi < NTX) {tx0, tx1, tx2, tx3} <= txmem[txi];
      else {tx0, tx1, tx2, tx3} <= 48'd0;
      txi <= txi + 1;
    end
  end

  initial begin
    $readmemh("coefs.hex", coefs);
    $readmemh("rx.hex", rxmem);
    $readmemh("tx.hex", txmem);
    repeat (5) @(posedge clk);
    rst <= 0;
    for (i = 0; i < NCOEF; i = i + 1) begin
      @(posedge clk);
      coef_addr <= coefs[i][ADDR_W+COEF_W-1:COEF_W];
      coef_data <= coefs[i][COEF_W-1:0];
      coef_we <= 1;
      @(posedge clk);
      coef_we <= 0;
    end
    ratio <= RATIO;
    repeat (20) @(posedge clk);
    for (i = 0; i < TICKS; i = i + 1) begin
      @(posedge clk);
      if (i < NRX) {rx0, rx1, rx2, rx3} <= rxmem[i];
      else {rx0, rx1, rx2, rx3} <= 48'd0;
      tick <= 1;
      @(posedge clk);
      tick <= 0;
      repeat (2) @(posedge clk);
    end
    $finish;
  end
endmodule
"""


def generate(path):
    from migen import Module, Signal
    from migen.fhdl.verilog import convert

    from cores.rate_filter import RateFilter

    class Top(Module):
        def __init__(self):
            self.mode = Signal(name="mode")
            self.tick = Signal(name="tick")
            self.rx = [Signal(12, name=f"rx{n}") for n in range(LANES)]
            self.tx = [Signal(12, name=f"tx{n}") for n in range(LANES)]
            self.ratio = Signal(RATIO_BITS, name="ratio")
            self.coef_addr = Signal(ADDR_BITS, name="coef_addr")
            self.coef_data = Signal(COEF_BITS, name="coef_data")
            self.coef_we = Signal(name="coef_we")
            self.tick_out = Signal(name="tick_out")
            self.tx_tick_out = Signal(name="tx_tick_out")
            self.rxo = [Signal(12, name=f"rxo{n}") for n in range(LANES)]
            self.txo = [Signal(12, name=f"txo{n}") for n in range(LANES)]
            self.submodules.dut = dut = RateFilter(self.mode, self.tick, self.rx, self.tx)
            # The CSRs' storage is only driven once a CSR bank finalises them.
            self.comb += [
                dut.rx_ratio.storage.eq(self.ratio),
                dut.tx_ratio.storage.eq(self.ratio),
                dut.coef_addr.storage.eq(self.coef_addr),
                dut.coef_data.storage.eq(self.coef_data),
                dut.coef_data.re.eq(self.coef_we),
                self.tick_out.eq(dut.rx_tick),
                self.tx_tick_out.eq(dut.tx_tick),
                [self.rxo[n].eq(dut.rx_out[n]) for n in range(LANES)],
                [self.txo[n].eq(dut.tx_out[n]) for n in range(LANES)],
            ]

    top = Top()
    ios = {top.mode, top.tick, top.ratio, top.coef_addr, top.coef_data, top.coef_we,
           top.tick_out, top.tx_tick_out, *top.rx, *top.tx, *top.rxo, *top.txo}
    path.write_text(str(convert(top, ios=ios, name="rate_filter_top")))


def word_hex(word):
    value = 0
    for lane in word:
        value = (value << 12) | (lane & SAMPLE_MASK)
    return f"{value:012x}"


def run(tmp_path, ratio, rx_inputs, tx_samples, ticks):
    rx_coefs, tx_coefs = coefficients(ratio)
    lines = []
    for direction, coefs in ((0, rx_coefs), (1, tx_coefs)):
        for n, value in enumerate(coefs):
            bank, phase = divmod(n, ratio)
            addr = phase | (bank << PHASE_BITS) | (direction << (PHASE_BITS + BANK_BITS))
            lines.append(f"{(addr << COEF_BITS) | (int(value) & ((1 << COEF_BITS) - 1)):x}")
    (tmp_path / "coefs.hex").write_text("\n".join(lines) + "\n")
    (tmp_path / "rx.hex").write_text("\n".join(word_hex(w) for w in rx_inputs or [(0,) * 4]) + "\n")
    (tmp_path / "tx.hex").write_text("\n".join(word_hex(w) for w in tx_samples or [(0,) * 4]) + "\n")
    tb = tmp_path / "tb.v"
    tb.write_text(TESTBENCH.replace("RATIO_W", str(RATIO_BITS)).replace("ADDR_W", str(ADDR_BITS))
                  .replace("COEF_W", str(COEF_BITS)))
    dut = tmp_path / "rate_filter_top.v"
    generate(dut)
    binary = tmp_path / "tb.vvp"
    subprocess.run(
        ["iverilog", "-g2005", "-o", str(binary),
         f"-Ptb.RATIO={ratio}", f"-Ptb.NCOEF={len(lines)}",
         f"-Ptb.NRX={max(len(rx_inputs), 1)}", f"-Ptb.NTX={max(len(tx_samples), 1)}",
         f"-Ptb.TICKS={ticks}", str(tb), str(dut)],
        check=True, capture_output=True, text=True)
    result = subprocess.run(["vvp", str(binary)], check=True, capture_output=True, text=True,
                            cwd=str(tmp_path))
    rx_seen, tx_seen = [], []
    for line in result.stdout.splitlines():
        parts = line.split()
        if parts and parts[0] in ("RX", "TX"):
            (rx_seen if parts[0] == "RX" else tx_seen).append(tuple(int(v) for v in parts[1:]))
    return rx_seen, tx_seen, rx_coefs, tx_coefs


@pytest.mark.parametrize("ratio", [2, 7, RATIO_MAX])
def test_verilog_decimator_matches_the_model(tmp_path, ratio):
    outputs = PHASE_TAPS + 4
    inputs = random_words(outputs * ratio, ratio)
    rx_seen, _, rx_coefs, _ = run(tmp_path, ratio, inputs, [], len(inputs) + 4 * ratio)
    for lane in range(LANES):
        want = decimate([w[lane] for w in inputs], rx_coefs, ratio)
        got = [s[lane] for s in rx_seen][:len(want)]
        assert got == want, f"ratio {ratio} lane {lane}\n got {got}\nwant {want}"


@pytest.mark.parametrize("ratio", [2, 7, RATIO_MAX])
def test_verilog_interpolator_matches_the_model(tmp_path, ratio):
    samples = PHASE_TAPS + 4
    stream = random_words(samples, ratio + 100, amplitude=1000)
    _, tx_seen, _, tx_coefs = run(tmp_path, ratio, [], stream, (samples + 6) * ratio)
    for lane in range(LANES):
        want = interpolate([s[lane] for s in stream], tx_coefs, ratio)
        got = [s[lane] for s in tx_seen]
        span = len(want) - ratio
        assert any(got[n:n + span] == want[:span] for n in range(len(got))), (
            f"ratio {ratio} lane {lane}\n got {got}\nwant {want}")
