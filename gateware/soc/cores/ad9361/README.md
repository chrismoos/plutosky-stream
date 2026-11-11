# AD9361 PHY and SPI cores

`phy.py`, `spi.py` and `prbs.py` come from the LiteX-M2SDR project
(Enjoy-Digital, BSD-2-Clause, commit 8c363ec6). `LICENSE` holds the license
text. `phy.py` has local changes: an optional `IDDR` receive capture, the
receive delay register kept for register-map compatibility, and a receive
pin-activity probe. They provide:

- `phy.py`: the full-duplex LVDS PHY (1R1T and 2R2T), with 12-bit I/Q stream
  endpoints in the `rfic` clock domain recovered from DATA_CLK.
- `spi.py`: the 24-bit SPI master.
- `prbs.py`: a PRBS checker for interface bring-up.

The M2SDR `core.py` (PCIe DMA packer) is not included.
