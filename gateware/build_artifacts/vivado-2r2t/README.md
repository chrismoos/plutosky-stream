# Vivado 2R2T build

Two RF channels, one receive stream and one transmit stream. Each stream
interleaves both channels. The build exports `CHANNELS = 2`, `RX_STREAMS = 1` and
`TX_STREAMS = 1`, and captures RX data and frame in IDDR inputs. The daemon reads
this directory's `csr.json`.

| File | Use |
|---|---|
| `csr.json` | register map for the daemon |
| `BOOT.bin` | SD card boot image |
| `plutosky_stream.bit.bin` | bitstream for `fpga_manager` on a running board |
| `plutosky_stream.bit` | bitstream before byte swapping |
| `plutosky_stream_*.rpt` | routed timing, utilization, DRC, IO and route-status reports |
| `timing_summary.txt` | per-clock timing table from the Vivado log |

The bitstream requires FCLK0 at 50 MHz and FCLK1 at 100 MHz. When the kernel
sets fabric clocks from the device tree, it must set `fclk0` to 50 000 000 and
`fclk1` to 100 000 000. `kernel/out/devicetree-split.dtb` sets both.
