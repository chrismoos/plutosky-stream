# Open-toolchain build flow

`soc.targets.plutosky_stream` can target yosys, nextpnr-xilinx and prjxray
instead of Vivado with `--toolchain openxc7`. No bitstream from this flow is
shipped. The only shipped bitstream is the Vivado build in
`../build_artifacts/vivado-2r2t/`.

## Requirements

- `yosys`
- `nextpnr-xilinx` with a chip database for `xc7z020clg400-1`
- prjxray: `xc7frames2bit`, a `fasm2frames` wrapper and the `fasm` Python
  package
- `bootgen`, only to produce the `.bit.bin` that `fpga_manager` loads

`nextpnr-xilinx`, `xc7frames2bit` and `fasm2frames` must be on `PATH`.
`CHIPDB` must name the directory holding `xc7z020clg400.bin`, and
`PRJXRAY_DB_DIR` the prjxray database directory.

## Build

```
.venv/bin/python -m soc.targets.plutosky_stream --build --no-compile-software \
    --toolchain openxc7 --output-dir build/openxc7
```

`--no-compile-software` skips the LiteX software step while still running the
gateware build. `--no-compile` writes project files only.

The target runs nextpnr with `timingstrict`, so a timing miss fails the build
and no bitstream is produced. `OPENXC7_SEED` sets the placement seed (default
7). Routing success depends on the seed.

`--sys-clk-freq` moves the control-plane constraint and the PS7 FCLK0 divisors
together, and `--stream-clk-freq` does the same for FCLK1. A bitstream built
this way must run with PS7 clocks that match the frequencies it was built for.

## Not validated

The flow has not been validated on hardware for `LVDS_25` on the 1.8 V banks,
the ODDR settings the AD9361 PHY relies on, or `--rx-iddr`. The design has not
been shown to meet timing at the default 50 MHz and 100 MHz clocks with this
flow.
