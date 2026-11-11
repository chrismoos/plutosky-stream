# plutosky-streamd

The daemon that runs on the board. It brings up the AD9363, configures the
FPGA's stream cores and the kernel's packet path, and serves the
[control protocol](../docs/control-protocol.md) and a UDP discovery responder.
Sample data does not pass through it: once a stream starts, frames move between
the Ethernet controller and the FPGA without it.

It finds every FPGA register by name in the `csr.json` from the gateware build,
so one binary serves any bitstream that exports the names in the
[CSR contract](../docs/csr-contract.md).

## Building

The board binary is a static executable built with `cargo-zigbuild`, which
requires `zig`:

```sh
cargo zigbuild --release --target armv7-unknown-linux-musleabihf
```

`cargo test --release` runs on the host without hardware. `--fake` runs the
daemon on the host against stand-ins for the radio, the FPGA and the kernel:

```sh
cargo run --release -- --fake --csr-json tests/fixtures/csr.json
```
