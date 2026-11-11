# Gateware

The FPGA design, written in Migen on LiteX for the Zynq-7020. It packs AD9363
samples into complete Ethernet frames in DDR for the Ethernet controller to
send, and plays received frames out to the AD9363 at their timestamps. A 64-bit
counter of radio samples stamps both directions. A rate filter decimates
receive and interpolates transmit, so each direction can run below the radio's
rate.

`build_artifacts/vivado-2r2t/` holds the shipped bitstream, boot image and
register map. The [CSR contract](../docs/csr-contract.md) lists the registers
the daemon uses.

## Simulation

```sh
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest -q -m "not slow"
```

## Building

Vivado builds the shipped bitstream. The first command writes the Vivado
project and the register map. `build_remote.sh` runs Vivado on another host,
and `collect_artifacts.sh` makes the boot image there and copies the results
back.

```sh
.venv/bin/python -m soc.targets.plutosky_stream --build --no-compile \
    --rx-streams 1 --tx-streams 1 --rx-iddr
REMOTE=user@host soc/build_remote.sh build/plutosky_stream
REMOTE=user@host VENDOR=<dir> soc/collect_artifacts.sh <remote dir> build_artifacts/vivado-2r2t
```
