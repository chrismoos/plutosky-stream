# Kernel

The board kernel is the ADI Linux tree the PlutoSDR firmware uses, with a patch
to the Ethernet driver. Received and transmitted frames sit in a reserved pool
of DDR slots that the FPGA reads and writes directly, and the driver passes
slot numbers between the Ethernet controller and the FPGA. Ordinary traffic,
SSH included, shares `eth0`. The [GEM slot interface](../docs/slot-ioctl.md)
describes what the daemon uses.

| Path | Content |
|---|---|
| `patches/` | The Ethernet driver patch |
| `dts/` | Device tree sources |
| `ramdisk/` | Adds the daemon and its init script to the stock PlutoSDR ramdisk |
| `out/` | The built kernel, device trees and ramdisk |

## Building

The build requires `dtc`, `bison`, `flex`, `bc`, `fakeroot` and
`arm-linux-gnueabihf-gcc-11`.

```sh
./kernel/build.sh
VENDOR_RAMDISK=<stock uramdisk.image.gz> ./kernel/ramdisk/build.sh <daemon> <csr.json>
```

`VENDOR_RAMDISK` is the stock PlutoSDR ramdisk from the board's SD card.

## Board configuration

`/opt/plutosky/config` on the board sets the Ethernet address, netmask and MTU,
the daemon's options, and the real-time priority of the kernel thread that
hands transmit frames to the FPGA.
