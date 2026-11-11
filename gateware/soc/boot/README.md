# Boot image

`BOOT.bin` holds the FSBL, the bitstream and u-boot, packed by `bootgen`. The
board's boot ROM loads it from the SD card, and the FSBL programs the FPGA
from it at power-on.

`bootgen.bif` defines the layout. It requires three files beside it:
`fsbl.elf` and `u-boot.elf` from the board's stock SD card, and
`plutosky_stream.bit` from the Vivado build.

```sh
source <Vivado install>/settings64.sh
bootgen -image bootgen.bif -arch zynq -o BOOT.bin -w on
```

`soc/collect_artifacts.sh` runs the same step. The shipped `BOOT.bin` is in
`../../build_artifacts/vivado-2r2t/`.

The bitstream's PS7 configuration sets FCLK0 to 50 MHz and FCLK1 to 100 MHz,
and the kernel device tree must set the same rates.

A running board loads the byte-swapped `plutosky_stream.bit.bin` through
`fpga_manager`. The daemon must be stopped first, because programming the FPGA
clears every register.
