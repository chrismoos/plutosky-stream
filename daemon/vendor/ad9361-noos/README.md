# AD9361 no-OS driver

Analog Devices' no-OS AD9361 driver (`ad9361.c`, `ad9361_api.c`,
`ad9361_conv.c`, `util.c` and headers), taken from the copy the LiteX-M2SDR
project includes (commit 8c363ec6), under the license in the file headers. The
daemon compiles it with the `cc` crate and supplies the `platform.h` hooks (SPI
transfer, GPIO, delays) from Rust over the SoC's CSRs.

The copy includes LiteX-M2SDR's changes, among them its `M2SDR_OC_*`
environment overrides. This repository adds one change in `ad9361.c`. When
`AD9361_TX_QUAD_CAL_ON_LO_CHANGE_ONLY` is defined, `ad9361_update_rf_bandwidth`
does not run the TX quadrature calibration, so rate and bandwidth changes run
only the filter calibrations. The calibration still runs at initialization and
when the TX LO moves more than 100 MHz, the schedule in UG-570 Table 1. The
daemon build defines it. Without the define the driver behaves as upstream.
