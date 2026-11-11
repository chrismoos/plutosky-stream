# plutosky-stream

[![CI](https://github.com/chrismoos/plutosky-stream/actions/workflows/ci.yml/badge.svg)](https://github.com/chrismoos/plutosky-stream/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

Timestamped I/Q streaming for the PlutoSky R1, a Zynq-7020 board with an
AD9363 transceiver and Gigabit Ethernet. A SoapySDR module presents the board
to the host as a two-channel full-duplex radio.

Receive samples are packed into VITA 49.2 packets by the FPGA and sent over
Ethernet with no per-packet work on the board's CPU. Transmit packets take the
reverse path and go out when the board's sample clock reaches their timestamp.

## Specifications

| Item | Value |
|---|---|
| Channels | 2 RX and 2 TX (2R2T), full duplex |
| Sample rate | 16.277 kS/s to 15.36 MS/s per channel, or to 30.72 MS/s on channel 0 alone |
| Sustained full duplex | 13 to 14 MS/s with one channel, or 6.5 to 7 MS/s per channel with two |
| Transmit only | 30.72 MS/s |
| Frequency | 70 MHz to 6 GHz |
| Analog bandwidth | 200 kHz to 20 MHz |
| Wire formats | 12-bit, 8-bit and block floating point 8-bit |
| Host link | Gigabit Ethernet, MTU 1500 |
| Time and clock source | internal only |

## Quick start

1. Copy the SD card set in `sdcard/` to the FAT partition of a card, or
   download it:

   ```sh
   for f in BOOT.bin uImage uramdisk.image.gz devicetree.dtb uEnv.txt SHA256SUMS; do
     curl -fLO https://github.com/chrismoos/plutosky-stream/raw/main/sdcard/$f
   done
   sha256sum -c SHA256SUMS
   ```

   Boot the board with the DIP switch in SD mode. The daemon starts at boot,
   and the Ethernet port comes up at 192.168.1.10/24. A directly connected host
   needs an address on 192.168.1.0/24, such as 192.168.1.2. SSH logs in as
   `root` with password `analog`.

2. Build and install the SoapySDR module. The module must be installed into
   the prefix of the SoapySDR runtime that loads it.

   Linux (Debian or Ubuntu):

   ```sh
   sudo apt install cmake g++ libsoapysdr-dev soapysdr-tools \
     libgrpc++-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc
   cmake -S soapy -B soapy/build -DCMAKE_BUILD_TYPE=Release
   cmake --build soapy/build -j
   sudo cmake --install soapy/build
   ```

   macOS (Homebrew):

   ```sh
   brew install cmake soapysdr grpc
   cmake -S soapy -B soapy/build -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_PREFIX_PATH=$(brew --prefix) -DCMAKE_INSTALL_PREFIX=$(brew --prefix)
   cmake --build soapy/build -j
   cmake --install soapy/build
   ```

   The module links against gRPC and protobuf. After an upgrade of either, the
   module must be rebuilt and reinstalled.

3. Find the board and probe it.

   ```sh
   SoapySDRUtil --find="driver=plutosky"
   SoapySDRUtil --probe="driver=plutosky,addr=192.168.1.10"
   ```

## Clients

| Client | Device string |
|---|---|
| SoapySDR applications and GNU Radio (`gr-soapy`) | `driver=plutosky` |
| UHD applications through SoapyUHD | `type=soapy,driver=plutosky` |

Device arguments, stream arguments and sample formats are in
[docs/soapy-usage.md](docs/soapy-usage.md).

## Limits

- **Full duplex:** sustained at 13 to 14 MS/s on one channel, or 6.5 to
  7 MS/s per channel on two, with 12-bit samples. Above that, the board starts
  dropping transmit packets: about 1 in 300,000 at 15.36 MS/s and 1 in 10,000
  at 20 MS/s. Receive drops nothing at these rates.
- **One direction only:** transmit alone or receive alone runs with no packet
  loss up to 30.72 MS/s.
- **Sample rates:** each direction has one rate, shared by both channels.
  Receive and transmit can run at different rates when one rate of at most
  15.36 MS/s is a whole multiple of both. Rates cannot change while a stream
  is running.
- **Tuning:** each direction has one LO, shared by both channels.
- **Clock:** there is no PPS input and no external reference.

## Building the SD card set

`sdcard/` is built from the prebuilt images in the repository. `make_sd.sh`
requires `mkimage` from u-boot-tools.

```sh
./boot/make_sd.sh kernel/out/zImage kernel/out/devicetree-split.dtb \
    kernel/out/rootfs.cpio.gz sdcard gateware/build_artifacts/vivado-2r2t/BOOT.bin
```

The images themselves are rebuilt as `kernel/README.md`, `daemon/README.md` and
`gateware/README.md` describe. The board address is `ETH_IPADDR` in
`kernel/ramdisk/overlay/opt/plutosky/config`.

## Documentation

| Document | Content |
|---|---|
| [SoapySDR module](docs/soapy-usage.md) | Device and stream arguments, formats, rates, timestamps, transmit behavior |
| [Wire formats](docs/wire-formats.md) | VITA 49.2 packet layout, the cs12, cs8 and bfp8 packings, packet sizes |
| [Control protocol](docs/control-protocol.md) | The daemon's gRPC service: sessions, radio and stream calls, status |
| [CSR contract](docs/csr-contract.md) | The FPGA registers the daemon programs |
| [GEM slot interface](docs/slot-ioctl.md) | The kernel driver's slot pool and ioctls |
| [Kernel](kernel/README.md), [daemon](daemon/README.md), [gateware](gateware/README.md) | What each part does and how to build it |

## License

Apache-2.0, with the exceptions listed in `NOTICE`.
