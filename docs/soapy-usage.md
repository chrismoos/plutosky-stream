# SoapySDR module

`soapy/` builds `plutosky`, a SoapySDR device module, and `plutosky_probe`, a
command-line check of the module. The module opens the board as a two-channel,
full-duplex radio with timestamped streams.

## Installation

The README gives the build and install commands. The module must be in the
search path of the SoapySDR runtime that loads it, which `SoapySDRUtil --info`
lists. A build tree works without installing:

```sh
export SOAPY_SDR_PLUGIN_PATH=$PWD/soapy/build
```

`plutosky_probe` checks discovery, receive, transmit and full duplex, and ends
each run with a `RESULT` line:

```sh
plutosky_probe --find
plutosky_probe --probe  --args=driver=plutosky,addr=192.168.1.10
plutosky_probe --duplex --args=driver=plutosky --rate=1000000 --secs=1
```

## Device arguments

| Argument | Default | Effect |
|---|---|---|
| `addr` | Discovered | Board address. Skips discovery. |
| `hardware` | Board serial | Name a UHD application matches, for example `B200` or `B210` for osmo-trx |
| `rx_time_offset_ns` | 0 | Subtracted from receive timestamps, rounded to whole samples |
| `cs16_scale` | `12`, or `16` through UHD | Scale of `CS16` samples |
| `duplex_tx_rate_limit_sps` | 10 MS/s or the sample rate, whichever is higher | TX send rate while receive is active |
| `tx_only_rate_limit_sps` | 36 MS/s | TX send rate with no receive stream |

Without `addr`, the module broadcasts on the discovery port and opens the
first board that answers.

## Streams

| Stream argument | Default | Values |
|---|---|---|
| `wire` | `cs12` | `cs12`, `cs8`, `bfp8` |
| `mtu` | `1500` | `1500` |
| `rx_queue` | 1024 | Receive packets buffered before data is lost |

| Wire | Stream formats | Samples per packet, one channel |
|---|---|---|
| `cs12` | `CS12`, `CS16`, `CF32` | 481 |
| `cs8` | `CS8`, `CS16`, `CF32` | 722 |
| `bfp8` | `CS16`, `CF32` | 688 |

Samples are 12-bit. `CF32` full scale is 1.0. A two-channel stream interleaves
both channels in each packet, so each channel gets half.
[`wire-formats.md`](wire-formats.md) describes the packings.

## Sample rates

The range is 16.277 kS/s to 15.36 MS/s per channel, or to 30.72 MS/s on
channel 0 alone.

- `setSampleRate` sets its direction for both channels. A rate cannot change
  while a stream runs.
- Receive and transmit rates may differ when a rate of at most 15.36 MS/s is a
  whole multiple of both, for example 7.68 MS/s receive with 15.36 MS/s
  transmit. A direction the application has not set follows the other.
- When both are set and no such rate exists, such as 10 MS/s with 4 MS/s, the
  direction set first takes the new rate, and its streams fail until it is set
  again.
- Above 15.36 MS/s a stream that includes channel 1 fails to open.

Full duplex sustains 13 to 14 MS/s with one channel, or 6.5 to 7 MS/s per
channel with two. Faster rates lose some TX packets, and each loss reports an
underflow. Receive alone and transmit alone are not limited this way.

## Timestamps and transmit

- Timestamps are nanoseconds. `readStream` returns the time of the first
  sample it returns. `getHardwareTime` and `setHardwareTime` read and set the
  board's clock.
- A transmit write with `SOAPY_SDR_HAS_TIME` starts at that time. Without it,
  a write continues the burst, or starts 10 ms ahead when there is none.
  `SOAPY_SDR_END_BURST` ends a burst, and a gap within a burst plays as zeros.
- A packet must reach the board about 1 ms before its timestamp, or it is
  dropped as late. Writers with tight deadlines should keep 2 to 3 ms of lead.
- There is no PPS input and no external reference. Timed tuning and gain
  commands apply at once.
- `tools/rx_delay_measure.py` measures the TX-to-RX delay and the
  `rx_time_offset_ns` that matches a reference device such as osmo-trx's B200.

`readStreamStatus` reports `SOAPY_SDR_TIME_ERROR` for a late packet,
`SOAPY_SDR_UNDERFLOW` when transmit ran out of samples, `SOAPY_SDR_OVERFLOW`
when receive lost samples, and `SOAPY_SDR_END_BURST` when a burst finished
playing.

## Gain and tuning

| Setting | Range |
|---|---|
| Frequency | 70 MHz to 6 GHz |
| Receive gain | -3 to 73 dB in 1 dB steps |
| Transmit gain | 0 to 89.75 dB in 0.25 dB steps, relative, 0 dB is maximum attenuation |
| Bandwidth | 200 kHz to 20 MHz |

Both channels of a direction share one LO. Stopping the transmit stream turns
TX RF off, and 0 dB gain does not.

## Tests

```sh
cmake --build soapy/build -j && ctest --test-dir soapy/build
python3 -m pytest tools/tests
```

The pytest suite drives the module against `tools/fake_daemon.py`, a stand-in
for the board.
