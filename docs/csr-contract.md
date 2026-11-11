# CSR contract

The FPGA's control and status registers sit on `M_AXI_GP0` from `0x43C00000`,
32 bits per word. The daemon looks each register up by name in the `csr.json`
the gateware build writes, so the names are the contract and the addresses are
not. A register the daemon needs and the map lacks stops the daemon at
start-up. Names follow LiteX, `<core>_<register>`.

| Group | Registers | Purpose |
|---|---|---|
| Identity | `soc_magic_value`, `soc_version_value` | `0x50534B53` ("PSKS") and the CSR layout version, 3. The daemon refuses any other version. |
| Constants | `channels`, `rx_streams`, `tx_streams` | RF channels and stream instances in the bitstream |
| Timebase | `timebase_*` | 64-bit counter of radio samples, with latch, load and a load armed for a given count |
| Rate filter | `rate_filter_*` | Ratio of radio samples to stream samples for each direction, and the filter coefficients. Optional. |
| Slot pool | `slots_*` | Where the slot rings sit in DDR |
| Receive | `rxpkt{N}_*` | Packetizer per receive stream: format, channels, frame template, slot range, counters |
| Transmit | `txpkt{M}_*` | Depacketizer per transmit stream: format, channels, stream id, submit queue, counters |
| AD9363 | `ad9361_*` | SPI, control pins, PHY mode, PRBS and receive delay taps |

- `rxpkt{N}_header` holds the frame template: 72 bytes in 18 words, from the
  destination MAC address through the VITA class id. The packetizer fills in
  the VITA packet count, packet size and timestamp.
- Stream settings must be written while the stream is disabled. They apply at
  the next enable.
- A register wider than one word spans consecutive words, most significant
  first, and takes effect when its last word is written.
