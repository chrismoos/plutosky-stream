# Control protocol

The daemon serves the gRPC service `plutosky.v1.Control` on TCP port 30000,
over plain HTTP/2 without TLS. The service is defined in
[`proto/plutosky/v1/control.proto`](../proto/plutosky/v1/control.proto), and a
client in any language can be generated from it. Sample data does not use this
service. It moves as VITA 49.2 packets over UDP, as
[`wire-formats.md`](wire-formats.md) describes.

## Discovery

A UDP datagram to port 30001 with the payload `plutosky-stream?`, broadcast or
sent to the board, is answered with a `DiscoveryReply`: name, serial, version,
control port and MAC address.

## Sessions

Read-only calls need no session. Calls that change the radio or a stream must
send the metadata key `plutosky-session`, set to the id of an open session.

`Session` is a bidirectional stream. The client sends `SessionStart`, the
daemon answers with `SessionOpened` and the session id, and the session lasts
as long as the call.

- The first call from a session that changes the radio or a stream takes
  control of the radio. Another session's claim fails with
  `FAILED_PRECONDITION` while the controller has a running stream, and
  otherwise takes over.
- When the controlling session ends, its streams close and TX turns off. A
  client that stops answering keepalive pings for 6 s is ended.
- While the controller has open streams, the daemon sends a `StatusTick` with
  every stream's counters each status interval, 10 ms by default.
  `SetStatusInterval` sets it from 1 to 1000 ms.

| Status counter | Receive | Transmit |
|---|---|---|
| `produced` | Packets written by the FPGA | Packets handed to the FPGA |
| `consumed` | Packets sent to the host | Timestamp of the last packet played plus its sample count |
| `late` | | Packets dropped because their timestamp had passed |
| `underrun` | | Sample periods an open burst had nothing to send |
| `overflow` | Samples dropped for want of a free slot | |
| `gaps` | | Breaks in the packet count |

## Radio

`GetInfo` returns the serial, version, channel count, rate range, wire formats
and MTUs. Each setter returns the value the radio settled on.

| RPC | Request |
|---|---|
| `SetFrequency`, `GetFrequency` | `dir`, `chan`, `hz` |
| `SetSampleRate`, `GetSampleRate` | `hz`, `dir` (optional) |
| `SetBandwidth`, `GetBandwidth` | `dir`, `chan`, `hz` |
| `SetGain`, `GetGain`, `SetGainMode` | `dir`, `chan`, `db` or `mode` |
| `ListAntennas`, `SetAntenna` | `dir`, `chan`, `name` |
| `SetLoopback` | `on` |
| `GetHardwareTime`, `SetHardwareTime` | `ticks` |
| `GetSensors` | |

- Both channels of a direction share one synthesizer and one sample rate.
- `SetSampleRate` sets one direction, or both when `dir` is omitted. It fails
  while a stream runs and takes a few seconds, so its deadline must be at
  least 10 s.
- Receive and transmit rates may differ when a rate of at most 15.36 MS/s is a
  whole multiple of both. The transceiver runs at that rate and the FPGA
  divides it down per direction. Above 15.36 MS/s only channel 0 runs and both
  directions must be equal.
- The reply reports both rates and `tick_rate_hz`, the rate the timebase
  counts at. Timestamps are in ticks at that rate, and a sample spans
  `rx_ticks_per_sample` or `tx_ticks_per_sample` ticks. `rx_delay_ticks` and
  `tx_delay_ticks` give the FPGA rate filter's delay per direction.
- TX RF turns on when a TX stream starts and off when none runs. TX gain 0 dB
  is maximum attenuation, not RF off.

## Streams

| RPC | Request | Reply |
|---|---|---|
| `OpenStream` | `dir`, `chans`, `format`, `mtu`, `host_ip`, `host_port` | `stream_id`, `data_port`, `samples_per_packet`, `window` |
| `StartStream` | `stream_id`, `at_ticks` (optional) | |
| `StopStream`, `CloseStream` | `stream_id` | |

- Before `StartStream` the host sends one VITA data packet with an empty
  payload from its data socket to `data_port`. The daemon sends the stream's
  frames back to that packet's source address and port.
- `window` is how many transmit packets the host may keep outstanding.
- `at_ticks` starts a receive stream at that timebase count. Transmit packets
  hold their own timestamps.

## Errors

A failed call returns a gRPC status with a message that states the cause.
`INVALID_ARGUMENT` is a malformed request, `NOT_FOUND` an unknown stream,
channel, antenna or MTU, `FAILED_PRECONDITION` a missing session, another
controller or a running stream, `OUT_OF_RANGE` a value outside its range,
`UNAVAILABLE` a transceiver refusal, and `ABORTED` a stream that could not be
opened, started or stopped.
