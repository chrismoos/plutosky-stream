# GEM slot interface

The Ethernet driver patch in `kernel/patches/` adds `/dev/gemslot0`. Frames
live in a reserved pool of DDR slots that the Ethernet controller and the FPGA
both read and write directly, and the driver moves slot numbers between them.
The definitions are in `include/uapi/linux/gemslot.h`.

## Slot pool

The pool holds two rings: board to host, which the FPGA fills and the Ethernet
controller sends, and host to board, which the Ethernet controller fills and
the FPGA reads. With the shipped kernel and device tree it is 40 MiB at
`0x2D800000`, with 2048 slots of 9728 bytes per ring. A frame starts 2 bytes
into its slot, so the IPv4 header and the VITA packet are word-aligned.

## Calls

Every call is an ioctl with magic `'G'`.

| Number | Call | Purpose |
|---|---|---|
| 0 | `GEMSLOT_GET_INFO` | Pool address, size and slot geometry |
| 1 | `GEMSLOT_TX_SUBMIT` | Diagnostic: send slots from user space |
| 2 | `GEMSLOT_TX_COMPLETIONS` | Diagnostic: slots the controller has sent |
| 3 | `GEMSLOT_RX_STEER` | Diagnostic: divert a UDP port's frames to user space |
| 4 | `GEMSLOT_RX_POLL` | Steered frames waiting |
| 5 | `GEMSLOT_RX_RETURN` | Give steered slots back |
| 6 | `GEMSLOT_WAIT` | Block until completions or steered frames are pending |
| 7 | `GEMSLOT_STREAM_START` | Hand a stream to the kernel pump |
| 8 | `GEMSLOT_STREAM_STOP` | Stop a stream and drain its slots |
| 9 | `GEMSLOT_STREAM_STATUS` | A stream's counters |
| 10 | `GEMSLOT_GET_CAPS` | Bit 0 set when the driver has the kernel pump |

The daemon streams through the kernel pump: a kernel thread moves slots
between the Ethernet controller and the FPGA with no per-frame work in user
space. One stream in each direction may run at a time. The daemon uses
`GEMSLOT_RX_POLL` and `GEMSLOT_RX_RETURN` at start-up to return slots an
earlier run left steered.

Frames that do not belong to a stream, SSH included, are copied into the
normal network stack.
