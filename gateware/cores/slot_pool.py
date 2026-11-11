"""Where the slot rings are in DDR.

The kernel owns the pool: it comes from a reserved-memory node, it is not
cached, and the driver hands its geometry to the daemon through an ioctl. The
daemon copies that geometry here so the packetizers and depacketizers compute
the same slot addresses the driver will hand to the GEM.

Slot i of a ring is at ``pool_base + ring_offset + i * slot_size``, and a frame
starts two bytes into it. Nothing here is written by the PL.
"""

from litex.gen import LiteXModule
from litex.soc.interconnect.csr import CSRStorage


class SlotPool(LiteXModule):
    def __init__(self):
        self.pool_base = CSRStorage(32, description="Physical base of the slot pool.")
        self.slot_size = CSRStorage(32, description="Bytes per slot.")
        self.count = CSRStorage(32, description="Slots per ring.")
        self.tx_offset = CSRStorage(32,
            description="Offset of the ring the packetizers write, the one GEM transmits.")
        self.rx_offset = CSRStorage(32,
            description="Offset of the ring GEM receives into, the one the depacketizers read.")
