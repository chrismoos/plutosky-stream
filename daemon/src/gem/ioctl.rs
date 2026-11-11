//! The `/dev/gemslot0` ABI, mirrored from the `macb` patch.
//!
//! The structs are `repr(C)` copies of the kernel's and the request codes are
//! built the same way `_IOR`/`_IOW`/`_IOWR` build them, so a mismatch shows up
//! as an `ENOTTY` rather than as silent corruption.

use super::SLOT_BATCH_MAX;

/// ioctl magic the patch uses for every slot call.
const GEMSLOT_MAGIC: u32 = b'G' as u32;

const IOC_NRBITS: u32 = 8;
const IOC_TYPEBITS: u32 = 8;
const IOC_SIZEBITS: u32 = 14;
const IOC_NRSHIFT: u32 = 0;
const IOC_TYPESHIFT: u32 = IOC_NRSHIFT + IOC_NRBITS;
const IOC_SIZESHIFT: u32 = IOC_TYPESHIFT + IOC_TYPEBITS;
const IOC_DIRSHIFT: u32 = IOC_SIZESHIFT + IOC_SIZEBITS;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

const fn ioc(dir: u32, nr: u32, size: usize) -> libc::c_ulong {
    ((dir << IOC_DIRSHIFT)
        | (GEMSLOT_MAGIC << IOC_TYPESHIFT)
        | (nr << IOC_NRSHIFT)
        | ((size as u32) << IOC_SIZESHIFT)) as libc::c_ulong
}

/// Sequence numbers in the order `docs/slot-ioctl.md` lists the calls.
const NR_GET_INFO: u32 = 0;
const NR_TX_SUBMIT: u32 = 1;
const NR_TX_COMPLETIONS: u32 = 2;
const NR_RX_STEER: u32 = 3;
const NR_RX_POLL: u32 = 4;
const NR_RX_RETURN: u32 = 5;
const NR_WAIT: u32 = 6;
const NR_STREAM_START: u32 = 7;
const NR_STREAM_STOP: u32 = 8;
const NR_STREAM_STATUS: u32 = 9;
const NR_GET_CAPS: u32 = 10;

pub const GEMSLOT_GET_INFO: libc::c_ulong = ioc(IOC_READ, NR_GET_INFO, size_of::<SlotPoolInfo>());
pub const GEMSLOT_TX_SUBMIT: libc::c_ulong = ioc(IOC_WRITE, NR_TX_SUBMIT, size_of::<SlotBatch>());
pub const GEMSLOT_TX_COMPLETIONS: libc::c_ulong =
    ioc(IOC_READ, NR_TX_COMPLETIONS, size_of::<SlotBatch>());
pub const GEMSLOT_RX_STEER: libc::c_ulong = ioc(IOC_WRITE, NR_RX_STEER, size_of::<SteerRequest>());
pub const GEMSLOT_RX_POLL: libc::c_ulong = ioc(IOC_READ, NR_RX_POLL, size_of::<SlotBatch>());
pub const GEMSLOT_RX_RETURN: libc::c_ulong = ioc(IOC_WRITE, NR_RX_RETURN, size_of::<SlotBatch>());
pub const GEMSLOT_WAIT: libc::c_ulong = ioc(IOC_READ | IOC_WRITE, NR_WAIT, size_of::<WaitArgs>());
pub const GEMSLOT_STREAM_START: libc::c_ulong =
    ioc(IOC_WRITE, NR_STREAM_START, size_of::<KernelStreamConfig>());
pub const GEMSLOT_STREAM_STOP: libc::c_ulong = ioc(IOC_WRITE, NR_STREAM_STOP, size_of::<u32>());
pub const GEMSLOT_STREAM_STATUS: libc::c_ulong = ioc(
    IOC_READ | IOC_WRITE,
    NR_STREAM_STATUS,
    size_of::<KernelStreamStatus>(),
);
pub const GEMSLOT_GET_CAPS: libc::c_ulong = ioc(IOC_READ, NR_GET_CAPS, size_of::<u32>());
pub const GEMSLOT_CAP_KERNEL_PUMP: u32 = 1;

/// `GEMSLOT_WAIT` event bits.
pub const GEMSLOT_EVENT_TX: u32 = 1 << 0;
pub const GEMSLOT_EVENT_RX: u32 = 1 << 1;

/// Geometry of the reserved-memory slot pool, filled by `GEMSLOT_GET_INFO`.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SlotPoolInfo {
    pub pool_phys: u32,
    pub pool_size: u32,
    pub slot_size: u32,
    /// Slots per ring, not across both rings.
    pub slot_count: u32,
    pub tx_offset: u32,
    pub rx_offset: u32,
}

/// One entry of a transmit or receive batch: `len` excludes the Ethernet FCS.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BatchEntry {
    pub slot: u16,
    pub len: u16,
}

/// A batch of up to [`SLOT_BATCH_MAX`] entries in either direction.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct SlotBatch {
    pub count: u32,
    pub entries: [BatchEntry; SLOT_BATCH_MAX],
}

impl Default for SlotBatch {
    fn default() -> Self {
        Self {
            count: 0,
            entries: [BatchEntry::default(); SLOT_BATCH_MAX],
        }
    }
}

/// `GEMSLOT_RX_STEER` argument: which UDP destination port to divert.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SteerRequest {
    pub udp_port: u16,
    pub enable: u16,
}

/// `GEMSLOT_WAIT` argument. The driver fills `tx_pending` and `rx_pending` on
/// return.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct WaitArgs {
    pub events: u32,
    pub timeout_ms: u32,
    pub tx_threshold: u32,
    pub rx_threshold: u32,
    pub tx_pending: u32,
    pub rx_pending: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct KernelStreamConfig {
    pub direction: u32,
    pub instance: u32,
    pub ctrl: u32,
    pub produced: u32,
    pub observed: u32,
    pub released: u32,
    pub irq_threshold: u32,
    pub submit: u32,
    pub consumed: u32,
    pub consumed_count: u32,
    pub tx_irq_threshold: u32,
    pub slot_first: u32,
    pub slot_count: u32,
    pub frame_len: u32,
    pub udp_port: u32,
    pub format: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct KernelStreamStatus {
    pub direction: u32,
    pub active: u32,
    pub produced: u64,
    pub submitted: u64,
    pub completed: u64,
    pub consumed_ticks: u64,
    pub dropped: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_codes_match_the_kernel_ioctl_encoding() {
        // _IOR('G', 0, struct gemslot_info) with a 24-byte struct.
        assert_eq!(size_of::<SlotPoolInfo>(), 24);
        assert_eq!(GEMSLOT_GET_INFO, 0x8018_4700);
        // _IOW('G', 1, struct gemslot_tx_batch), 4 + 64 * 4 bytes.
        assert_eq!(size_of::<SlotBatch>(), 260);
        assert_eq!(GEMSLOT_TX_SUBMIT, 0x4104_4701);
        assert_eq!(GEMSLOT_RX_STEER, 0x4004_4703);
        assert_eq!(size_of::<WaitArgs>(), 24);
        assert_eq!(GEMSLOT_WAIT, 0xc018_4706);
        assert_eq!(size_of::<KernelStreamConfig>(), 64);
        assert_eq!(size_of::<KernelStreamStatus>(), 48);
        assert_eq!(GEMSLOT_STREAM_START, 0x4040_4707);
        assert_eq!(GEMSLOT_STREAM_STOP, 0x4004_4708);
        assert_eq!(GEMSLOT_STREAM_STATUS, 0xc030_4709);
        assert_eq!(GEMSLOT_GET_CAPS, 0x8004_470a);
    }
}
