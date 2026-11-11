//! The GEM slot fast path: the `/dev/gemslot0` character device that the
//! patched `macb` driver exposes, and an in-memory stand-in for tests.
//!
//! The pool holds two rings of equal-sized slots in physically contiguous,
//! uncached memory. Slot `i` of a ring starts at
//! `pool_phys + ring_offset + i * slot_size`, and a frame starts at byte
//! offset [`SLOT_FRAME_OFFSET`] inside the slot so the IP header and the VITA
//! payload land 4-byte aligned.
//!
//! Ownership moves one way around each ring. A TX slot goes PL -> driver
//! (`tx_submit`) -> daemon (`tx_completions`) -> PL (released through the
//! packetizer CSR). An RX slot goes driver -> daemon (`rx_poll`) -> PL
//! (depacketizer submit FIFO) -> driver (`rx_return`).

mod device;
mod fake;
mod ioctl;

use std::io;
use std::time::Duration;

pub use device::SlotDevice;
pub use fake::{DEFAULT_SLOT_COUNT, DEFAULT_SLOT_SIZE, FakeSlotDevice};
pub use ioctl::{
    GEMSLOT_EVENT_RX, GEMSLOT_EVENT_TX, KernelStreamConfig, KernelStreamStatus, SlotPoolInfo,
};

/// Default character device the patched `macb` driver creates for GEM0.
pub const DEFAULT_GEMSLOT_PATH: &str = "/dev/gemslot0";

/// Frames start this far into a slot so that the IPv4 header and the VITA
/// payload are 4-byte aligned once the 14-byte Ethernet header is in place.
/// GEM's receive buffer offset is programmed to the same value.
pub const SLOT_FRAME_OFFSET: usize = 2;

/// Entries the driver moves in one ioctl.
pub const SLOT_BATCH_MAX: usize = 64;
pub const KERNEL_STREAM_RX: u32 = 0;
pub const KERNEL_STREAM_TX: u32 = 1;

/// Which of the pool's two rings a slot index refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ring {
    /// Frames the board sends: the packetizer fills them, GEM transmits them.
    Tx,
    /// Frames the board receives: GEM fills them, the depacketizer reads them.
    Rx,
}

/// One slot and the frame length in it, excluding the Ethernet FCS.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotEntry {
    pub slot: u16,
    pub len: u16,
}

/// Counts the driver reports from a [`Slots::wait`] call.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Pending {
    pub tx_completions: u32,
    pub rx_frames: u32,
}

/// What a wait call is watching for and how long it may block.
#[derive(Clone, Copy, Debug)]
pub struct WaitRequest {
    pub events: u32,
    pub timeout: Duration,
    pub tx_threshold: u32,
    pub rx_threshold: u32,
}

/// The slot ring interface the stream pump drives.
pub trait Slots: Send + Sync {
    fn info(&self) -> SlotPoolInfo;

    fn kernel_pump_available(&self) -> Option<bool> {
        None
    }

    fn stream_start(&self, _config: KernelStreamConfig) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "kernel stream pump",
        ))
    }

    fn stream_stop(&self, _direction: u32) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "kernel stream pump",
        ))
    }

    fn stream_status(&self, _direction: u32) -> io::Result<KernelStreamStatus> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "kernel stream pump",
        ))
    }

    /// Queues frames for transmission. Returns how many the driver accepted,
    /// which may be fewer than offered when its descriptor ring is full.
    fn tx_submit(&self, entries: &[SlotEntry]) -> io::Result<usize>;

    /// Collects slots GEM has finished sending.
    fn tx_completions(&self, out: &mut Vec<SlotEntry>) -> io::Result<()>;

    /// Enables or disables steering of frames whose UDP destination port
    /// matches.
    fn rx_steer(&self, udp_port: u16, enable: bool) -> io::Result<()>;

    /// Collects steered frames the driver has taken out of the receive ring.
    fn rx_poll(&self, out: &mut Vec<SlotEntry>) -> io::Result<()>;

    /// Gives steered slots back to the driver for refill.
    fn rx_return(&self, entries: &[SlotEntry]) -> io::Result<()>;

    /// Blocks until a threshold is met or the timeout passes.
    fn wait(&self, request: WaitRequest) -> io::Result<Pending>;

    /// Copies out of a slot, for debugging and for the host-side fake.
    fn read_slot(&self, ring: Ring, index: u16, offset: usize, out: &mut [u8]) -> io::Result<()>;

    /// Copies into a slot, for debugging and for the host-side fake.
    fn write_slot(&self, ring: Ring, index: u16, offset: usize, bytes: &[u8]) -> io::Result<()>;
}

impl SlotPoolInfo {
    /// Byte offset of `index` in `ring` from the start of the pool.
    pub fn slot_offset(&self, ring: Ring, index: u16) -> Option<usize> {
        if u32::from(index) >= self.slot_count {
            return None;
        }
        let ring_offset = match ring {
            Ring::Tx => self.tx_offset,
            Ring::Rx => self.rx_offset,
        };
        Some((ring_offset + u32::from(index) * self.slot_size) as usize)
    }

    /// Physical address of `index` in `ring`, the value the PL is programmed
    /// with.
    pub fn slot_phys(&self, ring: Ring, index: u16) -> Option<u32> {
        self.slot_offset(ring, index)
            .map(|off| self.pool_phys + off as u32)
    }
}
