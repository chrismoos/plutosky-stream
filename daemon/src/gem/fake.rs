//! In-memory slot rings for `--fake` and for tests.
//!
//! The fake keeps the same ownership discipline as the driver: a submitted
//! transmit slot is held until the test says GEM finished with it, and a
//! steered receive slot is held until the daemon returns it.

use std::collections::{BTreeSet, VecDeque};
use std::io;

use parking_lot::Mutex;

use super::ioctl::SlotPoolInfo;
use super::{
    KERNEL_STREAM_TX, KernelStreamConfig, KernelStreamStatus, Pending, Ring, SLOT_BATCH_MAX,
    SlotEntry, Slots, WaitRequest,
};

/// Slot geometry of the shipped pool. One slot holds a frame of up to 9000
/// bytes.
pub const DEFAULT_SLOT_SIZE: u32 = 9728;
/// Slots per ring in the documented pool.
pub const DEFAULT_SLOT_COUNT: u32 = 2048;
/// Physical address the fake reports for its pool. Nothing reads memory there.
const FAKE_POOL_PHYS: u32 = 0x1e00_0000;

#[derive(Default)]
struct Rings {
    /// Submitted and not yet reported complete: owned by the driver.
    tx_in_flight: VecDeque<SlotEntry>,
    /// Sent, waiting for the daemon to collect the completion.
    tx_done: VecDeque<SlotEntry>,
    /// Steered frames waiting for the daemon to poll them.
    rx_steered: VecDeque<SlotEntry>,
    /// Slots the daemon has given back for refill, in the order returned.
    rx_returned: Vec<SlotEntry>,
    steered_ports: BTreeSet<u16>,
    fail_next_steer_disable: bool,
    /// How many more entries `tx_submit` will accept, or `None` for no limit.
    tx_capacity: Option<usize>,
    /// Report every submitted slot as sent straight away, as GEM would.
    auto_complete: bool,
    kernel_mode: bool,
    kernel_configs: [Option<KernelStreamConfig>; 2],
    kernel_status: [KernelStreamStatus; 2],
}

pub struct FakeSlotDevice {
    info: SlotPoolInfo,
    pool: Mutex<Vec<u8>>,
    rings: Mutex<Rings>,
}

impl FakeSlotDevice {
    pub fn new(slot_size: u32, slot_count: u32) -> Self {
        let ring_bytes = slot_size * slot_count;
        let info = SlotPoolInfo {
            pool_phys: FAKE_POOL_PHYS,
            pool_size: ring_bytes * 2,
            slot_size,
            slot_count,
            tx_offset: 0,
            rx_offset: ring_bytes,
        };
        Self {
            info,
            pool: Mutex::new(vec![0; info.pool_size as usize]),
            rings: Mutex::new(Rings::default()),
        }
    }

    /// Reports a submitted slot as sent without waiting for
    /// [`Self::complete_transmits`], which is what `--fake` wants and what a
    /// test watching the hand-off does not.
    pub fn set_auto_complete(&self, auto_complete: bool) {
        self.rings.lock().auto_complete = auto_complete;
    }

    pub fn set_kernel_mode(&self, enabled: bool) {
        self.rings.lock().kernel_mode = enabled;
    }

    pub fn kernel_config(&self, direction: u32) -> Option<KernelStreamConfig> {
        self.rings
            .lock()
            .kernel_configs
            .get(direction as usize)
            .copied()
            .flatten()
    }

    pub fn set_kernel_status(&self, status: KernelStreamStatus) {
        if let Some(slot) = self
            .rings
            .lock()
            .kernel_status
            .get_mut(status.direction as usize)
        {
            *slot = status;
        }
    }

    /// Reports every in-flight transmit slot as sent, as GEM's completion
    /// interrupt would.
    pub fn complete_transmits(&self) -> usize {
        let mut rings = self.rings.lock();
        let moved = rings.tx_in_flight.len();
        while let Some(entry) = rings.tx_in_flight.pop_front() {
            rings.tx_done.push_back(entry);
        }
        moved
    }

    /// Delivers a steered frame to the daemon, as a host packet on a steered
    /// port would.
    pub fn deliver_steered(&self, entry: SlotEntry) {
        self.rings.lock().rx_steered.push_back(entry);
    }

    /// Slots handed back for refill, oldest first.
    pub fn returned_slots(&self) -> Vec<SlotEntry> {
        self.rings.lock().rx_returned.clone()
    }

    /// Transmit slots the driver still owns.
    pub fn transmits_in_flight(&self) -> Vec<SlotEntry> {
        self.rings.lock().tx_in_flight.iter().copied().collect()
    }

    /// UDP ports steering is enabled for.
    pub fn steered_ports(&self) -> Vec<u16> {
        self.rings.lock().steered_ports.iter().copied().collect()
    }

    pub fn fail_next_steer_disable(&self) {
        self.rings.lock().fail_next_steer_disable = true;
    }

    /// Caps how many entries the next submits accept, to exercise the pump's
    /// partial-accept path.
    pub fn set_transmit_capacity(&self, entries: Option<usize>) {
        self.rings.lock().tx_capacity = entries;
    }

    fn span(&self, ring: Ring, index: u16, offset: usize, len: usize) -> io::Result<usize> {
        let base = self.info.slot_offset(ring, index).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("slot {index} is outside the {ring:?} ring"),
            )
        })?;
        if offset + len > self.info.slot_size as usize {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "slot {index} span {offset}..{} is outside the slot",
                    offset + len
                ),
            ));
        }
        Ok(base + offset)
    }
}

impl Slots for FakeSlotDevice {
    fn info(&self) -> SlotPoolInfo {
        self.info
    }

    fn kernel_pump_available(&self) -> Option<bool> {
        self.rings.lock().kernel_mode.then_some(true)
    }

    fn stream_start(&self, config: KernelStreamConfig) -> io::Result<()> {
        let mut rings = self.rings.lock();
        if !rings.kernel_mode || config.direction > KERNEL_STREAM_TX {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "kernel stream pump",
            ));
        }
        let index = config.direction as usize;
        if rings.kernel_status[index].active != 0 {
            return Err(io::Error::new(
                io::ErrorKind::AlreadyExists,
                "stream active",
            ));
        }
        rings.kernel_configs[index] = Some(config);
        if config.direction == KERNEL_STREAM_TX {
            rings.steered_ports.insert(config.udp_port as u16);
        }
        rings.kernel_status[index] = KernelStreamStatus {
            direction: config.direction,
            active: 1,
            ..KernelStreamStatus::default()
        };
        Ok(())
    }

    fn stream_stop(&self, direction: u32) -> io::Result<()> {
        let mut rings = self.rings.lock();
        let Some(status) = rings.kernel_status.get_mut(direction as usize) else {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, "direction"));
        };
        status.active = 0;
        if direction == KERNEL_STREAM_TX
            && let Some(config) = rings.kernel_configs[direction as usize]
        {
            rings.steered_ports.remove(&(config.udp_port as u16));
        }
        Ok(())
    }

    fn stream_status(&self, direction: u32) -> io::Result<KernelStreamStatus> {
        self.rings
            .lock()
            .kernel_status
            .get(direction as usize)
            .copied()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "direction"))
    }

    fn tx_submit(&self, entries: &[SlotEntry]) -> io::Result<usize> {
        let mut rings = self.rings.lock();
        let mut accepted = entries.len().min(SLOT_BATCH_MAX);
        if let Some(capacity) = rings.tx_capacity.as_mut() {
            accepted = accepted.min(*capacity);
            *capacity -= accepted;
        }
        for entry in &entries[..accepted] {
            if rings.auto_complete {
                rings.tx_done.push_back(*entry);
            } else {
                rings.tx_in_flight.push_back(*entry);
            }
        }
        Ok(accepted)
    }

    fn tx_completions(&self, out: &mut Vec<SlotEntry>) -> io::Result<()> {
        let mut rings = self.rings.lock();
        for _ in 0..SLOT_BATCH_MAX {
            match rings.tx_done.pop_front() {
                Some(entry) => out.push(entry),
                None => break,
            }
        }
        Ok(())
    }

    fn rx_steer(&self, udp_port: u16, enable: bool) -> io::Result<()> {
        let mut rings = self.rings.lock();
        if !enable && rings.fail_next_steer_disable {
            rings.fail_next_steer_disable = false;
            return Err(io::Error::other("injected steering failure"));
        }
        if enable {
            rings.steered_ports.insert(udp_port);
        } else {
            rings.steered_ports.remove(&udp_port);
        }
        Ok(())
    }

    fn rx_poll(&self, out: &mut Vec<SlotEntry>) -> io::Result<()> {
        let mut rings = self.rings.lock();
        for _ in 0..SLOT_BATCH_MAX {
            match rings.rx_steered.pop_front() {
                Some(entry) => out.push(entry),
                None => break,
            }
        }
        Ok(())
    }

    fn rx_return(&self, entries: &[SlotEntry]) -> io::Result<()> {
        self.rings.lock().rx_returned.extend_from_slice(entries);
        Ok(())
    }

    fn wait(&self, _request: WaitRequest) -> io::Result<Pending> {
        let rings = self.rings.lock();
        Ok(Pending {
            tx_completions: rings.tx_done.len() as u32,
            rx_frames: rings.rx_steered.len() as u32,
        })
    }

    fn read_slot(&self, ring: Ring, index: u16, offset: usize, out: &mut [u8]) -> io::Result<()> {
        let at = self.span(ring, index, offset, out.len())?;
        out.copy_from_slice(&self.pool.lock()[at..at + out.len()]);
        Ok(())
    }

    fn write_slot(&self, ring: Ring, index: u16, offset: usize, bytes: &[u8]) -> io::Result<()> {
        let at = self.span(ring, index, offset, bytes.len())?;
        self.pool.lock()[at..at + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SLOT_SIZE: u32 = 256;
    const SLOT_COUNT: u32 = 8;

    #[test]
    fn slot_addresses_follow_the_ring_layout() {
        let device = FakeSlotDevice::new(SLOT_SIZE, SLOT_COUNT);
        let info = device.info();
        assert_eq!(info.slot_offset(Ring::Tx, 3), Some(3 * SLOT_SIZE as usize));
        assert_eq!(
            info.slot_offset(Ring::Rx, 0),
            Some((SLOT_SIZE * SLOT_COUNT) as usize)
        );
        assert_eq!(info.slot_offset(Ring::Tx, SLOT_COUNT as u16), None);
        assert_eq!(
            info.slot_phys(Ring::Rx, 1),
            Some(info.pool_phys + SLOT_SIZE * SLOT_COUNT + SLOT_SIZE)
        );
    }

    #[test]
    fn submitted_slots_stay_with_the_driver_until_completed() {
        let device = FakeSlotDevice::new(SLOT_SIZE, SLOT_COUNT);
        let entries = [
            SlotEntry { slot: 0, len: 64 },
            SlotEntry { slot: 1, len: 64 },
        ];
        assert_eq!(device.tx_submit(&entries).unwrap(), 2);
        let mut done = Vec::new();
        device.tx_completions(&mut done).unwrap();
        assert!(done.is_empty(), "nothing completes before GEM reports it");
        assert_eq!(device.complete_transmits(), 2);
        device.tx_completions(&mut done).unwrap();
        assert_eq!(done, entries);
    }

    #[test]
    fn transmit_capacity_limits_what_a_submit_accepts() {
        let device = FakeSlotDevice::new(SLOT_SIZE, SLOT_COUNT);
        device.set_transmit_capacity(Some(1));
        let entries = [SlotEntry { slot: 4, len: 8 }, SlotEntry { slot: 5, len: 8 }];
        assert_eq!(device.tx_submit(&entries).unwrap(), 1);
        assert_eq!(device.transmits_in_flight(), vec![entries[0]]);
        assert_eq!(device.tx_submit(&entries[1..]).unwrap(), 0);
    }

    #[test]
    fn slot_reads_and_writes_are_bounded_by_the_slot() {
        let device = FakeSlotDevice::new(SLOT_SIZE, SLOT_COUNT);
        device.write_slot(Ring::Rx, 2, 2, &[1, 2, 3, 4]).unwrap();
        let mut out = [0u8; 4];
        device.read_slot(Ring::Rx, 2, 2, &mut out).unwrap();
        assert_eq!(out, [1, 2, 3, 4]);
        let mut spill = [0u8; 4];
        assert!(
            device
                .read_slot(Ring::Rx, 2, SLOT_SIZE as usize - 2, &mut spill)
                .is_err()
        );
    }
}
