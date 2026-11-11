//! The slot pumps: the daemon's half of the hand-off between the PL and the
//! GEM driver.
//!
//! Board to host, [`PacketizerPump`] takes the slots the packetizer reports as
//! produced, submits them for transmission, and tells the packetizer a slot is
//! free again once GEM reports the frame sent. Host to board,
//! [`DepacketizerPump`] takes the frames the driver steered, hands their slot
//! indices to the depacketizer, and returns each slot for refill once the
//! depacketizer has read it.
//!
//! Both are driven one step at a time so the ownership transitions can be
//! tested without a thread or a clock.

use std::collections::VecDeque;
use std::io;
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::csr::CsrBus;
use crate::gem::{
    GEMSLOT_EVENT_TX, Ring, SLOT_BATCH_MAX, SLOT_FRAME_OFFSET, SlotEntry, Slots, WaitRequest,
};
use crate::time::Timebase;

use super::format::{VITA_HEADER_BYTES, WireFormat};
use super::header::{VITA_HEADER_AT, parse_vita_data_header};
use super::regs::{PacketizerCtrl, RxPacketizerRegs, SUBMIT_FIFO_DEPTH, TxDepacketizerRegs};

/// The counters a `status` event reports for one stream.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StreamCounters {
    pub produced: u64,
    pub consumed: u64,
    pub late: u64,
    pub underrun: u64,
    pub overflow: u64,
    pub gaps: u64,
}

/// How long a stopping stream waits for GEM to finish its in-flight frames.
const FINISH_TIMEOUT: Duration = Duration::from_millis(200);
const FINISH_POLL: Duration = Duration::from_millis(10);

/// Extends a free-running 32-bit CSR counter across its wrap.
#[derive(Clone, Copy, Debug, Default)]
struct WideCounter {
    value: u64,
    last: u32,
}

impl WideCounter {
    fn update(&mut self, raw: u32) -> u64 {
        self.value += u64::from(raw.wrapping_sub(self.last));
        self.last = raw;
        self.value
    }

    fn value(self) -> u64 {
        self.value
    }
}

/// Board to host. Owns the transmit ring slice the packetizer writes into.
pub struct PacketizerPump {
    regs: RxPacketizerRegs,
    slots: Arc<dyn Slots>,
    slot_first: u16,
    slot_count: u16,
    frame_len: u16,
    /// Sequence number of the next produced slot not yet queued for the
    /// driver. Slot indices repeat through the stream's range in this order.
    queued: u64,
    produced: WideCounter,
    overflow: WideCounter,
    /// Slots the daemon has reported free to the packetizer, which counts
    /// both the frames GEM sent and any the ring lost before submission.
    released: u64,
    /// Frames GEM actually sent, which is what the stream reports consumed.
    sent: u64,
    pending: VecDeque<SlotEntry>,
    batch: Vec<SlotEntry>,
    completions: Vec<SlotEntry>,
    passes: u64,
    submitted: u64,
    lost: u64,
    /// The hardware's produced count when this stream started. The counters
    /// keep running across stream restarts while the packetizer rewinds to
    /// its first slot, so sequence numbers and the released count are kept
    /// relative to it.
    base: u32,
}

impl PacketizerPump {
    pub fn new(
        regs: RxPacketizerRegs,
        slots: Arc<dyn Slots>,
        slot_first: u16,
        slot_count: u16,
        frame_len: u16,
    ) -> Self {
        Self {
            regs,
            slots,
            slot_first,
            slot_count,
            frame_len,
            queued: 0,
            produced: WideCounter::default(),
            overflow: WideCounter::default(),
            released: 0,
            sent: 0,
            pending: VecDeque::new(),
            batch: Vec::with_capacity(SLOT_BATCH_MAX),
            completions: Vec::new(),
            passes: 0,
            submitted: 0,
            lost: 0,
            base: 0,
        }
    }

    /// Reads the hardware counters before the packetizer is enabled and
    /// hands every slot back, so the stream starts with an empty ring and
    /// sequence 0 in its first slot.
    pub fn prime(&mut self, bus: &dyn CsrBus) -> io::Result<()> {
        // Completions still queued in the driver belong to whatever ran in
        // these slots before and must not count against this stream.
        loop {
            self.completions.clear();
            self.slots.tx_completions(&mut self.completions)?;
            if self.completions.is_empty() {
                break;
            }
        }
        let produced = self.regs.produced.read(bus) as u32;
        let overflow = self.regs.overflow.read(bus) as u32;
        self.base = produced;
        self.produced = WideCounter {
            value: 0,
            last: produced,
        };
        self.overflow = WideCounter {
            value: 0,
            last: overflow,
        };
        self.regs.released.write(bus, u64::from(produced));
        Ok(())
    }

    /// Collects the completions of frames still in flight when the stream
    /// stopped, so the next stream in these slots starts from a clean driver
    /// queue and the sent count is final.
    pub fn finish(&mut self, bus: &dyn CsrBus) {
        let deadline = Instant::now() + FINISH_TIMEOUT;
        while self.sent < self.submitted && Instant::now() < deadline {
            let _ = self.slots.wait(WaitRequest {
                events: GEMSLOT_EVENT_TX,
                timeout: FINISH_POLL,
                tx_threshold: 1,
                rx_threshold: 0,
            });
            self.completions.clear();
            if self.slots.tx_completions(&mut self.completions).is_err() {
                break;
            }
            if !self.completions.is_empty() {
                self.sent += self.completions.len() as u64;
                self.released += self.completions.len() as u64;
                self.write_released(bus);
            }
        }
    }

    fn write_released(&self, bus: &dyn CsrBus) {
        self.regs
            .released
            .write(bus, u64::from(self.base) + self.released);
    }

    /// Slot index the packetizer writes for sequence number `sequence`.
    fn slot_for(&self, sequence: u64) -> u16 {
        self.slot_first + (sequence % u64::from(self.slot_count)) as u16
    }

    /// One pass: collect newly produced slots, offer them to the driver, and
    /// release the ones GEM has finished with.
    pub fn step(&mut self, bus: &dyn CsrBus) -> io::Result<()> {
        self.passes += 1;
        let produced = self.produced.update(self.regs.produced.read(bus) as u32);
        self.overflow.update(self.regs.overflow.read(bus) as u32);
        let capacity = u64::from(self.slot_count);
        // The packetizer may write a slot only while produced - released is
        // under the ring depth, so anything further ahead than that was
        // overwritten before the daemon ever saw it. Account for those slots
        // and continue from the freshest one rather than falling behind.
        if produced > self.queued + capacity {
            let lost = produced - capacity - self.queued;
            log::warn!(
                "packetizer overran its {} slots: {lost} frames lost before submission",
                self.slot_count
            );
            self.queued += lost;
            self.released += lost;
            self.lost += lost;
            self.write_released(bus);
        }
        while self.queued < produced && self.queued - self.released < capacity {
            self.pending.push_back(SlotEntry {
                slot: self.slot_for(self.queued),
                len: self.frame_len,
            });
            self.queued += 1;
        }

        while !self.pending.is_empty() {
            self.batch.clear();
            self.batch
                .extend(self.pending.iter().take(SLOT_BATCH_MAX).copied());
            let accepted = self.slots.tx_submit(&self.batch)?;
            self.submitted += accepted as u64;
            self.pending.drain(..accepted);
            // The driver's descriptor ring is full: the rest waits for the
            // next pass rather than spinning here.
            if accepted < self.batch.len() {
                break;
            }
        }

        self.completions.clear();
        self.slots.tx_completions(&mut self.completions)?;
        if !self.completions.is_empty() {
            self.sent += self.completions.len() as u64;
            self.released += self.completions.len() as u64;
            self.write_released(bus);
        }
        Ok(())
    }

    pub fn counters(&self) -> StreamCounters {
        StreamCounters {
            produced: self.produced.value(),
            consumed: self.sent,
            overflow: self.overflow.value(),
            ..StreamCounters::default()
        }
    }

    /// One line of pump accounting for the log when a stream stops.
    pub fn summary(&self) -> String {
        format!(
            "passes={} produced={} submitted={} sent={} released={} lost={} overflow={} pending={}",
            self.passes,
            self.produced.value(),
            self.submitted,
            self.sent,
            self.released,
            self.lost,
            self.overflow.value(),
            self.pending.len()
        )
    }
}

/// A start the depacketizer cannot arm itself. The contract gives it no start
/// registers, so the pump enables it when the clock reaches the tick instead,
/// which keeps the control session free while the start is still in the
/// future.
struct PendingStart {
    at_ticks: u64,
    timebase: Timebase,
    ctrl: PacketizerCtrl,
}

/// Host to board. Owns the receive slots the driver steered to this stream.
pub struct DepacketizerPump {
    regs: TxDepacketizerRegs,
    slots: Arc<dyn Slots>,
    /// Frame length per receive slot, kept so a returned slot reports the
    /// length the driver gave the daemon.
    lengths: Vec<u16>,
    holding: VecDeque<SlotEntry>,
    /// Slot indices handed to the depacketizer and not yet reported consumed,
    /// so a stop can return them instead of leaking them from the free list.
    submitted_slots: VecDeque<u16>,
    format: WireFormat,
    /// Sample count just past each steered slot's last sample, read from its
    /// VITA header when the slot is submitted.
    end_ticks: Vec<u64>,
    /// Slots handed to the depacketizer and not yet reported consumed.
    outstanding: usize,
    submitted: u64,
    consumed_packets: u64,
    /// The stream's `consumed` counter: the end sample count of the latest
    /// packet the depacketizer has finished, which is what the host's transmit
    /// window and burst acknowledgements are measured against.
    consumed_ticks: u64,
    late: WideCounter,
    underrun: WideCounter,
    gaps: WideCounter,
    bad_id: WideCounter,
    steered: Vec<SlotEntry>,
    returns: Vec<SlotEntry>,
    pending_start: Option<PendingStart>,
}

impl DepacketizerPump {
    pub fn new(regs: TxDepacketizerRegs, slots: Arc<dyn Slots>, format: WireFormat) -> Self {
        let ring_slots = slots.info().slot_count as usize;
        Self {
            regs,
            slots,
            lengths: vec![0; ring_slots],
            holding: VecDeque::new(),
            submitted_slots: VecDeque::new(),
            format,
            end_ticks: vec![0; ring_slots],
            outstanding: 0,
            submitted: 0,
            consumed_packets: 0,
            consumed_ticks: 0,
            late: WideCounter::default(),
            underrun: WideCounter::default(),
            gaps: WideCounter::default(),
            bad_id: WideCounter::default(),
            steered: Vec::new(),
            returns: Vec::new(),
            pending_start: None,
        }
    }

    /// Holds the core stopped until the clock reaches `at_ticks`, then enables
    /// it with `ctrl`.
    pub fn start_at(mut self, at_ticks: u64, timebase: Timebase, ctrl: PacketizerCtrl) -> Self {
        self.pending_start = Some(PendingStart {
            at_ticks,
            timebase,
            ctrl,
        });
        self
    }

    /// Discards completions left by an earlier run before any new slots are submitted.
    pub fn discard_stale_completions(&mut self, bus: &dyn CsrBus) -> usize {
        let ready = self.regs.consumed_count.read(bus) as usize;
        for _ in 0..ready {
            self.regs.read_consumed(bus);
        }
        ready
    }

    /// One pass: collect steered frames, fill the depacketizer's submit FIFO,
    /// and give back every slot it has finished reading.
    pub fn step(&mut self, bus: &dyn CsrBus) -> io::Result<()> {
        if let Some(start) = &self.pending_start {
            if start.timebase.now(bus) < start.at_ticks {
                // Frames that arrive before the start stay with the driver
                // rather than piling up here, since the core is stopped.
                return Ok(());
            }
            self.regs.set_ctrl(bus, start.ctrl);
            self.pending_start = None;
        }
        // Drain the whole steer ring each pass. rx_poll returns at most one
        // batch, so a single call falls behind a host that buffers ahead and
        // the driver's steer ring fills, starving the receive free list. Loop
        // until the ring is empty, moving frames into `holding`.
        loop {
            self.steered.clear();
            self.slots.rx_poll(&mut self.steered)?;
            if self.steered.is_empty() {
                break;
            }
            for entry in self.steered.drain(..) {
                self.holding.push_back(entry);
            }
        }

        while self.outstanding < SUBMIT_FIFO_DEPTH {
            let Some(entry) = self.holding.pop_front() else {
                break;
            };
            let index = usize::from(entry.slot);
            match self.lengths.get_mut(index) {
                Some(slot) => *slot = entry.len,
                None => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("driver steered slot {} outside the ring", entry.slot),
                    ));
                }
            }
            self.end_ticks[index] = self.packet_end_ticks(entry.slot);
            self.regs.submit_slot(bus, entry.slot, entry.len);
            self.submitted_slots.push_back(entry.slot);
            self.outstanding += 1;
            self.submitted += 1;
        }

        let ready = self.regs.consumed_count.read(bus) as usize;
        self.returns.clear();
        // The core processes one submitted slot at a time, so its completion
        // count retires slots in submission order even if the crossed ID is stale.
        for _ in 0..ready.min(self.outstanding) {
            let reported_slot = self.regs.read_consumed(bus);
            let Some(slot) = self.submitted_slots.pop_front() else {
                break;
            };
            if reported_slot != slot {
                log::debug!(
                    "depacketizer reported consumed slot {reported_slot}, expected ordered slot {slot}"
                );
            }
            let len = self
                .lengths
                .get(usize::from(slot))
                .copied()
                .unwrap_or_default();
            self.returns.push(SlotEntry { slot, len });
            self.outstanding -= 1;
            self.consumed_packets += 1;
            if let Some(end) = self.end_ticks.get(usize::from(slot)) {
                self.consumed_ticks = self.consumed_ticks.max(*end);
            }
        }
        for batch in self.returns.chunks(SLOT_BATCH_MAX) {
            if let Err(error) = self.slots.rx_return(batch) {
                if error.kind() != io::ErrorKind::InvalidInput {
                    return Err(error);
                }
                for entry in batch {
                    if let Err(error) = self.slots.rx_return(std::slice::from_ref(entry))
                        && error.kind() != io::ErrorKind::InvalidInput
                    {
                        return Err(error);
                    }
                }
            }
        }

        self.late.update(self.regs.late.read(bus) as u32);
        self.underrun.update(self.regs.underrun.read(bus) as u32);
        self.gaps.update(self.regs.gaps.read(bus) as u32);
        self.bad_id.update(self.regs.bad_id.read(bus) as u32);
        Ok(())
    }

    /// Returns every receive slot this pump is responsible for when the stream
    /// stops: the driver's steer ring, the slots waiting in `holding`, and the
    /// ones handed to the now-stopped depacketizer. Without this those slots
    /// never go back to the free list, which starves the receive path over
    /// repeated transmit streams.
    pub fn drain(&mut self, bus: &dyn CsrBus) -> io::Result<()> {
        let ready = self.regs.consumed_count.read(bus) as usize;
        for _ in 0..ready.min(self.outstanding) {
            let _reported_slot = self.regs.read_consumed(bus);
            let Some(slot) = self.submitted_slots.pop_front() else {
                break;
            };
            let len = self
                .lengths
                .get(usize::from(slot))
                .copied()
                .unwrap_or_default();
            self.holding.push_back(SlotEntry { slot, len });
            self.outstanding = self.outstanding.saturating_sub(1);
        }
        loop {
            self.steered.clear();
            self.slots.rx_poll(&mut self.steered)?;
            if self.steered.is_empty() {
                break;
            }
            for entry in self.steered.drain(..) {
                self.holding.push_back(entry);
            }
        }
        let mut all: Vec<SlotEntry> = self.holding.drain(..).collect();
        for slot in self.submitted_slots.drain(..) {
            let len = self
                .lengths
                .get(usize::from(slot))
                .copied()
                .unwrap_or_default();
            all.push(SlotEntry { slot, len });
        }
        self.outstanding = 0;
        for entry in all {
            if let Err(error) = self.slots.rx_return(std::slice::from_ref(&entry)) {
                if error.kind() != io::ErrorKind::InvalidInput {
                    return Err(error);
                }
                log::debug!("receive slot {} was already returned", entry.slot);
            }
        }
        Ok(())
    }

    /// End sample count of the packet in a steered slot, or the current
    /// consumed count when the slot does not hold a data packet of this
    /// profile, so a malformed frame never moves the counter backwards.
    fn packet_end_ticks(&self, slot: u16) -> u64 {
        let mut vita = [0u8; VITA_HEADER_BYTES];
        let read = self.slots.read_slot(
            Ring::Rx,
            slot,
            SLOT_FRAME_OFFSET + VITA_HEADER_AT,
            &mut vita,
        );
        match read
            .ok()
            .and_then(|_| parse_vita_data_header(&vita, self.format))
        {
            Some(header) => header.end_ticks(),
            None => self.consumed_ticks,
        }
    }

    /// Packets the depacketizer dropped because their stream id was not this
    /// stream's, reported separately from the protocol's counters.
    pub fn bad_stream_id(&self) -> u64 {
        self.bad_id.value()
    }

    /// Packets the depacketizer has finished reading.
    pub fn consumed_packets(&self) -> u64 {
        self.consumed_packets
    }

    pub fn counters(&self) -> StreamCounters {
        StreamCounters {
            produced: self.submitted,
            consumed: self.consumed_ticks,
            late: self.late.value(),
            underrun: self.underrun.value(),
            overflow: 0,
            gaps: self.gaps.value(),
        }
    }
}

/// The pump for whichever direction a stream runs in.
pub enum SlotPump {
    /// Board to host.
    Packetizer(PacketizerPump),
    /// Host to board.
    Depacketizer(DepacketizerPump),
}

impl SlotPump {
    pub(super) fn needs_completion_poll(&self) -> bool {
        match self {
            Self::Packetizer(_) => false,
            Self::Depacketizer(pump) => pump.outstanding != 0 || !pump.holding.is_empty(),
        }
    }

    pub fn step(&mut self, bus: &dyn CsrBus) -> io::Result<()> {
        match self {
            SlotPump::Packetizer(pump) => pump.step(bus),
            SlotPump::Depacketizer(pump) => pump.step(bus),
        }
    }

    pub fn counters(&self) -> StreamCounters {
        match self {
            SlotPump::Packetizer(pump) => pump.counters(),
            SlotPump::Depacketizer(pump) => pump.counters(),
        }
    }

    pub fn summary(&self) -> String {
        match self {
            SlotPump::Packetizer(pump) => pump.summary(),
            SlotPump::Depacketizer(pump) => format!("{:?}", pump.counters()),
        }
    }

    /// Settles the driver queue after the stream stopped.
    pub fn finish(&mut self, bus: &dyn CsrBus) {
        match self {
            SlotPump::Packetizer(pump) => pump.finish(bus),
            SlotPump::Depacketizer(pump) => {
                if let Err(error) = pump.drain(bus) {
                    log::warn!("depacketizer drain on stop failed: {error}");
                }
            }
        }
    }

    /// The ring whose slots this pump moves.
    pub fn ring(&self) -> Ring {
        match self {
            SlotPump::Packetizer(_) => Ring::Tx,
            SlotPump::Depacketizer(_) => Ring::Rx,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;
    use crate::gem::FakeSlotDevice;
    use crate::stream::format::WireFormat;
    use crate::stream::regs::ChannelMask;
    use crate::testing::fixture_csr_map;

    const SLOT_SIZE: u32 = 2048;
    const RING_SLOTS: u32 = 16;
    const STREAM_SLOTS: u16 = 4;
    const SLOT_FIRST: u16 = 8;
    const FRAME_LEN: u16 = 1510;

    fn packetizer() -> (
        PacketizerPump,
        Arc<FakeSlotDevice>,
        FakeBus,
        RxPacketizerRegs,
    ) {
        let map = fixture_csr_map();
        let regs = RxPacketizerRegs::resolve(&map, 0).unwrap();
        let device = Arc::new(FakeSlotDevice::new(SLOT_SIZE, RING_SLOTS));
        let pump = PacketizerPump::new(
            regs,
            device.clone() as Arc<dyn Slots>,
            SLOT_FIRST,
            STREAM_SLOTS,
            FRAME_LEN,
        );
        (pump, device, FakeBus::new(), regs)
    }

    #[test]
    fn produced_slots_reach_the_driver_and_are_released_only_after_completion() {
        let (mut pump, device, bus, regs) = packetizer();
        bus.set(regs.produced.addr, 3);
        pump.step(&bus).unwrap();
        assert_eq!(
            device.transmits_in_flight(),
            vec![
                SlotEntry {
                    slot: 8,
                    len: FRAME_LEN
                },
                SlotEntry {
                    slot: 9,
                    len: FRAME_LEN
                },
                SlotEntry {
                    slot: 10,
                    len: FRAME_LEN
                },
            ]
        );
        assert!(
            bus.writes_to(regs.released.addr).is_empty(),
            "nothing is released before GEM reports the frames sent"
        );

        device.complete_transmits();
        pump.step(&bus).unwrap();
        assert_eq!(bus.writes_to(regs.released.addr), vec![3]);
        assert_eq!(pump.counters().produced, 3);
        assert_eq!(pump.counters().consumed, 3);
    }

    #[test]
    fn slot_indices_wrap_inside_the_streams_own_range() {
        let (mut pump, device, bus, regs) = packetizer();
        bus.set(regs.produced.addr, u32::from(STREAM_SLOTS));
        pump.step(&bus).unwrap();
        // The packetizer may not write past the ring depth until the daemon
        // has released those slots, so complete them before the next two.
        device.complete_transmits();
        pump.step(&bus).unwrap();
        bus.set(regs.produced.addr, u32::from(STREAM_SLOTS) + 2);
        pump.step(&bus).unwrap();
        let submitted: Vec<u16> = device
            .transmits_in_flight()
            .iter()
            .map(|entry| entry.slot)
            .collect();
        assert_eq!(submitted, vec![8, 9]);
        assert_eq!(pump.counters().produced, u64::from(STREAM_SLOTS) + 2);
    }

    #[test]
    fn a_full_driver_ring_leaves_the_rest_pending_for_the_next_pass() {
        let (mut pump, device, bus, regs) = packetizer();
        device.set_transmit_capacity(Some(2));
        bus.set(regs.produced.addr, 4);
        pump.step(&bus).unwrap();
        assert_eq!(device.transmits_in_flight().len(), 2);

        device.set_transmit_capacity(None);
        pump.step(&bus).unwrap();
        let submitted: Vec<u16> = device
            .transmits_in_flight()
            .iter()
            .map(|entry| entry.slot)
            .collect();
        assert_eq!(submitted, vec![8, 9, 10, 11]);
    }

    #[test]
    fn a_wrapped_produced_counter_keeps_counting_up() {
        let (mut pump, device, bus, regs) = packetizer();
        bus.set(regs.produced.addr, u32::MAX);
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().produced, u64::from(u32::MAX));
        device.complete_transmits();
        bus.set(regs.produced.addr, 1);
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().produced, u64::from(u32::MAX) + 2);
    }

    #[test]
    fn a_packetizer_that_ran_past_the_ring_depth_is_accounted_for_not_chased() {
        let (mut pump, device, bus, regs) = packetizer();
        let overrun = 1_000_000;
        bus.set(regs.produced.addr, overrun);
        pump.step(&bus).unwrap();
        assert_eq!(
            device.transmits_in_flight().len(),
            usize::from(STREAM_SLOTS),
            "only a ring's worth of slots is ever in flight"
        );
        let released = bus.writes_to(regs.released.addr);
        assert_eq!(
            released.first().copied(),
            Some(overrun - u32::from(STREAM_SLOTS)),
            "the lost frames are reported released so the packetizer can carry on"
        );
        assert_eq!(
            pump.counters().consumed,
            0,
            "a frame lost before submission was never sent"
        );
        device.complete_transmits();
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().consumed, u64::from(STREAM_SLOTS));
    }

    #[test]
    fn the_overflow_counter_is_reported_without_being_reset() {
        let (mut pump, _device, bus, regs) = packetizer();
        bus.set(regs.overflow.addr, 5);
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().overflow, 5);
        assert!(bus.writes_to(regs.overflow.addr).is_empty());
    }

    fn depacketizer() -> (
        DepacketizerPump,
        Arc<FakeSlotDevice>,
        FakeBus,
        TxDepacketizerRegs,
    ) {
        let map = fixture_csr_map();
        let regs = TxDepacketizerRegs::resolve(&map, 1).unwrap();
        let device = Arc::new(FakeSlotDevice::new(SLOT_SIZE, RING_SLOTS));
        let pump = DepacketizerPump::new(regs, device.clone() as Arc<dyn Slots>, WireFormat::Cs12);
        (pump, device, FakeBus::new(), regs)
    }

    #[test]
    fn steered_slots_reach_the_depacketizer_and_go_back_only_once_consumed() {
        let (mut pump, device, bus, regs) = depacketizer();
        device.deliver_steered(SlotEntry { slot: 2, len: 600 });
        device.deliver_steered(SlotEntry { slot: 5, len: 700 });
        pump.step(&bus).unwrap();
        assert_eq!(
            bus.writes_to(regs.submit.addr),
            vec![(600 << 16) | 2, (700 << 16) | 5]
        );
        assert!(
            device.returned_slots().is_empty(),
            "the PL still owns both slots"
        );

        bus.set(regs.consumed_count.addr, 2);
        bus.set(regs.consumed.addr, 2);
        let mut reads = 0;
        bus.set_read_hook(Box::new(move |addr, mem| {
            if addr == regs.consumed.addr {
                reads += 1;
                return Some(if reads == 1 { 2 } else { 5 });
            }
            let _ = mem;
            None
        }));
        pump.step(&bus).unwrap();
        assert_eq!(
            device.returned_slots(),
            vec![
                SlotEntry { slot: 2, len: 600 },
                SlotEntry { slot: 5, len: 700 },
            ]
        );
        assert_eq!(pump.counters().produced, 2);
        assert_eq!(pump.consumed_packets(), 2);
    }

    #[test]
    fn corrupted_completion_id_releases_the_oldest_submitted_slot() {
        let (mut pump, device, bus, regs) = depacketizer();
        device.deliver_steered(SlotEntry { slot: 2, len: 600 });
        pump.step(&bus).unwrap();

        bus.set(regs.consumed_count.addr, 1);
        bus.set(regs.consumed.addr, 5);
        pump.step(&bus).unwrap();
        assert_eq!(
            device.returned_slots(),
            vec![SlotEntry { slot: 2, len: 600 }]
        );
        assert_eq!(pump.consumed_packets(), 1);
    }

    #[test]
    fn tx_completions_are_polled_without_waiting_for_another_network_packet() {
        let (pump, device, bus, regs) = depacketizer();
        let mut pump = SlotPump::Depacketizer(pump);
        assert!(!pump.needs_completion_poll());
        device.deliver_steered(SlotEntry { slot: 2, len: 600 });
        pump.step(&bus).unwrap();
        assert!(pump.needs_completion_poll());
        bus.set(regs.consumed_count.addr, 1);
        bus.set(regs.consumed.addr, 2);
        pump.step(&bus).unwrap();
        assert!(!pump.needs_completion_poll());
    }

    #[test]
    fn stale_completions_are_discarded_before_submission() {
        let (mut pump, _device, bus, regs) = depacketizer();
        bus.set(regs.consumed_count.addr, 3);

        assert_eq!(pump.discard_stale_completions(&bus), 3);
    }

    /// A data packet header of this profile with `ticks` and `samples` of cs12
    /// payload, no trailer.
    fn vita_data_header(ticks: u64, samples: u32) -> [u8; VITA_HEADER_BYTES] {
        let words = 6 + (samples * 3).div_ceil(4);
        let header = (0b0001u32 << 28) | (1 << 27) | (0b01 << 20) | words;
        let mut out = [0u8; VITA_HEADER_BYTES];
        out[0..4].copy_from_slice(&header.to_be_bytes());
        out[16..20].copy_from_slice(&((ticks >> 32) as u32).to_be_bytes());
        out[20..24].copy_from_slice(&(ticks as u32).to_be_bytes());
        out
    }

    #[test]
    fn consumed_reports_the_end_sample_count_of_the_latest_finished_packet() {
        let (mut pump, device, bus, regs) = depacketizer();
        for (slot, ticks) in [(2u16, 1_000u64), (5u16, 1_481u64)] {
            device
                .write_slot(
                    Ring::Rx,
                    slot,
                    SLOT_FRAME_OFFSET + VITA_HEADER_AT,
                    &vita_data_header(ticks, 481),
                )
                .unwrap();
            device.deliver_steered(SlotEntry { slot, len: 1514 });
        }
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().consumed, 0, "nothing finished yet");

        bus.set(regs.consumed_count.addr, 1);
        bus.set(regs.consumed.addr, 2);
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().consumed, 1_481, "first packet's end sample");

        bus.set(regs.consumed_count.addr, 1);
        bus.set(regs.consumed.addr, 5);
        pump.step(&bus).unwrap();
        assert_eq!(pump.counters().consumed, 1_962);
        assert_eq!(pump.consumed_packets(), 2);
    }

    #[test]
    fn the_submit_fifo_is_never_overfilled() {
        let (mut pump, device, bus, regs) = depacketizer();
        let delivered = SUBMIT_FIFO_DEPTH + 8;
        for index in 0..delivered {
            device.deliver_steered(SlotEntry {
                slot: (index % RING_SLOTS as usize) as u16,
                len: 64,
            });
        }
        // The fake hands over one batch per poll, so drain it before checking.
        for _ in 0..delivered.div_ceil(SLOT_BATCH_MAX) {
            pump.step(&bus).unwrap();
        }
        assert_eq!(bus.writes_to(regs.submit.addr).len(), SUBMIT_FIFO_DEPTH);
    }

    #[test]
    fn the_transmit_counters_come_straight_from_the_depacketizer() {
        let (mut pump, _device, bus, regs) = depacketizer();
        bus.set(regs.late.addr, 3);
        bus.set(regs.underrun.addr, 4);
        bus.set(regs.gaps.addr, 5);
        bus.set(regs.bad_id.addr, 6);
        pump.step(&bus).unwrap();
        let counters = pump.counters();
        assert_eq!((counters.late, counters.underrun, counters.gaps), (3, 4, 5));
        assert_eq!(pump.bad_stream_id(), 6);
    }

    #[test]
    fn a_scheduled_start_holds_the_core_until_the_clock_reaches_the_tick() {
        let (pump, device, bus, regs) = depacketizer();
        let map = fixture_csr_map();
        let timebase = Timebase::resolve(&map).unwrap();
        let ctrl = PacketizerCtrl {
            enable: true,
            arm: false,
            format: WireFormat::Cs8,
            channels: ChannelMask::from_channels(&[0]),
        };
        const AT_TICKS: u64 = 100;
        let mut pump = pump.start_at(AT_TICKS, timebase, ctrl);
        device.deliver_steered(SlotEntry { slot: 1, len: 64 });

        bus.set(timebase.now_lo.addr, (AT_TICKS - 1) as u32);
        pump.step(&bus).unwrap();
        assert!(
            bus.writes_to(regs.ctrl.addr).is_empty(),
            "core still stopped"
        );
        assert!(bus.writes_to(regs.submit.addr).is_empty());

        bus.set(timebase.now_lo.addr, AT_TICKS as u32);
        pump.step(&bus).unwrap();
        assert_eq!(bus.writes_to(regs.ctrl.addr), vec![ctrl.bits()]);
        assert_eq!(bus.writes_to(regs.submit.addr).len(), 1);
    }

    #[test]
    fn a_slot_outside_the_ring_is_an_error_rather_than_a_panic() {
        let (mut pump, device, bus, _regs) = depacketizer();
        device.deliver_steered(SlotEntry {
            slot: RING_SLOTS as u16,
            len: 64,
        });
        assert!(pump.step(&bus).is_err());
    }
}
