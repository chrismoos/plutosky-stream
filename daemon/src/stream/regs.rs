//! The stream-side CSRs: the slot pool geometry, the receive packetizers and
//! the transmit depacketizers, all resolved by the names in
//! `docs/csr-contract.md`.

use crate::csr::{CsrBus, CsrMap, MapError, Reg};
use crate::gem::SlotPoolInfo;

use super::format::WireFormat;
use super::header::HEADER_TEMPLATE_WORDS;

/// Most packetizer and depacketizer instances the CSR contract allows, which
/// numbers them `N = 0..3`. A bitstream may build fewer, so the daemon counts
/// how many the loaded register map actually exports rather than assuming
/// this many.
pub const MAX_STREAMS: usize = 4;

/// `rxpkt{N}_ctrl` and `txpkt{M}_ctrl` bits.
pub const CTRL_ENABLE: u32 = 1 << 0;
pub const CTRL_ARM: u32 = 1 << 1;
const CTRL_FORMAT_SHIFT: u32 = 2;
const CTRL_CHANNEL_MASK_SHIFT: u32 = 4;

/// Entries the depacketizer's submit FIFO holds.
pub const SUBMIT_FIFO_DEPTH: usize = 64;
/// `txpkt{M}_submit` packs the slot index in the low half and the frame length
/// in the high half, in the field order of the driver's `gemslot_tx_entry`.
const SUBMIT_LENGTH_SHIFT: u32 = 16;
const SUBMIT_SLOT_MASK: u32 = 0xffff;

/// Which channels a stream uses, as the `ctrl` channel-mask field encodes
/// them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelMask(u32);

impl ChannelMask {
    /// Builds a mask from the channel list a stream was opened with.
    pub fn from_channels(channels: &[usize]) -> Self {
        Self(channels.iter().fold(0, |mask, ch| mask | (1 << ch)))
    }

    pub fn bits(self) -> u32 {
        self.0
    }

    pub fn count(self) -> usize {
        self.0.count_ones() as usize
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// The receive packetizer for one stream: board to host.
#[derive(Clone, Copy, Debug)]
pub struct RxPacketizerRegs {
    pub ctrl: Reg,
    pub samples_per_packet: Reg,
    pub frame_len: Reg,
    pub start_hi: Reg,
    pub start_lo: Reg,
    pub header: Reg,
    pub produced: Reg,
    pub observed: Reg,
    pub released: Reg,
    pub overflow: Reg,
    pub irq_threshold: Reg,
    pub slot_first: Reg,
    pub slot_count: Reg,
}

impl RxPacketizerRegs {
    pub fn resolve(map: &CsrMap, index: usize) -> Result<Self, MapError> {
        let name = |suffix: &str| format!("rxpkt{index}_{suffix}");
        let names: Vec<String> = [
            "ctrl",
            "samples_per_packet",
            "frame_len",
            "start_hi",
            "start_lo",
            "header",
            "produced",
            "observed",
            "released",
            "overflow",
            "irq_threshold",
            "slot_first",
            "slot_count",
        ]
        .iter()
        .map(|suffix| name(suffix))
        .collect();
        map.require_registers(&names.iter().map(String::as_str).collect::<Vec<_>>())?;
        let reg = |suffix: &str| map.reg(&name(suffix), 1);
        Ok(Self {
            ctrl: reg("ctrl")?,
            samples_per_packet: reg("samples_per_packet")?,
            frame_len: reg("frame_len")?,
            start_hi: reg("start_hi")?,
            start_lo: reg("start_lo")?,
            header: map.reg(&name("header"), HEADER_TEMPLATE_WORDS as u32)?,
            produced: reg("produced")?,
            observed: reg("observed")?,
            released: reg("released")?,
            overflow: reg("overflow")?,
            irq_threshold: reg("irq_threshold")?,
            slot_first: reg("slot_first")?,
            slot_count: reg("slot_count")?,
        })
    }

    /// Writes the template in frame order, word 0 at the lowest address.
    pub fn write_header(&self, bus: &dyn CsrBus, words: &[u32; HEADER_TEMPLATE_WORDS]) {
        bus.write_block(self.header.addr, words);
    }

    pub fn set_ctrl(&self, bus: &dyn CsrBus, ctrl: PacketizerCtrl) {
        self.ctrl.write(bus, u64::from(ctrl.bits()));
    }
}

/// The transmit depacketizer for one stream: host to board.
#[derive(Clone, Copy, Debug)]
pub struct TxDepacketizerRegs {
    pub ctrl: Reg,
    pub stream_id: Reg,
    pub submit: Reg,
    pub consumed: Reg,
    pub consumed_count: Reg,
    pub irq_threshold: Reg,
    pub late: Reg,
    pub underrun: Reg,
    pub gaps: Reg,
    pub bad_id: Reg,
    pub fifo_level: Reg,
}

impl TxDepacketizerRegs {
    pub fn resolve(map: &CsrMap, index: usize) -> Result<Self, MapError> {
        let name = |suffix: &str| format!("txpkt{index}_{suffix}");
        let names: Vec<String> = [
            "ctrl",
            "stream_id",
            "submit",
            "consumed",
            "consumed_count",
            "irq_threshold",
            "late",
            "underrun",
            "gaps",
            "bad_id",
            "fifo_level",
        ]
        .iter()
        .map(|suffix| name(suffix))
        .collect();
        map.require_registers(&names.iter().map(String::as_str).collect::<Vec<_>>())?;
        let reg = |suffix: &str| map.reg(&name(suffix), 1);
        Ok(Self {
            ctrl: reg("ctrl")?,
            stream_id: reg("stream_id")?,
            submit: reg("submit")?,
            consumed: reg("consumed")?,
            consumed_count: reg("consumed_count")?,
            irq_threshold: reg("irq_threshold")?,
            late: reg("late")?,
            underrun: reg("underrun")?,
            gaps: reg("gaps")?,
            bad_id: reg("bad_id")?,
            fifo_level: reg("fifo_level")?,
        })
    }

    /// Hands one filled receive slot to the depacketizer.
    pub fn submit_slot(&self, bus: &dyn CsrBus, slot: u16, len: u16) {
        let word = (u32::from(len) << SUBMIT_LENGTH_SHIFT) | u32::from(slot);
        self.submit.write(bus, u64::from(word));
    }

    /// Reads one slot index the depacketizer has finished with.
    pub fn read_consumed(&self, bus: &dyn CsrBus) -> u16 {
        (self.consumed.read(bus) as u32 & SUBMIT_SLOT_MASK) as u16
    }

    pub fn set_ctrl(&self, bus: &dyn CsrBus, ctrl: PacketizerCtrl) {
        self.ctrl.write(bus, u64::from(ctrl.bits()));
    }
}

/// The `ctrl` word of either direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PacketizerCtrl {
    pub enable: bool,
    /// Start at `start_hi`/`start_lo` rather than immediately. The
    /// depacketizer releases each packet at its own timestamp and ignores it.
    pub arm: bool,
    pub format: WireFormat,
    pub channels: ChannelMask,
}

impl PacketizerCtrl {
    pub fn bits(self) -> u32 {
        let mut bits = self.format.ctrl_code() << CTRL_FORMAT_SHIFT
            | self.channels.bits() << CTRL_CHANNEL_MASK_SHIFT;
        if self.enable {
            bits |= CTRL_ENABLE;
        }
        if self.arm {
            bits |= CTRL_ARM;
        }
        bits
    }

    /// The same configuration with the core stopped.
    pub fn stopped(self) -> Self {
        Self {
            enable: false,
            arm: false,
            ..self
        }
    }
}

/// The slot pool geometry the PL needs so it can address slots itself.
#[derive(Clone, Copy, Debug)]
pub struct SlotPoolRegs {
    pub pool_base: Reg,
    pub slot_size: Reg,
    pub count: Reg,
    pub tx_offset: Reg,
    pub rx_offset: Reg,
}

impl SlotPoolRegs {
    pub fn resolve(map: &CsrMap) -> Result<Self, MapError> {
        map.require_registers(&[
            "slots_pool_base",
            "slots_slot_size",
            "slots_count",
            "slots_tx_offset",
            "slots_rx_offset",
        ])?;
        Ok(Self {
            pool_base: map.reg("slots_pool_base", 1)?,
            slot_size: map.reg("slots_slot_size", 1)?,
            count: map.reg("slots_count", 1)?,
            tx_offset: map.reg("slots_tx_offset", 1)?,
            rx_offset: map.reg("slots_rx_offset", 1)?,
        })
    }

    /// Tells the PL where the driver put the pool.
    pub fn program(&self, bus: &dyn CsrBus, info: &SlotPoolInfo) {
        self.pool_base.write(bus, u64::from(info.pool_phys));
        self.slot_size.write(bus, u64::from(info.slot_size));
        self.count.write(bus, u64::from(info.slot_count));
        self.tx_offset.write(bus, u64::from(info.tx_offset));
        self.rx_offset.write(bus, u64::from(info.rx_offset));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;
    use crate::testing::fixture_csr_map;

    #[test]
    fn every_stream_instance_resolves_from_the_contract_names() {
        let map = fixture_csr_map();
        for index in 0..MAX_STREAMS {
            RxPacketizerRegs::resolve(&map, index).unwrap();
            TxDepacketizerRegs::resolve(&map, index).unwrap();
        }
        assert!(RxPacketizerRegs::resolve(&map, MAX_STREAMS).is_err());
        SlotPoolRegs::resolve(&map).unwrap();
    }

    #[test]
    fn a_missing_name_is_reported_with_the_name_in_it() {
        let map = CsrMap::from_json(r#"{"csr_registers": {}}"#).unwrap();
        let error = RxPacketizerRegs::resolve(&map, 0).unwrap_err().to_string();
        assert!(error.contains("rxpkt0_ctrl"), "{error}");
        assert!(error.contains("rxpkt0_header"), "{error}");
    }

    #[test]
    fn the_control_word_packs_enable_arm_format_and_channels() {
        let ctrl = PacketizerCtrl {
            enable: true,
            arm: true,
            format: WireFormat::Bfp8,
            channels: ChannelMask::from_channels(&[0, 1]),
        };
        assert_eq!(ctrl.bits(), 0b11_10_11);
        assert_eq!(ctrl.stopped().bits(), 0b11_10_00);
        let single = PacketizerCtrl {
            enable: true,
            arm: false,
            format: WireFormat::Cs12,
            channels: ChannelMask::from_channels(&[1]),
        };
        assert_eq!(single.bits(), 0b10_00_01);
    }

    #[test]
    fn a_submitted_slot_carries_its_length_in_the_high_half() {
        let map = fixture_csr_map();
        let regs = TxDepacketizerRegs::resolve(&map, 2).unwrap();
        let bus = FakeBus::new();
        regs.submit_slot(&bus, 0x0123, 0x1514);
        assert_eq!(bus.writes_to(regs.submit.addr), vec![0x1514_0123]);
        bus.set(regs.consumed.addr, 0x1514_0123);
        assert_eq!(regs.read_consumed(&bus), 0x0123);
    }

    #[test]
    fn channel_masks_count_the_channels_they_carry() {
        assert_eq!(ChannelMask::from_channels(&[0]).count(), 1);
        assert_eq!(ChannelMask::from_channels(&[0, 1]).count(), 2);
        assert_eq!(ChannelMask::from_channels(&[0, 1]).bits(), 0b11);
        assert!(ChannelMask::from_channels(&[]).is_empty());
    }
}
