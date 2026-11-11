//! Hardware time: the PL's 64-bit counter of sample periods.
//!
//! The counter runs in the RFIC clock domain, so the daemon reads it through a
//! latch: pulse `latch`, then read the two halves of the frozen value. Writing
//! it is the mirror image, and an armed load lets the counter jump at a future
//! count without the daemon being scheduled at that moment.

use std::time::Duration;

use crate::csr::{CsrBus, CsrMap, MapError, Reg};

pub const REG_TIMEBASE_CTRL: &str = "timebase_ctrl";
pub const REG_TIMEBASE_NOW_HI: &str = "timebase_now_hi";
pub const REG_TIMEBASE_NOW_LO: &str = "timebase_now_lo";
pub const REG_TIMEBASE_LOAD_HI: &str = "timebase_load_hi";
pub const REG_TIMEBASE_LOAD_LO: &str = "timebase_load_lo";
pub const REG_TIMEBASE_ARM_HI: &str = "timebase_arm_hi";
pub const REG_TIMEBASE_ARM_LO: &str = "timebase_arm_lo";

const REQUIRED_REGISTERS: &[&str] = &[
    REG_TIMEBASE_CTRL,
    REG_TIMEBASE_NOW_HI,
    REG_TIMEBASE_NOW_LO,
    REG_TIMEBASE_LOAD_HI,
    REG_TIMEBASE_LOAD_LO,
    REG_TIMEBASE_ARM_HI,
    REG_TIMEBASE_ARM_LO,
];

/// `timebase_ctrl` bits. Each is a pulse: writing a one starts the action and
/// the gateware clears it.
pub const CTRL_LATCH: u64 = 1 << 0;
pub const CTRL_LOAD: u64 = 1 << 1;
pub const CTRL_ARM_LOAD: u64 = 1 << 2;

const NANOS_PER_SECOND: u128 = 1_000_000_000;

/// The timebase CSRs, resolved by name.
#[derive(Clone, Copy, Debug)]
pub struct Timebase {
    pub ctrl: Reg,
    pub now_hi: Reg,
    pub now_lo: Reg,
    pub load_hi: Reg,
    pub load_lo: Reg,
    pub arm_hi: Reg,
    pub arm_lo: Reg,
}

impl Timebase {
    pub fn resolve(map: &CsrMap) -> Result<Self, MapError> {
        map.require_registers(REQUIRED_REGISTERS)?;
        let reg = |name| map.reg(name, 1);
        Ok(Self {
            ctrl: reg(REG_TIMEBASE_CTRL)?,
            now_hi: reg(REG_TIMEBASE_NOW_HI)?,
            now_lo: reg(REG_TIMEBASE_NOW_LO)?,
            load_hi: reg(REG_TIMEBASE_LOAD_HI)?,
            load_lo: reg(REG_TIMEBASE_LOAD_LO)?,
            arm_hi: reg(REG_TIMEBASE_ARM_HI)?,
            arm_lo: reg(REG_TIMEBASE_ARM_LO)?,
        })
    }

    /// Latches the counter and reads the frozen value.
    pub fn now(&self, bus: &dyn CsrBus) -> u64 {
        self.ctrl.write(bus, CTRL_LATCH);
        let hi = self.now_hi.read(bus);
        let lo = self.now_lo.read(bus);
        (hi << 32) | (lo & u64::from(u32::MAX))
    }

    /// Sets the counter immediately.
    pub fn set_now(&self, bus: &dyn CsrBus, ticks: u64) {
        self.write_load(bus, ticks);
        self.ctrl.write(bus, CTRL_LOAD);
    }

    /// Sets the counter to `value` the moment it reaches `at_ticks`.
    pub fn arm_load(&self, bus: &dyn CsrBus, at_ticks: u64, value: u64) {
        self.write_load(bus, value);
        self.arm_hi.write(bus, at_ticks >> 32);
        self.arm_lo.write(bus, at_ticks & u64::from(u32::MAX));
        self.ctrl.write(bus, CTRL_ARM_LOAD);
    }

    fn write_load(&self, bus: &dyn CsrBus, ticks: u64) {
        self.load_hi.write(bus, ticks >> 32);
        self.load_lo.write(bus, ticks & u64::from(u32::MAX));
    }
}

/// Sample periods as nanoseconds at `sample_rate_hz`, the same conversion
/// `SoapySDR::ticksToTimeNs` makes on the host.
pub fn ticks_to_nanos(ticks: u64, sample_rate_hz: u32) -> u64 {
    if sample_rate_hz == 0 {
        return 0;
    }
    let nanos = u128::from(ticks) * NANOS_PER_SECOND / u128::from(sample_rate_hz);
    nanos.min(u128::from(u64::MAX)) as u64
}

/// Nanoseconds as sample periods at `sample_rate_hz`, rounded to nearest so a
/// round trip through [`ticks_to_nanos`] returns the original count.
pub fn nanos_to_ticks(nanos: u64, sample_rate_hz: u32) -> u64 {
    if sample_rate_hz == 0 {
        return 0;
    }
    let rate = u128::from(sample_rate_hz);
    let ticks = (u128::from(nanos) * rate + NANOS_PER_SECOND / 2) / NANOS_PER_SECOND;
    ticks.min(u128::from(u64::MAX)) as u64
}

/// How long `ticks` sample periods take at `sample_rate_hz`, for deciding how
/// long to wait before a scheduled start.
pub fn ticks_to_duration(ticks: u64, sample_rate_hz: u32) -> Duration {
    Duration::from_nanos(ticks_to_nanos(ticks, sample_rate_hz))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;
    use crate::testing::fixture_csr_map;

    fn timebase() -> (Timebase, FakeBus) {
        let map = fixture_csr_map();
        (Timebase::resolve(&map).unwrap(), FakeBus::new())
    }

    #[test]
    fn reading_the_counter_latches_it_first() {
        let (timebase, bus) = timebase();
        bus.set(timebase.now_hi.addr, 0x0000_0002);
        bus.set(timebase.now_lo.addr, 0xdead_beef);
        assert_eq!(timebase.now(&bus), 0x0000_0002_dead_beef);
        assert_eq!(bus.writes_to(timebase.ctrl.addr), vec![CTRL_LATCH as u32]);
    }

    #[test]
    fn setting_the_counter_writes_both_halves_before_the_load_pulse() {
        let (timebase, bus) = timebase();
        timebase.set_now(&bus, 0x0000_0003_0000_0007);
        assert_eq!(
            bus.take_writes(),
            vec![
                (timebase.load_hi.addr, 3),
                (timebase.load_lo.addr, 7),
                (timebase.ctrl.addr, CTRL_LOAD as u32),
            ]
        );
    }

    #[test]
    fn an_armed_load_carries_both_the_trigger_and_the_value() {
        let (timebase, bus) = timebase();
        timebase.arm_load(&bus, 0x0000_0001_0000_0000, 0);
        assert_eq!(
            bus.take_writes(),
            vec![
                (timebase.load_hi.addr, 0),
                (timebase.load_lo.addr, 0),
                (timebase.arm_hi.addr, 1),
                (timebase.arm_lo.addr, 0),
                (timebase.ctrl.addr, CTRL_ARM_LOAD as u32),
            ]
        );
    }

    #[test]
    fn tick_and_nanosecond_conversions_round_trip_at_every_supported_rate() {
        for rate in [520_833, 1_000_000, 30_720_000, 61_440_000] {
            for ticks in [0u64, 1, 12_345, 61_440_000, 1 << 40] {
                let nanos = ticks_to_nanos(ticks, rate);
                assert_eq!(
                    nanos_to_ticks(nanos, rate),
                    ticks,
                    "rate {rate} ticks {ticks} became {nanos} ns"
                );
            }
        }
    }

    #[test]
    fn one_second_of_ticks_is_one_second_of_nanoseconds() {
        assert_eq!(ticks_to_nanos(61_440_000, 61_440_000), 1_000_000_000);
        assert_eq!(nanos_to_ticks(1_000_000_000, 30_720_000), 30_720_000);
        assert_eq!(
            ticks_to_duration(15_360_000, 30_720_000),
            Duration::from_millis(500)
        );
    }

    #[test]
    fn conversions_report_zero_rather_than_dividing_by_a_zero_rate() {
        assert_eq!(ticks_to_nanos(1_000, 0), 0);
        assert_eq!(nanos_to_ticks(1_000, 0), 0);
    }
}
