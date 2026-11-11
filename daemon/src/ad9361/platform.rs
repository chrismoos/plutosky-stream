//! The `platform.h` hooks the no-OS driver links against: SPI through the
//! SoC's CSR SPI master, RESETB through the config CSR, delays via sleep.

use std::ffi::{c_int, c_uint, c_ulong, c_void};
use std::sync::Arc;
use std::time::{Duration, Instant};

use log::warn;
use parking_lot::Mutex;

use crate::csr::{CsrBus, CsrMap, MapError, Reg};
use crate::rate_filter::RateFilter;

pub const REG_AD9361_CONFIG: &str = "ad9361_config";
pub const REG_AD9361_CTRL: &str = "ad9361_ctrl";
pub const REG_AD9361_STAT: &str = "ad9361_stat";
pub const REG_AD9361_SPI_CONTROL: &str = "ad9361_spi_control";
pub const REG_AD9361_SPI_STATUS: &str = "ad9361_spi_status";
pub const REG_AD9361_SPI_MOSI: &str = "ad9361_spi_mosi";
pub const REG_AD9361_SPI_MISO: &str = "ad9361_spi_miso";
pub const REG_AD9361_PHY_CONTROL: &str = "ad9361_phy_control";
pub const REG_AD9361_TEST: &str = "ad9361_test";
pub const REG_AD9361_PRBS_RX: &str = "ad9361_prbs_rx";
pub const REG_AD9361_RX_ACTIVITY: &str = "ad9361_rx_activity";

const REQUIRED_REGISTERS: &[&str] = &[
    REG_AD9361_CONFIG,
    REG_AD9361_CTRL,
    REG_AD9361_STAT,
    REG_AD9361_SPI_CONTROL,
    REG_AD9361_SPI_STATUS,
    REG_AD9361_SPI_MOSI,
    REG_AD9361_SPI_MISO,
    REG_AD9361_PHY_CONTROL,
    REG_AD9361_TEST,
    REG_AD9361_PRBS_RX,
    REG_AD9361_RX_ACTIVITY,
];

/// `ad9361_config` bits.
pub const CONFIG_RST_N: u32 = 1 << 0;
pub const CONFIG_ENABLE: u32 = 1 << 1;
pub const CONFIG_TXNRX: u32 = 1 << 4;

/// `ad9361_phy_control` bit 0 selects the data-port interleaving: clear for
/// 2R2T (8-way), set for 1R1T (4-way). Bit 1 loops TX data back to RX inside
/// the fabric.
pub const PHY_CONTROL_MODE_1R1T: u32 = 1 << 0;
pub const PHY_CONTROL_LOOPBACK: u32 = 1 << 1;
/// The daemon runs the PHY in 2R2T so both channels reach the packetizers.
pub const PHY_CONTROL_2R2T: u32 = 0;

/// `ad9361_spi_control` / `ad9361_spi_status` bits of the M2SDR SPI master.
const SPI_CONTROL_START: u32 = 1 << 0;
const SPI_CONTROL_LENGTH_SHIFT: u32 = 8;
const SPI_STATUS_DONE: u32 = 1 << 0;
/// Every AD9361 transaction is 16 command bits plus one data byte.
const SPI_TRANSFER_BITS: u32 = 24;
const SPI_MISO_MASK: u32 = 0xFF;
const SPI_TIMEOUT: Duration = Duration::from_millis(100);
const SPI_POLL_INTERVAL: Duration = Duration::from_micros(1);

/// AD9361 SPI command word: bit 15 write, bits 14..12 byte count minus one,
/// bits 9..0 register address.
const AD_CMD_WRITE: u16 = 1 << 15;
const AD_CMD_ADDR_MASK: u16 = 0x3FF;
const AD_CMD_BYTES: usize = 2;

/// GPIO number the driver uses for RESETB (matches the C shim).
const GPIO_RESETB: c_uint = 0;

const RESET_PULSE: Duration = Duration::from_millis(1);
const RESET_SETTLE: Duration = Duration::from_millis(10);

const EIO: c_int = 5;
const ENODEV: c_int = 19;

#[derive(Clone, Debug)]
pub struct Ad9361Regs {
    pub config: Reg,
    pub ctrl: Reg,
    pub stat: Reg,
    pub spi_control: Reg,
    pub spi_status: Reg,
    pub spi_mosi: Reg,
    pub spi_miso: Reg,
    pub phy_control: Reg,
    /// Selects the fabric TX sample source during LVDS calibration.
    pub test: Reg,
    /// Reports whether the fabric PRBS checker is synchronised.
    pub prbs_rx: Reg,
    /// Advances while receive samples arrive, one 16-bit counter per channel.
    pub rx_activity: Reg,
    /// Counts nonzero receive samples, for rejecting silent PRBS input.
    pub cap_activity: Option<Reg>,
    /// Absent from bitstreams built before the fabric rate filter.
    pub rate_filter: Option<RateFilter>,
}

impl Ad9361Regs {
    pub fn resolve(map: &CsrMap) -> Result<Self, MapError> {
        map.require_registers(REQUIRED_REGISTERS)?;
        let reg = |name| map.reg(name, 1);
        Ok(Self {
            config: reg(REG_AD9361_CONFIG)?,
            ctrl: reg(REG_AD9361_CTRL)?,
            stat: reg(REG_AD9361_STAT)?,
            spi_control: reg(REG_AD9361_SPI_CONTROL)?,
            spi_status: reg(REG_AD9361_SPI_STATUS)?,
            spi_mosi: reg(REG_AD9361_SPI_MOSI)?,
            spi_miso: reg(REG_AD9361_SPI_MISO)?,
            phy_control: reg(REG_AD9361_PHY_CONTROL)?,
            test: reg(REG_AD9361_TEST)?,
            prbs_rx: reg(REG_AD9361_PRBS_RX)?,
            rx_activity: reg(REG_AD9361_RX_ACTIVITY)?,
            cap_activity: reg("ad9361_cap_activity").ok(),
            rate_filter: RateFilter::resolve(map)?,
        })
    }
}

struct Platform {
    bus: Arc<dyn CsrBus>,
    regs: Ad9361Regs,
    /// Shadow of the write-only config CSR.
    config: u32,
}

static PLATFORM: Mutex<Option<Platform>> = Mutex::new(None);

/// Points the hooks at a bus. Must precede any driver call.
pub fn install(bus: Arc<dyn CsrBus>, regs: Ad9361Regs) {
    *PLATFORM.lock() = Some(Platform {
        bus,
        regs,
        config: 0,
    });
}

/// Holds RESETB low, then enables the part and lets it settle.
pub fn reset_pulse() {
    {
        let mut guard = PLATFORM.lock();
        if let Some(p) = guard.as_mut() {
            p.config = 0;
            p.regs.config.write(&*p.bus, p.config as u64);
        }
    }
    std::thread::sleep(RESET_PULSE);
    {
        let mut guard = PLATFORM.lock();
        if let Some(p) = guard.as_mut() {
            p.config = CONFIG_RST_N | CONFIG_ENABLE;
            p.regs.config.write(&*p.bus, p.config as u64);
        }
    }
    std::thread::sleep(RESET_SETTLE);
}

/// Controls the TX chain without changing RX enable or reset.
pub fn set_tx_enabled(enabled: bool) {
    let mut guard = PLATFORM.lock();
    if let Some(p) = guard.as_mut() {
        if enabled {
            p.config |= CONFIG_TXNRX;
        } else {
            p.config &= !CONFIG_TXNRX;
        }
        p.regs.config.write(&*p.bus, p.config as u64);
    }
}

impl Platform {
    /// One 24-bit transaction: command bytes then a data byte out, the
    /// register's byte back on a read.
    fn spi_xfer(&self, reg: u16, write: Option<u8>) -> Result<u8, c_int> {
        let bus = &*self.bus;
        let cmd = (if write.is_some() { AD_CMD_WRITE } else { 0 }) | (reg & AD_CMD_ADDR_MASK);
        let mosi = ((cmd as u32) << 8) | write.unwrap_or(0) as u32;
        self.regs.spi_mosi.write(bus, mosi as u64);
        self.regs.spi_control.write(
            bus,
            ((SPI_TRANSFER_BITS << SPI_CONTROL_LENGTH_SHIFT) | SPI_CONTROL_START) as u64,
        );
        let deadline = Instant::now() + SPI_TIMEOUT;
        while self.regs.spi_status.read(bus) as u32 & SPI_STATUS_DONE == 0 {
            if Instant::now() >= deadline {
                warn!("ad9361: SPI transfer to reg {reg:#05x} timed out");
                return Err(EIO);
            }
            std::thread::sleep(SPI_POLL_INTERVAL);
        }
        if write.is_some() {
            Ok(0)
        } else {
            Ok((self.regs.spi_miso.read(bus) as u32 & SPI_MISO_MASK) as u8)
        }
    }
}

/// Multi-byte transfers address descending registers, as the AD9361 does
/// natively. The 24-bit master needs one transaction per byte.
#[unsafe(no_mangle)]
pub extern "C" fn spi_write_then_read(
    _spi: *mut c_void,
    txbuf: *const u8,
    n_tx: c_uint,
    rxbuf: *mut u8,
    n_rx: c_uint,
) -> c_int {
    if txbuf.is_null() || (n_tx as usize) < AD_CMD_BYTES {
        return -EIO;
    }
    let tx = unsafe { std::slice::from_raw_parts(txbuf, n_tx as usize) };
    let cmd = u16::from_be_bytes([tx[0], tx[1]]);
    let reg = cmd & AD_CMD_ADDR_MASK;
    let guard = PLATFORM.lock();
    let Some(p) = guard.as_ref() else {
        return -ENODEV;
    };
    if cmd & AD_CMD_WRITE != 0 {
        for (i, byte) in tx[AD_CMD_BYTES..].iter().enumerate() {
            if let Err(e) = p.spi_xfer(reg.wrapping_sub(i as u16), Some(*byte)) {
                return -e;
            }
        }
        return 0;
    }
    if rxbuf.is_null() && n_rx > 0 {
        return -EIO;
    }
    let rx = unsafe { std::slice::from_raw_parts_mut(rxbuf, n_rx as usize) };
    for (i, slot) in rx.iter_mut().enumerate() {
        match p.spi_xfer(reg.wrapping_sub(i as u16), None) {
            Ok(v) => *slot = v,
            Err(e) => return -e,
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn spi_init(_device_id: u32, _clk_pha: u8, _clk_pol: u8) -> i32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn spi_read(_data: *mut u8, _bytes_number: u8) -> i32 {
    -EIO
}

#[unsafe(no_mangle)]
pub extern "C" fn gpio_init(_device_id: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn gpio_direction(_pin: u8, _direction: u8) {}

#[unsafe(no_mangle)]
pub extern "C" fn gpio_is_valid(number: c_int) -> bool {
    number == GPIO_RESETB as c_int
}

#[unsafe(no_mangle)]
pub extern "C" fn gpio_set_value(gpio: c_uint, value: c_int) {
    if gpio != GPIO_RESETB {
        warn!("ad9361: ignoring write to unknown GPIO {gpio}");
        return;
    }
    let mut guard = PLATFORM.lock();
    let Some(p) = guard.as_mut() else {
        return;
    };
    if value != 0 {
        p.config |= CONFIG_RST_N;
    } else {
        p.config &= !CONFIG_RST_N;
    }
    p.regs.config.write(&*p.bus, p.config as u64);
}

/// `c_ulong` is 32 bits on the board's armv7 target and 64 bits on a 64-bit
/// host, so the widening is real there and a no-op here.
#[allow(
    clippy::useless_conversion,
    reason = "c_ulong is narrower than u64 on the target this ships to"
)]
fn widen(value: c_ulong) -> u64 {
    value.into()
}

#[unsafe(no_mangle)]
pub extern "C" fn udelay(usecs: c_ulong) {
    std::thread::sleep(Duration::from_micros(widen(usecs)));
}

#[unsafe(no_mangle)]
pub extern "C" fn mdelay(msecs: c_ulong) {
    std::thread::sleep(Duration::from_millis(widen(msecs)));
}

#[unsafe(no_mangle)]
pub extern "C" fn msleep_interruptible(msecs: c_uint) -> c_ulong {
    std::thread::sleep(Duration::from_millis(msecs as u64));
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;

    fn install_fake() -> (Arc<FakeBus>, Ad9361Regs) {
        let map = crate::testing::fixture_csr_map();
        let regs = Ad9361Regs::resolve(&map).unwrap();
        let bus = Arc::new(FakeBus::new());
        bus.set(regs.spi_status.addr, SPI_STATUS_DONE);
        install(bus.clone(), regs.clone());
        (bus, regs)
    }

    // The hooks share one global platform, so the tests run under a lock.
    static TEST_LOCK: Mutex<()> = Mutex::new(());

    #[test]
    fn write_then_read_maps_to_24_bit_transfers() {
        let _guard = TEST_LOCK.lock();
        let (bus, regs) = install_fake();
        bus.set(regs.spi_miso.addr, 0xAB);
        let cmd = 0x0287u16.to_be_bytes();
        let mut rx = [0u8; 1];
        let rc = spi_write_then_read(std::ptr::null_mut(), cmd.as_ptr(), 2, rx.as_mut_ptr(), 1);
        assert_eq!(rc, 0);
        assert_eq!(rx[0], 0xAB);
        assert_eq!(bus.writes_to(regs.spi_mosi.addr), vec![0x02_87_00]);
        assert_eq!(
            bus.writes_to(regs.spi_control.addr),
            vec![(SPI_TRANSFER_BITS << SPI_CONTROL_LENGTH_SHIFT) | SPI_CONTROL_START]
        );
        bus.take_writes();

        let tx = [0x80 | 0x02, 0x87, 0x5A];
        let rc = spi_write_then_read(
            std::ptr::null_mut(),
            tx.as_ptr(),
            3,
            std::ptr::null_mut(),
            0,
        );
        assert_eq!(rc, 0);
        assert_eq!(bus.writes_to(regs.spi_mosi.addr), vec![0x82_87_5A]);
    }

    #[test]
    fn multi_byte_reads_walk_down_the_address() {
        let _guard = TEST_LOCK.lock();
        let (bus, regs) = install_fake();
        let cmd = (0x0010u16 | (1 << 12)).to_be_bytes();
        let mut rx = [0u8; 2];
        assert_eq!(
            spi_write_then_read(std::ptr::null_mut(), cmd.as_ptr(), 2, rx.as_mut_ptr(), 2),
            0
        );
        assert_eq!(
            bus.writes_to(regs.spi_mosi.addr),
            vec![0x00_10_00, 0x00_0F_00]
        );
    }

    #[test]
    fn reset_gpio_drives_config_rst_n() {
        let _guard = TEST_LOCK.lock();
        let (bus, regs) = install_fake();
        reset_pulse();
        assert_eq!(
            bus.writes_to(regs.config.addr),
            vec![0, CONFIG_RST_N | CONFIG_ENABLE]
        );
        bus.take_writes();
        assert!(gpio_is_valid(0));
        assert!(!gpio_is_valid(-1));
        gpio_set_value(GPIO_RESETB, 0);
        gpio_set_value(GPIO_RESETB, 1);
        assert_eq!(
            bus.writes_to(regs.config.addr),
            vec![CONFIG_ENABLE, CONFIG_RST_N | CONFIG_ENABLE]
        );
    }

    #[test]
    fn tx_enable_preserves_receive_and_reset() {
        let _guard = TEST_LOCK.lock();
        let (bus, regs) = install_fake();
        reset_pulse();
        bus.take_writes();
        set_tx_enabled(true);
        set_tx_enabled(false);
        assert_eq!(
            bus.writes_to(regs.config.addr),
            vec![
                CONFIG_RST_N | CONFIG_ENABLE | CONFIG_TXNRX,
                CONFIG_RST_N | CONFIG_ENABLE,
            ]
        );
    }

    #[test]
    fn spi_timeout_reports_eio() {
        let _guard = TEST_LOCK.lock();
        let (bus, regs) = install_fake();
        bus.set(regs.spi_status.addr, 0);
        let cmd = 0x0001u16.to_be_bytes();
        let mut rx = [0u8; 1];
        assert_eq!(
            spi_write_then_read(std::ptr::null_mut(), cmd.as_ptr(), 2, rx.as_mut_ptr(), 1),
            -EIO
        );
    }
}
