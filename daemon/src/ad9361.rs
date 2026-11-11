//! The AD9363 through ADI's no-OS driver, with SPI and GPIO routed over the
//! SoC's CSRs.
//!
//! The daemon is the single owner of the chip. It brings the part up in 2R2T
//! LVDS FDD mode with the transmitters at maximum attenuation, aligns both
//! LVDS directions with the PRBS sweep in [`interface_calibration`], and then
//! serves the control protocol's radio calls.

mod baseband_fir;
mod ffi;
mod interface_calibration;
mod platform;

use std::ffi::c_void;
use std::sync::Arc;
use std::time::Duration;

pub use platform::{
    Ad9361Regs, CONFIG_ENABLE, CONFIG_RST_N, CONFIG_TXNRX, PHY_CONTROL_2R2T, PHY_CONTROL_LOOPBACK,
    PHY_CONTROL_MODE_1R1T,
};

use crate::csr::{CsrBus, Reg};
use crate::radio::{
    CHANNEL_COUNT, Direction, FIR_BYPASS_MIN_HZ, GainControlMode, Radio, RadioError, RatePlan,
    SAMPLE_RATE_MIN_HZ, SampleRates, Sensors, TX_ATTENUATION_MAX_MDB, check_channel,
    check_sample_rate, fir_factor, max_sample_rate, plan_rates, resolve_antenna,
    tx_gain_attenuation_mdb, tx_gain_db_from_attenuation, with_tx_muted,
};
use crate::rate_filter::BYPASS_RATIO;

/// Reference clock the driver is told about: the board's 40 MHz XO.
pub const DEFAULT_REF_CLK_HZ: u32 = 40_000_000;
/// LO both synthesizers come up on, in the middle of the AD9363's range.
pub const DEFAULT_FREQUENCY_HZ: u64 = 2_400_000_000;
/// Sample rate the part comes up on, before a stream negotiates one.
/// The fabric PHY clocks its logic at DATA_CLK, which is four times the sample
/// rate in 2R2T and is constrained at 61.44 MHz, so the start-up rate stays
/// under 15.36 MS/s.
pub const DEFAULT_SAMPLE_RATE_HZ: u32 = 10_000_000;
/// Analog filter corner the part comes up on, inside the AD9363's 20 MHz cap.
pub const DEFAULT_BANDWIDTH_HZ: u32 = 18_000_000;
pub const DEFAULT_RX_GAIN_DB: i32 = 30;
pub const DEFAULT_GAIN_CONTROL_MODE: GainControlMode = GainControlMode::SlowAttack;

/// The AD9361 receive-gain calls take whole dB.
const RX_GAIN_DB_MIN: f64 = -10.0;
const RX_GAIN_DB_MAX: f64 = 77.0;
/// The temperature call reports millidegrees Celsius.
const MILLIDEGREES_PER_DEGREE: f64 = 1000.0;

/// RF port codes the no-OS port-select calls take for the board's single
/// balanced receive port and single transmit port.
const RX_PORT_A_BALANCED: u32 = 0;
const TX_PORT_A: u32 = 0;

/// `ad9361_test` values: the stream samples (normal) or the fabric PRBS
/// generator used while the transmit LVDS eye is measured.
const TX_TEST_SOURCE_NORMAL: u64 = 0;
const TX_TEST_SOURCE_PRBS: u64 = 1;

const INTERFACE_DELAY_DATA_MASK: u8 = 0x0f;
const INTERFACE_DELAY_CLOCK_SHIFT: u32 = 4;
const PRBS_SYNCED: u64 = 1;
const PRBS_SETTLE_TIME: Duration = Duration::from_millis(2);
const PRBS_POLL_INTERVAL: Duration = Duration::from_micros(50);
const PRBS_LOCK_OBSERVATIONS: usize = 64;
const TX_CHAIN_SETTLE: Duration = Duration::from_millis(1);
const TX_CHAIN_FLUSH: Duration = Duration::from_millis(1);
/// Low half of the `rx_activity` status word: the RX_FRAME rising-edge count.
/// The bits above it are sticky per-pin seen-low/seen-high flags, not a counter.
const RX_ACTIVITY_FRAME_EDGES_MASK: u64 = 0xFFFF;
const CAP_ACTIVITY_COUNTER_BITS: usize = 16;
const CAP_ACTIVITY_COUNTER_MASK: u64 = (1 << CAP_ACTIVITY_COUNTER_BITS) - 1;

fn nonzero_samples_advance(before: u64, after: u64) -> bool {
    (0..CHANNEL_COUNT).all(|channel| {
        let shift = channel * CAP_ACTIVITY_COUNTER_BITS;
        ((before >> shift) & CAP_ACTIVITY_COUNTER_MASK)
            != ((after >> shift) & CAP_ACTIVITY_COUNTER_MASK)
    })
}

/// Gain-control mode codes as the no-OS `rf_gain_ctrl_mode` enum orders them.
const GC_MODE_MANUAL: u8 = 0;
const GC_MODE_FAST_ATTACK: u8 = 1;
const GC_MODE_SLOW_ATTACK: u8 = 2;

fn gc_mode_code(mode: GainControlMode) -> u8 {
    match mode {
        GainControlMode::Manual => GC_MODE_MANUAL,
        GainControlMode::FastAttack => GC_MODE_FAST_ATTACK,
        GainControlMode::SlowAttack => GC_MODE_SLOW_ATTACK,
    }
}

fn gc_mode_from_code(code: u8) -> Result<GainControlMode, RadioError> {
    match code {
        GC_MODE_MANUAL => Ok(GainControlMode::Manual),
        GC_MODE_FAST_ATTACK => Ok(GainControlMode::FastAttack),
        GC_MODE_SLOW_ATTACK => Ok(GainControlMode::SlowAttack),
        other => Err(RadioError::Unavailable(format!(
            "AD9361 reported gain-control mode {other}, which the daemon does not name"
        ))),
    }
}

fn channel_count(two_channels: bool) -> usize {
    if two_channels { CHANNEL_COUNT } else { 1 }
}

#[derive(Clone, Copy, Debug)]
pub struct Ad9361Params {
    pub ref_clk_hz: u32,
    pub tx_lo_hz: u64,
    pub rx_lo_hz: u64,
    pub sample_rate_hz: u32,
    pub bandwidth_hz: u32,
    pub tx_attenuation_mdb: u32,
    pub rx_gain_db: i32,
    pub gain_control_mode: GainControlMode,
    /// 2R2T when set, 1R1T otherwise. The fabric PHY follows the same choice.
    pub two_channels: bool,
    /// Bit-mapped digital interface options passed to the driver's init
    /// parameters, for aligning the chip's LVDS conventions with the PHY.
    pub interface_flags: u32,
}

impl Default for Ad9361Params {
    fn default() -> Self {
        Self {
            ref_clk_hz: DEFAULT_REF_CLK_HZ,
            tx_lo_hz: DEFAULT_FREQUENCY_HZ,
            rx_lo_hz: DEFAULT_FREQUENCY_HZ,
            sample_rate_hz: DEFAULT_SAMPLE_RATE_HZ,
            bandwidth_hz: DEFAULT_BANDWIDTH_HZ,
            // Nothing radiates until the host asks for gain.
            tx_attenuation_mdb: TX_ATTENUATION_MAX_MDB,
            rx_gain_db: DEFAULT_RX_GAIN_DB,
            gain_control_mode: DEFAULT_GAIN_CONTROL_MODE,
            two_channels: true,
            interface_flags: 0,
        }
    }
}

pub struct Ad9361 {
    tx_enabled: bool,
    phy: *mut c_void,
    antenna: [&'static str; 2],
    /// The mode the part and the fabric PHY run in now.
    two_channels: bool,
    /// Whether the bitstream streams two channels. Only then does the mode
    /// follow the sample rate.
    two_channel_build: bool,
    ref_clk_hz: u32,
    interface_flags: u32,
    /// Receive gain settings of the channels 1R1T leaves off. Applying them
    /// to the part in 1R1T stops its receive data port.
    parked_rx_gain: [(GainControlMode, f64); CHANNEL_COUNT],
    bus: Arc<dyn CsrBus>,
    regs: Ad9361Regs,
    /// Re-run LVDS calibration after every sample-rate change. The eye is
    /// measured in delay taps of fixed size, so its position depends on the
    /// DATA_CLK period. Off while the PRBS is held or calibration is skipped.
    recalibrate_on_rate_change: bool,
    /// Receive and transmit rates of the last request that settled.
    last_sample_rate_request_hz: Option<(u32, u32)>,
    /// The radio rate the last request settled on, as the driver reads it back.
    /// The driver truncates, so it can sit just below the rate requested.
    settled_radio_hz: u32,
    /// Radio samples per receive and per transmit stream sample in the fabric
    /// rate filter.
    fabric_ratios: (u32, u32),
}

// The driver state is only ever touched through this handle.
unsafe impl Send for Ad9361 {}

fn check(op: &'static str, code: i32) -> Result<(), RadioError> {
    if code < 0 {
        Err(RadioError::Driver { op, code })
    } else {
        Ok(())
    }
}

impl Ad9361 {
    /// Pulses RESETB, selects 2R2T on the fabric PHY, brings the part up
    /// through the driver, and leaves it in FDD.
    pub fn init(
        bus: Arc<dyn CsrBus>,
        regs: Ad9361Regs,
        params: &Ad9361Params,
    ) -> Result<Self, RadioError> {
        check_sample_rate(
            params.sample_rate_hz,
            SAMPLE_RATE_MIN_HZ,
            channel_count(params.two_channels),
        )?;
        if params.sample_rate_hz < FIR_BYPASS_MIN_HZ {
            return Err(RadioError::Unavailable(format!(
                "start-up sample rate must be at least {FIR_BYPASS_MIN_HZ} Hz, \
                 lower rates are set after start-up"
            )));
        }
        let phy_mode = if params.two_channels {
            PHY_CONTROL_2R2T
        } else {
            PHY_CONTROL_MODE_1R1T
        };
        regs.phy_control.write(&*bus, u64::from(phy_mode));
        platform::install(bus.clone(), regs.clone());
        platform::reset_pulse();
        let mut phy: *mut c_void = std::ptr::null_mut();
        // SAFETY: phy is a live local the shim fills only on success, and every
        // other argument is a scalar.
        let rc = unsafe {
            ffi::psks_ad9361_init(
                &mut phy,
                params.ref_clk_hz,
                params.tx_lo_hz,
                params.rx_lo_hz,
                params.sample_rate_hz,
                params.bandwidth_hz,
                params.tx_attenuation_mdb as i32,
                params.rx_gain_db,
                gc_mode_code(params.gain_control_mode),
                u8::from(params.two_channels),
                params.interface_flags,
            )
        };
        check("ad9361_init", rc)?;
        let radio = Self {
            tx_enabled: false,
            phy,
            antenna: [crate::radio::RX_ANTENNAS[0], crate::radio::TX_ANTENNAS[0]],
            two_channels: params.two_channels,
            two_channel_build: params.two_channels,
            ref_clk_hz: params.ref_clk_hz,
            interface_flags: params.interface_flags,
            parked_rx_gain: [(params.gain_control_mode, f64::from(params.rx_gain_db));
                CHANNEL_COUNT],
            bus,
            regs,
            recalibrate_on_rate_change: false,
            last_sample_rate_request_hz: None,
            settled_radio_hz: 0,
            fabric_ratios: (BYPASS_RATIO, BYPASS_RATIO),
        };
        // A previous daemon may have left the filter decimating.
        if let Some(filter) = &radio.regs.rate_filter {
            filter.configure(&*radio.bus, BYPASS_RATIO, BYPASS_RATIO)?;
        }
        // SAFETY: init above returned a live driver allocation owned by radio.
        check("set FDD", unsafe { ffi::psks_ad9361_set_fdd(phy) })?;
        radio.verify_fir(1)?;
        let rate = radio.read_sample_rate()?;
        log::info!(
            "AD9363 up in {} FDD: sample_rate={rate} requested={} ensm_state={:#x}",
            if params.two_channels { "2R2T" } else { "1R1T" },
            params.sample_rate_hz,
            radio.ensm_state()
        );
        Ok(radio)
    }

    pub fn ensm_state(&self) -> u8 {
        // SAFETY: self owns the live driver allocation for its lifetime.
        unsafe { ffi::psks_ad9361_ensm_state(self.phy) }
    }

    /// Diagnostic: switches the chip's RX PRBS generator on and leaves it on.
    pub fn hold_rx_prbs(&self) -> Result<(), RadioError> {
        self.bist_prbs_rx(true)
    }

    fn bist_prbs_rx(&self, on: bool) -> Result<(), RadioError> {
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("bist_prbs_rx", unsafe {
            ffi::psks_ad9361_bist_prbs_rx(self.phy, u8::from(on))
        })
    }

    /// AD9361 internal TX-to-RX digital loopback. This drives the real RX data
    /// pins from the TX side, so a sync in loopback separates a non-delivering
    /// RX pin path from an unaligned interface.
    fn bist_loopback(&self, on: bool) -> Result<(), RadioError> {
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("bist_loopback", unsafe {
            ffi::psks_ad9361_bist_loopback(self.phy, i32::from(on))
        })
    }

    /// Aligns RX and TX LVDS timing with PRBS at the current sample rate and
    /// keeps doing so after each rate change. Both transmitters sit at full
    /// attenuation throughout.
    pub fn calibrate_interface(&mut self) -> Result<(), RadioError> {
        self.last_sample_rate_request_hz = None;
        self.recalibrate_on_rate_change = true;
        self.run_interface_calibration()
    }

    fn run_interface_calibration(&mut self) -> Result<(), RadioError> {
        with_tx_muted(self, Self::run_muted_interface_calibration)
    }

    fn run_muted_interface_calibration(&mut self) -> Result<(), RadioError> {
        let bus = self.bus.clone();
        let regs = self.regs.clone();
        let bus: &dyn CsrBus = &*bus;
        let regs = &regs;
        let result = interface_calibration::calibrate(&mut InterfaceCalibrationIo {
            radio: self,
            bus,
            prbs_rx: regs.prbs_rx,
            tx_test: regs.test,
            rx_activity: regs.rx_activity,
            cap_activity: regs.cap_activity.ok_or_else(|| {
                RadioError::Unavailable("PRBS calibration requires nonzero sample counters".into())
            })?,
            saved_power_down: None,
        });
        let calibration = result?;
        log::info!(
            "AD9363 LVDS aligned: rx_clock_taps={} rx_data_taps={} rx_eye_taps={} tx_clock_taps={} tx_data_taps={} tx_eye_taps={}",
            calibration.rx.delay.clock_taps,
            calibration.rx.delay.data_taps,
            calibration.rx.width_taps,
            calibration.tx.delay.clock_taps,
            calibration.tx.delay.data_taps,
            calibration.tx.width_taps,
        );
        Ok(())
    }

    fn set_tx_attenuation_mdb(&mut self, channel: u8, mdb: u32) -> Result<(), RadioError> {
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("set_tx_attenuation", unsafe {
            ffi::psks_ad9361_set_tx_atten(self.phy, channel, mdb)
        })
    }

    fn tx_attenuation_mdb(&self, channel: u8) -> Result<u32, RadioError> {
        let mut mdb = 0u32;
        // SAFETY: self owns the live driver allocation and mdb is a live local.
        check("get_tx_attenuation", unsafe {
            ffi::psks_ad9361_get_tx_atten(self.phy, channel, &mut mdb)
        })?;
        Ok(mdb)
    }

    /// Loads the programmable FIR for `factor`, or selects bypass for 1. The
    /// next sample-rate call applies it.
    fn select_fir(&self, factor: u32) -> Result<(), RadioError> {
        let coef: &[i16] = match factor {
            2 => &baseband_fir::FIR_X2,
            4 => &baseband_fir::FIR_X4,
            _ => &[],
        };
        // SAFETY: self owns the live driver allocation, and the shim copies the
        // coefficients before returning.
        check("select_fir", unsafe {
            ffi::psks_ad9361_select_fir(self.phy, factor, coef.as_ptr(), coef.len() as u8)
        })
    }

    fn verify_fir(&self, factor: u32) -> Result<(), RadioError> {
        let (mut rx, mut tx) = (0u32, 0u32);
        // SAFETY: self owns the live driver allocation and both out parameters are live locals.
        check("read FIR enable", unsafe {
            ffi::psks_ad9361_fir_factors(self.phy, &mut rx, &mut tx)
        })?;
        if rx != factor || tx != factor {
            return Err(RadioError::Unavailable(format!(
                "AD9363 FIR settled at RX x{rx} and TX x{tx}, expected x{factor}"
            )));
        }
        if factor == 1 {
            log::info!("AD9363 FIR bypassed");
        } else {
            log::info!("AD9363 FIR at x{factor} decimation and interpolation");
        }
        Ok(())
    }
}

struct InterfaceCalibrationIo<'a> {
    radio: &'a Ad9361,
    bus: &'a dyn CsrBus,
    prbs_rx: Reg,
    tx_test: Reg,
    rx_activity: Reg,
    cap_activity: Reg,
    saved_power_down: Option<u16>,
}

impl interface_calibration::InterfaceIo for InterfaceCalibrationIo<'_> {
    fn delay(
        &mut self,
        direction: interface_calibration::Direction,
    ) -> Result<interface_calibration::Delay, RadioError> {
        let tx = u8::from(direction == interface_calibration::Direction::Tx);
        // SAFETY: radio owns the live driver allocation throughout this borrow.
        let value = unsafe { ffi::psks_ad9361_get_interface_delay(self.radio.phy, tx) };
        check("get_interface_delay", value)?;
        Ok(interface_calibration::Delay {
            clock_taps: (value as u8) >> INTERFACE_DELAY_CLOCK_SHIFT,
            data_taps: (value as u8) & INTERFACE_DELAY_DATA_MASK,
        })
    }

    fn set_delay(
        &mut self,
        direction: interface_calibration::Direction,
        delay: interface_calibration::Delay,
    ) -> Result<(), RadioError> {
        let tx = u8::from(direction == interface_calibration::Direction::Tx);
        // SAFETY: radio owns the live driver allocation and the sweep bounds both delays.
        check("set_interface_delay", unsafe {
            ffi::psks_ad9361_set_interface_delay(
                self.radio.phy,
                tx,
                delay.clock_taps,
                delay.data_taps,
            )
        })
    }

    fn set_test_mode(&mut self, mode: interface_calibration::TestMode) -> Result<(), RadioError> {
        use interface_calibration::TestMode;
        self.tx_test.write(self.bus, TX_TEST_SOURCE_NORMAL);
        if let Some(saved) = self.saved_power_down.take() {
            platform::set_tx_enabled(false);
            // SAFETY: the driver is live and saved came from this calibration.
            check("restore_tx_analog", unsafe {
                ffi::psks_ad9361_restore_tx_analog(self.radio.phy, saved)
            })?;
        }
        let bist_off = self.radio.bist_prbs_rx(false);
        let loopback_off = self.radio.bist_loopback(false);
        bist_off?;
        loopback_off?;
        match mode {
            TestMode::Normal => Ok(()),
            TestMode::RxPrbs => self.radio.bist_prbs_rx(true),
            TestMode::TxLoopback => {
                self.radio.bist_loopback(true)?;
                let mut saved = 0;
                // SAFETY: the driver and output value remain live throughout the call.
                check("tx_analog_off", unsafe {
                    ffi::psks_ad9361_tx_analog_off(self.radio.phy, &mut saved)
                })?;
                self.saved_power_down = Some(saved);
                platform::set_tx_enabled(true);
                self.tx_test.write(self.bus, TX_TEST_SOURCE_PRBS);
                Ok(())
            }
        }
    }

    fn prbs_locked(&mut self) -> bool {
        std::thread::sleep(PRBS_SETTLE_TIME);
        let nonzero_before = self.cap_activity.read(self.bus);
        let mut previous = self.rx_activity.read(self.bus) & RX_ACTIVITY_FRAME_EDGES_MASK;
        for _ in 0..PRBS_LOCK_OBSERVATIONS {
            std::thread::sleep(PRBS_POLL_INTERVAL);
            let current = self.rx_activity.read(self.bus) & RX_ACTIVITY_FRAME_EDGES_MASK;
            let frames_advance = current != previous;
            if self.prbs_rx.read(self.bus) & PRBS_SYNCED == 0 || !frames_advance {
                return false;
            }
            previous = current;
        }
        nonzero_samples_advance(nonzero_before, self.cap_activity.read(self.bus))
    }
}

impl Ad9361 {
    fn retune_muted(&mut self, direction: Direction, hz: u64) -> Result<u64, RadioError> {
        match direction {
            // SAFETY: self owns the live driver allocation for its lifetime.
            Direction::Rx => check("set_rx_lo_freq", unsafe {
                ffi::psks_ad9361_set_rx_lo(self.phy, hz)
            })?,
            Direction::Tx => {
                // SAFETY: self owns the live driver allocation for its lifetime.
                check("set_tx_lo_freq", unsafe {
                    ffi::psks_ad9361_set_tx_lo(self.phy, hz)
                })?;
            }
        }
        self.frequency(direction)
    }

    fn read_frequency(&self, direction: Direction) -> Result<u64, RadioError> {
        let mut hz = 0u64;
        // SAFETY: self owns the live driver allocation and hz is a live local.
        let rc = unsafe {
            match direction {
                Direction::Rx => ffi::psks_ad9361_get_rx_lo(self.phy, &mut hz),
                Direction::Tx => ffi::psks_ad9361_get_tx_lo(self.phy, &mut hz),
            }
        };
        check("get_lo_freq", rc)?;
        Ok(hz)
    }

    /// Largest fabric ratio available: the rate filter needs 2R2T.
    fn fabric_ratio_max(&self) -> u32 {
        match &self.regs.rate_filter {
            Some(filter) if self.two_channel_build => filter.ratio_max(),
            _ => BYPASS_RATIO,
        }
    }

    /// Whether `channel`'s receive gain is held aside because 1R1T has it off.
    fn rx_parked(&self, direction: Direction, channel: usize) -> bool {
        direction == Direction::Rx && channel >= channel_count(self.two_channels)
    }

    /// 2R2T up to its ceiling on a two-channel bitstream, 1R1T otherwise.
    fn mode_for_rate(&self, hz: u32) -> bool {
        self.two_channel_build && hz <= max_sample_rate(CHANNEL_COUNT)
    }

    /// Brings the part up again in the other channel mode, as at start-up, and
    /// puts back the settings that returned to their start-up values. The
    /// sample rate is left for the caller to set.
    ///
    /// The driver's own mode switch keeps state it cached in the previous
    /// mode and leaves the data port idle in 1R1T.
    fn switch_channel_mode_muted(&mut self, two_channels: bool) -> Result<(), RadioError> {
        let lo = [
            self.read_frequency(Direction::Rx)?,
            self.read_frequency(Direction::Tx)?,
        ];
        let bandwidth = [
            self.read_bandwidth(Direction::Rx)?,
            self.read_bandwidth(Direction::Tx)?,
        ];
        // Channels off in the current mode read back from where they are parked.
        let mut rx_gain = self.parked_rx_gain;
        for (channel, slot) in rx_gain.iter_mut().enumerate() {
            *slot = (
                self.gain_mode(Direction::Rx, channel)?,
                self.gain(Direction::Rx, channel)?,
            );
        }

        let phy_control = self.regs.phy_control.read(&*self.bus) as u32;
        let phy_control = if two_channels {
            phy_control & !PHY_CONTROL_MODE_1R1T
        } else {
            phy_control | PHY_CONTROL_MODE_1R1T
        };
        self.regs
            .phy_control
            .write(&*self.bus, u64::from(phy_control));
        self.last_sample_rate_request_hz = None;
        platform::reset_pulse();
        let mut phy: *mut c_void = std::ptr::null_mut();
        // SAFETY: phy is a live local the shim fills only on success, and every
        // other argument is a scalar.
        let rc = unsafe {
            ffi::psks_ad9361_init(
                &mut phy,
                self.ref_clk_hz,
                lo[1],
                lo[0],
                DEFAULT_SAMPLE_RATE_HZ,
                bandwidth[0],
                TX_ATTENUATION_MAX_MDB as i32,
                DEFAULT_RX_GAIN_DB,
                gc_mode_code(DEFAULT_GAIN_CONTROL_MODE),
                u8::from(two_channels),
                self.interface_flags,
            )
        };
        check("ad9361_init", rc)?;
        // SAFETY: the previous allocation is replaced here and never used again.
        unsafe { ffi::psks_ad9361_free(self.phy) };
        self.phy = phy;
        self.two_channels = two_channels;
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("set FDD", unsafe { ffi::psks_ad9361_set_fdd(self.phy) })?;

        self.set_bandwidth_muted(Direction::Tx, bandwidth[1])?;
        self.parked_rx_gain = rx_gain;
        for (channel, (mode, db)) in rx_gain.into_iter().enumerate() {
            if mode == GainControlMode::Manual {
                self.set_gain(Direction::Rx, channel, db)?;
            } else {
                self.set_gain_mode(Direction::Rx, channel, mode)?;
            }
        }
        log::info!(
            "AD9363 switched to {} FDD",
            if two_channels { "2R2T" } else { "1R1T" }
        );
        Ok(())
    }

    fn plan_rates(&self, rx_hz: u32, tx_hz: u32) -> Result<RatePlan, RadioError> {
        let (min_hz, max_hz) = (self.min_sample_rate(), self.max_sample_rate());
        for hz in [rx_hz, tx_hz] {
            if !(min_hz..=max_hz).contains(&hz) {
                return Err(RadioError::SampleRateRange { hz, min_hz, max_hz });
            }
        }
        let plan = plan_rates(
            rx_hz,
            tx_hz,
            self.fabric_ratio_max(),
            max_sample_rate(CHANNEL_COUNT),
        )?;
        check_sample_rate(
            plan.radio_hz,
            SAMPLE_RATE_MIN_HZ,
            channel_count(self.mode_for_rate(plan.radio_hz)),
        )?;
        Ok(plan)
    }

    fn filter_delay(&self, direction: Direction, ratio: u32) -> u32 {
        match &self.regs.rate_filter {
            Some(filter) => filter.delay_ticks(direction, ratio),
            None => 0,
        }
    }

    fn rates_at(&self, radio_hz: u32, ratios: (u32, u32)) -> SampleRates {
        SampleRates {
            rx_hz: stream_rate(radio_hz, ratios.0),
            tx_hz: stream_rate(radio_hz, ratios.1),
            tick_rate_hz: radio_hz,
            rx_delay_ticks: self.filter_delay(Direction::Rx, ratios.0),
            tx_delay_ticks: self.filter_delay(Direction::Tx, ratios.1),
        }
    }

    fn set_sample_rates_muted(
        &mut self,
        request: (u32, u32),
        plan: RatePlan,
    ) -> Result<SampleRates, RadioError> {
        let ratios = (plan.rx_ratio, plan.tx_ratio);
        let radio_hz = plan.radio_hz;
        if self.last_sample_rate_request_hz == Some(request)
            && self.fabric_ratios == ratios
            && self.read_sample_rate()? == self.settled_radio_hz
            && self.read_tx_sample_rate()? == self.settled_radio_hz
        {
            log::debug!(
                "AD9363 sample rates remain {request:?} Hz, retaining calibrated interface"
            );
            return Ok(self.rates_at(self.settled_radio_hz, ratios));
        }
        self.last_sample_rate_request_hz = None;
        let factor = fir_factor(radio_hz);
        self.select_fir(factor)?;
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("set_rx_sampling_freq", unsafe {
            ffi::psks_ad9361_set_rx_rate(self.phy, radio_hz)
        })?;
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("set_tx_sampling_freq", unsafe {
            ffi::psks_ad9361_set_tx_rate(self.phy, radio_hz)
        })?;
        if self.recalibrate_on_rate_change {
            self.run_interface_calibration()?;
        }
        self.verify_fir(factor)?;
        if let Some(filter) = &self.regs.rate_filter {
            filter.configure(&*self.bus, ratios.0, ratios.1)?;
            self.fabric_ratios = ratios;
            if ratios != (BYPASS_RATIO, BYPASS_RATIO) {
                log::info!(
                    "fabric rate filter at rx x{} tx x{}, AD9363 at {radio_hz} Hz",
                    ratios.0,
                    ratios.1
                );
            }
        }
        let radio_rate = self.read_sample_rate()?;
        let tx_hz = self.read_tx_sample_rate()?;
        // The timebase counts one radio sample period for both directions, so a
        // part that rounded them apart would drift transmit against receive.
        if tx_hz != radio_rate {
            log::warn!(
                "AD9363 settled on rx {radio_rate} Hz but tx {tx_hz} Hz for a requested {radio_hz} Hz"
            );
        } else {
            self.last_sample_rate_request_hz = Some(request);
            self.settled_radio_hz = radio_rate;
        }
        Ok(self.rates_at(radio_rate, ratios))
    }

    fn read_tx_sample_rate(&self) -> Result<u32, RadioError> {
        let mut hz = 0u32;
        // SAFETY: self owns the live driver allocation and hz is a live local.
        check("get_tx_sampling_freq", unsafe {
            ffi::psks_ad9361_get_tx_rate(self.phy, &mut hz)
        })?;
        Ok(hz)
    }

    fn read_sample_rate(&self) -> Result<u32, RadioError> {
        let mut hz = 0u32;
        // SAFETY: self owns the live driver allocation and hz is a live local.
        check("get_rx_sampling_freq", unsafe {
            ffi::psks_ad9361_get_rx_rate(self.phy, &mut hz)
        })?;
        Ok(hz)
    }

    fn set_bandwidth_muted(&mut self, direction: Direction, hz: u32) -> Result<u32, RadioError> {
        // SAFETY: self owns the live driver allocation for its lifetime.
        let rc = unsafe {
            match direction {
                Direction::Rx => ffi::psks_ad9361_set_rx_bw(self.phy, hz),
                Direction::Tx => ffi::psks_ad9361_set_tx_bw(self.phy, hz),
            }
        };
        check("set_rf_bandwidth", rc)?;
        self.bandwidth(direction)
    }

    fn read_bandwidth(&self, direction: Direction) -> Result<u32, RadioError> {
        let mut hz = 0u32;
        // SAFETY: self owns the live driver allocation and hz is a live local.
        let rc = unsafe {
            match direction {
                Direction::Rx => ffi::psks_ad9361_get_rx_bw(self.phy, &mut hz),
                Direction::Tx => ffi::psks_ad9361_get_tx_bw(self.phy, &mut hz),
            }
        };
        check("get_rf_bandwidth", rc)?;
        Ok(hz)
    }
}

impl Radio for Ad9361 {
    fn set_tx_enabled(&mut self, enabled: bool) -> Result<(), RadioError> {
        if self.tx_enabled == enabled {
            return Ok(());
        }
        if !enabled {
            std::thread::sleep(TX_CHAIN_FLUSH);
        }
        platform::set_tx_enabled(enabled);
        self.tx_enabled = enabled;
        if enabled {
            std::thread::sleep(TX_CHAIN_SETTLE);
        }
        Ok(())
    }

    fn tx_enabled(&self) -> bool {
        self.tx_enabled
    }

    fn set_frequency(&mut self, direction: Direction, hz: u64) -> Result<u64, RadioError> {
        with_tx_muted(self, |radio| radio.retune_muted(direction, hz))
    }

    fn frequency(&self, direction: Direction) -> Result<u64, RadioError> {
        self.read_frequency(direction)
    }

    fn set_sample_rates(&mut self, rx_hz: u32, tx_hz: u32) -> Result<SampleRates, RadioError> {
        let plan = self.plan_rates(rx_hz, tx_hz)?;
        let two_channels = self.mode_for_rate(plan.radio_hz);
        with_tx_muted(self, |radio| {
            if two_channels != radio.two_channels {
                radio.switch_channel_mode_muted(two_channels)?;
            }
            radio.set_sample_rates_muted((rx_hz, tx_hz), plan)
        })
    }

    fn active_channels(&self) -> usize {
        channel_count(self.two_channels)
    }

    fn min_sample_rate(&self) -> u32 {
        SAMPLE_RATE_MIN_HZ.div_ceil(self.fabric_ratio_max())
    }

    fn sample_rates(&self) -> Result<SampleRates, RadioError> {
        Ok(self.rates_at(self.read_sample_rate()?, self.fabric_ratios))
    }

    fn set_bandwidth(&mut self, direction: Direction, hz: u32) -> Result<u32, RadioError> {
        with_tx_muted(self, |radio| radio.set_bandwidth_muted(direction, hz))
    }

    fn bandwidth(&self, direction: Direction) -> Result<u32, RadioError> {
        self.read_bandwidth(direction)
    }

    fn set_gain(
        &mut self,
        direction: Direction,
        channel: usize,
        db: f64,
    ) -> Result<f64, RadioError> {
        let channel = check_channel(channel)?;
        if self.rx_parked(direction, channel as usize) {
            let whole_db = db.clamp(RX_GAIN_DB_MIN, RX_GAIN_DB_MAX).round();
            self.parked_rx_gain[channel as usize] = (GainControlMode::Manual, whole_db);
            return Ok(whole_db);
        }
        match direction {
            Direction::Rx => {
                // The part only takes a gain in manual mode, and a host that
                // sets a gain means it.
                if self.gain_mode(direction, channel as usize)? != GainControlMode::Manual {
                    self.set_gain_mode(direction, channel as usize, GainControlMode::Manual)?;
                }
                let whole_db = db.clamp(RX_GAIN_DB_MIN, RX_GAIN_DB_MAX).round() as i32;
                // SAFETY: self owns the live driver allocation for its lifetime.
                check("set_rx_rf_gain", unsafe {
                    ffi::psks_ad9361_set_rx_gain(self.phy, channel, whole_db)
                })?;
            }
            Direction::Tx => {
                let mdb = tx_gain_attenuation_mdb(db)?;
                self.set_tx_attenuation_mdb(channel, mdb)?;
            }
        }
        self.gain(direction, channel as usize)
    }

    fn gain(&self, direction: Direction, channel: usize) -> Result<f64, RadioError> {
        let channel = check_channel(channel)?;
        if self.rx_parked(direction, channel as usize) {
            return Ok(self.parked_rx_gain[channel as usize].1);
        }
        match direction {
            Direction::Rx => {
                let mut db = 0i32;
                // SAFETY: self owns the live driver allocation and db is a live local.
                check("get_rx_rf_gain", unsafe {
                    ffi::psks_ad9361_get_rx_gain(self.phy, channel, &mut db)
                })?;
                Ok(f64::from(db))
            }
            Direction::Tx => Ok(tx_gain_db_from_attenuation(
                self.tx_attenuation_mdb(channel)?,
            )),
        }
    }

    fn set_gain_mode(
        &mut self,
        direction: Direction,
        channel: usize,
        mode: GainControlMode,
    ) -> Result<GainControlMode, RadioError> {
        let channel = check_channel(channel)?;
        if direction == Direction::Tx {
            return Err(RadioError::Unavailable(
                "the AD9363 transmit path has no automatic gain control".to_string(),
            ));
        }
        if self.rx_parked(direction, channel as usize) {
            self.parked_rx_gain[channel as usize].0 = mode;
            return Ok(mode);
        }
        // SAFETY: self owns the live driver allocation for its lifetime.
        check("set_rx_gain_control_mode", unsafe {
            ffi::psks_ad9361_set_gain_control_mode(self.phy, channel, gc_mode_code(mode))
        })?;
        self.gain_mode(direction, channel as usize)
    }

    fn gain_mode(
        &self,
        direction: Direction,
        channel: usize,
    ) -> Result<GainControlMode, RadioError> {
        let channel = check_channel(channel)?;
        if direction == Direction::Tx {
            return Ok(GainControlMode::Manual);
        }
        if self.rx_parked(direction, channel as usize) {
            return Ok(self.parked_rx_gain[channel as usize].0);
        }
        let mut code = 0u8;
        // SAFETY: self owns the live driver allocation and code is a live local.
        check("get_rx_gain_control_mode", unsafe {
            ffi::psks_ad9361_get_gain_control_mode(self.phy, channel, &mut code)
        })?;
        gc_mode_from_code(code)
    }

    fn set_antenna(
        &mut self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<&'static str, RadioError> {
        check_channel(channel)?;
        let resolved = resolve_antenna(direction, name)?;
        // SAFETY: self owns the live driver allocation for its lifetime.
        let rc = unsafe {
            match direction {
                Direction::Rx => ffi::psks_ad9361_set_rx_port(self.phy, RX_PORT_A_BALANCED),
                Direction::Tx => ffi::psks_ad9361_set_tx_port(self.phy, TX_PORT_A),
            }
        };
        check("set_rf_port", rc)?;
        self.antenna[direction_index(direction)] = resolved;
        Ok(resolved)
    }

    fn antenna(&self, direction: Direction, channel: usize) -> Result<&'static str, RadioError> {
        check_channel(channel)?;
        Ok(self.antenna[direction_index(direction)])
    }

    fn sensors(&mut self) -> Result<Sensors, RadioError> {
        let mut millidegrees = 0i32;
        // SAFETY: self owns the live driver allocation and the out parameter is a live local.
        check("get_temperature", unsafe {
            ffi::psks_ad9361_get_temperature(self.phy, &mut millidegrees)
        })?;
        let mut rssi_dbfs = [0.0; CHANNEL_COUNT];
        for (channel, slot) in rssi_dbfs.iter_mut().enumerate() {
            let (mut symbol, mut multiplier) = (0i32, 0i32);
            // SAFETY: self owns the live driver allocation and both out parameters are live locals.
            check("get_rx_rssi", unsafe {
                ffi::psks_ad9361_get_rx_rssi(self.phy, channel as u8, &mut symbol, &mut multiplier)
            })?;
            *slot = if multiplier == 0 {
                0.0
            } else {
                f64::from(symbol) / f64::from(multiplier)
            };
        }
        // SAFETY: self owns the live driver allocation for its lifetime.
        let rx_lock = unsafe { ffi::psks_ad9361_rx_lo_locked(self.phy) };
        check("read RX VCO lock", rx_lock)?;
        // SAFETY: self owns the live driver allocation for its lifetime.
        let tx_lock = unsafe { ffi::psks_ad9361_tx_lo_locked(self.phy) };
        check("read TX VCO lock", tx_lock)?;
        Ok(Sensors {
            temperature_c: f64::from(millidegrees) / MILLIDEGREES_PER_DEGREE,
            rssi_dbfs,
            rx_lo_locked: rx_lock == 1,
            tx_lo_locked: tx_lock == 1,
        })
    }

    fn set_loopback(&mut self, on: bool) -> Result<(), RadioError> {
        self.bist_loopback(on)
    }

    fn describe(&self) -> String {
        format!(
            "AD9363 via the no-OS driver, {} FDD, ENSM state {:#x}",
            if self.two_channels { "2R2T" } else { "1R1T" },
            self.ensm_state()
        )
    }
}

/// The driver truncates the rate it reads back, so a radio rate of 777599 Hz
/// for a requested 777600 Hz is a 388800 Hz stream, not 388799 Hz.
fn stream_rate(radio_hz: u32, ratio: u32) -> u32 {
    (radio_hz + ratio / 2) / ratio
}

fn direction_index(direction: Direction) -> usize {
    match direction {
        Direction::Rx => 0,
        Direction::Tx => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prbs_activity_requires_nonzero_samples_on_both_channels() {
        assert!(!nonzero_samples_advance(0, 0));
        assert!(!nonzero_samples_advance(0, 1));
        assert!(!nonzero_samples_advance(0, 1 << CAP_ACTIVITY_COUNTER_BITS));
        assert!(nonzero_samples_advance(
            0,
            1 | (1 << CAP_ACTIVITY_COUNTER_BITS)
        ));
        assert!(nonzero_samples_advance(u32::MAX as u64, 0));
    }

    #[test]
    fn gain_control_mode_codes_round_trip_through_the_no_os_enum() {
        for mode in [
            GainControlMode::Manual,
            GainControlMode::SlowAttack,
            GainControlMode::FastAttack,
        ] {
            assert_eq!(gc_mode_from_code(gc_mode_code(mode)).unwrap(), mode);
        }
        assert!(gc_mode_from_code(9).is_err());
    }

    #[test]
    fn the_stream_rate_rounds_the_truncated_radio_rate() {
        assert_eq!(stream_rate(777_599, 2), 388_800);
        assert_eq!(stream_rate(599_999, 6), 100_000);
        assert_eq!(stream_rate(520_864, 32), 16_277);
        assert_eq!(stream_rate(2_500_000, 1), 2_500_000);
    }

    #[test]
    fn the_default_rate_is_inside_the_advertised_range() {
        let min_hz = SAMPLE_RATE_MIN_HZ;
        assert!(check_sample_rate(DEFAULT_SAMPLE_RATE_HZ, min_hz, CHANNEL_COUNT).is_ok());
        assert!(check_sample_rate(min_hz, min_hz, CHANNEL_COUNT).is_ok());
    }
}
