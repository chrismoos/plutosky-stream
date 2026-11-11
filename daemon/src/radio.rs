//! The transceiver the daemon drives: the AD9363 on the board, or a stand-in
//! for host testing.
//!
//! Every setter reports the value the part actually settled on, read back from
//! the driver, because the AD9361's clock chain quantizes rates and bandwidths
//! and its gain tables quantize gain.

mod fake;

use thiserror::Error;

pub use fake::FakeRadio;

/// Receive and transmit channels the 2R2T configuration exposes.
pub const CHANNEL_COUNT: usize = 2;

/// Slowest sample rate the AD9363 reaches through its halfband decimators
/// alone: the 25 MHz minimum ADC clock over their largest ratio, 12. At and
/// above it the programmable FIR is bypassed.
pub const FIR_BYPASS_MIN_HZ: u32 = 2_083_334;
/// Ratios the programmable FIR adds below [`FIR_BYPASS_MIN_HZ`].
const FIR_FACTORS: [u32; 2] = [2, 4];
/// Slowest sample rate the AD9363 reaches, with the programmable FIR at its
/// largest ratio. Slower stream rates need the fabric rate filter.
pub const SAMPLE_RATE_MIN_HZ: u32 = FIR_BYPASS_MIN_HZ.div_ceil(FIR_FACTORS[1]);
/// Fastest DATA_CLK the fabric PHY is timing-constrained for.
const PHY_DATA_CLK_MAX_HZ: u32 = 61_440_000;
/// DATA_CLK cycles per sample in 1R1T. Each added channel adds the same again.
const DATA_CLK_CYCLES_PER_CHANNEL: u32 = 2;

/// Largest attenuation the AD9361 TX path accepts.
pub const TX_ATTENUATION_MAX_MDB: u32 = 89_750;
pub const TX_GAIN_MAX_DB: f64 = TX_ATTENUATION_MAX_MDB as f64 / 1000.0;
pub const TX_GAIN_STEP_DB: f64 = 0.25;
const TX_GAIN_STEP_MDB: u32 = 250;
const MDB_PER_DB: f64 = 1000.0;
pub(crate) const TX_MUTED_GAIN_DB: f64 = 0.0;

/// The board wires one balanced receive port and one transmit port per
/// channel, so each direction lists a single antenna. The names are the ones
/// ADI's own drivers use for those ports.
pub const RX_ANTENNAS: &[&str] = &["A_BALANCED"];
pub const TX_ANTENNAS: &[&str] = &["A"];

#[derive(Debug, Error)]
pub enum RadioError {
    #[error("{op} failed: driver returned {code}")]
    Driver { op: &'static str, code: i32 },
    #[error("channel {channel} does not exist: the radio has {CHANNEL_COUNT}")]
    NoSuchChannel { channel: usize },
    #[error("{direction:?} has no antenna named `{name}`")]
    NoSuchAntenna { direction: Direction, name: String },
    #[error("sample rate {hz} Hz is outside {min_hz}..={max_hz} Hz")]
    SampleRateRange { hz: u32, min_hz: u32, max_hz: u32 },
    #[error(
        "receive {rx_hz} Hz and transmit {tx_hz} Hz have no common radio rate: \
         the radio would run at {radio_hz} Hz, above {max_hz} Hz or more than {ratio_max} times one of them"
    )]
    NoCommonRate {
        rx_hz: u32,
        tx_hz: u32,
        radio_hz: u64,
        max_hz: u32,
        ratio_max: u32,
    },
    #[error("TX gain {db} dB is outside 0..={TX_GAIN_MAX_DB} dB")]
    TxGainRange { db: f64 },
    #[error("{0}")]
    Unavailable(String),
}

/// Which half of the transceiver a call addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Rx,
    Tx,
}

impl Direction {
    pub fn as_str(self) -> &'static str {
        match self {
            Direction::Rx => "rx",
            Direction::Tx => "tx",
        }
    }
}

/// How the AD9361 RX path picks its gain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GainControlMode {
    /// The value from `set_gain` is used as-is.
    Manual,
    /// The part tracks gain on its own, tuned for stationary levels.
    SlowAttack,
    /// The part tracks gain on its own, tuned to react quickly to bursts.
    FastAttack,
}

impl GainControlMode {
    pub fn as_str(self) -> &'static str {
        match self {
            GainControlMode::Manual => "manual",
            GainControlMode::SlowAttack => "slow_attack",
            GainControlMode::FastAttack => "fast_attack",
        }
    }

    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "manual" => Some(GainControlMode::Manual),
            "slow_attack" => Some(GainControlMode::SlowAttack),
            "fast_attack" => Some(GainControlMode::FastAttack),
            _ => None,
        }
    }
}

/// One reading of everything the part measures about itself.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sensors {
    pub temperature_c: f64,
    /// Per receive channel, in dB below full scale.
    pub rssi_dbfs: [f64; CHANNEL_COUNT],
    pub rx_lo_locked: bool,
    pub tx_lo_locked: bool,
}

/// The rates the part settled on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SampleRates {
    pub rx_hz: u32,
    pub tx_hz: u32,
    /// Timebase counts per second: the rate the radio itself runs at.
    pub tick_rate_hz: u32,
    /// Timebase counts by which the rate filter delays receive samples, so a
    /// receive timestamp reads that much late.
    pub rx_delay_ticks: u32,
    /// Timebase counts by which the rate filter delays transmit samples after
    /// their timestamp.
    pub tx_delay_ticks: u32,
}

/// The radio rate and the whole ratio of each direction to it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RatePlan {
    pub radio_hz: u32,
    pub rx_ratio: u32,
    pub tx_ratio: u32,
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// The slowest radio rate at or above [`SAMPLE_RATE_MIN_HZ`] that is a whole
/// multiple of both stream rates. `ratio_max` is the largest fabric ratio, 1
/// without the rate filter, and `filter_max_hz` the fastest radio rate the
/// filter runs at. Equal rates above `filter_max_hz` need no filter.
pub fn plan_rates(
    rx_hz: u32,
    tx_hz: u32,
    ratio_max: u32,
    filter_max_hz: u32,
) -> Result<RatePlan, RadioError> {
    let (rx, tx) = (u64::from(rx_hz.max(1)), u64::from(tx_hz.max(1)));
    let common = rx / gcd(rx, tx) * tx;
    let radio = common * u64::from(SAMPLE_RATE_MIN_HZ).div_ceil(common).max(1);
    let (rx_ratio, tx_ratio) = (radio / rx, radio / tx);
    let filtered = rx_ratio > 1 || tx_ratio > 1;
    let ratio_limit = u64::from(ratio_max.max(1));
    if rx_ratio > ratio_limit
        || tx_ratio > ratio_limit
        || (filtered && radio > u64::from(filter_max_hz))
    {
        return Err(RadioError::NoCommonRate {
            rx_hz,
            tx_hz,
            radio_hz: radio,
            max_hz: filter_max_hz,
            ratio_max,
        });
    }
    Ok(RatePlan {
        radio_hz: u32::try_from(radio).unwrap_or(u32::MAX),
        rx_ratio: rx_ratio as u32,
        tx_ratio: tx_ratio as u32,
    })
}

/// Returns an error unless `channel` exists.
pub fn check_channel(channel: usize) -> Result<u8, RadioError> {
    if channel >= CHANNEL_COUNT {
        return Err(RadioError::NoSuchChannel { channel });
    }
    Ok(channel as u8)
}

pub fn tx_gain_attenuation_mdb(db: f64) -> Result<u32, RadioError> {
    if !db.is_finite() || !(0.0..=TX_GAIN_MAX_DB).contains(&db) {
        return Err(RadioError::TxGainRange { db });
    }
    let gain_steps = (db / TX_GAIN_STEP_DB).round() as u32;
    Ok(TX_ATTENUATION_MAX_MDB - gain_steps * TX_GAIN_STEP_MDB)
}

pub fn tx_gain_db_from_attenuation(mdb: u32) -> f64 {
    f64::from(TX_ATTENUATION_MAX_MDB - mdb) / MDB_PER_DB
}

pub fn antennas(direction: Direction) -> &'static [&'static str] {
    match direction {
        Direction::Rx => RX_ANTENNAS,
        Direction::Tx => TX_ANTENNAS,
    }
}

/// The transceiver operations the control protocol maps onto. Each getter
/// reads the part rather than a shadow, so a value the part rounded is visible.
pub trait Radio: Send {
    /// Enables the RF transmit chain independently of receive and attenuation.
    fn set_tx_enabled(&mut self, enabled: bool) -> Result<(), RadioError>;
    fn tx_enabled(&self) -> bool;

    /// The AD9361 has one synthesizer per direction, shared by both channels,
    /// so a retune moves the other channel too.
    fn set_frequency(&mut self, direction: Direction, hz: u64) -> Result<u64, RadioError>;
    fn frequency(&self, direction: Direction) -> Result<u64, RadioError>;

    /// Sets the rate of each direction, shared by both channels. The radio
    /// runs at one rate both divide, see [`plan_rates`].
    fn set_sample_rates(&mut self, rx_hz: u32, tx_hz: u32) -> Result<SampleRates, RadioError>;
    /// Fastest rate [`Radio::set_sample_rates`] accepts.
    fn max_sample_rate(&self) -> u32 {
        max_sample_rate(1)
    }
    /// Channels that deliver samples at the current rate. Above the
    /// two-channel ceiling the part runs one channel.
    fn active_channels(&self) -> usize {
        CHANNEL_COUNT
    }
    /// Slowest rate [`Radio::set_sample_rates`] accepts.
    fn min_sample_rate(&self) -> u32 {
        SAMPLE_RATE_MIN_HZ
    }
    fn sample_rates(&self) -> Result<SampleRates, RadioError>;

    /// The analog filter corner, shared by both channels of a direction.
    fn set_bandwidth(&mut self, direction: Direction, hz: u32) -> Result<u32, RadioError>;
    fn bandwidth(&self, direction: Direction) -> Result<u32, RadioError>;

    /// Receive gain is the gain-table value in dB. Transmit gain runs from
    /// 0 dB at maximum attenuation to 89.75 dB at zero attenuation.
    fn set_gain(
        &mut self,
        direction: Direction,
        channel: usize,
        db: f64,
    ) -> Result<f64, RadioError>;
    fn gain(&self, direction: Direction, channel: usize) -> Result<f64, RadioError>;

    /// Gain mode applies to the receive path only.
    fn set_gain_mode(
        &mut self,
        direction: Direction,
        channel: usize,
        mode: GainControlMode,
    ) -> Result<GainControlMode, RadioError>;
    fn gain_mode(
        &self,
        direction: Direction,
        channel: usize,
    ) -> Result<GainControlMode, RadioError>;

    fn set_antenna(
        &mut self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<&'static str, RadioError>;
    fn antenna(&self, direction: Direction, channel: usize) -> Result<&'static str, RadioError>;

    fn sensors(&mut self) -> Result<Sensors, RadioError>;

    /// AD9361 internal TX-to-RX digital loopback. With it on, samples the
    /// depacketizer plays into the TX interface come straight back on the RX
    /// interface, so the round trip measures the TX-plus-RX pipeline delay.
    fn set_loopback(&mut self, on: bool) -> Result<(), RadioError>;

    fn describe(&self) -> String;
}

/// Runs RF reconfiguration with TX disabled and leaves it disabled on failure.
pub(crate) fn with_tx_muted<R: Radio, T>(
    radio: &mut R,
    operation: impl FnOnce(&mut R) -> Result<T, RadioError>,
) -> Result<T, RadioError> {
    let was_enabled = radio.tx_enabled();
    radio.set_tx_enabled(false)?;
    let mut saved = [TX_MUTED_GAIN_DB; CHANNEL_COUNT];
    for (channel, gain) in saved.iter_mut().enumerate() {
        *gain = radio.gain(Direction::Tx, channel)?;
    }
    let mute = |radio: &mut R| {
        let mut failure = None;
        for channel in 0..CHANNEL_COUNT {
            if let Err(error) = radio.set_gain(Direction::Tx, channel, TX_MUTED_GAIN_DB) {
                log::error!("could not mute TX channel {channel}: {error}");
                failure = Some(error);
            }
        }
        failure.map_or(Ok(()), Err)
    };
    mute(radio)?;
    let result = operation(radio)?;
    for (channel, gain) in saved.into_iter().enumerate() {
        if let Err(error) = radio.set_gain(Direction::Tx, channel, gain) {
            mute(radio)?;
            return Err(error);
        }
    }
    radio.set_tx_enabled(was_enabled)?;
    Ok(result)
}

/// Resolves an antenna name to the one the direction supports.
pub fn resolve_antenna(direction: Direction, name: &str) -> Result<&'static str, RadioError> {
    antennas(direction)
        .iter()
        .copied()
        .find(|candidate| candidate.eq_ignore_ascii_case(name))
        .ok_or_else(|| RadioError::NoSuchAntenna {
            direction,
            name: name.to_string(),
        })
}

/// Fastest per-channel sample rate the fabric PHY sustains with `channels`
/// channels interleaved on the data port.
pub fn max_sample_rate(channels: usize) -> u32 {
    let channels = u32::try_from(channels.max(1)).unwrap_or(u32::MAX);
    PHY_DATA_CLK_MAX_HZ / DATA_CLK_CYCLES_PER_CHANNEL.saturating_mul(channels)
}

/// Programmable FIR ratio for an in-range sample rate: 1 (bypassed) at and
/// above [`FIR_BYPASS_MIN_HZ`], otherwise the smallest ratio that lifts the
/// rate to it.
pub fn fir_factor(hz: u32) -> u32 {
    if hz >= FIR_BYPASS_MIN_HZ {
        return 1;
    }
    FIR_FACTORS
        .into_iter()
        .find(|factor| hz.saturating_mul(*factor) >= FIR_BYPASS_MIN_HZ)
        .unwrap_or(FIR_FACTORS[FIR_FACTORS.len() - 1])
}

/// Returns an error unless `hz` is at least `min_hz` and a rate the fabric PHY
/// reaches with `channels` channels.
pub fn check_sample_rate(hz: u32, min_hz: u32, channels: usize) -> Result<(), RadioError> {
    let max_hz = max_sample_rate(channels);
    if !(min_hz..=max_hz).contains(&hz) {
        return Err(RadioError::SampleRateRange { hz, min_hz, max_hz });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reconfiguration_mutes_both_channels_and_restores_each_gain() {
        let mut radio = FakeRadio::new();
        radio.set_gain(Direction::Tx, 0, 73.75).unwrap();
        radio.set_gain(Direction::Tx, 1, 59.75).unwrap();
        radio.set_tx_enabled(true).unwrap();
        with_tx_muted(&mut radio, |radio| {
            assert!(!radio.tx_enabled());
            for channel in 0..CHANNEL_COUNT {
                assert_eq!(radio.gain(Direction::Tx, channel)?, TX_MUTED_GAIN_DB);
            }
            Ok(())
        })
        .unwrap();
        assert_eq!(radio.gain(Direction::Tx, 0).unwrap(), 73.75);
        assert_eq!(radio.gain(Direction::Tx, 1).unwrap(), 59.75);
        assert!(radio.tx_enabled());
    }

    #[test]
    fn failed_reconfiguration_leaves_both_channels_muted() {
        let mut radio = FakeRadio::new();
        radio.set_gain(Direction::Tx, 0, 73.75).unwrap();
        radio.set_tx_enabled(true).unwrap();
        let result: Result<(), _> = with_tx_muted(&mut radio, |_| {
            Err(RadioError::Unavailable("calibration failed".into()))
        });
        assert!(result.is_err());
        assert!(!radio.tx_enabled());
        for channel in 0..CHANNEL_COUNT {
            assert_eq!(
                radio.gain(Direction::Tx, channel).unwrap(),
                TX_MUTED_GAIN_DB
            );
        }
    }

    #[test]
    fn antenna_lookup_is_case_insensitive_and_rejects_unknown_names() {
        assert_eq!(
            resolve_antenna(Direction::Rx, "a_balanced").unwrap(),
            "A_BALANCED"
        );
        assert_eq!(resolve_antenna(Direction::Tx, "A").unwrap(), "A");
        assert!(matches!(
            resolve_antenna(Direction::Tx, "B"),
            Err(RadioError::NoSuchAntenna { .. })
        ));
    }

    #[test]
    fn sample_rate_bounds_match_the_handshake() {
        assert_eq!(max_sample_rate(1), 30_720_000);
        assert_eq!(max_sample_rate(2), 15_360_000);
        for channels in [1, 2] {
            let max_hz = max_sample_rate(channels);
            let min_hz = SAMPLE_RATE_MIN_HZ;
            assert!(check_sample_rate(min_hz, min_hz, channels).is_ok());
            assert!(check_sample_rate(max_hz, min_hz, channels).is_ok());
            assert!(check_sample_rate(min_hz - 1, min_hz, channels).is_err());
            assert!(check_sample_rate(max_hz + 1, min_hz, channels).is_err());
        }
    }

    #[test]
    fn rate_plan_runs_the_radio_at_the_slowest_common_rate() {
        let filter_max = max_sample_rate(CHANNEL_COUNT);
        let plan = |rx, tx| plan_rates(rx, tx, 32, filter_max);
        assert_eq!(
            plan(7_680_000, 7_680_000).unwrap(),
            RatePlan {
                radio_hz: 7_680_000,
                rx_ratio: 1,
                tx_ratio: 1
            }
        );
        assert_eq!(
            plan(7_680_000, 15_360_000).unwrap(),
            RatePlan {
                radio_hz: 15_360_000,
                rx_ratio: 2,
                tx_ratio: 1
            }
        );
        assert_eq!(
            plan(3_840_000, 5_760_000).unwrap(),
            RatePlan {
                radio_hz: 11_520_000,
                rx_ratio: 3,
                tx_ratio: 2
            }
        );
        // Below the part's minimum the common rate is multiplied up.
        assert_eq!(
            plan(100_000, 200_000).unwrap(),
            RatePlan {
                radio_hz: 600_000,
                rx_ratio: 6,
                tx_ratio: 3
            }
        );
        assert_eq!(
            plan(100_000, 100_000).unwrap(),
            RatePlan {
                radio_hz: 600_000,
                rx_ratio: 6,
                tx_ratio: 6
            }
        );
        // Equal rates above the filter ceiling run unfiltered.
        assert_eq!(
            plan(30_720_000, 30_720_000).unwrap(),
            RatePlan {
                radio_hz: 30_720_000,
                rx_ratio: 1,
                tx_ratio: 1
            }
        );
    }

    #[test]
    fn rate_plan_rejects_pairs_without_a_common_rate() {
        let filter_max = max_sample_rate(CHANNEL_COUNT);
        for (rx, tx) in [
            (10_000_000, 4_000_000),
            (15_360_000, 30_720_000),
            (7_680_000, 7_680_001),
            (15_360_000, 100_000),
        ] {
            assert!(
                matches!(
                    plan_rates(rx, tx, 32, filter_max),
                    Err(RadioError::NoCommonRate { .. })
                ),
                "{rx} {tx}"
            );
        }
        // Without the filter only equal rates work.
        assert!(plan_rates(2_000_000, 4_000_000, 1, filter_max).is_err());
        assert!(plan_rates(4_000_000, 4_000_000, 1, filter_max).is_ok());
    }

    #[test]
    fn fir_ratio_follows_the_sample_rate() {
        assert_eq!(SAMPLE_RATE_MIN_HZ, 520_834);
        assert_eq!(fir_factor(15_360_000), 1);
        assert_eq!(fir_factor(FIR_BYPASS_MIN_HZ), 1);
        assert_eq!(fir_factor(FIR_BYPASS_MIN_HZ - 1), 2);
        assert_eq!(fir_factor(1_083_334), 2);
        assert_eq!(fir_factor(1_041_667), 2);
        assert_eq!(fir_factor(1_041_666), 4);
        assert_eq!(fir_factor(SAMPLE_RATE_MIN_HZ), 4);
        for hz in [SAMPLE_RATE_MIN_HZ, 700_000, 1_041_666, 1_083_334, 2_000_000] {
            assert!(hz * fir_factor(hz) >= FIR_BYPASS_MIN_HZ);
        }
    }

    #[test]
    fn gain_control_modes_round_trip_through_their_protocol_names() {
        for mode in [
            GainControlMode::Manual,
            GainControlMode::SlowAttack,
            GainControlMode::FastAttack,
        ] {
            assert_eq!(GainControlMode::parse(mode.as_str()), Some(mode));
        }
        assert_eq!(GainControlMode::parse("agc"), None);
    }

    #[test]
    fn transmit_gain_increases_as_attenuation_falls() {
        assert_eq!(tx_gain_attenuation_mdb(0.0).unwrap(), 89_750);
        assert_eq!(tx_gain_attenuation_mdb(69.75).unwrap(), 20_000);
        assert_eq!(tx_gain_attenuation_mdb(89.75).unwrap(), 0);
        assert_eq!(tx_gain_db_from_attenuation(20_000), 69.75);
        assert_eq!(tx_gain_attenuation_mdb(69.7).unwrap(), 20_000);
        for gain in [-20.0, -0.25, 90.0, f64::NAN, f64::INFINITY] {
            assert!(matches!(
                tx_gain_attenuation_mdb(gain),
                Err(RadioError::TxGainRange { .. })
            ));
        }
    }
}
