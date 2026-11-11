//! A transceiver that only remembers what it was told, for `--fake` and tests.
//!
//! It quantizes nothing, so a test that asserts on a reported value is
//! asserting on what the daemon asked for.

use super::{
    CHANNEL_COUNT, Direction, GainControlMode, Radio, RadioError, SAMPLE_RATE_MIN_HZ, SampleRates,
    Sensors, TX_MUTED_GAIN_DB, check_channel, check_sample_rate, max_sample_rate, plan_rates,
    resolve_antenna, tx_gain_attenuation_mdb, tx_gain_db_from_attenuation,
};

/// Largest fabric ratio the fake stands in for.
const FAKE_RATIO_MAX: u32 = 32;

/// Frequency both synthesizers report before anything tunes them, in the
/// middle of the AD9363's range.
const DEFAULT_FREQUENCY_HZ: u64 = 2_400_000_000;
/// Bandwidth both directions report before anything sets one.
const DEFAULT_BANDWIDTH_HZ: u32 = 18_000_000;
/// Receive gain both channels report before anything sets one.
const DEFAULT_RX_GAIN_DB: f64 = 30.0;
/// Die temperature the fake reports, a plausible idle reading.
const FAKE_TEMPERATURE_C: f64 = 42.0;
/// Receive level the fake reports, well below full scale.
const FAKE_RSSI_DBFS: f64 = 60.0;

pub struct FakeRadio {
    tx_enabled: bool,
    frequency_hz: [u64; 2],
    sample_rates: SampleRates,
    bandwidth_hz: [u32; 2],
    rx_gain_db: [f64; CHANNEL_COUNT],
    tx_gain_db: [f64; CHANNEL_COUNT],
    gain_mode: [GainControlMode; CHANNEL_COUNT],
    antenna: [&'static str; 2],
}

fn index(direction: Direction) -> usize {
    match direction {
        Direction::Rx => 0,
        Direction::Tx => 1,
    }
}

impl FakeRadio {
    pub fn new() -> Self {
        Self {
            tx_enabled: false,
            frequency_hz: [DEFAULT_FREQUENCY_HZ; 2],
            sample_rates: SampleRates {
                rx_hz: SAMPLE_RATE_MIN_HZ,
                tx_hz: SAMPLE_RATE_MIN_HZ,
                tick_rate_hz: SAMPLE_RATE_MIN_HZ,
                rx_delay_ticks: 0,
                tx_delay_ticks: 0,
            },
            bandwidth_hz: [DEFAULT_BANDWIDTH_HZ; 2],
            rx_gain_db: [DEFAULT_RX_GAIN_DB; CHANNEL_COUNT],
            tx_gain_db: [TX_MUTED_GAIN_DB; CHANNEL_COUNT],
            gain_mode: [GainControlMode::SlowAttack; CHANNEL_COUNT],
            antenna: [super::RX_ANTENNAS[0], super::TX_ANTENNAS[0]],
        }
    }
}

impl Default for FakeRadio {
    fn default() -> Self {
        Self::new()
    }
}

impl Radio for FakeRadio {
    fn set_tx_enabled(&mut self, enabled: bool) -> Result<(), RadioError> {
        self.tx_enabled = enabled;
        Ok(())
    }

    fn tx_enabled(&self) -> bool {
        self.tx_enabled
    }

    fn set_frequency(&mut self, direction: Direction, hz: u64) -> Result<u64, RadioError> {
        self.frequency_hz[index(direction)] = hz;
        Ok(hz)
    }

    fn frequency(&self, direction: Direction) -> Result<u64, RadioError> {
        Ok(self.frequency_hz[index(direction)])
    }

    fn set_sample_rates(&mut self, rx_hz: u32, tx_hz: u32) -> Result<SampleRates, RadioError> {
        check_sample_rate(rx_hz, SAMPLE_RATE_MIN_HZ, 1)?;
        check_sample_rate(tx_hz, SAMPLE_RATE_MIN_HZ, 1)?;
        let plan = plan_rates(rx_hz, tx_hz, FAKE_RATIO_MAX, max_sample_rate(CHANNEL_COUNT))?;
        self.sample_rates = SampleRates {
            rx_hz,
            tx_hz,
            tick_rate_hz: plan.radio_hz,
            rx_delay_ticks: 0,
            tx_delay_ticks: 0,
        };
        Ok(self.sample_rates)
    }

    fn sample_rates(&self) -> Result<SampleRates, RadioError> {
        Ok(self.sample_rates)
    }

    fn active_channels(&self) -> usize {
        if self.sample_rates.tick_rate_hz > max_sample_rate(CHANNEL_COUNT) {
            1
        } else {
            CHANNEL_COUNT
        }
    }

    fn set_bandwidth(&mut self, direction: Direction, hz: u32) -> Result<u32, RadioError> {
        self.bandwidth_hz[index(direction)] = hz;
        Ok(hz)
    }

    fn bandwidth(&self, direction: Direction) -> Result<u32, RadioError> {
        Ok(self.bandwidth_hz[index(direction)])
    }

    fn set_gain(
        &mut self,
        direction: Direction,
        channel: usize,
        db: f64,
    ) -> Result<f64, RadioError> {
        check_channel(channel)?;
        match direction {
            Direction::Rx => self.rx_gain_db[channel] = db,
            Direction::Tx => {
                self.tx_gain_db[channel] =
                    tx_gain_db_from_attenuation(tx_gain_attenuation_mdb(db)?);
            }
        }
        Ok(db)
    }

    fn gain(&self, direction: Direction, channel: usize) -> Result<f64, RadioError> {
        check_channel(channel)?;
        Ok(match direction {
            Direction::Rx => self.rx_gain_db[channel],
            Direction::Tx => self.tx_gain_db[channel],
        })
    }

    fn set_gain_mode(
        &mut self,
        _direction: Direction,
        channel: usize,
        mode: GainControlMode,
    ) -> Result<GainControlMode, RadioError> {
        check_channel(channel)?;
        self.gain_mode[channel] = mode;
        Ok(mode)
    }

    fn gain_mode(
        &self,
        _direction: Direction,
        channel: usize,
    ) -> Result<GainControlMode, RadioError> {
        check_channel(channel)?;
        Ok(self.gain_mode[channel])
    }

    fn set_antenna(
        &mut self,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<&'static str, RadioError> {
        check_channel(channel)?;
        let resolved = resolve_antenna(direction, name)?;
        self.antenna[index(direction)] = resolved;
        Ok(resolved)
    }

    fn antenna(&self, direction: Direction, channel: usize) -> Result<&'static str, RadioError> {
        check_channel(channel)?;
        Ok(self.antenna[index(direction)])
    }

    fn sensors(&mut self) -> Result<Sensors, RadioError> {
        Ok(Sensors {
            temperature_c: FAKE_TEMPERATURE_C,
            rssi_dbfs: [FAKE_RSSI_DBFS; CHANNEL_COUNT],
            rx_lo_locked: true,
            tx_lo_locked: true,
        })
    }

    fn set_loopback(&mut self, _on: bool) -> Result<(), RadioError> {
        Ok(())
    }

    fn describe(&self) -> String {
        "fake radio".to_string()
    }
}
