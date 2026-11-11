//! The fabric rate filter between the RFIC and the stream cores.
//!
//! The radio runs at a whole multiple of each direction's stream rate, that
//! direction's ratio. The gateware decimates receive and interpolates transmit
//! by their ratios with polyphase FIRs. Its timebase counts radio samples, so a
//! stream sample spans its ratio in timebase counts. A ratio of 1 bypasses that
//! direction's filter.
//!
//! The coefficients are a Kaiser-windowed sinc cut off at half the stream
//! rate. Its passband runs flat to 0.42 of the stream rate and its stopband
//! starts at 0.58, the same edges as the AD9363 FIR, at every ratio.

use crate::csr::{CsrBus, CsrMap, MapError, Reg};
use crate::radio::{Direction, RadioError};

pub const REG_RX_RATIO: &str = "rate_filter_rx_ratio";
pub const REG_TX_RATIO: &str = "rate_filter_tx_ratio";
pub const REG_COEF_ADDR: &str = "rate_filter_coef_addr";
pub const REG_COEF_DATA: &str = "rate_filter_coef_data";
const CONST_PHASE_TAPS: &str = "rate_filter_phase_taps";
const CONST_RATIO_MAX: &str = "rate_filter_ratio_max";
const CONST_COEF_BITS: &str = "rate_filter_coef_bits";
const CONST_RX_FRAC_BITS: &str = "rate_filter_rx_frac_bits";
const CONST_TX_FRAC_BITS: &str = "rate_filter_tx_frac_bits";

/// Ratio that bypasses both filters.
pub const BYPASS_RATIO: u32 = 1;
/// Radio samples the decimator's pipeline adds to its half-length group delay,
/// rounded up from 1.5. Measured through the AD9361 digital loopback.
const RX_PIPELINE_TICKS: u32 = 2;
/// Radio samples the interpolator's pipeline adds to its half-length group
/// delay and the one stream sample it holds each input, rounded down from 2.5.
/// Measured through the AD9361 digital loopback.
const TX_PIPELINE_TICKS: u32 = 2;
const KAISER_BETA: f64 = 6.0;
const BESSEL_TOLERANCE: f64 = 1e-15;

#[derive(Clone, Debug)]
pub struct RateFilter {
    rx_ratio: Reg,
    tx_ratio: Reg,
    coef_addr: Reg,
    coef_data: Reg,
    phase_taps: u32,
    ratio_max: u32,
    coef_bits: u32,
    rx_frac_bits: u32,
    tx_frac_bits: u32,
}

/// One coefficient set per direction, `phase_taps * ratio` taps each.
#[derive(Clone, Debug, PartialEq)]
pub struct Coefficients {
    pub rx: Vec<i32>,
    pub tx: Vec<i32>,
}

fn bits_for(max: u32) -> u32 {
    (u32::BITS - max.leading_zeros()).max(1)
}

fn bessel_i0(x: f64) -> f64 {
    let quarter_square = x * x / 4.0;
    let mut term = 1.0;
    let mut sum = 1.0;
    let mut k = 1.0;
    while term > BESSEL_TOLERANCE * sum {
        term *= quarter_square / (k * k);
        sum += term;
        k += 1.0;
    }
    sum
}

/// The low-pass for `ratio`, normalized to a DC gain of one.
pub fn prototype(phase_taps: u32, ratio: u32) -> Vec<f64> {
    let taps = (phase_taps * ratio) as usize;
    let center = (taps as f64 - 1.0) / 2.0;
    let window_norm = bessel_i0(KAISER_BETA);
    let mut h: Vec<f64> = (0..taps)
        .map(|n| {
            let offset = n as f64 - center;
            let x = offset / f64::from(ratio);
            let sinc = if x == 0.0 {
                1.0
            } else {
                (std::f64::consts::PI * x).sin() / (std::f64::consts::PI * x)
            };
            let edge = offset / center;
            let window = bessel_i0(KAISER_BETA * (1.0 - edge * edge).max(0.0).sqrt()) / window_norm;
            sinc * window
        })
        .collect();
    let sum: f64 = h.iter().sum();
    for tap in &mut h {
        *tap /= sum;
    }
    h
}

impl RateFilter {
    /// The filter, or `None` for a bitstream built without one.
    pub fn resolve(map: &CsrMap) -> Result<Option<Self>, MapError> {
        if map.register(REG_RX_RATIO).is_err() {
            return Ok(None);
        }
        let constant = |name| -> Result<u32, MapError> {
            let value = map.constant_u64(name)?;
            u32::try_from(value).map_err(|_| MapError::ConstantNotInteger {
                name: name.to_string(),
                value: value.to_string(),
            })
        };
        Ok(Some(Self {
            rx_ratio: map.reg(REG_RX_RATIO, 1)?,
            tx_ratio: map.reg(REG_TX_RATIO, 1)?,
            coef_addr: map.reg(REG_COEF_ADDR, 1)?,
            coef_data: map.reg(REG_COEF_DATA, 1)?,
            phase_taps: constant(CONST_PHASE_TAPS)?,
            ratio_max: constant(CONST_RATIO_MAX)?,
            coef_bits: constant(CONST_COEF_BITS)?,
            rx_frac_bits: constant(CONST_RX_FRAC_BITS)?,
            tx_frac_bits: constant(CONST_TX_FRAC_BITS)?,
        }))
    }

    pub fn ratio_max(&self) -> u32 {
        self.ratio_max
    }

    /// Radio samples a direction's filter delays its samples by at `ratio`,
    /// to the nearest whole one.
    pub fn delay_ticks(&self, direction: Direction, ratio: u32) -> u32 {
        if ratio <= BYPASS_RATIO {
            return 0;
        }
        let half_length = self.phase_taps * ratio / 2;
        match direction {
            Direction::Rx => half_length + RX_PIPELINE_TICKS,
            Direction::Tx => half_length + ratio + TX_PIPELINE_TICKS,
        }
    }

    /// Both coefficient sets for `ratio`, quantized for the gateware. Transmit
    /// taps are scaled by the ratio because zero-stuffing divides the signal
    /// by it.
    pub fn coefficients(&self, ratio: u32) -> Result<Coefficients, RadioError> {
        let h = prototype(self.phase_taps, ratio);
        let limit = 1i64 << (self.coef_bits - 1);
        let quantize = |scale: f64| -> Result<Vec<i32>, RadioError> {
            h.iter()
                .map(|tap| {
                    let value = (tap * scale).round() as i64;
                    if value >= limit || value < -limit {
                        return Err(RadioError::Unavailable(format!(
                            "rate filter tap {value} does not fit {} bits",
                            self.coef_bits
                        )));
                    }
                    Ok(value as i32)
                })
                .collect()
        };
        Ok(Coefficients {
            rx: quantize(f64::from(1u32 << self.rx_frac_bits))?,
            tx: quantize(f64::from(ratio) * f64::from(1u32 << self.tx_frac_bits))?,
        })
    }

    /// `coef_addr` for coefficient `index` of a `ratio` filter: the index
    /// modulo the ratio in the low field, the quotient above it, then the
    /// direction bit.
    fn coef_address(&self, index: usize, ratio: u32, tx: bool) -> u64 {
        let phase_bits = bits_for(self.ratio_max - 1);
        let bank_bits = bits_for(self.phase_taps - 1);
        let index = index as u64;
        let ratio = u64::from(ratio);
        (index % ratio)
            | ((index / ratio) << phase_bits)
            | (u64::from(tx) << (phase_bits + bank_bits))
    }

    /// Loads the coefficients of each direction, then both ratios. Only safe
    /// while no stream runs, because the gateware reads the coefficients live.
    pub fn configure(
        &self,
        bus: &dyn CsrBus,
        rx_ratio: u32,
        tx_ratio: u32,
    ) -> Result<(), RadioError> {
        for ratio in [rx_ratio, tx_ratio] {
            if !(BYPASS_RATIO..=self.ratio_max).contains(&ratio) {
                return Err(RadioError::Unavailable(format!(
                    "rate filter ratio {ratio} is outside 1..={}",
                    self.ratio_max
                )));
            }
        }
        let mask = (1u64 << self.coef_bits) - 1;
        for (tx, ratio) in [(false, rx_ratio), (true, tx_ratio)] {
            if ratio == BYPASS_RATIO {
                continue;
            }
            let coefficients = self.coefficients(ratio)?;
            let set = if tx {
                &coefficients.tx
            } else {
                &coefficients.rx
            };
            for (index, value) in set.iter().enumerate() {
                self.coef_addr
                    .write(bus, self.coef_address(index, ratio, tx));
                self.coef_data.write(bus, (*value as i64 as u64) & mask);
            }
        }
        self.rx_ratio.write(bus, u64::from(rx_ratio));
        self.tx_ratio.write(bus, u64::from(tx_ratio));
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;

    fn filter() -> RateFilter {
        let reg = |addr| Reg { addr, words: 1 };
        RateFilter {
            rx_ratio: reg(0x0),
            tx_ratio: reg(0xc),
            coef_addr: reg(0x4),
            coef_data: reg(0x8),
            phase_taps: 24,
            ratio_max: 32,
            coef_bits: 18,
            rx_frac_bits: 17,
            tx_frac_bits: 16,
        }
    }

    /// Gain in dB at `frequency`, given as a fraction of the stream rate.
    fn response_db(taps: &[i32], scale: f64, ratio: u32, frequency: f64) -> f64 {
        let w = 2.0 * std::f64::consts::PI * frequency / f64::from(ratio);
        let (mut re, mut im) = (0.0, 0.0);
        for (n, tap) in taps.iter().enumerate() {
            let value = f64::from(*tap) / scale;
            re += value * (w * n as f64).cos();
            im -= value * (w * n as f64).sin();
        }
        10.0 * (re * re + im * im).log10()
    }

    #[test]
    fn every_ratio_meets_the_band_edges_in_both_directions() {
        let filter = filter();
        for ratio in [2, 3, 5, 8, 17, 32] {
            let coefficients = filter.coefficients(ratio).unwrap();
            assert_eq!(coefficients.rx.len(), 24 * ratio as usize);
            let sets = [
                (&coefficients.rx, f64::from(1u32 << 17)),
                (&coefficients.tx, f64::from(ratio) * f64::from(1u32 << 16)),
            ];
            for (taps, scale) in sets {
                let pass: Vec<f64> = (0..=42)
                    .map(|n| response_db(taps, scale, ratio, f64::from(n) / 100.0))
                    .collect();
                let ripple = pass.iter().cloned().fold(f64::MIN, f64::max)
                    - pass.iter().cloned().fold(f64::MAX, f64::min);
                assert!(ripple < 0.1, "ratio {ratio} ripple {ripple}");
                let worst = (0..2000)
                    .map(|n| 0.58 + (f64::from(ratio) / 2.0 - 0.58) * f64::from(n) / 2000.0)
                    .map(|f| response_db(taps, scale, ratio, f))
                    .fold(f64::MIN, f64::max);
                assert!(worst < -55.0, "ratio {ratio} stopband {worst}");
            }
        }
    }

    #[test]
    fn transmit_phases_each_have_unity_gain() {
        let filter = filter();
        for ratio in [2, 7, 32] {
            let tx = filter.coefficients(ratio).unwrap().tx;
            for phase in 0..ratio as usize {
                let sum: i32 = tx.iter().skip(phase).step_by(ratio as usize).sum();
                let gain = f64::from(sum) / f64::from(1u32 << 16);
                assert!(
                    (gain - 1.0).abs() < 0.01,
                    "ratio {ratio} phase {phase} gain {gain}"
                );
            }
        }
    }

    #[test]
    fn configure_writes_bank_ordered_coefficients_then_the_ratios() {
        let filter = filter();
        let bus = FakeBus::new();
        filter.configure(&bus, 3, 2).unwrap();
        let addrs = bus.writes_to(0x4);
        let data = bus.writes_to(0x8);
        assert_eq!(addrs.len(), 24 * 3 + 24 * 2);
        assert_eq!(data.len(), addrs.len());
        // Coefficient 7 of a ratio 3 filter is phase 1 of bank 2.
        assert_eq!(addrs[7], 1 | (2 << 5));
        // Coefficient 7 of a ratio 2 filter is phase 1 of bank 3.
        assert_eq!(addrs[24 * 3 + 7], 1 | (3 << 5) | (1 << 10));
        let rx = filter.coefficients(3).unwrap().rx;
        let tx = filter.coefficients(2).unwrap().tx;
        assert_eq!(data[7], (rx[7] as u32) & 0x3FFFF);
        assert_eq!(data[24 * 3 + 7], (tx[7] as u32) & 0x3FFFF);
        assert_eq!(bus.writes_to(0x0), vec![3]);
        assert_eq!(bus.writes_to(0xc), vec![2]);
    }

    #[test]
    fn the_filter_delay_follows_the_ratio() {
        let filter = filter();
        assert_eq!(filter.delay_ticks(Direction::Rx, 1), 0);
        assert_eq!(filter.delay_ticks(Direction::Tx, 1), 0);
        assert_eq!(filter.delay_ticks(Direction::Rx, 2), 26);
        assert_eq!(filter.delay_ticks(Direction::Rx, 4), 50);
        assert_eq!(filter.delay_ticks(Direction::Tx, 2), 28);
        assert_eq!(filter.delay_ticks(Direction::Tx, 8), 106);
    }

    #[test]
    fn a_bypassed_direction_loads_no_coefficients() {
        let filter = filter();
        let bus = FakeBus::new();
        filter.configure(&bus, 1, 1).unwrap();
        assert!(bus.writes_to(0x4).is_empty());
        assert_eq!(bus.writes_to(0x0), vec![1]);
        assert_eq!(bus.writes_to(0xc), vec![1]);
        filter.configure(&bus, 1, 4).unwrap();
        assert_eq!(bus.writes_to(0x4).len(), 24 * 4);
        assert!(filter.configure(&bus, 0, 1).is_err());
        assert!(filter.configure(&bus, 1, 33).is_err());
    }
}
