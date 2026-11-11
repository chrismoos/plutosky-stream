//! Align both LVDS directions using PRBS at the configured sample rate.
//! RX BIST selects the receive eye first. TX loopback then selects the transmit
//! eye with RX delays fixed, and both paths return to normal before completion.

use crate::radio::RadioError;

pub(super) const DELAY_TAPS: usize = 16;
// The selected setting needs a passing delay tap on either side.
const MIN_EYE_TAPS: usize = 3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Direction {
    Rx,
    Tx,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum TestMode {
    Normal,
    RxPrbs,
    TxLoopback,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Delay {
    pub clock_taps: u8,
    pub data_taps: u8,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Eye {
    pub delay: Delay,
    pub width_taps: usize,
}

pub(super) struct Calibration {
    pub rx: Eye,
    pub tx: Eye,
}

pub(super) trait InterfaceIo {
    fn delay(&mut self, direction: Direction) -> Result<Delay, RadioError>;
    fn set_delay(&mut self, direction: Direction, delay: Delay) -> Result<(), RadioError>;
    fn set_test_mode(&mut self, mode: TestMode) -> Result<(), RadioError>;
    fn prbs_locked(&mut self) -> bool;
}

fn widest_run(row: &[bool; DELAY_TAPS]) -> Option<(usize, usize)> {
    let mut best = None;
    let mut start = 0;
    let mut width = 0;
    for (index, &locked) in row.iter().enumerate() {
        if !locked {
            width = 0;
            continue;
        }
        if width == 0 {
            start = index;
        }
        width += 1;
        if best.is_none_or(|(_, best_width)| width > best_width) {
            best = Some((start + width / 2, width));
        }
    }
    best.filter(|&(_, width)| width >= MIN_EYE_TAPS)
}

fn sweep(io: &mut dyn InterfaceIo, direction: Direction) -> Result<Eye, RadioError> {
    let mut best: Option<Eye> = None;
    for clock_taps in 0..DELAY_TAPS as u8 {
        let mut row = [false; DELAY_TAPS];
        for (data_taps, locked) in row.iter_mut().enumerate() {
            io.set_delay(
                direction,
                Delay {
                    clock_taps,
                    data_taps: data_taps as u8,
                },
            )?;
            *locked = io.prbs_locked();
        }
        log::debug!("AD9361 {direction:?} delay sweep: clock_taps={clock_taps} locked={row:?}");
        if let Some((data_taps, width_taps)) = widest_run(&row)
            && best.is_none_or(|eye| width_taps > eye.width_taps)
        {
            best = Some(Eye {
                delay: Delay {
                    clock_taps,
                    data_taps: data_taps as u8,
                },
                width_taps,
            });
        }
    }
    let eye = best.ok_or_else(|| RadioError::Unavailable(format!(
        "AD9361 {direction:?} LVDS calibration found no PRBS eye at least {MIN_EYE_TAPS} delay taps wide"
    )))?;
    io.set_delay(direction, eye.delay)?;
    if !io.prbs_locked() {
        return Err(RadioError::Unavailable(format!(
            "AD9361 {direction:?} LVDS calibration lost PRBS lock at the selected delay {delay:?}",
            delay = eye.delay
        )));
    }
    Ok(eye)
}

pub(super) fn calibrate(io: &mut dyn InterfaceIo) -> Result<Calibration, RadioError> {
    let saved_rx = io.delay(Direction::Rx)?;
    let saved_tx = io.delay(Direction::Tx)?;
    let result = (|| {
        io.set_test_mode(TestMode::RxPrbs)?;
        let rx = sweep(io, Direction::Rx)?;
        io.set_test_mode(TestMode::TxLoopback)?;
        let tx = sweep(io, Direction::Tx)?;
        Ok(Calibration { rx, tx })
    })();
    let cleanup = io.set_test_mode(TestMode::Normal);
    if result.is_err() || cleanup.is_err() {
        let restore_rx = io.set_delay(Direction::Rx, saved_rx);
        let restore_tx = io.set_delay(Direction::Tx, saved_tx);
        restore_rx?;
        restore_tx?;
    }
    cleanup?;
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAVED_RX: Delay = Delay {
        clock_taps: 0,
        data_taps: 4,
    };
    const SAVED_TX: Delay = Delay {
        clock_taps: 7,
        data_taps: 0,
    };
    const RX_EYE: Delay = Delay {
        clock_taps: 3,
        data_taps: 6,
    };
    const TX_EYE: Delay = Delay {
        clock_taps: 11,
        data_taps: 9,
    };

    struct Device {
        rx: Delay,
        tx: Delay,
        mode: TestMode,
        modes: Vec<TestMode>,
        missing_eye: Option<Direction>,
        fail_tx_write: bool,
        tx_measurements: usize,
    }

    impl Device {
        fn new() -> Self {
            Self {
                rx: SAVED_RX,
                tx: SAVED_TX,
                mode: TestMode::Normal,
                modes: Vec::new(),
                missing_eye: None,
                fail_tx_write: false,
                tx_measurements: 0,
            }
        }
    }

    impl InterfaceIo for Device {
        fn delay(&mut self, direction: Direction) -> Result<Delay, RadioError> {
            Ok(match direction {
                Direction::Rx => self.rx,
                Direction::Tx => self.tx,
            })
        }

        fn set_delay(&mut self, direction: Direction, delay: Delay) -> Result<(), RadioError> {
            if direction == Direction::Tx && self.fail_tx_write {
                self.fail_tx_write = false;
                return Err(RadioError::Driver {
                    op: "set_tx_delay",
                    code: -1,
                });
            }
            match direction {
                Direction::Rx => self.rx = delay,
                Direction::Tx => self.tx = delay,
            }
            Ok(())
        }

        fn set_test_mode(&mut self, mode: TestMode) -> Result<(), RadioError> {
            self.mode = mode;
            self.modes.push(mode);
            Ok(())
        }

        fn prbs_locked(&mut self) -> bool {
            match self.mode {
                TestMode::RxPrbs => {
                    self.missing_eye != Some(Direction::Rx)
                        && self.rx.clock_taps == RX_EYE.clock_taps
                        && (2..10).contains(&self.rx.data_taps)
                }
                TestMode::TxLoopback => {
                    assert_eq!(
                        self.rx, RX_EYE,
                        "RX timing must stay fixed throughout TX calibration"
                    );
                    self.tx_measurements += 1;
                    self.missing_eye != Some(Direction::Tx)
                        && self.tx.clock_taps == TX_EYE.clock_taps
                        && (4..14).contains(&self.tx.data_taps)
                }
                TestMode::Normal => false,
            }
        }
    }

    #[test]
    fn successful_rx_always_calibrates_tx_and_restores_normal_operation() {
        let mut device = Device::new();
        let result = calibrate(&mut device).unwrap();
        assert_eq!((result.rx.delay, result.tx.delay), (RX_EYE, TX_EYE));
        assert_eq!((result.rx.width_taps, result.tx.width_taps), (8, 10));
        assert_eq!(device.tx_measurements, DELAY_TAPS * DELAY_TAPS + 1);
        assert_eq!(
            device.modes,
            [TestMode::RxPrbs, TestMode::TxLoopback, TestMode::Normal]
        );
    }

    #[test]
    fn failed_eye_restores_delays_and_disables_test_sources() {
        for direction in [Direction::Rx, Direction::Tx] {
            let mut device = Device::new();
            device.missing_eye = Some(direction);
            assert!(calibrate(&mut device).is_err());
            assert_eq!(
                (device.rx, device.tx, device.mode),
                (SAVED_RX, SAVED_TX, TestMode::Normal)
            );
            if direction == Direction::Rx {
                assert_eq!(device.tx_measurements, 0);
            }
        }
    }

    #[test]
    fn spi_failure_restores_delays_and_disables_test_sources() {
        let mut device = Device::new();
        device.fail_tx_write = true;
        assert!(calibrate(&mut device).is_err());
        assert_eq!(
            (device.rx, device.tx, device.mode),
            (SAVED_RX, SAVED_TX, TestMode::Normal)
        );
    }

    #[test]
    fn isolated_locks_and_separate_edge_runs_do_not_form_a_valid_eye() {
        let mut row = [false; DELAY_TAPS];
        row[0] = true;
        row[1] = true;
        row[DELAY_TAPS - 1] = true;
        assert_eq!(widest_run(&row), None);
        row[5..10].fill(true);
        assert_eq!(widest_run(&row), Some((7, 5)));
    }
}
