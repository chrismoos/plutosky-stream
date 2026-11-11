//! Radio control between sessions, driven on the daemon directly.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::Duration;

use plutosky_streamd::testing::fake_daemon;

const ROUNDS: usize = 2000;
const DEADLOCK_TIMEOUT: Duration = Duration::from_secs(20);

#[test]
fn status_ticks_and_takeovers_from_two_sessions_do_not_deadlock() {
    let (daemon, _parts) = fake_daemon(0);
    let stop = Arc::new(AtomicBool::new(false));
    let ticker = {
        let daemon = daemon.clone();
        let stop = stop.clone();
        std::thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                let _ = daemon.status(1);
                let _ = daemon.status(2);
            }
        })
    };
    let (done, finished) = mpsc::channel();
    let claimer = {
        let daemon = daemon.clone();
        std::thread::spawn(move || {
            for round in 0..ROUNDS {
                let session = 1 + (round % 2) as u64;
                daemon
                    .set_loopback(session, false)
                    .expect("an idle controller gives way");
            }
            done.send(()).unwrap();
        })
    };
    let result = finished.recv_timeout(DEADLOCK_TIMEOUT);
    stop.store(true, Ordering::Relaxed);
    assert!(result.is_ok(), "status and a takeover deadlocked");
    claimer.join().unwrap();
    ticker.join().unwrap();
}
