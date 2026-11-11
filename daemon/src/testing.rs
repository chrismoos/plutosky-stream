//! Stand-ins the crate's own tests, the integration tests and `--fake` share.
//!
//! Nothing here reaches hardware: the CSR bus is the in-memory fake with the
//! gateware model behind it, the slot rings are in memory, and the radio only
//! remembers what it was told.

use std::net::Ipv4Addr;
use std::sync::Arc;

use crate::csr::{CsrBus, CsrMap, FakeBus};
use crate::daemon::{Daemon, Identity};
use crate::gem::{FakeSlotDevice, Slots};
use crate::net::{FixedLinkResolver, LinkResolver, MacAddr};
use crate::radio::{FakeRadio, Radio};
use crate::stream::StreamManager;
use crate::time::Timebase;

/// The register map checked in beside the tests, which follows
/// `docs/csr-contract.md` until the gateware build produces its own.
pub const FIXTURE_CSR_JSON: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/csr.json");

/// Sample rate the fake gateware runs its timebase and packetizers at.
pub const FAKE_SAMPLE_RATE_HZ: u32 = 30_720_000;
/// Slots are the board's size so the ring arithmetic is the real thing. There
/// are fewer of them than the board's [`crate::gem::DEFAULT_SLOT_COUNT`], which
/// keeps the fake pool a cheap allocation while still leaving every stream
/// instance a useful window.
pub const FAKE_SLOT_SIZE: u32 = crate::gem::DEFAULT_SLOT_SIZE;
pub const FAKE_SLOT_COUNT: u32 = 512;

const FAKE_BOARD_MAC: MacAddr = MacAddr([0x00, 0x0a, 0x35, 0x00, 0x11, 0x22]);
const FAKE_HOST_MAC: MacAddr = MacAddr([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01]);
const FAKE_DEVICE: &str = "eth0";
const FAKE_SERIAL: &str = "fake-000000000000";

/// Loads the checked-in register map.
pub fn fixture_csr_map() -> CsrMap {
    CsrMap::from_path(std::path::Path::new(FIXTURE_CSR_JSON))
        .expect("the checked-in csr.json fixture parses")
}

/// Link addresses the fake daemon reports for any host.
pub fn fake_link_resolver() -> FixedLinkResolver {
    FixedLinkResolver {
        host_mac: FAKE_HOST_MAC,
        board_mac: FAKE_BOARD_MAC,
        device: FAKE_DEVICE.to_string(),
    }
}

/// Everything a fake daemon is built from, kept so a test can look at the CSR
/// writes and the slot rings afterwards.
pub struct FakeParts {
    pub bus: Arc<FakeBus>,
    pub slots: Arc<FakeSlotDevice>,
    pub map: CsrMap,
}

/// Builds a daemon whose every dependency is a fake, with the gateware model
/// installed so streams actually move slots.
pub fn fake_daemon(control_port: u16) -> (Arc<Daemon>, FakeParts) {
    let map = fixture_csr_map();
    let bus = Arc::new(FakeBus::new());
    crate::fake_gateware::install(&bus, &map, FAKE_SAMPLE_RATE_HZ)
        .expect("the fixture carries every register the gateware model needs");
    let slots = Arc::new(FakeSlotDevice::new(FAKE_SLOT_SIZE, FAKE_SLOT_COUNT));
    let links: Arc<dyn LinkResolver> = Arc::new(fake_link_resolver());
    let timebase = Timebase::resolve(&map).expect("the fixture carries the timebase registers");
    let streams = StreamManager::new(
        bus.clone() as Arc<dyn CsrBus>,
        slots.clone() as Arc<dyn Slots>,
        links,
        &map,
        timebase,
    )
    .expect("the fixture carries every stream register");
    let identity = Identity {
        serial: FAKE_SERIAL.to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        mac: FAKE_BOARD_MAC,
    };
    let radio: Box<dyn Radio> = Box::new(FakeRadio::new());
    let daemon = Arc::new(Daemon::new(radio, streams, identity, control_port));
    (daemon, FakeParts { bus, slots, map })
}

/// The address a fake stream is opened towards: the loopback host, so a test
/// can send the hello packet itself.
pub fn fake_host_ip() -> Ipv4Addr {
    Ipv4Addr::LOCALHOST
}
