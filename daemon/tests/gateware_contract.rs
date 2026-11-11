//! The daemon against the register map the gateware build actually exports.
//!
//! `daemon/tests/fixtures/csr.json` is written to `docs/csr-contract.md`, so a
//! test against it only proves the daemon agrees with the daemon. This one
//! resolves every register the daemon needs out of
//! `gateware/build_artifacts/csr.json`, which is committed for exactly this
//! purpose, and catches the two maps drifting apart.

use std::path::PathBuf;
use std::sync::Arc;

use plutosky_streamd::csr::{CsrBus, CsrMap, FakeBus};
use plutosky_streamd::gem::{FakeSlotDevice, Slots};
use plutosky_streamd::net::LinkResolver;
use plutosky_streamd::stream::StreamManager;
use plutosky_streamd::stream::header::HEADER_TEMPLATE_WORDS;
use plutosky_streamd::stream::regs::{
    MAX_STREAMS, RxPacketizerRegs, SlotPoolRegs, TxDepacketizerRegs,
};
use plutosky_streamd::testing::{FAKE_SLOT_COUNT, FAKE_SLOT_SIZE, fake_link_resolver};
use plutosky_streamd::time::Timebase;

fn gateware_map() -> CsrMap {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../gateware/build_artifacts/csr.json");
    CsrMap::from_path(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
}

/// Instances the map exports, counted the way the stream manager counts them.
fn instances(map: &CsrMap) -> usize {
    (0..MAX_STREAMS)
        .take_while(|index| RxPacketizerRegs::resolve(map, *index).is_ok())
        .count()
}

#[test]
fn every_register_the_daemon_needs_is_in_the_gateware_map() {
    let map = gateware_map();
    Timebase::resolve(&map).expect("timebase registers");
    SlotPoolRegs::resolve(&map).expect("slot pool registers");

    let instances = instances(&map);
    assert!(instances > 0, "the map exports no stream instances");
    for index in 0..instances {
        let packetizer = RxPacketizerRegs::resolve(&map, index)
            .unwrap_or_else(|error| panic!("rxpkt{index}: {error}"));
        TxDepacketizerRegs::resolve(&map, index)
            .unwrap_or_else(|error| panic!("txpkt{index}: {error}"));
        assert_eq!(
            packetizer.header.words, HEADER_TEMPLATE_WORDS as u32,
            "rxpkt{index}_header is not the 72-byte template the daemon writes"
        );
    }
}

#[cfg(feature = "hardware")]
#[test]
fn the_ad9361_registers_are_in_the_gateware_map() {
    use plutosky_streamd::ad9361::Ad9361Regs;

    Ad9361Regs::resolve(&gateware_map()).expect("AD9361 registers");
}

#[test]
fn the_rate_filter_matches_the_coefficient_layout_the_daemon_writes() {
    use plutosky_streamd::rate_filter::RateFilter;

    let filter = RateFilter::resolve(&gateware_map())
        .expect("rate filter registers")
        .expect("the shipped bitstream has the rate filter");
    assert_eq!(filter.ratio_max(), 32);
    let coefficients = filter.coefficients(filter.ratio_max()).unwrap();
    assert_eq!(coefficients.rx.len(), 24 * 32);
}

#[test]
fn the_stream_manager_comes_up_on_the_gateware_map() {
    let map = gateware_map();
    let bus = Arc::new(FakeBus::new()) as Arc<dyn CsrBus>;
    let slots = Arc::new(FakeSlotDevice::new(FAKE_SLOT_SIZE, FAKE_SLOT_COUNT)) as Arc<dyn Slots>;
    let links = Arc::new(fake_link_resolver()) as Arc<dyn LinkResolver>;
    let timebase = Timebase::resolve(&map).unwrap();
    let manager = StreamManager::new(bus, slots, links, &map, timebase)
        .expect("the daemon comes up on the gateware's own register map");
    assert_eq!(
        manager.instances(),
        instances(&map),
        "the manager takes every instance the bitstream built"
    );
}

#[test]
fn the_soc_identity_registers_carry_the_names_the_daemon_reads() {
    let map = gateware_map();
    map.reg(plutosky_streamd::csr::REG_SOC_MAGIC, 1)
        .expect("soc magic register");
    map.reg(plutosky_streamd::csr::REG_SOC_VERSION, 1)
        .expect("soc version register");
}
