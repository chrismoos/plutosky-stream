//! What opening and starting a stream writes to the gateware.
//!
//! The expected frame template is assembled here from `docs/wire-formats.md`
//! rather than from the daemon's own builder, so the two have to agree by
//! construction and not by sharing code.

use std::net::{Ipv4Addr, SocketAddr, UdpSocket};
use std::time::{Duration, Instant};

use plutosky_streamd::control::ApiError;
use plutosky_streamd::csr::FakeBus;
use plutosky_streamd::daemon::{Daemon, SessionId, StatusSnapshot};
use plutosky_streamd::gem::Slots;
use plutosky_streamd::net::MacAddr;
use plutosky_streamd::radio::Direction;
use plutosky_streamd::stream::StreamRequest;
use plutosky_streamd::stream::format::{Mtu, PacketPlan, WireFormat};
use plutosky_streamd::stream::header::{FrameEndpoints, frame_header};
use plutosky_streamd::stream::regs::{MAX_STREAMS, RxPacketizerRegs, TxDepacketizerRegs};
use plutosky_streamd::testing::{FAKE_SAMPLE_RATE_HZ, FAKE_SLOT_COUNT, fake_daemon};
use serde_json::{Value, json};

/// The fixed addresses the fake link resolver reports.
const BOARD_MAC: MacAddr = MacAddr([0x00, 0x0a, 0x35, 0x00, 0x11, 0x22]);
const HOST_MAC: MacAddr = MacAddr([0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0x01]);

const CONTROL_PORT: u16 = 30000;
const HOST_PORT: u16 = 40000;

/// A single test session owns the daemon for the duration of the test.
const TEST_SESSION: SessionId = 1;
const SECONDS_PER_DAY: u64 = 86_400;

/// Runs a stream operation with its parameters as a JSON object, and returns
/// the result the same way, so the tables below stay readable.
fn dispatch(daemon: &Daemon, op: &str, params: Value) -> Result<Value, ApiError> {
    let stream_id = || params["stream_id"].as_u64().expect("`stream_id`") as u32;
    match op {
        "open_stream" => {
            let direction = match params["dir"].as_str() {
                Some("rx") => Direction::Rx,
                Some("tx") => Direction::Tx,
                other => panic!("`dir` {other:?}"),
            };
            let request = StreamRequest {
                direction,
                channels: params["chans"]
                    .as_array()
                    .expect("`chans`")
                    .iter()
                    .map(|chan| chan.as_u64().unwrap() as usize)
                    .collect(),
                format: WireFormat::parse(params["format"].as_str().unwrap()).unwrap(),
                mtu: Mtu::parse(params["mtu"].as_u64().unwrap() as usize).unwrap(),
                host_ip: params["host_ip"].as_str().unwrap().parse().unwrap(),
                host_port: params["host_port"].as_u64().unwrap() as u16,
            };
            let opened = daemon.open_stream(TEST_SESSION, request)?;
            Ok(json!({
                "stream_id": opened.stream_id,
                "data_port": opened.data_port,
                "samples_per_packet": opened.samples_per_packet,
                "window": opened.window,
            }))
        }
        "start_stream" => {
            daemon.start_stream(TEST_SESSION, stream_id(), params["at_ticks"].as_u64())?;
            Ok(json!({"ok": true}))
        }
        "stop_stream" => {
            daemon.stop_stream(TEST_SESSION, stream_id())?;
            Ok(json!({"ok": true}))
        }
        "close_stream" => {
            daemon.close_stream(TEST_SESSION, stream_id())?;
            Ok(json!({"ok": true}))
        }
        other => panic!("no test dispatch for `{other}`"),
    }
}

fn call(daemon: &Daemon, op: &str, params: Value) -> Value {
    dispatch(daemon, op, params).unwrap_or_else(|error| panic!("{op}: {error}"))
}

fn status(daemon: &Daemon) -> StatusSnapshot {
    daemon
        .status(TEST_SESSION)
        .expect("the session has an open stream")
}

/// Sends the packet the host must send before a stream starts, which is what
/// puts it in the neighbour table, and returns the port it was sent from. The
/// daemon addresses frames to that observed port, not to the declared one.
fn send_hello(data_port: u16) -> u16 {
    let socket = UdpSocket::bind(SocketAddr::from(([127, 0, 0, 1], 0))).unwrap();
    socket
        .send_to(&[], SocketAddr::from(([127, 0, 0, 1], data_port)))
        .unwrap();
    socket.local_addr().unwrap().port()
}

/// Assembles the frame template straight from the spec: 14 Ethernet bytes,
/// 20 IPv4, 8 UDP and 24 VITA, zero-filled out to the register's 72 bytes.
fn expected_template(
    plan: &PacketPlan,
    stream_id: u32,
    board_port: u16,
    host_port: u16,
) -> [u32; 18] {
    let mut frame = [0u8; 72];
    frame[0..6].copy_from_slice(&HOST_MAC.octets());
    frame[6..12].copy_from_slice(&BOARD_MAC.octets());
    frame[12..14].copy_from_slice(&0x0800u16.to_be_bytes());

    let total_length = (plan.frame_len - 14) as u16;
    frame[14] = 0x45;
    frame[16..18].copy_from_slice(&total_length.to_be_bytes());
    frame[20..22].copy_from_slice(&0x4000u16.to_be_bytes());
    frame[22] = 64;
    frame[23] = 17;
    frame[26..30].copy_from_slice(&Ipv4Addr::LOCALHOST.octets());
    frame[30..34].copy_from_slice(&Ipv4Addr::LOCALHOST.octets());
    let checksum = ip_checksum(&frame[14..34]);
    frame[24..26].copy_from_slice(&checksum.to_be_bytes());

    frame[34..36].copy_from_slice(&board_port.to_be_bytes());
    frame[36..38].copy_from_slice(&host_port.to_be_bytes());
    frame[38..40].copy_from_slice(&(total_length - 20).to_be_bytes());

    let vita_header = 0x1000_0000u32 | 0x0800_0000 | 0x0010_0000 | plan.packet_words as u32;
    frame[42..46].copy_from_slice(&vita_header.to_be_bytes());
    frame[46..50].copy_from_slice(&stream_id.to_be_bytes());
    frame[54..58]
        .copy_from_slice(&(0x0001_0000 | u32::from(plan.format.packet_class())).to_be_bytes());

    let mut words = [0u32; 18];
    for (index, word) in words.iter_mut().enumerate() {
        let at = index * 4;
        *word = u32::from_be_bytes([frame[at], frame[at + 1], frame[at + 2], frame[at + 3]]);
    }
    words
}

fn ip_checksum(header: &[u8]) -> u16 {
    let mut sum: u32 = header
        .chunks(2)
        .map(|pair| u32::from(u16::from_be_bytes([pair[0], pair[1]])))
        .sum();
    while sum >> 16 != 0 {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    !(sum as u16)
}

fn words_written(bus: &FakeBus, regs: &RxPacketizerRegs) -> [u32; 18] {
    let mut words = [0u32; 18];
    for (index, word) in words.iter_mut().enumerate() {
        *word = bus.get(regs.header.addr + index as u64 * 4);
    }
    words
}

#[test]
fn opening_and_starting_a_receive_stream_programs_the_packetizer() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "rx",
            "chans": [0, 1],
            "format": "cs12",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    let data_port = opened["data_port"].as_u64().unwrap() as u16;
    assert_eq!(opened["samples_per_packet"], json!(480));
    let slots_per_stream = FAKE_SLOT_COUNT as usize / MAX_STREAMS;
    assert_eq!(opened["window"], json!(slots_per_stream - 64));

    let host_port = send_hello(data_port);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    let regs = RxPacketizerRegs::resolve(&parts.map, 0).unwrap();
    let bus = &*parts.bus;
    assert_eq!(bus.writes_to(regs.samples_per_packet.addr), vec![480]);
    assert_eq!(bus.writes_to(regs.frame_len.addr), vec![1506]);
    assert_eq!(bus.writes_to(regs.slot_first.addr), vec![0]);
    assert_eq!(
        bus.writes_to(regs.slot_count.addr),
        vec![slots_per_stream as u32]
    );
    assert_eq!(bus.writes_to(regs.start_hi.addr), vec![0]);
    assert_eq!(bus.writes_to(regs.start_lo.addr), vec![0]);
    // enable, no arm, cs12 format, both channels.
    assert_eq!(
        bus.writes_to(regs.ctrl.addr).last().copied(),
        Some(0b11_00_01)
    );

    let plan = PacketPlan::for_channels(WireFormat::Cs12, Mtu::Standard, 2);
    assert_eq!(
        words_written(&parts.bus, &regs),
        expected_template(&plan, stream_id, data_port, host_port)
    );

    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
    call(&daemon, "close_stream", json!({"stream_id": stream_id}));
}

#[test]
fn a_scheduled_start_arms_the_packetizer_at_the_requested_tick() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "rx",
            "chans": [0],
            "format": "bfp8",
            "mtu": 9000,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    assert_eq!(opened["samples_per_packet"], json!(4336));
    send_hello(opened["data_port"].as_u64().unwrap() as u16);

    let at_ticks: u64 = 0x0000_0002_1234_5678;
    call(
        &daemon,
        "start_stream",
        json!({"stream_id": stream_id, "at_ticks": at_ticks}),
    );

    let regs = RxPacketizerRegs::resolve(&parts.map, 0).unwrap();
    assert_eq!(parts.bus.writes_to(regs.start_hi.addr), vec![2]);
    assert_eq!(parts.bus.writes_to(regs.start_lo.addr), vec![0x1234_5678]);
    // enable and arm, bfp8 format, channel 0 only.
    assert_eq!(
        parts.bus.writes_to(regs.ctrl.addr).last().copied(),
        Some(0b01_10_11)
    );
    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
}

#[test]
fn a_transmit_stream_claims_its_stream_id_and_steers_its_port() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "tx",
            "chans": [0, 1],
            "format": "cs8",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    let data_port = opened["data_port"].as_u64().unwrap() as u16;
    assert_eq!(opened["samples_per_packet"], json!(722));
    send_hello(data_port);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    assert_eq!(parts.slots.steered_ports(), vec![data_port]);
    let regs = TxDepacketizerRegs::resolve(&parts.map, 0).unwrap();
    assert_eq!(parts.bus.writes_to(regs.stream_id.addr), vec![stream_id]);
    // enable, no arm, cs8 format, both channels.
    assert_eq!(
        parts.bus.writes_to(regs.ctrl.addr).last().copied(),
        Some(0b11_01_01)
    );

    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
    assert!(
        parts.slots.steered_ports().is_empty(),
        "stopping a stream stops steering its port"
    );
}

#[test]
fn failed_steering_disable_can_be_retried_on_close() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "tx", "chans": [0], "format": "cs12", "mtu": 1500,
            "host_ip": "127.0.0.1", "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    let data_port = opened["data_port"].as_u64().unwrap() as u16;
    send_hello(data_port);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    parts.slots.fail_next_steer_disable();
    let error = dispatch(&daemon, "stop_stream", json!({"stream_id": stream_id})).unwrap_err();
    assert!(error.message.contains("disable steering"));
    assert_eq!(parts.slots.steered_ports(), vec![data_port]);

    call(&daemon, "close_stream", json!({"stream_id": stream_id}));
    assert!(parts.slots.steered_ports().is_empty());
    let regs = TxDepacketizerRegs::resolve(&parts.map, 0).unwrap();
    assert_eq!(parts.bus.get(regs.ctrl.addr) & 1, 0);
}

#[test]
fn a_scheduled_transmit_start_answers_at_once_and_leaves_the_core_stopped() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "tx",
            "chans": [0],
            "format": "cs12",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap();
    send_hello(opened["data_port"].as_u64().unwrap() as u16);

    // Far enough ahead that the fake timebase will not reach it during the
    // test: a day of ticks at the fake sample rate.
    let at_ticks = SECONDS_PER_DAY * u64::from(FAKE_SAMPLE_RATE_HZ);
    let started = Instant::now();
    call(
        &daemon,
        "start_stream",
        json!({"stream_id": stream_id, "at_ticks": at_ticks}),
    );
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "start_stream must not hold the session until the tick"
    );

    let regs = TxDepacketizerRegs::resolve(&parts.map, 0).unwrap();
    // Every write so far is the stop the manager issues at startup.
    assert!(
        parts
            .bus
            .writes_to(regs.ctrl.addr)
            .iter()
            .all(|&value| value == 0),
        "the depacketizer stays stopped until the clock reaches the tick"
    );
    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
}

#[test]
fn the_golden_template_matches_the_bytes_the_spec_describes() {
    let plan = PacketPlan::new(WireFormat::Cs12, Mtu::Standard);
    let endpoints = FrameEndpoints {
        board_mac: BOARD_MAC,
        host_mac: HOST_MAC,
        board_ip: Ipv4Addr::new(192, 168, 2, 1),
        host_ip: Ipv4Addr::new(192, 168, 2, 10),
        board_port: 30100,
        host_port: 40000,
    };
    let header = frame_header(&endpoints, &plan, 7);
    // One line per protocol layer, as `docs/wire-formats.md` lays them out:
    // Ethernet to aa:bb:cc:dd:ee:01 from 00:0a:35:00:11:22, IPv4 total 1496
    // with Don't Fragment, TTL 64, UDP, checksum afb9, 192.168.2.1 to
    // 192.168.2.10, UDP 30100 to 40000 of length 1476 with no checksum, and a
    // VITA header of 367 words for stream 7, class 0001/0001, zero timestamp.
    let expected = concat!(
        "aabbccddee01000a350011220800",
        "450005d8000040004011afb9c0a80201c0a8020a",
        "75949c4005c40000",
        "1810016f0000000700000000000100010000000000000000",
    );
    assert_eq!(hex(&header), expected);
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[test]
fn every_format_and_mtu_pair_reports_the_documented_samples_per_packet() {
    // Samples per packet for every format and MTU.
    let table = [
        ("cs12", 1500usize, 481usize),
        ("cs8", 1500, 722),
        ("bfp8", 1500, 688),
        ("cs12", 9000, 2981),
        ("cs8", 9000, 4472),
        ("bfp8", 9000, 4336),
    ];
    let (daemon, _parts) = fake_daemon(CONTROL_PORT);
    for (format, mtu, expected) in table {
        let opened = call(
            &daemon,
            "open_stream",
            json!({
                "dir": "rx",
                "chans": [0],
                "format": format,
                "mtu": mtu,
                "host_ip": "127.0.0.1",
                "host_port": HOST_PORT,
            }),
        );
        assert_eq!(
            opened["samples_per_packet"],
            json!(expected),
            "{format} at MTU {mtu}"
        );
        let plan = PacketPlan::new(WireFormat::parse(format).unwrap(), Mtu::parse(mtu).unwrap());
        assert!(
            plan.frame_len - 14 <= mtu,
            "{format} at MTU {mtu}: the IP packet is {} bytes",
            plan.frame_len - 14
        );
        call(
            &daemon,
            "close_stream",
            json!({"stream_id": opened["stream_id"]}),
        );
    }
}

#[test]
fn kernel_receive_stream_uses_driver_pump_without_a_daemon_slot_thread() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    parts.slots.set_kernel_mode(true);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "rx",
            "chans": [0],
            "format": "cs12",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    send_hello(opened["data_port"].as_u64().unwrap() as u16);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    let config = parts.slots.kernel_config(0).unwrap();
    let regs = RxPacketizerRegs::resolve(&parts.map, 0).unwrap();
    let csr_base = parts.map.io_span().unwrap().start as u32;
    assert_eq!(config.produced, regs.produced.addr as u32 - csr_base);
    assert_eq!(config.observed, regs.observed.addr as u32 - csr_base);
    assert_eq!(config.released, regs.released.addr as u32 - csr_base);
    assert_eq!(
        config.frame_len,
        PacketPlan::new(WireFormat::Cs12, Mtu::Standard).frame_len as u32
    );
    assert!(parts.slots.transmits_in_flight().is_empty());

    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
    assert_eq!(parts.slots.stream_status(0).unwrap().active, 0);
}

#[test]
fn receive_and_transmit_share_an_instance_without_sharing_slots() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    parts.slots.set_kernel_mode(true);
    let open = |dir: &str| {
        call(
            &daemon,
            "open_stream",
            json!({
                "dir": dir,
                "chans": [0],
                "format": "cs12",
                "mtu": 1500,
                "host_ip": "127.0.0.1",
                "host_port": HOST_PORT,
            }),
        )
    };
    let rx = open("rx");
    let tx = open("tx");
    let rx_id = rx["stream_id"].as_u64().unwrap() as u32;
    let tx_id = tx["stream_id"].as_u64().unwrap() as u32;
    send_hello(rx["data_port"].as_u64().unwrap() as u16);
    send_hello(tx["data_port"].as_u64().unwrap() as u16);
    call(&daemon, "start_stream", json!({"stream_id": rx_id}));
    call(&daemon, "start_stream", json!({"stream_id": tx_id}));
    assert_eq!(parts.slots.kernel_config(0).unwrap().instance, 0);
    assert_eq!(parts.slots.kernel_config(1).unwrap().instance, 0);
    call(&daemon, "stop_stream", json!({"stream_id": tx_id}));
    call(&daemon, "stop_stream", json!({"stream_id": rx_id}));
}

#[test]
fn kernel_transmit_stream_steers_its_port_and_stops_with_the_core() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    parts.slots.set_kernel_mode(true);
    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "tx",
            "chans": [0],
            "format": "cs12",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    let data_port = opened["data_port"].as_u64().unwrap() as u16;
    send_hello(data_port);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    let config = parts.slots.kernel_config(1).unwrap();
    let regs = TxDepacketizerRegs::resolve(&parts.map, 0).unwrap();
    let csr_base = parts.map.io_span().unwrap().start as u32;
    assert_eq!(config.submit, regs.submit.addr as u32 - csr_base);
    assert_eq!(
        config.tx_irq_threshold,
        regs.irq_threshold.addr as u32 - csr_base
    );
    assert_eq!(config.udp_port, u32::from(data_port));
    assert_eq!(parts.slots.steered_ports(), vec![data_port]);

    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
    assert!(parts.slots.steered_ports().is_empty());
    assert_eq!(parts.slots.stream_status(1).unwrap().active, 0);
    assert_eq!(parts.bus.get(regs.ctrl.addr), 0b0001_0000);
}

#[test]
fn kernel_transmit_status_excludes_errors_from_previous_streams() {
    let (daemon, parts) = fake_daemon(CONTROL_PORT);
    parts.slots.set_kernel_mode(true);
    let regs = TxDepacketizerRegs::resolve(&parts.map, 0).unwrap();
    parts.bus.set(regs.late.addr, 77);
    parts.bus.set(regs.underrun.addr, 99);
    parts.bus.set(regs.gaps.addr, 11);

    let opened = call(
        &daemon,
        "open_stream",
        json!({
            "dir": "tx",
            "chans": [0],
            "format": "cs12",
            "mtu": 1500,
            "host_ip": "127.0.0.1",
            "host_port": HOST_PORT,
        }),
    );
    let stream_id = opened["stream_id"].as_u64().unwrap() as u32;
    send_hello(opened["data_port"].as_u64().unwrap() as u16);
    call(&daemon, "start_stream", json!({"stream_id": stream_id}));

    let counters = &status(&daemon).streams[0].counters;
    assert_eq!((counters.late, counters.underrun, counters.gaps), (0, 0, 0));

    parts.bus.set(regs.late.addr, 79);
    parts.bus.set(regs.underrun.addr, 100);
    parts.bus.set(regs.gaps.addr, 14);
    let counters = &status(&daemon).streams[0].counters;
    assert_eq!((counters.late, counters.underrun, counters.gaps), (2, 1, 3));

    call(&daemon, "stop_stream", json!({"stream_id": stream_id}));
    call(&daemon, "close_stream", json!({"stream_id": stream_id}));
}
