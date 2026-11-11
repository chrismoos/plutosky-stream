//! The gRPC control service end to end, over a real TCP socket against a
//! daemon whose every hardware dependency is a fake.

use std::net::{SocketAddr, UdpSocket};
use std::sync::Arc;
use std::time::{Duration, Instant};

use prost::Message;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::transport::Channel;
use tonic::{Code, Request, Streaming};

use plutosky_streamd::control::proto::control_client::ControlClient;
use plutosky_streamd::control::proto::session_command::Command;
use plutosky_streamd::control::proto::session_event::Event;
use plutosky_streamd::control::proto::{self as pb, Direction, GainMode, SampleFormat};
use plutosky_streamd::control::{
    ControlService, DISCOVERY_PROBE, DiscoveryResponder, SESSION_METADATA_KEY, serve_on,
};
use plutosky_streamd::daemon::{DAEMON_NAME, Daemon};
use plutosky_streamd::testing::fake_daemon;

/// How long a test waits for a reply or an event before failing.
const REPLY_TIMEOUT: Duration = Duration::from_secs(5);
/// How long a test waits for status to report stream progress.
const STATUS_TIMEOUT: Duration = Duration::from_secs(3);
const HOST_PORT: u32 = 40000;

struct Fixture {
    address: SocketAddr,
    daemon: Arc<Daemon>,
}

async fn start_daemon() -> Fixture {
    let (daemon, _parts) = fake_daemon(0);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let address = listener.local_addr().unwrap();
    tokio::spawn(serve_on(ControlService::new(daemon.clone()), listener));
    Fixture { address, daemon }
}

async fn connect(fixture: &Fixture) -> ControlClient<Channel> {
    ControlClient::connect(format!("http://{}", fixture.address))
        .await
        .expect("the daemon accepts the connection")
}

/// An open Session call. Dropping it ends the session.
struct Session {
    id: String,
    events: Streaming<pb::SessionEvent>,
    commands: mpsc::Sender<pb::SessionCommand>,
}

impl Session {
    async fn open(client: &mut ControlClient<Channel>) -> Self {
        let (commands, receiver) = mpsc::channel(4);
        commands
            .send(pb::SessionCommand {
                command: Some(Command::Start(pb::SessionStart {
                    client: "test".to_string(),
                })),
            })
            .await
            .unwrap();
        let mut events = client
            .session(ReceiverStream::new(receiver))
            .await
            .expect("the session opens")
            .into_inner();
        let id = match events.message().await.unwrap() {
            Some(pb::SessionEvent {
                event: Some(Event::Opened(opened)),
            }) => {
                assert_eq!(opened.status_interval_ms, 10);
                opened.session_id
            }
            other => panic!("the session did not open: {other:?}"),
        };
        Self {
            id,
            events,
            commands,
        }
    }

    fn req<T>(&self, message: T) -> Request<T> {
        let mut request = Request::new(message);
        request
            .metadata_mut()
            .insert(SESSION_METADATA_KEY, self.id.parse().unwrap());
        request
    }

    /// Waits for a status tick whose stream passes `ready`.
    async fn status_until(
        &mut self,
        stream_id: u32,
        ready: impl Fn(&pb::StreamStatus) -> bool,
    ) -> (u64, pb::StreamStatus) {
        let deadline = Instant::now() + STATUS_TIMEOUT;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let event = tokio::time::timeout(remaining, self.events.message())
                .await
                .expect("a status tick arrives in time")
                .unwrap()
                .expect("the session stays open");
            if let Some(Event::Status(tick)) = event.event
                && let Some(stream) = tick.streams.iter().find(|s| s.stream_id == stream_id)
                && ready(stream)
            {
                return (tick.ticks, stream.clone());
            }
        }
    }
}

fn send_hello_packet(data_port: u32) {
    let socket = UdpSocket::bind(SocketAddr::from(([127, 0, 0, 1], 0))).unwrap();
    socket
        .send_to(&[], SocketAddr::from(([127, 0, 0, 1], data_port as u16)))
        .unwrap();
}

fn open_request(dir: Direction, chans: Vec<u32>) -> pb::OpenStreamRequest {
    pb::OpenStreamRequest {
        dir: dir as i32,
        chans,
        format: SampleFormat::Cs12 as i32,
        mtu: 1500,
        host_ip: "127.0.0.1".to_string(),
        host_port: HOST_PORT,
    }
}

fn channel(dir: Direction, chan: u32) -> pb::ChannelRef {
    pb::ChannelRef {
        dir: dir as i32,
        chan,
    }
}

async fn tx_rf_enabled(client: &mut ControlClient<Channel>) -> bool {
    client
        .get_sensors(pb::Empty {})
        .await
        .unwrap()
        .into_inner()
        .tx_rf_enabled
}

async fn tx_gain(client: &mut ControlClient<Channel>, chan: u32) -> f64 {
    client
        .get_gain(channel(Direction::Tx, chan))
        .await
        .unwrap()
        .into_inner()
        .db
}

/// Opens and starts a stream, returning its id.
async fn start_stream(
    client: &mut ControlClient<Channel>,
    session: &Session,
    request: pb::OpenStreamRequest,
) -> u32 {
    let opened = client
        .open_stream(session.req(request))
        .await
        .unwrap()
        .into_inner();
    send_hello_packet(opened.data_port);
    client
        .start_stream(session.req(pb::StartStreamRequest {
            stream_id: opened.stream_id,
            at_ticks: None,
        }))
        .await
        .unwrap();
    opened.stream_id
}

#[tokio::test(flavor = "multi_thread")]
async fn get_info_reports_what_the_board_can_do() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let info = client
        .get_info(pb::GetInfoRequest {
            client: "test".to_string(),
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(info.channels, 2);
    assert_eq!(info.min_rate_hz, 520_834);
    assert_eq!(info.max_rate_hz, 30_720_000);
    assert_eq!(
        info.formats,
        vec![
            SampleFormat::Cs12 as i32,
            SampleFormat::Cs8 as i32,
            SampleFormat::Bfp8 as i32
        ]
    );
    assert_eq!(info.mtus, vec![1500, 9000]);
    assert_eq!(info.serial, fixture.daemon.identity().serial);
}

#[tokio::test(flavor = "multi_thread")]
async fn every_radio_call_round_trips_and_reports_the_applied_value() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    let set = client
        .set_frequency(session.req(pb::SetFrequencyRequest {
            dir: Direction::Rx as i32,
            chan: 0,
            hz: 915_000_000,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(set.hz, 915_000_000);
    let got = client
        .get_frequency(channel(Direction::Rx, 1))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(got.hz, 915_000_000, "one synthesizer per direction");

    let rate = client
        .set_sample_rate(session.req(pb::SetSampleRateRequest {
            hz: 15_360_000,
            dir: 0,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(rate.hz, 15_360_000);
    assert_eq!(
        client
            .get_sample_rate(pb::Empty {})
            .await
            .unwrap()
            .into_inner()
            .hz,
        15_360_000
    );
    let too_fast = client
        .set_sample_rate(session.req(pb::SetSampleRateRequest {
            hz: 30_720_001,
            dir: 0,
        }))
        .await
        .unwrap_err();
    assert_eq!(too_fast.code(), Code::OutOfRange);

    let bandwidth = client
        .set_bandwidth(session.req(pb::SetBandwidthRequest {
            dir: Direction::Tx as i32,
            chan: 0,
            hz: 18_000_000,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(bandwidth.hz, 18_000_000);
    assert_eq!(
        client
            .get_bandwidth(channel(Direction::Tx, 0))
            .await
            .unwrap()
            .into_inner()
            .hz,
        18_000_000
    );

    for (dir, chan, db) in [(Direction::Rx, 1, 42.0), (Direction::Tx, 0, 69.75)] {
        let set = client
            .set_gain(session.req(pb::SetGainRequest {
                dir: dir as i32,
                chan,
                db,
            }))
            .await
            .unwrap()
            .into_inner();
        assert_eq!(set.db, db);
        let got = client
            .get_gain(channel(dir, chan))
            .await
            .unwrap()
            .into_inner();
        assert_eq!(got.db, db);
    }
    let negative = client
        .set_gain(session.req(pb::SetGainRequest {
            dir: Direction::Tx as i32,
            chan: 0,
            db: -20.0,
        }))
        .await
        .unwrap_err();
    assert_eq!(negative.code(), Code::OutOfRange);
    assert_eq!(tx_gain(&mut client, 0).await, 69.75);

    let mode = client
        .set_gain_mode(session.req(pb::SetGainModeRequest {
            dir: Direction::Rx as i32,
            chan: 0,
            mode: GainMode::FastAttack as i32,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(mode.mode, GainMode::FastAttack as i32);
    let antennas = client
        .list_antennas(pb::ListAntennasRequest {
            dir: Direction::Rx as i32,
        })
        .await
        .unwrap()
        .into_inner();
    assert_eq!(antennas.names, vec!["A_BALANCED"]);
    let antenna = client
        .set_antenna(session.req(pb::SetAntennaRequest {
            dir: Direction::Tx as i32,
            chan: 0,
            name: "A".to_string(),
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(antenna.name, "A");

    let sensors = client.get_sensors(pb::Empty {}).await.unwrap().into_inner();
    assert_eq!(sensors.rssi_dbfs.len(), 2);
    assert!(sensors.rx_lo_locked);
    assert!(sensors.tx_lo_locked);
}

#[tokio::test(flavor = "multi_thread")]
async fn hardware_time_is_set_and_read_back() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;
    let target = 1_000_000_000u64;
    let set = client
        .set_hardware_time(session.req(pb::HardwareTime { ticks: target }))
        .await
        .unwrap()
        .into_inner()
        .ticks;
    let read = client
        .get_hardware_time(pb::Empty {})
        .await
        .unwrap()
        .into_inner()
        .ticks;
    assert!(set >= target, "the clock is at or past what was loaded");
    assert!(read >= set, "the clock only moves forward");
    // A second of ticks at the fake rate is 30.72 million, so a read taken a
    // moment later stays well inside that.
    assert!(
        read - target < 30_720_000,
        "read {read} ticks after {target}"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn tx_rf_follows_running_streams_and_the_session() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;
    let set_gain = |session: &Session| {
        session.req(pb::SetGainRequest {
            dir: Direction::Tx as i32,
            chan: 0,
            db: 69.75,
        })
    };
    client.set_gain(set_gain(&session)).await.unwrap();
    assert!(!tx_rf_enabled(&mut client).await);

    let mut ids = Vec::new();
    for chan in 0..2 {
        ids.push(
            start_stream(
                &mut client,
                &session,
                open_request(Direction::Tx, vec![chan]),
            )
            .await,
        );
        assert!(tx_rf_enabled(&mut client).await);
    }
    client
        .stop_stream(session.req(pb::StreamRef { stream_id: ids[0] }))
        .await
        .unwrap();
    assert!(tx_rf_enabled(&mut client).await);
    client
        .stop_stream(session.req(pb::StreamRef { stream_id: ids[1] }))
        .await
        .unwrap();
    assert!(!tx_rf_enabled(&mut client).await);
    assert_eq!(tx_gain(&mut client, 0).await, 0.0);

    client.set_gain(set_gain(&session)).await.unwrap();
    client
        .start_stream(session.req(pb::StartStreamRequest {
            stream_id: ids[0],
            at_ticks: None,
        }))
        .await
        .unwrap();
    assert!(tx_rf_enabled(&mut client).await);

    drop(session);
    let deadline = Instant::now() + STATUS_TIMEOUT;
    while tx_rf_enabled(&mut client).await {
        assert!(
            Instant::now() < deadline,
            "ending the session must turn TX RF off"
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(tx_gain(&mut client, 0).await, 0.0);
}

#[tokio::test(flavor = "multi_thread")]
async fn a_stream_opens_starts_reports_status_and_closes() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let mut session = Session::open(&mut client).await;

    let opened = client
        .open_stream(session.req(open_request(Direction::Rx, vec![0])))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(opened.samples_per_packet, 481);
    assert!(opened.window > 0);
    send_hello_packet(opened.data_port);
    client
        .start_stream(session.req(pb::StartStreamRequest {
            stream_id: opened.stream_id,
            at_ticks: None,
        }))
        .await
        .unwrap();

    let (ticks, status) = session
        .status_until(opened.stream_id, |status| status.produced > 0)
        .await;
    assert!(ticks > 0);
    assert!(status.error.is_empty());

    client
        .stop_stream(session.req(pb::StreamRef {
            stream_id: opened.stream_id,
        }))
        .await
        .unwrap();
    client
        .close_stream(session.req(pb::StreamRef {
            stream_id: opened.stream_id,
        }))
        .await
        .unwrap();
    let gone = client
        .stop_stream(session.req(pb::StreamRef {
            stream_id: opened.stream_id,
        }))
        .await
        .unwrap_err();
    assert_eq!(gone.code(), Code::NotFound);
}

#[tokio::test(flavor = "multi_thread")]
async fn the_status_interval_is_set_on_the_session() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let mut session = Session::open(&mut client).await;
    let stream_id = start_stream(&mut client, &session, open_request(Direction::Rx, vec![0])).await;
    session.status_until(stream_id, |_| true).await;

    session
        .commands
        .send(pb::SessionCommand {
            command: Some(Command::SetStatusInterval(pb::SetStatusInterval {
                interval_ms: 0,
            })),
        })
        .await
        .unwrap();
    let error = loop {
        match tokio::time::timeout(REPLY_TIMEOUT, session.events.message())
            .await
            .expect("the daemon answers the command")
        {
            Ok(Some(_)) => continue,
            Ok(None) => panic!("the session ended without an error"),
            Err(status) => break status,
        }
    };
    assert_eq!(error.code(), Code::OutOfRange);
}

#[tokio::test(flavor = "multi_thread")]
async fn above_the_two_channel_ceiling_only_channel_0_streams() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    let opened_before = client
        .open_stream(session.req(open_request(Direction::Rx, vec![0, 1])))
        .await
        .unwrap()
        .into_inner();
    let rate = client
        .set_sample_rate(session.req(pb::SetSampleRateRequest {
            hz: 20_000_000,
            dir: 0,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(rate.hz, 20_000_000);

    send_hello_packet(opened_before.data_port);
    let start = client
        .start_stream(session.req(pb::StartStreamRequest {
            stream_id: opened_before.stream_id,
            at_ticks: None,
        }))
        .await
        .unwrap_err();
    assert_eq!(start.code(), Code::FailedPrecondition);
    let open = client
        .open_stream(session.req(open_request(Direction::Tx, vec![1])))
        .await
        .unwrap_err();
    assert_eq!(open.code(), Code::FailedPrecondition);
    assert!(open.message().contains("channel 1"), "{}", open.message());
    start_stream(&mut client, &session, open_request(Direction::Tx, vec![0])).await;
}

#[tokio::test(flavor = "multi_thread")]
async fn both_channels_stream_again_at_the_two_channel_ceiling() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;
    for hz in [20_000_000, 15_360_000] {
        client
            .set_sample_rate(session.req(pb::SetSampleRateRequest { hz, dir: 0 }))
            .await
            .unwrap();
    }
    start_stream(
        &mut client,
        &session,
        open_request(Direction::Rx, vec![0, 1]),
    )
    .await;
}

#[tokio::test(flavor = "multi_thread")]
async fn the_sample_rate_cannot_change_while_a_stream_runs() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;
    let mut request = open_request(Direction::Rx, vec![0, 1]);
    request.format = SampleFormat::Cs8 as i32;
    request.mtu = 9000;
    let stream_id = start_stream(&mut client, &session, request).await;

    let busy = client
        .set_sample_rate(session.req(pb::SetSampleRateRequest {
            hz: 5_000_000,
            dir: 0,
        }))
        .await
        .unwrap_err();
    assert_eq!(busy.code(), Code::FailedPrecondition);

    client
        .stop_stream(session.req(pb::StreamRef { stream_id }))
        .await
        .unwrap();
    let rate = client
        .set_sample_rate(session.req(pb::SetSampleRateRequest {
            hz: 5_000_000,
            dir: 0,
        }))
        .await
        .unwrap()
        .into_inner();
    assert_eq!(rate.hz, 5_000_000);
}

async fn set_rate(
    client: &mut ControlClient<Channel>,
    session: &Session,
    dir: Option<Direction>,
    hz: u32,
) -> Result<pb::SampleRateReply, tonic::Status> {
    let dir = dir.map_or(0, |dir| dir as i32);
    client
        .set_sample_rate(session.req(pb::SetSampleRateRequest { hz, dir }))
        .await
        .map(tonic::Response::into_inner)
}

#[tokio::test(flavor = "multi_thread")]
async fn each_direction_takes_its_own_sample_rate() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    // A direction the session has not set follows the one it sets.
    let rx = set_rate(&mut client, &session, Some(Direction::Rx), 7_680_000)
        .await
        .unwrap();
    assert_eq!(
        (rx.hz, rx.rx_hz, rx.tx_hz),
        (7_680_000, 7_680_000, 7_680_000)
    );
    assert_eq!(rx.tick_rate_hz, 7_680_000);

    let tx = set_rate(&mut client, &session, Some(Direction::Tx), 15_360_000)
        .await
        .unwrap();
    assert_eq!(
        (tx.hz, tx.rx_hz, tx.tx_hz),
        (15_360_000, 7_680_000, 15_360_000)
    );
    assert_eq!(tx.tick_rate_hz, 15_360_000);
    assert_eq!((tx.rx_ticks_per_sample, tx.tx_ticks_per_sample), (2, 1));

    // Rates that are not multiples of each other share a faster radio rate.
    let rx = set_rate(&mut client, &session, Some(Direction::Rx), 3_840_000)
        .await
        .unwrap();
    assert_eq!((rx.rx_hz, rx.tx_hz), (3_840_000, 15_360_000));
    let tx = set_rate(&mut client, &session, Some(Direction::Tx), 5_760_000)
        .await
        .unwrap();
    assert_eq!(tx.tick_rate_hz, 11_520_000);
    assert_eq!((tx.rx_ticks_per_sample, tx.tx_ticks_per_sample), (3, 2));

    let got = client
        .get_sample_rate(pb::Empty {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!(
        (got.hz, got.rx_hz, got.tx_hz),
        (3_840_000, 3_840_000, 5_760_000)
    );

    start_stream(&mut client, &session, open_request(Direction::Rx, vec![0])).await;
    start_stream(&mut client, &session, open_request(Direction::Tx, vec![0])).await;
}

#[tokio::test(flavor = "multi_thread")]
async fn a_rate_with_no_common_radio_rate_replaces_the_other_direction() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    set_rate(&mut client, &session, Some(Direction::Rx), 10_000_000)
        .await
        .unwrap();
    let tx = set_rate(&mut client, &session, Some(Direction::Tx), 4_000_000)
        .await
        .unwrap();
    assert_eq!((tx.rx_hz, tx.tx_hz), (4_000_000, 4_000_000));

    let refused = client
        .open_stream(session.req(open_request(Direction::Rx, vec![0])))
        .await
        .unwrap_err();
    assert_eq!(refused.code(), Code::FailedPrecondition);
    assert!(
        refused.message().contains("10000000") && refused.message().contains("4000000"),
        "{}",
        refused.message()
    );
    start_stream(&mut client, &session, open_request(Direction::Tx, vec![0])).await;
}

#[tokio::test(flavor = "multi_thread")]
async fn setting_a_replaced_rate_again_lets_its_streams_run() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    for (dir, hz) in [
        (Direction::Rx, 7_680_000),
        (Direction::Tx, 15_360_000),
        (Direction::Rx, 10_000_000),
        (Direction::Tx, 10_000_000),
    ] {
        set_rate(&mut client, &session, Some(dir), hz)
            .await
            .unwrap();
    }
    let got = client
        .get_sample_rate(pb::Empty {})
        .await
        .unwrap()
        .into_inner();
    assert_eq!((got.rx_hz, got.tx_hz), (10_000_000, 10_000_000));
    start_stream(&mut client, &session, open_request(Direction::Rx, vec![0])).await;
    start_stream(&mut client, &session, open_request(Direction::Tx, vec![0])).await;
}

#[tokio::test(flavor = "multi_thread")]
async fn hardware_time_keeps_its_time_of_day_across_a_tick_rate_change() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    set_rate(&mut client, &session, None, 5_000_000)
        .await
        .unwrap();
    client
        .set_hardware_time(session.req(pb::HardwareTime { ticks: 50_000_000 }))
        .await
        .unwrap();
    let rate = set_rate(&mut client, &session, None, 10_000_000)
        .await
        .unwrap();
    assert_eq!(rate.tick_rate_hz, 10_000_000);
    let ticks = client
        .get_hardware_time(pb::Empty {})
        .await
        .unwrap()
        .into_inner()
        .ticks;
    // Ten seconds on the old timebase, plus however long the calls took.
    assert!((100_000_000..120_000_000).contains(&ticks), "{ticks}");
}

#[tokio::test(flavor = "multi_thread")]
async fn bad_requests_say_what_was_wrong() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let session = Session::open(&mut client).await;

    let no_session = client
        .set_frequency(pb::SetFrequencyRequest {
            dir: Direction::Rx as i32,
            chan: 0,
            hz: 915_000_000,
        })
        .await
        .unwrap_err();
    assert_eq!(no_session.code(), Code::FailedPrecondition);
    assert!(no_session.message().contains(SESSION_METADATA_KEY));

    let mut stale = Request::new(pb::SetLoopbackRequest { on: true });
    stale
        .metadata_mut()
        .insert(SESSION_METADATA_KEY, "12345".parse().unwrap());
    let stale = client.set_loopback(stale).await.unwrap_err();
    assert_eq!(stale.code(), Code::FailedPrecondition);

    let missing_channel = client
        .get_gain(channel(Direction::Rx, 7))
        .await
        .unwrap_err();
    assert_eq!(missing_channel.code(), Code::NotFound);

    let no_direction = client
        .get_bandwidth(pb::ChannelRef { dir: 0, chan: 0 })
        .await
        .unwrap_err();
    assert_eq!(no_direction.code(), Code::InvalidArgument);

    let mut no_format = open_request(Direction::Rx, vec![0]);
    no_format.format = SampleFormat::Unspecified as i32;
    let no_format = client
        .open_stream(session.req(no_format))
        .await
        .unwrap_err();
    assert_eq!(no_format.code(), Code::InvalidArgument);
}

#[tokio::test(flavor = "multi_thread")]
async fn the_first_session_message_must_be_session_start() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let (commands, receiver) = mpsc::channel(1);
    commands
        .send(pb::SessionCommand {
            command: Some(Command::SetStatusInterval(pb::SetStatusInterval {
                interval_ms: 10,
            })),
        })
        .await
        .unwrap();
    let error = client
        .session(ReceiverStream::new(receiver))
        .await
        .unwrap_err();
    assert_eq!(error.code(), Code::InvalidArgument);
}

#[tokio::test(flavor = "multi_thread")]
async fn a_second_session_waits_for_a_running_stream_then_takes_over() {
    let fixture = start_daemon().await;
    let mut client = connect(&fixture).await;
    let first = Session::open(&mut client).await;
    let second = Session::open(&mut client).await;
    let stream_id = start_stream(&mut client, &first, open_request(Direction::Rx, vec![0])).await;

    let busy = client
        .set_loopback(second.req(pb::SetLoopbackRequest { on: true }))
        .await
        .unwrap_err();
    assert_eq!(busy.code(), Code::FailedPrecondition);

    client
        .stop_stream(first.req(pb::StreamRef { stream_id }))
        .await
        .unwrap();
    client
        .set_loopback(second.req(pb::SetLoopbackRequest { on: true }))
        .await
        .expect("an idle controller gives way");
    let gone = client
        .stop_stream(first.req(pb::StreamRef { stream_id }))
        .await
        .unwrap_err();
    assert_eq!(
        gone.code(),
        Code::NotFound,
        "the takeover closed the first session's stream"
    );
}

#[tokio::test(flavor = "multi_thread")]
async fn read_only_calls_need_no_session() {
    let fixture = start_daemon().await;
    let mut first = connect(&fixture).await;
    let mut second = connect(&fixture).await;
    assert!(first.get_sample_rate(pb::Empty {}).await.is_ok());
    assert!(second.get_hardware_time(pb::Empty {}).await.is_ok());
}

#[test]
fn a_discovery_probe_is_answered_with_the_boards_identity() {
    let (daemon, _parts) = fake_daemon(30000);
    let responder = DiscoveryResponder::bind(daemon.clone(), 0).expect("the port binds");
    let address = responder.local_addr().unwrap();
    std::thread::spawn(move || {
        let _ = responder.run();
    });

    let probe = UdpSocket::bind(SocketAddr::from(([127, 0, 0, 1], 0))).unwrap();
    probe.set_read_timeout(Some(REPLY_TIMEOUT)).unwrap();
    probe
        .send_to(
            DISCOVERY_PROBE,
            SocketAddr::from(([127, 0, 0, 1], address.port())),
        )
        .unwrap();
    let mut buffer = [0u8; 512];
    let (len, _) = probe.recv_from(&mut buffer).expect("the daemon answers");
    let reply = pb::DiscoveryReply::decode(&buffer[..len]).unwrap();
    assert_eq!(reply.name, DAEMON_NAME);
    assert_eq!(reply.control_port, 30000);
    assert_eq!(reply.serial, daemon.identity().serial);
    assert_eq!(reply.mac, daemon.identity().mac.to_string());
    assert!(!reply.version.is_empty());
}
