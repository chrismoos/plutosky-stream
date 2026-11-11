//! The gRPC `Control` service.

use std::collections::HashSet;
use std::net::{Ipv4Addr, SocketAddr};
use std::pin::Pin;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use parking_lot::Mutex;
use tokio::sync::mpsc;
use tokio_stream::Stream;
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use tonic::transport::Server;
use tonic::{Request, Response, Status, Streaming};

use super::proto::control_server::{Control, ControlServer};
use super::proto::session_command::Command;
use super::proto::session_event::Event;
use super::proto::{self as pb};
use crate::daemon::{Daemon, SessionId, StatusSnapshot};
use crate::radio::{Direction, GainControlMode, SampleRates};
use crate::stream::StreamRequest;
use crate::stream::format::{Mtu, WireFormat};

pub const DEFAULT_CONTROL_PORT: u16 = 30000;

/// Metadata key that carries the session id on calls that change the radio or
/// a stream.
pub const SESSION_METADATA_KEY: &str = "plutosky-session";

pub const DEFAULT_STATUS_INTERVAL: Duration = Duration::from_millis(10);
const MIN_STATUS_INTERVAL_MS: u32 = 1;
const MAX_STATUS_INTERVAL_MS: u32 = 1000;

/// HTTP/2 pings that detect a client that vanished without closing its
/// session, so its streams close and TX turns off.
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(2);
const KEEPALIVE_TIMEOUT: Duration = Duration::from_secs(6);

/// Events buffered per session before the status task waits for the client.
const SESSION_EVENT_QUEUE: usize = 16;

pub struct ControlService {
    daemon: Arc<Daemon>,
    sessions: Arc<Mutex<HashSet<SessionId>>>,
    next_session: AtomicU64,
}

impl ControlService {
    pub fn new(daemon: Arc<Daemon>) -> Self {
        // Ids start from the clock so a client holding an id from an earlier
        // daemon run does not match a new session.
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map_or(1, |elapsed| elapsed.as_nanos() as u64);
        Self {
            daemon,
            sessions: Arc::new(Mutex::new(HashSet::new())),
            next_session: AtomicU64::new(seed),
        }
    }

    fn session<T>(&self, request: &Request<T>) -> Result<SessionId, Status> {
        let value = request
            .metadata()
            .get(SESSION_METADATA_KEY)
            .ok_or_else(|| {
                Status::failed_precondition(format!(
                    "this call changes the radio and needs `{SESSION_METADATA_KEY}` metadata from an open Session"
                ))
            })?;
        let session = value
            .to_str()
            .ok()
            .and_then(|text| text.parse::<SessionId>().ok())
            .ok_or_else(|| {
                Status::invalid_argument(format!("`{SESSION_METADATA_KEY}` is not a session id"))
            })?;
        if !self.sessions.lock().contains(&session) {
            return Err(Status::failed_precondition(format!(
                "session {session} is not open"
            )));
        }
        Ok(session)
    }

    /// Runs a daemon call off the async workers. Radio calls can block for
    /// seconds, and a sample rate change reruns the LVDS calibration.
    async fn blocking<T, F>(&self, call: F) -> Result<Response<T>, Status>
    where
        T: Send + 'static,
        F: FnOnce(&Daemon) -> Result<T, Status> + Send + 'static,
    {
        let daemon = self.daemon.clone();
        tokio::task::spawn_blocking(move || call(&daemon))
            .await
            .map_err(|error| Status::internal(format!("daemon call failed: {error}")))?
            .map(Response::new)
    }
}

/// Serves the control service on `port` until the server fails.
pub async fn serve(
    service: ControlService,
    port: u16,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = tokio::net::TcpListener::bind(SocketAddr::from(([0, 0, 0, 0], port)))
        .await
        .map_err(|e| std::io::Error::new(e.kind(), format!("bind control port {port}: {e}")))?;
    serve_on(service, listener).await
}

/// Serves the control service on a bound listener, for tests on an ephemeral
/// port.
pub async fn serve_on(
    service: ControlService,
    listener: tokio::net::TcpListener,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    log::info!("control service on {}", listener.local_addr()?);
    Server::builder()
        .tcp_nodelay(true)
        .http2_keepalive_interval(Some(KEEPALIVE_INTERVAL))
        .http2_keepalive_timeout(Some(KEEPALIVE_TIMEOUT))
        .add_service(ControlServer::new(service))
        .serve_with_incoming(TcpListenerStream::new(listener))
        .await?;
    Ok(())
}

type SessionStream = Pin<Box<dyn Stream<Item = Result<pb::SessionEvent, Status>> + Send>>;

#[tonic::async_trait]
impl Control for ControlService {
    async fn get_info(
        &self,
        request: Request<pb::GetInfoRequest>,
    ) -> Result<Response<pb::BoardInfo>, Status> {
        let client = request.into_inner().client;
        let identity = self.daemon.identity().clone();
        self.blocking(move |daemon| {
            log::debug!("GetInfo from {client}");
            let caps = daemon.capabilities();
            Ok(pb::BoardInfo {
                serial: identity.serial,
                version: identity.version,
                channels: caps.channels as u32,
                min_rate_hz: caps.min_rate_hz,
                max_rate_hz: caps.max_rate_hz,
                formats: caps
                    .formats
                    .iter()
                    .map(|f| format_to_proto(*f) as i32)
                    .collect(),
                mtus: caps.mtus.iter().map(|m| m.bytes() as u32).collect(),
            })
        })
        .await
    }

    type SessionStream = SessionStream;

    async fn session(
        &self,
        request: Request<Streaming<pb::SessionCommand>>,
    ) -> Result<Response<Self::SessionStream>, Status> {
        let peer = request.remote_addr();
        let mut commands = request.into_inner();
        let client = match commands.message().await? {
            Some(pb::SessionCommand {
                command: Some(Command::Start(start)),
            }) => start.client,
            _ => {
                return Err(Status::invalid_argument(
                    "the first Session message must be SessionStart",
                ));
            }
        };
        let session = self.next_session.fetch_add(1, Ordering::Relaxed);
        self.sessions.lock().insert(session);
        log::info!(
            "session {session} opened by {client} from {}",
            peer.map_or_else(|| "?".to_string(), |addr| addr.to_string())
        );

        let (events, receiver) = mpsc::channel(SESSION_EVENT_QUEUE);
        let opened = pb::SessionEvent {
            event: Some(Event::Opened(pb::SessionOpened {
                session_id: session.to_string(),
                status_interval_ms: DEFAULT_STATUS_INTERVAL.as_millis() as u32,
            })),
        };
        events
            .send(Ok(opened))
            .await
            .map_err(|_| Status::cancelled("the client closed the session"))?;

        let daemon = self.daemon.clone();
        let sessions = self.sessions.clone();
        tokio::spawn(async move {
            run_session(session, &daemon, commands, events).await;
            sessions.lock().remove(&session);
            let released = daemon.clone();
            let _ = tokio::task::spawn_blocking(move || released.release_control(session)).await;
            log::info!("session {session} closed");
        });
        Ok(Response::new(Box::pin(ReceiverStream::new(receiver))))
    }

    async fn set_frequency(
        &self,
        request: Request<pb::SetFrequencyRequest>,
    ) -> Result<Response<pb::FrequencyReply>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::FrequencyReply {
                hz: d.set_frequency(session, direction, r.chan as usize, r.hz)?,
            })
        })
        .await
    }

    async fn get_frequency(
        &self,
        request: Request<pb::ChannelRef>,
    ) -> Result<Response<pb::FrequencyReply>, Status> {
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::FrequencyReply {
                hz: d.frequency(direction, r.chan as usize)?,
            })
        })
        .await
    }

    async fn set_sample_rate(
        &self,
        request: Request<pb::SetSampleRateRequest>,
    ) -> Result<Response<pb::SampleRateReply>, Status> {
        let session = self.session(&request)?;
        let request = request.into_inner();
        let dir = match pb::Direction::try_from(request.dir) {
            Ok(pb::Direction::Unspecified) => None,
            _ => Some(direction(request.dir)?),
        };
        self.blocking(move |d| {
            let rates = d.set_sample_rate(session, dir, request.hz)?;
            Ok(sample_rate_reply(rates, dir))
        })
        .await
    }

    async fn get_sample_rate(
        &self,
        _request: Request<pb::Empty>,
    ) -> Result<Response<pb::SampleRateReply>, Status> {
        self.blocking(|d| Ok(sample_rate_reply(d.sample_rates()?, None)))
            .await
    }

    async fn set_bandwidth(
        &self,
        request: Request<pb::SetBandwidthRequest>,
    ) -> Result<Response<pb::BandwidthReply>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::BandwidthReply {
                hz: d.set_bandwidth(session, direction, r.chan as usize, r.hz)?,
            })
        })
        .await
    }

    async fn get_bandwidth(
        &self,
        request: Request<pb::ChannelRef>,
    ) -> Result<Response<pb::BandwidthReply>, Status> {
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::BandwidthReply {
                hz: d.bandwidth(direction, r.chan as usize)?,
            })
        })
        .await
    }

    async fn set_gain(
        &self,
        request: Request<pb::SetGainRequest>,
    ) -> Result<Response<pb::GainReply>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::GainReply {
                db: d.set_gain(session, direction, r.chan as usize, r.db)?,
            })
        })
        .await
    }

    async fn get_gain(
        &self,
        request: Request<pb::ChannelRef>,
    ) -> Result<Response<pb::GainReply>, Status> {
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::GainReply {
                db: d.gain(direction, r.chan as usize)?,
            })
        })
        .await
    }

    async fn set_gain_mode(
        &self,
        request: Request<pb::SetGainModeRequest>,
    ) -> Result<Response<pb::GainModeReply>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        let mode = gain_mode(r.mode)?;
        self.blocking(move |d| {
            let mode = d.set_gain_mode(session, direction, r.chan as usize, mode)?;
            Ok(pb::GainModeReply {
                mode: gain_mode_to_proto(mode) as i32,
            })
        })
        .await
    }

    async fn list_antennas(
        &self,
        request: Request<pb::ListAntennasRequest>,
    ) -> Result<Response<pb::AntennaList>, Status> {
        let direction = direction(request.into_inner().dir)?;
        Ok(Response::new(pb::AntennaList {
            names: self
                .daemon
                .antennas(direction)
                .iter()
                .map(|name| name.to_string())
                .collect(),
        }))
    }

    async fn set_antenna(
        &self,
        request: Request<pb::SetAntennaRequest>,
    ) -> Result<Response<pb::AntennaReply>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        let direction = direction(r.dir)?;
        self.blocking(move |d| {
            Ok(pb::AntennaReply {
                name: d.set_antenna(session, direction, r.chan as usize, &r.name)?,
            })
        })
        .await
    }

    async fn set_loopback(
        &self,
        request: Request<pb::SetLoopbackRequest>,
    ) -> Result<Response<pb::LoopbackReply>, Status> {
        let session = self.session(&request)?;
        let on = request.into_inner().on;
        self.blocking(move |d| {
            Ok(pb::LoopbackReply {
                on: d.set_loopback(session, on)?,
            })
        })
        .await
    }

    async fn get_hardware_time(
        &self,
        _request: Request<pb::Empty>,
    ) -> Result<Response<pb::HardwareTime>, Status> {
        self.blocking(|d| {
            Ok(pb::HardwareTime {
                ticks: d.hardware_time(),
            })
        })
        .await
    }

    async fn set_hardware_time(
        &self,
        request: Request<pb::HardwareTime>,
    ) -> Result<Response<pb::HardwareTime>, Status> {
        let session = self.session(&request)?;
        let ticks = request.into_inner().ticks;
        self.blocking(move |d| {
            Ok(pb::HardwareTime {
                ticks: d.set_hardware_time(session, ticks)?,
            })
        })
        .await
    }

    async fn get_sensors(
        &self,
        _request: Request<pb::Empty>,
    ) -> Result<Response<pb::Sensors>, Status> {
        self.blocking(|d| {
            let (sensors, tx_rf_enabled) = d.sensors()?;
            Ok(pb::Sensors {
                temperature_c: sensors.temperature_c,
                rssi_dbfs: sensors.rssi_dbfs.to_vec(),
                rx_lo_locked: sensors.rx_lo_locked,
                tx_lo_locked: sensors.tx_lo_locked,
                tx_rf_enabled,
            })
        })
        .await
    }

    async fn open_stream(
        &self,
        request: Request<pb::OpenStreamRequest>,
    ) -> Result<Response<pb::OpenStreamReply>, Status> {
        let session = self.session(&request)?;
        let stream = stream_request(request.into_inner())?;
        self.blocking(move |d| {
            let opened = d.open_stream(session, stream)?;
            Ok(pb::OpenStreamReply {
                stream_id: opened.stream_id,
                data_port: u32::from(opened.data_port),
                samples_per_packet: opened.samples_per_packet as u32,
                window: opened.window as u32,
            })
        })
        .await
    }

    async fn start_stream(
        &self,
        request: Request<pb::StartStreamRequest>,
    ) -> Result<Response<pb::Empty>, Status> {
        let session = self.session(&request)?;
        let r = request.into_inner();
        self.blocking(move |d| {
            d.start_stream(session, r.stream_id, r.at_ticks)?;
            Ok(pb::Empty {})
        })
        .await
    }

    async fn stop_stream(
        &self,
        request: Request<pb::StreamRef>,
    ) -> Result<Response<pb::Empty>, Status> {
        let session = self.session(&request)?;
        let stream_id = request.into_inner().stream_id;
        self.blocking(move |d| {
            d.stop_stream(session, stream_id)?;
            Ok(pb::Empty {})
        })
        .await
    }

    async fn close_stream(
        &self,
        request: Request<pb::StreamRef>,
    ) -> Result<Response<pb::Empty>, Status> {
        let session = self.session(&request)?;
        let stream_id = request.into_inner().stream_id;
        self.blocking(move |d| {
            d.close_stream(session, stream_id)?;
            Ok(pb::Empty {})
        })
        .await
    }
}

/// Sends status at the session's interval and applies its commands, until the
/// client ends the call or stops reading.
async fn run_session(
    session: SessionId,
    daemon: &Arc<Daemon>,
    mut commands: Streaming<pb::SessionCommand>,
    events: mpsc::Sender<Result<pb::SessionEvent, Status>>,
) {
    let mut ticker = status_ticker(DEFAULT_STATUS_INTERVAL);
    loop {
        tokio::select! {
            command = commands.message() => match command {
                Ok(Some(pb::SessionCommand { command: Some(Command::SetStatusInterval(set)) })) => {
                    match status_interval(set.interval_ms) {
                        Ok(interval) => ticker = status_ticker(interval),
                        Err(status) => {
                            let _ = events.send(Err(status)).await;
                            return;
                        }
                    }
                }
                Ok(Some(_)) => {
                    let _ = events
                        .send(Err(Status::invalid_argument("SessionStart is only valid as the first message")))
                        .await;
                    return;
                }
                Ok(None) | Err(_) => return,
            },
            _ = ticker.tick() => {
                let status_daemon = daemon.clone();
                let snapshot = tokio::task::spawn_blocking(move || status_daemon.status(session))
                    .await
                    .ok()
                    .flatten();
                if let Some(snapshot) = snapshot
                    && events.send(Ok(status_event(snapshot))).await.is_err()
                {
                    return;
                }
            }
            _ = events.closed() => return,
        }
    }
}

fn status_ticker(interval: Duration) -> tokio::time::Interval {
    let mut ticker = tokio::time::interval(interval);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    ticker
}

fn status_interval(interval_ms: u32) -> Result<Duration, Status> {
    if !(MIN_STATUS_INTERVAL_MS..=MAX_STATUS_INTERVAL_MS).contains(&interval_ms) {
        return Err(Status::out_of_range(format!(
            "status interval {interval_ms} ms is outside {MIN_STATUS_INTERVAL_MS}..={MAX_STATUS_INTERVAL_MS} ms"
        )));
    }
    Ok(Duration::from_millis(u64::from(interval_ms)))
}

fn status_event(snapshot: StatusSnapshot) -> pb::SessionEvent {
    pb::SessionEvent {
        event: Some(Event::Status(pb::StatusTick {
            ticks: snapshot.ticks,
            streams: snapshot
                .streams
                .into_iter()
                .map(|stream| pb::StreamStatus {
                    stream_id: stream.stream_id,
                    produced: stream.counters.produced,
                    consumed: stream.counters.consumed,
                    late: stream.counters.late,
                    underrun: stream.counters.underrun,
                    overflow: stream.counters.overflow,
                    gaps: stream.counters.gaps,
                    error: stream.error.unwrap_or_default(),
                })
                .collect(),
        })),
    }
}

fn sample_rate_reply(rates: SampleRates, set: Option<Direction>) -> pb::SampleRateReply {
    let ticks_per_sample = |hz: u32| (rates.tick_rate_hz + hz / 2) / hz.max(1);
    pb::SampleRateReply {
        hz: match set {
            Some(Direction::Tx) => rates.tx_hz,
            _ => rates.rx_hz,
        },
        rx_hz: rates.rx_hz,
        tx_hz: rates.tx_hz,
        tick_rate_hz: rates.tick_rate_hz,
        rx_ticks_per_sample: ticks_per_sample(rates.rx_hz),
        tx_ticks_per_sample: ticks_per_sample(rates.tx_hz),
        rx_delay_ticks: rates.rx_delay_ticks,
        tx_delay_ticks: rates.tx_delay_ticks,
    }
}

fn direction(value: i32) -> Result<Direction, Status> {
    match pb::Direction::try_from(value) {
        Ok(pb::Direction::Rx) => Ok(Direction::Rx),
        Ok(pb::Direction::Tx) => Ok(Direction::Tx),
        _ => Err(Status::invalid_argument(
            "`dir` must be DIRECTION_RX or DIRECTION_TX",
        )),
    }
}

fn gain_mode(value: i32) -> Result<GainControlMode, Status> {
    match pb::GainMode::try_from(value) {
        Ok(pb::GainMode::Manual) => Ok(GainControlMode::Manual),
        Ok(pb::GainMode::SlowAttack) => Ok(GainControlMode::SlowAttack),
        Ok(pb::GainMode::FastAttack) => Ok(GainControlMode::FastAttack),
        _ => Err(Status::invalid_argument("`mode` is not a gain mode")),
    }
}

fn gain_mode_to_proto(mode: GainControlMode) -> pb::GainMode {
    match mode {
        GainControlMode::Manual => pb::GainMode::Manual,
        GainControlMode::SlowAttack => pb::GainMode::SlowAttack,
        GainControlMode::FastAttack => pb::GainMode::FastAttack,
    }
}

fn format_to_proto(format: WireFormat) -> pb::SampleFormat {
    match format {
        WireFormat::Cs12 => pb::SampleFormat::Cs12,
        WireFormat::Cs8 => pb::SampleFormat::Cs8,
        WireFormat::Bfp8 => pb::SampleFormat::Bfp8,
    }
}

fn format_from_proto(value: i32) -> Result<WireFormat, Status> {
    match pb::SampleFormat::try_from(value) {
        Ok(pb::SampleFormat::Cs12) => Ok(WireFormat::Cs12),
        Ok(pb::SampleFormat::Cs8) => Ok(WireFormat::Cs8),
        Ok(pb::SampleFormat::Bfp8) => Ok(WireFormat::Bfp8),
        _ => Err(Status::invalid_argument("`format` is not a sample format")),
    }
}

fn stream_request(r: pb::OpenStreamRequest) -> Result<StreamRequest, Status> {
    let mtu = Mtu::parse(r.mtu as usize).map_err(|error| Status::not_found(error.to_string()))?;
    let host_ip: Ipv4Addr = r.host_ip.parse().map_err(|_| {
        Status::invalid_argument(format!("`host_ip` is not an IPv4 address: {}", r.host_ip))
    })?;
    let host_port = u16::try_from(r.host_port)
        .map_err(|_| Status::invalid_argument("`host_port` is not a UDP port"))?;
    Ok(StreamRequest {
        direction: direction(r.dir)?,
        channels: r.chans.iter().map(|&chan| chan as usize).collect(),
        format: format_from_proto(r.format)?,
        mtu,
        host_ip,
        host_port,
    })
}
