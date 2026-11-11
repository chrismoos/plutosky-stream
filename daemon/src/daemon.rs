//! The daemon proper: one transceiver, one set of streams, and the control
//! operations on them.

use std::time::Instant;

use parking_lot::Mutex;

use crate::control::proto::DiscoveryReply;
use crate::control::{ApiError, ErrorCode};
use crate::net::MacAddr;
use crate::radio::{
    Direction, GainControlMode, Radio, RadioError, SampleRates, Sensors, TX_MUTED_GAIN_DB,
    antennas, max_sample_rate,
};
use crate::stream::format::{MTUS, Mtu, WIRE_FORMATS, WireFormat};
use crate::stream::{OpenedStream, StreamCounters, StreamError, StreamManager, StreamRequest};
use crate::time;

pub const DAEMON_NAME: &str = "plutosky-stream";

/// Identifies a control session for as long as its `Session` call stays open.
pub type SessionId = u64;

#[derive(Clone, Debug)]
pub struct Identity {
    pub serial: String,
    pub version: String,
    pub mac: MacAddr,
}

/// What the bitstream and radio support, for `GetInfo`.
#[derive(Clone, Debug)]
pub struct Capabilities {
    pub channels: usize,
    pub min_rate_hz: u32,
    pub max_rate_hz: u32,
    pub formats: [WireFormat; WIRE_FORMATS.len()],
    pub mtus: [Mtu; MTUS.len()],
}

/// A stream's counters at one status tick, and the reason it stopped if it
/// stopped on its own since the previous tick.
#[derive(Clone, Debug)]
pub struct StreamSnapshot {
    pub stream_id: u32,
    pub counters: StreamCounters,
    pub error: Option<String>,
}

#[derive(Clone, Debug)]
pub struct StatusSnapshot {
    pub ticks: u64,
    pub streams: Vec<StreamSnapshot>,
}

/// Lock order: `owner`, then `streams`, then `radio`.
pub struct Daemon {
    radio: Mutex<Box<dyn Radio>>,
    streams: Mutex<StreamManager>,
    identity: Identity,
    control_port: u16,
    channel_count: usize,
    /// The session that controls the radio, or none. Any session can read, but
    /// the first one to change the radio or a stream takes control and holds it
    /// until its `Session` call ends.
    owner: Mutex<Option<SessionId>>,
    rate_requests: Mutex<RateRequests>,
}

/// What the controlling session has asked of each direction's sample rate,
/// indexed receive then transmit.
#[derive(Default)]
struct RateRequests {
    session: Option<SessionId>,
    /// Directions whose rate this session set itself.
    set: [bool; 2],
    /// A rate this session set that a later request for the other direction
    /// replaced, because the two had no common radio rate.
    replaced: [Option<u32>; 2],
}

fn direction_index(direction: Direction) -> usize {
    match direction {
        Direction::Rx => 0,
        Direction::Tx => 1,
    }
}

fn other(direction: Direction) -> Direction {
    match direction {
        Direction::Rx => Direction::Tx,
        Direction::Tx => Direction::Rx,
    }
}

impl Daemon {
    pub fn new(
        radio: Box<dyn Radio>,
        streams: StreamManager,
        identity: Identity,
        control_port: u16,
    ) -> Self {
        let channel_count = streams.channel_count();
        Self {
            radio: Mutex::new(radio),
            streams: Mutex::new(streams),
            identity,
            control_port,
            channel_count,
            owner: Mutex::new(None),
            rate_requests: Mutex::new(RateRequests::default()),
        }
    }

    /// Grants `session` control of the radio. Free or already its own: granted.
    /// Held by another session that is not streaming: taken over, closing
    /// whatever that session left behind, so a client whose disconnect the
    /// daemon has not seen yet never strands the radio. Only an owner with a
    /// running stream keeps control against a new claimant.
    fn claim_control(&self, session: SessionId) -> Result<(), ApiError> {
        let mut owner = self.owner.lock();
        match *owner {
            None => {
                *owner = Some(session);
                Ok(())
            }
            Some(id) if id == session => Ok(()),
            Some(_) => {
                let mut streams = self.streams.lock();
                if streams.any_running() {
                    Err(ApiError::new(
                        ErrorCode::Busy,
                        "the radio is in use by another session",
                    ))
                } else {
                    streams.close_all()?;
                    self.disable_tx()?;
                    *owner = Some(session);
                    Ok(())
                }
            }
        }
    }

    /// Releases control and closes the streams when the controlling session
    /// ends. A session that never took control leaves everything untouched.
    pub fn release_control(&self, session: SessionId) {
        let mut owner = self.owner.lock();
        if *owner == Some(session) {
            if let Err(error) = self.close_all_streams() {
                log::error!("could not release control after disconnect: {error}");
            } else {
                *owner = None;
            }
        }
    }

    fn disable_tx(&self) -> Result<(), RadioError> {
        let mut radio = self.radio.lock();
        let mut first_error = radio.set_tx_enabled(false).err();
        for channel in 0..self.channel_count {
            if let Err(error) = radio.set_gain(Direction::Tx, channel, TX_MUTED_GAIN_DB) {
                log::error!("could not mute TX channel {channel}: {error}");
                if first_error.is_none() {
                    first_error = Some(error);
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }

    pub fn identity(&self) -> &Identity {
        &self.identity
    }

    pub fn discovery_reply(&self) -> DiscoveryReply {
        DiscoveryReply {
            name: DAEMON_NAME.to_string(),
            serial: self.identity.serial.clone(),
            version: self.identity.version.clone(),
            control_port: u32::from(self.control_port),
            mac: self.identity.mac.to_string(),
        }
    }

    pub fn capabilities(&self) -> Capabilities {
        let (min_rate_hz, max_rate_hz) = {
            let radio = self.radio.lock();
            (radio.min_sample_rate(), radio.max_sample_rate())
        };
        Capabilities {
            channels: self.channel_count,
            min_rate_hz,
            max_rate_hz,
            formats: WIRE_FORMATS,
            mtus: MTUS,
        }
    }

    fn check_channel(&self, channel: usize) -> Result<usize, ApiError> {
        if channel >= self.channel_count {
            return Err(ApiError::new(
                ErrorCode::NotFound,
                format!(
                    "no channel {channel}: the bitstream has {}",
                    self.channel_count
                ),
            ));
        }
        Ok(channel)
    }

    pub fn set_frequency(
        &self,
        session: SessionId,
        direction: Direction,
        channel: usize,
        hz: u64,
    ) -> Result<u64, ApiError> {
        self.claim_control(session)?;
        self.check_channel(channel)?;
        Ok(self.radio.lock().set_frequency(direction, hz)?)
    }

    pub fn frequency(&self, direction: Direction, channel: usize) -> Result<u64, ApiError> {
        self.check_channel(channel)?;
        Ok(self.radio.lock().frequency(direction)?)
    }

    /// Sets the rate of `direction`, or of both when it is `None`.
    ///
    /// The other direction keeps a rate this session set for it when the two
    /// share a radio rate. Otherwise it takes the new rate too, and a stream
    /// in that direction does not start until its rate is set again.
    pub fn set_sample_rate(
        &self,
        session: SessionId,
        direction: Option<Direction>,
        hz: u32,
    ) -> Result<SampleRates, ApiError> {
        self.claim_control(session)?;
        {
            let radio = self.radio.lock();
            let (min_hz, max_hz) = (radio.min_sample_rate(), radio.max_sample_rate());
            if !(min_hz..=max_hz).contains(&hz) {
                return Err(RadioError::SampleRateRange { hz, min_hz, max_hz }.into());
            }
        }
        let streams = self.streams.lock();
        if streams.any_running() {
            return Err(ApiError::new(
                ErrorCode::Busy,
                "the sample rate cannot change while a stream is running",
            ));
        }
        let mut requests = self.rate_requests.lock();
        if requests.session != Some(session) {
            *requests = RateRequests {
                session: Some(session),
                ..RateRequests::default()
            };
        }
        let mut radio = self.radio.lock();
        let before = radio.sample_rates()?;
        let time_before = (streams.hardware_time(), Instant::now());
        let after = match direction {
            None => {
                requests.set = [true; 2];
                requests.replaced = [None; 2];
                radio.set_sample_rates(hz, hz)?
            }
            Some(direction) => {
                let (this, that) = (
                    direction_index(direction),
                    direction_index(other(direction)),
                );
                let kept = [before.rx_hz, before.tx_hz][that];
                let mut pair = [hz; 2];
                if requests.set[that] {
                    pair[that] = kept;
                }
                let after = match radio.set_sample_rates(pair[0], pair[1]) {
                    Err(RadioError::NoCommonRate { .. }) => {
                        requests.replaced[that] = Some(kept);
                        radio.set_sample_rates(hz, hz)?
                    }
                    result => result?,
                };
                requests.set[this] = true;
                requests.replaced[this] = None;
                after
            }
        };
        if after.tick_rate_hz != before.tick_rate_hz {
            // A count is a different length of time now, so the timebase is
            // rewritten to read the same time of day.
            let (ticks, at) = time_before;
            let nanos = time::ticks_to_nanos(ticks, before.tick_rate_hz)
                .saturating_add(at.elapsed().as_nanos() as u64);
            streams.set_hardware_time(time::nanos_to_ticks(nanos, after.tick_rate_hz));
        }
        Ok(after)
    }

    pub fn sample_rates(&self) -> Result<SampleRates, ApiError> {
        Ok(self.radio.lock().sample_rates()?)
    }

    /// Refuses a stream whose rate a request for the other direction replaced.
    fn check_rate_request(&self, direction: Direction) -> Result<(), ApiError> {
        let requests = self.rate_requests.lock();
        let Some(replaced_hz) = requests.replaced[direction_index(direction)] else {
            return Ok(());
        };
        let rates = self.radio.lock().sample_rates()?;
        let now_hz = [rates.rx_hz, rates.tx_hz][direction_index(direction)];
        Err(ApiError::new(
            ErrorCode::Busy,
            format!(
                "the {} rate {replaced_hz} Hz has no common radio rate with the {} rate \
                 {now_hz} Hz and was replaced by it: set the {} rate again",
                direction.as_str(),
                other(direction).as_str(),
                direction.as_str(),
            ),
        ))
    }

    pub fn set_bandwidth(
        &self,
        session: SessionId,
        direction: Direction,
        channel: usize,
        hz: u32,
    ) -> Result<u32, ApiError> {
        self.claim_control(session)?;
        self.check_channel(channel)?;
        Ok(self.radio.lock().set_bandwidth(direction, hz)?)
    }

    pub fn bandwidth(&self, direction: Direction, channel: usize) -> Result<u32, ApiError> {
        self.check_channel(channel)?;
        Ok(self.radio.lock().bandwidth(direction)?)
    }

    pub fn set_gain(
        &self,
        session: SessionId,
        direction: Direction,
        channel: usize,
        db: f64,
    ) -> Result<f64, ApiError> {
        self.claim_control(session)?;
        self.check_channel(channel)?;
        Ok(self.radio.lock().set_gain(direction, channel, db)?)
    }

    pub fn gain(&self, direction: Direction, channel: usize) -> Result<f64, ApiError> {
        self.check_channel(channel)?;
        Ok(self.radio.lock().gain(direction, channel)?)
    }

    pub fn set_gain_mode(
        &self,
        session: SessionId,
        direction: Direction,
        channel: usize,
        mode: GainControlMode,
    ) -> Result<GainControlMode, ApiError> {
        self.claim_control(session)?;
        self.check_channel(channel)?;
        Ok(self.radio.lock().set_gain_mode(direction, channel, mode)?)
    }

    pub fn antennas(&self, direction: Direction) -> &'static [&'static str] {
        antennas(direction)
    }

    pub fn set_antenna(
        &self,
        session: SessionId,
        direction: Direction,
        channel: usize,
        name: &str,
    ) -> Result<String, ApiError> {
        self.claim_control(session)?;
        self.check_channel(channel)?;
        Ok(self
            .radio
            .lock()
            .set_antenna(direction, channel, name)?
            .to_string())
    }

    pub fn set_loopback(&self, session: SessionId, on: bool) -> Result<bool, ApiError> {
        self.claim_control(session)?;
        self.radio.lock().set_loopback(on)?;
        Ok(on)
    }

    pub fn hardware_time(&self) -> u64 {
        self.streams.lock().hardware_time()
    }

    pub fn set_hardware_time(&self, session: SessionId, ticks: u64) -> Result<u64, ApiError> {
        self.claim_control(session)?;
        Ok(self.streams.lock().set_hardware_time(ticks))
    }

    /// The radio's sensors and whether TX RF is on.
    pub fn sensors(&self) -> Result<(Sensors, bool), ApiError> {
        let mut radio = self.radio.lock();
        let sensors = radio.sensors()?;
        Ok((sensors, radio.tx_enabled()))
    }

    pub fn open_stream(
        &self,
        session: SessionId,
        request: StreamRequest,
    ) -> Result<OpenedStream, ApiError> {
        self.claim_control(session)?;
        self.check_active_channels(&request.channels)?;
        self.check_rate_request(request.direction)?;
        Ok(self.streams.lock().open(request)?)
    }

    /// Above the two-channel ceiling the radio runs channel 0 alone.
    fn check_active_channels(&self, channels: &[usize]) -> Result<(), ApiError> {
        let active = self.radio.lock().active_channels();
        match channels.iter().find(|&&channel| channel >= active) {
            Some(channel) if *channel < self.channel_count => Err(ApiError::new(
                ErrorCode::Busy,
                format!(
                    "channel {channel} is off at this sample rate: above {} Hz only channel 0 runs",
                    max_sample_rate(self.channel_count)
                ),
            )),
            _ => Ok(()),
        }
    }

    pub fn start_stream(
        &self,
        session: SessionId,
        stream_id: u32,
        at_ticks: Option<u64>,
    ) -> Result<(), ApiError> {
        self.claim_control(session)?;
        let mut streams = self.streams.lock();
        self.check_active_channels(&streams.channels(stream_id)?)?;
        self.check_rate_request(streams.direction(stream_id)?)?;
        if streams.direction(stream_id)? == Direction::Tx && !streams.any_tx_running() {
            self.radio.lock().set_tx_enabled(true)?;
        }
        if let Err(error) = streams.start(stream_id, at_ticks) {
            if !streams.any_tx_running() {
                self.disable_tx()?;
            }
            return Err(error.into());
        }
        Ok(())
    }

    pub fn stop_stream(&self, session: SessionId, stream_id: u32) -> Result<(), ApiError> {
        self.claim_control(session)?;
        let mut streams = self.streams.lock();
        let result = streams.stop(stream_id);
        if !streams.any_tx_running() {
            self.disable_tx()?;
        }
        Ok(result?)
    }

    pub fn close_stream(&self, session: SessionId, stream_id: u32) -> Result<(), ApiError> {
        self.claim_control(session)?;
        let mut streams = self.streams.lock();
        let result = streams.close(stream_id);
        if !streams.any_tx_running() {
            self.disable_tx()?;
        }
        Ok(result?)
    }

    /// The counters of every open stream, for the controlling session only.
    /// `None` for any other session, or when no stream is open. Also turns TX
    /// RF off when no TX stream runs.
    pub fn status(&self, session: SessionId) -> Option<StatusSnapshot> {
        // Holding streams while waiting for owner deadlocks against a claim
        // from another session.
        let owner = self.owner.lock();
        let streams = self.streams.lock();
        if !streams.any_tx_running()
            && let Err(error) = self.radio.lock().set_tx_enabled(false)
        {
            log::error!("could not disable idle TX RF chain: {error}");
        }
        if *owner != Some(session) {
            return None;
        }
        drop(owner);
        let ids = streams.ids();
        if ids.is_empty() {
            return None;
        }
        let ticks = streams.hardware_time();
        let snapshots = ids
            .into_iter()
            .filter_map(|stream_id| {
                let counters = streams.counters(stream_id).ok()?;
                Some(StreamSnapshot {
                    stream_id,
                    counters,
                    error: streams.take_failure(stream_id),
                })
            })
            .collect();
        Some(StatusSnapshot {
            ticks,
            streams: snapshots,
        })
    }

    /// Stops and closes every stream, for a session that ended.
    pub fn close_all_streams(&self) -> Result<(), ApiError> {
        let mut streams = self.streams.lock();
        let close_result = streams.close_all();
        let mute_result = self.disable_tx();
        close_result?;
        mute_result?;
        Ok(())
    }

    /// Releases every receive slot the daemon holds, for a clean exit. Closes
    /// the streams and drains the steer ring so no slot is left out of the free
    /// list when the process goes away. The gemslot driver does not reclaim on
    /// fd release, so without this a stop leaks slots and wedges the shared
    /// Ethernet path until the next daemon reclaims them at start-up.
    pub fn shutdown(&self) {
        let mut streams = self.streams.lock();
        if let Err(error) = streams.close_all() {
            log::error!("could not close all streams at shutdown: {error}");
        }
        streams.reclaim_steered();
        if let Err(error) = self.disable_tx() {
            log::error!("could not mute TX at shutdown: {error}");
        }
    }

    pub fn describe_radio(&self) -> String {
        self.radio.lock().describe()
    }
}

impl From<RadioError> for ApiError {
    fn from(error: RadioError) -> Self {
        let code = match error {
            RadioError::NoSuchChannel { .. } | RadioError::NoSuchAntenna { .. } => {
                ErrorCode::NotFound
            }
            RadioError::SampleRateRange { .. }
            | RadioError::NoCommonRate { .. }
            | RadioError::TxGainRange { .. } => ErrorCode::OutOfRange,
            _ => ErrorCode::Radio,
        };
        ApiError::new(code, error.to_string())
    }
}

impl From<StreamError> for ApiError {
    fn from(error: StreamError) -> Self {
        let code = match error {
            StreamError::NoSuchStream(_) | StreamError::NoSuchChannel { .. } => ErrorCode::NotFound,
            StreamError::NoInstanceFree { .. } | StreamError::AlreadyRunning(_) => ErrorCode::Busy,
            _ => ErrorCode::Stream,
        };
        ApiError::new(code, error.to_string())
    }
}
