//! Stream lifecycle: allocation, CSR programming and the slot pump thread.
//!
//! A stream is one VITA flow in one direction. Opening one reserves a
//! packetizer or depacketizer instance and the slice of the slot ring that
//! goes with it, binds the UDP data port, and works out the packet geometry.
//! Starting one learns the host's MAC from the neighbour table, programs the
//! frame template and the core, and puts a pump thread behind it.

pub mod format;
pub mod header;
pub mod pump;
pub mod regs;

use std::collections::BTreeMap;
use std::io;
use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use thiserror::Error;

use crate::csr::{CsrBus, CsrMap, MapError, Reg};
use crate::gem::{
    GEMSLOT_EVENT_RX, GEMSLOT_EVENT_TX, KERNEL_STREAM_RX, KERNEL_STREAM_TX, KernelStreamConfig,
    KernelStreamStatus, SlotPoolInfo, Slots, WaitRequest,
};
use crate::net::{LinkError, LinkResolver};
use crate::radio::Direction;
use crate::time::Timebase;

pub use format::{Mtu, PacketPlan, WireFormat};
pub use header::FrameEndpoints;
pub use pump::StreamCounters;
pub use regs::{ChannelMask, MAX_STREAMS};

use header::{frame_header, header_template_words};
use pump::{DepacketizerPump, PacketizerPump, SlotPump};
use regs::{PacketizerCtrl, RxPacketizerRegs, SlotPoolRegs, TxDepacketizerRegs};

/// How long `start_stream` waits for the host's first data packet, which is
/// what puts the host in the kernel's neighbour table.
const HELLO_TIMEOUT: Duration = Duration::from_secs(2);
/// How often the hello wait re-checks the socket.
const HELLO_POLL_INTERVAL: Duration = Duration::from_millis(10);
/// How long a pump pass blocks in the driver before looking again.
const PUMP_WAIT_TIMEOUT: Duration = Duration::from_millis(10);
/// How long the pump sleeps when the driver reported nothing to do, so a
/// backend that never blocks does not spin a core.
const PUMP_IDLE_SLEEP: Duration = Duration::from_millis(1);
const TX_COMPLETION_POLL_INTERVAL: Duration = Duration::from_micros(50);
/// Slots the driver may hold in one batch, kept out of the host's window so a
/// full window still leaves the pump a batch to work with.
const WINDOW_HEADROOM_SLOTS: usize = crate::gem::SLOT_BATCH_MAX;
/// Transmit window in slots, small enough to leave the GEM receive ring and
/// spare slots outside it.
const TX_WINDOW_SLOTS: usize = 1024;
/// Produced slots the packetizer collects before raising its interrupt.
const IRQ_THRESHOLD_SLOTS: u32 = 8;
/// Largest hello packet the daemon reads off the data port.
const HELLO_BUFFER_BYTES: usize = 2048;

#[derive(Debug, Error)]
pub enum StreamError {
    #[error("no stream {0}")]
    NoSuchStream(u32),
    #[error("all {instances} stream instances are in use")]
    NoInstanceFree { instances: usize },
    #[error("the register map exports no stream instances: rxpkt0 and txpkt0 are missing")]
    NoStreamInstances,
    #[error(
        "the slot pool has {slot_count} slots per ring, and {instances} streams need at least {needed}"
    )]
    PoolTooSmall {
        slot_count: u32,
        instances: usize,
        needed: usize,
    },
    #[error("stream {0} is already running")]
    AlreadyRunning(u32),
    #[error("stream {0} is not running")]
    NotRunning(u32),
    #[error("the GEM driver has no kernel stream pump; install the matching kernel image")]
    KernelPumpUnavailable,
    #[error("CSR address {0:#x} does not fit the kernel stream ABI")]
    CsrAddress(u64),
    #[error("a stream must carry at least one channel")]
    NoChannels,
    #[error("channel {channel} does not exist")]
    NoSuchChannel { channel: usize },
    #[error("host address {0} is not IPv4")]
    NotIpv4(IpAddr),
    #[error("the host sent no packet to data port {port} within {timeout:?}")]
    NoHello { port: u16, timeout: Duration },
    #[error(transparent)]
    Link(#[from] LinkError),
    #[error(transparent)]
    Map(#[from] MapError),
    #[error("{context}: {source}")]
    Io {
        context: &'static str,
        source: io::Error,
    },
}

fn io_error(context: &'static str) -> impl FnOnce(io::Error) -> StreamError {
    move |source| StreamError::Io { context, source }
}

fn csr_offset(reg: Reg, base: u64) -> Result<u32, StreamError> {
    reg.addr
        .checked_sub(base)
        .and_then(|offset| u32::try_from(offset).ok())
        .ok_or(StreamError::CsrAddress(reg.addr))
}

fn kernel_rx_config(
    stream: &Stream,
    regs: RxPacketizerRegs,
    csr_base: u64,
) -> Result<KernelStreamConfig, StreamError> {
    Ok(KernelStreamConfig {
        direction: KERNEL_STREAM_RX,
        instance: stream.instance as u32,
        ctrl: csr_offset(regs.ctrl, csr_base)?,
        produced: csr_offset(regs.produced, csr_base)?,
        observed: csr_offset(regs.observed, csr_base)?,
        released: csr_offset(regs.released, csr_base)?,
        irq_threshold: csr_offset(regs.irq_threshold, csr_base)?,
        slot_first: u32::from(stream.slot_first),
        slot_count: u32::from(stream.slot_count),
        frame_len: stream.plan.frame_len as u32,
        format: stream.plan.format.ctrl_code(),
        ..KernelStreamConfig::default()
    })
}

fn kernel_tx_config(
    stream: &Stream,
    regs: TxDepacketizerRegs,
    csr_base: u64,
) -> Result<KernelStreamConfig, StreamError> {
    Ok(KernelStreamConfig {
        direction: KERNEL_STREAM_TX,
        instance: stream.instance as u32,
        ctrl: csr_offset(regs.ctrl, csr_base)?,
        submit: csr_offset(regs.submit, csr_base)?,
        consumed: csr_offset(regs.consumed, csr_base)?,
        consumed_count: csr_offset(regs.consumed_count, csr_base)?,
        tx_irq_threshold: csr_offset(regs.irq_threshold, csr_base)?,
        slot_first: u32::from(stream.slot_first),
        slot_count: u32::from(stream.slot_count),
        udp_port: u32::from(stream.data_port),
        format: stream.plan.format.ctrl_code(),
        ..KernelStreamConfig::default()
    })
}

/// What `open_stream` asks for.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StreamRequest {
    /// `rx` is board to host, `tx` is host to board.
    pub direction: Direction,
    pub channels: Vec<usize>,
    pub format: WireFormat,
    pub mtu: Mtu,
    pub host_ip: Ipv4Addr,
    pub host_port: u16,
}

/// What `open_stream` answers with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OpenedStream {
    pub stream_id: u32,
    pub data_port: u16,
    pub samples_per_packet: usize,
    /// Packets the host may keep outstanding.
    pub window: usize,
}

struct PumpState {
    counters: Mutex<StreamCounters>,
    failure: Mutex<Option<String>>,
}

struct PumpThread {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
    state: Arc<PumpState>,
}

struct KernelStartTimer {
    stop: Arc<AtomicBool>,
    join: JoinHandle<()>,
}

#[derive(Clone, Copy, Default)]
struct KernelCounterBase {
    overflow: u32,
    late: u32,
    underrun: u32,
    gaps: u32,
}

impl KernelStartTimer {
    fn stop(self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.join.join();
    }
}

impl PumpThread {
    fn stop(mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

struct Stream {
    id: u32,
    /// Which packetizer or depacketizer instance the stream owns.
    instance: usize,
    request: StreamRequest,
    plan: PacketPlan,
    channels: ChannelMask,
    slot_first: u16,
    slot_count: u16,
    window: usize,
    data_socket: UdpSocket,
    data_port: u16,
    /// Where the host's hello packet came from and the board address on the
    /// way back. The observed source is what the frames must go to, since a
    /// host behind NAT declares an address the board cannot reach.
    path: Option<HostPath>,
    running: Option<PumpThread>,
    kernel_running: bool,
    kernel_timer: Option<KernelStartTimer>,
    kernel_counter_base: KernelCounterBase,
    kernel_failure_reported: AtomicBool,
    steering_enabled: bool,
    /// The last counters a stopped stream reported, so `status` does not jump
    /// back to zero when a stream stops.
    last_counters: StreamCounters,
}

impl Stream {
    fn ctrl(&self, enable: bool, arm: bool) -> PacketizerCtrl {
        PacketizerCtrl {
            enable,
            arm,
            format: self.plan.format,
            channels: self.channels,
        }
    }
}

/// Owns every open stream and the CSRs behind them.
pub struct StreamManager {
    bus: Arc<dyn CsrBus>,
    slots: Arc<dyn Slots>,
    links: Arc<dyn LinkResolver>,
    timebase: Timebase,
    channel_count: usize,
    pool: SlotPoolInfo,
    packetizers: Vec<RxPacketizerRegs>,
    depacketizers: Vec<TxDepacketizerRegs>,
    streams: BTreeMap<u32, Stream>,
    instances_in_use: [Vec<bool>; 2],
    next_stream_id: u32,
    csr_base: u64,
}

impl StreamManager {
    /// Resolves every stream CSR and tells the PL where the slot pool is.
    pub fn new(
        bus: Arc<dyn CsrBus>,
        slots: Arc<dyn Slots>,
        links: Arc<dyn LinkResolver>,
        map: &CsrMap,
        timebase: Timebase,
    ) -> Result<Self, StreamError> {
        let channel_count = map.channels()?;
        // A bitstream may build fewer instances than the contract's four, so
        // take the ones it exports and stop at the first gap. Instance 0 has
        // to be there, and a half-built instance is a broken map, not a
        // smaller one.
        let mut packetizers = Vec::new();
        let mut depacketizers = Vec::new();
        for index in 0..MAX_STREAMS {
            let Ok(packetizer) = RxPacketizerRegs::resolve(map, index) else {
                break;
            };
            depacketizers.push(TxDepacketizerRegs::resolve(map, index)?);
            packetizers.push(packetizer);
        }
        let instances = packetizers.len();
        if instances == 0 {
            return Err(StreamError::NoStreamInstances);
        }

        let pool = slots.info();
        let slots_per_stream = pool.slot_count / instances as u32;
        if slots_per_stream as usize <= WINDOW_HEADROOM_SLOTS {
            return Err(StreamError::PoolTooSmall {
                slot_count: pool.slot_count,
                instances,
                needed: (WINDOW_HEADROOM_SLOTS + 1) * instances,
            });
        }
        SlotPoolRegs::resolve(map)?.program(&*bus, &pool);
        log::info!("{instances} stream instances in this bitstream");
        // Nothing may be streaming into DDR before the daemon knows about it.
        for (index, regs) in packetizers.iter().enumerate() {
            regs.ctrl.write(&*bus, 0);
            depacketizers[index].ctrl.write(&*bus, 0);
        }
        Ok(Self {
            bus,
            slots,
            links,
            timebase,
            channel_count,
            pool,
            packetizers,
            depacketizers,
            streams: BTreeMap::new(),
            instances_in_use: [vec![false; instances], vec![false; instances]],
            next_stream_id: 1,
            csr_base: map.io_span().map_or(0, |span| span.start),
        })
    }

    /// Stream instances this bitstream built.
    pub fn instances(&self) -> usize {
        self.packetizers.len()
    }

    /// RF channels implemented by this bitstream.
    pub fn channel_count(&self) -> usize {
        self.channel_count
    }

    /// Slots each stream instance owns in its ring.
    fn slots_per_stream(&self) -> u16 {
        (self.pool.slot_count / self.instances() as u32) as u16
    }

    pub fn open(&mut self, request: StreamRequest) -> Result<OpenedStream, StreamError> {
        if request.channels.is_empty() {
            return Err(StreamError::NoChannels);
        }
        for &channel in &request.channels {
            if channel >= self.channel_count {
                return Err(StreamError::NoSuchChannel { channel });
            }
        }
        let direction = match request.direction {
            Direction::Rx => 0,
            Direction::Tx => 1,
        };
        let instance = self.instances_in_use[direction]
            .iter()
            .position(|used| !used)
            .ok_or(StreamError::NoInstanceFree {
                instances: self.instances(),
            })?;

        let data_socket = UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], 0)))
            .map_err(io_error("bind the data port"))?;
        data_socket
            .set_nonblocking(true)
            .map_err(io_error("set the data port non-blocking"))?;
        let data_port = data_socket
            .local_addr()
            .map_err(io_error("read the data port"))?
            .port();
        let plan = PacketPlan::for_channels(request.format, request.mtu, request.channels.len());
        let slots_per_stream = self.slots_per_stream();
        let stream_id = self.next_stream_id;
        self.next_stream_id += 1;
        self.instances_in_use[direction][instance] = true;
        let base_window = usize::from(slots_per_stream).saturating_sub(WINDOW_HEADROOM_SLOTS);
        let window = match request.direction {
            Direction::Tx => base_window.min(TX_WINDOW_SLOTS),
            Direction::Rx => base_window,
        };
        let stream = Stream {
            id: stream_id,
            instance,
            channels: ChannelMask::from_channels(&request.channels),
            request,
            plan,
            slot_first: instance as u16 * slots_per_stream,
            slot_count: slots_per_stream,
            window,
            data_socket,
            data_port,
            path: None,
            running: None,
            kernel_running: false,
            kernel_timer: None,
            kernel_counter_base: KernelCounterBase::default(),
            kernel_failure_reported: AtomicBool::new(false),
            steering_enabled: false,
            last_counters: StreamCounters::default(),
        };
        let opened = OpenedStream {
            stream_id,
            data_port,
            samples_per_packet: plan.samples_per_packet,
            window: stream.window,
        };
        log::info!(
            "stream {stream_id} open: dir={} format={} mtu={} channels={:#04b} data_port={data_port} samples_per_packet={} frame_len={} window={}",
            stream.request.direction.as_str(),
            plan.format,
            stream.request.mtu.bytes(),
            stream.channels.bits(),
            plan.samples_per_packet,
            plan.frame_len,
            stream.window,
        );
        self.streams.insert(stream_id, stream);
        Ok(opened)
    }

    pub fn start(&mut self, stream_id: u32, at_ticks: Option<u64>) -> Result<(), StreamError> {
        let stream = self
            .streams
            .get_mut(&stream_id)
            .ok_or(StreamError::NoSuchStream(stream_id))?;
        if stream.running.is_some() || stream.kernel_running {
            return Err(StreamError::AlreadyRunning(stream_id));
        }
        wait_for_hello(stream)?;

        match self.slots.kernel_pump_available() {
            Some(true) => {
                stream.kernel_counter_base = match stream.request.direction {
                    Direction::Rx => KernelCounterBase {
                        overflow: self.packetizers[stream.instance].overflow.read(&*self.bus)
                            as u32,
                        ..KernelCounterBase::default()
                    },
                    Direction::Tx => {
                        let regs = self.depacketizers[stream.instance];
                        KernelCounterBase {
                            late: regs.late.read(&*self.bus) as u32,
                            underrun: regs.underrun.read(&*self.bus) as u32,
                            gaps: regs.gaps.read(&*self.bus) as u32,
                            ..KernelCounterBase::default()
                        }
                    }
                };
                match stream.request.direction {
                    Direction::Rx => {
                        let regs = self.packetizers[stream.instance];
                        self.slots
                            .stream_start(kernel_rx_config(stream, regs, self.csr_base)?)
                            .map_err(io_error("start the kernel receive stream"))?;
                        if let Err(error) =
                            program_packetizer(&*self.bus, &*self.links, stream, regs, at_ticks)
                        {
                            let _ = self.slots.stream_stop(KERNEL_STREAM_RX);
                            return Err(error);
                        }
                    }
                    Direction::Tx => {
                        let regs = self.depacketizers[stream.instance];
                        regs.stream_id.write(&*self.bus, u64::from(stream.id));
                        self.slots
                            .stream_start(kernel_tx_config(stream, regs, self.csr_base)?)
                            .map_err(io_error("start the kernel transmit stream"))?;
                        let ctrl = stream.ctrl(true, false);
                        if let Some(ticks) = at_ticks {
                            let stop = Arc::new(AtomicBool::new(false));
                            let cancel = stop.clone();
                            let bus = self.bus.clone();
                            let timebase = self.timebase;
                            let join = std::thread::Builder::new()
                                .name(format!("stream-{stream_id}-start"))
                                .spawn(move || {
                                    while !cancel.load(Ordering::Relaxed) {
                                        if timebase.now(&*bus) >= ticks {
                                            regs.set_ctrl(&*bus, ctrl);
                                            break;
                                        }
                                        std::thread::sleep(Duration::from_millis(1));
                                    }
                                })
                                .map_err(io_error("schedule the transmit stream start"));
                            match join {
                                Ok(join) => {
                                    stream.kernel_timer = Some(KernelStartTimer { stop, join })
                                }
                                Err(error) => {
                                    let _ = self.slots.stream_stop(KERNEL_STREAM_TX);
                                    return Err(error);
                                }
                            }
                        } else {
                            regs.set_ctrl(&*self.bus, ctrl);
                        }
                    }
                }
                stream.kernel_running = true;
                stream
                    .kernel_failure_reported
                    .store(false, Ordering::Relaxed);
                log::info!(
                    "stream {stream_id} started with kernel slot pump at_ticks={at_ticks:?}"
                );
                return Ok(());
            }
            Some(false) => return Err(StreamError::KernelPumpUnavailable),
            None => {}
        }

        let pump = match stream.request.direction {
            Direction::Rx => {
                let regs = self.packetizers[stream.instance];
                let mut pump = PacketizerPump::new(
                    regs,
                    self.slots.clone(),
                    stream.slot_first,
                    stream.slot_count,
                    stream.plan.frame_len as u16,
                );
                // The counters are read and every slot handed back before the
                // packetizer is enabled, or the first frames would count as
                // lost.
                pump.prime(&*self.bus)
                    .map_err(io_error("prime the slot pump"))?;
                program_packetizer(&*self.bus, &*self.links, stream, regs, at_ticks)?;
                SlotPump::Packetizer(pump)
            }
            Direction::Tx => {
                let regs = self.depacketizers[stream.instance];
                self.slots
                    .rx_steer(stream.data_port, true)
                    .map_err(io_error("enable steering"))?;
                stream.steering_enabled = true;
                regs.stream_id.write(&*self.bus, u64::from(stream.id));
                let running = stream.ctrl(true, false);
                let mut pump = DepacketizerPump::new(regs, self.slots.clone(), stream.plan.format);
                let stale = pump.discard_stale_completions(&*self.bus);
                if stale != 0 {
                    log::warn!("discarded {stale} stale depacketizer completions");
                }
                SlotPump::Depacketizer(match at_ticks {
                    // The depacketizer has no armed start of its own, so the
                    // pump enables it when the clock gets there. Doing that
                    // here would hold the control session until the tick.
                    Some(ticks) => pump.start_at(ticks, self.timebase, running),
                    None => {
                        regs.set_ctrl(&*self.bus, running);
                        pump
                    }
                })
            }
        };
        stream.running = Some(spawn_pump(self.bus.clone(), self.slots.clone(), pump));
        log::info!("stream {stream_id} started at_ticks={at_ticks:?}");
        Ok(())
    }

    pub fn stop(&mut self, stream_id: u32) -> Result<(), StreamError> {
        let stream = self
            .streams
            .get_mut(&stream_id)
            .ok_or(StreamError::NoSuchStream(stream_id))?;
        if stream.kernel_running {
            if let Some(timer) = stream.kernel_timer.take() {
                timer.stop();
            }
            let direction = match stream.request.direction {
                Direction::Rx => {
                    self.packetizers[stream.instance]
                        .set_ctrl(&*self.bus, stream.ctrl(false, false));
                    KERNEL_STREAM_RX
                }
                Direction::Tx => {
                    self.depacketizers[stream.instance]
                        .set_ctrl(&*self.bus, stream.ctrl(false, false));
                    KERNEL_STREAM_TX
                }
            };
            let result = self
                .slots
                .stream_stop(direction)
                .map_err(io_error("stop the kernel stream"));
            if let Ok(status) = self.slots.stream_status(direction) {
                stream.last_counters = kernel_counters(
                    stream.request.direction,
                    status,
                    &*self.bus,
                    self.packetizers[stream.instance],
                    self.depacketizers[stream.instance],
                    stream.kernel_counter_base,
                );
            }
            stream.kernel_running = false;
            result?;
            log::info!("stream {stream_id} stopped");
            return Ok(());
        }
        let running = stream
            .running
            .take()
            .ok_or(StreamError::NotRunning(stream_id))?;
        let steering_result = match stream.request.direction {
            Direction::Rx => {
                self.packetizers[stream.instance].set_ctrl(&*self.bus, stream.ctrl(false, false));
                Ok(())
            }
            Direction::Tx => {
                self.depacketizers[stream.instance].set_ctrl(&*self.bus, stream.ctrl(false, false));
                let result = self
                    .slots
                    .rx_steer(stream.data_port, false)
                    .map_err(io_error("disable steering"));
                if result.is_ok() {
                    stream.steering_enabled = false;
                }
                result
            }
        };
        stream.last_counters = *running.state.counters.lock();
        running.stop();
        steering_result?;
        log::info!("stream {stream_id} stopped");
        Ok(())
    }

    pub fn close(&mut self, stream_id: u32) -> Result<(), StreamError> {
        let running = self
            .streams
            .get(&stream_id)
            .ok_or(StreamError::NoSuchStream(stream_id))?;
        if (running.running.is_some() || running.kernel_running)
            && let Err(error) = self.stop(stream_id)
        {
            log::warn!("stopping stream {stream_id} before close: {error}");
        }
        let stream = self
            .streams
            .get_mut(&stream_id)
            .ok_or(StreamError::NoSuchStream(stream_id))?;
        if stream.steering_enabled {
            self.slots
                .rx_steer(stream.data_port, false)
                .map_err(io_error("disable steering"))?;
            stream.steering_enabled = false;
        }
        if let Some(stream) = self.streams.remove(&stream_id) {
            let direction = match stream.request.direction {
                Direction::Rx => 0,
                Direction::Tx => 1,
            };
            self.instances_in_use[direction][stream.instance] = false;
        }
        log::info!("stream {stream_id} closed");
        Ok(())
    }

    /// Stops and closes every stream, for a session ending or the daemon
    /// shutting down.
    pub fn close_all(&mut self) -> Result<(), StreamError> {
        let mut first_error = None;
        for stream_id in self.streams.keys().copied().collect::<Vec<_>>() {
            if let Err(error) = self.close(stream_id) {
                log::warn!("closing stream {stream_id}: {error}");
                if first_error.is_none() {
                    first_error = Some(error);
                }
            }
        }
        first_error.map_or(Ok(()), Err)
    }

    /// Hands every slot still sitting in the driver's steer ring back to the
    /// free list. The driver has no fd-release cleanup, so a daemon that exits
    /// must return its slots itself or they shrink the receive free list and
    /// wedge the shared Ethernet path until the next daemon reclaims them.
    pub fn reclaim_steered(&self) {
        let mut buf = Vec::new();
        loop {
            buf.clear();
            if self.slots.rx_poll(&mut buf).is_err() || buf.is_empty() {
                break;
            }
            if self.slots.rx_return(&buf).is_err() {
                break;
            }
        }
    }

    pub fn ids(&self) -> Vec<u32> {
        self.streams.keys().copied().collect()
    }

    pub fn any_running(&self) -> bool {
        self.streams.values().any(|stream| {
            stream.running.is_some()
                || (stream.kernel_running
                    && self
                        .slots
                        .stream_status(match stream.request.direction {
                            Direction::Rx => KERNEL_STREAM_RX,
                            Direction::Tx => KERNEL_STREAM_TX,
                        })
                        .is_ok_and(|status| status.active != 0))
        })
    }

    pub fn channels(&self, stream_id: u32) -> Result<Vec<usize>, StreamError> {
        self.streams
            .get(&stream_id)
            .map(|stream| stream.request.channels.clone())
            .ok_or(StreamError::NoSuchStream(stream_id))
    }

    pub fn direction(&self, stream_id: u32) -> Result<Direction, StreamError> {
        self.streams
            .get(&stream_id)
            .map(|stream| stream.request.direction)
            .ok_or(StreamError::NoSuchStream(stream_id))
    }

    pub fn any_tx_running(&self) -> bool {
        self.streams.values().any(|stream| {
            stream.request.direction == Direction::Tx
                && (stream.running.as_ref().is_some_and(|running| {
                    running
                        .join
                        .as_ref()
                        .is_some_and(|join| !join.is_finished())
                }) || (stream.kernel_running
                    && self
                        .slots
                        .stream_status(KERNEL_STREAM_TX)
                        .is_ok_and(|status| status.active != 0)))
        })
    }

    /// Counters for a `status` event.
    pub fn counters(&self, stream_id: u32) -> Result<StreamCounters, StreamError> {
        let stream = self
            .streams
            .get(&stream_id)
            .ok_or(StreamError::NoSuchStream(stream_id))?;
        if stream.kernel_running {
            let direction = match stream.request.direction {
                Direction::Rx => KERNEL_STREAM_RX,
                Direction::Tx => KERNEL_STREAM_TX,
            };
            let status = self
                .slots
                .stream_status(direction)
                .map_err(io_error("read the kernel stream status"))?;
            return Ok(kernel_counters(
                stream.request.direction,
                status,
                &*self.bus,
                self.packetizers[stream.instance],
                self.depacketizers[stream.instance],
                stream.kernel_counter_base,
            ));
        }
        Ok(match &stream.running {
            Some(running) => *running.state.counters.lock(),
            None => stream.last_counters,
        })
    }

    /// The reason a stream stopped on its own, taken so it is reported once.
    pub fn take_failure(&self, stream_id: u32) -> Option<String> {
        let stream = self.streams.get(&stream_id)?;
        if stream.kernel_running {
            let direction = match stream.request.direction {
                Direction::Rx => KERNEL_STREAM_RX,
                Direction::Tx => KERNEL_STREAM_TX,
            };
            let failed = self
                .slots
                .stream_status(direction)
                .map_or(true, |status| status.active == 0);
            if failed && !stream.kernel_failure_reported.swap(true, Ordering::Relaxed) {
                return Some("the kernel stream pump stopped".to_string());
            }
            return None;
        }
        stream.running.as_ref()?.state.failure.lock().take()
    }

    pub fn hardware_time(&self) -> u64 {
        self.timebase.now(&*self.bus)
    }

    pub fn set_hardware_time(&self, ticks: u64) -> u64 {
        self.timebase.set_now(&*self.bus, ticks);
        self.timebase.now(&*self.bus)
    }
}

fn kernel_counters(
    direction: Direction,
    status: KernelStreamStatus,
    bus: &dyn CsrBus,
    rx: RxPacketizerRegs,
    tx: TxDepacketizerRegs,
    base: KernelCounterBase,
) -> StreamCounters {
    match direction {
        Direction::Rx => StreamCounters {
            produced: status.produced,
            consumed: status.completed,
            overflow: u64::from((rx.overflow.read(bus) as u32).wrapping_sub(base.overflow)),
            ..StreamCounters::default()
        },
        Direction::Tx => StreamCounters {
            produced: status.submitted,
            consumed: status.consumed_ticks,
            late: u64::from((tx.late.read(bus) as u32).wrapping_sub(base.late)),
            underrun: u64::from((tx.underrun.read(bus) as u32).wrapping_sub(base.underrun)),
            gaps: u64::from((tx.gaps.read(bus) as u32).wrapping_sub(base.gaps)),
            ..StreamCounters::default()
        },
    }
}

/// Programs everything the packetizer needs and arms or enables it.
fn program_packetizer(
    bus: &dyn CsrBus,
    links: &dyn LinkResolver,
    stream: &Stream,
    regs: RxPacketizerRegs,
    at_ticks: Option<u64>,
) -> Result<(), StreamError> {
    let path = stream.path.ok_or(StreamError::NoHello {
        port: stream.data_port,
        timeout: HELLO_TIMEOUT,
    })?;
    let link = links.resolve(*path.host.ip())?;
    let endpoints = FrameEndpoints {
        board_mac: link.board_mac,
        host_mac: link.host_mac,
        board_ip: path.board_ip,
        host_ip: *path.host.ip(),
        board_port: stream.data_port,
        host_port: path.host.port(),
    };
    let header = frame_header(&endpoints, &stream.plan, stream.id);
    regs.write_header(bus, &header_template_words(&header));
    regs.samples_per_packet
        .write(bus, stream.plan.samples_per_packet as u64);
    regs.frame_len.write(bus, stream.plan.frame_len as u64);
    regs.slot_first.write(bus, u64::from(stream.slot_first));
    regs.slot_count.write(bus, u64::from(stream.slot_count));
    regs.irq_threshold
        .write(bus, u64::from(IRQ_THRESHOLD_SLOTS));
    let start = at_ticks.unwrap_or(0);
    regs.start_hi.write(bus, start >> 32);
    regs.start_lo.write(bus, start & u64::from(u32::MAX));
    regs.set_ctrl(bus, stream.ctrl(true, at_ticks.is_some()));
    log::debug!(
        "stream {} header: host_mac={} board_mac={} {}:{} -> {}:{}",
        stream.id,
        link.host_mac,
        link.board_mac,
        endpoints.board_ip,
        endpoints.board_port,
        endpoints.host_ip,
        endpoints.host_port,
    );
    Ok(())
}

/// Reads the host's hello packet off the data port, which is what put the host
/// in the kernel's neighbour table, and records where it came from.
fn wait_for_hello(stream: &mut Stream) -> Result<(), StreamError> {
    if stream.path.is_some() {
        return Ok(());
    }
    let deadline = Instant::now() + HELLO_TIMEOUT;
    let mut buffer = [0u8; HELLO_BUFFER_BYTES];
    loop {
        match stream.data_socket.recv_from(&mut buffer) {
            Ok((len, from)) => {
                log::debug!(
                    "stream {} hello: {len} bytes from {from} on port {}",
                    stream.id,
                    stream.data_port
                );
                let host = match from {
                    SocketAddr::V4(host) => host,
                    SocketAddr::V6(_) => return Err(StreamError::NotIpv4(from.ip())),
                };
                if *host.ip() != stream.request.host_ip || host.port() != stream.request.host_port {
                    log::info!(
                        "stream {} host declared {}:{} but sends from {host}, using the observed path",
                        stream.id,
                        stream.request.host_ip,
                        stream.request.host_port
                    );
                }
                let board_ip = local_address_towards(*host.ip(), host.port())?;
                stream.path = Some(HostPath { host, board_ip });
                return Ok(());
            }
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                if Instant::now() >= deadline {
                    return Err(StreamError::NoHello {
                        port: stream.data_port,
                        timeout: HELLO_TIMEOUT,
                    });
                }
                std::thread::sleep(HELLO_POLL_INTERVAL);
            }
            Err(source) => {
                return Err(StreamError::Io {
                    context: "read the hello packet",
                    source,
                });
            }
        }
    }
}

/// The host endpoint a stream's frames go to and the board address they come
/// from, both taken from the hello packet's path.
#[derive(Clone, Copy, Debug)]
struct HostPath {
    host: SocketAddrV4,
    board_ip: Ipv4Addr,
}

/// The board's own address on the path to the host, which is the source
/// address the packetizer puts in every frame.
fn local_address_towards(host_ip: Ipv4Addr, host_port: u16) -> Result<Ipv4Addr, StreamError> {
    let probe = UdpSocket::bind(SocketAddr::from(([0, 0, 0, 0], 0)))
        .map_err(io_error("bind the route probe"))?;
    probe
        .connect(SocketAddr::from((host_ip, host_port.max(1))))
        .map_err(io_error("resolve the route to the host"))?;
    match probe
        .local_addr()
        .map_err(io_error("read the local address"))?
        .ip()
    {
        IpAddr::V4(address) => Ok(address),
        other => Err(StreamError::NotIpv4(other)),
    }
}

fn spawn_pump(bus: Arc<dyn CsrBus>, slots: Arc<dyn Slots>, mut pump: SlotPump) -> PumpThread {
    let stop = Arc::new(AtomicBool::new(false));
    let state = Arc::new(PumpState {
        counters: Mutex::new(StreamCounters::default()),
        failure: Mutex::new(None),
    });
    let request = match pump.ring() {
        crate::gem::Ring::Tx => WaitRequest {
            events: GEMSLOT_EVENT_TX,
            timeout: PUMP_WAIT_TIMEOUT,
            tx_threshold: 1,
            rx_threshold: 0,
        },
        crate::gem::Ring::Rx => WaitRequest {
            events: GEMSLOT_EVENT_RX,
            timeout: PUMP_WAIT_TIMEOUT,
            tx_threshold: 0,
            rx_threshold: 1,
        },
    };
    let join = std::thread::Builder::new()
        .name("slot-pump".to_string())
        .spawn({
            let stop = stop.clone();
            let state = state.clone();
            move || {
                if matches!(pump, SlotPump::Depacketizer(_)) {
                    separate_tx_from_network_irq();
                }
                while !stop.load(Ordering::Relaxed) {
                    // GEM receives no interrupt when the FPGA finishes reading a TX slot.
                    let wait_request = if pump.needs_completion_poll() {
                        WaitRequest {
                            timeout: Duration::ZERO,
                            ..request
                        }
                    } else {
                        request
                    };
                    let pending = match slots.wait(wait_request) {
                        Ok(pending) => pending,
                        Err(error) => {
                            log::warn!("slot wait failed: {error}");
                            *state.failure.lock() = Some(format!("slot wait failed: {error}"));
                            break;
                        }
                    };
                    if let Err(error) = pump.step(&*bus) {
                        log::warn!("slot pump failed ({:?} ring): {error}", pump.ring());
                        *state.failure.lock() = Some(format!("slot pump failed: {error}"));
                        break;
                    }
                    *state.counters.lock() = pump.counters();
                    if pending.tx_completions == 0 && pending.rx_frames == 0 {
                        let interval = if pump.needs_completion_poll() {
                            TX_COMPLETION_POLL_INTERVAL
                        } else {
                            PUMP_IDLE_SLEEP
                        };
                        std::thread::sleep(interval);
                    }
                }
                pump.finish(&*bus);
                log::debug!("pump stopped: {}", pump.summary());
            }
        })
        .expect("the pump thread is spawned once per stream start");
    PumpThread {
        stop,
        join: Some(join),
        state,
    }
}

fn separate_tx_from_network_irq() {
    // Pluto's GEM interrupt runs on CPU0, while the TX FIFO has only milliseconds of runway.
    // SAFETY: both affinity masks are initialized and their sizes match the libc calls.
    unsafe {
        let mut allowed: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(0, std::mem::size_of_val(&allowed), &mut allowed) != 0 {
            log::warn!(
                "cannot read TX pump CPU affinity: {}",
                std::io::Error::last_os_error()
            );
            return;
        }
        let Some(cpu) = (1..libc::CPU_SETSIZE as usize)
            .rev()
            .find(|&cpu| libc::CPU_ISSET(cpu, &allowed))
        else {
            return;
        };
        let mut selected: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_SET(cpu, &mut selected);
        if libc::sched_setaffinity(0, std::mem::size_of_val(&selected), &selected) != 0 {
            log::warn!(
                "cannot place TX pump on CPU{cpu}: {}",
                std::io::Error::last_os_error()
            );
        } else {
            log::debug!("TX pump on CPU{cpu}");
        }
    }
}
