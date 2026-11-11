//! `plutosky-streamd`: brings up the radio and the stream cores, then serves
//! the gRPC control service.

use std::path::PathBuf;
use std::sync::Arc;

use clap::Parser;

use plutosky_streamd::control::{
    ControlService, DEFAULT_CONTROL_PORT, DISCOVERY_PORT, DiscoveryResponder, serve,
};
use plutosky_streamd::csr::{CsrBus, CsrMap, FakeBus, check_soc_identity};
use plutosky_streamd::daemon::{DAEMON_NAME, Daemon, Identity};
use plutosky_streamd::gem::{DEFAULT_GEMSLOT_PATH, FakeSlotDevice, Slots};
use plutosky_streamd::net::{LinkResolver, MacAddr};
use plutosky_streamd::radio::{FakeRadio, Radio};
use plutosky_streamd::stream::StreamManager;
use plutosky_streamd::time::Timebase;
use plutosky_streamd::{fake_gateware, testing};

const DEVICE_TREE_SERIAL: &str = "/proc/device-tree/serial-number";
const DEFAULT_INTERFACE: &str = "eth0";
/// The board has two Cortex-A9 cores. Radio calls run on the blocking pool.
const CONTROL_WORKER_THREADS: usize = 2;
#[cfg(not(feature = "hardware"))]
const BOARD_REF_CLK_HZ: u32 = 40_000_000;

#[derive(Parser, Debug)]
#[command(name = DAEMON_NAME, about = "On-board daemon for the plutosky-stream SDR")]
struct Cli {
    #[arg(long, value_name = "PATH")]
    csr_json: PathBuf,

    /// Physical address the CSR region was placed at, when it differs from the
    /// one in the register map.
    #[arg(long, value_name = "ADDR", value_parser = parse_address)]
    csr_base: Option<u64>,

    /// Reach the registers through this UIO device instead of `/dev/mem`.
    #[arg(long, value_name = "PATH")]
    uio: Option<String>,

    #[arg(long, value_name = "PATH", default_value = DEFAULT_GEMSLOT_PATH)]
    gemslot: String,

    #[arg(long, default_value_t = DEFAULT_CONTROL_PORT)]
    control_port: u16,

    #[arg(long, default_value_t = DISCOVERY_PORT)]
    discovery_port: u16,

    /// Interface whose MAC the discovery reply reports.
    #[arg(long, value_name = "NAME", default_value = DEFAULT_INTERFACE)]
    interface: String,

    /// Sample rate the AD9363 starts at, before any client sets one.
    #[arg(long, value_name = "HZ", default_value_t = plutosky_streamd::ad9361::DEFAULT_SAMPLE_RATE_HZ)]
    sample_rate_hz: u32,

    /// Require this channel count in the bitstream. By default use its CSR map.
    #[arg(long, value_name = "N", value_parser = clap::value_parser!(u8).range(1..=2))]
    channels: Option<u8>,

    /// Diagnostic: skip the LVDS PRBS calibration at start-up. Samples cross
    /// the interface at whatever alignment the chip's default delays give
    #[arg(long)]
    skip_interface_calibration: bool,

    /// Diagnostic AD9361 digital interface options: 1 invert DATA_CLK, 2 frame
    /// level mode, 4 invert RX_FRAME, 8 swap ports, 16 no port swap, 32 invert
    /// data bus
    #[arg(long, value_name = "BITS", default_value_t = 0)]
    ad9361_flags: u32,

    /// Diagnostic: leave the chip's RX PRBS generator on and serve without
    /// calibrating, so the interface can be inspected while PRBS runs
    #[arg(long)]
    hold_rx_prbs: bool,

    #[arg(long, value_name = "HZ", default_value_t = default_ref_clk_hz())]
    ref_clk_hz: u32,

    /// Run every hardware dependency against a fake: no board, no radio, no
    /// slot device. The whole daemon serves the control protocol on the host.
    #[arg(long)]
    fake: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let cli = Cli::parse();

    // Block termination signals in every thread so only the shutdown thread
    // below receives them, and it can release the radio's slots before exit.
    block_termination_signals();

    let mut map = CsrMap::from_path(&cli.csr_json)?;
    if let Some(base) = cli.csr_base {
        map.rebase(base);
    }
    let channels = map.channels()? as u8;
    if let Some(requested) = cli.channels
        && requested != channels
    {
        return Err(format!(
            "--channels={requested} does not match the bitstream's {channels} channels"
        )
        .into());
    }

    let (bus, slots, radio, links, board_mac) = if cli.fake {
        build_fake(&map)?
    } else {
        build_hardware(&cli, &mut map, channels)?
    };

    let timebase = Timebase::resolve(&map)?;
    let streams = StreamManager::new(bus.clone(), slots, links, &map, timebase)?;
    let identity = Identity {
        serial: board_serial(board_mac),
        version: env!("CARGO_PKG_VERSION").to_string(),
        mac: board_mac,
    };
    let daemon = Arc::new(Daemon::new(radio, streams, identity, cli.control_port));
    log::info!(
        "{DAEMON_NAME} {} up: serial={} mac={board_mac} radio={}",
        env!("CARGO_PKG_VERSION"),
        daemon.identity().serial,
        daemon.describe_radio(),
    );

    let shutdown_daemon = daemon.clone();
    std::thread::Builder::new()
        .name("shutdown".to_string())
        .spawn(move || {
            wait_for_termination_signal();
            log::info!("termination signal: releasing radio slots before exit");
            shutdown_daemon.shutdown();
            std::process::exit(0);
        })?;

    let discovery = DiscoveryResponder::bind(daemon.clone(), cli.discovery_port)?;
    std::thread::Builder::new()
        .name("discovery".to_string())
        .spawn(move || {
            if let Err(error) = discovery.run() {
                log::error!("discovery responder stopped: {error}");
            }
        })?;

    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(CONTROL_WORKER_THREADS)
        .thread_name("control")
        .enable_all()
        .build()?
        .block_on(serve(ControlService::new(daemon), cli.control_port))
        .map_err(|error| error as Box<dyn std::error::Error>)
}

/// The signals the shutdown thread waits on. SIGTERM is what start-stop-daemon
/// and a plain kill send.
fn termination_signal_set() -> libc::sigset_t {
    // SAFETY: zeroed is a valid empty sigset_t that sigemptyset then fills in.
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGTERM);
        libc::sigaddset(&mut set, libc::SIGINT);
        set
    }
}

/// Blocks the termination signals in the calling thread. Threads spawned after
/// this inherit the mask, so the dedicated shutdown thread is the only one that
/// receives them through `sigwait`.
fn block_termination_signals() {
    let set = termination_signal_set();
    // SAFETY: `set` is a live, initialized sigset_t.
    unsafe {
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
    }
}

fn wait_for_termination_signal() {
    let set = termination_signal_set();
    let mut signal: libc::c_int = 0;
    // SAFETY: `set` is a live, initialized sigset_t and `signal` is a live int.
    unsafe {
        libc::sigwait(&set, &mut signal);
    }
}

type Parts = (
    Arc<dyn CsrBus>,
    Arc<dyn Slots>,
    Box<dyn Radio>,
    Arc<dyn LinkResolver>,
    MacAddr,
);

#[cfg(feature = "hardware")]
fn build_hardware(
    cli: &Cli,
    map: &mut CsrMap,
    channels: u8,
) -> Result<Parts, Box<dyn std::error::Error>> {
    use plutosky_streamd::ad9361::{Ad9361, Ad9361Params, Ad9361Regs};
    use plutosky_streamd::csr::{DevMem, Uio};
    use plutosky_streamd::gem::SlotDevice;
    use plutosky_streamd::net::KernelLinkResolver;

    let span = map
        .io_span()
        .ok_or("the register map has no registers to map")?;
    let bus: Arc<dyn CsrBus> = match &cli.uio {
        Some(path) => {
            // A UIO mapping starts at the region's own base, so the register
            // addresses are offsets into it.
            map.rebase(0);
            let span = map.io_span().ok_or("the register map has no registers")?;
            Arc::new(Uio::open(path, span.end as usize)?)
        }
        None => Arc::new(DevMem::open(span)?),
    };
    let version = check_soc_identity(&*bus, map)?;
    log::info!("SoC magic confirmed, CSR layout version {version}");

    let slots: Arc<dyn Slots> = Arc::new(SlotDevice::open(&cli.gemslot)?);
    // Reclaim before radio initialization, which can take several seconds.
    // Ethernet also uses this receive-slot pool and cannot wait for the
    // transceiver calibration after a daemon exits mid stream.
    reclaim_steered_slots(&*slots);

    let regs = Ad9361Regs::resolve(map)?;
    let params = Ad9361Params {
        ref_clk_hz: cli.ref_clk_hz,
        sample_rate_hz: cli.sample_rate_hz,
        two_channels: channels == 2,
        interface_flags: cli.ad9361_flags,
        ..Ad9361Params::default()
    };
    let mut radio = Ad9361::init(bus.clone(), regs.clone(), &params)?;
    if cli.hold_rx_prbs {
        radio.hold_rx_prbs()?;
        log::warn!("RX PRBS held on, calibration skipped, samples are the test pattern");
    } else if cli.skip_interface_calibration {
        log::warn!("LVDS interface calibration skipped, sample alignment is unverified");
    } else {
        radio.calibrate_interface()?;
    }

    let links = KernelLinkResolver;
    let board_mac = links.interface_mac(&cli.interface).unwrap_or_else(|error| {
        log::warn!("no MAC for {}: {error}", cli.interface);
        MacAddr::default()
    });
    Ok((bus, slots, Box::new(radio), Arc::new(links), board_mac))
}

/// Drains the driver's steer ring at start-up and hands every slot back, so
/// slots leaked by an earlier daemon do not shrink the receive free list.
fn reclaim_steered_slots(slots: &dyn Slots) {
    let mut buf = Vec::new();
    let mut total = 0usize;
    loop {
        buf.clear();
        if slots.rx_poll(&mut buf).is_err() || buf.is_empty() {
            break;
        }
        total += buf.len();
        if slots.rx_return(&buf).is_err() {
            break;
        }
    }
    if total > 0 {
        log::info!("reclaimed {total} steered receive slots left by a previous run");
    }
}

#[cfg(not(feature = "hardware"))]
fn build_hardware(
    _cli: &Cli,
    _map: &mut CsrMap,
    _channels: u8,
) -> Result<Parts, Box<dyn std::error::Error>> {
    Err("this build has no hardware support: run with --fake".into())
}

fn build_fake(map: &CsrMap) -> Result<Parts, Box<dyn std::error::Error>> {
    let bus = Arc::new(FakeBus::new());
    fake_gateware::install(&bus, map, testing::FAKE_SAMPLE_RATE_HZ)?;
    check_soc_identity(&*bus, map)?;
    let slots = Arc::new(FakeSlotDevice::new(
        testing::FAKE_SLOT_SIZE,
        testing::FAKE_SLOT_COUNT,
    ));
    // With no GEM behind it the fake would never report a frame sent, so the
    // ring would fill on the first stream.
    slots.set_auto_complete(true);
    let links = testing::fake_link_resolver();
    let board_mac = links.board_mac;
    Ok((
        bus,
        slots,
        Box::new(FakeRadio::new()),
        Arc::new(links),
        board_mac,
    ))
}

/// The board's serial number, falling back to its MAC when the device tree
/// does not provide one.
fn board_serial(mac: MacAddr) -> String {
    match std::fs::read_to_string(DEVICE_TREE_SERIAL) {
        Ok(text) => {
            let serial = text.trim_end_matches('\0').trim();
            if !serial.is_empty() {
                return serial.to_string();
            }
        }
        Err(error) => log::debug!("no serial in {DEVICE_TREE_SERIAL}: {error}"),
    }
    mac.to_string().replace(':', "")
}

#[cfg(feature = "hardware")]
fn default_ref_clk_hz() -> u32 {
    plutosky_streamd::ad9361::DEFAULT_REF_CLK_HZ
}

#[cfg(not(feature = "hardware"))]
fn default_ref_clk_hz() -> u32 {
    BOARD_REF_CLK_HZ
}

fn parse_address(text: &str) -> Result<u64, String> {
    let trimmed = text.trim();
    let parsed = match trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
    {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => trimmed.parse(),
    };
    parsed.map_err(|_| format!("`{text}` is not an address"))
}
