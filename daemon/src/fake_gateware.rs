//! A small model of the gateware behind a [`FakeBus`], so `--fake` runs the
//! whole daemon on a host with no board.
//!
//! It answers the identity registers, runs the timebase off the host clock at
//! the configured sample rate, advances each enabled packetizer's `produced`
//! counter at the rate its packets would come out, and echoes submitted slots
//! back through the depacketizer's consumed FIFO. It models timing, not
//! samples: nothing writes a frame into the slot pool.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

use parking_lot::Mutex;

use crate::csr::{CsrMap, FakeBus, MapError, Reg, SOC_MAGIC};
use crate::stream::regs::{CTRL_ENABLE, MAX_STREAMS};
use crate::time::{self, Timebase};

/// CSR layout version the model reports.
const FAKE_SOC_VERSION: u32 = 3;

struct Packetizer {
    ctrl: Reg,
    produced: Reg,
    samples_per_packet: Reg,
    /// When the core was enabled, or `None` while it is stopped.
    running_since: Option<Instant>,
    /// Packets produced before the current run started.
    produced_base: u64,
}

struct Depacketizer {
    submit: Reg,
    consumed: Reg,
    consumed_count: Reg,
    /// Slots handed over and not yet reported consumed.
    in_flight: Vec<u16>,
}

struct Model {
    started: Instant,
    sample_rate_hz: u32,
    timebase: Timebase,
    /// Added to the elapsed tick count, so a loaded time takes effect.
    tick_offset: u64,
    packetizers: Vec<Packetizer>,
    depacketizers: Vec<Depacketizer>,
}

impl Model {
    fn elapsed_ticks(&self) -> u64 {
        time::nanos_to_ticks(
            self.started.elapsed().as_nanos() as u64,
            self.sample_rate_hz,
        )
    }

    fn now_ticks(&self) -> u64 {
        self.tick_offset.wrapping_add(self.elapsed_ticks())
    }
}

/// Installs the model's hooks on `bus`.
pub fn install(bus: &FakeBus, map: &CsrMap, sample_rate_hz: u32) -> Result<(), MapError> {
    let timebase = Timebase::resolve(map)?;
    // Model only the instances this register map exports, the same way the
    // stream manager counts them.
    let mut packetizers = Vec::with_capacity(MAX_STREAMS);
    let mut depacketizers = Vec::with_capacity(MAX_STREAMS);
    for index in 0..MAX_STREAMS {
        let Ok(ctrl) = map.reg(&format!("rxpkt{index}_ctrl"), 1) else {
            break;
        };
        packetizers.push(Packetizer {
            ctrl,
            produced: map.reg(&format!("rxpkt{index}_produced"), 1)?,
            samples_per_packet: map.reg(&format!("rxpkt{index}_samples_per_packet"), 1)?,
            running_since: None,
            produced_base: 0,
        });
        depacketizers.push(Depacketizer {
            submit: map.reg(&format!("txpkt{index}_submit"), 1)?,
            consumed: map.reg(&format!("txpkt{index}_consumed"), 1)?,
            consumed_count: map.reg(&format!("txpkt{index}_consumed_count"), 1)?,
            in_flight: Vec::new(),
        });
    }

    bus.set(map.reg(crate::csr::REG_SOC_MAGIC, 1)?.addr, SOC_MAGIC);
    bus.set(
        map.reg(crate::csr::REG_SOC_VERSION, 1)?.addr,
        FAKE_SOC_VERSION,
    );
    for packetizer in &packetizers {
        bus.set(packetizer.produced.addr, 0);
    }

    let model = Arc::new(Mutex::new(Model {
        started: Instant::now(),
        sample_rate_hz,
        timebase,
        tick_offset: 0,
        packetizers,
        depacketizers,
    }));

    bus.set_write_hook(Box::new({
        let model = model.clone();
        move |addr, value, mem| on_write(&mut model.lock(), addr, value, mem)
    }));
    bus.set_read_hook(Box::new({
        let model = model.clone();
        move |addr, mem| on_read(&mut model.lock(), addr, mem)
    }));
    Ok(())
}

fn on_write(model: &mut Model, addr: u64, value: u32, mem: &mut HashMap<u64, u32>) {
    if addr == model.timebase.ctrl.addr {
        let bits = u64::from(value);
        if bits & time::CTRL_LOAD != 0 {
            let loaded = wide(mem, model.timebase.load_hi, model.timebase.load_lo);
            model.tick_offset = loaded.wrapping_sub(model.elapsed_ticks());
        }
        if bits & time::CTRL_LATCH != 0 {
            let now = model.now_ticks();
            mem.insert(model.timebase.now_hi.addr, (now >> 32) as u32);
            mem.insert(model.timebase.now_lo.addr, now as u32);
        }
        return;
    }
    for index in 0..model.packetizers.len() {
        if addr == model.packetizers[index].ctrl.addr {
            let enabled = value & CTRL_ENABLE != 0;
            let running = model.packetizers[index].running_since.is_some();
            if enabled && !running {
                model.packetizers[index].running_since = Some(Instant::now());
            } else if !enabled && running {
                let produced = produced_now(model, index, mem);
                let packetizer = &mut model.packetizers[index];
                packetizer.produced_base = produced;
                packetizer.running_since = None;
            }
            return;
        }
        if addr == model.depacketizers[index].submit.addr {
            model.depacketizers[index].in_flight.push(value as u16);
            return;
        }
    }
}

fn on_read(model: &mut Model, addr: u64, mem: &mut HashMap<u64, u32>) -> Option<u32> {
    for index in 0..model.packetizers.len() {
        if addr == model.packetizers[index].produced.addr {
            return Some(produced_now(model, index, mem) as u32);
        }
        if addr == model.depacketizers[index].consumed_count.addr {
            return Some(model.depacketizers[index].in_flight.len() as u32);
        }
        if addr == model.depacketizers[index].consumed.addr {
            let queue = &mut model.depacketizers[index].in_flight;
            return Some(if queue.is_empty() {
                0
            } else {
                u32::from(queue.remove(0))
            });
        }
    }
    None
}

/// Packets an enabled packetizer would have produced by now.
fn produced_now(model: &Model, index: usize, mem: &HashMap<u64, u32>) -> u64 {
    let packetizer = &model.packetizers[index];
    let Some(since) = packetizer.running_since else {
        return packetizer.produced_base;
    };
    let samples_per_packet = mem
        .get(&packetizer.samples_per_packet.addr)
        .copied()
        .unwrap_or(0);
    if samples_per_packet == 0 {
        return packetizer.produced_base;
    }
    let ticks = time::nanos_to_ticks(since.elapsed().as_nanos() as u64, model.sample_rate_hz);
    packetizer.produced_base + ticks / u64::from(samples_per_packet)
}

fn wide(mem: &HashMap<u64, u32>, hi: Reg, lo: Reg) -> u64 {
    let hi = mem.get(&hi.addr).copied().unwrap_or(0);
    let lo = mem.get(&lo.addr).copied().unwrap_or(0);
    (u64::from(hi) << 32) | u64::from(lo)
}
