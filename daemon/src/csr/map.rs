//! Loader for LiteX's `csr.json` export.

use std::collections::BTreeMap;
use std::ops::Range;

use serde::Deserialize;
use thiserror::Error;

use super::{CSR_WORD_BYTES, CsrBus};

/// The CSR data width this daemon is written for.
const SUPPORTED_CSR_DATA_WIDTH: u64 = 32;
/// Name of the LiteX region that holds the CSRs. Anchors `--csr-base`.
const CSR_REGION: &str = "csr";
const IO_MEMORY_TYPE: &str = "io";

#[derive(Debug, Error)]
pub enum MapError {
    #[error("read {path}: {source}")]
    Read {
        path: String,
        source: std::io::Error,
    },
    #[error("csr.json parse error: {0}")]
    Parse(#[from] serde_json::Error),
    #[error("csr.json is missing register(s): {0}")]
    MissingRegisters(String),
    #[error("csr.json is missing memory `{0}`")]
    MissingMemory(String),
    #[error("csr.json is missing constant `{0}`")]
    MissingConstant(String),
    #[error("csr.json is missing csr base `{0}`")]
    MissingBase(String),
    #[error("csr.json constant `{name}` is not an integer: {value}")]
    ConstantNotInteger { name: String, value: String },
    #[error("csr.json register `{name}` is {words} words; expected {expected}")]
    RegisterWidth {
        name: String,
        words: u32,
        expected: u32,
    },
    #[error("csr.json memory `{name}` is {size} bytes; {reason}")]
    MemorySize {
        name: String,
        size: u64,
        reason: String,
    },
    #[error("csr.json has config_csr_data_width={0}; only 32 is supported")]
    CsrDataWidth(u64),
    #[error("csr.json has channels={0}; expected 1 or 2")]
    ChannelCount(u64),
    #[error(
        "SoC magic is {found:#010x}, expected {expected:#010x}: wrong or unprogrammed bitstream"
    )]
    SocMagic { found: u32, expected: u32 },
    #[error("SoC CSR layout is version {found}, expected {expected}: install matching gateware")]
    SocVersion { found: u32, expected: u32 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Access {
    Ro,
    Rw,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
pub struct CsrRegister {
    pub addr: u64,
    /// Number of CSR words the register spans.
    pub size: u32,
    #[serde(rename = "type")]
    pub access: Access,
}

#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct CsrMemory {
    pub base: u64,
    pub size: u64,
    #[serde(rename = "type")]
    pub kind: String,
}

#[derive(Clone, Debug, Deserialize)]
pub struct CsrMap {
    #[serde(default)]
    csr_bases: BTreeMap<String, u64>,
    #[serde(default)]
    csr_registers: BTreeMap<String, CsrRegister>,
    #[serde(default)]
    constants: BTreeMap<String, serde_json::Value>,
    #[serde(default)]
    memories: BTreeMap<String, CsrMemory>,
}

/// A register handle: address plus width in CSR words. Wide registers are
/// read and written most-significant word first, matching LiteX, which
/// commits a wide storage register when its last (least significant) word
/// is written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reg {
    pub addr: u64,
    pub words: u32,
}

impl Reg {
    pub fn read(&self, bus: &dyn CsrBus) -> u64 {
        match self.words {
            1 => bus.read32(self.addr) as u64,
            _ => {
                // The two halves are not read atomically. Retry until the
                // high word is stable across the low-word read.
                let lo_addr = self.addr + CSR_WORD_BYTES;
                let mut hi = bus.read32(self.addr);
                loop {
                    let lo = bus.read32(lo_addr);
                    let hi2 = bus.read32(self.addr);
                    if hi2 == hi {
                        return ((hi as u64) << 32) | lo as u64;
                    }
                    hi = hi2;
                }
            }
        }
    }

    pub fn write(&self, bus: &dyn CsrBus, value: u64) {
        match self.words {
            1 => bus.write32(self.addr, value as u32),
            _ => {
                bus.write32(self.addr, (value >> 32) as u32);
                bus.write32(self.addr + CSR_WORD_BYTES, value as u32);
            }
        }
    }
}

impl CsrMap {
    pub fn from_json(text: &str) -> Result<Self, MapError> {
        let map: CsrMap = serde_json::from_str(text)?;
        if let Some(width) = map.constant_u64_opt("config_csr_data_width")?
            && width != SUPPORTED_CSR_DATA_WIDTH
        {
            return Err(MapError::CsrDataWidth(width));
        }
        Ok(map)
    }

    /// Reads and parses a `csr.json` from the gateware build.
    pub fn from_path(path: &std::path::Path) -> Result<Self, MapError> {
        let text = std::fs::read_to_string(path).map_err(|source| MapError::Read {
            path: path.display().to_string(),
            source,
        })?;
        Self::from_json(&text)
    }

    pub fn register(&self, name: &str) -> Result<&CsrRegister, MapError> {
        self.csr_registers
            .get(name)
            .ok_or_else(|| MapError::MissingRegisters(name.to_string()))
    }

    /// Looks up a register and checks its width, so a gateware whose wide
    /// registers were narrowed (or vice versa) fails at startup.
    pub fn reg(&self, name: &str, words: u32) -> Result<Reg, MapError> {
        let r = self.register(name)?;
        if r.size != words {
            return Err(MapError::RegisterWidth {
                name: name.to_string(),
                words: r.size,
                expected: words,
            });
        }
        Ok(Reg {
            addr: r.addr,
            words,
        })
    }

    /// Reports every missing name at once rather than the first one.
    pub fn require_registers(&self, names: &[&str]) -> Result<(), MapError> {
        let missing: Vec<&str> = names
            .iter()
            .copied()
            .filter(|n| !self.csr_registers.contains_key(*n))
            .collect();
        if missing.is_empty() {
            Ok(())
        } else {
            Err(MapError::MissingRegisters(missing.join(", ")))
        }
    }

    pub fn memory(&self, name: &str) -> Result<&CsrMemory, MapError> {
        self.memories
            .get(name)
            .ok_or_else(|| MapError::MissingMemory(name.to_string()))
    }

    pub fn base(&self, name: &str) -> Result<u64, MapError> {
        self.csr_bases
            .get(name)
            .copied()
            .ok_or_else(|| MapError::MissingBase(name.to_string()))
    }

    pub fn constant(&self, name: &str) -> Option<&serde_json::Value> {
        self.constants.get(name)
    }

    pub fn constant_u64(&self, name: &str) -> Result<u64, MapError> {
        self.constant_u64_opt(name)?
            .ok_or_else(|| MapError::MissingConstant(name.to_string()))
    }

    /// RF channels implemented by this bitstream.
    pub fn channels(&self) -> Result<usize, MapError> {
        match self.constant_u64("channels")? {
            channels @ 1..=2 => Ok(channels as usize),
            channels => Err(MapError::ChannelCount(channels)),
        }
    }

    fn constant_u64_opt(&self, name: &str) -> Result<Option<u64>, MapError> {
        match self.constants.get(name) {
            None => Ok(None),
            Some(v) => v
                .as_u64()
                .map(Some)
                .ok_or_else(|| MapError::ConstantNotInteger {
                    name: name.to_string(),
                    value: v.to_string(),
                }),
        }
    }

    /// Base of the CSR region as exported by the gateware build.
    pub fn csr_region_base(&self) -> Option<u64> {
        self.memories
            .get(CSR_REGION)
            .map(|m| m.base)
            .or_else(|| self.csr_bases.values().min().copied())
    }

    /// Moves every address so the CSR region starts at `new_base`. Lets one
    /// register map serve a bitstream whose AXI GP0 aperture was placed
    /// elsewhere.
    pub fn rebase(&mut self, new_base: u64) {
        let Some(old_base) = self.csr_region_base() else {
            return;
        };
        let shift = |addr: u64| addr.wrapping_sub(old_base).wrapping_add(new_base);
        for v in self.csr_bases.values_mut() {
            *v = shift(*v);
        }
        for r in self.csr_registers.values_mut() {
            r.addr = shift(r.addr);
        }
        for m in self.memories.values_mut() {
            m.base = shift(m.base);
        }
    }

    /// Smallest address range covering every register and I/O memory. This is
    /// the range a `/dev/mem` window must map.
    pub fn io_span(&self) -> Option<Range<u64>> {
        let mut lo = u64::MAX;
        let mut hi = 0u64;
        let mut any = false;
        for r in self.csr_registers.values() {
            lo = lo.min(r.addr);
            hi = hi.max(r.addr + r.size as u64 * CSR_WORD_BYTES);
            any = true;
        }
        for m in self.memories.values().filter(|m| m.kind == IO_MEMORY_TYPE) {
            lo = lo.min(m.base);
            hi = hi.max(m.base + m.size);
            any = true;
        }
        for b in self.csr_bases.values() {
            lo = lo.min(*b);
            hi = hi.max(*b + CSR_WORD_BYTES);
            any = true;
        }
        any.then_some(lo..hi)
    }

    pub fn registers(&self) -> impl Iterator<Item = (&str, &CsrRegister)> {
        self.csr_registers.iter().map(|(k, v)| (k.as_str(), v))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csr::FakeBus;

    const SAMPLE: &str = r#"{
        "csr_bases": {"rxpkt0": 4096, "identifier_mem": 8192},
        "csr_registers": {
            "rxpkt0_ctrl": {"addr": 4096, "size": 1, "type": "rw"},
            "timebase_now": {"addr": 4100, "size": 2, "type": "ro"}
        },
        "constants": {"channels": 2, "config_clock_frequency": 100000000, "config_csr_data_width": 32,
                      "config_cpu_type_zynq7000": null, "config_bus_standard": "AXI-LITE"},
        "memories": {
            "csr": {"base": 4096, "size": 65536, "type": "io"},
            "rxpkt0_header": {"base": 73728, "size": 64, "type": "io"},
            "sram": {"base": 268435456, "size": 8192, "type": "cached"}
        }
    }"#;

    #[test]
    fn parses_registers_memories_constants() {
        let map = CsrMap::from_json(SAMPLE).unwrap();
        let ctrl = map.register("rxpkt0_ctrl").unwrap();
        assert_eq!(ctrl.addr, 4096);
        assert_eq!(ctrl.access, Access::Rw);
        let tick = map.register("timebase_now").unwrap();
        assert_eq!((tick.size, tick.access), (2, Access::Ro));
        assert_eq!(map.memory("rxpkt0_header").unwrap().size, 64);
        assert_eq!(
            map.constant_u64("config_clock_frequency").unwrap(),
            100_000_000
        );
        assert_eq!(map.base("identifier_mem").unwrap(), 8192);
        assert_eq!(map.channels().unwrap(), 2);
        assert!(matches!(
            map.constant_u64("config_bus_standard"),
            Err(MapError::ConstantNotInteger { .. })
        ));
    }

    #[test]
    fn channel_count_follows_the_bitstream_map() {
        let one_channel = SAMPLE.replace("\"channels\": 2", "\"channels\": 1");
        assert_eq!(
            CsrMap::from_json(&one_channel).unwrap().channels().unwrap(),
            1
        );

        let unsupported = SAMPLE.replace("\"channels\": 2", "\"channels\": 3");
        assert!(matches!(
            CsrMap::from_json(&unsupported).unwrap().channels(),
            Err(MapError::ChannelCount(3))
        ));

        let absent = SAMPLE.replace("\"channels\": 2, ", "");
        assert!(matches!(
            CsrMap::from_json(&absent).unwrap().channels(),
            Err(MapError::MissingConstant(name)) if name == "channels"
        ));
    }

    #[test]
    fn missing_names_are_listed_together() {
        let map = CsrMap::from_json(SAMPLE).unwrap();
        let err = map
            .require_registers(&["rxpkt0_ctrl", "rxpkt0_frame_len", "rxpkt0_produced"])
            .unwrap_err();
        assert_eq!(
            err.to_string(),
            "csr.json is missing register(s): rxpkt0_frame_len, rxpkt0_produced"
        );
        assert!(matches!(
            map.memory("nope"),
            Err(MapError::MissingMemory(_))
        ));
        assert!(matches!(
            map.reg("timebase_now", 1),
            Err(MapError::RegisterWidth {
                words: 2,
                expected: 1,
                ..
            })
        ));
    }

    #[test]
    fn wide_register_msb_word_at_lower_address() {
        let map = CsrMap::from_json(SAMPLE).unwrap();
        let tick = map.reg("timebase_now", 2).unwrap();
        let bus = FakeBus::new();
        bus.set(4100, 0x1234_5678);
        bus.set(4104, 0x9abc_def0);
        assert_eq!(tick.read(&bus), 0x1234_5678_9abc_def0);
        tick.write(&bus, 0x0000_0001_0000_0002);
        assert_eq!(bus.writes_to(4100), vec![1]);
        assert_eq!(bus.writes_to(4104), vec![2]);
        assert_eq!(
            bus.take_writes(),
            vec![(4100, 1), (4104, 2)],
            "high word written first so the low-word write commits the value"
        );
    }

    #[test]
    fn wide_read_retries_when_high_word_moves() {
        let map = CsrMap::from_json(SAMPLE).unwrap();
        let tick = map.reg("timebase_now", 2).unwrap();
        let bus = FakeBus::new();
        bus.set(4100, 0);
        bus.set(4104, 0xffff_ffff);
        let mut reads = 0;
        bus.set_read_hook(Box::new(move |addr, mem| {
            reads += 1;
            // A carry into the high word can land right after the first low-word read.
            if addr == 4104 && reads == 2 {
                mem.insert(4100, 1);
                mem.insert(4104, 0);
            }
            None
        }));
        assert_eq!(tick.read(&bus), 0x1_0000_0000);
    }

    #[test]
    fn rebase_shifts_everything_and_span_covers_io_only() {
        let mut map = CsrMap::from_json(SAMPLE).unwrap();
        assert_eq!(map.io_span().unwrap(), 4096..73792);
        map.rebase(0x43C0_0000);
        assert_eq!(map.register("rxpkt0_ctrl").unwrap().addr, 0x43C0_0000);
        assert_eq!(map.register("timebase_now").unwrap().addr, 0x43C0_0004);
        assert_eq!(map.memory("rxpkt0_header").unwrap().base, 0x43C1_1000);
        assert_eq!(map.base("identifier_mem").unwrap(), 0x43C0_1000);
        assert_eq!(
            map.memory("sram").unwrap().base,
            268435456 - 4096 + 0x43C0_0000
        );
    }

    #[test]
    fn rejects_unsupported_csr_width() {
        let text = SAMPLE.replace(
            "\"config_csr_data_width\": 32",
            "\"config_csr_data_width\": 8",
        );
        assert!(matches!(
            CsrMap::from_json(&text),
            Err(MapError::CsrDataWidth(8))
        ));
    }
}
