//! Register access for the LiteX SoC: the `CsrBus` trait, `/dev/mem` and UIO
//! backends, an in-memory fake, and the `csr.json` register map.
//!
//! Names are the contract with the gateware, not addresses. The map is loaded
//! at startup from a `csr.json` produced by the gateware build, so one binary
//! serves any bitstream that exports the names in `docs/csr-contract.md`.

mod devmem;
mod fake;
mod map;
mod uio;

pub use devmem::DevMem;
pub use fake::{FakeBus, ReadHook, WriteHook};
pub use map::{Access, CsrMap, CsrMemory, CsrRegister, MapError, Reg};
pub use uio::Uio;

/// Bytes per CSR word. LiteX places consecutive sub-registers of a wide CSR
/// one word apart, most significant word first.
pub const CSR_WORD_BYTES: u64 = 4;

/// Fixed value of the SoC's read-only magic register, ASCII "PSKS". The daemon
/// reads it at startup to confirm a coherent link to a fully loaded SoC before
/// configuring anything.
pub const SOC_MAGIC: u32 = 0x5053_4b53;
pub const SOC_LAYOUT_VERSION: u32 = 3;
/// Read-only magic register in the gateware's `csr.json`.
pub const REG_SOC_MAGIC: &str = "soc_magic_value";
/// Read-only CSR layout version, incremented by the gateware on any layout
/// change.
pub const REG_SOC_VERSION: &str = "soc_version_value";

/// 32-bit access to the SoC's memory-mapped registers at absolute addresses.
pub trait CsrBus: Send + Sync {
    fn read32(&self, addr: u64) -> u32;
    fn write32(&self, addr: u64, value: u32);

    fn read_block(&self, addr: u64, out: &mut [u32]) {
        for (i, word) in out.iter_mut().enumerate() {
            *word = self.read32(addr + i as u64 * CSR_WORD_BYTES);
        }
    }

    fn write_block(&self, addr: u64, words: &[u32]) {
        for (i, word) in words.iter().enumerate() {
            self.write32(addr + i as u64 * CSR_WORD_BYTES, *word);
        }
    }
}

impl<T: CsrBus + ?Sized> CsrBus for std::sync::Arc<T> {
    fn read32(&self, addr: u64) -> u32 {
        (**self).read32(addr)
    }
    fn write32(&self, addr: u64, value: u32) {
        (**self).write32(addr, value)
    }
    fn read_block(&self, addr: u64, out: &mut [u32]) {
        (**self).read_block(addr, out)
    }
    fn write_block(&self, addr: u64, words: &[u32]) {
        (**self).write_block(addr, words)
    }
}

/// Reads the SoC magic and version and fails if the magic does not match, so a
/// stale or unprogrammed bitstream is caught before any register is written.
pub fn check_soc_identity(bus: &dyn CsrBus, map: &CsrMap) -> Result<u32, MapError> {
    let magic = map.reg(REG_SOC_MAGIC, 1)?.read(bus) as u32;
    if magic != SOC_MAGIC {
        return Err(MapError::SocMagic {
            found: magic,
            expected: SOC_MAGIC,
        });
    }
    let version = map.reg(REG_SOC_VERSION, 1)?.read(bus) as u32;
    if version != SOC_LAYOUT_VERSION {
        return Err(MapError::SocVersion {
            found: version,
            expected: SOC_LAYOUT_VERSION,
        });
    }
    Ok(version)
}
