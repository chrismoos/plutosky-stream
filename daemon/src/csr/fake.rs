//! In-memory `CsrBus` for tests and `--fake`: records writes and lets a
//! hook script what reads return.

use std::collections::HashMap;

use parking_lot::Mutex;

use super::CsrBus;

/// Called before each read with the address and the backing store. A
/// `Some` return overrides the stored value.
pub type ReadHook = Box<dyn FnMut(u64, &mut HashMap<u64, u32>) -> Option<u32> + Send>;
/// Called after each write with the address, value and the backing store.
pub type WriteHook = Box<dyn FnMut(u64, u32, &mut HashMap<u64, u32>) + Send>;

#[derive(Default)]
struct Inner {
    mem: HashMap<u64, u32>,
    writes: Vec<(u64, u32)>,
    read_hook: Option<ReadHook>,
    write_hook: Option<WriteHook>,
}

#[derive(Default)]
pub struct FakeBus {
    inner: Mutex<Inner>,
}

impl FakeBus {
    pub fn new() -> Self {
        Self::default()
    }

    /// Stores a value without recording it as a write.
    pub fn set(&self, addr: u64, value: u32) {
        self.inner.lock().mem.insert(addr, value);
    }

    /// Stored value, bypassing the read hook.
    pub fn get(&self, addr: u64) -> u32 {
        self.inner.lock().mem.get(&addr).copied().unwrap_or(0)
    }

    /// Every write so far, in order. Clears the log.
    pub fn take_writes(&self) -> Vec<(u64, u32)> {
        std::mem::take(&mut self.inner.lock().writes)
    }

    /// Values written to `addr` so far, in order (log left intact).
    pub fn writes_to(&self, addr: u64) -> Vec<u32> {
        self.inner
            .lock()
            .writes
            .iter()
            .filter(|(a, _)| *a == addr)
            .map(|(_, v)| *v)
            .collect()
    }

    pub fn set_read_hook(&self, hook: ReadHook) {
        self.inner.lock().read_hook = Some(hook);
    }

    pub fn set_write_hook(&self, hook: WriteHook) {
        self.inner.lock().write_hook = Some(hook);
    }
}

impl CsrBus for FakeBus {
    fn read32(&self, addr: u64) -> u32 {
        let mut inner = self.inner.lock();
        let inner = &mut *inner;
        if let Some(hook) = inner.read_hook.as_mut()
            && let Some(value) = hook(addr, &mut inner.mem)
        {
            return value;
        }
        inner.mem.get(&addr).copied().unwrap_or(0)
    }

    fn write32(&self, addr: u64, value: u32) {
        let mut inner = self.inner.lock();
        let inner = &mut *inner;
        inner.mem.insert(addr, value);
        inner.writes.push((addr, value));
        if let Some(hook) = inner.write_hook.as_mut() {
            hook(addr, value, &mut inner.mem);
        }
    }
}
