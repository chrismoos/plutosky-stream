//! `/dev/mem` backend: one `mmap` window covering the SoC's CSR aperture.

use std::fs::OpenOptions;
use std::io;
use std::ops::Range;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::ptr;

use super::{CSR_WORD_BYTES, CsrBus};

const DEV_MEM: &str = "/dev/mem";

pub struct DevMem {
    window: *mut u8,
    window_start: u64,
    window_len: usize,
}

// The mapping is shared device memory. Every access is a volatile word
// read/write, so concurrent use from several threads is sound.
unsafe impl Send for DevMem {}
unsafe impl Sync for DevMem {}

impl DevMem {
    /// Maps the pages covering `span` (absolute physical addresses).
    pub fn open(span: Range<u64>) -> io::Result<Self> {
        if span.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "empty register span",
            ));
        }
        // SAFETY: sysconf takes no pointers and reports failure as a negative value.
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
        if page <= 0 {
            return Err(io::Error::last_os_error());
        }
        let page = page as u64;
        let window_start = span.start & !(page - 1);
        let window_end = span.end.div_ceil(page) * page;
        let window_len = (window_end - window_start) as usize;

        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open(DEV_MEM)
            .map_err(|e| io::Error::new(e.kind(), format!("open {DEV_MEM}: {e}")))?;
        // SAFETY: the fd is open for read and write and outlives the call. A
        // failed mapping is reported as MAP_FAILED and handled below.
        let window = unsafe {
            libc::mmap(
                ptr::null_mut(),
                window_len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                window_start as libc::off_t,
            )
        };
        if window == libc::MAP_FAILED {
            let e = io::Error::last_os_error();
            return Err(io::Error::new(
                e.kind(),
                format!("mmap {DEV_MEM} at {window_start:#x} ({window_len} bytes): {e}"),
            ));
        }
        Ok(Self {
            window: window as *mut u8,
            window_start,
            window_len,
        })
    }

    fn slot(&self, addr: u64) -> *mut u32 {
        assert_eq!(
            addr % CSR_WORD_BYTES,
            0,
            "unaligned CSR access at {addr:#x}"
        );
        let off = addr
            .checked_sub(self.window_start)
            .filter(|o| o + CSR_WORD_BYTES <= self.window_len as u64)
            .unwrap_or_else(|| {
                panic!(
                    "CSR access at {addr:#x} outside the mapped window {:#x}..{:#x}",
                    self.window_start,
                    self.window_start + self.window_len as u64
                )
            });
        // SAFETY: the offset is inside the mapping, checked just above.
        unsafe { self.window.add(off as usize) as *mut u32 }
    }
}

impl CsrBus for DevMem {
    fn read32(&self, addr: u64) -> u32 {
        // SAFETY: slot() bounds-checks and aligns the address inside the mapping.
        unsafe { ptr::read_volatile(self.slot(addr)) }
    }

    fn write32(&self, addr: u64, value: u32) {
        // SAFETY: slot() bounds-checks and aligns the address inside the mapping.
        unsafe { ptr::write_volatile(self.slot(addr), value) }
    }
}

impl Drop for DevMem {
    fn drop(&mut self) {
        // SAFETY: the pointer and length are the ones returned by mmap in open().
        unsafe {
            libc::munmap(self.window as *mut libc::c_void, self.window_len);
        }
    }
}
