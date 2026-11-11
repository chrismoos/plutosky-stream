//! UIO backend: one `mmap` of a `/dev/uioN` map, for a kernel that exports the
//! CSR aperture as a UIO device instead of leaving it to `/dev/mem`.
//!
//! The mapped window starts at the UIO map's own base, so register addresses
//! from `csr.json` are rebased onto it by the caller before use.

use std::fs::OpenOptions;
use std::io;
use std::os::unix::io::AsRawFd;
use std::ptr;

use super::{CSR_WORD_BYTES, CsrBus};

/// UIO exposes map N at `mmap` offset `N * page_size`. The daemon only ever
/// uses map 0, the CSR aperture.
const UIO_MAP_INDEX: u64 = 0;

pub struct Uio {
    window: *mut u8,
    window_len: usize,
}

// The mapping is shared device memory and every access is a volatile word
// read/write, so concurrent use from several threads is sound.
unsafe impl Send for Uio {}
unsafe impl Sync for Uio {}

impl Uio {
    /// Maps `len` bytes of map 0 of `path` (for example `/dev/uio0`).
    pub fn open(path: &str, len: usize) -> io::Result<Self> {
        if len == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "empty register span",
            ));
        }
        let page = page_size()?;
        let window_len = len.div_ceil(page as usize) * page as usize;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| io::Error::new(e.kind(), format!("open {path}: {e}")))?;
        // SAFETY: the fd is open for read and write and outlives the call. A
        // failed mapping is reported as MAP_FAILED and handled below.
        let window = unsafe {
            libc::mmap(
                ptr::null_mut(),
                window_len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                (UIO_MAP_INDEX * page) as libc::off_t,
            )
        };
        if window == libc::MAP_FAILED {
            let e = io::Error::last_os_error();
            return Err(io::Error::new(
                e.kind(),
                format!("mmap {path} ({window_len} bytes): {e}"),
            ));
        }
        Ok(Self {
            window: window as *mut u8,
            window_len,
        })
    }

    fn slot(&self, addr: u64) -> *mut u32 {
        assert_eq!(
            addr % CSR_WORD_BYTES,
            0,
            "unaligned CSR access at {addr:#x}"
        );
        assert!(
            addr + CSR_WORD_BYTES <= self.window_len as u64,
            "CSR access at {addr:#x} outside the mapped UIO window of {} bytes",
            self.window_len
        );
        // SAFETY: the offset is inside the mapping, checked just above.
        unsafe { self.window.add(addr as usize) as *mut u32 }
    }
}

fn page_size() -> io::Result<u64> {
    // SAFETY: sysconf takes no pointers and reports failure as a negative value.
    let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    if page <= 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(page as u64)
}

impl CsrBus for Uio {
    fn read32(&self, addr: u64) -> u32 {
        // SAFETY: slot() bounds-checks and aligns the address inside the mapping.
        unsafe { ptr::read_volatile(self.slot(addr)) }
    }

    fn write32(&self, addr: u64, value: u32) {
        // SAFETY: slot() bounds-checks and aligns the address inside the mapping.
        unsafe { ptr::write_volatile(self.slot(addr), value) }
    }
}

impl Drop for Uio {
    fn drop(&mut self) {
        // SAFETY: the pointer and length are the ones returned by mmap in open().
        unsafe {
            libc::munmap(self.window as *mut libc::c_void, self.window_len);
        }
    }
}
