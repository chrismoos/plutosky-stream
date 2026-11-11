//! The real `/dev/gemslot0` client: ioctls plus an `mmap` of the slot pool.

use std::fs::{File, OpenOptions};
use std::io;
use std::os::unix::io::AsRawFd;
use std::ptr;

use super::ioctl::{
    BatchEntry, GEMSLOT_CAP_KERNEL_PUMP, GEMSLOT_GET_CAPS, GEMSLOT_GET_INFO, GEMSLOT_RX_POLL,
    GEMSLOT_RX_RETURN, GEMSLOT_RX_STEER, GEMSLOT_STREAM_START, GEMSLOT_STREAM_STATUS,
    GEMSLOT_STREAM_STOP, GEMSLOT_TX_COMPLETIONS, GEMSLOT_TX_SUBMIT, GEMSLOT_WAIT,
    KernelStreamConfig, KernelStreamStatus, SlotBatch, SlotPoolInfo, SteerRequest, WaitArgs,
};
use super::{Pending, Ring, SLOT_BATCH_MAX, SlotEntry, Slots, WaitRequest};

/// Longest wait the ioctl's millisecond field can express.
const WAIT_TIMEOUT_MAX_MS: u128 = u32::MAX as u128;
/// Slot indices a rejected return logs.
const REJECTED_SLOTS_LOGGED: usize = 8;

pub struct SlotDevice {
    file: File,
    info: SlotPoolInfo,
    pool: *mut u8,
    pool_len: usize,
    kernel_pump: bool,
}

// The pool is shared uncached memory reached only through the copy helpers
// below, each of which bounds-checks before touching it.
unsafe impl Send for SlotDevice {}
unsafe impl Sync for SlotDevice {}

impl SlotDevice {
    /// Opens the device, reads the pool geometry and maps the pool.
    pub fn open(path: &str) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|e| io::Error::new(e.kind(), format!("open {path}: {e}")))?;
        let mut info = SlotPoolInfo::default();
        // SAFETY: the request code is _IOR with this struct's size and the
        // pointer is to a live, correctly sized local.
        unsafe {
            ioctl(
                &file,
                GEMSLOT_GET_INFO,
                &mut info as *mut _ as *mut libc::c_void,
            )
        }
        .map_err(|e| io::Error::new(e.kind(), format!("{path}: GEMSLOT_GET_INFO: {e}")))?;
        if info.pool_size == 0 || info.slot_size == 0 || info.slot_count == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "{path}: empty slot pool (size {}, slot_size {}, slot_count {})",
                    info.pool_size, info.slot_size, info.slot_count
                ),
            ));
        }
        let mut caps = 0u32;
        // SAFETY: the request code is _IOR with a u32 and caps is a live local.
        let kernel_pump = match unsafe {
            ioctl(
                &file,
                GEMSLOT_GET_CAPS,
                &mut caps as *mut _ as *mut libc::c_void,
            )
        } {
            Ok(_) => caps & GEMSLOT_CAP_KERNEL_PUMP != 0,
            Err(error) if error.raw_os_error() == Some(libc::ENOTTY) => false,
            Err(error) => return Err(error),
        };
        let pool_len = info.pool_size as usize;
        // SAFETY: the fd is open for read and write and outlives the call. A
        // failed mapping is reported as MAP_FAILED and handled below.
        let pool = unsafe {
            libc::mmap(
                ptr::null_mut(),
                pool_len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                file.as_raw_fd(),
                0,
            )
        };
        if pool == libc::MAP_FAILED {
            let e = io::Error::last_os_error();
            return Err(io::Error::new(
                e.kind(),
                format!("mmap {path} ({pool_len} bytes): {e}"),
            ));
        }
        Ok(Self {
            file,
            info,
            pool: pool as *mut u8,
            pool_len,
            kernel_pump,
        })
    }

    fn batch_call(&self, request: libc::c_ulong, batch: &mut SlotBatch) -> io::Result<usize> {
        // SAFETY: the request codes are built from this struct's size and the
        // pointer is to a live, correctly sized local owned by the caller.
        let accepted = unsafe { ioctl(&self.file, request, batch as *mut _ as *mut libc::c_void) }?;
        Ok(accepted as usize)
    }

    /// Byte range of a slot inside the mapped pool, or an error when the index
    /// or the span falls outside it.
    fn span(&self, ring: Ring, index: u16, offset: usize, len: usize) -> io::Result<usize> {
        let base = self.info.slot_offset(ring, index).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("slot {index} is outside the {ring:?} ring"),
            )
        })?;
        let end = base + offset + len;
        if offset + len > self.info.slot_size as usize || end > self.pool_len {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!(
                    "slot {index} span {offset}..{} is outside the slot",
                    offset + len
                ),
            ));
        }
        Ok(base + offset)
    }
}

fn to_entries(batch: &SlotBatch, out: &mut Vec<SlotEntry>) {
    let count = (batch.count as usize).min(SLOT_BATCH_MAX);
    out.extend(batch.entries[..count].iter().map(|e| SlotEntry {
        slot: e.slot,
        len: e.len,
    }));
}

fn fill_batch(entries: &[SlotEntry]) -> SlotBatch {
    let count = entries.len().min(SLOT_BATCH_MAX);
    let mut batch = SlotBatch {
        count: count as u32,
        ..SlotBatch::default()
    };
    for (slot, entry) in batch.entries.iter_mut().zip(&entries[..count]) {
        *slot = BatchEntry {
            slot: entry.slot,
            len: entry.len,
        };
    }
    batch
}

/// # Safety
/// `arg` must point at a live object of the size encoded in `request`.
unsafe fn ioctl(file: &File, request: libc::c_ulong, arg: *mut libc::c_void) -> io::Result<i32> {
    // SAFETY: the caller guarantees the argument matches the request code, and
    // the fd is owned by the caller for the duration of the call.
    let rc = unsafe { libc::ioctl(file.as_raw_fd(), request as _, arg) };
    if rc < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(rc)
}

impl Slots for SlotDevice {
    fn info(&self) -> SlotPoolInfo {
        self.info
    }

    fn kernel_pump_available(&self) -> Option<bool> {
        Some(self.kernel_pump)
    }

    fn stream_start(&self, config: KernelStreamConfig) -> io::Result<()> {
        let mut config = config;
        // SAFETY: the request code is _IOW with this struct's size.
        unsafe {
            ioctl(
                &self.file,
                GEMSLOT_STREAM_START,
                &mut config as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(())
    }

    fn stream_stop(&self, direction: u32) -> io::Result<()> {
        let mut direction = direction;
        // SAFETY: the request code is _IOW with a u32.
        unsafe {
            ioctl(
                &self.file,
                GEMSLOT_STREAM_STOP,
                &mut direction as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(())
    }

    fn stream_status(&self, direction: u32) -> io::Result<KernelStreamStatus> {
        let mut status = KernelStreamStatus {
            direction,
            ..KernelStreamStatus::default()
        };
        // SAFETY: the request code is _IOWR with this struct's size.
        unsafe {
            ioctl(
                &self.file,
                GEMSLOT_STREAM_STATUS,
                &mut status as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(status)
    }

    fn tx_submit(&self, entries: &[SlotEntry]) -> io::Result<usize> {
        if entries.is_empty() {
            return Ok(0);
        }
        let mut batch = fill_batch(entries);
        let result = self.batch_call(GEMSLOT_TX_SUBMIT, &mut batch);
        if let Err(error) = &result {
            log::warn!(
                "TX_SUBMIT rejected: {error} count={} first_slot={} first_len={} slot_count={}",
                batch.count,
                entries.first().map(|e| e.slot).unwrap_or(0),
                entries.first().map(|e| e.len).unwrap_or(0),
                self.info.slot_count,
            );
        }
        result
    }

    fn tx_completions(&self, out: &mut Vec<SlotEntry>) -> io::Result<()> {
        let mut batch = SlotBatch::default();
        self.batch_call(GEMSLOT_TX_COMPLETIONS, &mut batch)?;
        to_entries(&batch, out);
        Ok(())
    }

    fn rx_steer(&self, udp_port: u16, enable: bool) -> io::Result<()> {
        let mut request = SteerRequest {
            udp_port,
            enable: u16::from(enable),
        };
        // SAFETY: the request code is _IOW with this struct's size and the
        // pointer is to a live, correctly sized local.
        unsafe {
            ioctl(
                &self.file,
                GEMSLOT_RX_STEER,
                &mut request as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(())
    }

    fn rx_poll(&self, out: &mut Vec<SlotEntry>) -> io::Result<()> {
        let mut batch = SlotBatch::default();
        self.batch_call(GEMSLOT_RX_POLL, &mut batch)?;
        to_entries(&batch, out);
        Ok(())
    }

    fn rx_return(&self, entries: &[SlotEntry]) -> io::Result<()> {
        if entries.is_empty() {
            return Ok(());
        }
        let mut batch = fill_batch(entries);
        if let Err(error) = self.batch_call(GEMSLOT_RX_RETURN, &mut batch) {
            let slots: Vec<u16> = entries
                .iter()
                .take(REJECTED_SLOTS_LOGGED)
                .map(|e| e.slot)
                .collect();
            log::warn!(
                "RX_RETURN rejected: {error} count={} slot_count={} first_slots={slots:?}",
                batch.count,
                self.info.slot_count,
            );
            return Err(error);
        }
        Ok(())
    }

    fn wait(&self, request: WaitRequest) -> io::Result<Pending> {
        let mut args = WaitArgs {
            events: request.events,
            timeout_ms: request.timeout.as_millis().min(WAIT_TIMEOUT_MAX_MS) as u32,
            tx_threshold: request.tx_threshold,
            rx_threshold: request.rx_threshold,
            tx_pending: 0,
            rx_pending: 0,
        };
        // SAFETY: the request code is _IOWR with this struct's size and the
        // pointer is to a live, correctly sized local.
        unsafe {
            ioctl(
                &self.file,
                GEMSLOT_WAIT,
                &mut args as *mut _ as *mut libc::c_void,
            )
        }?;
        Ok(Pending {
            tx_completions: args.tx_pending,
            rx_frames: args.rx_pending,
        })
    }

    fn read_slot(&self, ring: Ring, index: u16, offset: usize, out: &mut [u8]) -> io::Result<()> {
        let at = self.span(ring, index, offset, out.len())?;
        // SAFETY: span() bounds-checks the range inside the mapping, and the
        // pool is uncached device memory so a plain copy is the right access.
        unsafe { ptr::copy_nonoverlapping(self.pool.add(at), out.as_mut_ptr(), out.len()) };
        Ok(())
    }

    fn write_slot(&self, ring: Ring, index: u16, offset: usize, bytes: &[u8]) -> io::Result<()> {
        let at = self.span(ring, index, offset, bytes.len())?;
        // SAFETY: span() bounds-checks the range inside the mapping, and the
        // pool is uncached device memory so a plain copy is the right access.
        unsafe { ptr::copy_nonoverlapping(bytes.as_ptr(), self.pool.add(at), bytes.len()) };
        Ok(())
    }
}

impl Drop for SlotDevice {
    fn drop(&mut self) {
        // SAFETY: the pointer and length are the ones returned by mmap in open().
        unsafe {
            libc::munmap(self.pool as *mut libc::c_void, self.pool_len);
        }
    }
}
