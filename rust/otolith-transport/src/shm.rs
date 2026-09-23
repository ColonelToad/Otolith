//! SHM-mapping shim — the only `unsafe` in the steady-state path.
//!
//! Every `unsafe` block below cites its invariant:
//! * **I1 (size):** the segment is `fstat`-checked to exactly
//!   `CAP * size_of::<Slot>()` before mapping — no SIGBUS on access.
//! * **I2 (unique owner):** one mapping per role per process, owned by a
//!   single `Mapping`; access is single-threaded. No second reference to
//!   the bytes exists in-process, so forming `&mut` from the raw pointer
//!   cannot alias anything live.
//! * **I3 (cross-process):** the peer holds its own mapping of the same
//!   segment. `seq` is an atomic (alias-safe by construction); payload
//!   bytes are touched only through `UnsafeCell` raw copies gated by the
//!   seq handshake in `ring.rs` (claimed ⇒ producer-only, published ⇒
//!   consumer-read-then-free). Miri cannot execute two processes, so I3
//!   is audit-only; everything I3 rests on is Miri-checked single-process.
//!
//! Tests touching this module are `#[cfg_attr(miri, ignore)]`d: Miri does
//! not implement `shm_open`/`mmap`.

use std::ffi::CString;
use std::io;
use std::os::fd::FromRawFd;
use std::ptr::NonNull;
use std::time::Duration;

use super::ring::{Slot, CAP, SEG_BYTES};

/// Owned SHM mapping. Unmaps on drop; never unlinks (the orchestrator owns
/// cleanup, same as the C++ bench).
pub struct Mapping {
    ptr: NonNull<Slot>,
}

impl Mapping {
    fn map(fd: i32) -> io::Result<Self> {
        // SAFETY I1: caller fstat-verified `fd` to exactly SEG_BYTES.
        let p = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                SEG_BYTES,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            )
        };
        if p == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }
        Ok(Self {
            // SAFETY I1: mapping is SEG_BYTES of valid memory, Slot-aligned
            // (mmap returns page-aligned; Slot align is 8).
            ptr: unsafe { NonNull::new_unchecked(p as *mut Slot) },
        })
    }

    /// Sub side: unlink stale, create, size, map, init sequences.
    pub fn create(name: &str) -> io::Result<Self> {
        let cname = CString::new(name)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "shm name has NUL"))?;
        // SAFETY: ret stripped of O_CREAT races by unlink-first; the
        // orchestrator guarantees no live peer (same as C++ bench).
        unsafe { libc::shm_unlink(cname.as_ptr()) };
        // SAFETY: cname is a valid NUL-terminated path, no interior NUL.
        let fd = unsafe { libc::shm_open(cname.as_ptr(), libc::O_CREAT | libc::O_RDWR, 0o666) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: fd is ours from shm_open above; from_raw_fd takes ownership.
        let file = unsafe { std::fs::File::from_raw_fd(fd) };
        file.set_len(SEG_BYTES as u64)?;
        let m = Self::map(fd)?;
        // Initialise every seq before any peer can attach (I3 setup).
        // SAFETY I2: sole owner, single-threaded; slice spans exactly the
        // fstat-verified mapping.
        let slots = unsafe { std::slice::from_raw_parts_mut(m.ptr.as_ptr(), CAP) };
        for (i, s) in slots.iter().enumerate() {
            s.seq.store(i as u64, std::sync::atomic::Ordering::Relaxed);
        }
        Ok(m)
    }

    /// Pub side: wait for a fully-sized segment, then map.
    pub fn open_existing(name: &str) -> io::Result<Self> {
        let cname = CString::new(name)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "shm name has NUL"))?;
        for _ in 0..1000 {
            // ~10 s
            // SAFETY: valid path, no O_CREAT side effects.
            let fd = unsafe { libc::shm_open(cname.as_ptr(), libc::O_RDWR, 0o666) };
            if fd >= 0 {
                let mut st: libc::stat = unsafe { std::mem::zeroed() };
                // SAFETY: st is a valid zeroed stat struct.
                let ok =
                    unsafe { libc::fstat(fd, &mut st) } == 0 && st.st_size as usize == SEG_BYTES;
                if ok {
                    let m = Self::map(fd)?;
                    // SAFETY: fd ours; close after mapping (mapping holds it).
                    unsafe { libc::close(fd) };
                    return Ok(m);
                }
                // SAFETY: fd ours.
                unsafe { libc::close(fd) };
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "waiting for shm segment",
        ))
    }

    /// Whole-segment accessor. See I2.
    pub fn slots(&mut self) -> &mut [Slot] {
        // SAFETY I2: &mut self proves unique in-process access.
        unsafe { std::slice::from_raw_parts_mut(self.ptr.as_ptr(), CAP) }
    }
}

impl Drop for Mapping {
    fn drop(&mut self) {
        // SAFETY: ptr/len are exactly what mmap returned (I1).
        unsafe { libc::munmap(self.ptr.as_ptr() as *mut libc::c_void, SEG_BYTES) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg_attr(miri, ignore)] // Miri: no shm_open/mmap
    fn create_open_roundtrip() {
        Mapping::create("/otolith_test_e").unwrap();
        let _m = Mapping::open_existing("/otolith_test_e").unwrap();
        unsafe { libc::shm_unlink(c"/otolith_test_e".as_ptr()) };
    }
}
