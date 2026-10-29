//! Linux PPS API (`/dev/ppsN`), polled with a zero timeout like chrony does.

use crate::clock::Ns;
use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};

const PPS_CAPTUREASSERT: i32 = 0x01;
const PPS_API_VERS_1: i32 = 1;

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct PpsKtime {
    sec: i64,
    nsec: i32,
    flags: u32,
}

#[repr(C)]
#[derive(Default)]
struct PpsKinfo {
    assert_sequence: u32,
    clear_sequence: u32,
    assert_tu: PpsKtime,
    clear_tu: PpsKtime,
    current_mode: i32,
}

#[repr(C)]
#[derive(Default)]
struct PpsFdata {
    info: PpsKinfo,
    timeout: PpsKtime,
}

#[repr(C)]
#[derive(Default)]
struct PpsKparams {
    api_version: i32,
    mode: i32,
    assert_off_tu: PpsKtime,
    clear_off_tu: PpsKtime,
}

/// `_IOWR('p', nr, <pointer>)`: the kernel declares these with pointer types.
const fn ioc(dir: u64, nr: u64) -> u64 {
    (dir << 30) | ((std::mem::size_of::<usize>() as u64) << 16) | ((b'p' as u64) << 8) | nr
}
const PPS_GETPARAMS: u64 = ioc(2, 0xa1);
const PPS_SETPARAMS: u64 = ioc(1, 0xa2);
const PPS_FETCH: u64 = ioc(3, 0xa4);

pub struct Pps {
    fd: OwnedFd,
    last_seq: Option<u32>,
}

impl Pps {
    pub fn open(path: &str) -> io::Result<Self> {
        let c = CString::new(path)?;
        // SAFETY: valid C string; the fd is owned right away.
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: fd was just opened and is not owned elsewhere.
        let mut pps = Self { fd: unsafe { OwnedFd::from_raw_fd(fd) }, last_seq: None };
        let mut p = PpsKparams::default();
        pps.ioctl(PPS_GETPARAMS, &mut p)?;
        if p.mode & PPS_CAPTUREASSERT == 0 {
            p.api_version = PPS_API_VERS_1;
            p.mode |= PPS_CAPTUREASSERT;
            pps.ioctl(PPS_SETPARAMS, &mut p)?;
        }
        // An edge from before we opened is no use.
        pps.fetch()?;
        Ok(pps)
    }

    fn ioctl<T>(&mut self, req: u64, arg: &mut T) -> io::Result<()> {
        // SAFETY: arg is the struct the request expects.
        if unsafe { libc::ioctl(self.fd.as_raw_fd(), req as _, arg as *mut T) } < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// CLOCK_REALTIME of a new assert edge since the last call.
    pub fn fetch(&mut self) -> io::Result<Option<Ns>> {
        let mut d = PpsFdata::default();
        self.ioctl(PPS_FETCH, &mut d)?;
        let (seq, tu) = (d.info.assert_sequence, d.info.assert_tu);
        let new = self.last_seq.is_some_and(|s| s != seq);
        self.last_seq = Some(seq);
        Ok(new.then_some(tu.sec * 1_000_000_000 + tu.nsec as Ns))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kernel_layout() {
        if cfg!(target_pointer_width = "64") {
            assert_eq!((PPS_FETCH, PPS_GETPARAMS, PPS_SETPARAMS), (0xc008_70a4, 0x8008_70a1, 0x4008_70a2));
            assert_eq!(std::mem::size_of::<PpsFdata>(), 64);
            assert_eq!(std::mem::size_of::<PpsKparams>(), 40);
        }
    }
}
