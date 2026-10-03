//! Output to chrony's `refclock SOCK <path>`, one sample per controller sample.

use crate::clock::Ns;
use log::{info, warn};
use std::collections::BTreeMap;
use std::os::unix::net::UnixDatagram;
use std::path::{Path, PathBuf};

const SOCK_MAGIC: i32 = 0x534f_434b;
/// A silent board loses its socket to another after this.
const OWNER_TIMEOUT_NS: Ns = 10_000_000_000;

/// chrony's `struct sock_sample` (refclock_sock.c).
#[repr(C)]
struct SockSample {
    tv: libc::timeval,
    /// True minus system time at `tv`, seconds.
    offset: f64,
    pulse: libc::c_int,
    leap: libc::c_int,
    _pad: libc::c_int,
    magic: libc::c_int,
}

/// CLOCK_REALTIME and GNSS time of the same instant.
#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub t_realtime_ns: Ns,
    pub t_gnss_ns: Ns,
}

fn encode(p: Point) -> Vec<u8> {
    // timeval holds whole microseconds; the remainder goes into the offset.
    let us = p.t_realtime_ns.div_euclid(1000);
    let s = SockSample {
        tv: libc::timeval { tv_sec: us.div_euclid(1_000_000) as _, tv_usec: us.rem_euclid(1_000_000) as _ },
        offset: (p.t_gnss_ns - us * 1000) as f64 * 1e-9,
        pulse: 0,
        leap: 0,
        _pad: 0,
        magic: SOCK_MAGIC,
    };
    // SAFETY: SockSample is repr(C) plain data; the slice covers exactly it.
    unsafe { std::slice::from_raw_parts((&s as *const SockSample).cast::<u8>(), std::mem::size_of::<SockSample>()) }
        .to_vec()
}

struct Owner {
    serial: String,
    last_ns: Ns,
    /// Last send error, so it is logged once.
    error: Option<String>,
}

pub struct Chrony {
    sock: Option<UnixDatagram>,
    owners: BTreeMap<PathBuf, Owner>,
}

pub enum Sent {
    Ok,
    /// Another board owns the path.
    Busy(String),
}

impl Chrony {
    pub fn new() -> Self {
        // Non-blocking so a stalled chrony can't hold up the daemon.
        let sock = UnixDatagram::unbound()
            .and_then(|s| s.set_nonblocking(true).map(|()| s))
            .map_err(|e| warn!("chrony output: {e}"))
            .ok();
        Self { sock, owners: BTreeMap::new() }
    }

    pub fn send(&mut self, path: &Path, serial: &str, p: Point, now: Ns) -> Sent {
        let o = self.owners.entry(path.to_path_buf()).or_insert_with(|| Owner { serial: serial.into(), last_ns: now, error: None });
        if o.serial != serial {
            if now - o.last_ns < OWNER_TIMEOUT_NS {
                return Sent::Busy(o.serial.clone());
            }
            info!("chrony {}: now fed by {serial} (was {})", path.display(), o.serial);
            *o = Owner { serial: serial.into(), last_ns: now, error: None };
        }
        o.last_ns = now;
        let Some(sock) = &self.sock else { return Sent::Ok };
        match sock.send_to(&encode(p), path) {
            Ok(_) => {
                if o.error.take().is_some() {
                    info!("chrony {}: sending again", path.display());
                }
            }
            Err(e) => {
                let e = e.to_string();
                if o.error.as_ref() != Some(&e) {
                    warn!("chrony {}: {e} (is chrony running with `refclock SOCK {}`?)", path.display(), path.display());
                    o.error = Some(e);
                }
            }
        }
        Sent::Ok
    }

    /// Frees the board's sockets at once.
    pub fn release(&mut self, serial: &str) {
        self.owners.retain(|_, o| o.serial != serial);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_and_values() {
        let b = encode(Point { t_realtime_ns: 1_790_000_000_123_456_789, t_gnss_ns: 1_790_000_000_123_457_289 });
        assert_eq!(b.len(), std::mem::size_of::<SockSample>());
        if cfg!(all(target_os = "linux", target_pointer_width = "64")) {
            assert_eq!(b.len(), 40);
            assert_eq!(i64::from_ne_bytes(b[0..8].try_into().unwrap()), 1_790_000_000);
            assert_eq!(i64::from_ne_bytes(b[8..16].try_into().unwrap()), 123_456);
            let off = f64::from_ne_bytes(b[16..24].try_into().unwrap());
            assert!((off - 1289e-9).abs() < 1e-15, "{off}");
            assert_eq!(i32::from_ne_bytes(b[36..40].try_into().unwrap()), SOCK_MAGIC);
        }
    }

    #[test]
    fn one_board_per_socket() {
        let dir = std::env::temp_dir().join(format!("sofd-chrony-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("c.sock");
        let _ = std::fs::remove_file(&path);
        let server = UnixDatagram::bind(&path).unwrap();
        server.set_nonblocking(true).unwrap();
        let mut c = Chrony::new();
        let p = Point { t_realtime_ns: 0, t_gnss_ns: 0 };
        assert!(matches!(c.send(&path, "A", p, 0), Sent::Ok));
        assert!(matches!(c.send(&path, "B", p, 1), Sent::Busy(o) if o == "A"));
        assert!(matches!(c.send(&path, "B", p, OWNER_TIMEOUT_NS + 1), Sent::Ok));
        c.release("B");
        assert!(matches!(c.send(&path, "A", p, OWNER_TIMEOUT_NS + 2), Sent::Ok));
        let mut buf = [0u8; 64];
        let mut n = 0;
        while server.recv(&mut buf).is_ok() {
            n += 1;
        }
        assert_eq!(n, 3);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
