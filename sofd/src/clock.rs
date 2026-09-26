//! Host clocks. Timing uses CLOCK_MONOTONIC_RAW because NTP never slews it.

pub type Ns = i64;

#[inline]
fn gettime(clk: libc::clockid_t) -> Ns {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: valid out pointer; the clock ids used are always supported on Linux.
    unsafe { libc::clock_gettime(clk, &mut ts) };
    ts.tv_sec as Ns * 1_000_000_000 + ts.tv_nsec as Ns
}

#[inline]
pub fn raw() -> Ns {
    gettime(libc::CLOCK_MONOTONIC_RAW)
}

#[inline]
pub fn mono() -> Ns {
    gettime(libc::CLOCK_MONOTONIC)
}

#[inline]
pub fn realtime() -> Ns {
    gettime(libc::CLOCK_REALTIME)
}

/// `other - raw`, from the tightest of a few raw/other/raw reads.
pub fn offset_from_raw(other: fn() -> Ns) -> Ns {
    let mut best = (Ns::MAX, 0);
    for _ in 0..5 {
        let a = raw();
        let o = other();
        let b = raw();
        if b - a < best.0 {
            best = (b - a, o - (a + (b - a) / 2));
        }
    }
    best.1
}

/// CLOCK_REALTIME discipline state from adjtimex.
#[derive(Debug, Clone, serde::Serialize)]
pub struct RealtimeSync {
    /// STA_UNSYNC clear. chrony only clears it with `rtcsync`.
    pub kernel_synced: bool,
    /// Kernel frequency correction.
    pub freq_ppm: f64,
    pub est_error_ns: i64,
    pub max_error_ns: i64,
}

pub fn realtime_sync() -> Option<RealtimeSync> {
    // SAFETY: timex is plain data; modes = 0 makes adjtimex read-only.
    let mut tx: libc::timex = unsafe { std::mem::zeroed() };
    let state = unsafe { libc::adjtimex(&mut tx) };
    if state < 0 {
        return None;
    }
    Some(RealtimeSync {
        kernel_synced: state != libc::TIME_ERROR && tx.status & libc::STA_UNSYNC == 0,
        freq_ppm: tx.freq as f64 / 65536.0,
        est_error_ns: tx.esterror as i64 * 1000,
        max_error_ns: tx.maxerror as i64 * 1000,
    })
}

pub fn sleep_until_mono(t: Ns) {
    let ts = libc::timespec {
        tv_sec: (t / 1_000_000_000) as _,
        tv_nsec: (t % 1_000_000_000) as _,
    };
    loop {
        // SAFETY: valid timespec; remaining-time pointer may be null with TIMER_ABSTIME.
        let r = unsafe {
            libc::clock_nanosleep(libc::CLOCK_MONOTONIC, libc::TIMER_ABSTIME, &ts, std::ptr::null_mut())
        };
        if r != libc::EINTR {
            break;
        }
    }
}
