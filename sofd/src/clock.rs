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
    /// Realtime rate vs. the raw clock (freq + tick); phase slews add up to `MAX_PHASE_SLEW_PPM`.
    pub slew_ppm: f64,
    pub est_error_ns: i64,
    pub max_error_ns: i64,
}

/// Fastest phase slew of the kernel PLL and adjtime().
pub const MAX_PHASE_SLEW_PPM: f64 = 500.0;

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
        // tick is µs per 10 ms tick, nominally 10000.
        slew_ppm: tx.freq as f64 / 65536.0 + (tx.tick as f64 - 10_000.0) * 100.0,
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

/// Fast timestamps on CLOCK_MONOTONIC_RAW's scale, for bracketing register reads.
/// On aarch64 the ARM generic timer, read with one instruction (musl's clock_gettime
/// takes ~300 ns on a Raspberry Pi 4, ~20 ns here), which CLOCK_MONOTONIC_RAW counts
/// without scaling it, so raw = offset + count * ns_per_tick. Elsewhere the raw clock.
pub struct Ticks {
    ns_per_tick: f64,
    offset: Option<f64>,
    /// Narrowest pairing seen, in ns.
    #[cfg_attr(not(target_arch = "aarch64"), allow(dead_code))]
    best_width: Ns,
}

#[cfg(target_arch = "aarch64")]
mod counter {
    /// Earlier loads have completed (a device register read too), and later
    /// instructions start after it.
    #[inline(always)]
    pub fn read() -> u64 {
        let v: u64;
        // SAFETY: CNTVCT_EL0 is readable at EL0 on Linux (the vDSO relies on it).
        unsafe { core::arch::asm!("dsb ld", "isb", "mrs {v}, cntvct_el0", "isb", v = out(reg) v, options(nostack)) };
        v
    }
    pub fn hz() -> u64 {
        let v: u64;
        // SAFETY: as above.
        unsafe { core::arch::asm!("mrs {v}, cntfrq_el0", v = out(reg) v, options(nomem, nostack)) };
        v
    }
}

impl Default for Ticks {
    fn default() -> Self {
        Self::new()
    }
}

impl Ticks {
    #[cfg(target_arch = "aarch64")]
    pub fn new() -> Self {
        let mut t = Self { ns_per_tick: 1e9 / counter::hz() as f64, offset: None, best_width: Ns::MAX };
        t.pair(20);
        t
    }

    #[cfg(not(target_arch = "aarch64"))]
    pub fn new() -> Self {
        Self { ns_per_tick: 1.0, offset: Some(0.0), best_width: 0 }
    }

    #[cfg(target_arch = "aarch64")]
    #[inline(always)]
    pub fn read(&self) -> u64 {
        counter::read()
    }

    #[cfg(not(target_arch = "aarch64"))]
    #[inline(always)]
    pub fn read(&self) -> u64 {
        raw() as u64
    }

    /// Refreshes the offset; call before each use. Off aarch64 there is nothing to do.
    pub fn calibrate(&mut self) {
        #[cfg(target_arch = "aarch64")]
        self.pair(5);
    }

    /// The tightest of `n` counter reads between two raw clock reads, smoothed in.
    #[cfg(target_arch = "aarch64")]
    fn pair(&mut self, n: usize) {
        let mut best = (Ns::MAX, 0.0);
        for _ in 0..n {
            let a = raw();
            let c = counter::read();
            let b = raw();
            if b - a < best.0 {
                best = (b - a, (a + (b - a) / 2) as f64 - c as f64 * self.ns_per_tick);
            }
        }
        self.best_width = self.best_width.min(best.0);
        self.offset = Some(match self.offset {
            // A pairing much wider than the best was interrupted.
            Some(o) if best.0 > 3 * self.best_width => o,
            Some(o) if (o - best.1).abs() < 5_000.0 => o + (best.1 - o) * 0.02,
            _ => best.1,
        });
    }

    pub fn raw(&self, t: u64) -> Ns {
        (self.offset.unwrap_or(0.0) + t as f64 * self.ns_per_tick).round() as Ns
    }

    pub fn of_raw(&self, raw: Ns) -> u64 {
        ((raw as f64 - self.offset.unwrap_or(0.0)) / self.ns_per_tick).max(0.0) as u64
    }

    /// A duration in ticks, in ns.
    pub fn ns(&self, ticks: u64) -> Ns {
        (ticks as f64 * self.ns_per_tick).round() as Ns
    }
}
