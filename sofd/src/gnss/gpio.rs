//! A PPS on a Raspberry Pi GPIO (BCM283x/BCM2711), timed by polling the level
//! register around each expected edge, as the sampler times the frame counter.
//! The kernel's PPS timestamp comes from its interrupt, 4 to 8.5 us late on a
//! Pi 4. Polling brackets the edge to about +-0.2 us at 0.05 % of a core.

use crate::clock::{self, Ns};
use log::{debug, warn};
use std::ffi::CString;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;

const GPLEV0: usize = 0x34 / 4;
/// The edge is looked for this far either side of its prediction.
const WINDOW_NS: Ns = 300_000;
/// Misses in a row before searching the whole second again.
const MAX_MISSES: u32 = 3;
/// Between reads while searching for the first edge.
const SEARCH_STEP_NS: i64 = 20_000;

/// A rising edge: raw and realtime clock in the middle of the reads around it.
#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub raw_ns: Ns,
    pub realtime_ns: Ns,
}

struct Regs {
    base: *mut u32,
}

// SAFETY: the mapping is read only and lives as long as the struct.
unsafe impl Send for Regs {}

impl Regs {
    fn open() -> io::Result<Self> {
        let path = CString::new("/dev/gpiomem").unwrap();
        // SAFETY: valid C string; the fd is closed after mapping.
        let fd = unsafe { libc::open(path.as_ptr(), libc::O_RDONLY | libc::O_SYNC | libc::O_CLOEXEC) };
        if fd < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: maps the 4 KB GPIO block read only; checked below.
        let p = unsafe { libc::mmap(std::ptr::null_mut(), 4096, libc::PROT_READ, libc::MAP_SHARED, fd, 0) };
        // SAFETY: fd is ours.
        unsafe { libc::close(fd) };
        if p == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }
        Ok(Self { base: p.cast() })
    }

    fn level(&self, pin: u8) -> bool {
        // SAFETY: GPLEV0/1 lie inside the mapped block.
        let v = unsafe { std::ptr::read_volatile(self.base.add(GPLEV0 + pin as usize / 32)) };
        v & (1 << (pin % 32)) != 0
    }
}

impl Drop for Regs {
    fn drop(&mut self) {
        // SAFETY: unmaps what open() mapped.
        unsafe { libc::munmap(self.base.cast(), 4096) };
    }
}

pub struct Poller {
    edges: Receiver<Edge>,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Poller {
    pub fn start(pin: u8) -> io::Result<Self> {
        if pin > 53 {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("GPIO {pin} does not exist")));
        }
        let regs = Regs::open()?;
        let (tx, rx) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let s = stop.clone();
        let thread = std::thread::Builder::new().name(format!("pps-gpio{pin}")).spawn(move || run(&regs, pin, &tx, &s))?;
        Ok(Self { edges: rx, stop, thread: Some(thread) })
    }

    /// The next edge since the last call, if any.
    pub fn fetch(&mut self) -> io::Result<Option<Edge>> {
        match self.edges.try_recv() {
            Ok(e) => Ok(Some(e)),
            Err(mpsc::TryRecvError::Empty) => Ok(None),
            Err(mpsc::TryRecvError::Disconnected) => Err(io::Error::other("GPIO poller ended")),
        }
    }
}

impl Drop for Poller {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Away from CPU 0, which takes the GPIO interrupt (pps-gpio on the same pin
/// stalls a poller there for 5 us right at the edge).
fn pin_to_last_cpu() {
    // SAFETY: cpu_set_t is plain data; the calls only read and write it.
    unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) != 0 {
            return;
        }
        let Some(last) = (0..libc::CPU_SETSIZE as usize).rev().find(|&c| libc::CPU_ISSET(c, &set)) else { return };
        if last == 0 {
            return;
        }
        libc::CPU_ZERO(&mut set);
        libc::CPU_SET(last, &mut set);
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set);
    }
}

fn sleep_until_raw(raw: Ns) {
    let mono = raw + clock::offset_from_raw(clock::mono);
    clock::sleep_until_mono(mono);
}

fn run(regs: &Regs, pin: u8, out: &Sender<Edge>, stop: &AtomicBool) {
    if let Err(e) = crate::sampler::set_fifo(60) {
        warn!("pps gpio{pin}: no SCHED_FIFO ({e}); edges may be lost to preemption");
    }
    pin_to_last_cpu();
    let mut predicted: Option<Ns> = None;
    let mut misses = 0;
    let mut ticks = clock::Ticks::new();
    let mut min_read = u64::MAX;
    while !stop.load(Ordering::Relaxed) {
        let Some(p) = predicted else {
            // Only to 20 us: it predicts the next edge but is not reported.
            predicted = search(regs, pin, stop).map(|e| e.raw_ns + 1_000_000_000);
            continue;
        };
        sleep_until_raw(p - WINDOW_NS);
        ticks.calibrate();
        let edge = spin(regs, pin, p + WINDOW_NS, &ticks, &mut min_read);
        match edge {
            Some(e) => {
                misses = 0;
                predicted = Some(e.raw_ns + 1_000_000_000);
                if out.send(e).is_err() {
                    return;
                }
            }
            None => {
                misses += 1;
                predicted = match predicted {
                    Some(p) if misses < MAX_MISSES => Some(p + 1_000_000_000),
                    _ => None,
                };
            }
        }
    }
}

/// Reads every SEARCH_STEP_NS for up to 1.5 s; the edge only to that step.
fn search(regs: &Regs, pin: u8, stop: &AtomicBool) -> Option<Edge> {
    let end = clock::raw() + 1_500_000_000;
    let mut was_high = true;
    while clock::raw() < end && !stop.load(Ordering::Relaxed) {
        let t = clock::raw();
        let high = regs.level(pin);
        if high && !was_high {
            return Some(Edge { raw_ns: t, realtime_ns: t + clock::offset_from_raw(clock::realtime) });
        }
        was_high = high;
        sleep_until_raw(t + SEARCH_STEP_NS);
    }
    None
}

/// Polls until a low-to-high step or `end`. The edge lies between the counter
/// read before the last low read and the one after the first high read.
fn spin(regs: &Regs, pin: u8, end: Ns, ticks: &clock::Ticks, min_read: &mut u64) -> Option<Edge> {
    let end = ticks.of_raw(end);
    let mut last_low = ticks.read();
    if regs.level(pin) {
        debug!("pps gpio{pin}: high at the window start");
        return None;
    }
    let slack = ticks.of_raw(ticks.raw(0) + 1000);
    loop {
        let c0 = ticks.read();
        let high = regs.level(pin);
        let c1 = ticks.read();
        *min_read = (*min_read).min(c1 - c0);
        if high {
            let width = c1 - last_low;
            // Much longer than two reads: preempted in between, the edge is anywhere in there.
            if width > 4 * *min_read + slack {
                debug!("pps gpio{pin}: edge bracket {} ns, preempted", ticks.ns(width));
                return None;
            }
            let raw_ns = ticks.raw(last_low + width / 2);
            return Some(Edge { raw_ns, realtime_ns: raw_ns + clock::offset_from_raw(clock::realtime) });
        }
        last_low = c0;
        if c1 > end {
            debug!("pps gpio{pin}: no edge in the window");
            return None;
        }
    }
}
