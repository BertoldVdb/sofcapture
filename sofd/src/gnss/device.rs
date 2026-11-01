//! The tty of a sofgnss board and the thread that runs its session.

use super::gpio::Poller;
use super::pps::Pps;
use super::session::{Config, Output, Port, Session};
use super::timing::MIN_IDLE_MS;
use super::{Feed, Hub};
use crate::clock::{self, Ns};
use crate::daemon::{BoardOut, Msg};
use anyhow::{Context, Result};
use log::info;
use serde_json::json;
use std::collections::VecDeque;
use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;

const POLL_MS: i32 = 20;
static NEXT_ID: AtomicU64 = AtomicU64::new(1);

fn speed(baud: u32) -> io::Result<libc::speed_t> {
    Ok(match baud {
        4800 => libc::B4800,
        9600 => libc::B9600,
        19_200 => libc::B19200,
        38_400 => libc::B38400,
        57_600 => libc::B57600,
        115_200 => libc::B115200,
        230_400 => libc::B230400,
        460_800 => libc::B460800,
        921_600 => libc::B921600,
        _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("unsupported baud {baud}"))),
    })
}

fn check(r: libc::c_int) -> io::Result<()> {
    if r < 0 { Err(io::Error::last_os_error()) } else { Ok(()) }
}

pub struct Tty {
    fd: OwnedFd,
    /// First write or baud error; ends the session.
    err: Option<io::Error>,
}

impl Tty {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let c = CString::new(path)?;
        // SAFETY: valid C string; the fd is owned right away.
        let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC) };
        check(fd)?;
        // SAFETY: fd was just opened and is not owned elsewhere.
        let tty = Self { fd: unsafe { OwnedFd::from_raw_fd(fd) }, err: None };
        let fd = tty.fd.as_raw_fd();
        // SAFETY: termios is plain data; the calls only access it and fd.
        unsafe {
            check(libc::ioctl(fd, libc::TIOCEXCL))?;
            let mut t: libc::termios = std::mem::zeroed();
            check(libc::tcgetattr(fd, &mut t))?;
            libc::cfmakeraw(&mut t);
            t.c_cflag |= libc::CLOCAL | libc::CREAD;
            t.c_cflag &= !libc::CRTSCTS;
            t.c_cc[libc::VMIN] = 0;
            t.c_cc[libc::VTIME] = 0;
            check(libc::cfsetspeed(&mut t, speed(baud)?))?;
            check(libc::tcsetattr(fd, libc::TCSANOW, &t))?;
            // Input only: a new ttyACM echoes early bytes back to the device, and
            // flushing those in-flight writes locks up the Raspberry Pi's dwc_otg.
            check(libc::tcflush(fd, libc::TCIFLUSH))?;
        }
        Ok(tty)
    }

    /// Waits up to `timeout_ms` for data; Err once the device is gone.
    fn read(&mut self, buf: &mut [u8], timeout_ms: i32) -> io::Result<usize> {
        if let Some(e) = self.err.take() {
            return Err(e);
        }
        let mut p = libc::pollfd { fd: self.fd.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        // SAFETY: one valid pollfd.
        if unsafe { libc::poll(&mut p, 1, timeout_ms) } < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted { Ok(0) } else { Err(e) };
        }
        if p.revents & (libc::POLLHUP | libc::POLLERR | libc::POLLNVAL) != 0 {
            return Err(io::Error::new(io::ErrorKind::BrokenPipe, "device disconnected"));
        }
        if p.revents & libc::POLLIN == 0 {
            return Ok(0);
        }
        // SAFETY: buf is valid for buf.len() bytes.
        let n = unsafe { libc::read(self.fd.as_raw_fd(), buf.as_mut_ptr().cast(), buf.len()) };
        if n < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::WouldBlock { Ok(0) } else { Err(e) };
        }
        Ok(n as usize)
    }

    fn try_write(&mut self, mut data: &[u8]) -> io::Result<()> {
        while !data.is_empty() {
            // SAFETY: data is valid for data.len() bytes.
            let n = unsafe { libc::write(self.fd.as_raw_fd(), data.as_ptr().cast(), data.len()) };
            if n >= 0 {
                data = &data[n as usize..];
                continue;
            }
            let e = io::Error::last_os_error();
            if e.kind() != io::ErrorKind::WouldBlock {
                return Err(e);
            }
            let mut p = libc::pollfd { fd: self.fd.as_raw_fd(), events: libc::POLLOUT, revents: 0 };
            // SAFETY: one valid pollfd.
            if unsafe { libc::poll(&mut p, 1, 1000) } == 0 {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "write to device timed out"));
            }
        }
        Ok(())
    }

    fn try_set_baud(&mut self, baud: u32) -> io::Result<()> {
        let fd = self.fd.as_raw_fd();
        // Let bytes queued at the old baud drain; tcdrain could block forever.
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
        loop {
            let mut queued: libc::c_int = 0;
            // SAFETY: TIOCOUTQ writes one int.
            check(unsafe { libc::ioctl(fd, libc::TIOCOUTQ, &mut queued) })?;
            if queued == 0 {
                break;
            }
            if std::time::Instant::now() >= deadline {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "device does not take bytes (baud change)"));
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        // SAFETY: termios is plain data; the calls only access it and fd.
        unsafe {
            let mut t: libc::termios = std::mem::zeroed();
            check(libc::tcgetattr(fd, &mut t))?;
            check(libc::cfsetspeed(&mut t, speed(baud)?))?;
            check(libc::tcsetattr(fd, libc::TCSANOW, &t))
        }
    }
}

impl Port for Tty {
    fn write(&mut self, data: &[u8]) {
        if self.err.is_none() {
            self.err = self.try_write(data).err();
        }
    }

    fn set_baud(&mut self, baud: u32) {
        if self.err.is_none() {
            self.err = self.try_set_baud(baud).err();
        }
    }
}

/// Where a board is plugged in.
#[derive(Debug, Clone, PartialEq)]
pub struct BoardInfo {
    pub serial: String,
    pub tty: String,
    pub controller: String,
    pub port: String,
}

/// A session thread. Dropping it stops the thread and closes the tty.
struct Worker {
    stop: Arc<AtomicBool>,
    tx: Sender<Vec<u8>>,
    thread: Option<JoinHandle<()>>,
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

pub struct Board {
    pub info: BoardInfo,
    /// Unique per session, so late messages from an old one can be ignored.
    pub id: u64,
    /// Its SOF tags. The daemon puts it in the hub while the board is its
    /// controller's GNSS reference.
    pub feed: Arc<Feed>,
    worker: Worker,
}

impl Board {
    /// Written as is; the device only forwards whole frames.
    pub fn send(&self, data: Vec<u8>) {
        let _ = self.worker.tx.send(data);
    }
}

/// Where a receiver's pulses come from.
#[derive(Debug, Clone)]
pub enum PpsSpec {
    Device(String),
    Gpio(u8),
}

impl std::fmt::Display for PpsSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            PpsSpec::Device(p) => write!(f, "{p}"),
            PpsSpec::Gpio(n) => write!(f, "GPIO {n} (polled)"),
        }
    }
}

enum PulseSource {
    Device(Pps),
    Gpio(Poller),
}

impl PulseSource {
    /// A new edge: CLOCK_REALTIME, and CLOCK_MONOTONIC_RAW where it was read directly.
    fn fetch(&mut self) -> std::io::Result<Option<(Ns, Option<Ns>)>> {
        match self {
            PulseSource::Device(p) => Ok(p.fetch()?.map(|t| (t, None))),
            PulseSource::Gpio(g) => Ok(g.fetch()?.map(|e| (e.realtime_ns, Some(e.raw_ns)))),
        }
    }
}

/// A receiver on a serial port, with its pulses on a /dev/pps device or a polled GPIO.
pub struct Receiver {
    pub id: u64,
    worker: Worker,
}

impl Receiver {
    pub fn send(&self, data: Vec<u8>) {
        let _ = self.worker.tx.send(data);
    }
}

/// What a session thread needs besides its session.
struct Io {
    name: String,
    id: u64,
    tty: Tty,
    pps: Option<PulseSource>,
    /// Board sessions: their SOF tags.
    feed: Option<Arc<Feed>>,
}

fn start(io: Io, cfg: Config, out: &Sender<Msg>, done: impl FnOnce() + Send + 'static) -> Result<Worker> {
    let stop = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::channel();
    let (s, o) = (stop.clone(), out.clone());
    let thread = std::thread::Builder::new().name(format!("gnss-{}", io.name)).spawn(move || {
        let (name, id) = (io.name.clone(), io.id);
        // Clean up even on panic, or the board is never reopened.
        let reason = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(io, cfg, &rx, &o, &s)))
            .unwrap_or_else(|_| Some("board session panicked".into()));
        done();
        if let Some(r) = reason {
            let _ = o.send(Msg::Board { serial: name, id, out: BoardOut::Ended(r) });
        }
    });
    Ok(Worker { stop, tx, thread: Some(thread.context("starting board thread")?) })
}

/// SOF tags go to `Board::feed`, everything else to `out`. The session takes
/// its feed out of `hub` when it ends.
pub fn spawn(info: BoardInfo, cfg: Config, hub: &Hub, out: &Sender<Msg>) -> Result<Board> {
    let tty = Tty::open(&info.tty, cfg.initial_baud).with_context(|| format!("opening {}", info.tty))?;
    let feed = Arc::new(Feed::new(info.serial.clone()));
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let (h, c, fd) = (hub.clone(), info.controller.clone(), feed.clone());
    let io = Io { name: info.serial.clone(), id, tty, pps: None, feed: Some(feed.clone()) };
    let worker = start(io, cfg, out, move || {
        let mut hub = h.lock().unwrap();
        if hub.get(&c).is_some_and(|f| Arc::ptr_eq(f, &fd)) {
            hub.remove(&c);
        }
    })?;
    info!("{}: board on {} ({}), controller {}", info.serial, info.tty, info.port, info.controller);
    Ok(Board { info, id, feed, worker })
}

pub fn spawn_receiver(name: &str, tty: &str, pps: &PpsSpec, cfg: Config, out: &Sender<Msg>) -> Result<Receiver> {
    let t = Tty::open(tty, cfg.initial_baud).with_context(|| format!("opening {tty}"))?;
    let p = match pps {
        PpsSpec::Device(path) => PulseSource::Device(Pps::open(path).with_context(|| format!("opening {path}"))?),
        PpsSpec::Gpio(n) => PulseSource::Gpio(Poller::start(*n).with_context(|| format!("polling GPIO {n}"))?),
    };
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let io = Io { name: name.into(), id, tty: t, pps: Some(p), feed: None };
    let worker = start(io, Config { plain: true, ..cfg }, out, || {})?;
    info!("{name}: receiver on {tty}, pulses from {pps}");
    Ok(Receiver { id, worker })
}

/// Bytes read this soon after a pulse may have arrived before it.
const PULSE_GUARD_NS: Ns = MIN_IDLE_MS as Ns * 1_000_000;

/// Orders kernel PPS edges against the bytes read from the receiver.
struct Pulses {
    /// CLOCK_REALTIME of the last read that returned bytes.
    last_rx: Ns,
    /// Recent pulses: raw tick and CLOCK_REALTIME.
    recent: VecDeque<(u64, Ns)>,
}

impl Pulses {
    /// Raw tick and line idle time of a pulse at realtime `t` (raw `raw` if read
    /// directly), given whether the bytes of this read are already counted.
    fn pulse(&mut self, t: Ns, raw: Option<Ns>, read_at: Ns, bytes_before: bool, n: usize) -> (u64, u32) {
        let idle = if !bytes_before && n > 0 && read_at - t < PULSE_GUARD_NS { 0 } else { t - self.last_rx };
        let tick = raw.unwrap_or_else(|| t - clock::offset_from_raw(clock::realtime)) as u64;
        if self.recent.len() == 4 {
            self.recent.pop_front();
        }
        self.recent.push_back((tick, t));
        (tick, (idle.max(0) / 1_000_000).min(u32::MAX as Ns) as u32)
    }

    fn realtime_of(&self, tick: u64) -> Option<Ns> {
        self.recent.iter().find(|p| p.0 == tick).map(|p| p.1)
    }
}

/// Runs the session until stopped (None) or the device fails (Some(reason)).
fn run(mut io: Io, cfg: Config, tx: &mpsc::Receiver<Vec<u8>>, out: &Sender<Msg>, stop: &AtomicBool) -> Option<String> {
    let now = clock::raw();
    let nonce = format!("sofd{:08x}", (now as u64 ^ std::process::id() as u64) as u32);
    let mut session = Session::new(cfg, nonce, now);
    let mut buf = [0u8; 4096];
    let mut o = Output::default();
    let mut pulses = Pulses { last_rx: clock::realtime(), recent: VecDeque::new() };
    let send = |m: BoardOut| out.send(Msg::Board { serial: io.name.clone(), id: io.id, out: m }).is_ok();
    while !stop.load(Ordering::Relaxed) {
        let n = match io.tty.read(&mut buf, POLL_MS) {
            Ok(n) => n,
            Err(e) => return Some(e.to_string()),
        };
        let (now, read_at) = (clock::raw(), clock::realtime());
        let edge = match io.pps.as_mut().map(|p| p.fetch()) {
            Some(Err(e)) => return Some(format!("PPS: {e}")),
            Some(Ok(t)) => t,
            None => None,
        };
        // An edge after this read: its bytes came first.
        let bytes_first = edge.is_none_or(|(t, _)| t >= read_at);
        if bytes_first && n > 0 {
            session.on_bytes(&buf[..n], now, &mut io.tty, &mut o);
        }
        if let Some((t, raw)) = edge {
            let (tick, idle) = pulses.pulse(t, raw, read_at, bytes_first, n);
            session.on_pulse(tick, idle, now, &mut o);
        }
        if !bytes_first && n > 0 {
            session.on_bytes(&buf[..n], now, &mut io.tty, &mut o);
        }
        if n > 0 {
            pulses.last_rx = read_at;
        }
        while let Ok(data) = tx.try_recv() {
            if session.accepts_tx() {
                io.tty.write(&data);
            }
        }
        session.on_tick(now, &mut io.tty, &mut o);
        for t in o.tags.drain(..) {
            if let Some(f) = &io.feed {
                f.push(t);
            }
        }
        for mut ev in o.events.drain(..) {
            let m = if ev["event"] == "gnss_status" {
                let lock = if ev["locked"] == true { " locked" } else { "" };
                let msg = ev.get("message").map_or(String::new(), |m| format!(" {m}"));
                info!("{}: {}{lock}{msg}", io.name, ev["state"]);
                BoardOut::Status(ev)
            } else if ev["event"] == "gnss_cal" {
                BoardOut::Cal(ev)
            } else {
                if io.pps.is_some() {
                    let t = ev["tick"].as_u64().and_then(|t| pulses.realtime_of(t));
                    ev["t_realtime_ns"] = json!(t);
                }
                BoardOut::Pps(ev)
            };
            if !send(m) {
                return None;
            }
        }
        for (kind, f) in o.frames.drain(..) {
            if !send(BoardOut::Frame(kind, f)) {
                return None;
            }
        }
        // Close the port before the device restarts (OTA,OK, START, BOOT): transfers
        // cancelled when it drops off the bus can lock up the Raspberry Pi's dwc_otg.
        if session.restarting() {
            return Some("device restarting".into());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pulse_idle_time() {
        let ms = 1_000_000;
        let mut p = Pulses { last_rx: 1000 * ms, recent: VecDeque::new() };
        // Edge after the read: idle since the last bytes.
        assert_eq!(p.pulse(1700 * ms, None, 1690 * ms, true, 0).1, 700);
        // Bytes read just after the edge may have come before it.
        assert_eq!(p.pulse(2700 * ms, None, 2705 * ms, false, 20).1, 0);
        // Read well after the edge: the bytes came later.
        let (tick, idle) = p.pulse(3700 * ms, None, 3750 * ms, false, 20);
        assert_eq!(idle, 2700);
        assert_eq!(p.realtime_of(tick), Some(3700 * ms));
        // A polled edge brings its raw time.
        assert_eq!(p.pulse(4700 * ms, Some(42), 4690 * ms, true, 0).0, 42);
    }
}
