//! The tty of a sofgps board and the thread that runs its session.

use super::session::{Config, Output, Port, Session};
use super::{Feed, Hub};
use crate::clock;
use crate::daemon::{BoardOut, Msg};
use anyhow::{bail, Context, Result};
use log::info;
use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
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
            check(libc::tcflush(fd, libc::TCIOFLUSH))?;
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

/// Dropping it stops the thread and closes the tty.
pub struct Board {
    pub info: BoardInfo,
    /// Unique per session, so late messages from an old one can be ignored.
    pub id: u64,
    stop: Arc<AtomicBool>,
    tx: Sender<Vec<u8>>,
    thread: Option<JoinHandle<()>>,
}

impl Board {
    /// Written as is; the device only forwards whole frames.
    pub fn send(&self, data: Vec<u8>) {
        let _ = self.tx.send(data);
    }
}

impl Drop for Board {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// SOF tags go to the controller's feed in `hub`, everything else to `out`.
pub fn spawn(info: BoardInfo, cfg: Config, hub: &Hub, out: &Sender<Msg>) -> Result<Board> {
    let tty = Tty::open(&info.tty, cfg.initial_baud).with_context(|| format!("opening {}", info.tty))?;
    let feed = Arc::new(Feed::new(info.serial.clone()));
    {
        let mut h = hub.lock().unwrap();
        match h.get(&info.controller) {
            Some(f) if f.client.is_some() => info!("{}: board {} replaces GNSS reference {}", info.controller, info.serial, f.serial),
            Some(f) => bail!("controller {} already has board {} attached", info.controller, f.serial),
            None => {}
        }
        h.insert(info.controller.clone(), feed.clone());
    }
    let stop = Arc::new(AtomicBool::new(false));
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let (tx, rx) = mpsc::channel();
    let (s, h, o, i) = (stop.clone(), hub.clone(), out.clone(), info.clone());
    let thread = std::thread::Builder::new().name(format!("gnss-{}", info.serial)).spawn(move || {
        // Clean up even on panic, or the board is never reopened.
        let reason = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(tty, &i, id, cfg, &feed, &rx, &o, &s)))
            .unwrap_or_else(|_| Some("board session panicked".into()));
        let mut hub = h.lock().unwrap();
        if hub.get(&i.controller).is_some_and(|f| Arc::ptr_eq(f, &feed)) {
            hub.remove(&i.controller);
        }
        drop(hub);
        if let Some(r) = reason {
            let _ = o.send(Msg::Board { serial: i.serial.clone(), id, out: BoardOut::Ended(r) });
        }
    });
    let thread = match thread {
        Ok(t) => t,
        Err(e) => {
            hub.lock().unwrap().remove(&info.controller);
            return Err(e).context("starting board thread");
        }
    };
    info!("{}: board on {} ({}), GNSS reference of {}", info.serial, info.tty, info.port, info.controller);
    Ok(Board { info, id, stop, tx, thread: Some(thread) })
}

/// Runs the session until stopped (None) or the device fails (Some(reason)).
#[allow(clippy::too_many_arguments)]
fn run(
    mut tty: Tty,
    info: &BoardInfo,
    id: u64,
    cfg: Config,
    feed: &Feed,
    tx: &Receiver<Vec<u8>>,
    out: &Sender<Msg>,
    stop: &AtomicBool,
) -> Option<String> {
    let now = clock::raw();
    let nonce = format!("sofd{:08x}", (now as u64 ^ std::process::id() as u64) as u32);
    let mut session = Session::new(cfg, nonce, now);
    let mut buf = [0u8; 4096];
    let mut o = Output::default();
    let send = |m: BoardOut| out.send(Msg::Board { serial: info.serial.clone(), id, out: m }).is_ok();
    while !stop.load(Ordering::Relaxed) {
        let n = match tty.read(&mut buf, POLL_MS) {
            Ok(n) => n,
            Err(e) => return Some(e.to_string()),
        };
        let now = clock::raw();
        if n > 0 {
            session.on_bytes(&buf[..n], now, &mut tty, &mut o);
        }
        while let Ok(data) = tx.try_recv() {
            if session.accepts_tx() {
                tty.write(&data);
            }
        }
        session.on_tick(now, &mut tty, &mut o);
        for t in o.tags.drain(..) {
            feed.push(t);
        }
        for ev in o.events.drain(..) {
            let m = if ev["event"] == "gnss_status" {
                info!("{}: {} {}", info.serial, ev["state"], ev.get("message").map_or(String::new(), |m| m.to_string()));
                BoardOut::Status(ev)
            } else {
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
    }
    None
}
