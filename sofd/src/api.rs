//! Newline-delimited JSON protocol with the single peer program.
//!
//! sofd connects to the peer's Unix socket. Each line from the peer is a
//! request `{"cmd": ..., "id": <any, echoed>}`; each reply carries `"ok"`.
//! Unsolicited events (`"event": "sample" | "error" | "recovered" | "hello"`)
//! are interleaved with replies. The peer must send `{"cmd": "ping"}` at
//! least once per ping timeout; otherwise the connection is closed. When the
//! connection ends for any reason every subscription stops and sofd exits.

use crate::backend;
use crate::discovery::{self, BackendKind, DeviceQuery, MmioOverride};
use crate::sampler::{Sampler, SamplerConfig, MAX_RATE_HZ, MIN_RATE_HZ};
use anyhow::{anyhow, bail, Context, Result};
use log::{info, warn};
use serde::Deserialize;
use serde_json::{json, Map, Value};
use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::net::Shutdown;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::sync::mpsc::{self, Sender};
use std::sync::mpsc::RecvTimeoutError;
use std::time::{Duration, Instant};

pub enum Msg {
    /// A request line from the peer.
    Line(String),
    PeerClosed,
    /// A serialized event to forward to the peer.
    Out(String),
}

#[derive(Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum Command {
    Ping,
    List,
    Lookup(DeviceQuery),
    Subscribe { controller: String, rate_hz: Option<f64> },
    Unsubscribe { controller: String },
}

#[derive(Debug, Clone)]
pub struct Options {
    pub overrides: Vec<MmioOverride>,
    pub simulate: bool,
    /// Controllers to sample through the generic kernel fallback regardless of driver.
    pub force_generic: Vec<String>,
    /// Longest stretch the sofclock module may disable interrupts (0 = never).
    pub irq_off_max_ns: u32,
    pub default_rate_hz: f64,
    pub sampler: SamplerConfig,
}

/// Forces the controller out of runtime suspend while it is sampled, so the
/// frame counter keeps running; restores the previous setting on drop.
struct PmGuard {
    path: PathBuf,
    restore: String,
}

impl PmGuard {
    fn hold(ctrl: &discovery::Controller) -> Option<Self> {
        let path = ctrl.path.join("power/control");
        let restore = std::fs::read_to_string(&path).ok()?.trim().to_string();
        if restore == "on" {
            return None;
        }
        match std::fs::write(&path, "on") {
            Ok(()) => Some(Self { path, restore }),
            Err(e) => {
                warn!("{}: cannot disable runtime PM: {e}", ctrl.id);
                None
            }
        }
    }
}

impl Drop for PmGuard {
    fn drop(&mut self) {
        let _ = std::fs::write(&self.path, &self.restore);
    }
}

struct Subscription {
    // Field order matters: the sampler stops before runtime PM is restored.
    sampler: Sampler,
    rate_hz: f64,
    _pm: Option<PmGuard>,
}

pub struct Daemon {
    opts: Options,
    subs: BTreeMap<String, Subscription>,
}

impl Daemon {
    pub fn new(opts: Options) -> Self {
        Self { opts, subs: BTreeMap::new() }
    }

    pub fn stop_all(&mut self) {
        if !self.subs.is_empty() {
            info!("stopping {} sampler(s)", self.subs.len());
        }
        self.subs.clear();
    }

    pub fn list(&self) -> Result<Value> {
        let devices = discovery::devices(self.opts.simulate)?;
        let ctrls: Vec<Value> = discovery::controllers(self.opts.simulate)?
            .into_iter()
            .map(|c| {
                let mut v = serde_json::to_value(&c).unwrap();
                v["subscribed"] = json!(self.subs.contains_key(&c.id));
                v["devices"] = json!(devices.iter().filter(|d| d.controller == c.id).collect::<Vec<_>>());
                v
            })
            .collect();
        Ok(json!({ "controllers": ctrls }))
    }

    pub fn lookup(&self, q: &DeviceQuery) -> Result<Value> {
        let dev = discovery::lookup(q, self.opts.simulate)?;
        let ctrl = discovery::find_controller(&dev.controller, self.opts.simulate)?;
        Ok(json!({
            "controller": ctrl.id,
            "driver": ctrl.driver,
            "backend": ctrl.backend,
            "supported": match ctrl.backend {
                Some(BackendKind::KernelFrameNumber) => crate::kmod::available(),
                other => other.is_some(),
            },
            "buses": ctrl.buses,
            "device": dev,
        }))
    }

    pub fn subscribe(&mut self, id: &str, rate_hz: Option<f64>, out: &Sender<Msg>) -> Result<Value> {
        let rate = rate_hz.unwrap_or(self.opts.default_rate_hz);
        if !(MIN_RATE_HZ..=MAX_RATE_HZ).contains(&rate) {
            bail!("rate_hz must be within {MIN_RATE_HZ}..{MAX_RATE_HZ}");
        }
        if let Some(sub) = self.subs.get_mut(id) {
            sub.sampler.set_rate(rate);
            sub.rate_hz = rate;
        } else {
            let mut ctrl = discovery::find_controller(id, self.opts.simulate)?;
            if self.opts.force_generic.iter().any(|g| g == id) {
                ctrl.backend = Some(BackendKind::KernelFrameNumber);
            }
            let backend = backend::open(&ctrl, &self.opts.overrides, 2, self.opts.irq_off_max_ns)?;
            let pm = PmGuard::hold(&ctrl);
            let sampler = Sampler::spawn(id.to_string(), backend, rate, self.opts.sampler.clone(), out.clone())
                .context("starting sampler thread")?;
            info!("{id}: sampling at {rate} Hz");
            self.subs.insert(id.to_string(), Subscription { sampler, rate_hz: rate, _pm: pm });
        }
        Ok(json!({ "controller": id, "rate_hz": rate }))
    }

    pub fn unsubscribe(&mut self, id: &str) -> Value {
        let was = self.subs.remove(id).is_some();
        if was {
            info!("{id}: stopped");
        }
        json!({ "controller": id, "was_subscribed": was })
    }

    fn handle(&mut self, line: &str, out: &Sender<Msg>) -> Value {
        let req: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => return json!({ "ok": false, "error": format!("invalid JSON: {e}") }),
        };
        let id = req.get("id").cloned();
        let cmd_name = req.get("cmd").cloned();
        let result = serde_json::from_value::<Command>(req)
            .map_err(|e| anyhow!("invalid request: {e}"))
            .and_then(|cmd| match cmd {
                Command::Ping => Ok(json!({})),
                Command::List => self.list(),
                Command::Lookup(q) => self.lookup(&q),
                Command::Subscribe { controller, rate_hz } => self.subscribe(&controller, rate_hz, out),
                Command::Unsubscribe { controller } => Ok(self.unsubscribe(&controller)),
            });
        let mut reply = Map::new();
        if let Some(id) = id {
            reply.insert("id".into(), id);
        }
        if let Some(c) = cmd_name {
            reply.insert("cmd".into(), c);
        }
        match result {
            Ok(Value::Object(fields)) => {
                reply.insert("ok".into(), json!(true));
                reply.extend(fields);
            }
            Ok(other) => {
                reply.insert("ok".into(), json!(true));
                reply.insert("result".into(), other);
            }
            Err(e) => {
                reply.insert("ok".into(), json!(false));
                reply.insert("error".into(), json!(format!("{e:#}")));
            }
        }
        Value::Object(reply)
    }
}

fn send_line(w: &mut UnixStream, line: &str) -> std::io::Result<()> {
    let mut buf = Vec::with_capacity(line.len() + 1);
    buf.extend_from_slice(line.as_bytes());
    buf.push(b'\n');
    w.write_all(&buf)
}

/// Serve the connection until the peer disconnects, stops reading, or misses
/// its ping deadline. Returns Ok only for a clean disconnect by the peer.
pub fn run_session(stream: UnixStream, d: &mut Daemon, ping_timeout: Duration) -> Result<()> {
    // A peer that stops reading must not wedge us with samplers queuing up.
    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
    let (tx, rx) = mpsc::channel();
    let reader = stream.try_clone()?;
    let rtx = tx.clone();
    std::thread::Builder::new().name("peer-reader".into()).spawn(move || {
        for line in BufReader::new(reader).lines() {
            let Ok(l) = line else { break };
            if rtx.send(Msg::Line(l)).is_err() {
                break;
            }
        }
        let _ = rtx.send(Msg::PeerClosed);
    })?;

    let mut w = stream;
    let hello = json!({"event": "hello", "version": env!("CARGO_PKG_VERSION"), "pid": std::process::id()});
    let mut result = send_line(&mut w, &hello.to_string()).map_err(Into::into);
    let mut ping_deadline = Instant::now() + ping_timeout;
    while result.is_ok() {
        let wait = ping_deadline.saturating_duration_since(Instant::now());
        let line = match rx.recv_timeout(wait) {
            Ok(Msg::Line(l)) if l.trim().is_empty() => continue,
            Ok(Msg::Line(l)) => {
                let reply = d.handle(&l, &tx);
                if reply["cmd"] == "ping" && reply["ok"] == true {
                    ping_deadline = Instant::now() + ping_timeout;
                }
                reply.to_string()
            }
            Ok(Msg::Out(s)) => s,
            Ok(Msg::PeerClosed) | Err(RecvTimeoutError::Disconnected) => break,
            Err(RecvTimeoutError::Timeout) => {
                result = Err(anyhow!("no ping from peer within {} s", ping_timeout.as_secs_f64()));
                break;
            }
        };
        result = send_line(&mut w, &line).context("writing to peer");
    }
    d.stop_all();
    let _ = w.shutdown(Shutdown::Both);
    result
}
