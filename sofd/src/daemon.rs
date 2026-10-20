//! Main loop: runs sofgnss boards, samples controllers, and publishes to MQTT, chrony and stdout.

use crate::backend;
use crate::chrony::{self, Chrony, Sent};
use crate::clock::{self, Ns};
use crate::config;
use crate::discovery::{self, BackendKind, Controller, DeviceQuery, MmioOverride, UsbDevice};
use crate::gnss::device::{self, Board, BoardInfo};
use crate::gnss::firmware::Image;
use crate::gnss::framer::Kind;
use crate::gnss::external::{self, LOST_AFTER_S};
use crate::gnss::timing::{Source, Tag};
use crate::gnss::{session, Feed, Hub};
use crate::mqtt::{self, Mqtt};
use crate::power;
use crate::sampler::{Sampler, SamplerConfig, MAX_RATE_HZ, MIN_RATE_HZ};
use crate::uevent;
use anyhow::{anyhow, bail, Context, Result};
use log::{info, warn};
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

// 20a0:4288 is shared with other devices, so the strings must match too.
pub const BOARD_VID_PID: (&str, &str) = ("20a0", "4288");
pub const BOARD_MANUFACTURER: &str = "BertoldVdb";
pub const BOARD_PRODUCT: &str = "GNSS/SOF/PPS Bridge";

pub fn is_board(d: &UsbDevice) -> bool {
    (d.vid.as_str(), d.pid.as_str()) == BOARD_VID_PID
        && d.manufacturer.as_deref() == Some(BOARD_MANUFACTURER)
        && d.product.as_deref() == Some(BOARD_PRODUCT)
}
const SCAN_EVERY: Duration = Duration::from_secs(1);
/// Rescan even without uevents, in case one was missed.
const FULL_RESCAN: Duration = Duration::from_secs(60);
const RETRY_MIN: Duration = Duration::from_secs(1);
const RETRY_MAX: Duration = Duration::from_secs(30);
/// Doubles while the same error repeats.
const SAMPLER_RETRY: Duration = Duration::from_secs(10);
const SAMPLER_RETRY_MAX: Duration = Duration::from_secs(300);
/// After a reconnect, a retained request that isn't redelivered within this lapses.
const RECONFIRM: Duration = Duration::from_secs(10);
const RUNNING_STATES: [&str; 3] = ["ubx", "passive", "nmea_fallback"];
/// Wait for a firmware file to stop changing before loading it.
const IMAGE_SETTLE: Duration = Duration::from_secs(2);
/// Failed updates to one image before giving up on a board.
const MAX_UPDATE_TRIES: u32 = 2;
/// Reopen a restarting board after this even if it never left the bus.
const RESTART_HOLD: Duration = Duration::from_secs(5);

pub enum Msg {
    Sample(Box<SampleOut>),
    /// error, recovered, realtime_step
    Controller { controller: String, event: Value },
    /// `id`: the sending session.
    Board { serial: String, id: u64, out: BoardOut },
    Mqtt(mqtt::In),
}

pub struct SampleOut {
    pub controller: String,
    pub event: Value,
    /// Board serial or `mqtt:<client>`.
    pub board: Option<String>,
    /// Set while the GNSS reference is valid.
    pub chrony: Option<chrony::Point>,
}

pub enum BoardOut {
    Status(Value),
    Pps(Value),
    Frame(Kind, Vec<u8>),
    Ended(String),
}

#[derive(Debug, Clone)]
pub struct Options {
    pub overrides: Vec<MmioOverride>,
    pub simulate: bool,
    /// Use the kernel fallback for these controllers regardless of driver.
    pub force_generic: Vec<String>,
    /// Max time sofclock may disable interrupts (0 = never).
    pub irq_off_max_ns: u32,
    pub sampler: SamplerConfig,
    pub default_rate_hz: f64,
    /// `model` publish rate unless a client asks for more.
    pub model_hz: f64,
    pub power: power::Settings,
}

/// Keeps the controller out of runtime suspend so its frame counter runs.
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

struct Want {
    rate_hz: f64,
    model_hz: Option<f64>,
    samples: bool,
    /// Monotonic, and Unix ms for `info`.
    expires: Option<(Instant, u64)>,
    mqtt: bool,
}

fn expiry(after: Duration) -> (Instant, u64) {
    (Instant::now() + after, (SystemTime::now() + after).duration_since(UNIX_EPOCH).unwrap_or_default().as_millis() as u64)
}

struct StartFailed {
    message: String,
    retry_at: Instant,
    delay: Duration,
}

struct Running {
    // Field order matters: stop the sampler before restoring runtime PM.
    sampler: Sampler,
    rate_hz: f64,
    _pm: Option<PmGuard>,
}

struct Slot {
    board: Option<Board>,
    present: bool,
    retry_at: Instant,
    backoff: Duration,
    cfg: config::Board,
    state: String,
    /// ELF SHA-256 reported by the board.
    running_elf: Option<String>,
    status: Value,
    chrony_busy: bool,
    /// Already warned that its firmware is left alone.
    update_given_up: bool,
    /// The session ended and took its feed out of the hub, before its Ended arrived.
    ended: bool,
}

/// An MQTT client acting as a controller's GNSS reference.
struct ExtRef {
    client: String,
    feed: Arc<Feed>,
    last: Instant,
    /// A client whose tags never match must not hold the controller forever.
    rejected_since: Option<Instant>,
    chrony_busy: bool,
}

struct ImageSlot {
    image: Option<Arc<Image>>,
    mtime: Option<SystemTime>,
    /// New mtime and when it was first seen.
    changed: Option<(Option<SystemTime>, Instant)>,
}

struct Discovered {
    devices: Vec<UsbDevice>,
    controllers: Vec<Controller>,
    at: Instant,
}

pub struct Daemon {
    opts: Options,
    /// None: rescan every time.
    usb_watch: Option<Arc<uevent::Watch>>,
    discovered: Option<Discovered>,
    gnss: config::Gnss,
    tx: Sender<Msg>,
    samplers: BTreeMap<String, Running>,
    /// Held while anything is sampled.
    power: Option<power::Held>,
    /// controller -> client -> request
    wants: BTreeMap<String, BTreeMap<String, Want>>,
    start_failed: BTreeMap<String, StartFailed>,
    slots: BTreeMap<String, Slot>,
    hub: Hub,
    /// controller -> the board feeding its GNSS reference
    board_refs: BTreeMap<String, Arc<Feed>>,
    /// controller -> the external reference feeding it
    refs: BTreeMap<String, ExtRef>,
    /// Last problem told to each client, so it is told only once.
    ref_told: BTreeMap<(String, String), String>,
    images: BTreeMap<PathBuf, ImageSlot>,
    /// serial -> (image ELF SHA-256, attempts)
    update_tries: BTreeMap<String, (String, u32)>,
    mqtt: Option<Mqtt>,
    chrony: Chrony,
    stdout: bool,
    /// Stdout broke (e.g. closed pipe): stop.
    stdout_failed: std::cell::Cell<bool>,
    /// Last published `controllers/<id>/info`.
    infos: BTreeMap<String, Value>,
    /// When each controller's `model` was last published, and its state then.
    model_pub: BTreeMap<String, (Instant, ModelState)>,
    pub sample_limit: Option<u64>,
    samples: u64,
}

/// Changes that clients should see at once rather than at the next `model_hz` tick.
#[derive(Debug, PartialEq)]
struct ModelState {
    epoch: Value,
    fits: [bool; 3],
    reference: Value,
}

impl ModelState {
    fn of(sample: &Value) -> Self {
        Self {
            epoch: sample["epoch"].clone(),
            fits: ["model_raw", "model_realtime", "model_gnss"].map(|k| !sample[k].is_null()),
            reference: sample["gnss"]["board"].clone(),
        }
    }
}

fn backoff(b: Duration) -> Duration {
    (b * 2).clamp(RETRY_MIN, RETRY_MAX)
}

impl Daemon {
    pub fn new(opts: Options, gnss: config::Gnss, tx: Sender<Msg>, mqtt: Option<Mqtt>, stdout: bool) -> Self {
        let usb_watch = uevent::watch().map_err(|e| warn!("{e:#}; scanning USB every second")).ok();
        Self {
            opts,
            usb_watch,
            discovered: None,
            gnss,
            tx,
            samplers: BTreeMap::new(),
            power: None,
            wants: BTreeMap::new(),
            start_failed: BTreeMap::new(),
            slots: BTreeMap::new(),
            hub: Hub::default(),
            board_refs: BTreeMap::new(),
            refs: BTreeMap::new(),
            ref_told: BTreeMap::new(),
            images: BTreeMap::new(),
            update_tries: BTreeMap::new(),
            mqtt,
            chrony: Chrony::new(),
            stdout,
            stdout_failed: std::cell::Cell::new(false),
            infos: BTreeMap::new(),
            model_pub: BTreeMap::new(),
            sample_limit: None,
            samples: 0,
        }
    }

    pub fn want(&mut self, controller: &str, client: &str, rate_hz: f64) {
        let w = Want { rate_hz, model_hz: None, samples: false, expires: None, mqtt: false };
        self.wants.entry(controller.into()).or_default().insert(client.into(), w);
    }

    /// `model` publish rate: the configured one, or more if a client asks.
    fn model_hz(&self, controller: &str) -> f64 {
        let asked = self.wants.get(controller).into_iter().flat_map(|w| w.values()).filter_map(|w| w.model_hz);
        asked.fold(self.opts.model_hz, f64::max)
    }

    fn drop_want(&mut self, controller: &str, client: &str) {
        if let Some(w) = self.wants.get_mut(controller) {
            w.remove(client);
            if w.is_empty() {
                self.wants.remove(controller);
            }
        }
    }

    fn expire_wants(&mut self) {
        let now = Instant::now();
        let expired: Vec<(String, String)> = self
            .wants
            .iter()
            .flat_map(|(c, w)| w.iter().filter(|(_, w)| w.expires.is_some_and(|(t, _)| t <= now)).map(|(cl, _)| (c.clone(), cl.clone())))
            .collect();
        for (c, cl) in &expired {
            info!("{c}: {cl}'s request expired (not renewed within its ttl_s)");
            self.drop_want(c, cl);
        }
    }

    pub fn run(mut self, rx: &Receiver<Msg>, stop: &AtomicBool) {
        let mut next_scan = Instant::now();
        while !stop.load(Ordering::Relaxed) && !self.stdout_failed.get() && self.sample_limit.is_none_or(|l| self.samples < l) {
            let now = Instant::now();
            if now >= next_scan {
                self.scan();
                if let Some(p) = &mut self.power {
                    p.refresh();
                }
                next_scan = now + SCAN_EVERY;
            }
            let wait = next_scan.saturating_duration_since(now).min(Duration::from_millis(200));
            match rx.recv_timeout(wait) {
                Ok(m) => self.handle(m),
                Err(RecvTimeoutError::Timeout) => {}
                Err(RecvTimeoutError::Disconnected) => break,
            }
        }
        self.shutdown();
    }

    fn shutdown(&mut self) {
        let (b, s) = (self.slots.values().filter(|s| s.board.is_some()).count(), self.samplers.len());
        if b + s > 0 {
            info!("stopping {b} board(s), {s} sampler(s)");
        }
        let boards: Vec<String> = self.slots.iter().filter(|(_, s)| s.board.is_some()).map(|(k, _)| k.clone()).collect();
        for slot in self.slots.values_mut() {
            slot.board = None;
        }
        let sampled: Vec<String> = self.samplers.keys().cloned().collect();
        self.samplers.clear();
        self.power = None;
        // Clear retained state so it doesn't look like we're still running.
        for serial in boards {
            self.set_status(&serial, json!({"state": "stopped"}));
        }
        if let Some(m) = &self.mqtt {
            for c in sampled {
                m.publish(&format!("controllers/{c}/model"), Vec::new(), true);
            }
        }
        self.publish_infos();
        if let Some(m) = self.mqtt.take() {
            m.shutdown();
        }
    }

    // ---- output helpers ----

    fn print(&self, v: &Value) {
        use std::io::Write;
        if self.stdout && !self.stdout_failed.get() && writeln!(std::io::stdout().lock(), "{v}").is_err() {
            self.stdout_failed.set(true);
        }
    }

    fn board_event(&self, serial: &str, mut ev: Value) {
        ev["board"] = json!(serial);
        if let Some(m) = &self.mqtt {
            m.publish_event(&format!("gnss/{serial}/event"), &ev);
        }
        self.print(&ev);
    }

    fn controller_event(&self, controller: &str, ev: &Value) {
        if let Some(m) = &self.mqtt {
            m.publish_event(&format!("controllers/{controller}/event"), ev);
        }
        self.print(ev);
    }

    fn publish_status(&mut self, serial: &str) {
        let Some(slot) = self.slots.get(serial) else { return };
        if let Some(m) = &self.mqtt {
            m.publish_json(&format!("gnss/{serial}/status"), &slot.status, true);
        }
    }

    fn set_status(&mut self, serial: &str, status: Value) {
        if let Some(slot) = self.slots.get_mut(serial) {
            slot.state = status["state"].as_str().unwrap_or("").to_string();
            slot.status = status;
        }
        self.publish_status(serial);
    }

    // ---- messages ----

    fn handle(&mut self, m: Msg) {
        match m {
            Msg::Sample(s) => self.on_sample(*s),
            Msg::Controller { controller, event } => self.on_controller_event(&controller, &event),
            Msg::Board { serial, id, out } => {
                // Drop late messages from a replaced session.
                if self.slots.get(&serial).and_then(|s| s.board.as_ref()).is_some_and(|b| b.id == id) {
                    self.on_board(&serial, out);
                }
            }
            Msg::Mqtt(i) => self.on_mqtt(i),
        }
    }

    fn on_controller_event(&mut self, controller: &str, ev: &Value) {
        match ev["event"].as_str() {
            // The counter may have restarted: clear the model until the next sample.
            Some("error") if self.samplers.contains_key(controller) => {
                if let Some(m) = &self.mqtt {
                    m.publish(&format!("controllers/{controller}/model"), Vec::new(), true);
                }
            }
            Some(e @ ("ref_accepted" | "ref_rejected")) => {
                if let Some(r) = self.refs.get_mut(controller).filter(|r| ev["client"].as_str() == Some(r.client.as_str())) {
                    if e == "ref_accepted" {
                        r.rejected_since = None;
                    } else {
                        r.rejected_since.get_or_insert_with(Instant::now);
                    }
                }
            }
            _ => {}
        }
        self.controller_event(controller, ev);
    }

    fn on_sample(&mut self, s: SampleOut) {
        // Sampler already stopped and its model cleared: keep it cleared.
        if !self.samplers.contains_key(&s.controller) {
            return;
        }
        self.samples += 1;
        if let Some(m) = &self.mqtt {
            let c = &s.controller;
            let wants = self.wants.get(c);
            if wants.is_some_and(|w| w.values().any(|w| w.samples)) {
                m.publish_json(&format!("controllers/{c}/sample"), &s.event, false);
            }
            let rate = self.samplers.get(c).map_or(1.0, |r| r.rate_hz);
            // Half a sample early, or the interval would grow by up to one sample each time.
            let interval = Duration::from_secs_f64((1.0 / self.model_hz(c) - 0.5 / rate).max(0.0));
            let state = ModelState::of(&s.event);
            let now = Instant::now();
            let due = self.model_pub.get(c).is_none_or(|(t, st)| *st != state || now.duration_since(*t) >= interval);
            if due {
                let mut model = Map::new();
                for k in ["epoch", "unit_ns", "counter", "model_raw", "model_realtime", "model_gnss", "gnss", "frame_start"] {
                    model.insert(k.into(), s.event[k].clone());
                }
                m.publish_json(&format!("controllers/{c}/model"), &Value::Object(model), true);
                self.model_pub.insert(c.clone(), (now, state));
            }
        }
        self.print(&s.event);
        let (Some(name), Some(p)) = (s.board, s.chrony) else { return };
        // Queued from a reference that was just replaced: it must not take the socket back.
        if !self.hub.lock().unwrap().get(&s.controller).is_some_and(|f| f.serial == name) {
            return;
        }
        let client = name.strip_prefix(external::NAME_PREFIX);
        let path = match client {
            Some(c) => self.gnss.external(c).and_then(|e| e.chrony.clone()),
            None => self.slots.get(&name).and_then(|sl| sl.cfg.chrony.clone()),
        };
        let Some(path) = path else { return };
        let busy = match self.chrony.send(&path, &name, p, clock::raw()) {
            Sent::Ok => None,
            Sent::Busy(owner) => Some(owner),
        };
        let flag = match client {
            Some(_) => self.refs.get_mut(&s.controller).filter(|r| r.feed.serial == name).map(|r| &mut r.chrony_busy),
            None => self.slots.get_mut(&name).map(|sl| &mut sl.chrony_busy),
        };
        let was = flag.map(|f| std::mem::replace(f, busy.is_some())).unwrap_or(false);
        if let (Some(owner), false) = (busy, was) {
            warn!("{name}: chrony socket {} is fed by {owner}", path.display());
            let ev = json!({"event": "chrony_busy", "socket": path, "owner": owner});
            match client {
                Some(c) => {
                    let mut ev = ev;
                    ev["controller"] = json!(s.controller);
                    ev["client"] = json!(c);
                    self.controller_event(&s.controller, &ev);
                }
                None => self.board_event(&name, ev),
            }
        }
    }

    fn on_board(&mut self, serial: &str, out: BoardOut) {
        let Some(slot) = self.slots.get(serial) else { return };
        let controller = slot.board.as_ref().map(|b| b.info.controller.clone());
        match out {
            BoardOut::Status(mut ev) => {
                ev.as_object_mut().map(|o| o.remove("event"));
                if let Some(b) = &slot.board {
                    ev["controller"] = json!(b.info.controller);
                    ev["tty"] = json!(b.info.tty);
                    ev["port"] = json!(b.info.port);
                }
                self.on_status(serial, &ev);
                if RUNNING_STATES.contains(&ev["state"].as_str().unwrap_or("")) {
                    // Healthy again: reset the backoff.
                    if let Some(sl) = self.slots.get_mut(serial) {
                        sl.backoff = Duration::ZERO;
                    }
                }
                let mut printed = ev.clone();
                printed["event"] = json!("gnss_status");
                printed["board"] = json!(serial);
                self.print(&printed);
                self.set_status(serial, ev);
                self.assign_feeds();
            }
            BoardOut::Pps(mut ev) => {
                ev["board"] = json!(serial);
                ev["controller"] = json!(controller);
                if let Some(m) = &self.mqtt {
                    m.publish_json(&format!("gnss/{serial}/pps"), &ev, false);
                }
                self.print(&ev);
            }
            BoardOut::Frame(kind, f) => {
                if let Some(m) = &self.mqtt {
                    m.publish(&format!("gnss/{serial}/{}", mqtt::frame_topic(kind, &f)), f.clone(), false);
                }
                if self.stdout {
                    self.print(&frame_json(serial, kind, &f));
                }
            }
            BoardOut::Ended(reason) => {
                warn!("{serial}: {reason}");
                let restarting = slot.state == "restarting";
                let slot = self.slots.get_mut(serial).unwrap();
                slot.board = None;
                // Reopen once it re-enumerates, not while the old device is still leaving.
                slot.backoff = if restarting { Duration::ZERO } else { backoff(slot.backoff) };
                slot.retry_at = Instant::now() + if restarting { RESTART_HOLD } else { slot.backoff };
                self.chrony.release(serial);
                self.board_event(serial, json!({"event": "detached", "reason": reason}));
                self.set_status(serial, json!({"state": "detached", "message": reason}));
                self.assign_feeds();
                self.reconcile();
            }
        }
    }

    /// Picks the board that feeds each controller's GNSS reference: one whose
    /// receiver works (`locked`: pulses accepted), keeping the current one while
    /// it does. Boards without a working receiver stay open but feed nothing, so
    /// they block neither a board with GNSS on the same controller nor an MQTT
    /// reference.
    fn assign_feeds(&mut self) {
        let mut hub = self.hub.lock().unwrap();
        // An ended session took its feed out of the hub: don't put it back.
        for (c, f) in &self.board_refs {
            if !hub.get(c).is_some_and(|h| Arc::ptr_eq(h, f)) {
                if let Some(sl) = self.slots.get_mut(&f.serial).filter(|sl| sl.board.as_ref().is_some_and(|b| Arc::ptr_eq(&b.feed, f))) {
                    sl.ended = true;
                }
            }
        }
        let mut ready: BTreeMap<String, Vec<Arc<Feed>>> = BTreeMap::new();
        for slot in self.slots.values().filter(|s| !s.ended) {
            let working = RUNNING_STATES.contains(&slot.state.as_str()) && slot.status["locked"] == true;
            if let Some(b) = slot.board.as_ref().filter(|_| working) {
                ready.entry(b.info.controller.clone()).or_default().push(b.feed.clone());
            }
        }
        let mut changes: Vec<(String, Option<String>, Option<String>)> = vec![];
        let controllers: BTreeSet<String> = self.board_refs.keys().chain(ready.keys()).cloned().collect();
        for c in controllers {
            let feeds = ready.get(&c);
            let keep = self.board_refs.get(&c).is_some_and(|f| {
                hub.get(&c).is_some_and(|h| Arc::ptr_eq(h, f)) && feeds.is_some_and(|r| r.iter().any(|rf| Arc::ptr_eq(rf, f)))
            });
            if keep {
                continue;
            }
            let mut old = self.board_refs.remove(&c).map(|f| f.serial.clone());
            if hub.get(&c).is_some_and(|f| f.client.is_none()) {
                hub.remove(&c);
            }
            let new = feeds.map(|r| r[0].clone());
            if let Some(n) = &new {
                // Or the MQTT reference it takes over from.
                old = old.or(hub.insert(c.clone(), n.clone()).map(|f| f.serial.clone()));
                self.board_refs.insert(c.clone(), n.clone());
            }
            changes.push((c, new.map(|n| n.serial.clone()), old));
        }
        drop(hub);
        for (c, new, old) in changes {
            if let Some(o) = old.as_ref().filter(|o| !o.starts_with(external::NAME_PREFIX)) {
                // Let the next reference feed the chrony socket.
                self.chrony.release(o);
            }
            match (&new, &old) {
                (Some(n), Some(o)) => info!("{c}: board {n} replaces {o} as the GNSS reference"),
                (Some(n), None) => info!("{c}: board {n} is the GNSS reference"),
                (None, Some(o)) => info!("{c}: board {o} is no longer the GNSS reference"),
                (None, None) => continue,
            }
            self.controller_event(&c, &json!({"event": "reference", "controller": c, "board": new}));
        }
    }

    /// Tracks firmware updates.
    fn on_status(&mut self, serial: &str, ev: &Value) {
        let state = ev["state"].as_str().unwrap_or("").to_string();
        let elf = ev["firmware"]["elf_sha256"].as_str().map(str::to_string);
        let image = self.slots.get(serial).and_then(|s| s.cfg.firmware.clone()).and_then(|p| self.image(&p));
        let slot = self.slots.get_mut(serial).unwrap();
        slot.running_elf = elf.clone().or(slot.running_elf.take());
        let Some(img) = image else { return };
        if state == "updating" && slot.state != "updating" {
            let t = self.update_tries.entry(serial.into()).or_insert((img.elf_sha256.clone(), 0));
            if t.0 != img.elf_sha256 {
                *t = (img.elf_sha256.clone(), 0);
            }
            t.1 += 1;
        }
        if elf.as_deref() == Some(img.elf_sha256.as_str()) && self.update_tries.remove(serial).is_some() {
            info!("{serial}: now runs firmware {}", img.version);
            self.board_event(serial, json!({"event": "firmware_updated", "version": img.version}));
        }
    }

    fn on_mqtt(&mut self, i: mqtt::In) {
        match i {
            mqtt::In::Connected => {
                if let Some(m) = &self.mqtt {
                    m.publish_json("status", &json!({"state": "online", "version": env!("CARGO_PKG_VERSION"), "pid": std::process::id()}), true);
                    for (serial, slot) in &self.slots {
                        m.publish_json(&format!("gnss/{serial}/status"), &slot.status, true);
                    }
                }
                // Clear stale retained models of controllers we don't sample, or that left while we were away.
                if let Some(m) = &self.mqtt {
                    let found = self.discovered.iter().flat_map(|d| d.controllers.iter().map(|c| &c.id));
                    let ids: BTreeSet<&String> = found.chain(self.infos.keys()).collect();
                    for id in ids.into_iter().filter(|id| !self.samplers.contains_key(*id)) {
                        m.publish(&format!("controllers/{id}/model"), Vec::new(), true);
                    }
                }
                // Retained requests get redelivered; one withdrawn while we were away won't.
                for w in self.wants.values_mut().flat_map(|w| w.values_mut()).filter(|w| w.mqtt && w.expires.is_none()) {
                    w.expires = Some(expiry(RECONFIRM));
                }
                // Republish everything, and clear infos of controllers gone meanwhile.
                for v in self.infos.values_mut() {
                    *v = Value::Null;
                }
                self.publish_infos();
            }
            mqtt::In::Want { controller, client, req } => {
                match req {
                    Some(mqtt::WantReq { rate_hz, model_hz, samples, ttl_s }) => {
                        let rate = rate_hz.unwrap_or(self.opts.default_rate_hz).clamp(MIN_RATE_HZ, MAX_RATE_HZ);
                        let expires = ttl_s.map(|t| expiry(Duration::from_secs_f64(t)));
                        let w = self.wants.entry(controller.clone()).or_default();
                        // Renewals: no log line and no immediate retry of a failed controller.
                        let new = w.get(&client).is_none_or(|o| o.rate_hz != rate || o.expires.is_some() != ttl_s.is_some());
                        if new {
                            match ttl_s {
                                Some(t) => info!("{controller}: {client} wants {rate} Hz (ttl {t} s)"),
                                None => info!("{controller}: {client} wants {rate} Hz"),
                            }
                            self.start_failed.remove(&controller);
                        }
                        w.insert(client.clone(), Want { rate_hz: rate, model_hz, samples, expires, mqtt: true });
                    }
                    None => {
                        info!("{controller}: {client} no longer wants samples");
                        self.drop_want(&controller, &client);
                    }
                }
                self.reconcile();
                self.publish_infos();
            }
            mqtt::In::Ref { controller, client, payload, t_rx } => self.on_ref(&controller, &client, &payload, t_rx),
            mqtt::In::Tx { serial, data } => match self.slots.get(&serial).and_then(|s| s.board.as_ref()) {
                Some(b) => b.send(data),
                None => warn!("{serial}: tx for a board that is not attached"),
            },
            mqtt::In::Cmd { name, payload, reply: to } => {
                let result = self.command(&name, &payload);
                let mut reply = match result {
                    Ok(Value::Object(mut o)) => {
                        o.insert("ok".into(), json!(true));
                        Value::Object(o)
                    }
                    Ok(v) => json!({"ok": true, "result": v}),
                    Err(e) => json!({"ok": false, "error": format!("{e:#}")}),
                };
                if let Some(c) = to.correlation {
                    reply["correlation"] = c;
                }
                match (to.topic, &self.mqtt) {
                    (Some(topic), Some(m)) => m.reply(&topic, to.correlation_data, &reply),
                    _ => warn!("cmd/{name} without a reply topic (reply_to or MQTT 5 response topic)"),
                }
            }
        }
    }

    /// A board on the same controller takes precedence.
    fn on_ref(&mut self, controller: &str, client: &str, payload: &[u8], t_rx: Ns) {
        let Some(ext) = self.gnss.external(client).cloned() else {
            return self.ref_problem(controller, client, json!({"event": "ref_rejected", "reason": "not_allowed"}));
        };
        let tags = match external::parse(payload) {
            Ok(t) => t,
            Err(e) => return self.ref_problem(controller, client, json!({"event": "ref_rejected", "reason": "invalid", "message": e})),
        };
        let attached = self.hub.lock().unwrap().get(controller).cloned();
        let feed = match (attached, self.refs.get(controller)) {
            (Some(f), Some(r)) if r.client == client && Arc::ptr_eq(&f, &r.feed) => f,
            (Some(f), _) => {
                return self.ref_problem(controller, client, json!({"event": "ref_busy", "owner": f.serial}));
            }
            (None, _) => {
                let f = Arc::new(Feed::external(client));
                self.hub.lock().unwrap().insert(controller.into(), f.clone());
                info!("{controller}: MQTT client {client} is the GNSS reference");
                self.refs.insert(
                    controller.into(),
                    ExtRef { client: client.into(), feed: f.clone(), last: Instant::now(), rejected_since: None, chrony_busy: false },
                );
                f
            }
        };
        self.ref_told.remove(&(controller.to_string(), client.to_string()));
        for t in tags {
            let Some(utc_ns) = t.t_gnss_ns.checked_sub(ext.sof_delay_ns) else {
                let msg = "t_gnss_ns - sof_delay_ns overflows";
                return self.ref_problem(controller, client, json!({"event": "ref_rejected", "reason": "invalid", "message": msg}));
            };
            feed.push(Tag { at: t.at, utc_ns, unc_ns: t.unc_ns, t_rx, source: Source::External });
        }
        if let Some(r) = self.refs.get_mut(controller) {
            r.last = Instant::now();
        }
    }

    /// Sent once until it changes.
    fn ref_problem(&mut self, controller: &str, client: &str, mut ev: Value) {
        let key = (controller.to_string(), client.to_string());
        // Compare the whole event: a ref_busy with a new owner is news too.
        let what = ev.to_string();
        if self.ref_told.get(&key) == Some(&what) {
            return;
        }
        warn!("{controller}: tags from {client}: {ev}");
        self.ref_told.insert(key, what);
        ev["controller"] = json!(controller);
        ev["client"] = json!(client);
        self.controller_event(controller, &ev);
    }

    /// Drops external references that went quiet or were taken over by a board.
    fn check_refs(&mut self) {
        let lost: Vec<(String, Option<String>)> = {
            let hub = self.hub.lock().unwrap();
            self.refs
                .iter()
                .filter_map(|(c, r)| match hub.get(c) {
                    Some(f) if Arc::ptr_eq(f, &r.feed) => {
                        let lost = Duration::from_secs(LOST_AFTER_S);
                        (r.last.elapsed() > lost || r.rejected_since.is_some_and(|t| t.elapsed() > lost)).then(|| (c.clone(), None))
                    }
                    other => Some((c.clone(), other.map(|f| f.serial.clone()))),
                })
                .collect()
        };
        for (c, owner) in lost {
            let r = self.refs.remove(&c).expect("listed above");
            self.chrony.release(&r.feed.serial);
            match owner {
                None => {
                    let mut hub = self.hub.lock().unwrap();
                    if hub.get(&c).is_some_and(|f| Arc::ptr_eq(f, &r.feed)) {
                        hub.remove(&c);
                    }
                    drop(hub);
                    let reason = if r.last.elapsed() > Duration::from_secs(LOST_AFTER_S) { "silent" } else { "rejected" };
                    info!("{c}: GNSS reference {} lost ({reason} for {LOST_AFTER_S} s)", r.client);
                    self.ref_told.remove(&(c.clone(), r.client.clone()));
                    self.controller_event(&c, &json!({"event": "ref_lost", "controller": c, "client": r.client, "reason": reason}));
                }
                Some(owner) => self.ref_problem(&c, &r.client, json!({"event": "ref_busy", "owner": owner})),
            }
        }
    }

    fn command(&self, name: &str, payload: &[u8]) -> Result<Value> {
        match name {
            "list" => self.list(),
            "lookup" => {
                let q: DeviceQuery = serde_json::from_slice(payload).context("lookup request")?;
                lookup(&q, self.opts.simulate)
            }
            _ => bail!("unknown command {name}"),
        }
    }

    pub fn list(&self) -> Result<Value> {
        let mut v = match &self.discovered {
            Some(d) => list_of(&d.devices, &d.controllers),
            None => list(self.opts.simulate)?,
        };
        for c in v["controllers"].as_array_mut().into_iter().flatten() {
            let id = c["id"].as_str().unwrap_or("").to_string();
            c["rate_hz"] = json!(self.samplers.get(&id).map(|r| r.rate_hz));
            c["model_hz"] = json!(self.samplers.get(&id).map(|r| self.model_hz(&id).min(r.rate_hz)));
        }
        v["boards"] = json!(self.slots.iter().map(|(s, sl)| { let mut st = sl.status.clone(); st["board"] = json!(s); st }).collect::<Vec<_>>());
        Ok(v)
    }

    // ---- periodic work ----

    fn discover(&mut self) -> Result<()> {
        let changed = match &self.usb_watch {
            Some(w) if w.alive.load(Ordering::Relaxed) => w.changed.swap(false, Ordering::Relaxed),
            _ => true,
        };
        if changed || self.discovered.as_ref().is_none_or(|d| d.at.elapsed() >= FULL_RESCAN) {
            let devices = discovery::devices(self.opts.simulate).context("listing USB devices")?;
            let controllers = discovery::controllers(self.opts.simulate).context("listing USB controllers")?;
            self.discovered = Some(Discovered { devices, controllers, at: Instant::now() });
        }
        Ok(())
    }

    fn scan(&mut self) {
        if let Err(e) = self.discover() {
            warn!("{e:#}");
            return;
        }
        let devices = self.discovered.as_ref().map(|d| d.devices.clone()).unwrap_or_default();
        self.check_images();
        self.scan_boards(&devices);
        self.check_refs();
        self.expire_wants();
        self.check_samplers();
        self.reconcile();
        self.publish_infos();
    }

    /// A sampler thread that ended on its own panicked; retry it later.
    fn check_samplers(&mut self) {
        let dead: Vec<String> = self.samplers.iter().filter(|(_, r)| r.sampler.is_finished()).map(|(c, _)| c.clone()).collect();
        for c in dead {
            self.samplers.remove(&c);
            let msg = "sampler thread ended".to_string();
            warn!("{c}: {msg}; retrying in {} s", SAMPLER_RETRY.as_secs());
            if let Some(m) = &self.mqtt {
                m.publish(&format!("controllers/{c}/model"), Vec::new(), true);
            }
            self.controller_event(&c, &json!({"event": "error", "controller": c, "message": msg}));
            self.start_failed.insert(c, StartFailed { message: msg, retry_at: Instant::now() + SAMPLER_RETRY, delay: SAMPLER_RETRY });
        }
    }

    fn scan_boards(&mut self, devices: &[UsbDevice]) {
        let found: BTreeMap<String, BoardInfo> = devices
            .iter()
            // Explicitly named devices are used whatever their descriptors.
            .filter(|d| !matches!(self.gnss.attach, config::Attach::All) || is_board(d))
            .filter_map(|d| {
                let serial = d.serial.clone()?;
                let info = BoardInfo { serial: serial.clone(), tty: d.tty.clone()?, controller: d.controller.clone(), port: d.port.clone() };
                self.gnss.attach.includes(&serial).then_some((serial, info))
            })
            .collect();
        let now = Instant::now();
        let gone: Vec<String> = self.slots.iter().filter(|(s, sl)| sl.present && !found.contains_key(*s)).map(|(s, _)| s.clone()).collect();
        for serial in gone {
            let slot = self.slots.get_mut(&serial).unwrap();
            slot.present = false;
            slot.board = None;
            slot.backoff = Duration::ZERO;
            slot.retry_at = now;
            self.chrony.release(&serial);
            info!("{serial}: unplugged");
            self.set_status(&serial, json!({"state": "absent"}));
        }

        for (serial, info) in found {
            let slot = self.slots.entry(serial.clone()).or_insert_with(|| Slot {
                board: None,
                present: true,
                retry_at: now,
                backoff: Duration::ZERO,
                cfg: self.gnss.board(&serial),
                state: String::new(),
                running_elf: None,
                status: Value::Null,
                chrony_busy: false,
                update_given_up: false,
                ended: false,
            });
            slot.present = true;
            // Re-enumerated before the old session noticed.
            if slot.board.as_ref().is_some_and(|b| b.info != info) {
                slot.board = None;
            }
            if slot.board.is_none() && now >= slot.retry_at {
                self.open_board(info);
            }
        }
        self.assign_feeds();
    }

    fn open_board(&mut self, info: BoardInfo) {
        let serial = info.serial.clone();
        let b = self.gnss.board(&serial);
        let mut firmware = b.firmware.as_deref().and_then(|p| self.image(p));
        if let Some(img) = &firmware {
            let tries = self.update_tries.get(&serial).filter(|t| t.0 == img.elf_sha256).map_or(0, |t| t.1);
            if tries >= MAX_UPDATE_TRIES {
                let slot = self.slots.get_mut(&serial).unwrap();
                if !std::mem::replace(&mut slot.update_given_up, true) {
                    warn!("{serial}: {tries} updates to firmware {} failed; leaving its firmware alone", img.version);
                    self.board_event(&serial, json!({"event": "firmware_failed", "version": img.version, "tries": tries}));
                }
                firmware = None;
            }
        }
        let cfg = session::Config {
            baud: b.baud,
            initial_baud: b.initial_baud,
            configure: b.configure,
            strict: b.strict,
            frames: b.frames,
            div: b.div,
            sof_delay_ns: b.sof_delay_ns as Ns,
            firmware,
        };
        let result = device::spawn(info.clone(), cfg, &self.hub, &self.tx);
        let slot = self.slots.get_mut(&serial).unwrap();
        slot.cfg = b;
        match result {
            Ok(board) => {
                slot.board = Some(board);
                slot.ended = false;
                let st = json!({"state": "opening", "controller": info.controller, "tty": info.tty, "port": info.port});
                self.set_status(&serial, st);
            }
            Err(e) => {
                slot.backoff = backoff(slot.backoff);
                slot.retry_at = Instant::now() + slot.backoff;
                let msg = format!("{e:#}");
                warn!("{serial}: {msg}; retrying in {} s", slot.backoff.as_secs());
                self.set_status(&serial, json!({"state": "error", "message": msg, "tty": info.tty, "port": info.port}));
            }
        }
    }

    /// Cached; check_images reloads changed files.
    fn image(&mut self, path: &Path) -> Option<Arc<Image>> {
        if !self.images.contains_key(path) {
            let slot = match Image::load(path) {
                Ok(img) => {
                    info!("firmware {}: {} ({} bytes)", path.display(), img.version, img.data.len());
                    ImageSlot { mtime: img.mtime, image: Some(Arc::new(img)), changed: None }
                }
                Err(e) => {
                    warn!("{e:#}; boards keep their firmware");
                    ImageSlot { image: None, mtime: mtime(path), changed: None }
                }
            };
            self.images.insert(path.to_path_buf(), slot);
        }
        self.images[path].image.clone()
    }

    /// Reloads changed images and restarts boards that should update.
    fn check_images(&mut self) {
        let now = Instant::now();
        let mut changed = vec![];
        for (p, s) in &mut self.images {
            let m = mtime(p);
            if m == s.mtime {
                s.changed = None;
            } else if s.changed.is_none_or(|(c, _)| c != m) {
                s.changed = Some((m, now));
            } else if s.changed.is_some_and(|(_, t)| now >= t + IMAGE_SETTLE) {
                changed.push(p.clone());
            }
        }
        for p in changed {
            self.images.remove(&p);
            // A new image gets fresh tries and a fresh warning.
            for s in self.slots.values_mut().filter(|s| s.cfg.firmware.as_deref() == Some(p.as_path())) {
                s.update_given_up = false;
            }
            let Some(img) = self.image(&p) else { continue };
            let restart: Vec<String> = self
                .slots
                .iter()
                .filter(|(_, s)| s.board.is_some() && s.cfg.firmware.as_deref() == Some(p.as_path()))
                .filter(|(_, s)| s.running_elf.as_deref().is_some_and(|e| e != img.elf_sha256))
                .map(|(serial, _)| serial.clone())
                .collect();
            for serial in restart {
                info!("{serial}: new firmware {}, restarting the board session", img.version);
                self.update_tries.remove(&serial);
                let slot = self.slots.get_mut(&serial).unwrap();
                slot.board = None;
                slot.retry_at = Instant::now();
                self.chrony.release(&serial);
            }
        }
    }

    /// Starts, retunes and stops samplers to match boards and requests.
    fn reconcile(&mut self) {
        let mut desired: BTreeMap<String, f64> = BTreeMap::new();
        for (c, w) in &self.wants {
            if let Some(r) = w.values().map(|w| w.rate_hz).reduce(f64::max) {
                desired.insert(c.clone(), r);
            }
        }
        for b in self.slots.values().filter_map(|s| s.board.as_ref()) {
            let r = desired.entry(b.info.controller.clone()).or_insert(self.opts.default_rate_hz);
            *r = r.max(self.opts.default_rate_hz);
        }
        let stop: Vec<String> = self.samplers.keys().filter(|c| !desired.contains_key(*c)).cloned().collect();
        for c in stop {
            self.samplers.remove(&c);
            self.model_pub.remove(&c);
            info!("{c}: stopped");
            if let Some(m) = &self.mqtt {
                m.publish(&format!("controllers/{c}/model"), Vec::new(), true);
            }
        }
        let now = Instant::now();
        self.start_failed.retain(|c, _| desired.contains_key(c));
        for (c, rate) in desired {
            if let Some(r) = self.samplers.get_mut(&c) {
                if r.rate_hz != rate {
                    r.sampler.set_rate(rate);
                    r.rate_hz = rate;
                    info!("{c}: sampling at {rate} Hz");
                }
                continue;
            }
            if self.start_failed.get(&c).is_some_and(|f| now < f.retry_at) {
                continue;
            }
            match self.start_sampler(&c, rate) {
                Ok(r) => {
                    info!("{c}: sampling at {rate} Hz");
                    self.start_failed.remove(&c);
                    self.samplers.insert(c, r);
                }
                Err(e) => {
                    let msg = format!("{e:#}");
                    // Same error: back off, or a missing controller is retried every 10 s forever.
                    let delay = match self.start_failed.get(&c) {
                        Some(f) if f.message == msg => (f.delay * 2).min(SAMPLER_RETRY_MAX),
                        _ => SAMPLER_RETRY,
                    };
                    warn!("{c}: {msg}; retrying in {} s", delay.as_secs());
                    let ev = json!({"event": "error", "controller": c, "message": msg});
                    if let Some(m) = &self.mqtt {
                        m.publish_event(&format!("controllers/{c}/event"), &ev);
                    }
                    self.print(&ev);
                    self.start_failed.insert(c, StartFailed { message: msg, retry_at: now + delay, delay });
                }
            }
        }
        if self.samplers.is_empty() {
            self.power = None;
        } else if self.power.is_none() {
            self.power = power::Held::apply(&self.opts.power);
        }
    }

    fn start_sampler(&self, id: &str, rate: f64) -> Result<Running> {
        let mut ctrl = discovery::find_controller(id, self.opts.simulate)?;
        if self.opts.force_generic.iter().any(|g| g == id) {
            ctrl.backend = Some(BackendKind::KernelFrameNumber);
        }
        // A suspended controller reads all-ones and its counter stops.
        let pm = PmGuard::hold(&ctrl);
        let backend = backend::open(&ctrl, &self.opts.overrides, 2, self.opts.irq_off_max_ns)?;
        let sampler = Sampler::spawn(id.to_string(), backend, rate, self.opts.sampler.clone(), self.tx.clone(), self.hub.clone())
            .context("starting sampler thread")?;
        Ok(Running { sampler, rate_hz: rate, _pm: pm })
    }

    fn publish_infos(&mut self) {
        // While down, infos keeps what the broker has, so Connected can clear what went away.
        if !self.mqtt.as_ref().is_some_and(Mqtt::is_connected) {
            return;
        }
        let Ok(v) = self.list() else { return };
        let mut seen = BTreeSet::new();
        for c in v["controllers"].as_array().into_iter().flatten() {
            let id = c["id"].as_str().unwrap_or("").to_string();
            let mut info = c.clone();
            let wants = self.wants.get(&id);
            info["wanted_by"] = json!(wants.map(|w| w.keys().collect::<Vec<_>>()).unwrap_or_default());
            info["wants"] = wants
                .map(|w| {
                    w.iter()
                        .map(|(cl, w)| {
                            let req = json!({"rate_hz": w.rate_hz, "model_hz": w.model_hz, "samples": w.samples,
                                             "expires_ms": w.expires.map(|(_, ms)| ms)});
                            (cl.clone(), req)
                        })
                        .collect::<serde_json::Map<_, _>>()
                })
                .unwrap_or_default()
                .into();
            if self.infos.get(&id) != Some(&info) {
                self.mqtt.as_ref().unwrap().publish_json(&format!("controllers/{id}/info"), &info, true);
                self.infos.insert(id.clone(), info);
            }
            seen.insert(id);
        }
        let removed: Vec<String> = self.infos.keys().filter(|k| !seen.contains(*k)).cloned().collect();
        for id in removed {
            self.infos.remove(&id);
            self.mqtt.as_ref().unwrap().publish(&format!("controllers/{id}/info"), Vec::new(), true);
        }
    }
}

fn mtime(p: &Path) -> Option<SystemTime> {
    std::fs::metadata(p).and_then(|m| m.modified()).ok()
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn frame_json(serial: &str, kind: Kind, f: &[u8]) -> Value {
    match kind {
        Kind::Nmea => json!({"event": "gnss_frame", "board": serial, "type": "nmea", "text": String::from_utf8_lossy(f)}),
        Kind::Ubx => json!({"event": "gnss_frame", "board": serial, "type": "ubx", "class": f[2], "id": f[3],
                            "payload": hex(&f[6..f.len() - 2])}),
        Kind::Rtcm3 => json!({"event": "gnss_frame", "board": serial, "type": "rtcm3", "data": hex(f)}),
    }
}

pub fn list(simulate: bool) -> Result<Value> {
    Ok(list_of(&discovery::devices(simulate)?, &discovery::controllers(simulate)?))
}

fn list_of(devices: &[UsbDevice], controllers: &[Controller]) -> Value {
    let ctrls: Vec<Value> = controllers
        .iter()
        .map(|c| {
            let mut v = serde_json::to_value(c).unwrap();
            v["devices"] = json!(devices.iter().filter(|d| d.controller == c.id).collect::<Vec<_>>());
            v
        })
        .collect();
    json!({ "controllers": ctrls })
}

pub fn lookup(q: &DeviceQuery, simulate: bool) -> Result<Value> {
    let dev = discovery::lookup(q, simulate)?;
    let ctrl = discovery::find_controller(&dev.controller, simulate)?;
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

/// `spec` is a tty (used as is), or "auto" for the board on `controller`.
pub fn board_serial(spec: &str, controller: &str, simulate: bool) -> Result<String> {
    let dev = if spec == "auto" {
        discovery::devices(simulate)?
            .into_iter()
            .find(|d| d.controller == controller && is_board(d))
            .ok_or_else(|| anyhow!("no sofgnss board on {controller}"))?
    } else {
        let d = discovery::device_of_tty(spec, simulate)?;
        if !is_board(&d) {
            bail!("{spec} is not a sofgnss board ({}:{} {:?} {:?})", d.vid, d.pid, d.manufacturer, d.product);
        }
        d
    };
    dev.serial.ok_or_else(|| anyhow!("USB device {} has no serial number", dev.port))
}
