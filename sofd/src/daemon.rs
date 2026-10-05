//! Main loop: runs sofgps boards, samples controllers, and publishes to MQTT, chrony and stdout.

use crate::backend;
use crate::chrony::{self, Chrony, Sent};
use crate::clock::{self, Ns};
use crate::config;
use crate::discovery::{self, BackendKind, DeviceQuery, MmioOverride, UsbDevice};
use crate::gnss::device::{self, Board, BoardInfo};
use crate::gnss::firmware::Image;
use crate::gnss::framer::Kind;
use crate::gnss::external::{self, LOST_AFTER_S};
use crate::gnss::timing::{Source, Tag};
use crate::gnss::{session, Feed, Hub};
use crate::mqtt::{self, Mqtt};
use crate::power;
use crate::sampler::{Sampler, SamplerConfig, MAX_RATE_HZ, MIN_RATE_HZ};
use anyhow::{anyhow, bail, Context, Result};
use log::{info, warn};
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

pub const BOARD_MANUFACTURER: &str = "sofgps";
const SCAN_EVERY: Duration = Duration::from_secs(1);
const RETRY_MIN: Duration = Duration::from_secs(1);
const RETRY_MAX: Duration = Duration::from_secs(30);
/// A controller that could not be opened is tried again after this long.
const SAMPLER_RETRY: Duration = Duration::from_secs(10);
/// Wait for a firmware file to stop changing before loading it.
const IMAGE_SETTLE: Duration = Duration::from_secs(2);
/// Failed updates to one image before giving up on a board.
const MAX_UPDATE_TRIES: u32 = 2;

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
}

/// An MQTT client acting as a controller's GNSS reference.
struct ExtRef {
    client: String,
    feed: Arc<Feed>,
    last: Instant,
    chrony_busy: bool,
}

struct ImageSlot {
    image: Option<Arc<Image>>,
    mtime: Option<SystemTime>,
    /// New mtime and when it was first seen.
    changed: Option<(Option<SystemTime>, Instant)>,
}

pub struct Daemon {
    opts: Options,
    gnss: config::Gnss,
    tx: Sender<Msg>,
    samplers: BTreeMap<String, Running>,
    /// Held while anything is sampled.
    power: Option<power::Held>,
    /// controller -> client -> rate
    wants: BTreeMap<String, BTreeMap<String, f64>>,
    start_failed: BTreeMap<String, (String, Instant)>,
    slots: BTreeMap<String, Slot>,
    hub: Hub,
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
    /// Last published `controllers/<id>/info`.
    infos: BTreeMap<String, Value>,
    pub sample_limit: Option<u64>,
    samples: u64,
}

fn backoff(b: Duration) -> Duration {
    (b * 2).clamp(RETRY_MIN, RETRY_MAX)
}

impl Daemon {
    pub fn new(opts: Options, gnss: config::Gnss, tx: Sender<Msg>, mqtt: Option<Mqtt>, stdout: bool) -> Self {
        Self {
            opts,
            gnss,
            tx,
            samplers: BTreeMap::new(),
            power: None,
            wants: BTreeMap::new(),
            start_failed: BTreeMap::new(),
            slots: BTreeMap::new(),
            hub: Hub::default(),
            refs: BTreeMap::new(),
            ref_told: BTreeMap::new(),
            images: BTreeMap::new(),
            update_tries: BTreeMap::new(),
            mqtt,
            chrony: Chrony::new(),
            stdout,
            infos: BTreeMap::new(),
            sample_limit: None,
            samples: 0,
        }
    }

    pub fn want(&mut self, controller: &str, client: &str, rate_hz: f64) {
        self.wants.entry(controller.into()).or_default().insert(client.into(), rate_hz);
    }

    pub fn run(mut self, rx: &Receiver<Msg>, stop: &AtomicBool) {
        let mut next_scan = Instant::now();
        while !stop.load(Ordering::Relaxed) && self.sample_limit.is_none_or(|l| self.samples < l) {
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
        for slot in self.slots.values_mut() {
            slot.board = None;
        }
        self.samplers.clear();
        self.power = None;
        if let Some(m) = self.mqtt.take() {
            m.shutdown();
        }
    }

    // ---- output helpers ----

    fn print(&self, v: &Value) {
        if self.stdout {
            println!("{v}");
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
            Msg::Controller { controller, event } => self.controller_event(&controller, &event),
            Msg::Board { serial, id, out } => {
                // Drop late messages from a replaced session.
                if self.slots.get(&serial).and_then(|s| s.board.as_ref()).is_some_and(|b| b.id == id) {
                    self.on_board(&serial, out);
                }
            }
            Msg::Mqtt(i) => self.on_mqtt(i),
        }
    }

    fn on_sample(&mut self, s: SampleOut) {
        self.samples += 1;
        if let Some(m) = &self.mqtt {
            let c = &s.controller;
            m.publish_json(&format!("controllers/{c}/sample"), &s.event, false);
            let mut model = Map::new();
            for k in ["epoch", "unit_ns", "counter", "model_raw", "model_realtime", "model_gnss", "gnss", "frame_start"] {
                model.insert(k.into(), s.event[k].clone());
            }
            m.publish_json(&format!("controllers/{c}/model"), &Value::Object(model), true);
        }
        self.print(&s.event);
        let (Some(name), Some(p)) = (s.board, s.chrony) else { return };
        let client = name.strip_prefix(external::NAME_PREFIX);
        let path = match client {
            Some(c) => self.gnss.external.get(c).and_then(|e| e.chrony.clone()),
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
                let mut printed = ev.clone();
                printed["event"] = json!("gnss_status");
                printed["board"] = json!(serial);
                self.print(&printed);
                self.set_status(serial, ev);
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
                // A board restarting into new firmware comes back at once.
                slot.backoff = if restarting { Duration::ZERO } else { backoff(slot.backoff) };
                slot.retry_at = Instant::now() + slot.backoff;
                self.chrony.release(serial);
                self.board_event(serial, json!({"event": "detached", "reason": reason}));
                self.set_status(serial, json!({"state": "detached", "message": reason}));
                self.reconcile();
            }
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
                self.infos.clear();
                self.publish_infos();
            }
            mqtt::In::Want { controller, client, rate_hz } => {
                match rate_hz {
                    Some(r) => {
                        let rate = r.unwrap_or(self.opts.default_rate_hz).clamp(MIN_RATE_HZ, MAX_RATE_HZ);
                        info!("{controller}: {client} wants {rate} Hz");
                        self.want(&controller, &client, rate);
                    }
                    None => {
                        info!("{controller}: {client} no longer wants samples");
                        if let Some(w) = self.wants.get_mut(&controller) {
                            w.remove(&client);
                            if w.is_empty() {
                                self.wants.remove(&controller);
                            }
                        }
                    }
                }
                self.start_failed.remove(&controller);
                self.reconcile();
                self.publish_infos();
            }
            mqtt::In::Ref { controller, client, payload, t_rx } => self.on_ref(&controller, &client, &payload, t_rx),
            mqtt::In::Tx { serial, data } => match self.slots.get(&serial).and_then(|s| s.board.as_ref()) {
                Some(b) => b.send(data),
                None => warn!("{serial}: tx for a board that is not attached"),
            },
            mqtt::In::Cmd { name, payload, reply_to } => {
                let result = self.command(&name, &payload);
                let reply = match result {
                    Ok(Value::Object(mut o)) => {
                        o.insert("ok".into(), json!(true));
                        Value::Object(o)
                    }
                    Ok(v) => json!({"ok": true, "result": v}),
                    Err(e) => json!({"ok": false, "error": format!("{e:#}")}),
                };
                match (reply_to, &self.mqtt) {
                    (Some((topic, corr)), Some(m)) => m.reply(&topic, corr, &reply),
                    _ => warn!("cmd/{name} without a response topic"),
                }
            }
        }
    }

    /// A board on the same controller takes precedence.
    fn on_ref(&mut self, controller: &str, client: &str, payload: &[u8], t_rx: Ns) {
        let Some(ext) = self.gnss.external.get(client).cloned() else {
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
                self.refs.insert(controller.into(), ExtRef { client: client.into(), feed: f.clone(), last: Instant::now(), chrony_busy: false });
                f
            }
        };
        self.ref_told.remove(&(controller.to_string(), client.to_string()));
        for t in tags {
            feed.push(Tag { at: t.at, utc_ns: t.t_gnss_ns - ext.sof_delay_ns, unc_ns: t.unc_ns, t_rx, source: Source::External });
        }
        if let Some(r) = self.refs.get_mut(controller) {
            r.last = Instant::now();
        }
    }

    /// Sent once until it changes.
    fn ref_problem(&mut self, controller: &str, client: &str, mut ev: Value) {
        let key = (controller.to_string(), client.to_string());
        let what = format!("{}/{}", ev["event"], ev["reason"]);
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
                        (r.last.elapsed() > Duration::from_secs(LOST_AFTER_S)).then(|| (c.clone(), None))
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
                    info!("{c}: GNSS reference {} lost (no tags for {LOST_AFTER_S} s)", r.client);
                    self.ref_told.remove(&(c.clone(), r.client.clone()));
                    self.controller_event(&c, &json!({"event": "ref_lost", "controller": c, "client": r.client}));
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
        let mut v = list(self.opts.simulate)?;
        for c in v["controllers"].as_array_mut().into_iter().flatten() {
            let id = c["id"].as_str().unwrap_or("").to_string();
            c["rate_hz"] = json!(self.samplers.get(&id).map(|r| r.rate_hz));
        }
        v["boards"] = json!(self.slots.iter().map(|(s, sl)| { let mut st = sl.status.clone(); st["board"] = json!(s); st }).collect::<Vec<_>>());
        Ok(v)
    }

    // ---- periodic work ----

    fn scan(&mut self) {
        let devices = match discovery::devices(self.opts.simulate) {
            Ok(d) => d,
            Err(e) => {
                warn!("listing USB devices: {e:#}");
                return;
            }
        };
        self.check_images();
        self.scan_boards(&devices);
        self.check_refs();
        self.reconcile();
        self.publish_infos();
    }

    fn scan_boards(&mut self, devices: &[UsbDevice]) {
        let found: BTreeMap<String, BoardInfo> = devices
            .iter()
            .filter(|d| d.manufacturer.as_deref() == Some(BOARD_MANUFACTURER))
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
                slot.update_given_up = false;
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
            if let Some(r) = w.values().copied().reduce(f64::max) {
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
            info!("{c}: stopped");
            if let Some(m) = &self.mqtt {
                m.publish(&format!("controllers/{c}/model"), Vec::new(), true);
            }
        }
        if desired.is_empty() {
            self.power = None;
        } else if self.power.is_none() {
            self.power = power::Held::apply(&self.opts.power);
        }
        let now = Instant::now();
        self.start_failed.retain(|c, (_, t)| desired.contains_key(c) && now < *t + SAMPLER_RETRY);
        for (c, rate) in desired {
            if let Some(r) = self.samplers.get_mut(&c) {
                if r.rate_hz != rate {
                    r.sampler.set_rate(rate);
                    r.rate_hz = rate;
                    info!("{c}: sampling at {rate} Hz");
                }
                continue;
            }
            if self.start_failed.contains_key(&c) {
                continue;
            }
            match self.start_sampler(&c, rate) {
                Ok(r) => {
                    info!("{c}: sampling at {rate} Hz");
                    self.samplers.insert(c, r);
                }
                Err(e) => {
                    let msg = format!("{e:#}");
                    warn!("{c}: {msg}");
                    let ev = json!({"event": "error", "controller": c, "message": msg});
                    if let Some(m) = &self.mqtt {
                        m.publish_event(&format!("controllers/{c}/event"), &ev);
                    }
                    self.print(&ev);
                    self.start_failed.insert(c, (msg, now));
                }
            }
        }
    }

    fn start_sampler(&self, id: &str, rate: f64) -> Result<Running> {
        let mut ctrl = discovery::find_controller(id, self.opts.simulate)?;
        if self.opts.force_generic.iter().any(|g| g == id) {
            ctrl.backend = Some(BackendKind::KernelFrameNumber);
        }
        let backend = backend::open(&ctrl, &self.opts.overrides, 2, self.opts.irq_off_max_ns)?;
        let pm = PmGuard::hold(&ctrl);
        let sampler = Sampler::spawn(id.to_string(), backend, rate, self.opts.sampler.clone(), self.tx.clone(), self.hub.clone())
            .context("starting sampler thread")?;
        Ok(Running { sampler, rate_hz: rate, _pm: pm })
    }

    fn publish_infos(&mut self) {
        if self.mqtt.is_none() {
            return;
        }
        let Ok(v) = self.list() else { return };
        let mut seen = BTreeSet::new();
        for c in v["controllers"].as_array().into_iter().flatten() {
            let id = c["id"].as_str().unwrap_or("").to_string();
            let mut info = c.clone();
            info["wanted_by"] = json!(self.wants.get(&id).map(|w| w.keys().collect::<Vec<_>>()).unwrap_or_default());
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
    let devices = discovery::devices(simulate)?;
    let ctrls: Vec<Value> = discovery::controllers(simulate)?
        .into_iter()
        .map(|c| {
            let mut v = serde_json::to_value(&c).unwrap();
            v["devices"] = json!(devices.iter().filter(|d| d.controller == c.id).collect::<Vec<_>>());
            v
        })
        .collect();
    Ok(json!({ "controllers": ctrls }))
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

/// `spec` is a tty, or "auto" for the board on `controller`.
pub fn board_serial(spec: &str, controller: &str, simulate: bool) -> Result<String> {
    let dev = if spec == "auto" {
        discovery::devices(simulate)?
            .into_iter()
            .find(|d| d.controller == controller && d.manufacturer.as_deref() == Some(BOARD_MANUFACTURER))
            .ok_or_else(|| anyhow!("no sofgps board on {controller}"))?
    } else {
        discovery::device_of_tty(spec, simulate)?
    };
    dev.serial.ok_or_else(|| anyhow!("USB device {} has no serial number", dev.port))
}
