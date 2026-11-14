//! State machine for one sofgnss device, without I/O: handshake, firmware update, receiver setup,
//! then labelling pulses from TIM-TP, or from NMEA ZDA if the receiver doesn't answer UBX.

use super::firmware::Image;
use super::fix;
use super::framer::{Framer, Kind};
use super::nmea::{self, OtaMsg, Pvdb};
use super::timing::{Labeler, PpsDecision, Source, Tag};
use super::ubx::{self, Version};
use crate::clock::Ns;
use serde_json::{json, Value};
use std::sync::Arc;

const MS: Ns = 1_000_000;
const PING_EVERY: Ns = 10_000 * MS;
/// VER platform of the STM32 bootloader.
pub const BOOTLOADER: &str = "stm32g4-boot";
const HANDSHAKE_RETRY: Ns = 1_000 * MS;
const HANDSHAKE_WARN: Ns = 5_000 * MS;
/// Frames this soon after a baud change may be from before it.
const SETTLE: Ns = 250 * MS;
/// The device only forwards valid frames, so a wrong baud is just silence.
const PROBE_WAIT: Ns = 1_500 * MS;
/// Every rate the tty supports, in search order.
pub const SEARCH_BAUDS: [u32; 9] = [9600, 38_400, 115_200, 4800, 19_200, 57_600, 230_400, 460_800, 921_600];
const POLL_WAIT: Ns = 1_000 * MS;
const POLL_TRIES: u32 = 3;
const ACK_TRIES: u32 = 3;
/// Let the last message at the old baud leave the device's UART.
const SWITCH_WAIT: Ns = 200 * MS;
const VERIFY_WAIT: Ns = 3_500 * MS;
const FRAME_TIMEOUT: Ns = 5_000 * MS;
/// TIM-TP only comes with a fix. Missing this long with one, set the receiver up again.
const TIM_TP_TIMEOUT: Ns = 10_000 * MS;
/// Failed setups in a row before falling back to ZDA.
const MAX_FAILURES: u32 = 3;
/// Use TIM-TP while one arrived this recently, else ZDA.
const PASSIVE_TP_WINDOW: Ns = 3_000 * MS;
/// The receiver counts as working while a pulse was accepted this recently.
const LOCK_TIMEOUT: Ns = 5_000 * MS;
/// Serial timing: a read after this much silence starts the output of a new epoch.
const BURST_GAP: Ns = 100 * MS;
/// An epoch's time this long after its output began: the line never went quiet.
const BURST_MAX: Ns = 900 * MS;
/// Allowed change of the receiver's output delay from one epoch to the next.
const EPOCH_TOL: Ns = 20 * MS;
const SECOND: Ns = 1_000 * MS;
/// From the bus frame start to the board's SOF capture, unless configured. The STM32
/// captures after the SOF packet's start: 1.12 us behind a Pi 4's hub, 1.44 to 1.52 us
/// on a root port (DWC2, BCM2711 xHCI), measured against the GPS PPS polled on a GPIO.
pub fn default_sof_delay(platform: &str) -> Ns {
    if platform.starts_with("stm32") { 1300 } else { 0 }
}

/// Plain mode pulse ticks are ns.
pub const PLAIN_TICK_HZ: u32 = 1_000_000_000;
const VER_WAIT: Ns = 1_000 * MS;
const VER_TRIES: u32 = 3;
/// The device erases the update slot before READY.
const OTA_READY_WAIT: Ns = 15_000 * MS;
/// Max wait for `OTA,ACK` (sent every 32 KB) or `OTA,OK`.
const OTA_PROGRESS_WAIT: Ns = 10_000 * MS;
const OTA_RESTART_WAIT: Ns = 10_000 * MS;
/// The device drops an update 5 s after its last byte. Until then it takes ours as image data.
const OTA_ABORT_WAIT: Ns = 6_000 * MS;
const OTA_CHUNK: usize = 4096;
/// Max bytes in flight past the last ACK.
const OTA_WINDOW: usize = 64 * 1024;

#[derive(Debug, Clone)]
pub struct Config {
    /// Target receiver baud.
    pub baud: u32,
    /// Receiver baud before setup; also the tty baud without `configure`.
    pub initial_baud: u32,
    /// false: use the receiver stream as is.
    pub configure: bool,
    /// Keep retrying the setup instead of falling back to NMEA ZDA.
    pub strict: bool,
    /// GPS minus UTC, used until the receiver knows it.
    pub leap_seconds: Option<i8>,
    /// No sofgnss board: the port is the receiver's own, pulses come in through `on_pulse`.
    pub plain: bool,
    /// Plain mode: pulses come in at all.
    pub pps: bool,
    /// Plain mode: the port is the receiver's own USB port.
    pub usb: bool,
    /// Plain mode without pulses: date each epoch by the start of its output, this much
    /// after the epoch.
    pub serial_delay_ns: Option<Ns>,
    /// The GNSS to track, all others off. u-blox 9 and later only.
    pub constellations: Option<Vec<ubx::Constellation>>,
    /// Emit every GPS frame.
    pub frames: bool,
    /// Emit a fix per navigation epoch.
    pub fix: bool,
    pub div: Option<u32>,
    /// From bus frame start to the device's SOF capture.
    /// None: the board platform's default (`default_sof_delay`).
    pub sof_delay_ns: Option<Ns>,
    /// Sent if VER reports a different image.
    pub firmware: Option<Arc<Image>>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            baud: 38_400,
            initial_baud: 9600,
            configure: true,
            strict: false,
            leap_seconds: None,
            plain: false,
            pps: true,
            usb: false,
            serial_delay_ns: None,
            constellations: None,
            frames: true,
            fix: true,
            div: None,
            sof_delay_ns: None,
            firmware: None,
        }
    }
}

pub trait Port {
    fn write(&mut self, data: &[u8]);
    fn set_baud(&mut self, baud: u32);
}

#[derive(Default)]
pub struct Output {
    pub events: Vec<Value>,
    pub tags: Vec<Tag>,
    /// Only with `frames`.
    pub frames: Vec<(Kind, Vec<u8>)>,
}

/// From the device's VER reply.
#[derive(Debug, Clone, serde::Serialize)]
pub struct DeviceFirmware {
    pub version: String,
    /// The ESP-IDF version on ESP32 boards, "stm32g4" on STM32 boards, "stm32g4-boot"
    /// in the STM32 bootloader.
    pub platform: String,
    /// The OTA slot on ESP32 boards. In the bootloader the app's state: valid, invalid or empty.
    pub partition: String,
    pub elf_sha256: String,
    pub reset_reason: String,
}

struct Step {
    frame: Vec<u8>,
    /// None for the baud change: its ACK may be lost.
    ack: Option<ubx::Id>,
    /// Diagnostics: skipped if rejected or unanswered.
    optional: Option<String>,
    /// Wait POLL_WAIT after its ACK: a signal change restarts the receiver's GNSS.
    pause: bool,
}

impl Step {
    fn required(frame: Vec<u8>, ack: Option<ubx::Id>) -> Self {
        Self { frame, ack, optional: None, pause: false }
    }

    fn optional(frame: Vec<u8>, ack: ubx::Id, what: String) -> Self {
        Self { frame, ack: Some(ack), optional: Some(what), pause: false }
    }
}

fn ubx_name(id: ubx::Id) -> String {
    match id {
        ubx::NAV_PVT => "NAV-PVT".into(),
        ubx::NAV_SOL => "NAV-SOL".into(),
        ubx::NAV_POSLLH => "NAV-POSLLH".into(),
        ubx::NAV_VELNED => "NAV-VELNED".into(),
        ubx::NAV_TIMEUTC => "NAV-TIMEUTC".into(),
        ubx::NAV_SAT => "NAV-SAT".into(),
        ubx::NAV_SVINFO => "NAV-SVINFO".into(),
        ubx::MON_SPAN => "MON-SPAN".into(),
        ubx::MON_HW => "MON-HW".into(),
        (c, i) => format!("UBX {c:02x}/{i:02x}"),
    }
}

enum Rx {
    Idle,
    Version { since: Ns, tries: u32 },
    OtaStart { since: Ns },
    OtaSend { sent: usize, acked: usize, since: Ns, reported: usize },
    OtaDone { since: Ns },
    /// Quiet until the device gives up on the image.
    OtaAbort { since: Ns },
    Probe { idx: usize, since: Ns },
    Identify { since: Ns, tries: u32 },
    PortPoll { since: Ns, tries: u32 },
    /// `sends` 0: waiting before the next step, for late ACKs of a retried step (all
    /// CFG-MSG steps get the same ACK, so a late one must not count for the next) or
    /// for a GNSS restart.
    Configure { steps: Vec<Step>, since: Ns, tries: u32, sends: u32 },
    Switch { since: Ns },
    Verify { since: Ns },
    Running { last_frame: Ns, last_tp: Ns },
    /// `last_label`: last ZDA or TIM-TP.
    Fallback { last_label: Ns, last_tp: Option<Ns> },
    Passive { last_tp: Option<Ns> },
}

pub struct Session {
    cfg: Config,
    nonce: String,
    ready: bool,
    started: Ns,
    last_handshake: Option<Ns>,
    warned: bool,
    last_ping: Option<Ns>,
    framer: Framer,
    labeler: Option<Labeler>,
    tick_hz: u32,
    rx: Rx,
    baud: u32,
    failures: u32,
    receiver: Option<Version>,
    polled_port: Option<Vec<u8>>,
    firmware: Option<DeviceFirmware>,
    state: &'static str,
    message: Option<String>,
    /// Until NAV-STATUS arrives, the NMEA fix is used.
    last_nav_status: Option<Ns>,
    /// Since when NAV-STATUS reports a fix.
    fix_since: Option<Ns>,
    leap_warned: bool,
    last_accepted: Option<Ns>,
    /// A pulse was accepted within LOCK_TIMEOUT. Only then can the board be a reference.
    locked: bool,
    /// Serial timing: the read that began the current output, and the silence before it (ms).
    burst: Option<(Ns, u32)>,
    last_read: Option<Ns>,
    /// The current output's epoch is handled.
    epoch_seen: bool,
    /// Output start and UTC of the last epoch.
    prev_epoch: Option<(Ns, Ns)>,
    fixes: fix::Assembler,
}

impl Session {
    pub fn new(cfg: Config, nonce: String, now: Ns) -> Self {
        let baud = cfg.baud;
        let plain = cfg.plain;
        Self {
            cfg,
            nonce,
            ready: plain,
            started: now,
            last_handshake: None,
            warned: false,
            last_ping: None,
            framer: Framer::new(),
            labeler: plain.then(|| Labeler::new(PLAIN_TICK_HZ, Source::TimTp)),
            tick_hz: 0,
            rx: Rx::Idle,
            baud,
            failures: 0,
            receiver: None,
            polled_port: None,
            firmware: None,
            state: "handshake",
            message: None,
            last_nav_status: None,
            fix_since: None,
            leap_warned: false,
            last_accepted: None,
            locked: false,
            burst: None,
            last_read: None,
            epoch_seen: false,
            prev_epoch: None,
            fixes: fix::Assembler::default(),
        }
    }

    #[cfg(test)]
    pub fn state(&self) -> &'static str {
        self.state
    }

    /// The configured delay, else the platform's once VER told it.
    fn sof_delay(&self) -> Ns {
        let platform = self.firmware.as_ref().map_or("", |f| f.platform.as_str());
        self.cfg.sof_delay_ns.unwrap_or_else(|| default_sof_delay(platform))
    }

    fn status(&mut self, state: &'static str, message: Option<String>, out: &mut Output) {
        self.state = state;
        self.message = message.clone();
        let mut ev = json!({"event": "gnss_status", "state": state, "baud": self.baud, "locked": self.locked});
        if !self.cfg.plain {
            ev["sof_delay_ns"] = json!(self.sof_delay());
        }
        if let Some(d) = self.cfg.serial_delay_ns {
            ev["serial_delay_ns"] = json!(d);
        }
        if let Some(r) = &self.receiver {
            ev["receiver"] = json!(r);
        }
        if let Some(f) = &self.firmware {
            ev["firmware"] = json!(f);
        }
        if let Some(m) = message {
            ev["message"] = json!(m);
        }
        out.events.push(ev);
    }

    fn set_source(&mut self, s: Source) {
        if let Some(l) = &mut self.labeler {
            l.set_source(s);
        }
    }

    fn set_baud(&mut self, baud: u32, port: &mut dyn Port) {
        port.set_baud(baud);
        self.baud = baud;
        self.framer.reset();
        if let Some(l) = &mut self.labeler {
            l.reset();
        }
        (self.last_accepted, self.locked, self.fix_since) = (None, false, None);
        (self.burst, self.prev_epoch) = (None, None);
        self.fixes.reset();
    }

    fn on_fix(&mut self, f: Option<fix::Fix>, out: &mut Output) {
        if let Some(f) = f.filter(|_| self.cfg.fix) {
            let mut ev = serde_json::to_value(&f).expect("fix serializes");
            ev["event"] = json!("gnss_fix");
            out.events.push(ev);
        }
    }

    fn update_lock(&mut self, now: Ns, out: &mut Output) {
        let locked = self.last_accepted.is_some_and(|t| now - t < LOCK_TIMEOUT);
        if locked != self.locked {
            self.locked = locked;
            let (state, message) = (self.state, self.message.clone());
            self.status(state, message, out);
        }
    }

    pub fn on_bytes(&mut self, data: &[u8], now: Ns, port: &mut dyn Port, out: &mut Output) {
        if self.cfg.serial_delay_ns.is_some() && !data.is_empty() {
            let quiet = self.last_read.map_or(Ns::MAX, |t| now - t);
            if quiet >= BURST_GAP {
                self.burst = Some((now, (quiet / MS).min(u32::MAX as Ns) as u32));
                self.epoch_seen = false;
            }
            self.last_read = Some(now);
        }
        let mut frames = vec![];
        self.framer.feed(data, |k, f| frames.push((k, f.to_vec())));
        for (kind, f) in frames {
            if kind == Kind::Nmea && f.starts_with(nmea::PVDB_PREFIX) && !self.cfg.plain {
                self.on_pvdb(&f, now, port, out);
            } else if self.ready {
                self.on_gps(kind, &f, now, port, out);
            }
        }
    }

    /// Firmware written, or START or BOOT sent: the device is about to restart.
    pub fn restarting(&self) -> bool {
        matches!(self.rx, Rx::OtaDone { .. })
    }

    /// False during a firmware transfer.
    pub fn accepts_tx(&self) -> bool {
        !matches!(self.rx, Rx::OtaStart { .. } | Rx::OtaSend { .. } | Rx::OtaAbort { .. })
    }

    pub fn on_tick(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let f = self.fixes.on_tick(now);
        self.on_fix(f, out);
        if self.cfg.plain {
            if let Rx::Idle = self.rx {
                self.after_version(now, port, out);
            }
            self.rx_tick(now, port, out);
            return self.update_lock(now, out);
        }
        if self.accepts_tx() && self.last_ping.is_none_or(|t| now - t >= PING_EVERY) {
            port.write(&nmea::command("PING"));
            self.last_ping = Some(now);
        }
        if !self.ready {
            // Waiting for our nonce skips anything the device queued before the port opened.
            if self.last_handshake.is_none_or(|t| now - t >= HANDSHAKE_RETRY) {
                port.write(&nmea::command(&format!("INFO,{}", self.nonce)));
                self.last_handshake = Some(now);
            }
            if !self.warned && now - self.started >= HANDSHAKE_WARN {
                self.warned = true;
                self.status("handshake", Some("no reply from the sofgnss device".into()), out);
            }
            return;
        }
        self.rx_tick(now, port, out);
        self.update_lock(now, out);
    }

    fn on_pvdb(&mut self, f: &[u8], now: Ns, port: &mut dyn Port, out: &mut Output) {
        match nmea::pvdb(f) {
            Pvdb::Info { tick_hz, tag } => {
                self.tick_hz = tick_hz;
                if let Some(l) = &mut self.labeler {
                    l.set_tick_hz(tick_hz);
                }
                if !self.ready && tag == self.nonce {
                    self.ready = true;
                    self.labeler = Some(Labeler::new(tick_hz, Source::TimTp));
                    port.write(&nmea::command("VER"));
                    if let Some(d) = self.cfg.div {
                        port.write(&nmea::command(&format!("DIV,{d}")));
                    }
                    self.rx = Rx::Version { since: now, tries: 1 };
                }
            }
            _ if !self.ready => {}
            Pvdb::Ver { version, platform, partition, elf_sha256, reset_reason } => {
                self.firmware = Some(DeviceFirmware {
                    version: version.into(),
                    platform: platform.into(),
                    partition: partition.into(),
                    elf_sha256: elf_sha256.into(),
                    reset_reason: reset_reason.into(),
                });
                if let Rx::Version { .. } = self.rx {
                    self.after_version(now, port, out);
                }
            }
            Pvdb::Ota(m) => self.on_ota(m, now, port, out),
            Pvdb::Err(code) if code.starts_with("OTA") && matches!(self.rx, Rx::OtaStart { .. } | Rx::OtaSend { .. }) => {
                self.ota_failed(&format!("device error {code}"), now, port, out);
            }
            Pvdb::Pps { tick, rx_idle_ms } => self.on_pulse(tick, rx_idle_ms, now, out),
            Pvdb::Cal { tick, mode, synced, n, chip_rate, length } => {
                let utc_ns = self.labeler.as_ref().and_then(|l| l.utc_of(tick));
                out.events.push(json!({"event": "gnss_cal", "tick": tick, "mode": mode, "pps_synced": synced, "n": n,
                    "chip_rate": chip_rate, "length_ticks": length, "utc_ns": utc_ns}));
            }
            Pvdb::Sof { tick, frame } => {
                if let Some(l) = &mut self.labeler {
                    l.on_sof(tick, frame, now);
                }
            }
            Pvdb::Err(code @ ("NOAPP" | "BADAPP")) if matches!(self.rx, Rx::OtaDone { .. }) => {
                self.rx = Rx::Idle;
                self.status("no_app", Some(format!("bootloader did not start the app ({code})")), out);
            }
            Pvdb::Err(code) => {
                let state = self.state;
                self.status(state, Some(format!("device error {code}")), out);
            }
            Pvdb::Other => {}
        }
    }

    /// A PPS edge at `tick`, after the GPS line was quiet for `rx_idle_ms`.
    /// In plain mode ticks are ns of CLOCK_MONOTONIC_RAW.
    pub fn on_pulse(&mut self, tick: u64, rx_idle_ms: u32, now: Ns, out: &mut Output) {
        if let Some(l) = &mut self.labeler {
            let mut decisions: Vec<PpsDecision> = vec![];
            let mut tags = vec![];
            l.on_pps(tick, rx_idle_ms, &mut decisions, &mut tags);
            for d in decisions {
                if d.accepted {
                    self.last_accepted = Some(now);
                }
                let mut ev = serde_json::to_value(&d).expect("decision serializes");
                ev["event"] = json!("gnss_pps");
                out.events.push(ev);
            }
            for mut t in tags {
                t.utc_ns -= self.sof_delay();
                out.tags.push(t);
            }
        }
        self.update_lock(now, out);
    }

    fn on_gps(&mut self, kind: Kind, f: &[u8], now: Ns, port: &mut dyn Port, out: &mut Output) {
        let msg = (kind == Kind::Ubx).then(|| ubx::split(f));
        if self.cfg.frames {
            out.frames.push((kind, f.to_vec()));
        }
        let fixed = match (kind, msg) {
            (Kind::Ubx, Some((id, p))) => self.fixes.on_ubx(id, p, now),
            (Kind::Nmea, _) => self.fixes.on_nmea(&nmea::fields(f), now),
            _ => None,
        };
        self.on_fix(fixed, out);
        let mut tim_tp = false;
        let mut zda = false;
        let mut leap_differs = None;
        // Serial timing: the UTC of the epoch this output belongs to.
        let mut epoch = None;
        // Without pulses, labels would pile up waiting for one.
        let pulses = self.cfg.pps;
        if let Some(l) = &mut self.labeler {
            match (kind, msg) {
                (Kind::Ubx, Some((ubx::TIM_TP, p))) => {
                    if let Some(tp) = ubx::TimTp::parse(p) {
                        tim_tp = true;
                        if pulses {
                            l.on_tim_tp(&tp);
                        }
                    }
                }
                (Kind::Ubx, Some((ubx::NAV_TIMEGPS, p))) => {
                    if let Some(leap) = ubx::leap_seconds(p) {
                        let cfg = self.cfg.leap_seconds;
                        leap_differs = Some(leap.zip(cfg).filter(|(r, c)| r != c));
                        l.on_leap(leap.or(cfg));
                    }
                    epoch = Some(ubx::timegps_utc(p, self.cfg.leap_seconds));
                }
                (Kind::Ubx, Some((ubx::NAV_STATUS, p))) => {
                    if let Some(ok) = ubx::fix_ok(p) {
                        l.on_fix(ok);
                        self.last_nav_status = Some(now);
                        self.fix_since = if ok { self.fix_since.or(Some(now)) } else { None };
                    }
                }
                (Kind::Nmea, _) => match nmea::parse(f) {
                    nmea::Info::Zda(label) => {
                        zda = true;
                        epoch = Some(label);
                        if pulses {
                            l.on_zda(label);
                        }
                    }
                    // NAV-STATUS wins while it arrives; otherwise use the NMEA fix.
                    nmea::Info::Fix(ok)
                        if l.source() == Source::Zda || self.last_nav_status.is_none_or(|t| now - t >= PASSIVE_TP_WINDOW) =>
                    {
                        l.on_fix(ok)
                    }
                    _ => {}
                },
                _ => {}
            }
        }
        match leap_differs {
            Some(Some((r, c))) if !self.leap_warned => {
                self.leap_warned = true;
                let state = self.state;
                self.status(state, Some(format!("receiver reports {r} leap seconds, the config says {c}")), out);
            }
            Some(None) => self.leap_warned = false,
            _ => {}
        }
        self.rx_frame(kind, msg, tim_tp, zda, now, port, out);
        if let Some(e) = epoch {
            self.on_epoch(e, now, out);
        }
    }

    /// Serial timing: the epoch at `utc` began the output that started at the last burst.
    fn on_epoch(&mut self, utc: Result<Ns, &'static str>, now: Ns, out: &mut Output) {
        let (Some(delay), Some((start, quiet_ms))) = (self.cfg.serial_delay_ns, self.burst) else { return };
        if !matches!(self.rx, Rx::Running { .. } | Rx::Fallback { .. } | Rx::Passive { .. }) {
            return;
        }
        // One epoch per output: NAV-TIMEGPS and ZDA tell the same.
        if std::mem::replace(&mut self.epoch_seen, true) {
            return;
        }
        let fix = self.labeler.as_ref().is_some_and(Labeler::has_fix);
        let prev = self.prev_epoch.take();
        let reason = match utc {
            _ if now - start > BURST_MAX => Some("line_busy"),
            Err(e) => Some(e),
            Ok(_) if !fix => Some("no_fix"),
            Ok(u) => match prev {
                None => Some("first_epoch"),
                Some((_, pu)) if u <= pu || u - pu > 2 * SECOND => Some("label_jump"),
                Some((ps, pu)) if ((start - ps) - (u - pu)).abs() > EPOCH_TOL => Some("interval"),
                Some(_) => None,
            },
        };
        if reason != Some("line_busy") {
            self.prev_epoch = utc.ok().map(|u| (start, u));
        }
        if reason.is_none() {
            self.last_accepted = Some(now);
        }
        let mut ev = json!({"event": "gnss_pps", "source": "serial", "tick": start - delay, "rx_idle_ms": quiet_ms,
            "utc_ns": utc.ok(), "accepted": reason.is_none()});
        if let Some(r) = reason {
            ev["reason"] = json!(r);
        }
        out.events.push(ev);
        self.update_lock(now, out);
    }

    /// Called on VER, or after giving up on it.
    fn after_version(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let fw = self.firmware.clone();
        let boot = fw.as_ref().is_some_and(|f| f.platform == BOOTLOADER);
        // The STM32 bootloader starts the app only when asked. It checks the app again first.
        let start = boot && fw.as_ref().is_some_and(|f| f.partition == "valid");
        if let (Some(img), Some(f)) = (self.cfg.firmware.clone(), &fw) {
            let stm32 = f.platform.starts_with("stm32");
            let app_differs = f.elf_sha256 != img.elf_sha256 || (boot && !start);
            if stm32 != img.stm32 {
                let kind = if img.stm32 { "an STM32" } else { "an ESP32" };
                self.status("update_failed", Some(format!("firmware is {kind} image, the board is {}", f.platform)), out);
            } else if app_differs && stm32 && !boot {
                // Updates go through the bootloader.
                port.write(&nmea::command("BOOT"));
                self.rx = Rx::OtaDone { since: now };
                self.status("restarting", Some(format!("into the bootloader for firmware {}", img.version)), out);
                return;
            } else if app_differs {
                port.write(&nmea::command(&format!("OTA,{},{}", img.data.len(), img.sha256)));
                self.rx = Rx::OtaStart { since: now };
                let from = if boot && !start { "none" } else { f.version.as_str() };
                self.status("updating", Some(format!("firmware {from} -> {}", img.version)), out);
                return;
            }
        }
        if start {
            port.write(&nmea::command("START"));
            self.rx = Rx::OtaDone { since: now };
            self.status("restarting", Some("starting the app".into()), out);
            return;
        }
        if boot {
            self.rx = Rx::Idle;
            self.status("no_app", Some("bootloader has no valid app and no firmware to install".into()), out);
            return;
        }
        if self.cfg.configure {
            self.start_probe(now, port, out);
        } else {
            self.set_baud(self.cfg.initial_baud, port);
            self.rx = Rx::Passive { last_tp: None };
            self.status("passive", None, out);
        }
    }

    fn on_ota(&mut self, m: OtaMsg, now: Ns, port: &mut dyn Port, out: &mut Output) {
        match (&mut self.rx, m) {
            (Rx::OtaStart { .. }, OtaMsg::Ready) => {
                self.rx = Rx::OtaSend { sent: 0, acked: 0, since: now, reported: 0 };
            }
            (Rx::OtaSend { acked, since, .. }, OtaMsg::Ack(n)) => {
                *acked = n as usize;
                *since = now;
            }
            (Rx::OtaSend { .. }, OtaMsg::Ok) => {
                if self.firmware.as_ref().is_some_and(|f| f.platform == BOOTLOADER) {
                    port.write(&nmea::command("START"));
                }
                self.rx = Rx::OtaDone { since: now };
                self.status("restarting", Some("firmware written, device restarting".into()), out);
            }
            (Rx::OtaAbort { since }, OtaMsg::Ready) => *since = now,
            _ => {}
        }
    }

    fn ota_failed(&mut self, why: &str, now: Ns, port: &mut dyn Port, out: &mut Output) {
        self.status("update_failed", Some(why.to_string()), out);
        self.cfg.firmware = None;
        if matches!(self.rx, Rx::OtaStart { .. } | Rx::OtaSend { .. }) {
            self.rx = Rx::OtaAbort { since: now };
        } else {
            self.after_version(now, port, out);
        }
    }

    fn ota_tick(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let Some(img) = self.cfg.firmware.clone() else { return };
        let len = img.data.len();
        let mut progress = None;
        let failed = match &mut self.rx {
            Rx::OtaStart { since } => (now - *since >= OTA_READY_WAIT).then_some("no OTA,READY from the device"),
            Rx::OtaSend { sent, acked, since, reported } => {
                while *sent < len && *sent < *acked + OTA_WINDOW {
                    let end = (*sent + OTA_CHUNK).min(len);
                    port.write(&img.data[*sent..end]);
                    *sent = end;
                }
                let pct = *acked * 100 / len;
                if pct >= *reported + 25 {
                    *reported = pct - pct % 25;
                    progress = Some(pct);
                }
                (now - *since >= OTA_PROGRESS_WAIT).then_some("firmware transfer stalled")
            }
            Rx::OtaDone { since } => (now - *since >= OTA_RESTART_WAIT).then_some("device did not restart after the update"),
            _ => None,
        };
        if let Some(pct) = progress {
            self.status("updating", Some(format!("{pct}%")), out);
        }
        if let Some(why) = failed {
            self.ota_failed(why, now, port, out);
        }
    }

    fn start_probe(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        self.set_source(Source::TimTp);
        self.enter_probe(0, now, port);
        let msg = (self.failures > 0).then(|| format!("restarting receiver setup (failure {})", self.failures));
        self.status("probing", msg, out);
    }

    /// Target baud first: the receiver may already be set up. Then its
    /// power-on baud, then the other standard rates.
    fn bauds(&self) -> Vec<u32> {
        let mut b = vec![self.cfg.baud];
        for r in [self.cfg.initial_baud].into_iter().chain(SEARCH_BAUDS) {
            if !b.contains(&r) {
                b.push(r);
            }
        }
        b
    }

    fn enter_probe(&mut self, idx: usize, now: Ns, port: &mut dyn Port) {
        let baud = self.bauds()[idx];
        self.set_baud(baud, port);
        port.write(&ubx::frame(ubx::MON_VER, &[]));
        self.rx = Rx::Probe { idx, since: now };
    }

    fn fail(&mut self, why: &str, now: Ns, port: &mut dyn Port, out: &mut Output) {
        self.failures += 1;
        if self.failures >= MAX_FAILURES && !self.cfg.strict {
            self.fallback(format!("{why}; giving up on UBX setup, labelling pulses from NMEA ZDA"), now, out);
        } else {
            let state = self.state;
            self.status(state, Some(why.to_string()), out);
            self.start_probe(now, port, out);
        }
    }

    fn fallback(&mut self, message: String, now: Ns, out: &mut Output) {
        self.set_source(Source::Zda);
        self.rx = Rx::Fallback { last_label: now, last_tp: None };
        self.status("nmea_fallback", Some(message), out);
    }

    fn configure(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let Some(ver) = &self.receiver else {
            return self.fail("receiver not identified", now, port, out);
        };
        let target = self.cfg.baud;
        let diag = ubx::diagnostics(ver.generation());
        let steps = if ver.uses_valset() {
            let p = self.polled_port.as_ref().map_or(ubx::PORT_UART1, |p| p[0]);
            let p = match p {
                _ if self.cfg.usb => ubx::PORT_USB,
                ubx::PORT_UART2 | ubx::PORT_USB => p,
                _ => ubx::PORT_UART1,
            };
            let mut items: Vec<(u32, u32)> = ubx::ENABLED.iter().map(|&id| (ubx::key::msgout(id, p), 1)).collect();
            items.push((ubx::key::out_ubx(p), 1));
            items.push((ubx::key::out_nmea(p), 0));
            let diag_items: Vec<(u32, u32)> = diag.iter().map(|&(id, r)| (ubx::key::msgout(id, p), u32::from(r))).collect();
            let names: Vec<String> = diag.iter().map(|&(id, _)| ubx_name(id)).collect();
            // First: a signal change restarts the receiver's GNSS.
            let gnss = self.cfg.constellations.as_ref().map(|c| {
                let items: Vec<(u32, u32)> = ubx::Constellation::ALL.iter().map(|g| (g.signal_key(), c.contains(g) as u32)).collect();
                Step { pause: true, ..Step::optional(ubx::valset_ram(&items), ubx::CFG_VALSET, "constellations".into()) }
            });
            // USB has no baud, and the UART's is not ours to change.
            let baud = (p != ubx::PORT_USB).then(|| Step::required(ubx::valset_ram(&[(ubx::key::uart_baud(p), target)]), None));
            gnss.into_iter()
                .chain([
                    Step::required(ubx::valset_ram(&items), Some(ubx::CFG_VALSET)),
                    Step::optional(ubx::valset_ram(&diag_items), ubx::CFG_VALSET, names.join(", ")),
                ])
                .chain(baud)
                .collect()
        } else {
            let mut s: Vec<Step> = ubx::ENABLED.iter().map(|&id| Step::required(ubx::cfg_msg(id, 1), Some(ubx::CFG_MSG))).collect();
            s.extend(diag.iter().map(|&(id, r)| Step::optional(ubx::cfg_msg(id, r), ubx::CFG_MSG, ubx_name(id))));
            s.push(Step::required(ubx::cfg_prt(self.polled_port.as_deref(), target), None));
            s
        };
        let ignored = (self.cfg.constellations.is_some() && !ver.uses_valset())
            .then(|| "constellations need u-blox 9 or later; left as they are".to_string());
        self.rx = Rx::Configure { steps, since: now, tries: 0, sends: 0 };
        self.status("configuring", ignored, out);
        self.send_step(now, port);
    }

    fn skip_step(&mut self, why: &str, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let Rx::Configure { steps, .. } = &self.rx else { return };
        let what = steps[0].optional.clone().unwrap_or_default();
        self.status("configuring", Some(format!("receiver {why} {what}; continuing without")), out);
        self.next_step(now, port);
    }

    fn next_step(&mut self, now: Ns, port: &mut dyn Port) {
        let Rx::Configure { steps, since, tries, sends } = &mut self.rx else { return };
        let pause = steps.remove(0).pause;
        if steps.is_empty() {
            // No baud change at the end (USB).
            self.rx = Rx::Switch { since: now };
            return;
        }
        let wait = *sends > 1 || pause;
        (*since, *tries, *sends) = (now, 0, 0);
        if !wait {
            self.send_step(now, port);
        }
    }

    /// The baud change step has no ACK; go straight to Switch.
    fn send_step(&mut self, now: Ns, port: &mut dyn Port) {
        if let Rx::Configure { steps, since, sends, .. } = &mut self.rx {
            port.write(&steps[0].frame);
            *since = now;
            *sends += 1;
            if steps[0].ack.is_none() {
                self.rx = Rx::Switch { since: now };
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn rx_frame(&mut self, kind: Kind, msg: Option<(ubx::Id, &[u8])>, tim_tp: bool, zda: bool, now: Ns, port: &mut dyn Port, out: &mut Output) {
        match &mut self.rx {
            Rx::Idle | Rx::Switch { .. } | Rx::Version { .. } | Rx::OtaStart { .. } | Rx::OtaSend { .. } | Rx::OtaDone { .. } | Rx::OtaAbort { .. } => {}
            Rx::Probe { since, .. } => {
                if now - *since >= SETTLE {
                    self.rx = Rx::Identify { since: now, tries: 1 };
                    port.write(&ubx::frame(ubx::MON_VER, &[]));
                    self.status("identifying", None, out);
                }
            }
            Rx::Identify { .. } => {
                if let Some((ubx::MON_VER, p)) = msg {
                    self.receiver = Version::parse(p);
                    if self.receiver.is_none() {
                        return self.fail("unreadable UBX-MON-VER reply", now, port, out);
                    }
                    self.polled_port = None;
                    port.write(&ubx::frame(ubx::CFG_PRT, &[]));
                    self.rx = Rx::PortPoll { since: now, tries: 1 };
                }
            }
            Rx::PortPoll { .. } => match msg {
                Some((ubx::CFG_PRT, p)) if p.len() == 20 => {
                    self.polled_port = Some(p.to_vec());
                    self.configure(now, port, out);
                }
                // u-blox 10 has no CFG-PRT: configure UART1.
                Some((ubx::ACK_NAK, [0x06, 0x00])) => self.configure(now, port, out),
                _ => {}
            },
            Rx::Configure { steps, sends, .. } => match msg {
                Some((ubx::ACK_ACK, p)) if *sends > 0 && steps[0].ack.is_some_and(|a| p == [a.0, a.1]) => self.next_step(now, port),
                Some((ubx::ACK_NAK, p)) if *sends > 0 && steps[0].ack.is_some_and(|a| p == [a.0, a.1]) => {
                    if steps[0].optional.is_some() {
                        self.skip_step("rejected", now, port, out);
                    } else {
                        self.fail("receiver rejected a configuration message", now, port, out);
                    }
                }
                _ => {}
            },
            // Any enabled message proves the setup; TIM-TP waits for a fix.
            Rx::Verify { .. } => {
                if msg.is_some_and(|(id, _)| ubx::ENABLED.contains(&id)) {
                    self.failures = 0;
                    self.set_source(Source::TimTp);
                    self.rx = Rx::Running { last_frame: now, last_tp: now };
                    self.status("ubx", None, out);
                }
            }
            Rx::Running { last_frame, last_tp } => {
                *last_frame = now;
                if tim_tp {
                    *last_tp = now;
                }
                if kind == Kind::Nmea {
                    self.fail("receiver sends NMEA again (reset?)", now, port, out);
                }
            }
            Rx::Fallback { last_label, last_tp } => {
                // A setup that failed late may leave the receiver UBX-only: use its TIM-TP.
                if tim_tp {
                    *last_tp = Some(now);
                }
                if tim_tp || zda {
                    *last_label = now;
                }
                let s = if last_tp.is_some_and(|t| now - t < PASSIVE_TP_WINDOW) { Source::TimTp } else { Source::Zda };
                self.set_source(s);
            }
            Rx::Passive { last_tp } => {
                if tim_tp {
                    *last_tp = Some(now);
                }
                let s = if last_tp.is_some_and(|t| now - t < PASSIVE_TP_WINDOW) { Source::TimTp } else { Source::Zda };
                self.set_source(s);
            }
        }
    }

    fn rx_tick(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let bauds = self.bauds();
        match &mut self.rx {
            Rx::Idle | Rx::Fallback { .. } | Rx::Passive { .. } => {}
            Rx::OtaStart { .. } | Rx::OtaSend { .. } | Rx::OtaDone { .. } => self.ota_tick(now, port, out),
            Rx::OtaAbort { since } => {
                if now - *since >= OTA_ABORT_WAIT {
                    self.after_version(now, port, out);
                }
            }
            Rx::Version { since, tries } => {
                if now - *since >= VER_WAIT {
                    if *tries >= VER_TRIES {
                        let state = self.state;
                        self.status(state, Some("no VER reply; firmware not checked".into()), out);
                        self.after_version(now, port, out);
                    } else {
                        *tries += 1;
                        *since = now;
                        port.write(&nmea::command("VER"));
                    }
                }
            }
            Rx::Probe { idx, since } => {
                if now - *since >= PROBE_WAIT {
                    let next = (*idx + 1) % bauds.len();
                    self.enter_probe(next, now, port);
                    if next == 0 {
                        let list: Vec<String> = bauds.iter().map(u32::to_string).collect();
                        self.status("probing", Some(format!("no receiver data at {} baud", list.join(", "))), out);
                    }
                }
            }
            Rx::Identify { since, tries } => {
                if now - *since >= POLL_WAIT {
                    if *tries >= POLL_TRIES && self.cfg.strict {
                        self.fail("receiver does not answer UBX-MON-VER", now, port, out);
                    } else if *tries >= POLL_TRIES {
                        self.fallback("receiver does not answer UBX-MON-VER, labelling pulses from NMEA ZDA".into(), now, out);
                    } else {
                        *tries += 1;
                        *since = now;
                        port.write(&ubx::frame(ubx::MON_VER, &[]));
                    }
                }
            }
            Rx::PortPoll { since, tries } => {
                if now - *since >= POLL_WAIT {
                    if *tries >= POLL_TRIES {
                        self.configure(now, port, out);
                    } else {
                        *tries += 1;
                        *since = now;
                        port.write(&ubx::frame(ubx::CFG_PRT, &[]));
                    }
                }
            }
            Rx::Configure { since, tries, steps, sends } => {
                if now - *since >= POLL_WAIT && *sends == 0 {
                    self.send_step(now, port);
                } else if now - *since >= POLL_WAIT {
                    *tries += 1;
                    if *tries >= ACK_TRIES && steps[0].optional.is_some() {
                        self.skip_step("did not acknowledge", now, port, out);
                    } else if *tries >= ACK_TRIES {
                        self.fail("no acknowledgement from the receiver", now, port, out);
                    } else {
                        self.send_step(now, port);
                    }
                }
            }
            Rx::Switch { since } => {
                if now - *since >= SWITCH_WAIT {
                    self.set_baud(self.cfg.baud, port);
                    self.rx = Rx::Verify { since: now };
                    self.status("verifying", None, out);
                }
            }
            Rx::Verify { since } => {
                if now - *since >= VERIFY_WAIT {
                    self.fail("no UBX output after configuration", now, port, out);
                }
            }
            Rx::Running { last_frame, last_tp } => {
                if now - *last_frame >= FRAME_TIMEOUT {
                    self.fail("receiver stopped sending", now, port, out);
                } else if now - *last_tp >= TIM_TP_TIMEOUT && self.fix_since.is_some_and(|t| now - t >= TIM_TP_TIMEOUT) {
                    // The receiver still talks at this baud: no need to search.
                    let state = self.state;
                    self.status(state, Some("fix but no UBX-TIM-TP; configuring again".into()), out);
                    self.configure(now, port, out);
                }
            }
        }
        if let Rx::Fallback { last_label, .. } = self.rx {
            // Nothing usable for a while: set the receiver up again.
            if now - last_label >= FRAME_TIMEOUT {
                self.failures = 0;
                self.receiver = None;
                let state = self.state;
                self.status(state, Some("no ZDA or TIM-TP from the receiver".into()), out);
                self.start_probe(now, port, out);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    const UTC0_S: i64 = 1_790_000_000;
    const LEAP: i64 = 18;

    /// A sofgnss device plus receiver. GPS data only gets through when both use the same baud.
    struct Fake {
        host_baud: u32,
        rx_baud: u32,
        /// u-blox 9/10 (CFG-VALSET only); false = u-blox 5..8.
        gen9: bool,
        /// false: NMEA-only receiver.
        ublox: bool,
        nmea_out: bool,
        /// false: NMEA without ZDA.
        zda_out: bool,
        fix: bool,
        /// NAV-TIMEGPS leapSValid.
        leap_valid: bool,
        ubx_out: bool,
        enabled: BTreeSet<ubx::Id>,
        to_host: Vec<u8>,
        from_host: Framer,
        elf: String,
        platform: &'static str,
        partition: &'static str,
        /// START and BOOT received.
        boot_cmds: Vec<String>,
        /// Image bytes still expected, and those received.
        ota_left: usize,
        ota_image: Vec<u8>,
        ota_last_ms: i64,
        /// Slot erase before OTA,READY.
        ota_erase_ms: i64,
        ota_ready_at: Option<i64>,
        /// false: output only, ignores polls and config.
        answers: bool,
        /// Delay (ms) the ACK of the first CFG-MSG for this message.
        slow_cfg_msg: Option<(ubx::Id, i64)>,
        delay_next_ack_ms: i64,
        delayed: Vec<(i64, Vec<u8>)>,
        /// Drop the first CFG-MSG for this message.
        lose_cfg_msg: Option<ubx::Id>,
        /// Messages it lacks: CFG-MSG for them gets a NAK.
        nak: BTreeSet<ubx::Id>,
        /// CFG-MSG received, lost ones included.
        cfg_msgs: u32,
        now_ms: i64,
        /// No board: pulses go to `pulses` (ms), no $PVDB in either direction.
        plain: bool,
        pulses: Vec<i64>,
        /// CFG-SIGNAL keys set, and whether VALSETs with them get a NAK.
        signals: std::collections::BTreeMap<u32, u32>,
        nak_signals: bool,
        /// When each CFG-VALSET came (ms).
        valsets: Vec<i64>,
        /// On its own USB port: no baud, and USB configuration keys.
        usb: bool,
        /// A VALSET changed the UART1 baud.
        uart_baud_set: bool,
        /// Plain: false sends no pulses.
        pps: bool,
        /// The UBX output of this second comes this many ms late.
        late: Option<(i64, i64)>,
    }

    impl Fake {
        fn new(rx_baud: u32, gen9: bool, ublox: bool) -> Self {
            Self {
                host_baud: 9600,
                rx_baud,
                gen9,
                ublox,
                nmea_out: true,
                zda_out: true,
                fix: true,
                leap_valid: true,
                ubx_out: true,
                enabled: BTreeSet::new(),
                to_host: vec![],
                from_host: Framer::new(),
                elf: "00".repeat(32),
                platform: "v5.5.1",
                partition: "ota_0",
                boot_cmds: vec![],
                ota_left: 0,
                ota_image: vec![],
                ota_last_ms: 0,
                ota_erase_ms: 0,
                ota_ready_at: None,
                answers: true,
                slow_cfg_msg: None,
                delay_next_ack_ms: 0,
                delayed: vec![],
                lose_cfg_msg: None,
                nak: BTreeSet::new(),
                cfg_msgs: 0,
                now_ms: 0,
                plain: false,
                pulses: vec![],
                signals: Default::default(),
                nak_signals: false,
                valsets: vec![],
                usb: false,
                uart_baud_set: false,
                pps: true,
                late: None,
            }
        }

        fn same_baud(&self) -> bool {
            self.usb || self.host_baud == self.rx_baud
        }

        fn gps(&mut self, bytes: &[u8]) {
            if self.same_baud() {
                self.to_host.extend(bytes);
            }
        }

        fn ack(&mut self, id: ubx::Id, ok: bool) {
            let f = ubx::frame(if ok { ubx::ACK_ACK } else { ubx::ACK_NAK }, &[id.0, id.1]);
            match std::mem::take(&mut self.delay_next_ack_ms) {
                0 => self.gps(&f),
                d => self.delayed.push((self.now_ms + d, f)),
            }
        }

        fn on_ubx(&mut self, id: ubx::Id, p: &[u8]) {
            match id {
                ubx::MON_VER => {
                    let mut v = vec![0u8; 40];
                    if self.gen9 {
                        v[30..38].copy_from_slice(b"000A0000");
                        let mut ext = [0u8; 30];
                        ext[..13].copy_from_slice(b"PROTVER=34.10");
                        v.extend(ext);
                    } else {
                        v[30..38].copy_from_slice(b"00040005");
                    }
                    self.gps(&ubx::frame(ubx::MON_VER, &v));
                }
                ubx::CFG_PRT if self.gen9 => self.ack(id, false),
                ubx::CFG_PRT if p.is_empty() => {
                    let mut c = vec![0u8; 20];
                    c[0] = if self.usb { ubx::PORT_USB } else { ubx::PORT_UART1 };
                    c[8..12].copy_from_slice(&self.rx_baud.to_le_bytes());
                    c[12] = 7;
                    c[14] = self.ubx_out as u8 | (self.nmea_out as u8) << 1;
                    self.gps(&ubx::frame(ubx::CFG_PRT, &c));
                    self.ack(id, true);
                }
                ubx::CFG_PRT => {
                    self.ack(id, true);
                    // The USB port ignores the baud.
                    if !self.usb {
                        self.rx_baud = u32::from_le_bytes(p[8..12].try_into().unwrap());
                    }
                    self.ubx_out = p[14] & 1 != 0;
                    self.nmea_out = p[14] & 2 != 0;
                }
                ubx::CFG_MSG if !self.gen9 => {
                    self.cfg_msgs += 1;
                    if self.lose_cfg_msg == Some((p[0], p[1])) {
                        self.lose_cfg_msg = None;
                        return;
                    }
                    if self.nak.contains(&(p[0], p[1])) {
                        return self.ack(id, false);
                    }
                    if let Some((_, d)) = self.slow_cfg_msg.filter(|(m, _)| *m == (p[0], p[1])) {
                        self.slow_cfg_msg = None;
                        self.delay_next_ack_ms = d;
                    }
                    if p[2] > 0 {
                        self.enabled.insert((p[0], p[1]));
                    } else {
                        self.enabled.remove(&(p[0], p[1]));
                    }
                    self.ack(id, true);
                }
                ubx::CFG_VALSET if self.gen9 => {
                    self.valsets.push(self.now_ms);
                    let mut items = vec![];
                    let mut i = 4;
                    while i < p.len() {
                        let k = u32::from_le_bytes(p[i..i + 4].try_into().unwrap());
                        let n = if k >> 28 == 4 { 4 } else { 1 };
                        let mut v = [0u8; 4];
                        v[..n].copy_from_slice(&p[i + 4..i + 4 + n]);
                        items.push((k, u32::from_le_bytes(v)));
                        i += 4 + n;
                    }
                    let signal = |k: u32| k >> 16 == 0x1031;
                    if self.nak_signals && items.iter().any(|&(k, _)| signal(k)) {
                        return self.ack(id, false);
                    }
                    self.ack(id, true);
                    // Only the keys of the port it is on change its output.
                    let port = if self.usb { ubx::PORT_USB } else { ubx::PORT_UART1 };
                    for (k, v) in items {
                        if signal(k) {
                            self.signals.insert(k, v);
                        }
                        match k {
                            0x4052_0001 if self.usb => self.uart_baud_set = true,
                            0x4052_0001 => self.rx_baud = v,
                            k if k == ubx::key::out_ubx(port) => self.ubx_out = v != 0,
                            k if k == ubx::key::out_nmea(port) => self.nmea_out = v != 0,
                            _ => {
                                for id in ubx::ENABLED {
                                    if ubx::key::msgout(id, port) == k && v > 0 {
                                        self.enabled.insert(id);
                                    }
                                }
                            }
                        }
                    }
                }
                ubx::CFG_MSG | ubx::CFG_VALSET => self.ack(id, false),
                _ => {}
            }
        }

        /// UBX before each pulse, then the pulse, NMEA after it, and a SOF tag every 100 ms.
        fn emit(&mut self, ms: i64) {
            self.now_ms = ms;
            for (_, f) in self.delayed.extract_if(.., |(due, _)| *due <= ms).collect::<Vec<_>>() {
                self.gps(&f);
            }
            if self.ota_ready_at.is_some_and(|t| t <= ms) {
                self.ota_ready_at = None;
                self.ota_last_ms = ms;
                self.to_host.extend(nmea::command("OTA,READY"));
            }
            if self.ota_left > 0 && self.ota_ready_at.is_none() && ms - self.ota_last_ms > 5000 {
                self.ota_left = 0;
                self.to_host.extend(nmea::command("ERR,OTATIMEOUT"));
            }
            let (s, r) = (ms.div_euclid(1000), ms.rem_euclid(1000));
            let ubx_at = 700 + self.late.filter(|l| l.0 == s).map_or(0, |l| l.1);
            if r == ubx_at && self.ublox && self.ubx_out {
                let gps = UTC0_S + s + 1 - 315_964_800 + LEAP;
                let (week, tow) = (gps / 604_800, gps % 604_800);
                // The epoch this output follows.
                let (epoch_week, epoch_tow) = ((gps - 1) / 604_800, (gps - 1) % 604_800);
                // Like the LEA-5T: no TIM-TP without a fix.
                let fix = self.fix;
                for id in self.enabled.clone().into_iter().filter(|&id| fix || id != ubx::TIM_TP) {
                    let mut p = vec![0u8; 16];
                    match id {
                        ubx::TIM_TP => {
                            p[..4].copy_from_slice(&((tow * 1000) as u32).to_le_bytes());
                            p[12..14].copy_from_slice(&(week as u16).to_le_bytes());
                        }
                        ubx::NAV_TIMEGPS => {
                            p[..4].copy_from_slice(&((epoch_tow * 1000) as u32).to_le_bytes());
                            p[8..10].copy_from_slice(&(epoch_week as u16).to_le_bytes());
                            // An unknown count is the firmware default.
                            (p[10], p[11]) = if self.leap_valid { (LEAP as u8, 0x07) } else { (15, 0x03) };
                        }
                        _ if self.fix => {
                            p[4] = 3;
                            p[5] = 0x0D;
                        }
                        _ => p[5] = 0x0C,
                    }
                    self.gps(&ubx::frame(id, &p));
                }
            }
            if r == 0 && self.plain {
                if self.pps {
                    self.pulses.push(ms);
                }
            } else if r == 0 {
                self.to_host.extend(nmea::command(&format!("PPS,{},300", ms * 80_000 + 1000)));
            }
            if r == 100 && self.nmea_out {
                let t = UTC0_S + s;
                let sod = t.rem_euclid(86_400);
                let hms = format!("{:02}{:02}{:02}.00", sod / 3600, sod / 60 % 60, sod % 60);
                // 2026-09-21 is day 20_717.
                assert_eq!(t.div_euclid(86_400), 20_717, "test date");
                let q = if self.fix { 1 } else { 0 };
                self.gps(&nmea::sentence(&format!("GPGGA,{hms},1,N,2,E,{q},08,0.9,10,M,0,M,,")));
                if self.zda_out {
                    self.gps(&nmea::sentence(&format!("GPZDA,{hms},21,09,2026,00,00")));
                }
            }
            if r % 100 == 50 && !self.plain {
                self.to_host.extend(nmea::command(&format!("SOF,{},{}", ms * 80_000 + 1000, ms)));
            }
        }
    }

    impl Port for Fake {
        fn write(&mut self, mut data: &[u8]) {
            if self.ota_left > 0 {
                self.ota_last_ms = self.now_ms;
                let n = self.ota_left.min(data.len());
                let before = self.ota_image.len();
                self.ota_image.extend(&data[..n]);
                self.ota_left -= n;
                if self.ota_image.len() / 32_768 > before / 32_768 {
                    self.to_host.extend(nmea::command(&format!("OTA,ACK,{}", self.ota_image.len())));
                }
                if self.ota_left == 0 {
                    self.to_host.extend(nmea::command("OTA,OK"));
                }
                data = &data[n..];
            }
            let mut frames = vec![];
            self.from_host.feed(data, |k, f| frames.push((k, f.to_vec())));
            for (k, f) in frames {
                if f.starts_with(nmea::PVDB_PREFIX) {
                    assert!(!self.plain, "board command to a plain receiver");
                    let fl = nmea::fields(&f);
                    if fl[2] == "INFO" && fl.len() > 3 {
                        self.to_host.extend(nmea::command(&format!("INFO,80000000,100,0,0,0,0,0,{}", fl[3])));
                    } else if fl[2] == "VER" {
                        let (p, part, elf) = (self.platform, self.partition, &self.elf);
                        self.to_host.extend(nmea::command(&format!("VER,v1,{p},{part},0,{elf},POWERON")));
                    } else if fl[2] == "START" || fl[2] == "BOOT" {
                        self.boot_cmds.push(fl[2].to_string());
                    } else if fl[2] == "OTA" {
                        self.ota_left = fl[3].parse().unwrap();
                        self.ota_ready_at = Some(self.now_ms + self.ota_erase_ms);
                    } else if fl[2] == "DIV" {
                        self.to_host.extend(nmea::command("ERR,BADDIV"));
                    }
                } else if k == Kind::Ubx && self.ublox && self.answers && self.same_baud() {
                    let (id, p) = ubx::split(&f);
                    self.on_ubx(id, p);
                }
            }
        }

        fn set_baud(&mut self, baud: u32) {
            self.host_baud = baud;
        }
    }

    struct Run {
        session: Session,
        fake: Fake,
        ms: i64,
        out: Output,
    }

    impl Run {
        fn new(fake: Fake) -> Self {
            Self::with(Config::default(), fake)
        }

        fn with(cfg: Config, fake: Fake) -> Self {
            Self { session: Session::new(cfg, "sofdtest".into(), 0), fake, ms: 0, out: Output::default() }
        }

        fn for_secs(&mut self, secs: i64) -> &mut Self {
            for _ in 0..secs * 100 {
                for i in 0..10 {
                    self.fake.emit(self.ms + i);
                }
                self.ms += 10;
                let now = self.ms * MS;
                let bytes = std::mem::take(&mut self.fake.to_host);
                self.session.on_bytes(&bytes, now, &mut self.fake, &mut self.out);
                for p in std::mem::take(&mut self.fake.pulses) {
                    self.session.on_pulse(p as u64 * 1_000_000 + 1000, 300, now, &mut self.out);
                }
                self.session.on_tick(now, &mut self.fake, &mut self.out);
            }
            self
        }

        fn states(&self) -> Vec<String> {
            self.out
                .events
                .iter()
                .filter(|e| e["event"] == "gnss_status")
                .map(|e| e["state"].as_str().unwrap().to_string())
                .collect()
        }

        fn check_tags(&self, source: Source) {
            assert!(self.out.tags.len() > 20, "{} tags; states {:?}", self.out.tags.len(), self.states());
            for t in &self.out.tags {
                let super::super::timing::At::Frame { frame, .. } = t.at else { panic!("{t:?}") };
                assert_eq!(t.utc_ns, (UTC0_S * 1000 + frame as i64) * MS, "{t:?}");
                assert_eq!(t.source, source);
            }
        }
    }

    #[test]
    fn configures_legacy_receiver() {
        let mut r = Run::new(Fake::new(9600, false, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!((r.fake.rx_baud, r.fake.nmea_out, r.fake.ubx_out), (38_400, false, true));
        // u-blox 5: base set plus legacy diagnostics.
        let want: BTreeSet<ubx::Id> =
            ubx::ENABLED.into_iter().chain([ubx::NAV_SOL, ubx::NAV_TIMEUTC, ubx::NAV_POSLLH, ubx::NAV_VELNED, ubx::NAV_SVINFO, ubx::MON_HW]).collect();
        assert_eq!(r.fake.enabled, want);
        r.check_tags(Source::TimTp);
        assert!(r.out.frames.iter().any(|(k, _)| *k == Kind::Nmea));
        assert!(r.out.events.iter().any(|e| e["event"] == "gnss_pps" && e["accepted"] == true));
    }

    #[test]
    fn configures_generation_10_receiver() {
        let mut r = Run::new(Fake::new(9600, true, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!((r.fake.rx_baud, r.fake.nmea_out, r.fake.enabled.len()), (38_400, false, 3));
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn already_configured_receiver_is_found_at_target_baud() {
        let mut f = Fake::new(38_400, false, true);
        f.nmea_out = false;
        f.enabled = ubx::ENABLED.into_iter().collect();
        let mut r = Run::new(f);
        r.for_secs(6);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!(r.states()[..2], ["probing", "identifying"]);
    }

    #[test]
    fn finds_a_receiver_at_an_unconfigured_baud() {
        let mut r = Run::new(Fake::new(115_200, false, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!(r.fake.rx_baud, 38_400);
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn nmea_only_receiver_falls_back_to_zda() {
        let mut r = Run::with(Config { initial_baud: 4800, ..Config::default() }, Fake::new(4800, false, false));
        r.for_secs(30);
        assert_eq!(r.session.state(), "nmea_fallback", "{:?}", r.states());
        r.check_tags(Source::Zda);
    }

    #[test]
    fn late_ack_of_a_retried_step_does_not_skip_the_next() {
        let mut f = Fake::new(9600, false, true);
        // TIM-TP's ACK comes late and NAV-TIMEGPS is lost: the late ACK must not count for it.
        f.slow_cfg_msg = Some((ubx::TIM_TP, 1500));
        f.lose_cfg_msg = Some(ubx::NAV_TIMEGPS);
        let mut r = Run::new(f);
        r.for_secs(25);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(ubx::ENABLED.iter().all(|id| r.fake.enabled.contains(id)));
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn lost_step_does_not_slow_the_next() {
        let mut f = Fake::new(9600, false, true);
        f.lose_cfg_msg = Some(ubx::TIM_TP);
        let mut r = Run::new(f);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        // 9 steps, and one resend of the lost one.
        assert_eq!(r.fake.cfg_msgs, 10);
    }

    #[test]
    fn diagnostics_the_receiver_lacks_are_skipped() {
        let mut f = Fake::new(9600, false, true);
        f.nak = [ubx::NAV_SVINFO, ubx::MON_HW].into_iter().collect();
        let mut r = Run::new(f);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(ubx::ENABLED.iter().all(|id| r.fake.enabled.contains(id)));
        assert!(r.out.events.iter().any(|e| e["message"] == "receiver rejected NAV-SVINFO; continuing without"));
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn fallback_follows_tim_tp_of_a_ubx_only_receiver() {
        // Set up earlier (UBX only, TIM-TP on) but no longer answering polls.
        let mut f = Fake::new(38_400, false, true);
        f.nmea_out = false;
        f.enabled = ubx::ENABLED.into_iter().collect();
        f.answers = false;
        let mut r = Run::new(f);
        r.for_secs(25);
        assert_eq!(r.session.state(), "nmea_fallback", "{:?}", r.states());
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn fallback_without_labels_never_locks() {
        let mut f = Fake::new(9600, false, false);
        f.zda_out = false;
        let mut r = Run::new(f);
        r.for_secs(30);
        assert!(r.states().contains(&"nmea_fallback".to_string()), "{:?}", r.states());
        assert!(!r.out.events.iter().any(|e| e["locked"] == true));
    }

    #[test]
    fn passive_locks_only_while_pulses_are_accepted() {
        let cfg = Config { configure: false, ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(4800, false, true));
        r.for_secs(10);
        assert_eq!(r.states(), ["passive"]);
        assert!(!r.session.locked);
        r.fake.rx_baud = 9600;
        r.for_secs(10);
        assert!(r.session.locked, "{:?}", r.states());
        r.check_tags(Source::Zda);
        r.fake.rx_baud = 4800;
        r.for_secs(10);
        assert!(!r.session.locked);
    }

    #[test]
    fn ubx_without_fix_is_not_locked() {
        let mut f = Fake::new(9600, false, true);
        f.fix = false;
        let mut r = Run::new(f);
        r.for_secs(30);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!(r.states().iter().filter(|s| *s == "probing").count(), 1, "{:?}", r.states());
        assert!(!r.session.locked);
        r.fake.fix = true;
        r.for_secs(5);
        assert!(r.session.locked);
        let last = r.out.events.iter().rev().find(|e| e["event"] == "gnss_status").unwrap();
        assert_eq!(last["state"], "ubx");
        assert_eq!(last["locked"], true);
    }

    #[test]
    fn lost_tim_tp_with_a_fix_reconfigures_without_searching() {
        let mut r = Run::new(Fake::new(9600, false, true));
        r.for_secs(15);
        assert_eq!(r.session.state(), "ubx");
        r.fake.enabled.remove(&ubx::TIM_TP);
        r.out = Output::default();
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.states().contains(&"configuring".to_string()), "{:?}", r.states());
        assert!(!r.states().contains(&"probing".to_string()), "{:?}", r.states());
        assert!(r.fake.enabled.contains(&ubx::TIM_TP));
        assert!(r.session.locked);
    }

    #[test]
    fn strict_keeps_retrying_the_setup() {
        let cfg = Config { strict: true, ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(9600, false, false));
        r.for_secs(120);
        assert!(!r.states().contains(&"nmea_fallback".to_string()), "{:?}", r.states());
        assert!(r.states().iter().filter(|s| *s == "identifying").count() > 3, "{:?}", r.states());
        // Once the receiver answers, setup goes through.
        r.fake.ublox = true;
        r.for_secs(60);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
    }

    #[test]
    fn configured_leap_seconds_fill_in_until_the_receiver_knows() {
        let mut f = Fake::new(9600, false, true);
        f.leap_valid = false;
        let mut r = Run::new(f);
        r.for_secs(20);
        assert!(r.out.tags.is_empty());
        assert!(r.out.events.iter().any(|e| e["reason"] == "no_leap_seconds"));

        let mut f = Fake::new(9600, false, true);
        f.leap_valid = false;
        let mut r = Run::with(Config { leap_seconds: Some(LEAP as i8), ..Config::default() }, f);
        r.for_secs(20);
        assert!(r.session.locked);
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn receiver_leap_seconds_win_over_the_config() {
        let mut r = Run::with(Config { leap_seconds: Some(17), ..Config::default() }, Fake::new(9600, false, true));
        r.for_secs(20);
        r.check_tags(Source::TimTp);
        let msg = format!("receiver reports {LEAP} leap seconds, the config says 17");
        // Once, not at every NAV-TIMEGPS. The lock change repeats the message.
        let warned: Vec<_> = r.out.events.iter().filter(|e| e["message"] == msg.as_str()).collect();
        assert_eq!(warned.iter().filter(|e| e["locked"] == false).count(), 1, "{warned:?}");
    }

    #[test]
    fn plain_receiver_labels_kernel_pulses() {
        let mut f = Fake::new(9600, false, true);
        f.plain = true;
        let mut r = Run::with(Config { plain: true, ..Config::default() }, f);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.session.locked);
        let accepted: Vec<&Value> = r.out.events.iter().filter(|e| e["event"] == "gnss_pps" && e["accepted"] == true).collect();
        assert!(accepted.len() > 10, "{:?}", r.out.events);
        for e in accepted {
            let tick = e["tick"].as_i64().unwrap();
            assert_eq!(e["utc_ns"].as_i64().unwrap(), UTC0_S * 1_000_000_000 + tick - 1000, "{e}");
        }
        assert!(r.out.tags.is_empty());
    }

    #[test]
    fn publishes_a_fix_per_epoch() {
        let mut pvt = vec![0u8; 92];
        pvt[4..6].copy_from_slice(&2026u16.to_le_bytes());
        (pvt[6], pvt[7], pvt[11], pvt[20], pvt[21], pvt[23]) = (9, 21, 0x07, 3, 0x01, 11);
        pvt[28..32].copy_from_slice(&481_234_567i32.to_le_bytes());
        let frame = ubx::frame(ubx::NAV_PVT, &pvt);
        let fixes = |r: &Run| r.out.events.iter().filter(|e| e["event"] == "gnss_fix").cloned().collect::<Vec<Value>>();

        let mut r = Run::new(Fake::new(9600, false, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        // Before the setup the receiver sent NMEA: a fix from its GGA.
        let nmea: Vec<Value> = fixes(&r);
        assert!(!nmea.is_empty() && nmea.iter().all(|f| f["source"] == "nmea" && f["fix"] == "3d"), "{nmea:?}");
        // The fake's GGA: 1 minute north, 8 satellites.
        assert!(nmea[0]["lat_deg"] == 0.0166667 && nmea[0]["sats_used"] == 8, "{}", nmea[0]);
        let now = r.ms * MS;
        r.session.on_bytes(&frame, now, &mut r.fake, &mut r.out);
        let f = fixes(&r).pop().unwrap();
        assert_eq!((f["source"].as_str(), f["fix"].as_str(), f["lat_deg"].as_f64(), f["sats_used"].as_u64()), (Some("nav_pvt"), Some("3d"), Some(48.1234567), Some(11)));
        assert_eq!((f["leap_seconds"].as_i64(), f["utc_ns"].as_i64()), (Some(LEAP), Some(20_717 * 86_400 * 1_000_000_000)));

        // Turned off.
        let mut r = Run::with(Config { fix: false, ..Config::default() }, Fake::new(9600, false, true));
        r.for_secs(20);
        let now = r.ms * MS;
        r.session.on_bytes(&frame, now, &mut r.fake, &mut r.out);
        assert!(fixes(&r).is_empty());
    }

    /// A receiver without pulses, on its USB port if `usb`.
    fn no_pps(gen9: bool, ublox: bool, usb: bool, serial_delay_ms: Option<i64>) -> Run {
        let mut f = Fake::new(9600, gen9, ublox);
        (f.plain, f.pps, f.usb) = (true, false, usb);
        let cfg = Config { plain: true, pps: false, usb, serial_delay_ns: serial_delay_ms.map(|d| d * MS), ..Config::default() };
        Run::with(cfg, f)
    }

    fn serial_events(r: &Run) -> Vec<&Value> {
        r.out.events.iter().filter(|e| e["event"] == "gnss_pps").inspect(|e| assert_eq!(e["source"], "serial", "{e}")).collect()
    }

    #[test]
    fn usb_receiver_without_pulses_only_passes_data() {
        let mut r = no_pps(false, true, true, None);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert_eq!((r.fake.nmea_out, r.fake.ubx_out, r.fake.enabled.len() >= 3), (false, true, true));
        assert!(!r.session.locked);
        assert!(!r.out.events.iter().any(|e| e["event"] == "gnss_pps"));
        assert!(r.out.frames.iter().any(|(k, f)| *k == Kind::Ubx && ubx::split(f).0 == ubx::TIM_TP));
    }

    #[test]
    fn generation_9_on_usb_gets_usb_keys_and_no_baud() {
        let mut r = no_pps(true, true, true, None);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        // Only the USB keys turn its NMEA off and the messages on.
        assert_eq!((r.fake.nmea_out, r.fake.enabled.len(), r.fake.uart_baud_set), (false, 3, false));
    }

    #[test]
    fn serial_timing_dates_epochs_by_their_output() {
        // The output of each epoch starts 700 ms after it and is read 10 ms later.
        let mut r = no_pps(false, true, false, Some(710));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.session.locked);
        let ev = serial_events(&r);
        assert_eq!(ev[0]["reason"], "first_epoch");
        let accepted: Vec<&&Value> = ev.iter().filter(|e| e["accepted"] == true).collect();
        assert!(accepted.len() > 8, "{ev:?}");
        for e in accepted {
            let tick = e["tick"].as_i64().unwrap();
            assert_eq!(tick % 1_000_000_000, 0, "{e}");
            assert_eq!(e["utc_ns"].as_i64().unwrap(), UTC0_S * 1_000_000_000 + tick, "{e}");
            assert!(e["rx_idle_ms"].as_u64().unwrap() > 500, "{e}");
        }
        assert!(r.out.events.iter().any(|e| e["event"] == "gnss_status" && e["serial_delay_ns"] == 710 * MS));
    }

    #[test]
    fn serial_timing_uses_zda_of_an_nmea_receiver() {
        // NMEA comes 100 ms after the epoch.
        let mut r = no_pps(false, false, false, Some(110));
        r.for_secs(30);
        assert_eq!(r.session.state(), "nmea_fallback", "{:?}", r.states());
        assert!(r.session.locked);
        let ev = serial_events(&r);
        let accepted: Vec<&&Value> = ev.iter().filter(|e| e["accepted"] == true).collect();
        assert!(accepted.len() > 5, "{ev:?}");
        for e in accepted {
            assert_eq!(e["utc_ns"].as_i64().unwrap(), UTC0_S * 1_000_000_000 + e["tick"].as_i64().unwrap(), "{e}");
        }
    }

    #[test]
    fn serial_timing_rejects_late_output_and_no_fix() {
        let mut r = no_pps(false, true, false, Some(710));
        r.fake.late = Some((25, 50));
        r.for_secs(30);
        let at = |r: &Run, s: i64| {
            let utc = (UTC0_S + s) * 1_000_000_000;
            serial_events(r).into_iter().find(|e| e["utc_ns"] == utc).unwrap_or_else(|| panic!("no epoch {s}")).clone()
        };
        assert_eq!(at(&r, 24)["accepted"], true);
        // 50 ms late, then back: both intervals are off.
        assert_eq!((at(&r, 25)["reason"].clone(), at(&r, 26)["reason"].clone()), (json!("interval"), json!("interval")));
        assert_eq!(at(&r, 27)["accepted"], true);
        r.fake.fix = false;
        r.for_secs(10);
        assert_eq!(at(&r, 35)["reason"], "no_fix");
        assert!(!r.session.locked);
    }

    #[test]
    fn sets_constellations_on_generation_9() {
        use ubx::Constellation::*;
        let cfg = Config { constellations: Some(vec![Gps, Galileo, Beidou]), ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(38_400, true, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        let on: Vec<u32> = r.fake.signals.iter().filter(|(_, v)| **v == 1).map(|(k, _)| *k).collect();
        assert_eq!(on, vec![Gps.signal_key(), Galileo.signal_key(), Beidou.signal_key()]);
        assert_eq!(r.fake.signals.len(), 6);
        // The receiver's GNSS restarts: 0.5 s at least before the next.
        assert!(r.fake.valsets[1] - r.fake.valsets[0] >= 500, "{:?}", r.fake.valsets);
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn rejected_constellations_do_not_stop_the_setup() {
        let cfg = Config { constellations: Some(vec![ubx::Constellation::Gps]), ..Config::default() };
        let mut f = Fake::new(38_400, true, true);
        f.nak_signals = true;
        let mut r = Run::with(cfg, f);
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.out.events.iter().any(|e| e["message"] == "receiver rejected constellations; continuing without"));
    }

    #[test]
    fn constellations_are_left_alone_before_generation_9() {
        let cfg = Config { constellations: Some(vec![ubx::Constellation::Gps]), ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(9600, false, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.fake.signals.is_empty());
        assert!(r.out.events.iter().any(|e| e["message"] == "constellations need u-blox 9 or later; left as they are"));
    }

    #[test]
    fn stm32_boards_default_to_their_sof_delay() {
        let mut f = Fake::new(9600, false, true);
        f.platform = "stm32g4";
        let mut r = Run::new(f);
        r.for_secs(20);
        assert!(r.out.tags.len() > 20);
        for t in &r.out.tags {
            let super::super::timing::At::Frame { frame, .. } = t.at else { panic!("{t:?}") };
            assert_eq!(t.utc_ns, (UTC0_S * 1000 + frame as i64) * MS - 1300, "{t:?}");
        }
        assert_eq!(r.out.events.iter().rev().find(|e| e["event"] == "gnss_status").unwrap()["sof_delay_ns"], 1300);
        // Configured, even 0, it wins.
        let mut f = Fake::new(9600, false, true);
        f.platform = "stm32g4";
        let mut r = Run::with(Config { sof_delay_ns: Some(0), ..Config::default() }, f);
        r.for_secs(20);
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn receiver_reset_restarts_setup() {
        let mut r = Run::new(Fake::new(9600, false, true));
        r.for_secs(15);
        assert_eq!(r.session.state(), "ubx");
        // Power cycle: back to its defaults.
        r.fake.rx_baud = 9600;
        r.fake.nmea_out = true;
        r.fake.enabled.clear();
        r.out = Output::default();
        r.for_secs(25);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.states().contains(&"probing".to_string()));
        assert_eq!(r.fake.rx_baud, 38_400);
        r.check_tags(Source::TimTp);
    }

    #[test]
    fn receiver_at_a_nonstandard_baud_is_not_found() {
        let mut r = Run::new(Fake::new(1200, false, true));
        r.for_secs(20);
        assert_eq!(r.session.state(), "probing");
        let msg = "no receiver data at 38400, 9600, 115200, 4800, 19200, 57600, 230400, 460800, 921600 baud";
        assert!(r.out.events.iter().any(|e| e["message"] == msg));
    }

    #[test]
    fn updates_firmware_that_differs() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        let cfg = Config { firmware: Some(img.clone()), ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(9600, false, true));
        r.for_secs(5);
        assert_eq!(r.session.state(), "restarting", "{:?}", r.states());
        assert!(r.session.restarting());
        // The image arrived intact: nothing (PING, setup) was mixed into it.
        assert!(r.fake.ota_image == img.data);
        assert!(r.out.events.iter().any(|e| e["state"] == "updating" && e["message"] == "firmware v1 -> v2"));
        // The device never restarts here: the session gives up and carries on.
        r.for_secs(20);
        assert!(r.states().contains(&"update_failed".to_string()));
        assert_eq!(r.session.state(), "ubx");
    }

    #[test]
    fn other_device_errors_do_not_fail_the_update() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        // The fake answers DIV with ERR,BADDIV, in the same read as VER.
        let cfg = Config { firmware: Some(img.clone()), div: Some(5), ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(9600, false, true));
        r.for_secs(5);
        assert_eq!(r.session.state(), "restarting", "{:?}", r.states());
        assert!(r.fake.ota_image == img.data);
    }

    #[test]
    fn failed_update_waits_for_the_device_to_give_up() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        let mut f = Fake::new(9600, false, true);
        // READY comes after sofd gave up on it.
        f.ota_erase_ms = 16_000;
        let mut r = Run::with(Config { firmware: Some(img), ..Config::default() }, f);
        r.for_secs(40);
        assert!(r.states().contains(&"update_failed".to_string()));
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.fake.ota_image.is_empty());
    }

    #[test]
    fn esp_image_is_not_sent_to_stm32() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        let mut f = Fake::new(9600, false, true);
        f.platform = "stm32g4";
        let mut r = Run::with(Config { firmware: Some(img), ..Config::default() }, f);
        r.for_secs(15);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.fake.ota_image.is_empty());
        let msg = "firmware is an ESP32 image, the board is stm32g4";
        assert!(r.out.events.iter().any(|e| e["state"] == "update_failed" && e["message"] == msg));
    }

    fn stm32_image() -> Arc<super::super::firmware::Image> {
        let mut img = super::super::firmware::test_image("v2", 0xcd, 50_000);
        img.stm32 = true;
        Arc::new(img)
    }

    #[test]
    fn stm32_app_updates_through_the_bootloader() {
        let mut f = Fake::new(9600, false, true);
        f.platform = "stm32g4";
        let mut r = Run::with(Config { firmware: Some(stm32_image()), ..Config::default() }, f);
        r.for_secs(5);
        assert_eq!(r.fake.boot_cmds, ["BOOT"]);
        assert!(r.session.restarting());
        assert!(r.fake.ota_image.is_empty());
        assert!(!r.states().contains(&"updating".to_string()), "{:?}", r.states());
    }

    #[test]
    fn bootloader_installs_the_image_then_starts_it() {
        let img = stm32_image();
        let mut f = Fake::new(9600, false, true);
        (f.platform, f.partition, f.elf) = (BOOTLOADER, "empty", String::new());
        let mut r = Run::with(Config { firmware: Some(img.clone()), ..Config::default() }, f);
        r.for_secs(5);
        assert!(r.fake.ota_image == img.data);
        assert_eq!(r.fake.boot_cmds, ["START"]);
        assert!(r.session.restarting());
        assert!(r.out.events.iter().any(|e| e["state"] == "updating" && e["message"] == "firmware none -> v2"));
    }

    #[test]
    fn bootloader_starts_a_valid_app() {
        let img = stm32_image();
        for (firmware, partition, elf, start) in [
            (Some(img.clone()), "valid", img.elf_sha256.clone(), true),
            (None, "valid", "ab".repeat(32), true),
            (None, "invalid", "ab".repeat(32), false),
            (None, "empty", String::new(), false),
        ] {
            let mut f = Fake::new(9600, false, true);
            (f.platform, f.partition, f.elf) = (BOOTLOADER, partition, elf);
            let mut r = Run::with(Config { firmware, ..Config::default() }, f);
            r.for_secs(5);
            assert!(r.fake.ota_image.is_empty());
            assert_eq!(r.fake.boot_cmds.len(), start as usize, "{partition}");
            assert_eq!(r.session.state(), if start { "restarting" } else { "no_app" }, "{partition}");
        }
    }

    #[test]
    fn stm32_image_is_not_sent_to_esp32() {
        let mut r = Run::with(Config { firmware: Some(stm32_image()), ..Config::default() }, Fake::new(9600, false, true));
        r.for_secs(15);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.fake.ota_image.is_empty());
        let msg = "firmware is an STM32 image, the board is v5.5.1";
        assert!(r.out.events.iter().any(|e| e["state"] == "update_failed" && e["message"] == msg));
    }

    #[test]
    fn same_firmware_is_not_sent() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        let mut f = Fake::new(9600, false, true);
        f.elf = img.elf_sha256.clone();
        let mut r = Run::with(Config { firmware: Some(img), ..Config::default() }, f);
        r.for_secs(15);
        assert_eq!(r.session.state(), "ubx", "{:?}", r.states());
        assert!(r.fake.ota_image.is_empty());
    }
}
