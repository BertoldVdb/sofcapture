//! State machine for one sofgps device, without I/O: handshake, firmware update, receiver setup,
//! then labelling pulses from TIM-TP, or from NMEA ZDA if the receiver doesn't answer UBX.

use super::firmware::Image;
use super::framer::{Framer, Kind};
use super::nmea::{self, OtaMsg, Pvdb};
use super::timing::{Labeler, PpsDecision, Source, Tag};
use super::ubx::{self, Version};
use crate::clock::Ns;
use serde_json::{json, Value};
use std::sync::Arc;

const MS: Ns = 1_000_000;
const PING_EVERY: Ns = 10_000 * MS;
const HANDSHAKE_RETRY: Ns = 1_000 * MS;
const HANDSHAKE_WARN: Ns = 5_000 * MS;
/// Frames this soon after a baud change may be from before it.
const SETTLE: Ns = 250 * MS;
/// The device only forwards valid frames, so a wrong baud is just silence.
const PROBE_WAIT: Ns = 1_500 * MS;
const POLL_WAIT: Ns = 1_000 * MS;
const POLL_TRIES: u32 = 3;
const ACK_TRIES: u32 = 3;
/// Let the last message at the old baud leave the device's UART.
const SWITCH_WAIT: Ns = 200 * MS;
const VERIFY_WAIT: Ns = 3_500 * MS;
const FRAME_TIMEOUT: Ns = 5_000 * MS;
const TIM_TP_TIMEOUT: Ns = 10_000 * MS;
/// Failed setups in a row before falling back to ZDA.
const MAX_FAILURES: u32 = 3;
/// Use TIM-TP while one arrived this recently, else ZDA.
const PASSIVE_TP_WINDOW: Ns = 3_000 * MS;
const VER_WAIT: Ns = 1_000 * MS;
const VER_TRIES: u32 = 3;
/// The device erases the update slot before READY.
const OTA_READY_WAIT: Ns = 15_000 * MS;
/// Max wait for `OTA,ACK` (sent every 32 KB) or `OTA,OK`.
const OTA_PROGRESS_WAIT: Ns = 10_000 * MS;
const OTA_RESTART_WAIT: Ns = 10_000 * MS;
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
    /// Emit every GPS frame.
    pub frames: bool,
    pub div: Option<u32>,
    /// From bus frame start to the device's SOF capture.
    pub sof_delay_ns: Ns,
    /// Sent if VER reports a different image.
    pub firmware: Option<Arc<Image>>,
}

impl Default for Config {
    fn default() -> Self {
        Self { baud: 38_400, initial_baud: 9600, configure: true, frames: true, div: None, sof_delay_ns: 0, firmware: None }
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
    pub elf_sha256: String,
    pub reset_reason: String,
}

struct Step {
    frame: Vec<u8>,
    /// None for the baud change: its ACK may be lost.
    ack: Option<ubx::Id>,
    /// Diagnostics: skipped if rejected or unanswered.
    optional: Option<String>,
}

impl Step {
    fn required(frame: Vec<u8>, ack: Option<ubx::Id>) -> Self {
        Self { frame, ack, optional: None }
    }

    fn optional(frame: Vec<u8>, ack: ubx::Id, what: String) -> Self {
        Self { frame, ack: Some(ack), optional: Some(what) }
    }
}

fn ubx_name(id: ubx::Id) -> String {
    match id {
        ubx::NAV_PVT => "NAV-PVT".into(),
        ubx::NAV_SOL => "NAV-SOL".into(),
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
    Probe { idx: usize, since: Ns },
    Identify { since: Ns, tries: u32 },
    PortPoll { since: Ns, tries: u32 },
    /// `stale`: ACKs still due for retries of the previous step. All CFG-MSG steps get the
    /// same ACK, so a late one must not count for the next step.
    Configure { steps: Vec<Step>, since: Ns, tries: u32, sends: u32, stale: u32 },
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
    /// Until NAV-STATUS arrives, the NMEA fix is used.
    last_nav_status: Option<Ns>,
}

impl Session {
    pub fn new(cfg: Config, nonce: String, now: Ns) -> Self {
        let baud = cfg.baud;
        Self {
            cfg,
            nonce,
            ready: false,
            started: now,
            last_handshake: None,
            warned: false,
            last_ping: None,
            framer: Framer::new(),
            labeler: None,
            tick_hz: 0,
            rx: Rx::Idle,
            baud,
            failures: 0,
            receiver: None,
            polled_port: None,
            firmware: None,
            state: "handshake",
            last_nav_status: None,
        }
    }

    #[cfg(test)]
    pub fn state(&self) -> &'static str {
        self.state
    }

    fn status(&mut self, state: &'static str, message: Option<String>, out: &mut Output) {
        self.state = state;
        let mut ev = json!({"event": "gnss_status", "state": state, "baud": self.baud});
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
    }

    pub fn on_bytes(&mut self, data: &[u8], now: Ns, port: &mut dyn Port, out: &mut Output) {
        let mut frames = vec![];
        self.framer.feed(data, |k, f| frames.push((k, f.to_vec())));
        for (kind, f) in frames {
            if kind == Kind::Nmea && f.starts_with(nmea::PVDB_PREFIX) {
                self.on_pvdb(&f, now, port, out);
            } else if self.ready {
                self.on_gps(kind, &f, now, port, out);
            }
        }
    }

    /// False during a firmware transfer.
    pub fn accepts_tx(&self) -> bool {
        !matches!(self.rx, Rx::OtaStart { .. } | Rx::OtaSend { .. })
    }

    pub fn on_tick(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
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
                self.status("handshake", Some("no reply from the sofgps device".into()), out);
            }
            return;
        }
        self.rx_tick(now, port, out);
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
            Pvdb::Ver { version, elf_sha256, reset_reason } => {
                self.firmware = Some(DeviceFirmware {
                    version: version.into(),
                    elf_sha256: elf_sha256.into(),
                    reset_reason: reset_reason.into(),
                });
                if let Rx::Version { .. } = self.rx {
                    self.after_version(now, port, out);
                }
            }
            Pvdb::Ota(m) => self.on_ota(m, now, out),
            Pvdb::Err(code) if matches!(self.rx, Rx::OtaStart { .. } | Rx::OtaSend { .. }) => {
                self.ota_failed(&format!("device error {code}"), now, port, out);
            }
            Pvdb::Pps { tick, rx_idle_ms } => {
                if let Some(l) = &mut self.labeler {
                    let mut decisions: Vec<PpsDecision> = vec![];
                    let mut tags = vec![];
                    l.on_pps(tick, rx_idle_ms, &mut decisions, &mut tags);
                    for d in decisions {
                        let mut ev = serde_json::to_value(&d).expect("decision serializes");
                        ev["event"] = json!("gnss_pps");
                        out.events.push(ev);
                    }
                    for mut t in tags {
                        t.utc_ns -= self.cfg.sof_delay_ns;
                        out.tags.push(t);
                    }
                }
            }
            Pvdb::Sof { tick, frame } => {
                if let Some(l) = &mut self.labeler {
                    l.on_sof(tick, frame, now);
                }
            }
            Pvdb::Err(code) => {
                let state = self.state;
                self.status(state, Some(format!("device error {code}")), out);
            }
            Pvdb::Other => {}
        }
    }

    fn on_gps(&mut self, kind: Kind, f: &[u8], now: Ns, port: &mut dyn Port, out: &mut Output) {
        let msg = (kind == Kind::Ubx).then(|| ubx::split(f));
        if self.cfg.frames {
            out.frames.push((kind, f.to_vec()));
        }
        let mut tim_tp = false;
        let mut zda = false;
        if let Some(l) = &mut self.labeler {
            match (kind, msg) {
                (Kind::Ubx, Some((ubx::TIM_TP, p))) => {
                    if let Some(tp) = ubx::TimTp::parse(p) {
                        tim_tp = true;
                        l.on_tim_tp(&tp);
                    }
                }
                (Kind::Ubx, Some((ubx::NAV_TIMEGPS, p))) => {
                    if let Some(leap) = ubx::leap_seconds(p) {
                        l.on_leap(leap);
                    }
                }
                (Kind::Ubx, Some((ubx::NAV_STATUS, p))) => {
                    if let Some(ok) = ubx::fix_ok(p) {
                        l.on_fix(ok);
                        self.last_nav_status = Some(now);
                    }
                }
                (Kind::Nmea, _) => match nmea::parse(f) {
                    nmea::Info::Zda(label) => {
                        zda = true;
                        l.on_zda(label);
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
        self.rx_frame(kind, msg, tim_tp, zda, now, port, out);
    }

    /// Called on VER, or after giving up on it.
    fn after_version(&mut self, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let running = self.firmware.as_ref().map(|f| f.elf_sha256.clone());
        if let Some(img) = self.cfg.firmware.clone() {
            if running.as_ref().is_some_and(|r| *r != img.elf_sha256) {
                port.write(&nmea::command(&format!("OTA,{},{}", img.data.len(), img.sha256)));
                self.rx = Rx::OtaStart { since: now };
                let from = self.firmware.as_ref().map_or("?", |f| f.version.as_str()).to_string();
                self.status("updating", Some(format!("firmware {from} -> {}", img.version)), out);
                return;
            }
        }
        if self.cfg.configure {
            self.start_probe(now, port, out);
        } else {
            self.set_baud(self.cfg.initial_baud, port);
            self.rx = Rx::Passive { last_tp: None };
            self.status("passive", None, out);
        }
    }

    fn on_ota(&mut self, m: OtaMsg, now: Ns, out: &mut Output) {
        match (&mut self.rx, m) {
            (Rx::OtaStart { .. }, OtaMsg::Ready) => {
                self.rx = Rx::OtaSend { sent: 0, acked: 0, since: now, reported: 0 };
            }
            (Rx::OtaSend { acked, since, .. }, OtaMsg::Ack(n)) => {
                *acked = n as usize;
                *since = now;
            }
            (Rx::OtaSend { .. }, OtaMsg::Ok) => {
                self.rx = Rx::OtaDone { since: now };
                self.status("restarting", Some("firmware written, device restarting".into()), out);
            }
            _ => {}
        }
    }

    fn ota_failed(&mut self, why: &str, now: Ns, port: &mut dyn Port, out: &mut Output) {
        self.status("update_failed", Some(why.to_string()), out);
        self.cfg.firmware = None;
        self.after_version(now, port, out);
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

    /// Target baud first: the receiver may already be set up.
    fn bauds(&self) -> Vec<u32> {
        let mut b = vec![self.cfg.baud];
        if self.cfg.initial_baud != self.cfg.baud {
            b.push(self.cfg.initial_baud);
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
        if self.failures >= MAX_FAILURES {
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
            let p = self.polled_port.as_ref().map_or(1, |p| p[0]);
            let p = if p == 2 { 2 } else { 1 };
            let mut items: Vec<(u32, u32)> = ubx::ENABLED.iter().map(|&id| (ubx::key::msgout(id, p), 1)).collect();
            items.push((ubx::key::uart_out_ubx(p), 1));
            items.push((ubx::key::uart_out_nmea(p), 0));
            let diag_items: Vec<(u32, u32)> = diag.iter().map(|&(id, r)| (ubx::key::msgout(id, p), u32::from(r))).collect();
            let names: Vec<String> = diag.iter().map(|&(id, _)| ubx_name(id)).collect();
            vec![
                Step::required(ubx::valset_ram(&items), Some(ubx::CFG_VALSET)),
                Step::optional(ubx::valset_ram(&diag_items), ubx::CFG_VALSET, names.join(", ")),
                Step::required(ubx::valset_ram(&[(ubx::key::uart_baud(p), target)]), None),
            ]
        } else {
            let mut s: Vec<Step> = ubx::ENABLED.iter().map(|&id| Step::required(ubx::cfg_msg(id, 1), Some(ubx::CFG_MSG))).collect();
            s.extend(diag.iter().map(|&(id, r)| Step::optional(ubx::cfg_msg(id, r), ubx::CFG_MSG, ubx_name(id))));
            s.push(Step::required(ubx::cfg_prt(self.polled_port.as_deref(), target), None));
            s
        };
        self.rx = Rx::Configure { steps, since: now, tries: 0, sends: 0, stale: 0 };
        self.status("configuring", None, out);
        self.send_step(now, port);
    }

    fn skip_step(&mut self, why: &str, now: Ns, port: &mut dyn Port, out: &mut Output) {
        let Rx::Configure { steps, tries, sends, stale, .. } = &mut self.rx else { return };
        let what = steps.remove(0).optional.unwrap_or_default();
        (*tries, *stale, *sends) = (0, *sends - 1, 0);
        self.status("configuring", Some(format!("receiver {why} {what}; continuing without")), out);
        self.send_step(now, port);
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
            Rx::Idle | Rx::Switch { .. } | Rx::Version { .. } | Rx::OtaStart { .. } | Rx::OtaSend { .. } | Rx::OtaDone { .. } => {}
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
            Rx::Configure { steps, tries, sends, stale, .. } => match msg {
                Some((ubx::ACK_ACK, p)) if steps[0].ack.is_some_and(|a| p == [a.0, a.1]) => {
                    if *stale > 0 {
                        // Late ACK for a retry of the previous step.
                        *stale -= 1;
                        return;
                    }
                    steps.remove(0);
                    (*tries, *stale, *sends) = (0, *sends - 1, 0);
                    self.send_step(now, port);
                }
                Some((ubx::ACK_NAK, p)) if steps[0].ack.is_some_and(|a| p == [a.0, a.1]) => {
                    if steps[0].optional.is_some() {
                        self.skip_step("rejected", now, port, out);
                    } else {
                        self.fail("receiver rejected a configuration message", now, port, out);
                    }
                }
                _ => {}
            },
            Rx::Verify { .. } => {
                if tim_tp {
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
                        self.status("probing", Some(format!("no receiver data at {} baud", list.join(" or "))), out);
                    }
                }
            }
            Rx::Identify { since, tries } => {
                if now - *since >= POLL_WAIT {
                    if *tries >= POLL_TRIES {
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
            Rx::Configure { since, tries, steps, .. } => {
                if now - *since >= POLL_WAIT {
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
                    self.fail("no UBX-TIM-TP after configuration", now, port, out);
                }
            }
            Rx::Running { last_frame, last_tp } => {
                if now - *last_frame >= FRAME_TIMEOUT {
                    self.fail("receiver stopped sending", now, port, out);
                } else if now - *last_tp >= TIM_TP_TIMEOUT {
                    self.fail("no UBX-TIM-TP", now, port, out);
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

    /// A sofgps device plus receiver. GPS data only gets through when both use the same baud.
    struct Fake {
        host_baud: u32,
        rx_baud: u32,
        /// u-blox 9/10 (CFG-VALSET only); false = u-blox 5..8.
        gen9: bool,
        /// false: NMEA-only receiver.
        ublox: bool,
        nmea_out: bool,
        ubx_out: bool,
        enabled: BTreeSet<ubx::Id>,
        to_host: Vec<u8>,
        from_host: Framer,
        elf: String,
        /// Image bytes still expected, and those received.
        ota_left: usize,
        ota_image: Vec<u8>,
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
        now_ms: i64,
    }

    impl Fake {
        fn new(rx_baud: u32, gen9: bool, ublox: bool) -> Self {
            Self {
                host_baud: 9600,
                rx_baud,
                gen9,
                ublox,
                nmea_out: true,
                ubx_out: true,
                enabled: BTreeSet::new(),
                to_host: vec![],
                from_host: Framer::new(),
                elf: "00".repeat(32),
                ota_left: 0,
                ota_image: vec![],
                answers: true,
                slow_cfg_msg: None,
                delay_next_ack_ms: 0,
                delayed: vec![],
                lose_cfg_msg: None,
                nak: BTreeSet::new(),
                now_ms: 0,
            }
        }

        fn gps(&mut self, bytes: &[u8]) {
            if self.host_baud == self.rx_baud {
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
                    c[0] = 1;
                    c[8..12].copy_from_slice(&self.rx_baud.to_le_bytes());
                    c[12] = 7;
                    c[14] = self.ubx_out as u8 | (self.nmea_out as u8) << 1;
                    self.gps(&ubx::frame(ubx::CFG_PRT, &c));
                    self.ack(id, true);
                }
                ubx::CFG_PRT => {
                    self.ack(id, true);
                    self.rx_baud = u32::from_le_bytes(p[8..12].try_into().unwrap());
                    self.ubx_out = p[14] & 1 != 0;
                    self.nmea_out = p[14] & 2 != 0;
                }
                ubx::CFG_MSG if !self.gen9 => {
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
                    self.ack(id, true);
                    let mut i = 4;
                    while i < p.len() {
                        let k = u32::from_le_bytes(p[i..i + 4].try_into().unwrap());
                        let n = if k >> 28 == 4 { 4 } else { 1 };
                        let mut v = [0u8; 4];
                        v[..n].copy_from_slice(&p[i + 4..i + 4 + n]);
                        let v = u32::from_le_bytes(v);
                        i += 4 + n;
                        match k {
                            0x4052_0001 => self.rx_baud = v,
                            0x1074_0001 => self.ubx_out = v != 0,
                            0x1074_0002 => self.nmea_out = v != 0,
                            _ => {
                                for id in ubx::ENABLED {
                                    if ubx::key::msgout(id, 1) == k && v > 0 {
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
            let (s, r) = (ms.div_euclid(1000), ms.rem_euclid(1000));
            if r == 700 && self.ublox && self.ubx_out {
                let gps = UTC0_S + s + 1 - 315_964_800 + LEAP;
                let (week, tow) = (gps / 604_800, gps % 604_800);
                for id in self.enabled.clone() {
                    let mut p = vec![0u8; 16];
                    match id {
                        ubx::TIM_TP => {
                            p[..4].copy_from_slice(&((tow * 1000) as u32).to_le_bytes());
                            p[12..14].copy_from_slice(&(week as u16).to_le_bytes());
                        }
                        ubx::NAV_TIMEGPS => {
                            p[10] = LEAP as u8;
                            p[11] = 0x07;
                        }
                        _ => {
                            p[4] = 3;
                            p[5] = 0x0D;
                        }
                    }
                    self.gps(&ubx::frame(id, &p));
                }
            }
            if r == 0 {
                self.to_host.extend(nmea::command(&format!("PPS,{},300", ms * 80_000 + 1000)));
            }
            if r == 100 && self.nmea_out {
                let t = UTC0_S + s;
                let sod = t.rem_euclid(86_400);
                let hms = format!("{:02}{:02}{:02}.00", sod / 3600, sod / 60 % 60, sod % 60);
                // 2026-09-21 is day 20_717.
                assert_eq!(t.div_euclid(86_400), 20_717, "test date");
                self.gps(&nmea::sentence(&format!("GPGGA,{hms},1,N,2,E,1,08,0.9,10,M,0,M,,")));
                self.gps(&nmea::sentence(&format!("GPZDA,{hms},21,09,2026,00,00")));
            }
            if r % 100 == 50 {
                self.to_host.extend(nmea::command(&format!("SOF,{},{}", ms * 80_000 + 1000, ms)));
            }
        }
    }

    impl Port for Fake {
        fn write(&mut self, mut data: &[u8]) {
            if self.ota_left > 0 {
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
                    let fl = nmea::fields(&f);
                    if fl[2] == "INFO" && fl.len() > 3 {
                        self.to_host.extend(nmea::command(&format!("INFO,80000000,100,0,0,0,0,0,{}", fl[3])));
                    } else if fl[2] == "VER" {
                        self.to_host.extend(nmea::command(&format!("VER,v1,v5.5.1,ota_0,0,{},POWERON", self.elf)));
                    } else if fl[2] == "OTA" {
                        self.ota_left = fl[3].parse().unwrap();
                        self.to_host.extend(nmea::command("OTA,READY"));
                    }
                } else if k == Kind::Ubx && self.ublox && self.answers && self.host_baud == self.rx_baud {
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
            ubx::ENABLED.into_iter().chain([ubx::NAV_SOL, ubx::NAV_TIMEUTC, ubx::NAV_SVINFO, ubx::MON_HW]).collect();
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
    fn receiver_at_unexpected_baud_is_not_found() {
        let mut r = Run::new(Fake::new(115_200, false, true));
        r.for_secs(10);
        assert_eq!(r.session.state(), "probing");
        assert!(r.out.events.iter().any(|e| e["message"] == "no receiver data at 38400 or 9600 baud"));
    }

    #[test]
    fn updates_firmware_that_differs() {
        let img = Arc::new(super::super::firmware::test_image("v2", 0xcd, 200_000));
        let cfg = Config { firmware: Some(img.clone()), ..Config::default() };
        let mut r = Run::with(cfg, Fake::new(9600, false, true));
        r.for_secs(5);
        assert_eq!(r.session.state(), "restarting", "{:?}", r.states());
        // The image arrived intact: nothing (PING, setup) was mixed into it.
        assert!(r.fake.ota_image == img.data);
        assert!(r.out.events.iter().any(|e| e["state"] == "updating" && e["message"] == "firmware v1 -> v2"));
        // The device never restarts here: the session gives up and carries on.
        r.for_secs(20);
        assert!(r.states().contains(&"update_failed".to_string()));
        assert_eq!(r.session.state(), "ubx");
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
