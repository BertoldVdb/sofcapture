//! Labels each PPS edge with its UTC second and converts SOF tags to UTC by
//! interpolating between the edges around them.
//!
//! UBX-TIM-TP arrives before the pulse it describes, NMEA ZDA after it.
//! A label is only trusted if the GPS line was idle at both pulses around it,
//! so it can't have been in flight across a pulse.

use crate::clock::Ns;
use serde::Serialize;
use std::collections::VecDeque;

pub const MIN_IDLE_MS: u32 = 10;
/// Allowed pulse interval error, as a fraction of a second (crystal ≈ 10 ppm).
const INTERVAL_TOL: f64 = 200e-6;
const FIX_MAX_AGE: u64 = 2;
const SECOND: Ns = 1_000_000_000;
/// 12.5 ns capture step plus ~46 ns rms spin jitter.
pub const TAG_UNC_NS: Ns = 50;
/// Labelling needs every tag back to the previous pulse, 2 s with ZDA: 4 s at div 1.
const MAX_PENDING: usize = 4096;
/// Measured on a LEA-5T: 3.9 ns rms edge noise with -qErr, 5.0 ns without, 8.0 ns with +qErr.
pub const QERR_SIGN: i64 = -1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    TimTp,
    Zda,
    /// An MQTT client's own tags.
    External,
}

#[derive(Debug, Clone, Serialize)]
pub struct PpsDecision {
    pub tick: u64,
    pub rx_idle_ms: u32,
    pub source: Source,
    /// UTC of the edge (qErr applied); null when no label was available.
    pub utc_ns: Option<Ns>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub qerr_ps: Option<i32>,
    pub accepted: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<&'static str>,
    #[serde(skip)]
    idx: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum At {
    /// Device frame number (unwrapped or 11-bit) and microframe (high speed).
    Frame { frame: u64, microframe: u8 },
    /// sofd's own counter, as the client read it from the model topic.
    Counter { epoch: u64, counter: i64 },
}

#[derive(Debug, Clone)]
pub struct Tag {
    pub at: At,
    pub utc_ns: Ns,
    pub unc_ns: Ns,
    /// CLOCK_MONOTONIC_RAW when the tag's sentence was read.
    pub t_rx: Ns,
    pub source: Source,
}

/// Pulse UTC as sent (before qErr) and qErr.
type Label = Result<(Ns, Option<i32>), &'static str>;

fn edge(label: Ns, qerr_ps: Option<i32>) -> Ns {
    label + QERR_SIGN * qerr_ps.unwrap_or(0) as Ns / 1000
}

struct Pulse {
    idx: u64,
    tick: u64,
    rx_idle_ms: u32,
    label: Option<Ns>,
}

pub struct Labeler {
    tick_hz: f64,
    source: Source,
    idx: u64,
    /// TIM-TP: the previous pulse. ZDA: the pulse waiting for its label.
    prev: Option<Pulse>,
    /// ZDA: label of the pulse before `prev`.
    prev_label: Option<Ns>,
    /// Labels received since the last pulse.
    between: Vec<Label>,
    fix: Option<(bool, u64)>,
    leap_s: Option<i8>,
    last_ok: Option<(u64, u64, Ns)>,
    /// SOF tags waiting for the pulse after them: (tick, frame, t_rx).
    pending: VecDeque<(u64, u64, Ns)>,
}

impl Labeler {
    pub fn new(tick_hz: u32, source: Source) -> Self {
        Self {
            tick_hz: tick_hz as f64,
            source,
            idx: 0,
            prev: None,
            prev_label: None,
            between: vec![],
            fix: None,
            leap_s: None,
            last_ok: None,
            pending: VecDeque::new(),
        }
    }

    pub fn source(&self) -> Source {
        self.source
    }

    /// Keeps fix and leap seconds.
    pub fn reset(&mut self) {
        self.prev = None;
        self.prev_label = None;
        self.between.clear();
        self.last_ok = None;
        self.pending.clear();
    }

    pub fn set_source(&mut self, s: Source) {
        if s != self.source {
            self.source = s;
            self.reset();
        }
    }

    pub fn set_tick_hz(&mut self, hz: u32) {
        self.tick_hz = hz as f64;
    }

    pub fn on_fix(&mut self, ok: bool) {
        self.fix = Some((ok, self.idx));
    }

    pub fn on_leap(&mut self, leap_s: Option<i8>) {
        self.leap_s = leap_s;
    }

    pub fn on_tim_tp(&mut self, tp: &super::ubx::TimTp) {
        if self.source == Source::TimTp {
            let q = tp.qerr();
            self.between.push(tp.utc_ns(self.leap_s).map(|t| (t, q)));
        }
    }

    pub fn on_zda(&mut self, label: Result<Ns, &'static str>) {
        if self.source == Source::Zda {
            self.between.push(label.map(|t| (t, None)));
        }
    }

    pub fn on_sof(&mut self, tick: u64, frame: u64, t_rx: Ns) {
        if self.pending.len() == MAX_PENDING {
            self.pending.pop_front();
        }
        self.pending.push_back((tick, frame, t_rx));
    }

    pub fn on_pps(&mut self, tick: u64, rx_idle_ms: u32, decisions: &mut Vec<PpsDecision>, tags: &mut Vec<Tag>) {
        self.idx += 1;
        let between = std::mem::take(&mut self.between);
        let label: Label = match (between.len(), self.source) {
            (1, _) => between[0],
            (0, Source::TimTp) => Err("no_tim_tp"),
            (0, Source::Zda | Source::External) => Err("no_zda"),
            (_, Source::TimTp) => Err("multiple_tim_tp"),
            (_, Source::Zda | Source::External) => Err("multiple_zda"),
        };
        let fix_ok = matches!(self.fix, Some((true, i)) if self.idx - i <= FIX_MAX_AGE);
        let interval_ok =
            |a: &Pulse| ((tick.wrapping_sub(a.tick)) as f64 - self.tick_hz).abs() <= self.tick_hz * INTERVAL_TOL;
        let cur = Pulse { idx: self.idx, tick, rx_idle_ms, label: None };

        match self.source {
            Source::TimTp => {
                // TIM-TP describes this pulse.
                let (cand, qerr) = label.map_or((None, None), |(t, q)| (Some(t), q));
                let reason = match &self.prev {
                    None => Some("first_pulse"),
                    Some(p) if !interval_ok(p) => Some("interval"),
                    Some(p) if p.rx_idle_ms < MIN_IDLE_MS || rx_idle_ms < MIN_IDLE_MS => Some("line_busy"),
                    Some(_) if label.is_err() => label.err(),
                    Some(_) if !fix_ok => Some("no_fix"),
                    Some(p) if p.label.zip(cand).is_none_or(|(a, b)| b - a != SECOND) => Some("label_jump"),
                    Some(_) => None,
                };
                let d = PpsDecision {
                    tick,
                    rx_idle_ms,
                    source: Source::TimTp,
                    utc_ns: cand.map(|t| edge(t, qerr)),
                    qerr_ps: qerr,
                    accepted: reason.is_none(),
                    reason,
                    idx: self.idx,
                };
                self.prev = Some(Pulse { label: cand, ..cur });
                self.decided(d, decisions, tags);
            }
            // External tags never reach a labeler.
            Source::Zda | Source::External => {
                // ZDA describes the previous pulse.
                if let Some(p) = self.prev.take() {
                    let cand = label.ok().map(|l| l.0);
                    let reason = if !interval_ok(&p) {
                        Some("interval")
                    } else if p.rx_idle_ms < MIN_IDLE_MS || rx_idle_ms < MIN_IDLE_MS {
                        Some("line_busy")
                    } else if label.is_err() {
                        label.err()
                    } else if !fix_ok {
                        Some("no_fix")
                    } else if self.prev_label.zip(cand).is_none_or(|(a, b)| b - a != SECOND) {
                        Some("label_jump")
                    } else {
                        None
                    };
                    self.prev_label = cand;
                    let d = PpsDecision {
                        tick: p.tick,
                        rx_idle_ms: p.rx_idle_ms,
                        source: Source::Zda,
                        utc_ns: cand,
                        qerr_ps: None,
                        accepted: reason.is_none(),
                        reason,
                        idx: p.idx,
                    };
                    self.decided(d, decisions, tags);
                }
                self.prev = Some(cur);
            }
        }
    }

    fn decided(&mut self, d: PpsDecision, decisions: &mut Vec<PpsDecision>, tags: &mut Vec<Tag>) {
        if d.accepted {
            let utc = d.utc_ns.expect("accepted pulses have a label");
            if let Some((_, t0, u0)) = self.last_ok.filter(|l| l.0 + 1 == d.idx) {
                let scale = (utc - u0) as f64 / (d.tick - t0) as f64;
                for &(tick, frame, t_rx) in self.pending.iter().filter(|p| p.0 > t0 && p.0 <= d.tick) {
                    tags.push(Tag {
                        at: At::Frame { frame, microframe: 0 },
                        utc_ns: u0 + ((tick - t0) as f64 * scale).round() as Ns,
                        unc_ns: TAG_UNC_NS,
                        t_rx,
                        source: d.source,
                    });
                }
            }
            self.last_ok = Some((d.idx, d.tick, utc));
        } else {
            self.last_ok = None;
        }
        self.pending.retain(|p| p.0 > d.tick);
        decisions.push(d);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss::ubx::TimTp;

    const HZ: u32 = 80_000_000;
    const UTC0: Ns = 1_790_000_000 * SECOND;

    fn tp(utc: Ns) -> TimTp {
        // UTC time base, so leap seconds don't matter.
        let s = utc / SECOND - 315_964_800;
        TimTp {
            tow_ms: ((s % 604_800) * 1000) as u32,
            tow_sub_ms: 0,
            qerr_ps: (s % 7 * 3000 - 9000) as i32,
            week: (s / 604_800) as u16,
            flags: 3,
            ref_info: 0,
        }
    }

    /// Runs `n` seconds; `label(k)` decides what is sent around pulse k.
    fn run(source: Source, n: u64, idle: impl Fn(u64) -> u32, mut label: impl FnMut(u64) -> u32) -> (Vec<PpsDecision>, Vec<Tag>) {
        let mut l = Labeler::new(HZ, source);
        let (mut d, mut t) = (vec![], vec![]);
        for k in 0..n {
            let tick = 1_000 + k * HZ as u64;
            let utc = UTC0 + k as Ns * SECOND;
            l.on_fix(true);
            if source == Source::TimTp {
                for _ in 0..label(k) {
                    l.on_tim_tp(&tp(utc));
                }
            }
            l.on_pps(tick, idle(k), &mut d, &mut t);
            if source == Source::Zda {
                for _ in 0..label(k) {
                    l.on_zda(Ok(utc));
                }
            }
            for j in 0..10 {
                l.on_sof(tick + j * 8_000_000 + 4_000_000, k * 1000 + j * 100 + 50, 0);
            }
        }
        (d, t)
    }

    #[test]
    fn tim_tp_labels_and_interpolates() {
        let (d, t) = run(Source::TimTp, 5, |_| 300, |_| 1);
        let ok: Vec<bool> = d.iter().map(|d| d.accepted).collect();
        assert_eq!(ok, [false, true, true, true, true]);
        assert_eq!(d[0].reason, Some("first_pulse"));
        // Tags between accepted pulses 1-2, 2-3, 3-4.
        assert_eq!(t.len(), 30);
        let first = &t[0];
        assert_eq!(first.at, At::Frame { frame: 1050, microframe: 0 });
        let q = |k: i64| QERR_SIGN * ((UTC0 / SECOND + k - 315_964_800) % 7 * 3000 - 9000) / 1000;
        assert_eq!(d[1].utc_ns, Some(UTC0 + SECOND + q(1)));
        assert_eq!(first.utc_ns, UTC0 + SECOND + 50_000_000 + (q(1) + (q(2) - q(1)) / 20));
    }

    #[test]
    fn busy_line_rejects_both_neighbours() {
        let (d, t) = run(Source::TimTp, 6, |k| if k == 3 { 4 } else { 300 }, |_| 1);
        let ok: Vec<bool> = d.iter().map(|d| d.accepted).collect();
        assert_eq!(ok, [false, true, true, false, false, true]);
        assert_eq!(d[3].reason, Some("line_busy"));
        // Only interval 1-2 yields tags.
        assert_eq!(t.len(), 10);
    }

    #[test]
    fn missing_or_duplicate_tim_tp_rejected() {
        let (d, _) = run(Source::TimTp, 5, |_| 300, |k| match k {
            2 => 0,
            3 => 2,
            _ => 1,
        });
        assert_eq!(d[2].reason, Some("no_tim_tp"));
        assert_eq!(d[3].reason, Some("multiple_tim_tp"));
        // Pulse 3 had no single label, so 4 can't check continuity.
        assert_eq!(d[4].reason, Some("label_jump"));
    }

    #[test]
    fn missing_pulse_rejected() {
        let mut l = Labeler::new(HZ, Source::TimTp);
        let (mut d, mut t) = (vec![], vec![]);
        for k in [0u64, 1, 3] {
            l.on_fix(true);
            l.on_tim_tp(&tp(UTC0 + k as Ns * SECOND));
            l.on_pps(k * HZ as u64, 300, &mut d, &mut t);
        }
        assert_eq!(d[2].reason, Some("interval"));
    }

    #[test]
    fn zda_labels_previous_pulse() {
        let (d, t) = run(Source::Zda, 6, |_| 300, |_| 1);
        // Decided one pulse late: pulses 0..4.
        let ok: Vec<bool> = d.iter().map(|d| d.accepted).collect();
        assert_eq!(ok, [false, true, true, true, true]);
        assert_eq!(d[1].utc_ns, Some(UTC0 + SECOND));
        assert_eq!(t.len(), 30);
        assert_eq!(t[0].utc_ns, UTC0 + SECOND + 50_000_000);
    }

    #[test]
    fn no_fix_rejected() {
        let mut l = Labeler::new(HZ, Source::TimTp);
        let (mut d, mut t) = (vec![], vec![]);
        l.on_fix(false);
        for k in 0..3u64 {
            l.on_tim_tp(&tp(UTC0 + k as Ns * SECOND));
            l.on_pps(k * HZ as u64, 300, &mut d, &mut t);
        }
        assert_eq!(d[2].reason, Some("no_fix"));
    }

    #[test]
    fn keeps_every_tag_at_div_1() {
        let mut l = Labeler::new(HZ, Source::Zda);
        let (mut d, mut t) = (vec![], vec![]);
        for k in 0..4u64 {
            let tick = k * HZ as u64;
            l.on_fix(true);
            l.on_pps(tick, 300, &mut d, &mut t);
            l.on_zda(Ok(UTC0 + k as Ns * SECOND));
            for j in 0..1000 {
                l.on_sof(tick + j * 80_000 + 40_000, k * 1000 + j, 0);
            }
        }
        // Pulse 2 is decided at pulse 3 and needs the tags after pulse 1.
        assert_eq!(t.len(), 1000);
    }
}
