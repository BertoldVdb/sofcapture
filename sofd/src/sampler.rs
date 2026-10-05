//! Per-controller sampling thread: reads the frame counter, fits it, emits events.

use crate::backend::{Backend, HS_UNIT_NS};
use crate::chrony;
use crate::clock::{self, Ns, RealtimeSync};
use crate::daemon::{Msg, SampleOut};
use crate::gnss::join::Join;
use crate::gnss::timing::{At, Source};
use crate::gnss::{Feed, Hub};
use crate::model::{self, Fit, Model};
use log::{debug, warn};
use serde::Serialize;
use serde_json::json;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::JoinHandle;

pub const MIN_RATE_HZ: f64 = 0.1;
pub const MAX_RATE_HZ: f64 = 1000.0;
/// Max single sleep, so a stop request is seen quickly.
const SLEEP_SLICE_NS: Ns = 100_000_000;
const ERROR_BACKOFF_NS: Ns = 1_000_000_000;
/// Retry delay after a stalled sample.
const RETRY_NS: Ns = 5_000_000;
const MAX_RETRIES: u32 = 3;
/// CLOCK_REALTIME moving against CLOCK_MONOTONIC_RAW by more than this
/// (1 ms plus 10% of elapsed time; slewing tops out near 8%) is a step.
const REALTIME_STEP_NS: Ns = 1_000_000;
const REALTIME_MAX_SLEW: f64 = 0.1;
/// Pass GNSS time on only from a fresh, filled fit: never extrapolate through an outage.
const GNSS_MIN_POINTS: usize = 5;
const GNSS_MAX_AGE_NS: Ns = 5_000_000_000;

static LAST_EPOCH: AtomicU64 = AtomicU64::new(0);

/// A new epoch: at least the current Unix time in milliseconds, so epochs
/// stay unique across sofd restarts (unless the clock went back), and
/// increasing within one.
fn new_epoch() -> u64 {
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_millis() as u64);
    let mut epoch = 0;
    let _ = LAST_EPOCH.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |last| {
        epoch = (last + 1).max(now);
        Some(epoch)
    });
    epoch
}

#[derive(Debug, Clone)]
pub struct SamplerConfig {
    pub window: usize,
    pub rt_prio: Option<i32>,
    pub cpu: Option<usize>,
}

#[derive(Serialize)]
struct SampleEvent<'a> {
    event: &'static str,
    controller: &'a str,
    epoch: u64,
    /// Unwrapped frame counter, in units of `unit_ns`.
    counter: i64,
    unit_ns: u32,
    /// 11-bit USB frame number as seen on the bus (matches a device's SOF frame number).
    bus_frame: u32,
    /// Microframe within `bus_frame` (high-speed only).
    #[serde(skip_serializing_if = "Option::is_none")]
    microframe: Option<u32>,
    t_raw_ns: Ns,
    t_mono_ns: Ns,
    t_realtime_ns: Ns,
    unc_ns: Ns,
    /// Samples dropped (stalls, wide brackets) since the last event.
    missed: u32,
    mono_minus_raw_ns: Ns,
    realtime_minus_raw_ns: Ns,
    /// Fit against CLOCK_MONOTONIC_RAW (undisciplined host clock).
    model_raw: Option<&'a Fit>,
    /// Fit against CLOCK_REALTIME; `ppm` is the true error if the clock is disciplined.
    model_realtime: Option<&'a Fit>,
    realtime_sync: Option<RealtimeSync>,
    /// Start of the 1 ms frame containing this sample.
    frame_start: Option<FrameStart>,
    /// Fit against GNSS UTC (Unix ns).
    model_gnss: Option<&'a Fit>,
    gnss: Option<GnssRef<'a>>,
}

#[derive(Serialize)]
struct GnssRef<'a> {
    /// USB serial of the board, or `mqtt:<client>`.
    board: &'a str,
    /// What labelled the newest tag: `tim_tp`, `zda` or `external`.
    source: Option<Source>,
    /// host frame = device frame + frame_offset.
    frame_offset: Option<i64>,
    /// CLOCK_REALTIME minus GNSS UTC at `frame_start`.
    realtime_minus_gnss_ns: Option<Ns>,
}

/// A 1 ms frame boundary, where full-speed devices see SOF.
#[derive(Serialize)]
struct FrameStart {
    bus_frame: u32,
    counter: i64,
    t_raw_ns: Option<Ns>,
    t_realtime_ns: Option<Ns>,
    t_gnss_ns: Option<Ns>,
}

pub struct Sampler {
    stop: Arc<AtomicBool>,
    period_ns: Arc<AtomicI64>,
    thread: Option<JoinHandle<()>>,
}

fn period_for(rate_hz: f64) -> Ns {
    (1e9 / rate_hz.clamp(MIN_RATE_HZ, MAX_RATE_HZ)) as Ns
}

impl Sampler {
    pub fn spawn(
        controller: String,
        backend: Box<dyn Backend>,
        rate_hz: f64,
        cfg: SamplerConfig,
        out: Sender<Msg>,
        hub: Hub,
    ) -> std::io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let period_ns = Arc::new(AtomicI64::new(period_for(rate_hz)));
        let (s, p) = (stop.clone(), period_ns.clone());
        let thread = std::thread::Builder::new()
            .name(format!("sof-{controller}"))
            .spawn(move || run(&controller, backend, &cfg, &s, &p, &out, &hub))?;
        Ok(Self { stop, period_ns, thread: Some(thread) })
    }

    pub fn set_rate(&self, rate_hz: f64) {
        self.period_ns.store(period_for(rate_hz), Ordering::Relaxed);
    }
}

impl Drop for Sampler {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn apply_scheduling(cfg: &SamplerConfig) {
    if let Some(cpu) = cfg.cpu {
        // SAFETY: cpu_set_t is plain data; the calls only read it.
        let r = unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_SET(cpu, &mut set);
            libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set)
        };
        if r != 0 {
            warn!("pinning to cpu {cpu}: {}", std::io::Error::last_os_error());
        }
    }
    if let Some(prio) = cfg.rt_prio {
        // SAFETY: sched_param is plain data (musl adds extra fields).
        let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
        param.sched_priority = prio;
        // SAFETY: pid 0 is the calling thread; param is valid.
        if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) } != 0 {
            warn!("SCHED_FIFO priority {prio}: {}", std::io::Error::last_os_error());
        }
    }
}

fn run(
    id: &str,
    mut backend: Box<dyn Backend>,
    cfg: &SamplerConfig,
    stop: &AtomicBool,
    period_ns: &AtomicI64,
    out: &Sender<Msg>,
    hub: &Hub,
) {
    apply_scheduling(cfg);
    let mut epoch = new_epoch();
    let mut model = Model::new(cfg.window);
    let mut fit: Option<Fit> = None;
    let mut model_rt = Model::new(cfg.window);
    let mut model_gnss = Model::new(cfg.window);
    let mut join = Join::default();
    let mut feed: Option<Arc<Feed>> = None;
    let mut gnss_source: Option<Source> = None;
    // Newest GNSS tag gave sofd's counter directly.
    let mut by_counter = false;
    let mut last_ok = false;
    // Last state reported to an external reference.
    let mut ref_state: Option<&'static str> = None;
    // When the newest GNSS tag was received (raw clock).
    let mut last_tag_rx: Option<Ns> = None;
    // (t_raw, realtime - raw) of the previous sample, for step detection.
    let mut prev_rt_off: Option<(Ns, Ns)> = None;
    let mut prev: Option<(i64, Ns)> = None;
    let mut prev_raw = 0u32;
    let counter_bits = backend.counter_bits();
    let mut unit = 0u32;
    let mut last_error: Option<String> = None;
    let mut next = clock::mono();
    // Sooner than `next` when retrying a miss.
    let mut wake = next;
    let mut retries = 0;
    let mut missed = 0u32;

    loop {
        loop {
            let now = clock::mono();
            if now >= wake || stop.load(Ordering::Relaxed) {
                break;
            }
            clock::sleep_until_mono(wake.min(now + SLEEP_SLICE_NS));
        }
        if stop.load(Ordering::Relaxed) {
            return;
        }
        let period = period_ns.load(Ordering::Relaxed);
        let now = clock::mono();
        if now >= next {
            next += period;
            if next <= now {
                // Fell behind: skip ahead instead of bursting.
                next = now + period;
            }
            retries = 0;
        }
        wake = next;

        let mono_off = clock::offset_from_raw(clock::mono);
        let real_off = clock::offset_from_raw(clock::realtime);
        let s = match backend.sample() {
            Ok(Some(s)) if s.unc_ns <= s.unit_ns as Ns / 4 => s,
            Ok(miss) => {
                // Just a stall: keep the fits, the unwrap bridges the gap.
                if let Some(s) = miss {
                    debug!("{id}: dropping sample with ±{} ns uncertainty", s.unc_ns);
                }
                missed += 1;
                if retries < MAX_RETRIES {
                    retries += 1;
                    wake = (clock::mono() + RETRY_NS).min(next);
                }
                continue;
            }
            Err(e) => {
                let msg = format!("{e:#}");
                if last_error.as_ref() != Some(&msg) {
                    warn!("{id}: {msg}");
                    let ev = json!({"event": "error", "controller": id, "message": msg});
                    if out.send(Msg::Controller { controller: id.into(), event: ev }).is_err() {
                        return;
                    }
                    last_error = Some(msg);
                }
                // The counter may have restarted underneath us.
                (prev, fit, epoch) = (None, None, new_epoch());
                model.clear();
                model_rt.clear();
                model_gnss.clear();
                join.reset();
                next = clock::mono() + period.max(ERROR_BACKOFF_NS);
                wake = next;
                continue;
            }
        };
        if last_error.take().is_some() {
            let ev = json!({"event": "recovered", "controller": id});
            if out.send(Msg::Controller { controller: id.into(), event: ev }).is_err() {
                return;
            }
        }
        if s.unit_ns != unit {
            if unit != 0 {
                epoch = new_epoch();
            }
            (unit, prev, fit) = (s.unit_ns, None, None);
            model.clear();
            model_rt.clear();
            model_gnss.clear();
            join.reset();
        }

        let period_est = fit.as_ref().map_or(unit as f64, |f| f.period_ns);
        let mut counter = model::unwrap(prev, s.counter, s.t_ns, period_est, counter_bits);
        // Far off the model means the counter jumped (reset, resume).
        // With an unknown wrap point, compare raw counts over gaps too short to wrap.
        let jumped = match (&fit, counter_bits, prev) {
            (Some(f), Some(_), _) => (s.t_ns - f.predict(counter)).abs() > unit as Ns / 4,
            (_, None, Some((pc, _))) => {
                let (dc, draw) = (counter - pc, s.counter as i64 - prev_raw as i64);
                dc < 512 && draw >= 0 && draw != dc
            }
            _ => false,
        };
        if jumped {
            warn!("{id}: frame counter discontinuity, starting new epoch");
            epoch = new_epoch();
            model.clear();
            model_rt.clear();
            model_gnss.clear();
            join.reset();
            counter = s.counter as i64;
        }
        prev = Some((counter, s.t_ns));
        prev_raw = s.counter;
        model.push(counter, s.t_ns, s.unc_ns);
        fit = model.fit(unit as f64);

        if let Some((pt, poff)) = prev_rt_off {
            let step = real_off - poff;
            if step.abs() > REALTIME_STEP_NS + ((s.t_ns - pt) as f64 * REALTIME_MAX_SLEW) as Ns {
                warn!("{id}: CLOCK_REALTIME stepped by {step} ns, restarting realtime fit");
                model_rt.clear();
                let ev = json!({"event": "realtime_step", "controller": id, "step_ns": step});
                if out.send(Msg::Controller { controller: id.into(), event: ev }).is_err() {
                    return;
                }
            }
        }
        prev_rt_off = Some((s.t_ns, real_off));
        model_rt.push(counter, s.t_ns + real_off, s.unc_ns);
        let fit_rt = model_rt.fit(unit as f64);

        // Bus frame from the raw count; the unwrapped counter may wrap elsewhere.
        let raw = s.counter;
        let hs = unit == HS_UNIT_NS;
        let frame_counter = if hs { counter - counter.rem_euclid(8) } else { counter };
        let t_raw_fs = fit.as_ref().map(|f| f.predict(frame_counter));
        let t_rt_fs = fit_rt.as_ref().map(|f| f.predict(frame_counter));
        let bus_frame = if hs { raw >> 3 } else { raw } & 0x7ff;

        // GNSS reference: match SOF tags to our frames by bus frame number.
        let attached = hub.lock().unwrap().get(id).cloned();
        if feed.as_ref().map(Arc::as_ptr) != attached.as_ref().map(Arc::as_ptr) {
            feed = attached;
            model_gnss.clear();
            join = Join::new(feed.as_ref().map_or(0, |f| f.max_lag));
            gnss_source = None;
            last_tag_rx = None;
            ref_state = None;
            last_ok = false;
        }
        if let (Some(fd), Some(f)) = (&feed, &fit) {
            let per_frame: i64 = if hs { 8 } else { 1 };
            let frame_now = counter.div_euclid(per_frame);
            let bus_of = |h: i64| (bus_frame as i64 + h - frame_now).rem_euclid(2048) as u32;
            for tag in fd.drain() {
                let matched = match tag.at {
                    At::Frame { frame, microframe } => {
                        let at_rx = f.counter_ref as f64 + (tag.t_rx - f.t_ref_ns) as f64 / f.period_ns;
                        let micro = if hs { i64::from(microframe) } else { 0 };
                        join.host_frame(frame, at_rx / per_frame as f64, bus_of).map(|h| h * per_frame + micro).ok_or("no_match")
                    }
                    At::Counter { epoch: e, counter: c } => if e == epoch { Ok(c) } else { Err("epoch") },
                };
                last_ok = matched.is_ok();
                if let Ok(c) = matched {
                    model_gnss.push(c, tag.utc_ns, tag.unc_ns);
                    gnss_source = Some(tag.source);
                    last_tag_rx = Some(tag.t_rx);
                    by_counter = matches!(tag.at, At::Counter { .. });
                }
                // Tell external references about changes; they can't see why we reject.
                let state = matched.map_or_else(|r| r, |_| "accepted");
                if let Some(client) = fd.client.as_deref().filter(|_| ref_state != Some(state)) {
                    ref_state = Some(state);
                    let ev = match matched {
                        Ok(_) => json!({"event": "ref_accepted", "controller": id, "client": client,
                                        "frame_offset": if by_counter { None } else { join.offset() }}),
                        Err(reason) => json!({"event": "ref_rejected", "controller": id, "client": client, "reason": reason}),
                    };
                    if out.send(Msg::Controller { controller: id.into(), event: ev }).is_err() {
                        return;
                    }
                }
            }
        }
        let fit_gnss = model_gnss.fit(unit as f64);
        let t_gnss_fs = fit_gnss.as_ref().map(|f| f.predict(frame_counter));
        let gnss_valid = fit_gnss.as_ref().is_some_and(|f| f.n >= GNSS_MIN_POINTS)
            && last_ok
            && last_tag_rx.is_some_and(|t| s.t_ns - t < GNSS_MAX_AGE_NS);
        let t_realtime = s.t_ns + real_off;
        let chrony_point = fit_gnss
            .as_ref()
            .filter(|_| gnss_valid)
            .map(|f| chrony::Point { t_realtime_ns: t_realtime, t_gnss_ns: f.predict(counter) });
        let gnss = feed.as_ref().map(|fd| GnssRef {
            board: &fd.serial,
            source: gnss_source,
            frame_offset: if by_counter { None } else { join.offset() },
            realtime_minus_gnss_ns: t_rt_fs.zip(t_gnss_fs).map(|(r, g)| r - g),
        });

        let frame_start = (t_raw_fs.is_some() || t_rt_fs.is_some()).then_some(FrameStart {
            bus_frame,
            counter: frame_counter,
            t_raw_ns: t_raw_fs,
            t_realtime_ns: t_rt_fs,
            t_gnss_ns: t_gnss_fs,
        });
        let ev = SampleEvent {
            event: "sample",
            controller: id,
            epoch,
            counter,
            unit_ns: unit,
            bus_frame,
            microframe: hs.then_some(raw & 7),
            t_raw_ns: s.t_ns,
            t_mono_ns: s.t_ns + mono_off,
            t_realtime_ns: t_realtime,
            unc_ns: s.unc_ns,
            missed,
            mono_minus_raw_ns: mono_off,
            realtime_minus_raw_ns: real_off,
            model_raw: fit.as_ref(),
            model_realtime: fit_rt.as_ref(),
            realtime_sync: clock::realtime_sync(),
            frame_start,
            model_gnss: fit_gnss.as_ref(),
            gnss,
        };
        missed = 0;
        let sample = SampleOut {
            controller: id.to_string(),
            event: serde_json::to_value(&ev).expect("sample event serializes"),
            board: feed.as_ref().map(|f| f.serial.clone()),
            chrony: chrony_point,
        };
        if out.send(Msg::Sample(Box::new(sample))).is_err() {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epochs_follow_unix_ms_and_increase() {
        let t0 = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_millis() as u64;
        let (a, b) = (new_epoch(), new_epoch());
        assert!(a >= t0 && b > a);
    }
}
