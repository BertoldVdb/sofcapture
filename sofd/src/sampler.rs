//! One thread per subscribed controller: sleeps between samples, takes a
//! timing sample through its backend, unwraps and fits it, and emits events.

use crate::api::Msg;
use crate::backend::{Backend, HS_UNIT_NS};
use crate::clock::{self, Ns, RealtimeSync};
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

/// Counter epochs are unique per process; the unwrapped counter restarts
/// whenever the epoch changes (new subscription, speed change, controller reset).
static NEXT_EPOCH: AtomicU64 = AtomicU64::new(1);

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
}

/// A 1 ms frame boundary, where full-speed devices see SOF.
#[derive(Serialize)]
struct FrameStart {
    bus_frame: u32,
    counter: i64,
    t_raw_ns: Option<Ns>,
    t_realtime_ns: Option<Ns>,
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
    ) -> std::io::Result<Self> {
        let stop = Arc::new(AtomicBool::new(false));
        let period_ns = Arc::new(AtomicI64::new(period_for(rate_hz)));
        let (s, p) = (stop.clone(), period_ns.clone());
        let thread = std::thread::Builder::new()
            .name(format!("sof-{controller}"))
            .spawn(move || run(&controller, backend, &cfg, &s, &p, &out))?;
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
) {
    apply_scheduling(cfg);
    let new_epoch = || NEXT_EPOCH.fetch_add(1, Ordering::Relaxed);
    let mut epoch = new_epoch();
    let mut model = Model::new(cfg.window);
    let mut fit: Option<Fit> = None;
    let mut model_rt = Model::new(cfg.window);
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
                    if out.send(Msg::Out(ev.to_string())).is_err() {
                        return;
                    }
                    last_error = Some(msg);
                }
                // The counter may have restarted underneath us.
                (prev, fit, epoch) = (None, None, new_epoch());
                model.clear();
                model_rt.clear();
                next = clock::mono() + period.max(ERROR_BACKOFF_NS);
                wake = next;
                continue;
            }
        };
        if last_error.take().is_some() {
            let ev = json!({"event": "recovered", "controller": id});
            if out.send(Msg::Out(ev.to_string())).is_err() {
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
                if out.send(Msg::Out(ev.to_string())).is_err() {
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
        let frame_start = (t_raw_fs.is_some() || t_rt_fs.is_some()).then(|| FrameStart {
            bus_frame: if hs { raw >> 3 } else { raw } & 0x7ff,
            counter: frame_counter,
            t_raw_ns: t_raw_fs,
            t_realtime_ns: t_rt_fs,
        });
        let ev = SampleEvent {
            event: "sample",
            controller: id,
            epoch,
            counter,
            unit_ns: unit,
            bus_frame: if hs { raw >> 3 } else { raw } & 0x7ff,
            microframe: hs.then_some(raw & 7),
            t_raw_ns: s.t_ns,
            t_mono_ns: s.t_ns + mono_off,
            t_realtime_ns: s.t_ns + real_off,
            unc_ns: s.unc_ns,
            missed,
            mono_minus_raw_ns: mono_off,
            realtime_minus_raw_ns: real_off,
            model_raw: fit.as_ref(),
            model_realtime: fit_rt.as_ref(),
            realtime_sync: clock::realtime_sync(),
            frame_start,
        };
        missed = 0;
        let line = serde_json::to_string(&ev).expect("sample event serializes");
        if out.send(Msg::Out(line)).is_err() {
            return;
        }
    }
}
