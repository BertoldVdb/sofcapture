//! Power settings held while sampling: deep sleep states and slow clock ramps widen the brackets.

use log::{info, warn};
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

#[derive(Debug, Clone, Default)]
pub struct Settings {
    pub max_cpu_latency_us: Option<u32>,
    pub cpu_governor: Option<String>,
    /// If set, the governor is only changed on this CPU.
    pub cpu: Option<usize>,
}

/// Firmware turbo switch as seen by intel_pstate.
const NO_TURBO: &str = "/sys/devices/system/cpu/intel_pstate/no_turbo";

/// Dropping it restores the settings.
pub struct Held {
    /// The latency limit lasts while this stays open.
    _latency: Option<File>,
    governor: Option<(String, Vec<PathBuf>)>,
    /// Files we changed, with their old value.
    restore: Vec<(PathBuf, String)>,
    /// `no_turbo` when the governor was last set.
    no_turbo: Option<String>,
}

fn no_turbo() -> Option<String> {
    std::fs::read_to_string(NO_TURBO).ok().map(|s| s.trim().to_string())
}

fn cpus() -> Vec<usize> {
    let online = std::fs::read_to_string("/sys/devices/system/cpu/online").unwrap_or_default();
    let mut v = vec![];
    for part in online.trim().split(',').filter(|p| !p.is_empty()) {
        let (a, b) = part.split_once('-').unwrap_or((part, part));
        if let (Ok(a), Ok(b)) = (a.parse::<usize>(), b.parse::<usize>()) {
            v.extend(a..=b);
        }
    }
    v
}

impl Held {
    pub fn apply(s: &Settings) -> Option<Self> {
        if s.max_cpu_latency_us.is_none() && s.cpu_governor.is_none() {
            return None;
        }
        let latency = s.max_cpu_latency_us.and_then(|us| {
            let r = File::options()
                .write(true)
                .open("/dev/cpu_dma_latency")
                .and_then(|mut f| f.write_all(&(us.min(i32::MAX as u32) as i32).to_ne_bytes()).map(|()| f));
            match r {
                Ok(f) => {
                    info!("CPU wake-up latency limited to {us} µs while sampling");
                    Some(f)
                }
                Err(e) => {
                    warn!("/dev/cpu_dma_latency: {e}");
                    None
                }
            }
        });
        let mut held = Self { _latency: latency, governor: None, restore: vec![], no_turbo: no_turbo() };
        if let Some(g) = &s.cpu_governor {
            let mut paths = vec![];
            for c in s.cpu.map_or_else(cpus, |c| vec![c]) {
                let path = PathBuf::from(format!("/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_governor"));
                let Ok(old) = std::fs::read_to_string(&path) else { continue };
                let old = old.trim().to_string();
                // Write even if unchanged: it makes the driver recompute its turbo limits.
                match std::fs::write(&path, g) {
                    Ok(()) => {
                        if old != *g {
                            held.restore.push((path.clone(), old));
                        }
                        paths.push(path);
                    }
                    Err(e) => warn!("cpu{c}: cpufreq governor {g}: {e}"),
                }
            }
            if !paths.is_empty() {
                info!("cpufreq governor {g} on {} CPU(s) while sampling", paths.len());
                held.governor = Some((g.clone(), paths));
            }
        }
        Some(held)
    }

    /// Call periodically. intel_pstate (HWP) only picks up a turbo change when the
    /// governor is set again; otherwise it can stay stuck at base clock.
    pub fn refresh(&mut self) {
        let Some((g, paths)) = &self.governor else { return };
        let now = no_turbo();
        if now == self.no_turbo {
            return;
        }
        info!("turbo {}: re-applying cpufreq governor {g}", if now.as_deref() == Some("1") { "disabled" } else { "enabled" });
        for path in paths {
            if let Err(e) = std::fs::write(path, g) {
                warn!("{}: {g}: {e}", path.display());
            }
        }
        self.no_turbo = now;
    }
}

impl Drop for Held {
    fn drop(&mut self) {
        for (path, old) in &self.restore {
            if let Err(e) = std::fs::write(path, old) {
                warn!("{}: restoring {old}: {e}", path.display());
            }
        }
    }
}
