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

/// Dropping it restores the settings.
pub struct Held {
    /// The latency limit lasts while this stays open.
    _latency: Option<File>,
    /// cpufreq governor files and their previous value.
    governors: Vec<(PathBuf, String)>,
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
        let mut governors = vec![];
        if let Some(g) = &s.cpu_governor {
            let targets = s.cpu.map_or_else(cpus, |c| vec![c]);
            for c in targets {
                let path = PathBuf::from(format!("/sys/devices/system/cpu/cpu{c}/cpufreq/scaling_governor"));
                let Ok(old) = std::fs::read_to_string(&path) else { continue };
                let old = old.trim().to_string();
                if old == *g {
                    continue;
                }
                match std::fs::write(&path, g) {
                    Ok(()) => governors.push((path, old)),
                    Err(e) => warn!("cpu{c}: cpufreq governor {g}: {e}"),
                }
            }
            if !governors.is_empty() {
                info!("cpufreq governor {g} on {} CPU(s) while sampling", governors.len());
            }
        }
        Some(Self { _latency: latency, governors })
    }
}

impl Drop for Held {
    fn drop(&mut self) {
        for (path, old) in &self.governors {
            if let Err(e) = std::fs::write(path, old) {
                warn!("{}: restoring {old}: {e}", path.display());
            }
        }
    }
}
