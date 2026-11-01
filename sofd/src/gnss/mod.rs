//! sofgnss boards: USB GPS bridges that timestamp PPS and SOF on one counter,
//! giving their controller's frame times in GNSS UTC.

pub mod device;
pub mod external;
pub mod firmware;
pub mod framer;
pub mod gpio;
pub mod join;
pub mod nmea;
pub mod pps;
pub mod session;
pub mod timing;
pub mod ubx;

use crate::clock::Ns;
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};
use timing::Tag;

/// 10 s at div 1, the longest time between samples (0.1 Hz).
const MAX_QUEUED_TAGS: usize = 10_000;

/// Tags from one board or MQTT client, waiting for the sampler.
pub struct Feed {
    /// USB serial of the board, or `mqtt:<client>`.
    pub serial: String,
    /// Set for an MQTT reference.
    pub client: Option<String>,
    /// See `join::BOARD_MAX_LAG`.
    pub max_lag: i64,
    /// Set for a receiver's host clock: the sampler dates its own samples by it.
    pub host: Option<Arc<HostClock>>,
    tags: Mutex<VecDeque<Tag>>,
}

impl Feed {
    pub fn new(serial: String) -> Self {
        Self { serial, client: None, max_lag: join::BOARD_MAX_LAG, host: None, tags: Mutex::new(VecDeque::new()) }
    }

    pub fn external(client: &str) -> Self {
        Self {
            serial: format!("{}{client}", external::NAME_PREFIX),
            client: Some(client.into()),
            max_lag: join::MQTT_MAX_LAG,
            host: None,
            tags: Mutex::new(VecDeque::new()),
        }
    }

    pub fn push(&self, t: Tag) {
        let mut q = self.tags.lock().unwrap();
        if q.len() == MAX_QUEUED_TAGS {
            q.pop_front();
        }
        q.push_back(t);
    }

    pub fn drain(&self) -> Vec<Tag> {
        self.tags.lock().unwrap().drain(..).collect()
    }
}

impl Feed {
    pub fn host(receiver: &str, clock: Arc<HostClock>) -> Self {
        Self { host: Some(clock), ..Self::new(format!("{HOST_PREFIX}{receiver}")) }
    }
}

/// `Feed::serial` of a receiver's host clock.
pub const HOST_PREFIX: &str = "receiver:";

/// GNSS time of the raw clock, from a receiver's accepted pulses.
pub struct HostClock {
    /// (raw ns, UTC ns) of recent pulses.
    pulses: Mutex<VecDeque<(Ns, Ns)>>,
    /// The pulses' timing uncertainty.
    pub unc_ns: Ns,
}

/// Past the last pulse, extrapolate this far at most.
const HOST_EXTRAPOLATE_NS: Ns = 1_500_000_000;

impl HostClock {
    pub fn new(unc_ns: Ns) -> Self {
        Self { pulses: Mutex::new(VecDeque::new()), unc_ns }
    }

    pub fn push(&self, raw: Ns, utc: Ns) {
        let mut p = self.pulses.lock().unwrap();
        if p.back().is_some_and(|&(r, u)| raw <= r || utc <= u) {
            p.clear();
        }
        if p.len() == 8 {
            p.pop_front();
        }
        p.push_back((raw, utc));
    }

    pub fn clear(&self) {
        self.pulses.lock().unwrap().clear();
    }

    /// Between two pulses, or a little past the last one at the last interval's rate.
    pub fn utc_of(&self, raw: Ns) -> Option<Ns> {
        let p = self.pulses.lock().unwrap();
        let i = p.iter().position(|&(r, _)| r >= raw).unwrap_or(p.len());
        let (a, b) = match i {
            0 => return None,
            i if i < p.len() => (p[i - 1], p[i]),
            _ if p.len() >= 2 && raw - p[p.len() - 1].0 <= HOST_EXTRAPOLATE_NS => (p[p.len() - 2], p[p.len() - 1]),
            _ => return None,
        };
        let rate = (b.1 - a.1) as f64 / (b.0 - a.0) as f64;
        Some(a.1 + ((raw - a.0) as f64 * rate).round() as Ns)
    }
}

/// Feeds by controller id.
pub type Hub = Arc<Mutex<BTreeMap<String, Arc<Feed>>>>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_clock_interpolates_and_extrapolates() {
        let c = HostClock::new(300);
        assert_eq!(c.utc_of(5), None);
        // Raw clock 10 ppm slow against UTC.
        let s = 1_000_000_000;
        for k in 0..3 {
            c.push(k * (s - 10_000), 1_790_000_000 * s + k * s);
        }
        assert_eq!(c.utc_of(s - 10_000 + (s - 10_000) / 2), Some(1_790_000_000 * s + s + s / 2));
        assert_eq!(c.utc_of(2 * (s - 10_000) + (s - 10_000)), Some(1_790_000_000 * s + 3 * s));
        assert_eq!(c.utc_of(2 * (s - 10_000) + 2 * s), None);
        assert_eq!(c.utc_of(-1), None);
        // A label going back starts over.
        c.push(3 * s, 1_789_000_000 * s);
        assert_eq!(c.utc_of(3 * s + 5), None);
    }
}
