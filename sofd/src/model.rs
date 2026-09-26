//! Counter unwrapping and a least-squares fit of frame start time against the counter.

use crate::clock::Ns;
use serde::Serialize;
use std::collections::VecDeque;

/// Uses elapsed time, so gaps longer than one wrap are fine.
pub fn unwrap(prev: Option<(i64, Ns)>, raw: u32, t: Ns, period_ns: f64, bits: Option<u32>) -> i64 {
    let Some((pc, pt)) = prev else { return raw as i64 };
    let expected = pc as f64 + (t - pt) as f64 / period_ns;
    match bits {
        Some(bits) => {
            let m = (1i64 << bits) as f64;
            raw as i64 + ((expected - raw as f64) / m).round() as i64 * m as i64
        }
        None => expected.round() as i64,
    }
}

/// Start of counter `c` = `t_ref_ns + (c - counter_ref) * period_ns`.
#[derive(Debug, Clone, Serialize)]
pub struct Fit {
    pub counter_ref: i64,
    pub t_ref_ns: Ns,
    /// Host ns per counter tick.
    pub period_ns: f64,
    /// Controller frequency error vs. the host clock; positive = too fast.
    pub ppm: f64,
    pub resid_rms_ns: f64,
    pub n: usize,
}

impl Fit {
    // Only the offset goes through f64: absolute realtime ns would lose precision.
    pub fn predict(&self, c: i64) -> Ns {
        self.t_ref_ns + ((c - self.counter_ref) as f64 * self.period_ns).round() as Ns
    }
}

pub struct Model {
    window: usize,
    pts: VecDeque<(i64, Ns, Ns)>,
}

impl Model {
    pub fn new(window: usize) -> Self {
        Self { window: window.max(3), pts: VecDeque::new() }
    }

    pub fn clear(&mut self) {
        self.pts.clear();
    }

    pub fn push(&mut self, counter: i64, t: Ns, unc: Ns) {
        if self.pts.len() == self.window {
            self.pts.pop_front();
        }
        self.pts.push_back((counter, t, unc));
    }

    pub fn fit(&self, unit_ns: f64) -> Option<Fit> {
        let &(c0, t0, _) = self.pts.back()?;
        let mut uncs: Vec<Ns> = self.pts.iter().map(|p| p.2).collect();
        uncs.sort_unstable();
        let limit = (uncs[uncs.len() / 2] * 3).max(1_000);
        // Center on the newest point to keep f64 precision.
        // Weight by 1/unc² so stalled reads (wide brackets) count less.
        let pts: Vec<(f64, f64, f64)> = self
            .pts
            .iter()
            .filter(|p| p.2 <= limit)
            .map(|p| ((p.0 - c0) as f64, (p.1 - t0) as f64, 1.0 / (p.2.max(1) as f64).powi(2)))
            .collect();
        let n = pts.len();
        if n < 3 {
            return None;
        }
        let sw: f64 = pts.iter().map(|p| p.2).sum();
        let xbar = pts.iter().map(|p| p.2 * p.0).sum::<f64>() / sw;
        let ybar = pts.iter().map(|p| p.2 * p.1).sum::<f64>() / sw;
        let sxx: f64 = pts.iter().map(|p| p.2 * (p.0 - xbar).powi(2)).sum();
        let sxy: f64 = pts.iter().map(|p| p.2 * (p.0 - xbar) * (p.1 - ybar)).sum();
        if sxx == 0.0 {
            return None;
        }
        let b = sxy / sxx;
        let a = ybar - b * xbar;
        let ss: f64 = pts.iter().map(|p| (p.1 - a - b * p.0).powi(2)).sum();
        Some(Fit {
            counter_ref: c0,
            t_ref_ns: t0 + a.round() as Ns,
            period_ns: b,
            ppm: (unit_ns / b - 1.0) * 1e6,
            resid_rms_ns: (ss / (n - 2).max(1) as f64).sqrt(),
            n,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::COUNTER_BITS;

    #[test]
    fn unwrap_across_wraps_and_gaps() {
        let p = 125_000.0;
        let b = Some(COUNTER_BITS);
        assert_eq!(unwrap(None, 100, 0, p, b), 100);
        // 16380 -> 3 across one wrap, 7 ticks later.
        assert_eq!(unwrap(Some((16380, 0)), 3, 7 * 125_000, p, b), 16384 + 3);
        // 10.5 s gap, several wraps.
        let ticks = 84_000i64;
        let raw = ((500 + ticks) & 0x3fff) as u32;
        assert_eq!(unwrap(Some((500, 0)), raw, ticks * 125_000, p, b), 500 + ticks);
        // Unknown wrap point: elapsed time only.
        assert_eq!(unwrap(Some((500, 0)), 7, 100 * 1_000_000, 1e6, None), 600);
    }

    #[test]
    fn realtime_scale_keeps_ns_resolution() {
        let t0: Ns = 1_790_454_400_291_980_001;
        let mut m = Model::new(8);
        for i in 0..8i64 {
            m.push(i * 800, t0 + i * 100_000_000, 100);
        }
        let f = m.fit(125_000.0).unwrap();
        assert_eq!(f.predict(8 * 800), t0 + 800_000_000);
        assert_eq!(f.predict(3), t0 + 375_000);
    }

    #[test]
    fn wide_brackets_carry_little_weight() {
        let mut m = Model::new(64);
        for i in 0..20i64 {
            // One stalled read, 20 µs late with a 25 µs bracket.
            let (err, unc) = if i == 10 { (20_000, 25_000) } else { (0, 400) };
            m.push(i * 800, i * 100_000_000 + err, unc);
        }
        let f = m.fit(125_000.0).unwrap();
        assert!((f.predict(10 * 800) - 1_000_000_000).abs() < 100, "{f:?}");
    }

    #[test]
    fn fit_recovers_rate_and_offset() {
        let mut m = Model::new(64);
        let period = 125_000.0 * (1.0 + 20e-6);
        for i in 0..40i64 {
            let c = 1000 + i * 800;
            let jitter = if i % 2 == 0 { 200 } else { -200 };
            m.push(c, 5_000_000 + (c as f64 * period) as Ns + jitter, 300);
        }
        let f = m.fit(125_000.0).unwrap();
        // Period 20 ppm long = frequency 20 ppm slow.
        assert!((f.ppm + 20.0).abs() < 0.1, "ppm {}", f.ppm);
        assert!((f.predict(1000) as f64 - (5_000_000.0 + 1000.0 * period)).abs() < 300.0);
        assert!(f.resid_rms_ns < 250.0);
    }
}
