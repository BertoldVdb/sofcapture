//! Matches the device's frame numbers to the host controller's frame count.
//!
//! Both see the same SOFs, so their low 11 bits agree; the remaining
//! ambiguity (multiples of 2048 frames, 2.048 s) is resolved by when the tag
//! was received, which is a few ms after its frame. The offset is then fixed
//! and every later tag is checked against it.

/// Furthest a tag's frame may lie before the frame at its receive time.
const MAX_LAG: i64 = 500;
/// Slack the other way: the receive-time estimate lags slightly.
const MAX_LEAD: i64 = 50;
/// Misses in a row before searching for the offset again.
const RELOCK_AFTER: u32 = 3;

#[derive(Default)]
pub struct Join {
    offset: Option<i64>,
    misses: u32,
}

impl Join {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// host frame = device frame + offset.
    pub fn offset(&self) -> Option<i64> {
        self.offset
    }

    /// `est`: host frame at receive time. `bus_of`: 11-bit bus frame of a host frame.
    pub fn host_frame(&mut self, dev: u64, est: f64, bus_of: impl Fn(i64) -> u32) -> Option<i64> {
        let dev_bus = (dev & 0x7ff) as i64;
        let plausible = |hf: i64| (-MAX_LAG..=MAX_LEAD).contains(&(hf - est.round() as i64));
        if let Some(k) = self.offset {
            let hf = dev as i64 + k;
            if bus_of(hf) as i64 == dev_bus && plausible(hf) {
                self.misses = 0;
                return Some(hf);
            }
            self.misses += 1;
            if self.misses < RELOCK_AFTER {
                return None;
            }
            self.reset();
        }
        let h0 = est.round() as i64;
        let hf = h0 + (dev_bus - bus_of(h0) as i64 + 1024).rem_euclid(2048) - 1024;
        if !plausible(hf) {
            return None;
        }
        self.offset = Some(hf - dev as i64);
        Some(hf)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn locks_across_wrap_and_follows() {
        let bus = |h: i64| (h + 700).rem_euclid(2048) as u32;
        let k = 2 * 2048 - 700;
        let mut j = Join::default();
        assert_eq!(j.host_frame((9000 - k) as u64, 9003.2, bus), Some(9000));
        assert_eq!(j.offset(), Some(k));
        // 3 ms receive latency, across wraps.
        for h in (9100..20000).step_by(100) {
            assert_eq!(j.host_frame((h - k) as u64, h as f64 + 3.0, bus), Some(h));
        }
    }

    #[test]
    fn relocks_after_device_renumbering() {
        let mut j = Join::default();
        let bus = |h: i64| (h & 0x7ff) as u32;
        assert_eq!(j.host_frame(3650, 3050.0, bus), None, "600 frames after its receive time");
        assert_eq!(j.host_frame(3000, 3002.0, bus), Some(3000));
        // Device count jumps by 4096.
        for i in 1..RELOCK_AFTER {
            assert_eq!(j.host_frame(3100 + 4096, 3102.0, bus), None, "miss {i}");
        }
        assert_eq!(j.host_frame(3100 + 4096, 3102.0, bus), Some(3100));
        assert_eq!(j.offset(), Some(-4096));
    }
}
