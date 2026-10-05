//! Matches device frame numbers to the host controller's frame count.
//! The low 11 bits agree; the receive time picks which 2048-frame wrap.

/// Max frames between a board tag's frame and its receive time.
pub const BOARD_MAX_LAG: i64 = 500;
/// The same for MQTT clients. Plus MAX_LEAD it must stay under 2048.
pub const MQTT_MAX_LAG: i64 = 1900;
/// Slack the other way: the receive-time estimate lags slightly.
const MAX_LEAD: i64 = 50;
/// Misses in a row before searching for the offset again.
const RELOCK_AFTER: u32 = 3;

pub struct Join {
    offset: Option<i64>,
    misses: u32,
    max_lag: i64,
}

impl Default for Join {
    fn default() -> Self {
        Self::new(BOARD_MAX_LAG)
    }
}

impl Join {
    pub fn new(max_lag: i64) -> Self {
        Self { offset: None, misses: 0, max_lag }
    }

    pub fn reset(&mut self) {
        *self = Self::new(self.max_lag);
    }

    /// host frame = device frame + offset.
    pub fn offset(&self) -> Option<i64> {
        self.offset
    }

    /// `est`: host frame at receive time. `bus_of`: 11-bit bus frame of a host frame.
    pub fn host_frame(&mut self, dev: u64, est: f64, bus_of: impl Fn(i64) -> u32) -> Option<i64> {
        let dev_bus = (dev & 0x7ff) as i64;
        let max_lag = self.max_lag;
        let plausible = |hf: i64| (-max_lag..=MAX_LEAD).contains(&(hf - est.round() as i64));
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
        // Latest host frame with the same bus number before the receive time.
        let top = est.round() as i64 + MAX_LEAD;
        let hf = top - (bus_of(top) as i64 - dev_bus).rem_euclid(2048);
        if !plausible(hf) {
            return None;
        }
        // A bare 11-bit frame number wraps, so there's no offset to keep.
        if dev >= 2048 {
            self.offset = Some(hf - dev as i64);
        }
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

    #[test]
    fn mqtt_tags_may_be_late() {
        let bus = |h: i64| (h & 0x7ff) as u32;
        // 1.5 s late: too late for a board, fine for MQTT.
        assert_eq!(Join::default().host_frame(3000, 4500.0, bus), None);
        let mut j = Join::new(MQTT_MAX_LAG);
        assert_eq!(j.host_frame(3000, 4500.0, bus), Some(3000));
        j.reset();
        assert_eq!(j.host_frame(3000, 4950.0, bus), None, "older than the lag");
    }

    #[test]
    fn bare_bus_frame_numbers_match_each_time() {
        let bus = |h: i64| (h & 0x7ff) as u32;
        let mut j = Join::new(MQTT_MAX_LAG);
        for h in (10_000..20_000).step_by(300) {
            assert_eq!(j.host_frame((h & 0x7ff) as u64, h as f64 + 1200.0, bus), Some(h));
            assert_eq!(j.offset(), None);
        }
    }
}
