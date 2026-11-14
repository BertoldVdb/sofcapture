//! One fix per navigation epoch, from UBX-NAV-PVT, from the NAV messages of
//! receivers without it, or from NMEA.

use super::nmea::days_from_civil;
use super::ubx::{self, Id};
use crate::clock::Ns;
use serde::Serialize;

const MS: Ns = 1_000_000;
/// The parts of one epoch come in a row: this long after the last, it is complete.
const FLUSH_AFTER: Ns = 300 * MS;
/// A better source seen this recently silences the lesser ones.
const SOURCE_HOLD: Ns = 3_000 * MS;

#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct Fix {
    /// The epoch. None until the receiver knows date and time.
    pub utc_ns: Option<Ns>,
    pub time_valid: bool,
    pub t_acc_ns: Option<u32>,
    /// GPS minus UTC, as the receiver reports it.
    pub leap_seconds: Option<i8>,
    pub fix: &'static str,
    pub fix_ok: bool,
    pub differential: bool,
    pub carrier: &'static str,
    pub lat_deg: Option<f64>,
    pub lon_deg: Option<f64>,
    /// Above the ellipsoid.
    pub height_m: Option<f64>,
    pub msl_m: Option<f64>,
    pub h_acc_m: Option<f64>,
    pub v_acc_m: Option<f64>,
    pub speed_mps: Option<f64>,
    /// Course over ground.
    pub course_deg: Option<f64>,
    pub vel_ned_mps: Option<[f64; 3]>,
    pub speed_acc_mps: Option<f64>,
    pub sats_used: Option<u8>,
    pub pdop: Option<f64>,
    pub hdop: Option<f64>,
    pub source: &'static str,
}

impl Fix {
    /// Without a fix the receiver repeats an old or empty position.
    fn without_position(mut self) -> Self {
        if self.fix == "none" {
            (self.lat_deg, self.lon_deg, self.height_m, self.msl_m) = (None, None, None, None);
            (self.h_acc_m, self.v_acc_m, self.speed_acc_mps) = (None, None, None);
            (self.speed_mps, self.course_deg, self.vel_ned_mps) = (None, None, None);
        }
        self
    }
}

fn u16_at(p: &[u8], i: usize) -> u16 {
    u16::from_le_bytes(p[i..i + 2].try_into().unwrap())
}

fn u32_at(p: &[u8], i: usize) -> u32 {
    u32::from_le_bytes(p[i..i + 4].try_into().unwrap())
}

fn i32_at(p: &[u8], i: usize) -> i32 {
    i32::from_le_bytes(p[i..i + 4].try_into().unwrap())
}

/// Integer `v` in units of 1/`per`.
fn scaled(v: impl Into<f64>, per: f64) -> f64 {
    v.into() / per
}

fn fix_type(t: u8) -> &'static str {
    match t {
        1 => "dead_reckoning",
        2 => "2d",
        3 => "3d",
        4 => "gnss_dr",
        5 => "time_only",
        _ => "none",
    }
}

/// UTC ns of a calendar time, `nano` as the receiver's signed correction.
fn utc_of(y: u16, mo: u8, d: u8, h: u8, mi: u8, s: u8, nano: i32) -> Option<Ns> {
    let ok = (1980..=2200).contains(&y) && (1..=12).contains(&mo) && (1..=31).contains(&d) && h < 24 && mi < 60 && s <= 60;
    let secs = days_from_civil(y.into(), mo.into(), d.into()) * 86_400 + h as i64 * 3600 + mi as i64 * 60 + s as i64;
    ok.then(|| secs * 1_000_000_000 + nano as Ns)
}

fn from_pvt(p: &[u8], leap: Option<i8>) -> Option<Fix> {
    if p.len() < 78 {
        return None;
    }
    let (valid, flags) = (p[11], p[21]);
    let time_ok = valid & 0x03 == 0x03;
    let utc = utc_of(u16_at(p, 4), p[6], p[7], p[8], p[9], p[10], i32_at(p, 16)).filter(|_| time_ok);
    let f = Fix {
        utc_ns: utc,
        time_valid: valid & 0x07 == 0x07 && utc.is_some(),
        t_acc_ns: Some(u32_at(p, 12)),
        leap_seconds: leap,
        fix: fix_type(p[20]),
        fix_ok: flags & 0x01 != 0,
        differential: flags & 0x02 != 0,
        carrier: match flags >> 6 {
            1 => "float",
            2 => "fixed",
            _ => "none",
        },
        lon_deg: Some(scaled(i32_at(p, 24), 1e7)),
        lat_deg: Some(scaled(i32_at(p, 28), 1e7)),
        height_m: Some(scaled(i32_at(p, 32), 1e3)),
        msl_m: Some(scaled(i32_at(p, 36), 1e3)),
        h_acc_m: Some(scaled(u32_at(p, 40), 1e3)),
        v_acc_m: Some(scaled(u32_at(p, 44), 1e3)),
        vel_ned_mps: Some([scaled(i32_at(p, 48), 1e3), scaled(i32_at(p, 52), 1e3), scaled(i32_at(p, 56), 1e3)]),
        speed_mps: Some(scaled(i32_at(p, 60), 1e3)),
        course_deg: Some(scaled(i32_at(p, 64), 1e5)),
        speed_acc_mps: Some(scaled(u32_at(p, 68), 1e3)),
        sats_used: Some(p[23]),
        pdop: Some(scaled(u16_at(p, 76), 1e2)),
        hdop: None,
        source: "nav_pvt",
    };
    Some(f.without_position())
}

/// NAV-SOL, NAV-TIMEUTC, NAV-POSLLH and NAV-VELNED of one epoch (u-blox 5 and 6).
#[derive(Default)]
struct Legacy {
    itow: u32,
    last: Ns,
    sol: Option<Vec<u8>>,
    timeutc: Option<Vec<u8>>,
    posllh: Option<Vec<u8>>,
    velned: Option<Vec<u8>>,
}

impl Legacy {
    fn complete(&self) -> bool {
        self.sol.is_some() && self.timeutc.is_some() && self.posllh.is_some() && self.velned.is_some()
    }

    /// Needs NAV-SOL: only it tells the fix.
    fn fix(&self, leap: Option<i8>) -> Option<Fix> {
        let s = self.sol.as_ref()?;
        let mut f = Fix {
            leap_seconds: leap,
            fix: fix_type(s[10]),
            fix_ok: s[11] & 0x01 != 0,
            differential: s[11] & 0x02 != 0,
            carrier: "none",
            speed_acc_mps: Some(scaled(u32_at(s, 40), 1e2)),
            pdop: Some(scaled(u16_at(s, 44), 1e2)),
            sats_used: Some(s[47]),
            source: "nav_sol",
            ..Fix::default()
        };
        if let Some(t) = &self.timeutc {
            let utc = utc_of(u16_at(t, 12), t[14], t[15], t[16], t[17], t[18], i32_at(t, 8)).filter(|_| t[19] & 0x04 != 0);
            (f.utc_ns, f.time_valid, f.t_acc_ns) = (utc, t[19] & 0x07 == 0x07 && utc.is_some(), Some(u32_at(t, 4)));
        }
        if let Some(p) = &self.posllh {
            (f.lon_deg, f.lat_deg) = (Some(scaled(i32_at(p, 4), 1e7)), Some(scaled(i32_at(p, 8), 1e7)));
            (f.height_m, f.msl_m) = (Some(scaled(i32_at(p, 12), 1e3)), Some(scaled(i32_at(p, 16), 1e3)));
            (f.h_acc_m, f.v_acc_m) = (Some(scaled(u32_at(p, 20), 1e3)), Some(scaled(u32_at(p, 24), 1e3)));
        }
        if let Some(v) = &self.velned {
            f.vel_ned_mps = Some([scaled(i32_at(v, 4), 1e2), scaled(i32_at(v, 8), 1e2), scaled(i32_at(v, 12), 1e2)]);
            (f.speed_mps, f.course_deg) = (Some(scaled(u32_at(v, 20), 1e2)), Some(scaled(i32_at(v, 24), 1e5)));
            f.speed_acc_mps = Some(scaled(u32_at(v, 28), 1e2));
        }
        Some(f.without_position())
    }
}

/// GGA, RMC and GSA of one epoch.
#[derive(Default)]
struct Nmea {
    /// hhmmss.ss of GGA and RMC.
    time: String,
    last: Ns,
    gga: Option<Vec<String>>,
    rmc: Option<Vec<String>>,
    gsa: Option<Vec<String>>,
}

/// ddmm.mmmm with its hemisphere to degrees.
fn nmea_angle(v: &str, hemi: &str) -> Option<f64> {
    let x: f64 = v.parse().ok()?;
    let deg = (x / 100.0).trunc();
    let d = deg + (x - deg * 100.0) / 60.0;
    let d = (d * 1e7).round() / 1e7;
    match hemi {
        "N" | "E" => Some(d),
        "S" | "W" => Some(-d),
        _ => None,
    }
}

fn num(s: &str) -> Option<f64> {
    s.parse().ok()
}

impl Nmea {
    fn utc(&self) -> Option<Ns> {
        let r = self.rmc.as_ref()?;
        let (t, date) = (r.get(1)?, r.get(9)?);
        if t.len() < 6 || date.len() != 6 || !t.is_ascii() || !date.is_ascii() {
            return None;
        }
        let n = |s: &str| s.parse::<u8>().ok();
        let secs: f64 = t[4..].parse().ok()?;
        let nano = (secs.fract() * 1e9).round() as i32;
        utc_of(2000 + n(&date[4..6])? as u16, n(&date[2..4])?, n(&date[0..2])?, n(&t[0..2])?, n(&t[2..4])?, secs as u8, nano)
    }

    fn fix(&self) -> Option<Fix> {
        let (g, r) = (self.gga.as_ref(), self.rmc.as_ref());
        if g.is_none() && r.is_none() {
            return None;
        }
        fn field(s: Option<&Vec<String>>, i: usize) -> &str {
            s.and_then(|s| s.get(i)).map_or("", String::as_str)
        }
        let quality = g.map(|_| field(g, 6).parse::<u8>().unwrap_or(0));
        let active = r.map(|_| field(r, 2) == "A");
        let alt = num(field(g, 9));
        let fix = match (field(self.gsa.as_ref(), 2), quality, active) {
            ("2", ..) => "2d",
            ("3", ..) => "3d",
            ("1", ..) | (_, Some(0), _) | (_, None, Some(false)) => "none",
            (_, Some(6), _) => "dead_reckoning",
            (_, Some(_), _) if alt.is_some() => "3d",
            _ => "2d",
        };
        let utc = self.utc();
        let (lat, lon) = match g {
            Some(_) => (nmea_angle(field(g, 2), field(g, 3)), nmea_angle(field(g, 4), field(g, 5))),
            None => (nmea_angle(field(r, 3), field(r, 4)), nmea_angle(field(r, 5), field(r, 6))),
        };
        let f = Fix {
            utc_ns: utc,
            time_valid: utc.is_some(),
            fix,
            fix_ok: quality.is_none_or(|q| q > 0) && active.unwrap_or(true),
            differential: matches!(quality, Some(2 | 4 | 5)),
            carrier: match quality {
                Some(4) => "fixed",
                Some(5) => "float",
                _ => "none",
            },
            lat_deg: lat,
            lon_deg: lon,
            height_m: alt.zip(num(field(g, 11))).map(|(a, sep)| ((a + sep) * 1e3).round() / 1e3),
            msl_m: alt,
            // Knots.
            speed_mps: num(field(r, 7)).map(|k| (k * 1852.0 / 3.6).round() / 1e3),
            course_deg: num(field(r, 8)),
            sats_used: field(g, 7).parse().ok(),
            pdop: num(field(self.gsa.as_ref(), 15)),
            hdop: num(field(g, 8)).or(num(field(self.gsa.as_ref(), 16))),
            source: "nmea",
            ..Fix::default()
        };
        Some(f.without_position())
    }
}

/// Collects a receiver's output into fixes. NAV-PVT has all of one, the others
/// are put together by epoch.
#[derive(Default)]
pub struct Assembler {
    /// From NAV-TIMEGPS, once the receiver knows them.
    leap: Option<i8>,
    last_pvt: Option<Ns>,
    last_legacy: Option<Ns>,
    legacy: Option<Legacy>,
    nmea: Option<Nmea>,
}

impl Assembler {
    /// Drops what is half collected, after a baud change.
    pub fn reset(&mut self) {
        (self.legacy, self.nmea) = (None, None);
    }

    fn recent(t: Option<Ns>, now: Ns) -> bool {
        t.is_some_and(|t| now - t < SOURCE_HOLD)
    }

    pub fn on_ubx(&mut self, id: Id, p: &[u8], now: Ns) -> Option<Fix> {
        let min = match id {
            ubx::NAV_TIMEGPS => {
                if let Some(l) = ubx::leap_seconds(p) {
                    self.leap = l;
                }
                return None;
            }
            ubx::NAV_PVT => {
                let f = from_pvt(p, self.leap)?;
                (self.last_pvt, self.legacy, self.nmea) = (Some(now), None, None);
                return Some(f);
            }
            ubx::NAV_SOL => 52,
            ubx::NAV_TIMEUTC => 20,
            ubx::NAV_POSLLH => 28,
            ubx::NAV_VELNED => 36,
            _ => return None,
        };
        if p.len() < min || Self::recent(self.last_pvt, now) {
            return None;
        }
        (self.last_legacy, self.nmea) = (Some(now), None);
        let itow = u32_at(p, 0);
        // A new epoch began before the last was complete.
        let mut out = None;
        if self.legacy.as_ref().is_some_and(|l| l.itow != itow) {
            out = self.legacy.take().and_then(|l| l.fix(self.leap));
        }
        let l = self.legacy.get_or_insert_with(|| Legacy { itow, ..Legacy::default() });
        l.last = now;
        let part = match id {
            ubx::NAV_SOL => &mut l.sol,
            ubx::NAV_TIMEUTC => &mut l.timeutc,
            ubx::NAV_POSLLH => &mut l.posllh,
            _ => &mut l.velned,
        };
        *part = Some(p.to_vec());
        if l.complete() {
            out = self.legacy.take().and_then(|l| l.fix(self.leap));
        }
        out
    }

    /// `f`: the sentence's fields, the address first.
    pub fn on_nmea(&mut self, f: &[&str], now: Ns) -> Option<Fix> {
        if Self::recent(self.last_pvt, now) || Self::recent(self.last_legacy, now) {
            return None;
        }
        let kind = f.first().filter(|a| a.len() == 5 && a.is_ascii()).map_or("", |a| &a[2..]);
        let own = |f: &[&str]| Some(f.iter().map(|s| s.to_string()).collect::<Vec<_>>());
        match kind {
            "GGA" | "RMC" if f.len() > 6 => {
                let mut out = None;
                if self.nmea.as_ref().is_some_and(|n| n.time != f[1]) {
                    out = self.nmea.take().and_then(|n| n.fix());
                }
                let n = self.nmea.get_or_insert_with(|| Nmea { time: f[1].into(), ..Nmea::default() });
                n.last = now;
                if kind == "GGA" {
                    n.gga = own(f);
                } else {
                    n.rmc = own(f);
                }
                out
            }
            // No time of its own: it follows the GGA and RMC it belongs to.
            "GSA" => {
                if let Some(n) = &mut self.nmea {
                    (n.gsa, n.last) = (own(f), now);
                }
                None
            }
            _ => None,
        }
    }

    /// An epoch whose parts stopped coming.
    pub fn on_tick(&mut self, now: Ns) -> Option<Fix> {
        if self.legacy.as_ref().is_some_and(|l| now - l.last >= FLUSH_AFTER) {
            return self.legacy.take().and_then(|l| l.fix(self.leap));
        }
        if self.nmea.as_ref().is_some_and(|n| now - n.last >= FLUSH_AFTER) {
            return self.nmea.take().and_then(|n| n.fix());
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss::nmea;

    /// 2026-09-21 12:00:00 UTC.
    const T0: Ns = (20_717 * 86_400 + 43_200) * 1_000_000_000;

    fn pvt(fix: u8) -> Vec<u8> {
        let mut p = vec![0u8; 92];
        p[4..6].copy_from_slice(&2026u16.to_le_bytes());
        (p[6], p[7], p[8], p[9], p[10], p[11]) = (9, 21, 12, 0, 0, 0x07);
        p[12..16].copy_from_slice(&24u32.to_le_bytes());
        p[16..20].copy_from_slice(&(-355i32).to_le_bytes());
        (p[20], p[21], p[23]) = (fix, 0x01 | 0x02 | 2 << 6, 14);
        p[24..28].copy_from_slice(&111_234_567i32.to_le_bytes());
        p[28..32].copy_from_slice(&(-481_234_567i32).to_le_bytes());
        p[32..36].copy_from_slice(&512_340i32.to_le_bytes());
        p[36..40].copy_from_slice(&465_120i32.to_le_bytes());
        p[40..44].copy_from_slice(&1800u32.to_le_bytes());
        p[44..48].copy_from_slice(&2600u32.to_le_bytes());
        p[48..52].copy_from_slice(&(-20i32).to_le_bytes());
        p[56..60].copy_from_slice(&10i32.to_le_bytes());
        p[60..64].copy_from_slice(&20i32.to_le_bytes());
        p[64..68].copy_from_slice(&18_740_000i32.to_le_bytes());
        p[68..72].copy_from_slice(&80u32.to_le_bytes());
        p[76..78].copy_from_slice(&131u16.to_le_bytes());
        p
    }

    #[test]
    fn test_date() {
        assert_eq!(utc_of(2026, 9, 21, 12, 0, 0, 0), Some(T0));
        assert_eq!(utc_of(2026, 13, 1, 0, 0, 0, 0), None);
    }

    #[test]
    fn nav_pvt_is_a_whole_fix() {
        let mut a = Assembler::default();
        let mut tg = vec![0u8; 16];
        (tg[10], tg[11]) = (18, 0x07);
        assert_eq!(a.on_ubx(ubx::NAV_TIMEGPS, &tg, 0), None);
        let f = a.on_ubx(ubx::NAV_PVT, &pvt(3), 0).unwrap();
        assert_eq!((f.utc_ns, f.time_valid, f.t_acc_ns, f.leap_seconds), (Some(T0 - 355), true, Some(24), Some(18)));
        assert_eq!((f.fix, f.fix_ok, f.differential, f.carrier), ("3d", true, true, "fixed"));
        assert_eq!((f.lat_deg, f.lon_deg), (Some(-48.1234567), Some(11.1234567)));
        assert_eq!((f.height_m, f.msl_m, f.h_acc_m, f.v_acc_m), (Some(512.34), Some(465.12), Some(1.8), Some(2.6)));
        assert_eq!((f.speed_mps, f.course_deg, f.speed_acc_mps), (Some(0.02), Some(187.4), Some(0.08)));
        assert_eq!(f.vel_ned_mps, Some([-0.02, 0.0, 0.01]));
        assert_eq!((f.sats_used, f.pdop, f.hdop, f.source), (Some(14), Some(1.31), None, "nav_pvt"));
        // Plain numbers in JSON.
        let j = serde_json::to_string(&f).unwrap();
        assert!(j.contains("\"lat_deg\":-48.1234567") && j.contains("\"course_deg\":187.4"), "{j}");
    }

    #[test]
    fn no_fix_has_no_position_and_an_unknown_time_no_utc() {
        let mut a = Assembler::default();
        let mut p = pvt(0);
        (p[11], p[21]) = (0x00, 0x00);
        let f = a.on_ubx(ubx::NAV_PVT, &p, 0).unwrap();
        assert_eq!((f.fix, f.fix_ok, f.utc_ns, f.time_valid, f.leap_seconds), ("none", false, None, false, None));
        assert_eq!((f.lat_deg, f.msl_m, f.speed_mps, f.vel_ned_mps, f.h_acc_m), (None, None, None, None, None));
        assert_eq!(f.sats_used, Some(14));
        // u-blox 7 sends 84 bytes. Shorter is not a NAV-PVT.
        assert!(a.on_ubx(ubx::NAV_PVT, &pvt(3)[..84], 0).is_some());
        assert!(a.on_ubx(ubx::NAV_PVT, &pvt(3)[..60], 0).is_none());
    }

    fn legacy(itow: u32) -> [(Id, Vec<u8>); 4] {
        let mut sol = vec![0u8; 52];
        (sol[10], sol[11], sol[47]) = (3, 0x0D, 9);
        sol[40..44].copy_from_slice(&50u32.to_le_bytes());
        sol[44..46].copy_from_slice(&210u16.to_le_bytes());
        let mut t = vec![0u8; 20];
        t[4..8].copy_from_slice(&60u32.to_le_bytes());
        t[12..14].copy_from_slice(&2026u16.to_le_bytes());
        (t[14], t[15], t[16], t[19]) = (9, 21, 12, 0x07);
        let mut pos = vec![0u8; 28];
        pos[4..8].copy_from_slice(&111_234_567i32.to_le_bytes());
        pos[8..12].copy_from_slice(&481_234_567i32.to_le_bytes());
        pos[12..16].copy_from_slice(&512_340i32.to_le_bytes());
        pos[16..20].copy_from_slice(&465_120i32.to_le_bytes());
        pos[20..24].copy_from_slice(&3000u32.to_le_bytes());
        let mut vel = vec![0u8; 36];
        vel[4..8].copy_from_slice(&150i32.to_le_bytes());
        vel[20..24].copy_from_slice(&150u32.to_le_bytes());
        vel[24..28].copy_from_slice(&9_000_000i32.to_le_bytes());
        vel[28..32].copy_from_slice(&20u32.to_le_bytes());
        let mut all = [(ubx::NAV_POSLLH, pos), (ubx::NAV_SOL, sol), (ubx::NAV_VELNED, vel), (ubx::NAV_TIMEUTC, t)];
        for (_, p) in &mut all {
            p[0..4].copy_from_slice(&itow.to_le_bytes());
        }
        all
    }

    #[test]
    fn older_receivers_fix_is_put_together_by_epoch() {
        let mut a = Assembler::default();
        let parts = legacy(1000);
        for (id, p) in &parts[..3] {
            assert_eq!(a.on_ubx(*id, p, 0), None);
        }
        let f = a.on_ubx(parts[3].0, &parts[3].1, 0).unwrap();
        assert_eq!((f.utc_ns, f.time_valid, f.t_acc_ns, f.source), (Some(T0), true, Some(60), "nav_sol"));
        assert_eq!((f.fix, f.fix_ok, f.differential, f.sats_used, f.pdop), ("3d", true, false, Some(9), Some(2.1)));
        assert_eq!((f.lat_deg, f.lon_deg, f.height_m, f.msl_m, f.h_acc_m), (Some(48.1234567), Some(11.1234567), Some(512.34), Some(465.12), Some(3.0)));
        assert_eq!((f.speed_mps, f.course_deg, f.vel_ned_mps, f.speed_acc_mps), (Some(1.5), Some(90.0), Some([1.5, 0.0, 0.0]), Some(0.2)));
        assert_eq!(a.on_tick(10_000 * MS), None);

        // Without NAV-POSLLH and NAV-VELNED: out once its parts stop, or the next epoch begins.
        let (sol, t) = (&parts[1], &parts[3]);
        assert_eq!(a.on_ubx(sol.0, &sol.1, 0), None);
        assert_eq!(a.on_ubx(t.0, &t.1, 10 * MS), None);
        assert_eq!(a.on_tick(200 * MS), None);
        let f = a.on_tick(320 * MS).unwrap();
        assert_eq!((f.fix, f.utc_ns, f.lat_deg, f.speed_mps, f.speed_acc_mps), ("3d", Some(T0), None, None, Some(0.5)));
        let next = legacy(2000);
        assert_eq!(a.on_ubx(sol.0, &sol.1, 1000 * MS), None);
        assert_eq!(a.on_ubx(next[1].0, &next[1].1, 2000 * MS).map(|f| f.fix), Some("3d"));
        // NAV-SOL is needed.
        a.reset();
        assert_eq!(a.on_ubx(t.0, &t.1, 3000 * MS), None);
        assert_eq!(a.on_tick(4000 * MS), None);
    }

    fn fields_of(a: &mut Assembler, body: &str, now: Ns) -> Option<Fix> {
        let s = nmea::sentence(body);
        a.on_nmea(&nmea::fields(&s), now)
    }

    #[test]
    fn nmea_fix_from_gga_rmc_gsa() {
        let mut a = Assembler::default();
        assert_eq!(fields_of(&mut a, "GNRMC,120000.00,A,4807.40740,N,01107.40740,E,1.944,187.4,210926,,,A", 0), None);
        assert_eq!(fields_of(&mut a, "GNGGA,120000.00,4807.40740,N,01107.40740,E,2,09,0.95,465.1,M,47.2,M,,", 10 * MS), None);
        assert_eq!(fields_of(&mut a, "GNGSA,A,3,01,02,03,04,05,06,07,08,09,,,,1.80,0.95,1.50", 20 * MS), None);
        assert_eq!(a.on_tick(100 * MS), None);
        let f = a.on_tick(330 * MS).unwrap();
        assert_eq!((f.utc_ns, f.time_valid, f.source), (Some(T0), true, "nmea"));
        assert_eq!((f.fix, f.fix_ok, f.differential, f.carrier), ("3d", true, true, "none"));
        assert_eq!((f.lat_deg, f.lon_deg), (Some(48.1234567), Some(11.1234567)));
        assert_eq!((f.msl_m, f.height_m, f.sats_used), (Some(465.1), Some(512.3), Some(9)));
        assert_eq!((f.speed_mps, f.course_deg, f.pdop, f.hdop), (Some(1.0), Some(187.4), Some(1.8), Some(0.95)));
        assert_eq!((f.vel_ned_mps, f.h_acc_m, f.t_acc_ns, f.leap_seconds), (None, None, None, None));

        // No fix: the next epoch's time flushes it, with nothing but the time.
        assert_eq!(fields_of(&mut a, "GPRMC,120001.00,V,,,,,,,210926,,,N", 1000 * MS), None);
        assert_eq!(fields_of(&mut a, "GPGGA,120001.00,,,,,0,00,99.99,,,,,,", 1010 * MS), None);
        let f = fields_of(&mut a, "GPRMC,120002.00,V,,,,,,,210926,,,N", 2000 * MS).unwrap();
        assert_eq!((f.fix, f.fix_ok, f.utc_ns, f.lat_deg, f.sats_used), ("none", false, Some(T0 + 1_000_000_000), None, Some(0)));
    }

    #[test]
    fn the_best_source_wins() {
        let mut a = Assembler::default();
        assert!(a.on_ubx(ubx::NAV_PVT, &pvt(3), 0).is_some());
        // A receiver that also sends the older messages and NMEA.
        for (id, p) in &legacy(1000) {
            assert_eq!(a.on_ubx(*id, p, 100 * MS), None);
        }
        assert_eq!(fields_of(&mut a, "GPGGA,120000.00,4807.40740,N,01107.40740,E,1,09,0.95,465.1,M,47.2,M,,", 200 * MS), None);
        assert_eq!(a.on_tick(1000 * MS), None);
        // NAV-PVT gone: the older messages take over.
        let now = 5000 * MS;
        let parts = legacy(2000);
        assert!(parts.iter().filter_map(|(id, p)| a.on_ubx(*id, p, now)).count() == 1);
        assert_eq!(fields_of(&mut a, "GPGGA,120005.00,4807.40740,N,01107.40740,E,1,09,0.95,465.1,M,47.2,M,,", now), None);
        assert_eq!(a.on_tick(now + 1000 * MS), None);
    }
}
