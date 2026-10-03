//! UBX messages sofd reads and the receiver configuration it writes (RAM only).

use super::framer::ubx_checksum;
use crate::clock::Ns;

pub type Id = (u8, u8);
pub const NAV_STATUS: Id = (0x01, 0x03);
pub const NAV_TIMEGPS: Id = (0x01, 0x20);
pub const ACK_NAK: Id = (0x05, 0x00);
pub const ACK_ACK: Id = (0x05, 0x01);
pub const CFG_PRT: Id = (0x06, 0x00);
pub const CFG_MSG: Id = (0x06, 0x01);
pub const CFG_VALSET: Id = (0x06, 0x8A);
pub const MON_VER: Id = (0x0A, 0x04);
pub const TIM_TP: Id = (0x0D, 0x01);

/// Enabled once per navigation solution.
pub const ENABLED: [Id; 3] = [TIM_TP, NAV_TIMEGPS, NAV_STATUS];

const GPS_EPOCH_UNIX_S: i64 = 315_964_800;
const WEEK_S: i64 = 604_800;

pub fn frame(id: Id, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![0xB5, 0x62, id.0, id.1];
    f.extend((payload.len() as u16).to_le_bytes());
    f.extend(payload);
    let ck = ubx_checksum(&f[2..]);
    f.extend(ck);
    f
}

/// Expects a whole, checksum-checked frame.
pub fn split(frame: &[u8]) -> (Id, &[u8]) {
    ((frame[2], frame[3]), &frame[6..frame.len() - 2])
}

fn u16_at(p: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([p[o], p[o + 1]])
}

fn u32_at(p: &[u8], o: usize) -> u32 {
    u32::from_le_bytes(p[o..o + 4].try_into().unwrap())
}

/// UBX-TIM-TP: time of the *next* time pulse.
#[derive(Debug, Clone, Copy)]
pub struct TimTp {
    pub tow_ms: u32,
    /// In 2^-32 ms.
    pub tow_sub_ms: u32,
    pub qerr_ps: i32,
    pub week: u16,
    pub flags: u8,
    pub ref_info: u8,
}

impl TimTp {
    pub fn parse(p: &[u8]) -> Option<Self> {
        (p.len() >= 16).then(|| Self {
            tow_ms: u32_at(p, 0),
            tow_sub_ms: u32_at(p, 4),
            qerr_ps: u32_at(p, 8) as i32,
            week: u16_at(p, 12),
            flags: p[14],
            ref_info: p[15],
        })
    }

    /// Flags bit 4 (protocol 16+) marks qErr invalid.
    pub fn qerr(&self) -> Option<i32> {
        (self.flags & 0x10 == 0).then_some(self.qerr_ps)
    }

    /// Unix ns. A GNSS time base must be GPS.
    pub fn utc_ns(&self, leap_s: Option<i8>) -> Result<Ns, &'static str> {
        let sub = ((self.tow_sub_ms as u64 * 1_000_000) >> 32) as Ns;
        let t = (GPS_EPOCH_UNIX_S + self.week as i64 * WEEK_S) * 1_000_000_000 + self.tow_ms as Ns * 1_000_000 + sub;
        if self.flags & 1 != 0 {
            return Ok(t);
        }
        // refInfo.timeRefGnss (protocol 18+): 0 = GPS; older receivers send 0.
        if self.ref_info & 0x0F != 0 {
            return Err("tp_time_base");
        }
        let leap = leap_s.ok_or("no_leap_seconds")?;
        Ok(t - leap as Ns * 1_000_000_000)
    }
}

/// UBX-NAV-TIMEGPS: leap seconds (GPS - UTC), if the receiver knows them.
pub fn leap_seconds(p: &[u8]) -> Option<Option<i8>> {
    (p.len() >= 12).then(|| (p[11] & 0x04 != 0).then_some(p[10] as i8))
}

/// UBX-NAV-STATUS: gpsFixOk.
pub fn fix_ok(p: &[u8]) -> Option<bool> {
    (p.len() >= 6).then(|| p[5] & 1 != 0)
}

/// UBX-MON-VER.
#[derive(Debug, Clone, Default, serde::Serialize)]
pub struct Version {
    pub sw: String,
    pub hw: String,
    pub ext: Vec<String>,
}

fn cstr(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).trim().to_string()
}

impl Version {
    pub fn parse(p: &[u8]) -> Option<Self> {
        if p.len() < 40 {
            return None;
        }
        Some(Self { sw: cstr(&p[..30]), hw: cstr(&p[30..40]), ext: p[40..].chunks(30).map(cstr).collect() })
    }

    /// "PROTVER=34.10" (or "PROTVER 18.00") among the extension strings.
    pub fn protocol(&self) -> Option<f32> {
        self.ext.iter().find_map(|e| {
            let rest = e.split("PROTVER").nth(1)?;
            rest.trim_start_matches(['=', ' ']).split(|c: char| !(c.is_ascii_digit() || c == '.')).next()?.parse().ok()
        })
    }

    /// u-blox 10 dropped CFG-PRT/CFG-MSG.
    pub fn uses_valset(&self) -> bool {
        let hw = u32::from_str_radix(&self.hw, 16).unwrap_or(0);
        self.protocol().is_some_and(|p| p >= 27.0) || hw == 0x0019_0000 || hw == 0x000A_0000
    }
}

/// Sets the rate on the port the message arrives on.
pub fn cfg_msg(id: Id, rate: u8) -> Vec<u8> {
    frame(CFG_MSG, &[id.0, id.1, rate])
}

/// Keeps the polled port settings (UART1 8N1 if none), sets baud and UBX-only output.
pub fn cfg_prt(polled: Option<&[u8]>, baud: u32) -> Vec<u8> {
    let mut p: Vec<u8> = match polled {
        Some(p) if p.len() == 20 => p.to_vec(),
        _ => {
            let mut p = vec![0u8; 20];
            p[0] = 1;
            p[4..8].copy_from_slice(&0x08D0u32.to_le_bytes());
            p[12..14].copy_from_slice(&0x0007u16.to_le_bytes());
            p
        }
    };
    p[8..12].copy_from_slice(&baud.to_le_bytes());
    p[14..16].copy_from_slice(&0x0001u16.to_le_bytes());
    frame(CFG_PRT, &p)
}

/// Configuration keys (u-blox generation 9/10 interface description).
pub mod key {
    pub fn uart_baud(port: u8) -> u32 {
        if port == 2 { 0x4053_0001 } else { 0x4052_0001 }
    }
    pub fn uart_out_ubx(port: u8) -> u32 {
        if port == 2 { 0x1076_0001 } else { 0x1074_0001 }
    }
    pub fn uart_out_nmea(port: u8) -> u32 {
        if port == 2 { 0x1076_0002 } else { 0x1074_0002 }
    }
    /// CFG-MSGOUT-UBX_* for UART1; UART2 is the next key.
    pub fn msgout(id: super::Id, port: u8) -> u32 {
        let uart1 = match id {
            super::TIM_TP => 0x2091_017E,
            super::NAV_TIMEGPS => 0x2091_0048,
            super::NAV_STATUS => 0x2091_001B,
            _ => unreachable!("no CFG-MSGOUT key for {id:?}"),
        };
        if port == 2 { uart1 + 1 } else { uart1 }
    }
}

/// CFG-VALSET to the RAM layer.
pub fn valset_ram(items: &[(u32, u32)]) -> Vec<u8> {
    let mut p = vec![0u8, 0x01, 0, 0];
    for &(k, v) in items {
        p.extend(k.to_le_bytes());
        let size = match (k >> 28) & 7 {
            1 | 2 => 1,
            3 => 2,
            4 => 4,
            _ => 8,
        };
        p.extend(&(v as u64).to_le_bytes()[..size]);
    }
    frame(CFG_VALSET, &p)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn poll_frame_matches_reference() {
        assert_eq!(frame(MON_VER, &[]), [0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]);
    }

    #[test]
    fn tim_tp_to_utc() {
        // Week 2437, TOW 345600.000 s in GPS time, 18 leap seconds.
        let mut p = vec![];
        p.extend(345_600_000u32.to_le_bytes());
        p.extend((1u32 << 31).to_le_bytes()); // 0.5 ms
        p.extend((-1500i32).to_le_bytes());
        p.extend(2437u16.to_le_bytes());
        p.extend([0, 0]);
        let tp = TimTp::parse(&p).unwrap();
        let gps = (GPS_EPOCH_UNIX_S + 2437 * WEEK_S + 345_600) * 1_000_000_000 + 500_000;
        assert_eq!(tp.utc_ns(Some(18)), Ok(gps - 18_000_000_000));
        assert_eq!(tp.utc_ns(None), Err("no_leap_seconds"));
        assert_eq!(tp.qerr(), Some(-1500));
        let utc_based = TimTp { flags: 1, ..tp };
        assert_eq!(utc_based.utc_ns(None), Ok(gps));
        assert_eq!(TimTp { ref_info: 3, ..tp }.utc_ns(Some(18)), Err("tp_time_base"));
    }

    #[test]
    fn version_generation() {
        let mut p = vec![0u8; 40];
        p[..12].copy_from_slice(b"ROM SPG 5.10");
        p[30..38].copy_from_slice(b"000A0000");
        let mut ext = [0u8; 30];
        ext[..13].copy_from_slice(b"PROTVER=34.10");
        p.extend(ext);
        let v = Version::parse(&p).unwrap();
        assert_eq!(v.protocol(), Some(34.1));
        assert!(v.uses_valset());
        let mut old = vec![0u8; 40];
        old[..12].copy_from_slice(b"6.02 (36023)");
        old[30..38].copy_from_slice(b"00040005");
        let v = Version::parse(&old).unwrap();
        assert_eq!(v.protocol(), None);
        assert!(!v.uses_valset());
    }

    #[test]
    fn valset_encoding() {
        let f = valset_ram(&[(key::uart_baud(1), 38400), (key::uart_out_nmea(1), 0), (key::msgout(TIM_TP, 1), 1)]);
        let (id, p) = split(&f);
        assert_eq!(id, CFG_VALSET);
        assert_eq!(&p[..4], &[0, 1, 0, 0]);
        assert_eq!(&p[4..12], &[0x01, 0x00, 0x52, 0x40, 0x00, 0x96, 0x00, 0x00]);
        assert_eq!(&p[12..17], &[0x02, 0x00, 0x74, 0x10, 0x00]);
        assert_eq!(&p[17..22], &[0x7E, 0x01, 0x91, 0x20, 0x01]);
    }

    #[test]
    fn cfg_prt_keeps_port_and_input() {
        let mut polled = vec![0u8; 20];
        polled[0] = 1;
        polled[12] = 0x07;
        polled[14] = 0x03;
        let f = cfg_prt(Some(&polled), 38400);
        let (_, p) = split(&f);
        assert_eq!(u32_at(p, 8), 38400);
        assert_eq!((p[0], p[12], u16_at(p, 14)), (1, 0x07, 0x0001));
    }
}
