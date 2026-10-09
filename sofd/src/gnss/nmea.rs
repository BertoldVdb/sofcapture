//! The NMEA sentences sofd reads (ZDA, RMC, GGA) and sofgnss `$PVDB,SOFPPS`.

use crate::clock::Ns;

pub const PVDB_PREFIX: &[u8] = b"$PVDB,SOFPPS,";

/// Fields between `$` and `*hh`.
pub fn fields(frame: &[u8]) -> Vec<&str> {
    let body = &frame[1..frame.len().saturating_sub(3).max(1)];
    std::str::from_utf8(body).unwrap_or("").split(',').collect()
}

pub fn sentence(body: &str) -> Vec<u8> {
    let ck = body.bytes().fold(0u8, |a, b| a ^ b);
    format!("${body}*{ck:02X}\r\n").into_bytes()
}

pub fn command(fields: &str) -> Vec<u8> {
    sentence(&format!("PVDB,SOFPPS,{fields}"))
}

/// "GPZDA" -> "ZDA"
fn kind<'a>(f: &[&'a str]) -> &'a str {
    f.first().filter(|a| a.len() == 5).map_or("", |a| &a[2..])
}

fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (m + if m > 2 { -3 } else { 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

#[derive(Debug, PartialEq)]
pub enum Info {
    /// UTC ns, or why it can't label a pulse.
    Zda(Result<Ns, &'static str>),
    Fix(bool),
    Other,
}

pub fn parse(frame: &[u8]) -> Info {
    let f = fields(frame);
    match kind(&f) {
        "ZDA" if f.len() >= 5 => Info::Zda(zda(&f)),
        "RMC" if f.len() > 2 => Info::Fix(f[2] == "A"),
        "GGA" if f.len() > 6 => Info::Fix(f[6].parse::<u32>().is_ok_and(|q| q > 0)),
        _ => Info::Other,
    }
}

fn zda(f: &[&str]) -> Result<Ns, &'static str> {
    let t = f[1];
    let num = |s: &str| s.parse::<i64>().map_err(|_| "zda_empty");
    if t.len() < 6 {
        return Err("zda_empty");
    }
    let (hh, mm) = (num(&t[0..2])?, num(&t[2..4])?);
    let secs: f64 = t[4..].parse().map_err(|_| "zda_empty")?;
    if secs.fract() != 0.0 {
        return Err("zda_fraction");
    }
    let (d, m, y) = (num(f[2])?, num(f[3])?, num(f[4])?);
    let s = days_from_civil(y, m, d) * 86_400 + hh * 3600 + mm * 60 + secs as i64;
    Ok(s * 1_000_000_000)
}

#[derive(Debug, PartialEq)]
pub enum Pvdb<'a> {
    Pps { tick: u64, rx_idle_ms: u32 },
    Sof { tick: u64, frame: u64 },
    Info { tick_hz: u32, tag: &'a str },
    Ver { version: &'a str, elf_sha256: &'a str, reset_reason: &'a str },
    Ota(OtaMsg),
    Err(&'a str),
    Other,
}

#[derive(Debug, PartialEq)]
pub enum OtaMsg {
    Ready,
    /// Bytes received.
    Ack(u64),
    Ok,
}

pub fn pvdb(frame: &[u8]) -> Pvdb<'_> {
    let f = fields(frame);
    let n = |i: usize| f.get(i).and_then(|s| s.parse::<u64>().ok());
    match f.get(2).copied() {
        Some("PPS") => match (n(3), n(4)) {
            (Some(tick), Some(idle)) => Pvdb::Pps { tick, rx_idle_ms: idle as u32 },
            _ => Pvdb::Other,
        },
        Some("SOF") => match (n(3), n(4)) {
            (Some(tick), Some(frame)) => Pvdb::Sof { tick, frame },
            _ => Pvdb::Other,
        },
        Some("INFO") => match n(3) {
            Some(hz) => Pvdb::Info { tick_hz: hz as u32, tag: f.get(10).copied().unwrap_or("") },
            None => Pvdb::Other,
        },
        Some("VER") if f.len() >= 9 => Pvdb::Ver { version: f[3], elf_sha256: f[7], reset_reason: f[8] },
        Some("OTA") => match (f.get(3).copied(), n(4)) {
            (Some("READY"), _) => Pvdb::Ota(OtaMsg::Ready),
            (Some("ACK"), Some(b)) => Pvdb::Ota(OtaMsg::Ack(b)),
            (Some("OK"), _) => Pvdb::Ota(OtaMsg::Ok),
            _ => Pvdb::Other,
        },
        Some("ERR") => Pvdb::Err(f.get(3).copied().unwrap_or("")),
        _ => Pvdb::Other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(body: &str) -> Vec<u8> {
        let mut s = sentence(body);
        s.truncate(s.len() - 2);
        s
    }

    #[test]
    fn zda_and_fix() {
        let t = (days_from_civil(2026, 10, 3) * 86_400 + 12 * 3600 + 34 * 60 + 56) * 1_000_000_000;
        assert_eq!(parse(&frame("GPZDA,123456.00,03,10,2026,00,00")), Info::Zda(Ok(t)));
        assert_eq!(parse(&frame("GNZDA,123456.50,03,10,2026,00,00")), Info::Zda(Err("zda_fraction")));
        assert_eq!(parse(&frame("GPZDA,,,,,,")), Info::Zda(Err("zda_empty")));
        assert_eq!(parse(&frame("GPRMC,123456.00,A,1,N,2,E,0,0,031026,,,A")), Info::Fix(true));
        assert_eq!(parse(&frame("GPGGA,123456.00,,,,,0,00,99.99,,,,,,")), Info::Fix(false));
        assert_eq!(days_from_civil(1970, 1, 1), 0);
        assert_eq!(days_from_civil(2000, 3, 1), 11_017);
    }

    #[test]
    fn pvdb_sentences() {
        assert_eq!(pvdb(&frame("PVDB,SOFPPS,PPS,800000000,312")), Pvdb::Pps { tick: 800_000_000, rx_idle_ms: 312 });
        assert_eq!(pvdb(&frame("PVDB,SOFPPS,SOF,12345,6789")), Pvdb::Sof { tick: 12345, frame: 6789 });
        assert_eq!(
            pvdb(&frame("PVDB,SOFPPS,INFO,80000000,100,1,2,3,4,5,sofd1")),
            Pvdb::Info { tick_hz: 80_000_000, tag: "sofd1" }
        );
        assert_eq!(
            pvdb(&frame("PVDB,SOFPPS,VER,v1,v5.5.1,ota_0,0,abcd,SW")),
            Pvdb::Ver { version: "v1", elf_sha256: "abcd", reset_reason: "SW" }
        );
        assert_eq!(pvdb(&frame("PVDB,SOFPPS,OTA,ACK,32768")), Pvdb::Ota(OtaMsg::Ack(32768)));
        assert_eq!(pvdb(&frame("PVDB,SOFPPS,OTA,OK")), Pvdb::Ota(OtaMsg::Ok));
        assert_eq!(command("PING"), b"$PVDB,SOFPPS,PING*19\r\n".to_vec());
    }
}
