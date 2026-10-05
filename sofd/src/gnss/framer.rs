//! Splits a sofgps byte stream into NMEA, UBX and RTCM3 frames, using the same
//! rules as the firmware's FrameParser. Invalid bytes are dropped.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Nmea,
    Ubx,
    Rtcm3,
}

const MAX_NMEA: usize = 256;
/// Whole frame, header and checksum included.
pub const MAX_UBX: usize = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Nmea,
    NmeaCk1,
    NmeaCk2,
    UbxSync,
    UbxHeader,
    UbxBody,
    RtcmHeader,
    RtcmBody,
}

pub struct Framer {
    buf: Vec<u8>,
    state: State,
    need: usize,
    xor: u8,
    ck_hi: u8,
}

fn hex(b: u8) -> Option<u8> {
    (b as char).to_digit(16).map(|v| v as u8)
}

pub fn ubx_checksum(body: &[u8]) -> [u8; 2] {
    let (mut a, mut b) = (0u8, 0u8);
    for &x in body {
        a = a.wrapping_add(x);
        b = b.wrapping_add(a);
    }
    [a, b]
}

/// CRC-24Q as used by RTCM3.
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc = 0u32;
    for &x in data {
        crc ^= (x as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x100_0000 != 0 {
                crc ^= 0x186_4CFB;
            }
        }
    }
    crc & 0xFF_FFFF
}

impl Framer {
    pub fn new() -> Self {
        Self { buf: Vec::with_capacity(MAX_UBX), state: State::Idle, need: 0, xor: 0, ck_hi: 0 }
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.buf.clear();
    }

    /// NMEA frames come as `$...*hh` without the line ending.
    pub fn feed(&mut self, data: &[u8], mut f: impl FnMut(Kind, &[u8])) {
        for &b in data {
            if let Some(kind) = self.byte(b) {
                f(kind, &self.buf);
            }
        }
    }

    fn start(&mut self, b: u8) {
        self.buf.clear();
        self.state = match b {
            b'$' | b'!' => State::Nmea,
            0xB5 => State::UbxSync,
            0xD3 => State::RtcmHeader,
            _ => State::Idle,
        };
        if self.state != State::Idle {
            self.buf.push(b);
            self.xor = 0;
        }
    }

    fn byte(&mut self, b: u8) -> Option<Kind> {
        match self.state {
            State::Idle => self.start(b),
            State::Nmea => match b {
                b'*' => {
                    self.buf.push(b);
                    self.state = State::NmeaCk1;
                }
                // Leave room for "*hh\r\n", as the firmware does.
                0x20..=0x7e if b != b'$' && b != b'!' => {
                    if self.buf.len() + 5 > MAX_NMEA {
                        self.state = State::Idle;
                    } else {
                        self.buf.push(b);
                        self.xor ^= b;
                    }
                }
                _ => self.start(b),
            },
            State::NmeaCk1 | State::NmeaCk2 => match hex(b) {
                Some(v) if self.state == State::NmeaCk1 => {
                    self.buf.push(b);
                    self.ck_hi = v;
                    self.state = State::NmeaCk2;
                }
                Some(v) => {
                    self.buf.push(b);
                    self.state = State::Idle;
                    if (self.ck_hi << 4 | v) == self.xor {
                        return Some(Kind::Nmea);
                    }
                }
                None => self.start(b),
            },
            State::UbxSync => {
                if b == 0x62 {
                    self.buf.push(b);
                    self.state = State::UbxHeader;
                } else {
                    self.start(b);
                }
            }
            State::UbxHeader => {
                self.buf.push(b);
                if self.buf.len() == 6 {
                    self.need = 8 + u16::from_le_bytes([self.buf[4], self.buf[5]]) as usize;
                    self.state = if self.need > MAX_UBX { State::Idle } else { State::UbxBody };
                }
            }
            State::UbxBody => {
                self.buf.push(b);
                if self.buf.len() == self.need {
                    self.state = State::Idle;
                    let n = self.need;
                    if ubx_checksum(&self.buf[2..n - 2]) == self.buf[n - 2..] {
                        return Some(Kind::Ubx);
                    }
                }
            }
            State::RtcmHeader => {
                self.buf.push(b);
                if self.buf.len() == 3 {
                    if self.buf[1] & 0xFC != 0 {
                        self.state = State::Idle;
                    } else {
                        self.need = 6 + (((self.buf[1] & 3) as usize) << 8 | self.buf[2] as usize);
                        self.state = State::RtcmBody;
                    }
                }
            }
            State::RtcmBody => {
                self.buf.push(b);
                if self.buf.len() == self.need {
                    self.state = State::Idle;
                    let n = self.need;
                    if crc24q(&self.buf[..n - 3]).to_be_bytes()[1..] == self.buf[n - 3..] {
                        return Some(Kind::Rtcm3);
                    }
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frames(data: &[u8]) -> Vec<(Kind, Vec<u8>)> {
        let mut out = vec![];
        Framer::new().feed(data, |k, f| out.push((k, f.to_vec())));
        out
    }

    #[test]
    fn splits_mixed_stream() {
        let mut s = b"junk$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n".to_vec();
        s.extend([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]);
        let rtcm_body = [0xD3, 0x00, 0x02, 0x3E, 0xD0];
        let crc = crc24q(&rtcm_body).to_be_bytes();
        s.extend(rtcm_body);
        s.extend(&crc[1..]);
        s.extend(b"$GPXXX,1*00\r\n"); // bad checksum
        s.extend(b"$GPXXX,no checksum\r\n");
        let f = frames(&s);
        assert_eq!(f.len(), 3, "{f:?}");
        assert_eq!(f[0].0, Kind::Nmea);
        assert!(f[0].1.starts_with(b"$GPGGA") && f[0].1.ends_with(b"*47"));
        assert_eq!(f[1], (Kind::Ubx, vec![0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34]));
        assert_eq!(f[2].0, Kind::Rtcm3);
    }

    #[test]
    fn oversize_ubx_header_resyncs() {
        let mut s = vec![0xB5, 0x62, 0x01, 0x02, 0xFF, 0xFF];
        s.extend(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
        assert_eq!(frames(&s).len(), 1);
    }
}
