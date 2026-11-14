//! The configuration file: HuJSON (JSON with comments and trailing commas).

use crate::gnss::session::SEARCH_BAUDS;
use crate::gnss::ubx::Constellation;
use anyhow::{Context, Result};
use serde::{Deserialize, Deserializer};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// Without it, sofd does not connect to a broker.
    pub mqtt: Option<Mqtt>,
    pub sampler: Sampler,
    pub gnss: Gnss,
    /// Per USB controller, by sofd's id (e.g. "0000:01:00.0").
    pub controllers: BTreeMap<String, ControllerOptions>,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ControllerOptions {
    /// How long after the host sees the frame counter step the frame starts on the bus.
    /// Unset: the measured value for known controllers, else 0.
    pub counter_lead_ns: Option<i64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mqtt {
    #[serde(default = "Mqtt::default_host")]
    pub host: String,
    #[serde(default = "Mqtt::default_port")]
    pub port: u16,
    /// Topic prefix; default `sofd/<hostname>`.
    pub prefix: Option<String>,
    /// Default `sofd-<hostname>`.
    pub client_id: Option<String>,
    pub username: Option<String>,
    pub password: Option<String>,
    /// Read the password from this file (trailing whitespace removed).
    pub password_file: Option<PathBuf>,
}

impl Mqtt {
    fn default_host() -> String {
        "localhost".into()
    }

    fn default_port() -> u16 {
        1883
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Sampler {
    /// Samples in the rate/offset fit.
    pub window: usize,
    /// Span of GNSS tags in the model_gnss fit.
    pub gnss_window_s: f64,
    /// Run sampler threads SCHED_FIFO at this priority.
    pub rt_prio: Option<i32>,
    /// Pin sampler threads to this CPU.
    pub cpu: Option<usize>,
    /// Rate for boards' controllers and for requests that give none.
    pub default_rate_hz: f64,
    /// How often `model` is published, unless a client asks for more.
    pub model_hz: f64,
    /// Cap CPU wake-up latency while sampling (/dev/cpu_dma_latency).
    pub max_cpu_latency_us: Option<u32>,
    /// While sampling, use this cpufreq governor (on `cpu` if set, else all CPUs).
    pub cpu_governor: Option<String>,
}

impl Default for Sampler {
    fn default() -> Self {
        Self { window: 64, gnss_window_s: 6.0, rt_prio: None, cpu: None, default_rate_hz: 50.0, model_hz: 1.0, max_cpu_latency_us: None, cpu_governor: None }
    }
}

#[derive(Debug, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Gnss {
    pub attach: Attach,
    /// u-blox modules on their own USB port, by name (USB serial, else `usb-<port>`).
    pub usb_receivers: Attach,
    pub defaults: BoardOptions,
    /// Per-serial overrides of `defaults`.
    pub boards: BTreeMap<String, BoardOptions>,
    /// MQTT clients allowed to act as GNSS reference, by client name. `*` allows any other.
    pub external: BTreeMap<String, External>,
    /// Receivers without a board, by name: on a serial port, or options for a USB module.
    pub receivers: BTreeMap<String, Receiver>,
}

impl Default for Gnss {
    fn default() -> Self {
        Self {
            attach: Attach::All,
            usb_receivers: Attach::None,
            defaults: BoardOptions::default(),
            boards: BTreeMap::new(),
            external: BTreeMap::new(),
            receivers: BTreeMap::new(),
        }
    }
}

/// A receiver without a sofgnss board. Unset options fall back to `gnss.defaults`.
/// Without `tty` these are the options of the USB module of that name.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Receiver {
    pub tty: String,
    /// A /dev/pps device, or `pps_gpio`: a Raspberry Pi GPIO polled around each pulse.
    /// Neither: no pulses.
    pub pps: Option<String>,
    pub pps_gpio: Option<u8>,
    #[serde(deserialize_with = "nullable")]
    pub serial_delay_ns: Option<Option<i64>>,
    pub baud: Option<u32>,
    pub initial_baud: Option<u32>,
    pub configure: Option<bool>,
    pub strict: Option<bool>,
    pub leap_seconds: Option<i8>,
    pub constellations: Option<Vec<Constellation>>,
    pub frames: Option<bool>,
    pub fix: Option<bool>,
    #[serde(deserialize_with = "nullable")]
    pub chrony: Option<Option<String>>,
}

impl Receiver {
    fn has_pulses(&self) -> bool {
        self.pps.is_some() || self.pps_gpio.is_some()
    }

    fn options(&self) -> BoardOptions {
        BoardOptions {
            baud: self.baud,
            initial_baud: self.initial_baud,
            configure: self.configure,
            strict: self.strict,
            leap_seconds: self.leap_seconds,
            constellations: self.constellations.clone(),
            frames: self.frames,
            fix: self.fix,
            chrony: self.chrony.clone(),
            serial_delay_ns: self.serial_delay_ns,
            ..BoardOptions::default()
        }
    }
}

/// An MQTT client that may publish SOF tags (`controllers/<id>/ref/<client>`).
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct External {
    /// chrony SOCK refclock path fed from this reference.
    pub chrony: Option<PathBuf>,
    /// Subtracted from every tag's time.
    pub sof_delay_ns: i64,
}

/// Which sofgnss boards sofd uses.
#[derive(Debug, Clone, PartialEq, Default)]
pub enum Attach {
    #[default]
    All,
    None,
    Serials(Vec<String>),
}

impl<'de> Deserialize<'de> for Attach {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum Raw {
            Word(String),
            List(Vec<String>),
        }
        match Raw::deserialize(d)? {
            Raw::Word(w) if w == "all" => Ok(Attach::All),
            Raw::Word(w) if w == "none" => Ok(Attach::None),
            Raw::Word(w) => Err(serde::de::Error::custom(format!("attach: expected \"all\", \"none\" or a list of serials, got \"{w}\""))),
            Raw::List(l) => Ok(Attach::Serials(l)),
        }
    }
}

impl Attach {
    pub fn includes(&self, serial: &str) -> bool {
        match self {
            Attach::All => true,
            Attach::None => false,
            Attach::Serials(l) => l.iter().any(|s| s == serial),
        }
    }
}

/// Unset fields fall back to `defaults`, then built-ins; `null` clears `chrony`, `firmware`
/// and `serial_delay_ns`.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BoardOptions {
    pub baud: Option<u32>,
    pub initial_baud: Option<u32>,
    pub configure: Option<bool>,
    pub strict: Option<bool>,
    pub leap_seconds: Option<i8>,
    pub constellations: Option<Vec<Constellation>>,
    pub frames: Option<bool>,
    pub fix: Option<bool>,
    pub div: Option<u32>,
    pub sof_delay_ns: Option<i64>,
    #[serde(deserialize_with = "nullable")]
    pub chrony: Option<Option<String>>,
    #[serde(deserialize_with = "nullable")]
    pub firmware: Option<Option<PathBuf>>,
    /// Receivers without pulses: date each epoch by the arrival of the receiver's output,
    /// this much after the epoch.
    #[serde(deserialize_with = "nullable")]
    pub serial_delay_ns: Option<Option<i64>>,
}

fn nullable<'de, D: Deserializer<'de>, T: Deserialize<'de>>(d: D) -> Result<Option<Option<T>>, D::Error> {
    Option::<T>::deserialize(d).map(Some)
}

/// Options of one board after merging.
#[derive(Debug, Clone, PartialEq)]
pub struct Board {
    /// Baud sofd switches the receiver to.
    pub baud: u32,
    /// Receiver's power-on baud; the only one used without `configure`.
    pub initial_baud: u32,
    pub configure: bool,
    /// Never fall back to NMEA ZDA, keep retrying the setup.
    pub strict: bool,
    /// GPS minus UTC until the receiver knows it.
    pub leap_seconds: Option<i8>,
    pub constellations: Option<Vec<Constellation>>,
    pub frames: bool,
    /// Publish a fix per navigation epoch.
    pub fix: bool,
    pub div: Option<u32>,
    pub sof_delay_ns: Option<i64>,
    /// chrony SOCK refclock path, `{serial}` substituted.
    pub chrony: Option<PathBuf>,
    pub firmware: Option<PathBuf>,
    /// Receivers only: serial timing.
    pub serial_delay_ns: Option<i64>,
}

impl Gnss {
    pub fn board(&self, serial: &str) -> Board {
        Board { serial_delay_ns: None, ..self.merge(serial, self.boards.get(serial).cloned().unwrap_or_default()) }
    }

    /// Board-only options (`div`, `sof_delay_ns`, `firmware`) don't apply.
    pub fn receiver(&self, name: &str) -> Board {
        let r = self.receivers.get(name);
        let b = self.merge(name, r.map(Receiver::options).unwrap_or_default());
        let pulses = r.is_some_and(Receiver::has_pulses);
        Board { div: None, sof_delay_ns: None, firmware: None, serial_delay_ns: b.serial_delay_ns.filter(|_| !pulses), ..b }
    }

    fn merge(&self, serial: &str, b: BoardOptions) -> Board {
        let d = &self.defaults;
        Board {
            baud: b.baud.or(d.baud).unwrap_or(38_400),
            initial_baud: b.initial_baud.or(d.initial_baud).unwrap_or(9600),
            configure: b.configure.or(d.configure).unwrap_or(true),
            strict: b.strict.or(d.strict).unwrap_or(false),
            leap_seconds: b.leap_seconds.or(d.leap_seconds),
            constellations: b.constellations.or(d.constellations.clone()),
            frames: b.frames.or(d.frames).unwrap_or(true),
            fix: b.fix.or(d.fix).unwrap_or(true),
            div: b.div.or(d.div),
            sof_delay_ns: b.sof_delay_ns.or(d.sof_delay_ns),
            chrony: b.chrony.or(d.chrony.clone()).flatten().map(|p| PathBuf::from(p.replace("{serial}", serial))),
            firmware: b.firmware.or(d.firmware.clone()).flatten(),
            serial_delay_ns: b.serial_delay_ns.or(d.serial_delay_ns).flatten(),
        }
    }

    pub fn external(&self, client: &str) -> Option<&External> {
        self.external.get(client).or_else(|| self.external.get("*"))
    }
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
        Self::parse(&text).with_context(|| format!("in {}", path.display()))
    }

    pub fn parse(text: &str) -> Result<Self> {
        let cfg: Config = serde_json::from_str(&strip_hujson(text))?;
        if let Some(m) = &cfg.mqtt {
            if m.prefix.as_deref().is_some_and(|p| p.is_empty() || p.ends_with('/') || p.contains(['+', '#'])) {
                anyhow::bail!("mqtt.prefix must be non-empty, without wildcards or a trailing '/'");
            }
        }
        cfg.validate()?;
        Ok(cfg)
    }

    /// Range checks serde can't express.
    fn validate(&self) -> Result<()> {
        let s = &self.sampler;
        let (lo, hi) = (crate::sampler::MIN_RATE_HZ, crate::sampler::MAX_RATE_HZ);
        if !(lo..=hi).contains(&s.default_rate_hz) {
            anyhow::bail!("sampler.default_rate_hz must be within {lo}..={hi}");
        }
        if !(lo..=hi).contains(&s.model_hz) {
            anyhow::bail!("sampler.model_hz must be within {lo}..={hi}");
        }
        if !(3..=100_000).contains(&s.window) {
            anyhow::bail!("sampler.window must be within 3..=100000");
        }
        if !(1.0..=600.0).contains(&s.gnss_window_s) {
            anyhow::bail!("sampler.gnss_window_s must be within 1..=600");
        }
        if s.cpu.is_some_and(|c| c >= libc::CPU_SETSIZE as usize) {
            anyhow::bail!("sampler.cpu must be below {}", libc::CPU_SETSIZE);
        }
        if s.rt_prio.is_some_and(|p| !(1..=99).contains(&p)) {
            anyhow::bail!("sampler.rt_prio must be within 1..=99");
        }
        for (id, c) in &self.controllers {
            if c.counter_lead_ns.is_some_and(|l| l.abs() > 1_000_000) {
                anyhow::bail!("controllers.{id}.counter_lead_ns must be within ±1 ms");
            }
        }
        if let Some(serial) = self.gnss.boards.iter().find_map(|(k, b)| b.serial_delay_ns.map(|_| k)) {
            anyhow::bail!("gnss.boards.{serial}.serial_delay_ns: boards have pulses, this is for receivers without");
        }
        for (name, r) in &self.gnss.receivers {
            if name.is_empty() || name.contains(['/', '+', '#']) {
                anyhow::bail!("gnss.receivers: name {name:?} must be non-empty, without '/' or wildcards");
            }
            if self.gnss.boards.contains_key(name) {
                anyhow::bail!("gnss.receivers.{name}: also a board in gnss.boards");
            }
            let pulses = r.has_pulses();
            if r.tty.is_empty() && pulses {
                anyhow::bail!("gnss.receivers.{name}: pps and pps_gpio need a tty (without one these are a USB module's options)");
            }
            if (r.pps.is_some() && r.pps_gpio.is_some()) || r.pps.as_ref().is_some_and(|p| p.is_empty()) {
                anyhow::bail!("gnss.receivers.{name}: give at most one of pps or pps_gpio");
            }
            if r.serial_delay_ns.flatten().is_some() && pulses {
                anyhow::bail!("gnss.receivers.{name}.serial_delay_ns is for receivers without pulses");
            }
            if r.pps_gpio.is_some_and(|n| n > 53) {
                anyhow::bail!("gnss.receivers.{name}.pps_gpio must be a GPIO number (0-53)");
            }
        }
        let receivers = self.gnss.receivers.iter().map(|(k, r)| (format!("gnss.receivers.{k}"), r.options(), self.gnss.receiver(k)));
        let boards = (self.gnss.boards.iter().map(|(k, b)| (format!("gnss.boards.{k}"), b.clone(), self.gnss.board(k))))
            .chain([("gnss.defaults".to_string(), self.gnss.defaults.clone(), self.gnss.board(""))])
            .chain(receivers);
        for (what, b, merged) in boards {
            if b.serial_delay_ns.flatten().is_some_and(|d| !(0..=1_000_000_000).contains(&d)) {
                anyhow::bail!("{what}.serial_delay_ns must be within 0..=1000000000");
            }
            if merged.strict && !merged.configure {
                anyhow::bail!("{what}: strict needs configure");
            }
            if let Some(c) = &b.constellations {
                // Pulses are labelled on the GPS grid, with its leap seconds.
                if !c.contains(&Constellation::Gps) {
                    anyhow::bail!("{what}.constellations must include \"gps\"");
                }
                if c.iter().enumerate().any(|(i, g)| c[..i].contains(g)) {
                    anyhow::bail!("{what}.constellations lists one twice");
                }
            }
            if b.leap_seconds.is_some_and(|l| !(0..=60).contains(&l)) {
                anyhow::bail!("{what}.leap_seconds must be within 0..=60");
            }
            if b.div.is_some_and(|d| !(1..=1_000_000).contains(&d)) {
                anyhow::bail!("{what}.div must be within 1..=1000000");
            }
            for (name, baud) in [("baud", b.baud), ("initial_baud", b.initial_baud)] {
                if baud.is_some_and(|r| !SEARCH_BAUDS.contains(&r)) {
                    let mut rates = SEARCH_BAUDS;
                    rates.sort();
                    anyhow::bail!("{what}.{name} must be one of {rates:?}");
                }
            }
        }
        let delays = (self.gnss.boards.iter().map(|(k, b)| (format!("gnss.boards.{k}"), b.sof_delay_ns)))
            .chain([("gnss.defaults".to_string(), self.gnss.defaults.sof_delay_ns)])
            .chain(self.gnss.external.iter().map(|(k, e)| (format!("gnss.external.{k}"), Some(e.sof_delay_ns))));
        for (what, d) in delays {
            if d.is_some_and(|d| d.abs() > 1_000_000_000) {
                anyhow::bail!("{what}.sof_delay_ns must be within ±1 s");
            }
        }
        Ok(())
    }
}

/// Comments and trailing commas become spaces, so error positions still match.
pub fn strip_hujson(s: &str) -> String {
    let mut b: Vec<u8> = s.as_bytes().to_vec();
    let (mut i, mut in_str) = (0, false);
    // Pending comma: blanked if a closing bracket comes next.
    let mut comma: Option<usize> = None;
    while i < b.len() {
        let c = b[i];
        if in_str {
            match c {
                b'\\' => i += 1,
                b'"' => in_str = false,
                _ => {}
            }
            i += 1;
            continue;
        }
        match (c, b.get(i + 1)) {
            (b'/', Some(b'/')) => {
                while i < b.len() && b[i] != b'\n' {
                    b[i] = b' ';
                    i += 1;
                }
                continue;
            }
            (b'/', Some(b'*')) => {
                let end = (i + 2..b.len().saturating_sub(1)).find(|&j| b[j] == b'*' && b[j + 1] == b'/');
                let end = end.map_or(b.len(), |j| j + 2);
                for x in &mut b[i..end] {
                    if *x != b'\n' {
                        *x = b' ';
                    }
                }
                i = end;
                continue;
            }
            (b'}' | b']', _) => {
                if let Some(j) = comma.take() {
                    b[j] = b' ';
                }
            }
            (b',', _) => comma = Some(i),
            (b'"', _) => {
                in_str = true;
                comma = None;
            }
            (c, _) if !c.is_ascii_whitespace() => comma = None,
            _ => {}
        }
        i += 1;
    }
    String::from_utf8(b).expect("only ASCII bytes were replaced")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hujson() {
        let s = "{\n // c\n \"a\": \"x//y\", /* b,\n */ \"l\": [1, 2,],\n}";
        let v: serde_json::Value = serde_json::from_str(&strip_hujson(s)).unwrap();
        assert_eq!(v, serde_json::json!({"a": "x//y", "l": [1, 2]}));
        assert_eq!(strip_hujson(r#"{"a":"\",}"}"#), r#"{"a":"\",}"}"#);
    }

    #[test]
    fn board_merging() {
        let c = Config::parse(
            r#"{
              "gnss": {
                "attach": ["A1"],
                "defaults": { "chrony": "/run/chrony.{serial}.sock", "initial_baud": 115200, "firmware": "/fw.bin" },
                "boards": { "A1": { "baud": 115200, "chrony": null }, "B2": { "firmware": null } },
                "external": { "app": { "chrony": "/run/c.sock", "sof_delay_ns": 40 }, "other": {} },
              },
            }"#,
        )
        .unwrap();
        assert!(c.mqtt.is_none());
        assert_eq!(c.gnss.attach, Attach::Serials(vec!["A1".into()]));
        let a = c.gnss.board("A1");
        assert_eq!((a.baud, a.initial_baud, a.chrony, a.firmware.as_deref()), (115_200, 115_200, None, Some(Path::new("/fw.bin"))));
        let b = c.gnss.board("B2");
        assert_eq!((b.baud, b.chrony.as_deref(), b.firmware), (38_400, Some(Path::new("/run/chrony.B2.sock")), None));
        assert_eq!(c.gnss.external["app"].chrony.as_deref(), Some(Path::new("/run/c.sock")));
        assert_eq!((c.gnss.external["app"].sof_delay_ns, c.gnss.external["other"].chrony.is_none()), (40, true));
        assert!(c.gnss.external("sdrrx").is_none());
        let c = Config::parse(
            r#"{"gnss":{"defaults":{"chrony":"/run/c.{serial}.sock","div":10,"baud":115200},
                "receivers":{"ttyS0":{"tty":"/dev/ttyS0","pps":"/dev/pps0","initial_baud":38400}}}}"#,
        )
        .unwrap();
        let r = c.gnss.receiver("ttyS0");
        assert_eq!((r.baud, r.initial_baud, r.div, r.chrony.as_deref()), (115_200, 38_400, None, Some(Path::new("/run/c.ttyS0.sock"))));
        assert_eq!(c.gnss.receivers["ttyS0"].pps.as_deref(), Some("/dev/pps0"));
        let c = Config::parse(r#"{"gnss":{"receivers":{"g":{"tty":"/dev/ttyAMA3","pps_gpio":20}}}}"#).unwrap();
        assert_eq!(c.gnss.receivers["g"].pps_gpio, Some(20));
        // Without pulses, and the options of a USB module.
        let c = Config::parse(
            r#"{"gnss":{"usb_receivers":"all","defaults":{"chrony":"/run/c.sock"},
                "receivers":{"u":{"tty":"/dev/ttyS1"},"usb-1-1.3":{"serial_delay_ns":45000000,"baud":115200}}}}"#,
        )
        .unwrap();
        assert_eq!(c.gnss.usb_receivers, Attach::All);
        assert_eq!((c.gnss.receivers["u"].pps.is_none(), c.gnss.receivers["u"].pps_gpio), (true, None));
        let u = c.gnss.receiver("usb-1-1.3");
        assert_eq!((u.serial_delay_ns, u.baud, u.chrony.as_deref()), (Some(45_000_000), 115_200, Some(Path::new("/run/c.sock"))));
        assert_eq!(c.gnss.receiver("other").serial_delay_ns, None);
        // A default delay is for every receiver without pulses.
        let c = Config::parse(
            r#"{"gnss":{"defaults":{"serial_delay_ns":38000000},"receivers":{"p":{"tty":"/dev/ttyS0","pps":"/dev/pps0"},
                "t":{"tty":"/dev/ttyS1"},"a":{"serial_delay_ns":5},"off":{"serial_delay_ns":null}}}}"#,
        )
        .unwrap();
        let delay = |n| c.gnss.receiver(n).serial_delay_ns;
        assert_eq!((delay("usb-1-4"), delay("t"), delay("a"), delay("off"), delay("p")), (Some(38_000_000), Some(38_000_000), Some(5), None, None));
        assert_eq!(c.gnss.board("A").serial_delay_ns, None);
        let c = Config::parse(r#"{"controllers":{"0000:01:00.0":{"counter_lead_ns":128000}}}"#).unwrap();
        assert_eq!(c.controllers["0000:01:00.0"].counter_lead_ns, Some(128_000));
        let c = Config::parse(r#"{"gnss":{"defaults":{"constellations":["gps","galileo","beidou"]},"boards":{"A":{"constellations":["gps"]}}}}"#).unwrap();
        assert_eq!(c.gnss.board("B").constellations, Some(vec![Constellation::Gps, Constellation::Galileo, Constellation::Beidou]));
        assert_eq!(c.gnss.board("A").constellations, Some(vec![Constellation::Gps]));
        let c = Config::parse(r#"{"gnss":{"external":{"*":{"sof_delay_ns":5},"app":{}}}}"#).unwrap();
        assert_eq!((c.gnss.external("sdrrx").unwrap().sof_delay_ns, c.gnss.external("app").unwrap().sof_delay_ns), (5, 0));
        assert_eq!(Config::parse("{}").unwrap().gnss.attach, Attach::All);
        assert_eq!(Config::parse("{}").unwrap().gnss.usb_receivers, Attach::None);
        assert_eq!(Config::default().gnss.usb_receivers, Attach::None);
        assert!(Config::parse(r#"{"gnss":{"attach":"some"}}"#).is_err());
        assert!(Config::parse(r#"{"mqtt":{"prefix":"a/#"}}"#).is_err());
        for bad in [
            r#"{"sampler":{"default_rate_hz":0}}"#,
            r#"{"sampler":{"default_rate_hz":5000}}"#,
            r#"{"sampler":{"model_hz":0}}"#,
            r#"{"sampler":{"window":2}}"#,
            r#"{"sampler":{"gnss_window_s":0.5}}"#,
            r#"{"gnss":{"defaults":{"strict":true,"configure":false}}}"#,
            r#"{"gnss":{"defaults":{"leap_seconds":-1}}}"#,
            r#"{"controllers":{"0000:01:00.0":{"counter_lead_ns":2000000}}}"#,
            r#"{"gnss":{"defaults":{"constellations":["galileo"]}}}"#,
            r#"{"gnss":{"defaults":{"constellations":["gps","gps"]}}}"#,
            r#"{"gnss":{"defaults":{"constellations":["gps","navic"]}}}"#,
            r#"{"gnss":{"defaults":{"constellations":[]}}}"#,
            r#"{"gnss":{"receivers":{"a":{"pps":"/dev/pps0"}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps":""}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps":"/dev/pps0","serial_delay_ns":0}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"serial_delay_ns":-1}}}}"#,
            r#"{"gnss":{"defaults":{"serial_delay_ns":2000000000}}}"#,
            r#"{"gnss":{"boards":{"A":{"serial_delay_ns":0}}}}"#,
            r#"{"gnss":{"usb_receivers":"some"}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps":"/dev/pps0","pps_gpio":20}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps_gpio":60}}}}"#,
            r#"{"gnss":{"receivers":{"a/b":{"tty":"/dev/ttyS0","pps":"/dev/pps0"}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps":"/dev/pps0","div":5}}}}"#,
            r#"{"gnss":{"receivers":{"a":{"tty":"/dev/ttyS0","pps":"/dev/pps0","baud":1234}}}}"#,
            r#"{"gnss":{"boards":{"a":{}},"receivers":{"a":{"tty":"/dev/ttyS0","pps":"/dev/pps0"}}}}"#,
            r#"{"gnss":{"defaults":{"strict":true},"boards":{"A":{"configure":false}}}}"#,
            r#"{"sampler":{"cpu":5000}}"#,
            r#"{"sampler":{"rt_prio":0}}"#,
            r#"{"gnss":{"defaults":{"sof_delay_ns":2000000000}}}"#,
            r#"{"gnss":{"external":{"a":{"sof_delay_ns":-2000000000}}}}"#,
            r#"{"gnss":{"defaults":{"div":0}}}"#,
            r#"{"gnss":{"boards":{"A1":{"div":1000001}}}}"#,
            r#"{"gnss":{"defaults":{"baud":12345}}}"#,
            r#"{"gnss":{"boards":{"A1":{"initial_baud":1200}}}}"#,
        ] {
            assert!(Config::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn example_file_parses() {
        let c = Config::load(&Path::new(env!("CARGO_MANIFEST_DIR")).join("sofd.example.hujson")).unwrap();
        assert_eq!(c.mqtt.unwrap().port, 1883);
        assert_eq!(c.gnss.board("X").initial_baud, 9600);
    }
}
