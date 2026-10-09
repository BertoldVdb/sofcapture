//! The configuration file: HuJSON (JSON with comments and trailing commas).

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
    /// Run sampler threads SCHED_FIFO at this priority.
    pub rt_prio: Option<i32>,
    /// Pin sampler threads to this CPU.
    pub cpu: Option<usize>,
    /// Rate for boards' controllers and for requests that give none.
    pub default_rate_hz: f64,
    /// Cap CPU wake-up latency while sampling (/dev/cpu_dma_latency).
    pub max_cpu_latency_us: Option<u32>,
    /// While sampling, use this cpufreq governor (on `cpu` if set, else all CPUs).
    pub cpu_governor: Option<String>,
}

impl Default for Sampler {
    fn default() -> Self {
        Self { window: 64, rt_prio: None, cpu: None, default_rate_hz: 50.0, max_cpu_latency_us: None, cpu_governor: None }
    }
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Gnss {
    pub attach: Attach,
    pub defaults: BoardOptions,
    /// Per-serial overrides of `defaults`.
    pub boards: BTreeMap<String, BoardOptions>,
    /// MQTT clients allowed to act as GNSS reference, by client name.
    pub external: BTreeMap<String, External>,
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

/// Unset fields fall back to `defaults`, then built-ins; `null` clears `chrony`/`firmware`.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct BoardOptions {
    pub baud: Option<u32>,
    pub initial_baud: Option<u32>,
    pub configure: Option<bool>,
    pub frames: Option<bool>,
    pub div: Option<u32>,
    pub sof_delay_ns: Option<i64>,
    #[serde(deserialize_with = "nullable")]
    pub chrony: Option<Option<String>>,
    #[serde(deserialize_with = "nullable")]
    pub firmware: Option<Option<PathBuf>>,
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
    pub frames: bool,
    pub div: Option<u32>,
    pub sof_delay_ns: i64,
    /// chrony SOCK refclock path, `{serial}` substituted.
    pub chrony: Option<PathBuf>,
    pub firmware: Option<PathBuf>,
}

impl Gnss {
    pub fn board(&self, serial: &str) -> Board {
        let (d, b) = (&self.defaults, self.boards.get(serial).cloned().unwrap_or_default());
        Board {
            baud: b.baud.or(d.baud).unwrap_or(38_400),
            initial_baud: b.initial_baud.or(d.initial_baud).unwrap_or(9600),
            configure: b.configure.or(d.configure).unwrap_or(true),
            frames: b.frames.or(d.frames).unwrap_or(true),
            div: b.div.or(d.div),
            sof_delay_ns: b.sof_delay_ns.or(d.sof_delay_ns).unwrap_or(0),
            chrony: b.chrony.or(d.chrony.clone()).flatten().map(|p| PathBuf::from(p.replace("{serial}", serial))),
            firmware: b.firmware.or(d.firmware.clone()).flatten(),
        }
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
        if !(3..=100_000).contains(&s.window) {
            anyhow::bail!("sampler.window must be within 3..=100000");
        }
        if s.cpu.is_some_and(|c| c >= libc::CPU_SETSIZE as usize) {
            anyhow::bail!("sampler.cpu must be below {}", libc::CPU_SETSIZE);
        }
        if s.rt_prio.is_some_and(|p| !(1..=99).contains(&p)) {
            anyhow::bail!("sampler.rt_prio must be within 1..=99");
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
        assert_eq!(Config::parse("{}").unwrap().gnss.attach, Attach::All);
        assert!(Config::parse(r#"{"gnss":{"attach":"some"}}"#).is_err());
        assert!(Config::parse(r#"{"mqtt":{"prefix":"a/#"}}"#).is_err());
        for bad in [
            r#"{"sampler":{"default_rate_hz":0}}"#,
            r#"{"sampler":{"default_rate_hz":5000}}"#,
            r#"{"sampler":{"window":2}}"#,
            r#"{"sampler":{"cpu":5000}}"#,
            r#"{"sampler":{"rt_prio":0}}"#,
            r#"{"gnss":{"defaults":{"sof_delay_ns":2000000000}}}"#,
            r#"{"gnss":{"external":{"a":{"sof_delay_ns":-2000000000}}}}"#,
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
