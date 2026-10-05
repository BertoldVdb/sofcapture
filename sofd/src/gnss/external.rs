//! SOF tags from MQTT clients, on `controllers/<id>/ref/<client>`.

use super::timing::At;
use crate::clock::Ns;
use serde::Deserialize;
use serde_json::Value;

pub const NAME_PREFIX: &str = "mqtt:";
/// 1σ
pub const DEFAULT_UNC_NS: Ns = 100;
/// More would overflow the tag queue.
pub const MAX_BATCH: usize = 256;
pub const LOST_AFTER_S: u64 = 10;

#[derive(Debug, Clone, PartialEq)]
pub struct ExtTag {
    pub at: At,
    pub t_gnss_ns: Ns,
    pub unc_ns: Ns,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Raw {
    frame: Option<u64>,
    microframe: Option<u8>,
    epoch: Option<u64>,
    counter: Option<i64>,
    t_gnss_ns: Ns,
    unc_ns: Option<Ns>,
}

/// One tag, or `{"tags":[…]}`.
pub fn parse(payload: &[u8]) -> Result<Vec<ExtTag>, String> {
    let v: Value = serde_json::from_slice(payload).map_err(|e| e.to_string())?;
    let raws: Vec<Raw> = match v.get("tags") {
        Some(t) => serde_json::from_value(t.clone()).map_err(|e| format!("tags: {e}"))?,
        None => vec![serde_json::from_value(v).map_err(|e| e.to_string())?],
    };
    if raws.len() > MAX_BATCH {
        return Err(format!("{} tags in one message, at most {MAX_BATCH}", raws.len()));
    }
    raws.into_iter().map(tag).collect()
}

fn tag(r: Raw) -> Result<ExtTag, String> {
    let at = match (r.frame, r.epoch, r.counter) {
        (Some(frame), None, None) => {
            let microframe = r.microframe.unwrap_or(0);
            if microframe > 7 {
                return Err(format!("microframe {microframe}: 0-7"));
            }
            At::Frame { frame, microframe }
        }
        (None, Some(epoch), Some(counter)) if r.microframe.is_none() => At::Counter { epoch, counter },
        _ => return Err("give either frame (and microframe) or epoch and counter".into()),
    };
    let unc_ns = r.unc_ns.unwrap_or(DEFAULT_UNC_NS);
    if unc_ns <= 0 {
        return Err(format!("unc_ns {unc_ns}: must be positive"));
    }
    Ok(ExtTag { at, t_gnss_ns: r.t_gnss_ns, unc_ns })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forms_and_batches() {
        let t = parse(br#"{"frame":9213374,"t_gnss_ns":1791151735000000000,"unc_ns":20}"#).unwrap();
        assert_eq!(t, [ExtTag { at: At::Frame { frame: 9_213_374, microframe: 0 }, t_gnss_ns: 1_791_151_735_000_000_000, unc_ns: 20 }]);
        let t = parse(br#"{"tags":[{"frame":5,"microframe":3,"t_gnss_ns":1},{"epoch":7,"counter":-2,"t_gnss_ns":2}]}"#).unwrap();
        assert_eq!(t[0].at, At::Frame { frame: 5, microframe: 3 });
        assert_eq!((t[1].at, t[1].unc_ns), (At::Counter { epoch: 7, counter: -2 }, DEFAULT_UNC_NS));
    }

    #[test]
    fn rejects() {
        for bad in [
            &br#"{"frame":1}"#[..],                                       // no time
            br#"{"frame":1,"t_gnss_ns":1.79e18}"#,                         // not an integer
            br#"{"frame":1,"epoch":2,"counter":3,"t_gnss_ns":1}"#,         // both forms
            br#"{"epoch":2,"t_gnss_ns":1}"#,                               // no counter
            br#"{"epoch":2,"counter":3,"microframe":1,"t_gnss_ns":1}"#,    // microframe is in the counter
            br#"{"frame":1,"microframe":8,"t_gnss_ns":1}"#,
            br#"{"frame":1,"t_gnss_ns":1,"unc_ns":0}"#,
            br#"{"frame":1,"t_gnss_ns":1,"extra":0}"#,
            b"not json",
        ] {
            assert!(parse(bad).is_err(), "{}", String::from_utf8_lossy(bad));
        }
        let many = format!(r#"{{"tags":[{}]}}"#, vec![r#"{"frame":1,"t_gnss_ns":1}"#; MAX_BATCH + 1].join(","));
        assert!(parse(many.as_bytes()).is_err());
    }
}
