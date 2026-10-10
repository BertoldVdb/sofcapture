//! MQTT 5 client. Topics, all below the prefix:
//!
//! - `status` (retained, set offline by the will)
//! - `controllers/<id>/info`, `model` (retained), `sample`, `event`
//! - `controllers/<id>/want/<client>` (retained, from clients): sampling request
//! - `controllers/<id>/ref/<client>` (from clients): SOF tags from an external GNSS reference
//! - `gnss/<serial>/status` (retained), `pps`, `event`, raw frames `nmea/<address>`,
//!   `ubx/<class>/<id>`, `rtcm3/<type>`, and `tx` (from clients, sent to the receiver)
//! - `cmd/<name>`: replies go to the MQTT 5 response topic, or `reply_to` in the body

use crate::config;
use crate::daemon::Msg;
use anyhow::{Context, Result};
use bytes::Bytes;
use log::{debug, info, warn};
use rumqttc::v5::mqttbytes::v5::{LastWill, Packet, Publish, PublishProperties};
use rumqttc::v5::mqttbytes::QoS;
use rumqttc::v5::{Client, Event, MqttOptions, RecvTimeoutError};
use rumqttc::Outgoing;
use serde_json::{json, Value};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

const KEEP_ALIVE: Duration = Duration::from_secs(10);
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
/// Publishes are dropped when this queue is full.
const QUEUE: usize = 4096;
/// Also sent to the broker, so it drops bigger messages instead of us disconnecting.
const MAX_PACKET: u32 = 256 * 1024;
const SUBSCRIPTIONS: [&str; 4] = ["controllers/+/want/+", "controllers/+/ref/+", "gnss/+/tx", "cmd/+"];

/// Received from MQTT.
#[derive(Debug, PartialEq)]
pub enum In {
    /// Republish retained state.
    Connected,
    /// `req` None: request withdrawn.
    Want { controller: String, client: String, req: Option<WantReq> },
    Tx { serial: String, data: Vec<u8> },
    /// `t_rx`: arrival time, CLOCK_MONOTONIC_RAW.
    Ref { controller: String, client: String, payload: Vec<u8>, t_rx: crate::clock::Ns },
    /// `payload` has the reply fields removed.
    Cmd { name: String, payload: Vec<u8>, reply: Reply },
}

#[derive(Debug, Default, PartialEq)]
pub struct Reply {
    /// `reply_to` from the body, else the MQTT 5 response topic.
    pub topic: Option<String>,
    /// Sent back as a property.
    pub correlation_data: Option<Bytes>,
    /// Echoed in the reply body.
    pub correlation: Option<Value>,
}

pub const MAX_TTL_S: f64 = 86_400.0;

/// A client's `want` request.
#[derive(Debug, Default, PartialEq)]
pub struct WantReq {
    /// None: the default rate.
    pub rate_hz: Option<f64>,
    /// How often to publish `model`; None: the configured rate.
    pub model_hz: Option<f64>,
    /// Publish every sample on `sample`.
    pub samples: bool,
    /// Lapses unless renewed.
    pub ttl_s: Option<f64>,
}

/// None and null are absent; anything but a positive number is an error.
fn positive(req: &serde_json::Map<String, Value>, key: &str) -> Result<Option<f64>, String> {
    match req.get(key) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => match v.as_f64() {
            Some(r) if r > 0.0 && r.is_finite() => Ok(Some(r)),
            _ => Err(format!("{key} {v} is not a positive number")),
        },
    }
}

/// Non-JSON bodies pass through unchanged.
fn command_request(prefix: &str, topic: &str, payload: &[u8], props: Option<&PublishProperties>) -> (Vec<u8>, Reply) {
    let mut reply = Reply {
        topic: props.and_then(|pr| pr.response_topic.clone()),
        correlation_data: props.and_then(|pr| pr.correlation_data.clone()),
        correlation: None,
    };
    let mut request = payload.to_vec();
    if let Ok(Value::Object(mut body)) = serde_json::from_slice::<Value>(payload) {
        match body.remove("reply_to") {
            None => {}
            Some(Value::String(t)) if !t.is_empty() && !t.contains(['+', '#']) => reply.topic = Some(t),
            Some(v) => warn!("mqtt: {topic}: reply_to {v} is not a topic name"),
        }
        reply.correlation = body.remove("correlation");
        request = Value::Object(body).to_string().into_bytes();
    }
    // A reply inside our own tree could act as a request.
    if reply.topic.as_ref().is_some_and(|t| t == prefix || t.starts_with(&format!("{prefix}/"))) {
        warn!("mqtt: {topic}: not replying inside {prefix}/");
        reply.topic = None;
    }
    (request, reply)
}

pub struct Mqtt {
    client: Client,
    prefix: String,
    stop: Arc<AtomicBool>,
    /// While down, publishes are dropped: queued samples would block the resubscribe.
    connected: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

pub fn hostname() -> String {
    let mut b = [0u8; 256];
    // SAFETY: the buffer is valid for its length.
    if unsafe { libc::gethostname(b.as_mut_ptr().cast(), b.len()) } != 0 {
        return "localhost".into();
    }
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

impl Mqtt {
    pub fn connect(cfg: &config::Mqtt, out: Sender<Msg>) -> Result<Self> {
        let host = hostname();
        let prefix = cfg.prefix.clone().unwrap_or_else(|| format!("sofd/{host}"));
        let id = cfg.client_id.clone().unwrap_or_else(|| format!("sofd-{host}"));
        let mut o = MqttOptions::new(id, &cfg.host, cfg.port);
        o.set_keep_alive(KEEP_ALIVE);
        o.set_clean_start(true);
        o.set_max_packet_size(Some(MAX_PACKET));
        o.set_last_will(LastWill::new(format!("{prefix}/status"), json!({"state": "offline"}).to_string(), QoS::AtLeastOnce, true, None));
        let password = match (&cfg.password, &cfg.password_file) {
            (_, Some(f)) => Some(
                std::fs::read_to_string(f)
                    .with_context(|| format!("reading mqtt password file {}", f.display()))?
                    .trim_end()
                    .to_string(),
            ),
            (p, None) => p.clone(),
        };
        if let Some(user) = &cfg.username {
            o.set_credentials(user, password.unwrap_or_default());
        }
        let (client, mut conn) = Client::new(o, QUEUE);
        let stop = Arc::new(AtomicBool::new(false));
        let connected = Arc::new(AtomicBool::new(false));
        let (c, s, p, up) = (client.clone(), stop.clone(), prefix.clone(), connected.clone());
        let broker = format!("{}:{}", cfg.host, cfg.port);
        let thread = std::thread::Builder::new()
            .name("mqtt".into())
            .spawn(move || {
                let mut last_error: Option<String> = None;
                // Retried when the queue was full.
                let mut pending: Vec<&str> = vec![];
                // Keep going until shutdown()'s disconnect has been sent.
                loop {
                    pending.retain(|t| match c.try_subscribe(format!("{p}/{t}"), QoS::AtLeastOnce) {
                        Ok(()) => false,
                        Err(e) => {
                            debug!("mqtt: subscribing {t}: {e}; retrying");
                            true
                        }
                    });
                    let ev = match conn.recv_timeout(Duration::from_millis(500)) {
                        Ok(ev) => ev,
                        Err(RecvTimeoutError::Timeout) => continue,
                        Err(RecvTimeoutError::Disconnected) => break,
                    };
                    match ev {
                        Ok(Event::Incoming(Packet::ConnAck(_))) => {
                            info!("mqtt: connected to {broker}, prefix {p}");
                            last_error = None;
                            up.store(true, Ordering::Relaxed);
                            pending = SUBSCRIPTIONS.to_vec();
                            if out.send(Msg::Mqtt(In::Connected)).is_err() {
                                break;
                            }
                        }
                        Ok(Event::Incoming(Packet::Publish(publish))) => {
                            if let Some(m) = parse(&p, &publish) {
                                if out.send(Msg::Mqtt(m)).is_err() {
                                    break;
                                }
                            }
                        }
                        Ok(Event::Outgoing(Outgoing::Disconnect)) if s.load(Ordering::Relaxed) => break,
                        Ok(_) => {}
                        Err(e) => {
                            up.store(false, Ordering::Relaxed);
                            if s.load(Ordering::Relaxed) {
                                break;
                            }
                            let e = e.to_string();
                            if last_error.as_ref() != Some(&e) {
                                warn!("mqtt: {broker}: {e}; retrying");
                                last_error = Some(e);
                            }
                            std::thread::sleep(RECONNECT_DELAY);
                        }
                    }
                }
            })
            .context("starting mqtt thread")?;
        Ok(Self { client, prefix, stop, connected, thread: Some(thread) })
    }

    fn up(&self, topic: &str) -> bool {
        let up = self.connected.load(Ordering::Relaxed);
        if !up {
            debug!("mqtt: not connected, dropped {topic}");
        }
        up
    }

    /// Dropped while disconnected or when the queue is full.
    pub fn publish(&self, topic: &str, payload: impl Into<Bytes>, retain: bool) {
        if !self.up(topic) {
            return;
        }
        let qos = if retain { QoS::AtLeastOnce } else { QoS::AtMostOnce };
        if let Err(e) = self.client.try_publish(format!("{}/{topic}", self.prefix), qos, retain, payload) {
            debug!("mqtt: dropped {topic}: {e}");
        }
    }

    pub fn publish_json(&self, topic: &str, v: &Value, retain: bool) {
        self.publish(topic, v.to_string(), retain);
    }

    pub fn publish_event(&self, topic: &str, v: &Value) {
        if !self.up(topic) {
            return;
        }
        if let Err(e) = self.client.try_publish(format!("{}/{topic}", self.prefix), QoS::AtLeastOnce, false, v.to_string()) {
            debug!("mqtt: dropped {topic}: {e}");
        }
    }

    pub fn reply(&self, topic: &str, correlation: Option<Bytes>, v: &Value) {
        if !self.up(topic) {
            return;
        }
        let props = PublishProperties { correlation_data: correlation, ..Default::default() };
        if let Err(e) = self.client.try_publish_with_properties(topic, QoS::AtLeastOnce, false, v.to_string(), props) {
            debug!("mqtt: dropped reply to {topic}: {e}");
        }
    }

    /// Waits up to a second for the broker.
    pub fn shutdown(mut self) {
        self.publish_json("status", &json!({"state": "offline"}), true);
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.client.try_disconnect();
        let t0 = Instant::now();
        while self.thread.as_ref().is_some_and(|t| !t.is_finished()) && t0.elapsed() < Duration::from_secs(1) {
            std::thread::sleep(Duration::from_millis(20));
        }
        if let Some(t) = self.thread.take().filter(|t| t.is_finished()) {
            let _ = t.join();
        }
    }
}

fn parse(prefix: &str, p: &Publish) -> Option<In> {
    let topic = std::str::from_utf8(&p.topic).ok()?;
    let rest = topic.strip_prefix(prefix)?.strip_prefix('/')?;
    let parts: Vec<&str> = rest.split('/').collect();
    match parts.as_slice() {
        ["controllers", ctrl, "want", client] => {
            let req = if p.payload.is_empty() {
                None
            } else {
                // Ignore bad requests: "off" or {"rate_hz":0} must not start sampling.
                let req = match serde_json::from_slice::<Value>(&p.payload) {
                    Ok(Value::Object(o)) => o,
                    Ok(_) | Err(_) => {
                        warn!("mqtt: {topic}: not a JSON object; request ignored");
                        return None;
                    }
                };
                let (rate_hz, model_hz) = match (positive(&req, "rate_hz"), positive(&req, "model_hz")) {
                    (Ok(r), Ok(m)) => (r, m),
                    (Err(e), _) | (_, Err(e)) => {
                        warn!("mqtt: {topic}: {e}; request ignored");
                        return None;
                    }
                };
                let samples = match req.get("samples") {
                    None | Some(Value::Null) => false,
                    Some(Value::Bool(b)) => *b,
                    Some(v) => {
                        warn!("mqtt: {topic}: samples {v} is not true or false; request ignored");
                        return None;
                    }
                };
                let ttl_s = match req.get("ttl_s") {
                    None => None,
                    Some(t) => match t.as_f64() {
                        Some(t) if t > 0.0 && t <= MAX_TTL_S => Some(t),
                        // Not "no TTL": that would keep a dead client's request forever.
                        _ => {
                            warn!("mqtt: {topic}: ttl_s {t} is not in (0, {MAX_TTL_S}] s; request ignored");
                            return None;
                        }
                    },
                };
                Some(WantReq { rate_hz, model_hz, samples, ttl_s })
            };
            Some(In::Want { controller: ctrl.to_string(), client: client.to_string(), req })
        }
        ["controllers", ctrl, "ref", client] if !p.payload.is_empty() => Some(In::Ref {
            controller: ctrl.to_string(),
            client: client.to_string(),
            payload: p.payload.to_vec(),
            t_rx: crate::clock::raw(),
        }),
        ["gnss", serial, "tx"] if !p.payload.is_empty() => Some(In::Tx { serial: serial.to_string(), data: p.payload.to_vec() }),
        ["cmd", name] => {
            let (payload, reply) = command_request(prefix, topic, &p.payload, p.properties.as_ref());
            Some(In::Cmd { name: name.to_string(), payload, reply })
        }
        _ => None,
    }
}

/// Below `gnss/<serial>/`.
pub fn frame_topic(kind: crate::gnss::framer::Kind, f: &[u8]) -> String {
    use crate::gnss::framer::Kind;
    match kind {
        Kind::Nmea => {
            let addr = f[1..].split(|&c| c == b',' || c == b'*').next().unwrap_or_default();
            format!("nmea/{}", String::from_utf8_lossy(addr))
        }
        Kind::Ubx => format!("ubx/{:02x}/{:02x}", f[2], f[3]),
        Kind::Rtcm3 => format!("rtcm3/{}", (u16::from(f[3]) << 4) | u16::from(f[4] >> 4)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss::framer::Kind;

    fn publish(topic: &str, payload: &[u8]) -> Publish {
        Publish::new(topic, QoS::AtLeastOnce, payload.to_vec(), None)
    }

    #[test]
    fn incoming_topics() {
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/0000:00:14.0/want/app1", br#"{"rate_hz":20}"#));
        let req = |r: Option<f64>| Some(WantReq { rate_hz: r, ..Default::default() });
        assert_eq!(w, Some(In::Want { controller: "0000:00:14.0".into(), client: "app1".into(), req: req(Some(20.0)) }));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", b""));
        assert_eq!(w, Some(In::Want { controller: "c".into(), client: "app1".into(), req: None }));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", b"{}"));
        assert!(matches!(w, Some(In::Want { req: Some(WantReq { rate_hz: None, model_hz: None, samples: false, ttl_s: None }), .. })));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", br#"{"rate_hz":20,"ttl_s":10}"#));
        assert!(matches!(w, Some(In::Want { req: Some(WantReq { rate_hz: Some(r), ttl_s: Some(t), .. }), .. }) if r == 20.0 && t == 10.0));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", br#"{"model_hz":5,"samples":true}"#));
        assert!(matches!(w, Some(In::Want { req: Some(WantReq { rate_hz: None, model_hz: Some(m), samples: true, .. }), .. }) if m == 5.0));
        for bad in [r#"{"ttl_s":0}"#, r#"{"ttl_s":-1}"#, r#"{"ttl_s":"10"}"#, r#"{"ttl_s":1e9}"#, "off", "null", "[1]", r#"{"rate_hz":0}"#, r#"{"rate_hz":"20"}"#, r#"{"model_hz":0}"#, r#"{"samples":1}"#] {
            assert_eq!(parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", bad.as_bytes())), None, "{bad}");
        }
        assert_eq!(parse("sofd/pc", &publish("sofd/pc/gnss/E07/tx", b"\xb5b")), Some(In::Tx { serial: "E07".into(), data: b"\xb5b".to_vec() }));
        assert_eq!(parse("sofd/pc", &publish("sofd/pcx/gnss/E07/tx", b"x")), None);
        let r = parse("sofd/pc", &publish("sofd/pc/controllers/c/ref/app1", b"{}"));
        assert!(matches!(r, Some(In::Ref { controller, client, .. }) if controller == "c" && client == "app1"));
    }

    #[test]
    fn command_replies() {
        // MQTT 3.1.1 style: everything in the body.
        let c = parse("sofd/pc", &publish("sofd/pc/cmd/lookup", br#"{"port":"1-2","reply_to":"ci/42","correlation":{"job":7}}"#));
        let Some(In::Cmd { name, payload, reply }) = c else { panic!("{c:?}") };
        assert_eq!(name, "lookup");
        assert_eq!(serde_json::from_slice::<Value>(&payload).unwrap(), json!({"port": "1-2"}));
        assert_eq!(reply, Reply { topic: Some("ci/42".into()), correlation_data: None, correlation: Some(json!({"job": 7})) });

        // MQTT 5 properties; the body's reply_to wins over the response topic.
        let props = PublishProperties {
            response_topic: Some("me/reply".into()),
            correlation_data: Some(Bytes::from_static(b"x")),
            ..Default::default()
        };
        let mut p = publish("sofd/pc/cmd/list", b"");
        p.properties = Some(props.clone());
        let Some(In::Cmd { payload, reply, .. }) = parse("sofd/pc", &p) else { panic!() };
        assert!(payload.is_empty());
        assert_eq!(reply, Reply { topic: Some("me/reply".into()), correlation_data: Some(Bytes::from_static(b"x")), correlation: None });
        let mut p = publish("sofd/pc/cmd/list", br#"{"reply_to":"ci/1"}"#);
        p.properties = Some(props);
        let Some(In::Cmd { reply, .. }) = parse("sofd/pc", &p) else { panic!() };
        assert_eq!(reply.topic.as_deref(), Some("ci/1"));

        // Wildcards, empty or non-string topics are not reply topics.
        for bad in [r#"{"reply_to":"ci/#"}"#, r#"{"reply_to":"ci/+/x"}"#, r#"{"reply_to":""}"#, r#"{"reply_to":5}"#, r#"{"reply_to":"sofd/pc/gnss/E07/tx"}"#] {
            let Some(In::Cmd { reply, payload, .. }) = parse("sofd/pc", &publish("sofd/pc/cmd/list", bad.as_bytes())) else { panic!() };
            assert_eq!(reply.topic, None, "{bad}");
            assert_eq!(payload, b"{}");
        }
        // Not JSON: passed on for the command to reject.
        let Some(In::Cmd { payload, .. }) = parse("sofd/pc", &publish("sofd/pc/cmd/lookup", b"nope")) else { panic!() };
        assert_eq!(payload, b"nope");
    }

    #[test]
    fn frame_topics() {
        assert_eq!(frame_topic(Kind::Nmea, b"$GPZDA,1*00"), "nmea/GPZDA");
        assert_eq!(frame_topic(Kind::Ubx, &crate::gnss::ubx::frame(crate::gnss::ubx::TIM_TP, &[])), "ubx/0d/01");
        // RTCM3: 0xD3, length, then a 12-bit message number.
        assert_eq!(frame_topic(Kind::Rtcm3, &[0xD3, 0x00, 0x13, 0x3E, 0xD0, 0, 0, 0]), "rtcm3/1005");
    }
}
