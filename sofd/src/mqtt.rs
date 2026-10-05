//! MQTT 5 client. Topics, all below the prefix:
//!
//! - `status` (retained; the will sets it offline)
//! - `controllers/<id>/info|model` (retained), `.../sample`, `.../event`
//! - `controllers/<id>/want/<client>` (retained, written by clients): sample this controller
//! - `controllers/<id>/ref/<client>` (written by clients): SOF tags of an external GNSS reference
//! - `gnss/<serial>/status` (retained), `.../pps`, `.../event`, `.../nmea/<address>`,
//!   `.../ubx/<class>/<id>`, `.../rtcm3/<type>` (raw frames), `.../tx` (client → receiver)
//! - `cmd/<name>` (request/response through the MQTT 5 response topic)

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

/// Received from MQTT.
#[derive(Debug, PartialEq)]
pub enum In {
    /// Republish retained state.
    Connected,
    /// A sampling request; `rate_hz` None = the request was withdrawn.
    Want { controller: String, client: String, rate_hz: Option<Option<f64>> },
    Tx { serial: String, data: Vec<u8> },
    /// `t_rx`: arrival time, CLOCK_MONOTONIC_RAW.
    Ref { controller: String, client: String, payload: Vec<u8>, t_rx: crate::clock::Ns },
    Cmd { name: String, payload: Vec<u8>, reply_to: Option<(String, Option<Bytes>)> },
}

pub struct Mqtt {
    client: Client,
    prefix: String,
    stop: Arc<AtomicBool>,
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
        let (c, s, p) = (client.clone(), stop.clone(), prefix.clone());
        let broker = format!("{}:{}", cfg.host, cfg.port);
        let thread = std::thread::Builder::new()
            .name("mqtt".into())
            .spawn(move || {
                let mut last_error: Option<String> = None;
                // Keep going until shutdown()'s disconnect has been sent.
                loop {
                    let ev = match conn.recv_timeout(Duration::from_millis(500)) {
                        Ok(ev) => ev,
                        Err(RecvTimeoutError::Timeout) => continue,
                        Err(RecvTimeoutError::Disconnected) => break,
                    };
                    match ev {
                        Ok(Event::Incoming(Packet::ConnAck(_))) => {
                            info!("mqtt: connected to {broker}, prefix {p}");
                            last_error = None;
                            for t in ["controllers/+/want/+", "controllers/+/ref/+", "gnss/+/tx", "cmd/+"] {
                                if let Err(e) = c.try_subscribe(format!("{p}/{t}"), QoS::AtLeastOnce) {
                                    warn!("mqtt: subscribing {t}: {e}");
                                }
                            }
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
        Ok(Self { client, prefix, stop, thread: Some(thread) })
    }

    /// Queues a publish below the prefix; dropped (with a debug log) when the queue is full.
    pub fn publish(&self, topic: &str, payload: impl Into<Bytes>, retain: bool) {
        let qos = if retain { QoS::AtLeastOnce } else { QoS::AtMostOnce };
        if let Err(e) = self.client.try_publish(format!("{}/{topic}", self.prefix), qos, retain, payload) {
            debug!("mqtt: dropped {topic}: {e}");
        }
    }

    pub fn publish_json(&self, topic: &str, v: &Value, retain: bool) {
        self.publish(topic, v.to_string(), retain);
    }

    pub fn publish_event(&self, topic: &str, v: &Value) {
        if let Err(e) = self.client.try_publish(format!("{}/{topic}", self.prefix), QoS::AtLeastOnce, false, v.to_string()) {
            debug!("mqtt: dropped {topic}: {e}");
        }
    }

    pub fn reply(&self, topic: &str, correlation: Option<Bytes>, v: &Value) {
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
            let rate_hz = if p.payload.is_empty() {
                None
            } else {
                let v: Value = serde_json::from_slice(&p.payload).unwrap_or_else(|e| {
                    warn!("mqtt: {topic}: {e}; using the default rate");
                    Value::Null
                });
                Some(v.get("rate_hz").and_then(Value::as_f64))
            };
            Some(In::Want { controller: ctrl.to_string(), client: client.to_string(), rate_hz })
        }
        ["controllers", ctrl, "ref", client] if !p.payload.is_empty() => Some(In::Ref {
            controller: ctrl.to_string(),
            client: client.to_string(),
            payload: p.payload.to_vec(),
            t_rx: crate::clock::raw(),
        }),
        ["gnss", serial, "tx"] if !p.payload.is_empty() => Some(In::Tx { serial: serial.to_string(), data: p.payload.to_vec() }),
        ["cmd", name] => {
            let props = p.properties.as_ref();
            let reply_to = props.and_then(|pr| pr.response_topic.clone()).map(|t| (t, props.and_then(|pr| pr.correlation_data.clone())));
            Some(In::Cmd { name: name.to_string(), payload: p.payload.to_vec(), reply_to })
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
        assert_eq!(w, Some(In::Want { controller: "0000:00:14.0".into(), client: "app1".into(), rate_hz: Some(Some(20.0)) }));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", b""));
        assert_eq!(w, Some(In::Want { controller: "c".into(), client: "app1".into(), rate_hz: None }));
        let w = parse("sofd/pc", &publish("sofd/pc/controllers/c/want/app1", b"{}"));
        assert!(matches!(w, Some(In::Want { rate_hz: Some(None), .. })));
        assert_eq!(parse("sofd/pc", &publish("sofd/pc/gnss/E07/tx", b"\xb5b")), Some(In::Tx { serial: "E07".into(), data: b"\xb5b".to_vec() }));
        assert_eq!(parse("sofd/pc", &publish("sofd/pcx/gnss/E07/tx", b"x")), None);
        let r = parse("sofd/pc", &publish("sofd/pc/controllers/c/ref/app1", b"{}"));
        assert!(matches!(r, Some(In::Ref { controller, client, .. }) if controller == "c" && client == "app1"));
    }

    #[test]
    fn frame_topics() {
        assert_eq!(frame_topic(Kind::Nmea, b"$GPZDA,1*00"), "nmea/GPZDA");
        assert_eq!(frame_topic(Kind::Ubx, &crate::gnss::ubx::frame(crate::gnss::ubx::TIM_TP, &[])), "ubx/0d/01");
        // RTCM3: 0xD3, length, then a 12-bit message number.
        assert_eq!(frame_topic(Kind::Rtcm3, &[0xD3, 0x00, 0x13, 0x3E, 0xD0, 0, 0, 0]), "rtcm3/1005");
    }
}
