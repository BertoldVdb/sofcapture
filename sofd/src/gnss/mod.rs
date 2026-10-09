//! sofgnss boards: USB GPS bridges that timestamp PPS and SOF on one counter,
//! giving their controller's frame times in GNSS UTC.

pub mod device;
pub mod external;
pub mod firmware;
pub mod framer;
pub mod join;
pub mod nmea;
pub mod session;
pub mod timing;
pub mod ubx;

use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};
use timing::Tag;

const MAX_QUEUED_TAGS: usize = 256;

/// Tags from one board or MQTT client, waiting for the sampler.
pub struct Feed {
    /// USB serial of the board, or `mqtt:<client>`.
    pub serial: String,
    /// Set for an MQTT reference.
    pub client: Option<String>,
    /// See `join::BOARD_MAX_LAG`.
    pub max_lag: i64,
    tags: Mutex<VecDeque<Tag>>,
}

impl Feed {
    pub fn new(serial: String) -> Self {
        Self { serial, client: None, max_lag: join::BOARD_MAX_LAG, tags: Mutex::new(VecDeque::new()) }
    }

    pub fn external(client: &str) -> Self {
        Self {
            serial: format!("{}{client}", external::NAME_PREFIX),
            client: Some(client.into()),
            max_lag: join::MQTT_MAX_LAG,
            tags: Mutex::new(VecDeque::new()),
        }
    }

    pub fn push(&self, t: Tag) {
        let mut q = self.tags.lock().unwrap();
        if q.len() == MAX_QUEUED_TAGS {
            q.pop_front();
        }
        q.push_back(t);
    }

    pub fn drain(&self) -> Vec<Tag> {
        self.tags.lock().unwrap().drain(..).collect()
    }
}

/// Feeds by controller id.
pub type Hub = Arc<Mutex<BTreeMap<String, Arc<Feed>>>>;
