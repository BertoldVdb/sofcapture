//! sofgps boards: USB GPS bridges that timestamp PPS and SOF on one counter,
//! giving their controller's frame times in GNSS UTC.

pub mod device;
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

/// SOF tags of one attached device, waiting for its controller's sampler.
pub struct Feed {
    /// USB serial of the board.
    pub serial: String,
    tags: Mutex<VecDeque<Tag>>,
}

impl Feed {
    pub fn new(serial: String) -> Self {
        Self { serial, tags: Mutex::new(VecDeque::new()) }
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
