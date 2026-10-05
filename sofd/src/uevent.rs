//! Kernel uevents, so USB discovery runs only when the bus changes.

use anyhow::{bail, Result};
use log::{debug, warn};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Raw kernel events; udev's processed ones are group 2.
const KERNEL_GROUP: u32 = 1;

pub struct Watch {
    /// Set on USB/tty events or lost events; the daemon clears it when it scans.
    pub changed: AtomicBool,
    /// Cleared if the socket fails; the daemon then scans every time.
    pub alive: AtomicBool,
}

pub fn watch() -> Result<Arc<Watch>> {
    let w = Arc::new(Watch { changed: AtomicBool::new(true), alive: AtomicBool::new(true) });
    // SAFETY: plain socket calls on a descriptor we own.
    let fd = unsafe { libc::socket(libc::AF_NETLINK, libc::SOCK_DGRAM | libc::SOCK_CLOEXEC, libc::NETLINK_KOBJECT_UEVENT) };
    if fd < 0 {
        bail!("netlink uevent socket: {}", std::io::Error::last_os_error());
    }
    // SAFETY: sockaddr_nl is plain data; fd is valid.
    let r = unsafe {
        let mut addr: libc::sockaddr_nl = std::mem::zeroed();
        addr.nl_family = libc::AF_NETLINK as libc::sa_family_t;
        addr.nl_groups = KERNEL_GROUP;
        // Big buffer for bursts (a hub with many devices). FORCE needs root.
        let size: libc::c_int = 1 << 20;
        let p = (&size as *const libc::c_int).cast();
        let len = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
        if libc::setsockopt(fd, libc::SOL_SOCKET, libc::SO_RCVBUFFORCE, p, len) != 0 {
            libc::setsockopt(fd, libc::SOL_SOCKET, libc::SO_RCVBUF, p, len);
        }
        libc::bind(fd, (&addr as *const libc::sockaddr_nl).cast(), std::mem::size_of::<libc::sockaddr_nl>() as libc::socklen_t)
    };
    if r != 0 {
        let e = std::io::Error::last_os_error();
        // SAFETY: fd is ours and not used afterwards.
        unsafe { libc::close(fd) };
        bail!("binding the uevent socket: {e}");
    }
    let watch = w.clone();
    std::thread::Builder::new().name("uevent".into()).spawn(move || {
        let mut buf = vec![0u8; 16384];
        loop {
            // SAFETY: buf is valid for its length.
            let n = unsafe { libc::recv(fd, buf.as_mut_ptr().cast(), buf.len(), 0) };
            if n < 0 {
                let e = std::io::Error::last_os_error();
                match e.raw_os_error() {
                    Some(libc::EINTR) => {}
                    // Events were dropped; rescan.
                    Some(libc::ENOBUFS) => watch.changed.store(true, Ordering::Relaxed),
                    _ => {
                        warn!("uevent socket: {e}; falling back to scanning every second");
                        watch.alive.store(false, Ordering::Relaxed);
                        return;
                    }
                }
                continue;
            }
            if relevant(&buf[..n as usize]) {
                debug!("uevent: {}", String::from_utf8_lossy(buf[..n as usize].split(|&b| b == 0).next().unwrap_or_default()));
                watch.changed.store(true, Ordering::Relaxed);
            }
        }
    })?;
    Ok(w)
}

/// Message format: `ACTION@DEVPATH\0KEY=VALUE\0...`
fn relevant(msg: &[u8]) -> bool {
    msg.split(|&b| b == 0).any(|f| f == b"SUBSYSTEM=usb" || f == b"SUBSYSTEM=tty")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn filters_subsystems() {
        assert!(relevant(b"add@/devices/platform/soc/3f980000.usb/usb1/1-1\0ACTION=add\0SUBSYSTEM=usb\0DEVTYPE=usb_device\0"));
        assert!(relevant(b"add@/devices/.../tty/ttyACM1\0ACTION=add\0SUBSYSTEM=tty\0"));
        assert!(!relevant(b"change@/devices/virtual/net/wlan0\0ACTION=change\0SUBSYSTEM=net\0"));
        assert!(!relevant(b"add@/x\0SUBSYSTEM=usbmisc\0"));
    }
}
