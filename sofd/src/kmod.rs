//! Backends using the sofclock kernel module (kmod/): xHCI where the kernel won't
//! let us map registers, and a generic fallback for any controller.

use crate::backend::{Backend, RawSample, FS_UNIT_NS, HS_UNIT_NS};
use crate::clock::Ns;
use anyhow::{anyhow, bail, Context, Result};
use log::info;
use std::fs::{File, OpenOptions};
use std::os::unix::io::AsRawFd;
use std::path::Path;

pub const DEVICE: &str = "/dev/sofclock";

const MODE_XHCI_PCI: u32 = 0;
const MODE_GENERIC: u32 = 1;

/// Mirrors `struct sofclock_req` in kmod/sofclock.h.
#[repr(C)]
#[derive(Default, Clone)]
struct Req {
    mode: u32,
    domain: u32,
    bus: u8,
    devfn: u8,
    edges: u16,
    busnum: u32,
    irq_off_max_ns: u32,
    counter: u32,
    unit_ns: u32,
    reserved: u32,
    t_ns: i64,
    unc_ns: i64,
}

/// `_IOWR('S', 0x03, struct sofclock_req)`
const IOC_SAMPLE: u64 = (3 << 30) | ((std::mem::size_of::<Req>() as u64) << 16) | ((b'S' as u64) << 8) | 0x03;

pub struct Kmod {
    dev: File,
    req: Req,
    /// Tick length; measured at open in generic mode.
    unit_ns: u32,
}

pub fn available() -> bool {
    Path::new(DEVICE).exists()
}

/// "dddd:bb:dd.f"
fn parse_bdf(id: &str) -> Result<(u32, u8, u8)> {
    let bad = || anyhow!("'{id}' is not a PCI address");
    let (domain, rest) = id.split_once(':').ok_or_else(bad)?;
    let (bus, rest) = rest.split_once(':').ok_or_else(bad)?;
    let (dev, func) = rest.split_once('.').ok_or_else(bad)?;
    let dev = u8::from_str_radix(dev, 16).map_err(|_| bad())?;
    let func = u8::from_str_radix(func, 16).map_err(|_| bad())?;
    if dev > 31 || func > 7 {
        return Err(bad());
    }
    Ok((
        u32::from_str_radix(domain, 16).map_err(|_| bad())?,
        u8::from_str_radix(bus, 16).map_err(|_| bad())?,
        dev << 3 | func,
    ))
}

fn classify_unit(measured_ns: f64) -> Option<u32> {
    [HS_UNIT_NS, FS_UNIT_NS]
        .into_iter()
        .find(|&u| (measured_ns / u as f64 - 1.0).abs() < 0.1)
}

fn open_device() -> Result<File> {
    OpenOptions::new()
        .read(true)
        .write(true)
        .open(DEVICE)
        .with_context(|| format!("opening {DEVICE} (is the sofclock module loaded?)"))
}

impl Kmod {
    pub fn open_xhci_pci(pci_id: &str, edges: u32, irq_off_max_ns: u32) -> Result<Self> {
        let (domain, bus, devfn) = parse_bdf(pci_id)?;
        let req = Req {
            mode: MODE_XHCI_PCI,
            domain,
            bus,
            devfn,
            edges: edges.clamp(1, 4) as u16,
            irq_off_max_ns,
            ..Default::default()
        };
        let mut k = Self { dev: open_device()?, req, unit_ns: HS_UNIT_NS };
        // Report "not xHCI" / "not bound" now rather than on every sample.
        k.sample()?;
        Ok(k)
    }

    /// Any controller, via usb_get_current_frame_number().
    pub fn open_generic(busnum: u32, edges: u32, irq_off_max_ns: u32) -> Result<Self> {
        let req = Req { mode: MODE_GENERIC, busnum, edges: edges.clamp(1, 4) as u16, irq_off_max_ns, ..Default::default() };
        let mut k = Self { dev: open_device()?, req, unit_ns: 0 };
        k.unit_ns = k.calibrate()?;
        info!("bus {busnum}: generic frame counter ticks every {} µs", k.unit_ns / 1000);
        Ok(k)
    }

    /// Learns the tick length from two nearby steps.
    fn calibrate(&mut self) -> Result<u32> {
        let mut last = String::from("no clean steps");
        for _ in 0..10 {
            let (Some(a), Some(b)) = (self.ioctl()?, self.ioctl()?) else { continue };
            let d = b.counter.wrapping_sub(a.counter);
            if !(1..=16).contains(&d) {
                last = format!("counter went {} -> {}", a.counter, b.counter);
                continue;
            }
            let per_tick = (b.t_ns - a.t_ns) as f64 / d as f64;
            if let Some(u) = classify_unit(per_tick) {
                return Ok(u);
            }
            last = format!("{per_tick:.0} ns per tick");
        }
        bail!("cannot determine the frame counter's tick length ({last})")
    }

    /// Ok(None) on a transient miss.
    fn ioctl(&mut self) -> Result<Option<Req>> {
        let mut r = self.req.clone();
        // SAFETY: r matches the kernel's struct layout and outlives the call.
        let rc = unsafe { libc::ioctl(self.dev.as_raw_fd(), IOC_SAMPLE as _, &mut r) };
        if rc == 0 {
            return Ok(Some(r));
        }
        let err = std::io::Error::last_os_error();
        match err.raw_os_error() {
            Some(libc::EAGAIN) => Ok(None),
            Some(libc::EIO) => bail!("controller halted/not running or frame counter not advancing (sofclock)"),
            Some(libc::ENODEV) => bail!("controller not found or its driver not bound (sofclock)"),
            Some(libc::EOPNOTSUPP) => bail!("not an xHCI controller (sofclock)"),
            _ => Err(err).context("sofclock ioctl"),
        }
    }
}

impl Backend for Kmod {
    fn sample(&mut self) -> Result<Option<RawSample>> {
        Ok(self.ioctl()?.map(|r| RawSample {
            counter: r.counter,
            t_ns: r.t_ns as Ns,
            unc_ns: r.unc_ns as Ns,
            unit_ns: self.unit_ns,
        }))
    }

    fn counter_bits(&self) -> Option<u32> {
        // Generic drivers wrap wherever they like.
        (self.req.mode == MODE_XHCI_PCI).then_some(crate::backend::COUNTER_BITS)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abi() {
        assert_eq!(std::mem::size_of::<Req>(), 48);
        assert_eq!(std::mem::offset_of!(Req, t_ns), 32);
        assert_eq!(IOC_SAMPLE, 0xc030_5303);
        assert_eq!(parse_bdf("0000:00:14.0").unwrap(), (0, 0, 0xa0));
        assert_eq!(parse_bdf("0001:3a:00.3").unwrap(), (1, 0x3a, 3));
        assert!(parse_bdf("3610000.usb").is_err());
    }

    #[test]
    fn unit_classification() {
        assert_eq!(classify_unit(1_000_400.0), Some(FS_UNIT_NS));
        assert_eq!(classify_unit(124_100.0), Some(HS_UNIT_NS));
        assert_eq!(classify_unit(500_000.0), None);
    }
}
