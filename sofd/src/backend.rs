//! Frame counter register access per controller family. Each sample is a counter value
//! and the CLOCK_MONOTONIC_RAW time its (micro)frame started.

use crate::clock::{self, Ns};
use crate::discovery::{self, BackendKind, Controller, MmioOverride};
use crate::kmod;
use crate::mmio::Mmio;
use anyhow::{anyhow, bail, Context, Result};
use log::info;

/// Both xHCI MFINDEX and DWC2 HFNUM.FrNum are 14-bit counters.
pub const COUNTER_BITS: u32 = 14;
pub const COUNTER_MASK: u32 = (1 << COUNTER_BITS) - 1;

pub const HS_UNIT_NS: u32 = 125_000;
pub const FS_UNIT_NS: u32 = 1_000_000;

#[derive(Debug, Clone)]
pub struct RawSample {
    /// Masked to COUNTER_BITS.
    pub counter: u32,
    /// Start of that (micro)frame, CLOCK_MONOTONIC_RAW.
    pub t_ns: Ns,
    /// Half-width of the uncertainty around `t_ns`.
    pub unc_ns: Ns,
    /// HS_UNIT_NS (microframes) or FS_UNIT_NS (frames).
    pub unit_ns: u32,
}

pub trait Backend: Send {
    /// `Ok(None)` is a transient miss (stall); `Err` means the controller is unusable.
    fn sample(&mut self) -> Result<Option<RawSample>>;

    /// None if the wrap point is unknown.
    fn counter_bits(&self) -> Option<u32> {
        Some(COUNTER_BITS)
    }
}

/// `irq_off_max_ns`: longest IRQ-off stretch for the sofclock module (0 = never).
pub fn open(c: &Controller, overrides: &[MmioOverride], edges: u32, irq_off_max_ns: u32) -> Result<Box<dyn Backend>> {
    let kind = c.backend.ok_or_else(|| anyhow!("{} has no root hub", c.id))?;
    match kind {
        BackendKind::Sim => return Ok(Box::new(Sim::new())),
        BackendKind::KernelFrameNumber => {
            if !kmod::available() {
                bail!(
                    "no register backend for driver '{}' of {}; load the sofclock module for the generic fallback",
                    c.driver,
                    c.id
                );
            }
            let busnum = *c.buses.first().ok_or_else(|| anyhow!("{} has no root hub", c.id))?;
            let k = kmod::Kmod::open_generic(busnum, 1, irq_off_max_ns)
                .with_context(|| format!("generic frame counter of {} (driver '{}')", c.id, c.driver))?;
            info!("{}: using the generic fallback, usb_get_current_frame_number() (driver '{}')", c.id, c.driver);
            return Ok(Box::new(k));
        }
        BackendKind::XhciMfindex | BackendKind::Dwc2Hfnum => {}
    }
    let direct = || -> Result<Box<dyn Backend>> {
        let r = discovery::mmio_region(c, overrides)?;
        let regs = Mmio::map(&r.path, r.offset, r.len).with_context(|| format!("mapping registers of {}", c.id))?;
        Ok(match kind {
            BackendKind::XhciMfindex => Box::new(Xhci::new(regs, edges.max(1))?),
            BackendKind::Dwc2Hfnum => Box::new(Dwc2::new(regs)?),
            BackendKind::Sim | BackendKind::KernelFrameNumber => unreachable!(),
        })
    };
    match direct() {
        Ok(b) => Ok(b),
        // Kernel won't let us map the registers: try the sofclock module.
        Err(e) if kind == BackendKind::XhciMfindex && c.bus_type == "pci" && kmod::available() => {
            let k = kmod::Kmod::open_xhci_pci(&c.id, edges, irq_off_max_ns)
                .with_context(|| format!("direct access failed ({e:#}); sofclock module failed too"))?;
            info!("{}: using the sofclock kernel module ({e:#})", c.id);
            Ok(Box::new(k))
        }
        Err(e) => Err(e),
    }
}

/// xHCI: MFINDEX has no sub-frame phase, so spin reading it until it steps.
pub struct Xhci {
    regs: Mmio,
    usbsts: usize,
    mfindex: usize,
    edges: u32,
}

impl Xhci {
    pub fn new(regs: Mmio, edges: u32) -> Result<Self> {
        let cap = regs.read32(0);
        let caplen = (cap & 0xff) as usize;
        let version = cap >> 16;
        if !matches!(version, 0x0090 | 0x0095 | 0x0096 | 0x0100 | 0x0110 | 0x0120) || caplen < 0x20 {
            bail!("registers do not look like xHCI (CAPLENGTH {caplen:#x}, HCIVERSION {version:#x})");
        }
        let rtsoff = (regs.read32(0x18) & !0x1f) as usize;
        if rtsoff + 4 > regs.len() {
            bail!("xHCI runtime registers at {rtsoff:#x} lie outside the mapped {:#x} bytes", regs.len());
        }
        Ok(Self { usbsts: caplen + 4, mfindex: rtsoff, regs, edges })
    }
}

impl Backend for Xhci {
    fn sample(&mut self) -> Result<Option<RawSample>> {
        if self.regs.read32(self.usbsts) & 1 != 0 {
            bail!("controller halted (USBSTS.HCH); MFINDEX is not running");
        }
        let regs = &self.regs;
        let off = self.mfindex;
        let read = || regs.read32(off) & COUNTER_MASK;

        let deadline = clock::raw() + 2_000_000 + self.edges as Ns * HS_UNIT_NS as Ns;
        let mut best: Option<RawSample> = None;
        let mut found = 0;
        // A step seen in v happened between t_prev and t_next.
        let mut t_prev = clock::raw();
        let mut v_prev = read();
        let v_start = v_prev;
        let mut t_cur = clock::raw();
        let mut min_read = t_cur - t_prev;
        loop {
            let v = read();
            let t_next = clock::raw();
            min_read = min_read.min(t_next - t_cur);
            // Skip jumps of more than one and brackets much longer than two reads:
            // we were interrupted. Same limit as the sofclock module.
            let clean = t_next - t_prev <= 4 * min_read + 1000;
            if v != v_prev && v == (v_prev + 1) & COUNTER_MASK && clean {
                let s = RawSample {
                    counter: v,
                    t_ns: t_prev + (t_next - t_prev) / 2,
                    unc_ns: (t_next - t_prev + 1) / 2,
                    unit_ns: HS_UNIT_NS,
                };
                if best.as_ref().is_none_or(|b| s.unc_ns < b.unc_ns) {
                    best = Some(s);
                }
                found += 1;
                if found >= self.edges {
                    break;
                }
            }
            t_prev = t_cur;
            t_cur = t_next;
            v_prev = v;
            if t_next > deadline {
                break;
            }
        }
        // No step in over 2 ms: the counter is stopped.
        if best.is_none() && read() == v_start {
            bail!("MFINDEX is not advancing");
        }
        Ok(best)
    }
}

const GINTSTS: usize = 0x014;
const GSNPSID: usize = 0x040;
const HFIR: usize = 0x404;
const HFNUM: usize = 0x408;
const HPRT0: usize = 0x440;

/// Synopsys DWC2 (Raspberry Pi), host mode. HFNUM.FrRem gives the frame phase,
/// so no spinning: take the tightest read of a short burst.
pub struct Dwc2 {
    regs: Mmio,
}

impl Dwc2 {
    const BURST: usize = 8;

    pub fn new(regs: Mmio) -> Result<Self> {
        let id = regs.read32(GSNPSID);
        if !matches!(id >> 16, 0x4f54 | 0x5531 | 0x5532) {
            bail!("registers do not look like DWC2 (GSNPSID {id:#010x})");
        }
        Ok(Self { regs })
    }
}

impl Backend for Dwc2 {
    fn sample(&mut self) -> Result<Option<RawSample>> {
        if self.regs.read32(GINTSTS) & 1 == 0 {
            bail!("DWC2 core is in device mode; only host mode is supported");
        }
        let hprt = self.regs.read32(HPRT0);
        if hprt & (1 << 2) == 0 {
            bail!("root port not enabled (no device attached?); no SOFs are sent");
        }
        let unit_ns = if (hprt >> 17) & 3 == 0 { HS_UNIT_NS } else { FS_UNIT_NS };
        let frint = self.regs.read32(HFIR) & 0xffff;
        if frint == 0 {
            bail!("HFIR.FrInt is zero");
        }

        let mut best = (Ns::MAX, 0, 0);
        for _ in 0..Self::BURST {
            let t0 = clock::raw();
            let v = self.regs.read32(HFNUM);
            let t1 = clock::raw();
            if t1 - t0 < best.0 {
                best = (t1 - t0, t0 + (t1 - t0) / 2, v);
            }
        }
        let (width, mid, v) = best;
        let frnum = v & 0xffff;
        let frrem = v >> 16;
        // FrRem counts down from FrInt to 0 across one (micro)frame.
        let elapsed = frint.saturating_sub(frrem) as f64 / frint as f64 * unit_ns as f64;
        Ok(Some(RawSample {
            counter: frnum & COUNTER_MASK,
            t_ns: mid - elapsed.round() as Ns,
            unc_ns: (width + 1) / 2 + (unit_ns / frint) as Ns,
            unit_ns,
        }))
    }
}

/// Fake high-speed controller (+23.5 ppm, ±300 ns jitter) for testing without hardware.
pub struct Sim {
    t0: Ns,
    period: f64,
    rng: u64,
}

impl Sim {
    pub fn new() -> Self {
        let now = clock::raw();
        Self { t0: now - 1_234_567, period: HS_UNIT_NS as f64 * (1.0 + 23.5e-6), rng: now as u64 | 1 }
    }
}

impl Backend for Sim {
    fn sample(&mut self) -> Result<Option<RawSample>> {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let noise = (self.rng % 601) as Ns - 300;
        let n = ((clock::raw() - self.t0) as f64 / self.period).ceil() as i64;
        let edge = self.t0 + (n as f64 * self.period).round() as Ns;
        Ok(Some(RawSample { counter: n as u32 & COUNTER_MASK, t_ns: edge + noise, unc_ns: 300, unit_ns: HS_UNIT_NS }))
    }
}
