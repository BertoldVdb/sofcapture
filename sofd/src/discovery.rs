//! Finds USB host controllers, their devices and their register addresses in sysfs.

use anyhow::{anyhow, bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::str::FromStr;

const USB_DEVICES: &str = "/sys/bus/usb/devices";
const DT_BASE: &str = "/sys/firmware/devicetree/base";
const IOMEM: &str = "/proc/iomem";
const MAX_MAP: u64 = 1 << 20;

pub const SIM_ID: &str = "sim0";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BackendKind {
    XhciMfindex,
    Dwc2Hfnum,
    /// No register access: usb_get_current_frame_number() via the sofclock module.
    KernelFrameNumber,
    Sim,
}

#[derive(Debug, Clone, Serialize)]
pub struct Controller {
    pub id: String,
    pub driver: String,
    pub bus_type: String,
    pub backend: Option<BackendKind>,
    pub buses: Vec<u32>,
    #[serde(skip)]
    pub path: PathBuf,
}

#[derive(Debug, Clone, Serialize)]
pub struct UsbDevice {
    pub port: String,
    pub busnum: u32,
    pub devnum: u32,
    pub vid: String,
    pub pid: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub serial: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub product: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub manufacturer: Option<String>,
    /// Serial port, e.g. /dev/ttyACM0.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tty: Option<String>,
    pub controller: String,
}

/// All given fields must match.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct DeviceQuery {
    /// "vvvv:pppp" in hex.
    pub device: Option<String>,
    pub serial: Option<String>,
    /// sysfs port path, e.g. "1-1.2".
    pub port: Option<String>,
    pub busnum: Option<u32>,
    pub devnum: Option<u32>,
}

/// `ID=ADDR[:LEN]` from the command line, mapped via /dev/mem.
#[derive(Debug, Clone)]
pub struct MmioOverride {
    pub id: String,
    pub addr: u64,
    pub len: u64,
}

impl FromStr for MmioOverride {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let (id, rest) = s.split_once('=').ok_or_else(|| anyhow!("expected ID=ADDR[:LEN]"))?;
        let (addr, len) = match rest.split_once(':') {
            Some((a, l)) => (parse_hex(a)?, parse_hex(l)?),
            None => (parse_hex(rest)?, 0x10000),
        };
        Ok(Self { id: id.to_string(), addr, len })
    }
}

fn parse_hex(s: &str) -> Result<u64> {
    let s = s.trim();
    let s = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")).unwrap_or(s);
    u64::from_str_radix(s, 16).with_context(|| format!("bad hex number '{s}'"))
}

#[derive(Debug, Clone)]
pub struct MmioRegion {
    pub path: PathBuf,
    pub offset: u64,
    pub len: usize,
}

fn read_trim(p: &Path) -> Option<String> {
    fs::read_to_string(p).ok().map(|s| s.trim().to_string())
}

fn link_name(p: &Path) -> Option<String> {
    fs::read_link(p).ok()?.file_name()?.to_str().map(String::from)
}

fn file_name(p: &Path) -> String {
    p.file_name().and_then(|n| n.to_str()).unwrap_or_default().to_string()
}

fn backend_for_driver(driver: &str) -> BackendKind {
    match driver {
        "xhci_hcd" | "xhci-hcd" | "xhci-pci" | "xhci-plat-hcd" | "tegra-xusb" | "xhci-tegra" => {
            BackendKind::XhciMfindex
        }
        "dwc_otg" | "dwc2" => BackendKind::Dwc2Hfnum,
        _ => BackendKind::KernelFrameNumber,
    }
}

fn is_root_hub(p: &Path) -> bool {
    let name = file_name(p);
    let numbered = name
        .strip_prefix("usb")
        .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()));
    numbered && p.join("busnum").exists()
}

fn controller_path_of(dev: &Path) -> Option<PathBuf> {
    dev.ancestors().find(|p| is_root_hub(p))?.parent().map(Path::to_path_buf)
}

fn sim_controller() -> Controller {
    Controller {
        id: SIM_ID.into(),
        driver: "sim".into(),
        bus_type: "virtual".into(),
        backend: Some(BackendKind::Sim),
        buses: vec![],
        path: PathBuf::new(),
    }
}

pub fn controllers(simulate: bool) -> Result<Vec<Controller>> {
    let mut map: BTreeMap<String, Controller> = BTreeMap::new();
    for entry in fs::read_dir(USB_DEVICES).with_context(|| format!("reading {USB_DEVICES}"))? {
        let Ok(real) = fs::canonicalize(entry?.path()) else { continue };
        if !is_root_hub(&real) {
            continue;
        }
        let Some(ctrl) = real.parent() else { continue };
        let busnum = read_trim(&real.join("busnum")).and_then(|s| s.parse().ok()).unwrap_or(0);
        let id = file_name(ctrl);
        let c = map.entry(id.clone()).or_insert_with(|| {
            let driver = link_name(&ctrl.join("driver")).unwrap_or_default();
            Controller {
                id,
                backend: Some(backend_for_driver(&driver)),
                driver,
                bus_type: link_name(&ctrl.join("subsystem")).unwrap_or_default(),
                buses: vec![],
                path: ctrl.to_path_buf(),
            }
        });
        c.buses.push(busnum);
    }
    let mut out: Vec<Controller> = map.into_values().collect();
    for c in &mut out {
        c.buses.sort_unstable();
    }
    if simulate {
        out.push(sim_controller());
    }
    Ok(out)
}

pub fn find_controller(id: &str, simulate: bool) -> Result<Controller> {
    controllers(simulate)?
        .into_iter()
        .find(|c| c.id == id)
        .ok_or_else(|| anyhow!("no USB host controller '{id}'"))
}

pub fn devices(simulate: bool) -> Result<Vec<UsbDevice>> {
    let mut out = vec![];
    for entry in fs::read_dir(USB_DEVICES).with_context(|| format!("reading {USB_DEVICES}"))? {
        let entry = entry?;
        let port = entry.file_name().to_string_lossy().into_owned();
        // Skip interfaces ("1-1:1.0") and root hubs.
        if port.contains(':') || port.starts_with("usb") {
            continue;
        }
        let Ok(real) = fs::canonicalize(entry.path()) else { continue };
        let Some(ctrl) = controller_path_of(&real) else { continue };
        let num = |f: &str| read_trim(&real.join(f)).and_then(|s| s.parse().ok()).unwrap_or(0);
        out.push(UsbDevice {
            busnum: num("busnum"),
            devnum: num("devnum"),
            vid: read_trim(&real.join("idVendor")).unwrap_or_default(),
            pid: read_trim(&real.join("idProduct")).unwrap_or_default(),
            serial: read_trim(&real.join("serial")),
            product: read_trim(&real.join("product")),
            manufacturer: read_trim(&real.join("manufacturer")),
            tty: tty_of(&real, &port),
            controller: file_name(&ctrl),
            port,
        });
    }
    if simulate {
        out.push(UsbDevice {
            port: format!("{SIM_ID}-1"),
            busnum: 0,
            devnum: 1,
            vid: "0000".into(),
            pid: "0000".into(),
            serial: Some("SIM".into()),
            product: Some("Simulated device".into()),
            manufacturer: None,
            tty: None,
            controller: SIM_ID.into(),
        });
    }
    out.sort_by(|a, b| a.port.cmp(&b.port));
    Ok(out)
}

/// cdc-acm puts the tty at `<intf>/tty/ttyACM0`, usb-serial at `<intf>/ttyUSB0`.
fn tty_of(dev: &Path, port: &str) -> Option<String> {
    let mut intfs: Vec<PathBuf> = fs::read_dir(dev)
        .ok()?
        .flatten()
        .map(|e| e.path())
        .filter(|p| file_name(p).starts_with(&format!("{port}:")))
        .collect();
    intfs.sort();
    intfs.iter().find_map(|i| {
        let mut names: Vec<String> = fs::read_dir(i.join("tty"))
            .into_iter()
            .chain(fs::read_dir(i))
            .flatten()
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .filter(|n| n.starts_with("tty") && n != "tty")
            .collect();
        names.sort();
        names.into_iter().next().map(|n| format!("/dev/{n}"))
    })
}

pub fn device_of_tty(tty: &str, simulate: bool) -> Result<UsbDevice> {
    let real = fs::canonicalize(tty).with_context(|| format!("resolving {tty}"))?;
    let name = file_name(&real);
    let sys = fs::canonicalize(Path::new("/sys/class/tty").join(&name).join("device"))
        .with_context(|| format!("{name} is not a device tty"))?;
    let usb = sys
        .ancestors()
        .find(|p| !file_name(p).contains(':') && p.join("idVendor").exists())
        .ok_or_else(|| anyhow!("{name} is not a USB serial port"))?;
    let port = file_name(usb);
    let mut dev = devices(simulate)?
        .into_iter()
        .find(|d| d.port == port)
        .ok_or_else(|| anyhow!("USB device {port} of {name} not found"))?;
    dev.tty = Some(format!("/dev/{name}"));
    Ok(dev)
}

pub fn lookup(q: &DeviceQuery, simulate: bool) -> Result<UsbDevice> {
    let vidpid = q
        .device
        .as_deref()
        .map(|d| {
            let (v, p) = d.split_once(':').ok_or_else(|| anyhow!("device must be 'vvvv:pppp'"))?;
            Ok::<_, anyhow::Error>((v.to_ascii_lowercase(), p.to_ascii_lowercase()))
        })
        .transpose()?;
    if vidpid.is_none() && q.serial.is_none() && q.port.is_none() && q.busnum.is_none() {
        bail!("lookup needs at least one of device, serial, port, busnum");
    }
    let hits: Vec<UsbDevice> = devices(simulate)?
        .into_iter()
        .filter(|d| {
            vidpid.as_ref().is_none_or(|(v, p)| &d.vid == v && &d.pid == p)
                && q.serial.as_ref().is_none_or(|s| d.serial.as_ref() == Some(s))
                && q.port.as_ref().is_none_or(|p| &d.port == p)
                && q.busnum.is_none_or(|b| d.busnum == b)
                && q.devnum.is_none_or(|n| d.devnum == n)
        })
        .collect();
    match hits.len() {
        0 => bail!("no matching USB device"),
        1 => Ok(hits.into_iter().next().unwrap()),
        _ => {
            let ports: Vec<&str> = hits.iter().map(|d| d.port.as_str()).collect();
            bail!("ambiguous: matches ports {}; add serial or port", ports.join(", "))
        }
    }
}

/// PCI: BAR0 via sysfs. Platform devices: device tree, then /proc/iomem.
pub fn mmio_region(c: &Controller, overrides: &[MmioOverride]) -> Result<MmioRegion> {
    let dev_mem = |addr: u64, len: u64| MmioRegion {
        path: "/dev/mem".into(),
        offset: addr,
        len: len.min(MAX_MAP) as usize,
    };
    if let Some(o) = overrides.iter().find(|o| o.id == c.id) {
        return Ok(dev_mem(o.addr, o.len));
    }
    if c.bus_type == "pci" {
        let path = c.path.join("resource0");
        let len = fs::metadata(&path).with_context(|| format!("stat {}", path.display()))?.len();
        return Ok(MmioRegion { path, offset: 0, len: len.min(MAX_MAP) as usize });
    }
    if let Some((addr, len)) = from_devicetree(&c.path).context("device tree lookup")? {
        return Ok(dev_mem(addr, len));
    }
    if let Some((addr, len)) = from_iomem(&c.id)? {
        return Ok(dev_mem(addr, len));
    }
    bail!("cannot locate registers of {}; pass --mmio {}=ADDR[:LEN]", c.id, c.id)
}

fn be_cells(bytes: &[u8]) -> Vec<u32> {
    bytes.chunks_exact(4).map(|c| u32::from_be_bytes(c.try_into().unwrap())).collect()
}

/// Keeps the low 64 bits.
fn cells_to_u64(c: &[u32]) -> u64 {
    c.iter().fold(0u64, |a, &x| a.checked_shl(32).unwrap_or(0) | x as u64)
}

fn read_cells(node: &Path, name: &str, default: usize) -> usize {
    fs::read(node.join(name))
        .ok()
        .and_then(|b| be_cells(&b).first().copied())
        .map_or(default, |v| v as usize)
}

fn from_devicetree(ctrl: &Path) -> Result<Option<(u64, u64)>> {
    let Ok(node) = fs::canonicalize(ctrl.join("of_node")) else { return Ok(None) };
    let Ok(reg) = fs::read(node.join("reg")) else { return Ok(None) };
    let reg = be_cells(&reg);
    let parent = node.parent().context("of_node has no parent")?;
    let ac = read_cells(parent, "#address-cells", 2);
    let sc = read_cells(parent, "#size-cells", 1);
    let names: Vec<String> = fs::read(node.join("reg-names"))
        .map(|b| {
            b.split(|&c| c == 0)
                .filter(|s| !s.is_empty())
                .map(|s| String::from_utf8_lossy(s).into_owned())
                .collect()
        })
        .unwrap_or_default();
    // Tegra xusb has several windows; the xHCI one is named "hcd".
    let idx = names.iter().position(|n| n == "hcd").unwrap_or(0);
    let entry = reg
        .chunks_exact(ac + sc)
        .nth(idx)
        .with_context(|| format!("{}: no reg entry {idx}", node.display()))?;
    let mut addr = cells_to_u64(&entry[..ac]);
    let size = cells_to_u64(&entry[ac..]);

    // Bus to CPU address via the parents' `ranges`, e.g. 0x7e980000 -> 0x3f980000 on BCM283x.
    let root = fs::canonicalize(DT_BASE)?;
    let mut bus = parent.to_path_buf();
    while bus != root {
        let up = bus.parent().context("device tree walk escaped root")?.to_path_buf();
        match fs::read(bus.join("ranges")) {
            Err(_) => break,
            Ok(r) if r.is_empty() => {}
            Ok(r) => {
                let cac = read_cells(&bus, "#address-cells", 2);
                let csc = read_cells(&bus, "#size-cells", 1);
                let pac = read_cells(&up, "#address-cells", 2);
                let hit = be_cells(&r).chunks_exact(cac + pac + csc).find_map(|e| {
                    let child = cells_to_u64(&e[..cac]);
                    let par = cells_to_u64(&e[cac..cac + pac]);
                    let len = cells_to_u64(&e[cac + pac..]);
                    (addr >= child && addr - child < len).then(|| par + (addr - child))
                });
                addr = hit.with_context(|| format!("{addr:#x} not covered by ranges of {}", bus.display()))?;
            }
        }
        bus = up;
    }
    Ok(Some((addr, size)))
}

fn from_iomem(id: &str) -> Result<Option<(u64, u64)>> {
    let text = fs::read_to_string(IOMEM).with_context(|| format!("reading {IOMEM}"))?;
    let prefix = format!("{id} ");
    let mut hits = vec![];
    for line in text.lines() {
        let Some((range, name)) = line.trim().split_once(" : ") else { continue };
        if name != id && !name.starts_with(&prefix) {
            continue;
        }
        let Some((s, e)) = range.split_once('-') else { continue };
        hits.push((name.to_string(), parse_hex(s)?, parse_hex(e)?));
    }
    if !hits.is_empty() && hits.iter().all(|h| h.1 == 0 && h.2 == 0) {
        bail!("{IOMEM} hides addresses from non-root users");
    }
    let pick = hits.iter().find(|h| h.0.contains("hcd")).or(hits.first());
    Ok(pick.map(|h| (h.1, h.2 - h.1 + 1)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn override_parsing() {
        let o: MmioOverride = "3610000.usb=0x3610000:0x40000".parse().unwrap();
        assert_eq!((o.id.as_str(), o.addr, o.len), ("3610000.usb", 0x3610000, 0x40000));
        let o: MmioOverride = "x=3f980000".parse().unwrap();
        assert_eq!((o.addr, o.len), (0x3f980000, 0x10000));
    }

    #[test]
    fn multi_cell_numbers() {
        assert_eq!(cells_to_u64(&[0x7e980000]), 0x7e980000);
        assert_eq!(cells_to_u64(&[0x1, 0x2]), 0x1_0000_0002);
        assert_eq!(cells_to_u64(&[0x9, 0x1, 0x2]), 0x1_0000_0002);
    }
}
