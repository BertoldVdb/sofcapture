//! Read-only mapping of a controller's register window.

use anyhow::{bail, Context, Result};
use std::fs::OpenOptions;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::path::Path;

pub struct Mmio {
    map: *mut libc::c_void,
    map_len: usize,
    base: *const u8,
    len: usize,
}

// Read-only mapping, owned by one sampler thread.
unsafe impl Send for Mmio {}

impl Mmio {
    /// `path` is `/dev/mem` or a sysfs PCI `resourceN`.
    pub fn map(path: &Path, offset: u64, len: usize) -> Result<Self> {
        if !offset.is_multiple_of(4) {
            bail!("register address {offset:#x} is not 4-byte aligned");
        }
        let file = OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_SYNC)
            .open(path)
            .with_context(|| format!("opening {} (needs root)", path.display()))?;
        // SAFETY: sysconf has no preconditions.
        let page = unsafe { libc::sysconf(libc::_SC_PAGESIZE) } as u64;
        let aligned = offset & !(page - 1);
        let delta = (offset - aligned) as usize;
        let map_len = (delta + len + page as usize - 1) & !(page as usize - 1);
        let off: libc::off_t = aligned
            .try_into()
            .context("register address does not fit off_t on this platform")?;
        // SAFETY: fresh read-only shared mapping of a file we opened; checked below.
        let map = unsafe {
            libc::mmap(std::ptr::null_mut(), map_len, libc::PROT_READ, libc::MAP_SHARED, file.as_raw_fd(), off)
        };
        if map == libc::MAP_FAILED {
            let err = std::io::Error::last_os_error();
            let hint = match err.raw_os_error() {
                Some(libc::EINVAL | libc::EPERM) => {
                    " (the kernel refuses to map registers owned by a driver when built with \
                     CONFIG_IO_STRICT_DEVMEM; boot with iomem=relaxed)"
                }
                _ => "",
            };
            bail!("mmap {} @ {:#x}+{:#x}: {err}{hint}", path.display(), offset, len);
        }
        // SAFETY: delta < page <= map_len.
        let base = unsafe { (map as *const u8).add(delta) };
        Ok(Self { map, map_len, base, len })
    }

    /// Fenced so the timestamps around it really bracket the access.
    #[inline]
    pub fn read32(&self, off: usize) -> u32 {
        assert!(off.is_multiple_of(4) && off + 4 <= self.len, "register offset {off:#x} outside mapping");
        // arm64 doesn't order the device load against clock_gettime, so a stalled
        // read could finish after the next timestamp. x86's vDSO already orders it.
        #[cfg(target_arch = "aarch64")]
        // SAFETY: barriers only.
        unsafe {
            std::arch::asm!("isb", options(nostack, preserves_flags))
        };
        // SAFETY: bounds and alignment checked above; the mapping lives as long as self.
        let v = unsafe { std::ptr::read_volatile(self.base.add(off) as *const u32) };
        #[cfg(target_arch = "aarch64")]
        // SAFETY: barriers only.
        unsafe {
            std::arch::asm!("dsb ld", options(nostack, preserves_flags))
        };
        u32::from_le(v)
    }

    pub fn len(&self) -> usize {
        self.len
    }
}

impl Drop for Mmio {
    fn drop(&mut self) {
        // SAFETY: unmapping exactly what mmap returned.
        unsafe { libc::munmap(self.map, self.map_len) };
    }
}
