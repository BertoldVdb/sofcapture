//! sofgnss firmware image for OTA updates: an ESP-IDF app `.bin`, or a signed STM32
//! image (sofgnss/stm32g4/sign.py: a 128-byte header, then the app).

use anyhow::{bail, Context, Result};
use sha2::{Digest, Sha256};
use std::path::Path;
use std::time::SystemTime;

/// After esp_image_header_t and the first segment header.
const APP_DESC_OFFSET: usize = 32;
const APP_DESC_MAGIC: u32 = 0xABCD_5432;
const STM32_MAGIC: &[u8] = b"SGFW";
const STM32_HEADER: usize = 128;
const PROJECT: &str = "sofgnss";

#[derive(Debug)]
pub struct Image {
    pub data: Vec<u8>,
    /// Of the whole file; the device checks it after the transfer.
    pub sha256: String,
    /// From the app descriptor; the device reports it in VER.
    pub elf_sha256: String,
    pub version: String,
    /// For the STM32 bootloader, not an ESP32 app.
    pub stm32: bool,
    pub mtime: Option<SystemTime>,
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn cstr(b: &[u8]) -> String {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

impl Image {
    pub fn load(path: &Path) -> Result<Self> {
        let data = std::fs::read(path).with_context(|| format!("reading firmware {}", path.display()))?;
        let mtime = std::fs::metadata(path).and_then(|m| m.modified()).ok();
        Self::parse(data, mtime).with_context(|| format!("firmware {}", path.display()))
    }

    pub fn parse(data: Vec<u8>, mtime: Option<SystemTime>) -> Result<Self> {
        let stm32 = data.starts_with(STM32_MAGIC);
        let off = if stm32 {
            let Some(h) = data.get(..STM32_HEADER) else { bail!("too short for a signed STM32 image") };
            STM32_HEADER + u32::from_le_bytes(h[12..16].try_into().unwrap()) as usize
        } else {
            APP_DESC_OFFSET
        };
        let Some(d) = data.get(off..off + 256) else { bail!("too short for a sofgnss image") };
        if u32::from_le_bytes(d[..4].try_into().unwrap()) != APP_DESC_MAGIC {
            bail!("not a sofgnss image (no app descriptor)");
        }
        let project = cstr(&d[48..80]);
        if project != PROJECT {
            bail!("image is for project \"{project}\", not \"{PROJECT}\"");
        }
        Ok(Self {
            sha256: hex(&Sha256::digest(&data)),
            elf_sha256: hex(&d[144..176]),
            version: cstr(&d[16..48]),
            stm32,
            data,
            mtime,
        })
    }
}

#[cfg(test)]
pub fn test_image(version: &str, elf: u8, len: usize) -> Image {
    let mut d = vec![0u8; len.max(APP_DESC_OFFSET + 256)];
    let a = APP_DESC_OFFSET;
    d[a..a + 4].copy_from_slice(&APP_DESC_MAGIC.to_le_bytes());
    d[a + 16..a + 16 + version.len()].copy_from_slice(version.as_bytes());
    d[a + 48..a + 48 + PROJECT.len()].copy_from_slice(PROJECT.as_bytes());
    d[a + 144..a + 176].fill(elf);
    Image::parse(d, None).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor() {
        let i = test_image("v1.2", 0xab, 1000);
        assert_eq!((i.version.as_str(), i.elf_sha256.len()), ("v1.2", 64));
        assert!(i.elf_sha256.starts_with("abab"));
        let mut bad = i.data.clone();
        bad[APP_DESC_OFFSET + 48] = b'x';
        assert!(Image::parse(bad, None).is_err());
        assert!(Image::parse(vec![0; 10], None).is_err());
    }

    #[test]
    fn stm32_image() {
        let mut d = vec![0u8; STM32_HEADER];
        d[..4].copy_from_slice(STM32_MAGIC);
        d[12..16].copy_from_slice(&0x1d8u32.to_le_bytes());
        let mut app = test_image("v3", 0xef, 1000).data;
        app.splice(..0, vec![0u8; 0x1d8 - APP_DESC_OFFSET]);
        d.extend(app);
        let i = Image::parse(d, None).unwrap();
        assert!(i.stm32);
        assert_eq!(i.version, "v3");
        assert!(i.elf_sha256.starts_with("efef"));
        assert!(!test_image("v3", 0xef, 1000).stm32);
    }

    #[test]
    fn real_build_if_present() {
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../sofgnss/esp32s3/build/sofgnss.bin");
        if p.exists() {
            let i = Image::load(&p).unwrap();
            assert_eq!(i.elf_sha256.len(), 64);
        }
        let p = Path::new(env!("CARGO_MANIFEST_DIR")).join("../sofgnss/stm32g4/build/sofgnss-signed.bin");
        if p.exists() {
            assert!(Image::load(&p).unwrap().stm32);
        }
    }
}
