#!/usr/bin/env python3
"""Writes the ELF's SHA-256 into the app descriptor of the .bin, as esptool does."""
import hashlib
import struct
import sys

MAGIC = struct.pack("<I", 0xABCD5432)
PROJECT = b"sofgnss\0"
SHA_OFFSET = 144  # app_elf_sha256 in esp_app_desc_t


def main(elf_path, bin_path):
    with open(elf_path, "rb") as f:
        digest = hashlib.sha256(f.read()).digest()
    with open(bin_path, "rb") as f:
        image = bytearray(f.read())
    found = [i for i in range(0, len(image) - 256, 4)
             if image[i:i + 4] == MAGIC and image[i + 48:i + 56] == PROJECT]
    if len(found) != 1:
        sys.exit(f"{bin_path}: expected one app descriptor, found {len(found)}")
    off = found[0] + SHA_OFFSET
    image[off:off + 32] = digest
    with open(bin_path, "wb") as f:
        f.write(image)
    print(f"app descriptor at 0x{found[0]:x}, elf sha256 {digest.hex()}")


if __name__ == "__main__":
    main(*sys.argv[1:])
