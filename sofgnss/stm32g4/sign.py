#!/usr/bin/env python3
"""Signed images for the STM32 bootloader (layout in boot/image.h).

    sign.py keygen KEY.pem                     new P-256 key
    sign.py pubkey KEY.pem OUT.h               public key for the bootloader build
    sign.py sign KEY.pem app.bin OUT.bin       header + app: what OTA sends
    sign.py full boot.bin signed.bin OUT.bin   bootloader + header + app, for DFU or SWD
"""
import hashlib
import os
import struct
import sys

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, utils

MAGIC = 0x57464753  # "SGFW"
HEADER = 128
BOOT_SIZE = 0x6000
APP_OFFSET = 0x7000  # from the flash start: bootloader, then the info page
APP_ADDR = 0x08000000 + APP_OFFSET
DESC_MAGIC = struct.pack("<I", 0xABCD5432)


def load_key(path):
    with open(path, "rb") as f:
        key = serialization.load_pem_private_key(f.read(), None, default_backend())
    if not isinstance(key, ec.EllipticCurvePrivateKey) or key.curve.name != "secp256r1":
        sys.exit(f"{path}: not a P-256 key")
    return key


def keygen(path):
    if os.path.exists(path):
        sys.exit(f"{path} exists")
    key = ec.generate_private_key(ec.SECP256R1(), default_backend())
    pem = key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                            serialization.NoEncryption())
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as f:
        f.write(pem)


def pubkey(path, out):
    n = load_key(path).public_key().public_numbers()
    xy = n.x.to_bytes(32, "big") + n.y.to_bytes(32, "big")
    rows = ",\n".join("    " + ", ".join(f"0x{b:02x}" for b in xy[i:i + 16]) for i in range(0, 64, 16))
    text = f"// From {os.path.basename(path)} by sign.py.\n#include <stdint.h>\nstatic const uint8_t kPublicKey[64] = {{\n{rows}}};\n"
    old = open(out).read() if os.path.exists(out) else None
    if text != old:
        with open(out, "w") as f:
            f.write(text)


def sign(path, app_path, out):
    key = load_key(path)
    app = open(app_path, "rb").read()
    found = [i for i in range(0, min(len(app), 1024), 4) if app[i:i + 4] == DESC_MAGIC]
    if len(found) != 1:
        sys.exit(f"{app_path}: no app descriptor in the first 1 KB")
    sp, reset = struct.unpack_from("<II", app, 0)
    if not APP_ADDR <= reset < APP_ADDR + len(app):
        sys.exit(f"{app_path}: reset vector 0x{reset:08x} is outside the app, linked for another address?")
    head = struct.pack("<IIII16x", MAGIC, APP_ADDR, len(app), found[0]) + hashlib.sha256(app).digest()
    der = key.sign(head, ec.ECDSA(hashes.SHA256()))
    r, s = utils.decode_dss_signature(der)
    with open(out, "wb") as f:
        f.write(head + r.to_bytes(32, "big") + s.to_bytes(32, "big") + app)


def full(boot_path, signed_path, out):
    boot = open(boot_path, "rb").read()
    signed = open(signed_path, "rb").read()
    if len(boot) > BOOT_SIZE:
        sys.exit(f"{boot_path}: {len(boot)} bytes, the bootloader has {BOOT_SIZE}")
    image = boot.ljust(BOOT_SIZE, b"\xff") + signed[:HEADER].ljust(APP_OFFSET - BOOT_SIZE, b"\xff")
    with open(out, "wb") as f:
        f.write(image + signed[HEADER:])


if __name__ == "__main__":
    cmds = {"keygen": (keygen, 1), "pubkey": (pubkey, 2), "sign": (sign, 3), "full": (full, 3)}
    if len(sys.argv) < 2 or sys.argv[1] not in cmds or len(sys.argv) - 2 != cmds[sys.argv[1]][1]:
        sys.exit(__doc__)
    cmds[sys.argv[1]][0](*sys.argv[2:])
