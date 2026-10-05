#!/usr/bin/env python3
"""Update a running sofgps over its CDC port.

    tools/ota.py [--port /dev/ttyACM0] [image.bin]     (default build/sofgps.bin)

Protocol (see README): $PVDB,SOFPPS,OTA,<size>,<sha256>*hh -> OTA,READY, then
the raw image, ACKs every 32 KB, OTA,OK, and the device restarts into the new
image. Afterwards this sends VER, which both shows the version and confirms
the image (otherwise the next reset rolls back to the previous one).
"""
import argparse
import glob
import hashlib
import os
import struct
import sys
import time

import serial

APP_DESC_OFFSET = 32  # esp_image_header_t + first esp_image_segment_header_t
APP_DESC_MAGIC = 0xABCD5432


def image_descriptor(image):
    """(version, project, elf_sha256_hex) from the image's esp_app_desc_t."""
    d = image[APP_DESC_OFFSET:APP_DESC_OFFSET + 256]
    magic, = struct.unpack_from("<I", d, 0)
    if magic != APP_DESC_MAGIC:
        sys.exit("not an ESP-IDF application image (no app descriptor)")
    version = d[16:48].split(b"\0")[0].decode()
    project = d[48:80].split(b"\0")[0].decode()
    sha = d[144:176].hex()
    return version, project, sha


def sentence(body):
    s = 0
    for c in body:
        s ^= ord(c)
    return f"${body}*{s:02X}\r\n".encode()


def read_until(port, needle, timeout, on_line=None):
    """Return the first line containing needle or ERR, or None on timeout."""
    end = time.time() + timeout
    while time.time() < end:
        line = port.readline()
        if not line:
            continue
        text = line.decode(errors="replace").strip()
        if on_line:
            on_line(text)
        if needle in text or ",ERR," in text:
            return text
    return None


def open_port(name, timeout=20.0):
    """Open the port, waiting for udev to apply its permissions first: the
    node appears (root-only) a few seconds before the rules run."""
    end = time.time() + timeout
    while not os.access(name, os.R_OK | os.W_OK) and time.time() < end:
        time.sleep(0.2)
    while True:
        try:
            return serial.Serial(name, 115200, timeout=0.5)
        except serial.SerialException:
            if time.time() > end:
                raise
            time.sleep(0.2)


def last_ver(port, wait=1.5):
    """Latest VER line within `wait` seconds, skipping older ones in the backlog."""
    found = None
    end = time.time() + wait
    while time.time() < end:
        line = port.readline()
        if line and b",VER," in line:
            found = line.decode(errors="replace").strip()
    return found


def wait_for_port(present, timeout):
    end = time.time() + timeout
    while time.time() < end:
        ports = glob.glob("/dev/ttyACM*")
        if bool(ports) == present:
            return ports[0] if ports else None
        time.sleep(0.2)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "build", "sofgps.bin"))
    ap.add_argument("--port", default=None)
    ap.add_argument("--force", action="store_true", help="update even if the device runs this image")
    args = ap.parse_args()

    port_name = args.port or wait_for_port(True, 15)  # the device may be mid-restart
    if not port_name:
        sys.exit("no /dev/ttyACM* port")
    image = open(args.image, "rb").read()
    digest = hashlib.sha256(image).hexdigest()
    version, project, elf_sha = image_descriptor(image)
    print(f"{args.image}: {project} {version}, {len(image)} bytes, elf sha256 {elf_sha[:16]}...")

    p = open_port(port_name)
    p.write(sentence("PVDB,SOFPPS,VER"))
    ver = last_ver(p)
    print(f"running: {ver or 'no VER reply (old firmware?)'}")
    if ver and len(ver.split(",")) > 7:
        running_sha = ver.split(",")[7].split("*")[0]
        if running_sha == elf_sha and not args.force:
            print("device already runs this image; nothing to do (--force to update anyway)")
            return

    p.write(sentence(f"PVDB,SOFPPS,OTA,{len(image)},{digest}"))
    ready = read_until(p, ",OTA,READY", 15)  # erasing the slot takes a moment
    if not ready or ",ERR," in ready:
        sys.exit(f"device did not accept the update: {ready}")

    t0 = time.time()
    sent = 0
    acked = 0
    chunk = 4096
    while sent < len(image):
        p.write(image[sent:sent + chunk])
        sent += chunk
        # Read replies as we go; SOF and INFO keep streaming.
        while p.in_waiting:
            text = p.readline().decode(errors="replace").strip()
            if ",OTA,ACK," in text:
                acked = int(text.split(",OTA,ACK,")[1].split("*")[0])
                print(f"\r  {acked * 100 // len(image):3d}%", end="", flush=True)
            elif ",ERR," in text:
                sys.exit(f"\nupdate failed: {text}")
    p.flush()
    result = read_until(p, ",OTA,OK", 30)
    print(f"\r  100%  {len(image) / 1024 / (time.time() - t0):.0f} KB/s")
    if not result or ",ERR," in result:
        sys.exit(f"update failed: {result}")
    print("image accepted, device restarting")
    p.close()

    if wait_for_port(False, 10) is not None and glob.glob("/dev/ttyACM*"):
        print("warning: port did not go away; continuing")
    new_port = wait_for_port(True, 20)
    if not new_port:
        sys.exit("device did not come back; reset it (a bad image rolls back)")
    p = open_port(new_port)
    # The first VER may be from before the confirm; ask until it shows confirmed.
    ver = None
    for _ in range(5):
        p.write(sentence("PVDB,SOFPPS,VER"))
        ver = last_ver(p) or ver
        if ver and not ver.split(",")[6].startswith("1"):
            break
    p.close()
    if not ver:
        sys.exit("no VER reply from the new image; it is not confirmed and a reset rolls back")
    print(f"now running: {ver}")
    if ver.split(",")[6].startswith("1"):
        print("warning: image still pending verification")


if __name__ == "__main__":
    main()
