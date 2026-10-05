#!/usr/bin/env python3
"""Update a running sofgps over its CDC port.

    tools/ota.py [--port /dev/ttyACM0] [image.bin]     (default build/sofgps.bin)

Sends the image (protocol in README), waits for the restart, then sends VER,
which confirms the new image so the next reset doesn't roll back.

The board is found again through its /dev/serial/by-id link. The port keeps
its current line coding, since changing it would also change the GPS UART.
"""
import argparse
import fcntl
import glob
import hashlib
import os
import struct
import sys
import termios
import time

import serial

APP_DESC_OFFSET = 32  # esp_image_header_t + first esp_image_segment_header_t
APP_DESC_MAGIC = 0xABCD5432
BY_ID = "/dev/serial/by-id"
BY_ID_PATTERN = "usb-sofgps_*"  # manufacturer string "sofgps"
CMSPAR = getattr(termios, "CMSPAR", 0o10000000000)


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


def line_settings(name):
    """pyserial settings matching the port's current line coding."""
    fd = os.open(name, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        cflag, ospeed = attrs[2], attrs[5]
    finally:
        os.close(fd)
    speeds = {getattr(termios, n): int(n[1:]) for n in dir(termios)
              if n.startswith("B") and n[1:].isdigit()}
    if ospeed not in speeds:
        sys.exit(f"{name}: cannot read the current baud rate")
    sizes = {termios.CS5: 5, termios.CS6: 6, termios.CS7: 7, termios.CS8: 8}
    if not cflag & termios.PARENB:
        parity = serial.PARITY_NONE
    elif cflag & CMSPAR:
        parity = serial.PARITY_MARK if cflag & termios.PARODD else serial.PARITY_SPACE
    else:
        parity = serial.PARITY_ODD if cflag & termios.PARODD else serial.PARITY_EVEN
    return dict(baudrate=speeds[ospeed], bytesize=sizes[cflag & termios.CSIZE], parity=parity,
                stopbits=serial.STOPBITS_TWO if cflag & termios.CSTOPB else serial.STOPBITS_ONE)


def open_port(name, timeout=20.0):
    """Open the port exclusively, waiting for udev to set its permissions."""
    end = time.time() + timeout
    while not os.access(name, os.R_OK | os.W_OK) and time.time() < end:
        time.sleep(0.2)
    while True:
        try:
            p = serial.Serial(name, timeout=0.5, exclusive=True, **line_settings(name))
            # pyserial's lock only stops other pyserial users; TIOCEXCL stops sofd too.
            fcntl.ioctl(p.fileno(), termios.TIOCEXCL)
            return p
        except (serial.SerialException, OSError) as e:
            if time.time() > end:
                sys.exit(f"cannot open {name} ({e}); is another program (sofd) using it?")
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


def wait_for(cond, timeout):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.2)
    return False


def by_id_link(port):
    real = os.path.realpath(port)
    for link in sorted(glob.glob(os.path.join(BY_ID, "*"))):
        if os.path.realpath(link) == real:
            return link
    return None


def find_port():
    """Find the one sofgps port, waiting in case the device is restarting."""
    found = []

    def look():
        found[:] = sorted(glob.glob(os.path.join(BY_ID, BY_ID_PATTERN)))
        if not found and not os.path.isdir(BY_ID):
            found[:] = sorted(glob.glob("/dev/ttyACM*"))
        return bool(found)

    if not wait_for(look, 15):
        sys.exit("no sofgps port found; use --port")
    if len(found) > 1:
        sys.exit("several candidate ports, choose one with --port: " + " ".join(found))
    return found[0]


def ver_fields(ver):
    """(pending, elf_sha256) of a VER sentence."""
    f = ver.split("*")[0].split(",")
    return (f[6] if len(f) > 6 else ""), (f[7] if len(f) > 7 else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "build", "sofgps.bin"))
    ap.add_argument("--port", default=None)
    ap.add_argument("--force", action="store_true", help="update even if the device runs this image")
    args = ap.parse_args()

    port_name = args.port or find_port()
    # Path to reopen after the restart. udev may still be creating the link.
    if wait_for(lambda: by_id_link(port_name), 3):
        stable = by_id_link(port_name)
    else:
        print(f"warning: no {BY_ID} link for {port_name}; will reopen {port_name} after the restart")
        stable = port_name
    image = open(args.image, "rb").read()
    digest = hashlib.sha256(image).hexdigest()
    version, project, elf_sha = image_descriptor(image)
    print(f"{args.image}: {project} {version}, {len(image)} bytes, elf sha256 {elf_sha[:16]}...")
    print(f"device: {stable}")

    p = open_port(port_name)
    p.write(sentence("PVDB,SOFPPS,VER"))
    ver = last_ver(p)
    print(f"running: {ver or 'no VER reply (old firmware?)'}")
    if ver and ver_fields(ver)[1] == elf_sha and not args.force:
        print("device already runs this image; nothing to do (--force to update anyway)")
        return

    p.write(sentence(f"PVDB,SOFPPS,OTA,{len(image)},{digest}"))
    ready = read_until(p, ",OTA,READY", 15)  # erasing the slot takes a moment
    if not ready or ",ERR," in ready:
        sys.exit(f"device did not accept the update: {ready}")

    t0 = time.time()
    sent = 0
    chunk = 4096
    result = None
    while sent < len(image):
        p.write(image[sent:sent + chunk])
        sent += chunk
        # Read replies as we go; SOF and INFO keep streaming.
        while p.in_waiting and not result:
            text = p.readline().decode(errors="replace").strip()
            if ",OTA,ACK," in text:
                acked = int(text.split(",OTA,ACK,")[1].split("*")[0])
                print(f"\r  {acked * 100 // len(image):3d}%", end="", flush=True)
            elif ",OTA,OK" in text:
                result = text  # can arrive before the last write returns
            elif ",ERR," in text:
                sys.exit(f"\nupdate failed: {text}")
    p.flush()
    if not result:
        result = read_until(p, ",OTA,OK", 30)
    print(f"\r  100%  {len(image) / 1024 / (time.time() - t0):.0f} KB/s")
    if not result or ",ERR," in result:
        sys.exit(f"update failed: {result}")
    print("image accepted, device restarting")
    p.close()

    # Wait for the board to re-enumerate.
    if not wait_for(lambda: not os.path.exists(stable), 10):
        print("warning: port did not go away; continuing")
    if not wait_for(lambda: os.path.exists(stable), 20):
        sys.exit(f"{stable} did not come back; reset the board (a bad image rolls back)")
    p = open_port(stable)
    # The first VER may be from before the confirm; ask until it shows confirmed.
    ver = None
    for _ in range(5):
        p.write(sentence("PVDB,SOFPPS,VER"))
        ver = last_ver(p) or ver
        if ver and ver_fields(ver)[0] == "0":
            break
    p.close()
    if not ver:
        sys.exit("no VER reply from the new image; it is not confirmed and a reset rolls back")
    print(f"now running: {ver}")
    pending, running_sha = ver_fields(ver)
    if running_sha != elf_sha:
        sys.exit("the device does not run the new image (rolled back?)")
    if pending != "0":
        sys.exit("the new image is still pending verification; a reset rolls back")


if __name__ == "__main__":
    main()
