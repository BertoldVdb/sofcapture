#!/usr/bin/env python3
"""Minimal stand-in for the peer program: listens on a Unix socket, waits for
sofd to connect, looks up a device, subscribes to its controller, prints a few
samples, unsubscribes and exits.

    tools/fake_peer.py /tmp/sof.sock --port sim0-1 &
    sofd --simulate run --connect /tmp/sof.sock
"""
import argparse
import json
import os
import socket
import time

ap = argparse.ArgumentParser()
ap.add_argument("sock")
ap.add_argument("--port", help="sysfs port path of the device, e.g. 1-1.2")
ap.add_argument("--device", help="vvvv:pppp")
ap.add_argument("--rate", type=float, default=10)
ap.add_argument("--samples", type=int, default=20)
ap.add_argument("--ping-every", type=float, default=10, help="seconds; 0 disables pings")
args = ap.parse_args()

if os.path.exists(args.sock):
    os.unlink(args.sock)
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(args.sock)
srv.listen(1)
print("waiting for sofd on", args.sock)
conn, _ = srv.accept()
f = conn.makefile("rw", buffering=1)


def request(**req):
    f.write(json.dumps(req) + "\n")
    while True:
        msg = json.loads(f.readline())
        if "event" in msg:
            print("event:", msg)
            continue
        print("reply:", msg)
        return msg


query = {k: v for k, v in (("port", args.port), ("device", args.device)) if v}
found = request(cmd="lookup", id=1, **query)
if not found["ok"]:
    raise SystemExit(found["error"])
ctrl = found["controller"]
sub = request(cmd="subscribe", id=2, controller=ctrl, rate_hz=args.rate)
if not sub["ok"]:
    raise SystemExit(sub["error"])

n = 0
last_ping = time.monotonic()
while n < args.samples:
    if args.ping_every and time.monotonic() - last_ping >= args.ping_every:
        f.write(json.dumps({"cmd": "ping"}) + "\n")
        last_ping = time.monotonic()
    line = f.readline()
    if not line:
        raise SystemExit("sofd closed the connection")
    ev = json.loads(line)
    if ev.get("event") != "sample":
        print("message:", ev)
        continue
    n += 1
    m = ev["model_realtime"] or {}
    print(
        f"frame {ev['bus_frame']:4}.{ev.get('microframe', '-')} counter {ev['counter']:>9} "
        f"t_raw {ev['t_raw_ns']} ±{ev['unc_ns']} ns  "
        f"ppm {m.get("ppm", float("nan")):8.3f}  rms {m.get('resid_rms_ns', float('nan')):7.1f} ns"
    )

request(cmd="unsubscribe", id=3, controller=ctrl)
conn.close()
