# sofcapture

Timestamps USB start-of-frame (SOF) on a Linux host against GNSS time.

Every USB host controller sends a SOF packet every 1 ms (full speed) or 125 µs
(high speed), carrying an 11-bit frame number. All devices on that controller
see the same SOFs, so the frame number is a clock that the host and its
devices share. sofcapture gives each frame its start time in host time
(`CLOCK_MONOTONIC_RAW`, `CLOCK_REALTIME`) and, when a GNSS reference is
present, in GNSS UTC.

I use it for two things. The first is timestamping data from USB devices such
as SDRs, ADCs and sensors. If a device knows which frame its data belongs to,
it gets an exact host or UTC time for it. The second is disciplining the host
clock. sofd feeds chrony a GNSS-referenced time that doesn't depend on how
chrony steers the clock, and on a laptop that gets to about 50 to 100 ns rms.

## How it works

```
 GNSS receiver ──UART + PPS──▶ sofgnss board ──USB (CDC)──▶ host controller ◀── sofd samples
                              timestamps PPS and the                           its frame counter
                              SOFs it receives on one                          against host clocks
                              hardware counter
                                                         sofd ──▶ MQTT (models, samples, GNSS frames)
                                                              └─▶ chrony (SOCK refclock)
```

1. sofd, the host daemon written in Rust, reads the controller's frame
   counter register a few dozen times per second. That is `MFINDEX` on xHCI,
   `HFNUM` on DWC2, or any controller through the sofclock kernel module. A
   least-squares fit maps the counter to each host clock.
2. sofgnss is the firmware for an STM32G474 board or, alternatively, an
   ESP32-S3 board. It bridges a u-blox receiver to USB and timestamps both the
   receiver's PPS and the incoming SOFs on the same hardware counter. sofd
   matches the board's frame numbers to the controller's and labels each
   pulse with its UTC second. That gives
   a third fit, from the controller counter to GNSS UTC (`model_gnss`).
3. sofd publishes the fits over MQTT, so any client can turn a frame number
   into a time. It can also send samples to chrony.

Without a board, sofd still gives SOF timing against the host clocks. A
program that already knows the GNSS time of some SOFs can be the GNSS
reference over MQTT instead of a board. sofd can also run a receiver that is
not on a board, wired to a serial port with its PPS on the host, or a u-blox
module on its own USB port, which gives its data and at best a time good to
milliseconds (see
[Receivers without a board](sofd/README.md#receivers-without-a-board)).

## Parts

| Directory | What | README |
|---|---|---|
| [`sofd/`](sofd) | Host daemon: controller sampling, fits, board handling, MQTT, chrony, firmware updates | [sofd/README.md](sofd/README.md), [MQTT-GUIDE.md](sofd/MQTT-GUIDE.md) for client programs |
| [`sofd/kmod/`](sofd/kmod) | `sofclock` kernel module, needed on PCs with locked-down `/dev/mem` and for controllers sofd has no register backend for | [sofd/kmod/README.md](sofd/kmod/README.md) |
| [`sofref/`](sofref) | Go module for programs that feed sofd a GNSS reference from their own device (PPS + SOF captures → tags over MQTT) | [sofref/README.md](sofref/README.md) |
| [`sofgnss/`](sofgnss) | GNSS/SOF/PPS bridge firmware for the WeAct STM32G474 core board (default) and the Seeed XIAO ESP32-S3, OTA and SDR calibration tools | [sofgnss/README.md](sofgnss/README.md) |

Tested on a Raspberry Pi CM0 (dwc_otg), a Jetson Orin (tegra-xusb) and a
Comet Lake PC (xHCI through sofclock), with a u-blox LEA-5T receiver.

## Setting up a system

### 1. Hardware

You need a sofgnss board and a u-blox receiver with a PPS output
(generations 5 to 10 are supported). The board is either of:

- the **WeAct STM32G474 core board**, the default. It captures every SOF in
  hardware, with about 14 ns rms of stamp noise. Wiring:
  [sofgnss/stm32g4/README.md](sofgnss/stm32g4/README.md#hardware-weact-stm32g474-core-board).
- alternatively, the **Seeed Studio XIAO ESP32-S3**, which takes SOF
  timestamps in software, with about 46 ns rms. Wiring:
  [sofgnss/esp32s3/README.md](sofgnss/esp32s3/README.md#hardware-seeed-studio-xiao-esp32-s3-defaults).

Both run the same protocol and sofd handles them alike. The
[comparison in sofgnss/README.md](sofgnss/README.md#choosing-a-board) has the
details.

Plug the board into the controller whose SOFs you want timed. Hubs in between
are fine. sofd uses one board per controller. The board's LED shows what it
receives: on means valid GPS frames, and blinks mean PPS (see
[Status LED](sofgnss/README.md#status-led)).

### 2. Firmware

Create the board's signing key once and keep a backup, because a board only
accepts updates signed with the same key. Then build the firmware and flash
it once. The board pages have the steps:
[STM32G474](sofgnss/stm32g4/README.md#building-and-flashing) (USB DFU or
ST-LINK) and [ESP32-S3](sofgnss/esp32s3/README.md#building-and-flashing) (ROM
bootloader). After that, sofd updates boards itself (`firmware` option,
step 5).

### 3. Build and install sofd

```sh
cd sofd
cargo build --release                       # native
# Raspberry Pi / Jetson from a PC (static binary):
# rustup target add aarch64-unknown-linux-musl
# CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=rust-lld cargo build --release --target aarch64-unknown-linux-musl

sudo install -m 755 target/release/sofd /usr/local/bin/sofd
sudo install -D -m 644 sofd.example.hujson /etc/sofd/sofd.hujson
sudo install -m 644 sofd.service /etc/systemd/system/
```

Check that sofd sees your controllers and the board:

```sh
sofd list                                    # controllers, backend, "supported", devices
sudo sofd probe <controller> --gnss auto     # e.g. 0000:00:14.0 or 3f980000.usb, Ctrl-C to stop
```

`probe` prints samples as JSON. Once the board has locked, which takes a few
seconds after the receiver has a fix, `model_gnss` is filled in.

### 4. Register access (PCs only)

On the Raspberry Pi and the Jetson, sofd maps the registers through
`/dev/mem` and needs nothing else. Most desktop distribution kernels
(`CONFIG_IO_STRICT_DEVMEM`, lockdown) refuse that mapping. If `sofd list`
shows `"supported": false`, use one of these:

- boot with `iomem=relaxed` on the kernel command line, or
- build and load the sofclock module, which sofd then uses automatically
  (setup in [sofd/kmod/README.md](sofd/kmod/README.md)). This is also the
  only option for controllers that are not xHCI or DWC2.

### 5. Configure sofd

Edit `/etc/sofd/sofd.hujson`. It is JSON with comments and trailing commas,
and every section is optional. Without the file, sofd uses every board and
runs without MQTT or chrony. A typical GNSS time server:

```jsonc
{
  "mqtt": { "host": "localhost", "port": 1883 },   // leave out: no MQTT
  "sampler": {
    "default_rate_hz": 50,           // sampling rate of a board's controller = chrony sample rate
    "rt_prio": 50,                   // SCHED_FIFO for the sampler threads
    // "max_cpu_latency_us": 200,    // PCs: these two together roughly halve the sample noise,
    // "cpu_governor": "performance",//      at the cost of power (held only while sampling)
  },
  "gnss": {
    "attach": "all",                 // or "none", or a list of board USB serials
    "defaults": {
      "baud": 38400,                 // rate sofd switches the receiver to (keep ≥ 38400)
      "initial_baud": 9600,          // receiver's power-on rate (38400 for u-blox M10/F9)
      "chrony": "/run/chrony.sofgnss.sock",
      "firmware": "/var/lib/sofd/sofgnss.bin",     // boards are updated to this image
    },
    "boards": {
      // "003000343645501720393657": { "sof_delay_ns": 1300 },   // per-board overrides, by USB serial
    },
  },
}
```

A few things about this config:

- sofd configures the receiver itself, in RAM only. It enables the UBX
  timing messages, turns NMEA off and sets `baud`. Set `"configure": false` to
  use the receiver as it is, at `initial_baud`.
- Install the `firmware` image by copying
  `sofgnss/stm32g4/build/sofgnss-signed.bin` (`sofgnss/esp32s3/build/sofgnss.bin`
  for ESP32-S3 boards) to a temporary file and renaming it into place, rather
  than writing over it. sofd watches the file and updates attached boards when
  it changes. Boards of the other kind are left alone.
- `sof_delay_ns` is the board's fixed delay from the start of the bus frame to
  its capture. sofd's default for STM32 boards (1300 ns) was measured and is
  good to about 0.2 µs. For ESP32-S3 boards it is 0, so calibrate it against a
  known reference if you need absolute accuracy below a microsecond.
- Every option and its limits are documented in
  [sofd/README.md](sofd/README.md#configuration).

Start the service:

```sh
sudo systemctl daemon-reload && sudo systemctl enable --now sofd
journalctl -u sofd -f
```

### 6. chrony (optional)

Add the socket from `gnss.defaults.chrony` to `/etc/chrony/chrony.conf`.
chrony creates the socket and sofd sends to it:

```
refclock SOCK /run/chrony.sofgnss.sock refid SOF poll 1 filter 100 precision 1e-7
leapsectz right/UTC
```

`filter` should be about rate × 2^poll, which is 100 at 50 Hz. Then run
`sudo systemctl restart chrony` and check with `chronyc sources`. After a
while `SOF` should become the selected source (`*`). If several boards are
attached, put `{serial}` in the path to get one socket per board.

### 7. MQTT (optional)

Any MQTT 5 broker works, for example mosquitto on the same host. sofd
publishes under `sofd/<hostname>` (change with `mqtt.prefix`):

```sh
mosquitto_sub -v -t 'sofd/+/status' -t 'sofd/+/gnss/+/status' -t 'sofd/+/controllers/+/model'
```

- `controllers/<id>/model` holds the current fits. It is published once a
  second, and at once when something changes. To turn a frame number into a
  time, use `t = t_ref_ns + (counter - counter_ref) × period_ns`.
- To have a controller without a board sampled, publish a retained
  `controllers/<id>/want/<client>`. The same request can ask for the model
  more often (`model_hz`) or for every sample (`samples`).
- Receiver frames appear on `gnss/<serial>/nmea|ubx|rtcm3/...`. Bytes
  published on `gnss/<serial>/tx` go to the receiver.

[sofd/MQTT-GUIDE.md](sofd/MQTT-GUIDE.md) walks a client program through
finding its controller, requesting sampling, and converting its device's frame
numbers to UTC, and shows how to act as a GNSS reference itself
(`gnss.external`).

## Things to know

- Programs that open the board's serial port themselves (instead of going
  through sofd) must send `$PVDB,SOFPPS,PING*19` at least once a minute.
  Otherwise the board's host watchdog restarts it.
- Only one program can open the board at a time. Stop sofd before running
  `tools/ota.py` or `tools/sdrcal.py log`.
- On the Raspberry Pi, never flush output (`TCOFLUSH`/`TCIOFLUSH`) on the
  board's tty. Unlinking a full-speed write that is in flight behind a hub
  can freeze the dwc_otg driver.

## License

MIT, see [LICENSE](LICENSE).
