# sofd

sofd works out when a USB host controller sends its start-of-frame (SOF)
packets, in terms of the host's own clocks. It reads the controller's frame
counter register a few times per second and compares it with
`CLOCK_MONOTONIC_RAW` and `CLOCK_REALTIME`.

| Controller | Platforms | Register | How a sample is taken |
|---|---|---|---|
| xHCI | Jetson Orin, PCs | `MFINDEX` (125 µs microframes) | spins across the next counter step or two, at most about 250 µs per sample |
| DWC2 host | Raspberry Pi (dwc_otg / dwc2) | `HFNUM` FrNum + FrRem | a burst of 8 reads. FrRem gives the phase, so there is no spinning |
| any other | anything Linux drives | `usb_get_current_frame_number()` via the sofclock module | spins across the next step (usually 1 ms frames) |

Each sample gives the start time of one frame or microframe. A least-squares
fit over the last `sampler.window` samples then gives the controller's period
and offset against each clock. Against the raw oscillator the result is smooth,
but only as good as the host crystal. Against CLOCK_REALTIME it gives the true
SOF frequency and UTC phase, as long as the system clock is disciplined by
GNSS, PTP or NTP. sofd fits a line instead of taking a moving average because
the crystals differ by tens of ppm, and an average would lag by
`drift × window/2`.

## Requirements

- sofd runs as root, because it reads the registers through `/dev/mem` or the
  sysfs PCI `resource0` file.
- Kernels built with `CONFIG_IO_STRICT_DEVMEM=y` (the Debian and Ubuntu desktop
  kernels) or running in lockdown refuse to map registers that a driver owns.
  Boot with `iomem=relaxed`, or load the `sofclock` module described below,
  which sofd then uses automatically.
- While a controller is sampled, its runtime PM is forced `on`. The old setting
  is restored afterwards.

## Building

```sh
cargo build --release                       # target/release/sofd
# static aarch64 binary (Raspberry Pi, Jetson) from a PC:
rustup target add aarch64-unknown-linux-musl
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER=rust-lld cargo build --release --target aarch64-unknown-linux-musl
```

The [top-level README](../README.md#setting-up-a-system) walks through setting
up a complete system, from the board and sofd to the configuration, chrony and
MQTT.

## Commands

```
sudo sofd run [--config /etc/sofd/sofd.hujson]   # the service (see Configuration)
sofd list                                        # controllers, backends, devices (JSON)
sofd lookup --device 0483:5740 [--serial X]      # controller for a device
sofd lookup --port 1-1.2
sofd probe 0000:00:14.0 --rate 10 --count 20     # sample one controller, events to stdout
sofd probe 0000:00:14.0 --gnss auto              # with the sofgnss board on that controller
```

The global options are:

- `--simulate` adds a synthetic controller `sim0` with a device on port
  `sim0-1`, so you can test without hardware or root.
- `--mmio ID=ADDR[:LEN]` gives a register window that discovery cannot find.
- `--generic ID` forces the generic kernel fallback for a controller.
- `--irq-off-max-us N` lets the sofclock module disable interrupts for up to
  N µs. The default is 0, which means never.

`probe` also takes `--config` for sampler and board options. It never connects
to MQTT.

`run` keeps going until it gets SIGINT, SIGTERM, SIGHUP or SIGQUIT. It then
restores the power settings and clears its retained MQTT state, so the models
are emptied, boards show `stopped` and `status` goes offline. It uses every
sofgnss board it finds, whether plugged in now or later. It samples the
controllers those boards are on, as well as any controller an MQTT client asks
for, and leaves the others alone. If a controller cannot be opened, sofd
retries after 10 s, and then less often (up to every 5 min) while the same
error keeps coming back.

`run` stores its last epoch in `/var/lib/sofd/epoch` (`StateDirectory=sofd` in
`sofd.service`). That way epochs keep increasing across restarts, even on a
machine without an RTC whose clock comes up behind. To install sofd as a
service:

```sh
sudo install -m 755 target/release/sofd /usr/local/bin/sofd
sudo install -D -m 644 sofd.example.hujson /etc/sofd/sofd.hujson   # then edit
sudo install -m 644 sofd.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now sofd
```

## Configuration

The configuration file is HuJSON, which is JSON with `//` and `/* */` comments
and trailing commas. The default path is `/etc/sofd/sofd.hujson`. Without that
file sofd runs with the defaults below, which means all boards, no MQTT and no
chrony. Every section is optional, and `sofd.example.hujson` shows them all.

```jsonc
{
  "mqtt": {                       // omit: no broker connection
    "host": "localhost", "port": 1883,
    "prefix": "sofd/pc1",         // default "sofd/<hostname>"
    "client_id": "sofd-pc1",      // default "sofd-<hostname>"
    "username": "sofd", "password_file": "/etc/sofd/mqtt.pass",   // or "password"
  },
  "sampler": {
    "window": 64, "default_rate_hz": 50, "model_hz": 1, "rt_prio": 50, "cpu": 2,
    "max_cpu_latency_us": 200,    // while sampling: no CPU sleep state slower to leave than this
    "cpu_governor": "performance",// while sampling (on "cpu" if set, else all CPUs)
  },
  "gnss": {
    "attach": "all",              // "all" | "none" | ["E072A1F9D198", …] (USB serials)
    "defaults": {                 // every board; "boards" overrides per serial
      "baud": 38400,              // rate sofd switches the receiver to
      "initial_baud": 9600,       // the receiver's rate before configuration (power-on)
      "configure": true,          // false: use the receiver as it is, at initial_baud
      "frames": true,             // publish every receiver frame
      "div": null,                // board SOF tag divider (board default)
      "sof_delay_ns": 0,          // bus frame start to the board's capture
      "chrony": "/run/chrony.sofgnss.sock",   // SOCK refclock; "{serial}" is substituted
      "firmware": "/var/lib/sofd/sofgnss.bin",// image the boards should run
    },
    "boards": { "E072A1F9D198": { "chrony": null } },   // null switches chrony or firmware off
    "external": { "myapp": { "chrony": "/run/chrony.myapp.sock", "sof_delay_ns": 0 } },
  },
}
```

sofd reads the file once at startup. To apply changes, run
`systemctl restart sofd`. SIGHUP stops sofd, it does not reload it. The only
thing sofd watches afterwards is the contents of the `firmware` image. Unknown
keys are an error, so a misspelled option stops sofd at startup instead of
being silently ignored. A file given with `--config` must exist. Without
`--config`, a missing `/etc/sofd/sofd.hujson` just means the built-in defaults.

| Option | Default | Meaning |
|---|---|---|
| `mqtt` | absent | Leave it out to run without a broker (chrony only, for example). sofd uses MQTT 5 over plain TCP and reconnects by itself. |
| `mqtt.host`, `mqtt.port` | `localhost`, `1883` | Broker. |
| `mqtt.prefix` | `sofd/<hostname>` | Prefix of every topic. It must not be empty, contain `+` or `#`, or end in `/`. |
| `mqtt.client_id` | `sofd-<hostname>` | MQTT client id. |
| `mqtt.username` | none | Credentials are only sent when this is set. |
| `mqtt.password`, `mqtt.password_file` | none | The file wins when both are given. Trailing whitespace in it is removed. |
| `sampler.window` | 64 | Number of samples in each least-squares fit (3–100000). A longer window averages out more noise but follows crystal drift more slowly. At 50 Hz, 64 samples span 1.3 s. |
| `sampler.default_rate_hz` | 50 | Sampling rate (0.1–1000 Hz) for a controller with a board, and for MQTT requests without `rate_hz`. It is also the rate of chrony samples. |
| `sampler.model_hz` | 1 | How often `controllers/<id>/model` is published (0.1–1000 Hz). A client can ask for more with `model_hz` in its want request. Changes such as a new epoch, a fit appearing or disappearing, or a different GNSS reference are published straight away. |
| `sampler.rt_prio` | none | Run the sampler threads SCHED_FIFO at this priority (1–99) and lock sofd's memory. |
| `sampler.cpu` | none | Pin the sampler threads to this CPU. |
| `sampler.max_cpu_latency_us` | none | While sampling, keep the CPUs out of sleep states that take longer than this to leave. |
| `sampler.cpu_governor` | none | While sampling, use this cpufreq governor on `cpu` if it is set, or on all CPUs otherwise. The previous governor is restored afterwards. |
| `gnss.attach` | `"all"` | Which sofgnss boards to use. `"all"` means every board with the sofgnss descriptors and `"none"` means none. A list of USB serials picks those boards, whatever their descriptors say. |
| `gnss.defaults` | | Board options (below) for every board. |
| `gnss.boards.<serial>` | | Board options for one board, by USB serial. Each option set here replaces the one from `defaults`. Options left out fall back to `defaults`, then to the built-in default. |
| `gnss.external.<client>` | none | An MQTT client that may act as a GNSS reference ([below](#gnss-reference-from-an-mqtt-client)). It takes `chrony` (a socket path) and `sof_delay_ns` (as for boards). |

Board options:

| Option | Default | Meaning |
|---|---|---|
| `baud` | 38400 | Rate sofd switches the receiver to. Keep it at 38400 or more for the diagnostics messages. |
| `initial_baud` | 9600 | The receiver's rate before configuration, usually its power-on rate (38400 for u-blox M10/F9). sofd tries it right after `baud`, and if neither gives data it also searches the standard rates. |
| `configure` | `true` | With `false` sofd leaves the receiver as it is and uses it at `initial_baud`. Pulses are then labelled from TIM-TP if the receiver sends it, and from NMEA ZDA otherwise. |
| `frames` | `true` | Publish every receiver frame on `gnss/<serial>/nmea\|ubx\|rtcm3/…`. Set it to `false` to save broker traffic. sofd still uses the frames itself. |
| `div` | board's own (100) | The board timestamps one SOF in every `div` frames (1–1000000). The default gives 10 tags per second at 1 ms frames. More tags add points to `model_gnss` but use more USB bandwidth. |
| `sof_delay_ns` | 0 | The board's fixed delay from the start of the bus frame to its capture (within ±1 s). It is subtracted from every tag's GNSS time, so a positive value moves `model_gnss` and chrony earlier. |
| `chrony` | none | Path of a chrony SOCK refclock socket. `{serial}` is replaced by the board's serial. `null` in `boards` turns it off for one board. |
| `firmware` | none | The sofgnss image (`build/sofgnss.bin`) the board should run. Boards running another image are updated. `null` in `boards` turns updates off for one board. |

A controller sampled for a board runs at `sampler.default_rate_hz` (50). That
is also the rate of the samples sent to chrony, and of MQTT requests that do
not give a rate. Each sample costs a spin of up to one microframe (125 µs),
which is about 0.3 % of a CPU at 50 Hz. `rt_prio` runs the sampler threads
SCHED_FIFO and locks memory, and `cpu` pins them. Values out of range are
refused at startup. The ranges are 0.1–1000 Hz for rates, 3–100000 for
`window`, 1–99 for `rt_prio` and ±1 s for `sof_delay_ns`.

`max_cpu_latency_us` and `cpu_governor` trade power for tighter samples. They
only apply while some controller is being sampled. The latency limit is held
through an open `/dev/cpu_dma_latency`, which the kernel releases when sofd
exits, however it exits. The governor is set by writing `scaling_governor`,
and the previous one is restored when sampling stops or sofd exits, though not
after a crash. These are the results on the Comet Lake PC (xHCI through
sofclock, C-state exit latencies of 151 µs for C2 and 1034 µs for C3) with a
release build at 10 Hz:

| Settings | sample `unc_ns` | `model_raw` rms | realtime − GNSS noise |
|---|---|---|---|
| none (`powersave`, all C-states) | ±1460 ns | 440 ns | 23 ns |
| `max_cpu_latency_us: 200` only | ±1460 ns | 420 ns | 22 ns |
| `cpu_governor: performance` only | ±1460 ns | 440 ns | 23 ns |
| both | ±560 ns | 173 ns | 9 ns |

Only the combination helps there. Setting the limit below C2's exit latency
(to 10 µs) gave no further gain. A debug build is too slow to show any
difference.

The bracket is the duration of one register read, which follows the core and
uncore clocks. On that laptop the firmware switches turbo on and off by
itself, for minutes at a time (`intel_pstate/no_turbo`). Without turbo the
reads take ±1000–1450 ns even with both settings, which gives about 440 ns of
sample noise instead of 165 ns. With HWP, `performance` pins the cores at the
maximum that was valid when the governor was set, so sofd sets it again
whenever `no_turbo` changes. Otherwise a start without turbo would hold every
core at base clock for as long as sofd samples.

## MQTT

sofd speaks MQTT 5 over plain TCP, and all topics are below the prefix.
Retained topics hold the current state, so a client that connects later sees
it at once. [MQTT-GUIDE.md](MQTT-GUIDE.md) shows how a client can time USB
frames from these topics.

| Topic | Retained | Payload |
|---|---|---|
| `status` | yes | `{"state":"online","version","pid"}`. The will sets `{"state":"offline"}`. |
| `controllers/<id>/info` | yes | The controller and its devices, `rate_hz` and `model_hz` (null when not sampled), `wanted_by`, and `wants` with each client's `rate_hz`, `model_hz`, `samples` and `expires_ms`. |
| `controllers/<id>/model` | yes | The latest fits (`epoch`, `unit_ns`, `counter`, `model_*`, `gnss`, `frame_start`). Published at `model_hz` (1 Hz by default), and straight away on a new epoch, when a fit appears or disappears, or when the GNSS reference changes. Emptied when sampling stops, on a controller error until the next sample, and when sofd stops. |
| `controllers/<id>/sample` | | Every sample, but only while some client's request has `"samples":true`. |
| `controllers/<id>/event` | | `error`, `recovered` and `realtime_step`, plus `ref_*` and `chrony_busy` for MQTT references. |
| `controllers/<id>/want/<client>` | yes, by clients | `{"rate_hz":20}`, or `{}` for the default rate. `"model_hz":5` publishes `model` more often and `"samples":true` publishes `sample`. An empty payload withdraws the request. With `"ttl_s":10` the request lapses unless it is renewed, and it is not retained. Anything else is ignored, such as a payload that is not an object, a rate ≤ 0 or a `samples` that is not a boolean. |
| `controllers/<id>/ref/<client>` | by clients | SOF tags of an MQTT GNSS reference (below). |
| `gnss/<serial>/status` | yes | Board state (below). |
| `gnss/<serial>/pps` | | Each pulse and whether it was used. |
| `gnss/<serial>/event` | | `detached`, `firmware_updated`, `firmware_failed`, `chrony_busy`. |
| `gnss/<serial>/nmea/<address>` | | Raw sentence, e.g. `.../nmea/GPZDA`. |
| `gnss/<serial>/ubx/<class>/<id>` | | Raw UBX frame (binary, hex ids), e.g. `.../ubx/0d/01`. |
| `gnss/<serial>/rtcm3/<type>` | | Raw RTCM3 frame, e.g. `.../rtcm3/1005`. |
| `gnss/<serial>/tx` | by clients | Bytes for the receiver, written as they are. |
| `cmd/list`, `cmd/lookup` | | Request and response. sofd replies to the MQTT 5 response topic, or to `reply_to` in the body for MQTT 3.1.1. `correlation` is echoed back. Replies are not retained. |

A controller is sampled while a board is on it or any client wants it, at the
highest rate anyone asked for. Its `model` is published at the highest
`model_hz` asked for, at least `sampler.model_hz` and at most the sampling
rate. A client should set its will to an empty retained payload on its own
`want/<client>` topic, so the request disappears along with the client. A
client without a will (MQTT 3.1.1 tooling, CI jobs) sends `ttl_s` instead and
renews the request, not retained, about every `ttl_s`/3 (see MQTT-GUIDE.md).
Retained requests survive a sofd restart. After a broker reconnect, a request
has to be delivered again within 10 s or it lapses, and retained requests
arrive again by themselves. This way a request that was withdrawn while sofd
was away, or whose client's will fired, does not linger.

```sh
mosquitto_sub -v -t 'sofd/pc1/gnss/+/status' -t 'sofd/pc1/controllers/+/model'
mosquitto_pub -r -t 'sofd/pc1/controllers/0000:00:14.0/want/me' -m '{"rate_hz":20}'
mosquitto_pub -r -t 'sofd/pc1/controllers/0000:00:14.0/want/me' -n          # withdraw
printf '\xb5\x62\x0a\x04\x00\x00\x0e\x34' | mosquitto_pub -t 'sofd/pc1/gnss/E072A1F9D198/tx' -s   # MON-VER poll
mosquitto_rr -V 5 -t sofd/pc1/cmd/lookup -e me/reply -m '{"port":"1-2"}'   # or "device"+"serial", "busnum"+"devnum"
mosquitto_pub -V mqttv311 -t sofd/pc1/cmd/list -m '{"reply_to":"ci/job42","correlation":42}'   # MQTT 3.1.1: reply topic in the body
```

`tx` is not filtered, so a client can reconfigure or reset the receiver.
sofd's timeouts then redo the setup. Bytes are only dropped while a firmware
image is being sent.

Here is a sample, as published on request and as `probe` prints it:

```jsonc
{"event":"sample","controller":"0000:00:14.0",
 "epoch":1759614087123,  // counter continuity domain: new when sampling (re)starts, speed change, controller reset
 "counter":1234567,      // unwrapped frame counter in units of unit_ns
 "unit_ns":125000,       // 125000 = HS microframes, 1000000 = FS frames
 "bus_frame":1234,       // 11-bit frame number on the wire (compare with a device's SOF frame number);
                         // null where the controller's count does not give it (EHCI via the generic path)
 "microframe":3,         // HS only
 "t_raw_ns":…, "t_mono_ns":…, "t_realtime_ns":…,   // start of that (micro)frame
 "unc_ns":350,           // half-width of the interval it is known to lie in
 "missed":0,             // attempts discarded since the previous sample (stalls, brackets > unit/4)
 "mono_minus_raw_ns":…, "realtime_minus_raw_ns":…,
 "model_raw":{"counter_ref":…, "t_ref_ns":…, "period_ns":125000.0029,
              "ppm":-0.02, "resid_rms_ns":180, "n":64},   // vs CLOCK_MONOTONIC_RAW; null until 3 samples
 "model_realtime":{… same fields, t_ref_ns in CLOCK_REALTIME …},   // restarts after a realtime step
 "model_gnss":{… same fields, t_ref_ns in GNSS UTC …},             // only while the GNSS reference is valid
 "realtime_sync":{"kernel_synced":false, "freq_ppm":35.1,         // from adjtimex; chrony only sets
                  "est_error_ns":0, "max_error_ns":4508000},      // kernel_synced with `rtcsync`
 "frame_start":{"bus_frame":1234, "counter":1234560,   // 1 ms frame containing this sample
                "t_raw_ns":…, "t_realtime_ns":…, "t_gnss_ns":…},   // from the fits; each null while its fit is
 "gnss":{"board":"E072A1F9D198", "source":"tim_tp",   // or "zda"; null without a board
         "frame_offset":-14336,                       // host frame = board frame + offset
         "realtime_minus_gnss_ns":-919790}}           // CLOCK_REALTIME - GNSS UTC at frame_start
{"event":"realtime_step","controller":"…","step_ns":…}   // CLOCK_REALTIME moved more than the kernel's slew allows
{"event":"error","controller":"…","message":"…"}   // once per distinct error; retried every ≥1 s
{"event":"recovered","controller":"…"}
```

The sample's own timestamp is the start of whichever microframe it caught
(`microframe` 0–7). Full-speed devices only see SOF at 1 ms frame boundaries,
which is microframe 0. `frame_start` gives that boundary for the sample's
frame, predicted from the fits, so it is also less noisy than a single sample.

Within one epoch, each model gives the start of any counter value `c` as
`t = t_ref_ns + (c - counter_ref) * period_ns`. `ppm` is the controller's
frequency error against that clock, and it is positive when SOFs come too
fast. While the time daemon is not slewing away an offset,
`model_realtime.ppm` ≈ `model_raw.ppm` − `realtime_sync.freq_ppm`. During a
slew the realtime fit includes the slew, which is correct because
CLOCK_REALTIME itself runs at that rate. `t_raw_ns` and `t_mono_ns` only mean
something on the host itself.

An epoch is at least the Unix time in milliseconds at which it began, and it
is always above the last epoch of the previous run (`/var/lib/sofd/epoch`).
Epochs therefore stay unique across sofd restarts, and a stored
`(epoch, counter)` pair cannot be confused with a later count.

`model_gnss`, `frame_start.t_gnss_ns` and `gnss.realtime_minus_gnss_ns` are
only published while the GNSS reference is valid. That means at least 5
points, with the newest tag matched and received within the last 5 s. During
a GNSS outage they are null rather than extrapolated.

## GNSS reference (sofgnss)

A [sofgnss](../sofgnss) board timestamps the GPS PPS and the USB SOFs it
receives on one 80 MHz counter. It becomes the GNSS reference of the
controller it is plugged into. That controller's samples then gain
`model_gnss` (the counter against GNSS UTC, whose `ppm` is the controller's
true frequency error), `frame_start.t_gnss_ns` and `gnss`.

### Finding boards

sofd notices USB changes through kernel uevents, on the netlink socket that
udev listens on. It only reads the USB tree from sysfs when a USB device,
interface or tty changes, and once a minute regardless. Without that socket it
reads the tree every second.

When `attach` is `"all"`, boards are found by an exact match on VID:PID
20a0:4288 (which is shared), manufacturer `BertoldVdb` and product
`GNSS/SOF/PPS Bridge`. Serials listed explicitly, and a tty given to
`probe --gnss`, are used whatever their descriptors say. Boards are identified
by USB serial, so a board keeps its options on any port. A board that appears
is opened and one that disappears is marked `absent`. When a board's session
fails, sofd reopens it after 1, 2, 4 … 30 s, and starts again from 1 s once a
session has reached a running state.

Only one board per controller is used. The device's frame numbers are matched
to the controller's by their low 11 bits, which are identical on the wire, and
by the time each tag was received. The match is checked on every tag and
redone after three misses.

The states on `gnss/<serial>/status` are `opening`, `updating`, `restarting`
and `update_failed` for firmware, then `probing`, `identifying`,
`configuring`, `verifying`, `ubx` (running), `nmea_fallback`, `passive`,
`detached`, `absent` and `error`. The status also carries `controller`, `tty`,
`port`, `baud`, `receiver` (MON-VER), `firmware` (`version`, `elf_sha256`,
`reset_reason`) and `message`.

### Receiver setup

With `configure` on, sofd sets up the receiver in RAM only, so a power cycle
brings back the receiver's saved settings.

1. Find the receiver. sofd listens for 1.5 s at `baud`, since the receiver may
   still be configured from an earlier session, then at `initial_baud`, then
   at the other standard rates (9600, 38400, 115200, 4800, 19200, 57600,
   230400, 460800, 921600). It repeats this until GPS frames arrive. The board
   only forwards valid frames, so a wrong baud just looks like silence. A
   receiver at one of the configured rates is found within 3 s. One at another
   standard rate takes up to about 14 s.
2. Identify it with UBX-MON-VER and poll its port with CFG-PRT.
3. Enable TIM-TP, NAV-TIMEGPS, NAV-STATUS and the diagnostics below, turn NMEA
   output off and set the baud. Up to u-blox 8 this uses CFG-MSG and CFG-PRT.
   Generation 9 and 10 (protocol ≥ 27, or MON-VER hardware 00190000/000A0000)
   get CFG-VALSET on the RAM layer. The time pulse stays on the receiver's own
   grid (GPS by default), and NAV-TIMEGPS gives the leap seconds for its
   labels. The edge is the same second boundary on either grid.
4. Switch the port to the new baud and wait for TIM-TP.

sofd also turns on some diagnostics for monitoring peers. They are published
like every other receiver frame on `gnss/<serial>/ubx/<class>/<id>`, and sofd
itself does not use them. The large ones only come every 5th navigation epoch
so they do not keep the line busy at the pulse. At 38400 baud the 5th epoch
carries about 1 KB, which takes 0.25 s, so keep `baud` at 38400 or more.

| | u-blox 9/10 | u-blox 8 | u-blox 7 | u-blox 5/6 |
|---|---|---|---|---|
| solution, with UTC (every epoch) | NAV-PVT `01/07` | NAV-PVT | NAV-PVT | NAV-SOL `01/06` + NAV-TIMEUTC `01/21` |
| satellites: C/N0, elevation, azimuth, use (every 5th) | NAV-SAT `01/35` | NAV-SAT | NAV-SVINFO `01/30` | NAV-SVINFO |
| RF (every 5th) | MON-SPAN `0a/31` (spectrum) | MON-HW `0a/09` (noise, AGC, jamming) | MON-HW | MON-HW |

The generation comes from MON-VER, from the hardware version if possible and
the protocol version otherwise. If the receiver rejects a diagnostics message
or does not acknowledge it, sofd leaves it out and says so in a `configuring`
status message. It never fails the setup.

Any timeout restarts at step 1. That covers a missing ack, no TIM-TP after
setup, no frames for 5 s, no TIM-TP for 10 s, and NMEA appearing again after a
receiver reset. A receiver that does not answer MON-VER, or that fails setup
three times in a row, is used as it is, and its pulses are labelled from NMEA
ZDA instead. With `configure:false` the tty runs at `initial_baud`, and pulses
are labelled from TIM-TP when it arrives and from ZDA otherwise.

### Labelling pulses

TIM-TP, which is sent before the pulse it describes, or ZDA, which is sent
after it, gives each PPS its UTC second. A pulse is only used if

- it came 1 s after the previous one,
- the board reported the GPS line idle (`rx_idle_ms` ≥ 10) at both pulses
  around the label,
- exactly one label arrived in between,
- the receiver reported a fix (NAV-STATUS, or RMC/GGA in ZDA mode) within two
  pulses,
- and the label is exactly 1 s after the previous one.

TIM-TP edges are corrected by qErr, so the edge is the label minus qErr.
Measured on a LEA-5T, this brings the edge noise down to the counter's own
3.9 ns rms, from 5.0 ns uncorrected. Each SOF tag's UTC is interpolated
between the two accepted pulses around it. Tags therefore arrive about 1 s
late (2 s with ZDA), which `model_gnss` absorbs.

```jsonc
// gnss/<serial>/pps
{"event":"gnss_pps","board":"…","controller":"…","tick":…,"rx_idle_ms":312,"source":"tim_tp",
 "utc_ns":…,"qerr_ps":3922,"accepted":true}            // or "accepted":false,"reason":"line_busy"
```

sofd keeps the tty exclusive (TIOCEXCL) and pings the board every 10 s to keep
its host watchdog happy. When it opens the tty it only flushes input. A new
ttyACM starts with ECHO on, so bytes that arrive before sofd sets raw mode are
echoed back. Unlinking those in-flight writes, which is what an output flush
does, locks up the Raspberry Pi's dwc_otg USB driver for a full-speed device
behind a hub.

### Firmware

With `firmware` set, sofd updates any board whose `VER` reports a different
ELF SHA-256 than the image's app descriptor, before it sets up the receiver.
It uses the board's `OTA` command, the same transfer as
`sofgnss/tools/ota.py`. The transfer takes about 3 s, then the board restarts
and is reopened, about 6 s in all. sofd closes the tty as soon as the board
reports the image written, while it still answers, and reopens it once the
board has re-enumerated (or after 5 s if it never leaves the bus).

The file is watched. A changed image is loaded once it has been stable for
2 s, and boards running something else are updated at once. Write the file
with a rename, not in place. If a board still runs the old image after 2
attempts with the same file, sofd leaves it alone (`firmware_failed`) until
the file changes or sofd restarts. The board confirms the new image on the
first command after its restart, so its bootloader rolls back a broken image
that never reaches the USB handshake.

### chrony

With `chrony` set, each sample of the board's controller goes to chrony's
SOCK refclock as one `struct sock_sample`. It holds the sample's
CLOCK_REALTIME and the GNSS time of that same instant from `model_gnss`.
Because `model_gnss` maps the frame counter to GNSS time without going
through CLOCK_REALTIME, the measurement does not depend on how chrony steers
the clock. Samples are only sent while the reference is valid, meaning the
frames are matched, `model_gnss` has at least 5 points and a pulse was
accepted within the last 5 s.

One board feeds one socket. With a default path and several boards, the first
valid board owns the socket until it is gone or has been silent for 10 s, and
the others get `chrony_busy`. Put `{serial}` in the path to get one source per
board. The leap flag is always 0, so give chrony `leapsectz right/UTC`.

```
refclock SOCK /run/chrony.sofgnss.sock refid SOF poll 1 filter 100 precision 1e-7
```

`poll 1` lets chrony update every 2 s, which follows the PC crystal's
temperature wander much better than the default of 16 s. `filter` should be
about rate × 2^poll, which is 100 at 50 Hz. Measured on the Comet Lake laptop
(clock against GNSS, rms of 10 s means):

| chrony refclock options | sofd rate | rms | max |
|---|---|---|---|
| defaults (poll 4) | 10 Hz | 271 ns | 783 ns |
| `poll 1 filter 20 precision 1e-7` | 10 Hz | 100 ns | 322 ns |
| `poll 1 filter 100 precision 1e-7` | 50 Hz | 57 ns | 163 ns |

Turbo was off for most of the last run (440 ns per-sample noise), and the
filter averages that out.

The offset includes the board's fixed delay from the bus frame start to its
capture. Calibrate it with `sof_delay_ns` or chrony's `offset`.
`struct sock_sample` uses the host's `struct timeval`, so on 32-bit systems
the layouts differ when chrony is built with 64-bit `time_t` and sofd is not,
or the other way round.

With a LEA-5T on the Comet Lake PC, sofd locked 4–5 s after attach and
accepted 117 of 120 pulses (the rest were at start-up). The `model_gnss`
residual was 44 ns rms, which is the board's SOF capture.
`realtime_minus_gnss_ns` stayed at −0.92 ms with 255 ns rms over 2 minutes
against an NTP-disciplined clock. Single chrony samples scatter by the
host-side bracket, about 165 ns rms with turbo and 440 ns without. That
scatter is white, so chrony's filter averages it down.

### GNSS reference from an MQTT client

A program that times SOFs against GNSS itself, for example with its own USB
device, can be a controller's GNSS reference instead of a board. It publishes
tags on `controllers/<id>/ref/<client>` in one of two forms. A tag can carry
the device's frame number, which sofd matches by receive time, so each tag
must arrive within about 1.9 s. Or it can carry sofd's own epoch and counter,
which works with any latency.

sofd only accepts clients listed in `gnss.external`, and only that table can
send them to chrony. A board on the controller takes precedence. A reference
that sends nothing for 10 s, or whose tags have all been rejected for 10 s, is
dropped. Tag times must lie between 2000 and 2100, and `unc_ns` between 1 ns
and 1 s. Samples then carry `gnss.board` `mqtt:<client>` and `source`
`external`. [MQTT-GUIDE.md](MQTT-GUIDE.md) has the payload, the events and an
example client.

## sofclock kernel module (PCs, unknown controllers)

`kmod/` holds a small module that exposes `/dev/sofclock` (root only). It has
one ioctl that works in two modes.

- In xHCI (PCI) mode it times MFINDEX steps through the bound `xhci_hcd`
  driver's own mapping (`hcd->regs`). This is for kernels that forbid mapping
  the registers from user space. The device lock keeps the driver from
  unbinding in the middle of a read.
- Generic mode is for controllers whose driver has no register backend in
  sofd. It polls `usb_get_current_frame_number()` on the controller's root
  hub. Most drivers count 1 ms frames (xHCI reports `MFINDEX >> 3`), so the
  spin can take up to about 1 ms per step. sofd measures the tick length
  (125 µs or 1 ms) at startup. The wrap point is up to the driver (for EHCI it
  is the schedule size, usually 1024 frames), so sofd carries the counter
  forward by elapsed time and takes `bus_frame` from the raw value. This mode
  is only used when sofd does not know a controller's driver, or when it is
  forced with `--generic ID`.

[kmod/README.md](kmod/README.md) covers building, loading at boot, Secure Boot
signing, interrupt handling and the ioctl. sofd's own user-space xHCI spin
applies the same bracket test as the module.

For a controller with a known driver, sofd maps the registers directly and
falls back to the module's xHCI mode for PCI xHCI. A controller with an
unknown driver gets the module's generic mode. `list` and `lookup` report
`"backend": "kernel_frame_number"` for unknown drivers, with `"supported"`
true only while the module is loaded.

On a Comet Lake laptop xHCI, the module's xHCI mode gave a median bracket of
±540 ns (±1.46 µs while the CPU sits in a slow power state) and about
200–400 ns rms, with or without interrupts disabled. Generic mode on the same
controller gave the same brackets and about 420 ns rms. Run side by side, its
frame boundaries were within 128 ns and 0.015 ppm of the register path.
`--cpu`, `--rt-prio` and a `performance` governor help with the power-state
effect.

A driver whose `get_frame_number` could sleep would be unsafe in generic mode,
which calls it with interrupts disabled. No such driver is known in mainline.

## Outliers

Samples are judged by their timestamp bracket, not by their distance from the
fit. The register read is fenced (`isb` / `dsb ld` on arm64), so the bracket
always contains the access and the frame start provably lies inside it.

- On xHCI, a step is skipped if MFINDEX jumped by more than one (it stalled
  mid-spin) or if its bracket is longer than 4 of the fastest reads plus 1 µs
  (something paused the loop inside it). The tighter of 2 steps is kept. On
  DWC2 the tightest of 8 reads is kept.
- A sample wider than a quarter unit (31 µs at high speed) is a miss. It is
  retried after 5 ms, up to 3 times per period, and counted in `missed`.
  Misses keep the epoch and the fits.
- The fits drop points wider than 3× the window's median bracket and weight
  the rest by 1/unc².
- Only a controller fault (halted, counter stopped for more than 2 ms, port
  disabled) or a jump off the fit starts a new epoch. A jump means more than a
  quarter unit, or 5 ppm of the gap since the previous sample if that is
  larger, because at low rates crystal drift alone moves the extrapolation
  that far. Frame tags queued from before a new epoch are discarded.
- A GNSS tag, from a board or an MQTT reference, is left out (`inconsistent`)
  when its time differs from `model_gnss` by more than 0.5 ms. This catches a
  tag that arrived more than about 2 s after its frame and was matched to the
  frame one wrap (2.048 s) later. The fit only restarts from the new tags if
  they keep disagreeing, at least 5 in a row received over at least 3 s,
  because then the reference itself jumped. A burst of late tags arrives all
  at once and never restarts the fit.
- A CLOCK_REALTIME step is a change of `realtime_minus_raw_ns` beyond what the
  kernel's frequency and tick corrections (adjtimex) allow, plus 500 ppm of
  phase slew and 20 µs. The realtime fit then restarts.

## Notes

- A device is tied to its controller through sysfs (device → `usbN` root hub →
  parent), so companion controllers, add-in cards and docks resolve correctly.
- Platform controllers are located through the device tree `reg` (preferring
  reg-name `hcd`) translated through the parent `ranges`, with `/proc/iomem` as
  a fallback. The registers are validated (HCIVERSION / GSNPSID) before use.
- sofd has been verified on a Raspberry Pi CM0 (dwc_otg), a Jetson Orin
  (tegra-xusb) and a Comet Lake PC xHCI through sofclock.
