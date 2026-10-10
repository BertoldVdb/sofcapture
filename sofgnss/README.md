# sofgnss

sofgnss is a USB GPS bridge with hardware timestamps for PPS and USB SOF. It
shows up on the host as a CDC-ACM serial port connected to a GPS receiver.
Besides passing the GPS data through, it timestamps the receiver's PPS edge
and the USB start-of-frame packets it gets from the host, both on one
free-running hardware counter. Both timestamps go into the serial stream as
`$PVDB,SOFPPS,...` sentences, so a host that reads the GPS port sees GPS time,
PPS and its own USB frame clock side by side.

The firmware runs on the ESP32-S3 (ESP-IDF 5.5, TinyUSB). The
platform-independent part is written so it can be reused on an STM32.

On the host, [sofd](../sofd) uses the board as the GNSS reference for its USB
controller. The [top-level README](../README.md#setting-up-a-system) explains
how to set up the whole system.

## What it does

1. **USB to serial.** The board has one CDC-ACM port. Baud rate, data bits,
   parity and stop bits set by the host (`stty`, termios, pyserial and so on)
   are applied to the GPS UART.
2. **PPS timestamping.** PPS goes to the timer's hardware capture input, so
   interrupt latency doesn't end up in the timestamp. The input needs one
   clean edge per pulse. An edge less than 0.5 s after the last accepted one
   is dropped as a glitch and counted in INFO's `<pps_rejected>`. That way
   ringing or a slow, noisy edge can't produce extra PPS sentences, DCD
   pulses or calibration bursts. The downside is that a real pulse right
   after a glitch is lost too.
3. **Framing of the GPS stream.** Bytes from the receiver are split into whole
   NMEA, UBX (up to `SOFGNSS_PARSER_BYTES`, 4096 bytes) and RTCM3 messages,
   each checked by its checksum or CRC, and then queued. The host's stream to
   the GPS is framed the same way. If the queue overflows, the oldest *whole*
   messages are dropped. Messages are never cut. Bytes that don't belong to
   a recognised frame are dropped. NMEA sentences must carry a `*hh` checksum
   or they are dropped. A sentence counts as complete as soon as its checksum
   arrives and is forwarded with a normalised `\r\n`, so receivers that end
   lines with only CR, only LF or nothing at all lose nothing.
4. **SOF timestamping.** One USB frame in every N is timestamped on the same
   counter as PPS. N is the divider, by default 100, which gives 10 per
   second. The frame number is unwrapped from 11 bits to 64 bits.
5. **Sentences.** On PPS the board sends `$PVDB,SOFPPS,PPS,<tick>,<rx_idle_ms>*hh`,
   and on a tagged SOF it sends `$PVDB,SOFPPS,SOF,<tick>,<frame>*hh`.

## Hardware (Seeed Studio XIAO ESP32-S3 defaults)

| Signal              | XIAO pin | GPIO | Kconfig                    |
|---------------------|----------|------|----------------------------|
| GPS TX ← board      | D6 (TX)  | 43   | `SOFGNSS_GPS_TX_GPIO`       |
| GPS RX → board      | D7 (RX)  | 44   | `SOFGNSS_GPS_RX_GPIO`       |
| PPS in              | D8       | 7    | `SOFGNSS_PPS_GPIO`          |
| Status LED          | on-board | 21   | `SOFGNSS_LED_GPIO`          |
| Debug console TX/RX | D0 / D1  | 1/2  | `ESP_CONSOLE_UART_*_GPIO`  |
| USB                 | USB-C    | 19/20|                            |

The USB-C connector is the CDC device (native OTG PHY), so the ESP-IDF console
moves to UART0 on D0/D1 at 115200. The ROM bootloader still prints a few lines
on GPIO43 at reset, which u-blox receivers ignore.

The timestamp counter is the MCPWM capture timer. It runs at 80 MHz (12.5 ns)
and is 32 bits wide, and the firmware unwraps it to 64 bits. 80 MHz is the
fastest clock on the ESP32-S3 that has a hardware capture input.

The yellow status LED shows what the receiver delivers:

| LED | Meaning |
|---|---|
| off | nothing, so no valid GPS data and no PPS |
| off, 100 ms flash at each PPS | PPS only |
| on | valid GPS frames (baud rate and framing right), no PPS |
| on, 100 ms gap at each PPS | valid GPS frames and PPS |

"Valid" means a NMEA, UBX or RTCM frame with a correct checksum in the last
2 s. Garbage from a wrong baud rate does not light it. The red LED is the
battery-charge indicator and can't be controlled.

## Building and flashing

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
tools/ota.py                    # update a running board, see below
idf.py monitor -p /dev/ttyUSBx  # console is on D0/D1, not on USB
idf.py menuconfig               # "sofgnss board" and "sofgnss application" menus
```

### Updating a running board

In normal usage, sofd will update the board to the configued firmware. 
Alternatively, `tools/ota.py` sends the new image over the CDC port
into the inactive application slot (A/B, see `partitions.csv`). The device
verifies it and restarts into it. The first host command after the restart
confirms the image, and the tool sends `VER` for that. Until the image is
confirmed, a reset boots the previous image again (bootloader rollback).

The tool opens the port exclusively, so stop sofd first. It keeps the port's
current line coding, so the GPS UART is not retuned. After the restart it
finds the board again by its `/dev/serial/by-id` link (USB serial number), and
it fails unless the board comes back confirmed on the new image.

```sh
idf.py build && tools/ota.py            # ~10 s including the restart
```

### First flash or bricked board

This needs the ROM bootloader once. Hold **BOOT** while pressing **RESET**,
then run `idf.py -p /dev/ttyACM0 flash`. The ROM latches download mode, so
press **RESET** again after flashing to start the app.

The two application slots are 1 MB each (0x100000, at 0x10000 and 0x110000).
The image is about 390 KB. Wi-Fi and Bluetooth are not used, so the build
links no binary blobs (`CONFIG_APP_NO_BLOBS`).

### Signing key

Application images are signed with RSA-3072 using the Secure Boot V2 scheme,
but without hardware secure boot. A running board only accepts an update that
is signed with the same key as the image it runs, and answers `ERR,OTAVERIFY`
otherwise. The build signs `build/sofgnss.bin` with
`keys/sofgnss_signing_key.pem`, which is not in git. Create it once, keep a
backup, and use the same key on every machine that builds updates:

```sh
mkdir -p keys && espsecure.py generate_signing_key --version 2 --scheme rsa3072 keys/sofgnss_signing_key.pem
```

With a new or lost key, each board needs the ROM bootloader flash above once.
The bootloader itself doesn't check signatures, because the S3 can't do that
without hardware secure boot. A serial flash can therefore always install any
image. The signature protects the CDC update path, not against someone with
physical access.

### Host tests

These test the portable core (parser, queue, unwrap, sentences, and the SOF
scheduler against a simulated USB bus):

```sh
cmake -S test/host -B build-host && cmake --build build-host && build-host/sofgnss_tests
```

## Protocol

All sentences carry a standard NMEA checksum and end in CR LF. Counter values
are the 64-bit unwrapped tick count, and the low 32 bits equal the hardware
register. The INFO sentence gives the number of ticks per second.

Device to host:

| Sentence | Meaning |
|---|---|
| `$PVDB,SOFPPS,PPS,<tick>,<rx_idle_ms>` | PPS edge captured at `<tick>`. `<rx_idle_ms>` is how long the GPS line had been quiet at the edge, at least. It is 0 if GPS bytes were still on their way (see below). |
| `$PVDB,SOFPPS,SOF,<tick>,<frame>` | SOF of USB frame `<frame>` (64-bit unwrapped) received at `<tick>` |
| `$PVDB,SOFPPS,INFO,<tick_hz>,<divider>,<pps>,<sof_tags>,<sof_missed>,<dropped>,<parse_errors>,<tag>,<pps_rejected>` | Sent with an empty `<tag>` when the host connects and every `SOFGNSS_INFO_INTERVAL_S` seconds (default 1). A requested INFO echoes the request's tag. `<pps_rejected>` counts PPS edges dropped as glitches since boot. It comes after `<tag>` so older parsers keep working. |
| `$PVDB,SOFPPS,VER,<version>,<idf>,<partition>,<pending>,<elf_sha256>,<reset_reason>` | Firmware version, running slot, whether the image still waits for confirmation, the image's ELF SHA-256 and why the device last booted (`POWERON`, `SW`, `PANIC`, `TASK_WDT`, `INT_WDT`, `BROWNOUT`, ...). The SHA-256 is the value in the `.bin`'s app descriptor at offset 32, so a host can tell whether an update is needed. Sent when the host connects and on request. |
| `$PVDB,SOFPPS,USB,<mounted>,<suspended>,<dtr>,<lc_events>,<ls_events>,<baud>` | USB control-path diagnostics, sent with every INFO |
| `$PVDB,SOFPPS,OTA,READY` / `OTA,ACK,<bytes>` / `OTA,OK` | Firmware update progress |
| `$PVDB,SOFPPS,ERR,<code>` | Reply to a malformed command or a failed update (`BADDIV`, `BADTAG`, `BADCAL`, `UNKNOWN`, `OTAARGS`, `OTABUSY`, `OTABEGIN`, `OTAWRITE`, `OTAVERIFY`, `OTATIMEOUT`, `OTALINK`). `OTABUSY` means an `OTA` command came in while an update was running or waiting for its restart, and that update is left alone. `OTALINK` means the update was aborted because the host went away. That happens when USB is unconfigured, reset or suspended, and with `SOFGNSS_REQUIRE_DTR` also when DTR drops. |

Host to device. The host's stream is framed like the receiver's. Only whole,
checksum-valid NMEA, UBX and RTCM3 frames are forwarded to the GPS. Everything
else is dropped, such as NMEA without a checksum, other binary protocols and
plain-text consoles. A `$PVDB,SOFPPS` sentence with a valid checksum is a
command and is not forwarded. Commands are only recognised as NMEA frames, so
the inside of a UBX or RTCM3 frame is never mistaken for one. A command ends at
its `*hh`, and a line ending after it is optional. Frames are written to the
GPS whole. The board waits for UART space rather than cutting one.

| Sentence | Meaning |
|---|---|
| `$PVDB,SOFPPS,DIV,<n>` | Tag one frame in every window of `n` (1…1000000). With `SOFGNSS_SOF_DITHER` (the default) that is `k*n + r`, and without it `frame % n == 0` (see below). Replies with INFO. |
| `$PVDB,SOFPPS,INFO[,<tag>]` | Request an INFO sentence. `<tag>` is 1 to 16 characters from `A-Z a-z 0-9 . _ -` and is echoed in the reply. A host can send `INFO,<nonce>` after opening the port and treat everything after the INFO that echoes it as data produced after the request. That is how to skip the backlog. |
| `$PVDB,SOFPPS,PING` | No reply. It counts as host activity for the host-silence watchdog. |
| `$PVDB,SOFPPS,VER` | Request a VER sentence. This also confirms a freshly installed image. |
| `$PVDB,SOFPPS,SIMPPS` | Test hook that fires the PPS capture in software. It takes the real path through the ISR, sentence, LED and DCD. Within 0.5 s of another edge it is dropped as a glitch. |
| `$PVDB,SOFPPS,OTA,<size>,<sha256hex>` | Start a firmware update. Wait for `OTA,READY`, then send exactly `<size>` raw image bytes. Nothing else may be sent until `OTA,OK` or `ERR`. The transfer aborts after 5 s without data (counted from `READY`, after the slot erase) or when the host goes away. |

Any valid command confirms a newly installed image and so cancels the
rollback.

Example: `$PVDB,SOFPPS,DIV,100*4F`.

Messages are written to USB whole. When the host isn't reading, the outbox
drops the oldest whole messages, never single bytes, so opening the port
delivers an intact backlog followed by live data. `SOFGNSS_REQUIRE_DTR` is on
by default and changes this. Output is then discarded while the host does not
assert DTR, which Linux does whenever the port is open, and whatever is still
queued when DTR drops is cleared. An open then starts with live data, and
nothing arrives before the host has put the port into raw mode. This matters
because a new Linux tty starts with ECHO on and would echo the data back. Host
software must assert DTR, which Linux, macOS and pyserial do by default. Turn
the option off for software that doesn't.

Messages leave in the order they were queued. A GPS frame is queued when its
last byte has been parsed, and a PPS sentence when the edge is handled,
microseconds later. So everything the GPS sent before the edge comes before
the `PPS` sentence, unless it was still on its way through the receive path at
the edge. It could be on the wire, in the UART FIFO or driver buffer, or in a
frame that is still being assembled.

`<rx_idle_ms>` tells you which case applies. It is 0 when any GPS byte was
still in that path when the PPS was handled. A byte that moved between stages
in the meantime may count twice, but none are missed. Otherwise it is the time
from the last hand-over of GPS bytes by the UART driver to the edge. That is a
lower bound on how long the line had been quiet, short by at most about 2
character times, because the driver hands bytes over after 64 bytes or 2
characters of silence. With no GPS data since boot it is the uptime.

A host that pairs messages with pulses by order, such as a UBX-TIM-TP with
the pulse it announces, can use a pulse when `<rx_idle_ms>` is at least about
10 for it and for the previous pulse. A receiver that is still working through
a backlog sends back to back, so a quiet line also means the receiver had
nothing queued. The field can't see lost messages or lost pulses, but two
cheap checks cover those. There should be exactly one TIM-TP since the
previous `PPS`, and 1 s of ticks to the previous `PPS`. Pairing TIM-TP by its
`towMS` avoids the question entirely.

A UBX frame larger than `SOFGNSS_PARSER_BYTES` is skipped by its declared
length and counted in `parse_errors`. The default of 4096 is enough for a
UBX-RXM-RAWX with 127 measurements, and RTCM3 frames are at most 1029 bytes by
format. The skipped payload is not searched for frame starts, because it could
fake them. A GPS frame that stops halfway, for example after a corrupted UBX
or RTCM length byte, is ended after `SOFGNSS_FRAME_TIMEOUT_MS` (200 ms)
without data. The same goes for a skip. This cleans it up in the gap after the
receiver's burst. A frame or skip is also ended when it isn't complete 1.25
times its wire time at the current baud (by its declared length) plus
`SOFGNSS_FRAME_SLACK_MS` (100 ms) after its first byte. Receivers send a
message back to back, so only a broken frame takes longer. An example is a
false frame start with a large length on a line that isn't saturated, which
would otherwise swallow the frames behind it. On a saturated line such a frame
completes on time and its checksum rejects it. Both cases count in
`parse_errors`. The host's frames to the GPS are limited to 2048 bytes.

Which frames get tagged is decided on the unwrapped frame number, one frame in
every window of N frames (N = divider). With `SOFGNSS_SOF_DITHER` (on by
default) that frame is `k*N + r`, where `r` is pseudo-random in `[0, N/2)` and
a fixed function of the window `k`. This keeps the capture quantization error
from forming a slow sawtooth across samples. With dither off it is `k*N`. The
sentence always carries the frame number, so hosts don't need to know the
pattern.

The spin polls the frame-number register about every 38 CPU cycles (13 ticks)
and checks its deadline on the CPU cycle counter. Every poll interval is
therefore the same length and the stamp error is uniform within one interval.
Measured against a local linear fit on an Intel xHCI root port, the error is
3.7 ticks (46 ns) RMS, with under 1 % of stamps up to 12 ticks late. With
diagnostics on, those late stamps had normal poll and capture timing, so the
frame number itself changed late in the host or USB core, and the capture was
fine. `SOFGNSS_SOF_DEBUG` (off by default) adds a `$PVDB,SOFPPS,SOFDBG`
sentence after each SOF with the spin's cycle counts, for looking into this.

## SDR calibration output

This is for measuring an SDR's timing and frequency against the same clock as
PPS and SOF. `$PVDB,SOFPPS,CAL,<mode>*hh` starts one 6.55 ms burst per PPS on
**D3 (GPIO4)**. Without PPS it sends one per second on its own clock.

| Mode | Signal | Use |
|---|---|---|
| `NOISE` | Degree-18 maximal-length sequence, 262143 chips at 40 Mchip/s plus one 0 pad bit | Timing. It correlates to a single sharp peak with about 54 dB processing gain. |
| `SQ2` | 2 MHz square wave (SPI, 10 zero bits and 10 one bits) | Frequency near the 160 MHz null. Odd harmonics sit every 4 MHz, for example at 158 and 162 MHz. The 162 MHz line is about 38 dB below the fundamental, so use about 20 dB less attenuation than for `SQ20`. |
| `SQ20` | 20 MHz square wave (SPI, `0xAA`) | Frequency, with odd harmonics at 20, 60, 100 … MHz |
| `SQ40` | 40 MHz square wave (LEDC) | Frequency, with odd harmonics at 40, 120, 200 … MHz |
| `OFF` | | Default after boot |

Every frequency comes from the same crystal and PLL as the 80 MHz counter, and
PPS measures that counter against GPS, so the transmitted frequencies are
known to ppb level. Every signal changes level only on a 25 ns (SPI) or
12.5 ns (LEDC) grid. All of them therefore have a spectral null at every
multiple of 40 MHz, including 160 MHz. `SQ2` is the way to get a line close to
such a frequency, and `NOISE` still correlates there (V-shaped spectrum). Each
burst start is captured on the counter and reported:

```
$PVDB,SOFPPS,CAL,<tick>,<mode>,<pps_synced>,<n>*hh
```

The burst starts a few microseconds after the PPS edge. The exact difference
is in the `PPS` and `CAL` ticks rather than assumed. For `NOISE`, `SQ2` and
`SQ20` the tick is the SPI chip-select edge, exact to one tick. For `SQ40` it
is a GPIO edge written together with the LEDC start, within about 50 ns.
**D4 (GPIO5)** carries that start marker and is low during the burst. It needs
no wiring and is handy as a scope trigger. A spectrum analyser in max-hold
shows the harmonic comb.

The sequence, for correlators, is sent MSB first from a right-shift Fibonacci
LFSR starting at `state = 0x3FFFF`. Per chip it computes `out = state & 1`,
`fb = (state ^ (state >> 7)) & 1` and `state = (state >> 1) | (fb << 17)`.

The GPIO is far too strong for the SDR's input, so connect it like this:
GPIO4 → 1 kΩ → 100 nF → 50 Ω to ground → 30–40 dB of attenuators → coax.
The noise spectrum has nulls at multiples of 40 MHz, about −16 dB at 2 MHz
away and −10 dB at 4 MHz. Prefer tunings a few MHz away from them.

`tools/sdrcal.py` is a reference analysis in Python. It needs system Python
with numpy, scipy and pyserial, and `--help` lists its options. `log` records
the sentences with host timestamps while bursts run. `tone` measures the SDR's
frequency error from a square-wave recording. `noise` correlates the NOISE
bursts and fits the SDR sample index against device ticks, which gives the
sample-rate error (the LO error for an SDR with one crystal), the timing
residuals and the measured PPS-to-burst offset. `selftest` checks both on
synthetic recordings with known errors.

A note for correlators: tuned onto a spectral null, for example exactly
160 MHz, the correlation has sidelobes about ±1.25 samples from the peak at
60 % of its height. That is higher than the integer-sample points of a peak
lying between samples. Find the peak on an upsampled correlation (the tool
uses 8x) before refining.

## Watchdogs

- The hardware task watchdog is fed by every firmware task. A task that
  stalls for 10 s, a panic, or interrupts left disabled will reboot the
  device.
- The host-silence watchdog (`SOFGNSS_HOST_WDT`, on by default) restarts the
  device and makes it re-enumerate if no data arrives from the host for 60 s.
  Control requests such as a baud change don't count as data, and time with
  the bus suspended doesn't count as silence. This recovers a wedged USB
  path. It also means host software must send something at least once a
  minute, and `$PVDB,SOFPPS,PING*hh` is meant for that. A purely passive
  reader would see the device disappear every minute, so build with the
  option off for that use.

Every boot, and every deliberate restart before it happens, first takes the
device off the bus for 300 ms by turning off the D+ pull-up. A software reset
keeps the USB pads as they were, and a hub could then miss the restart. The
host kept talking to the old address and got STALLs, which on a Raspberry Pi
drives cdc_acm's stall recovery into a kernel livelock. Shorter detaches
turned out to be unreliable behind hubs.

With `SOFGNSS_REQUIRE_DTR` (the default), output while the port is closed is
discarded, so an open starts with live data. Without it the outbox keeps the
most recent ~12 KB of messages and delivers them intact on the next open. A
host that only wants live data then sends `INFO,<nonce>` right after opening
and discards everything up to the `INFO` that echoes the nonce.

`VER` reports the reason for the last boot.

## PPS on the DCD line

Each PPS edge also raises the CDC carrier-detect line for
`SOFGNSS_DCD_PULSE_MS` (100 ms), which is the convention gpsd and ntpd use for
serial receivers. On Linux the change is visible through the usual modem-line
calls. `TIOCMIWAIT` wakes a waiter about 1-2 ms after the edge, because the
interrupt endpoint is polled every 1 ms, and `TIOCMGET` reads the level.
gpsd's PPS thread uses exactly this when there is no kernel PPS device.

The kernel PPS line discipline (`ldattach PPS`, `/dev/ppsN`) does **not** work
with CDC-ACM devices. The cdc-acm driver records DCD changes but never calls
the line discipline's `dcd_change`. This was verified on Linux 5.10, and
usb-serial adapters such as FTDI do call it. So this path only gives
millisecond-class timestamps in userspace. The exact edge time is in the
`$PVDB,SOFPPS,PPS` sentence on the same port. Without a receiver,
`$PVDB,SOFPPS,SIMPPS*hh` fires the whole path once.

## How the SOF timestamp is taken

The ESP32-S3 has no hardware path from the USB core to a timer. The SOF
timestamp is therefore taken in software, with the jitter of a register poll
rather than of an interrupt:

1. From the last tagged SOF and the measured frame period, predict when the
   next target frame arrives.
2. Sleep until `SOFGNSS_SOF_LEAD_US` (default 200 µs) before it, using a
   one-shot `esp_timer` with ISR dispatch.
3. On core 1, with interrupts masked, spin reading the DWC2 `DSTS.SOFFN`
   register. The moment it changes, trigger a software capture of the timer
   and read it. Detection is as fine as one register read (~100 ns), which is
   at the noise floor of a full-speed SOF anyway.
4. Report the stamp only if the caught frame is the target. Otherwise count it
   as missed and plan again. The spin gives up after 3 ms so a suspended bus
   can't hang the task.

Everything else (TinyUSB, UART, parser, PPS interrupt) runs on core 0.

## Layout

```
components/core/         portable, no platform includes (C++17)
  hal.h                  the interface a port implements
  frame_parser.*         NMEA / UBX / RTCM3 framer
  msg_queue.*            drop-oldest message ring
  unwrap.h               32→64-bit tick and 11→64-bit frame unwrapping
  pvdb.*                 $PVDB sentence formatting, checksum, command recognition
  timing_reporter.*      PPS/SOF events → sentences
  sof_spin_source.*      predict / sleep / spin SOF tagging (ports without HW SOF capture)
components/port_esp32s3/ HAL implementation: TinyUSB glue, MCPWM capture, UART, esp_timer
main/app_main.cpp        task wiring (FreeRTOS)
test/host/               host build of core with a simulated HAL
```

