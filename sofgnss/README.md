# sofgnss

sofgnss is a USB GPS bridge with hardware timestamps for PPS and USB SOF. It
shows up on the host as a CDC-ACM serial port connected to a GPS receiver.
Besides passing the GPS data through, it timestamps the receiver's PPS edge
and the USB start-of-frame packets it gets from the host, both on one
free-running hardware counter. Both timestamps go into the serial stream as
`$PVDB,SOFPPS,...` sentences, so a host that reads the GPS port sees GPS time,
PPS and its own USB frame clock side by side.

The firmware runs on two boards that share the platform-independent core and
speak the same protocol:

- the **STM32G474** (bare metal, TinyUSB), the default board:
  [wiring, building, flashing and details](stm32g4/README.md)
- the **ESP32-S3** (ESP-IDF 5.5, TinyUSB), alternatively, with software SOF
  timestamps: [wiring, building, flashing and details](esp32s3/README.md)

On the host, [sofd](../sofd) uses the board as the GNSS reference for its USB
controller. The [top-level README](../README.md#setting-up-a-system) explains
how to set up the whole system.

## Choosing a board

| | STM32G474 (default) | ESP32-S3 |
|---|---|---|
| Board | WeAct STM32G474 core board (STM32G474CEU6), any G474 works | Seeed Studio XIAO ESP32-S3 |
| SOF timestamp | Hardware capture of every SOF | Software, a CPU core spins on the frame-number register for each tagged SOF |
| Counter | 144 MHz (6.9 ns) | 80 MHz (12.5 ns) |
| SOF stamp noise | 2.0 ticks (14 ns) rms | 3.7 ticks (46 ns) rms, under 1 % of stamps up to 12 ticks (150 ns) late |
| `sof_delay_ns` in sofd | 1300 ns built in, measured | 0, calibrate it yourself |
| Update signature | ECDSA P-256, checked by a resident bootloader at every start | RSA-3072, checked by the running app. A serial flash through the ROM bootloader takes any image. |
| First flash | USB DFU (BOOT0) or ST-LINK | USB ROM bootloader (BOOT button) |
| Toolchain | arm-none-eabi-gcc, CMake, Python 3 with `cryptography` | ESP-IDF 5.5 |
| SDR calibration output | 72 Mchip/s, no `SQ40` | 40 Mchip/s, all modes |

sofd treats both boards alike. Use the ESP32-S3 when cost matters more than
the last few tens of nanoseconds.

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

## Status LED

The status LED shows what the receiver delivers:

| LED | Meaning |
|---|---|
| off | nothing, so no valid GPS data and no PPS |
| off, 100 ms flash at each PPS | PPS only |
| on | valid GPS frames (baud rate and framing right), no PPS |
| on, 100 ms gap at each PPS | valid GPS frames and PPS |

"Valid" means a NMEA, UBX or RTCM frame with a correct checksum in the last
2 s. Garbage from a wrong baud rate does not light it.

## Host tests

These test the portable core (parser, queue, unwrap, sentences, and the SOF
scheduler against a simulated USB bus):

```sh
cmake -S test/host -B test/host/build && cmake --build test/host/build && test/host/build/sofgnss_tests
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
| `$PVDB,SOFPPS,VER,<version>,<platform>,<partition>,<pending>,<elf_sha256>,<reset_reason>` | Firmware version, platform (`stm32g4` on the STM32, `stm32g4-boot` from its bootloader, see [Bootloader](stm32g4/README.md#bootloader), the ESP-IDF version on the ESP32-S3), running slot, whether the image still waits for confirmation, the image's ELF SHA-256 and why the device last booted. The STM32 reports `POWERON`, `EXT`, `SW`, `PANIC` (a fault handler) or `WDT`, the ESP32-S3 `POWERON`, `SW`, `PANIC`, `TASK_WDT`, `INT_WDT`, `BROWNOUT` and so on. The SHA-256 is the value in the image's app descriptor, so a host can tell whether an update is needed. Sent when the host connects and on request. |
| `$PVDB,SOFPPS,USB,<mounted>,<suspended>,<dtr>,<lc_events>,<ls_events>,<baud>` | USB control-path diagnostics, sent with every INFO |
| `$PVDB,SOFPPS,OTA,READY` / `OTA,ACK,<bytes>` / `OTA,OK` | Firmware update progress |
| `$PVDB,SOFPPS,ERR,<code>` | Reply to a malformed command or a failed update (`BADDIV`, `BADTAG`, `BADCAL`, `NOCAL` (a mode the board can't send: `SQ40` on the STM32), `UNKNOWN`, `OTAARGS`, `OTABUSY`, `OTABEGIN`, `OTAWRITE`, `OTAVERIFY`, `OTATIMEOUT`, `OTALINK`, and on the STM32 `OTABOOT`, `NOAPP`, `BADAPP`). `OTABUSY` means an `OTA` command came in while an update was running or waiting for its restart, and that update is left alone. `OTALINK` means the update was aborted because the host went away. That happens when USB is unconfigured, reset or suspended, and with `SOFGNSS_REQUIRE_DTR` also when DTR drops. |

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
| `$PVDB,SOFPPS,OTA,<size>,<sha256hex>` | Start a firmware update. Wait for `OTA,READY`, then send exactly `<size>` raw image bytes. Nothing else may be sent until `OTA,OK` or `ERR`. The transfer aborts after 5 s without data (counted from `READY`, after the slot erase) or when the host goes away. On the STM32 only the bootloader takes it, the app answers `ERR,OTABOOT`. |
| `$PVDB,SOFPPS,BOOT` | STM32 only: restart into the bootloader. |
| `$PVDB,SOFPPS,START` | STM32 bootloader only: check the installed app and start it, or answer `ERR,NOAPP` (none installed) or `ERR,BADAPP` (hash or signature wrong). |

Any valid command confirms a newly installed ESP32-S3 image and so cancels the
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
still in that path when the PPS was handled. Otherwise it is a lower bound on
how long the line had been quiet. On the STM32 it is the time since the UART
last detected an idle line, one character time after the last byte. On the
ESP32-S3 it is the time from the last hand-over of GPS bytes by the UART
driver to the edge. That is short by at most about 2 character times, because
the driver hands bytes over after 64 bytes or 2 characters of silence, and a
byte that moved between stages in the meantime may count twice, but none are
missed. With no GPS data since boot it is the uptime.

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

## SDR calibration output

This is for measuring an SDR's timing and frequency against the same clock as
PPS and SOF. `$PVDB,SOFPPS,CAL,<mode>*hh` starts one burst per PPS on the
board's burst pin. Without PPS it sends one per second on its own clock.

| Mode | Signal | STM32G474 (72 Mchip/s) | ESP32-S3 (40 Mchip/s) |
|---|---|---|---|
| `NOISE` | Timing. Degree-18 maximal-length sequence, 262143 chips plus one 0 pad bit. It correlates to a single sharp peak with about 54 dB processing gain. | 3.64 ms, nulls at multiples of 72 MHz (144, 216 MHz) | 6.55 ms, nulls at multiples of 40 MHz (160 MHz) |
| `SQ2` | Frequency near a null. Square wave of 10 zero and 10 one chips. | 3.6 MHz, odd harmonics every 7.2 MHz (154.8, 162 MHz) | 2 MHz, odd harmonics every 4 MHz (158, 162 MHz) |
| `SQ20` | Frequency. Square wave, `0xAA`. | 36 MHz, odd harmonics at 36, 108, 180 … MHz | 20 MHz, odd harmonics at 20, 60, 100 … MHz |
| `SQ40` | Frequency. Square wave from LEDC. | not available, `ERR,NOCAL` | 40 MHz, odd harmonics at 40, 120, 200 … MHz |
| `OFF` | | Default after boot | Default after boot |

Every frequency comes from the same crystal and PLL as the counter, and PPS
measures that counter against GPS, so the transmitted frequencies are known to
ppb level. Every signal changes level only on the chip grid, so all of them
have a spectral null at every multiple of the chip rate. `SQ2` is the way to
get a line close to such a frequency, and `NOISE` still correlates there
(V-shaped spectrum). Next to a null the noise spectrum is about 16 dB down
5 % of the chip rate away and 10 dB down at 10 % (2 and 4 MHz on the
ESP32-S3, 3.6 and 7.2 MHz on the STM32). Prefer tunings a few MHz away from
them. A spectrum analyser in max-hold shows the harmonic comb.

Each burst start is captured on the counter and reported once the burst has
ended:

```
$PVDB,SOFPPS,CAL,<tick>,<mode>,<pps_synced>,<n>,<chip_rate>,<length>*hh
```

`<chip_rate>` is the SPI rate in chips per second. `<length>` is the burst's
measured length in ticks, or 0 when it wasn't measured. Older firmware leaves
both out. On both boards a gapless burst is 2 ticks per chip (524288) plus a
few ticks of setup. A stall in the stream would make it longer, so tools drop
bursts longer than the others.

The sequence, for correlators, is sent MSB first from a right-shift Fibonacci
LFSR starting at `state = 0x3FFFF`. Per chip it computes `out = state & 1`,
`fb = (state ^ (state >> 7)) & 1` and `state = (state >> 1) | (fb << 17)`.

The pin is far too strong for the SDR's input, so connect it like this:
burst pin → 1 kΩ → 100 nF → 50 Ω to ground → 30 to 40 dB of attenuators →
coax.

The pins and the board-specific timing are on the board pages:
[STM32G474](stm32g4/README.md#sdr-calibration-output) (PA7, and a wire from
PA4 to PA9 for the `CAL` ticks) and
[ESP32-S3](esp32s3/README.md#sdr-calibration-output) (D3).

### Analysis

`tools/sdrcal.py` is a reference analysis in Python. It needs system Python
with numpy, scipy and pyserial, and `--help` lists its options. `log` records
the sentences with host timestamps while bursts run. `tone` measures the SDR's
frequency error from a square-wave recording. `noise` correlates the NOISE
bursts and fits the SDR sample index against device ticks, which gives the
sample-rate error (the LO error for an SDR with one crystal), the timing
residuals and the measured PPS-to-burst offset. `selftest` checks both on
synthetic recordings with known errors.

It takes the tick rate from INFO and the chip rate from CAL in the log, so it
handles both boards. It skips NOISE bursts longer than the median length plus
one tick.

A note for correlators: tuned onto a spectral null, for example exactly
160 MHz on the ESP32-S3, the correlation has sidelobes about ±1.25 samples
from the peak at 60 % of its height. That is higher than the integer-sample
points of a peak lying between samples. Find the peak on an upsampled
correlation (the tool uses 8x) before refining.

## Watchdogs

- A hardware watchdog reboots a stalled device. On the STM32 it is the
  independent watchdog (about 4 s), fed by the main loop. On the ESP32-S3 it
  is the task watchdog, fed by every firmware task. A task that stalls for
  10 s, a panic, or interrupts left disabled will reboot the device.
- The host-silence watchdog restarts the device and makes it re-enumerate if
  no data arrives from the host for 60 s. Control requests such as a baud
  change don't count as data. This recovers a wedged USB path. It also means
  host software must send something at least once a minute, and
  `$PVDB,SOFPPS,PING*hh` is meant for that. On the STM32 it is always on,
  also while the bus is suspended. On the ESP32-S3 it is `SOFGNSS_HOST_WDT`
  (on by default), and time with the bus suspended doesn't count as silence.
  A purely passive reader would see the device disappear every minute, so
  build the ESP32-S3 with the option off for that use.

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
calls. `TIOCMIWAIT` wakes a waiter about 1 to 2 ms after the edge, because the
interrupt endpoint is polled every 1 ms, and `TIOCMGET` reads the level.
gpsd's PPS thread uses exactly this when there is no kernel PPS device.

The kernel PPS line discipline (`ldattach PPS`, `/dev/ppsN`) does **not** work
with CDC-ACM devices. The cdc-acm driver records DCD changes but never calls
the line discipline's `dcd_change`. This was verified on Linux 5.10, and
usb-serial adapters such as FTDI do call it. So this path only gives
millisecond-class timestamps in userspace. The exact edge time is in the
`$PVDB,SOFPPS,PPS` sentence on the same port. Without a receiver,
`$PVDB,SOFPPS,SIMPPS*hh` fires the whole path once.

## Layout

```
core/                    portable, no platform includes (C++17), shared by both ports
  include/sofgnss/hal.h  the interface a port implements
  frame_parser.*         NMEA / UBX / RTCM3 framer
  msg_queue.*            drop-oldest message ring
  unwrap.h               32→64-bit tick and 11→64-bit frame unwrapping
  pvdb.*                 $PVDB sentence formatting, checksum, command recognition
  timing_reporter.*      PPS/SOF events → sentences
  cal_signal.*           calibration burst patterns
  sof_spin_source.*      predict / sleep / spin SOF tagging (ports without HW SOF capture)
stm32g4/                 STM32G474, bare-metal CMake project
  README.md              wiring, building, flashing, bootloader
  boot/                  bootloader: USB, OTA into flash, signature check
  sign.py                signing key, signed and full images
  src/main.cpp           main loop
  src/*.cpp              HAL: clocks, TIM2 capture, UART, USB, calibration
  include/               pins and settings (port.h, config.h), TinyUSB config
esp32s3/                 ESP32-S3, ESP-IDF project (idf.py)
  README.md              wiring, building, flashing
  main/app_main.cpp      task wiring (FreeRTOS), Kconfig menus
  components/port_esp32s3/  HAL: TinyUSB glue, MCPWM capture, UART, OTA, calibration
test/host/               host build of core with a simulated HAL
tools/                   ota.py (updates for both boards), sdrcal.py (calibration analysis)
keys/                    signing keys for both boards, not in git
```
