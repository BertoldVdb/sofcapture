# sofgnss on the ESP32-S3

The Seeed Studio XIAO ESP32-S3 is a cheaper alternative to the default
[STM32G474 board](../stm32g4/README.md). It has no hardware path from the USB
core to a timer, so SOF timestamps are taken in software (see
[below](#how-the-sof-timestamp-is-taken)) with about three times the noise of
the STM32. The protocol, the GPS framing and everything else both boards share
are in the [sofgnss README](../README.md).

The firmware is an ESP-IDF 5.5 project.

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

Wire the receiver's TX to D7 (GPIO44), its RX to D6 (GPIO43), its PPS or time
pulse to D8 (GPIO7), and GND and 3.3 V to GND and 3V3. The pins are
changeable in `idf.py menuconfig`. The yellow LED shows what the receiver
delivers (see [Status LED](../README.md#status-led)). The red LED is the battery-charge indicator and can't be controlled.

## How the SOF timestamp is taken

The SOF timestamp is taken in software, with the jitter of a register poll
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

The spin polls the frame-number register about every 38 CPU cycles (13 ticks)
and checks its deadline on the CPU cycle counter. Every poll interval is
therefore the same length and the stamp error is uniform within one interval.
Measured against a local linear fit on an Intel xHCI root port, the error is
3.7 ticks (46 ns) RMS, with under 1 % of stamps up to 12 ticks late. With
diagnostics on, those late stamps had normal poll and capture timing, so the
frame number itself changed late in the host or USB core, and the capture was
fine. `SOFGNSS_SOF_DEBUG` (off by default) adds a `$PVDB,SOFPPS,SOFDBG`
sentence after each SOF with the spin's cycle counts, for looking into this.

## Building and flashing

Create the [signing key](#signing-key) first. Then, in `sofgnss/`:

```sh
cd esp32s3
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build                    # esp32s3/build/sofgnss.bin
../tools/ota.py                 # update a running board, see below
idf.py monitor -p /dev/ttyUSBx  # console is on D0/D1, not on USB
idf.py menuconfig               # "sofgnss board" and "sofgnss application" menus
```

## Updating a running board

In normal use sofd updates the board to the configured firmware.
Alternatively, `tools/ota.py` sends the new image over the CDC port into the
inactive application slot (A/B, see `partitions.csv`). The device verifies it
and restarts into it. The first host command after the restart confirms the
image, and the tool sends `VER` for that. Until the image is confirmed, a
reset boots the previous image again (bootloader rollback).

The tool opens the port exclusively, so stop sofd first. It keeps the port's
current line coding, so the GPS UART is not retuned. After the restart it
finds the board again by its `/dev/serial/by-id` link (USB serial number), and
it fails unless the board comes back confirmed on the new image.

```sh
idf.py build && ../tools/ota.py         # in esp32s3/, ~10 s including the restart
```

## First flash or bricked board

This needs the ROM bootloader once. Hold **BOOT** while pressing **RESET**,
then run `idf.py -p /dev/ttyACM0 flash`. The ROM latches download mode, so
press **RESET** again after flashing to start the app.

The two application slots are 1 MB each (0x100000, at 0x10000 and 0x110000).
The image is about 390 KB. Wi-Fi and Bluetooth are not used, so the build
links no binary blobs (`CONFIG_APP_NO_BLOBS`).

## Signing key

Application images are signed with RSA-3072 using the Secure Boot V2 scheme,
but without hardware secure boot. A running board only accepts an update that
is signed with the same key as the image it runs, and answers `ERR,OTAVERIFY`
otherwise. The build signs `esp32s3/build/sofgnss.bin` with
`keys/sofgnss_signing_key.pem` (in `sofgnss/`, next to `esp32s3/`), which is
not in git. Create it once, keep a backup, and use the same key on every
machine that builds updates:

```sh
mkdir -p keys && espsecure.py generate_signing_key --version 2 --scheme rsa3072 keys/sofgnss_signing_key.pem
```

With a new or lost key, each board needs the ROM bootloader flash above once.
The bootloader itself doesn't check signatures, because the S3 can't do that
without hardware secure boot. A serial flash can therefore always install any
image. The signature protects the CDC update path, not against someone with
physical access.

## SDR calibration output

The modes and the sentence are described under
[SDR calibration output](../README.md#sdr-calibration-output).

| Signal | Pin |
|---|---|
| Burst output | D3 (GPIO4) |
| Start marker, low during the burst | D4 (GPIO5) |

`NOISE`, `SQ2` and `SQ20` go out over SPI at 40 Mchip/s, `SQ40` from LEDC.
Every signal changes level on a 25 ns (SPI) or 12.5 ns (LEDC) grid, so all
have nulls at multiples of 40 MHz, 160 MHz included. With `SQ2` the 162 MHz
line is about 38 dB below the fundamental, so use about 20 dB less attenuation
than for `SQ20`.

The burst starts about 0.1 ms after the PPS edge (measured 40 to 120 µs). The
exact difference is in the `PPS` and `CAL` ticks rather than assumed. For
`NOISE`, `SQ2` and `SQ20` the tick is the SPI chip-select edge, exact to one
tick. For `SQ40` it is a GPIO edge written together with the LEDC start,
within about 50 ns. The start marker needs no wiring and is handy as a scope
trigger.

Both chip-select edges are captured in hardware: the SPI pulls it low before
the first bit and releases it right after the last, so the length is exact to
a tick. A gapless burst is 524288 plus 3 ticks of chip-select setup and hold.
Measured: every `NOISE` and `SQ20` burst 524291. `SQ40` reports 0, because
software ends it.
