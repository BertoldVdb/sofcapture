# sofgnss on the STM32G474

This is the default sofgnss board. It captures every USB SOF in hardware.
The protocol, the GPS framing and everything else both boards share are in
the [sofgnss README](../README.md), and the
[ESP32-S3 page](../esp32s3/README.md) covers the cheaper alternative.

The firmware is a bare-metal main loop, and interrupts only take timestamps
and move bytes. It is sized for the smallest part (G474xB: 128 KB flash,
96 KB of SRAM used). The USB serial number is the 96-bit chip UID (24 hex
digits). Commands below run in `sofgnss/`.

## Hardware (WeAct STM32G474 core board)

The board has an STM32G474CEU6, an 8 MHz crystal and USB-C. Its headers are
labelled with the pin names. The pins are set in `stm32g4/include/port.h`, and
the GPS UART is on PB10/PB11, which every G474 package has:

| Signal              | Pin  | Function    |
|---------------------|------|-------------|
| GPS TX ← board      | PB10 | USART3_TX   |
| GPS RX → board      | PB11 | USART3_RX   |
| PPS in              | PA1  | TIM2_CH2, rising edge |
| Status LED          | PC6  | the board's LED |
| Debug console TX    | PA2  | LPUART1, 115200, to a 3.3 V USB serial adapter. Off by default (`CONFIG_SOFGNSS_CONSOLE` in `include/config.h`). |
| USB D− / D+         | PA11 / PA12 | USB-C |

Wire the receiver's TX to PB11, its RX to PB10, its PPS or time pulse to PA1,
and GND and 3.3 V to GND and 3V3. The LED on PC6 shows what the receiver
delivers (see [Status LED](../README.md#status-led)).

## How the SOF timestamp is taken

The SOF timestamp is taken in hardware. TIM2 is the 32-bit timestamp counter
at the 144 MHz core clock (6.9 ns, wraps every 30 s). Channel 1 captures
internal trigger 11, the USB SOF (only TIM2 has it), and channel 2 captures
PPS from a pin. The TIM2 interrupt reads `USB_FNR` next to each capture and
drops the pair if another SOF arrived meanwhile (counted in INFO's
`<sof_missed>`). Every SOF is captured, and the divider and dither then pick
the tagged frames. Against a local linear fit the stamps scatter by 2.0 ticks
(14 ns) rms.

The core clock comes from HSE through the PLL. At boot the firmware measures
HSE against HSI16 (HSE/32 into TIM17) and accepts an 8, 12 or 24 MHz external
clock or crystal. It tries bypass mode first, because a crystal gives no clock
there while an external clock would also run in crystal mode, only with more
jitter. A mode that starts but measures wrong is rejected. The console shows
the measurement and the choice.

USB runs from the same PLL (288 MHz / 6), which is why the core clock is 144
MHz and not the maximum 170 MHz. With USB on HSI48 the SOF trigger crosses
clock domains, and TIM2 then saw only 80 to 86 % of the SOFs, plus some
captures flagged without a new value. On the shared clock it gets all of them.
Without a usable HSE the core runs from HSI16 (about 1 % off) and USB from
HSI48, trimmed to the host's SOF by the CRS, so SOF captures are unreliable.

## Building and flashing

You need arm-none-eabi-gcc, CMake, Python 3 with `cryptography`, and
dfu-util or OpenOCD for the first flash. Create the signing key first (see
[Signing key](#signing-key)), then:

```sh
cmake -S stm32g4 -B stm32g4/build          # fetches TinyUSB, CMSIS and micro-ecc
cmake --build stm32g4/build                # bootloader, app and the images below
cmake --build stm32g4/build --target dfu   # everything over USB (dfu-util)
cmake --build stm32g4/build --target flash # or over ST-LINK (OpenOCD)
tools/ota.py stm32g4/build/sofgnss-signed.bin   # app update over the CDC port
```

The build writes `sofgnss-signed.bin`, the app with its signed header, which
is what OTA sends, and `sofgnss-full.bin`, bootloader, header and app for an
empty chip. For DFU, hold BOOT0, press and release NRST, then release BOOT0.
The chip's ROM bootloader then shows up as `0483:df11`. After the download it
jumps to our bootloader, which resets itself to start clean.

After a flash the board waits in its bootloader until a host starts the app.
sofd does that by itself, also without `firmware` set.

## Updating a running board

In normal use sofd updates the board to the configured firmware.
`tools/ota.py stm32g4/build/sofgnss-signed.bin` does the same by hand. It
sends `BOOT` to a running app, the image to the bootloader, then `START`.

The tool opens the port exclusively, so stop sofd first. It keeps the port's
current line coding, so the GPS UART is not retuned. After each restart it
finds the board again by its `/dev/serial/by-id` link (USB serial number), and
it fails unless the board comes back running the new image.

## Bootloader

Flash layout: the bootloader in the first 24 KB, the image header in the next
4 KB, the app from 0x08007000. The bootloader uses only CCM SRAM, so the app's
panic flag survives, and it leaves the reset flags for the app to read.

The bootloader never starts the app by itself, because the board is of no use
without a host. It enumerates with the same USB identity and answers `INFO`,
`VER`, `PING`, `OTA` and `START`. Its VER reports platform `stm32g4-boot`, the
installed app's version and ELF SHA-256, and in the partition field the app's
state: `valid`, `invalid` or `empty`. The host then sends `START`, or `OTA`
with a new image followed by `START`. A running app is sent `BOOT` first.
sofd and `tools/ota.py` do all of this.

`START` checks the app's SHA-256 and signature every time (about 1 s), then
puts USB, clocks and interrupts back to reset state and jumps to the app. An
update erases the header first, writes the app, checks it and writes the
header last, so an interrupted or bad update leaves the state `empty`, and the
bootloader stays reachable for the next try.

The watchdog starts at the bootloader's first instruction, and the bootloader
also restarts after 60 s without host data, in case USB came up wrong. The
bootloader itself is only written over DFU or SWD.

## Signing key

Images are signed with ECDSA P-256 (micro-ecc). The key is
`keys/stm32_signing_key.pem` (in `sofgnss/`), not in git. Create it once, keep
a backup, and use the same key on every machine that builds updates:

```sh
mkdir -p keys && stm32g4/sign.py keygen keys/stm32_signing_key.pem
```

The bootloader build embeds its public key, so boards only take images signed
with the key of the bootloader they run. With a new or lost key, each board
needs a DFU or SWD flash once.

## SDR calibration output

The modes and the sentence are described under
[SDR calibration output](../README.md#sdr-calibration-output).

| Signal | Pin |
|---|---|
| Burst output | PA7 (SPI1_MOSI) |
| Start marker, low during the burst | PA4 (SPI1_NSS) |
| Start capture (TIM2_CH3) | PA9 |

The patterns go out over SPI1 at 72 Mchip/s, the 144 MHz bus clock divided by
2. 40 Mchip/s isn't reachable while USB shares the PLL, so 160 MHz is clear of
the nulls here.

Wire PA4 to PA9. The marker falls as the SPI starts with its FIFO already
filled by DMA, and TIM2 captures that edge, so the `CAL` tick is exact to one
tick. Without the wire the bursts still go out, but no `CAL` sentences come.
A burst starts from the PPS interrupt, normally about 7 µs after the edge.

DMA feeds the SPI from a 32 KB buffer in CCM SRAM. USB register and packet
memory accesses are slow and share the peripheral bus with that DMA. While the
host streamed every SOF they stalled the stream by up to 30 chips per burst.
So USB waits out each 3.6 ms burst: its interrupt is masked (the host gets
NAKs) and the main loop does no USB work, DCD changes included. A PPS that
arrives while USB work is running starts its burst right after that work
instead, tens of microseconds later.

Each burst's length is measured from the captured start edge to the moment
the SPI reports its last bit gone (FIFO empty, not busy), read in the
burst-end interrupt. A gapless burst is 524288 plus a fixed 15 ticks, good to
about a tick. Measured with every SOF streamed to the host: 62 of 63 bursts
exactly 524303, one 524305. The firmware also logs bursts more than 64 ticks
too long on the console.
