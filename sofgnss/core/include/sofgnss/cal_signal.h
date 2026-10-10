// Calibration bursts for an SDR connected by coax. A burst is kCalBits bits
// sent over SPI at the port's chip rate (hal::cal_chip_rate()), MSB first:
//   Noise:    18-bit LFSR sequence plus a zero pad bit. One sharp correlation peak.
//   Square20: square wave at half the chip rate (20 MHz on the ESP32-S3).
//   Square2:  square wave at a twentieth of the chip rate (2 MHz on the ESP32-S3).
//   Square40: 40 MHz square wave from the LED PWM (ESP32-S3 only).
// LFSR: state = 0x3FFFF; each bit: out = state & 1;
//   fb = (state ^ (state >> 7)) & 1; state = (state >> 1) | (fb << 17).
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgnss {

constexpr size_t kCalBits = size_t{1} << 18;
constexpr size_t kCalBytes = kCalBits / 8;

enum class CalMode : uint8_t { Off, Noise, Square20, Square40, Square2 };

constexpr uint32_t cal_burst_us(uint32_t chip_rate) {
  return static_cast<uint32_t>(uint64_t{kCalBits} * 1000000 / chip_rate);
}

const char* cal_mode_name(CalMode m);

// All zero for Off and Square40, which don't use SPI.
void cal_fill(CalMode mode, uint8_t* out);

}  // namespace sofgnss
