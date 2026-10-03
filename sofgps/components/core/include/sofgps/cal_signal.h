// Calibration bursts for an SDR connected by coax. A burst is kCalBits bits
// sent over SPI at 40 Mbit/s, MSB first (6.55 ms):
//   Noise:    18-bit LFSR sequence plus a zero pad bit; one sharp correlation peak.
//   Square20: 20 MHz square wave.
//   Square2:  2 MHz square wave; harmonics either side of the 160 MHz null.
//   Square40: 40 MHz square wave from the LED PWM (SPI can't do 80 Mbit/s).
// LFSR: state = 0x3FFFF; each bit: out = state & 1;
//   fb = (state ^ (state >> 7)) & 1; state = (state >> 1) | (fb << 17).
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgps {

constexpr size_t kCalBits = size_t{1} << 18;
constexpr size_t kCalBytes = kCalBits / 8;
constexpr uint32_t kCalChipRate = 40000000;

enum class CalMode : uint8_t { Off, Noise, Square20, Square40, Square2 };

constexpr uint32_t kCalBurstUs = static_cast<uint32_t>(uint64_t{kCalBits} * 1000000 / kCalChipRate);

const char* cal_mode_name(CalMode m);

// All zero for Off and Square40, which don't use SPI.
void cal_fill(CalMode mode, uint8_t* out);

}  // namespace sofgps
