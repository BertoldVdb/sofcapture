#include "sofgnss/cal_signal.h"

#include <cstring>

namespace sofgnss {

const char* cal_mode_name(CalMode m) {
  switch (m) {
    case CalMode::Noise: return "NOISE";
    case CalMode::Square20: return "SQ20";
    case CalMode::Square40: return "SQ40";
    case CalMode::Square2: return "SQ2";
    default: return "OFF";
  }
}

void cal_fill(CalMode mode, uint8_t* out) {
  switch (mode) {
    case CalMode::Square20:
      memset(out, 0xAA, kCalBytes);  // starts high, ends low
      return;
    case CalMode::Square2: {
      // 40-bit cycle; the burst stops 24 bits into one, while low.
      static const uint8_t cycle[5] = {0x00, 0x3F, 0xF0, 0x03, 0xFF};
      for (size_t i = 0; i < kCalBytes; ++i) out[i] = cycle[i % 5];
      return;
    }
    case CalMode::Noise: {
      memset(out, 0, kCalBytes);
      uint32_t s = 0x3FFFF;
      for (size_t i = 0; i < kCalBits - 1; ++i) {  // last bit stays 0
        if (s & 1) out[i >> 3] |= static_cast<uint8_t>(0x80 >> (i & 7));
        uint32_t fb = (s ^ (s >> 7)) & 1;
        s = (s >> 1) | (fb << 17);
      }
      return;
    }
    default:
      memset(out, 0, kCalBytes);
  }
}

}  // namespace sofgnss
