#include <cstring>
#include <vector>

#include "check.h"
#include "sofgps/cal_signal.h"

using namespace sofgps;

TEST(cal_noise_matches_reference) {
  std::vector<uint8_t> buf(kCalBytes);
  cal_fill(CalMode::Noise, buf.data());
  // Reference values from a separate Python implementation.
  const uint8_t head[8] = {0xff, 0xff, 0xc0, 0x07, 0xf0, 0xff, 0xe3, 0xc3};
  const uint8_t tail[8] = {0xe1, 0x83, 0x88, 0x11, 0xe0, 0x38, 0x7f, 0x00};
  CHECK(memcmp(buf.data(), head, 8) == 0);
  CHECK(memcmp(buf.data() + kCalBytes - 8, tail, 8) == 0);
  uint32_t h = 0, ones = 0;
  for (uint8_t b : buf) {
    h = h * 31 + b;
    ones += __builtin_popcount(b);
  }
  CHECK_EQ(h, 0x7ddbb0d6u);
  CHECK_EQ(ones, 131072u);  // 2^17 ones in a degree-18 m-sequence
  CHECK_EQ(buf[kCalBytes - 1] & 1, 0);  // pad bit
}

TEST(cal_square_pattern) {
  std::vector<uint8_t> buf(kCalBytes);
  cal_fill(CalMode::Square20, buf.data());
  for (uint8_t b : buf) CHECK_EQ(b, 0xAA);
  CHECK_EQ(buf[0] & 0x80, 0x80);
  CHECK_EQ(buf[kCalBytes - 1] & 1, 0);     // ends at the idle level
}

TEST(cal_square2_pattern) {
  std::vector<uint8_t> buf(kCalBytes);
  cal_fill(CalMode::Square2, buf.data());
  size_t run = 0;
  int level = 0, runs = 0;
  for (size_t i = 0; i < kCalBits; ++i) {
    int bit = (buf[i >> 3] >> (7 - (i & 7))) & 1;
    if (i == 0) CHECK_EQ(bit, 0);
    if (bit == level) {
      run++;
    } else {
      CHECK_EQ(run, 10u);
      level = bit;
      run = 1;
      runs++;
    }
  }
  CHECK_EQ(level, 0);
  CHECK_EQ(run, 4u);   // last run is cut short
  CHECK(runs > 26000);
}

TEST(cal_mode_names) {
  CHECK(strcmp(cal_mode_name(CalMode::Square2), "SQ2") == 0);
  CHECK(strcmp(cal_mode_name(CalMode::Noise), "NOISE") == 0);
  CHECK(strcmp(cal_mode_name(CalMode::Square20), "SQ20") == 0);
  CHECK(strcmp(cal_mode_name(CalMode::Square40), "SQ40") == 0);
  CHECK(strcmp(cal_mode_name(CalMode::Off), "OFF") == 0);
  CHECK_EQ(kCalBurstUs, 6553u);
}
