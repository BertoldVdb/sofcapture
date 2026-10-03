#include "check.h"
#include "sofgps/unwrap.h"

using namespace sofgps;

TEST(tick_unwrap_low_bits_match_hardware) {
  TickUnwrapper u;
  uint64_t v = u.update(0x12345678);
  CHECK_EQ(v & 0xFFFFFFFF, 0x12345678u);
  CHECK_EQ(u.update(0x12345700) & 0xFFFFFFFF, 0x12345700u);
}

TEST(tick_unwrap_forward_across_wrap) {
  TickUnwrapper u;
  uint64_t a = u.update(0xFFFFFF00);
  uint64_t b = u.update(0x00000100);
  CHECK_EQ(b - a, 0x200u);
}

TEST(tick_unwrap_accepts_slightly_earlier_values) {
  TickUnwrapper u;
  uint64_t a = u.update(0x00000010);
  uint64_t b = u.update(0xFFFFFFF0);  // 0x20 ticks before a, across the wrap
  CHECK_EQ(a - b, 0x20u);
  uint64_t c = u.update(0x00000030);
  CHECK_EQ(c - a, 0x20u);
}

TEST(tick_unwrap_long_run) {
  TickUnwrapper u;
  uint64_t prev = u.update(0);
  uint32_t hw = 0;
  for (int i = 0; i < 1000; ++i) {
    hw += 80000000;  // 1 s at 80 MHz
    uint64_t v = u.update(hw);
    CHECK_EQ(v - prev, 80000000u);
    prev = v;
  }
}

TEST(frame_unwrap_across_2048) {
  FrameUnwrapper u;
  CHECK_EQ(u.update(2040), 2040u);
  CHECK_EQ(u.update(2047), 2047u);
  CHECK_EQ(u.update(0), 2048u);
  CHECK_EQ(u.update(5), 2053u);
  CHECK_EQ(u.update(4), 2053u + 2047u);  // moved "backwards" = 2047 frames ahead
}

TEST(frame_unwrap_masks_input) {
  FrameUnwrapper u;
  CHECK_EQ(u.update(0x3800 | 10), 10u);
  CHECK_EQ(u.update(0x3800 | 11), 11u);
}
