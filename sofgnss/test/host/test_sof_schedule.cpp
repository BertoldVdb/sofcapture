#include "check.h"
#include "sofgnss/sof_schedule.h"

using namespace sofgnss;

TEST(schedule_plain_multiples) {
  SofSchedule s{100, false};
  CHECK(s.is_target(0));
  CHECK(s.is_target(700));
  CHECK(!s.is_target(701));
  CHECK_EQ(s.next_target_after(0), 100u);
  CHECK_EQ(s.next_target_after(99), 100u);
  CHECK_EQ(s.next_target_after(100), 200u);
}

TEST(schedule_dither_properties) {
  SofSchedule s{100, true};
  uint32_t nonzero = 0;
  for (uint64_t w = 0; w < 10000; ++w) {
    uint64_t t = s.target(w);
    CHECK(t >= w * 100 && t < w * 100 + 50);
    CHECK(s.is_target(t));
    CHECK_EQ(s.window_of(t), w);
    CHECK_EQ(s.next_target_after(t), s.target(w + 1));
    if (w > 0) CHECK_EQ(s.next_target_after(w * 100 - 1), t);
    if (t % 100) nonzero++;
  }
  CHECK(nonzero > 9000);
}

TEST(schedule_dither_is_deterministic_and_spread) {
  SofSchedule a{100, true}, b{100, true};
  uint32_t histogram[50] = {};
  for (uint64_t w = 0; w < 50000; ++w) {
    CHECK_EQ(a.offset(w), b.offset(w));
    histogram[a.offset(w)]++;
  }
  // roughly uniform
  for (uint32_t h : histogram) CHECK(h > 700 && h < 1300);
}

TEST(schedule_small_dividers) {
  SofSchedule one{1, true};
  for (uint64_t f = 0; f < 10; ++f) CHECK(one.is_target(f));
  CHECK_EQ(one.next_target_after(5), 6u);
  SofSchedule two{2, true};  // too small to dither
  CHECK(two.is_target(4));
  CHECK(!two.is_target(5));
  SofSchedule three{3, true};
  CHECK(three.is_target(6));
}
