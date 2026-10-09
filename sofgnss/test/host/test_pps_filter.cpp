#include "check.h"
#include "sofgnss/pps_filter.h"

using namespace sofgnss;

namespace {
constexpr uint32_t kHz = 80000000;
}

TEST(pps_filter_passes_regular_pulses) {
  PpsFilter f;
  uint32_t tick = 0xFFF00000;  // wraps during the run
  uint64_t us = 1000000;
  for (int i = 0; i < 100; ++i) {
    CHECK(f.accept(tick, us, kHz));
    tick += kHz;
    us += 1000000;
  }
  CHECK_EQ(f.rejected(), 0u);
}

TEST(pps_filter_rejects_glitch_after_edge) {
  PpsFilter f;
  CHECK(f.accept(1000, 1000000, kHz));
  CHECK(!f.accept(1000 + kHz / 1000, 1001000, kHz));      // 1 ms later: ringing
  CHECK(!f.accept(1000 + kHz / 2 - 1, 1499999, kHz));     // just under 0.5 s
  CHECK(f.accept(1000 + kHz, 2000000, kHz));
  CHECK_EQ(f.rejected(), 2u);
  // A glitch doesn't move the reference.
  CHECK(!f.accept(1000 + kHz + kHz / 4, 2250000, kHz));
  CHECK(f.accept(1000 + 2 * kHz, 3000000, kHz));
  CHECK_EQ(f.rejected(), 3u);
}

TEST(pps_filter_accepts_edge_after_counter_wrap) {
  // One full counter wrap later the tick gap looks tiny.
  PpsFilter f;
  CHECK(f.accept(5000, 1000000, kHz));
  uint64_t wrap_us = (uint64_t{1} << 32) * 1000000 / kHz;
  CHECK(f.accept(5000 + kHz / 100, 1000000 + wrap_us + 10000, kHz));
  CHECK_EQ(f.rejected(), 0u);
}

TEST(pps_filter_first_edge_always_accepted) {
  PpsFilter f;
  CHECK(f.accept(0, 0, kHz));
  CHECK_EQ(f.rejected(), 0u);
}
