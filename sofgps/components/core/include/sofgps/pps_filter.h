// Drops PPS edges that come too soon after the last one (ringing, bounce).
// Header-only so the ISR can use it.
#pragma once

#include <cstdint>

namespace sofgps {

class PpsFilter {
 public:
  static constexpr uint64_t kMinGapUs = 500000;

  // now_us only guards against the tick counter wrapping (53 s at 80 MHz).
  bool accept(uint32_t tick, uint64_t now_us, uint32_t tick_hz) {
    if (have_ && now_us - last_us_ < 2 * kMinGapUs &&
        static_cast<uint64_t>(tick - last_tick_) * 1000000 < kMinGapUs * tick_hz) {
      rejected_++;
      return false;
    }
    have_ = true;
    last_tick_ = tick;
    last_us_ = now_us;
    return true;
  }

  uint32_t rejected() const { return rejected_; }

 private:
  uint32_t last_tick_ = 0;
  uint64_t last_us_ = 0;
  bool have_ = false;
  uint32_t rejected_ = 0;
};

}  // namespace sofgps
