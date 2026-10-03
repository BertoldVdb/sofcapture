// Picks which USB frames get tagged: one per window of `divider` frames.
// Without dither it's the first one. With dither it's a pseudo-random offset
// in [0, divider/2) computed from the window index, so all users agree.
// Dither stops the timing error from forming a slow sawtooth that a short
// fit can't average out.
#pragma once

#include <cstdint>

namespace sofgps {

struct SofSchedule {
  uint32_t divider;
  bool dither;

  uint32_t offset(uint64_t window) const {
    uint32_t span = divider / 2;
    if (!dither || span == 0) return 0;
    // splitmix64 hash
    uint64_t x = window * 0x9E3779B97F4A7C15ull + 0x632BE59BD9B4E019ull;
    x ^= x >> 30;
    x *= 0xBF58476D1CE4E5B9ull;
    x ^= x >> 27;
    x *= 0x94D049BB133111EBull;
    x ^= x >> 31;
    return static_cast<uint32_t>(x % span);
  }

  uint64_t target(uint64_t window) const { return window * divider + offset(window); }
  uint64_t window_of(uint64_t frame) const { return frame / divider; }
  bool is_target(uint64_t frame) const { return frame == target(window_of(frame)); }

  uint64_t next_target_after(uint64_t frame) const {
    uint64_t w = window_of(frame);
    uint64_t t = target(w);
    return t > frame ? t : target(w + 1);
  }
};

}  // namespace sofgps
