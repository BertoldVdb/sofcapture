#pragma once

#include <cstdint>

namespace sofgnss {

// Unwraps the 32-bit tick counter. Values may go slightly backwards (a PPS
// captured before a later SOF) but must stay within 2^31 of the last one.
// Starts at 2^32 + tick so earlier captures don't go negative.
class TickUnwrapper {
 public:
  uint64_t update(uint32_t tick) {
    if (!init_) {
      init_ = true;
      last_ = (uint64_t{1} << 32) | tick;
      return last_;
    }
    int32_t delta = static_cast<int32_t>(tick - static_cast<uint32_t>(last_));
    last_ += static_cast<int64_t>(delta);
    return last_;
  }
  uint64_t last() const { return last_; }
  bool initialized() const { return init_; }
  void reset() { init_ = false; }

 private:
  uint64_t last_ = 0;
  bool init_ = false;
};

// Unwraps the 11-bit USB frame number. Call at least every 2047 frames.
class FrameUnwrapper {
 public:
  uint64_t update(uint16_t frame) {
    frame &= 0x7FF;
    if (!init_) {
      init_ = true;
      last11_ = frame;
      frame_ = frame;
      return frame_;
    }
    frame_ += static_cast<uint16_t>((frame - last11_) & 0x7FF);
    last11_ = frame;
    return frame_;
  }
  uint64_t last() const { return frame_; }
  bool initialized() const { return init_; }
  void reset() { init_ = false; }

 private:
  uint64_t frame_ = 0;
  uint16_t last11_ = 0;
  bool init_ = false;
};

}  // namespace sofgnss
