// Ring buffer of whole messages; drops the oldest when full. Not thread-safe.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgnss {

class MsgQueue {
 public:
  static constexpr size_t kMaxMsg = 0xFFFF;

  MsgQueue(uint8_t* storage, size_t capacity);

  // False only if the message is bigger than the ring.
  bool push(const uint8_t* data, size_t len);

  // Truncates to cap but returns the full length (0 if empty).
  size_t pop(uint8_t* out, size_t cap);

  size_t peek_len() const;  // 0 when empty
  size_t count() const { return count_; }
  size_t used() const { return used_; }
  size_t capacity() const { return cap_; }
  uint32_t dropped() const { return dropped_; }
  bool empty() const { return count_ == 0; }
  void clear();

 private:
  void drop_oldest();
  void write(const uint8_t* src, size_t len);
  void read(size_t offset, uint8_t* dst, size_t len) const;

  uint8_t* buf_;
  size_t cap_;
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t used_ = 0;
  size_t count_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace sofgnss
