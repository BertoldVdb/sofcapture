#include "sofgnss/msg_queue.h"

#include <cstring>

namespace sofgnss {

namespace {
constexpr size_t kHeader = 2;  // length prefix, little-endian
}

MsgQueue::MsgQueue(uint8_t* storage, size_t capacity) : buf_(storage), cap_(capacity) {}

void MsgQueue::clear() {
  head_ = tail_ = used_ = count_ = 0;
}

void MsgQueue::write(const uint8_t* src, size_t len) {
  size_t first = cap_ - tail_;
  if (first > len) first = len;
  memcpy(buf_ + tail_, src, first);
  if (len > first) memcpy(buf_, src + first, len - first);
  tail_ = (tail_ + len) % cap_;
  used_ += len;
}

void MsgQueue::read(size_t offset, uint8_t* dst, size_t len) const {
  size_t pos = (head_ + offset) % cap_;
  size_t first = cap_ - pos;
  if (first > len) first = len;
  memcpy(dst, buf_ + pos, first);
  if (len > first) memcpy(dst + first, buf_, len - first);
}

size_t MsgQueue::peek_len() const {
  if (count_ == 0) return 0;
  uint8_t hdr[kHeader];
  read(0, hdr, kHeader);
  return hdr[0] | (static_cast<size_t>(hdr[1]) << 8);
}

void MsgQueue::drop_oldest() {
  size_t len = peek_len() + kHeader;
  head_ = (head_ + len) % cap_;
  used_ -= len;
  count_--;
  dropped_++;
}

bool MsgQueue::push(const uint8_t* data, size_t len) {
  if (len > kMaxMsg || len + kHeader > cap_) return false;
  while (cap_ - used_ < len + kHeader) drop_oldest();
  uint8_t hdr[kHeader] = {static_cast<uint8_t>(len & 0xFF), static_cast<uint8_t>(len >> 8)};
  write(hdr, kHeader);
  write(data, len);
  count_++;
  return true;
}

size_t MsgQueue::pop(uint8_t* out, size_t cap) {
  if (count_ == 0) return 0;
  size_t len = peek_len();
  read(kHeader, out, len < cap ? len : cap);
  head_ = (head_ + kHeader + len) % cap_;
  used_ -= kHeader + len;
  count_--;
  return len;
}

}  // namespace sofgnss
