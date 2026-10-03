#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "sofgps/msg_queue.h"

using namespace sofgps;

namespace {

bool push_str(MsgQueue& q, const std::string& s) {
  return q.push(reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

std::string pop_str(MsgQueue& q) {
  uint8_t buf[256];
  size_t n = q.pop(buf, sizeof buf);
  return std::string(reinterpret_cast<char*>(buf), n < sizeof buf ? n : sizeof buf);
}

}  // namespace

TEST(queue_fifo_order) {
  uint8_t storage[64];
  MsgQueue q(storage, sizeof storage);
  CHECK(q.empty());
  CHECK(push_str(q, "one"));
  CHECK(push_str(q, "two"));
  CHECK(push_str(q, "three"));
  CHECK_EQ(q.count(), 3u);
  CHECK_EQ(q.peek_len(), 3u);
  CHECK(pop_str(q) == "one");
  CHECK(pop_str(q) == "two");
  CHECK(pop_str(q) == "three");
  CHECK(q.empty());
  CHECK_EQ(q.pop(storage, 1), 0u);
  CHECK_EQ(q.dropped(), 0u);
}

TEST(queue_wraps_around_storage) {
  uint8_t storage[32];
  MsgQueue q(storage, sizeof storage);
  for (int i = 0; i < 100; ++i) {
    std::string s = "msg" + std::to_string(i) + "-padding";
    CHECK(push_str(q, s));
    CHECK(pop_str(q) == s);
  }
  CHECK_EQ(q.dropped(), 0u);
}

TEST(queue_drops_oldest_whole_messages) {
  uint8_t storage[32];  // fits two 12-byte messages, not three
  MsgQueue q(storage, sizeof storage);
  CHECK(push_str(q, "AAAAAAAAAAAA"));
  CHECK(push_str(q, "BBBBBBBBBBBB"));
  CHECK_EQ(q.count(), 2u);
  CHECK(push_str(q, "CCCCCCCCCCCC"));
  CHECK_EQ(q.count(), 2u);
  CHECK_EQ(q.dropped(), 1u);
  CHECK(pop_str(q) == "BBBBBBBBBBBB");
  CHECK(pop_str(q) == "CCCCCCCCCCCC");
}

TEST(queue_drops_several_to_fit_a_large_message) {
  uint8_t storage[40];
  MsgQueue q(storage, sizeof storage);
  for (int i = 0; i < 5; ++i) CHECK(push_str(q, "abcdef"));  // fills the ring
  CHECK_EQ(q.count(), 5u);
  CHECK(push_str(q, std::string(30, 'z')));  // needs 32: drops four
  CHECK_EQ(q.count(), 2u);
  CHECK_EQ(q.dropped(), 4u);
  CHECK(pop_str(q) == "abcdef");
  CHECK(pop_str(q) == std::string(30, 'z'));
}

TEST(queue_rejects_message_larger_than_ring) {
  uint8_t storage[16];
  MsgQueue q(storage, sizeof storage);
  CHECK(!push_str(q, std::string(15, 'x')));
  CHECK(push_str(q, std::string(14, 'x')));
  CHECK_EQ(q.dropped(), 0u);
}

TEST(queue_pop_truncates_but_reports_full_length) {
  uint8_t storage[64];
  MsgQueue q(storage, sizeof storage);
  CHECK(push_str(q, "0123456789"));
  uint8_t out[4];
  CHECK_EQ(q.pop(out, sizeof out), 10u);
  CHECK(memcmp(out, "0123", 4) == 0);
  CHECK(q.empty());
  CHECK_EQ(q.used(), 0u);
}

TEST(queue_clear) {
  uint8_t storage[64];
  MsgQueue q(storage, sizeof storage);
  push_str(q, "a");
  push_str(q, "b");
  q.clear();
  CHECK(q.empty());
  CHECK_EQ(q.used(), 0u);
  CHECK(push_str(q, "c"));
  CHECK(pop_str(q) == "c");
}
