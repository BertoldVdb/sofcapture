#include <string>
#include <vector>

#include "check.h"
#include "hal_sim.h"
#include "sofgnss/hal.h"
#include "sofgnss/ota_receiver.h"

using namespace sofgnss;

namespace {

struct Replies {
  std::vector<std::string> lines;
  bool has(const char* text) const {
    for (const std::string& l : lines)
      if (l.find(text) != std::string::npos) return true;
    return false;
  }
};

void reply(const char* s, size_t n, void* ctx) {
  static_cast<Replies*>(ctx)->lines.emplace_back(s, n);
}

const std::string kSha(64, 'a');

void start(OtaReceiver& r, const std::string& args) { r.start(args.c_str(), args.size()); }

}  // namespace

TEST(ota_rejects_bad_arguments) {
  sim::reset();
  Replies out;
  OtaReceiver r(reply, &out);
  start(r, "12345");
  start(r, "0," + kSha);
  start(r, "12,abc");
  start(r, "x12," + kSha);
  CHECK_EQ(out.lines.size(), 4u);
  for (const std::string& l : out.lines) CHECK(l.find("ERR,OTAARGS") != std::string::npos);
  CHECK(!r.receiving());
  CHECK_EQ(sim::state().ota_aborts, 0);
}

TEST(ota_begin_failure_reported) {
  sim::reset();
  sim::state().ota_begin_ok = false;
  Replies out;
  OtaReceiver r(reply, &out);
  start(r, "100," + kSha);
  CHECK(out.has("ERR,OTABEGIN"));
  CHECK(!r.receiving());
}

TEST(ota_full_transfer_with_acks_and_trailing_bytes) {
  sim::reset();
  Replies out;
  OtaReceiver r(reply, &out);
  const uint32_t size = 3 * OtaReceiver::kAckBytes + 100;
  start(r, std::to_string(size) + "," + kSha);
  CHECK(out.has("OTA,READY"));
  CHECK(r.receiving());

  std::vector<uint8_t> chunk(4096, 0x5A);
  uint32_t sent = 0;
  uint64_t t = 2000;
  while (sent + chunk.size() <= size) {
    CHECK_EQ(r.feed(chunk.data(), chunk.size(), t += 1000), chunk.size());
    sent += chunk.size();
  }
  // image tail plus 10 bytes of GPS traffic
  std::vector<uint8_t> last(size - sent + 10, 0x5A);
  size_t taken = r.feed(last.data(), last.size(), t += 1000);
  CHECK_EQ(taken, size - sent);
  CHECK(!r.receiving());
  CHECK(r.finished());
  CHECK(out.has("OTA,OK"));
  CHECK_EQ(sim::state().ota_bytes, size);
  int acks = 0;
  for (const std::string& l : out.lines)
    if (l.find("OTA,ACK,") != std::string::npos) acks++;
  CHECK_EQ(acks, 3);
  CHECK(!sim::state().ota_active);

  // busy until restart
  start(r, "10," + kSha);
  CHECK(out.has("ERR,OTABUSY"));
}

TEST(ota_verify_failure) {
  sim::reset();
  sim::state().ota_end_ok = false;
  Replies out;
  OtaReceiver r(reply, &out);
  start(r, "50," + kSha);
  std::vector<uint8_t> data(50, 1);
  r.feed(data.data(), data.size(), 2000);
  CHECK(out.has("ERR,OTAVERIFY"));
  CHECK(!r.finished());
  CHECK(!r.receiving());
  start(r, "50," + kSha);
  CHECK(r.receiving());
}

TEST(ota_timeout_aborts) {
  sim::reset();
  Replies out;
  OtaReceiver r(reply, &out);
  start(r, "1000," + kSha);
  std::vector<uint8_t> data(100, 1);
  r.feed(data.data(), data.size(), 2000);
  r.poll(2000 + OtaReceiver::kTimeoutUs - 1);
  CHECK(r.receiving());
  r.poll(2000 + OtaReceiver::kTimeoutUs + 1);
  CHECK(!r.receiving());
  CHECK(out.has("ERR,OTATIMEOUT"));
  CHECK_EQ(sim::state().ota_aborts, 1);
  CHECK_EQ(r.feed(data.data(), data.size(), 9000000), 0u);
}

TEST(ota_timeout_starts_after_slot_erase) {
  sim::reset();
  sim::state().ota_begin_ticks = 3ull * sim::state().tick_hz;  // a 3 s erase
  Replies out;
  OtaReceiver r(reply, &out);
  uint64_t before = hal::uptime_us();
  start(r, "1000," + kSha);
  uint64_t after = hal::uptime_us();
  CHECK_EQ(after - before, 3000000u);
  r.poll(before + OtaReceiver::kTimeoutUs + 1);  // would have expired from before
  CHECK(r.receiving());
  r.poll(after + OtaReceiver::kTimeoutUs + 1);
  CHECK(!r.receiving());
  CHECK(out.has("ERR,OTATIMEOUT"));
}

TEST(ota_busy_does_not_abort_running_transfer) {
  sim::reset();
  Replies out;
  OtaReceiver r(reply, &out);
  start(r, "100," + kSha);
  std::vector<uint8_t> data(50, 1);
  r.feed(data.data(), data.size(), 2000);
  start(r, "100," + kSha);
  CHECK(out.has("ERR,OTABUSY"));
  CHECK(r.receiving());
  CHECK(sim::state().ota_active);
  CHECK_EQ(sim::state().ota_aborts, 0);
  r.feed(data.data(), data.size(), 3000);
  CHECK(r.finished());
  CHECK(out.has("OTA,OK"));
}

TEST(ota_cancel_aborts_only_a_running_transfer) {
  sim::reset();
  Replies out;
  OtaReceiver r(reply, &out);
  r.cancel("OTALINK");  // idle: no reply
  CHECK(out.lines.empty());
  start(r, "100," + kSha);
  r.cancel("OTALINK");
  CHECK(!r.receiving());
  CHECK(out.has("ERR,OTALINK"));
  CHECK_EQ(sim::state().ota_aborts, 1);
  start(r, "100," + kSha);
  CHECK(r.receiving());
}
