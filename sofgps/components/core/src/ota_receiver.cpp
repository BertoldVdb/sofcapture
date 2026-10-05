#include "sofgps/ota_receiver.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "sofgps/hal.h"
#include "sofgps/pvdb.h"

namespace sofgps {

namespace {

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parse_sha256(const char* hex, size_t len, uint8_t out[32]) {
  if (len != 64) return false;
  for (size_t i = 0; i < 32; ++i) {
    int hi = hex_value(hex[2 * i]);
    int lo = hex_value(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) return false;
    out[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return true;
}

}  // namespace

void OtaReceiver::say(const char* status, uint32_t value, bool with_value) {
  char s[pvdb::kMaxSentence];
  size_t n = with_value ? pvdb::format_ota(s, sizeof s, status, value)
                        : pvdb::format_ota(s, sizeof s, status);
  reply_(s, n, ctx_);
}

void OtaReceiver::fail(const char* reason) {
  if (receiving_) hal::ota_abort();
  receiving_ = false;
  err(reason);
}

void OtaReceiver::err(const char* reason) {
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_err(s, sizeof s, reason);
  reply_(s, n, ctx_);
}

// args: "<size>,<sha256hex>"
void OtaReceiver::start(const char* args, size_t len) {
  if (receiving_ || finished_) {
    err("OTABUSY");
    return;
  }
  char buf[96];
  if (len >= sizeof buf) {
    fail("OTAARGS");
    return;
  }
  memcpy(buf, args, len);
  buf[len] = '\0';
  char* comma = strchr(buf, ',');
  if (!comma) {
    fail("OTAARGS");
    return;
  }
  *comma = '\0';
  char* end;
  unsigned long size = strtoul(buf, &end, 10);
  if (*end != '\0' || size == 0 || size > 0xFFFFFFFFul) {
    fail("OTAARGS");
    return;
  }
  if (!parse_sha256(comma + 1, strlen(comma + 1), sha_)) {
    fail("OTAARGS");
    return;
  }
  if (!hal::ota_begin(static_cast<uint32_t>(size))) {
    fail("OTABEGIN");
    return;
  }
  size_ = static_cast<uint32_t>(size);
  received_ = 0;
  next_ack_ = kAckBytes;
  last_rx_us_ = hal::uptime_us();  // timeout starts after the erase
  receiving_ = true;
  say("READY", 0, false);
}

size_t OtaReceiver::feed(const uint8_t* data, size_t len, uint64_t now_us) {
  if (!receiving_) return 0;
  size_t take = size_ - received_;
  if (take > len) take = len;
  if (take && !hal::ota_write(data, take)) {
    fail("OTAWRITE");
    return len;  // the rest was image data too
  }
  received_ += static_cast<uint32_t>(take);
  last_rx_us_ = now_us;

  if (received_ >= next_ack_ && received_ < size_) {
    say("ACK", received_, true);
    next_ack_ += kAckBytes;
  }
  if (received_ == size_) {
    receiving_ = false;
    if (hal::ota_end(sha_)) {
      finished_ = true;
      say("OK", 0, false);
    } else {
      fail("OTAVERIFY");
    }
  }
  return take;
}

void OtaReceiver::poll(uint64_t now_us) {
  if (receiving_ && now_us - last_rx_us_ > kTimeoutUs) fail("OTATIMEOUT");
}

void OtaReceiver::cancel(const char* reason) {
  if (receiving_) fail(reason);
}

}  // namespace sofgps
