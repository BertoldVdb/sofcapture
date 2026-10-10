// Firmware update over the CDC port.
// Host sends $PVDB,SOFPPS,OTA,<size>,<sha256hex>*hh, waits for OTA,READY, then
// sends the raw image. Device replies ACK every kAckBytes, then OK or ERR,<reason>.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgnss {

class OtaReceiver {
 public:
  using Reply = void (*)(const char* sentence, size_t len, void* ctx);

  static constexpr uint32_t kAckBytes = 32768;
  static constexpr uint64_t kTimeoutUs = 5000000;  // abort after this long without data

  OtaReceiver(Reply reply, void* ctx) : reply_(reply), ctx_(ctx) {}

  // args: what follows "OTA,". The timeout starts after the slow slot erase.
  void start(const char* args, size_t len);

  // Returns bytes used; anything after the image is normal traffic.
  size_t feed(const uint8_t* data, size_t len, uint64_t now_us);

  // Call regularly; checks the timeout.
  void poll(uint64_t now_us);

  // Aborts a running transfer with ERR,<reason>.
  void cancel(const char* reason);

  bool receiving() const { return receiving_; }
  bool finished() const { return finished_; }  // restart when convenient

 private:
  void fail(const char* reason);  // abort, then err()
  void err(const char* reason);   // reply ERR,<reason>
  void say(const char* status, uint32_t value, bool with_value);

  Reply reply_;
  void* ctx_;
  bool receiving_ = false;
  bool finished_ = false;
  uint32_t size_ = 0;
  uint32_t received_ = 0;
  uint32_t next_ack_ = 0;
  uint64_t last_rx_us_ = 0;
  uint8_t sha_[32] = {};
};

}  // namespace sofgnss
