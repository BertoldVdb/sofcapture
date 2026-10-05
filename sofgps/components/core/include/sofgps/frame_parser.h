// Splits the GPS byte stream into NMEA, UBX and RTCM3 frames.
// NMEA needs a valid "*hh" checksum and always comes out ending in "\r\n".
// Binary frames too big for the buffer are skipped using their length field.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgps {

enum class FrameType : uint8_t { Nmea, Ubx, Rtcm3 };

struct ParserStats {
  uint32_t nmea = 0;
  uint32_t ubx = 0;
  uint32_t rtcm3 = 0;
  uint32_t bad_checksum = 0;  // also NMEA without a checksum
  uint32_t junk_bytes = 0;  // bytes outside frames, incl. aborted ones
  uint32_t oversize = 0;    // too big for the buffer
  uint32_t timeouts = 0;    // dropped by abort_partial()
  uint32_t errors() const { return bad_checksum + oversize + timeouts; }
};

class FrameParser {
 public:
  using Callback = void (*)(FrameType type, const uint8_t* data, size_t len, void* ctx);

  // Includes "\r\n". NMEA says 82, but receivers send longer ones.
  static constexpr size_t kMaxNmea = 256;

  FrameParser(uint8_t* buf, size_t cap, Callback cb, void* ctx);

  void feed(const uint8_t* data, size_t len);
  void reset();

  // If the GPS goes quiet mid-frame, call abort_partial(); otherwise a
  // corrupted length could swallow the frames that follow.
  bool in_frame() const { return state_ != State::Idle; }
  size_t partial_len() const { return in_frame() ? (len_ ? len_ : 1) : 0; }
  // Length from the frame header, or kMaxNmea if not known yet.
  size_t declared_len() const { return declared_ ? declared_ : kMaxNmea; }
  // How long a frame may take to arrive: wire time + 25% + slack_us.
  static uint64_t max_frame_us(size_t bytes, uint32_t baud, uint64_t slack_us);
  void abort_partial();
  const ParserStats& stats() const { return stats_; }

  static uint32_t crc24q(const uint8_t* data, size_t len);

 private:
  enum class State : uint8_t {
    Idle,
    Nmea,        // sentence body
    NmeaCk1,     // first checksum digit
    NmeaCk2,     // second checksum digit
    UbxSync,     // got 0xB5, want 0x62
    UbxHeader,   // class, id, length
    UbxBody,     // payload + checksum
    RtcmHeader,  // length bytes
    RtcmBody,    // payload + CRC
    Skip,        // rest of an oversize frame
  };

  void byte(uint8_t b);
  void start(uint8_t b);
  void abort_to_idle(uint8_t b);
  void skip_oversize();
  void finish_nmea(uint8_t received);
  void finish_ubx();
  void finish_rtcm();
  void emit(FrameType type);

  uint8_t* buf_;
  size_t cap_;
  size_t len_ = 0;
  size_t need_ = 0;
  size_t declared_ = 0;
  uint8_t nmea_xor_ = 0;
  uint8_t nmea_ck_hi_ = 0;
  State state_ = State::Idle;
  Callback cb_;
  void* ctx_;
  ParserStats stats_;
};

}  // namespace sofgps
