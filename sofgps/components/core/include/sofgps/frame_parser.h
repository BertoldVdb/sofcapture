// Splits the GPS byte stream into whole NMEA, UBX and RTCM3 frames.
//
// UBX and RTCM3 frames are delivered as their original bytes. An NMEA
// sentence must carry a "*hh" checksum; it is complete as soon as the second
// hex digit arrives and is delivered with a normalised "\r\n" terminator,
// whatever line ending (if any) the receiver sent. Sentences without a
// checksum, with a wrong one, and bytes that belong to no frame are dropped.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sofgps {

enum class FrameType : uint8_t { Nmea, Ubx, Rtcm3 };

struct ParserStats {
  uint32_t nmea = 0;
  uint32_t ubx = 0;
  uint32_t rtcm3 = 0;
  uint32_t bad_checksum = 0;  // includes NMEA sentences without a checksum
  uint32_t junk_bytes = 0;  // bytes outside any frame, plus aborted partial frames
  uint32_t oversize = 0;    // frames that did not fit the buffer
  uint32_t timeouts = 0;    // partial frames dropped by abort_partial()
  uint32_t errors() const { return bad_checksum + oversize + timeouts; }
};

class FrameParser {
 public:
  using Callback = void (*)(FrameType type, const uint8_t* data, size_t len, void* ctx);

  // Longest NMEA sentence accepted. The standard says 82; real receivers
  // exceed it, so allow a generous margin.
  static constexpr size_t kMaxNmea = 256;
  // Longest UBX frame accepted (header, payload and checksum). A header that
  // claims more is not taken as a frame start, so a corrupted or false length
  // cannot hold up more than this many bytes of the stream.
  static constexpr size_t kMaxUbx = 2048;

  FrameParser(uint8_t* buf, size_t cap, Callback cb, void* ctx);

  void feed(const uint8_t* data, size_t len);
  void reset();

  // If the GPS goes quiet mid-frame, call abort_partial(); otherwise a
  // corrupted length could swallow the frames that follow.
  bool in_frame() const { return state_ != State::Idle; }
  size_t partial_len() const { return in_frame() ? (len_ ? len_ : 1) : 0; }
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
  };

  void byte(uint8_t b);
  void start(uint8_t b);
  void abort_to_idle(uint8_t b);
  void finish_nmea(uint8_t received);
  void finish_ubx();
  void finish_rtcm();
  void emit(FrameType type);

  uint8_t* buf_;
  size_t cap_;
  size_t len_ = 0;
  size_t need_ = 0;
  uint8_t nmea_xor_ = 0;
  uint8_t nmea_ck_hi_ = 0;
  State state_ = State::Idle;
  Callback cb_;
  void* ctx_;
  ParserStats stats_;
};

}  // namespace sofgps
