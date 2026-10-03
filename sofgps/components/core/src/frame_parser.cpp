#include "sofgps/frame_parser.h"

namespace sofgps {

namespace {

bool nmea_printable(uint8_t b) { return b >= 0x20 && b <= 0x7E; }

int hex_value(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

}  // namespace

FrameParser::FrameParser(uint8_t* buf, size_t cap, Callback cb, void* ctx)
    : buf_(buf), cap_(cap), cb_(cb), ctx_(ctx) {}

void FrameParser::reset() {
  state_ = State::Idle;
  len_ = 0;
  need_ = 0;
}

void FrameParser::abort_partial() {
  if (state_ == State::Idle) return;
  stats_.timeouts++;
  stats_.junk_bytes += static_cast<uint32_t>(len_);
  reset();
}

void FrameParser::feed(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) byte(data[i]);
}

// CRC-24Q, as used by RTCM3.
uint32_t FrameParser::crc24q(const uint8_t* data, size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (int b = 0; b < 8; ++b) {
      crc <<= 1;
      if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
  }
  return crc & 0xFFFFFF;
}

void FrameParser::emit(FrameType type) {
  switch (type) {
    case FrameType::Nmea: stats_.nmea++; break;
    case FrameType::Ubx: stats_.ubx++; break;
    case FrameType::Rtcm3: stats_.rtcm3++; break;
  }
  cb_(type, buf_, len_, ctx_);
}

void FrameParser::start(uint8_t b) {
  len_ = 0;
  if (b == '$' || b == '!') {
    buf_[len_++] = b;
    nmea_xor_ = 0;
    state_ = State::Nmea;
  } else if (b == 0xB5) {
    buf_[len_++] = b;
    state_ = State::UbxSync;
  } else if (b == 0xD3) {
    buf_[len_++] = b;
    state_ = State::RtcmHeader;
  } else {
    // CR/LF after an NMEA checksum is normal.
    if (b != '\r' && b != '\n') stats_.junk_bytes++;
    state_ = State::Idle;
  }
}

// Drop the partial frame and retry b as a frame start.
void FrameParser::abort_to_idle(uint8_t b) {
  stats_.junk_bytes += static_cast<uint32_t>(len_);
  state_ = State::Idle;
  start(b);
}

void FrameParser::byte(uint8_t b) {
  switch (state_) {
    case State::Idle:
      start(b);
      break;

    case State::Nmea:
      if (b == '*') {
        buf_[len_++] = b;
        state_ = State::NmeaCk1;
      } else if (b == '\r' || b == '\n') {
        // no checksum
        stats_.bad_checksum++;
        stats_.junk_bytes += static_cast<uint32_t>(len_);
        len_ = 0;
        state_ = State::Idle;
      } else if (b == '$' || b == '!') {
        abort_to_idle(b);
      } else if (nmea_printable(b)) {
        // Room for "*hh\r\n" must remain.
        if (len_ + 5 > kMaxNmea || len_ + 5 > cap_) {
          stats_.oversize++;
          stats_.junk_bytes += static_cast<uint32_t>(len_);
          len_ = 0;
          state_ = State::Idle;
        } else {
          buf_[len_++] = b;
          nmea_xor_ ^= b;
        }
      } else {
        abort_to_idle(b);
      }
      break;

    case State::NmeaCk1:
      if (hex_value(b) >= 0) {
        buf_[len_++] = b;
        nmea_ck_hi_ = static_cast<uint8_t>(hex_value(b));
        state_ = State::NmeaCk2;
      } else {
        stats_.bad_checksum++;
        abort_to_idle(b);
      }
      break;

    case State::NmeaCk2:
      if (hex_value(b) >= 0) {
        buf_[len_++] = b;
        finish_nmea(static_cast<uint8_t>((nmea_ck_hi_ << 4) | hex_value(b)));
      } else {
        stats_.bad_checksum++;
        abort_to_idle(b);
      }
      break;

    case State::UbxSync:
      if (b == 0x62) {
        buf_[len_++] = b;
        state_ = State::UbxHeader;
      } else {
        abort_to_idle(b);
      }
      break;

    case State::UbxHeader:
      buf_[len_++] = b;
      if (len_ == 6) {
        size_t payload = buf_[4] | (static_cast<size_t>(buf_[5]) << 8);
        need_ = 6 + payload + 2;
        if (need_ > cap_ || need_ > kMaxUbx) {
          stats_.oversize++;
          state_ = State::Idle;
        } else {
          state_ = State::UbxBody;
        }
      }
      break;

    case State::UbxBody:
      buf_[len_++] = b;
      if (len_ == need_) finish_ubx();
      break;

    case State::RtcmHeader:
      buf_[len_++] = b;
      if (len_ == 3) {
        if (buf_[1] & 0xFC) {
          // reserved bits set: not RTCM3
          stats_.junk_bytes += static_cast<uint32_t>(len_);
          state_ = State::Idle;
          break;
        }
        size_t payload = (static_cast<size_t>(buf_[1] & 0x03) << 8) | buf_[2];
        need_ = 3 + payload + 3;
        if (need_ > cap_) {
          stats_.oversize++;
          state_ = State::Idle;
        } else {
          state_ = State::RtcmBody;
        }
      }
      break;

    case State::RtcmBody:
      buf_[len_++] = b;
      if (len_ == need_) finish_rtcm();
      break;
  }
}

// Any CR/LF the receiver sends after "*hh" is dropped in Idle.
void FrameParser::finish_nmea(uint8_t received) {
  state_ = State::Idle;
  if (received == nmea_xor_) {
    buf_[len_++] = '\r';
    buf_[len_++] = '\n';
    emit(FrameType::Nmea);
  } else {
    stats_.bad_checksum++;
  }
  len_ = 0;
}

void FrameParser::finish_ubx() {
  state_ = State::Idle;
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < len_ - 2; ++i) {
    ck_a = static_cast<uint8_t>(ck_a + buf_[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  if (ck_a == buf_[len_ - 2] && ck_b == buf_[len_ - 1]) {
    emit(FrameType::Ubx);
  } else {
    stats_.bad_checksum++;
  }
}

void FrameParser::finish_rtcm() {
  state_ = State::Idle;
  uint32_t crc = crc24q(buf_, len_ - 3);
  uint32_t got = (static_cast<uint32_t>(buf_[len_ - 3]) << 16) |
                 (static_cast<uint32_t>(buf_[len_ - 2]) << 8) | buf_[len_ - 1];
  if (crc == got) {
    emit(FrameType::Rtcm3);
  } else {
    stats_.bad_checksum++;
  }
}

}  // namespace sofgps
