#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "sofgnss/frame_parser.h"

using namespace sofgnss;

namespace {

struct Frame {
  FrameType type;
  std::vector<uint8_t> data;
};

struct Sink {
  std::vector<Frame> frames;
};

void collect(FrameType type, const uint8_t* data, size_t len, void* ctx) {
  static_cast<Sink*>(ctx)->frames.push_back({type, std::vector<uint8_t>(data, data + len)});
}

struct Fixture {
  uint8_t buf[4096];
  Sink sink;
  FrameParser parser{buf, sizeof buf, collect, &sink};
  void feed(const std::vector<uint8_t>& v) { parser.feed(v.data(), v.size()); }
  void feed(const std::string& s) {
    parser.feed(reinterpret_cast<const uint8_t*>(s.data()), s.size());
  }
  void feed_bytewise(const std::vector<uint8_t>& v) {
    for (uint8_t b : v) parser.feed(&b, 1);
  }
};

std::vector<uint8_t> bytes(const std::string& s) { return {s.begin(), s.end()}; }

std::vector<uint8_t> operator+(std::vector<uint8_t> a, const std::vector<uint8_t>& b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

std::string nmea(const std::string& body) {
  uint8_t sum = 0;
  for (char c : body) sum ^= static_cast<uint8_t>(c);
  char tail[8];
  snprintf(tail, sizeof tail, "*%02X\r\n", sum);
  return "$" + body + tail;
}

std::vector<uint8_t> ubx(uint8_t cls, uint8_t id, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f = {0xB5, 0x62, cls, id, static_cast<uint8_t>(payload.size() & 0xFF),
                            static_cast<uint8_t>(payload.size() >> 8)};
  f = f + payload;
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < f.size(); ++i) {
    a = static_cast<uint8_t>(a + f[i]);
    b = static_cast<uint8_t>(b + a);
  }
  f.push_back(a);
  f.push_back(b);
  return f;
}

std::vector<uint8_t> rtcm(const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> f = {0xD3, static_cast<uint8_t>(payload.size() >> 8),
                            static_cast<uint8_t>(payload.size() & 0xFF)};
  f = f + payload;
  uint32_t crc = FrameParser::crc24q(f.data(), f.size());
  f.push_back(crc >> 16);
  f.push_back(crc >> 8);
  f.push_back(crc);
  return f;
}

const std::string kGga = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
const std::vector<uint8_t> kAckAck = {0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x01, 0x0F, 0x38};
const std::vector<uint8_t> k1005 = {0xD3, 0x00, 0x13, 0x3E, 0xD7, 0xD3, 0x02, 0x02, 0x98, 0x0E, 0xDE,
                                    0xEF, 0x34, 0xB4, 0xBD, 0x62, 0xAC, 0x09, 0x41, 0x98, 0x6F, 0x33,
                                    0x36, 0x0B, 0x98};

}  // namespace

TEST(parser_nmea_reference_sentence) {
  Fixture f;
  f.feed(kGga);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Nmea);
  CHECK(f.sink.frames[0].data == bytes(kGga));
}

TEST(parser_nmea_bad_checksum_dropped) {
  Fixture f;
  f.feed("$GPGGA,123519,4807.038,N*48\r\n");
  CHECK(f.sink.frames.empty());
  CHECK_EQ(f.parser.stats().bad_checksum, 1u);
  f.feed(nmea("GPRMC,1"));
  CHECK_EQ(f.sink.frames.size(), 1u);
}

TEST(parser_nmea_without_checksum_dropped) {
  Fixture f;
  f.feed("$PMTK001,314,3\r\n$PMTK001,314,4\r");
  CHECK(f.sink.frames.empty());
  CHECK_EQ(f.parser.stats().bad_checksum, 2u);
  f.feed(kGga);
  CHECK_EQ(f.sink.frames.size(), 1u);
}

TEST(parser_nmea_terminators_normalised) {
  std::string full = nmea("GPZDA,1");
  std::string body = full.substr(0, full.size() - 2);
  for (const std::string& tail : {std::string("\r\n"), std::string("\n"), std::string("\r"),
                                  std::string("")}) {
    Fixture f;
    f.feed(body + tail);
    f.feed(kGga);
    CHECK_EQ(f.sink.frames.size(), 2u);
    CHECK(f.sink.frames[0].data == bytes(full));
    CHECK(f.sink.frames[1].data == bytes(kGga));
    CHECK_EQ(f.parser.stats().junk_bytes, 0u);
  }
}

TEST(parser_nmea_complete_at_checksum) {
  Fixture f;
  std::string full = nmea("GPZDA,1");
  f.feed(full.substr(0, full.size() - 2));  // no terminator yet
  CHECK_EQ(f.sink.frames.size(), 1u);       // delivered already
  CHECK(!f.parser.in_frame());
  f.feed(kAckAck);
  CHECK_EQ(f.sink.frames.size(), 2u);
  CHECK(f.sink.frames[1].type == FrameType::Ubx);
}

TEST(parser_nmea_malformed_checksum_dropped) {
  Fixture f;
  f.feed("$GPZDA,1*4\r\n");   // one hex digit
  f.feed("$GPZDA,1*ZZ\r\n");  // not hex
  f.feed("$GPZDA,1*00\r\n");  // wrong value
  CHECK(f.sink.frames.empty());
  CHECK_EQ(f.parser.stats().bad_checksum, 3u);
  f.feed(kGga);
  CHECK_EQ(f.sink.frames.size(), 1u);
}

TEST(parser_ubx_reference_and_bad_checksum) {
  Fixture f;
  f.feed(kAckAck);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Ubx);
  CHECK(f.sink.frames[0].data == kAckAck);

  std::vector<uint8_t> bad = kAckAck;
  bad.back() ^= 1;
  f.feed(bad);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK_EQ(f.parser.stats().bad_checksum, 1u);
}

TEST(parser_ubx_payload_with_sync_bytes_inside) {
  Fixture f;
  std::vector<uint8_t> frame = ubx(0x02, 0x15, {0xB5, 0x62, 0xD3, '$', '\n', 0x00});
  f.feed(frame);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].data == frame);
}

TEST(parser_rtcm_reference_and_bad_crc) {
  Fixture f;
  f.feed(k1005);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Rtcm3);
  CHECK(f.sink.frames[0].data == k1005);

  std::vector<uint8_t> bad = k1005;
  bad[10] ^= 0x10;
  f.feed(bad);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK_EQ(f.parser.stats().bad_checksum, 1u);
}

TEST(parser_rtcm_reserved_bits_reject_false_start) {
  Fixture f;
  f.feed(std::vector<uint8_t>{0xD3, 0xFF, 0x01, 0x02});
  CHECK(f.sink.frames.empty());
  f.feed(kAckAck);
  CHECK_EQ(f.sink.frames.size(), 1u);
}

TEST(parser_junk_dropped_between_frames) {
  Fixture f;
  std::vector<uint8_t> stream =
      bytes("xx\x01\x02") + bytes(kGga) + bytes("zz") + kAckAck + std::vector<uint8_t>{0x00, 0xFF} +
      rtcm({1, 2, 3}) + bytes("tail");
  f.feed(stream);
  CHECK_EQ(f.sink.frames.size(), 3u);
  CHECK(f.sink.frames[0].type == FrameType::Nmea);
  CHECK(f.sink.frames[1].type == FrameType::Ubx);
  CHECK(f.sink.frames[2].type == FrameType::Rtcm3);
  CHECK_EQ(f.parser.stats().junk_bytes, 4u + 2u + 2u + 4u);
}

TEST(parser_bytewise_matches_bulk) {
  std::vector<uint8_t> stream = bytes(kGga) + kAckAck + k1005 + bytes(nmea("GPRMC,x"));
  Fixture bulk, bytewise;
  bulk.feed(stream);
  bytewise.feed_bytewise(stream);
  CHECK_EQ(bulk.sink.frames.size(), 4u);
  CHECK_EQ(bytewise.sink.frames.size(), 4u);
  for (size_t i = 0; i < 4; ++i) CHECK(bulk.sink.frames[i].data == bytewise.sink.frames[i].data);
}

TEST(parser_nmea_interrupted_by_new_start) {
  Fixture f;
  f.feed("$GPGGA,partial" + nmea("GPRMC,ok"));
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].data == bytes(nmea("GPRMC,ok")));
}

TEST(parser_nmea_interrupted_by_binary_resyncs) {
  Fixture f;
  f.feed(bytes("$GPGGA,part") + kAckAck);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Ubx);
}

TEST(parser_oversize_frames_dropped) {
  uint8_t small[64];
  Sink sink;
  FrameParser p(small, sizeof small, collect, &sink);
  std::vector<uint8_t> big = ubx(1, 2, std::vector<uint8_t>(100, 0xAA));
  p.feed(big.data(), big.size());
  CHECK(sink.frames.empty());
  CHECK_EQ(p.stats().oversize, 1u);

  std::string longnmea = "$" + std::string(300, 'A') + "\r\n";
  p.feed(reinterpret_cast<const uint8_t*>(longnmea.data()), longnmea.size());
  CHECK(sink.frames.empty());
  CHECK_EQ(p.stats().oversize, 2u);

  p.feed(kAckAck.data(), kAckAck.size());
  CHECK_EQ(sink.frames.size(), 1u);
}

TEST(parser_abort_partial_recovers_from_bad_length) {
  Fixture f;
  // UBX header for a 1000-byte payload, then nothing.
  f.feed(std::vector<uint8_t>{0xB5, 0x62, 0x01, 0x07, 0xE8, 0x03, 1, 2, 3});
  CHECK(f.parser.in_frame());
  f.parser.abort_partial();
  CHECK(!f.parser.in_frame());
  CHECK_EQ(f.parser.stats().timeouts, 1u);
  CHECK_EQ(f.parser.stats().junk_bytes, 9u);
  f.feed(kGga);
  CHECK_EQ(f.sink.frames.size(), 1u);
  // no-op when idle
  f.parser.abort_partial();
  CHECK_EQ(f.parser.stats().timeouts, 1u);
}

TEST(parser_partial_len_tracks_frame_in_progress) {
  Fixture f;
  CHECK_EQ(f.parser.partial_len(), 0u);
  f.feed(std::vector<uint8_t>{0xB5});
  CHECK(f.parser.partial_len() >= 1u);
  f.feed(std::vector<uint8_t>(kAckAck.begin() + 1, kAckAck.begin() + 6));
  CHECK_EQ(f.parser.partial_len(), 6u);
  f.feed(std::vector<uint8_t>(kAckAck.begin() + 6, kAckAck.end()));
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK_EQ(f.parser.partial_len(), 0u);
  std::string s = nmea("GPZDA,1");
  f.feed(s.substr(0, 4));
  CHECK_EQ(f.parser.partial_len(), 4u);
  f.feed(s.substr(4));  // done at "*hh"
  CHECK_EQ(f.parser.partial_len(), 0u);
}

TEST(parser_text_inside_ubx_is_not_a_sentence) {
  Fixture f;
  // UBX payload that is a valid NMEA sentence.
  std::string cmd = nmea("PVDB,SOFPPS,PING");
  std::vector<uint8_t> frame = {0xB5, 0x62, 0x02, 0x10, static_cast<uint8_t>(cmd.size()), 0x00};
  frame.insert(frame.end(), cmd.begin(), cmd.end());
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < frame.size(); ++i) {
    a = static_cast<uint8_t>(a + frame[i]);
    b = static_cast<uint8_t>(b + a);
  }
  frame.push_back(a);
  frame.push_back(b);
  f.feed(frame);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Ubx);
  CHECK(f.sink.frames[0].data == frame);
}

TEST(parser_ubx_up_to_buffer_size_accepted) {
  Fixture f;
  std::vector<uint8_t> big = ubx(0x02, 0x15, std::vector<uint8_t>(3000, 0x11));
  std::vector<uint8_t> full = ubx(0x02, 0x15, std::vector<uint8_t>(sizeof f.buf - 8, 0x22));
  f.feed(big + full + kAckAck);
  CHECK_EQ(f.sink.frames.size(), 3u);
  CHECK(f.sink.frames[0].data == big);
  CHECK(f.sink.frames[1].data == full);
  CHECK(f.sink.frames[2].data == kAckAck);
  CHECK_EQ(f.parser.stats().oversize, 0u);
}

TEST(parser_oversize_ubx_skipped_by_declared_length) {
  uint8_t small[64];
  Sink sink;
  FrameParser p(small, sizeof small, collect, &sink);
  // Payload ends with a fake frame start.
  std::vector<uint8_t> payload(100, 0xAA);
  const std::vector<uint8_t> fake = {'$', 0xB5, 0x62, 0x01, 0x02, 40, 0x00};
  std::copy(fake.begin(), fake.end(), payload.end() - fake.size());
  std::vector<uint8_t> big = ubx(1, 2, payload);
  std::vector<uint8_t> stream = big + kAckAck + bytes(nmea("GPZDA,1"));
  p.feed(stream.data(), stream.size());
  CHECK_EQ(sink.frames.size(), 2u);
  CHECK(sink.frames[0].data == kAckAck);
  CHECK(sink.frames[1].type == FrameType::Nmea);
  CHECK_EQ(p.stats().oversize, 1u);
  CHECK_EQ(p.stats().junk_bytes, big.size());
}

TEST(parser_oversize_rtcm_skipped_by_declared_length) {
  uint8_t small[64];
  Sink sink;
  FrameParser p(small, sizeof small, collect, &sink);
  std::vector<uint8_t> payload(200, 0x00);
  payload[190] = 0xB5;
  payload[191] = 0x62;
  std::vector<uint8_t> stream = rtcm(payload) + kAckAck;
  p.feed(stream.data(), stream.size());
  CHECK_EQ(sink.frames.size(), 1u);
  CHECK(sink.frames[0].data == kAckAck);
  CHECK_EQ(p.stats().oversize, 1u);
}

TEST(parser_oversize_skip_ended_by_abort_partial) {
  Fixture f;
  // Header claims 65535 bytes, then the stream stops.
  f.feed(std::vector<uint8_t>{0xB5, 0x62, 0x01, 0x07, 0xFF, 0xFF, 1, 2, 3});
  CHECK_EQ(f.parser.stats().oversize, 1u);
  CHECK(f.parser.in_frame());
  CHECK(f.parser.partial_len() > 0u);
  f.parser.abort_partial();
  CHECK(!f.parser.in_frame());
  CHECK_EQ(f.parser.stats().timeouts, 1u);
  f.feed(kGga);
  CHECK_EQ(f.sink.frames.size(), 1u);
  CHECK(f.sink.frames[0].type == FrameType::Nmea);
}

TEST(parser_nmea_longest_sentence_fits_buffer) {
  for (size_t cap : {size_t{16}, FrameParser::kMaxNmea}) {
    uint8_t buf[300];
    memset(buf, 0xEE, sizeof buf);
    Sink sink;
    FrameParser p(buf, cap, collect, &sink);
    std::string fits = nmea(std::string(cap - 6, 'A'));
    std::string over = nmea(std::string(cap - 5, 'A'));
    CHECK_EQ(fits.size(), cap);
    p.feed(reinterpret_cast<const uint8_t*>(fits.data()), fits.size());
    p.feed(reinterpret_cast<const uint8_t*>(over.data()), over.size());
    CHECK_EQ(sink.frames.size(), 1u);
    CHECK(sink.frames[0].data == bytes(fits));
    CHECK_EQ(p.stats().oversize, 1u);
    CHECK_EQ(buf[cap], 0xEE);  // nothing written past the buffer
  }
}

TEST(parser_reset_discards_partial) {
  Fixture f;
  f.feed(std::vector<uint8_t>(kAckAck.begin(), kAckAck.begin() + 5));
  f.parser.reset();
  f.feed(kAckAck);
  CHECK_EQ(f.sink.frames.size(), 1u);
}

TEST(parser_declared_len_and_frame_age_limit) {
  Fixture f;
  CHECK_EQ(f.parser.declared_len(), FrameParser::kMaxNmea);
  // 3000-byte payload
  f.feed(std::vector<uint8_t>{0xB5, 0x62, 0x02, 0x15, 0xB8, 0x0B});
  CHECK(f.parser.in_frame());
  CHECK_EQ(f.parser.declared_len(), 6u + 3000 + 2);
  f.parser.abort_partial();
  f.feed(std::string("$GPZDA"));
  CHECK_EQ(f.parser.declared_len(), FrameParser::kMaxNmea);
  // 1.067 s on the wire, +25%, +100 ms
  CHECK_EQ(FrameParser::max_frame_us(4096, 38400, 100000), 1066666u + 266666 + 100000);
  CHECK_EQ(FrameParser::max_frame_us(82, 9600, 100000), 85416u + 21354 + 100000);
  CHECK(FrameParser::max_frame_us(256, 0, 100000) > 8000000u);
}
