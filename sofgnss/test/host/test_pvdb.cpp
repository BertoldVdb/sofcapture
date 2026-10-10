#include <cstring>
#include <string>
#include <vector>

#include "check.h"
#include "sofgnss/pvdb.h"

using namespace sofgnss;

namespace {

std::string with_checksum(const std::string& body) {
  char tail[8];
  snprintf(tail, sizeof tail, "*%02X\r\n", pvdb::checksum(body.data(), body.size()));
  return "$" + body + tail;
}

}  // namespace

TEST(pvdb_checksum_reference) {
  const char body[] = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
  CHECK_EQ(pvdb::checksum(body, strlen(body)), 0x47);
}

TEST(pvdb_format_pps_and_sof) {
  char s[pvdb::kMaxSentence];
  size_t n = pvdb::format_pps(s, sizeof s, 123456789012ull, 0);
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,PPS,123456789012,0"));
  CHECK(pvdb::verify(s, n));
  n = pvdb::format_pps(s, sizeof s, 5, 4294967295u);
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,PPS,5,4294967295"));
  CHECK(pvdb::verify(s, n));

  n = pvdb::format_sof(s, sizeof s, 0xFFFFFFFFFFull, 987654321ull);
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,SOF,1099511627775,987654321"));
  CHECK(pvdb::verify(s, n));
}

TEST(pvdb_format_cal) {
  char s[pvdb::kMaxSentence];
  size_t n =
      pvdb::format_cal(s, sizeof s, 18446744073709551615ull, "NOISE", true, 7, 72000000, 524309);
  CHECK(std::string(s, n) ==
        with_checksum("PVDB,SOFPPS,CAL,18446744073709551615,NOISE,1,7,72000000,524309"));
  CHECK(pvdb::verify(s, n));
}

TEST(pvdb_format_info_and_err) {
  char s[pvdb::kMaxSentence];
  pvdb::Info info{80000000, 100, 1, 2, 3, 4, 5, nullptr, 0};
  size_t n = pvdb::format_info(s, sizeof s, info);
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,INFO,80000000,100,1,2,3,4,5,,0"));
  info.tag = "req-42";
  info.pps_rejected = 6;
  n = pvdb::format_info(s, sizeof s, info);
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,INFO,80000000,100,1,2,3,4,5,req-42,6"));
  // longest possible INFO
  info = {4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u,
          4294967295u, "0123456789abcdef", 4294967295u};
  n = pvdb::format_info(s, sizeof s, info);
  CHECK(std::string(s, n) ==
        with_checksum("PVDB,SOFPPS,INFO,4294967295,4294967295,4294967295,4294967295,4294967295,"
                      "4294967295,4294967295,0123456789abcdef,4294967295"));
  n = pvdb::format_err(s, sizeof s, "BADDIV");
  CHECK(std::string(s, n) == with_checksum("PVDB,SOFPPS,ERR,BADDIV"));
}

TEST(pvdb_valid_tag) {
  CHECK(pvdb::valid_tag("a", 1));
  CHECK(pvdb::valid_tag("Req_1.2-x", 9));
  CHECK(pvdb::valid_tag("0123456789abcdef", 16));
  CHECK(!pvdb::valid_tag("", 0));
  CHECK(!pvdb::valid_tag("0123456789abcdefg", 17));
  CHECK(!pvdb::valid_tag("a,b", 3));
  CHECK(!pvdb::valid_tag("a*b", 3));
  CHECK(!pvdb::valid_tag("a b", 3));
}

TEST(pvdb_format_rejects_small_buffer) {
  char s[8];
  CHECK_EQ(pvdb::format_pps(s, sizeof s, 1, 0), 0u);
}

TEST(pvdb_verify) {
  std::string good = with_checksum("PVDB,SOFPPS,INFO");
  CHECK(pvdb::verify(good.data(), good.size()));
  std::string no_crlf = good.substr(0, good.size() - 2);
  CHECK(pvdb::verify(no_crlf.data(), no_crlf.size()));
  std::string bad = good;
  bad[bad.size() - 3] ^= 1;
  CHECK(!pvdb::verify(bad.data(), bad.size()));
  CHECK(!pvdb::verify("$PVDB,SOFPPS,INFO", 17));
  CHECK(!pvdb::verify("$P*ZZ", 5));
}

TEST(pvdb_command_fields) {
  const char* f;
  size_t n;
  std::string s = with_checksum("PVDB,SOFPPS,DIV,250");
  CHECK(pvdb::command_fields(reinterpret_cast<const uint8_t*>(s.data()), s.size(), &f, &n));
  CHECK(std::string(f, n) == "DIV,250");
  s = with_checksum("PVDB,SOFPPS,PING");
  CHECK(pvdb::command_fields(reinterpret_cast<const uint8_t*>(s.data()), s.size(), &f, &n));
  CHECK(std::string(f, n) == "PING");
  s = with_checksum("PVDB,OTHER,1");
  CHECK(!pvdb::command_fields(reinterpret_cast<const uint8_t*>(s.data()), s.size(), &f, &n));
  s = with_checksum("PUBX,00");
  CHECK(!pvdb::command_fields(reinterpret_cast<const uint8_t*>(s.data()), s.size(), &f, &n));
}
