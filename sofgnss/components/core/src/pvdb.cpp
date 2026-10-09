#include "sofgnss/pvdb.h"

#include <cinttypes>
#include <cstdio>
#include <cstring>

namespace sofgnss::pvdb {

namespace {

constexpr char kPrefix[] = "$PVDB,SOFPPS,";
constexpr size_t kPrefixLen = sizeof(kPrefix) - 1;

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

}  // namespace

uint8_t checksum(const char* body, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) sum ^= static_cast<uint8_t>(body[i]);
  return sum;
}

size_t format(char* out, size_t cap, const char* body) {
  size_t body_len = strlen(body);
  size_t total = 1 + body_len + 3 + 2;  // '$' body "*hh" "\r\n"
  if (total + 1 > cap) return 0;
  out[0] = '$';
  memcpy(out + 1, body, body_len);
  snprintf(out + 1 + body_len, cap - 1 - body_len, "*%02X\r\n", checksum(body, body_len));
  return total;
}

size_t format_pps(char* out, size_t cap, uint64_t tick, uint32_t rx_idle_ms) {
  char body[64];
  snprintf(body, sizeof body, "PVDB,SOFPPS,PPS,%" PRIu64 ",%" PRIu32, tick, rx_idle_ms);
  return format(out, cap, body);
}

size_t format_sof(char* out, size_t cap, uint64_t tick, uint64_t frame) {
  char body[96];
  snprintf(body, sizeof body, "PVDB,SOFPPS,SOF,%" PRIu64 ",%" PRIu64, tick, frame);
  return format(out, cap, body);
}

size_t format_err(char* out, size_t cap, const char* text) {
  char body[96];
  snprintf(body, sizeof body, "PVDB,SOFPPS,ERR,%s", text);
  return format(out, cap, body);
}

size_t format_info(char* out, size_t cap, const Info& info) {
  char body[140];
  snprintf(body, sizeof body,
           "PVDB,SOFPPS,INFO,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32
           ",%" PRIu32 ",%s,%" PRIu32,
           info.tick_hz, info.divider, info.pps, info.sof_tags, info.sof_missed, info.dropped,
           info.parse_errors, info.tag ? info.tag : "", info.pps_rejected);
  return format(out, cap, body);
}

bool valid_tag(const char* tag, size_t len) {
  if (len == 0 || len > kMaxTag) return false;
  for (size_t i = 0; i < len; ++i) {
    char c = tag[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              c == '.' || c == '_' || c == '-';
    if (!ok) return false;
  }
  return true;
}

size_t format_usb(char* out, size_t cap, const UsbInfo& u) {
  char body[96];
  snprintf(body, sizeof body, "PVDB,SOFPPS,USB,%d,%d,%d,%" PRIu32 ",%" PRIu32 ",%" PRIu32, u.mounted,
           u.suspended, u.dtr, u.line_coding_events, u.line_state_events, u.baud);
  return format(out, cap, body);
}

size_t format_cal(char* out, size_t cap, uint64_t tick, const char* mode, bool pps_synced,
                  uint32_t n) {
  char body[96];
  snprintf(body, sizeof body, "PVDB,SOFPPS,CAL,%" PRIu64 ",%s,%d,%" PRIu32, tick, mode,
           pps_synced, n);
  return format(out, cap, body);
}

size_t format_ver(char* out, size_t cap, const char* version, const char* idf,
                  const char* partition, bool pending_verify, const char* elf_sha256,
                  const char* reset_reason) {
  char body[180];
  snprintf(body, sizeof body, "PVDB,SOFPPS,VER,%s,%s,%s,%d,%s,%s", version, idf, partition,
           pending_verify, elf_sha256, reset_reason);
  return format(out, cap, body);
}

size_t format_ota(char* out, size_t cap, const char* status) {
  char body[64];
  snprintf(body, sizeof body, "PVDB,SOFPPS,OTA,%s", status);
  return format(out, cap, body);
}

size_t format_ota(char* out, size_t cap, const char* status, uint32_t value) {
  char body[64];
  snprintf(body, sizeof body, "PVDB,SOFPPS,OTA,%s,%" PRIu32, status, value);
  return format(out, cap, body);
}

bool verify(const char* s, size_t len) {
  while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n')) len--;
  if (len < 4 || s[0] != '$' || s[len - 3] != '*') return false;
  int hi = hex_value(s[len - 2]);
  int lo = hex_value(s[len - 1]);
  if (hi < 0 || lo < 0) return false;
  return checksum(s + 1, len - 4) == static_cast<uint8_t>((hi << 4) | lo);
}

bool command_fields(const uint8_t* frame, size_t len, const char** fields, size_t* fields_len) {
  constexpr size_t kTail = 5;  // "*hh\r\n"
  if (len < kPrefixLen + kTail || memcmp(frame, kPrefix, kPrefixLen) != 0) return false;
  *fields = reinterpret_cast<const char*>(frame) + kPrefixLen;
  *fields_len = len - kPrefixLen - kTail;
  return true;
}

}  // namespace sofgnss::pvdb
