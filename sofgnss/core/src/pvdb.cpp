#include "sofgnss/pvdb.h"

#include <cstring>

namespace sofgnss::pvdb {

namespace {

constexpr char kPrefix[] = "$PVDB,SOFPPS,";
constexpr size_t kPrefixLen = sizeof(kPrefix) - 1;
constexpr char kHex[] = "0123456789ABCDEF";

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Builds a sentence in place, without printf. Text that does not fit makes
// finish() return 0.
class Builder {
 public:
  Builder(char* out, size_t cap) : out_(out), cap_(cap) {}

  Builder& put(char c) {
    if (len_ < cap_) out_[len_] = c;
    len_++;
    return *this;
  }
  Builder& str(const char* s) {
    while (*s) put(*s++);
    return *this;
  }
  Builder& num(uint64_t v) {
    char digits[20];
    int n = 0;
    do {
      digits[n++] = static_cast<char>('0' + v % 10);
      v /= 10;
    } while (v);
    while (n) put(digits[--n]);
    return *this;
  }
  Builder& field(const char* s) { return put(',').str(s); }
  Builder& field(uint64_t v) { return put(',').num(v); }

  // Appends "*hh\r\n" and a NUL. Returns the length without the NUL.
  size_t finish() {
    if (len_ + 5 + 1 > cap_) return 0;
    uint8_t sum = checksum(out_ + 1, len_ - 1);
    put('*').put(kHex[sum >> 4]).put(kHex[sum & 15]).put('\r').put('\n');
    out_[len_] = '\0';
    return len_;
  }

 private:
  char* out_;
  size_t cap_;
  size_t len_ = 0;
};

Builder sentence(char* out, size_t cap, const char* type) {
  Builder b(out, cap);
  b.str(kPrefix).str(type);
  return b;
}

}  // namespace

uint8_t checksum(const char* body, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) sum ^= static_cast<uint8_t>(body[i]);
  return sum;
}

size_t format(char* out, size_t cap, const char* body) {
  return Builder(out, cap).put('$').str(body).finish();
}

size_t format_pps(char* out, size_t cap, uint64_t tick, uint32_t rx_idle_ms) {
  return sentence(out, cap, "PPS").field(tick).field(rx_idle_ms).finish();
}

size_t format_sof(char* out, size_t cap, uint64_t tick, uint64_t frame) {
  return sentence(out, cap, "SOF").field(tick).field(frame).finish();
}

size_t format_err(char* out, size_t cap, const char* text) {
  return sentence(out, cap, "ERR").field(text).finish();
}

size_t format_info(char* out, size_t cap, const Info& info) {
  return sentence(out, cap, "INFO")
      .field(info.tick_hz)
      .field(info.divider)
      .field(info.pps)
      .field(info.sof_tags)
      .field(info.sof_missed)
      .field(info.dropped)
      .field(info.parse_errors)
      .field(info.tag ? info.tag : "")
      .field(info.pps_rejected)
      .finish();
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
  return sentence(out, cap, "USB")
      .field(uint64_t{u.mounted})
      .field(uint64_t{u.suspended})
      .field(uint64_t{u.dtr})
      .field(u.line_coding_events)
      .field(u.line_state_events)
      .field(u.baud)
      .finish();
}

size_t format_cal(char* out, size_t cap, uint64_t tick, const char* mode, bool pps_synced,
                  uint32_t n, uint32_t chip_rate, uint32_t length) {
  return sentence(out, cap, "CAL")
      .field(tick)
      .field(mode)
      .field(uint64_t{pps_synced})
      .field(n)
      .field(chip_rate)
      .field(length)
      .finish();
}

size_t format_ver(char* out, size_t cap, const char* version, const char* idf,
                  const char* partition, bool pending_verify, const char* elf_sha256,
                  const char* reset_reason) {
  return sentence(out, cap, "VER")
      .field(version)
      .field(idf)
      .field(partition)
      .field(uint64_t{pending_verify})
      .field(elf_sha256)
      .field(reset_reason)
      .finish();
}

size_t format_ota(char* out, size_t cap, const char* status) {
  return sentence(out, cap, "OTA").field(status).finish();
}

size_t format_ota(char* out, size_t cap, const char* status, uint32_t value) {
  return sentence(out, cap, "OTA").field(status).field(value).finish();
}

bool parse_u32(const char* s, size_t len, uint32_t* out) {
  if (len == 0) return false;
  uint64_t v = 0;
  for (size_t i = 0; i < len; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    v = v * 10 + static_cast<uint64_t>(s[i] - '0');
    if (v > 0xFFFFFFFFu) return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

void hex(char* out, const uint8_t* data, size_t len) {
  static constexpr char kLower[] = "0123456789abcdef";
  for (size_t i = 0; i < len; ++i) {
    out[2 * i] = kLower[data[i] >> 4];
    out[2 * i + 1] = kLower[data[i] & 15];
  }
  out[2 * len] = '\0';
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
