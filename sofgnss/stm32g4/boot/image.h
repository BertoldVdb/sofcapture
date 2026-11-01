// Flash layout and the signed image header. sign.py writes the same layout.
#pragma once

#include <cstdint>

namespace boot {

constexpr uint32_t kInfoAddr = 0x08006000;  // the header, alone in its pages
constexpr uint32_t kAppAddr = 0x08007000;
constexpr uint32_t kMagic = 0x57464753;  // "SGFW"

struct Header {
  uint32_t magic;
  uint32_t app_addr;
  uint32_t app_len;
  uint32_t desc_off;  // app descriptor, from the app start
  uint8_t reserved[16];
  uint8_t app_sha256[32];
  uint8_t sig[64];  // P-256 r and s over the SHA-256 of the 64 bytes above
};
static_assert(sizeof(Header) == 128, "Header layout");

}  // namespace boot
