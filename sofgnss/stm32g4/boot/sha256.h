// SHA-256 (FIPS 180-4), small rather than fast.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t h[8];
  uint64_t len;  // bytes
  uint8_t buf[64];
  uint32_t used;
} sha256_ctx;

void sha256_init(sha256_ctx* c);
void sha256_update(sha256_ctx* c, const void* data, size_t len);
void sha256_final(sha256_ctx* c, uint8_t out[32]);

#ifdef __cplusplus
}
#endif
