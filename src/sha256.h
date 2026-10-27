#ifndef SHA256_H_
#define SHA256_H_

/*
 * Small streaming SHA-256 (FIPS 180-4), used to verify firmware uploads.
 * Pure C so it also runs in the host tests.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t state[8];
    uint64_t length;        // Bytes processed
    uint8_t block[64];
    size_t block_len;
} sha256_ctx_t;

void sha256_init(sha256_ctx_t * ctx);
void sha256_update(sha256_ctx_t * ctx, const void * data, size_t len);
void sha256_final(sha256_ctx_t * ctx, uint8_t digest[32]);
void sha256_to_hex(const uint8_t digest[32], char hex[65]);

#ifdef __cplusplus
}
#endif

#endif  // SHA256_H_
