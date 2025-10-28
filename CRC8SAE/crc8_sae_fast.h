#pragma once
#include <stdint.h>
#include <stddef.h>
#include "crc8_sae.h"
#include "crc8_sae_fast_lut.h"

// ---- Force-inline macro (GCC/Clang/MSVC) ----
#if defined(_MSC_VER)
  #define ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
  #define ALWAYS_INLINE __attribute__((always_inline)) inline
#else
  #define ALWAYS_INLINE inline
#endif

// ---- SAE-J1850 LUT defined in a separate TU ----
extern const uint8_t crc8_sae_table[256];  // poly = 0x1D, MSB-first

// ---- Core CRC-8/SAE-J1850 (LUT-based) ----
// Fixed by spec: init=0xFF, poly=0x1D (poly arg kept only for signature parity)
static ALWAYS_INLINE bool check_crc8_generic_fast(const uint8_t* data,
                                             uint8_t length,
                                             uint8_t xorout,
                                             uint8_t init = 0xFF,
                                             uint8_t /*poly*/ = 0x1D) {
  if (!data || length < 2) return false;

  uint8_t crc = init;
  for (uint8_t j = 0; j < (uint8_t)(length - 1); ++j) {
    crc = crc8_sae_table[crc ^ data[j]];
  }
  crc ^= xorout;
  return data[length - 1] == crc;
}

// ---- Lookup wrapper (skip check if ID not in table) ----
// If ID is found → validate CRC (expects last byte to be CRC).
// If ID is NOT found → skip CRC and return true (frame assumed to carry no CRC).
static ALWAYS_INLINE bool check_crc8_lookup_fast(uint32_t id,
                                            const uint8_t* data,
                                            uint8_t length,
                                            const CrcParams* table,
                                            size_t table_len,
                                            uint8_t global_init = 0xFF,
                                            uint8_t poly        = 0x1D) {
  (void)poly; // fixed polynomial for SAE-J1850

  const CrcParams* p = nullptr;
  if (table && table_len) {
    for (size_t i = 0; i < table_len; ++i) {
      if (table[i].id == id) { p = &table[i]; break; }
    }
  }

  if (!p) return true;            // Unknown ID → no CRC expected
  if (!data || length < 2) return false;

  return check_crc8_generic_fast(data, length, p->xorout, global_init);
}
