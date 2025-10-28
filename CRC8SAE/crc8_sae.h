// crc8_sae.h
#pragma once
#include <stdint.h>
#include <stddef.h>

// ---- CRC parameter struct (user-provided table entry) ----
struct CrcParams {
  uint32_t id;     // CAN ID
  uint8_t xorout;  // final XOR value (may vary per ID)
};

// ---- Core CRC-8 checker (MSB-first, tail CRC) ----
// Computes CRC over data[0 .. length-2], compares to data[length-1].
// Parameterized by xorout, init, and polynomial.
//
// For SAE-J1850 (fixed by spec):
//   poly   = 0x1D
//   init   = 0xFF
//   no reflection (MSB-first)
static inline bool check_crc8_generic(const uint8_t* data,
                                      uint8_t length,
                                      uint8_t xorout,
                                      uint8_t init = 0xFF,
                                      uint8_t poly = 0x1D) {
  if (!data || length < 2) return false;

  uint8_t crc = init;
  for (uint8_t j = 0; j < (uint8_t)(length - 1); ++j) {
    crc ^= data[j];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ poly);
      else
        crc <<= 1;
    }
  }
  crc ^= xorout;
  return data[length - 1] == crc;
}

// ---- Lookup wrapper: selects xorout by CAN ID ----
// Behavior:
// - If ID is found in 'table': perform CRC check (requires length >= 2).
// - If ID is NOT found: SKIP CRC check and RETURN true (frame expected to have no CRC).
static inline bool check_crc8_lookup(uint32_t id,
                                     const uint8_t* data,
                                     uint8_t length,
                                     const CrcParams* table,
                                     size_t table_len,
                                     uint8_t global_init = 0xFF,
                                     uint8_t poly = 0x1D) {
  const CrcParams* p = nullptr;
  if (table && table_len) {
    for (size_t i = 0; i < table_len; ++i) {
      if (table[i].id == id) { p = &table[i]; break; }
    }
  }

  // ID not in table: no CRC expected for this frame — do not fail the check.
  if (!p) return true;

  // ID found: validate CRC over payload (CRC byte expected as last byte).
  if (!data || length < 2) return false;
  return check_crc8_generic(data, length, p->xorout, global_init, poly);
}
