#pragma once
#include <ACAN_T4.h>
#include "crc8_sae_fast.h"   // brings in check_crc8_lookup_fast + extern crc8_table[256]

// ---- User-defined per-ID CRC xorout table (learned) ----
static const CrcParams kCrcTable_User_Fast[] = {
  {0x081, 0x60},
  {0x082, 0xCF},
  {0x083, 0xAA},
  {0x084, 0x8C},
  {0x085, 0xE9},
  {0x100, 0xAA},
  {0x101, 0xCF},
  {0x102, 0x60},
  {0x105, 0x46},
  {0x106, 0xE9},
  {0x107, 0x8C},
  {0x123, 0x39},
  {0x125, 0x7A},
  {0x126, 0xD5},
  {0x131, 0xED},
  {0x133, 0x27},
  {0x135, 0x64},
  {0x136, 0xCB},
  {0x141, 0xB7},
  {0x142, 0x18},
  {0x143, 0x7D},
  {0x145, 0x3E},
  {0x151, 0xA9},
  {0x153, 0x63},
  {0x155, 0x20},
  {0x161, 0x8B},
  {0x162, 0x24},
  {0x165, 0x02},
  {0x170, 0xF0},
  {0x171, 0x95},
  {0x172, 0x3A},
  {0x175, 0x1C},
};

// ---- Example function: fast LUT-based CRC check ----
// If ID is known: validate CRC
// If ID is not listed: skip check (assume no CRC)
static inline bool example_function_fast(const CANMessage& frame) {
  if (frame.rtr) return false;        // No data in RTR frames
  if (frame.len > 8) return false;    // Classic CAN limit

  return check_crc8_lookup_fast(
    frame.id,
    frame.data,
    frame.len,
    kCrcTable_User_Fast,
    sizeof(kCrcTable_User_Fast) / sizeof(kCrcTable_User_Fast[0]),
    0xFF, // SAE-J1850 init (fixed by spec)
    0x1D  // SAE-J1850 poly (ignored here; crc8_table must match)
  );
}
