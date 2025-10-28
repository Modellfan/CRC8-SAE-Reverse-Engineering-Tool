#pragma once
#include <Arduino.h>  // for Serial

// ---- Generic CRC-8 Lookup Table Generator (MSB-first, no reflection) ----
// Generates and prints a 256-entry lookup table for any 8-bit polynomial.
// Prints C++-style code (each byte formatted as 0x??).
// Call from setup(), e.g. `print_crc8_table(0x1D);`

static void print_crc8_table(uint8_t poly) {
  uint8_t table[256];

  // Build lookup table
  for (int dividend = 0; dividend < 256; ++dividend) {
    uint8_t rem = static_cast<uint8_t>(dividend);
    for (int bit = 0; bit < 8; ++bit) {
      if (rem & 0x80)
        rem = static_cast<uint8_t>((rem << 1) ^ poly);
      else
        rem <<= 1;
    }
    table[dividend] = rem;
  }

  // Print formatted as a valid C++ array
  Serial.println();
  Serial.print("// ---- CRC-8 lookup table (poly 0x");
  if (poly < 0x10) Serial.print("0");
  Serial.print(poly, HEX);
  Serial.println(", MSB-first) ----");
  Serial.println("static const uint8_t crc8_table[256] = {");

  for (int i = 0; i < 256; ++i) {
    if (i % 16 == 0) Serial.print("  ");  // indent rows
    Serial.print("0x");
    if (table[i] < 0x10) Serial.print("0");
    Serial.print(table[i], HEX);
    if (i != 255) Serial.print(", ");
    if (i % 16 == 15) Serial.println();
  }

  Serial.println("};");
  Serial.println("// ---- End of CRC-8 table ----");
}
