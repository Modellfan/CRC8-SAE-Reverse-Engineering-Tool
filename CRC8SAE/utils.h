#include <ACAN_T4.h>

// ---------- Helper: advanced CAN print (CRC check for ALL frames) ----------
static void printCAN (const CANMessage & frame) {
  // Frame type & ID
  Serial.print(frame.ext ? "EXT " : "STD ");
  Serial.print("ID=0x");
  Serial.print(frame.id, HEX);

  // Flags
  if (frame.rtr) {
    Serial.print(" RTR");
  }

  // DLC and data
  Serial.print(" DLC=");
  Serial.print(frame.len);
  Serial.print(" Data=");
  if (frame.rtr) {
    Serial.print("<remote request>");
  } else {
    for (uint8_t i = 0; i < frame.len; i++) {
      if (i) Serial.print(' ');
      uint8_t b = frame.data[i];
      if (b < 0x10) Serial.print('0');
      Serial.print(b, HEX);
    }
  }

  // Optional timestamp (depends on library build)
  #ifdef ACAN_T4_MESSAGE_HAS_TIMESTAMP
    Serial.print(" TS=");
    Serial.print(frame.timestamp);
  #endif

  // CRC for ALL frames (meaningful only if byte0 is CRC of bytes 1..len-1)
  Serial.print(" | CRC8 ");
  if (!frame.rtr && frame.len >= 2) {
    bool ok = check_crc8_sae_lookup(frame, frame.len);
    Serial.print(ok ? "OK" : "FAIL");
  } else {
    Serial.print("N/A");
  }

  Serial.println();
}

// ---------- Helper: print CRC fault ----------
static void printCRCFault(const CANMessage &frame) {
  Serial.print("CRC FAULT ID=0x");
  Serial.print(frame.id, HEX);
  Serial.print(" Data=");
  for (uint8_t i = 0; i < frame.len; i++) {
    if (i) Serial.print(' ');
    if (frame.data[i] < 0x10) Serial.print('0');
    Serial.print(frame.data[i], HEX);
  }
  Serial.println();
}