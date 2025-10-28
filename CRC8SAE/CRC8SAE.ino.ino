#include <ACAN_T4.h>
#include "crc8_sae_learner.h"
#include "crc8_sae_user_example.h"       // normal validate: example_function(...)
#include "crc8_sae_user_example_fast.h"  // fast LUT validate: example_function_fast(...)
#include "crc8_sae_fast_helper.h"        // print_crc8_table(...)

// ======= Define ALL extern'd globals inside CRCDetectFlex (matches the header) =======
namespace CRCDetectFlex {
  bool         g_learning_enabled = true;

  CRCPlacement g_place            = P_TAIL;  // HEAD or TAIL

  bool         g_poly_locked      = true;    // true => use g_poly; false => only valid if g_poly_search_all is true
  uint8_t      g_poly             = 0x1D;    // default J1850 polynomial (MSB-first, 0x1D)
  bool         g_poly_search_all  = false;   // true => sweep 0x01..0xFF (used only if !g_poly_locked)

  bool         g_init_locked      = true;    // true => use g_init; false => only valid if g_init_search_all is true
  uint8_t      g_init             = 0xFF;    // common automotive init
  bool         g_init_search_all  = false;   // true => sweep 0x00..0xFF (used only if !g_init_locked)
}

// ======= Run modes =======
enum RunMode : uint8_t {
  MODE_LEARN = 0,
  MODE_VALIDATE_NORMAL = 1,
  MODE_VALIDATE_FAST   = 2,
  MODE_BENCH           = 3
};
static RunMode g_mode = MODE_LEARN;

// ======= Bench config =======
static uint32_t g_bench_iters = 1000; // iterations per frame for high-res timing

// ======= High-resolution timing (Teensy 4.x macros) =======
#if defined(__IMXRT1062__)
// Teensy 4.x (Cortex-M7 @ 600 MHz typical)
static inline void dwt_enable_cycle_counter() {
  ARM_DEMCR |= ARM_DEMCR_TRCENA;            // enable tracing
  ARM_DWT_CYCCNT = 0;                       // reset counter
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;   // enable cycle counter
}
static inline uint32_t dwt_read_cycles() {
  return ARM_DWT_CYCCNT;
}
static inline uint64_t cycles_to_ns(uint32_t cycles) {
  const uint32_t hz = F_CPU_ACTUAL; // current CPU frequency
  return (uint64_t)cycles * 1000000000ULL / (uint64_t)hz;
}
#else
// Fallback for non-Teensy builds: use micros() (lower resolution)
static inline void dwt_enable_cycle_counter() {}
static inline uint32_t dwt_read_cycles() { return micros(); }
static inline uint64_t cycles_to_ns(uint32_t us) { return (uint64_t)us * 1000ULL; }
#endif

// ======= Console helpers =======
static void printHelp();
static void printHowTo();

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { String out = buf; buf = ""; return out; }
    buf += c;
  }
  return String();
}

static bool parseHexU8(const String& s, uint8_t* out) {
  if (!out) return false;
  String t = s; t.trim(); t.toUpperCase();
  if (t.startsWith("0X")) t = t.substring(2);
  char *endp = nullptr;
  long v = strtol(t.c_str(), &endp, 16);
  if (endp == t.c_str()) return false; // no digits
  if (v < 0 || v > 255) return false;
  *out = (uint8_t)v;
  return true;
}

static void printFrameOneLine(const CANMessage& f, const char* prefix) {
  Serial.print(prefix);
  Serial.print(" id=0x"); Serial.print(f.id, HEX);
  Serial.print(" len=");  Serial.print(f.len);
  Serial.print(" data=");
  for (uint8_t i = 0; i < f.len; ++i) {
    if (i) Serial.print(' ');
    uint8_t b = f.data[i];
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
  }
  Serial.println();
}

static void handleCommand(const String& line) {
  String cmd = line; cmd.trim();

  // Normalize for parsing; keep original for values after keywords
  String lower = cmd; lower.toLowerCase();

  if (!lower.length()) return;

  if (lower == "help" || lower == "?") { printHelp(); return; }
  if (lower == "howto" || lower == "guide" || lower == "usage") { printHowTo(); return; }

  // --- Modes ---
  if (lower == "mode learn") {
    g_mode = MODE_LEARN;
    CRCDetectFlex::g_learning_enabled = true;
    Serial.println("Mode: LEARN (learning enabled; no mismatch prints).");
    return;
  }
  if (lower == "mode validate") {
    g_mode = MODE_VALIDATE_NORMAL;
    CRCDetectFlex::g_learning_enabled = false;
    Serial.println("Mode: VALIDATE (normal example_function; prints failed frames).");
    return;
  }
  if (lower == "mode validate_fast") {
    g_mode = MODE_VALIDATE_FAST;
    CRCDetectFlex::g_learning_enabled = false;
    Serial.println("Mode: VALIDATE_FAST (LUT example_function_fast; prints failed frames).");
    return;
  }
  if (lower == "mode bench") {
    g_mode = MODE_BENCH;
    CRCDetectFlex::g_learning_enabled = false;
    Serial.println("Mode: BENCH (high-resolution cycles; averaged over iterations).");
    return;
  }

  // --- Bench config ---
  if (lower.startsWith("bench iters ")) {
    String arg = cmd.substring(12); arg.trim();
    long v = strtol(arg.c_str(), nullptr, 10);
    if (v < 1) v = 1;
    if (v > 1000000L) v = 1000000L;
    g_bench_iters = (uint32_t)v;
    Serial.print("Bench iterations set to "); Serial.println(g_bench_iters);
    return;
  }

  // --- Info / Export ---
  if (lower == "status") { CRCDetectFlex::print_status(); return; }
  if (lower == "export") { CRCDetectFlex::print_export_table(); return; }
  if (lower == "export fast") { CRCDetectFlex::print_export_table_fast(); return; }

  // --- Learning controls (effective only in LEARN mode) ---
  if (lower == "learn start") { CRCDetectFlex::g_learning_enabled = true;  Serial.println("Learning ENABLED.");  return; }
  if (lower == "learn stop")  { CRCDetectFlex::g_learning_enabled = false; Serial.println("Learning DISABLED."); return; }
  if (lower == "learn reset") { CRCDetectFlex::reset_learning(); return; }

  // --- Placement / params (for learner) ---
  if (lower.startsWith("set place ")) {
    String arg = cmd.substring(10); arg.trim(); arg.toUpperCase();
    if (arg == "TAIL") { CRCDetectFlex::g_place = CRCDetectFlex::P_TAIL; Serial.println("placement=TAIL"); }
    else if (arg == "HEAD") { CRCDetectFlex::g_place = CRCDetectFlex::P_HEAD; Serial.println("placement=HEAD"); }
    else Serial.println("Use: set place TAIL|HEAD");
    return;
  }

  if (lower.startsWith("set poly ")) {
    String arg = cmd.substring(9); arg.trim();
    if (arg.equalsIgnoreCase("ALL")) {
      CRCDetectFlex::g_poly_locked = false; CRCDetectFlex::g_poly_search_all = true;
      Serial.println("poly=SEARCH-ALL (0x01..0xFF)");
    } else {
      uint8_t v;
      if (!parseHexU8(arg, &v)) { Serial.println("Usage: set poly 0x1D  OR  set poly ALL"); return; }
      CRCDetectFlex::g_poly = v;
      CRCDetectFlex::g_poly_locked = true; CRCDetectFlex::g_poly_search_all = false;
      Serial.print("poly locked to 0x"); Serial.println(CRCDetectFlex::g_poly, HEX);
    }
    return;
  }

  if (lower.startsWith("set init ")) {
    String arg = cmd.substring(9); arg.trim();
    if (arg.equalsIgnoreCase("ALL")) {
      CRCDetectFlex::g_init_locked = false; CRCDetectFlex::g_init_search_all = true;
      Serial.println("init=SEARCH-ALL (0x00..0xFF)");
    } else {
      uint8_t v;
      if (!parseHexU8(arg, &v)) { Serial.println("Usage: set init 0xFF  OR  set init ALL"); return; }
      CRCDetectFlex::g_init = v;
      CRCDetectFlex::g_init_locked = true; CRCDetectFlex::g_init_search_all = false;
      Serial.print("init locked to 0x"); Serial.println(CRCDetectFlex::g_init, HEX);
    }
    return;
  }

  // --- Generate a 256-entry table for a given polynomial (MSB-first) ---
  // Usage: gen table 0x1D
  if (lower.startsWith("gen table ")) {
    String arg = cmd.substring(10); arg.trim();
    uint8_t poly;
    if (!parseHexU8(arg, &poly)) {
      Serial.println("Usage: gen table 0x1D");
      return;
    }
    print_crc8_table(poly);
    return;
  }

  Serial.println("Unknown command. Type 'help'.");
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  help / ?                 - show this help");
  Serial.println("  howto / guide / usage    - wiring + SavvyCAN replay setup (>= 10 ms between frames)");
  Serial.println("  status                   - print learner status & per-ID results (sorted by CAN ID)");
  Serial.println("  export                   - print copy-paste header (crc8_sae_user_example.h) with learned params");
  Serial.println("  export fast              - print copy-paste header (crc8_sae_user_example_fast.h) with learned params");
  Serial.println();
  Serial.println("  mode learn               - enable learner (silent on mismatches)");
  Serial.println("  mode validate            - validate via example_function (normal), print failed frames");
  Serial.println("  mode validate_fast       - validate via example_function_fast (LUT), print failed frames");
  Serial.println("  mode bench               - run high-res benchmark (CPU cycles, averaged)");
  Serial.println("  bench iters N            - set benchmark iterations (default 1000)");
  Serial.println();
  Serial.println("  set place TAIL|HEAD      - CRC position (learner only)");
  Serial.println("  set poly 0x1D|ALL        - lock polynomial or search ALL (learner)");
  Serial.println("  set init 0xFF|ALL        - lock init or search ALL (learner)");
  Serial.println("  learn start|stop         - enable/disable learning (in LEARN mode)");
  Serial.println("  learn reset              - clear learned per-ID state");
  Serial.println();
  Serial.println("  gen table 0xPP           - print 256-entry MSB-first CRC-8 table for polynomial 0xPP");
  Serial.println();
}

static void printHowTo() {
  Serial.println();
  Serial.println("========== HOW TO USE: CRC-8 Learner ==========");
  Serial.println();
  Serial.println("1) Hardware & Wiring (Teensy 4.x + CAN1 @ 500 kbit/s):");
  Serial.println("   • Use a 3.3V-compatible CAN transceiver (e.g. MCP2562FD-*, SN65HVD230, TJA1051T/3).");
  Serial.println("   • Teensy pins (CAN1):  TX=22  RX=23  (Teensy 4.0/4.1)");
  Serial.println("     - Transceiver TXD  <- Teensy pin 22 (CAN1 TX)");
  Serial.println("     - Transceiver RXD  -> Teensy pin 23 (CAN1 RX)");
  Serial.println("     - Power the transceiver: VCC to 3.3V (or 5V if allowed), GND to GND.");
  Serial.println("     - EN/STBY/SLP pins: set to ENABLE state (EN=HIGH, STBY/SLP=LOW) per datasheet.");
  Serial.println("     - CANH/CANL connect to the CAN bus with proper 120Ω termination at BOTH ends.");
  Serial.println("     - Ensure a common ground across all CAN nodes.");
  Serial.println();
  Serial.println("   NOTE: This sketch configures ACAN_T4 for CAN1 @ 500 kbit/s:");
  Serial.println("         ACAN_T4_Settings settings(500 * 1000);");
  Serial.println();
  Serial.println("2) Sending frames with SavvyCAN (REQUIRED timing for learning):");
  Serial.println("   • Connect your CAN adapter (CANable/candleLight/PEAK/etc.) to the SAME CAN bus as the Teensy.");
  Serial.println("   • In SavvyCAN, set the adapter bitrate to 500 kbit/s.");
  Serial.println("   • Use Replay to inject frames from a log/stream:");
  Serial.println("     - Set MINIMUM inter-frame delay to >= 10 ms (VERY IMPORTANT).");
  Serial.println("     - Avoid burst modes; keep realistic pacing.");
  Serial.println("     - Start replay; Teensy listens on the PHYSICAL CAN bus (not USB frames).");
  Serial.println();
  Serial.println("3) Operating modes (type into the serial console):");
  Serial.println("   mode learn            : Enable learner (silent on CRC mismatch).");
  Serial.println("   mode validate         : Validate via example_function (normal). Print failed frames.");
  Serial.println("   mode validate_fast    : Validate via example_function_fast (LUT). Print failed frames.");
  Serial.println("   mode bench            : Run both; high-res timing & mismatch warning.");
  Serial.println();
  Serial.println("4) Learner controls (LEARN mode only):");
  Serial.println("   set place TAIL|HEAD   : Where the CRC byte resides.");
  Serial.println("   set poly 0x1D|ALL     : Lock polynomial or search 0x01..0xFF.");
  Serial.println("   set init 0xFF|ALL     : Lock init or search 0x00..0xFF.");
  Serial.println("   learn start|stop      : Enable/disable learning.");
  Serial.println("   learn reset           : Clear learned per-ID state.");
  Serial.println();
  Serial.println("5) Status / Export / Table:");
  Serial.println("   status                : Show learner status and per-ID results.");
  Serial.println("   export                : Emit header for normal validator.");
  Serial.println("   export fast           : Emit header for LUT validator.");
  Serial.println("   gen table 0xPP        : Print 256-entry MSB-first CRC-8 table.");
  Serial.println();
  Serial.println("6) Typical workflow:");
  Serial.println("   - Wire Teensy + transceiver, verify 120Ω termination.");
  Serial.println("   - Flash this sketch; ensure 'CAN1 OK' at startup.");
  Serial.println("   - In SavvyCAN, replay frames to the bus with >= 10 ms between frames.");
  Serial.println("   - Use 'mode learn' until IDs solve; then 'export' or 'export fast'.");
  Serial.println("   - Switch to 'mode validate'/'mode validate_fast' to verify on live traffic.");
  Serial.println();
  Serial.println("Tips:");
  Serial.println("   • If no frames appear: check bitrate, wiring, common GND, and termination.");
  Serial.println("   • If CRC never solves: try 'set poly ALL' and/or 'set init ALL'.");
  Serial.println("   • If CRC byte is first in payload: use 'set place HEAD'.");
  Serial.println("===============================================");
  Serial.println();
}

// ======= Setup / Loop =======
void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("\nCRC-8 Learner Console ready.");
  printHelp();

  dwt_enable_cycle_counter(); // enable high-resolution timing (Teensy 4.x)

  ACAN_T4_Settings settings(500 * 1000); // 500 kbit/s
  const uint32_t errorCode = ACAN_T4::can1.begin(settings);
  if (0 == errorCode) Serial.println("CAN1 OK");
  else { Serial.print("Error CAN1: 0x"); Serial.println(errorCode, HEX); }
}

void loop() {
  // Serial console
  String line = readLine();
  if (line.length()) handleCommand(line);

  // CAN RX
  while (ACAN_T4::can1.available() > 0) {
    CANMessage rx;
    if (!ACAN_T4::can1.receive(rx)) continue;

    if (g_mode == MODE_LEARN) {
      // Try to validate (if solved) else feed learner — silent on mismatches
      bool ok = CRCDetectFlex::validate(rx);
      if (!ok) CRCDetectFlex::learn_from_frame(rx);
      continue;
    }

    if (g_mode == MODE_VALIDATE_NORMAL) {
      bool ok = example_function(rx);
      if (!ok) printFrameOneLine(rx, "CRC FAIL (normal)");
      continue;
    }

    if (g_mode == MODE_VALIDATE_FAST) {
      bool ok = example_function_fast(rx);
      if (!ok) printFrameOneLine(rx, "CRC FAIL (fast)");
      continue;
    }

    if (g_mode == MODE_BENCH) {
      // Optional warm-up to stabilize I-cache/branch predictor
      (void)example_function(rx);
      (void)example_function_fast(rx);

      volatile uint32_t guard = 0; // prevent over-optimization by the compiler

      // Time normal (g_bench_iters times)
      uint32_t c0 = dwt_read_cycles();
      for (uint32_t i = 0; i < g_bench_iters; ++i) {
        bool ok = example_function(rx);
        guard += ok; // use result
      }
      uint32_t c1 = dwt_read_cycles();
      uint32_t cyc_norm_total = c1 - c0;

      // Time fast (g_bench_iters times)
      uint32_t c2 = dwt_read_cycles();
      for (uint32_t i = 0; i < g_bench_iters; ++i) {
        bool ok = example_function_fast(rx);
        guard += ok;
      }
      uint32_t c3 = dwt_read_cycles();
      uint32_t cyc_fast_total = c3 - c2;

      // Average per-call cycles
      uint32_t denom = (g_bench_iters ? g_bench_iters : 1);
      uint32_t cyc_norm = cyc_norm_total / denom;
      uint32_t cyc_fast = cyc_fast_total / denom;

      uint64_t ns_norm = cycles_to_ns(cyc_norm);
      uint64_t ns_fast = cycles_to_ns(cyc_fast);
      int32_t  cyc_delta = (int32_t)cyc_norm - (int32_t)cyc_fast;
      int64_t  ns_delta  = (int64_t)ns_norm - (int64_t)ns_fast;

      Serial.print("BENCH id=0x"); Serial.print(rx.id, HEX);
      Serial.print(" len="); Serial.print(rx.len);
      Serial.print(" iters="); Serial.print(g_bench_iters);
      Serial.print(" | normal="); Serial.print(cyc_norm); Serial.print(" cyc (");
      Serial.print((unsigned long)ns_norm); Serial.print(" ns)");
      Serial.print(" | fast="); Serial.print(cyc_fast); Serial.print(" cyc (");
      Serial.print((unsigned long)ns_fast); Serial.print(" ns)");
      Serial.print(" | delta="); Serial.print(cyc_delta); Serial.print(" cyc (");
      Serial.print((long)ns_delta); Serial.println(" ns)");

      // Also flag logical mismatches on a single pass (cheap check)
      bool ok_norm = example_function(rx);
      bool ok_fast = example_function_fast(rx);
      if (ok_norm != ok_fast) {
        printFrameOneLine(rx, "RESULT MISMATCH");
      } else if (!ok_norm) {
        printFrameOneLine(rx, "CRC FAIL (both)");
      }
      (void)guard; // silence unused warning
      continue;
    }
  }
}
