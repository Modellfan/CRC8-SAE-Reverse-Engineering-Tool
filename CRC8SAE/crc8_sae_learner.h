#pragma once
#include <ACAN_T4.h>

/***** ==== Flexible CRC-8 Learner with Consensus Locking (poly/init), per-ID mode+xorout ==== *****/
namespace CRCDetectFlex {

// ---------- Generic bitwise CRC-8 (MSB-first, no reflection) ----------
static inline uint8_t crc8_bitwise(const uint8_t *data, std::size_t length,
                                   uint8_t poly, uint8_t init, uint8_t xorout) {
  uint8_t crc = init;
  while (length--) {
    crc ^= *data++;
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ poly);
      else            crc <<= 1;
    }
  }
  return (uint8_t)(crc ^ xorout);
}

// ---------- CRC placement ----------
enum CRCPlacement : uint8_t { P_TAIL = 0, P_HEAD = 1 };

// ---------- Coverage modes ----------
enum CoverMode : uint8_t {
  C_PAY_0_ALL = 0,
  C_PAY_1_ALL,
  C_PAY_2_ALL,
  C_DLC_PAY0,
  C_ID_BE_PAY0,
  C_ID_LE_PAY0,
  C_ID_BE_DLC_PAY0,
  C_ID_LE_DLC_PAY0,
};
static const CoverMode kCoverModes[] = {
  C_PAY_0_ALL, C_PAY_1_ALL, C_PAY_2_ALL, C_DLC_PAY0,
  C_ID_BE_PAY0, C_ID_LE_PAY0, C_ID_BE_DLC_PAY0, C_ID_LE_DLC_PAY0,
};

// ---------- Globals that are defined in your main (.ino) inside namespace CRCDetectFlex ----------
extern bool         g_learning_enabled;

extern CRCPlacement g_place;

extern bool         g_poly_locked;
extern uint8_t      g_poly;
extern bool         g_poly_search_all;

extern bool         g_init_locked;
extern uint8_t      g_init;
extern bool         g_init_search_all;

// ---------- Learner tuning knobs (internal statics; adjust via code if desired) ----------
static uint8_t g_min_samples_per_id = 3;   // require at least S samples before solving an ID
static uint8_t g_consensus_ids      = 2;   // need K IDs agreeing on (poly,init) to lock
static bool    g_unlock_on_conflict = true;// if a solved ID later fails validation, unlock globals

// ---------- Buckets / candidate handling ----------
struct Sample { uint8_t len; uint8_t bytes[8]; };

struct Candidate {
  uint8_t  poly = 0;
  uint8_t  init = 0;
  CoverMode mode = C_PAY_0_ALL;
  uint8_t  xorout = 0;
  bool     valid = false; // true means it validates across all stored samples
};

struct Bucket {
  bool      used   = false;
  uint32_t  id     = 0;
  Sample    s[8];
  uint8_t   count  = 0;

  // final chosen params (when committed per-ID after globals lock)
  bool       solved = false;
  CoverMode  mode;
  uint8_t    xorout = 0;

  // small pool of hypotheses (validated against all samples so far)
  Candidate  cands[4];
};

static Bucket  g_buckets[32]; // up to 32 IDs

// ---------- Forward declarations for hex printers used in announcements ----------
static void print_hex_id(uint32_t id);
static void print_hex_u8(uint8_t v);

// ---------- Announcement helpers ----------
static void announce_new_bucket(uint32_t id) {
  Serial.print("Learning: tracking new ID ");
  print_hex_id(id);
  Serial.println();
}

static void announce_per_id_solved(const Bucket& b, uint8_t poly, uint8_t init) {
  Serial.print("Learned ID ");
  print_hex_id(b.id);
  Serial.print(": poly=");   print_hex_u8(poly);
  Serial.print(", init=");   print_hex_u8(init);
  Serial.print(", mode=");   Serial.print((int)b.mode);
  Serial.print(", xorout="); print_hex_u8(b.xorout);
  Serial.print(", place=");  Serial.println(g_place == P_TAIL ? "TAIL" : "HEAD");
}

// ---------- Helpers ----------
static Bucket* get_bucket(uint32_t id) {
  for (auto &b : g_buckets) if (b.used && b.id == id) return &b;
  for (auto &b : g_buckets) if (!b.used) {
    b.used = true; b.id = id;
    announce_new_bucket(id);          // announce when a new ID is first tracked
    return &b;
  }
  return nullptr;
}

static void add_sample(Bucket* b, const CANMessage& f) {
  const uint8_t idx = b->count % 8;
  b->s[idx].len = f.len;
  for (uint8_t i = 0; i < f.len && i < 8; ++i) b->s[idx].bytes[i] = f.data[i];
  b->count++;
}

static bool has_variation(const Bucket& b) {
  if (b.count < g_min_samples_per_id) return false; // gate by sample count
  const uint8_t N = (b.count < 3) ? b.count : 3;
  if (N < 2) return false;
  for (uint8_t i = 1; i < N; ++i) {
    if (b.s[i].len != b.s[i-1].len) return true;
    const uint8_t cover = (uint8_t)(b.s[i].len ? (b.s[i].len - 1) : 0);
    for (uint8_t k = 0; k < cover && k < 7; ++k) {
      uint8_t bi = (g_place == P_TAIL) ? b.s[i].bytes[k]     : b.s[i].bytes[k+1];
      uint8_t bj = (g_place == P_TAIL) ? b.s[i-1].bytes[k]   : b.s[i-1].bytes[k+1];
      if (bi != bj) return true;
    }
  }
  return false;
}

// Build the covered buffer (exclude CRC at HEAD or TAIL)
static uint8_t build_covered(const CANMessage& f, CoverMode mode, CRCPlacement place, uint8_t *out) {
  uint8_t n = 0;
  auto add = [&](uint8_t b){ out[n++] = b; };

  auto for_payload_excluding_crc_from = [&](uint8_t from_index){
    if (place == P_TAIL) {
      const uint8_t crc_pos = f.len - 1;
      for (uint8_t i = from_index; i < crc_pos; ++i) add(f.data[i]);
    } else {
      // HEAD: CRC at index 0; payload starts at 1
      for (uint8_t i = (uint8_t)(from_index ? from_index+1 : 1); i < f.len; ++i) add(f.data[i]);
    }
  };

  switch (mode) {
    case C_PAY_0_ALL:      { for_payload_excluding_crc_from(0); } break;
    case C_PAY_1_ALL:      { for_payload_excluding_crc_from(1); } break;
    case C_PAY_2_ALL:      { for_payload_excluding_crc_from(2); } break;
    case C_DLC_PAY0:       { add(f.len); for_payload_excluding_crc_from(0); } break;
    case C_ID_BE_PAY0:     { add((uint8_t)(f.id >> 8)); add((uint8_t)(f.id & 0xFF)); for_payload_excluding_crc_from(0); } break;
    case C_ID_LE_PAY0:     { add((uint8_t)(f.id & 0xFF)); add((uint8_t)(f.id >> 8)); for_payload_excluding_crc_from(0); } break;
    case C_ID_BE_DLC_PAY0: { add((uint8_t)(f.id >> 8)); add((uint8_t)(f.id & 0xFF)); add(f.len); for_payload_excluding_crc_from(0); } break;
    case C_ID_LE_DLC_PAY0: { add((uint8_t)(f.id & 0xFF)); add((uint8_t)(f.id >> 8)); add(f.len); for_payload_excluding_crc_from(0); } break;
  }
  return n;
}

static uint8_t observed_crc_byte(const CANMessage& f, CRCPlacement place) {
  return (place == P_TAIL) ? f.data[f.len - 1] : f.data[0];
}

static bool try_mode_and_solve(const Bucket& b, uint8_t poly, uint8_t init,
                               CRCPlacement place, CoverMode mode, uint8_t* out_xorout) {
  if (b.count == 0) return false;

  // Derive xorout from first sample
  CANMessage f{}; f.id = b.id; f.len = b.s[0].len;
  for (uint8_t i = 0; i < f.len && i < 8; ++i) f.data[i] = b.s[0].bytes[i];
  if (f.len < 2) return false;

  uint8_t buf[20];
  const uint8_t m = build_covered(f, mode, place, buf);
  const uint8_t obs0 = observed_crc_byte(f, place);

  uint8_t xo = (uint8_t)(crc8_bitwise(buf, m, poly, init, 0x00) ^ obs0);

  // Verify across all available samples (up to 8)
  const uint8_t N = (b.count < 8) ? b.count : 8;
  for (uint8_t i = 0; i < N; ++i) {
    CANMessage fi{}; fi.id = b.id; fi.len = b.s[i].len;
    for (uint8_t k = 0; k < fi.len && k < 8; ++k) fi.data[k] = b.s[i].bytes[k];
    if (fi.len < 2) return false;

    uint8_t bufi[20];
    const uint8_t mi = build_covered(fi, mode, place, bufi);
    const uint8_t calc = crc8_bitwise(bufi, mi, poly, init, xo);
    if (calc != observed_crc_byte(fi, place)) return false;
  }
  *out_xorout = xo;
  return true;
}

static void add_or_update_candidate(Bucket& b, const Candidate& c) {
  // If identical candidate exists, keep it valid
  for (auto &x : b.cands) {
    if (x.valid && x.poly == c.poly && x.init == c.init && x.mode == c.mode && x.xorout == c.xorout) {
      x.valid = true;
      return;
    }
  }
  // Insert into first free slot, else overwrite index 0
  for (auto &x : b.cands) {
    if (!x.valid) { x = c; return; }
  }
  b.cands[0] = c;
}

// Try to find candidates for this ID without locking globals.
// If globals are already locked and a candidate matches, commit per-ID immediately (announce).
static bool solve_id_params(Bucket& b) {
  if (!g_learning_enabled) return false;
  if (!has_variation(b))   return false;

  auto success = [&](uint8_t poly, uint8_t init, CoverMode m, uint8_t xo)->bool {
    Candidate c{poly, init, m, xo, true};
    add_or_update_candidate(b, c);

    // If globals are already locked and match, commit per-ID now
    if (g_poly_locked && g_init_locked && poly == g_poly && init == g_init) {
      if (!b.solved) {
        b.mode = m; b.xorout = xo; b.solved = true;
        announce_per_id_solved(b, poly, init);     // announce on first commit
      }
    }
    return true;
  };

  auto try_all_modes = [&](uint8_t poly, uint8_t init)->bool {
    for (CoverMode m : kCoverModes) {
      uint8_t xo = 0;
      if (try_mode_and_solve(b, poly, init, g_place, m, &xo)) {
        return success(poly, init, m, xo);
      }
    }
    return false;
  };

  // Case 1: both locked -> only verify modes for the locked pair
  if (g_poly_locked && g_init_locked) {
    return try_all_modes(g_poly, g_init);
  }

  // Case 2: search according to switches (if not set to search and not locked, we do nothing)
  if (g_poly_locked && !g_init_locked) {
    if (!g_init_search_all) return false;
    for (uint16_t init = 0x00; init <= 0xFF; ++init)
      if (try_all_modes(g_poly, (uint8_t)init)) return true;
    return false;
  }

  if (!g_poly_locked && g_init_locked) {
    if (!g_poly_search_all) return false;
    for (uint16_t p = 0x01; p <= 0xFF; ++p)
      if (try_all_modes((uint8_t)p, g_init)) return true;
    return false;
  }

  if (!g_poly_locked && !g_init_locked) {
    if (!(g_poly_search_all && g_init_search_all)) return false;
    for (uint16_t p = 0x01; p <= 0xFF; ++p)
      for (uint16_t init = 0x00; init <= 0xFF; ++init)
        if (try_all_modes((uint8_t)p, (uint8_t)init)) return true;
    return false;
  }

  return false;
}

// After attempting solves, check if enough IDs agree on (poly,init) to lock globally.
// Then commit per-ID params that match the consensus (announce each commit).
static void consensus_lock_if_ready() {
  if (g_poly_locked && g_init_locked) return; // already locked

  struct PairCount { uint8_t poly, init, count; };
  PairCount pc[8]; uint8_t pcN = 0;

  auto bump = [&](uint8_t p, uint8_t i){
    for (uint8_t k = 0; k < pcN; ++k) {
      if (pc[k].poly == p && pc[k].init == i) { pc[k].count++; return; }
    }
    if (pcN < 8) { pc[pcN++] = {p, i, 1}; }
  };

  for (auto &b : g_buckets) {
    if (!b.used || b.count < g_min_samples_per_id) continue;
    for (auto &c : b.cands) if (c.valid) bump(c.poly, c.init);
  }

  // Find the best (poly,init)
  uint8_t bestIdx = 0xFF; uint8_t bestCount = 0;
  for (uint8_t k = 0; k < pcN; ++k) {
    if (pc[k].count > bestCount) { bestCount = pc[k].count; bestIdx = k; }
  }

  if (bestIdx != 0xFF && bestCount >= g_consensus_ids) {
    g_poly = pc[bestIdx].poly; g_poly_locked = true;
    g_init = pc[bestIdx].init; g_init_locked = true;
    Serial.print("Consensus lock: poly=0x"); Serial.print(g_poly, HEX);
    Serial.print(", init=0x"); Serial.println(g_init, HEX);

    // Commit per-ID using the matching candidate
    for (auto &b : g_buckets) {
      if (!b.used || b.solved) continue;
      for (auto &c : b.cands) {
        if (c.valid && c.poly == g_poly && c.init == g_init) {
          b.mode = c.mode; b.xorout = c.xorout; b.solved = true;
          announce_per_id_solved(b, g_poly, g_init);   // announce on commit
          break;
        }
      }
    }
  }
}

// ---- Public API ----
static void learn_from_frame(const CANMessage& f) {
  if (!g_learning_enabled) return;
  if (f.rtr || f.len < 2) return;
  Bucket* b = get_bucket(f.id);
  if (!b) return;
  add_sample(b, f);

  if (has_variation(*b)) {
    (void)solve_id_params(*b);
  }
  consensus_lock_if_ready();
}

// Validate a frame using learned params; on conflict optionally unlock globals & unsolve ID
static bool validate(const CANMessage& f) {
  if (f.len < 2) return false;
  for (auto &b : g_buckets) {
    if (b.used && b.id == f.id && b.solved) {
      uint8_t buf[20];
      const uint8_t m = build_covered(f, b.mode, g_place, buf);
      const uint8_t calc = crc8_bitwise(buf, m, g_poly, g_init, b.xorout);
      const uint8_t obs  = observed_crc_byte(f, g_place);
      bool ok = (calc == obs);
      if (!ok && g_unlock_on_conflict) {
        // Backtrack: unsolve this ID and unlock globals so consensus can re-form
        b.solved = false;
        g_poly_locked = false;
        g_init_locked = false;
        Serial.println("Conflict detected: unlocking global (poly,init) and clearing per-ID solution.");
      }
      return ok;
    }
  }
  return false;
}

static void reset_learning() {
  for (auto &b : g_buckets) { b = Bucket{}; }
  // Do NOT alter external globals here (poly/init/search flags). Your main controls those.
  Serial.println("Learning state reset (per-ID buckets and candidates).");
}

// ---------- Sorted status + export utilities ----------
static void print_hex_id(uint32_t id) {
  Serial.print("0x");
  if (id <= 0x7FF) { // 11-bit: 3 hex digits
    if (id < 0x100) Serial.print('0');
    if (id < 0x010) Serial.print('0');
    Serial.print(id, HEX);
  } else { // 29-bit: 8 hex digits
    if (id < 0x10000000UL) Serial.print('0');
    if (id < 0x01000000UL) Serial.print('0');
    if (id < 0x00100000UL) Serial.print('0');
    if (id < 0x00010000UL) Serial.print('0');
    if (id < 0x00001000UL) Serial.print('0');
    if (id < 0x00000100UL) Serial.print('0');
    if (id < 0x00000010UL) Serial.print('0');
    Serial.print(id, HEX);
  }
}
static void print_hex_u8(uint8_t v) {
  Serial.print("0x");
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

// Build an index of used buckets (optionally only solved), sort by CAN id (insertion sort)
static uint8_t build_sorted_index(uint8_t idx[], bool only_solved) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < (uint8_t)(sizeof(g_buckets)/sizeof(g_buckets[0])); ++i) {
    const Bucket &b = g_buckets[i];
    if (b.used && (!only_solved || b.solved)) idx[n++] = i;
  }
  // insertion sort by id
  for (uint8_t i = 1; i < n; ++i) {
    uint8_t key = idx[i];
    uint8_t j = i;
    while (j > 0 && g_buckets[idx[j-1]].id > g_buckets[key].id) {
      idx[j] = idx[j-1];
      --j;
    }
    idx[j] = key;
  }
  return n;
}

static void print_status() {
  Serial.println("=== CRCDetectFlex Status ===");
  Serial.print("placement : "); Serial.println(g_place == P_TAIL ? "TAIL" : "HEAD");

  Serial.print("poly      : ");
  if (g_poly_locked) { print_hex_u8(g_poly); Serial.println(" (locked)"); }
  else if (g_poly_search_all) { Serial.println("SEARCH-ALL"); }
  else { Serial.println("N/A"); }

  Serial.print("init      : ");
  if (g_init_locked) { print_hex_u8(g_init); Serial.println(" (locked)"); }
  else if (g_init_search_all) { Serial.println("SEARCH-ALL"); }
  else { Serial.println("N/A"); }

  Serial.print("min samples per ID : "); Serial.println(g_min_samples_per_id);
  Serial.print("consensus IDs      : "); Serial.println(g_consensus_ids);
  Serial.print("unlock on conflict : "); Serial.println(g_unlock_on_conflict ? "ON" : "OFF");

  Serial.print("learning  : "); Serial.println(g_learning_enabled ? "ENABLED" : "DISABLED");

  uint8_t idx[sizeof(g_buckets)/sizeof(g_buckets[0])] = {0};
  const uint8_t n = build_sorted_index(idx, /*only_solved=*/false);

  for (uint8_t k = 0; k < n; ++k) {
    const Bucket &b = g_buckets[idx[k]];
    Serial.print("ID ");
    print_hex_id(b.id);
    Serial.print("  samples="); Serial.print(b.count);
    if (b.solved) {
      Serial.print("  [mode="); Serial.print((int)b.mode);
      Serial.print(", xorout="); print_hex_u8(b.xorout); Serial.println("]");
    } else {
      // Print candidate summary
      uint8_t vc = 0; for (auto &c : b.cands) if (c.valid) ++vc;
      Serial.print("  [unsolved, "); Serial.print(vc); Serial.println(" candidate(s)]");
    }
  }
  Serial.println("============================");
}

static void print_export_table() {
  uint8_t idx[sizeof(g_buckets)/sizeof(g_buckets[0])] = {0};
  const uint8_t n = build_sorted_index(idx, /*only_solved=*/true);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("// Copy the following code into: crc8_sae_user_example.h");
  Serial.println("// ========================================================");
  Serial.println("#pragma once");
  Serial.println("#include <ACAN_T4.h>");
  Serial.println("#include \"crc8_sae.h\"");
  Serial.println();
  Serial.println("// ---- User-defined per-ID CRC xorout table (learned) ----");
  Serial.println("static const CrcParams kCrcTable_User[] = {");

  for (uint8_t k = 0; k < n; ++k) {
    const Bucket &b = g_buckets[idx[k]];
    Serial.print("  {0x");
    if (b.id < 0x100) Serial.print('0');
    if (b.id < 0x10)  Serial.print('0');
    Serial.print(b.id, HEX);
    Serial.print(", 0x");
    if (b.xorout < 0x10) Serial.print('0');
    Serial.print(b.xorout, HEX);
    Serial.println("},");
  }
  Serial.println("};");
  Serial.println();
  Serial.println("// ---- Example function: calls CRC check using learned parameters ----");
  Serial.println("static inline bool example_function(const CANMessage& frame) {");
  Serial.println("  if (frame.rtr) return false;");
  Serial.println("  if (frame.len < 2 || frame.len > 8) return false;");
  Serial.println();
  Serial.println("  return check_crc8_lookup(");
  Serial.println("    frame.id,");
  Serial.println("    frame.data,");
  Serial.println("    frame.len,");
  Serial.println("    kCrcTable_User,");
  Serial.println("    sizeof(kCrcTable_User) / sizeof(kCrcTable_User[0]),");
  Serial.print("    0x"); if (g_init < 0x10) Serial.print('0'); Serial.print(g_init, HEX); Serial.println(",  // learned init");
  Serial.print("    0x"); if (g_poly < 0x10) Serial.print('0'); Serial.print(g_poly, HEX); Serial.println("   // learned polynomial");
  Serial.println("  );");
  Serial.println("}");
  Serial.println("============================================================");
  Serial.println();
}

static void print_export_table_fast() {
  uint8_t idx[sizeof(g_buckets)/sizeof(g_buckets[0])] = {0};
  const uint8_t n = build_sorted_index(idx, /*only_solved=*/true);

  Serial.println();
  Serial.println("============================================================");
  Serial.println("// Copy the following code into: crc8_sae_user_example_fast.h");
  Serial.println("// ========================================================");
  Serial.println("#pragma once");
  Serial.println("#include <ACAN_T4.h>");
  Serial.println("#include \"crc8_sae_fast.h\"");
  Serial.println();
  Serial.println("// ---- User-defined per-ID CRC xorout table (learned) ----");
  Serial.println("// Any ID NOT listed here is treated as \"no CRC present\".");
  Serial.println("static const CrcParams kCrcTable_User_Fast[] = {");

  for (uint8_t k = 0; k < n; ++k) {
    const Bucket &b = g_buckets[idx[k]];
    Serial.print("  {0x");
    if (b.id < 0x100) Serial.print('0');
    if (b.id < 0x10)  Serial.print('0');
    Serial.print(b.id, HEX);
    Serial.print(", 0x");
    if (b.xorout < 0x10) Serial.print('0');
    Serial.print(b.xorout, HEX);
    Serial.println("},");
  }

  Serial.println("};");
  Serial.println();
  Serial.println("// ---- Example function: fast LUT-based CRC check ----");
  Serial.println("// If ID is known: validate CRC");
  Serial.println("// If ID is not listed: skip check (assume no CRC)");
  Serial.println("static inline bool example_function_fast(const CANMessage& frame) {");
  Serial.println("  if (frame.rtr) return false;  // No data in RTR frames");
  Serial.println("  if (frame.len > 8) return false;  // Classic CAN limit");
  Serial.println();
  Serial.println("  return check_crc8_lookup(");
  Serial.println("    frame.id,");
  Serial.println("    frame.data,");
  Serial.println("    frame.len,");
  Serial.println("    kCrcTable_User_Fast,");
  Serial.println("    sizeof(kCrcTable_User_Fast) / sizeof(kCrcTable_User_Fast[0]),");
  Serial.print("    0x"); if (g_init < 0x10) Serial.print('0'); Serial.print(g_init, HEX); Serial.println(", // learned init");
  Serial.print("    0x"); if (g_poly < 0x10) Serial.print('0'); Serial.print(g_poly, HEX); Serial.println("  // learned polynomial");
  Serial.println("  );");
  Serial.println("}");
  Serial.println("============================================================");
  Serial.println();
}


} // namespace CRCDetectFlex
