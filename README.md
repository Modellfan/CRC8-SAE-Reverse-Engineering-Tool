# CRC-8 Learner for CAN (Teensy 4.x)

Reverse-engineer CRC-8 parameters from live CAN traffic, validate against learned parameters (normal and LUT-fast implementations), and benchmark both with high-resolution timing.

## Features

- Works on Teensy 4.0 / 4.1 using ACAN_T4 (CAN1 @ 500 kbit/s)
- Learns per-ID CRC placement (HEAD/TAIL), polynomial, and init value
- Validates both normal and LUT (lookup table) implementations
- High-resolution BENCH mode using CPU cycle counter
- Serial console interface with clear commands and a built-in "howto" guide

---

## Hardware Setup

**Required:**
- Teensy 4.0 or 4.1
- 3.3V-compatible CAN transceiver (e.g., MCP2562FD, SN65HVD230, or TJA1051T/3)

**Wiring (CAN1 on Teensy 4.x):**
- Teensy pin 22 → transceiver TXD
- Teensy pin 23 → transceiver RXD
- VCC → 3.3V (or 5V if allowed by transceiver)
- GND → GND
- Enable pins (EN/STBY/SLP) → enable state (EN=HIGH, STBY/SLP=LOW)
- CANH/CANL → CAN bus (terminated with 120 Ω at both ends)
- Common ground across all CAN nodes

The sketch configures CAN1 for 500 kbit/s:
```
ACAN_T4_Settings settings(500 * 1000);
```

---

## Software Requirements

- Arduino IDE with Teensyduino
- ACAN_T4 library (install via Library Manager or from GitHub)

**Project files:**
- BMWi3CRC.ino (main sketch)
- crc8_sae_learner.h / .cpp
- crc8_sae_user_example.h
- crc8_sae_user_example_fast.h
- crc8_sae_fast_helper.h

---

## Uploading

1. Open `BMWi3CRC.ino` in Arduino IDE.
2. Select:
   - Board: Teensy 4.0 or 4.1
   - USB Type: Serial
   - CPU Speed: Default (600 MHz)
3. Upload to Teensy.
4. Open Serial Monitor at 115200 baud.

You should see:
```
CRC-8 Learner Console ready.
CAN1 OK
```

---

## Using SavvyCAN to Send Frames

This program listens on the physical CAN bus (not USB).

1. Connect a CAN adapter (CANable, candleLight, PEAK, etc.) to the **same CAN bus** as the Teensy.
2. In **SavvyCAN**:
   - Set bitrate to **500 kbit/s**
   - Open the Replay window and load a log file
   - **Set a minimum inter-frame delay of at least 10 ms**
   - Avoid "burst" or zero-delay replays
3. Start replay. The Teensy will receive frames on CAN1.

---

## Serial Commands

Use a serial console (115200 baud) to interact.

### Basic Commands

| Command | Description |
|----------|--------------|
| `help` or `?` | Show command list |
| `howto`, `guide`, `usage` | Display wiring and setup guide |
| `status` | Show learner state and per-ID results |
| `export` | Print learned parameters as `crc8_sae_user_example.h` |
| `export fast` | Print LUT version (`crc8_sae_user_example_fast.h`) |
| `gen table 0xPP` | Print 256-entry MSB-first CRC-8 table for polynomial `0xPP` |

### Modes

| Command | Description |
|----------|-------------|
| `mode learn` | Enable learning mode (silent on mismatches) |
| `mode validate` | Validate frames using normal CRC |
| `mode validate_fast` | Validate frames using LUT-based CRC |
| `mode bench` | Run high-resolution benchmark |

### Bench Settings

| Command | Description |
|----------|-------------|
| `bench iters N` | Set number of iterations per frame (default 1000) |

### Learner Controls (active in LEARN mode)

| Command | Description |
|----------|-------------|
| `set place TAIL|HEAD` | CRC byte position |
| `set poly 0x1D|ALL` | Lock or search all possible polynomials |
| `set init 0xFF|ALL` | Lock or search all possible init values |
| `learn start|stop` | Enable or disable learning |
| `learn reset` | Reset all learned data |

---

## Typical Workflow

1. Wire Teensy and transceiver, ensure bus termination.
2. Upload sketch and verify "CAN1 OK".
3. Replay frames from SavvyCAN (≥10 ms delay).
4. Set mode:
   - `mode learn` to find CRC parameters
   - `status` to view progress
   - `export` or `export fast` once learned
5. Use `mode validate` or `mode validate_fast` for live validation.
6. Use `mode bench` to compare normal vs fast implementation timing.
   - Adjust iterations with `bench iters N` for precision.

---

## Benchmark Example Output

Example high-resolution timing (averaged over iterations):
```
BENCH id=0x1A3 len=8 iters=1000 | normal=92 cyc (153 ns) | fast=60 cyc (100 ns) | delta=32 cyc (53 ns)
```

---

## Troubleshooting

**No frames shown:**
- Check CAN wiring, bitrate (500 kbit/s), and ground.
- Ensure both Teensy and adapter are on the same bus.

**CRC FAIL messages:**
- Learner not yet solved: switch to `mode learn`
- Try `set poly ALL` or `set init ALL`
- Check CRC position: `set place TAIL` (default) or `HEAD`

**Unstable learning:**
- Use ≥10 ms frame spacing in SavvyCAN
- Avoid rapid/bursty playback

**Bus errors:**
- Verify 120 Ω termination at both ends
- Ensure transceiver EN/STBY pins are correctly driven

---

## File Overview

| File | Description |
|------|--------------|
| `BMWi3CRC.ino` | Main sketch and console |
| `crc8_sae_learner.h/.cpp` | Learner logic per CAN ID |
| `crc8_sae_user_example.h` | Normal validator |
| `crc8_sae_user_example_fast.h` | LUT validator |
| `crc8_sae_fast_helper.h` | CRC table generation tools |

---

## Notes

- Always test on a safe CAN setup (bench first).
- Do not replay frames directly on a live vehicle network unless you know what they do.

---

## License

Use an open-source license of your choice (MIT / Apache-2.0 recommended).

---

## Credits

- ACAN_T4 by Pierre Molinaro  
- Teensy 4.x platform by PJRC  

---

End of File
