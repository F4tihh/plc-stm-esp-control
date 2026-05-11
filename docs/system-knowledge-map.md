# System Knowledge Map (STM32H755 ↔ PLC Modbus TCP ↔ SD7RS20 ↔ ESP32)

This document is a **read-only engineering knowledge map** of the current system, built from:

- STM32 firmware excerpts (notably `CM7/Core/Src/main.c`)
- PLC operand/register screenshots (GMSuite “Operandlar” tables)
- PLC ladder screenshots
- HMI screenshots
- SD7 series manual (`docs/reference_materials/SD7_SERVO_KILAVUZ_V9_23032026.pdf`)

It is intentionally documentation-only: **no protocol changes**, **no refactoring**, **no behavior changes**.

---

## 1) System architecture (roles + layers)

### High-level blocks

- **ESP32 (wireless UI layer)**
  - Hosts AP + Web UI
  - Sends user commands to STM32 over UART (single-character commands)
  - Receives live telemetry lines from STM32 (`LIVE,distance,speed`)
  - Performs local test calculations (10 m test, graphs, max/avg, etc.)

- **STM32H755 (supervisory controller)**
  - Receives UART commands from ESP32
  - Acts as **Modbus TCP client/master** to the PLC (Ethernet/LwIP)
  - Writes PLC coils/registers for start/stop and setpoints
  - Reads PLC telemetry (distance/speed) and forwards to ESP32 as `LIVE,...`

- **PLC (industrial motion authority)**
  - Owns the operational state machine (start latch, limits, completion)
  - Computes distance + speed telemetry from servo feedback + calibration
  - Bridges between:
    - **Modbus TCP map** exposed to STM32/HMI
    - **RS485 Modbus RTU master** map used to talk to the drive / gateway

- **Servo system**
  - SD7RS20 servo drive + motor + drum/rope mechanism
  - Safety chain includes limits and STO (Safe Torque Off) at drive level

---

## 2) Modbus holding register table (knowledge map)

**Important conventions**

- PLC screenshots show Modbus addresses in “40001/42011 style”.
- STM32 code uses **0-based offsets** for holding registers (e.g., `raw = human - 40001`).
- “Real” signals are **float32** and typically occupy **2 holding registers**.

Legend:
- **Owner**
  - **HMI**: intended to be written by operator UI
  - **PLC**: internal PLC computation/state
  - **STM32**: written/read by STM32 supervisory layer
  - **Drive/RTU**: values bridged from the drive-side RS485/RTU layer
- **Risk**
  - **Safe**: read-only telemetry or non-motion reporting
  - **Caution**: can change test references or internal PLC behavior
  - **Danger**: can directly change motion/safety/limits; do not write casually

| Address | Name | Type | R/W (intent) | Owner (typical) | Meaning | Risk to write | Evidence (where visible) |
|---:|---|---|---|---|---|---|---|
| 40001 | SERVO_HIZI | Word (uint16) | R/W | PLC | Main speed command word used by ladder; routed from fixed/recipe/variable logic | **Danger** | Operand screenshot; ladder uses `SERVO_HIZI` |
| 40002 | Servo_hiz_gelen | Word | R | Drive/RTU | Speed “incoming” mirror (drive/RTU → PLC) | Caution | Operand screenshot |
| 40003 | Servo_hiz_giden | Word | R/W | Drive/RTU / PLC | Speed “outgoing” mirror (PLC → drive/RTU) | **Danger** | Operand screenshot; RS485 tag list shows write |
| 40004 | SERVO_TORKU | Word (uint16) | R/W | PLC | Main torque command word used by ladder; routed from fixed/recipe/variable logic | **Danger** | Operand screenshot; ladder uses `SERVO_TORKU` |
| 40005 | Servo_tork_gelen | Word | R | Drive/RTU | Torque “incoming” mirror (drive/RTU → PLC) | Caution | Operand screenshot |
| 40006 | Servo_tork_giden | Word | R/W | Drive/RTU / PLC | Torque “outgoing” mirror (PLC → drive/RTU) | **Danger** | Operand screenshot; RS485 tag list shows write |
| 40007 | Servo_mevcut_hiz | Word | R | Drive/RTU | Current speed feedback (telemetry) | Safe | Operand screenshot |
| 40008..(many) | R_HIZ0..R_HIZ* | Word | R/W | HMI | Recipe speed table rows (REÇETE page) selected by `R_SAYAC` | Caution | Operand screenshots show long `R_HIZ*` sequences |
| 40109..(many) | R_TORK0..R_TORK* | Word | R/W | HMI | Recipe torque table rows selected by `R_SAYAC` | Caution | Operand screenshots show `R_TORK*` sequences (e.g. 40109, 40198+) |
| 40210..40310 | R_MESAFE0..R_MESAFE100 | Word | R/W | HMI | Recipe distance table rows selected by `R_SAYAC` | Caution | Operand screenshots show `R_MESAFE0 @40210` to `R_MESAFE100 @40310` |
| 40311 | SERVO_MESAFE | Word | R/W | PLC | Distance setpoint used by meter/timed workflows; involved in completion logic | **Danger** | Operand screenshot; ladder compares distance vs setpoint |
| 40312 | Ekran_servo_sifirlama | Word | R/W | HMI | “Servo reset/zero” word (exact semantics not fully captured) | **Danger** | Operand screenshot |
| 40313 | EKRAN_SERVO_SABIT_HIZ | Word | R/W | HMI | Fixed-mode speed setpoint (HMI SABİT MOD) | **Danger** | Operand screenshot; ladder routes fixed mode setpoints |
| 40314 | EKRAN_SERVO_SABIT_TORK | Word | R/W | HMI | Fixed-mode torque setpoint (HMI SABİT MOD) | **Danger** | Operand screenshot; ladder routes fixed mode setpoints |
| 40317 | EKRAN_MEVVCUT_TORK | Word | R | Drive/RTU | Current torque feedback (telemetry) | Safe | Operand screenshot |
| 42001 | R_SAYAC | Integer | R/W | HMI / PLC | Recipe index/counter; ladder increments and “KÜME OKU” selects recipe rows | Caution | Operand screenshot; ladder `TOPLA R_SAYAC + 1` |
| 42003 | Tambur_capi | Real (float32, 2 regs) | R/W | HMI | Drum diameter (calibration) | **Danger** | Operand screenshot; ladder uses in circumference calc |
| 42005 | Tambur_cevresi | Real (2 regs) | R/W | PLC/HMI | Drum circumference (derived from diameter) | Caution | Operand screenshot; ladder computes it |
| 42007 | Bir_turdaki_pulse_miktari | Real (2 regs) | R/W | HMI | Pulses-per-rev (calibration) | **Danger** | Operand screenshot; ladder divides by pulses |
| 42009 | Metre_kalibrasyon_orani | Real (2 regs) | R/W | HMI | Calibration ratio for distance | **Danger** | Operand screenshot; ladder applies calibration |
| 42011 | Gercek_metre | Real (2 regs) | R | PLC | Primary distance telemetry in metres (after tare) | Safe | Operand screenshot; ladder forms `Gercek_metre` |
| 42013 | Servo_mevcut_pozisyon_real | Real (2 regs) | R | Drive/RTU | Servo position feedback input to metre computation | Safe | Operand screenshot; ladder uses it |
| 42015 | Gercek_mm | Real (2 regs) | R | PLC | Intermediate distance in mm | Safe | Operand screenshot; ladder divides by 1000 |
| 42017 | Metre_cikarilan | Real (2 regs) | R/W | PLC | Tare offset; set to `Metre_gelen` on reset | Caution | Operand screenshot; ladder assigns on tare |
| 42019 | Metre_gelen | Real (2 regs) | R | PLC | Raw metres before tare subtraction | Safe | Operand screenshot; ladder uses in subtraction |
| 42021 | EKRAN_MAX_MESAFE | Real (2 regs) | R/W | HMI | Max distance limit threshold | **Danger** | Operand screenshot; ladder asserts `MAX_LIMITTE` |
| 42023 | EKRAN_MIN_MESAFE | Real (2 regs) | R/W | HMI | Min distance limit threshold | **Danger** | Operand screenshot; ladder asserts `MIN_LIMITTE` |
| 42025 | Tambur_hizi | Real (2 regs) | R | PLC | Drum speed intermediate | Safe | Operand screenshot; ladder computes it |
| 42027 | Hiz_mm_dk | Real (2 regs) | R | PLC | Speed intermediate (mm/min) | Safe | Operand screenshot; ladder computes it |
| 42029 | Hiz_m_sn | Real (2 regs) | R | PLC | Speed telemetry (m/s) | Safe | Operand screenshot; ladder computes it |
| 42031 | Hiz_m_dk | Real (2 regs) | R | PLC | Speed intermediate (m/min) | Safe | Operand screenshot; ladder computes it |
| 42033 | Hiz_km_s | Real (2 regs) | R | PLC | Speed telemetry (km/h) | Safe | Operand screenshot; ladder computes it |
| 42035 | Mevcut_hiz_int | Integer | R | PLC | Integer speed used as part of conversion chain | Safe | Operand screenshot; ladder uses it |
| 42037 | Max_hiz_m_sn | Real (2 regs) | R/W | PLC/HMI | Max speed tracker (m/s); resettable | Caution | Operand screenshot; ladder updates + resets |
| 42039 | Max_hiz_km_s | Real (2 regs) | R/W | PLC/HMI | Max speed tracker (km/h); resettable | Caution | Operand screenshot; ladder updates + resets |
| 42041 | Mevcut_hiz_real | Real (2 regs) | R | Drive/RTU | Current speed in Real feeding conversion chain | Safe | Operand screenshot; ladder uses it |

### Drive/RTU-side register hints (RS485 tag list screenshot)

One reference screenshot shows PLC COM2 configured as **ModbusRTU Master** (38400, little-endian) and an RS485 “slave tag list” with addresses like:

| RTU Address | Tag | Direction (from screenshot) | Notes |
|---:|---|---|---|
| 40812 | Servo_hiz_gelen | Read | drive/RTU → PLC |
| 40812 | Servo_hiz_giden | Write | PLC → drive/RTU |
| 40814 | Servo_tork_gelen / Servo_tork_giden | Read/Write | torque bridge |
| 42823 | Servo_mevcut_hiz | Read | feedback |
| 42824 | Servo_mevcut_tork | Read | feedback |
| 40806 | Servo_yon_secimi | Write | direction/config-like word |

**Interpretation:** PLC likely bridges a drive/gateway map on RS485 into the Modbus TCP map exposed to STM32/HMI.

---

## 3) Coil / bit table

Notes:
- Coil numbers below are shown in PLC operand screenshots as “Modbus Adresi”.
- STM32 code uses **0-based coil addressing** when writing (human coil N → raw N-1).

| Coil (human) | Name | R/W (intent) | Owner (typical) | Function | Risk to write | Evidence |
|---:|---|---|---|---|---|---|
| 2 | EKRAN_STOP | R/W | HMI / STM32 | Stop request / stop interlock. Blocks start latch and triggers stop behavior | **Danger** | Operand screenshot + ladder start rung interlock |
| 3 | EKRAN_START | R/W | HMI / STM32 | Start request. Latches `SISTEM_STARTLI` when safe | **Danger** | Operand screenshot + ladder start rung |
| 4 | SISTEM_STARTLI / ISTEM_STARTLI | R/W | PLC | Run latch. When false, PLC forces speed/torque commands to 0 | **Danger** | Operand screenshot + ladder |
| 5 | EKRAN_METRE_AKTIF | R/W | HMI | Enables metre-based workflow path | Caution | Operand screenshot + ladder |
| 6 | EKRAN_YON_SECIMI | R/W | HMI | Direction select; affects >= / <= distance comparisons | **Danger** | Operand screenshot + ladder branches |
| 7 | Metre_sifirlama | R/W | HMI | Tare/reset: `Metre_cikarilan = Metre_gelen` | Caution | Operand screenshot + ladder assignment |
| 8 | EKRAN_DEGISKEN_MOD | R/W | HMI | Variable mode select. Ladder uses NC contact (“NO kontak” tooltip shown) | **Danger** | Operand screenshot + ladder |
| 9 | MAX_LIMITTE | R | PLC | Asserted when `Gercek_metre >= EKRAN_MAX_MESAFE` | Safe | Operand screenshot + ladder compare |
| 10 | MIN_LIMITTE | R | PLC | Asserted when `Gercek_metre < EKRAN_MIN_MESAFE` | Safe | Operand screenshot + ladder compare |
| 11 | ISLEM_BITTI | R/W | PLC | Process finished; also an interlock preventing start | Caution | Operand screenshot + ladder |
| 12 | EKRAN_MAX_HIZ_SIFIRLA | R/W | HMI | Resets max-speed trackers | Caution | Operand screenshot + ladder reset rung |
| 13 | METRE_AKTIF | R/W | PLC | System-side “metre active” enable (distinct from EKRAN_METRE_AKTIF) | Caution | Operand screenshot |

---

## 4) Data-flow diagram (text form)

### Command path (user → motion)

1. **User presses web UI button** (ESP32 web page)
2. ESP32 endpoint handler sends a **single UART character** to STM32:
   - `R` start, `S` stop, `A/Z` torque ±, `K/M` speed ±
3. STM32 reads UART2 and maps command to PLC actions:
   - Writes **coils** (start/stop/latch) and/or **holding registers** (speed/torque setpoints)
4. STM32 sends Modbus TCP requests to PLC (Ethernet/LwIP sockets)
5. PLC ladder logic:
   - Applies interlocks (STOP, limits, ISLEM_BITTI)
   - Latches run state (`ISTEM_STARTLI`)
   - Routes setpoints (recipe/fixed/variable) into live command words
6. PLC bridges setpoints/feedback over **RS485 Modbus RTU** to the drive/gateway
7. SD7RS20 drives motor/drum/rope mechanism

### Telemetry path (motion → UI)

1. Drive feedback → PLC (likely via RTU bridge + internal PLC signals)
2. PLC computes:
   - `Gercek_metre` (float)
   - `Hiz_m_sn`, `Hiz_km_s` (float)
   - max speed trackers, etc.
3. STM32 periodically reads PLC telemetry (at least `Gercek_metre` in current code)
4. STM32 emits UART line to ESP32:
   - `LIVE,distance,speed`
5. ESP32 parses LIVE line and updates:
   - live numbers + graphs
   - 10 m test calculations

---

## 5) PLC state-machine understanding (inferred from ladder screenshots)

### Primary states

- **Idle**
  - `ISTEM_STARTLI = 0`
  - Ladder forces `SERVO_HIZI = 0` and `SERVO_TORKU = 0` when not running

- **Arming**
  - HMI/STM32 sets `EKRAN_START = 1`
  - Interlocks must be satisfied:
    - `EKRAN_STOP = 0`
    - `MIN_LIMITTE = 0`, `MAX_LIMITTE = 0`
    - `ISLEM_BITTI = 0`

- **Running (latched)**
  - `ISTEM_STARTLI = 1` (self-holding coil visible)
  - Setpoint routing depends on mode:
    - Variable vs fixed/recipe selection
  - Telemetry computations and max tracking operate continuously

- **Limit-hit**
  - `MAX_LIMITTE` asserted when `Gercek_metre >= EKRAN_MAX_MESAFE`
  - `MIN_LIMITTE` asserted when `Gercek_metre < EKRAN_MIN_MESAFE`
  - These are used as interlocks (prevent start) and likely influence stop logic elsewhere

- **Completed**
  - `ISLEM_BITTI = 1` indicates process completion (rung shows it depends on mode and distance/setpoint satisfaction)

### Key transitions

- **Idle → Running**: `EKRAN_START=1` AND interlocks OK
- **Running → Idle**: `EKRAN_STOP=1` (or latch cleared); plus forced setpoints to 0 when not running
- **Running → Completed**: `ISLEM_BITTI` asserted by completion logic
- **Any → Limit-hit**: threshold comparisons on `Gercek_metre`

---

## 6) PLC ↔ SD7RS20 communication assumptions

### What is directly evidenced

- PLC is configured as **Modbus RTU Master** on RS485 (screenshot shows COM2 RS485, 38400 baud, little-endian).
- RS485 tag list shows registers for speed/torque in/out and feedback (e.g., 40812, 40814, 42823, 42824).
- SD7 manual confirms the drive supports:
  - **RS485 Modbus RTU protocol (RJ45 port)**
  - DI/DO assignments including **positive/negative limits**, **alarm clear**, **servo enable**, and **STO port**.

### Most likely topology

- **PLC ↔ (SD7RS20 or gateway) via RS485 Modbus RTU**
- PLC maps RTU registers into PLC operands, then exposes a simplified **Modbus TCP map** to HMI/STM32.

### Implications

- Even if STM32 writes correct Modbus TCP setpoints, the actual motion depends on:
  - Servo enable chain (Servo-ON)
  - Alarm state + alarm reset
  - Limits and STO safety chain
  - PLC state machine and interlocks

### What remains uncertain

- Whether the RTU addresses shown (408xx/428xx) are SD7RS20-native registers or a gateway’s map.
- Exact SD7 Modbus register map (not extracted from the manual pages we viewed in this session).

---

## 7) STM32 / ESP32 / PLC responsibilities (current)

### ESP32

- Wi‑Fi AP + web interface endpoints
- Sends UART command characters to STM32 (R/S/A/Z/K/M)
- Parses UART LIVE telemetry
- Runs the user-facing test logic (10 m test), graphs, tables, max/avg calculations

### STM32H755

- UART command intake and mapping to PLC writes
- Modbus TCP master/client over LwIP sockets
- Telemetry read from PLC and forward to ESP32 as `LIVE,distance,speed`

From the STM32 code we analyzed earlier:
- PLC connection settings used: **192.168.1.100:502**, Unit ID **1**
- Telemetry transmit format: `LIVE,<distance>,<speed>`

### PLC

- Motion and safety authority:
  - start latch
  - stop interlock
  - limits
  - completion
- Computes distance/speed telemetry from servo feedback + drum calibration
- Bridges RTU ↔ internal operands ↔ TCP map

---

## 8) Which computations happen where (PLC vs STM32 vs ESP32)

### PLC computations (confirmed by ladder)

- Metre calculation pipeline:
  - `Servo_mevcut_pozisyon_real` + drum constants + calibration → `Gercek_mm` → `Metre_gelen`
  - `Gercek_metre = Metre_gelen - Metre_cikarilan`
  - Tare via `Metre_sifirlama` sets `Metre_cikarilan = Metre_gelen`
- Speed pipeline:
  - `Mevcut_hiz_real` → `Hiz_m_sn`, `Hiz_km_s`, and maxima
- Limit logic:
  - `MAX_LIMITTE`, `MIN_LIMITTE` from `Gercek_metre` vs thresholds
- Run latch + completion:
  - `ISTEM_STARTLI`, `ISLEM_BITTI`
- Setpoint routing:
  - recipe/fixed/variable selection and copying into live command words

### STM32 computations (currently)

- Minimal: acts as command bridge + Modbus transport
- Formats LIVE telemetry strings
- (In `main.c` path we saw) distance read comes from PLC float read and speed may be placeholder depending on implementation stage

### ESP32 computations (current design intent)

- 10 m test logic and UI analytics:
  - delta distance/time
  - instant speed (if not trusted)
  - max speed, average speed
  - overshoot interpolation and corrected finish time
  - chart/table logging

---

## 9) Safe-to-write vs dangerous signals (operational guidance)

### Generally safe (still validate in your environment)

- **Read-only telemetry**
  - `Gercek_metre`, `Hiz_m_sn`, `Hiz_km_s`, `Max_hiz_*`, feedback words/position
- **Non-motion UI actions**
  - `EKRAN_MAX_HIZ_SIFIRLA` (resets max trackers)
  - `Metre_sifirlama` (tare/reset reference) *(Caution: it changes reference frame and can affect limits/completion logic)*

### Dangerous to write casually (motion/safety critical)

- **Start/stop/latch**
  - `EKRAN_START`, `EKRAN_STOP`, `SISTEM_STARTLI`
- **Motion setpoints**
  - `SERVO_HIZI`, `SERVO_TORKU`, `SERVO_MESAFE`
  - fixed-mode setpoints `EKRAN_SERVO_SABIT_HIZ` (40313), `EKRAN_SERVO_SABIT_TORK` (40314)
  - recipe arrays (`R_HIZ*`, `R_TORK*`, `R_MESAFE*`) if PLC can execute them immediately
- **Mode/direction**
  - `EKRAN_DEGISKEN_MOD`
  - `EKRAN_YON_SECIMI`
- **Safety thresholds + calibration**
  - `EKRAN_MIN_MESAFE`, `EKRAN_MAX_MESAFE`
  - drum/calc constants (`Tambur_capi`, pulses-per-rev, calibration ratio)

Reason: these affect **direction, enable conditions, stop behavior, limits, and the physical motion envelope**.

---

## 10) Recommended future migration architecture (low-risk roadmap)

### Principle: PLC remains the safety + motion authority

Keep the PLC as:
- interlock owner (STOP, limits, alarms, STO chain)
- run latch owner
- setpoint routing owner
- distance/speed computation authority (single source of truth)

### Phase 0: Documentation + guardrails (no functional changes)

- Freeze a clear “PLC API surface”:
  - list exactly which coils/registers the STM32 is allowed to write
  - document datatypes (Word vs Real/float)
  - document 0-based vs 1-based addressing conventions

### Phase 1: Safer telemetry + explicit write gating

- Preserve the UART framing (`LIVE,distance,speed`) but ensure values are sourced from PLC truth:
  - `Gercek_metre` (42011)
  - `Hiz_m_sn` (42029)
- Add “write gates” on the STM32 side (policy-level):
  - e.g. only allow changing dangerous setpoints when PLC is not running, or when an explicit “armed” state is verified

### Phase 2: Versioned supervisory interface (optional, backwards compatible)

- Keep existing single-char commands for stability, but add an optional versioned command channel:
  - capabilities query
  - read-only diagnostics
  - explicit command acknowledgements
- Do **not** bypass PLC safety (STO/limits/Servo-ON).

### Avoid (high risk)

- Replacing PLC safety logic with STM32 logic
- Changing PLC↔drive RTU mapping without a full drive register-map validation
- Any change that weakens STO/limit chain behavior

