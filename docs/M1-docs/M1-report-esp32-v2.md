# PROJECT WINDIFY — TECHNICAL CONTEXT LOG
**Last updated:** 2026-04-21
**Format:** Raw technical context for AI prompt injection
**Authors:** Nhóm 24 — Quốc Hưng (ESP32) · Minh Chiến (STM32)

---

## 1. PROJECT SCOPE & CONSTRAINTS

**Project title:** Design and Implementation of a Low-Cost 6-DOF Indoor Mini Drone
for STEM Education (Future Academy / Windidy ecosystem)

**Academic context:**
- Institution: FAST Faculty, University of Science and Technology, University of Danang (DUT)
- Supervisor: TS. Ngô Đình Thanh
- Client: Windidy Co. — Mr. Lê Thái Bảo Toàn
- Defense venue: On-site at Windidy (enterprise defense, not formal faculty committee)
- Milestone: M1 (10% of total capstone grade) — due 2026-04-22
- Remaining milestones: M2, M3
- Defense time: 10–15 minutes, English language, 2-person team

**Assessment criteria (Bảng 2 — M1, 5 sub-criteria × 20% each):**
1. Overview: problem relevance, existing solutions, commercial products
2. Preliminary solution: block diagram, design process, materials
3. Expected results: method, process, model, publication
4. Evaluation method: efficiency, cost, complexity, accuracy criteria
5. Implementation plan + work distribution with timelines

---

## 2. HARDWARE STACK

### MCU Platform A — STM32F103C8T6 (Minh Chiến)
- IDE: Keil C (MDK-ARM)
- Architecture: Bare-metal, super-loop
- I2C: Bit-bang recovery + hardware I2C1 on PB6/PB7, 400kHz Fast Mode
- UART: USART1 on PA9/PA10, 115200 baud, retargeted via fputc → Hercules terminal
- IMU: MPU6050 clone (WHO_AM_I = 0x70, expected 0x68 for genuine MPU6050)
  - DLPF: 0x1A = 0x03 → 41Hz cutoff
  - Gyro range: 0x1B = 0x08 → ±500 dps, scale factor 65.5 LSB/dps
  - Accel range: 0x1C = 0x10 → ±8g, scale factor 4096 LSB/g
  - **Hardware anomaly:** accel_z_raw at level position reads ~4596 instead of theoretical 4096
    → offset error: ~500–600 LSB due to OEM clone tolerance
  - Calibration: 1000-sample averaging, accel_z_offset = mean(az_raw) - 4096
  - Result: Roll/Pitch/Yaw output verified via Hercules UART log
- Bluetooth: HC-05 / JDY-33 connected successfully — UART telemetry to PC
  - Decision: BLE dropped for motor control due to unstable latency
  - RF joystick (partner's implementation) used instead
- Motor driver: AO3400A N-channel MOSFET + SS54 flyback diode
  - PWM via TIM peripheral
  - **Hardware incident:** flyback diode installed in reverse polarity by partner
    → 2× AO3400A destroyed → 1-week delay for component replacement
  - Status: single motor PWM verified independently using bench power supply
  - Status: 4-motor integration pending — requires drone frame (not yet procured)
- FSM button debounce: drn_button.cpp (PA0 = GPIO4, PA1 = GPIO5)

### MCU Platform B — ESP32-S3-SuperMini (Quốc Hưng)
- IDE: Arduino IDE (not ESP-IDF) — partner's choice
- IMU: Same MPU6050 clone (WHO_AM_I = 0x70) — knowledge transfer from STM32 phase
- HAL layer: drn_mpu6050.h/.cpp — ESP-IDF style, structured after STM32 xx_ files
  - I2C: GPIO8 (SDA), GPIO9 (SCL), 400kHz, i2c_master_write_read_device API
  - DLPF: 0x1A = 0x03 → 41Hz (same as STM32)
  - Accel scale: 4096 LSB/g (±8g config)
  - Gyro scale: 65.5 LSB/dps (±500 dps config)
  - Complementary filter: alpha = 0.98 (gyro weight), 0.02 (accel weight)
  - Calibration: 1000 samples, Z-axis offset = mean - 4096
- BLE: telemetry.cpp — NimBLE stack, Nordic UART Service (NUS) UUID
  - Status: functional in Old Core_Files/ — not yet integrated into drn_ HAL
  - Decision: BLE control dropped, remains as telemetry-only option
- RF: partner's implementation, hardware joystick
- Motor PWM: LEDC 16kHz on AO3400A — functional in Old Core_Files/hal_timer_pwm.cpp
  - Not yet refactored into drn_ HAL namespace
- PID: Old Core_Files/pid_control.cpp — Kp=1.2, Ki=0.0, Kd=0.0 (P-only, not tuned)
- FreeRTOS: present but not yet used for dual-core task pinning
  - Current architecture: single super-loop with vTaskDelay(1ms)
  - Target architecture (M2): xTaskCreatePinnedToCore — Core0 BLE, Core1 flight

---

## 3. SOFTWARE ARCHITECTURE

### Sensor fusion pipeline (both platforms)
```
I2C burst read (14 bytes, reg 0x3B–0x48)
→ Parse big-endian int16_t: ax, ay, az, temp, gx, gy, gz
→ Subtract calibration offsets
→ Scale: accel /= 4096.0f (g), gyro /= 65.5f (dps)
→ Compute Δt from timer
→ Accel angle: roll = atan2(ay, az) × 57.296, pitch = atan2(-ax, √(ay²+az²)) × 57.296
→ Complementary filter:
     roll  = 0.98×(roll  + gx×dt) + 0.02×accel_roll
     pitch = 0.98×(pitch + gy×dt) + 0.02×accel_pitch
     yaw   = yaw + gz×dt  (gyro integration only, no magnetometer)
```

### Key file inventory
| File | Platform | Status | Purpose |
|------|----------|--------|---------|
| drn_mpu6050.h/.cpp | ESP32-S3 | Implemented, verified | IMU HAL + complementary filter |
| drn_button.h/.cpp | ESP32-S3 | Implemented | FSM debounce, single/double/hold |
| drn_main_board_choose.h/.cpp | ESP32-S3 | Implemented | Board abstraction macro |
| xx_mpu_complementary_filter.h/.c | STM32 | Implemented, verified | Full bare-metal IMU pipeline |
| xx_mpu_data_fusion.h/.c | STM32 | Implemented, verified | Alternative IMU implementation |
| Old Core/telemetry.h/.cpp | ESP32-S3 | Functional, not refactored | BLE NimBLE NUS service |
| Old Core/hal_timer_pwm.h/.cpp | ESP32-S3 | Functional, not refactored | LEDC 16kHz motor PWM |
| Old Core/pid_control.h/.cpp | ESP32-S3 | Skeleton only (Ki=Kd=0) | PID structure, not tuned |

---

## 4. M1 REPORT STATUS (Internship.pdf)

**Positive elements:**
- 5 sections present per template: Overview, Preliminary Solution, Expected Results,
  Evaluation Method, Implementation Plan
- Quantitative success targets defined: <5cm altitude error, <3s recovery, 400Hz loop, 5–7 min flight
- Market comparison table: DJI Tello vs Crazyflie 2.1 vs Proposed
- System block diagram: Power → Sensory Suite → ESP32-S3 → Actuation
- BOM table with component-function mapping
- IEEE-style citation [1]–[5] with reference list
- Work distribution table (Quốc Hưng / Minh Chiến per task)

**Critical deficiencies:**
1. Section 5 Timeline column is completely blank → violates criterion 1.5 directly
2. BOM table lacks unit price and total cost → cannot verify "BOM < 50% DJI Tello" claim
3. Report uses "Kalman filter" as proposed method, but current code uses Complementary Filter
   → internal contradiction; recommend framing CF as M1 baseline, Kalman as M2 target
4. Spec table lists "400Hz loop frequency" as current — actual code runs at ~200Hz super-loop
   → must label as "(Target)" not current measurement
5. Report length ~19 pages exceeds 15-page template limit
6. DLPF listed as 188Hz in report (0x01) but actual code uses 41Hz (0x03) — inconsistency

---

## 5. SLIDE DECK STATUS 
---
[Slide ở đây](../assets/Windify_M1_v2.pptx(2).pptx.pdf)

---

## 6. TEAM DYNAMICS & CONTEXT

- 2-person team: Quốc Hưng (primary ESP32, drone frame, RF joystick, tuning)
  and Minh Chiến (primary STM32, sensor validation, BLE prototyping)
- Partner (Quốc Hưng) has been tuning full drone + MPU6050 since internship period
  (code partially sourced from public repository + AI-assisted refinement)
- BLE control feature: requested by partner ~1 month ago, dropped by partner 2 days before deadline
- Minh Chiến developed STM32 pipeline independently, then ported knowledge to ESP32-S3 drn_ HAL
- Minh Chiến used AI-assisted code generation but performed independent debugging:
  I2C recovery logic, hardware anomaly detection (WHO_AM_I mismatch, offset error),
  MOSFET failure root-cause analysis (reverse diode polarity)
- Tone for defense: no blame attribution, factual work distribution, honest status reporting

---

## 7. DEFENSE STRATEGY NOTES

**Strongest assets (Minh Chiến):**
1. MPU6050 clone detection (WHO_AM_I=0x70) — hardware anomaly found through measurement
2. Offset calibration empirical result: 4596 vs theoretical 4096 → quantifiable hardware deviation
3. MOSFET failure forensics: identified reverse-polarity diode as root cause
4. Knowledge transfer: STM32 bare-metal → ESP32-S3 IDF (architecture migration documented)
5. Telemetry UART pipeline: verified data transmission via Hercules

**Framing for incomplete items:**
- 4-motor integration: "pending drone frame procurement — single motor PWM verified"
- BLE control: "deprioritized due to latency instability — architecture supports future integration"
- Dual-core RTOS: "target M2 — M1 validates sensor pipeline as prerequisite"
- Kalman filter: "proposed as M2 upgrade — M1 establishes Complementary Filter baseline"

---
*End of context log — v1.0 — 2026-04-21*