# TÀI LIỆU KỸ THUẬT NỘI BỘ — PROJECT WINDIFY (Group 21)

**Phiên bản:** Post-M2 / Pre-Final — cập nhật sau debate kỹ thuật  
**Ngày cập nhật:** Tháng 6, 2026  
**Thành viên:** Pham Minh Chien · Vo Quoc Hung  
**Trạng thái:** v1 ✅ PASS — v2 ⏳ Cần flight test

---

## MỤC LỤC

1. [Tóm tắt tình trạng](#1-tóm-tắt)
2. [GPIO Map chính thức](#2-gpio-map)
3. [Ba khái niệm Yaw Correction](#3-yaw-correction)
4. [DOF hệ thống hiện tại](#4-dof-status)
5. [So sánh firmware](#5-firmware-comparison)
6. [Trạng thái từng module](#6-modules)
7. [Kế hoạch tích hợp & lộ trình code](#7-integration)
8. [Debate kỹ thuật: Tilt Compensation & thứ tự phát triển](#8-debate)
9. [Arming Logic — đề xuất cải thiện](#9-arming)
10. [Phát hiện kỹ thuật: I2C vs SPI init order](#10-spi-i2c)
11. [I2C Bus Discrepancy — đã xác nhận](#11-i2c)
12. [Open Issues & Câu hỏi bảo vệ](#12-open-issues)

---

## 1. Tóm tắt

| Module | Trạng thái | Ghi chú |
|--------|-----------|---------|
| MPU6050 + Kalman | ✅ Flight tested | 1D per-axis Roll/Pitch |
| VL53L1X Altitude Hold | ✅ Flight tested | PD + Tilt Compensation |
| Yaw PID (Dynamic Bias) | ⚠️ Cải thiện nhưng chưa đủ | Macro-drift còn lại |
| PMW3901 — v1 bench test | ✅ PASS | dX/dY confirmed, IMU+ToF+Flow song song |
| PMW3901 — v2 Yaw Anchoring | ⏳ Cần flight test | Code xong, chưa bay |
| Position Hold (v3) | ❌ Chưa implement | Post-Final scope |

**Phát hiện quan trọng khi tích hợp v1:**
- GPIO 10/11/12/13 hoạt động tốt — lỗi trước đó do cắm nhầm MOSI/MISO
- Pin thực tế: MOSI→GPIO13, MISO→GPIO11 (ngược với M2 Report)
- SPI.begin() phải gọi **trước** tất cả I2C init — nếu không PMW3901 INIT FAIL

---

## 2. GPIO Map chính thức

**Bảng từ M2 Report Table 1 + xác nhận thực tế:**

| GPIO | Chức năng | Interface | Ghi chú |
|------|-----------|-----------|---------|
| 1 | Motor RR | PWM 8kHz/10-bit | AO3400A |
| 2 | Motor RL | PWM 8kHz/10-bit | AO3400A |
| 3 | Motor FL | PWM 8kHz/10-bit | AO3400A |
| 4 | Motor FR | PWM 8kHz/10-bit | AO3400A |
| 5 | MPU6050 SDA | I2C\_IMU | ✅ Confirmed |
| 6 | MPU6050 SCL | I2C\_IMU | ✅ Confirmed |
| 7 | Battery Monitor | ADC 12-bit | Voltage divider 2×100kΩ |
| 8 | VL53L1X SDA | I2C\_TOF | ✅ Confirmed |
| 9 | VL53L1X SCL | I2C\_TOF | ✅ Confirmed |
| 10 | PMW3901 CS | SPI | ✅ Confirmed |
| **11** | **PMW3901 MISO** | **SPI** | **⚠️ Ngược M2 Report — MISO thực tế** |
| 12 | PMW3901 SCK | SPI | ✅ Confirmed |
| **13** | **PMW3901 MOSI** | **SPI** | **⚠️ Ngược M2 Report — MOSI thực tế** |
| 14 | RC CH3 Throttle | PWM Interrupt | ELRS WeAct Mini RX |
| 15 | RC CH2 Pitch | PWM Interrupt | ELRS WeAct Mini RX |
| 16 | RC CH1 Roll | PWM Interrupt | ELRS WeAct Mini RX |
| 39 | RC CH4 Yaw | PWM Interrupt | ELRS WeAct Mini RX |
| 40 | RC CH5 AUX1/Arm | PWM Interrupt | ELRS WeAct Mini RX |
| 41 | RC CH6 AUX2/Mode | PWM Interrupt | ELRS WeAct Mini RX |
| 48 | RGB LED NeoPixel | WS2812 | Status indicator |

> **⚠️ Cần sửa trong Final Report:** M2 Table 1 ghi MOSI=11, MISO=13. Thực tế ngược lại. Sửa thành MOSI=13, MISO=11.

---

## 3. Ba khái niệm Yaw Correction

❗ **Misconception:** "2000 samples calibration fix được hết Yaw drift." Sai — static calibration chỉ là tầng 1 trong 3 tầng.

| | **1. Static Calibration** | **2. Dynamic Bias** | **3. Yaw Anchoring (PMW3901)** |
|--|--------------------------|--------------------|---------------------------------|
| **Biến** | `yawGyroOffset` | `dynamicYawBias` | `yawAnchorCorrection` |
| **Khi nào tính** | 1 lần khi khởi động | Mỗi 4ms trong flight | Mỗi 10ms khi forward flight gate mở |
| **Tham chiếu** | Gyro Z lúc yên | Gyro Z lúc ổn định | **Mặt đất — hoàn toàn độc lập** |
| **Fix được gì** | Bias hằng số | Bias trôi do nhiệt độ | Sai số tích lũy còn lại |
| **Cơ chế** | Avg 2000 samples | EMA α=0.000001 (TC≈4000s) | `atan2(ΔX, ΔY)` cross-track slip |
| **File** | Cả hai | `flycode` only | `flight-code-v2.ino` |
| **Điểm cốt lõi** | Gyro tự sửa lúc yên | Gyro tự sửa lúc bay | **Thoát khỏi gyro — dùng ground truth** |

✅ **PMW3901 vượt trội:** 1 và 2 vẫn tin vào gyro làm thước đo. PMW3901 quan sát drone di chuyển thế nào so với mặt đất thực — sai số gyro bao nhiêu cũng bắt được.

❗ **"PMW3901 không hoạt động khi hover?"**
✅ Không phải nhược điểm — khi hover, Yaw drift không gây hậu quả. Vấn đề chỉ xuất hiện khi bay tới mà mũi lệch hướng → đúng lúc forward flight gate mở. Giới hạn và nhu cầu trùng nhau hoàn toàn.

---

## 4. DOF hệ thống hiện tại

| DOF | Trục | Trạng thái | Sensor |
|-----|------|-----------|--------|
| Attitude | Roll | ✅ Kalman filtered | MPU6050 |
| Attitude | Pitch | ✅ Kalman filtered | MPU6050 |
| Attitude | Yaw | ⚠️ Tích phân + bias filter (drift còn lại) | MPU6050 |
| Position | Z | ✅ PD hold, flight tested | VL53L1X |
| Position | X | ❌ Chưa kiểm soát | — |
| Position | Y | ❌ Chưa kiểm soát | — |

**Lộ trình:** 3/6 DOF → **4/6** sau v2 Yaw Anchoring → **6/6** sau v3 Position Hold.

---

## 5. So sánh firmware

| Tiêu chí | `droneflightcode.ino` (Cũ) | `flycode_Yaw_ok_thang5.ino` (v0) | `flight-code-v2.ino` (Hiện tại) |
|----------|---------------------------|----------------------------------|----------------------------------|
| PWM Motor | 16kHz | 8kHz | 8kHz |
| I2C Bus | Single | Dual TwoWire | Dual TwoWire |
| Kalman Rmeasure | 0.13 | 0.22 | 0.22 |
| Roll/Pitch fusion | Rotate 45° | Không rotate | Không rotate |
| Yaw Static Bias | ✅ | ✅ | ✅ |
| Yaw Dynamic Bias | ❌ | ✅ | ✅ |
| Yaw I-term | ❌ | ✅ | ✅ |
| Altitude Hold | ❌ | ✅ | ✅ |
| PMW3901 | ❌ | ❌ | ✅ |
| Yaw Anchoring | ❌ | ❌ | ✅ |
| Arming | 2 bước (AUX2 mid) | 1 bước | 2 bước (throttle-low 1s) |

---

## 6. Trạng thái từng module

### 6.1 IMU & Kalman

**✅ Hoàn chỉnh, flight tested**

| Param | Giá trị | Ý nghĩa |
|-------|---------|---------|
| Rmeasure | 0.22 | Ít tin accelerometer (motor 8520 rung) |
| Qangle | 0.003 | Process noise góc |
| Qbias | 0.003 | Drift noise gyro |
| Loop | 250Hz (4ms) | Busy-wait cuối loop |

❗ **"Kalman 1D hay 2D?"**
✅ "1D" = ước lượng 1 góc per axis. Hai sensor (accel + gyro) đều là input vào mỗi 1D filter. Không thể làm 3D Kalman thiếu magnetometer — accel chỉ cảm nhận trọng lực, mù với phép xoay quanh trục Z.

### 6.2 VL53L1X Altitude Hold

**✅ Hoàn chỉnh, flight tested**

- Tilt Compensation: `1/(cos(roll)×cos(pitch))`, cap 1.15×
- 3 kịch bản: push throttle / pull throttle / deadband PD hold
- `P_alt = 0.09f`, `D_alt = 0.1f`
- Mode Short, timing budget 20ms, continuous 20ms

### 6.3 PMW3901 Optical Flow

**v1 ✅ PASS bench test — v2 ⏳ Cần flight test**

- Motion Mode OK, 100Hz non-blocking
- Pin thực tế: CS=10, MOSI=13, SCK=12, MISO=11
- SPI phải init TRƯỚC tất cả I2C (phát hiện khi tích hợp v1)
- v2: `atan2f(ΔX, ΔY)` → slip angle → inject P-only vào Yaw PID

**Quy trình đo Yaw drift (cần làm cho Final Report):**
```
1. Log fYaw 60s ở v0 (không có anchoring) → drift_rate_before [°/phút]
2. Log fYaw 60s ở v2 khi bay forward → drift_rate_after [°/phút]
3. So sánh và ghi vào Final Report
```

---

## 7. Kế hoạch tích hợp & lộ trình code

### 7.1 Các phiên bản

| File | Mục tiêu | Trạng thái |
|------|----------|-----------|
| `flycode_Yaw_ok_thang5.ino` | v0 baseline — Alt Hold + Yaw PID | ✅ Flight tested |
| `flight-code-v1-fixed.ino` | PMW3901 tích hợp, bench verify | ✅ PASS |
| `flight-code-v2.ino` | Yaw Anchoring active | ⏳ Cần flight test |
| `flight-code-v3.ino` | Position Hold X/Y | ❌ Post-Final |

### 7.2 v2 — Forward Flight Gate

```cpp
bool forwardFlightGate =
    (rcValue[1] > 1600)           &&  // Pitch >20% forward
    (abs(rcValue[0] - 1500) < 80) &&  // Roll near center
    (abs(stickYawRate) < 15)      &&  // Không ra lệnh Yaw
    (filteredHeight > 100)        &&  // >10cm trên không
    (throttle > 200);                 // Motor đang chạy
```

### 7.3 v2 — Yaw Anchoring math

```cpp
float slip_angle = atan2f((float)deltaX, (float)deltaY);  // radian
// Không cần velocity scaling — chỉ dùng góc (direction), không dùng magnitude
yawAnchorCorrection = constrain(slip_angle * YAW_ANCHOR_GAIN, -30, 30);
float yawRateError  = stickYawRate - gz_filtered + yawAnchorCorrection;
```

**Tuning `YAW_ANCHOR_GAIN`:** Bắt đầu 0.3, tăng 0.1 mỗi lần bay. Nếu Yaw oscillate → giảm. Max thực nghiệm ~1.5.

### 7.4 v3 — Position Hold (tương lai, Post-Final)

Khi nâng cấp lên Position Hold, cần thêm:
1. Velocity scaling: `vx = deltaX × filteredHeight × SCALE_FACTOR / dt`
2. Tích phân pos: `pos_x += vx × dt`
3. Tilt Compensation optical flow: dùng Roll/Pitch từ MPU6050 để tách chuyển động tịnh tiến vs xoay
4. X/Y PID → output là Roll/Pitch angle setpoint
5. Calibrate `SCALE_FACTOR` thực nghiệm

---

## 8. Debate kỹ thuật: Tilt Compensation & thứ tự phát triển

*Tổng hợp từ debate Chiến–Hùng (Gemini moderated, June 2026)*

### 8.1 3 bộ PID có tốn CPU không?

**Không.** ESP32-S3 xử lý 3 PID loops dưới 1% CPU. Inner loop (Attitude PID, 250Hz) và Outer loop (Alt Hold + Yaw Anchor, ~100Hz) cascaded hoàn toàn trong ngưỡng an toàn.

### 8.2 Có nên tắt PMW3901 khi drone nghiêng không?

❌ **Không nên tắt** — đây là điểm Chiến đúng.

- Với **v2 (Yaw Anchoring):** Tắt sensor khi nghiêng không gây "Loss of Odometry" vì không có tọa độ nào được tích lũy. Gate condition `forwardFlightGate` đã tự động tắt correction khi Roll lớn.
- Với **v3 (Position Hold):** Tắt sensor sẽ mất tracking vị trí. Phải dùng Tilt Compensation liên tục:

```
// Đơn giản (linear approx):
real_vx = raw_vx / cos(pitch)
real_vy = raw_vy / cos(roll)

// Đầy đủ (cần rotation matrix 3×3 từ Roll, Pitch, Yaw)
```

### 8.3 Thứ tự phát triển đúng

❗ Có 2 ngữ cảnh khác nhau — không được nhầm lẫn:

| Ngữ cảnh | Thứ tự đúng | Lý do |
|----------|-------------|-------|
| **Bench Testing (kiểm thử tay)** | ToF → PMW3901 → MPU6050 | Test cô lập từng sensor để xác nhận phần cứng |
| **Software Architecture / Flight Tuning** | MPU6050 → ToF → PMW3901 | Inner loop phải ổn định trước, outer loop mới có ý nghĩa |

Chiến đúng về bench testing order. Hưng đúng về flight tuning order. Đây là 2 giai đoạn khác nhau, không mâu thuẫn.

### 8.4 Tilt Compensation — khi nào cần?

| Use case | Cần Tilt Compensation? | Lý do |
|----------|------------------------|-------|
| Yaw Anchoring (v2) | ❌ Không cần | `atan2(ΔX, ΔY)` chỉ dùng hướng, không dùng magnitude |
| Position Hold (v3) | ✅ Bắt buộc | Cần `vx, vy` chính xác để tích phân vị trí |

---

## 9. Arming Logic — cải thiện từ v0

```cpp
// BƯỚC 1: Throttle giữ đáy (<1100µs) liên tục 1 giây
if (rcValue[2] < 1100) {
    if (millis() - throttleLowStart > 1000) safetyReleased = true;
} else { throttleLowStart = millis(); safetyReleased = false; }

// BƯỚC 2: AUX1 flip → armed
if (!isArmed && safetyReleased && isReadyToFly && rcValue[4] > 1550) {
    isArmed = true; safetyReleased = false;
    fYaw = 0; iTermRoll = 0; iTermPitch = 0; iTermYaw = 0;
}

// Emergency disarm (riêng biệt, không liên quan Yaw):
if (isArmed && (abs(fRoll) > 40 || abs(fPitch) > 40)) isArmed = false;
```

---

## 10. Phát hiện kỹ thuật: SPI phải init trước I2C

**Vấn đề phát hiện khi tích hợp v1:**
Gọi `SPI.begin()` + `flow.begin()` sau khi I2C đã init → PMW3901 INIT FAIL nhất quán.

**Root cause:** Trên ESP32-S3, khởi tạo I2C (`I2C_IMU.begin()`, `tof.init()`) ảnh hưởng GPIO matrix routing. Nếu SPI.begin() chạy sau → conflict.

**Fix:** Trong `setup()`:
```cpp
// ✅ Thứ tự đúng:
SPI.begin(SCK, MISO, MOSI, CS);  // 1. SPI trước
flow.begin();                      // 2. PMW3901 init
I2C_IMU.begin(5, 6, 400000);      // 3. I2C_IMU sau
I2C_TOF.begin(8, 9, 400000);      // 4. I2C_TOF sau
tof.init(); tof.startContinuous(); // 5. ToF sau cùng
```

---

## 11. I2C Bus Discrepancy — ✅ Đã xác nhận

| Nguồn | GPIO 5, 6 | GPIO 8, 9 |
|-------|-----------|-----------|
| M2 Report | VL53L1X ❌ sai | MPU6050 ❌ sai |
| Code + xác nhận Hưng (12:33, 6/2026) | MPU6050 ✅ | VL53L1X ✅ |

**Hành động:** Sửa Final Report Table 1: GPIO 5/6 = MPU6050, GPIO 8/9 = VL53L1X.

---

## 12. Open Issues & Câu hỏi bảo vệ

### Open Issues

| # | Vấn đề | Mức độ |
|---|--------|--------|
| 1 | Flight test v2, đo drift rate trước/sau anchoring | 🔴 Critical |
| 2 | Tune `YAW_ANCHOR_GAIN` (bắt đầu 0.3) | 🔴 Critical |
| 3 | Sửa M2 Report: GPIO 5/6=MPU6050, 8/9=VL53L1X, MOSI=13, MISO=11 | 🟠 High |
| 4 | Log fYaw drift rate [°/phút] v0 vs v2 cho Final Report | 🟠 High |
| 5 | Xác nhận idleSpeed=70 không gây motor stall trên batch thực tế | 🟡 Medium |

### Câu hỏi bảo vệ

**"Tại sao Yaw không dùng Kalman?"**
MPU6050 là 6-DOF. Accelerometer mù với phép xoay quanh trục Z. Không có reference → không filter được. Cần magnetometer (9-DOF) hoặc optical flow.

**"PMW3901 100Hz đủ cho 8kHz PWM không?"**
Cascaded control: PMW3901 thuộc outer loop ~100Hz, không cạnh tranh với inner loop. Quán tính vật lý drone không thay đổi trong 10ms.

**"Tilt Compensation ở đâu trong code?"**
Hiện tại v2 dùng `atan2(ΔX, ΔY)` — chỉ cần hướng, không cần Tilt Compensation. Tilt Compensation sẽ cần ở v3 (Position Hold) khi tính velocity thực từ pixel counts.

**"Hệ thống hiện tại là mấy DOF?"**
4/6 sau v2 (Roll, Pitch, Altitude, Yaw). Tiêu đề "6DOF" trong M2 Report là mục tiêu Final (v3).

**"3 bộ PID có tốn CPU không?"**
Dưới 1% CPU của ESP32-S3 FPU. Inner loop 250Hz + Outer loops ~100Hz hoàn toàn trong ngưỡng an toàn.

---

*Tài liệu tổng hợp từ: M2 Report, `flycode_Yaw_ok_thang5.ino`, `flight-code-v1-fixed.ino`, `flight-code-v2.ino`, `HungVo_IMU.h/.cpp`, debate kỹ thuật Chiến–Hùng (June 2026).*