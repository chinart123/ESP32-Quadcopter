# TÀI LIỆU KỸ THUẬT NỘI BỘ — PROJECT WINDIFY (Group 21)

**Phiên bản:** M2-Post / Pre-Final  
**Ngày cập nhật:** Tháng 6, 2026  
**Thành viên:** Pham Minh Chien · Vo Quoc Hung  
**Trạng thái:** Đang phát triển — Giai đoạn tích hợp PMW3901

---

## MỤC LỤC

1. [Tóm tắt tình trạng kỹ thuật](#1-tóm-tắt)
2. [Kiến trúc phần cứng & GPIO Map](#2-gpio-map)
3. [Ba khái niệm Yaw Correction](#3-yaw-correction)
4. [Trạng thái DOF hệ thống hiện tại](#4-dof-status)
5. [So sánh firmware cũ vs mới](#5-firmware-comparison)
6. [Trạng thái từng module](#6-modules)
7. [Kế hoạch tích hợp PMW3901](#7-integration)
8. [Arming Logic — đề xuất cải thiện](#8-arming)
9. [Phát hiện kỹ thuật: I2C Bus Discrepancy](#9-i2c)
10. [Open Issues & Câu hỏi bảo vệ](#10-open-issues)

---

## 1. Tóm tắt

Sau M2, firmware `flycode_Yaw_ok_thang5.ino` đã đạt trạng thái cất cánh và giữ độ cao. Hệ thống hiện kiểm soát được **3/6 DOF** (Roll, Pitch, Altitude). Vấn đề còn lại là **Yaw macro-drift** — mũi drone xoay dần do thiếu tham chiếu tuyệt đối cho trục Z. Nhiệm vụ giai đoạn Final: merge PMW3901 vào flycode để thực hiện Yaw Anchoring.

| Module | Trạng thái | Ghi chú |
|--------|-----------|---------|
| MPU6050 + Kalman Filter | ✅ Hoàn chỉnh, flight tested | 1D per-axis cho Roll/Pitch |
| VL53L1X Altitude Hold | ✅ Hoàn chỉnh, flight tested | PD + Tilt Compensation |
| Yaw PID (I-term + Dynamic Bias) | ⚠️ Cải thiện nhưng chưa đủ | Macro-drift còn lại |
| PMW3901 Motion Mode | 🔄 Test độc lập 10Hz OK | Chưa merge vào flycode |
| Yaw Anchoring | ❌ Chưa implement | Mục tiêu Final |

---

## 2. GPIO Map

Bảng chính thức từ **M2 Report — Table 1**. Pin 10–13 dành riêng cho PMW3901 SPI, không xung đột với bất kỳ peripheral nào đang hoạt động.

| GPIO | Chức năng | Interface |
|------|-----------|-----------|
| 1 | Motor RR | PWM 8kHz/10-bit |
| 2 | Motor RL | PWM 8kHz/10-bit |
| 3 | Motor FL | PWM 8kHz/10-bit |
| 4 | Motor FR | PWM 8kHz/10-bit |
| 5 | MPU6050 SDA | I2C\_IMU ✅ (code đúng, report sai) |
| 6 | MPU6050 SCL | I2C\_IMU ✅ (code đúng, report sai) |
| 7 | Battery Monitor | ADC 12-bit |
| 8 | VL53L1X SDA | I2C\_TOF ✅ (code đúng, report sai) |
| 9 | VL53L1X SCL | I2C\_TOF ✅ (code đúng, report sai) |
| **10** | **PMW3901 CS** | **SPI — chính thức M2** |
| **11** | **PMW3901 MOSI** | **SPI — chính thức M2** |
| **12** | **PMW3901 SCK** | **SPI — chính thức M2** |
| **13** | **PMW3901 MISO** | **SPI — chính thức M2** |
| 14 | RC CH3 Throttle | PWM Interrupt |
| 15 | RC CH2 Pitch | PWM Interrupt |
| 16 | RC CH1 Roll | PWM Interrupt |
| 39 | RC CH4 Yaw | PWM Interrupt |
| 40 | RC CH5 AUX1/Arm | PWM Interrupt |
| 41 | RC CH6 AUX2/Mode | PWM Interrupt |
| 48 | RGB LED NeoPixel | WS2812 |

**Khi merge PMW3901:** thay `#define PMW_CS_PIN 4 / SCK 7 / MOSI 5 / MISO 6` (pin cũ trong Optical Sensor-v5) bằng `10 / 12 / 11 / 13`.

---

## 3. Ba khái niệm Yaw Correction

❗ **Misconception thường gặp:** "2000 samples calibration fix được 60–70% Yaw drift là xong." Đúng một phần — static calibration chỉ fix bias hằng số. Còn 2 tầng sai số khác cần xử lý riêng.

| | **1. Static Calibration** | **2. Dynamic Bias** | **3. Yaw Anchoring (PMW3901)** |
|--|--------------------------|--------------------|---------------------------------|
| **Biến** | `yawGyroOffset` | `dynamicYawBias` | `yawAnchorCorrection` |
| **Khi nào tính** | 1 lần khi khởi động | Mỗi 4ms trong flight | Mỗi 10ms khi forward flight |
| **Tham chiếu** | Gyro Z lúc đứng yên | Gyro Z lúc bay ổn định | **Mặt đất — hoàn toàn độc lập** |
| **Fix được gì** | Bias hằng số từ đầu | Bias trôi chậm do nhiệt độ | Sai số tích lũy còn lại bất kể nguồn gốc |
| **Cơ chế** | Trung bình 2000 samples | EMA α=0.000001 (TC ≈ 4000s) | `atan2(ΔX, ΔY)` — cross-track slip |
| **File** | Cả hai | `flycode` only | Chưa implement |
| **Điểm cốt lõi** | Nhờ gyro tự sửa gyro lúc yên | Nhờ gyro tự sửa gyro lúc bay | **Thoát khỏi gyro — dùng thực tế bên ngoài** |

✅ PMW3901 vượt trội vì: phương pháp 1 và 2 vẫn đang tin vào gyro làm thước đo — chỉ cắt bớt được sai số có hệ thống của gyro, không xóa hoàn toàn. PMW3901 quan sát drone di chuyển thế nào so với mặt đất thực — sai số gyro bao nhiêu cũng bắt được.

❗ **"PMW3901 không hoạt động khi hover — đây là nhược điểm?"**  
✅ Không phải nhược điểm thực sự — đây là giới hạn vật lý. Khi hover, ΔX ≈ ΔY ≈ 0, `atan2(0,0)` không xác định. Nhưng khi hover tại chỗ, Yaw drift cũng không gây hậu quả gì — drone không di chuyển nên xoay mũi một chút không quan trọng. Vấn đề chỉ xuất hiện khi đang bay tới mà mũi lệch hướng, drone bay chéo — đúng lúc đó forward flight gate mở và correction can thiệp. Giới hạn và nhu cầu trùng nhau hoàn toàn.

---

## 4. Trạng thái DOF hệ thống hiện tại

❗ **Tiêu đề "6DOF" trong M2 Report là mục tiêu cuối, không phải trạng thái hiện tại.**

| DOF | Trục | Trạng thái | Sensor |
|-----|------|-----------|--------|
| Attitude | Roll | ✅ Kalman filtered | MPU6050 |
| Attitude | Pitch | ✅ Kalman filtered | MPU6050 |
| Attitude | Yaw | ⚠️ Tích phân + bias filter (drift còn lại) | MPU6050 |
| Position | Z (độ cao) | ✅ PD hold, flight tested | VL53L1X |
| Position | X (ngang) | ❌ Chưa kiểm soát | PMW3901 |
| Position | Y (dọc) | ❌ Chưa kiểm soát | PMW3901 |

**Lộ trình:** 3/6 DOF hiện tại → **4/6** sau Yaw Anchoring → **6/6** sau Position Hold.

❗ **"Sensor chỉ đo số rồi báo về MCU — cái gì thực sự kiểm soát X/Y/Z?"**  
✅ Phần mềm (PID trong MCU) mới là thứ kiểm soát. Sensor chỉ cung cấp số đo. VL53L1X và PMW3901 đều chỉ "đo và báo" — sự khác biệt là Z đã có PID loop hoàn chỉnh dùng số đo đó, còn X/Y thì chưa. Khi PMW3901 được merge và PID position hold được viết, X/Y mới được kiểm soát.

---

## 5. So sánh firmware

| Tiêu chí | `droneflightcode.ino` (Cũ) | `flycode_Yaw_ok_thang5.ino` (Mới) |
|----------|---------------------------|-------------------------------------|
| PWM Motor | 16kHz, 10-bit | 8kHz, 10-bit |
| I2C Bus | Single Wire | Dual TwoWire (I2C\_IMU + I2C\_TOF) |
| Kalman Rmeasure | 0.13 | 0.22 |
| Roll/Pitch Fusion | `fRoll = fR×0.7071 + fP×0.7071` (rotate 45°) | Trực tiếp từ Kalman, không rotate |
| Yaw — Static Bias | `yawGyroOffset` | `yawGyroOffset` (giữ nguyên) |
| Yaw — Dynamic Bias | ❌ Không có | ✅ EMA α=0.000001 |
| Yaw — I-term | ❌ P và D only | ✅ PID đầy đủ + anti-windup |
| Altitude Hold | ❌ Không có | ✅ VL53L1X PD + Tilt Compensation |
| Safety threshold | Roll/Pitch > 45° | Roll/Pitch > 40° |
| Arming Logic | 2 bước (safetyReleased) | 1 bước |
| idleSpeed | 120 | 70 |
| Calibration samples | 3000 | 2000 |
| PMW3901 | ❌ | ❌ (chưa tích hợp) |

### Roll/Pitch Fusion — 0.7071 là gì?

❗ **"Hưng có merge 2 thứ làm một và skip giai đoạn không?"**  
✅ Không — đây là phép xoay tọa độ 45° thuần phần mềm. Trên X-frame quadcopter, trục Roll/Pitch vật lý của drone lệch 45° so với trục X/Y của MPU6050. Code cũ bù bằng ma trận xoay. Code mới bỏ rotation — MPU6050 đã được mount thẳng theo trục drone hoặc sau test thấy không cần. Không liên quan gì đến merge hay tách measurement.

### Rmeasure 0.13 → 0.22 — ý nghĩa

❗ **"Tăng Rmeasure có nghĩa là 8kHz rung động hơn 16kHz?"**  
✅ Không kết luận được — hai con số được tune ở điều kiện khác nhau (tần số rung khác nhau). Rmeasure=0.22 là kết quả tuning empirical của Hưng cho 8kHz. Không thể so sánh trực tiếp để nói tần số nào rung hơn. Chỉ biết: tăng Rmeasure = filter tin accelerometer ít hơn, dựa gyro nhiều hơn.

### Calibration samples (3000→2000) và idleSpeed (120→70)

❗ **"Hai thông số này có liên quan nhau không?"**  
✅ Hoàn toàn độc lập. `calibration samples` ảnh hưởng đến độ chính xác của bias offset đầu vào Kalman (nhiều samples = trung bình tốt hơn = gyro input sạch hơn). `idleSpeed` là tốc độ motor tối thiểu khi armed — không liên quan Kalman hay sensor. Giảm idleSpeed 120→70: motor ít nóng hơn khi hover nhưng tăng nguy cơ **stall** (motor đứng hẳn nếu duty quá thấp vì không đủ torque thắng ma sát tĩnh).

---

## 6. Trạng thái từng module

### 6.1 IMU & Kalman Filter

**Trạng thái: ✅ Hoàn chỉnh, flight tested**

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| DLPF | 10Hz | Lọc hardware — cắt tần cao trước khi vào MCU |
| Gyro range | ±500 dps (hệ số 65.5) | Đủ cho maneuver drone nhỏ |
| Accel range | ±8g (hệ số 4096) | — |
| Rmeasure | 0.22 | Ít tin accelerometer |
| Qangle | 0.003 | Process noise của góc |
| Qbias | 0.003 | Drift noise của gyro |

❗ **"Kalman 1D là gì? Có phải 6 giá trị raw chỉ thành 1D thôi không? Có thể làm 3D không?"**  
✅ "1D" chỉ số chiều của state space được ước lượng, không phải số sensor input. Mỗi filter ước lượng 1 góc duy nhất — `kalmanR` cho Roll, `kalmanP` cho Pitch. Cả hai sensor (accel + gyro) đều là input vào mỗi 1D filter. Nội bộ thư viện Kalman.h dùng state vector `[angle, gyro_bias]` (2 phần tử) nhưng output là 1 góc.

Về 3D: **Không thể** thiếu magnetometer. Accelerometer chỉ cảm nhận vector trọng lực (luôn thẳng đứng) — phép xoay quanh trục Z (Yaw) không thay đổi vector này. Nên không có cách dùng `ax, ay, az` để tính Yaw. Cần magnetometer (MPU9250) hoặc thay thế bằng PMW3901 optical flow cho môi trường indoor.

### 6.2 VL53L1X Altitude Hold

**Trạng thái: ✅ Hoàn chỉnh, flight tested**

```cpp
// Tilt Compensation (flycode dòng 332-337)
float tiltCompensation = 1.0f / (cos(rollRad) * cos(pitchRad));
// Khi drone nghiêng góc θ, lực đẩy thực hướng lên = T×cos(θ)
// → Phải tăng throttle lên 1/cos(θ) để giữ độ cao
if (abs(fRoll) < 5 && abs(fPitch) < 5) tiltCompensation = 1.0f;  // skip gần thẳng
tiltCompensation = constrain(tiltCompensation, 1.0f, 1.15f);      // cap 15% ≈ tilt 30°
```

- Độ cao test: VL53L1X Short mode đáng tin đến ~1.3m. `tofHeight` constrain 30–2000mm. Indoor test thường 30–80cm
- `targetHeight` set động khi pilot bật AUX2 — không cố định cứng

❗ **"Safety threshold Roll/Pitch >40° có liên quan đến Yaw không?"**  
✅ Không. Đây là crash protection riêng biệt: nếu drone nghiêng >40° Roll hoặc Pitch, tự động disarm (cắt motor) để tránh tai nạn. Không có ngưỡng tương tự cho Yaw trong code.

### 6.3 Yaw PID & Dynamic Bias

**Trạng thái: ⚠️ Cải thiện nhưng chưa đủ — macro-drift còn lại**

```cpp
// Dynamic Bias — EMA cực chậm (α = 0.000001)
// Chỉ cập nhật khi drone ổn định:
if (abs(stickYawRate) < 8 && abs(fRoll) < 6 && abs(fPitch) < 6
    && abs(verticalSpeed) < 40 && throttle > 180) {
    dynamicYawBias = (dynamicYawBias * 0.999999f) + (rawGyroZ * 0.000001f);
}
// Time constant = 1/0.000001 × 4ms ≈ 4000 giây — không phải số hardcode
// Đây là tính toán thực của ESP32 FPU mỗi 4ms, không phải hằng số cố định
```

**Quy trình đo drift rate (cần làm trước Final Report):**
1. Flash code với `Serial.println(fYaw)` trong loop
2. Drone đặt yên hoặc hover ổn định, log 120 giây
3. `drift_rate = (fYaw_120s - fYaw_0s) / 120` [°/s]
4. Lặp 3 lần ở nhiệt độ khác nhau (vừa bật máy vs đã chạy 5 phút)
5. Ghi vào Final Report — hiện tại con số "60%" và "30–60 giây" là ước lượng chưa đo thực nghiệm

### 6.4 PMW3901 Optical Flow

**Trạng thái: 🔄 Đã test độc lập 10Hz, chưa tích hợp**

- Optical Sensor-v5: Motion Mode OK, `flow.readMotionCount(&deltaX, &deltaY)` với thư viện Bitcraze
- Pin cần cập nhật khi merge: `{4,5,6,7}` → `{10,11,12,13}` (M2 Report chính thức)
- Tần số mục tiêu sau merge: 100Hz (non-blocking millis 10ms)

---

## 7. Kế hoạch tích hợp PMW3901 → Yaw Anchoring

### 7.1 Kiến trúc tổng thể

```
OUTER LOOP (~100Hz)                    INNER LOOP (~250Hz)
┌──────────────────────┐              ┌──────────────────────────┐
│ PMW3901 Motion Mode  │              │ MPU6050 + Kalman         │
│ ΔX, ΔY @ 100Hz       │              │ Roll/Pitch filtered      │
│         ↓            │              │          ↓               │
│ slip = atan2(ΔX,ΔY)  │──P-only ──▶ │ Yaw PID (P+I+D)         │
│ (chỉ khi gate mở)    │  correction  │ Roll/Pitch PID           │
└──────────────────────┘              │          ↓               │
                                       │    Motor Mixer           │
         VL53L1X @ ~50Hz              │          ↓               │
         ΔHeight, vertSpeed ──────▶   │   PWM @ 8kHz             │
                                       └──────────────────────────┘
```

### 7.2 Forward Flight Gate — điều kiện kích hoạt

```cpp
bool forwardFlightGate =
    (rcValue[1] > 1600)           &&  // Pitch stick >20% forward
                                      // → PMW3901 cần ground flow thực, hover ΔX≈ΔY≈0
    (abs(rcValue[0] - 1500) < 80) &&  // Roll gần center
                                      // → tránh ΔX bị ô nhiễm bởi lệnh lách có chủ ý
    (abs(stickYawRate) < 15)      &&  // Không có lệnh Yaw
                                      // → tránh fight với pilot khi đang chủ động xoay
    (filteredHeight > 100)        &&  // Airborne >10cm
                                      // → phản xạ sàn <10cm không tin được
    (throttle > 200);                 // Motor đang chạy
                                      // → tránh correction khi đang hạ cánh
```

### 7.3 Yaw Anchoring implementation

```cpp
// Outer loop — thêm vào loop() sau khi đọc PMW3901
float yawAnchorCorrection = 0;

if (forwardFlightGate && pmw_dataReady) {
    float slip_angle = atan2f((float)deltaX, (float)deltaY);  // radian

    // 💡 Simple: P-only (đủ cho mục tiêu này)
    yawAnchorCorrection = slip_angle * YAW_ANCHOR_GAIN;  // tune từ 0.3
    yawAnchorCorrection = constrain(yawAnchorCorrection, -30, 30);

    // ⚙️ Nếu muốn tối ưu hơn: dùng P-D (Kp + Kd damping tránh oscillation)
    // hoặc P-I (Kp + Ki cho constant wind disturbance).
    // Tránh full PID vì inner Yaw đã có I-term — double integral khó tune.
}

// Inner loop — inject vào Yaw rate error (dòng 369 flycode)
float yawRateError = stickYawRate - gz_filtered + yawAnchorCorrection;
// PID bên dưới chạy bình thường
```

### 7.4 Velocity Scaling — KHÔNG cần cho Yaw Anchoring

❗ **"Velocity scaling có cần thiết không? Độ cao cao hơn thì sai số nhiều hơn?"**  
✅ `atan2(ΔX, ΔY)` chỉ tính góc hướng dịch chuyển, không phải magnitude. Góc này độc lập với độ cao — ở 2m hay 8m, nếu drone trượt 30° sang trái thì tỉ lệ ΔX/ΔY (và do đó atan2) vẫn như nhau. Velocity scaling (`ΔX × height × constant`) chỉ cần khi tính vận tốc thực m/s cho Position Hold (tương lai). Cho Yaw Anchoring: bỏ qua hoàn toàn.

**Khi nâng cấp lên Position Hold (tương lai), những thứ cần thêm:**
```cpp
// 1. Scale pixel → velocity (cần calibrate SCALE_FACTOR thực nghiệm)
float vx = (float)deltaX * filteredHeight * SCALE_FACTOR / dt_flow;
float vy = (float)deltaY * filteredHeight * SCALE_FACTOR / dt_flow;

// 2. Tích phân velocity → position (tự động, không hardcode)
pos_x += vx * dt;  // dt từ millis() — không cần set cứng
pos_y += vy * dt;

// 3. Reset gốc khi bật Position Hold
// target_x = 0; target_y = 0;

// 4. Thêm X/Y PID riêng
// Output = Roll/Pitch angle setpoint (không phải throttle)

// 5. Tilt compensation cho optical flow
// (MPU6050 Roll/Pitch cần thiết để tách chuyển động tịnh tiến vs xoay)
```

`SCALE_FACTOR` phải calibrate thực nghiệm: bay ngang tốc độ đã biết, đọc ΔX counts, tính ngược ra hệ số.

### 7.5 Implementation Plan

| Bước | Công việc | Ưu tiên |
|------|-----------|---------|
| 1 | Cập nhật SPI pins → `{10,11,12,13}` trong flycode | 🔴 Critical |
| 2 | Test Motion Mode với pin mới — xác nhận ΔX/ΔY OK | 🔴 Critical |
| 3 | Thêm `flow.readMotionCount()` non-blocking (millis 10ms) vào flycode | 🟠 High |
| 4 | Implement `forwardFlightGate` | 🟠 High |
| 5 | Implement `atan2f(ΔX,ΔY)` + inject vào Yaw PID | 🟠 High |
| 6 | Bench test: quan sát `yawAnchorCorrection` qua Serial khi xoay vật lý | 🟡 Medium |
| 7 | Flight test: tune `YAW_ANCHOR_GAIN` bắt đầu từ 0.3 | 🟡 Medium |
| 8 | Log Yaw drift trước/sau — ghi vào Final Report | 🟡 Medium |

---

## 8. Arming Logic — đề xuất cải thiện

Code mới (`flycode`) dùng 1 bước — đơn giản nhưng dễ arm bất ngờ do nhiễu tín hiệu.

**Đề xuất 2 bước dùng sustained throttle-low:**

```cpp
static unsigned long throttleLowStart = 0;
static bool safetyReleased = false;

// BƯỚC 1: Throttle giữ ở đáy liên tục 1 giây → mở khóa
if (rcValue[2] < 1100) {
    if (millis() - throttleLowStart > 1000) safetyReleased = true;
} else {
    throttleLowStart = millis();
    safetyReleased = false;  // Reset nếu throttle nhấc lên
}

// BƯỚC 2: AUX1 flip → arm
if (!isArmed && safetyReleased && rcValue[4] > 1550) {
    isArmed = true;
    armStartTime = millis();
    safetyReleased = false;  // Reset để lần sau phải làm lại bước 1
    fYaw = 0; iTermRoll = 0; iTermPitch = 0; iTermYaw = 0;
}

// DISARM:
if (rcValue[4] < 1450) { isArmed = false; safetyReleased = false; throttleLowStart = millis(); }

// EMERGENCY DISARM (crash protection — giữ nguyên):
if (isArmed && (abs(fRoll) > 40 || abs(fPitch) > 40)) isArmed = false;
```

Ưu điểm so với code cũ (`safetyReleased` via AUX2 mid 1490–1510µs): cửa sổ 20µs rất dễ miss. Dùng sustained throttle-low 1s không phụ thuộc RC channel thứ 6, không thể miss, loại trừ arm do rung tay.

---

## 9. Phát hiện kỹ thuật: I2C Bus Discrepancy ✅ Đã xác nhận

| Nguồn | GPIO 5, 6 | GPIO 8, 9 |
|-------|-----------|-----------|
| M2 Report Table 1 | VL53L1X (TOF) — ❌ sai | MPU6050 (IMU) — ❌ sai |
| flycode source code | `I2C_IMU` → MPU6050 ✅ | `I2C_TOF` → VL53L1X ✅ |

**Xác nhận từ Vo Quoc Hung (12:33, tháng 6/2026):** Wiring thực tế đã được thay đổi sau khi viết M2 Report. Code phản ánh đúng hardware thực tế.

**Hành động cần làm trước Final defense:** Sửa M2 Report Table 1:

> GPIO 5/6 = **MPU6050 (I2C\_IMU / SDA-SCL)**  
> GPIO 8/9 = **VL53L1X (I2C\_TOF / SDA-SCL)**

---

## 10. Open Issues & Câu hỏi bảo vệ

### Open Issues

| # | Vấn đề | Mức độ |
|---|--------|--------|
| 1 | Đo drift rate thực nghiệm (log fYaw 120s) trước khi viết Final Report | 🟠 High |
| 2 | Merge PMW3901 vào flycode với pin GPIO 10–13 | 🔴 Critical |
| 3 | Calibrate `YAW_ANCHOR_GAIN` qua flight test | 🟡 Medium |
| 4 | Sửa I2C bus assignment trong Final Report (GPIO 5/6=MPU6050, 8/9=VL53L1X) — đã xác nhận | 🔴 Critical |
| 5 | Xác nhận `idleSpeed = 70` không gây stall trên batch motor thực tế | 🟡 Medium |

### Câu hỏi bảo vệ — gợi ý trả lời

**"Tại sao dùng Kalman thay vì Complementary Filter?"**  
Dynamic gain K thích nghi với mức độ nhiễu — R=0.22 được tune cho profile rung động của motor 8520 coreless. CF dùng trọng số tĩnh 0.98/0.02, không adjust khi điều kiện thay đổi.

**"Tại sao Yaw không dùng Kalman?"**  
MPU6050 là 6-DOF. Accelerometer chỉ cảm nhận vector trọng lực — phép xoay quanh trục Z không thay đổi vector này, nên accelerometer mù với Yaw. Không có reference → không filter được. Cần magnetometer (9-DOF) hoặc optical flow.

**"PMW3901 100Hz có đủ cho drone 8kHz PWM không?"**  
Cascaded control: PMW3901 thuộc outer loop ~100Hz, không cạnh tranh trực tiếp với inner loop. Quán tính vật lý của drone không thay đổi trong 10ms — 100Hz thừa sức detect drift trước khi tích lũy thành vấn đề.

**"Tại sao dùng 8kHz thay vì 16kHz PWM?"**  
8kHz tạo torque tốt hơn cho motor 8520 coreless. 16kHz gây switching losses cao hơn ở AO3400A tại tần số này, dẫn đến heating. Trade-off giữa smoothness và nhiệt.

**"Hệ thống hiện tại là mấy DOF?"**  
Hiện tại kiểm soát tốt 3/6 DOF (Roll, Pitch, Altitude). Yaw đang ở trạng thái partial (bias filter nhưng chưa anchoring). Sau tích hợp PMW3901: 4/6. Tiêu đề "6DOF" trong report là mục tiêu Final.

---

*Tài liệu tổng hợp từ: M2 Report (Group 21), `flycode_Yaw_ok_thang5.ino`, `droneflightcode.ino`, `HungVo_IMU.h/.cpp`, và toàn bộ quá trình phát triển Optical Sensor-v1 đến v5.*