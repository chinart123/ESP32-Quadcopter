# 🚁 Hệ Thống Điều Khiển RC Drone — Tài Liệu Toàn Diện Cho Newbie

> **Mục tiêu tài liệu:** Giải thích từ A→Z toàn bộ hệ thống RC Drone bằng ngôn ngữ đơn giản nhất, từ sóng vô tuyến cho đến cánh quạt quay. Áp dụng thực tế với phần cứng sẵn có.

---

## 📑 Mục Lục

- [1. Bức Tranh Toàn Cảnh](#1-bức-tranh-toàn-cảnh)
  - [1.1 Hệ thống RC Drone là gì?](#11-hệ-thống-rc-drone-là-gì)
  - [1.2 PID là gì? (Giải thích cho newbie hoàn toàn)](#12-pid-là-gì-giải-thích-cho-newbie-hoàn-toàn)
- [2. Giải Phẫu Tín Hiệu](#2-giải-phẫu-tín-hiệu)
  - [2.1 Kênh (Channel) là gì?](#21-kênh-channel-là-gì)
  - [2.2 PWM là gì?](#22-pwm-là-gì)
- [3. Phân Biệt Các Giao Thức](#3-phân-biệt-các-giao-thức)
  - [3.1 Sơ đồ phân tầng — Cái gì nằm ở đâu?](#31-sơ-đồ-phân-tầng--cái-gì-nằm-ở-đâu)
  - [3.2 Phân biệt ELRS và CRSF — Sự nhầm lẫn kinh điển](#32-phân-biệt-elrs-và-crsf--sự-nhầm-lẫn-kinh-điển)
  - [3.3 Cấu trúc một gói tin CRSF](#33-cấu-trúc-một-gói-tin-crsf)
- [4. Giải Phẫu Hệ Thống TBS Tango 2 Pro](#4-giải-phẫu-hệ-thống-tbs-tango-2-pro)
  - [4.1 Nhầm lẫn phổ biến nhất](#41-nhầm-lẫn-phổ-biến-nhất)
  - [4.2 Bên trong TBS Tango 2 Pro (TX) có gì?](#42-bên-trong-tbs-tango-2-pro-tx-có-gì)
  - [4.3 HC-12 tương đương với phần nào?](#43-hc-12-tương-đương-với-phần-nào)
- [5. Bảng So Sánh Toàn Diện](#5-bảng-so-sánh-toàn-diện)
- [6. Giải Thích Cuộc Tranh Luận](#6-giải-thích-cuộc-tranh-luận)
  - [6.1 Hai người bạn đang tranh luận gì?](#61-hai-người-bạn-đang-tranh-luận-gì)
  - [6.2 Ai đúng với phần cứng nào?](#62-ai-đúng-với-phần-cứng-nào)
  - [6.3 Tại sao cả hai đều cần thiết?](#63-tại-sao-cả-hai-đều-cần-thiết)
- [7. Sơ Đồ Kiến Trúc Phần Cứng](#7-sơ-đồ-kiến-trúc-phần-cứng)
- [8. Sơ Đồ Luồng Phần Mềm FC](#8-sơ-đồ-luồng-phần-mềm-fc)
- [9. Ghi Chú Linh Kiện Sẵn Có + Công Việc](#9-ghi-chú-linh-kiện-sẵn-có--công-việc)
  - [9.1 Bản đồ linh kiện hiện tại](#91-bản-đồ-linh-kiện-hiện-tại)
  - [9.2 Kiến Trúc Dual-Platform — Cách A (Độc Lập Song Song)](#92-kiến-trúc-dual-platform--cách-a-độc-lập-song-song)
  - [9.3 Sơ Đồ Nối Dây — WeAct ELRS Mini RX → ESP32-S3 Supermini](#93-sơ-đồ-nối-dây--weact-elrs-mini-rx--esp32-s3-supermini)
  - [9.4 Giai Đoạn 1 — Kịch Bản A: Phone → BT → JDY-33 → ESP32-S3](#94-giai-đoạn-1--kịch-bản-a-phone--bt--jdy-33--esp32-s3)
  - [9.5 Giai Đoạn 1 — Kịch Bản B: WeAct ELRS → 6 PWM → ESP32-S3](#95-giai-đoạn-1--kịch-bản-b-weact-elrs--6-pwm--esp32-s3)
  - [9.6 Giai Đoạn 2A (Hưng Võ) — Đọc 6 Kênh PWM bằng attachInterrupt()](#96-giai-đoạn-2a-hưng-võ--đọc-6-kênh-pwm-bằng-attachinterrupt)
  - [9.7 Giai Đoạn 2B (Bạn) — HITL: ESP32 Giả Lập IMU qua I2C](#97-giai-đoạn-2b-bạn--hitl-esp32-giả-lập-imu-qua-i2c)
  - [9.8 Giai Đoạn 3 — MPU6050 Thật + Tune PID](#98-giai-đoạn-3--mpu6050-thật--tune-pid)
  - [9.9 Phân Công Chân STM32F103 — Không Xung Đột](#99-phân-công-chân-stm32f103--không-xung-đột)
  - [9.10 PWM Output — drn_timer_pwm.c](#910-pwm-output--drn_timer_pwmc)
  - [9.11 UART CRSF Parser — STM32F103](#911-uart-crsf-parser--stm32f103)
  - [9.12 Bảng Roadmap Tổng Hợp](#912-bảng-roadmap-tổng-hợp)
---

## 1. Bức Tranh Toàn Cảnh 
[⬆️](#-mục-lục)

### 1.1 Hệ thống RC Drone là gì? 
[⬆️](#-mục-lục)

Hãy nghĩ đơn giản như điều khiển xe đồ chơi, nhưng phức tạp hơn nhiều:

```
[Tay Cầm Phi Công]  ──sóng radio──►  [Mạch Nhận Trên Drone]  ──dây──►  [Não Drone]  ──dây──►  [4 Motor]
  (Transmitter / TX)     OTA              (Receiver / RX)               (Flight Controller)      (ESC + Motor)
```

**Quadcopter cần 2 luồng thông tin song song để bay:**

| Luồng | Câu hỏi | Nguồn dữ liệu | Giao thức |
|-------|---------|---------------|-----------|
| **Setpoint** | *"Phi công muốn máy bay làm gì?"* | Tay cầm → RX → FC | CRSF / PWM |
| **Current State** | *"Máy bay đang ở trạng thái nào?"* | IMU (MPU6050) → FC | I2C / SPI |

> ⚠️ **Thiếu 1 trong 2 → PID không hoạt động → drone không bay được.**

---

### 1.2 PID là gì? (Giải thích cho newbie hoàn toàn) 
[⬆️](#-mục-lục)

PID = **P**roportional + **I**ntegral + **D**erivative — một thuật toán điều khiển.

**Ví dụ thực tế bằng lái xe ô tô:**

```
Bạn muốn đi 60 km/h  ← đây là SETPOINT (từ tay cầm)
Xe đang đi 40 km/h   ← đây là CURRENT STATE (từ IMU/cảm biến)
Sai số = 60 - 40 = 20 km/h

P: "Sai bao nhiêu thì nhấn ga bấy nhiêu"
I: "Sai lâu rồi mà chưa sửa thì nhấn thêm"
D: "Sai đang giảm nhanh thì bớt nhấn lại"

→ Kết quả: Xe tiến dần về 60 km/h, không vọt lên 80 rồi tụt xuống 50
```

**Áp dụng vào Drone:**
```
Phi công gạt cần sang phải 30%  → Setpoint = "Nghiêng phải 15°"
IMU báo drone đang ở            → Current State = "Đang ở 0° (thẳng)"
Error = 15° - 0° = 15°

PID tính → Motor trái quay nhanh hơn, Motor phải quay chậm lại
Drone từ từ nghiêng sang phải 15°
```

---

## 2. Giải Phẫu Tín Hiệu 
[⬆️](#-mục-lục)

### 2.1 Kênh (Channel) là gì? 
[⬆️](#-mục-lục)

Một "kênh" = một chiều điều khiển:

| Kênh | Tên | Ý nghĩa | Giá trị |
|------|-----|---------|---------|
| CH1 | Roll | Nghiêng trái/phải | 1000–2000 µs |
| CH2 | Pitch | Nghiêng trước/sau | 1000–2000 µs |
| CH3 | Throttle | Ga (lên/xuống) | 1000–2000 µs |
| CH4 | Yaw | Xoay trái/phải | 1000–2000 µs |
| CH5 | AUX1 | Bật/tắt tính năng | 1000/1500/2000 µs |
| CH6 | AUX2 | Bật/tắt tính năng | 1000/1500/2000 µs |

> `1000 µs` = tối thiểu (trái/dưới/tắt) · `1500 µs` = giữa (trung tâm) · `2000 µs` = tối đa (phải/trên/bật)

### 2.2 PWM là gì? 
[⬆️](#-mục-lục)

**PWM (Pulse Width Modulation)** = truyền thông tin bằng độ rộng xung điện.

```
CH3 = 1000µs (Ga thấp nhất):
 ▔|_____________________________|▔  (1ms cao, 19ms thấp)

CH3 = 1500µs (Ga giữa):
 ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔|______________|▔  (1.5ms cao, 18.5ms thấp)

CH3 = 2000µs (Ga tối đa):
 ▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔▔|▔  (2ms cao, 18ms thấp)
```

**Vấn đề của PWM truyền thống:** Cần **1 dây vật lý cho 1 kênh** → 6 kênh = 6 dây.

---

## 3. Phân Biệt Các Giao Thức 
[⬆️](#-mục-lục)

### 3.1 Sơ đồ phân tầng — Cái gì nằm ở đâu? 
[⬆️](#-mục-lục)

```
┌────────────────────────────────────────────────────────────────┐
│  TẦNG ỨNG DỤNG (Application Layer)                            │
│  "Phi công muốn Roll = 15°, Throttle = 60%"                   │
├────────────────────────────────────────────────────────────────┤
│  TẦNG GIAO THỨC DÂY (Wire Protocol — giữa RX và FC)           │
│  ► CRSF Protocol: gói tin UART 420k–1M baud                   │
│  ► PWM: 6 dây vật lý riêng lẻ                                 │
│  ► Custom HC-12: protocol tự định nghĩa qua UART               │
├────────────────────────────────────────────────────────────────┤
│  TẦNG GIAO THỨC SÓNG (Air Protocol — bay trong không khí)     │
│  ► ELRS: LoRa 2.4GHz, mã nguồn mở, độ trễ siêu thấp          │
│  ► Crossfire (TBS): 868/915MHz, độ trễ thấp, tầm xa           │
│  ► HC-12: 433MHz FSK, đơn giản, transparent serial            │
│  ► ESP-NOW: 2.4GHz, tận dụng chip Wi-Fi của ESP32             │
│  ► Bluetooth: 2.4GHz FHSS, tiêu chuẩn consumer               │
├────────────────────────────────────────────────────────────────┤
│  TẦNG VẬT LÝ (Physical Layer)                                 │
│  Sóng điện từ RF bay trong không khí                          │
└────────────────────────────────────────────────────────────────┘
```

### 3.2 Phân biệt ELRS và CRSF — Sự nhầm lẫn kinh điển 
[⬆️](#-mục-lục)

> **Quy tắc nhớ:** ELRS = Sóng trên trời · CRSF = Dây dưới đất

| | CRSF | ELRS |
|-|------|------|
| **Viết tắt** | CrossFire Serial Protocol | ExpressLRS |
| **Bản chất** | Giao thức UART (dây nối) | Giao thức sóng vô tuyến |
| **Nằm ở đâu** | Giữa RX module và FC | Bay trong không khí TX↔RX |
| **Ai tạo ra** | Team BlackSheep (TBS) — đóng | Cộng đồng mã nguồn mở |
| **Phần cứng** | Không cần (chỉ là bytes UART) | SX1276 / SX1280 LoRa chip |
| **Tốc độ** | 420,000 – 1,000,000 baud | Lên đến 1000 Hz packet rate |

**Mối quan hệ giữa chúng:**
```
[ELRS TX module]  ──ELRS sóng──►  [ELRS RX module]  ──CRSF UART──►  [STM32/ESP32 FC]
                   (Air Protocol)                     (Wire Protocol)
```

**Tại sao ELRS lại dùng CRSF để nói chuyện với FC?**

Vì khi ELRS ra đời, tất cả Flight Controller trên thế giới (Betaflight, ArduPilot, iNav) đã hỗ trợ đọc CRSF qua UART. ELRS "vay mượn" ngôn ngữ CRSF để không phải viết lại từ đầu.

> **Kết quả thực tế:** Code C/C++ xử lý UART interrupt trên STM32F103 hoàn toàn giống nhau dù bạn dùng TBS RX hay ELRS RX. Sự khác biệt chỉ nằm ở con sóng vô hình trên không.

### 3.3 Cấu trúc một gói tin CRSF 
[⬆️](#-mục-lục)

```
[ 0xC8 ][ LEN ][ TYPE ][ PAYLOAD ... ][ CRC8 ]
    ↑       ↑      ↑          ↑            ↑
  Sync    Độ dài  Loại    Dữ liệu kênh   Kiểm tra lỗi
  byte    frame   gói     (11 bit/kênh)  (CRC-8/DVB-S2)

TYPE = 0x16 → RC Channels Packed (16 kênh × 11 bit = 22 bytes payload)
```

**Cách unpack 11 bit cho 16 kênh:**
```c
// 16 kênh × 11 bit = 176 bit = 22 byte payload
ch[0]  = (buf[3]       | buf[4]  << 8) & 0x07FF;  // bit 0–10
ch[1]  = (buf[4]  >> 3 | buf[5]  << 5) & 0x07FF;  // bit 11–21
ch[2]  = (buf[5]  >> 6 | buf[6]  << 2 | buf[7] << 10) & 0x07FF;
// ... tiếp tục cho đến ch[15]

// Chuyển sang microseconds
uint16_t us = (uint16_t)(ch[i] / 1.638f + 880.0f);  // ~988–2012 µs
```

---

## 4. Giải Phẫu Hệ Thống TBS Tango 2 Pro 
[⬆️](#-mục-lục)

### 4.1 Nhầm lẫn phổ biến nhất 
[⬆️](#-mục-lục)

> ❌ **Sai:** "TBS Tango 2 Pro là cái module nhỏ gắn trên drone"
> ✅ **Đúng:** TBS Tango 2 Pro là **cái tay cầm cầm tay của phi công** (giống như tay cầm game PlayStation)

Cái gắn trên drone là **TBS Crossfire Nano RX** — một sản phẩm **bán riêng biệt**.

### 4.2 Bên trong TBS Tango 2 Pro (TX) có gì? 
[⬆️](#-mục-lục)

```
┌──────────────────────────────────────────────────────────┐
│              TBS Tango 2 Pro (Tay Cầm)                   │
│                                                          │
│  [Joystick L]──┐                                         │
│  [Joystick R]──┤                                         │
│  [Switch A]────┤→ [STM32 MCU]──→ [Crossfire TX Module]──→ Anten
│  [Switch B]────┤   (EdgeTX OS)    (RF chip 868/915MHz)   │
│  [Potmeter]────┘                                         │
│  [Display]←──── OLED/LCD                                 │
│  [Battery] 18650 × 2                                     │
└──────────────────────────────────────────────────────────┘
```

| Thành phần | Vai trò |
|-----------|---------|
| Joystick (Gimbal) | Đầu vào vật lý analog |
| MCU + EdgeTX OS | Đọc joystick, mix kênh, encode CRSF |
| Crossfire TX Module | Biến data thành sóng 868/915 MHz |
| Antenna | Phát/nhận sóng |

### 4.3 HC-12 tương đương với phần nào? 
[⬆️](#-mục-lục)

```
TBS Tango 2 Pro:
  Joystick → MCU(EdgeTX) → [Crossfire TX Module + Antenna]
                                       ↑
                         HC-12 chỉ tương đương phần này
                         (RF transceiver + MCU nhỏ)

HC-12 THIẾU:
  ✗ Không có Joystick vật lý
  ✗ Không có hệ điều hành xử lý đầu vào
  ✗ Không có thuật toán FHSS/nhảy tần chống nhiễu
  ✗ Không có CRSF encoding/decoding tinh vi
```

---
## 5. Bảng So Sánh Toàn Diện 
[⬆️](#-mục-lục)

| Tiêu chí | HC-12 | ESP-NOW | Bluetooth (JDY-33/HC-05) | ELRS | TBS Crossfire |
|---------|-------|---------|--------------------------|------|---------------|
| **Băng tần** | 433 MHz | 2.4 GHz | 2.4 GHz | 2.4 GHz | 868/915 MHz |
| **Tầm xa** | ~1 km | ~200 m | ~10–30 m | 1–10 km | 1–40 km |
| **Độ trễ** | ~20 ms | ~3–5 ms | ~30–100 ms | 3–10 ms | 5–15 ms |
| **Giao thức dây** | Transparent UART | SPI/UART | SPP UART | CRSF UART | CRSF UART |
| **Số kênh** | Tùy code | Tùy code | Tùy code | 16 kênh | 16 kênh |
| **Phần cứng** | Module rời | Tích hợp ESP32 | Module rời | SX127x/SX1280 | Chip TBS |
| **Độ khó setup** | ⭐ Rất dễ | ⭐⭐ Dễ | ⭐ Rất dễ | ⭐⭐⭐ Trung bình | ⭐⭐⭐ Trung bình |
| **Dùng cho RC drone** | ✅ Được (tầm gần) | ✅ Được (tầm gần) | ⚠️ Hạn chế | ✅ Chuyên dụng | ✅ Chuyên dụng |
| **Giá** | ~30k VNĐ | Miễn phí (có sẵn) | ~20–50k VNĐ | ~200–500k VNĐ | ~500k–2M VNĐ |

> **ESP-NOW là phần mềm, không phải phần cứng.** Nó là protocol tầng Data Link chạy trên RF hardware của ESP32. Bạn không "mua" ESP-NOW — nó đã có sẵn bên trong mọi chip ESP32.

---

## 6. Giải Thích Cuộc Tranh Luận 
[⬆️](#-mục-lục)

### 6.1 Hai người bạn đang tranh luận gì? 
[⬆️](#-mục-lục)

**Câu hỏi gốc:** *"ESP32 FC đọc lệnh từ phi công bằng cách nào?"*

#### 🧑 Ý của Hưng Võ — "Đọc lệnh RC" (PWM Input Capture)

```
[WeAct ELRS Mini RX] ──6 dây PWM──► [6 chân GPIO của ESP32-S3]
  CH1 Roll  → GPIO16
  CH2 Pitch → GPIO15
  ...
```

- **Tên kỹ thuật:** Multi-channel PWM Input Capture
- **Mục đích:** Test ESP32-S3 FC có đọc được lệnh từ tay cầm thật không
- **Bản chất:** Test *"Máy bay phản ứng với ý muốn phi công như thế nào?"*

#### 🧑 Ý của bạn — "Giả lập IMU" (HITL Simulation)

```
[ESP32 + MPU6050 thật] ──I2C──► [ESP32-S3 FC]
       HOẶC
[ESP32 GIẢ LẬP góc nghiêng] ──I2C──► [ESP32-S3 FC]
```

- **Tên kỹ thuật:** Hardware-In-The-Loop (HITL) Simulation
- **Mục đích:** Test thuật toán PID mà không cần drone bay thật
- **Bản chất:** Test *"Máy bay phản ứng với vật lý như thế nào?"*

### 6.2 Ai đúng với phần cứng nào? 
[⬆️](#-mục-lục)

| RX Module | Có chân PWM vật lý? | Ý "6 dây PWM" của Hưng đúng? | Giao thức thực tế |
|-----------|---------------------|------------------------------|-------------------|
| WeAct ELRS Mini RX V1 | ✅ Có CH1–CH6 | ✅ Đúng | ELRS air + PWM wire |
| TBS Crossfire Nano RX | ❌ Chỉ có TX/RX | ❌ Sai | Crossfire air + CRSF wire |
| HC-12 | ❌ Chỉ có TX/RX | ❌ Sai | FSK 433MHz + transparent UART |

> **Ý tưởng giả lập IMU của bạn đúng với MỌI loại RX** vì nó đi qua I2C — hoàn toàn độc lập.

### 6.3 Tại sao cả hai đều cần thiết? 
[⬆️](#-mục-lục)

```
                    ┌─────────────────────┐
SETPOINT ──────────►│                     │──────► Motor 1
(từ ý Hưng Võ)      │  PID Controller     │──────► Motor 2
                    │  trên ESP32-S3      │──────► Motor 3
CURRENT STATE ─────►│                     │──────► Motor 4
(từ ý bạn)          └─────────────────────┘

Thiếu SETPOINT  → FC không biết phi công muốn gì → không bay được
Thiếu CURRENT STATE → FC không biết drone đang ở đâu → không giữ thăng bằng
```

---

## 7. Sơ Đồ Kiến Trúc Phần Cứng 
[⬆️](#-mục-lục)

> 📌 **Placeholder — Xem file SVG đính kèm**

```
[hardware_architecture_hc12.svg]

Mô tả: Sơ đồ block diagram toàn hệ thống với phần cứng hiện tại.
- TX Side:   ESP32-S3 Supermini + HC-12 (giả lập tay cầm)
- Air Link:  HC-12 433MHz transparent serial
- FC:        STM32F103C8T6 + HC-12 RX + HC-05/JDY-33 (telemetry)
- IMU:       MPU6050 (hoặc ESP32 giả lập HITL)
- Output:    PWM → ESC → Motor
- Upgrade:   Ghi chú upgrade path → WeAct ELRS RX → CRSF
```

![Hardware Architecture Block Diagram](./hardware_architecture_hc12.svg)

---

## 8. Sơ Đồ Luồng Phần Mềm FC 
[⬆️](#-mục-lục)

> 📌 **Placeholder — Xem file SVG đính kèm**

```
[fc_software_flow_hc12.svg]

Mô tả: Flowchart xử lý dữ liệu HC-12 trên STM32F103C8T6.
- UART RX Interrupt (HC-12 data)
- Parse giao thức đơn giản (comma-separated hoặc binary frame)
- Extract 6 channel values → microseconds
- Failsafe check
- Main Loop: Đọc IMU → PID → PWM
- Upgrade path note: → CRSF parser khi có ELRS RX
```

![FC Software Flow](./fc_software_flow_hc12.svg)

---

## 9. Ghi Chú Linh Kiện Sẵn Có + Công Việc 
[⬆️](#-mục-lục)

### 9.1 Bản đồ linh kiện hiện tại 
[⬆️](#-mục-lục)

```
┌──────────────────────────────────────────────────────────────────────┐
│  PLATFORM A — ESP32-S3 Supermini  (Dự án chính)                     │
│  Vai trò: Flight Controller độc lập — bay được một mình             │
│  Trạng thái: Đang test giai đoạn 2A+2B                              │
├──────────────────────────────────────────────────────────────────────┤
│  PLATFORM B — STM32F103C8T6       (Học hỏi / bare-metal)            │
│  Vai trò: Flight Controller độc lập — bay được một mình             │
│  Trạng thái: Song song, tách biệt hoàn toàn khỏi Platform A         │
├──────────────────────────────────────────────────────────────────────┤
│  WeAct ELRS Mini RX V1  ← Hưng đang cầm trên tay                   │
│  Vai trò: RC Receiver — nhận sóng ELRS, xuất 6 dây PWM              │
├──────────────────────────────────────────────────────────────────────┤
│  HC-12 × 2     → RF Link 433MHz (TX sim → Platform B)               │
│  JDY-33/HC-05  → Bluetooth telemetry + debug                        │
│  MPU6050       → IMU thật (GĐ3+, dùng cho cả 2 platform)           │
└──────────────────────────────────────────────────────────────────────┘
```

### 9.2 Kiến Trúc Dual-Platform — Cách A (Độc Lập Song Song) 
[⬆️](#-mục-lục)

> Dự án này chọn **Cách A**: 2 FC hoàn toàn riêng biệt, mỗi cái có thể điều khiển drone một mình. Không phải master/slave, không phải co-processor.

```
                    ┌──────────────────────────────┐
  WeAct ELRS RX ──► │  Platform A: ESP32-S3        │ ──PWM──► ESC → Motor
  (6 dây PWM)       │  Flight Controller chính     │
  MPU6050 ─────────►│  ledcWrite() / MCPWM         │
                    └──────────────────────────────┘
                              (độc lập)
                    ┌──────────────────────────────┐
  HC-12 / ELRS ───► │  Platform B: STM32F103C8T6  │ ──PWM──► ESC → Motor
  (CRSF UART)       │  Flight Controller học hỏi  │
  MPU6050 ─────────►│  drn_timer_pwm.c / TIM3     │
                    └──────────────────────────────┘

Hai platform TEST RIÊNG, ghi nhật ký RIÊNG, code RIÊNG.
Không bao giờ cùng gắn vào 1 drone cùng lúc.
```

---

### 9.3 Sơ Đồ Nối Dây — WeAct ELRS Mini RX → ESP32-S3 Supermini 
[⬆️](#-mục-lục)

```
[Mạch WeAct ELRS Mini RX V1]        [ESP32-S3 Supermini — Flight Controller]
                                      
  CH1 (Roll)         ──────────────►  GPIO 16
  CH2 (Pitch)        ──────────────►  GPIO 15
  CH3 (Throttle)     ──────────────►  GPIO 14
  CH4 (Yaw)          ──────────────►  GPIO 39  ⚠️ JTAG MTCK
  CH5 (AUX 1)        ──────────────►  GPIO 40  ⚠️ JTAG MTDO
  CH6 (AUX 2)        ──────────────►  GPIO 41  ⚠️ JTAG MTDI
  5V  (VCC)          ──────────────►  5V / VBUS
  GND                ──────────────►  GND

⚠️  GPIO 39/40/41 mặc định được chip ESP32-S3 dùng cho JTAG debugger.
    Nếu đọc luôn 0 hoặc giá trị rác → JTAG đang chiếm dụng 3 chân này.
    Cách kiểm tra nhanh: Serial.println(digitalRead(39)) sau khi attachInterrupt.
    Nếu cần, thay bằng: GPIO 4 / 5 / 6 (không có conflict).
```
graph LR
    subgraph RX ["Mạch Nhận (WeAct ELRS Mini RX)"]
        direction TB
        VCC[5V / VCC]
        GND1[GND]
        CH1[CH1 - Roll]
        CH2[CH2 - Pitch]
        CH3[CH3 - Throttle]
        CH4[CH4 - Yaw]
        CH5[CH5 - AUX1]
        CH6[CH6 - AUX2]
    end

    subgraph FC ["Não Máy Bay (ESP32-S3 Supermini)"]
        direction TB
        VBUS[5V / VBUS]
        GND2[GND]
        P16[GPIO 16]
        P15[GPIO 15]
        P14[GPIO 14]
        P39[GPIO 39]
        P40[GPIO 40]
        P41[GPIO 41]
    end

    %% Cấp nguồn (Đỏ / Đen)
    VCC <== "Cấp nguồn 5V" ===> VBUS
    GND1 <== "Đồng bộ Mass" ===> GND2

    %% 6 Dây tín hiệu PWM (Màu xanh/nét đứt hoặc nét liền)
    CH1 -- "PWM (1000-2000µs)" --> P16
    CH2 -- "PWM (1000-2000µs)" --> P15
    CH3 -- "PWM (1000-2000µs)" --> P14
    CH4 -- "PWM (1000-2000µs)" --> P39
    CH5 -- "PWM (1000-2000µs)" --> P40
    CH6 -- "PWM (1000-2000µs)" --> P41

    %% Tùy chỉnh màu sắc cho sơ đồ
    style VCC fill:#ffcccc,stroke:#ff0000,stroke-width:2px
    style VBUS fill:#ffcccc,stroke:#ff0000,stroke-width:2px
    style GND1 fill:#cccccc,stroke:#333333,stroke-width:2px
    style GND2 fill:#cccccc,stroke:#333333,stroke-width:2px
    
    classDef signal fill:#e1f5fe,stroke:#03a9f4,stroke-width:2px;
    class CH1,CH2,CH3,CH4,CH5,CH6,P16,P15,P14,P39,P40,P41 signal;
---

## ── PLATFORM A: ESP32-S3 Supermini (Dự Án Chính) ──

### 9.4 Giai Đoạn 1 — Kịch Bản A: Phone → BT → JDY-33 → ESP32-S3 
[⬆️](#-mục-lục)

**Mục tiêu:** Bắt đầu nhanh nhất, không cần bạn kia, không cần WeAct.

```
[Điện thoại Android]
  Bluetooth Serial Monitor (CH Play)
  Gõ: $1500,1500,1000,1500,1000,1000
        ↓ BT SPP
[JDY-33 / HC-05] → UART2 (GPIO17 TX / GPIO18 RX của ESP32-S3)
        ↓
[ESP32-S3 FC] → parse → PID → LEDC PWM → ESC
```

**Bảng lệnh test nhanh:**

| Gõ vào app | Ý nghĩa | Kết quả |
|-----------|---------|---------|
| `$1500,1500,1000,1500,1000,1000` | Trung tâm, ga 0 | Motor dừng |
| `$1700,1500,1500,1500,1000,1000` | Roll phải 20% | Motor trái > phải |
| `$1500,1500,1600,1500,1000,1000` | Throttle 60% | Tất cả motor tăng |

**Checklist:**
- [ ] JDY-33: VCC→3.3V, GND→GND, TX→GPIO18(ESP32 RX), RX→GPIO17(ESP32 TX)
- [ ] Pair BT trên điện thoại (PIN: 1234)
- [ ] Gửi lệnh → LED GPIO2 nhấp nháy mỗi khi parse thành công
- [ ] Test failsafe: Tắt BT 1 giây → throttle tự về 1000

---

### 9.5 Giai Đoạn 1 — Kịch Bản B: WeAct ELRS → 6 PWM → ESP32-S3 
[⬆️](#-mục-lục)

**Mục tiêu:** Xác nhận ESP32-S3 đọc đủ 6 kênh từ WeAct, không delay.

> Đây là task hiện tại của **Hưng Võ** — dùng sơ đồ nối dây ở mục 9.3.

**Checklist:**
- [ ] Nối dây theo sơ đồ 9.3
- [ ] Flash code GĐ2A (mục 9.6) lên ESP32-S3
- [ ] Bật tay cầm ELRS + WeAct → Serial Monitor hiển thị 6 giá trị thay đổi
- [ ] Di chuyển từng cần → xác nhận đúng kênh (Roll không bị lẫn Throttle)
- [ ] Kiểm tra failsafe: tắt tay cầm → throttle về 1000 sau 500ms

---

### 9.6 Giai Đoạn 2A (Hưng Võ) — Đọc 6 Kênh PWM bằng `attachInterrupt()` 
[⬆️](#-mục-lục)

**Mục tiêu:** Code C++ đọc chính xác 6 tín hiệu PWM từ WeAct ELRS RX.

```cpp
// Platform A: ESP32-S3 — Đọc 6 kênh PWM từ WeAct ELRS Mini RX V1
// GĐ2A — Phụ trách: Hưng Võ
// File: src/rc_input.cpp

#define CH1_PIN 16  // Roll
#define CH2_PIN 15  // Pitch
#define CH3_PIN 14  // Throttle
#define CH4_PIN 39  // Yaw     ⚠️ JTAG MTCK — đổi GPIO4 nếu không đọc được
#define CH5_PIN 40  // AUX1   ⚠️ JTAG MTDO — đổi GPIO5 nếu không đọc được
#define CH6_PIN 41  // AUX2   ⚠️ JTAG MTDI — đổi GPIO6 nếu không đọc được

volatile uint32_t rise_time[6] = {0};
volatile uint16_t ch_us[6]     = {1500, 1500, 1000, 1500, 1000, 1000};
volatile bool     new_data      = false;
uint32_t          last_pkt_ms   = 0;

// Template ISR — đo độ rộng xung (RISING → FALLING)
void IRAM_ATTR isr_generic(int idx, int pin) {
    if (digitalRead(pin) == HIGH) {
        rise_time[idx] = micros();
    } else {
        uint32_t width = micros() - rise_time[idx];
        // Chấp nhận xung hợp lệ trong khoảng 800–2200µs
        if (width >= 800 && width <= 2200) {
            ch_us[idx]  = (uint16_t)width;
            last_pkt_ms = millis();
            new_data    = true;
        }
    }
}

// Mỗi kênh cần hàm ISR riêng (không thể truyền tham số vào ISR)
void IRAM_ATTR isr_ch1() { isr_generic(0, CH1_PIN); }
void IRAM_ATTR isr_ch2() { isr_generic(1, CH2_PIN); }
void IRAM_ATTR isr_ch3() { isr_generic(2, CH3_PIN); }
void IRAM_ATTR isr_ch4() { isr_generic(3, CH4_PIN); }
void IRAM_ATTR isr_ch5() { isr_generic(4, CH5_PIN); }
void IRAM_ATTR isr_ch6() { isr_generic(5, CH6_PIN); }

void rc_input_init() {
    const int   pins[6] = {CH1_PIN, CH2_PIN, CH3_PIN,
                            CH4_PIN, CH5_PIN, CH6_PIN};
    void (*isrs[6])()   = {isr_ch1, isr_ch2, isr_ch3,
                            isr_ch4, isr_ch5, isr_ch6};
    for (int i = 0; i < 6; i++) {
        pinMode(pins[i], INPUT);
        attachInterrupt(digitalPinToInterrupt(pins[i]), isrs[i], CHANGE);
    }
}

void rc_input_loop() {
    // Failsafe: không nhận gói tin trong 500ms → throttle về min
    if (millis() - last_pkt_ms > 500) {
        ch_us[2] = 1000;  // CH3 Throttle → tối thiểu
    }

    // Debug: in ra Serial Monitor để verify
    Serial.printf("CH: %4d %4d %4d %4d %4d %4d\n",
        ch_us[0], ch_us[1], ch_us[2],
        ch_us[3], ch_us[4], ch_us[5]);
}

// TODO GĐ sau: Nâng cấp lên MCPWM hardware peripheral
// ──────────────────────────────────────────────────────
// Lý do: 6 ISR phần mềm tranh nhau CPU → jitter ~5–50µs/kênh
// Khi PID chạy ở 1kHz, jitter tích lũy có thể gây drone rung (oscillate)
// Giải pháp: ESP-IDF mcpwm_capture_new_channel() — đo bằng hardware,
//            không chiếm CPU, chính xác đến 0.05µs
// Tham khảo: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/mcpwm.html
```

**Cấu trúc thư mục — Platform A:**
```
quadcopter-esp32/
├── assets/           ← Sơ đồ, datasheet, ảnh kết nối
│   ├── wiring/
│   │   └── weact_elrs_to_esp32s3.png
│   └── datasheets/
├── include/          ← Header files
│   ├── rc_input.h    ← Khai báo rc_input_init(), ch_us[]
│   ├── pid.h
│   └── imu.h
└── src/              ← Source files
    ├── main.cpp
    ├── rc_input.cpp  ← Code GĐ2A ở trên
    ├── pid.cpp
    └── imu.cpp
```

---

### 9.7 Giai Đoạn 2B (Bạn) — HITL: ESP32 Giả Lập IMU qua I2C 
[⬆️](#-mục-lục)

**Mục tiêu:** Test PID mà không cần MPU6050 thật — một ESP32 đóng vai "cảm biến giả".

> Giai đoạn 2A và 2B chạy **độc lập**. 2A test đường vào (RC input). 2B test đường phản hồi (IMU). Kết hợp cả hai → PID loop hoàn chỉnh.

**Kết nối:**
```
[ESP32 HITL Simulator]           [ESP32-S3 FC]
  GPIO8  (SDA) ──────────────►   SDA (GPIO8)
  GPIO9  (SCL) ──────────────►   SCL (GPIO9)
  GND          ──────────────►   GND
```

**Code ESP32 HITL — I2C Slave giả MPU6050:**
```cpp
// File: src/hitl_imu_sim.cpp  (chạy trên ESP32 riêng, KHÔNG phải ESP32-S3 FC)
#include <Wire.h>

float fake_roll  = 0.0f;   // Đơn vị: độ
float fake_pitch = 0.0f;
float fake_yaw   = 0.0f;

void setup() {
    Wire.begin(0x68);           // Giả lập địa chỉ MPU6050
    Wire.onRequest(send_data);
    Serial.begin(115200);
    Serial.println("HITL IMU Sim ready. Commands: R<val> P<val> Y<val>");
}

void send_data() {
    // Gửi 6 bytes: roll(int16) + pitch(int16) + yaw(int16) × 100
    int16_t r = (int16_t)(fake_roll  * 100);
    int16_t p = (int16_t)(fake_pitch * 100);
    int16_t y = (int16_t)(fake_yaw   * 100);
    Wire.write((uint8_t*)&r, 2);
    Wire.write((uint8_t*)&p, 2);
    Wire.write((uint8_t*)&y, 2);
}

void loop() {
    // Nhận lệnh từ Serial: "R15.5" → roll=15.5°, "P-10" → pitch=-10°
    if (Serial.available()) {
        char cmd = Serial.read();
        float val = Serial.parseFloat();
        if      (cmd == 'R') fake_roll  = val;
        else if (cmd == 'P') fake_pitch = val;
        else if (cmd == 'Y') fake_yaw   = val;
        Serial.printf("Set: R=%.1f P=%.1f Y=%.1f\n",
                      fake_roll, fake_pitch, fake_yaw);
    }
}
```

**Checklist:**
- [ ] Flash HITL code lên ESP32 phụ (không phải ESP32-S3 FC)
- [ ] Nối I2C: GPIO8(SDA) và GPIO9(SCL) giữa 2 board
- [ ] ESP32-S3 FC: code đọc I2C địa chỉ 0x68 @ 400kHz
- [ ] Gõ "R15" → Serial Monitor ESP32-S3 hiển thị roll thay đổi
- [ ] Gõ "P-20" → PID output thay đổi → duty cycle LEDC thay đổi

---

### 9.8 Giai Đoạn 3 — MPU6050 Thật + Tune PID 
[⬆️](#-mục-lục)

**Mục tiêu:** Thay HITL bằng cảm biến thật, tune Kp/Ki/Kd.

```
[MPU6050 thật]
  VCC → 3.3V
  GND → GND
  SDA → GPIO8  (ESP32-S3)    ← I2C bus 400kHz
  SCL → GPIO9  (ESP32-S3)
  AD0 → GND    (địa chỉ 0x68)
  INT → GPIO7  (optional — data ready interrupt)
```

> **Lưu ý cấu hình:** I2C bus được thiết lập ở **400kHz (Fast Mode)** — đảm bảo đọc được gyro ở 1kHz mà không bị bottleneck.

```cpp
// include/imu.h
#define IMU_I2C_FREQ   400000   // 400kHz Fast Mode
#define IMU_I2C_SDA    8
#define IMU_I2C_SCL    9
#define IMU_ADDR       0x68

// src/imu.cpp
void imu_init() {
    Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL, IMU_I2C_FREQ);
    // Wake up MPU6050
    write_register(IMU_ADDR, 0x6B, 0x00);
    // Set gyro range ±500°/s
    write_register(IMU_ADDR, 0x1B, 0x08);
    // Set accel range ±4g
    write_register(IMU_ADDR, 0x1C, 0x08);
    // DLPF 42Hz
    write_register(IMU_ADDR, 0x1A, 0x03);
}
```

**Checklist:**
- [ ] Tháo board HITL, cắm MPU6050 thật vào GPIO8/GPIO9
- [ ] Verify I2C scan tìm thấy 0x68
- [ ] Implement Complementary Filter: `angle = 0.98*(angle+gyro*dt) + 0.02*accel_angle`
- [ ] Tune PID qua BT: gõ "KP1.2", "KI0.05", "KD0.08" từ điện thoại

---

## ── PLATFORM B: STM32F103C8T6 (Học Hỏi / Bare-metal) ──

> Platform B hoàn toàn độc lập. Code, pinout, file riêng. Không liên quan đến Platform A trong quá trình test.

### 9.9 Phân Công Chân STM32F103 — Không Xung Đột 
[⬆️](#-mục-lục)

```
⚠️  CẢNH BÁO: PA9 = UART1_TX = TIM1_CH2  →  XUNG ĐỘT!
               PA10 = UART1_RX = TIM1_CH3  →  XUNG ĐỘT!
✅  GIẢI PHÁP: Dùng TIM3 (PA6/PA7/PB0/PB1) cho ESC PWM

┌──────────────────────────────────────────────────────────┐
│  UART1  (HC-12 hoặc ELRS RX):                           │
│    PA9  (UART1_TX) ──► HC-12 RX / WeAct TX              │
│    PA10 (UART1_RX) ◄── HC-12 TX / WeAct RX              │
│                                                          │
│  UART2  (JDY-33/HC-05 Bluetooth telemetry):             │
│    PA2  (UART2_TX) ──► BT RX                            │
│    PA3  (UART2_RX) ◄── BT TX (qua 1kΩ nếu HC-05 5V)   │
│                                                          │
│  I2C1  (MPU6050):                                       │
│    PB6  (I2C1_SCL) ──► MPU6050 SCL    [400kHz]         │
│    PB7  (I2C1_SDA) ──► MPU6050 SDA                     │
│                                                          │
│  TIM3  (4× ESC — KHÔNG xung đột với UART1):            │
│    PA6  (TIM3_CH1) ──► ESC Motor 1 (Front-Left)        │
│    PA7  (TIM3_CH2) ──► ESC Motor 2 (Front-Right)       │
│    PB0  (TIM3_CH3) ──► ESC Motor 3 (Rear-Right)        │
│    PB1  (TIM3_CH4) ──► ESC Motor 4 (Rear-Left)        │
│                                                          │
│  Misc:                                                   │
│    PC13 → LED debug                                     │
│    PA0  → MPU6050 INT (optional)                        │
└──────────────────────────────────────────────────────────┘
```

### 9.10 PWM Output — `drn_timer_pwm.c` 
[⬆️](#-mục-lục)

File `drn_timer_pwm.c` là file C chịu trách nhiệm xuất xung PWM trên **STM32F103C8T6**. ESP32-S3 (Platform A) dùng LEDC peripheral với file và hàm hoàn toàn khác.

```c
// src/drn_timer_pwm.c  — Platform B: STM32F103C8T6
// TIM3: PSC=71, ARR=2499 → f = 72MHz / (72 × 2500) = 400Hz
// Hoặc: PSC=17, ARR=999  → f = 72MHz / (18 × 1000) = 16kHz (DSHOT-like)

#define PWM_FREQ_STANDARD  400   // Hz — 4 cánh quạt tiêu chuẩn
#define PWM_FREQ_FAST      4000  // Hz — phản ứng nhanh hơn

// CCR value: 1000 = 1ms (min), 2000 = 2ms (max)
void drn_timer_pwm_set(uint8_t channel, uint16_t pulse_us) {
    // Clamp
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    switch (channel) {
        case 1: TIM3->CCR1 = pulse_us; break;  // PA6 → ESC Motor 1
        case 2: TIM3->CCR2 = pulse_us; break;  // PA7 → ESC Motor 2
        case 3: TIM3->CCR3 = pulse_us; break;  // PB0 → ESC Motor 3
        case 4: TIM3->CCR4 = pulse_us; break;  // PB1 → ESC Motor 4
    }
}
```

**Cấu trúc thư mục — Platform B:**
```
quadcopter-stm32/
├── assets/           ← Sơ đồ, datasheet
│   └── wiring/
│       └── stm32f103_pinout.png
├── include/          ← Header files
│   ├── drn_timer_pwm.h
│   ├── crsf_parser.h
│   └── imu.h
└── src/              ← Source files
    ├── main.c
    ├── drn_timer_pwm.c   ← TIM3 PWM 500Hz/16kHz
    ├── crsf_parser.c     ← Decode CRSF 0xC8 + CRC8 + 11-bit unpack
    └── imu.c             ← MPU6050 I2C @ 400kHz
```

### 9.11 UART CRSF Parser — STM32F103 
[⬆️](#-mục-lục)

```c
// src/crsf_parser.c  — Platform B
// Đọc ELRS RX hoặc TBS Crossfire RX qua UART1 @ 420000 baud

void USART1_IRQHandler(void) {
    uint8_t byte = (uint8_t)(USART1->DR & 0xFF);

    if (byte == 0xC8) {          // Sync byte CRSF
        rx_idx = 0;
        rx_buf[rx_idx++] = byte;
    } else if (rx_idx > 0) {
        rx_buf[rx_idx++] = byte;
        uint8_t expected_len = rx_buf[1] + 2;  // LEN field + 2 header bytes
        if (rx_idx >= expected_len) {
            if (crsf_validate_crc(rx_buf, rx_idx)) {
                if (rx_buf[2] == 0x16) {         // RC Channels Packed
                    crsf_unpack_channels(rx_buf);
                    last_pkt_ms = HAL_GetTick();
                }
            }
            rx_idx = 0;  // Reset cho frame tiếp theo
        }
    }
}
```

---

### 9.12 Bảng Roadmap Tổng Hợp 
[⬆️](#-mục-lục)

#### Platform A — ESP32-S3 Supermini

| GĐ | Việc cần làm | Kết quả kiểm chứng |
|----|-------------|-------------------|
| **1A** | Phone BT → JDY-33 → UART → parse CSV | 6 kênh thay đổi theo lệnh app |
| **1B** | WeAct ELRS → 6 PWM → attachInterrupt | Serial monitor hiển thị 6 giá trị |
| **2A** *(Hưng)* | Code `rc_input.cpp` đầy đủ, failsafe | Gạt cần → duty cycle LEDC đổi |
| **2B** *(Bạn)* | HITL I2C slave, PID phản ứng góc | Gõ "R30" → motor trái tăng |
| **3** | MPU6050 thật @ 400kHz, tune PID | Frame bench giữ thăng bằng |
| **4** | ESC + Motor + Frame | Bay được ✈️ |

#### Platform B — STM32F103C8T6

| GĐ | Việc cần làm | Kết quả kiểm chứng |
|----|-------------|-------------------|
| **1** | TIM3 init, `drn_timer_pwm.c`, xuất PWM | Oscilloscope: 400Hz đúng duty |
| **2** | UART1 CRSF parser, decode 0xC8 | Serial: 16 kênh unpack đúng |
| **3** | MPU6050 I2C @ 400kHz, filter | Roll/Pitch in ra đúng góc thật |
| **4** | PID + Mixer đầy đủ | Bench test: frame tự cân bằng |

---

*Tài liệu cập nhật lần cuối: Tháng 05/2026*
*Platform A: ESP32-S3 Supermini + WeAct ELRS Mini RX V1*
*Platform B: STM32F103C8T6 (bare-metal, học hỏi)*