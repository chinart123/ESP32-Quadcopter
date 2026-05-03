# Tài Liệu Kỹ Thuật: Thiết Kế Hệ Thống Điều Khiển Drone (Bluetooth App & Firmware)

**Ngày lập:** 02/05/2026  
**Phiên bản:** v2.0 (Cải tiến toàn diện)  
**Chủ đề:** Đánh giá cấu trúc phần mềm và phần cứng cho hệ thống giao tiếp Bluetooth giữa Drone và Ứng dụng điều khiển Android  
**Nền tảng mục tiêu:** ESP32-S3 Supermini (FreeRTOS + ESP-IDF) và STM32F103C8T6 (Bare-metal)  
**Tác giả:** Tài liệu kỹ thuật nội bộ - Đồ án tốt nghiệp  

---

## Mục Lục

1. [Tổng quan hệ thống](#1-tổng-quan-hệ-thống)
2. [Kiến trúc hệ thống & Sơ đồ luồng dữ liệu](#2-kiến-trúc-hệ-thống--sơ-đồ-luồng-dữ-liệu)
3. [So sánh giao thức: BLE vs Bluetooth Classic (SPP)](#3-so-sánh-giao-thức-ble-vs-bluetooth-classic-spp)
4. [Bảng phân tích độ phức tạp: Thiết kế Bluetooth App & Firmware](#4-bảng-phân-tích-độ-phức-tạp-thiết-kế-bluetooth-app--firmware)
5. [Cấu trúc gói tin & Định nghĩa giao thức điều khiển](#5-cấu-trúc-gói-tin--định-nghĩa-giao-thức-điều-khiển)
6. [Đánh giá chuyên sâu: Nền tảng nào phù hợp hơn?](#6-đánh-giá-chuyên-sâu-nền-tảng-nào-phù-hợp-hơn)
7. [Rủi ro kỹ thuật & Cách phòng tránh](#7-rủi-ro-kỹ-thuật--cách-phòng-tránh)
8. [Khuyến nghị triển khai & Lộ trình phát triển](#8-khuyến-nghị-triển-khai--lộ-trình-phát-triển)
9. [Tài liệu tham khảo & Nguồn mở](#9-tài-liệu-tham-khảo--nguồn-mở)
10. [Phụ lục kỹ thuật](#10-phụ-lục-kỹ-thuật)

---

## 1. Tổng quan hệ thống

Việc xây dựng một hệ thống điều khiển drone hoàn chỉnh qua Bluetooth yêu cầu hai thành phần cốt lõi phối hợp chặt chẽ:

1. **Firmware (MCU):** Mã nguồn chạy trên vi điều khiển để nhận lệnh, bóc tách gói tin, điều khiển tốc độ động cơ (ESC/PWM), đọc cảm biến IMU (MPU-6050/ICM-42688), và thực thi vòng điều khiển PID.
2. **App Android:** Ứng dụng phát tín hiệu điều khiển thời gian thực gồm các trục: **Throttle** (ga), **Roll** (lăn trái/phải), **Pitch** (ngả trước/sau), **Yaw** (xoay trái/phải).

### 1.1 Thông số kỹ thuật mục tiêu

| Thông số | Yêu cầu tối thiểu | Lý tưởng |
|---|---|---|
| **Độ trễ điều khiển (Latency)** | < 100ms | < 30ms |
| **Tần suất gửi lệnh** | 10 Hz | 50 Hz |
| **Khoảng cách điều khiển** | 10m | 30m+ |
| **Tải gói tin mỗi lần** | < 20 bytes | 8–12 bytes |
| **Khả năng phục hồi khi mất tín hiệu** | Failsafe (throttle = 0) | Hover tự động |

---

## 2. Kiến trúc hệ thống & Sơ đồ luồng dữ liệu

### 2.1 Kiến trúc tổng thể

```
┌─────────────────────────────────────────────────┐
│               ANDROID APP                        │
│  ┌──────────┐   ┌──────────┐   ┌─────────────┐  │
│  │ Joystick │   │ Throttle │   │  Arm/Disarm │  │
│  │ (Roll/   │   │  Slider  │   │   Button    │  │
│  │  Pitch)  │   │          │   │             │  │
│  └────┬─────┘   └────┬─────┘   └──────┬──────┘  │
│       └──────────────┼─────────────────┘          │
│              ┌───────▼────────┐                   │
│              │  Packet Builder│ (serialize → bytes)│
│              └───────┬────────┘                   │
│                      │ BLE GATT Write / SPP Send  │
└──────────────────────┼─────────────────────────────┘
                       │ Bluetooth (BLE / Classic)
┌──────────────────────▼─────────────────────────────┐
│                    MCU (ESP32-S3 / STM32F103)       │
│  ┌────────────────┐    ┌────────────────────────┐   │
│  │  BT Receiver   │    │    Packet Parser        │   │
│  │  (NimBLE/UART) │───>│  (Safe String / DMA)   │   │
│  └────────────────┘    └──────────┬─────────────┘   │
│                                   │                  │
│              ┌────────────────────▼──────────────┐   │
│              │      PID Controller               │   │
│              │  (Roll / Pitch / Yaw / Altitude)  │   │
│              └──────────┬────────────────────────┘   │
│                         │ PWM Output                 │
│  ┌──────────────────────▼────────────────────────┐   │
│  │  Motor Driver (ESC × 4)                       │   │
│  │  Motor1  Motor2  Motor3  Motor4               │   │
│  └───────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────┘
```

### 2.2 Luồng dữ liệu chi tiết (Data Flow)

```
[Ngón tay người dùng]
        │
        ▼ (touch event ~60Hz)
[Joystick View → normalize → [-100, 100]]
        │
        ▼ (timer 20ms = 50Hz)
[Đóng gói: Header + Roll + Pitch + Yaw + Throttle + Checksum]
        │
        ▼ (BLE Write Without Response / UART TX)
[Bluetooth PHY Layer]  ←── độ trễ vật lý ~5–15ms
        │
        ▼
[MCU Interrupt / GATT Callback]
        │
        ▼ (bóc tách, kiểm tra CRC)
[Giá trị điều khiển → PID setpoint]
        │
        ▼ (vòng lặp PID ~250–500Hz)
[PWM → ESC → Motor RPM]
```

---

## 3. So sánh giao thức: BLE vs Bluetooth Classic (SPP)

| Tiêu chí | BLE (Bluetooth Low Energy) | Bluetooth Classic (SPP) |
|---|---|---|
| **Phiên bản Bluetooth** | 4.0+ | 2.0+ |
| **Tốc độ tối đa (thực tế)** | ~1–3 Mbps (PHY) / ~100 kbps ứng dụng | ~1–3 Mbps |
| **Độ trễ (Latency)** | 7.5ms – 4000ms (Connection Interval) | 20–100ms |
| **Tiêu thụ điện** | Rất thấp (~10–50 mA) | Cao (~150–300 mA) |
| **Khoảng cách** | 10–100m (tùy công suất) | 10–30m |
| **ESP32-S3 hỗ trợ** | ✅ Native (NimBLE / Bluedroid) | ✅ (Bluedroid) |
| **STM32F103 hỗ trợ** | ❌ Cần module BLE rời (HM-10) | ✅ HC-05 / JDY-31 |
| **Android API** | `BluetoothLeScanner` / GATT | `BluetoothSocket` / SPP UUID |
| **Phù hợp drone** | ✅ Khuyến nghị (latency thấp, cấu hình được) | ⚠️ Được, nhưng latency cao hơn |
| **Độ phức tạp code Android** | Cao hơn (GATT callback, MTU) | Thấp hơn (stream I/O đơn giản) |

> **Kết luận giao thức:** Với drone điều khiển thời gian thực, ưu tiên **BLE với Connection Interval = 7.5–20ms** để đạt latency thấp nhất. Nếu dùng STM32 + HC-05, chấp nhận latency 50–100ms.

---

## 4. Bảng phân tích độ phức tạp: Thiết kế Bluetooth App & Firmware

| Nền tảng (MCU) | Hướng triển khai | Firmware (Nhúng) | App Android | Ưu điểm | Nhược điểm | Chi phí linh kiện | Kỹ năng C cần có |
|---|---|---|---|---|---|---|---|
| **ESP32-S3 Supermini** *(FreeRTOS / ESP-IDF)* | **Tự viết (Custom)** | Viết BLE GATT Server với NimBLE. Tự định nghĩa Service UUID + Characteristic UUID cho từng kênh (Roll, Pitch, Yaw, Throttle). Dùng `xQueueSend()` truyền dữ liệu từ BLE callback task sang PID task. | Viết Android app (Java/Kotlin) dùng `BluetoothGatt` API. Kết nối → Discover Services → Write Characteristic mỗi 20ms. | - Latency thấp nhất (~10–30ms). Kiểm soát hoàn toàn giao thức. Bảo mật tốt. | - Học curve dốc với GATT / UUID / MTU. Phải rành FreeRTOS task management. | ~$5–8 (ESP32-S3 Supermini) | **Cao:** Safe String, ring buffer, FreeRTOS queue, con trỏ hàm callback |
| | **Dùng bên thứ 3 (Blynk / Dabble)** | Cài Blynk library hoặc Dabble ESP32. Lắng nghe chuỗi định dạng tiêu chuẩn qua BLE Serial. | Tải Blynk App / Dabble App. Kéo thả joystick widget. Nhập tên WiFi/BLE. Xong. | - Phát triển trong vài giờ. Không cần viết UI. Phù hợp để test thuật toán PID nhanh. | - Phụ thuộc server Blynk (cần internet). Dabble có latency ~50–80ms. UI không custom được nhiều. | ~$5–8 | **Thấp:** Đọc chuỗi định dạng sẵn |
| **STM32F103C8T6** *(Bare-metal / HAL)* | **Tự viết (Custom)** | Viết UART DMA Circular Buffer để nhận byte liên tục từ HC-05/JDY-31. Tự code ring buffer + hàm bóc tách gói tin. Cấu hình TIM1 PWM cho ESC. | Viết Android app dùng `BluetoothSocket` kết nối SPP UUID (`00001101-...`). Gửi chuỗi lệnh mỗi 50ms. | - Deterministic timing, không bị ảnh hưởng bởi OS scheduler. Hiểu sâu phần cứng. | - Tốn thời gian đấu nối UART, chuyển mức logic 3.3V. Bare-metal ngắt UART dễ gây Hard Fault. | ~$3 (STM32) + ~$5 (HC-05) = ~$8 | **Rất cao:** DMA, ring buffer, con trỏ hàm ISR, bóc tách chuỗi từ stream |
| | **Dùng STM32CubeMX + MIT App Inventor** | Gen code HAL bằng CubeMX. Cấu hình UART + DMA IT. Ghép byte thủ công trong callback. | Thiết kế giao diện bằng khối (Block) trong MIT App Inventor. Gửi chuỗi `"T:50,R:10,P:-5,Y:0\n"` | - Tốc độ prototyping nhanh cho phần app. CubeMX giảm code boilerplate. | - HAL không có cơ chế bóc tách chuỗi an toàn. MIT App dễ gửi ký tự lỗi. Không có Try-Catch ở firmware. | ~$8 | **Trung bình:** Hiểu HAL callback, tự ghép byte, xử lý overflow |

---

## 5. Cấu trúc gói tin & Định nghĩa giao thức điều khiển

Đây là phần **thường bị bỏ qua trong các tài liệu tương tự** nhưng cực kỳ quan trọng để đảm bảo tính ổn định của hệ thống.

### 5.1 Giao thức nhị phân (Binary Protocol) — Khuyến nghị

Định dạng nhị phân nhỏ gọn, parse nhanh, không bị lỗi encoding:

```
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┐
│ HEADER │  THR   │  ROLL  │ PITCH  │  YAW   │ FLAGS  │  CRC8  │
│  0xAA  │ int8_t │ int8_t │ int8_t │ int8_t │ uint8_t│ uint8_t│
│ 1 byte │ 1 byte │ 1 byte │ 1 byte │ 1 byte │ 1 byte │ 1 byte │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┘
                   Tổng: 7 bytes/gói tin
```

**Giải thích các trường:**

| Trường | Kiểu | Phạm vi | Ý nghĩa |
|---|---|---|---|
| `HEADER` | `uint8_t` | `0xAA` cố định | Byte đồng bộ hóa gói tin |
| `THR` | `int8_t` | 0 → 100 | Throttle (ga, 0 = dừng, 100 = full) |
| `ROLL` | `int8_t` | -100 → +100 | Nghiêng trái/phải |
| `PITCH` | `int8_t` | -100 → +100 | Ngả trước/sau |
| `YAW` | `int8_t` | -100 → +100 | Xoay trái/phải |
| `FLAGS` | `uint8_t` | bit field | bit0=ARM, bit1=LAND, bit2=EMERGENCY |
| `CRC8` | `uint8_t` | 0–255 | XOR của bytes 0–5 |

**Code firmware (C) để parse gói tin:**

```c
#define PACKET_HEADER   0xAA
#define PACKET_SIZE     7

typedef struct {
    int8_t  throttle;
    int8_t  roll;
    int8_t  pitch;
    int8_t  yaw;
    uint8_t flags;
} DroneCmd_t;

static uint8_t crc8_calc(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) crc ^= data[i];
    return crc;
}

bool parse_packet(const uint8_t *buf, size_t len, DroneCmd_t *cmd) {
    if (len < PACKET_SIZE) return false;
    if (buf[0] != PACKET_HEADER) return false;
    if (crc8_calc(buf, PACKET_SIZE - 1) != buf[PACKET_SIZE - 1]) return false;

    cmd->throttle = (int8_t)buf[1];
    cmd->roll     = (int8_t)buf[2];
    cmd->pitch    = (int8_t)buf[3];
    cmd->yaw      = (int8_t)buf[4];
    cmd->flags    = buf[5];
    return true;
}
```

### 5.2 Giao thức văn bản (Text Protocol) — Đơn giản hơn cho STM32 + MIT App

Dùng khi không muốn xử lý nhị phân ở phía App:

```
Format: "A,THR,ROLL,PITCH,YAW,FLAGS\n"
Ví dụ: "A,075,-030,+010,+000,01\n"
```

> ⚠️ **Cảnh báo:** Text protocol dễ bị lỗi nếu App gửi thiếu/thừa ký tự. Luôn kiểm tra header `'A'` và kết thúc `'\n'` trước khi parse.

### 5.3 Định nghĩa UUID cho ESP32-S3 BLE GATT

```c
// Service UUID (tự định nghĩa - 128-bit)
#define DRONE_SERVICE_UUID    "12345678-1234-1234-1234-123456789ABC"

// Characteristic UUID cho gửi lệnh (Write Without Response)
#define DRONE_CMD_CHAR_UUID   "12345678-1234-1234-1234-123456789ABD"

// Characteristic UUID cho telemetry từ drone về app (Notify)
#define DRONE_TELE_CHAR_UUID  "12345678-1234-1234-1234-123456789ABE"
```

---

## 6. Đánh giá chuyên sâu: Nền tảng nào phù hợp hơn?

**Kết luận thực tiễn: ESP32-S3 Supermini vượt trội hoàn toàn so với STM32F103 trong bài toán điều khiển drone qua Bluetooth Android App.**

Có 5 lý do cốt lõi:

### 6.1 Lợi thế phần cứng tích hợp

- **ESP32-S3:** SoC tích hợp sẵn Wi-Fi 802.11 b/g/n và Bluetooth 5.0 LE. Developer không cần đấu nối mạch ngoài, xử lý nhiễu tín hiệu UART, hay lo về mức logic 3.3V/5V.
- **STM32F103:** Bắt buộc dùng module ngoại vi (HC-05 = Bluetooth Classic SPP, HM-10 = BLE). HC-05 giao tiếp UART 9600–115200 baud, dễ mất byte nếu không có DMA.

### 6.2 Hệ sinh thái phần mềm trưởng thành

- **ESP-IDF** cung cấp NimBLE stack đầy đủ, FreeRTOS tích hợp, và hàng trăm component sẵn có (LVGL, MQTT, HTTP).
- STM32 HAL không có BLE stack — phải viết driver UART từ đầu hoặc dùng AT command set (dễ bị timeout, không reliable).

### 6.3 Lợi thế phát triển App Android

- Với ESP32-S3 BLE, Android app dùng `BluetoothGatt` API chuẩn — không cần pairing như Bluetooth Classic, kết nối nhanh hơn (~1–3 giây vs ~5–10 giây với HC-05).
- GATT Characteristic `Write Without Response` cho phép gửi gói 20ms mà không chờ ACK → latency thực tế 15–25ms.

### 6.4 Khả năng mở rộng (Scalability)

| Tính năng nâng cao | ESP32-S3 | STM32F103 + HC-05 |
|---|---|---|
| OTA Firmware Update | ✅ qua Wi-Fi (esp_ota) | ❌ Phải cắm cáp |
| Telemetry gửi về App | ✅ BLE Notify | ⚠️ Phức tạp (bán song công) |
| Web dashboard | ✅ ESP-IDF HTTP Server | ❌ |
| GPS tích hợp | ✅ UART còn dư | ⚠️ Hết UART |
| Log dữ liệu bay | ✅ SPIFFS/SD Card | ⚠️ Phức tạp |

### 6.5 Chi phí & Tiêu thụ điện

| | ESP32-S3 Supermini | STM32F103C8T6 + HC-05 |
|---|---|---|
| Giá linh kiện | ~$5–8 | ~$3 + $5 = ~$8 |
| Dòng điện khi BLE active | ~80–120 mA (cả chip) | ~200–300 mA (HC-05 riêng ~150mA) |
| Điện áp hoạt động | 3.3V | 3.3V (STM32) / 5V (HC-05) |

---

## 7. Rủi ro kỹ thuật & Cách phòng tránh

### 7.1 Rủi ro phía Firmware

| Rủi ro | Nền tảng | Nguyên nhân | Cách phòng tránh |
|---|---|---|---|
| **Stack overflow FreeRTOS task** | ESP32-S3 | Task BLE callback có stack quá nhỏ | Cấp tối thiểu 4096 bytes cho BLE task; dùng `uxTaskGetStackHighWaterMark()` để monitor |
| **Mất gói tin UART** | STM32 | Không dùng DMA, xử lý trong polling loop | Bắt buộc dùng UART DMA Circular mode + ring buffer 256 bytes |
| **Hard Fault khi parse chuỗi** | STM32 | `strtok()` ghi đè buffer, con trỏ NULL | Dùng Safe String: kiểm tra bounds trước, không dùng `strtok()` trực tiếp trên buffer nhận |
| **Drone không dừng khi mất kết nối** | Cả hai | Không có timeout/failsafe | Dùng timer WDT: nếu không nhận lệnh trong 500ms → throttle = 0, ARM = false |
| **Race condition giữa BLE task và PID task** | ESP32-S3 | Hai task cùng đọc/ghi biến chia sẻ | Dùng `FreeRTOS Queue` hoặc `Mutex` để bảo vệ shared data |

### 7.2 Rủi ro phía App Android

| Rủi ro | Nguyên nhân | Cách phòng tránh |
|---|---|---|
| **Gatt callback không trên Main Thread** | Android chạy GATT callback trên background thread | Dùng `Handler(Looper.getMainLooper()).post()` để cập nhật UI |
| **Kết nối BLE bị drop sau 5–10 phút** | Android Power Manager / Doze Mode | Dùng `WakeLock` hoặc `requestConnectionPriority(HIGH)` |
| **Joystick gửi quá nhanh → tràn MTU** | Gói tin > 20 bytes mà chưa negotiate MTU | Negotiate MTU lên 512 bytes bằng `requestMtu(512)` hoặc dùng gói 7 bytes (nhị phân) |
| **Latency cao khi màn hình tắt** | CPU throttling | Giữ màn hình bật khi điều khiển (`FLAG_KEEP_SCREEN_ON`) |

---

## 8. Khuyến nghị triển khai & Lộ trình phát triển

### 8.1 Khuyến nghị theo mục tiêu đồ án

| Mục tiêu đồ án | Nền tảng khuyến nghị | App khuyến nghị | Lý do |
|---|---|---|---|
| **Bay tự động + PID + Sensor Fusion** (đồ án nghiên cứu kỹ thuật) | ESP32-S3 + Custom BLE | Android Native (Kotlin) | Tiết kiệm thời gian dev App, tập trung vào thuật toán |
| **Hệ thống hoàn chỉnh từ đầu đến cuối** (đồ án toàn diện) | ESP32-S3 + Custom BLE | Android Native Custom | Kiểm soát hoàn toàn, ấn tượng hội đồng |
| **Rèn luyện kỹ năng nhúng bare-metal** | STM32F103 + HC-05 | MIT App Inventor | Hiểu sâu phần cứng, phù hợp kỹ sư embedded |
| **Thử nghiệm nhanh thuật toán PID** | ESP32-S3 + Dabble | Dabble App | Chạy được trong 1 ngày |

### 8.2 Lộ trình phát triển cho ESP32-S3 (Khuyến nghị chính)

```
Tuần 1–2: Nền tảng BLE
├── Cài ESP-IDF + VS Code
├── Chạy ví dụ NimBLE GATT Server
├── Test kết nối từ app nRF Connect (Android)
└── Verify gói tin binary 7 bytes nhận đúng

Tuần 3–4: App Android cơ bản
├── Tạo project Android Studio (Kotlin)
├── Implement BluetoothLeScanner → tìm thiết bị
├── Kết nối GATT → Write Characteristic
└── Test joystick ảo gửi dữ liệu thật

Tuần 5–6: Tích hợp Firmware hoàn chỉnh
├── FreeRTOS: BLE task → Queue → PID task
├── Parse gói tin + CRC8 validation
├── Failsafe timeout 500ms
└── PWM output cho ESC (LEDC timer ESP32)

Tuần 7–8: Tích hợp phần cứng và bay thử
├── Kết nối ESC + Motor + IMU (MPU-6050 I2C)
├── Tuning PID Roll/Pitch/Yaw
├── Test toàn hệ thống trên bench
└── Bay thử có dây buộc an toàn
```

### 8.3 Checklist kỹ thuật trước khi bay thử

- [ ] CRC8 validation hoạt động, từ chối gói tin lỗi
- [ ] Failsafe: mất kết nối > 500ms → throttle = 0
- [ ] Arm/Disarm cần gesture đặc biệt (tránh arm nhầm)
- [ ] Giới hạn góc nghiêng tối đa ±30° trong PID
- [ ] LED status hiển thị trạng thái kết nối BLE
- [ ] App hiển thị "Connected/Disconnected" rõ ràng
- [ ] Test mất kết nối cố ý (tắt Bluetooth điện thoại)
- [ ] Đo latency thực tế (timestamp gửi vs timestamp nhận)

---

## 9. Tài liệu tham khảo & Nguồn mở

### 9.1 ESP32-S3 & BLE

| Tài liệu | Link | Mức độ |
|---|---|---|
| ESP-IDF NimBLE GATT Server Example | https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/nimble/bleprph | Sinh viên |
| NimBLE Documentation (Apache) | https://mynewt.apache.org/latest/network/docs/nimble_overview.html | Chuyên sâu |
| ESP32-S3 Datasheet | https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf | Tra cứu |
| ESP-IDF FreeRTOS Guide | https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos.html | Bắt buộc |

### 9.2 Android BLE Development

| Tài liệu | Link | Mức độ |
|---|---|---|
| Android BLE Guide (Official) | https://developer.android.com/develop/connectivity/bluetooth/ble/ble-overview | Bắt buộc |
| BLE GATT Android sample (Google) | https://github.com/android/connectivity-samples/tree/main/BluetoothLeGatt | Sinh viên |
| RxAndroidBle (reactive BLE library) | https://github.com/dariuszseweryn/RxAndroidBle | Nâng cao |

### 9.3 STM32 + HC-05

| Tài liệu | Link | Mức độ |
|---|---|---|
| STM32 UART DMA Circular Buffer | https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx | Bắt buộc |
| HC-05 AT Command Reference | https://components101.com/wireless/hc-05-bluetooth-module | Cơ bản |
| Safe Ring Buffer C implementation | https://github.com/dhess/c-ringbuf | Tham khảo |

### 9.4 Giao thức & Thiết kế hệ thống

| Tài liệu | Link | Mức độ |
|---|---|---|
| MAVLink Protocol (chuẩn drone giao tiếp) | https://mavlink.io/en/ | Tham khảo |
| MultiWii Serial Protocol (MSP) | https://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol | Tham khảo |
| CRC8 Algorithm explanation | https://crccalc.com/ | Công cụ |

---

## 10. Phụ lục kỹ thuật

### 10.1 Sơ đồ kết nối phần cứng ESP32-S3

```
ESP32-S3 Supermini
    │
    ├── GPIO4 → ESC Motor 1 (LEDC CH0, PWM 50Hz)
    ├── GPIO5 → ESC Motor 2 (LEDC CH1, PWM 50Hz)
    ├── GPIO6 → ESC Motor 3 (LEDC CH2, PWM 50Hz)
    ├── GPIO7 → ESC Motor 4 (LEDC CH3, PWM 50Hz)
    │
    ├── GPIO8 (SDA) → MPU-6050 SDA (I2C)
    ├── GPIO9 (SCL) → MPU-6050 SCL (I2C)
    │
    ├── GPIO2 → LED Status (BLE connected/disconnected)
    │
    └── USB (built-in) → Power + Debug UART
```

### 10.2 Sơ đồ kết nối phần cứng STM32F103 + HC-05

```
STM32F103C8T6
    │
    ├── PA9  (UART1 TX) → HC-05 RX (qua điện trở 1kΩ, hạ 3.3V→3.3V OK)
    ├── PA10 (UART1 RX) → HC-05 TX
    │
    ├── PA8  → ESC Motor 1 (TIM1 CH1, PWM 50Hz)
    ├── PA9  → ESC Motor 2 (TIM1 CH2)  [Chú ý: xung đột UART — dùng UART2]
    ├── PB6  → ESC Motor 3 (TIM4 CH1)
    ├── PB7  → ESC Motor 4 (TIM4 CH2)
    │
    ├── PB6  (I2C1 SCL) → MPU-6050 SCL
    ├── PB7  (I2C1 SDA) → MPU-6050 SDA
    │
    └── PC13 → LED Status (onboard)

HC-05 Module
    ├── VCC → 5V
    ├── GND → GND
    ├── RX  → STM32 PA9 (qua voltage divider 3.3V nếu cần)
    └── TX  → STM32 PA10
```

### 10.3 Cấu hình BLE Connection Parameter tối ưu cho drone

```c
// Trong ESP-IDF NimBLE, cấu hình connection parameters
struct ble_gap_upd_params params = {
    .itvl_min = 6,          // 6 * 1.25ms = 7.5ms (minimum interval)
    .itvl_max = 16,         // 16 * 1.25ms = 20ms (maximum interval)
    .latency = 0,           // Slave latency = 0 (không skip connection event)
    .supervision_timeout = 100,  // 100 * 10ms = 1 second timeout
};
ble_gap_update_params(conn_handle, &params);
```

### 10.4 Công thức chuyển đổi Joystick → PWM ESC

```
Joystick raw: [-100, +100] (từ App Android)
PWM ESC: [1000µs, 2000µs] (tiêu chuẩn PWM RC)

Throttle:  pwm = 1000 + (throttle / 100.0f) * 1000
Roll:      ảnh hưởng Motor1 ↑ và Motor2 ↓ (hoặc ngược lại)
Pitch:     ảnh hưởng Motor1,2 ↑ và Motor3,4 ↓
Yaw:       ảnh hưởng cặp Motor chéo theo chiều quay cánh

Motor1_PWM = base_throttle + PID_roll - PID_pitch + PID_yaw
Motor2_PWM = base_throttle - PID_roll - PID_pitch - PID_yaw
Motor3_PWM = base_throttle - PID_roll + PID_pitch + PID_yaw
Motor4_PWM = base_throttle + PID_roll + PID_pitch - PID_yaw
```

### 10.5 Glossary (Bảng giải thích thuật ngữ)

| Thuật ngữ | Giải thích |
|---|---|
| **BLE** | Bluetooth Low Energy — giao thức tiết kiệm điện, phù hợp IoT và drone |
| **GATT** | Generic Attribute Profile — cấu trúc dữ liệu của BLE (Service → Characteristic) |
| **UUID** | Universally Unique Identifier — định danh 128-bit cho Service/Characteristic |
| **MTU** | Maximum Transmission Unit — kích thước tối đa một gói BLE (mặc định 23 bytes) |
| **DMA** | Direct Memory Access — truyền dữ liệu trực tiếp bộ nhớ, không cần CPU |
| **ISR** | Interrupt Service Routine — hàm xử lý ngắt phần cứng |
| **ESC** | Electronic Speed Controller — mạch điều khiển tốc độ motor drone |
| **PID** | Proportional-Integral-Derivative — thuật toán điều khiển phản hồi cổ điển |
| **SPP** | Serial Port Profile — giao thức Bluetooth Classic giả lập cổng COM |
| **CRC** | Cyclic Redundancy Check — mã kiểm tra lỗi gói tin |
| **FreeRTOS** | Real-Time Operating System miễn phí, dùng cho vi điều khiển |
| **NimBLE** | Lightweight BLE stack của Apache, được Espressif tích hợp vào ESP-IDF |