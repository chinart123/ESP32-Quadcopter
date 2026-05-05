# MODULE GIAO TIẾP PWM — ESP32-S3
### Tài liệu tích hợp & nghiệm thu | Dự án Flight Controller

> **Mục đích tài liệu:** Hướng dẫn nhóm từng bước — từ đấu dây, nạp code, đến kiểm tra tín hiệu thực tế — cho module truyền tín hiệu điều khiển (PWM) giữa 2 node ESP32-S3. Mỗi bước đều có giải thích *tại sao* để cả nhóm hiểu logic, không chỉ copy-paste.

---

## Mục lục

1. [Bối cảnh & Kiến trúc tổng quan](#1-bối-cảnh--kiến-trúc-tổng-quan)
2. [Sơ đồ đấu dây](#2-sơ-đồ-đấu-dây)
3. [Cấu hình trước khi nạp code](#3-cấu-hình-trước-khi-nạp-code)
4. [Mã nguồn — Chuẩn Arduino API](#4-mã-nguồn--chuẩn-arduino-api-v3x)
5. [Mã nguồn — Chuẩn ESP-IDF Native](#5-mã-nguồn--chuẩn-esp-idf-native)
6. [So sánh 2 chuẩn code](#6-so-sánh-2-chuẩn-code)
7. [Quy trình nghiệm thu](#7-quy-trình-nghiệm-thu)
8. [Câu hỏi thường gặp](#8-câu-hỏi-thường-gặp)

---

## 1. Bối cảnh & Kiến trúc tổng quan

### Tại sao dùng PWM để truyền tín hiệu?

Trong hệ thống điều khiển máy bay (flight controller), tín hiệu điều khiển truyền thống từ remote đến FC dùng **PWM (Pulse Width Modulation)** — một chuẩn đã tồn tại hàng chục năm trong RC hobby. Thay vì truyền con số `1500`, hệ thống truyền một **xung điện có độ rộng 1500 microseconds** — FC đọc độ rộng xung đó và dịch ra lệnh điều khiển.

Ở dự án này, 2 ESP32-S3 đóng vai:

| Node | Vai trò | Hành vi |
|------|---------|---------|
| **TX** (Tay cầm) | Phát xung | Liên tục xuất xung PWM 50Hz ra GPIO7 |
| **RX** (Flight Controller) | Nhận xung | Đo độ rộng xung đến từ GPIO6, đưa vào vòng PID |

### Tại sao dùng Hardware Interrupt thay vì `pulseIn()`?

`pulseIn()` là hàm đơn giản nhưng có một vấn đề nghiêm trọng: nó **chặn toàn bộ CPU** trong thời gian chờ xung. Với xung 50Hz (chu kỳ 20ms), CPU bị "đóng băng" tới 20ms mỗi lần gọi — vòng lặp PID không thể chạy mượt.

**Hardware Interrupt** giải quyết điều này hoàn toàn:
- Khi có cạnh lên/xuống trên chân GPIO, phần cứng tự động gọi hàm `pwmInterruptHandler()`
- CPU không phải chờ — nó đang làm việc khác và chỉ bị "gọi" khi xung thực sự đến
- Vòng lặp PID chạy liên tục, chỉ tốn ~1µs để đọc biến `pwmValue` đã được ISR cập nhật

```
Không dùng Interrupt:       Dùng Hardware Interrupt:
┌──────────────────┐        ┌──────────────────┐
│ loop() bị BLOCK  │        │ loop() chạy tự do│
│  chờ xung 20ms   │        │ PID, filter, ... │
│  ...chờ...       │        │                  │
│  đọc được xung   │        │   [ISR kích hoạt]│
│  tiếp tục loop() │        │   đo xung ~2µs   │
└──────────────────┘        └──────────────────┘
```

### Kiến trúc Dual Core (tùy chọn)

ESP32-S3 có 2 nhân xử lý. Khi bật `USE_DUAL_CORE`:

```
Core 0 (FreeRTOS Task)          Core 1 (loop())
┌─────────────────────────┐     ┌──────────────────────────┐
│ rxTask / txTask         │     │ Vòng lặp PID             │
│ - In log Serial         │     │ Bộ lọc Complementary     │
│ - Xử lý ngoại vi        │     │ Bộ lọc Kalman            │
│ - Giao tiếp module      │     │ (100% hiệu năng)         │
└─────────────────────────┘     └──────────────────────────┘
```

---

## 2. Sơ đồ đấu dây

### Bảng kết nối

| Tín hiệu | ESP32-S3 TX (Tay cầm) | ESP32-S3 RX (FC) | Ghi chú |
|----------|----------------------|-------------------|---------|
| PWM Signal | **GPIO7** | **GPIO6** | Dây tín hiệu, nối thẳng |
| GND chung | **GND** | **GND** | ⚠️ **Bắt buộc** — thiếu dây này tín hiệu sẽ nhiễu hoặc không đọc được |
| Nguồn | 5V (USB) | 5V (USB) | Có thể cấp riêng biệt, không cần nối chung |

### Sơ đồ nối

```
[ESP32-S3 TX]              [ESP32-S3 RX]
    GPIO7  ────────────────►  GPIO6
    GND    ────────────────►  GND
```

> **⚠️ Cảnh báo an toàn:**
> - Tuyệt đối **không** đưa 5V vào chân GPIO — chân GPIO chịu tối đa 3.3V
> - ESP32-S3 Supermini dùng logic 3.3V, nối trực tiếp 2 board là an toàn
> - Không cần điện trở kéo — RX đã cấu hình `INPUT_PULLDOWN` trong code

> **💡 Tại sao phải nối GND chung?**
> Điện áp là khái niệm *tương đối*. Khi TX xuất "3.3V", con số đó chỉ có nghĩa so với GND của chính nó. Nếu RX có GND khác mức, nó sẽ đọc tín hiệu sai hoặc không đọc được gì.

---

## 3. Cấu hình trước khi nạp code

Ở đầu mỗi file code có 2 nhóm `#define` cần chỉnh trước khi biên dịch:

### A. Chọn vai trò (Role)

```cpp
#define ROLE_TRANSMITTER  // ← Giữ dòng này để nạp cho Tay cầm (TX)
//#define ROLE_RECEIVER   // ← Bỏ comment dòng này để nạp cho FC (RX)
```

> Chỉ được bật **một trong hai**. Compiler dùng `#ifdef` để chọn đúng khối code cần biên dịch — phần còn lại bị loại hoàn toàn ra khỏi bộ nhớ flash.

### B. Chọn kiến trúc (Core)

```cpp
#define USE_DUAL_CORE     // ← Bật: dùng 2 Core (FreeRTOS)
//#define USE_DUAL_CORE   // ← Tắt (//) : chạy 1 Core
```

| Chế độ | Khi nào dùng | Hành vi |
|--------|-------------|---------|
| **Dual Core** | Giai đoạn tích hợp đầy đủ | Core 0 lo Serial/ngoại vi, Core 1 chạy PID toàn tốc |
| **Single Core** | Debug đơn giản, kiểm tra nhanh | Mọi thứ chạy chung Core 1, nhưng **không bị block** nhờ Interrupt |

---

## 4. Mã nguồn — Chuẩn Arduino API (v3.x)

**Khi nào dùng:** Giai đoạn phát triển, debug, prototype. Code ngắn gọn, dễ đọc.

**Điểm đặc biệt:** Không cần `#include` thủ công — Arduino IDE tự chèn `esp32-hal-ledc.h`, `FreeRTOS.h`, `Arduino.h` ở tầng dưới.

```cpp
// ====================================================================
// MODULE PWM GIAO TIẾP — CHUẨN ARDUINO API v3.x (ESP32-S3)
// ====================================================================

// --- BƯỚC 1: CHỌN VAI TRÒ ---
#define ROLE_TRANSMITTER   // Nạp cho Tay cầm (TX)
//#define ROLE_RECEIVER    // Nạp cho Flight Controller (RX)

// --- BƯỚC 2: CHỌN KIẾN TRÚC ---
#define USE_DUAL_CORE      // Bật 2 Core | Tắt (//) để chạy 1 Core

// --- KHAI BÁO CHÂN ---
const int pwmTxPin = 7;    // GPIO7: chân phát PWM
const int pwmRxPin = 6;    // GPIO6: chân nhận PWM

TaskHandle_t Core0TaskHandle;

// ====================================================================
// KHỐI NGẮT PHẦN CỨNG (CHỈ BIÊN DỊCH KHI LÀ RX)
// ====================================================================
#ifdef ROLE_RECEIVER
  volatile unsigned long pulseStartTime = 0;
  volatile unsigned long pwmValue = 0;

  // IRAM_ATTR: Bắt buộc đặt hàm ISR vào IRAM (bộ nhớ trong)
  // để phần cứng gọi được ngay lập tức, không phụ thuộc Flash cache
  void IRAM_ATTR pwmInterruptHandler() {
    if (digitalRead(pwmRxPin) == HIGH) {
      pulseStartTime = micros();       // Ghi nhận thời điểm cạnh lên
    } else {
      pwmValue = micros() - pulseStartTime; // Tính độ rộng xung
    }
  }
#endif

// ====================================================================
// SETUP: CHẠY MỘT LẦN KHI KHỞI ĐỘNG
// ====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Khởi tạo phần cứng theo vai trò ---
  #ifdef ROLE_TRANSMITTER
    Serial.println("[SYS_INIT] Node TX | Chuẩn: Arduino API | GPIO7");
    // ledcAttach(pin, frequency, resolution)
    // - 50Hz: chuẩn PWM servo/RC truyền thống
    // - 14-bit: bắt buộc với ESP32-S3 (S3 không hỗ trợ resolution cao hơn ổn định)
    if (ledcAttach(pwmTxPin, 50, 14)) {
      // Duty 1229 = 1500µs ở thang 14-bit
      // Tính: (1500µs / 20000µs) × 2^14 = 7.5% × 16384 ≈ 1229
      ledcWrite(pwmTxPin, 1229);
      Serial.println("[HW_OK] Timer ON | 50Hz | Duty: 1500µs (neutral)");
    } else {
      Serial.println("[HW_ERR] Lỗi khởi tạo Timer TX!");
    }

  #elif defined(ROLE_RECEIVER)
    Serial.println("[SYS_INIT] Node RX | Chuẩn: Interrupt | GPIO6");
    pinMode(pwmRxPin, INPUT_PULLDOWN);
    // digitalPinToInterrupt(): chuyển số chân sang số interrupt tương ứng
    // CHANGE: kích hoạt ISR cả khi cạnh LÊN và cạnh XUỐNG
    attachInterrupt(digitalPinToInterrupt(pwmRxPin), pwmInterruptHandler, CHANGE);
  #endif

  // --- Phân bổ tài nguyên FreeRTOS ---
  #ifdef USE_DUAL_CORE
    Serial.println("[SYS_CORE] Chế độ: DUAL CORE");
    #ifdef ROLE_TRANSMITTER
      // Tham số: (hàm, tên, stack, tham số, priority, handle, core)
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0);
    #elif defined(ROLE_RECEIVER)
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0);
    #endif
  #else
    Serial.println("[SYS_CORE] Chế độ: SINGLE CORE");
  #endif

  Serial.println("--------------------------------------------------");
}

// ====================================================================
// LOOP: CHẠY LIÊN TỤC TRÊN CORE 1
// ====================================================================
void loop() {
  #ifdef USE_DUAL_CORE
    #ifdef ROLE_TRANSMITTER
      // Core 1 rảnh — chỗ để sau này xử lý joystick, màn hình, nút bấm
      Serial.println("[CORE_1] TX idle. Sẵn sàng xử lý input...");
      delay(2000);
    #elif defined(ROLE_RECEIVER)
      // Core 1 chạy PID — KHÔNG bị delay bởi việc đọc xung
      Serial.println("[CORE_1] PID loop running @ max speed...");
      delay(500);
    #endif

  #else // SINGLE CORE
    #ifdef ROLE_TRANSMITTER
      Serial.println("[SINGLE] TX đang phát PWM nền bằng hardware timer...");
      delay(2000);
    #elif defined(ROLE_RECEIVER)
      // Không block — ISR đã cập nhật pwmValue ở nền
      if (pwmValue > 0) {
        Serial.print("[SINGLE] [RX] Độ rộng xung: ");
        Serial.print(pwmValue);
        Serial.println(" µs");
      }
      delay(20);
    #endif
  #endif
}

// ====================================================================
// TASK CORE 0 (CHỈ BIÊN DỊCH KHI BẬT DUAL CORE)
// ====================================================================
#ifdef USE_DUAL_CORE

  #ifdef ROLE_TRANSMITTER
  void txTask(void *pvParam) {
    for (;;) {
      // Placeholder: sau này đọc ADC joystick, tính duty mới, gọi ledcWrite()
      vTaskDelay(2000 / portTICK_PERIOD_MS); // Dùng vTaskDelay, KHÔNG dùng delay()
    }
  }
  #endif

  #ifdef ROLE_RECEIVER
  void rxTask(void *pvParam) {
    for (;;) {
      if (pwmValue > 0) {
        Serial.print("[CORE_0] [RX] Độ rộng xung: ");
        Serial.print(pwmValue);
        Serial.println(" µs");
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
  #endif

#endif
```

---

## 5. Mã nguồn — Chuẩn ESP-IDF Native

**Khi nào dùng:** Khi cần can thiệp sâu vào phần cứng, hoặc chuẩn bị nâng cấp lên **MCPWM** để điều khiển ESC trực tiếp. Đây là chuẩn công nghiệp.

**Điểm khác biệt chính so với Arduino API:** Phải `#include "driver/ledc.h"` thủ công, và cấu hình Timer/Channel qua `struct` thay vì gọi hàm wrapper.

```cpp
// ====================================================================
// MODULE PWM GIAO TIẾP — CHUẨN ESP-IDF NATIVE (ESP32-S3)
// ====================================================================

#define ROLE_TRANSMITTER
//#define ROLE_RECEIVER

#define USE_DUAL_CORE

const int pwmTxPin = 7;
const int pwmRxPin = 6;

TaskHandle_t Core0TaskHandle;

// KHÁC BIỆT #1: Phải include thủ công thư viện lõi
#ifdef ROLE_TRANSMITTER
  #include "driver/ledc.h"
#endif

#ifdef ROLE_RECEIVER
  volatile unsigned long pulseStartTime = 0;
  volatile unsigned long pwmValue = 0;

  void IRAM_ATTR pwmInterruptHandler() {
    if (digitalRead(pwmRxPin) == HIGH) {
      pulseStartTime = micros();
    } else {
      pwmValue = micros() - pulseStartTime;
    }
  }
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  #ifdef ROLE_TRANSMITTER
    Serial.println("[SYS_INIT] Node TX | Chuẩn: ESP-IDF Native | GPIO7");

    // KHÁC BIỆT #2: Cấu hình Timer qua struct — kiểm soát từng thanh ghi
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE, // ESP32-S3 chỉ có LOW_SPEED_MODE
        .duty_resolution = LEDC_TIMER_14_BIT,   // 14-bit = 16384 bậc duty
        .timer_num       = LEDC_TIMER_0,         // Dùng Timer vật lý số 0
        .freq_hz         = 50,                   // 50Hz chuẩn RC/servo
        .clk_cfg         = LEDC_AUTO_CLK         // Để chip tự chọn nguồn clock tối ưu
    };
    ledc_timer_config(&ledc_timer);

    // KHÁC BIỆT #3: Cấu hình Channel — gắn Timer vào chân GPIO cụ thể
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = pwmTxPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,  // Không dùng interrupt từ LEDC
        .timer_sel  = LEDC_TIMER_0,        // Gắn vào Timer 0 đã cấu hình trên
        .duty       = 1229,                // 1500µs neutral
        .hpoint     = 0                    // Điểm bắt đầu xung (thường để 0)
    };
    ledc_channel_config(&ledc_channel);

    Serial.println("[HW_OK] Timer ON | 50Hz | Duty: 1500µs (neutral)");

  #elif defined(ROLE_RECEIVER)
    Serial.println("[SYS_INIT] Node RX | Chuẩn: Interrupt EXTI | GPIO6");
    pinMode(pwmRxPin, INPUT_PULLDOWN);
    // Với ESP-IDF, có thể truyền số chân trực tiếp (không cần digitalPinToInterrupt)
    attachInterrupt(pwmRxPin, pwmInterruptHandler, CHANGE);
  #endif

  #ifdef USE_DUAL_CORE
    Serial.println("[SYS_CORE] Chế độ: DUAL CORE");
    #ifdef ROLE_TRANSMITTER
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0);
    #elif defined(ROLE_RECEIVER)
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0);
    #endif
  #else
    Serial.println("[SYS_CORE] Chế độ: SINGLE CORE");
  #endif

  Serial.println("--------------------------------------------------");
}

void loop() {
  #ifdef USE_DUAL_CORE
    #ifdef ROLE_TRANSMITTER
      Serial.println("[CORE_1] TX idle...");
      delay(2000);
    #elif defined(ROLE_RECEIVER)
      Serial.println("[CORE_1] PID loop running @ max speed...");
      delay(500);
    #endif
  #else
    #ifdef ROLE_TRANSMITTER
      delay(2000);
    #elif defined(ROLE_RECEIVER)
      if (pwmValue > 0) {
        Serial.print("[SINGLE] [RX] Độ rộng xung: ");
        Serial.println(pwmValue);
      }
      delay(20);
    #endif
  #endif
}

#ifdef USE_DUAL_CORE
  #ifdef ROLE_TRANSMITTER
  void txTask(void *pvParam) {
    for (;;) { vTaskDelay(2000 / portTICK_PERIOD_MS); }
  }
  #endif

  #ifdef ROLE_RECEIVER
  void rxTask(void *pvParam) {
    for (;;) {
      if (pwmValue > 0) {
        Serial.print("[CORE_0] [RX] Độ rộng xung: ");
        Serial.println(pwmValue);
      }
      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
  #endif
#endif
```

---

## 6. So sánh 2 chuẩn code

| Tiêu chí | Arduino API (v3.x) | ESP-IDF Native |
|----------|-------------------|----------------|
| **Độ phức tạp** | Thấp | Cao |
| **Cần `#include` thủ công** | Không | Có (`driver/ledc.h`) |
| **Cách cấu hình Timer** | `ledcAttach(pin, freq, bits)` | Struct `ledc_timer_config_t` |
| **Kiểm soát phần cứng** | Gián tiếp qua wrapper | Trực tiếp qua thanh ghi |
| **Phù hợp** | Prototype, debug | Production, nâng cấp lên MCPWM |
| **Phần RX (Interrupt)** | Giống nhau | Giống nhau |
| **Bước tiếp theo** | Khó nâng cấp ESC | ✅ Tiền đề trực tiếp cho MCPWM |

> **Khuyến nghị:** Dùng Arduino API để validate phần cứng và logic trước. Chuyển sang ESP-IDF Native khi chuẩn bị tích hợp ESC thực tế.

---

## 7. Quy trình nghiệm thu

### Bước 1 — Đo kiểm vật lý Node TX bằng Multimeter

**Mục đích:** Xác nhận Timer phần cứng đang xuất xung thật sự trước khi nối RX.

**Thao tác:**
1. Chuyển đồng hồ VOM sang chế độ **DC Voltage (VDC)**
2. Que đen → `GND` | Que đỏ → `GPIO7` của TX
3. Đọc giá trị hiển thị

**Kết quả mong đợi:**

| Kết quả đo | Ý nghĩa |
|-----------|---------|
| **~0.24V – 0.25V** ✅ | Timer hoạt động đúng. Duty 7.5% × 3.3V ≈ 0.248V |
| `0.00V` ❌ | Timer không khởi động — kiểm tra lại `ledcAttach` / `ledc_channel_config` |
| `3.30V` ❌ | Chân bị kéo HIGH cứng — có thể đoản mạch hoặc sai chân |

> **Giải thích phép tính:** Xung 1500µs trong chu kỳ 20000µs (50Hz) cho Duty Cycle = 1500/20000 = **7.5%**. VOM đo điện áp DC trung bình, nên hiển thị: 3.3V × 7.5% ≈ **0.248V**.

---

### Bước 2 — Kiểm tra Serial Monitor Node TX

Baudrate: `115200`

**Log chuẩn (Dual Core):**
```
[SYS_INIT] Node TX | Chuẩn: ... | GPIO7
[HW_OK] Timer ON | 50Hz | Duty: 1500µs (neutral)
[SYS_CORE] Chế độ: DUAL CORE
--------------------------------------------------
[CORE_1] TX idle. Sẵn sàng xử lý input...
[CORE_1] TX idle. Sẵn sàng xử lý input...
```

**Checklist TX:**
- [ ] Dòng `[HW_OK]` xuất hiện (không phải `[HW_ERR]`)
- [ ] Log lặp đều đặn, không bị treo
- [ ] Không có thông báo lỗi lạ

---

### Bước 3 — Kiểm tra Serial Monitor Node RX

**Điều kiện:** TX đã bật, dây tín hiệu và GND đã nối.

**Log chuẩn (Dual Core, nối dây đúng):**
```
[SYS_INIT] Node RX | Chuẩn: Interrupt | GPIO6
[SYS_CORE] Chế độ: DUAL CORE
--------------------------------------------------
[CORE_1] PID loop running @ max speed...
[CORE_0] [RX] Độ rộng xung: 1500 µs
[CORE_0] [RX] Độ rộng xung: 1501 µs
[CORE_1] PID loop running @ max speed...
[CORE_0] [RX] Độ rộng xung: 1500 µs
```

**Checklist RX:**
- [ ] Giá trị xung hiển thị ~1500µs (±5µs là bình thường)
- [ ] Giá trị ổn định, không nhảy loạn (>±50µs → kiểm tra GND chung)
- [ ] Log Core 0 và Core 1 xen kẽ nhau (hiện tượng bình thường của RTOS)

> **Lưu ý:** Một vài ký tự bị đè lên nhau trên log là đặc tính bình thường của hệ thống đa luồng (2 Core cùng ghi Serial). Không ảnh hưởng đến dữ liệu thực.

---

### Bước 4 — Checklist tổng nghiệm thu

| Hạng mục | Trạng thái | Ghi chú |
|----------|-----------|---------|
| VOM đo GPIO7 = ~0.25V | ☐ | |
| TX log hiển thị `[HW_OK]` | ☐ | |
| RX log hiển thị giá trị ~1500µs | ☐ | |
| Giá trị xung ổn định (±5µs) | ☐ | |
| Không có HW_ERR | ☐ | |
| Thử tắt TX → RX ngừng nhận (xác nhận kết nối) | ☐ | |

---

## 8. Câu hỏi thường gặp

**Q: Tại sao Code 1 không cần `#include` mà Code 2 thì cần?**

Arduino IDE khi chọn board ESP32 sẽ tự động chèn ngầm bộ "ESP32 Arduino Core", bao gồm `esp32-hal-ledc.h`, `FreeRTOS.h`, `Arduino.h`. Code trông "sạch" hơn nhưng thực chất thư viện vẫn được nạp ở tầng dưới. ESP-IDF Native dùng thư viện cấp thấp hơn không nằm trong gói tự động đó, nên phải `#include` thủ công.

---

**Q: Tại sao không dùng thư viện `ESP32Servo`?**

`ESP32Servo` là thư viện bên ngoài do cộng đồng viết — thực chất nó chỉ *bọc* thêm lớp ngoài cho `ledc`. Dùng `ledc` trực tiếp thì nhẹ hơn, ít dependency hơn, và quan trọng là **không cần cài thêm gì từ Library Manager**. `ESP32Servo` phù hợp nếu bạn muốn API quen thuộc như Arduino Uno (`servo.write(90)`), nhưng không phù hợp cho dự án flight controller cần kiểm soát chặt chẽ.

---

**Q: Nếu tắt `USE_DUAL_CORE`, hệ thống RX có bị lag không?**

Không. Dù chạy chung 1 Core, tín hiệu xung vẫn được đo bởi **hardware interrupt** hoạt động hoàn toàn độc lập với `loop()`. CPU chỉ mất ~2µs để ghi thời gian vào biến `pwmValue` mỗi khi ISR kích hoạt. Vòng lặp `loop()` không bao giờ phải chờ xung — chỉ đọc biến kết quả đã có sẵn.

---

**Q: Làm thế nào để điều khiển được (thay đổi duty)?**

Hiện tại code TX phát cố định 1500µs (neutral). Để điều khiển thực:
```cpp
// Đọc ADC từ joystick (0–4095 với 12-bit ADC)
int joystickVal = analogRead(JOYSTICK_PIN);

// Map sang thang duty 14-bit tương ứng 1000µs–2000µs
// 1000µs = duty 819 | 2000µs = duty 1638
int duty = map(joystickVal, 0, 4095, 819, 1638);
ledcWrite(pwmTxPin, duty);  // Arduino API
// hoặc: ledc_set_duty + ledc_update_duty  // ESP-IDF Native
```

---

**Q: Có thể truyền không dây được không?**

Code này là giải pháp có dây (direct wire). Để truyền không dây cần thêm module RF (ví dụ nRF24L01, LoRa) hoặc dùng WiFi/ESP-NOW. Kiến trúc Interrupt và Dual Core vẫn giữ nguyên, chỉ thay phần phát/nhận vật lý.

---

*Tài liệu version 2.0 — Module PWM ESP32-S3 | Cập nhật sau nghiệm thu thực tế*