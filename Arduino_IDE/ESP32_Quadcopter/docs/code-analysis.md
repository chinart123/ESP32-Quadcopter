# ARDUINO-CODE-UPDATE

## 1. CODE GIAO TIẾP (CẬP NHẬT MỚI NHẤT)

Dưới đây là 2 dạng code đã được tối ưu hóa bằng Ngắt phần cứng (Hardware Interrupt) để không làm nghẽn CPU, thay thế hoàn toàn cho hàm `pulseIn()` cũ. Bạn có thể chọn 1 trong 2 để sử dụng.

### DẠNG 1: CHUẨN ARDUINO IDE (API v3.x)
Code ngắn gọn, dễ hiểu, dùng các hàm bọc sẵn của Arduino. Bắt buộc chỉnh Duty Resolution về 14-bit cho dòng chip S3.

```cpp
// ========================================== 
// 1. CẤU HÌNH VAI TRÒ (CHỌN 1 TRONG 2) 
// ========================================== 
#define ROLE_TRANSMITTER // Bật dòng này để làm mạch Phát (Tay cầm) 
//#define ROLE_RECEIVER // Bật dòng này để làm mạch Nhận (Drone FC) 
 
// ========================================== 
// 2. CẤU HÌNH KIẾN TRÚC (BẬT ĐỂ DÙNG 2 CORE) 
// ========================================== 
#define USE_DUAL_CORE // Bật dòng này để dùng FreeRTOS tách luồng. Tắt đi (//) để chạy 1 Core. 
 
// --- Khai báo chân (Pins) ĐÃ ĐƯỢC CHUYỂN SANG VÙNG AN TOÀN --- 
const int pwmTxPin = 7; // Chân phát PWM (chỉ dùng khi là TX) 
const int pwmRxPin = 6; // Chân nhận PWM (chỉ dùng khi là RX) 
 
TaskHandle_t Core0TaskHandle; // Biến quản lý Task nếu dùng Dual Core 

// --- KHỐI NGẮT PHẦN CỨNG (DÙNG CHO RX) ---
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
  delay(1000); // Chờ Serial ổn định 
 
  // --------------------------------------------------------- 
  // KHỞI TẠO PHẦN CỨNG THEO VAI TRÒ 
  // --------------------------------------------------------- 
  #ifdef ROLE_TRANSMITTER 
    Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) - ARDUINO API <<<"); 
    // ESP32-S3 chỉ hỗ trợ tối đa 14-bit
    if (ledcAttach(pwmTxPin, 50, 14)) {
      ledcWrite(pwmTxPin, 1229); // Phát độ rộng 1500us ở thang 14-bit
    }
  #elif defined(ROLE_RECEIVER) 
    Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) - NGẮT ARDUINO <<<"); 
    pinMode(pwmRxPin, INPUT_PULLDOWN); 
    attachInterrupt(digitalPinToInterrupt(pwmRxPin), pwmInterruptHandler, CHANGE);
  #endif 
 
  // --------------------------------------------------------- 
  // KHỞI TẠO KIẾN TRÚC THEO SỐ CORE 
  // --------------------------------------------------------- 
  #ifdef USE_DUAL_CORE 
    Serial.println("Kiến trúc: DUAL CORE (Sử dụng FreeRTOS trên Core 0)"); 
    #ifdef ROLE_TRANSMITTER 
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0); 
    #elif defined(ROLE_RECEIVER) 
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0); 
    #endif 
  #else 
    Serial.println("Kiến trúc: SINGLE CORE (Chạy toàn bộ trên Core 1)"); 
  #endif 
  Serial.println("============================="); 
} 

void loop() { 
  // ============================================================== 
  // VÒNG LẬP CHÍNH CỦA HỆ THỐNG (LUÔN CHẠY TRÊN CORE 1) 
  // ============================================================== 
  #ifdef USE_DUAL_CORE 
    // --- NẾU DÙNG DUAL CORE --- 
    #ifdef ROLE_TRANSMITTER 
      Serial.println("[Core 1] Đang rảnh rỗi chờ xử lý nút bấm/màn hình..."); 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)..."); 
      delay(500); // Giả lập thời gian tính toán thuật toán 
    #endif 

  #else 
    // --- NẾU CHỈ DÙNG 1 CORE (SINGLE CORE) --- 
    #ifdef ROLE_TRANSMITTER 
      Serial.println("[Single Core] TX vẫn đang phát PWM tự động bằng phần cứng..."); 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      // Không còn bị chặn bởi pulseIn. Chỉ cần lấy biến từ Ngắt ra in.
      if (pwmValue > 0) { 
        Serial.print("[Single Core] Nhận được PWM: "); 
        Serial.print(pwmValue); 
        Serial.println(" us"); 
      } 
      delay(20); 
    #endif 
  #endif 
}

// ============================================================= 
// CÁC HÀM XỬ LÝ TRÊN CORE 0 (CHỈ ĐƯỢC DỊCH KHI BẬT DUAL CORE) 
// ============================================================= 
#ifdef USE_DUAL_CORE 

  #ifdef ROLE_TRANSMITTER 
  void txTask(void *pvParam) { 
    for(;;) { 
      // Serial.println("[Core 0] Task phụ của TX..."); 
      vTaskDelay(2000 / portTICK_PERIOD_MS); // Bắt buộc dùng vTaskDelay 
    } 
  } 
  #endif 

  #ifdef ROLE_RECEIVER 
  void rxTask(void *pvParam) { 
    for(;;) { 
      // Nhiệm vụ chờ xung đã do phần cứng (EXTI) lo, Core 0 chỉ việc lấy kết quả
      if (pwmValue > 0) { 
        Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): "); 
        Serial.print(pwmValue); 
        Serial.println(" us"); 
      } 
      vTaskDelay(20 / portTICK_PERIOD_MS); // Nghỉ ngơi nhường CPU cho WiFi/OS 
    } 
  } 
  #endif 

#endif
```

### DẠNG 2: CHUẨN ESP-IDF NATIVE (Cấp Công nghiệp)
Sử dụng trực tiếp các Struct của Espressif, can thiệp sâu vào thanh ghi phần cứng. Tối ưu nhất và là tiền đề để chuyển sang dùng MCPWM điều khiển ESC sau này.

```cpp
// ========================================== 
#define ROLE_TRANSMITTER 
//#define ROLE_RECEIVER  

#define USE_DUAL_CORE 
 
const int pwmTxPin = 7; 
const int pwmRxPin = 6; 
 
TaskHandle_t Core0TaskHandle; 

// KHÁC BIỆT: Bắt buộc gọi thư viện lõi cho TX
#ifdef ROLE_TRANSMITTER
  #include "driver/ledc.h" 
#endif

// --- KHỐI NGẮT PHẦN CỨNG (DÙNG CHO RX) ---
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
 
  // --------------------------------------------------------- 
  #ifdef ROLE_TRANSMITTER 
    Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) - ESP-IDF NATIVE <<<"); 
    
    // Cấu hình Timer bằng Struct 14-bit
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pwmTxPin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 1229, // 1500us
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

  #elif defined(ROLE_RECEIVER) 
    Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) - NGẮT EXTI <<<"); 
    pinMode(pwmRxPin, INPUT_PULLDOWN); 
    attachInterrupt(pwmRxPin, pwmInterruptHandler, CHANGE);
  #endif 
 
  // --------------------------------------------------------- 
  #ifdef USE_DUAL_CORE 
    #ifdef ROLE_TRANSMITTER 
      xTaskCreatePinnedToCore(txTask, "TX_Task", 2048, NULL, 1, &Core0TaskHandle, 0); 
    #elif defined(ROLE_RECEIVER) 
      xTaskCreatePinnedToCore(rxTask, "RX_Task", 2048, NULL, 2, &Core0TaskHandle, 0); 
    #endif 
  #endif 
} 

// ... Khối void loop() và hàm Core 0 tương tự như DẠNG 1 ...
void loop() { 
  #ifdef USE_DUAL_CORE 
    #ifdef ROLE_TRANSMITTER 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)..."); 
      delay(500); 
    #endif 
  #else 
    #ifdef ROLE_TRANSMITTER 
      delay(2000); 
    #elif defined(ROLE_RECEIVER) 
      if (pwmValue > 0) { 
        Serial.print("[Single Core] Nhận được PWM: "); 
        Serial.println(pwmValue); 
      } 
      delay(20); 
    #endif 
  #endif 
}

#ifdef USE_DUAL_CORE 
  #ifdef ROLE_TRANSMITTER 
  void txTask(void *pvParam) { 
    for(;;) { vTaskDelay(2000 / portTICK_PERIOD_MS); } 
  } 
  #endif 
  #ifdef ROLE_RECEIVER 
  void rxTask(void *pvParam) { 
    for(;;) { 
      if (pwmValue > 0) { 
        Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): "); 
        Serial.println(pwmValue); 
      } 
      vTaskDelay(20 / portTICK_PERIOD_MS); 
    } 
  } 
  #endif 
#endif
```

[Placeholder for table](../assets/dual-core-RF-distributed-table.png)

## 2. Chuyện gì xảy ra khi nạp code

### Xét trên thực tế khi nạp code:

1. Nếu bạn tắt `#define USE_DUAL_CORE` (Chế độ 1 Core):
   - Compiler sẽ tự động xóa sạch các hàm khởi tạo Task của FreeRTOS và hàm rxTask/txTask ra khỏi bộ nhớ.
   - Đối với vai trò Nhận (RX), tuy code chạy chung trên vòng lặp `loop()`, nhưng hệ thống **không còn bị nghẽn**. Hàm `pulseIn()` đã được loại bỏ, thay vào đó Ngắt phần cứng (Interrupt) âm thầm đo xung ở dưới nền và chỉ trả kết quả `pwmValue` lên khi hoàn tất.
   - Kết quả: Con chip chạy mượt mà trên 1 Core, vòng lặp PID chỉ tốn đúng 1 microgiây để đọc giá trị biến mà không phải mỏi mòn chờ xung.

2. Nếu bạn bật `#define USE_DUAL_CORE` (Chế độ 2 Core):
   - Compiler sẽ nạp các hàm cấu hình Task của FreeRTOS.
   - Việc in kết quả ra Serial được "nhấc" hoàn toàn ra khỏi `loop()` và đưa vào `rxTask` chạy trên Core 0. Vòng lặp `loop()` trên Core 1 trở nên tuyệt đối cách ly và rảnh rỗi.
   - Kết quả: Vòng lặp PID (Core 1) được hưởng trọn vẹn 100% hiệu năng tính toán, trong khi Core 0 lo việc giao tiếp và in ấn thông tin.

## 3. Có dùng thư viện không?

### 1. Tại sao có dùng ledc nhưng không thấy `#include` (Ở dạng Code 1)?

Trong môi trường Arduino IDE, khi bạn chọn board là ESP32, hệ thống sẽ tự động nạp một bộ khung gọi là "ESP32 Arduino Core".

Bộ khung này tự động chèn ngầm (auto-include) các thư viện cơ bản nhất để giao tiếp với phần cứng của chip, bao gồm:

- Thư viện điều khiển LEDC: `esp32-hal-ledc.h` (Bạn có thể thấy các hàm `ledcAttach`, `ledcWrite` nằm trong đoạn code TX).
- Thư viện hệ điều hành: `FreeRTOS.h` và `task.h` (Cho phép gọi `xTaskCreatePinnedToCore`, `vTaskDelay`).
- Thư viện chuẩn của Arduino: `Arduino.h` (Chứa hàm `attachInterrupt()`, `delay()`, `pinMode()`).

Vì trình biên dịch đã tự động gọi chúng ở tầng dưới, bạn không cần phải viết tay `#include` vào đầu file `.ino` nữa. Code trông sẽ "sạch" và gọn gàng hơn rất nhiều. *(Lưu ý: Đối với code dạng 2 chuẩn ESP-IDF, bạn bắt buộc phải `#include "driver/ledc.h"` vì đây là thư viện cấp thấp, không được IDE tự động chèn).*

### 2. Sự khác biệt giữa ledc và ESP32Servo

Đoạn code Master ở trên tôi ưu tiên dùng API lõi là `ledc` thay vì `ESP32Servo` vì tính chất của chúng khác nhau:

- `ledc` (Thư viện lõi / Internal Core): Nằm sẵn trong chip. Dùng trực tiếp nó sẽ giúp code nhẹ nhất, sát với phần cứng nhất.
- `ESP32Servo` (Thư viện ngoài / External Library): Đây là thư viện do cộng đồng viết thêm (bọc bên ngoài ledc cho dễ xài giống Arduino Uno). Nếu bạn muốn dùng nó, bạn bắt buộc phải tải nó từ Library Manager và phải ghi rõ `#include <ESP32Servo.h>` ở đầu file.

## 4. Sơ đồ luồng hoạt động

[Placeholder for flowchart](../assets/dual-core-RF-flowchart.png)

## 5. Lưu ý và cách đấu dây

- **TX:** Đã phát xung 50Hz, 1500µs liên tục trên GPIO7. Để điều khiển được, bạn cần đọc ADC từ joystick (hoặc cảm biến) và thay đổi giá trị `ledcWrite` tương ứng.
- **RX:** Dùng Ngắt phần cứng (Interrupt) đọc chính xác độ rộng xung từ GPIO6. Bạn có thể ánh xạ giá trị đọc được thành lệnh điều khiển ESC/servo.
- **Phần cứng:** Code đã chạy ổn định trên ESP32-S3 Supermini. Đã quy đổi thông số xung timer về chuẩn **14-bit** của dòng chip S3. Cặp chân giao tiếp đã được dời khỏi khu vực tổn thương.

### Bảng đấu dây cho hệ thống 2 ESP32-S3 Supermini

| Chức năng | ESP32-S3 TX (Tay cầm) | ESP32-S3 RX (Drone) | Ghi chú |
|-----------|------------------------|----------------------|---------|
| Tín hiệu PWM | **GPIO7** (pwmTxPin) | **GPIO6** (pwmRxPin) | Nối trực tiếp dây tín hiệu giữa 2 GPIO |
| Mát chung | GND | GND | Nối chung GND để có cùng điện thế tham chiếu |
| Nguồn nuôi (nếu cần) | 5V (hoặc 3.3V) | 5V (hoặc 3.3V) | Có thể cấp nguồn riêng từ cổng USB hoặc pin |

Sơ đồ nối dây đơn giản nhất:
```
[ESP32-S3 TX] GPIO7  -----> GPIO6 [ESP32-S3 RX]
[ESP32-S3 TX] GND    -----> GND   [ESP32-S3 RX]
```
![Placeholder for wiring diagram](placeholder_wiring.png)

### Những điểm cần lưu ý

1. Không cần điện trở kéo: Chân RX đã được cấu hình `INPUT_PULLDOWN`, kết hợp mức cao/thấp từ TX sẽ tự thiết lập chuẩn tín hiệu.
2. Điện áp: ESP32-S3 Supermini dùng logic 3.3V, hoàn toàn tương thích khi nối trực tiếp.
3. Cảnh báo an toàn: Tuyệt đối không cắm ngược các chân tín hiệu/nguồn, đặc biệt là nguồn 5V vào các chân GPIO như sự cố đoản mạch trước đó để tránh hỏng chân vật lý.
4. Nếu muốn truyền không dây: Đây là giải pháp có dây. Muốn không dây bạn cần dùng module RF hoặc WiFi/BLE (code này không hỗ trợ).
5. Khoảng cách dây: Với tần số 50Hz, dây tín hiệu có thể dài vài mét mà không bị suy hao nhiều.