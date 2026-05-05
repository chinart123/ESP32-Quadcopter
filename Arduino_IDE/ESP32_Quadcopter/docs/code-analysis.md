# ARDUINO-CODE-UPDATE

## 1. CODE-VER 1

```cpp
// ========================================== 
// 1. CẤU HÌNH VAI TRÒ (CHỌN 1 TRONG 2) 
// ========================================== 
#define ROLE_TRANSMITTER // Bật dòng này để làm mạch Phát (Tay cầm) 
//#define ROLE_RECEIVER // Bật dòng này để làm mạch Nhận (Drone) 
 
// ========================================== 
// 2. CẤU HÌNH KIẾN TRÚC (BẬT ĐỂ DÙNG 2 CORE) 
// ========================================== 
#define USE_DUAL_CORE // Bật dòng này để dùng FreeRTOS tách luồng. Tắt đi (//) 
để chạy 1 Core. 
 
// --- Khai báo chân (Pins) --- 
const int pwmTxPin = 4; // Chân phát PWM (chỉ dùng khi là TX) 
const int pwmRxPin = 5; // Chân nhận PWM (chỉ dùng khi là RX) 
 
TaskHandle_t Core0TaskHandle; // Biến quản lý Task nếu dùng Dual Core 
 
void setup() { 
 Serial.begin(115200); 
 delay(1000); // Chờ Serial ổn định 
 
 // --------------------------------------------------------- 
 // KHỞI TẠO PHẦN CỨNG THEO VAI TRÒ 
 // --------------------------------------------------------- 
 #ifdef ROLE_TRANSMITTER 
 Serial.println(">>> CHẾ ĐỘ: TAY CẦM PHÁT (TX) <<<"); 
 ledcSetup(0, 50, 16); // Tần số 50Hz, 16-bit 
 ledcAttachPin(pwmTxPin, 0); // Gắn chân 
 ledcWrite(0, 4915); // Phát độ rộng 1500us 
 #elif defined(ROLE_RECEIVER) 
 Serial.println(">>> CHẾ ĐỘ: DRONE NHẬN (RX) <<<"); 
 pinMode(pwmRxPin, INPUT); 
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
 Serial.println("Kiến trúc: SINGLE CORE (Chạy tuần tự trên Core 1)"); 
 #endif 
 Serial.println("============================="); 
} 

void loop() { 
 // ============================================================== 
 // VÒNG LẬP CHÍNH CỦA HỆ THỐNG (LUÔN CHẠY TRÊN CORE 1) 
 // ============================================================== 
 #ifdef USE_DUAL_CORE 
 // --- NẾU DÙNG DUAL CORE --- 
 // Các tác vụ block/chờ đợi đã bị ném sang Core 0. Core 1 rảnh rỗi. 
 #ifdef ROLE_TRANSMITTER 
 Serial.println("[Core 1] Đang rảnh rỗi chờ xử lý nút bấm/màn hình..."); 
 delay(1000); 
 #elif defined(ROLE_RECEIVER) 
 Serial.println("[Core 1] Đang chạy vòng lặp PID siêu mượt (không bị delay)..."); 
 delay(10); // Giả lập thời gian tính toán thuật toán 
 #endif 
 #else 
 // --- NẾU CHỈ DÙNG 1 CORE (SINGLE CORE) --- 
 // Mọi thứ phải xếp hàng chạy tuần tự tại đây 
 #ifdef ROLE_TRANSMITTER 
 Serial.println("[Core 1] TX vẫn đang phát PWM tự động bằng phần cứng..."); 
 delay(1000); 
 #elif defined(ROLE_RECEIVER) 
 // Lệnh đọc này chặn (block) toàn bộ Core 1, PID phải đợi. 
 unsigned long pulse = pulseln(pwmRxPin, HIGH); 
 if (pulse > 0) { 
 Serial.println("[Core 1] Nhận được PWM: "); 
 Serial.print(pulse); 
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
 Serial.println("[Core 0] Task phụ của TX (Ví dụ: Đọc ADC từ Joystick)..."); 
 vTaskDelay(500 / portTICK_PERIOD_MS); // Bắt buộc dùng vTaskDelay 
 } 
 } 
 #endif 

 #ifdef ROLE_RECEIVER 
 void rxTask(void *pvParam) { 
 for(;;) { 
 // Tác vụ chờ xung (block) được nhốt hoàn toàn ở Core 0 
 unsigned long pulse = pulseln(pwmRxPin, HIGH); 
 if (pulse > 0) { 
 Serial.print("[Core 0] Nhận được PWM (Không ảnh hưởng Core 1): "); 
 Serial.print(pulse); 
 Serial.println(" us"); 
 } 
 vTaskDelay(10 / portTICK_PERIOD_MS); // Nghỉ ngơi nhường CPU cho WiFi/OS 
 } 
 } 
 #endif 

 #endif
```
[Placeholder for table](../assets/dual-core-RF-distributed-table.png)
## 2. Chuyện gì xảy ra khi nạp code

### Xét trên thực tế khi nạp code:

1. Nếu bạn tắt #define USE_DUAL_CORE (Chế độ 1 Core):
   - Compiler sẽ tự động xóa sạch các hàm khởi tạo Task của FreeRTOS và hàm rxTask/txTask ra khỏi bộ nhớ.
   - Đối với vai trò Nhận (RX), hàm pulseIn() lại nằm chỉnh inh ngay giữa vòng lặp loop().
   - Kết quả: Con chip chạy một đoạn mã y hệt như file RX_1Core rời rạc ban này. Hàm pulseIn() lại tiếp tục chặn (block) Core 1, khiến hệ thống chỉ hưởng lợi ngầm khoảng 30% - 40% từ Core 0 xử lý WiFi.

2. Nếu bạn bật #define USE_DUAL_CORE (Chế độ 2 Core):
   - Compiler sẽ nạp các hàm cấu hình Task của FreeRTOS.
   - Hàm pulseIn() bị nhắc hoàn toàn ra khỏi loop() và "nhốt" vào rxTask chạy trên Core 0. Vòng lặp loop() trên Core 1 trở nên trống rỗng và rảnh rỗi.
   - Kết quả: Con chip chạy một đoạn mã y hệt file RX_2Core rời rạc. Vòng lập PID (Core 1) được cách ly an toàn khỏi sự chậm trễ của xung PWM, giúp hệ thống hướng lợi 100% hiệu năng Dual Core.

## 3. Có dùng thư viện không?

### 1. Tại sao có dùng ledc nhưng không thấy #include?

Trong môi trường Arduino IDE, khi bạn chọn board là ESP32, hệ thống sẽ tự động nạp một bộ khung gọi là "ESP32 Arduino Core".

Bộ khung này tự động chèn ngầm (auto- include) các thư viện cơ bản nhất để giao tiếp với phần cứng của chip, bao gồm:

- Thư viện điều khiển LEDC: esp32-hal-ledc.h (Bạn có thể thấy các hàm ledcSetup, ledcAttachPin, ledcWrite nằm trong đoạn code TX).
- Thư viện hệ điều hành: FreeRTOS.h và task.h (Cho phép gọi xTaskCreatePinnedToCore, vTaskDelay).
- Thư viện chuẩn của Arduino: Arduino.h (Chứa hàm pulseIn(), delay(), pinMode()).

Vi trình biên dịch đã tự động gọi chúng ở tầng dưới, bạn không cần phải viết tay #include vào đầu file .ino nữa. Code trông sẽ "sạch" và gọn gàng hơn rất nhiều.

### 2. Sự khác biệt giữa ledc và ESP32Servo

Đoạn code Master ở trên tôi ưu tiên dùng API lôi là ledc thay vi ESP32Servo vi tính chất của chúng khác nhau:

- ledc (Thư viện lõi / Internal Core): Nằm sẵn trong chip. Dùng trực tiếp nó sẽ giúp code nhẹ nhất, sát với phần cứng nhất, và không cần cài thêm gì cả (không cần #include).
- ESP32Servo (Thư viện ngoài / External Library): Đây là thư viện do cộng đồng viết thêm (bọc bên ngoài ledc cho dễ xài giống Arduino Uno). Nếu bạn muốn dùng nó, bạn bắt buộc phải tải nó từ Library Manager và phải ghi rõ #include <ESP32Servo.h> ở đầu file.

## 4. Sơ đồ luồng hoạt động

[Placeholder for flowchart](../assets/dual-core-RF-flowchart.png)

## 5. Lưu ý và cách đấu dây

- TX: Đã phát xung 50Hz, 1500µs liên tục trên GPIO4. Để điều khiển được, bạn cần đọc ADC từ joystick (hoặc cảm biến) và thay đổi giá trị ledcWrite(0, ...) tương ứng.
- RX: Dùng pulseIn đọc chính xác độ rộng xung từ GPIO5. Bạn có thể ánh xạ giá trị đọc được thành lệnh điều khiển ESC/servo.
- Code đã chạy ổn định trên phần cứng ESP32-S3 Supermini mà không cần chỉnh sửa thư viện.

### Bảng đấu dây cho hệ thống 2 ESP32-S3 Supermini

| Chức năng | ESP32-S3 TX (Tay cầm) | ESP32-S3 RX (Drone) | Ghi chú |
|-----------|------------------------|----------------------|---------|
| Tín hiệu PWM | GPIO4 (pwmTxPin) | GPIO5 (pwmRxPin) | Nối trực tiếp dây tín hiệu giữa 2 GPIO |
| Mát chung | GND | GND | Nối chung GND để có cùng điện thế tham chiếu |
| Nguồn nuôi (nếu cần) | 5V (hoặc 3.3V) | 5V (hoặc 3.3V) | Có thể cấp nguồn riêng từ cổng USB hoặc pin |

Sơ đồ nối dây đơn giản nhất:
```
[ESP32-S3 TX] GPIO4  -----> GPIO5 [ESP32-S3 RX]
[ESP32-S3 TX] GND    -----> GND   [ESP32-S3 RX]
```
![Placeholder for wiring diagram](placeholder_wiring.png)

### Những điểm cần lưu ý

1. Không cần điện trở kéo: Chân RX đã được cấu hình INPUT, mức cao/thấp từ TX sẽ tự thiết lập.
2. Điện áp: ESP32-S3 Supermini dùng logic 3.3V, hoàn toàn tương thích khi nối trực tiếp.
3. Nếu muốn truyền không dây: Đây là giải pháp có dây. Muốn không dây bạn cần dùng module RF hoặc WiFi/BLE (code này không hỗ trợ).
4. Khoảng cách dây: Với tần số 50Hz, dây tín hiệu có thể dài vài mét mà không bị suy hao nhiều.
`````
