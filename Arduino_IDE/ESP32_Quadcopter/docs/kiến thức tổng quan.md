# Technical Log: Đánh giá kiến trúc PWM cho ESC/Servo
**Module**: Flight Controller Motor Output
**Directory**: `ESP32 RTOS Evolution`

Dưới đây là bảng đánh giá tổng quan các thư viện được cấu trúc lại, tập trung tối đa vào mục tiêu xây dựng Hệ thống điều khiển bay (Flight Controller) cho Drone:

| Thông số / Tiêu chí | Arduino IDE | ESP-IDF |
| :--- | :--- | :--- |
| **Thư viện (Lựa chọn 1)** | **ESP32Servo** | **LEDC (Driver)** |
| **Ưu điểm** | Setup cực kỳ nhanh. API quen thuộc, dễ gọi hàm trực tiếp. | Quản lý bằng struct chuẩn C. Tương thích hoàn hảo để cấp phát trong các task của FreeRTOS. |
| **Nhược điểm** | Độ trễ (latency) lớn, dễ bị nhiễu nhịp thời gian khi vòng lặp PID tính toán nặng. | Vẫn là phần cứng cho đèn LED, phải cập nhật từng kênh một (gây lệch siêu nhỏ giữa các motor). |
| **Ghi chú (Cho Drone)** | Chỉ nên dùng để **test nguyên lý** hoặc test xem ESC/Motor còn sống hay không. | Cân bằng tốt giữa độ khó và hiệu năng. Dễ cấu trúc code hơn MCPWM. |
| **Thư viện (Lựa chọn 2)** | **LEDC (Core)** | **MCPWM** |
| **Ưu điểm** | Linh hoạt cấu hình tần số (lên 400Hz). Xử lý nhẹ nhàng hơn ESP32Servo. | **Đồng bộ hóa tuyệt đối (Sync)** xuất 4 xung ESC ở cùng 1 thời điểm. Hỗ trợ ngắt an toàn. |
| **Nhược điểm** | Không có cơ chế đồng bộ phần cứng (Hardware Sync) giữa 4 kênh. | Khó học, API phức tạp, đòi hỏi hiểu biết sâu về thanh ghi Timer. |
| **Ghi chú (Cho Drone)** | Phù hợp để làm **bản nháp (Prototype)** ban đầu trước khi chuyển qua cấu trúc phức tạp. | **Tiêu chuẩn công nghiệp**. Là vũ khí tối thượng cho nhánh kiến trúc RTOS nâng cao. |

### 💡 Chiến lược áp dụng thực tế cho Drone Project:

1. **Giai đoạn Test phần cứng (Arduino IDE + ESP32Servo):** Dùng để xác nhận bạn cắm dây đúng, ESC nhận được tín hiệu 1500us và motor quay ổn định. Không cần mất thời gian viết code dài ở bước này.
2. **Giai đoạn Tối ưu thuật toán (ESP-IDF + MCPWM):** Khi bạn đã dựng xong phần khung cho hệ thống nhúng và bắt đầu ghép nối bộ lọc (như Complementary Filter) với vòng lặp PID, sự chênh lệch dù chỉ là vài microgiây giữa 4 motor cũng sẽ làm drone bị trôi (drift). Lúc này, chuyển sang dùng ngoại vi **MCPWM** trên ESP-IDF để phần cứng tự động khóa đồng bộ 4 xung PWM sẽ giúp giải phóng đáng kể CPU, đảm bảo tần số phản hồi real-time tốt nhất.



# Technical Log: ESP32 Core Architecture & RTOS Relationship

**Q1: Nếu "không dùng RTOS" thì code chạy trên 1 core hay 2 core của ESP32?**
* **Trả lời:** Khung phần mềm chuẩn của ESP32 (ESP-IDF/Arduino) luôn chạy FreeRTOS ngầm. Nếu chỉ viết code tuần tự bình thường (không gọi API tạo task của RTOS), code của bạn sẽ mặc định chạy trên **1 core duy nhất (Core 1)**. Để ép tác vụ chạy trên Core 0, bắt buộc phải dùng lệnh của FreeRTOS (ví dụ: `xTaskCreatePinnedToCore()`).

**Q2: Các API mặc định trên Arduino IDE có tận dụng được Dual Core không?**
* **Trả lời:** Có, theo cách phân công sẵn bởi thư viện:
    * **Core 1 (App Core):** Chạy toàn bộ code trong hàm `setup()` và `loop()`.
    * **Core 0 (Pro Core):** Chạy ngầm các tác vụ hệ thống (WiFi, Bluetooth, các tiến trình nền).
    * *Lưu ý:* User vẫn có thể gọi hàm FreeRTOS ngay trong file `.ino` của Arduino để đưa task tính toán nặng sang Core 0 chạy song song.

**Q3: Mối quan hệ giữa RTOS, ARDUINO IDE, ESP-IDF và DUAL CORE là gì?**
* **Trả lời:** Có thể hiểu theo kiến trúc phân tầng (từ dưới lên trên):
    1. **Tầng Hardware (DUAL CORE):** Lõi chip vật lý (Core 0 và Core 1).
    2. **Tầng OS (FreeRTOS):** Hệ điều hành phân bổ tác vụ và quản lý 2 core phần cứng.
    3. **Tầng SDK (ESP-IDF):** Khung lập trình gốc (C) của Espressif, bao bọc FreeRTOS và cung cấp API điều khiển phần cứng chuyên sâu.
    4. **Tầng Wrapper (ARDUINO IDE):** Lớp vỏ bọc (C++) nằm trên cùng, giấu đi sự phức tạp của ESP-IDF và RTOS. Nó giúp code dễ dàng như vi điều khiển đơn luồng truyền thống, nhưng bản chất bên dưới hệ thống vẫn dịch lệnh và chạy trên nền Dual Core & FreeRTOS.

# Technical Log: Phân bổ tài nguyên Dual Core trên Arduino IDE (ESP32)
**Module**: Core Assignment & Optimization
**Lưu ý kỹ thuật:** Bản thân các thư viện không tự giới hạn số Core. Việc chạy 1 Core hay 2 Core phụ thuộc hoàn toàn vào **cách bạn cấu trúc mã lệnh** (code tuần tự trong `loop()` hay dùng Task của FreeRTOS).

Dưới đây là bảng phân loại cách sử dụng thư viện/API tương ứng với kiến trúc phần cứng trên môi trường Arduino IDE:

| Cấu trúc thực thi | API / Thư viện tiêu biểu | Bản chất hoạt động trên ESP32 | Ứng dụng thực tế cho Drone |
| :--- | :--- | :--- | :--- |
| **Chạy Tuần tự<br>(1 Core - Mặc định là Core 1)** | - `ESP32Servo`<br>- `ledcWrite()`<br>- `analogWrite()`<br>- `Wire.h` (Đọc I2C) | Toàn bộ mã đặt trong `setup()` và `loop()` mặc định bị "nhốt" vào **Core 1** (App Core). Mã chạy tuần tự từ trên xuống dưới, dễ bị trễ nhịp (latency) nếu có tác vụ nặng. | Dùng để test nguyên lý cơ bản, đọc cảm biến hoặc điều khiển motor ở mức độ đơn giản (chưa ghép PID). |
| **Chạy Đa luồng<br>(2 Core - Core 0 & Core 1)** | - **FreeRTOS API** (Tích hợp sẵn):<br>  + `xTaskCreatePinnedToCore()`<br>  + `vTaskDelay()`<br>  + `xQueueSend()` | Sử dụng hàm tạo Task của FreeRTOS để ép các khối lệnh chạy song song. Tách việc ra cho **Core 0** (chịu tải mạng/cảm biến) và **Core 1** (chịu tải thuật toán). | - **Core 1:** Chuyên biệt chạy vòng lặp PID tốc độ cao và xuất PWM cho ESC.<br>- **Core 0:** Nhận data Bluetooth/RC và đọc MPU6050. |

### 💡 Tóm tắt để xác nhận (Validation point):
Trên Arduino IDE, nếu bạn chỉ include thư viện (như `ESP32Servo`) và gọi trong `loop()`, nó là **1 Core**. Để thực sự dùng được **2 Core**, bạn **bắt buộc phải sử dụng API của FreeRTOS** để bao bọc các hàm của thư viện đó và gán (pin) chúng sang Core 0.