# NHẬT KÝ DỰ ÁN QUADCUPTER (PHẦN 2): TIẾN HÓA ESP32-S3 & KIẾN TRÚC RTOS
**Đối tượng:** ESP32-S3 Supermini (Dual-Core)
**Kiến trúc:** FreeRTOS (Đa nhiệm - Event-driven)

## 1. PHÂN TÍCH CHIẾN LƯỢC HIỆN TẠI
Việc chuyển dịch sang ESP32-S3 không chỉ là nâng cấp phần cứng mà là thay đổi tư duy quản lý luồng dữ liệu.

### 1.1. Giải pháp "Non-blocking" và Dual-Core
* **Tách biệt nhiệm vụ:** Core 0 dành cho giao tiếp (Bluetooth/Wifi), Core 1 dành riêng cho tính toán bay (Flight Core Task).
* **Kỹ thuật Ngắt (Interrupt):** Sử dụng chân INT của MPU6500 để đánh thức Task điều khiển qua Semaphore, giải phóng hoàn toàn CPU khỏi việc "chờ đợi" I2C.
* **Phòng ngừa rủi ro:** Hiện tại vẫn tách riêng Bluetooth+MPU và Động cơ để tránh sụt áp và nhiễu điện từ khi dùng nguồn tổ ong thay vì pin LiPo.

### 1.2. Tối ưu hóa Động cơ (16kHz PWM)
* **Điểm ngọt 16kHz:** Tần số này giúp motor chạy im lặng và bảo vệ chổi than.
* **Phân tích Nhiệt độ:** Nhiệt độ MOSFET tăng thêm $+22^\circ C$ (tổng khoảng $52^\circ C$) do tổn hao chuyển mạch ($P_{sw}$) tăng gấp 4 lần so với 4kHz.
* **Tản nhiệt tự nhiên:** Không dùng tản nhôm. Luồng gió từ cánh quạt (Propwash) sẽ làm mát MOSFET cưỡng bức, giảm hệ số tỏa nhiệt từ 100 xuống 25.

## 2. BẢNG TÍNH TOÁN VÀ THÔNG SỐ (ESP32-S3)

| Thông số / Công thức | Giá trị / Phép tính | Ý nghĩa hệ thống |
| :--- | :--- | :--- |
| **Xung nhịp CPU** | 240 MHz | Sức mạnh tính toán số thực vượt trội. |
| **Loop Time (dt)** | $1000 / 500Hz = 2.0ms$ | Chu kỳ cố định cho vòng lặp PID. |
| **I2C Time** | $(14 \times 9 / 400) + 0.15 = 0.46ms$ | Thời gian vật lý để đọc 14 byte dữ liệu thô. |
| **PWM Frequency** | 16 kHz | Tần số tối ưu cho cảm giác bay và tuổi thọ motor. |
| **Duty Cycle Limit** | 80% | Bảo vệ motor 3.7V khi dùng nguồn 4.6V. |

## 3. MÃ NGUỒN CẤU TRÚC (RTOS)

```cpp
// [Trích drn_main_board_choose.c] HAL linh hoạt giữa 2 board
void drn_main_board_choose_Delay_ms(unsigned int ms) {
#ifdef STM32F103C8T6
    // STM32: CPU bị kẹt trong vòng lặp (Blocking)
    uint32_t start_time = DRN_Millis();
    while ((DRN_Millis() - start_time) < ms) { } 
#elif defined(ESP32_S3_SUPERMINI)
    // ESP32: Task nhường CPU cho việc khác (Non-blocking)
    vTaskDelay(ms / portTICK_PERIOD_MS);
#endif
}

// [Trích drn_time.c] Lấy mili-giây từ hệ điều hành
uint32_t DRN_Millis(void) {
#elif defined(ESP32_S3_SUPERMINI)
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
#endif
}
```

## 4. PROMPT TÁI HIỆN SƠ ĐỒ (CHO DEEPSEEK)
> **Prompt:** Hãy vẽ sơ đồ luồng Mermaid (graph LR) cho kiến trúc Lõi kép ESP32-S3. Chia làm 2 subgraph: `Core 0 (System)` và `Core 1 (Flight Controller)`. Trong Core 0, có `Task Bluetooth` đẩy lệnh vào `Hàng đợi (Queue)`. Trong Core 1, `Task PID` nằm ở trạng thái nghỉ, chỉ thức dậy khi có `Ngắt INT từ MPU`. Sau đó `Task PID` thực hiện: `Lấy micro-giây (esp_timer)`, `Tính toán PID`, và `Cập nhật LEDC PWM (16kHz)`. Hãy tô màu xanh lá cho Core 1 để thể hiện tính ưu tiên cao và hiệu năng.

---