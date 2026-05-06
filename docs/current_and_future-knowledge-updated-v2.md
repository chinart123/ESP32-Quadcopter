# Lộ trình kiến thức kỹ thuật — Quadcopter Mini
## Phiên bản 2.0 — Cập nhật theo thực tế dự án
### Hai track song song: STM32F1 (học sâu) + ESP32-S3 (platform bay thật)

> **Nguyên tắc đọc tài liệu này:**
> - **Track A (STM32F1 + HAL):** Học để hiểu sâu cơ chế phần cứng. Kiến thức ở đây chuyển thẳng sang Track B vì logic là giống nhau, chỉ khác tên API.
> - **Track B (ESP32-S3):** Platform chạy drone thật. Đọc sau khi đã hiểu concept từ Track A.
> - **Track C (Thuật toán):** Không phụ thuộc chip. Hiểu một lần, dùng ở cả hai.

---

## Trạng thái hiện tại (tháng bắt đầu viết tài liệu này)

| Module | STM32F1 | ESP32-S3 |
|--------|---------|----------|
| MPU6050 đọc data | ✅ Polling hoạt động | ✅ ISR interrupt hoạt động |
| PWM motor | ✅ Quay được 20/50/70/90% | ✅ LEDC 50Hz phát được |
| PWM nhận RC | ❌ Chưa làm | ✅ ISR đo pulse width (file code-analysis) |
| PID control | ⚠️ Có file, chưa kết hợp IMU+Motor | ❌ Chưa làm |
| Bluetooth telemetry | ⚠️ HC-05/JDY-33 UART, chưa tích hợp | ❌ Chưa làm |
| DMA cho I2C | ❌ Bước tiếp theo | ❌ Bước tiếp theo |
| Kalman / Comp. filter | ✅ Đã hiểu complementary filter | ✅ (dùng kết quả từ STM32F1) |
| VL535L1X (ToF độ cao) | ❌ Không có trong BOM STM32 | ❌ Chưa làm |
| PMW3901 (Optical Flow) | ❌ Không có trong BOM STM32 | ❌ Chưa làm |

---

## Track A — STM32F1 + HAL (Keil C, không dùng CubeMX)

> **Mục đích:** Xây nền tảng bare-metal thật sự. Hiểu từng peripheral từ bên trong, debug trực tiếp, không bị lớp FreeRTOS che khuất.
>
> **Cách thêm HAL vào Keil:** Chỉ thêm file `.c/.h` cần thiết cho từng giai đoạn — không import toàn bộ package. Repo gốc: [STM32CubeF1 v1.8.0](https://github.com/STMicroelectronics/STM32CubeF1)

### A0 — HAL Core (Bắt buộc, thêm trước tất cả)

Những file này là nền móng — thiếu bất kỳ file nào HAL sẽ không compile được.

| File cần thêm | Mục đích | Ghi chú |
|---|---|---|
| `stm32f1xx_hal.c/.h` | HAL_Init(), HAL_Delay(), tick | Bắt buộc đầu tiên |
| `stm32f1xx_hal_rcc.c/.h` | Cấu hình clock (HSI/HSE/PLL) | Không có = chip chạy sai tốc độ |
| `stm32f1xx_hal_gpio.c/.h` | GPIO input/output, alternate function | Cần trước I2C và TIM |
| `stm32f1xx_hal_cortex.c/.h` | NVIC priority, SysTick | Cần cho interrupt bất kỳ |

**Lưu ý Keil:** Sau khi thêm file, phải vào `Options for Target → C/C++` thêm include path trỏ đến thư mục `Inc` của HAL Driver. Không cần thêm define `USE_HAL_DRIVER` nếu đã có trong `stm32f1xx.h`.

---

### A1 — I2C HAL + MPU6050 (Đang làm — viết lại từ polling sang HAL)

**Mục đích học:** Hiểu cách HAL trừu tượng hóa việc đọc/ghi I2C mà không cần thao tác trực tiếp vào thanh ghi `I2C_CR1`, `I2C_SR1`, `I2C_DR`.

| File cần thêm | Mục đích |
|---|---|
| `stm32f1xx_hal_i2c.c/.h` | I2C master blocking + interrupt |

**Hai hàm cốt lõi cần hiểu trước:**

```c
// Blocking — dùng để test nhanh, CPU chờ đến khi xong
HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, buf, len, HAL_MAX_DELAY);
HAL_I2C_Master_Receive(&hi2c1, MPU_ADDR, buf, len, HAL_MAX_DELAY);

// Interrupt — CPU không chờ, callback được gọi khi xong
HAL_I2C_Master_Receive_IT(&hi2c1, MPU_ADDR, buf, 14);
// → ISR tự gọi HAL_I2C_MasterRxCpltCallback() khi nhận đủ 14 byte
```

**Đường đi dữ liệu (quan trọng — hiểu cái này thì DMA sau dễ hơn):**

```
[Polling]  CPU → I2C_CR1 (START) → chờ SR1.SB → ghi DR → chờ SR1.BTF → đọc DR → lặp
[Interrupt] CPU khởi động, ra đi làm việc khác → phần cứng I2C tự chạy → NVIC gọi ISR → HAL gọi callback
[DMA]      CPU khởi động, ra đi làm việc khác → DMA tự đọc DR → ghi thẳng vào RAM → ngắt TC một lần duy nhất
```

**Lỗi thường gặp với HAL I2C trên STM32F103 (Errata):**

Chip F103 có lỗi silicon I2C nổi tiếng. HAL v1.8.0 đã có workaround nhưng cần biết để debug:
- Nếu bus bị treo (SDA stuck LOW): gọi `HAL_I2C_DeInit()` rồi `HAL_I2C_Init()` để reset peripheral
- Đọc `SR1` rồi `SR2` theo đúng thứ tự — đọc sai thứ tự sẽ xóa cờ không đúng cách
- Tài liệu tham khảo: [Errata ES096](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) — đọc mục "I2C limitations"

---

### A2 — PWM Motor HAL (Cải thiện từ code hiện có)

**Mục đích học:** Hiểu cách TIM hardware tạo xung PWM mà không cần viết thủ công vào `TIMx_CCR`. HAL wrapper `HAL_TIM_PWM_Start()` bên dưới chỉ là set bit `CEN` và `ARPE` cho bạn.

| File cần thêm | Mục đích |
|---|---|
| `stm32f1xx_hal_tim.c/.h` | TIM PWM generation |
| `stm32f1xx_hal_tim_ex.c/.h` | TIM extended (cần cho một số config) |

**Ánh xạ sang drone:**

```c
// Thay vì viết thủ công:
TIM3->CCR1 = duty_value;

// Dùng HAL:
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_value);
// Dòng này compile thành cùng 1 lệnh assembly — không có overhead
```

**Bước kế tiếp sau khi A1 và A2 xong:** Kết hợp `IMU data → complementary filter → PID → PWM duty` trong một vòng lặp có timing cố định. Đây là lúc cần quyết định thuật toán PID (xem Track C).

---

### A3 — DMA cho I2C (Bước nâng cấp sau khi A1 ổn định)

> **Khi nào cần:** Khi vòng PID chạy ở 500Hz và bạn nhận thấy jitter (dao động thời gian giữa các lần đọc IMU). Nếu chạy 200Hz và ổn định thì chưa cần vội.

| File cần thêm | Mục đích |
|---|---|
| `stm32f1xx_hal_dma.c/.h` | DMA controller |

**Đường đi DMA cho I2C1_RX trên STM32F103:**

```
MPU6050 → I2C1 peripheral (APB1) → request đến DMA1_Channel7
→ DMA đọc I2C1_DR → ghi vào RAM buffer
→ Ngắt Transfer Complete (TCIF7) → một lần duy nhất cho 14 byte
```

**Cấu hình tối thiểu:**

```c
// Gắn DMA vào I2C handle (thêm vào HAL_I2C_MspInit)
hdma_i2c1_rx.Instance = DMA1_Channel7;
hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;   // địa chỉ I2C_DR cố định
hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;        // địa chỉ RAM tăng dần
hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
HAL_DMA_Init(&hdma_i2c1_rx);
__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

// Gọi DMA transfer (non-blocking, CPU rảnh tay ngay)
HAL_I2C_Master_Receive_DMA(&hi2c1, MPU_ADDR, imu_buffer, 14);
// → callback HAL_I2C_MasterRxCpltCallback() được gọi khi đủ 14 byte
```

---

### A4 — Bluetooth Telemetry HAL (HC-05 / JDY-33)

> **Vai trò thật sự của HC-05 trong dự án này:** Không phải điều khiển realtime mà là **debug channel** — gửi góc nghiêng, PID output, PWM duty về PC/phone để quan sát khi drone đang bay.

| File cần thêm | Mục đích |
|---|---|
| `stm32f1xx_hal_uart.c/.h` | UART transmit/receive |

**Cấu hình thực tế:**

```c
// Gửi telemetry mỗi 100ms (10Hz là đủ để quan sát, không cần nhanh hơn)
// Đừng gửi trong vòng PID 500Hz — sẽ làm trễ timing
sprintf(buf, "R:%.1f P:%.1f Y:%.1f PWM:%d\r\n", roll, pitch, yaw, pwm_out);
HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 10); // timeout 10ms
```

**Giới hạn cần biết:** HC-05 classic Bluetooth có latency ~20ms và không ổn định dưới nhiễu 2.4GHz. Dùng tốt cho debug nhưng không dùng được làm control link cho drone.

---

## Track B — ESP32-S3 (Platform drone thật, BOM chính thức)

> **Nguyên tắc:** Sau khi hiểu cơ chế ở Track A, Track B chỉ là "dịch" sang API của ESP-IDF. Logic interrupt, DMA, timing là giống nhau. Chỉ tên hàm và struct là khác.
>
> **Về FreeRTOS:** Nếu không bật WiFi/BLE và chỉ dùng 1 PID task trên Core 1 + 1 ISR, hành vi gần như identical với bare-metal STM32F1. Không phức tạp hơn.

### B1 — PWM nhận RC (Đã có — file code-analysis.md)

Kiến trúc đã validate:
- ISR trên GPIO6, đo `micros()` tại cạnh lên/xuống
- `volatile unsigned long pwmValue` — đọc từ PID task
- `IRAM_ATTR` bắt buộc cho ISR

**Lỗi tiềm năng cần fix trước khi lên drone (từ phân tích trước):**

```c
// Fix 1: Đọc atomic để tránh torn read
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portENTER_CRITICAL(&mux);
unsigned long snapshot = pwmValue;
portEXIT_CRITICAL(&mux);

// Fix 2: Failsafe mất tín hiệu
if (millis() - lastPulseTime > 100) {
    snapshot = 1000; // disarm — về 0% throttle
}

// Fix 3: Sanity check xung hợp lệ
if (snapshot < 800 || snapshot > 2200) snapshot = 1500; // loại giá trị rác
```

---

### B2 — LEDC PWM Motor → MCPWM (Nâng cấp khi test ESC thật)

**Hiện tại (LEDC — đủ cho test):**

```cpp
ledcAttach(MOTOR_PIN, 50, 14);
ledcWrite(MOTOR_PIN, duty); // duty = 819 (1000µs) đến 1638 (2000µs)
```

**Nâng cấp lên MCPWM (khi cần điều khiển 4 motor chính xác đồng thời):**

```c
#include "driver/mcpwm_prelude.h"
// MCPWM cho phép sync 4 channel, jitter < 100ns
// So với LEDC jitter ~1µs — quan trọng khi cần motor đồng đều
```

Chuyển sang MCPWM khi: LEDC gây rung không đều giữa 4 motor hoặc khi cần failsafe hardware.

---

### B3 — I2C GDMA + Semaphore cho MPU6050

> **Đọc A1 và A3 trước** — concept DMA là y chang, chỉ khác API.

**Tài liệu:** [ESP-IDF I2C Driver (new API)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html)

```c
// Tạo I2C bus và device handle (new driver — không dùng i2c_driver_install cũ)
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t mpu_handle;

i2c_master_bus_config_t bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_8,
    .scl_io_num = GPIO_NUM_9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
};
i2c_new_master_bus(&bus_cfg, &bus_handle);

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x68, // MPU6050 AD0=LOW
    .scl_speed_hz = 400000, // Fast mode
};
i2c_master_bus_add_device(bus_handle, &dev_cfg, &mpu_handle);

// Đọc 14 byte burst (acc_x, acc_y, acc_z, temp, gyro_x, gyro_y, gyro_z)
uint8_t reg = 0x3B;
uint8_t buf[14];
i2c_master_transmit_receive(mpu_handle, &reg, 1, buf, 14, -1); // -1 = no timeout
```

**Kiến trúc FreeRTOS cho PID task (đây là pattern an toàn nhất):**

```c
// ISR — cực ngắn, chỉ nhả semaphore
void IRAM_ATTR mpu_isr_handler(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(imu_semaphore, &woken);
    portYIELD_FROM_ISR(woken); // scheduler chạy PID task ngay lập tức
}

// PID task — chạy trên Core 1
void pid_task(void *arg) {
    while (1) {
        // Block tại đây — không tốn CPU khi chờ
        if (xSemaphoreTake(imu_semaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            read_mpu6050(buf);
            compute_complementary_filter();
            compute_cascade_pid();
            update_motor_pwm();
        } else {
            // Timeout 5ms = MPU không gửi data → failsafe
            set_all_motors(IDLE_THROTTLE);
        }
    }
}

// Setup
void app_main() {
    imu_semaphore = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 5, NULL, 1); // Core 1
    gpio_isr_handler_add(MPU_INT_PIN, mpu_isr_handler, NULL);
}
```

**Lý do kiến trúc này không đáng sợ:** Chỉ có 1 task thật sự (`pid_task`) + 1 ISR. Không có mutex, không có priority inversion. Semaphore ở đây chỉ là "cờ báo có data mới" — đơn giản như `volatile bool` nhưng an toàn hơn.

---

### B4 — VL535L1X ToF Sensor (Độ cao laser)

> **Chưa có trong lộ trình cũ — bổ sung mới**

**Giao tiếp:** I2C (dùng chung bus với MPU6050, địa chỉ mặc định `0x29`)

**Thư viện khuyến nghị:** [pololu/vl53l1x-arduino](https://github.com/pololu/vl53l1x-arduino) hoặc driver C thuần từ STMicroelectronics.

**Tích hợp vào PID task:**

```c
// Đọc độ cao mỗi 50ms (20Hz — ToF không nhanh hơn được)
// Không đọc trong mọi vòng PID 500Hz — dùng kết quả cũ nếu chưa có sample mới
static uint32_t last_tof_read = 0;
if (millis() - last_tof_read > 50) {
    altitude_mm = vl53l1x_read_distance();
    last_tof_read = millis();
}
// Altitude PID dùng altitude_mm làm feedback
```

**Lưu ý:** VL535L1X hoạt động tốt trên bề mặt phẳng, sàn gỗ, thảm. Mất chính xác trên mặt gương hoặc ngoài sáng mạnh. Tầm đo hiệu quả 0–200cm cho indoor hover.

---

### B5 — PMW3901 Optical Flow (Giữ vị trí ngang)

> **Chưa có trong lộ trình cũ — bổ sung mới**

**Giao tiếp:** SPI (không dùng I2C — cần tốc độ cao hơn)

```c
#include "driver/spi_master.h"
// SPI mode 3, max 2MHz cho PMW3901
// Đọc deltaX, deltaY mỗi 20ms (50Hz)
```

**Tích hợp vào hệ thống:**

```
deltaX, deltaY từ PMW3901 (pixel/frame)
    → nhân với hệ số tỉ lệ theo độ cao (từ VL535L1X)
    → ra được velocity (cm/s)
    → đưa vào Position PID (outer loop)
    → output là Roll/Pitch setpoint cho Attitude PID (inner loop)
```

Đây chính là kiến trúc Cascade PID 3 tầng mà bạn đã quen từ xe tự cân bằng, chỉ thêm một tầng ngoài cùng.

---

## Track C — Thuật toán (Không phụ thuộc chip)

> Học một lần ở Track A, áp dụng lại ở Track B. Code chỉ thay tên biến.

### C1 — Complementary Filter (Đã hiểu — tổng kết)

```c
// alpha càng cao → tin gyro hơn (ít drift nhưng tích lũy lâu dài)
// alpha càng thấp → tin accelerometer hơn (ít drift nhưng nhạy rung động)
// Thực tế: alpha = 0.98 là điểm tốt nhất cho drone indoor
angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle;
```

**Khi nào cần Kalman thay thế:** Nếu drone hoạt động ngoài trời (gió mạnh, nhiễu lớn) hoặc cần độ chính xác góc < 0.5°. Cho indoor hover, complementary filter là đủ và dễ tune hơn nhiều.

---

### C2 — Cascade PID cho Drone (Bước kết hợp IMU + Motor)

**Đây là bước bạn đang bị kẹt — IMU và Motor đã có nhưng chưa kết hợp.**

Kiến trúc đúng cho quadcopter:

```
[Outer loop — 50Hz]         [Inner loop — 500Hz]
                             
RC setpoint (góc mong muốn)
    ↓
Attitude PID
  error = setpoint_angle - current_angle (từ comp. filter)
  output = angular_rate_setpoint
    ↓
Rate PID  ←── angular_rate thực tế (từ gyro trực tiếp, không qua filter)
  error = rate_setpoint - gyro_rate
  output = motor_mix
    ↓
Motor Mixing:
  Motor1 = base_throttle + roll_out + pitch_out - yaw_out
  Motor2 = base_throttle - roll_out + pitch_out + yaw_out
  Motor3 = base_throttle - roll_out - pitch_out - yaw_out
  Motor4 = base_throttle + roll_out - pitch_out + yaw_out
```

**Thứ tự tune PID (quan trọng — sai thứ tự = drone không bay được):**

1. Tune **Rate PID** trước (P nhỏ cho đến khi drone giữ được góc tay đỡ, không dao động)
2. Tune **Attitude PID** sau khi Rate PID ổn
3. Tune **Position PID** (nếu có PMW3901) cuối cùng

**Giá trị khởi điểm cho micro drone 8.5×20mm motor:**

```c
// Rate PID — cẩn thận, bắt đầu rất nhỏ
rate_pid.kP = 0.5f;
rate_pid.kI = 0.0f; // bật I sau khi P ổn
rate_pid.kD = 0.01f;

// Attitude PID
att_pid.kP = 3.0f;
att_pid.kI = 0.0f;
att_pid.kD = 0.0f; // D thường không cần ở outer loop
```

---

### C3 — Bluetooth Telemetry — Debug Protocol

**Format dữ liệu gợi ý (gọn, dễ parse bằng Python/Serial plotter):**

```c
// Gửi 10Hz — không ảnh hưởng PID loop
// Format: CSV cho dễ plot bằng Arduino Serial Plotter hoặc Python matplotlib
sprintf(buf, "%.2f,%.2f,%.2f,%d,%d,%d,%d\r\n",
    roll, pitch, yaw,
    motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);
HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 10);
```

**Python script nhận telemetry (2 dòng):**
```python
import serial, matplotlib.pyplot as plt
# Đủ để plot realtime roll/pitch khi đang tune PID
```

---

## Lộ trình thực tế — Thứ tự ưu tiên

```
NGAY BÂY GIỜ (STM32F1 — Track A)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
A2 → Kết hợp IMU + Motor trong 1 loop có timing cố định
C2 → Implement Rate PID với gyro raw (chưa cần Attitude PID)
     Mục tiêu: cầm drone bằng tay, nghiêng → motor phản ứng đúng chiều

SAU KHI RATE PID HOẠT ĐỘNG (STM32F1)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
C2 → Thêm Attitude PID (outer loop)
A4 → Thêm HC-05 telemetry để quan sát PID output khi tune
A3 → Nâng cấp I2C lên DMA nếu jitter > 0.5ms

SONG SONG (ESP32-S3 — Track B)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
B1 → Fix 3 lỗi tiềm năng trong ISR PWM (critical + failsafe + sanity check)
B3 → Port code PID từ STM32F1 sang (chỉ thay API, logic giữ nguyên)

KHI ESP32-S3 BAY ĐƯỢC (Track B tiếp)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
B4 → Tích hợp VL535L1X (altitude hold)
B5 → Tích hợp PMW3901 (position hold)
B2 → Nâng cấp LEDC → MCPWM nếu cần

GIAI ĐOẠN CUỐI
━━━━━━━━━━━━━━
Thay HC-05 bằng ESP-NOW hoặc ELRS cho control link thật
```

---

## Tài liệu tham khảo

| Tài liệu | Link | Dùng cho |
|---|---|---|
| STM32CubeF1 HAL v1.8.0 | [GitHub](https://github.com/STMicroelectronics/STM32CubeF1) | Track A — tất cả |
| STM32F103 Reference Manual (RM0008) | [ST PDF](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) | A3 DMA — đọc khi cần hiểu sâu thanh ghi |
| STM32F103 Errata (ES096) | [ST PDF](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) | A1 — I2C bug workaround |
| ESP32-S3 Technical Reference Manual | [Espressif PDF](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf) | B3, B5 — khi cần hiểu GDMA/SPI sâu |
| ESP-IDF I2C Driver (new API) | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html) | B3 |
| ESP-IDF GPIO & Interrupt | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) | B1, B3 |
| ESP-IDF FreeRTOS | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_idf.html) | B3 — chỉ đọc phần Task + Semaphore |
| MPU6050 Register Map | [InvenSense PDF](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) | A1, B3 — địa chỉ thanh ghi |
| VL53L1X Datasheet | [ST](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) | B4 |

---

*Tài liệu version 2.0 — Cập nhật theo BOM thực tế (ESP32-S3 + STM32F1 dual track)*
*Xóa: Toàn bộ nội dung bare-metal STM32F103 register level (thay bằng HAL)*
*Xóa: Kế hoạch Bluetooth làm control link (thay bằng debug-only role)*
*Thêm: B4 VL535L1X, B5 PMW3901, C2 Cascade PID implementation, C3 Telemetry protocol*