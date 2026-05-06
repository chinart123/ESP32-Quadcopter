# Lộ trình kiến thức kỹ thuật — Quadcopter Mini
## Phiên bản 3.0
### Hai track song song: STM32F1 (học sâu) + ESP32-S3 (platform bay thật)

---

## Mục lục

| # | Mục | Trạng thái |
|---|-----|-----------|
| | **Trạng thái hiện tại** | |
| | [Bảng tiến độ tổng quan](#trạng-thái-hiện-tại) | |
| | **Track A — STM32F1 + HAL (Keil C)** | |
| A0 | [HAL Core — file bắt buộc](#a0--hal-core-bắt-buộc-thêm-trước-tất-cả) | ✅ Nền móng |
| A1 | [I2C HAL + MPU6050](#a1--i2c-hal--mpu6050-đang-làm--viết-lại-từ-polling-sang-hal) | ⚠️ Đang làm |
| A2 | [PWM Motor HAL](#a2--pwm-motor-hal-đã-hoạt-động--tổng-kết) | ✅ Quay được |
| A3 | [**Test tích hợp IMU + Motor (không PID)**](#a3--test-tích-hợp-imu--motor-không-pid--bước-tiếp-theo-quan-trọng-nhất) | ❌ Bước tiếp theo |
| A4 | [Cascade PID — Rate loop](#a4--cascade-pid-rate-loop-inner-loop--làm-trước) | ❌ Sau A3 |
| A5 | [Cascade PID — Attitude loop](#a5--cascade-pid-attitude-loop-outer-loop--làm-sau-a4) | ❌ Sau A4 |
| A6 | [Bluetooth Telemetry HC-05](#a6--bluetooth-telemetry-hc-05-debug-channel--không-phải-control) | ⚠️ Có data, chưa có PID |
| A7 | [I2C Interrupt HAL_IT](#a7--i2c-interrupt-hal_it--nâng-cấp-từ-a1) | ❌ Nâng cấp từ A1 |
| A8 | [I2C DMA](#a8--i2c-dma--nâng-cấp-từ-a7) | ❌ Nâng cấp từ A7 |
| | **Track B — ESP32-S3** | |
| B0 | [drn_main_board_choose — abstraction layer](#b0--drn_main_board_choose--abstraction-layer) | ⚠️ Cần đọc kỹ |
| B1 | [MPU6050 Polling → Port từ STM32F1](#b1--mpu6050-polling--nâng-lên-interrupt-port-từ-a1) | ✅ Đang dùng polling |
| B2 | [PWM Motor LEDC](#b2--pwm-motor-ledc-port-từ-a2) | ❌ Học từ A2 |
| B3 | [PWM nhận RC — ISR](#b3--pwm-nhận-rc--isr-đã-có-cần-fix-3-lỗi) | ✅ Có, cần fix 3 lỗi |
| B4 | [Test tích hợp IMU + Motor (không PID)](#b4--test-tích-hợp-imu--motor-không-pid-port-từ-a3) | ❌ Sau B2 |
| B5 | [Cascade PID — Port từ STM32F1](#b5--cascade-pid--port-từ-stm32f1) | ❌ Sau B4 |
| B6 | [Telemetry USB CDC](#b6--telemetry-usb-cdc-thay-thế-hc-05-trong-giai-đoạn-debug) | ❌ Debug nhanh |
| B7 | [I2C Interrupt + GDMA](#b7--i2c-interrupt--gdma-port-từ-a7a8) | ❌ Nâng cấp từ B1 |
| B8 | [VL535L1X ToF — Độ cao](#b8--vl535l1x-tof-sensor-độ-cao-laser) | ❌ Sau PID ổn định |
| B9 | [PMW3901 Optical Flow — Giữ vị trí](#b9--pmw3901-optical-flow-giữ-vị-trí-ngang) | ❌ Sau B8 |
| | **Track C — Thuật toán (không phụ thuộc chip)** | |
| C1 | [Complementary Filter](#c1--complementary-filter-đã-hiểu--tổng-kết) | ✅ Đã hiểu |
| C2 | [Motor Mixer — Logic và test](#c2--motor-mixer-mô-phỏng-xong--cần-test-thật-ở-a3b4) | ⚠️ Mô phỏng xong, chưa test thật |
| C3 | [Cascade PID — Thứ tự tune](#c3--cascade-pid-thứ-tự-và-giá-trị-khởi-điểm) | ❌ Bước tiếp theo quan trọng nhất |
| C4 | [Quy trình test tích hợp từng bước](#c4--quy-trình-test-tích-hợp-từng-bước) | ❌ Đọc trước khi làm A3/B4 |
| | [**Tài liệu tham khảo**](#tài-liệu-tham-khảo) | |

---

## Trạng thái hiện tại

| Module | STM32F1 | ESP32-S3 | Ghi chú |
|--------|---------|----------|---------|
| MPU6050 đọc data | ✅ Polling | ✅ Polling | ESP32 chưa dùng interrupt |
| PWM motor | ✅ 20/50/70/90% | ❌ Chưa quay | Học từ STM32F1 sẽ nhanh |
| PWM nhận RC | ❌ | ✅ ISR có sẵn | Cần fix 3 lỗi (xem B3) |
| IMU + Motor kết hợp | ❌ Bước A3 | ❌ Bước B4 | **Đây là điểm bị kẹt** |
| PID | ⚠️ Có file, chưa kết hợp | ❌ | Chờ sau A3 |
| Motor-mixer | ⚠️ Mô phỏng log terminal | ❌ | Chưa test motor thật |
| Bluetooth telemetry | ✅ HC-05 → Hercules + Python + App | ❌ | Dùng USB CDC thay thế |
| I2C Interrupt | ❌ | ❌ | Sau polling ổn định |
| I2C DMA | ❌ | ❌ | Sau interrupt |
| VL535L1X | ❌ | ❌ | Sau PID bay được |
| PMW3901 | ❌ | ❌ | Sau VL535L1X |

> **Nguyên tắc đọc file này:**
> - Track A học sâu cơ chế. Track B chỉ là "dịch" API sang ESP-IDF — logic giống hệt.
> - Track C không phụ thuộc chip — hiểu một lần, dùng ở cả hai.
> - **Đọc C4 trước khi bắt đầu A3 hoặc B4** — quy trình test đúng giúp tránh mất nhiều ngày debug không cần thiết.

---

## Track A — STM32F1 + HAL (Keil C, không dùng CubeMX)

> Repo gốc: [STM32CubeF1 v1.8.0](https://github.com/STMicroelectronics/STM32CubeF1)
> Website ST: [stm32cubef1](https://www.st.com/en/embedded-software/stm32cubef1.html)
> Cách thêm vào Keil: Chỉ thêm file `.c/.h` cần thiết từng bước. Sau khi thêm, vào `Options for Target → C/C++` thêm include path trỏ đến thư mục `Inc`.

---

### A0 — HAL Core (Bắt buộc, thêm trước tất cả)

| File | Mục đích |
|------|---------|
| `stm32f1xx_hal.c/.h` | HAL_Init(), HAL_Delay(), tick |
| `stm32f1xx_hal_rcc.c/.h` | Cấu hình clock HSI/HSE/PLL |
| `stm32f1xx_hal_gpio.c/.h` | GPIO input/output, alternate function |
| `stm32f1xx_hal_cortex.c/.h` | NVIC priority, SysTick |

---

### A1 — I2C HAL + MPU6050 (Đang làm — viết lại từ polling sang HAL)

**File cần thêm:** `stm32f1xx_hal_i2c.c/.h`

**Ba mức độ sử dụng — học theo thứ tự:**

```c
// Mức 1 — Blocking (đang làm, CPU chờ)
HAL_I2C_Master_Receive(&hi2c1, MPU_ADDR, buf, 14, HAL_MAX_DELAY);

// Mức 2 — Interrupt (xem A7, CPU không chờ)
HAL_I2C_Master_Receive_IT(&hi2c1, MPU_ADDR, buf, 14);
// → HAL_I2C_MasterRxCpltCallback() được gọi khi xong

// Mức 3 — DMA (xem A8, CPU không chờ, không tốn NVIC time)
HAL_I2C_Master_Receive_DMA(&hi2c1, MPU_ADDR, buf, 14);
// → HAL_I2C_MasterRxCpltCallback() được gọi khi xong (cùng callback)
```

**Đường đi dữ liệu — hiểu cái này thì A7, A8 dễ hơn nhiều:**
```
[Blocking]   CPU ghi CR1→START → chờ SR1.SB → ghi DR → chờ BTF → đọc DR → lặp 14 lần
[Interrupt]  CPU khởi động → ra đi → phần cứng I2C tự chạy → NVIC gọi ISR → HAL gọi callback
[DMA]        CPU khởi động → ra đi → DMA đọc DR → ghi RAM → ngắt TC một lần duy nhất
```

**Lỗi Errata F103 hay gặp:** Nếu I2C bus bị treo (SDA stuck LOW):
```c
HAL_I2C_DeInit(&hi2c1);
HAL_I2C_Init(&hi2c1); // reset peripheral — workaround chính thức
```
Tài liệu: [Errata ES096](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) mục "I2C limitations"

---

### A2 — PWM Motor HAL (Đã hoạt động — tổng kết)

**File cần thêm:** `stm32f1xx_hal_tim.c/.h`, `stm32f1xx_hal_tim_ex.c/.h`

```c
// Thay đổi duty cycle realtime — không có overhead so với viết thẳng register
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_value);
// Compile thành: TIM3->CCR1 = duty_value; — 1 lệnh assembly duy nhất
```

Đã validate: 20/50/70/90% duty quay được. Bước tiếp theo: A3.

---

### A3 — Test tích hợp IMU + Motor, không PID ⬅ Bước tiếp theo quan trọng nhất

> **Tại sao phải có bước này trước PID:** Nếu nhảy thẳng vào PID mà motor quay sai chiều, bạn sẽ không biết bug đến từ PID logic hay từ motor wiring hay từ mixer. Bước này isolate vấn đề.

**Mục tiêu:** Nghiêng board bằng tay → motor phản ứng đúng chiều. Chưa cần chính xác, chưa cần ổn định — **chỉ cần đúng chiều.**

```c
// Code test tối thiểu — không có PID, không có filter phức tạp
#define BASE_THROTTLE 700  // PWM đủ để motor quay nhẹ, không đủ để bay

void test_integration_loop(void) {
    float pitch = get_complementary_pitch(); // từ code IMU đã có

    // Hệ số 5 — tùy chỉnh, bắt đầu nhỏ
    int correction = (int)(pitch * 5.0f);

    // Motor bố trí: FRONT = mũi drone, REAR = đuôi
    // Khi pitch dương (mũi cúi): tăng motor trước, giảm motor sau
    int m_front = BASE_THROTTLE - correction;
    int m_rear  = BASE_THROTTLE + correction;

    // Clamp an toàn
    m_front = CLAMP(m_front, 500, 1800);
    m_rear  = CLAMP(m_rear,  500, 1800);

    set_motor(MOTOR_FRONT, m_front);
    set_motor(MOTOR_REAR,  m_rear);

    // Log để verify — gửi qua HC-05 hoặc UART-USB
    printf("P=%.2f | mF=%d mR=%d\r\n", pitch, m_front, m_rear);
}
```

**Cách test an toàn — bập bênh (Prop Test Rig):**
```
Dùng dây buộc hoặc que kẹp drone ở điểm giữa frame
→ Drone chỉ được nghiêng theo 1 trục (pitch hoặc roll)
→ Nghiêng tay → drone tự bù lại → thả tay → về thăng bằng
→ Không cần cánh quạt lúc đầu — chỉ cần log để verify chiều
→ Khi log đúng chiều mới lắp cánh quạt và test lực thật
```

**Checklist trước khi qua A4:**
- [ ] Log cho thấy m_front tăng khi mũi cúi xuống (pitch > 0)
- [ ] Log cho thấy m_rear giảm đồng thời
- [ ] Motor thật phản ứng đúng với log (chiều quay, tốc độ tương đối)
- [ ] Không có motor nào bị kẹt hoặc bị cấp quá nhiều điện

---

### A4 — Cascade PID: Rate Loop (Inner loop — làm trước)

> **Đọc C3 trước** để hiểu tại sao Rate PID phải tune trước Attitude PID.

**Chỉ thêm P term trước — I=0, D=0:**

```c
typedef struct {
    float kP, kI, kD;
    float integral;
    float prev_error;
} PID_t;

float pid_compute(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kP * error + pid->kI * pid->integral + pid->kD * derivative;
}

// Rate PID — đọc gyro RAW, không qua complementary filter
// setpoint = 0 nghĩa là "muốn đứng yên"
PID_t rate_pitch = {.kP = 0.5f, .kI = 0.0f, .kD = 0.0f};

float gyro_y = get_raw_gyro_y(); // deg/s, không filter
float rate_output = pid_compute(&rate_pitch, 0, gyro_y, dt);

int m_front = BASE_THROTTLE - (int)rate_output;
int m_rear  = BASE_THROTTLE + (int)rate_output;
```

**Dấu hiệu Kp đúng:**
- Quá nhỏ: drone không chống lại lực nghiêng tay
- Quá lớn: drone rung (oscillation) khi thả tay
- Vừa: cầm tay nghiêng rồi thả → drone cản lại, về gần thẳng bằng

---

### A5 — Cascade PID: Attitude Loop (Outer loop — làm sau A4)

```c
// Outer loop (50Hz) → tạo rate setpoint cho inner loop
PID_t att_pitch = {.kP = 3.0f, .kI = 0.0f, .kD = 0.0f};

float current_angle = get_complementary_pitch(); // có filter
float rc_setpoint   = 0; // sau này đọc từ RC channel
float rate_setpoint = pid_compute(&att_pitch, rc_setpoint, current_angle, dt_outer);

// Inner loop (500Hz) dùng rate_setpoint thay vì 0
float rate_output = pid_compute(&rate_pitch, rate_setpoint, gyro_y, dt_inner);
```

---

### A6 — Bluetooth Telemetry HC-05 (Debug channel — không phải control)

**File cần thêm:** `stm32f1xx_hal_uart.c/.h`

**Đang hoạt động:** Hercules (PC) + Serial Bluetooth Monitor (phone) nhận raw MPU6050 data.

**Nâng cấp khi có PID — thêm PID output vào log:**

```c
// Gửi 10Hz — không gửi trong mọi vòng PID 500Hz
static uint32_t last_bt_send = 0;
if (HAL_GetTick() - last_bt_send >= 100) {
    // Format CSV → Arduino Serial Plotter / Python matplotlib đọc được ngay
    sprintf(buf, "%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\r\n",
        roll, pitch, yaw,
        rate_output, att_output,
        m1_pwm, m2_pwm, m3_pwm, m4_pwm); // thêm m4 sau
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 10);
    last_bt_send = HAL_GetTick();
}
```

**Python log đang có — bổ sung thêm cột PID:**
```python
# Thêm header vào parser hiện có
headers = ['roll','pitch','yaw','rate_out','att_out','m1','m2','m3','m4']
```

---

### A7 — I2C Interrupt (HAL_IT) — Nâng cấp từ A1

> **Khi nào cần:** Khi PID loop ở A4/A5 đã ổn định nhưng bạn thấy vòng lặp bị block tại `HAL_I2C_Master_Receive()` làm jitter timing.

**Không cần thêm file mới** — `stm32f1xx_hal_i2c.c` đã có sẵn HAL_IT functions.

```c
// Trong setup: enable I2C global interrupt trong NVIC
HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

// Trong IRQ handler file (stm32f1xx_it.c):
void I2C1_EV_IRQHandler(void) { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void) { HAL_I2C_ER_IRQHandler(&hi2c1); }

// Thay blocking call bằng:
HAL_I2C_Master_Receive_IT(&hi2c1, MPU_ADDR, imu_buf, 14);

// HAL tự gọi callback này khi đủ 14 byte:
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    data_ready_flag = 1; // PID loop đọc flag này
}
```

---

### A8 — I2C DMA — Nâng cấp từ A7

> **Khi nào cần:** Khi chạy 500Hz và interrupt overhead (~2µs mỗi byte × 14 byte = ~28µs) bắt đầu ảnh hưởng timing. Thực tế với F103 72MHz, thường không cần DMA cho 500Hz — nhưng đây là kiến thức nền cho ESP32-S3 GDMA.

**File cần thêm thêm:** `stm32f1xx_hal_dma.c/.h`

```c
// Thêm vào HAL_I2C_MspInit():
hdma_i2c1_rx.Instance                 = DMA1_Channel7; // F103: I2C1_RX = Ch7
hdma_i2c1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
hdma_i2c1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;  // I2C_DR cố định
hdma_i2c1_rx.Init.MemInc              = DMA_MINC_ENABLE;   // RAM tăng dần
hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
hdma_i2c1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
hdma_i2c1_rx.Init.Mode                = DMA_NORMAL;
hdma_i2c1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
HAL_DMA_Init(&hdma_i2c1_rx);
__HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);

// Trong NVIC:
HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
void DMA1_Channel7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c1_rx); }

// Thay IT call bằng DMA call — callback GIỐNG HỆT A7:
HAL_I2C_Master_Receive_DMA(&hi2c1, MPU_ADDR, imu_buf, 14);
// → HAL_I2C_MasterRxCpltCallback() vẫn được gọi — không đổi PID code
```

**Điểm mấu chốt của DMA:** Callback giống hệt interrupt — code PID không đổi. Chỉ đổi 1 dòng `_IT` thành `_DMA` và thêm DMA init.

---

## Track B — ESP32-S3 (Platform drone thật)

> **Nguyên tắc:** Sau khi hiểu ở Track A, Track B chỉ là đổi tên API. Không học lại concept.

---

### B0 — drn_main_board_choose — Abstraction Layer

File này có 2 implement riêng biệt: một cho STM32F1, một cho ESP32-S3. Đây là điểm thuận lợi lớn — khi STM32F1 đã chạy tốt, porting sang ESP32 chủ yếu là implement lại các hàm trong file ESP32 version.

**Cần kiểm tra trước khi port:**
```
□ Các hàm có cùng signature không? (return type, tham số)
□ Timing assumption có khác không?
  → HAL_Delay(1) = 1ms chính xác trên STM32
  → vTaskDelay(1) = 1 tick (mặc định 1ms) trên ESP32 — nhưng có thể bị trễ nếu scheduler bận
□ Kiểu dữ liệu có match không?
  → STM32 HAL dùng uint8_t/uint16_t
  → ESP-IDF dùng uint8_t/uint16_t — thường giống, nhưng kiểm tra kỹ float precision
```

---

### B1 — MPU6050 Polling → Nâng lên Interrupt (Port từ A1)

**Hiện tại:** Polling đang hoạt động. Giữ nguyên cho đến khi A7 (interrupt) xong trên STM32F1, rồi port sang.

```c
// ESP-IDF equivalent của HAL_I2C_Master_Receive_IT:
// Dùng new I2C driver (không dùng i2c_driver_install cũ)
i2c_master_transmit_receive(mpu_handle, &reg_addr, 1, buf, 14, -1);

// Khi nâng lên interrupt: dùng GPIO interrupt từ chân INT của MPU6050
// (xem B7 — cùng pattern với A7 nhưng dùng gpio_isr_handler_add)
```

---

### B2 — PWM Motor LEDC (Port từ A2)

**Không cần học lại concept** — chỉ đổi API:

```c
// STM32F1 (A2):
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);

// ESP32-S3 (B2) — equivalent:
ledcAttach(MOTOR1_PIN, 50, 14); // 50Hz, 14-bit
ledcWrite(MOTOR1_PIN, duty);    // duty: 819(1000µs) → 1638(2000µs)

// Thay đổi realtime:
ledcWrite(MOTOR1_PIN, new_duty); // Gọi mỗi vòng PID — không có overhead
```

**Nâng lên MCPWM sau** khi LEDC hoạt động — chỉ cần khi cần sync chính xác 4 motor hoặc dùng ESC 3D.

---

### B3 — PWM nhận RC — ISR (Đã có, cần fix 3 lỗi)

Code ISR đã hoạt động. **Fix 3 lỗi này trước khi tích hợp vào PID:**

```c
// Fix 1: Đọc atomic — tránh torn read khi ISR ghi đồng thời
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
// Trong loop():
portENTER_CRITICAL(&mux);
unsigned long snapshot = pwmValue;
portEXIT_CRITICAL(&mux);

// Fix 2: Failsafe mất tín hiệu
static unsigned long lastPulseTime = 0;
// Trong ISR: lastPulseTime = millis();
// Trong loop():
if (millis() - lastPulseTime > 100) snapshot = 1000; // disarm

// Fix 3: Sanity check
if (snapshot < 800 || snapshot > 2200) snapshot = 1500; // loại giá trị rác
```

---

### B4 — Test tích hợp IMU + Motor, không PID (Port từ A3)

Hoàn toàn giống A3, chỉ đổi API set_motor:

```c
// Thay HAL TIM set_compare bằng ledcWrite
ledcWrite(MOTOR_FRONT_PIN, BASE_THROTTLE - correction);
ledcWrite(MOTOR_REAR_PIN,  BASE_THROTTLE + correction);

// Log qua USB CDC — không cần HC-05
Serial.printf("P=%.2f | mF=%d mR=%d\r\n", pitch, m_front, m_rear);
```

**Tại sao dùng USB CDC thay HC-05 ở bước này:** ESP32-S3 Supermini có USB CDC built-in — cắm vào PC là có Serial Monitor ngay. Không cần setup thêm gì. Sau khi xác nhận xong, tắt debug output đi.

---

### B5 — Cascade PID — Port từ STM32F1

Code PID struct và `pid_compute()` từ A4/A5 copy nguyên sang — không đổi gì. Chỉ đổi phần đọc sensor và set motor.

**Kiến trúc FreeRTOS cho PID task:**

```c
// ISR — cực ngắn
void IRAM_ATTR mpu_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(imu_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

// PID task trên Core 1 — hành vi như bare-metal
void pid_task(void *arg) {
    while (1) {
        if (xSemaphoreTake(imu_sem, pdMS_TO_TICKS(5)) == pdTRUE) {
            read_mpu6050();
            complementary_filter();
            rate_output  = pid_compute(&rate_pid,  rate_setpoint,  gyro_y,  dt);
            att_output   = pid_compute(&att_pid,   att_setpoint,   pitch,   dt_outer);
            motor_mixer(att_output);
        } else {
            motor_mixer(0); // failsafe: timeout 5ms = mất data IMU
        }
    }
}
```

**Lý do không đáng sợ:** 1 task + 1 ISR + 1 semaphore = không có mutex, không có deadlock, không có priority inversion. Đơn giản như bare-metal, chỉ thêm lớp scheduler nhẹ.

---

### B6 — Telemetry USB CDC (Thay thế HC-05 trong giai đoạn debug)

```c
// Không cần setup gì thêm trên ESP32-S3 Supermini — USB CDC built-in
// Gửi 10Hz trong task riêng hoặc trong loop() nếu single core
Serial.printf("%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d\r\n",
    roll, pitch, rate_out, att_out, m1, m2, m3, m4);
```

Dùng Arduino Serial Plotter hoặc script Python hiện có (chỉ đổi COM port). Sau khi PID ổn định, tắt hoàn toàn — không cần HC-05 cho ESP32.

---

### B7 — I2C Interrupt + GDMA (Port từ A7/A8)

> Làm sau khi B5 (PID) đã ổn định. Concept giống hệt A7/A8 — chỉ đổi tên API.

```c
// ESP-IDF GDMA equivalent của DMA HAL:
// Dùng i2c_master_transmit_receive() async với callback
// hoặc gpio interrupt từ chân INT MPU6050 (cách đơn giản hơn)

// Pattern GPIO interrupt — giống EXTI trên STM32:
gpio_set_intr_type(MPU_INT_PIN, GPIO_INTR_POSEDGE);
gpio_isr_handler_add(MPU_INT_PIN, mpu_isr, NULL);
// → mpu_isr gọi xSemaphoreGiveFromISR → pid_task thức dậy
```

---

### B8 — VL535L1X ToF Sensor (Độ cao laser)

**Giao tiếp:** I2C, địa chỉ `0x29`, dùng chung bus với MPU6050.

**Đọc 20Hz** (không cần nhanh hơn — sensor không nhanh hơn được):

```c
static uint32_t last_tof = 0;
if (millis() - last_tof > 50) {
    altitude_mm = vl53l1x_read();
    last_tof = millis();
}
// altitude_mm đưa vào Altitude PID (outer-outer loop)
```

**Giới hạn thực tế:** Hoạt động tốt indoor trên sàn gỗ/thảm. Không chính xác trên gương hoặc ngoài sáng mạnh.

---

### B9 — PMW3901 Optical Flow (Giữ vị trí ngang)

**Giao tiếp:** SPI — không dùng I2C vì cần tốc độ cao hơn.

```c
// Đọc deltaX, deltaY mỗi 20ms (50Hz)
// Tích hợp vào cascade PID:
// deltaX/Y → scale theo altitude_mm → velocity (cm/s) → Position PID → roll/pitch setpoint
```

**Đây là tầng ngoài cùng của cascade PID 3 tầng:**
```
Position PID (20Hz)  → roll/pitch setpoint
  Attitude PID (50Hz) → rate setpoint
    Rate PID (500Hz)   → motor output
```

---

## Track C — Thuật toán (Không phụ thuộc chip)

---

### C1 — Complementary Filter (Đã hiểu — tổng kết)

```c
// alpha = 0.98: tin gyro 98%, accelerometer 2%
// Tốt cho indoor hover. Không cần Kalman.
angle = alpha * (angle + gyro_rate * dt) + (1.0f - alpha) * accel_angle;
```

Kalman tốt hơn khi: outdoor nhiều gió, cần độ chính xác góc < 0.5°. Với drone indoor, complementary filter là đủ và dễ tune hơn.

---

### C2 — Motor Mixer (Mô phỏng xong — cần test thật ở A3/B4)

**Layout motor chuẩn X-frame:**
```
    M1(↺) --- M2(↻)
      |    ✕    |
    M4(↻) --- M3(↺)

↺ = ngược chiều kim đồng hồ
↻ = theo chiều kim đồng hồ
```

**Mixer công thức:**
```c
// throttle: base power | roll/pitch/yaw: PID outputs
motor[1] = throttle + pitch + roll - yaw;  // Front-Left  ↺
motor[2] = throttle + pitch - roll + yaw;  // Front-Right ↻
motor[3] = throttle - pitch - roll - yaw;  // Rear-Right  ↺
motor[4] = throttle - pitch + roll + yaw;  // Rear-Left   ↻

// Clamp sau khi tính
for (int i = 0; i < 4; i++)
    motor[i] = CLAMP(motor[i], MIN_THROTTLE, MAX_THROTTLE);
```

**Cách verify mixer trước khi test motor thật:**
1. Set pitch = +10, roll = 0, yaw = 0, throttle = 1000
2. Kiểm tra: M1 và M2 (front) tăng, M3 và M4 (rear) giảm — đúng vì cần nâng mũi lên
3. Log đang có đã làm đúng bước này — bước tiếp là test với motor thật (A3)

---

### C3 — Cascade PID: Thứ tự và giá trị khởi điểm

**Thứ tự tune — sai thứ tự = không tune được:**

```
Bước 1: Rate PID (chỉ P, I=0, D=0)
  → Cầm drone bằng tay, nghiêng → drone kháng lại
  → Nếu rung: giảm P. Nếu không phản ứng: tăng P.
  → Khi P đúng: thêm D nhỏ để giảm overshoot
  → I thêm cuối cùng để bù drift

Bước 2: Attitude PID (sau khi Rate P đã ổn)
  → Chỉ cần P, thường không cần I và D ở outer loop

Bước 3: Altitude PID (sau khi bay được)
  → Dùng VL535L1X làm feedback

Bước 4: Position PID (sau khi Altitude ổn)
  → Dùng PMW3901 làm feedback
```

**Giá trị khởi điểm cho motor 8.5×20mm (Coreless, nhỏ, phản hồi nhanh):**

| Loop | Kp | Ki | Kd |
|------|----|----|----|
| Rate | 0.5 | 0.0 | 0.01 |
| Attitude | 3.0 | 0.0 | 0.0 |
| Altitude | 1.0 | 0.1 | 0.5 |

Đây là giá trị bắt đầu — luôn bay lần đầu với throttle thấp (không rời đất) và tăng dần.

---

### C4 — Quy trình test tích hợp từng bước

> **Đọc mục này trước khi bắt đầu A3 hoặc B4.** Đây là quy trình để biết chính xác đang test gì ở mỗi bước.

**Giai đoạn 0 — Đã xong ✅**
- MPU6050 đọc data, in ra terminal
- Motor quay được ở mức cố định
- Motor-mixer mô phỏng đúng chiều trên log terminal

**Giai đoạn 1 — IMU + Motor, không PID (A3/B4)**
- Dụng cụ: Drone buộc vào cọc hoặc bập bênh, chưa cần cánh quạt
- Test: Nghiêng tay → log motor đúng chiều → lắp cánh quạt → test lực thật
- Confirm: Motor tăng/giảm đúng chiều theo góc nghiêng

**Giai đoạn 2 — Rate PID (A4/B5)**
- Dụng cụ: Cùng bập bênh, có cánh quạt, throttle thấp
- Test: Thả tay → drone tự chống lại lực nghiêng
- Observe qua log: `rate_error` dao động xung quanh 0, không phân kỳ
- Confirm: Không rung, không drift

**Giai đoạn 3 — Attitude PID + bay thấp**
- Dụng cụ: Không gian rộng, lưới an toàn nếu có
- Test: Throttle vừa đủ rời đất ~10cm, thả tay → giữ góc thẳng
- Observe qua telemetry: `att_error` về 0 sau ~0.5 giây
- Confirm: Không bị drift về một phía

**Giai đoạn 4 — Altitude hold (sau khi có VL535L1X)**
- Test: Tăng throttle từ từ → drone tự giữ độ cao khi thả tay

**Giai đoạn 5 — Position hold (sau khi có PMW3901)**
- Test: Drone hover → đẩy nhẹ → tự về vị trí cũ

**Câu hỏi hay gặp ở từng giai đoạn:**
- "Motor quay đúng chiều nhưng drone vẫn lật" → Kiểm tra chiều quay vật lý của từng motor (prop thuận/nghịch)
- "Log đúng nhưng motor không phản ứng" → Kiểm tra MIN_THROTTLE có đủ để motor quay không
- "Rate PID rung ngay khi bật" → Kp quá lớn, giảm 50% và thử lại
- "Attitude PID drift chậm" → Cần thêm Ki nhỏ vào Rate loop

---

## Tài liệu tham khảo

| Tài liệu | Link | Dùng cho |
|---|---|---|
| STM32CubeF1 HAL v1.8.0 | [GitHub](https://github.com/STMicroelectronics/STM32CubeF1) | A0–A8 |
| STM32CubeF1 trên ST.com | [ST website](https://www.st.com/en/embedded-software/stm32cubef1.html) | A0–A8 |
| STM32F103 Reference Manual RM0008 | [ST PDF](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) | A7, A8 — khi cần hiểu thanh ghi sâu |
| STM32F103 Errata ES096 | [ST PDF](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) | A1 — I2C bug workaround |
| ESP32-S3 Technical Reference Manual | [Espressif PDF](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf) | B7, B9 — GDMA/SPI sâu |
| ESP-IDF I2C Driver (new API) | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html) | B1, B7 |
| ESP-IDF GPIO & Interrupt | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) | B3, B7 |
| ESP-IDF FreeRTOS | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_idf.html) | B5 — chỉ đọc Task + Semaphore |
| MPU6050 Register Map | [InvenSense PDF](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) | A1, B1 |
| VL53L1X Datasheet | [ST](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) | B8 |

---

*Version 3.0 — Cập nhật: thêm mục lục anchor, C4 quy trình test tích hợp, B6 USB CDC telemetry, sửa trạng thái polling ESP32, sửa 2 implement riêng biệt drn_main_board_choose, bổ sung thứ tự tune PID chi tiết*