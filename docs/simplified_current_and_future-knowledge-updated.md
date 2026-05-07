# Lộ trình kiến thức kỹ thuật — Quadcopter Mini
## Phiên bản 4.0
### Bốn track: STM32F1 (Keil) + ESP32-S3 (Arduino IDE) + ESP32-S3 (ESP-IDF) + Thuật toán

> [↑ Mục lục](#mục-lục)


---

## Mục lục

> [→ Trạng thái tổng quan](#trạng-thái-hiện-tại) | [→ Tài liệu tham khảo](#tài-liệu-tham-khảo)

| ***Track A — STM32F1 + Keil*** | | ***Track B — ESP32 Arduino IDE*** | | ***Track C — ESP32 ESP-IDF*** | | ***Track D — Thuật toán*** | |
|---|---|---|---|---|---|---|---|
| [A0](#a0--hal-core-bắt-buộc-thêm-trước-tất-cả) *HAL Core* | ✅ | [B0](#b0--drn_main_board_choose--abstraction-layer) *drn_main_board_choose* | ✅ | [C0](#c0--drn_main_board_choose--esp-idf-version) *drn_main_board_choose* | ✅ ESP32 only | [D1](#d1--complementary-filter-đã-hiểu--tổng-kết) *Complementary Filter* | ✅ |
| [A1](#a1--i2c-hal--mpu6050-đang-làm--viết-lại-từ-polling-sang-hal) *I2C HAL + MPU6050* | ⚠️ | [B1](#b1--mpu6050--arduino-ide-port-từ-a1) *MPU6050* | ❌ chưa có trong .ino | [C1](#c1--mpu6050--esp-idf-new-i2c-driver) *MPU6050 200Hz* | ✅ | [D2](#d2--motor-mixer-mô-phỏng-xong--cần-test-thật-ở-a3b4) *Motor Mixer* | ⚠️ log only |
| [A2](#a2--pwm-motor-hal-đã-hoạt-động--tổng-kết) *PWM Motor TIM3* | ✅ Stage 10.5 | [B2](#b2--pwm-motor--ledcwrite-port-từ-a2) *PWM Motor ledcWrite* | ❌ | [C2](#c2--pwm-motor--ledc-esp-idf) *PWM Motor LEDC* | ❌ | [D3](#d3--cascade-pid-thứ-tự-và-giá-trị-khởi-điểm) *Cascade PID — tune* | ❌ |
| [A3](#a3--test-tích-hợp-imu--motor-không-pid--bước-tiếp-theo-quan-trọng-nhất) ***Test IMU+Motor*** | ❌ ← tiếp theo | [B3](#b3--pwm-nhận-rc--isr-cần-fix-3-lỗi) *PWM RC ISR* | ✅ cần fix | [C3](#c3--pwm-nhận-rc--isr-esp-idf) *PWM RC ISR* | ✅ | [D4](#d4--quy-trình-test-tích-hợp-từng-bước) ***Quy trình test*** | ❌ đọc trước A3 |
| [A4](#a4--cascade-pid-rate-loop-inner-loop--làm-trước) *Rate PID* | ⚠️ struct có | [B4](#b4--test-tích-hợp-imu--motor-không-pid) *Test IMU+Motor* | ❌ | [C4](#c4--test-tích-hợp-imu--motor-không-pid) *Test IMU+Motor* | ❌ | | |
| [A5](#a5--cascade-pid-attitude-loop-outer-loop--làm-sau-a4) *Attitude PID* | ❌ | [B5](#b5--cascade-pid--tune-trên-arduino-ide) *Cascade PID* | ❌ | [C5](#c5--cascade-pid--freertos-task) *PID FreeRTOS* | ❌ M2 | | |
| [A6](#a6--bluetooth-telemetry-hc-05-debug-channel--không-phải-control) *BT Telemetry HC-05* | ⚠️ | [B6](#b6--telemetry-usb-cdc) *Telemetry USB CDC* | ❌ | [C6](#c6--telemetry-usb-cdc--esp-idf) *Telemetry USB CDC* | ❌ | | |
| [A7](#a7--i2c-interrupt-hal_it--nâng-cấp-từ-a1) *I2C Interrupt* | ❌ | [B7](#b7--nâng-cấp-rc--crsf-tbs-tango-2-làm-khi-có-hardware) *CRSF / TBS Tango 2* | ❌ | [C7](#c7--nâng-cấp-rc--crsf-esp-idf) *CRSF ESP-IDF* | ❌ | | |
| [A8](#a8--i2c-dma--nâng-cấp-từ-a7) *I2C DMA* | ❌ | [B8](#b8--vl535l1x--độ-cao-laser) *VL535L1X ToF* | ❌ | [C8](#c8--i2c-interrupt--gdma-nâng-cấp-từ-c1) *I2C + GDMA* | ❌ config sẵn | | |
| | | [B9](#b9--pmw3901--optical-flow-giữ-vị-trí-ngang) *PMW3901 Flow* | ❌ | [C9](#c9--vl535l1x--esp-idf) *VL535L1X ESP-IDF* | ❌ | | |
| | | | | [C10](#c10--pmw3901--spi-esp-idf) *PMW3901 SPI* | ❌ | | |

---

## Trạng thái hiện tại

> Cập nhật theo repo thực tế — `chinart123-esp32-quadcopter` & `chinart123-stm32_quadcopter`

| Module | STM32F1 (Keil) | ESP32-S3 (ESP-IDF) | Ghi chú từ repo |
|--------|---------------|-------------------|----------------|
| **MPU6050 đọc data** | ✅ Polling | ✅ Polling 200Hz (5ms) | ESP32 có 2 driver: `drn_raw_mpu6050` (raw) + `drn_mpu6050` (filter) |
| **Complementary filter** | ⚠️ Hiểu, chưa tích hợp | ✅ Trong `drn_mpu6050` | Có simulator HTML (`imu_simulator_complementary_filter_v1/v2.html`) |
| **Calibration** | ❌ | ✅ `DRN_RawMPU6050_Calibrate()` — 1000 mẫu | Tự trừ offset ax/ay/az/gx/gy/gz |
| **PWM motor** | ✅ TIM3 4kHz, 4 kênh | ❌ Chưa làm | **Stage 10.5 Final Release** — đã chuyển PA6→**PA7** sau sự cố cháy Gate |
| **PWM nhận RC** | ❌ | ✅ ISR (`IRAM_ATTR`) | Arduino IDE có sẵn, ESP-IDF cần port. Fix 3 lỗi trước dùng (xem B3/C3) |
| **PID struct** | ⚠️ `PID_Roll/Pitch/Yaw` có, chưa kết hợp IMU | ❌ | `pid_control.h/.c` trong STM32 repo |
| **Motor mixer** | ⚠️ Mô phỏng log terminal | ❌ | Chưa test motor thật |
| **Bluetooth telemetry** | ⚠️ HC-05 → Hercules + Python + App | ❌ | Dùng USB CDC thay thế cho ESP32 |
| **Milestone** | Stage 10.5 ✅ | M1 ✅ → M2 ❌ | M2 target: `xTaskCreatePinnedToCore` dual-core |
| **I2C Interrupt** | ❌ | ❌ | STM32: sau A1 polling ổn. ESP32: GDMA đã config trong sdkconfig |
| **I2C GDMA** | — | ❌ (config sẵn) | `CONFIG_GDMA_CTRL_FUNC_IN_IRAM=y` — chỉ cần enable trong driver |
| **VL535L1X / PMW3901** | ❌ | ❌ | Sau khi PID bay được |

> **Nguyên tắc đọc file này:**
> - **Track A** — học sâu bare-metal → HAL. Hiện tại STM32 repo đang dùng bare-metal register (`TIM3->CCR`, `RCC->APB1ENR`) — chưa dùng HAL. Track A là lộ trình chuyển sang HAL.
> - **Track B** — Arduino IDE, code ngắn, thư viện sẵn, logic giống Track A.
> - **Track C** — ESP-IDF, không có Arduino wrapper. M2 milestone = dual core FreeRTOS.
> - **Track D** — thuật toán, không phụ thuộc chip. Hiểu 1 lần, dùng cho cả 3 track.
> - **PID tune trên cả 3 môi trường:** KeilC (A) → Arduino IDE (B) → ESP-IDF (C). Cả 3 cho kết quả giống nhau = hiểu thật sự.
> - **Đọc D4 trước A3, B4, C4.**

---

## Track A — STM32F1 + Keil (Bare-metal → HAL)

> **Thực tế repo hiện tại:** Code STM32 đang dùng bare-metal register trực tiếp (`TIM3->CCR`, `RCC->APB1ENR`, `TIM3->PSC`). Track A là lộ trình chuyển dần sang HAL để code dễ maintain và dễ port hơn.
>
> Repo gốc HAL: [STM32CubeF1 v1.8.0](https://github.com/STMicroelectronics/STM32CubeF1) | [ST website](https://www.st.com/en/embedded-software/stm32cubef1.html)
> Cách thêm vào Keil: **Options for Target → C/C++ → Include Paths** trỏ đến thư mục `Inc` của HAL Driver.

---

### A0 — HAL Core (Bắt buộc, thêm trước tất cả)

> [↑ Mục lục](#mục-lục)


| File | Mục đích |
|------|---------|
| `stm32f1xx_hal.c/.h` | HAL_Init(), HAL_Delay(), tick |
| `stm32f1xx_hal_rcc.c/.h` | Cấu hình clock HSI/HSE/PLL |
| `stm32f1xx_hal_gpio.c/.h` | GPIO input/output, alternate function |
| `stm32f1xx_hal_cortex.c/.h` | NVIC priority, SysTick |

---

### A1 — I2C HAL + MPU6050 (Đang làm — viết lại từ polling sang HAL)

> [↑ Mục lục](#mục-lục)


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

### A2 — PWM Motor HAL (✅ Stage 10.5 Final Release — bare-metal TIM3)

> [↑ Mục lục](#mục-lục)


> **Trạng thái thực tế:** Motor đã quay được bằng bare-metal register. `drn_motor_pwm.c` dùng TIM3 trực tiếp. Track A mục này là lộ trình viết lại bằng `HAL_TIM_PWM`.

**⚠️ Lưu ý phần cứng quan trọng từ Stage 10 bug log:** Chân PA6 (TIM3_CH1) đã bị cháy Gate do sự cố. Tất cả code PWM đã chuyển sang **PA7 (TIM3_CH2)**. Không dùng lại PA6.

**Bare-metal hiện tại (đang chạy):**
```c
// drn_motor_pwm.c — TIM3 4kHz, 4 kênh
TIM3->PSC = 17;   // 72MHz / (17+1) = 4MHz
TIM3->ARR = 999;  // 4MHz / 1000 = 4kHz
TIM3->CCR2 = ccr_value; // PA7 = CH2 (sau khi PA6 bị cháy)
TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
```

**HAL equivalent (mục tiêu Track A):**
```c
// File cần thêm: stm32f1xx_hal_tim.c/.h + stm32f1xx_hal_tim_ex.c/.h
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_value); // PA7 = CH2
// Compile thành cùng 1 lệnh: TIM3->CCR2 = duty_value
```

---

### A3 — Test tích hợp IMU + Motor, không PID ⬅ Bước tiếp theo quan trọng nhất

> [↑ Mục lục](#mục-lục)


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

> [↑ Mục lục](#mục-lục)


> **Trạng thái repo:** `pid_control.h/.c` đã có `PID_Controller_t` struct với `PID_Roll`, `PID_Pitch`, `PID_Yaw`. Hàm `PID_Compute()` đã có. **Chưa kết hợp với IMU output** — đây là bước A3 → A4.

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

> [↑ Mục lục](#mục-lục)


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

> [↑ Mục lục](#mục-lục)


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

> [↑ Mục lục](#mục-lục)


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

> [↑ Mục lục](#mục-lục)


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

## Track B — ESP32-S3 (Arduino IDE)

> **Cài đặt một lần:** Vào **File → Preferences**, thêm URL:
> `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
> Sau đó **Tools → Board → Boards Manager** → tìm `esp32 by Espressif` → cài.
> Chọn board: **Tools → Board → ESP32 Arduino → ESP32S3 Dev Module**

> **Triết lý Track B:** Arduino IDE che đi phần lớn cơ chế bên dưới — `Wire.begin()` thay cho toàn bộ I2C init của ESP-IDF, `ledcWrite()` thay cho LEDC/MCPWM config. Đây là lý do code ngắn hơn nhiều so với ESP-IDF, nhưng logic PID và thuật toán là y chang Track A.

---

### B0 — drn_main_board_choose — Abstraction Layer

> [↑ Mục lục](#mục-lục)


File này có 2 implement riêng biệt: STM32F1 và ESP32-S3. Khi port code từ A sang B, các hàm trong abstraction layer là chỗ duy nhất cần thay đổi — PID logic và thuật toán giữ nguyên.

**Checklist khi port:**
```
□ Signature hàm có giống nhau không? (tên, tham số, kiểu trả về)
□ HAL_Delay(1) = 1ms chính xác | delay(1) trên Arduino = ~1ms nhưng có thể bị interrupt chen
□ Kiểu float: cả hai dùng 32-bit IEEE 754 — kết quả tính toán PID sẽ giống nhau
□ Chiều pin có được map đúng không trong version ESP32?
```

---

### B1 — MPU6050 — Arduino IDE (Port từ A1)

> [↑ Mục lục](#mục-lục)


**Thư viện cần cài:** Vào **Tools → Manage Libraries** → tìm `MPU6050 by Electronic Cats` → cài.

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float pitch = 0, roll = 0;
float alpha = 0.98;          // ← tune nếu cần: 0.95–0.99
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);          // SDA=GPIO8, SCL=GPIO9 trên Supermini
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 lỗi — kiểm tra dây!");
    while(1);
  }
  lastTime = millis();
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0f;
  lastTime = millis();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accel_pitch = atan2(ax/16384.0f, az/16384.0f) * 180.0f / PI;
  float accel_roll  = atan2(ay/16384.0f, az/16384.0f) * 180.0f / PI;

  // Complementary filter — giống hệt A1, chỉ đổi tên biến
  pitch = alpha * (pitch + (gy/131.0f) * dt) + (1-alpha) * accel_pitch;
  roll  = alpha * (roll  + (gx/131.0f) * dt) + (1-alpha) * accel_roll;

  Serial.printf("Pitch=%.2f Roll=%.2f\n", pitch, roll);
  delay(10); // 100Hz
}
```

**So sánh với A1 (HAL):** `Wire.begin(8,9)` thay cho toàn bộ `hi2c1` struct + `HAL_I2C_Init()`. Bên dưới Arduino vẫn gọi ESP-IDF I2C driver — chỉ là được wrapper lại.

**Nâng cấp lên Interrupt:** Khi A7 xong trên STM32F1, port bằng cách dùng chân INT của MPU6050:
```cpp
attachInterrupt(digitalPinToInterrupt(4), mpuISR, RISING); // GPIO4 = INT pin
```

---

### B2 — PWM Motor — ledcWrite (Port từ A2)

> [↑ Mục lục](#mục-lục)


```cpp
// Chân motor — sửa theo sơ đồ thực tế
#define M1 16
#define M2 17
#define M3 18
#define M4 19

// Chuyển microseconds → duty 14-bit ở 50Hz
// 50Hz = 20000µs chu kỳ, 14-bit = 16383 max
void setMotor(int pin, int us) {
  us = constrain(us, 1000, 2000);
  ledcWrite(pin, map(us, 0, 20000, 0, 16383));
}

void setup() {
  ledcAttach(M1, 50, 14);   // 50Hz, 14-bit
  ledcAttach(M2, 50, 14);
  ledcAttach(M3, 50, 14);
  ledcAttach(M4, 50, 14);

  // Gửi 1000µs để khởi động — giữ 2 giây
  setMotor(M1, 1000); setMotor(M2, 1000);
  setMotor(M3, 1000); setMotor(M4, 1000);
  delay(2000);
}
```

**So sánh với A2 (HAL TIM):**
```
STM32F1:   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_value);
ESP32:     ledcWrite(M1, duty_value);
```
Compile thành cùng một thao tác ghi register — không có overhead khác nhau.

**MCPWM sau này:** Nâng cấp khi cần sync 4 motor chính xác hơn hoặc dùng ESC 3D. Với Coreless Motor + MOSFET A03400A trực tiếp, `ledcWrite` là đủ.

---

### B3 — PWM nhận RC — ISR (Cần fix 3 lỗi)

> [↑ Mục lục](#mục-lục)


> **Tương lai:** Khi có TBS Tango 2 Pro + CRSF receiver thật → thay toàn bộ bằng B7. Hiện tại dùng PWM ISR để test.

```cpp
#define RC_PIN 6  // ← sửa nếu khác

volatile unsigned long pwmValue    = 1500;
volatile unsigned long pulseStart  = 0;
volatile unsigned long lastPulse   = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Fix 1

void IRAM_ATTR rcISR() {
  if (digitalRead(RC_PIN) == HIGH) {
    pulseStart = micros();
  } else {
    unsigned long w = micros() - pulseStart; // Fix 3: unsigned → tự xử lý overflow
    if (w >= 800 && w <= 2200) {            // Fix 3: sanity check trong ISR
      portENTER_CRITICAL_ISR(&mux);
      pwmValue = w;
      lastPulse = millis();
      portEXIT_CRITICAL_ISR(&mux);
    }
  }
}

// Đọc an toàn từ loop()
unsigned long getRCValue() {
  portENTER_CRITICAL(&mux);          // Fix 1: atomic read
  unsigned long val = pwmValue;
  unsigned long age = millis() - lastPulse;
  portEXIT_CRITICAL(&mux);
  if (age > 100) return 1000;        // Fix 2: failsafe mất tín hiệu
  return val;
}

void setup() {
  pinMode(RC_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_PIN), rcISR, CHANGE);
}
```

---

### B4 — Test tích hợp IMU + Motor (không PID)

> [↑ Mục lục](#mục-lục)


> **Đọc C4 trước.** Tháo cánh quạt. Mục tiêu: nghiêng board → motor đúng chiều.

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float pitch = 0, roll = 0;
float alpha = 0.98;
unsigned long lastTime = 0;

#define BASE_THROTTLE 1200  // ← đủ quay nhẹ, chưa đủ bay
#define SCALE 3             // ← giảm nếu motor giật mạnh

void setMotor(int pin, int us) {
  ledcWrite(pin, map(constrain(us,1000,2000), 0, 20000, 0, 16383));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  mpu.initialize();
  ledcAttach(16, 50, 14); ledcAttach(17, 50, 14);
  ledcAttach(18, 50, 14); ledcAttach(19, 50, 14);
  setMotor(16,1000); setMotor(17,1000);
  setMotor(18,1000); setMotor(19,1000);
  delay(2000);
  lastTime = millis();
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0f;
  lastTime = millis();

  int16_t ax,ay,az,gx,gy,gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  pitch = alpha*(pitch+(gy/131.0f)*dt) + (1-alpha)*(atan2(ax/16384.0f,az/16384.0f)*180/PI);
  roll  = alpha*(roll +(gx/131.0f)*dt) + (1-alpha)*(atan2(ay/16384.0f,az/16384.0f)*180/PI);

  int pc = (int)(pitch * SCALE);
  int rc = (int)(roll  * SCALE);

  int mF = BASE_THROTTLE - pc;   // mũi cúi → tăng motor trước
  int mR = BASE_THROTTLE + pc;
  int mL = BASE_THROTTLE - rc;
  int mRi= BASE_THROTTLE + rc;

  setMotor(16, mF); setMotor(17, mR);
  setMotor(18, mL); setMotor(19, mRi);

  // Xem log — nếu chiều sai báo bạn đổi dấu
  Serial.printf("P=%.1f R=%.1f | mF=%d mR=%d mL=%d mRi=%d\n",
    pitch, roll, mF, mR, mL, mRi);
  delay(10);
}
```

---

### B5 — Cascade PID — Tune trên Arduino IDE

> [↑ Mục lục](#mục-lục)


> **Mục tiêu:** Tune KP/KI/KD trên Arduino IDE, so sánh kết quả với KeilC (A4/A5). Nếu giống nhau = hiểu đúng, không phải may mắn.

```cpp
// Struct PID — copy từ A4, không đổi gì
typedef struct {
  float kP, kI, kD;
  float integral, prev_error;
} PID_t;

float pid_compute(PID_t *p, float sp, float meas, float dt) {
  float err = sp - meas;
  p->integral = constrain(p->integral + err*dt, -200, 200);
  float d = (err - p->prev_error) / dt;
  p->prev_error = err;
  return p->kP*err + p->kI*p->integral + p->kD*d;
}

// ← Điền số sau khi tune trên KeilC, so sánh xem có cần chỉnh không
PID_t rate_pid = {0.5f, 0.0f, 0.01f, 0, 0};
PID_t att_pid  = {3.0f, 0.0f, 0.0f,  0, 0};

// Trong loop() — Rate PID dùng gyro raw (không filter):
float gyro_y_raw = gy / 131.0f;
float rate_out = pid_compute(&rate_pid, 0, gyro_y_raw, dt);

// Attitude PID dùng góc từ complementary filter:
float att_out = pid_compute(&att_pid, 0, pitch, dt_outer);
```

**Cách verify cross-platform:**
```
Bước 1: Tune KeilC đến khi drone giữ thăng bằng trên bập bênh
Bước 2: Copy nguyên Kp/Ki/Kd sang Arduino IDE
Bước 3: Bay thử — nếu phản ứng giống hệt → port thành công
Bước 4 (sau): Port tiếp sang ESP-IDF để hoàn chỉnh bộ 3
```

---

### B6 — Telemetry USB CDC

> [↑ Mục lục](#mục-lục)


```cpp
// ESP32-S3 Supermini có USB CDC built-in — không cần HC-05 hay module thêm
// Gửi 10Hz — đừng gửi trong mọi vòng PID 500Hz

static unsigned long lastLog = 0;
if (millis() - lastLog >= 100) {
  // Format CSV → Arduino Serial Plotter vẽ đồ thị realtime
  Serial.printf("%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d\n",
    pitch, roll, rate_out, att_out, m1, m2, m3, m4);
  lastLog = millis();
}
```

Mở **Tools → Serial Plotter** trong Arduino IDE để xem đồ thị realtime. Dùng script Python hiện có (chỉ đổi COM port) để ghi log file.

**Dấu hiệu cần xem trong log:**
- `rate_out` dao động nhỏ quanh 0 → Rate PID ổn định
- `pitch` không về 0 khi thả tay → cần tăng Kp Attitude
- Motor một bên liên tục cao hơn bên kia → kiểm tra lại motor mixer hoặc mounting IMU lệch

---

### B7 — Nâng cấp RC — CRSF / TBS Tango 2 (Làm khi có hardware)

> [↑ Mục lục](#mục-lục)


> **Làm bước này khi:** Đã xác nhận receiver đi kèm TBS Tango 2 Pro là loại nào (TBS Nano, Micro, hay ExpressLRS).

**CRSF khác PWM hoàn toàn — không dùng ISR đo pulse:**
```
PWM:   1 dây / 1 kênh, đo độ rộng xung (µs)
CRSF:  1 dây UART / 16 kênh, gói tin nhị phân 420000 baud, latency ~4ms
```

**Thư viện:** [CRSFforArduino](https://github.com/ZZ-Cat/CRSFforArduino) — hỗ trợ ESP32 Arduino IDE.

```cpp
#include <CRSFforArduino.hpp>

// CRSF kết nối UART1: RX=GPIO44, TX=GPIO43 (Supermini)
CRSFforArduino crsf(&Serial1);

void setup() {
  crsf.begin();
}

void loop() {
  crsf.update(); // gọi thường xuyên nhất có thể

  // Đọc 16 kênh — thay hoàn toàn getRCValue() cũ
  int throttle = crsf.getChannel(1); // 988 → 2012µs equivalent
  int roll     = crsf.getChannel(2);
  int pitch    = crsf.getChannel(3);
  int yaw      = crsf.getChannel(4);
}
```

**Khi chuyển sang CRSF:** Xóa toàn bộ code ISR ở B3 — CRSF thay thế hoàn toàn, không dùng song song.

---

### B8 — VL535L1X — Độ cao laser

> [↑ Mục lục](#mục-lục)


**Thư viện:** **Tools → Manage Libraries** → tìm `VL53L1X by Pololu` → cài.

```cpp
#include <VL53L1X.h>

VL53L1X tof;
int altitude_mm = 0;
unsigned long lastTof = 0;

// Trong setup():
// Wire.begin(8, 9); // chung bus với MPU6050 — chỉ gọi 1 lần
// tof.init();
// tof.setDistanceMode(VL53L1X::Short); // tốt hơn cho indoor
// tof.startContinuous(50);             // đọc mỗi 50ms

// Trong loop() — 20Hz, không cần nhanh hơn:
if (millis() - lastTof >= 50) {
  altitude_mm = tof.read();
  lastTof = millis();
}
// altitude_mm → Altitude PID (outer loop thêm vào sau B5)
```

**Lưu ý:** MPU6050 (0x68) và VL535L1X (0x29) dùng chung dây I2C được — địa chỉ khác nhau.

---

### B9 — PMW3901 — Optical Flow (Giữ vị trí ngang)

> [↑ Mục lục](#mục-lục)


**Thư viện — cài thủ công:**
1. Vào https://github.com/bitcraze/Bitcraze_PMW3901 → **Code → Download ZIP**
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**

**Giao tiếp:** SPI (không dùng I2C — cần tốc độ cao hơn).

```cpp
#include <Bitcraze_PMW3901.h>

Bitcraze_PMW3901 flow(10); // CS = GPIO10
int16_t deltaX, deltaY;
unsigned long lastFlow = 0;

// Trong setup(): flow.begin();

// Trong loop() — 20Hz:
if (millis() - lastFlow >= 50) {
  flow.readMotionCount(&deltaX, &deltaY);
  lastFlow = millis();
  // deltaX/Y → scale theo altitude_mm → velocity → Position PID
}
```

**Cascade PID 3 tầng khi có đủ cảm biến:**
```
Position PID  (20Hz) — dùng PMW3901  → roll/pitch setpoint
  Attitude PID (50Hz) — dùng IMU góc  → rate setpoint
    Rate PID   (500Hz) — dùng gyro raw → motor output
```

---

## Track C — ESP32-S3 (ESP-IDF)

> **Milestone hệ thống:**
> - **M1 ✅ Hoàn thành:** Single super-loop `vTaskDelay(1ms)`, MPU6050 polling 200Hz (`drn_mpu6050` + `drn_raw_mpu6050`), complementary filter, calibration 1000 mẫu.
> - **M2 ❌ Target:** `xTaskCreatePinnedToCore` — Core 0: BLE/comms, Core 1: flight loop. GDMA đã config sẵn trong sdkconfig (`CONFIG_GDMA_CTRL_FUNC_IN_IRAM=y`).
>
> **Tài liệu chính:** [ESP-IDF docs ESP32-S3](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/) · ESP-IDF version: **5.3.1** (từ sdkconfig)

---

### C0 — drn_main_board_choose — ESP-IDF version

> [↑ Mục lục](#mục-lục)


Checklist khi port từ Arduino IDE sang ESP-IDF:
```
□ ledcWrite()         → ledc_set_duty() + ledc_update_duty()
□ Wire.begin()        → i2c_new_master_bus() + i2c_master_bus_add_device()
□ attachInterrupt()   → gpio_isr_handler_add()
□ delay(ms)           → vTaskDelay(pdMS_TO_TICKS(ms))
□ millis()            → esp_timer_get_time() / 1000  (microsecond timer)
□ Serial.printf()     → ESP_LOGI() hoặc printf() qua USB CDC
```

---

### C1 — MPU6050 — ESP-IDF new I2C driver

> [↑ Mục lục](#mục-lục)


> **Trạng thái repo M1 ✅:** Có **2 driver song song:**
> - `drn_raw_mpu6050` — đọc raw ADC, không filter, có calibration 1000 mẫu. Dùng để so sánh trước/sau filter.
> - `drn_mpu6050` — có complementary filter tích hợp, chạy 200Hz (5ms interval).
> Cả hai đều dùng new I2C master driver (ESP-IDF v5.x), không dùng `i2c_driver_install()` cũ.

```c
#include "driver/i2c_master.h"

i2c_master_bus_handle_t bus;
i2c_master_dev_handle_t mpu;

// Init một lần trong app_main()
i2c_master_bus_config_t bus_cfg = {
    .i2c_port      = I2C_NUM_0,
    .sda_io_num    = GPIO_NUM_8,
    .scl_io_num    = GPIO_NUM_9,
    .clk_source    = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_new_master_bus(&bus_cfg, &bus);

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = 0x68,   // AD0=LOW
    .scl_speed_hz    = 400000, // Fast mode
};
i2c_master_bus_add_device(bus, &dev_cfg, &mpu);

// Đọc 14 byte burst (acc + temp + gyro)
uint8_t reg = 0x3B;
uint8_t buf[14];
i2c_master_transmit_receive(mpu, &reg, 1, buf, 14, -1);
```

**So sánh với B1 (Arduino):** `Wire.beginTransmission(0x68)` + `Wire.requestFrom()` bên dưới gọi đúng các hàm này — chỉ là được wrap lại.

---

### C2 — PWM Motor — LEDC ESP-IDF

> [↑ Mục lục](#mục-lục)


```c
#include "driver/ledc.h"

// Config timer một lần
ledc_timer_config_t timer = {
    .speed_mode      = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,
    .timer_num       = LEDC_TIMER_0,
    .freq_hz         = 50,  // 50Hz cho motor/ESC
    .clk_cfg         = LEDC_AUTO_CLK,
};
ledc_timer_config(&timer);

// Config từng channel (1 channel = 1 motor)
ledc_channel_config_t ch = {
    .gpio_num   = 16,  // Motor 1
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel    = LEDC_CHANNEL_0,
    .timer_sel  = LEDC_TIMER_0,
    .duty       = 0,
    .hpoint     = 0,
};
ledc_channel_config(&ch);
// Lặp lại cho CHANNEL_1/2/3 với GPIO 17/18/19

// Set duty realtime — tương đương ledcWrite() của Arduino
void set_motor(ledc_channel_t ch, int us) {
    us = (us < 1000) ? 1000 : (us > 2000) ? 2000 : us;
    uint32_t duty = (uint32_t)((us * 16383UL) / 20000UL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}
```

---

### C3 — PWM nhận RC — ISR ESP-IDF

> [↑ Mục lục](#mục-lục)


```c
#include "driver/gpio.h"
#include "esp_timer.h"

static volatile uint32_t pwm_value    = 1500;
static volatile uint64_t pulse_start  = 0;
static volatile uint64_t last_pulse   = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR rc_isr(void *arg) {
    if (gpio_get_level(GPIO_NUM_6)) {
        pulse_start = esp_timer_get_time(); // microseconds
    } else {
        uint64_t w = esp_timer_get_time() - pulse_start;
        if (w >= 800 && w <= 2200) {
            portENTER_CRITICAL_ISR(&mux);
            pwm_value  = (uint32_t)w;
            last_pulse = esp_timer_get_time() / 1000; // ms
            portEXIT_CRITICAL_ISR(&mux);
        }
    }
}

// Setup trong app_main():
gpio_config_t io = {
    .pin_bit_mask = (1ULL << GPIO_NUM_6),
    .mode         = GPIO_MODE_INPUT,
    .intr_type    = GPIO_INTR_ANYEDGE,
};
gpio_config(&io);
gpio_install_isr_service(0);
gpio_isr_handler_add(GPIO_NUM_6, rc_isr, NULL);

// Đọc an toàn:
uint32_t get_rc_value(void) {
    portENTER_CRITICAL(&mux);
    uint32_t val = pwm_value;
    uint64_t age = esp_timer_get_time()/1000 - last_pulse;
    portEXIT_CRITICAL(&mux);
    return (age > 100) ? 1000 : val; // failsafe
}
```

---

### C4 — Test tích hợp IMU + Motor (không PID)

> [↑ Mục lục](#mục-lục)


> Giống A3 và B4 về logic — chỉ đổi API. Đọc D4 trước.

```c
// Trong FreeRTOS task:
void test_task(void *arg) {
    float pitch = 0, roll = 0, alpha = 0.98f;
    uint64_t last_t = esp_timer_get_time();

    while (1) {
        uint64_t now = esp_timer_get_time();
        float dt = (now - last_t) / 1e6f;
        last_t = now;

        // Đọc IMU (dùng hàm từ C1)
        read_mpu6050(buf);
        float accel_pitch = atan2f(ax/16384.f, az/16384.f) * 180/M_PI;
        float gyro_y      = gy / 131.f;
        pitch = alpha*(pitch + gyro_y*dt) + (1-alpha)*accel_pitch;

        int corr = (int)(pitch * 3);
        set_motor(LEDC_CHANNEL_0, 1200 - corr); // Front
        set_motor(LEDC_CHANNEL_1, 1200 + corr); // Rear

        printf("P=%.1f | mF=%d mR=%d\n", pitch, 1200-corr, 1200+corr);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

---

### C5 — Cascade PID — FreeRTOS task

> [↑ Mục lục](#mục-lục)


> **Kiến trúc:** 1 ISR (MPU INT pin) → nhả semaphore → PID task thức dậy → đọc IMU → tính PID → set motor. Không có mutex, không có deadlock.

```c
static SemaphoreHandle_t imu_sem;

static void IRAM_ATTR mpu_isr(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(imu_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

void pid_task(void *arg) {
    PID_t rate_pid = {0.5f, 0.0f, 0.01f, 0, 0};
    PID_t att_pid  = {3.0f, 0.0f, 0.0f,  0, 0};

    while (1) {
        if (xSemaphoreTake(imu_sem, pdMS_TO_TICKS(5)) == pdTRUE) {
            read_mpu6050(buf);
            complementary_filter(&pitch, &roll, buf, dt);

            float rate_out = pid_compute(&rate_pid, 0, gyro_y, dt);
            float att_out  = pid_compute(&att_pid,  0, pitch,  dt);
            motor_mixer(BASE_THROTTLE, att_out, att_out_roll, 0);
        } else {
            // Failsafe: 5ms không có data IMU → tắt motor
            set_all_motors(1000);
        }
    }
}

void app_main(void) {
    imu_sem = xSemaphoreCreateBinary();
    // Setup GPIO INT từ MPU6050
    gpio_isr_handler_add(GPIO_NUM_4, mpu_isr, NULL);
    // Chạy PID task trên Core 1 — không share CPU với WiFi
    xTaskCreatePinnedToCore(pid_task, "PID", 4096, NULL, 5, NULL, 1);
}
```

**Verify cross-platform:** Copy Kp/Ki/Kd từ A4/A5 sang đây. Nếu drone phản ứng giống hệt = port thành công.

---

### C6 — Telemetry USB CDC — ESP-IDF

> [↑ Mục lục](#mục-lục)


```c
#include "esp_log.h"

// Gửi 10Hz trong task riêng — không gửi trong PID task
static void telemetry_task(void *arg) {
    while (1) {
        printf("%.2f,%.2f,%.3f,%.3f,%d,%d,%d,%d\n",
            pitch, roll, rate_out, att_out, m1, m2, m3, m4);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
// xTaskCreate(telemetry_task, "TEL", 2048, NULL, 2, NULL);
```

Dùng cùng script Python / Hercules đang có, chỉ đổi COM port.

---

### C7 — Nâng cấp RC — CRSF ESP-IDF

> [↑ Mục lục](#mục-lục)


> Làm khi đã xác nhận receiver TBS Tango 2 Pro — xem B7 để hiểu CRSF là gì.

```c
#include "driver/uart.h"

// CRSF dùng UART1, baud 420000
uart_config_t crsf_cfg = {
    .baud_rate = 420000,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};
uart_param_config(UART_NUM_1, &crsf_cfg);
uart_set_pin(UART_NUM_1, GPIO_NUM_43, GPIO_NUM_44, -1, -1);
uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

// Parse CRSF packet — header 0xC8, length, type, payload, CRC
// Kênh 1–16 nằm trong payload type 0x16 (RC_CHANNELS_PACKED)
```

---

### C8 — I2C Interrupt + GDMA (Nâng cấp từ C1)

> [↑ Mục lục](#mục-lục)


> **GDMA đã config sẵn trong sdkconfig** (`CONFIG_GDMA_CTRL_FUNC_IN_IRAM=y`) — hardware sẵn sàng, chỉ cần enable trong driver. Đây là bước M2, làm sau khi C5 ổn định.

```c
// Cách 1 — GPIO interrupt từ chân INT MPU6050 (đơn giản hơn, làm trước)
// MPU6050 INT pin → GPIO4 → ISR → semaphore → PID task

// Cách 2 — GDMA thật sự (burst 14 byte không tốn CPU — M2)
// i2c_master_transmit_receive() với DMA mode, ESP-IDF v5.3.1
// Tài liệu: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html
```

---

### C9 — VL535L1X — ESP-IDF

> [↑ Mục lục](#mục-lục)


```c
// Giao tiếp I2C, địa chỉ 0x29, dùng chung bus với MPU6050
// Dùng driver C từ ST: https://github.com/stm32duino/VL53L1X

// Đọc 20Hz trong telemetry task hoặc task riêng:
static uint32_t last_tof = 0;
if (esp_timer_get_time()/1000 - last_tof >= 50) {
    VL53L1X_GetDistance(dev, &altitude_mm);
    last_tof = esp_timer_get_time()/1000;
}
```

---

### C10 — PMW3901 — SPI ESP-IDF

> [↑ Mục lục](#mục-lục)


```c
#include "driver/spi_master.h"

spi_bus_config_t buscfg = {
    .miso_io_num = GPIO_NUM_13,
    .mosi_io_num = GPIO_NUM_11,
    .sclk_io_num = GPIO_NUM_12,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};
spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 2 * 1000 * 1000, // 2MHz
    .mode           = 3,                // SPI mode 3
    .spics_io_num   = GPIO_NUM_10,
    .queue_size     = 1,
};
spi_device_handle_t pmw;
spi_bus_add_device(SPI2_HOST, &devcfg, &pmw);

// Đọc deltaX, deltaY mỗi 20ms → Position PID
```

---

## Track D — Thuật toán (Không phụ thuộc chip hay IDE)

---

### D1 — Complementary Filter (✅ Đã implement trong `drn_mpu6050`)

> [↑ Mục lục](#mục-lục)


> **Trạng thái repo:** Filter đã chạy trong `drn_mpu6050.cpp` trên ESP32. Có 2 simulator HTML để visualize: `imu_simulator_complementary_filter_v1/v2.html`. Thông số alpha có thể điều chỉnh trực tiếp trên simulator trước khi đưa vào code.

```c
// alpha = 0.98: tin gyro 98%, accelerometer 2%
// Tốt cho indoor hover. Không cần Kalman.
angle = alpha * (angle + gyro_rate * dt) + (1.0f - alpha) * accel_angle;
```

Kalman tốt hơn khi: outdoor nhiều gió, cần độ chính xác góc < 0.5°. Với drone indoor, complementary filter là đủ và dễ tune hơn.

---

### D2 — Motor Mixer (Mô phỏng xong — cần test thật ở A3/B4)

> [↑ Mục lục](#mục-lục)


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

### D3 — Cascade PID: Thứ tự và giá trị khởi điểm

> [↑ Mục lục](#mục-lục)


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

### D4 — Quy trình test tích hợp từng bước

> [↑ Mục lục](#mục-lục)


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
| STM32F103 Reference Manual RM0008 | [ST PDF](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) | A7, A8 |
| STM32F103 Errata ES096 | [ST PDF](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) | A1 — I2C bug |
| MPU6050 Library (Electronic Cats) | [GitHub](https://github.com/ElectronicCats/mpu6050) | B1 — Arduino |
| VL53L1X Library (Pololu) | [GitHub](https://github.com/pololu/vl53l1x-arduino) | B8 — Arduino |
| PMW3901 Library (Bitcraze) | [GitHub](https://github.com/bitcraze/Bitcraze_PMW3901) | B9 — Arduino |
| CRSFforArduino | [GitHub](https://github.com/ZZ-Cat/CRSFforArduino) | B7 — CRSF Arduino |
| ESP32-S3 Technical Reference Manual | [Espressif PDF](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf) | C8, C10 |
| ESP-IDF I2C Driver (new API) | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/i2c.html) | C1, C8 |
| ESP-IDF LEDC Driver | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/ledc.html) | C2 |
| ESP-IDF GPIO & Interrupt | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html) | C3, C8 |
| ESP-IDF SPI Master | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html) | C10 |
| ESP-IDF FreeRTOS | [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/freertos_idf.html) | C5 — Task + Semaphore |
| MPU6050 Register Map | [InvenSense PDF](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf) | A1, B1, C1 |
| VL53L1X Datasheet | [ST](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html) | B8, C9 |

---

*Version 5.0 — Cấu trúc: 4 track theo hệ trục tọa độ. TOC dạng bảng 2×2 thay cho danh sách dọc.*
*Cập nhật từ repo thực tế: Stage 10.5 Final Release (PA7 thay PA6), M1 ESP32 hoàn thành, drn_raw_mpu6050 + drn_mpu6050 dual driver, GDMA config sẵn, PID struct có trong STM32 chưa kết hợp IMU.*