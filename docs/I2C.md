# Phân tích Polling I2C và Roadmap Chuyển Đổi Sang Interrupt/DMA/FreeRTOS
## Dành cho 2 repo Quadcopter: STM32F103C8T6 & ESP32-S3 Supermini
### Phiên bản: ESP-IDF 5.3.1 | STM32 bare-metal (không HAL)

---

> **Ghi chú an toàn trước khi bắt đầu:** Trong toàn bộ quá trình refactor I2C, hãy giữ throttle = 0 (motor không quay). Các thay đổi driver có thể khiến flight loop bị block tạm thời. Chỉ chạy motor sau khi đã qua test ở Giai đoạn tương ứng.

---

# Phần 1: Các đoạn Polling cần thay thế — phân tích từ code thực tế

## 1A. STM32F103C8T6

### Polling 1: `MPU_Read_Multi` – vòng lặp chờ từng byte (`i2c_mpu_debug.c`, dòng 899–904)

```c
// HIỆN TẠI — polling chặn CPU hoàn toàn
I2C1->CR1 |= I2C_CR1_ACK;
for(uint8_t i = 0; i < size; i++) {
    if(i == size - 1) { I2C1->CR1 &= ~I2C_CR1_ACK; I2C1->CR1 |= I2C_CR1_STOP; }
    timeout = 500000;
    while(!(I2C1->SR1 & I2C_SR1_RXNE)) { if(--timeout == 0) goto i2c_error; }
    data[i] = I2C1->DR;
}
```

**Thay bằng:** `DMA1_Channel7` tự động đọc `I2C1->DR` vào buffer mỗi khi `RXNE=1`. CPU chỉ bị ngắt một lần duy nhất khi toàn bộ 14 byte đã hoàn tất (cờ `TCIF7`).

**Lưu ý quan trọng:** KHÔNG bật `I2C_CR2_ITBUFEN` ở bước trung gian nếu mục tiêu là DMA. Kết hợp `ITBUFEN` + 14 byte + 500Hz sẽ tạo ra 7.000 ngắt/giây chỉ riêng cho I2C, phản tác dụng hoàn toàn.

---

### Polling 2: `Core_I2C_Read` – hàm dùng trong `Filter_Machine_Run` (`xx_mpu_complementary_filter.c`, dòng 1949–1974)

```c
// HIỆN TẠI — cùng pattern với MPU_Read_Multi, nhưng dùng riêng trong bộ lọc
for(uint8_t i = 0; i < size; i++) {
    if(i == size - 1) { I2C1->CR1 &= ~I2C_CR1_ACK; I2C1->CR1 |= I2C_CR1_STOP; }
    t = 500000; while(!(I2C1->SR1 & I2C_SR1_RXNE)) { if(--t==0) goto err; }
    data[i] = I2C1->DR;
}
```

File này có **bộ lọc bù (complementary filter)** riêng — nơi tiêu thụ trực tiếp 14 byte từ I2C vào `Filter_IMU` (Roll, Pitch, Yaw). Khi chuyển sang DMA, buffer DMA 14 byte phải được `Filter_Machine_Run` đọc từ SRAM chứ không phải từ `I2C1->DR` trực tiếp. Cần khai báo buffer ở scope toàn cục hoặc static để cả ISR và filter cùng truy cập được.

---

### Polling 3: `MPU_WriteReg` – chờ TXE và BTF (`i2c_mpu_debug.c`, dòng 805–821)

```c
// HIỆN TẠI
I2C1->DR = reg;
t = 100000; while(!(I2C1->SR1 & I2C_SR1_TXE)) { if(--t==0) return 0x07; }
I2C1->DR = data;
t = 100000; while(!(I2C1->SR1 & I2C_SR1_BTF)) { if(--t==0) return 0x07; }
```

**Quyết định:** Giữ lại polling cho ghi 1–2 byte. Thời gian chiếm CPU chỉ ~40µs, không ảnh hưởng flight loop 500Hz. Thay bằng DMA TX chỉ cần thiết nếu viết nhiều byte liên tiếp (ví dụ: ghi burst cấu hình lúc boot).

---

### Polling 4: Chờ SB và ADDR trong Start sequence (`i2c_mpu_debug.c`, dòng 807–812)

```c
// HIỆN TẠI
while(!(I2C1->SR1 & I2C_SR1_SB))   { if(--t==0) return 0x05; }
while(!(I2C1->SR1 & I2C_SR1_ADDR)) { if(--t==0) return 0x06; }
```

**Thay bằng:** Ngắt I2C Event (`EV5`, `EV6`) xử lý trong ISR theo state machine. DMA bắt đầu sau khi `ADDR` được clear — không cần CPU polling chờ.

---

### ⚠️ Errata STM32F103 — BẮT BUỘC đọc trước khi implement DMA

STM32F103C8T6 có lỗi silicon liên quan đến I2C + DMA, được ghi trong **Errata sheet ES096**, không có trong RM0008:

- **Lỗi 2.14.4:** Bit `BTF` có thể không được set đúng lúc khi dùng DMA mode. **Workaround:** Bật bit `LAST` trong `I2C_CR2` để I2C tự động NACK byte cuối cùng và tạo STOP, thay vì dựa vào `BTF`.
- **Lỗi 2.14.7:** `SDA` có thể bị kẹt ở mức thấp (stuck low) sau khi bus reset nếu không thực hiện bit-bang recovery đúng thứ tự. **Workaround:** Giữ lại hàm `I2C_Bus_Recovery()` hiện có trong `i2c_mpu_debug.c` — đây là workaround phần cứng bắt buộc, không phải code thừa.

**Tài liệu cần đọc:** [Errata ES096](https://www.st.com/resource/en/errata_sheet/es096-stm32f103x8-and-stm32f103xb-device-errata-stmicroelectronics.pdf) — mục 2.14.x (I2C limitations). Đọc **trước** RM0008 Ch.26.

---

## 1B. ESP32-S3 Supermini (ESP-IDF 5.3.1)

> **Xác nhận phiên bản API:** Repo đang dùng ESP-IDF **5.3.1** (xác nhận từ `sdkconfig`). Ở phiên bản này, `i2c_master_write_to_device` và `i2c_master_write_read_device` thuộc **Legacy I2C driver** (`driver/i2c.h`) — đã deprecated từ IDF v5.1. Driver mới dùng `driver/i2c_master.h` với handle-based API.

---

### Polling 1: `i2c_read_multi` trong `drn_mpu6050.cpp` (dòng 6618–6623)

```cpp
// HIỆN TẠI — Legacy driver, blocking
static esp_err_t i2c_read_multi(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, DRN_MPU6050_I2C_ADDR,
                                        &reg_addr, 1, data, len,
                                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}
```

**API cần thay (IDF 5.3.1 — New Driver):**
```cpp
// THAY BẰNG — New driver, async
// Khởi tạo (1 lần trong Init):
i2c_master_bus_config_t bus_cfg = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_8,
    .scl_io_num = GPIO_NUM_9,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_cfg, &bus_handle);

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = DRN_MPU6050_I2C_ADDR,
    .scl_speed_hz    = 400000,
};
i2c_master_dev_handle_t dev_handle;
i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);

// Đọc async trong flight loop:
i2c_master_transmit_receive_async(dev_handle,
    &reg_addr, 1,        // write: địa chỉ register
    dma_buffer, 14,      // read: 14 byte vào double-buffer
    portMAX_DELAY,
    i2c_done_callback,   // callback khi GDMA hoàn tất
    NULL);
```

---

### Polling 2: `i2c_raw_read_multi` trong `drn_raw_mpu6050.cpp` (dòng 6929–6934)

Cùng pattern với `drn_mpu6050.cpp`. Cả hai file dùng chung `I2C_NUM_0`, `GPIO_NUM_8/9` — chỉ khác địa chỉ slave (`DRN_MPU6050_I2C_ADDR` vs `DRN_RAW_MPU6050_I2C_ADDR`). Khi migrate sang new driver, dùng chung 1 `bus_handle`, tạo 2 `dev_handle` riêng biệt.

---

### Polling 3: `i2c_write_reg` / `i2c_raw_write_reg` — ghi 1–2 byte

**Quyết định:** Giữ lại (hoặc migrate sang `i2c_master_transmit` của new driver). Ghi 1 byte lúc boot/calibration không ảnh hưởng flight loop. Ưu tiên migrate để đồng bộ với bus_handle mới, tránh dùng 2 driver cùng lúc gây conflict.

---

### Polling 4: `DRN_MPU6050_Run_Task` — throttle 200Hz bằng polling thời gian (dòng 6826–6835)

```cpp
// HIỆN TẠI — polling millis() trong main loop
void DRN_MPU6050_Run_Task(uint32_t current_time_ms) {
    if ((current_time_ms - xx_last_read_ms) >= 5) {  // 200Hz
        xx_last_read_ms = current_time_ms;
        if (DRN_MPU6050_Read_Raw() == ESP_OK) DRN_MPU6050_Compute();
    }
}
```

**Thay bằng:** FlightTask (GĐ3) block trên `ulTaskNotifyTake`, được đánh thức bởi GPIO interrupt từ chân INT của MPU. Bỏ hoàn toàn vòng lặp polling thời gian này.

---

### Không cần thay — `MPU_Read_WhoAmI` / `DRN_RawMPU6050_Init`

Các hàm đọc WHO_AM_I và init chạy **1 lần khi boot**, không ảnh hưởng flight loop. Có thể giữ blocking. Nếu muốn đồng nhất, migrate sang `i2c_master_transmit_receive` (synchronous) của new driver thay vì async — không cần callback.

---

# Phần 2: Bảng Roadmap 3 giai đoạn

## 2A. STM32F103C8T6 (Bare-metal)

| Giai đoạn | Mô tả công việc | Register / Bit chính | Tài liệu cần đọc (ưu tiên) | Tiêu chí test thành công |
| :--- | :--- | :--- | :--- | :--- |
| **GĐ1: EXTI interrupt từ chân INT MPU** | 1. Cắm chân INT của MPU6500 vào PA8 (hoặc chân EXTI khả dụng).<br>2. Cấu hình EXTI line, trigger Rising edge.<br>3. ISR cực ngắn: chỉ set 1 flag biến `volatile uint8_t imu_ready = 1`, không đọc I2C bên trong ISR.<br>4. Main loop kiểm tra flag, gọi `MPU_Read_Multi` (vẫn polling) khi flag set, rồi clear flag.<br>5. **Giữ nguyên hoàn toàn** `MPU_Read_Multi` polling — giai đoạn này chỉ thêm trigger EXTI. | `EXTI->IMR`, `EXTI->RTSR`, `EXTI->PR`, `NVIC_EnableIRQ(EXTIx_IRQn)` | **RM0008 Ch.8** (EXTI): mục 8.2 (EXTI line mapping), 8.3.3 (Rising trigger). **Datasheet STM32F103**: Table 8 (pin AF), xác nhận PA8 là EXTI8. | Logic analyzer: xung INT từ MPU6500 → ISR đánh thức → `MPU_Read_Multi` được gọi. Chu kỳ giữa 2 lần đọc ổn định 2ms ± 200µs. Không cần DMA để đạt cột mốc này. |
| **GĐ2: DMA1_Channel7 cho I2C RX** | 1. **Đọc Errata ES096 mục 2.14** trước khi code.<br>2. Cấu hình `DMA_CPAR = (uint32_t)&I2C1->DR`, `DMA_CMAR = (uint32_t)dma_buf`, `DMA_CNDTR = 14`.<br>3. Bật `I2C_CR2_DMAEN` và `I2C_CR2_LAST` (workaround errata: LAST đảm bảo NACK+STOP byte cuối đúng thứ tự).<br>4. **KHÔNG bật** `I2C_CR2_ITBUFEN` — chỉ bật `I2C_CR2_ITEVTEN` để xử lý SB và ADDR bằng state machine ISR ngắn.<br>5. Bật ngắt DMA TC (`DMA_CCR_TCIE`). ISR DMA: set flag `imu_dma_done = 1`, clear `TCIF7`.<br>6. Main loop đọc từ `dma_buf[14]` (SRAM) thay vì đọc trực tiếp từ I2C — cập nhật `Filter_Machine_Run` để đọc từ buffer này. | `DMA_CPAR`, `DMA_CMAR`, `DMA_CNDTR`, `DMA_CCR` (bit `EN`, `MINC`, `TCIE`, `CIRC`), `I2C_CR2_DMAEN`, `I2C_CR2_LAST`, `DMA_ISR_TCIF7`, `DMA_IFCR_CTCIF7` | **Errata ES096** (ĐỌC TRƯỚC): mục 2.14.4, 2.14.7.<br>**RM0008 Ch.13.3.5**: Table 78 (DMA1 channel mapping — I2C1_RX = Channel7).<br>**RM0008 Ch.26.3.6**: I2C DMA requests, bit LAST. | Oscilloscope: chân SCL hoàn tất 14 byte clock mà CPU không thực hiện vòng lặp nào giữa các byte. Logic analyzer: STOP condition xuất hiện **sau** byte 14, không trước. PWM 4 motor (TIM3) không bị jitter khi I2C đang truyền. |
| **GĐ3: FreeRTOS + Task Notification** | 1. Port FreeRTOS lên STM32F103 (dùng CMSIS-RTOS2 hoặc FreeRTOS trực tiếp).<br>2. Tạo `FlightTask` với priority = `configMAX_PRIORITIES - 1`.<br>3. DMA TC ISR gọi `vTaskNotifyGiveFromISR()` **với điều kiện** priority của ngắt DMA thấp hơn `configMAX_SYSCALL_INTERRUPT_PRIORITY` — nếu không, assertion fault sẽ xảy ra.<br>4. `FlightTask` gọi `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5))` — timeout 5ms để detect mất tín hiệu IMU.<br>5. Thêm watchdog: nếu `ulTaskNotifyTake` timeout quá 3 lần liên tiếp, set throttle = 0 và gọi `I2C_Bus_Recovery()`. | `xTaskCreate`, `vTaskNotifyGiveFromISR`, `ulTaskNotifyTake`, `configMAX_SYSCALL_INTERRUPT_PRIORITY`, `NVIC_SetPriority` | **FreeRTOS Ref Manual**: Task Notifications (thay cho semaphore, latency thấp hơn ~30%).<br>**FreeRTOS Port Guide**: Cortex-M3 port, `configMAX_SYSCALL_INTERRUPT_PRIORITY`.<br>**RM0008 Ch.9** (NVIC): priority grouping. | Drone treo dây: `FlightTask` wake-up jitter < 50µs. Ngắt DMA TC và EXTI coexist không assertion fault. Kéo dây I2C → watchdog detect trong 15ms → throttle về 0 tự động. |

---

## 2B. ESP32-S3 Supermini (ESP-IDF 5.3.1 + FreeRTOS SMP)

> **Chiến lược:** Bắt đầu từ ESP32-S3 trước STM32 vì FreeRTOS đã có sẵn, new I2C driver đã có GDMA tích hợp, và dual-core giúp isolate FlightTask khỏi Wi-Fi/BLE. Kết quả validate ở đây trước khi port logic sang STM32 bare-metal.

| Giai đoạn | Mô tả công việc | API / Register chính | Tài liệu cần đọc (ưu tiên) | Tiêu chí test thành công |
| :--- | :--- | :--- | :--- | :--- |
| **GĐ1: Migrate sang New I2C Driver (IDF 5.x)** | 1. Thay `#include "driver/i2c.h"` → `#include "driver/i2c_master.h"` trong `drn_mpu6050.cpp` và `drn_raw_mpu6050.cpp`.<br>2. Tạo 1 `i2c_master_bus_handle_t` dùng chung (GPIO8=SDA, GPIO9=SCL, 400kHz).<br>3. Tạo 2 `i2c_master_dev_handle_t` riêng biệt cho cùng địa chỉ slave (drn_mpu6050 và drn_raw_mpu6050 — nếu 2 file dùng cùng 1 địa chỉ, chỉ cần 1 handle).<br>4. Thay `i2c_master_write_read_device` → `i2c_master_transmit_receive` (synchronous, vẫn blocking nhưng dùng new driver). FlightTask **gọi trong task context**, không phải trong ISR.<br>5. **Giữ FreeRTOS task thông thường** — giai đoạn này chưa async, chưa callback. | `i2c_new_master_bus()`, `i2c_master_bus_add_device()`, `i2c_master_transmit_receive()`, `i2c_del_master_bus()` | **ESP-IDF 5.x I2C Driver Guide**: "New I2C driver" section, migration từ legacy.<br>**TRM Ch.27**: I2C Controller (hiểu FIFO, command table).<br>**TRM Ch.6**: IO MUX — xác nhận GPIO8/9 hỗ trợ I2C fast mode. | Build không warning deprecated. UART log không có `ESP_ERR_TIMEOUT`. Chu kỳ đọc 14 byte giữ nguyên so với legacy driver. |
| **GĐ2: GPIO Interrupt + GDMA Async** | 1. Cấu hình chân INT của MPU (GPIO4 hoặc chân khả dụng) bằng `gpio_install_isr_service` + `gpio_isr_handler_add`, trigger RISING edge.<br>2. ISR chỉ gọi `xTaskNotifyFromISR()` — **không** gọi `i2c_master_transmit_receive_async` bên trong ISR.<br>3. FlightTask (đã pin vào Core 1) đợi notify, rồi gọi `i2c_master_transmit_receive_async()` với callback.<br>4. **Callback chạy trong GDMA ISR context** — chỉ được: swap pointer double-buffer, gọi `xTaskNotifyFromISR()`. **Không được:** `memcpy`, `printf`, malloc, hay bất kỳ blocking call nào.<br>5. Implement double-buffer: `uint8_t dma_buf[2][14]`, `uint8_t active = 0`. Callback: `active ^= 1`. FlightTask đọc từ `dma_buf[1 - active]`. | `gpio_isr_handler_add()`, `i2c_master_transmit_receive_async()`, `xTaskNotifyFromISR()`, `xTaskNotifyWait()`, double-buffer pattern | **TRM Ch.27.4.2**: I2C DMA Interface.<br>**TRM Ch.3.5**: GDMA channels, burst size.<br>**TRM Ch.5**: Interrupt Matrix — xác nhận priority của GDMA ISR.<br>**ESP-IDF**: `i2c_master_transmit_receive_async` API ref. | GDMA callback latency < 200µs sau khi 14 byte hoàn tất. CPU usage Core 1 (đo bằng `esp_cpu_cycle_count`) giảm so với GĐ1. `memcheck`: không có race condition trên double-buffer (kiểm tra bằng cách log `active` trong callback và task). |
| **GĐ3: FlightTask hoàn chỉnh + Watchdog IMU** | 1. `xTaskCreatePinnedToCore(FlightTask, ..., 4096, NULL, configMAX_PRIORITIES-1, NULL, 1)` — pin Core 1, priority cao nhất.<br>2. Trong FlightTask: `xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(5))` — timeout 5ms. Nếu timeout quá 3 lần liên tiếp: log error, set throttle = 0, gọi `i2c_master_bus_reset(bus_handle)`.<br>3. `PID_Compute` và `Motor_Mixer` chạy trực tiếp trong FlightTask sau khi dữ liệu IMU sẵn sàng — không qua queue để tránh overhead.<br>4. Core 0 xử lý Wi-Fi/BLE, telemetry (nếu có) — không ảnh hưởng FlightTask.<br>5. Thêm `esp_task_wdt_add` cho FlightTask để detect deadlock. | `xTaskCreatePinnedToCore(..., 1)`, `xTaskNotifyWait()`, `i2c_master_bus_reset()`, `esp_task_wdt_add()`, `esp_timer_get_time()` | **ESP-IDF FreeRTOS SMP**: `xTaskCreatePinnedToCore`, core affinity.<br>**ESP-IDF**: Task WDT API.<br>**TRM Ch.5**: Interrupt Matrix, xác nhận priority GDMA vs FreeRTOS tick. | Drone treo dây bay ổn định 10 phút: UART CSV log đủ 500 mẫu/giây (đo bằng `esp_timer_get_time` delta). Không có Yaw drift đột ngột. Rút dây I2C → `i2c_master_bus_reset` được gọi trong 15ms → throttle = 0. WDT không trigger trong điều kiện bình thường. |

---

# Phần 3: Bảng "KHÔNG làm" — các bẫy quan trọng

| # | Đừng làm | Lý do | Thay bằng |
| :--- | :--- | :--- | :--- |
| 1 | Bật `I2C_CR2_ITBUFEN` trên STM32 khi mục tiêu là DMA | Tạo ISR bão: 14 byte × 500Hz = 7.000 ngắt/giây chỉ cho I2C, counter-productive | Chỉ bật `ITEVTEN` cho SB/ADDR, để DMA xử lý data bytes |
| 2 | Gọi `memcpy(dst, dma_buf, 14)` bên trong GDMA callback (ESP32) | Callback chạy trong ISR context — `memcpy` 14 byte có thể vượt thời gian cho phép, gây watchdog hoặc stack overflow | Chỉ swap pointer double-buffer (`active ^= 1`), gọi `xTaskNotifyFromISR` |
| 3 | Dùng `i2c_master_write_read_device` (legacy) song song với new driver | Conflict giữa 2 driver trên cùng I2C port — undefined behavior | Migrate toàn bộ sang new driver, dùng 1 `bus_handle` duy nhất |
| 4 | Bỏ hàm `I2C_Bus_Recovery()` trên STM32 | Đây là workaround bắt buộc cho errata silicon — không có trong RM0008 | Giữ lại, gọi nó trong error path của DMA TC ISR khi detect timeout |
| 5 | Đặt DMA TC interrupt priority **cao hơn** `configMAX_SYSCALL_INTERRUPT_PRIORITY` (STM32+FreeRTOS) | `vTaskNotifyGiveFromISR` sẽ gây assertion fault tại runtime | Đặt DMA TC priority **thấp hơn** giá trị `configMAX_SYSCALL_INTERRUPT_PRIORITY` (số lớn hơn = priority thấp hơn trên Cortex-M) |
| 6 | Refactor I2C khi motor đang chạy | Flight loop có thể bị block trong lúc chuyển đổi driver, gây mất điều khiển | Giữ throttle = 0 trong toàn bộ quá trình refactor, chỉ test motor sau khi qua tiêu chí test của từng giai đoạn |
| 7 | Bỏ `I2C_CR2_LAST` khi dùng DMA trên STM32 | Byte 13 và 14 sẽ bị ACK sai, STOP condition xuất hiện sai vị trí — errata 2.14.4 | Luôn bật `LAST` cùng với `DMAEN` khi đọc |

---

# Phần 4: Gợi ý thứ tự bắt đầu

**Bắt đầu từ ESP32-S3** vì:
1. FreeRTOS và GDMA đã có sẵn trong SDK — ít rủi ro brick hơn STM32 bare-metal.
2. Có thể validate logic filter + PID trên ESP32 trước, sau đó dùng kết quả đó để so sánh với STM32.
3. Nếu gặp lỗi I2C trên ESP32, có `ESP_LOGE` và monitor serial dễ debug hơn. Trên STM32 bare-metal cần oscilloscope hoặc J-Link.

**Lộ trình gợi ý:**
1. ESP32-S3 GĐ1 (migrate new driver) → test → ✓
2. ESP32-S3 GĐ2 (GPIO INT + GDMA async) → test → ✓
3. ESP32-S3 GĐ3 (FlightTask + watchdog) → test bay treo → ✓
4. STM32 GĐ1 (EXTI từ INT MPU) → test → ✓
5. STM32 GĐ2 (DMA1_Channel7) → test oscilloscope → ✓
6. STM32 GĐ3 (FreeRTOS port) → test bay treo → ✓