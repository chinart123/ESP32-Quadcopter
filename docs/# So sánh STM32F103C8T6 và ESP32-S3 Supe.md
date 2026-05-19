# So sánh STM32F103C8T6 và ESP32-S3 Supermini
### Góc độ: Xử lý DSP, Điều khiển Thời gian Thực & Điểm yếu Kỹ thuật

---

## Mục lục

1. [Tổng quan kiến trúc](#1-tổng-quan-kiến-trúc)
2. [Bảng so sánh cốt lõi](#2-bảng-so-sánh-cốt-lõi)
3. [Phân tích điểm yếu — Tầng 1: Cơ bản](#3-phân-tích-điểm-yếu--tầng-1-cơ-bản)
4. [Phân tích điểm yếu — Tầng 2: Trung cấp](#4-phân-tích-điểm-yếu--tầng-2-trung-cấp)
5. [Phân tích điểm yếu — Tầng 3: Phức tạp](#5-phân-tích-điểm-yếu--tầng-3-phức-tạp)
6. [Bài toán định mức thời gian thực tế](#6-bài-toán-định-mức-thời-gian-thực-tế)
7. [Chiến lược thiết kế Firmware](#7-chiến-lược-thiết-kế-firmware)
8. [Tóm tắt ưu tiên xử lý](#8-tóm-tắt-ưu-tiên-xử-lý)

---

## 1. Tổng quan kiến trúc

Điểm khởi đầu quan trọng cần hiểu rõ: **lõi Cortex-M3 không có phần cứng xử lý số thực (FPU) chuyên dụng** như dòng Cortex-M4. Do đó, các hàm CMSIS-DSP trên STM32F103 sẽ được thực thi bằng phần mềm (software emulation), đòi hỏi thiết kế chu kỳ tính toán rất cẩn thận để không làm nghẽn hệ thống.

Sự khác biệt này — cùng với hàng loạt đặc điểm kiến trúc khác — tạo ra ranh giới rõ ràng giữa hai vi điều khiển khi triển khai các hệ thống điều khiển thời gian thực.

---

## 2. Bảng so sánh cốt lõi

| Tiêu chí | STM32F103C8T6 | ESP32-S3 Supermini |
| :--- | :--- | :--- |
| **Kiến trúc & Xung nhịp** | ARM Cortex-M3, đơn nhân, 72 MHz | Xtensa LX7, lõi kép, tối đa 240 MHz |
| **FPU (Floating-Point Unit)** | **Không có.** Mọi biến `float`/`double` không có hỗ trợ phần cứng | **Có.** FPU đơn độ chính xác 32-bit. Có thêm tập lệnh vector AI/MAC |
| **Cơ chế tính toán số thực** | **Software Emulation:** Trình biên dịch chèn hàng chục phép toán số nguyên để "giả lập" một phép cộng/nhân số thực | **Hardware Execution:** Xử lý trực tiếp biến `float` ở cấp độ thanh ghi CPU |
| **SRAM** | **20 KB** | **512 KB** (internal) + hỗ trợ PSRAM ngoài |
| **Flash** | 64 KB (C8T6) | 4–16 MB (tùy module) |
| **Thư viện DSP** | CMSIS-DSP — chạy ở chế độ mô phỏng số thực, rất tốn tài nguyên | ESP-DSP — tận dụng tập lệnh SIMD (nhân/cộng nhiều phần tử ma trận cùng lúc trong 1 chu kỳ) |
| **Thời gian nghịch đảo ma trận 3×3** | Tốn hàng nghìn chu kỳ máy, chiếm thời gian thực thi rất dài | Vài chục đến vài trăm chu kỳ máy, gần như tức thời |
| **ADC** | 12-bit, tốc độ ~1 MSPS, chất lượng ổn định | 12-bit (thực tế ~11-bit do noise), tốc độ ~83 kSPS |
| **Tính xác định thời gian** | **Cao.** Không cache, không WiFi → mọi lệnh thực thi với độ trễ tuyệt đối xác định | **Thấp hơn.** Cache miss và WiFi ISR gây jitter không đoán trước |
| **Điện áp I/O** | Nhiều chân 5V-tolerant | **Tất cả GPIO chỉ chịu 3.3V** |
| **Kích thước file `.bin`** | Lớn hơn — trình biên dịch nhúng thêm thư viện giả lập float | Nhỏ gọn hơn — lệnh float ánh xạ trực tiếp thành mã máy |
| **Phù hợp ứng dụng** | Hard real-time, safety-critical, môi trường xác định | Tính toán nặng, AI/ML edge, kết nối không dây |

---

## 3. Phân tích điểm yếu — Tầng 1: Cơ bản

> *Những điểm này thường bị bỏ qua vì "ai cũng biết", nhưng lại gây hậu quả nghiêm trọng nhất trong thực tế.*

### 3.1 Bộ nhớ RAM — Điểm nghẽn thực sự của STM32F103

Tài liệu so sánh thường chỉ nhắc đến Flash mà bỏ qua RAM — thứ nguy hiểm hơn nhiều khi chạy DSP.

| | STM32F103C8T6 | ESP32-S3 |
|---|---|---|
| SRAM | **20 KB** | **512 KB** + PSRAM tùy chọn |
| Stack mặc định an toàn | ~2–4 KB | ~8–16 KB |

**Hệ quả thực tế:** Ma trận Kalman 6×6 kiểu `float` chiếm ~144 bytes. Khi CMSIS-DSP cấp phát buffer tạm để tính nghịch đảo, nó cần thêm workspace. Trên 20 KB RAM, chỉ cần vài lớp gọi hàm lồng nhau là **stack overflow** — không crash ngay mà gây lỗi ngẫu nhiên, rất khó debug.

---

### 3.2 Bẫy `double` ẩn trong C/C++ — Cả hai chip đều bị ảnh hưởng

ESP32-S3 có FPU **đơn độ chính xác** (32-bit `float`). Điều thường không được nêu rõ:

- `double` (64-bit) trên **cả hai chip** đều là software emulation
- Trên ESP32-S3: `double` chậm hơn `float` khoảng **8–15 lần**
- Nếu developer vô tình dùng hằng số kiểu `1.0` thay vì `1.0f`, trình biên dịch ngầm ép kiểu sang `double` → toàn bộ pipeline tính toán bị kéo xuống SW emulation **mà không có cảnh báo nào**

```c
// ❌ SAI — ngầm dùng double, kéo toàn bộ biểu thức sang SW emulation
float result = sensorValue * 0.0174533;

// ✅ ĐÚNG — rõ ràng là float
float result = sensorValue * 0.0174533f;
```

---

### 3.3 Điện áp I/O và giao tiếp ngoại vi

| | STM32F103C8T6 | ESP32-S3 |
|---|---|---|
| Mức điện áp I/O | Nhiều chân **5V-tolerant** | **Tất cả GPIO chỉ chịu 3.3V** |
| Giao tiếp cảm biến 5V | Trực tiếp, không cần thêm linh kiện | Cần level-shifter → thêm độ trễ và điểm lỗi phần cứng |

---

## 4. Phân tích điểm yếu — Tầng 2: Trung cấp

> *Những điểm này ảnh hưởng trực tiếp đến hành vi hệ thống thời gian thực trong điều kiện vận hành thực tế.*

### 4.1 Tính xác định thời gian (Determinism) — Điểm yếu nghiêm trọng nhất của ESP32-S3

Đây là điểm **quan trọng nhất** thường bị bỏ qua khi ca ngợi ESP32-S3.

**Cache Miss:** ESP32-S3 có ICache và DCache. Lần đầu thực thi một hàm DSP, cache chưa có dữ liệu → phải fetch từ Flash qua bus → **độ trễ tăng đột biến không đoán trước được**, có thể lên 50–100µs cho lần đầu tiên.

**WiFi/BT Radio Interrupt:** Khi WiFi đang hoạt động, radio interrupt có thể chiếm CPU **lên đến 2–3ms liên tục** để xử lý gói tin — xảy ra ngẫu nhiên, không theo lịch. Trong hệ thống điều khiển 4kHz (250µs/vòng), một sự kiện 2ms nghĩa là **bỏ lỡ 8 chu kỳ điều khiển liên tiếp**.

```
ESP32-S3:  |--5µs--|-------2000µs WiFi ISR-------|--5µs--|  ← jitter không thể đoán
STM32F103: |--200µs--|--200µs--|--200µs--|--200µs--|       ← chậm nhưng hoàn toàn đều
```

**STM32F103 ngược lại:** Không có cache, không có WiFi → mọi lệnh thực thi với độ trễ **tuyệt đối xác định (deterministic)**. Đây là lý do các hệ thống safety-critical (y tế, ô tô, hàng không) vẫn dùng Cortex-M3/M4 dù chậm hơn nhiều.

---

### 4.2 Chất lượng ADC — ESP32-S3 yếu hơn trong đo lường analog

| | STM32F103C8T6 | ESP32-S3 |
|---|---|---|
| Độ phân giải | 12-bit | 12-bit (thực tế ~11-bit do noise) |
| Tốc độ lấy mẫu | **~1 MSPS** | **~83 kSPS** (chậm hơn 12 lần) |
| Tuyến tính | Tốt, ổn định | Kém ở dải điện áp thấp (phi tuyến đáng kể) |
| Noise | Thấp | **Cao hơn đáng kể** |

**Hệ quả:** Nếu hệ thống đọc back-EMF hoặc dòng điện động cơ qua ADC, ESP32-S3 có thể là **điểm yếu phần cứng**, không phải điểm mạnh. STM32F103 thường được ưu tiên cho các ứng dụng đo lường analog chính xác.

> 📝 **Làm rõ phạm vi ảnh hưởng (bổ sung từ góp ý Gemini):** Trong bài toán cân bằng không gian 3 chiều, dữ liệu gia tốc và góc từ IMU đã được số hóa sẵn bên trong cảm biến và truyền qua I2C/SPI dưới dạng số nguyên — ADC của ESP32-S3 **không tham gia vào luồng dữ liệu này**. ADC yếu chỉ thực sự là vấn đề khi bạn dùng ESP32-S3 để đo trực tiếp:
> - **Dòng điện động cơ** (để làm Current-loop PID — vòng điều khiển dòng điện)
> - **Điện áp pin Li-Po** (để theo dõi mức pin)
>
> Nếu dự án không có hai nhu cầu trên, nhược điểm ADC của ESP32-S3 ít ảnh hưởng đến hiệu năng điều khiển thực tế.

---

### 4.3 DMA và xung đột bus ngoại vi

| | STM32F103C8T6 | ESP32-S3 |
|---|---|---|
| Kênh DMA | 7 kênh (DMA1), độc lập | Có, nhưng chia sẻ với WiFi/USB DMA controller |
| Xung đột DMA | Thấp — ít ngoại vi cạnh tranh | **Cao** khi WiFi + SPI + I2C cùng hoạt động |
| I2C tốc độ cao | Fast-mode 400kHz | Fast-mode 400kHz + Ultra-Fast 1MHz |

> ⚠️ **I2C Timeout khi chạy song song WiFi (bổ sung từ góp ý Gemini):** Khi I2C chạy ở 400kHz đồng thời với WiFi đang hoạt động, ESP-IDF đôi khi gặp hiện tượng **I2C timeout / clock stretching lỗi** — nguyên nhân là WiFi ISR trễ làm chip không kịp phản hồi tín hiệu đồng hồ I2C, dẫn đến mất gói dữ liệu cảm biến IMU.
>
> **Giải pháp:** Khai báo hàm xử lý ngắt I2C với thuộc tính `IRAM_ATTR` trong ESP-IDF, buộc hàm đó nạp vào RAM thay vì Flash — giúp chip thực thi ngắt nhanh hơn đáng kể kể cả khi WiFi đang chiếm bus:
> ```c
> // ✅ Đặt ISR vào RAM để tránh cache miss khi WiFi đang bận
> void IRAM_ATTR i2c_isr_handler(void *arg) {
>     // xử lý ngắt I2C
> }
> ```

---

### 4.4 Công cụ Debug và Profiling

| | STM32F103C8T6 | ESP32-S3 |
|---|---|---|
| Debug interface | SWD/JTAG — kết nối ST-Link, debug cycle nhanh | JTAG — cần OpenOCD, cấu hình phức tạp hơn |
| Profiling chu kỳ CPU | Dễ — dùng DWT cycle counter (1 lệnh) | Khó hơn — cần Xtensa-specific profiling tools |
| Phân tích stack overflow | Compiler flag `-fstack-usage` + linker script | Tương tự nhưng PSRAM/IRAM memory mapping dễ nhầm |
| Kiểm chứng thời gian thực | Logic analyzer trên GPIO — toggle 1 lệnh | Tương tự, nhưng WiFi ISR có thể "nuốt" GPIO toggle |

---

## 5. Phân tích điểm yếu — Tầng 3: Phức tạp

> *Những vấn đề này chỉ bộc lộ khi hệ thống ở quy mô lớn hoặc điều kiện vận hành khắc nghiệt.*

### 5.1 Mô hình Dual-Core của ESP32-S3 — Con dao hai lưỡi

Thường được trình bày như lợi thế ("Core 1 chạy điều khiển, Core 0 chạy WiFi"), nhưng thực tế phức tạp hơn:

**Shared memory bus:** Hai core tranh chấp băng thông bộ nhớ. Khi Core 0 đang DMA lớn cho WiFi, Core 1 truy cập RAM bị **stall** (dừng chờ bus) — không có cảnh báo, không có lỗi, chỉ có độ trễ không thể đoán.

**Mutex/Spinlock overhead:** Bất kỳ dữ liệu nào chia sẻ giữa 2 core (ví dụ: góc IMU đưa lên Web dashboard) đều cần lock. Nếu thiết kế sai, Core 1 thời gian thực bị block bởi Core 0 đang giữ mutex.

**FreeRTOS scheduler jitter:** Trên ESP-IDF, FreeRTOS tick mặc định là **1ms** → task thời gian thực vẫn bị scheduler overhead mỗi 1ms — gấp 4 lần chu kỳ điều khiển 4kHz.

---

### 5.2 Fixed-Point Arithmetic — Giải pháp hiệu quả nhất cho STM32F103 thường bị bỏ qua

Khi nói STM32F103 thiếu FPU, giải pháp thực tế nhất thường không được đề xuất là: **Q-format Fixed-Point Arithmetic** (Q15, Q31).

CMSIS-DSP có toàn bộ bộ hàm fixed-point (`arm_mat_mult_q31`, bộ lọc Q15...) chạy **hoàn toàn bằng integer ALU** của Cortex-M3, với tốc độ gần bằng Cortex-M4 có FPU.

| Phương pháp | Tốc độ trên M3 | Độ chính xác | Độ khó |
|---|---|---|---|
| `float` (SW emulation) | Chậm (hàng nghìn chu kỳ) | Cao | Dễ viết |
| `double` (SW emulation) | Rất chậm (gấp đôi float) | Rất cao | Dễ viết, dễ nhầm |
| Q31 Fixed-Point | **Nhanh** (vài chục chu kỳ) | Đủ dùng | Cần hiểu Q-format scaling |

**Yêu cầu kỹ thuật:** Phải hiểu **Q-format scaling** để tránh overflow — đây là kỹ năng DSP cơ bản nhưng thường bị bỏ qua trong tài liệu nhúng.

---

### 5.3 Thermal Throttling của ESP32-S3 ở 240MHz

Ở xung nhịp 240MHz liên tục, ESP32-S3 **tự động giảm xung** khi nhiệt độ vượt ngưỡng — đặc biệt trong vỏ hộp kín hoặc môi trường nóng như cạnh động cơ.

| Điều kiện | Xung nhịp thực tế | Thời gian tính toán |
|---|---|---|
| Phòng lab (25°C, không tải WiFi) | 240 MHz | ~5µs/vòng Kalman |
| Môi trường nóng / WiFi liên tục | Throttle xuống 160–80 MHz | ~9–15µs/vòng (tăng 1.5–3×) |

**STM32F103 ngược lại:** Không có cơ chế throttling → hiệu năng **hoàn toàn dự đoán được** ở mọi nhiệt độ trong dải hoạt động (-40°C đến 85°C).

---

## 6. Bài toán định mức thời gian thực tế

### Yêu cầu hệ thống điều khiển động cơ 4kHz

Nếu PWM băm ở tần số 4kHz, toàn bộ vòng lặp phải hoàn tất trong **250µs**.

Trong 250µs đó, vi điều khiển phải tuần tự thực hiện:

```
[Đọc IMU qua I2C] → [Chạy Kalman Filter] → [Chạy PID] → [Cập nhật PWM]
```

### Kết quả trên STM32F103C8T6

```
Đọc IMU (I2C 400kHz, 14 byte): ~30µs
Kalman Filter (SW float emulation): ~150–200µs  ← chiếm 60–80% budget!
PID controller (SW float):          ~20µs
Cập nhật Timer PWM:                 ~1µs
─────────────────────────────────────────────
TỔNG:                               ~200–250µs  ← sát hoặc vượt deadline!
```

Khi vượt 250µs: xung PWM bị gián đoạn → động cơ nhận lệnh trễ → rung lắc hoặc mất kiểm soát.

**Giải pháp trên STM32F103:** Thay Kalman Filter bằng **Q31 Fixed-Point** hoặc **Complementary Filter** để giải phóng ~170µs trong budget.

### Kết quả trên ESP32-S3 Supermini (điều kiện lý tưởng)

```
Đọc IMU (I2C 400kHz, 14 byte): ~30µs
Kalman Filter (HW FPU + ESP-DSP): ~5–10µs
PID controller (HW float):         ~1µs
Cập nhật Timer PWM:                ~1µs
─────────────────────────────────────────────
TỔNG:                              ~37–42µs  ← còn dư ~208µs
```

**Tuy nhiên** — khi WiFi ISR xảy ra đột ngột (2–3ms), hệ thống bỏ lỡ 8–12 chu kỳ điều khiển liên tiếp dù trung bình vẫn rất nhanh.

---

## 7. Chiến lược thiết kế Firmware

### 7.1 Phân cấp thuật toán theo phần cứng

Thiết kế macro để tự động chọn thuật toán phù hợp với từng target:

```c
#ifdef TARGET_STM32F103
    // Dùng Q31 fixed-point hoặc Complementary Filter
    complementary_filter_update(&angle, gyro, accel, dt);

#elif defined(TARGET_ESP32S3)
    // Dùng Kalman Filter đầy đủ với FPU
    kalman_filter_update(&state, measurement);
#endif
```

### 7.2 Tận dụng `constexpr` để tính toán trước ma trận tĩnh

Dùng `constexpr` từ C++20/C++23 để tính toán **tại thời điểm biên dịch (compile-time)** các ma trận không đổi:

```cpp
// Tính tại compile-time, không tốn chu kỳ CPU lúc runtime
constexpr float Q_matrix[3][3] = compute_process_noise_matrix(SIGMA_GYRO);
constexpr float R_matrix[2][2] = compute_measurement_noise_matrix(SIGMA_ACCEL);
```

Điều này giúp STM32F103 tiết kiệm đáng kể chu kỳ máy quý giá lúc runtime.

> ⚠️ **Giới hạn quan trọng (bổ sung từ góp ý Gemini):** `constexpr` chỉ áp dụng được cho các ma trận **tĩnh** — tức là những giá trị được xác định trước và không thay đổi trong suốt vòng đời hệ thống, như nhiễu hệ thống $Q$ hay nhiễu cảm biến $R$. Các ma trận **động học** cần cập nhật liên tục theo từng chu kỳ điều khiển — điển hình là ma trận hiệp phương sai $P$ và hệ số Kalman $K$ — vẫn bắt buộc phải tính bằng CPU ở runtime, không thể dùng `constexpr`.

### 7.3 Tránh bẫy `double` ngầm định

```c
// ❌ Các hằng số này ngầm là double → kéo cả biểu thức sang SW emulation
float angle = gyro_raw * 0.0174533 + offset * 1.0;

// ✅ Luôn thêm hậu tố 'f' cho hằng số float
float angle = gyro_raw * 0.0174533f + offset * 1.0f;
```

Thêm compiler flag để bắt lỗi này sớm:
```makefile
CFLAGS += -Wdouble-promotion -Wfloat-conversion
```

### 7.4 Cô lập WiFi khỏi vòng lặp thời gian thực (ESP32-S3)

```cpp
// Core 1: vòng lặp điều khiển thời gian thực (PINNED)
xTaskCreatePinnedToCore(control_loop_task, "control", 4096,
                        NULL, configMAX_PRIORITIES - 1, NULL, 1);

// Core 0: WiFi, logging, UI (không pin core)
xTaskCreate(wifi_task, "wifi", 8192, NULL, 5, NULL);

// Chia sẻ dữ liệu an toàn qua queue (không mutex spinlock)
xQueueSend(telemetry_queue, &sensor_data, 0); // non-blocking
```

---

## 8. Tóm tắt ưu tiên xử lý

| Mức độ | Vấn đề | Chip bị ảnh hưởng | Giải pháp |
|---|---|---|---|
| 🔴 Nghiêm trọng | WiFi ISR jitter (2–3ms đột ngột) | ESP32-S3 | Tách WiFi sang Core 0, dùng Queue |
| 🔴 Nghiêm trọng | RAM 20KB, nguy cơ stack overflow DSP | STM32F103 | Giảm bậc ma trận, dùng fixed-point |
| 🟠 Quan trọng | Fixed-point Q31 — giải pháp thay thế FPU | STM32F103 | Dùng CMSIS-DSP Q31 functions |
| 🟠 Quan trọng | ADC noise cao, tốc độ thấp (~83kSPS) | ESP32-S3 | Thêm hardware filter, hoặc dùng ADC ngoài |
| 🟡 Cần lưu ý | Thermal throttling ở 240MHz liên tục | ESP32-S3 | Tản nhiệt, giới hạn xung hoặc dùng 160MHz |
| 🟡 Cần lưu ý | Bẫy `double` ngầm trong C/C++ | Cả hai | Thêm compiler flag `-Wdouble-promotion` |
| 🟡 Cần lưu ý | DMA bus contention khi multi-peripheral | ESP32-S3 | Lập lịch DMA không chồng chéo |
| 🟢 Nâng cao | Dual-core shared bus stall + mutex deadlock | ESP32-S3 | Dùng lock-free queue giữa các core |
| 🟢 Nâng cao | Cache miss gây độ trễ lần đầu (50–100µs) | ESP32-S3 | Warm-up cache trước khi vào vòng điều khiển |
| 🟢 Nâng cao | I2C timeout khi WiFi + I2C 400kHz chạy song song | ESP32-S3 | Khai báo ISR với `IRAM_ATTR` |

---

## Kết luận nhanh

| Tình huống | Chọn |
|---|---|
| Điều khiển thời gian thực cứng (hard real-time), deadline tuyệt đối | **STM32F103** (sau khi tối ưu Q31) |
| Đo lường analog chính xác (ADC chất lượng cao) | **STM32F103** |
| Tính toán AI/ML edge, Kalman phức tạp, kết nối không dây | **ESP32-S3** |
| Prototype nhanh, không yêu cầu determinism tuyệt đối | **ESP32-S3** |
| Môi trường nhiệt độ cao, vỏ hộp kín cạnh động cơ | **STM32F103** (không throttle) |

> **Nguyên tắc chung:** ESP32-S3 *nhanh hơn trung bình* nhưng *kém xác định hơn*. STM32F103 *chậm hơn trung bình* nhưng *hoàn toàn xác định*. Với hệ thống điều khiển, **tính xác định thường quan trọng hơn tốc độ trung bình**.