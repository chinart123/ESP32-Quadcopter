# PMW3901 Optical Flow Sensor — Tài liệu kỹ thuật đầy đủ

**Platform:** ESP32-S3 Supermini  
**Sensor:** PMW3901 (SPI Interface)  
**Mục tiêu:** Đọc ảnh 35×35 thật từ môi trường + tracking hướng di chuyển  
**Trạng thái:** ✅ Frame Grab pipeline hoạt động thành công

---

## Mục lục

1. [Bức tranh toàn cảnh](#1-bức-tranh-toàn-cảnh)
2. [PMW3901 hoạt động như thế nào](#2-pmw3901-hoạt-động-như-thế-nào)
3. [Vai trò của ESP32-S3](#3-vai-trò-của-esp32-s3)
4. [Đấu dây phần cứng](#4-đấu-dây-phần-cứng)
5. [Luồng dữ liệu đầy đủ](#5-luồng-dữ-liệu-đầy-đủ)
6. [Kiến trúc code cuối cùng](#6-kiến-trúc-code-cuối-cùng)
7. [Nhật ký debug — từng lỗi và cách fix](#7-nhật-ký-debug--từng-lỗi-và-cách-fix)
8. [Nguồn tài liệu tham khảo](#8-nguồn-tài-liệu-tham-khảo)
9. [Q&A — câu hỏi khái niệm quan trọng](#9-qa--câu-hỏi-khái-niệm-quan-trọng)
10. [Hướng phát triển tiếp theo](#10-hướng-phát-triển-tiếp-theo)
11. [Checklist debug nhanh](#11-checklist-debug-nhanh)

---

## 1. Bức tranh toàn cảnh

### Hệ thống gồm những gì?

```
[PMW3901]  ---SPI--->  [ESP32-S3]  ---USB--->  [Máy tính: Processing App]
 Cảm biến              Cầu nối                  Hiển thị ảnh 35×35
 quang học             dịch ngôn ngữ
```

| Thành phần | Vai trò |
|-----------|---------|
| PMW3901 | Chụp ảnh bề mặt liên tục, tính toán chuyển động |
| ESP32-S3 | Nhận data qua SPI, chuyển tiếp qua USB (không xử lý gì thêm) |
| Processing App | Nhận byte stream từ USB, vẽ ảnh 35×35 lên màn hình |

### Tiến độ 3 giai đoạn

```
Giai đoạn 1: Test pipeline (data ngẫu nhiên)     ✅ Hoàn thành
Giai đoạn 2: Đọc ảnh thật từ PMW3901             ✅ Hoàn thành
Giai đoạn 3: Tích hợp vào drone / quadcopter     ⏳ Chưa bắt đầu
```

---

## 2. PMW3901 hoạt động như thế nào

### 2.1 Bên trong chip có 3 khối

**Khối 1 — Đèn LED hồng ngoại:** Tự chiếu sáng bề mặt phía dưới. Nếu không bật đèn này trong lúc khởi tạo, cảm biến mù hoàn toàn — ảnh sẽ ra toàn đen.

**Khối 2 — Ma trận quang học 35×35:** Lưới 1225 điểm cảm biến ánh sáng. Chụp liên tục từ 100 đến 500 khung/giây. Mỗi pixel là một số từ 0 (tối) đến 255 (sáng).

**Khối 3 — DSP Engine:** So sánh frame hiện tại với frame trước, tính ra Delta X và Delta Y — bề mặt đã trượt bao nhiêu pixel theo mỗi chiều.

### 2.2 Hai chế độ hoạt động

Chip **luôn chụp ảnh** ở cả hai chế độ. Sự khác biệt là dữ liệu thô đi đâu sau khi được chụp:

```
Ma trận quang học 35×35  (LUÔN chụp)
           │
           ▼
     Frame Buffer (SRAM nội bộ)
      ┌────┴────┐
      │         │
Motion Mode    Frame Grab Mode
      │         │
      ▼         ▼
DSP tính       Bạn đọc được
Delta X/Y      1225 bytes thật
rồi XÓA frame  qua SPI
```

| Chế độ | Output | Thanh ghi | Dùng khi nào |
|--------|--------|-----------|--------------|
| **Motion Mode** (mặc định) | Delta X, Delta Y | `0x03`, `0x04` | Drone đang bay |
| **Frame Grab Mode** | Mảng ảnh 35×35 | `0x58` | Debug, kiểm tra lens |

### 2.3 Tại sao không chạy song song hai chế độ?

**Lý do 1 — Xung đột bộ nhớ:** Frame Buffer chỉ có một cổng đọc/ghi. DSP và SPI không thể truy cập cùng lúc. Khi DSP đang ghi đè ở tốc độ ~1000 lần nhanh hơn SPI, dữ liệu bạn đọc về sẽ bị xé — nửa frame cũ, nửa frame mới.

**Lý do 2 — Tốc độ không tương thích:** DSP tiêu thụ một frame trong vài micro-giây. Đọc 1225 bytes qua SPI cần ~5ms. Không tạm dừng DSP thì frame bị ghi đè liên tục trong lúc đọc.

**Lý do 3 — Chip được thiết kế cho drone, không phải camera:** Motion Mode là sản phẩm chính. Frame Grab là tính năng debug được Pixart thêm vào sau, chủ yếu để kỹ sư kiểm tra lens.

> **Hệ quả thực tế:** Khi drone bay thật, phải ở Motion Mode. Frame Grab chỉ dùng khi drone đứng yên để debug. Đây là ràng buộc vật lý cứng, không phải lời khuyên tùy chọn.

### 2.4 Ảnh bàn cờ Checkerboard là gì?

Khi thấy ảnh bàn cờ trên Processing, chip đang ở **Motion Mode** (chế độ mặc định khi bật nguồn). Thanh ghi `0x58` bị khóa và trả về test pattern cứng. Không phải lỗi phần cứng — chip chưa nhận được lệnh chuyển sang Frame Grab Mode.

### 2.5 Encoding pixel từ thanh ghi 0x58

Đây là điểm phức tạp nhất, khác hoàn toàn so với burst read thông thường.

M��i lần đọc thanh ghi `0x58` trả về 1 byte với 2 bits trạng thái ở trên cùng:

```
bits[7:6] = 00 hoặc 11  →  invalid, đọc lại
bits[7:6] = 01          →  byte này chứa 6 bits TRÊN của pixel (bits[5:0])
bits[7:6] = 10          →  byte này chứa 2 bits DƯỚI của pixel (bits[3:2])
```

Để lấy 1 pixel hoàn chỉnh (8-bit):
```
1. Đọc 0x58 cho đến khi bits[7:6] = 01  →  lưu vào 'a'
2. Đọc 0x58 lần nữa (lúc này bits[7:6] = 10)  →  lưu vào 'b'
3. pixel = (a << 2) | (b & 0x0C)
```

---

## 3. Vai trò của ESP32-S3

ESP32-S3 là **cầu nối bắt buộc** — PMW3901 chỉ nói được SPI, máy tính chỉ có USB.

Trong mỗi vòng lặp, ESP32-S3 làm đúng 3 việc:

```
1. Gọi enableFrameBuffer() một lần trong setup()
   → Tắt DSP, mở khóa Frame Buffer

2. Kích hoạt capture: regWrite(0x58, 0xFF)

3. Đọc 1225 pixels từ 0x58 (mỗi pixel cần 2 lần đọc)

4. Gửi qua USB: '*' + 1225 bytes
```

ESP32-S3 không xử lý, không tính toán hình ảnh — chỉ dịch ngôn ngữ SPI → USB.

---

## 4. Đấu dây phần cứng

### Sơ đồ kết nối

```
PMW3901          ESP32-S3 Supermini
───────          ──────────────────
VCC      ──────► 3V3   (⚠️ KHÔNG dùng 5V — hỏng chip ngay lập tức)
GND      ──────► GND   (bắt buộc nối chung GND)
CLK      ──────► GPIO 7   (CLK = SCK = SCLK, chỉ khác tên)
MOSI     ──────► GPIO 5   (một số module ghi là SDI)
MISO     ──────► GPIO 6   (một số module ghi là SDO)
CS       ──────► GPIO 4   (một số module ghi là NCS hoặc CSN)
```

### Cấu hình SPI

```cpp
SPI.begin(PMW_SCK_PIN, PMW_MISO_PIN, PMW_MOSI_PIN, PMW_CS_PIN);
// Speed: 4MHz | Mode: SPI_MODE3 | Bit order: MSBFIRST
```

> **Quan trọng:** Gọi `SPI.begin()` với custom pins **trước** `flow.begin()`. Nếu gọi ngược lại, Bitcraze library sẽ reset về default pins của ESP32-S3.

---

## 5. Luồng dữ liệu đầy đủ

```
setup():
  SPI.begin(custom pins)
  flow.begin()           ← Bitcraze lo toàn bộ Power-up Sequence
       │
       ├─► CS toggle: HIGH→LOW→HIGH (reset chip về trạng thái xác định)
       ├─► Power-on reset: write 0x5A vào 0x3A
       ├─► Đọc dọn rác: đọc 0x02→0x06
       ├─► Nạp ~60 lệnh cấu hình thanh ghi
       └─► Bật LED hồng ngoại

  enableFrameBuffer()    ← tự implement với timeout
       │
       └─► Ghi 11 lệnh, trigger capture đầu tiên (0x58=0xFF)
           Chờ chip báo sẵn sàng (có timeout)

loop():
  regWrite(0x58, 0xFF)   ← trigger capture
  
  for 1225 pixels:
    poll 0x58 cho đến bits[7:6] = 01
    đọc 0x58 lần nữa lấy bits dưới
    ghép thành pixel 8-bit
  
  Serial.write('*')
  Serial.write(buffer, 1225)
  
  re-arm: regWrite(0x70, 0x00) + regWrite(0x58, 0xFF)
```

### Giao thức truyền lên Processing

```
ESP32 gửi: [*] [byte_0] [byte_1] ... [byte_1224]
            │   └──────────────────────────────┘
          Header            1225 bytes payload
```

Processing dùng State Machine:
- **Trạng thái 1:** Tìm header `'*'`
- **Trạng thái 2:** Đếm đủ 1225 bytes → gọi `drawFrame()`

---

## 6. Kiến trúc code cuối cùng

**File:** `Crazyflie-PMW3901-v1.ino`

### Cấu trúc 3 tầng

```
Tầng 1: SPI Low-level
  regWrite()       — 50µs trước/sau byte, 200µs sau CS HIGH
  regRead()        — 50µs tSRAD, 100µs sau data, 200µs sau CS HIGH

Tầng 2: Mode Management  
  enableFrameBuffer()  — ghi 11 lệnh + chờ chip ready (có timeout)

Tầng 3: Frame Capture
  readFrame()      — 1225 pixels × 2 reads mỗi pixel, có timeout
```

### Timing SPI (lấy từ source thật Bitcraze library)

| Thông số | Giá trị | Ý nghĩa |
|----------|---------|---------|
| SPI Speed | 4 MHz | Tốc độ clock |
| Trước address byte (CS LOW → clock) | 50µs | Ổn định CS |
| tSRAD (address → data byte) | 50µs | Chip chuẩn bị data |
| Sau data byte | 100µs | Chip latch xong |
| Sau CS HIGH | 200µs | Khoảng cách giữa giao dịch |

### Tại sao mỗi hàm phải có `beginTransaction/endTransaction` riêng?

```cpp
// ❌ Sai — gọi một lần trong setup(), để nguyên
SPI.beginTransaction(...);  // trong setup()
// ... 1000 lệnh sau ...
regWrite(...);  // SPI đang ở trạng thái undefined

// ✅ Đúng — bọc trong từng hàm
void regWrite(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(...);
  // ...giao dịch...
  SPI.endTransaction();
}
```

---

## 7. Nhật ký debug — từng lỗi và cách fix

### Lỗi 1: Product_ID = 0xFF (Sanity Check thất bại)

**Triệu chứng:** Serial Monitor in `Product_ID = 0xFF -> FAIL`

**Nguyên nhân gốc rễ:** `SPI.beginTransaction()` được gọi một lần trong `setup()` rồi không bao giờ gọi `endTransaction()`. SPI bus bị treo trạng thái undefined → tất cả reads trả về 0xFF.

**Fix:** Bọc `beginTransaction/endTransaction` trong **từng hàm** `regWrite`/`regRead`, không phải gọi một lần ở ngoài.

---

### Lỗi 2: Ảnh bàn cờ Checkerboard

**Triệu chứng:** Processing hiện ảnh bàn cờ đen trắng đều tăm tắp.

**Nguyên nhân:** Chip đang ở Motion Mode (mặc định). `enableFrameBuffer()` chưa được gọi, hoặc các lệnh ghi vào chip không được latch do vi phạm timing.

**Fix:** Gọi `enableFrameBuffer()` sau `flow.begin()`. Đảm bảo `tSCLK-NCS ≥ 20µs` trong `regWrite`.

---

### Lỗi 3: Màn hình Processing toàn xám đồng nhất

**Triệu chứng:** Processing hiện màu xám đều, không thay đổi dù di chuyển cảm biến.

**Nguyên nhân:** `enableFrameBuffer()` trong Bitcraze library có `do-while` không có timeout:
```cpp
do {
  temp  = registerRead(0x58);
  check = temp >> 6;
} while (check == 0x03);  // ← treo vô tận nếu chip không trả lời đúng
```
`setup()` không bao giờ kết thúc → `loop()` không bao giờ chạy → không có byte nào gửi lên Processing → Processing hiện màu xám mặc định của nó (không phải từ dữ liệu chip).

**Fix:** Tự implement `enableFrameBuffer()` và `readFrame()` với timeout:
```cpp
int timeout = 5000;
do {
  temp  = regRead(0x58);
  check = temp >> 6;
  timeout--;
} while (check == 0x03 && timeout > 0);

if (timeout <= 0) { Serial.println("FAIL: timeout"); return false; }
```

---

### Lỗi 4: Serial Monitor không in gì cả

**Triệu chứng:** Serial Monitor hoàn toàn trắng, kể cả dòng đầu tiên.

**Nguyên nhân:** ESP32-S3 với CDC on Boot enabled sẽ **reset khi Serial Monitor mở**. Với `delay(2000)`, toàn bộ code chạy và in xong trước khi USB CDC enumerate xong → tất cả output bị mất.

**Fix:** Tăng từ `delay(2000)` lên `delay(3000)`. Trên ESP32-S3 Dev Board với CDC on Boot, USB cần ít nhất 2.5–3 giây để ổn định sau reset.

```cpp
// ❌ delay(2000) — quá ngắn với CDC on Boot
// ✅ delay(3000) — đủ thời gian cho USB CDC enumerate
```

---

### Lỗi 5: Đọc pixel từ sai thanh ghi

**Triệu chứng:** Màn hình xám hoặc hiển thị bàn cờ dù `enableFrameBuffer()` đã được gọi.

**Nguyên nhân:** Đọc pixel từ `0x0D` bằng Burst Read — đây là thanh ghi của chế độ khác, không phải Frame Grab. Thanh ghi đúng là `0x58` với validity polling.

**Fix:**
```cpp
// ❌ Sai — burst read từ 0x0D
burstReadRegister(0x0D, frameBuffer, 1225);

// ✅ Đúng — poll từng pixel từ 0x58
for (int i = 0; i < 1225; i++) {
  do { a = regRead(0x58); hold = a >> 6; }
  while (hold == 0x03 || hold == 0x00);
  if (hold == 0x01) {
    b = regRead(0x58);
    pixel = (a << 2) | (b & 0x0C);
    buf[count++] = pixel;
  }
}
```

---

## 8. Nguồn tài liệu tham khảo

### Đánh giá từng nguồn

| Nguồn | URL | Frame Grab? | Đánh giá |
|-------|-----|-------------|----------|
| Bitcraze Arduino Library | github.com/bitcraze/Bitcraze_PMW3901 | ✅ Có | Nguồn chính xác nhất cho Arduino |
| Crazyflie Firmware | github.com/bitcraze/crazyflie-firmware | ❌ Không | Chỉ có Motion Mode, nhưng timing rất có giá trị |
| Pimoroni PAA5100JE | github.com/pimoroni/pmw3901-python | ✅ Có | Chip thế hệ sau, API gần giống |

### Điểm khác biệt quan trọng giữa hai nguồn

| Thông số | Bitcraze Arduino | Crazyflie Firmware |
|----------|-----------------|-------------------|
| SPI Speed | 4 MHz | 2 MHz |
| tSRAD | 50µs | 500µs |
| Sau CS HIGH | 200µs | 200µs |
| Frame Grab | ✅ Có | ❌ Không có |
| CS Toggle | HIGH→LOW→HIGH | HIGH (40ms) → H→L→H |

> **Kết luận:** Crazyflie dùng timing conservative hơn nhiều (500µs vs 50µs) vì chạy trên STM32 với điều kiện nhiễu cao hơn. Arduino ESP32 hoạt động tốt với 50µs.

---

## 9. Q&A — câu hỏi khái niệm quan trọng

**Q: Tại sao PMW3901 có 2 chế độ thay vì luôn xuất ảnh thật?**

A: Chip luôn chụp ảnh ở cả hai chế độ — sự khác biệt là dữ liệu thô đi đâu sau khi chụp. Frame Buffer chỉ có một cổng đọc/ghi. DSP (Motion Mode) và SPI (Frame Grab Mode) không thể cùng truy cập lúc một lúc. Đây là ràng buộc vật lý phần cứng, không phải quyết định thiết kế tùy tiện.

---

**Q: ESP32-S3 đang thực sự làm gì trong hệ thống? Nếu quét môi trường thật thì giao tiếp SPI đang làm gì?**

A: ESP32-S3 là cầu nối bắt buộc — PMW3901 không có USB, máy tính không có SPI. ESP32-S3 không xử lý hay tính toán hình ảnh, chỉ làm 3 việc: nhận lệnh từ máy tính (nếu cần), gửi lệnh SPI sang PMW3901, nhận data SPI về và đẩy qua USB.

---

**Q: Quy trình có phải là: Motion Mode thu thập data → chuyển Frame Grab Mode → lấy data đó ra không?**

A: Không. Hai chế độ xử lý hai loại dữ liệu hoàn toàn độc lập. Frame Grab Mode tự chụp ảnh môi trường ngay lập tức khi được kích hoạt, không cần "mượn" data từ Motion Mode. Để chỉ hiển thị ảnh 35×35, bạn ở Frame Grab Mode mãi — không cần chuyển qua lại.

---

**Q: Tutorial YouTube chỉ hiện X/Y, không hiện ảnh — tại sao?**

A: Vì họ đang dùng Motion Mode — tính năng chính, thiết kế gốc của chip. Frame Grab lên màn hình là usecase rất hiếm, hầu như không có tutorial công khai. Đó là lý do phải tự mò từ source code.

---

**Q: Crazyflie firmware có thể dùng cho Frame Grab không?**

A: Không. `pmw3901.c` trong Crazyflie firmware chỉ có 2 hàm: `pmw3901Init()` và `pmw3901ReadMotion()`. Không có frame capture. Crazyflie chỉ cần Delta X/Y để bay. Tuy nhiên firmware này cung cấp thông tin timing SPI rất có giá trị.

---

**Q: Tại sao `flow.readFrameBuffer()` của Bitcraze library cũng cho màn hình xám?**

A: Vì hàm đó có `do-while` không có timeout. Nếu chip không phản hồi đúng trạng thái, hàm treo vô tận. `setup()` không hoàn thành, `loop()` không bao giờ chạy, không có gì gửi lên Processing → Processing hiện màu xám mặc định của nó.

---

## 10. Hướng phát triển tiếp theo

### Ngã ba 1 — Màn hình OLED có cần không?

```
Không cần  →  Giữ nguyên kiến trúc hiện tại (ESP32 → USB → PC)
Có cần     →  ESP32 phải vừa đọc SPI vừa render OLED
              Cần RTOS hoặc DMA để không block luồng đọc SPI
```

### Ngã ba 2 — Delta X/Y dùng để làm gì?

```
Chỉ hiển thị hướng  →  Đơn giản: đọc 2 thanh ghi 0x03 và 0x04
Đưa vào PID drone   →  Phức tạp: polling rate 100-500Hz
                        Phải tắt Frame Grab khi đang bay
                        Hai chế độ không thể chạy cùng lúc
```

### Ngã ba 3 — Code ghép vào firmware drone không?

```
Không cần  →  Giữ file .ino đơn lẻ
Có cần     →  Refactor: src/pmw3901_driver.cpp + include/pmw3901_driver.h
              Bỏ tư duy viết trong loop() Arduino
              Chuyển sang class/struct quản lý driver
```

---

## 11. Checklist debug nhanh

```
□ Serial Monitor có in dòng đầu tiên không?
     Không → delay() quá ngắn. Tăng lên 3000ms (CDC on Boot cần ≥ 3s)

□ flow.begin() thành công?
     Không → Kiểm tra VCC = 3.3V (không phải 5V), kiểm tra MISO/MOSI không nhầm

□ enableFrameBuffer() thành công?
     Timeout → SPI timing sai, hoặc chip chưa init xong. Thêm delay sau flow.begin()
     
□ Processing hiện màu xám đồng nhất?
     → do-while trong enableFrameBuffer/readFrame không có timeout → treo vô tận

□ Ảnh bàn cờ?
     → Chip vẫn ở Motion Mode. enableFrameBuffer() chưa chạy hoặc không hiệu lực

□ Ảnh toàn đen?
     → LED hồng ngoại chưa bật. Kiểm tra setLed(true) hoặc sequence bật LED trong init

□ Ảnh xám đồng nhất dù frame grab chạy?
     → Cảm biến đang nhìn vào bề mặt quá đồng đều (giấy trắng, tường trắng)
        Thử di chuyển trên bề mặt có họa tiết

□ count < 1225 trong readFrame()?
     → Chip gửi byte "10" trước byte "01" (lệch pha). Re-arm lại bằng cách
        ghi 0x70=0x00 và 0x58=0xFF rồi chờ ready state
        
□ Processing không nhận '*' header?
     → Serial Monitor và Processing đang mở cùng lúc trên cùng COM port
        Chỉ được mở một trong hai
```

---

*Tài liệu tổng hợp từ toàn bộ quá trình phát triển: Optical Sensor v1 → v2 → v3 → Crazyflie-PMW3901-v1*  
*Nguồn tham khảo: Bitcraze_PMW3901 library source, Crazyflie firmware pmw3901.c*