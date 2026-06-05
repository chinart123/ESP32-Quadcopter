# BẢN TỔNG HỢP DEBATE KỸ THUẬT: PMW3901 · VL53L1X · MPU6050
**Người tham gia:** Pham Minh Chien (Chiến) · Vo Quoc Hung (Hưng)  
**Moderator gốc:** Gemini  
**Phân tích bổ sung:** Claude (Project Windify Technical Advisor)  
**Ngày:** Tháng 6, 2026

---

## Vấn đề 1: Số lượng PID và Logic khi drone nghiêng

**Chiến hỏi:**
1. Dùng 3 bộ PID cho ToF, MPU6050 và PMW3901 có tốn CPU không?
2. Hưng muốn "tắt PMW3901 khi drone nghiêng, lưu vị trí, đợi MPU6050 ổn rồi bật lại". Chiến không đồng ý — phải dùng Tilt Compensation liên tục.

**Gemini nhận định:** Chiến đúng hoàn toàn. Việc tắt cảm biến gây Loss of Odometry.

**✅ Claude bổ sung — đúng nhưng chưa đủ ngữ cảnh:**

**Về CPU:** 3 PID loops không tốn tài nguyên. ESP32-S3 FPU xử lý dưới 1% — thiết kế chuẩn của mọi flight controller hiện đại.

**Về tắt PMW3901:** Câu trả lời phụ thuộc vào use case cụ thể:

| Use case | Tắt sensor khi nghiêng? | Lý do |
|----------|------------------------|-------|
| **Yaw Anchoring (v2 hiện tại)** | Tắt được | Gate condition trong code đã tự tắt correction khi Roll lớn. Không tích lũy vị trí → không mất gì |
| **Position Hold (v3 tương lai)** | ❌ Không được | Tắt = mất tracking vị trí (Loss of Odometry thực sự) |

**Kết luận:** Hưng không sai hoàn toàn — chỉ sai ở scope Position Hold. Với Yaw Anchoring (mục tiêu hiện tại của project), logic gate condition đã handle việc này mà không cần tắt sensor cứng.

**Công thức Tilt Compensation cho v3 (mà Gemini không đưa ra):**
```
// Đơn giản (linear approximation, đủ dùng):
real_vx = raw_vx / cos(pitch_rad)
real_vy = raw_vy / cos(roll_rad)

// Đầy đủ (cần rotation matrix từ Euler angles):
// [real_vx]   [R(roll,pitch,yaw)]^(-1)   [raw_vx]
// [real_vy] =                           × [raw_vy]
// [real_vz]                               [raw_vz]
```

---

## Vấn đề 2: Thứ tự phát triển và rủi ro lây nhiễm logic

**Chiến lập luận:** Test ToF + PMW3901 trước bằng tay, sau mới đưa MPU6050 vào. Nếu MPU6050 vào trước, khi có bug không biết nguồn gốc.

**Gemini nhận định:** Chiến đúng ở góc độ bench testing. MPU6050 không kế thừa từ ToF. PMW3901 mới phụ thuộc ToF (cần height để scale). Trình tự flight tuning đúng: MPU6050 → ToF → PMW3901.

**❌ Claude phát hiện mâu thuẫn trong Gemini:**

Gemini vừa validate "trình tự đúng là PMW3901 → ToF → MPU6050" (vì Chiến nói vậy), vừa kết luận "trình tự flight tuning phải là MPU6050 → ToF → PMW3901" — **hai phát biểu ngược nhau** mà không giải thích.

**✅ Phân tích đúng — phân biệt 2 ngữ cảnh:**

| Ngữ cảnh | Thứ tự | Mục đích |
|----------|--------|---------|
| **Bench Testing (kiểm thử tay)** | Sensor nào muốn test thì test trước | Xác nhận phần cứng hoạt động, cô lập bug |
| **Software Architecture** | MPU6050 → VL53L1X → PMW3901 | Inner loop ổn định → outer loops mới có ý nghĩa |
| **Code init trong setup()** | SPI (PMW3901) → I2C_IMU → I2C_TOF | Yêu cầu phần cứng ESP32-S3 (phát hiện thực nghiệm) |

Chiến đúng về bench testing order. Hưng đúng về flight tuning order. Đây là 2 giai đoạn khác nhau — không mâu thuẫn, không cần tranh luận.

**Thêm: "nhiễm logic" mà Chiến lo ngại không xảy ra vì:**
- MPU6050 chạy hoàn toàn độc lập trên I2C\_IMU bus riêng
- ToF chạy độc lập trên I2C\_TOF bus riêng
- PMW3901 chạy trên SPI riêng
- Ba bus không share code path hay biến nào

---

## Vấn đề 3: Vai trò MPU6050 trước khi cất cánh

**Chiến hỏi:** Drone chưa cất cánh, đưa MPW6050 vào để làm gì?

**Gemini nhận định:** MPU6050 phải code trước vì Tilt Compensation cần dữ liệu góc từ nó.

**✅ Claude xác nhận + bổ sung:**

Đúng. Và cụ thể hơn — MPU6050 cần được code trước vì:

1. **Inner loop của flight controller:** Roll/Pitch PID dùng Kalman output từ MPU6050 mỗi 4ms. Không có MPU6050 = drone không thể giữ thăng bằng = không thể bay để test bất cứ thứ gì.

2. **Tilt Compensation cho PMW3901:** Dù chỉ bench test PMW3901, nếu muốn validate kết quả `raw_vx / cos(pitch)`, bạn cần `pitch` từ MPU6050.

3. **smartCalibrate() baseline:** Hàm này đọc 2000 samples để tính offset. Phải chạy trước tất cả sensor khác.

**Nhưng lưu ý:** Với Yaw Anchoring (v2 hiện tại), Tilt Compensation không bắt buộc — `atan2(ΔX, ΔY)` chỉ cần hướng của vector dịch chuyển, không cần magnitude chính xác. Tilt Compensation chỉ bắt buộc ở v3 (Position Hold).

---

## Vấn đề 4: Kiểm chứng MPU6050 khi bench test

**Chiến lập luận:** Khi test trên sàn, thước Eke đo góc còn chuẩn hơn MPU6050.

**Gemini nhận định:** Đúng về thực dụng. Tham chiếu là trọng lực (1G). `smartCalibrate()` tự khử bias.

**✅ Claude xác nhận + thêm quy trình cụ thể:**

Chiến đúng — đây là phương pháp kiểm chứng chuẩn trong embedded systems gọi là **Hardware-In-The-Loop validation**.

**Quy trình kiểm chứng MPU6050 thực tế:**

```
1. Đặt drone phẳng hoàn toàn (dùng thước thủy):
   → Serial phải in: Roll ≈ 0°, Pitch ≈ 0°, Gyro XYZ ≈ 0 dps

2. Nghiêng drone 30° quanh Roll axis (đo bằng thước Eke):
   → Serial phải in: Roll ≈ 30°, Pitch ≈ 0°

3. Nghiêng drone 45° quanh Pitch axis:
   → Serial phải in: Roll ≈ 0°, Pitch ≈ 45°

4. Xoay drone 90° quanh Yaw axis:
   → Serial phải in: fYaw thay đổi ~90° từ điểm ban đầu
   → Lưu ý: fYaw sẽ drift dần sau đó (expected)
```

Sai số chấp nhận được: ±2° cho Roll/Pitch (sau smartCalibrate).

---

## Tóm tắt phân tích

| Vấn đề | Gemini đúng | Gemini chưa đủ/sai |
|--------|-------------|-------------------|
| 3 PID tốn CPU | ✅ | Không đưa số cụ thể (<1%) |
| Tắt PMW3901 khi nghiêng | ✅ (đúng cho Position Hold) | ❌ Không phân biệt Yaw Anchoring vs Position Hold |
| Thứ tự phát triển | ✅ Cuối cùng đúng | ❌ Mâu thuẫn nội tại, không giải thích 2 ngữ cảnh |
| MPU6050 phải code trước | ✅ | Không giải thích tại sao với use case cụ thể |
| Kiểm chứng bằng thước Eke | ✅ | Không đưa quy trình cụ thể |
| Tilt Compensation formula | ❌ Không đưa ra | Cần bổ sung công thức |

**Điểm Gemini: 6/10** — Đúng về mặt định hướng nhưng không đủ kỹ thuật, tự mâu thuẫn ở vấn đề 2, và không phân biệt Yaw Anchoring vs Position Hold dẫn đến một số kết luận áp dụng sai scope.

**Kết luận cho project Windify:**
- v2 (Yaw Anchoring): không cần Tilt Compensation, không cần turn off sensor — logic gate đã handle
- v3 (Position Hold): cần Tilt Compensation với công thức `real_vx = raw_vx / cos(pitch)`
- Bench testing và Flight tuning có thứ tự riêng — không tranh luận trên cùng ngữ cảnh