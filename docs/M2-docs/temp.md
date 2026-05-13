# Nhật kí đánh giá báo cáo Capstone Project (M2)

## 1. Tóm tắt nội dung từng mục

### 1.1. Introduction
- Mục tiêu: phát triển drone mini 6DOF giá rẻ, nguồn mở cho giáo dục STEM.
- Kết quả M1: khung 3D, động cơ 8520, Kalman cơ bản, ESP32-S3.
- Mục tiêu M2: đạt trạng thái cất cánh ổn định, khắc phục dao động, chưa tích hợp VL53L1X và PMW3901 cho đến khi bay ổn định.

### 1.2. Proposed Methods
- **2.1 Hardware interface**: ESP32-S3, bảng GPIO chi tiết cho 4 động cơ (PWM), MPU6050 (I2C), VL53L1X, ELRS receiver (6 kênh PWM), PMW3901 (SPI), LED RGB.
- **2.2 Voltage acquisition**: Phân áp 2×100kΩ, hạ 4.2V xuống 2.1V, ADC raw ≈ 2774 khi đầy pin. LED báo mức xanh từ 2544–2774 (4.2V–3.85V).
- **2.3 Thiết kế khung 3D**: Shapr3D, PETG, cảm biến VL53L1X và PMW3901 đặt ở tâm đáy.
- **2.4 Mosfet motor driver**: Thiết kế AO3400, mô phỏng dòng 1.4A. So sánh mô phỏng (4.2V, 50%) và thực tế (4.6V, 50%) – kết quả phù hợp.
- **2.5 ELRS receiver**: Thử nghiệm 2 ESP32-S3 (TX-RX) qua 6 dây song song, sau đó thay bằng module ELRS (ESP32C3 + SX1281). Tay cầm Radiomaster Boxer, 6 kênh: Throttle, Pitch, Roll, Yaw, AUX1 (Arm), AUX2 (Mode).
- **2.6 Phân công nhiệm vụ**: Bảng phân công 7 phase từ 07/04 đến 15/06. Các task đã xong: nghiên cứu, chọn phần cứng, lắp ráp, driver MOSFET, ELRS, battery monitor. Đang thực hiện: Kalman, PID.

### 1.3. Up-to-date Results
- **3.1 Hardware**: Drone cỡ lòng bàn tay, khung PETG, động cơ 8520, pin trung tâm, giá đỡ cảm biến đáy hoàn chỉnh.
- **3.2 Battery indicator**: ADC đo được 2375 → tương ứng 4.10V, sai lệch 0.10V so với đồng hồ vạn năng.
- **3.3 ELRS integration**: Đo xung PWM 1500µs (50% throttle) và 2000µs (100% throttle) thành công.

### 1.4. Future Work
- **4.1 Task còn lại**: Implement optical flow (18/05–31/05), thử nghiệm bay (01/06–10/06), viết báo cáo cuối (11/06–15/06).
- **4.2 Cải thiện thuật toán**: Tinh chỉnh PID, lọc Kalman, cắt động cơ khi nghiêng >45°, thu thập telemetry.

### 1.5. References
Danh sách 6 tài liệu (Kalman, PID, drone STEM, datasheet ELRS).

---

## 2. Các mâu thuẫn trong từng mục

### 2.1. Mâu thuẫn chính (giữa mục 2.2 và 3.2 – giá trị ADC và điện áp pin)

| Mục | Lý thuyết / đo đạc | Giá trị |
|------|-------------------|---------|
| 2.2 (lý thuyết) | Vbat=4.2V → Vout=2.1V → ADC raw = (2.1/3.1)×4095 ≈ **2774** | Mức xanh LED: 2544–2774 (4.2V–3.85V) |
| 3.2 (thực tế) | ADC raw = **2375** → tương ứng **4.10V** (sai lệch 0.10V so với đồng hồ) |  |

**Phân tích mâu thuẫn:**
- Với ADC = 2375 → Vin(ADC) = (2375/4095)×3.1 ≈ **1.80V**.
- Từ phân áp hệ số 0.5 → Vbat = 1.80×2 = **3.60V**, không phải 4.10V.
- Nếu Vbat = 4.10V → Vout = 2.05V → ADC raw = (2.05/3.1)×4095 ≈ **2707**, không phải 2375.

**Kết luận:** Dữ liệu thực tế (ADC=2375) và khẳng định pin đang 4.10V là **không nhất quán** với công thức phân áp đã trình bày. Cần kiểm tra lại phép đo hoặc tính toán hệ số phân áp thực tế.

### 2.2. Các mâu thuẫn / lỗi hình thức khác

| Mô tả | Vị trí |
|-------|--------|
| Số Figure trùng (Figure 9 vừa là sơ đồ mạch motor driver vừa là wiring diagram dual-platform) | Trang 9 và trang 15 |
| Số Table trùng (Table 3 vừa là so sánh mô phỏng vs thực tế motor driver, vừa là sequence truyền tín hiệu PWM song song) | Trang 14 và trang 17 |
| Typo: “2744/4095” (đúng ra 2774) | Trang 6 |
| Điều kiện so sánh không đồng nhất: mô phỏng 4.2V, thực tế 4.6V → không thể kết luận công suất phù hợp một cách định lượng chính xác | Mục 2.4, Table 3 |

---

## 3. Kết luận chung

Báo cáo nhìn chung có cấu trúc rõ ràng, nội dung kỹ thuật chi tiết. **Tồn tại một mâu thuẫn quan trọng về số liệu ADC và điện áp pin giữa lý thuyết (2.2) và thực nghiệm (3.2).** Cần kiểm tra lại phép đo hoặc hiệu chỉnh công thức phân áp trước khi tiếp tục các bước tích hợp cảm biến quang học và laser. Các lỗi hình thức (trùng figure/table, typo) nên được sửa trước khi nộp báo cáo chính thức.