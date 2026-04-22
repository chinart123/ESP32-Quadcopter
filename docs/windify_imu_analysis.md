# NHẬT KÝ KỸ THUẬT: PHÂN TÍCH HỆ THỐNG XỬ LÝ TÍN HIỆU IMU MPU6050
**Dự án:** Project Windify (Mini Quadcopter)  
**Nền tảng:** STM32F103C8T6 (Bare-metal) & ESP32-S3 (FreeRTOS)  
**Tác giả:** Phạm Minh Chiến

---

## 1. Giải thích về Luồng Tín hiệu và Sự khác biệt giữa các Sơ đồ

> **Câu hỏi của tôi:** Giải thích vì sao góc Yaw bị làm mờ và chưa được tính? Hình như chỉ có svg1 mới bị thôi, còn svg2 đã tính rồi phải không?

Trong quá trình phát triển, việc quan sát hệ thống qua các sơ đồ giúp xác định rõ giới hạn của phần cứng.

* **Sơ đồ IMU Pipeline (svg1):** Thể hiện chi tiết mức thấp (Low-level). Ở đây, góc Yaw bị làm mờ và chú thích *"Yaw drifts — no absolute ref"* vì sơ đồ này phản ánh đúng bản chất vật lý của cảm biến MPU6050 (6 trục). Nó không có mốc tham chiếu tuyệt đối cho trục Z.
* **Sơ đồ System Overview (svg2):** Thể hiện kiến trúc cấp cao (High-level). Ở đây, Yaw được ghi chung với Roll và Pitch trong khối "Fused Angles" vì đây là luồng dữ liệu cần thiết cho bộ điều khiển PID. Tuy nhiên, giá trị Yaw này thực chất vẫn là giá trị bị trôi được tính từ Gyroscope.  

[Sơ đồ IMU Pipeline chi tiết](../assets/svg1_imu_pipeline.svg)
![Sơ đồ IMU Pipeline chi tiết](../assets/svg1_imu_pipeline.svg)
[Tổng quan Kiến trúc Hệ thống Quadcopter](../assets/svg2_system_overview.svg)
![Tổng quan Kiến trúc Hệ thống Quadcopter](../assets/svg2_system_overview.svg)

---

## 2. Khái niệm "Trôi" (Drift) trong IMU

> **Câu hỏi của tôi:** Tôi chưa hiểu khái niệm 'trôi' ở đây lắm? Có phải điều này đồng nghĩa rằng drone vẫn có thể cân bằng nếu đem ra test thực tế, nhưng sẽ bị trôi sang phương ngang vô định phải không? Vậy là chúng ta sẽ không thể đoán trước được drone sẽ trôi quay vòng sang trái hay phải đúng không?

Cần phân biệt rõ hai loại trôi để tránh nhầm lẫn trong quá trình Tuning:

1.  **Trôi Vị trí (Spatial Drift):** Drone bị dạt sang trái/phải hoặc tiến/lùi do gió hoặc lệch trọng tâm. Điều này được giải quyết bằng GPS hoặc Optical Flow.
2.  **Trôi Góc (Angular/Heading Drift):** Đây là hiện tượng mũi drone tự xoay tròn từ từ quanh trục Z dù không có tác động điều khiển. Hướng trôi (trái hay phải) phụ thuộc hoàn toàn vào dấu của sai số tĩnh ($Bias$) trong phần cứng.

**Tại sao Yaw lại trôi?**
Vì MPU6050 không có la bàn điện tử (Magnetometer), nó không thể biết hướng Bắc ở đâu. Nó chỉ dựa vào con quay hồi chuyển (Gyroscope). Khi tích phân vận tốc góc theo thời gian, sai số nhỏ sẽ cộng dồn lớn dần, khiến vi điều khiển "tưởng" drone đang xoay.

---

## 3. Cấu trúc Phần cứng MEMS: "Người bảo vệ" và "Kẻ mồ côi"

> **Câu hỏi của tôi:** Thực chất cái Accelerometer chứa những cái gì mà được xem 'cố định tuyệt đối và bất biến dựa trên trọng lực trái đất? Cơ chế của la bàn này là gì? Nó giúp giải quyết vấn đề gì?

Gia tốc kế (Accelerometer) là **100% phần cứng cơ khí siêu nhỏ**. Bên trong chip có một "khối lượng chuẩn" treo bằng lò xo silicon. Trọng lực Trái Đất luôn kéo khối lượng này xuống, tạo ra một véc-tơ cố định hướng về tâm Trái Đất.
Trong khi đó, La bàn điện tử (Magnetometer) đo từ trường Trái Đất để tạo mốc phương ngang chỉ về cực Bắc.

* **Roll và Pitch:** Có Accelerometer làm "Người bảo vệ". Trọng lực Trái Đất là mốc tuyệt đối không bao giờ mất đi, giúp bù sai số cho Gyro.
* **Yaw:** Là "Kẻ mồ côi". Do MPU6050 không có La bàn, xoay trên mặt phẳng ngang không làm đổi véc-tơ trọng lực, nên hệ thống hoàn toàn thiếu mốc tham chiếu.  

[Cấu trúc cơ khí MEMS bên trong chip MPU6050](../assets/imu_mems_structure.png)  
![Cấu trúc cơ khí MEMS bên trong chip MPU6050](../assets/imu_mems_structure.png)

---

## 4. Các Công thức Toán học Cốt lõi

> **Câu hỏi của tôi:** Khi vi điều khiển có ghi nhận hành vi trôi Yaw này, lí do vì sao khiến nó không ảnh hưởng các hướng quay còn lại, các công thức nào biểu thị điều đó?

Yaw bị trôi nhưng Roll và Pitch vẫn cân bằng tuyệt đối trên không nhờ sự xuất hiện của Bộ lọc bù (Complementary Filter). Gia tốc kế sẽ "ghim" chặt hai trục này lại.

### Công thức Tích phân góc Yaw (Trục mồ côi)
Góc Yaw được tính bằng cách cộng dồn vận tốc góc ($G_z$) nhân với khoảng thời gian ($dt$). Không có thành phần bù trừ nào ở đây.

$$Yaw_{t} = Yaw_{t-1} + (G_{z\_true} + Bias + Noise) \cdot dt$$

### Công thức Bộ lọc bù (Trục được bảo vệ)
Sử dụng hệ số $\alpha$ (thường là 0.98) để lấy phần mượt mà từ Gyro và phần tuyệt đối ($Accel$) để khử trôi. Lực kéo của $(1 - \alpha) \cdot Accel$ khiến Roll và Pitch miễn nhiễm với trôi.

$$Roll_{new} = \alpha \cdot (Roll_{old} + G_x \cdot dt) + (1 - \alpha) \cdot Accel_{Roll}$$

$$Pitch_{new} = \alpha \cdot (Pitch_{old} + G_y \cdot dt) + (1 - \alpha) \cdot Accel_{Pitch}$$

---

## 5. Chế độ Acro và Cảm biến Phụ trợ

> **Câu hỏi của tôi:** Lí do vì sao bạn tôi để con drone trên trục 3D để test tuning thì nó vẫn cân bằng được nhỉ? Hiện có 2 con cảm biến (VL53L1X, PMW3901) chưa được tích hợp, có phải nó sẽ giải quyết được vấn đề này không?

Nó vẫn cân bằng vì khả năng chống lật không phụ thuộc vào góc Yaw tuyệt đối, mà phụ thuộc vào Rate PID (vận tốc góc). 
Các Pilot chuyên nghiệp thường bay ở chế độ **Acro (Rate Mode)**, tự dùng tay chỉnh hướng mũi. Hai cảm biến còn lại không giải quyết được vấn đề Yaw Drift:
* **VL53L1X (ToF):** Giữ độ cao (Altitude Hold).
* **PMW3901 (Optical Flow):** Giữ vị trí đứng yên tại chỗ (Position Hold).

---

## 6. Bảng Tra cứu Thông số Thuật toán

> **Câu hỏi của tôi:** Lập một bảng ghi chú gồm các cột (STT, Tên biến, Tính chất, Tác nhân, Nguồn gốc, Ghi chú, Vị trí code) cho hệ thống này dựa trên 4 file code.

*(Lưu ý: File `drn_raw_mpu6050` bị bỏ qua trong bảng này vì bản chất file raw không triển khai thuật toán tính Yaw và Bộ lọc bù)*

### Bảng 1: Trục Yaw (Heading) - Kẻ mồ côi

| STT | Tên thông số | Tính chất | Bị ảnh hưởng bởi | Nguồn gốc | Ghi chú cho Newbie | Vị trí / Dòng code |
| :-- | :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | `gyro_z_raw` | Thay đổi | Lực xoay vật lý + Nhiễu | Phần cứng | Dữ liệu thô từ cảm biến, nhảy số liên tục. | `drn_mpu6050.cpp` (Dòng 207) |
| 2 | `gyro_z_offset` | Cố định | Quy trình Calibration | Phần mềm | Là cái "tật" của chip, được đo lúc đứng im để trừ đi. | `drn_mpu6050.cpp` (Dòng 173) |
| 3 | `gyro_z_dps` | Thay đổi | `(raw - offset) / 65.5` | Phần mềm | Vận tốc góc đã chuẩn hóa, nhưng vẫn còn nhiễu. | `drn_mpu6050.cpp` (Dòng 246) |
| 4 | `delta_time_s` | Thay đổi | System Tick | Phần mềm | Thời gian trôi qua giữa 2 lần đọc cảm biến ($dt$). | `drn_mpu6050.cpp` (Dòng 250) |
| 5 | `yaw_deg` | Thay đổi | Phép tích phân | Phần mềm | Góc Yaw bị trôi dần theo thời gian do thiếu mốc. | `drn_mpu6050.cpp` (Dòng 265) |

<br>

### Bảng 2: Trục Roll & Pitch - Những người được bảo vệ

| STT | Tên thông số | Tính chất | Bị ảnh hưởng bởi | Nguồn gốc | Ghi chú cho Newbie | Vị trí / Dòng code |
| :-- | :--- | :--- | :--- | :--- | :--- | :--- |
| 1 | `FILTER_ALPHA` | Cố định | Cài đặt tĩnh (0.98) | Phần mềm | Tỷ lệ tin tưởng Gyro (mượt) so với Accel (thẳng). | `drn_mpu6050.h` (Dòng 30) |
| 2 | `accel_x_g` | Thay đổi | Trọng lực + Rung động | Phần cứng | Gia tốc thực tế đo bằng cấu trúc MEMS. | `drn_mpu6050.cpp` (Dòng 242) |
| 3 | `accel_roll` | Thay đổi | Hàm `atan2f` | Phần mềm | Góc nghiêng tuyệt đối so với mặt đất (nhưng nhiễu). | `drn_mpu6050.cpp` (Dòng 254) |
| 4 | `roll_deg` | Thay đổi | Complementary Filter | Phần mềm | Sản phẩm cuối cùng: mượt mà và không bao giờ trôi. | `drn_mpu6050.cpp` (Dòng 261) |

---

## 7. Mô phỏng Trực quan
Để hiểu rõ hơn về cách nhiễu và sai số tĩnh cộng dồn làm trôi góc Yaw, hãy chạy widget mô phỏng dưới đây:

[Mở Widget mô phỏng Yaw Drift](../assets/yaw_drift_widget.html)

## 8. Vai trò của Gyroscope (Con quay hồi chuyển) và Hiện tượng Nhiễu

> **Câu hỏi của tôi:** Nãy giờ chúng ta mới nhắc đến Accelerometer thôi, còn Gyroscope thì chưa. Tác dụng của nó để làm gì vậy? Có những cái nào cố định, cái nào thay đổi? Tại sao khi dùng bản Raw, góc nhảy rất mạnh (noise) khi rung board hoặc bật motor do thiếu sự bù đắp mượt mà từ Gyroscope?

Nếu Accelerometer là "mỏ neo" tuyệt đối với mặt đất, thì **Gyroscope chính là "bộ giảm xóc" mượt mà nhất** của hệ thống IMU. Con quay hồi chuyển (Gyroscope) chỉ đo **vận tốc góc** (tốc độ xoay của drone - độ/giây), hoàn toàn không đo gia tốc tuyến tính.

**Giải thích hiện tượng góc nhảy múa (Noise) ở bản Raw:**
Khi động cơ quadcopter quay, nó tạo ra độ rung cơ học (vibration) cực kỳ mạnh. Accelerometer vô cùng nhạy cảm với các rung động này. Nó nhầm tưởng các dao động cơ học $\pm 2g$ từ motor chính là sự thay đổi của trọng lực. Kết quả là phép toán `atan2` trong bản Raw sẽ trả về một góc nghiêng bị giật tung lên xuống (noise).

Tuy nhiên, **Gyroscope hoàn toàn miễn nhiễm với rung động cơ học tuyến tính**. Nó chỉ quan tâm đến việc drone có đang "xoay" hay không. Nhờ đặc tính này, tín hiệu từ Gyroscope cực kỳ mượt và phản hồi ngay lập tức (short-term accuracy). 

Thuật toán dùng phép tích phân để biến vận tốc góc của Gyro thành "Góc nghiêng" (Angle):

$$Angle_{gyro} = Angle_{old} + (G_{measured} \cdot dt)$$

Sau đó, Bộ lọc bù (Complementary Filter) sẽ lấy **98% sự mượt mà này của Gyroscope**, kết hợp với **2% độ chính xác tuyệt đối của Accelerometer**, tạo ra thông số góc nghiêng hoàn hảo cuối cùng.

---

### Bảng 3: Thông số Thuật toán Gyroscope (Con quay hồi chuyển)

| STT | Tên biến trong Code | Tính chất | Bị ảnh hưởng / Tác nhân thay đổi | Nguồn gốc (Cứng / Mềm) | Ghi chú cho Newbie | Vị trí / Dòng code |
| :-- | :------------------ | :-------- | :------------------------------- | :--------------------- | :----------------- | :----------------- |
| 1 | `gyro_x_raw`, `gyro_y_raw` | Thay đổi liên tục | Chuyển động xoay vật lý (Coriolis effect) + Nhiễu điện. | **Phần Cứng:** Đọc I2C từ thanh ghi MPU6050 (0x43). | Dữ liệu thô. Nếu drone xoay quanh trục X hoặc Y, số này sẽ thay đổi rất nhanh và mượt. | `drn_mpu6050.cpp` (Dòng 211) |
| 2 | `gyro_x_offset`, `gyro_y_offset` | Cố định (sau startup) | Quy trình `DRN_MPU6050_Calibrate` lúc đứng im. | **Phần Mềm:** Giá trị trung bình cộng (1000 mẫu). | "Tật" lệch số của cảm biến. Lấy số thô trừ đi số này sẽ ra số 0 tròn trĩnh khi drone nằm yên. | `drn_mpu6050.cpp` (Dòng 171) |
| 3 | `gyro_x_dps`, `gyro_y_dps` | Thay đổi | Kết quả của phép tính `(raw - offset) / 65.5f`. | **Phần Mềm:** Toán học chuẩn hóa đơn vị. | Độ/giây (Degrees Per Second - dps). Đây là giá trị cốt lõi để tính toán góc mượt và dùng cho Rate PID. | `drn_mpu6050.cpp` (Dòng 245) |
| 4 | Thành phần Tích phân `(Gyro \cdot dt)` | Thay đổi | `gyro_x_dps * delta_time_s` | **Phần Mềm:** Thuật toán (bên trong Complementary Filter). | Sự thay đổi góc trong một chu kỳ code (VD: 5ms). Cộng dồn cái này giúp góc thay đổi cực kỳ trơn tru. | `drn_mpu6050.cpp` (Dòng 261) |