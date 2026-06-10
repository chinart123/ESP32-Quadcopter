# Nhật ký Tinh chỉnh Flight Controller: Giữ Vị trí (Quang sai PMW3901)
**Phần cứng:** ESP32-S3 Supermini + VL53L1X + PMW3901 + MPU6050
**Hệ thống phụ:** Điều hướng Giữ Vị trí bằng PID lồng nhau (Cascaded PID)

## 1. Các Hiện tượng Bay Quan sát được
* **Trôi chéo & Lao về phía trước (Diagonal Drift & Forward Surge):** Dù thông số Kp/Kd cao hay thấp, drone vẫn biểu hiện lỗi dính trục (coupled-axis) rất nhất quán. Nó trượt chéo (Từ Trái-Lùi sang Phải-Tiến) và sau đó chồm mạnh về phía trước. Khâu Kp càng lớn, lực lao tới càng bị khuếch đại mạnh.
* **Giới hạn Cơ bắp PWM (PWM Actuation Constraint):** Việc ép giới hạn lực PWM xuất ra ở vòng lặp trong (`shared_pwm_opt`) xuống mức `±10.0f` giúp giảm thiểu các phản ứng giật cục tức thời, cho phép drone trụ được tối đa khoảng ~2.5 giây trên không trước khi mất kiểm soát hoàn toàn do gió/quán tính trôi vượt quá giới hạn lực cản cho phép của động cơ.
* **Sự bất thường của Logic Phanh (Brake Logic Anomaly):** Việc giảm thời gian ép phanh cứng (`millis() - brakeStartTimeX < 50` thay vì 100ms) làm giảm mức độ lao về phía trước. Điều này chứng tỏ logic phanh đang tác dụng lực sai vector (sai hướng).

## 2. Nghịch lý Tinh chỉnh Thông số (SF vs. DEADBAND)
Phát hiện một sự mâu thuẫn nghiêm trọng giữa Hệ số tỷ lệ (SF - Scale Factor) và Vùng ngắt nhiễu vận tốc (Velocity Deadband):
* **SF Chuẩn vật lý (0.0012):** Dựa trên bài test cơ bản ở độ cao 1100mm, SF 0.0012 phản ánh đúng tỷ lệ quang học thực tế. Tuy nhiên, nó lại hoạt động rất tệ trong logic code gộp hiện tại. Do vận tốc tính toán ra quá nhỏ (vì quá chính xác), chuyển động trôi thực tế không thể đâm thủng được bức tường ngắt nhiễu (`POS_DEADBAND`), khiến vòng lặp PID bị "mù" trước các dao động trôi chậm.
* **SF Phóng đại (0.0075 - 0.075):** Bản code gộp tạm thời sử dụng một SF khổng lồ phi thực tế. Nó khuếch đại các vi chuyển động, giúp chúng dễ dàng vượt qua bức tường deadband. Điều này tạo ra một trạng thái siêu nhạy ("ổn định giả tạo") quanh điểm tâm, lừa PID phải liên tục sửa lỗi lắt nhắt, từ đó che lấp đi những lỗ hổng logic cốt lõi bên dưới.

## 3. Thông số Hiệu chuẩn (Step 5A Độc lập / Chờ thực hiện Step 5B)
Bài test cảm biến độc lập trước đó (Step 5A) ở độ cao H=765mm với SF=0.005 cho ra kết quả phi tuyến tính nghiêm trọng (dịch chuyển vật lý 100mm nhưng tính toán ra chỉ 22mm). Sự sai lệch này khả năng cao là do mới chỉ test module quang học rời rạc chứ chưa có hệ động học của toàn bộ khung frame drone.
* **Hành động cần làm:** Việc test trượt động học (Step 5B) đang phải tạm hoãn chờ có phần cứng thực tế. Ngay khi cầm lại khung drone hoàn chỉnh, Step 5B bắt buộc phải được thực hiện lại qua nhiều mốc khoảng cách (100mm, 200mm, 500mm) ở một độ cao cố định để trích xuất được hệ số SF tuyến tính tuyệt đối trước khi đưa vào vòng lặp PID mới.

## 4. Lỗ hổng Kiến trúc & Chiến lược Cấu trúc lại Code (Refactoring)
Logic code Giữ vị trí đang gộp hiện tại chứa một số nguyên mẫu thiết kế sai lệch (anti-patterns):
* **Ép phanh bằng hàm thời gian:** Việc dùng `millis()` để bơm các xung Kp tùy tiện (`Kp_vel_inner = 20.0f / 10.0f` trong khoảng 50/100ms) đã phá vỡ hoàn toàn nguyên lý điều khiển đạo hàm (Kd) tiêu chuẩn và sinh ra các vector lực hất văng khó lường.
* **Dính trục tọa độ (Axis Coupling):** Hiện tượng văng chéo là bằng chứng rõ ràng nhất cho thấy sự sai lệch hướng (chênh lệch góc Yaw) giữa trục tọa độ phần cứng của camera PMW3901 và mảng tính toán IMU/Mixer xuất lực ra động cơ.

**Kết luận:** Logic PID lồng nhau hiện tại và các đoạn mã ép phanh bằng thời gian sẽ bị loại bỏ hoàn toàn. Kiến trúc này cần được đập đi xây lại toàn diện để tạo ra một vòng lặp Vị trí (Outer P) -> Vận tốc (Inner PI/PD) độc lập, rành mạch và không bị dính trục.