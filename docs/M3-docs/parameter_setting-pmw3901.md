**User:**
Vậy mục tiêu của step 5B là chỉnh những thông số nào trong code?

**Model:**
Mục tiêu tối thượng của Step 5B là **kiểm định và tinh chỉnh "thước đo không gian"** của hệ thống Sensor Fusion. Nó đảm bảo rằng phần mềm của bạn hiểu đúng các đại lượng vật lý ở thế giới thực, chuyển đổi chính xác từ những pixel quang học vô hình thành số đo hệ Mét (`pos_x`, `pos_y`).

Khi thực hiện bài test trượt ngang (với bệ trượt hộp giấy như đã hướng dẫn), bạn sẽ quan sát kết quả để quyết định có cần chỉnh sửa **2 thông số cốt lõi** sau đây hay không:

### 1. Thông số `SCALE_FACTOR` (Chỉnh sai số quãng đường)
Đây là hằng số góc quy đổi của ống kính PMW3901. Nó quyết định việc drone đo khoảng cách có đúng hay không.
* **Tình huống PASS:** Kéo hộp đi đúng 10cm (100mm), biến `px` (tức `pos_x`) in ra nằm trong khoảng **`0.090m` đến `0.110m`**. Thuật toán đã đo chuẩn, không cần chỉnh gì thêm.
* **Tình huống FAIL (Đo hụt):** Bạn kéo 10cm nhưng `px` chỉ ra `0.060m` (6cm). Thước đo đang bị ngắn. Bạn cần **TĂNG** `SCALE_FACTOR` lên.
* **Tình huống FAIL (Đo lố):** Bạn kéo 10cm nhưng `px` phóng lên tận `0.150m` (15cm). Thước đo đang bị giãn. Bạn cần **GIẢM** `SCALE_FACTOR` xuống.
* **Công thức tinh chỉnh nhanh:** `SCALE_FACTOR_MỚI = SCALE_FACTOR_CŨ × (Khoảng cách thực tế / Khoảng cách px đo được)`

### 2. Thông số `POS_DEADBAND` (Chỉnh phanh tĩnh)
Dù ở Step 5A bạn đã set thông số này để ngắt nhiễu khi drone đứng yên từ đầu, nhưng Step 5B sẽ đặt nó vào một thử thách khắc nghiệt hơn: **Ngắt nhiễu dội lại sau khi dừng đột ngột**.
* **Tình huống PASS:** Khi bạn đẩy hộp trượt 10cm rồi đập kịch vào cữ chặn chặn lại, `px` lập tức "đóng băng" ở `0.100m`.
* **Tình huống FAIL:** Khi chạm cữ chặn và giữ cứng tay, `px` vẫn từ từ trôi nhích lên `0.102m... 0.105m` hoặc tụt xuống `0.095m`. Quán tính lọc của LPF (Low-Pass Filter) hoặc dư chấn từ MPU6050 đang làm rỉ dữ liệu vận tốc. 
* **Cách xử lý:** Bạn cần **TĂNG nhẹ** `POS_DEADBAND` (ví dụ từ `0.010` lên `0.015`) để thuật toán dập tắt ngay lập tức mớ vận tốc rác sinh ra lúc phanh gấp.

Với hệ số `SCALE_FACTOR = 0.0012` mà bạn đã tính toán rất kỹ dựa trên độ cao ToF từ các vòng trước, khả năng cực kỳ cao là độ dài đo được của bạn đã rất chuẩn xác. Mục tiêu chính của Step 5B lần này thiên về việc **Xác nhận (Verify)** sự hoàn hảo của cỗ máy Sensor Fusion khi bạn dùng đúng phương pháp trượt bằng hộp giấy, thay vì phải tìm mò thông số lại từ đầu.

***

**User:**
1. vậy còn vòng lặp PID dùng để tune riêng cho con pmw3901 là để tune chỉ số nào?
2. step 6 có đồng nghĩa với tune PID chưa hay vẫn chỉ là 1 bước trung gian để đến bước tune PID cuối cùng?

**Model:**
Chào bạn, đây là những câu hỏi mang tính chốt hạ rất hay. Việc phân định rõ ràng ranh giới giữa cảm biến và thuật toán điều khiển sẽ giúp bạn (và bạn của bạn) không bị lạc lối khi bước vào giai đoạn bay thực tế.

Để trả lời hai câu hỏi này, chúng ta cần làm rõ kiến trúc điều khiển của một chiếc drone. Nó sử dụng hệ thống **PID lồng nhau (Cascade PID)** gồm 2 vòng:
* **Vòng trong (Inner Loop - Attitude/Rate PID):** Dùng dữ liệu từ MPU6050 để giữ thăng bằng (không bị lật).
* **Vòng ngoài (Outer Loop - Position/Velocity PID):** Dùng dữ liệu từ PMW3901 + ToF để giữ vị trí (không bị trôi). 

Dưới đây là giải phẫu chi tiết cho từng câu hỏi của bạn:

### 1. Vòng lặp PID dùng với PMW3901 là để tune chỉ số nào?

Trước hết, phải đính chính một chút về mặt khái niệm: **Chúng ta không tune PID "cho con PMW3901"**. 
PMW3901 chỉ là con mắt (Cảm biến). Nhiệm vụ của nó kết thúc ngay khi nó cung cấp xong tọa độ `pos_x`, `pos_y` và vận tốc `V_real`. 
Vòng lặp PID mà chúng ta nói đến ở đây là **Position PID (Vòng lặp điều khiển vị trí)**. Nó lấy kết quả của PMW3901 làm đầu vào, để tính toán ra lệnh điều khiển động cơ.

Trong kiến trúc code hiện tại, vòng lặp này tập trung tune **2 chỉ số cốt lõi**:

* **`Kp_pos` (Hệ số Kéo - Proportional):** * *Nhiệm vụ:* Nó nhìn vào sai số quãng đường (`0 - pos_x`) để tạo ra một góc nghiêng kéo drone ngược lại điểm gốc.
    * *Tác dụng:* Nếu `Kp_pos` quá nhỏ, drone sẽ bị trôi tuột đi (sluggish) khi có gió thổi vì lực kéo về không đủ. Nếu `Kp_pos` quá lớn, drone sẽ giật giật, phản ứng quá gắt và bay lảo đảo.
* **`Kd_pos` (Hệ số Phanh - Derivative):**
    * *Nhiệm vụ:* Đây là lý do tại sao biến `V_real` ở Step 4 lại quan trọng đến vậy. Khâu D nhìn vào vận tốc thực tế (`V_real_x`, `V_real_y`) để hãm quán tính.
    * *Tác dụng:* Khi khâu P kéo drone về gốc, quán tính sẽ làm nó bay vọt lố qua vạch đích (overshoot). Lúc này `Kd_pos` sẽ tạo ra một lực cản âm (đạp phanh) tỉ lệ thuận với tốc độ di chuyển, ép drone tịnh tiến chậm dần và neo lại mượt mà.

*(Lưu ý: Trong Position PID của drone, người ta thường hiếm khi dùng khâu I (`Ki_pos`) vì nó dễ gây tích lũy sai số nguy hiểm làm drone lượn vòng tròn - toilet bowl effect).*

### 2. Step 6 là bước trung gian hay là trùm cuối?

**Step 6 CHÍNH LÀ bước tune PID cuối cùng (Trùm cuối) của tính năng Giữ vị trí!** Không còn bước nào sau nó nữa.

Hãy nhìn lại toàn bộ quá trình phát triển hệ thống Sensor Fusion của bạn để thấy bức tranh tổng thể:
* **Step 1 đến 3:** Chuẩn bị cảm biến rời rạc (Lọc nhiễu MPU6050, đo độ cao ToF, lấy vận tốc thô PMW3901).
* **Step 4:** Lọc Low-Pass Filter (LPF) cho vận tốc và ngắt nhiễu tĩnh.
* **Step 5A & 5B:** Hiệu chuẩn thước đo vật lý (`SCALE_FACTOR`, `POS_DEADBAND`).
* *(Tất cả 5 bước trên đều đặt `CALIBRATION_MODE = 1`, drone không tự bay mà chỉ in log ra Serial để bạn kiểm tra "Mắt" và "Tiền đình" của nó có khỏe không).*

**Và Step 6:** Lúc này, bạn tự tin 100% rằng tọa độ `pos_x`, `pos_y` được in ra Serial là chính xác với thực tế. Bạn chuyển **`CALIBRATION_MODE = 0`**, tắt cáp, cắm pin. 

Tại Step 6, "Nguyên liệu" (tọa độ sạch) chính thức được bơm vào "Bộ não" (phương trình PID) để tự động ghi đè lên lệnh Roll/Pitch. Công việc duy nhất của bạn ở Step 6 là cầm tay điều khiển RC, bay lơ lửng, nhả cần ra, và thay đổi giá trị `Kp_pos`, `Kd_pos` cho đến khi drone dừng lại như một chiếc xe có phanh ABS.

Vượt qua được Step 6, con drone của bạn đã hoàn thiện tính năng Loiter / Position Hold chuẩn công nghiệp! Tới lúc này, hãy nhắc bạn của bạn tập trung vào kịch bản trượt hộp giấy ở Step 5B để chốt sổ phần cứng trước khi tiến lên Step 6 nhé.