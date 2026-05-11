# NHẬT KÝ TÌM HIỂU KỸ THUẬT (SCIENTIFIC RESEARCH LOG)
**Chuyên đề:** Phân tích kiến trúc viễn thông và giao thức giao tiếp trên Drone
**Mục tiêu:** Đánh giá luồng dữ liệu nội bộ (I2C) và luồng dữ liệu ngoại vi (RC Link), định hướng tối ưu hóa cho hệ thống Bare-metal.

---

## CHƯƠNG 1: HỆ SINH THÁI I2C VÀ BÀI TOÁN THỜI GIAN THỰC
### 1.1. Bản chất giao thức (Master - Slave)
I2C hoạt động dựa trên triết lý **Chủ - Tớ (Master - Slave)** khắt khe. Vi điều khiển (MCU) đóng vai trò là Master, nắm quyền sinh sát bằng cách tạo ra xung nhịp Clock (SCL). Các cảm biến như MPU6050 là Slave, chỉ được phép phản hồi dữ liệu trên chân SDA khi Master gọi đúng địa chỉ (ví dụ: `0x68`) và cấp xung nhịp. 

### 1.2. Phân tích Toán học: Quỹ thời gian 2ms (Control Loop Budget)
Trong thiết kế Flight Controller, tốc độ vòng lặp (Control Loop) tiêu chuẩn là 500Hz, tương đương với quỹ thời gian cực kỳ chật hẹp: **2.00ms (2000µs) cho mỗi vòng**.
Trong đúng 2ms này, CPU phải hoàn tất một khối lượng công việc khổng lồ:
* **Đọc cảm biến (I2C):** Ở tốc độ bus 400kHz, việc kéo 14 bytes dữ liệu từ MPU6050 tốn khoảng **365µs**.
* **Tính toán PID (Math):** Xử lý mảng số thực float (đặc biệt nặng nề trên các chip thiếu FPU cứng như STM32F103).
* **Xuất xung động cơ (Motor PWM):** Ra lệnh cho ESC.

**Vấn đề kỹ thuật (Bottleneck):** Nếu sử dụng phương pháp **I2C Polling (Hỏi vòng)**, CPU sẽ bị "đóng băng" hoàn toàn trong 365µs chỉ để chờ đọc xong dữ liệu. Điều này ăn lẹm nghiêm trọng vào quỹ 2ms, làm tăng nguy cơ trễ deadline (Loop time overrun), gây mất ổn định khi bay. Hướng giải quyết bắt buộc cho kiến trúc Bare-metal tương lai là chuyển sang dùng **Interrupt (Ngắt)** hoặc **DMA (Direct Memory Access)** để phần cứng tự động thu thập I2C, giải phóng CPU đi tính toán PID.

### 1.3. Sự phân mảnh và các "Hậu duệ" của I2C
Dù thông số kỹ thuật gốc thuộc về NXP (Philips), I2C đã rẽ nhánh thành nhiều phiên bản để giải quyết các bài toán thương mại và kỹ thuật:
* **TWI (Two-Wire Interface - Atmel):** Bản chất 100% là I2C, sinh ra chỉ để lách luật bản quyền tên gọi thương hiệu của Philips.
* **SMBus (System Management Bus - Intel):** Khắc phục nhược điểm "chết chùm" của I2C. Nếu một Slave bị treo và giữ chân SDA ở mức Thấp, SMBus có cơ chế **Timeout (35ms)** để tự động reset đường truyền, chống treo hệ thống (Bus hang).
* **I3C (MIPI Alliance):** Thế hệ thừa kế hiện đại, tốc độ cao gấp 10 lần, cho phép Slave chủ động giơ tay phát biểu (In-band Interrupt) thay vì thụ động chờ Master gọi tên.

---

## CHƯƠNG 2: HỆ SINH THÁI RC VÀ CUỘC ĐUA ĐỘ TRỄ (LATENCY)
### 2.1. Bản chất giao thức (Broadcaster - Listener)
Trái ngược với I2C, hệ thống RC truyền thống mang bản chất **Truyền phát một chiều (Simplex Communication)**. Tay cầm (TX - Phát thanh viên) liên tục xả dữ liệu vào không trung, không cần biết mạch nhận (RX - Thính giả) có đang nghe hay không, và hoàn toàn không có cơ chế hồi đáp (No ACK). 

### 2.2. Phân tích Toán học: Quỹ thời gian 20ms (RC Link Budget)
Hệ thống RC kế thừa một di sản Analog từ thập niên 70, dựa trên chuẩn điều chế độ rộng xung (PWM) với các mốc thời gian kinh điển:
* **Dải giá trị kênh:** `1000µs` (Kịch góc dưới) — `1500µs` (Trung tâm) — `2000µs` (Kịch góc trên). Con số thời gian này là **mã hóa tín hiệu (encoding)**, không phải thời gian xoay cơ học của động cơ.
* **Khung truyền 20ms (50Hz):** Để truyền 6 kênh trên 1 luồng sóng duy nhất, hệ thống dùng kỹ thuật **PPM (Time-Division Multiplexing)**. 
    * **12ms đầu tiên:** 6 kênh xếp hàng lần lượt truyền tín hiệu (mỗi kênh tốn tối đa 2ms).
    * **8ms cuối cùng:** Đây là **Khoảng nghỉ (Blanking Interval / Sync Pulse)**. Khoảng im lặng này là sống còn để mạch RX nhận biết chu kỳ đã kết thúc, reset lại con trỏ mảng (Index = 0) và kiểm tra an toàn (Failsafe Watchdog) trước khi đón gói tin của chu kỳ 20ms tiếp theo.

### 2.3. Sự tiến hóa của RC Digital và Bước tiến CRSF
Khi tiến lên môi trường kỹ thuật số (Digital), giao thức RC chứng kiến cuộc chiến khốc liệt về chuẩn giao tiếp:
* **SBUS (Futaba):** Góp công gộp 16 kênh vào 1 dây cáp duy nhất bằng UART. Tuy nhiên, rào cản kỹ thuật là tốc độ baud dị biệt (100,000 bps) và **Tín hiệu đảo ngược (Inverted UART)**, ép các vi điều khiển như STM32F1 phải thiết kế thêm cổng NOT gate vật lý.
* **FPort (FrSky):** Giải quyết bài toán truyền nhận 2 chiều (Telemetry) trên 1 dây (Half-duplex), nhưng vẫn giữ thiết kế Inverted bảo thủ.
* **CRSF / Crossfire (Team BlackSheep):** Giao thức định chuẩn lại toàn bộ ngành FPV. Chạy trên **Non-inverted UART** tiêu chuẩn, tốc độ baud cực cao (420k - 1M bps), đóng gói dữ liệu nén giúp triệt tiêu độ trễ (Latency) xuống chỉ còn 2-3ms. Tính minh bạch và phần cứng thân thiện giúp CRSF trở thành "ngôn ngữ quốc dân" cho các hệ thống mở như ExpressLRS.

---

## CHƯƠNG 3: SỰ GIAO THOA CỦA HAI DÒNG THỜI GIAN (2MS VÀ 20MS)
Sự kỳ diệu của hệ thống điều khiển bay nằm ở cách ghép nối hai chu kỳ hoàn toàn bất đồng bộ này:
1.  **Nhịp độ con người (20ms/chu kỳ - 50Hz):** Phản xạ của ngón tay phi công là chậm. Cứ mỗi 20ms, hệ thống CRSF mới rót xuống FC một giá trị **Setpoint** mới (Ví dụ: "Nghiêng 15 độ").
2.  **Nhịp độ máy móc (2ms/chu kỳ - 500Hz):** CPU không nằm chờ tay cầm. Trong thời gian 20ms chờ đợi lệnh mới, CPU đã tự động lặp lại **10 vòng chiến đấu** (10 x 2ms). Nó dùng I2C cào góc nghiêng thực tế từ MPU6050, đưa vào thuật toán PID để so khớp với cái Setpoint "15 độ" kia, rồi liên tục điều phối ESC để cưỡng lại sức gió.

**Kết luận:** Quỹ `20ms` là tốc độ con người cập nhật mục tiêu. Quỹ `2ms` là tốc độ CPU thực thi cơ học để bảo vệ mục tiêu đó. Tối ưu hóa hệ thống Bare-metal chính là việc dùng DMA/Interrupt để nhét càng nhiều phép toán PID càng tốt vào trong cái khe hẹp `2ms` đó.