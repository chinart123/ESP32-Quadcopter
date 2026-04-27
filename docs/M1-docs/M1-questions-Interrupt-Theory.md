Chào bạn, một người chuyển từ lập trình bare-metal trên STM32 sang môi trường RTOS trên ESP32 chắc chắn sẽ gặp cú sốc về cách quản lý thời gian và tài nguyên. Những thắc mắc của bạn cực kỳ thực tế và là "bài học xương máu" của dân làm firmware. 

Dưới đây là giải đáp chi tiết cho từng vấn đề của bạn:

### 1. Vì sao nên dùng chân INT ở MPU6050? Lợi thế và độ phức tạp
Trong lập trình drone, vòng lặp PID cần một thứ tối quan trọng: **Thời gian lấy mẫu (dt) phải cố định tuyệt đối**.

* **Nếu không dùng INT (Polling):** CPU của ESP32 phải liên tục vào hàm đọc I2C để hỏi MPU6050 "Có data mới chưa?". Việc này tiêu tốn CPU vô ích. Hơn nữa, do FreeRTOS chia sẻ CPU cho nhiều task khác (như Wi-Fi), thời gian giữa 2 lần đọc cảm biến sẽ bị rung rật (jitter). `dt` thay đổi liên tục sẽ làm thành phần D (Derivative) của bộ PID bị nhiễu loạn, drone bay sẽ bị rung lắc (oscillation).
* **Nếu dùng INT:** MPU6050 có một bộ nhớ đệm (FIFO) bên trong. Bạn cài đặt cho nó lấy mẫu đúng ở tần số 500Hz (2ms). Cứ đúng 2ms, MPU6050 xuất một xung ra chân INT. ESP32 đang làm việc khác (hoặc đang ngủ) sẽ lập tức bị "đánh thức" để đi lấy data.
* **Độ phức tạp:** Chắc chắn là tăng lên. Bạn phải viết thêm cấu hình GPIO Interrupt cho chân đó trên ESP32 và tạo một hàm phục vụ ngắt (ISR). Tuy nhiên, đây là sự đánh đổi hoàn toàn xứng đáng để có một hệ thống bay ổn định.

---

### 2. Ngắt có làm ảnh hưởng đến FreeRTOS và "bay màu" CPU không?
**Câu trả lời ngắn gọn: CÓ NGUY CƠ RẤT CAO nếu bạn mang thói quen từ bare-metal sang.**

Ở STM32F1 (bare-metal), khi có ngắt EXTI, chương trình nhảy vào hàm phục vụ ngắt (ISR). Bạn có thể viết vài vòng `for`, `delay`, hoặc gọi SPI/I2C ngay trong đó rồi mới thoát ra. Hệ thống chỉ bị trễ chứ hiếm khi crash.

**Trên ESP32 + FreeRTOS thì khác:**
Hệ điều hành có một "cảnh sát" gọi là **Interrupt Watchdog (IWDT)**. Nhiệm vụ của nó là canh chừng các ngắt. Nếu một hàm ISR chạy quá lâu (thường là vài trăm micro giây) hoặc bị block (chờ I2C/UART), IWDT sẽ kết luận là hệ thống bị treo, nó lập tức kích hoạt lỗi `Interrupt wdt timeout on CPU0/1` và tự động **Reset (Reboot) toàn bộ ESP32**. Bạn sẽ thấy console văng ra một rổ lỗi "Core dump" đỏ rực.

**2.1 MPU6050 có làm "bay màu" CPU không?**
Sẽ "bay màu" nếu bạn thiết kế sai: Kéo chân INT -> Nhảy vào ISR -> **Gọi hàm I2C_Read() ngay trong ISR** -> Crash ngay lập tức. (Vì giao tiếp I2C mất thời gian và sử dụng các cơ chế chờ, tuyệt đối cấm dùng trong ISR của FreeRTOS).

---

### 3. Phân loại các hành vi dùng Ngắt trong FreeRTOS

Dưới đây là 2 bảng đối chiếu những tư duy lập trình ngắt dễ gây "toang" hệ thống và những kĩ thuật an toàn được chuẩn hóa.

**Bảng 1: Các loại ngắt / Hành vi trong ngắt làm "bay não" CPU (Cần tránh)**

| Loại ngắt / Hành vi | Hậu quả trên FreeRTOS (ESP32) | Lý do hệ thống sụp đổ |
| :--- | :--- | :--- |
| **Giao tiếp ngoại vi (I2C/SPI/UART) ngay trong ISR** | Bị Watchdog khởi động lại chip (Crash). | Các hàm này tốn thời gian, hoặc chứa các API có tính chất block (chờ đợi), vi phạm nguyên tắc "ISR phải thực thi chớp nhoáng". |
| **Dùng các hàm FreeRTOS thông thường trong ISR** | Lỗi Memory, Task Scheduler bị lỗi, Panic. | Các hàm như `vTaskDelay`, `xQueueSend`, `xSemaphoreGive` được thiết kế cho Task. Trong ISR, bạn bắt buộc phải dùng các phiên bản có đuôi **`...FromISR`**. |
| **Ngắt mức (Level Trigger) không xóa cờ ngắt** | CPU bị kẹt cứng (Deadlock), bay màu CPU. | Nếu chân ngắt vẫn giữ mức High/Low mà trong ISR bạn quên xóa cờ ngắt (Clear Interrupt Flag), CPU vừa thoát ra sẽ bị hút ngược lại vào ISR vĩnh viễn. |
| **Tính toán Float/Toán học phức tạp trong ISR** | Gây trễ hệ thống, rớt gói tin Wi-Fi/Bluetooth. | Bộ xử lý dấu phẩy động (FPU) trong ngắt tốn nhiều chu kỳ máy. Cản trở các ngắt ưu tiên cao hơn của Wi-Fi/OS. |

**Bảng 2: Kĩ thuật dùng Ngắt an toàn dưới sự kiểm soát của FreeRTOS**

| Kĩ thuật an toàn | Cách hoạt động | Ưu điểm & Sự bảo vệ từ FreeRTOS |
| :--- | :--- | :--- |
| **Deferred Interrupt Processing (Xử lý ngắt hoãn lại)** | **BÍ KÍP QUAN TRỌNG NHẤT:** ISR chỉ làm 1 việc duy nhất là báo hiệu (gửi Semaphore/Queue) để đánh thức một Task ưu tiên cao đang ngủ. ISR lập tức kết thúc. | Task bị đánh thức sẽ chạy như một luồng bình thường, tha hồ gọi I2C, SPI, tính toán Float mà không sợ Watchdog gõ đầu. |
| **Sử dụng API `...FromISR`** | Dùng `xSemaphoreGiveFromISR()` hoặc `xQueueSendFromISR()` kèm biến `pxHigherPriorityTaskWoken`. | FreeRTOS nhận biết context switch đang xảy ra ở cấp độ phần cứng và an toàn chuyển đổi ngữ cảnh ngay khi thoát ngắt. |
| **Phân quyền ưu tiên (Interrupt Priority)** | Các ngắt phần cứng (I2C, Timer) được cấp mức ưu tiên (1-7). ESP-IDF có cơ chế HAL tự động quản lý. | Đảm bảo ngắt quan trọng của Wi-Fi/Bluetooth (do nhân ESP-IDF quản lý) không bị các ngắt GPIO của người dùng cản trở. |

---

### 4. Áp dụng ngắt cho dự án Drone của bạn

Dựa vào các kĩ thuật an toàn ở Bảng 2, đây là hướng dẫn cụ thể cách bạn tổ chức phần mềm cho chiếc drone nhỏ của mình để hệ thống mượt mà và không bao giờ crash.

| Nguồn sinh ngắt | Mục đích | Cách xử lý an toàn (Theo chuẩn FreeRTOS) |
| :--- | :--- | :--- |
| **1. Cảm biến MPU6050 (Chân INT)** | Báo có data góc nghiêng mới (VD: 500 lần/giây). | **Dùng Deferred Processing (Binary Semaphore):**<br>1. ISR chỉ gọi `xSemaphoreGiveFromISR()`.<br>2. Một Task (ví dụ `drone_core_task`) dùng `xSemaphoreTake()` để nằm ngủ chờ.<br>3. Ngay khi có ngắt, Task tỉnh dậy, gọi hàm đọc I2C, chạy bộ lọc Madgwick, tính PID và xuất PWM. |
| **2. Nút nhấn cơ học (2 nút)** | Bật/tắt động cơ, hiệu chuẩn (Calibrate) MPU. | **Dùng FreeRTOS Queue:**<br>1. Chống dội phím (Debounce) bằng tụ điện phần cứng hoặc Timer.<br>2. ISR nút nhấn gọi `xQueueSendFromISR()` đẩy mã nút nhấn vào hàng đợi.<br>3. Task giao diện sẽ đọc Queue này và xử lý bật/tắt (không làm tắc nghẽn vòng lặp PID). |
| **3. Wi-Fi (Nhận lệnh UDP từ App)** | Nhận dữ liệu điều khiển (Throttle, Pitch, Roll, Yaw). | **Không cần viết ISR:**<br>Bộ thư viện LwIP của ESP-IDF đã tự động xử lý ngắt Wi-Fi ở tầng cực thấp. Bạn chỉ cần mở một Task riêng, chạy hàm `recvfrom()` (hàm này tự động block/ngủ cho đến khi có gói tin mạng rớt xuống) rồi lưu data vào một biến toàn cục an toàn. |
| **4. Xuất xung PWM cho 4 Động cơ** | Điều khiển tốc độ quay của cánh quạt. | **Dùng Ngoại vi Phần cứng (LEDC / MCPWM):**<br>Không dùng ngắt Timer của CPU để tạo PWM (như STM32 đôi khi hay làm). Sử dụng bộ LEDC của ESP32, nó tự động tạo xung bằng phần cứng. Task PID chỉ cần cập nhật giá trị Duty Cycle (`ledc_set_duty`) xuống thanh ghi là xong. |

**Tóm lại:** "Não" của ESP32 rất mạnh, nhưng FreeRTOS có kỷ luật thép. Hãy coi ISR chỉ là "người bấm chuông cửa" (Semaphore/Queue) và bỏ chạy. Kẻ ra mở cửa và bưng bê đồ đạc (đọc I2C, tính toán) phải luôn là các "Task" bên trong nhà.